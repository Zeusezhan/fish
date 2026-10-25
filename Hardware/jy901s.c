/* ============================================================
 * JY901S 九轴姿态传感器驱动 (I2C)
 *
 * 【数据流】
 *   JY901S_ReadIsr()            <-- 只在 TIM3 中断里调用
 *     -> jy_i2c_read_regs(0x39, buf, 14)   一次读 7 个寄存器
 *     -> jy_parse_fast()   小端解析 + 合理性校验
 *     -> 通过校验才写 jy901s_data
 *
 * 【为什么只读 0x39~0x3F】
 *   姿态闭环只需要 Yaw(0x3F) 和 GZ(0x39)。这两个寄存器之间隔着
 *   磁场和 Roll/Pitch, 所以最小连续区间就是 0x39~0x3F 共 14 字节。
 *   100kHz 下约 1.5ms —— 这个时间直接等于"定时器中断被占用多久",
 *   所以不要再把区间扩大到 0x34(加速度)那一端。
 *
 * 【坏帧处理】
 *   通信失败 / 全 0 / 全 FF / 角速度越界 / 航向单拍跳变超限,
 *   只要命中一条就丢弃本拍, jy901s_data 保持上一拍的值。
 *   调用方(中断)拿到 0 之后沿用上一拍输出继续闭环, 绝不停摆。
 * ============================================================ */

#include "main.h"
#include "i2c.h"
#include "gpio.h"
#include "jy901s.h"
#include "REG.h"

/* 模块解算数据(外部只读) */
JY901S_Data_t jy901s_data;

/* ---- Yaw 解缠绕用的内部变量(存的是模块原始角度, 不带方向符号) ---- */
static uint8_t  jy_has_yaw = 0;          /* 是否已经收到第一帧, 用于初始化基准 */
static float    jy_last_raw_yaw = 0.0f;  /* 上一帧原始 Yaw, 用来检测跳变 */
static int32_t  jy_wrap_cnt = 0;         /* 越过 ±180 的次数(正负各算一圈) */
static float    jy_yaw_unwrapped_raw = 0.0f;  /* 解缠绕后的原始角度 */

/* ---- 软件清零基准: 对外角度 = 解缠绕角度 - 该基准 ---- */
static float    jy_zero_base = 0.0f;

/* ---- I2C 轮询计时 ---- */
static uint32_t jy_next_poll_tick = 0;   /* 下一次允许读取的时刻 */
static uint8_t  jy_first_poll = 1;       /* 1 = 第一次调用立刻读, 不等间隔 */
static uint8_t  jy_fail_cnt = 0;         /* 连续读取失败次数 */
static uint32_t jy_update_tick = 0;      /* 最近一次成功解析数据的时刻 */
static uint8_t  jy_diag_latch = 0;       /* 本次读取中是否出现过坏帧 */

/* ---- 诊断计数(供 OLED / 调参界面显示) ---- */
static uint32_t jy_ok_total   = 0;       /* 成功帧数 */
static uint32_t jy_fail_total = 0;       /* I2C 通信失败次数 */
static uint32_t jy_bad_total  = 0;       /* 通信成功但被判为坏帧的次数 */

/* ==================== I2C 底层 ==================== */

/**
  * @brief  总线自恢复
  * @note   如果上一次传输异常中断, I2C 外设会停在 BUSY 状态,
  *         之后所有读写都会立即失败且无法自行恢复。
  *         这里检测到非 READY 就复位一次外设, 让后续读写能继续。
  *
  *         ★ 这里绝对不能有 HAL_Delay —— 本函数会被定时器中断调用,
  *           而 TIM3(优先级 5) 会抢占 SysTick(优先级 15), 中断里
  *           等 tick 就是等一个永远不会到来的事件, 直接死机。
  */
static void jy_i2c_check_recover(void)
{
    if (HAL_I2C_GetState(JY901S_I2C_HANDLE) != HAL_I2C_STATE_READY)
    {
        HAL_I2C_DeInit(JY901S_I2C_HANDLE);
        HAL_I2C_Init(JY901S_I2C_HANDLE);
    }
}

/**
  * @brief  从指定寄存器起连续读取若干字节
  * @param  reg  起始寄存器地址
  * @param  buf  接收缓冲区
  * @param  len  要读取的字节数
  * @retval 0 = 成功, 1 = 失败
  * @note   只用 STM32 标准的 Mem_Read 时序(寄存器地址 + 重复起始 + 读)。
  *
  *         原来还有一条"先写寄存器地址并 STOP、再单独读"的备用时序,
  *         但它中间有 HAL_Delay(2) —— 一旦被搬进定时器中断就是死锁。
  *         而且那条时序本身就违反了 JY901S 的寄存器时序要求, 只会
  *         制造更多失败。已删除。
  */
static uint8_t jy_i2c_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    jy_i2c_check_recover();

    if (HAL_I2C_Mem_Read(JY901S_I2C_HANDLE, (uint16_t)(JY901S_I2C_ADDR << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, buf, len, JY901S_I2C_TIMEOUT) == HAL_OK)
    {
        return 0;
    }

    return 1;
}

/**
  * @brief  向指定寄存器连续写入若干字节
  * @param  reg  起始寄存器地址
  * @param  buf  待写入的数据
  * @param  len  要写入的字节数
  * @retval 0 = 成功, 1 = 失败
  * @note   仅用于写模块配置(如回传频率), 读数据不走这里。
  */
static uint8_t jy_i2c_write_regs(uint8_t reg, const uint8_t *buf, uint16_t len)
{
    uint8_t frame[JY901S_REG_BYTES + 1];
    uint16_t i;

    if (len > JY901S_REG_BYTES)
    {
        return 1;
    }

    jy_i2c_check_recover();

    /* ---- 主时序: 寄存器地址 + 重复起始 + 数据 ---- */
    if (HAL_I2C_Mem_Write(JY901S_I2C_HANDLE, (uint16_t)(JY901S_I2C_ADDR << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len,
                          JY901S_I2C_TIMEOUT) == HAL_OK)
    {
        return 0;
    }

    /* ---- 备用时序: 寄存器地址与数据拼在一帧里发 ---- */
    frame[0] = reg;
    for (i = 0; i < len; i++)
    {
        frame[i + 1] = buf[i];
    }
    if (HAL_I2C_Master_Transmit(JY901S_I2C_HANDLE, (uint16_t)(JY901S_I2C_ADDR << 1),
                                frame, (uint16_t)(len + 1), JY901S_I2C_TIMEOUT) == HAL_OK)
    {
        return 0;
    }

    return 1;
}

/* ==================== 数据解析 ==================== */

/**
  * @brief  根据解缠绕后的原始角度更新对外的 yaw 数据
  * @note   会同时更新两个输出:
  *           yaw_continuous —— 连续角度, 可超过 ±180, 供内部累计使用
  *           yaw            —— 折算回 -180~180, 供显示/判断使用
  */
static void jy_update_yaw_outputs(void)
{
    float rel = (jy_yaw_unwrapped_raw - jy_zero_base) * JY901S_YAW_SIGN;

    jy901s_data.yaw_continuous = rel;

    while (rel > 180.0f)
    {
        rel -= 360.0f;
    }
    while (rel < -180.0f)
    {
        rel += 360.0f;
    }
    jy901s_data.yaw = rel;
}

/**
  * @brief  原始寄存器值 -> 物理量
  * @param  raw    寄存器原始值(int16)
  * @param  range  该量的满量程(如加速度 16.0f 表示 ±16g)
  * @retval 换算后的物理量
  */
static float jy_scale(int16_t raw, float range)
{
    return (float)raw / 32768.0f * range;
}

/**
  * @brief  把 14 字节原始缓冲区(寄存器 0x39~0x3F)校验并解析成物理量
  * @param  buf  14 字节接收缓冲区(小端, 低字节在前)
  * @retval 1 = 这一帧有效, 已更新 jy901s_data; 0 = 坏帧, 整个丢弃
  * @note   校验顺序按"代价从低到高"排列, 只要发现不对就立刻返回,
  *         不让坏数据进入 PID。
  */
static uint8_t jy_parse_fast(const uint8_t *buf)
{
    int16_t raw_gz;
    int16_t raw_roll;
    int16_t raw_pitch;
    int16_t raw_yaw;
    float   gz;
    float   yaw;
    float   diff;
    int32_t wrap_cnt;
    float   unwrapped;
    uint8_t i;
    uint8_t all_zero = 1u;
    uint8_t all_ff   = 1u;

    /* ---- 0. 整帧全 0x00 / 全 0xFF: 总线悬空或模块没应答 ---- */
    for (i = 0; i < JY901S_REG_FAST_BYTES; i++)
    {
        if (buf[i] != 0x00u)
        {
            all_zero = 0u;
        }
        if (buf[i] != 0xFFu)
        {
            all_ff = 0u;
        }
    }
    if ((all_zero != 0u) || (all_ff != 0u))
    {
        return 0u;
    }

    /* ---- 1. 取出各个寄存器(小端) ---- */
    raw_gz    = (int16_t)(((uint16_t)buf[1]  << 8) | buf[0]);    /* 0x39 GZ    */
    raw_roll  = (int16_t)(((uint16_t)buf[9]  << 8) | buf[8]);    /* 0x3D Roll  */
    raw_pitch = (int16_t)(((uint16_t)buf[11] << 8) | buf[10]);   /* 0x3E Pitch */
    raw_yaw   = (int16_t)(((uint16_t)buf[13] << 8) | buf[12]);   /* 0x3F Yaw   */

    gz  = jy_scale(raw_gz, 2000.0f);    /* 角速度满量程 ±2000deg/s */
    yaw = jy_scale(raw_yaw, 180.0f);    /* 角度满量程 ±180deg */

    /* ---- 2. 角速度范围 ---- */
    if ((gz > JY901S_MAX_GZ_ABS) || (gz < -JY901S_MAX_GZ_ABS))
    {
        return 0u;
    }

    /* ---- 3. 航向解缠绕 + 单拍跳变校验 ----
     * 模块的 Yaw 在 +180/-180 之间跳变, 先按跳变方向累加圈数把它"拉直",
     * 再看这一拍相对上一拍走了多少度。
     * 超过门限说明读到了"半个旧帧 + 半个新帧", 宁可丢这一拍也不能让
     * 一个假角度直接进 PID —— 那正是原来"偶发猛打方向"的来源。 */
    wrap_cnt = jy_wrap_cnt;

    if (!jy_has_yaw)
    {
        /* 第一帧: 只建立基准, 不做跳变判断 */
        unwrapped = yaw + (float)wrap_cnt * 360.0f;
    }
    else
    {
        diff = yaw - jy_last_raw_yaw;
        if (diff > 180.0f)
        {
            wrap_cnt--;         /* 正向越过 +180, 跳到 -180 */
        }
        else if (diff < -180.0f)
        {
            wrap_cnt++;         /* 反向越过 -180, 跳到 +180 */
        }

        unwrapped = yaw + (float)wrap_cnt * 360.0f;
        diff = unwrapped - jy_yaw_unwrapped_raw;
        if ((diff > JY901S_MAX_YAW_STEP) || (diff < -JY901S_MAX_YAW_STEP))
        {
            return 0u;
        }
    }

    /* ---- 4. 全部校验通过, 提交 ---- */
    jy_has_yaw           = 1u;
    jy_last_raw_yaw      = yaw;
    jy_wrap_cnt          = wrap_cnt;
    jy_yaw_unwrapped_raw = unwrapped;

    jy901s_data.gz    = gz * JY901S_YAW_SIGN;
    jy901s_data.gx    = 0.0f;   /* 精简读取区间不含 GX/GY, 闭环用不到 */
    jy901s_data.gy    = 0.0f;
    jy901s_data.ax    = 0.0f;   /* 同上, 加速度也不再读取 */
    jy901s_data.ay    = 0.0f;
    jy901s_data.az    = 0.0f;
    jy901s_data.roll  = jy_scale(raw_roll, 180.0f);
    jy901s_data.pitch = jy_scale(raw_pitch, 180.0f);

    jy_update_yaw_outputs();
    jy901s_data.ready = 1u;
    jy_update_tick = HAL_GetTick();

    return 1u;
}

/* ==================== 对外接口 ==================== */

/**
  * @brief  初始化 JY901S 驱动
  * @note   调用前需要保证 MX_I2C1_Init() 已经执行。
  *         本函数只清零内部状态, 不阻塞等待模块;
  *         是否真的收到了数据, 看 JY901S_ReadOnce() 的返回值,
  *         或直接读全局标志 jy901s_data.ready。
  */
void JY901S_Init(void)
{
    jy901s_data.ready = 0;
    jy901s_data.yaw = 0.0f;
    jy901s_data.yaw_continuous = 0.0f;

    jy_has_yaw = 0;
    jy_last_raw_yaw = 0.0f;
    jy_wrap_cnt = 0;
    jy_yaw_unwrapped_raw = 0.0f;
    jy_zero_base = 0.0f;

    jy_next_poll_tick = HAL_GetTick();
    jy_first_poll = 1;
    jy_fail_cnt = 0;
    jy_diag_latch = 0u;

    jy_ok_total   = 0u;
    jy_fail_total = 0u;
    jy_bad_total  = 0u;
}

/**
  * @brief  进入"正在写 jy901s_data"状态, 期间屏蔽 TIM3 中断
  * @note   ★ 只在闭环未运行的上电阶段使用(零点、写模块配置)。
  *         闭环运行期间 JY901S 的读取已经在 TIM3 中断内部完成,
  *         主循环不再碰 I2C, 也就不需要这对函数了。
  */
void JY901S_BeginUpdate(void)
{
    NVIC_DisableIRQ(TIM3_IRQn);
}

/**
  * @brief  结束写状态, 恢复 TIM3 中断
  */
void JY901S_EndUpdate(void)
{
    NVIC_EnableIRQ(TIM3_IRQn);
}

/**
  * @brief  取一份数据快照(中断里调用)
  * @param  out  输出缓冲区
  * @retval 1 = 有效, 0 = 还没收到过数据
  */
uint8_t JY901S_TryRead(JY901S_Data_t *out)
{
    if ((out == NULL) || (jy901s_data.ready == 0u))
    {
        return 0;
    }

    *out = jy901s_data;
    return 1;
}

/**
  * @brief  最近一次成功读到数据的时刻
  * @retval HAL_GetTick() 时基的毫秒值
  */
uint32_t JY901S_GetUpdateTick(void)
{
    return jy_update_tick;
}

/**
  * @brief  取出并清除"上次读取是否被判为坏帧"的标志
  * @retval 1 = 发生过(通信成功但数据不可信), 0 = 没有
  * @note   用来区分两类失败:
  *           JY901S_ReadIsr() 返回 0 且本标志为 0
  *             -> I2C 传输本身就失败了(HAL 层无应答/超时)
  *           返回 0 且本标志为 1
  *             -> 传输成功, 但整帧全 0/全 FF、或角速度越界、
  *                或航向单拍跳变超过门限
  *         后者比例很高时, 通常是外部上拉不足导致误码,
  *         或者模块回传率太低(10Hz), 20ms 一读总是跨刷新点。
  */
uint8_t JY901S_TakeDiagLatch(void)
{
    uint8_t v = jy_diag_latch;

    jy_diag_latch = 0u;
    return v;
}

/**
  * @brief  当前配置的 I2C 总线速率(Hz)
  * @note   ★ 这个值来自 hi2c1.Init.ClockSpeed, 是配置值, 不是实测值。
  *         HAL 不会因为传输失败自动降速(原来注释里那句说法是错的),
  *         所以它永远返回 100000。判断信号质量请看
  *         JY901S_GetFailCount() / JY901S_GetBadCount() 的比例。
  */
uint32_t JY901S_GetBusSpeed(void)
{
    return JY901S_I2C_HANDLE->Init.ClockSpeed;
}

/**
  * @brief  读一次姿态数据(只在定时器中断里调用)
  * @retval 1 = 这一拍数据有效, 0 = 无效(调用方应沿用上一拍)
  * @note   这是全工程唯一访问 JY901S 的地方。
  *
  *         ★ 为什么放在定时器中断里
  *           原来在主循环里轮询, 一次读 2~5ms, 期间可能被中断打断;
  *           而且主循环还要刷 OLED(一次 30ms 以上), 读数节奏完全不可控。
  *           搬进 20ms 的 TIM3 中断之后:
  *             - 采样时刻严格固定, PID 的 dt 是常数
  *             - 只有这一个上下文碰 I2C, 不需要任何锁
  *             - "读完立刻算 PID", 数据新鲜度最高
  *
  *         ★ 中断里能花多久
  *           读 14 字节 @100kHz 约 1.5ms, 加上最坏 3ms 的 HAL 超时,
  *           单次最坏约 4.5ms。20ms 周期下占用不到 25%, 且期间
  *           USART1 中断(优先级 0)仍可抢占, 电机指令不会丢。
  */
uint8_t JY901S_ReadIsr(void)
{
    uint8_t buf[JY901S_REG_FAST_BYTES];

    if (jy_i2c_read_regs(JY901S_REG_FAST_START, buf, JY901S_REG_FAST_BYTES) != 0)
    {
        jy_fail_total++;
        return 0u;
    }

    if (jy_parse_fast(buf) == 0u)
    {
        jy_bad_total++;
        jy_diag_latch = 1u;
        return 0u;
    }

    jy_ok_total++;
    return 1u;
}

/**
  * @brief  立即读取一次并解析(兼容旧接口)
  * @retval 0 = 成功, 1 = I2C 通信失败或整帧数据无效
  * @note   内部就是 JY901S_ReadIsr(), 只是返回值约定反转。
  *         只在上电自检和模块配置阶段使用; 闭环运行期间由
  *         TIM3 中断直接调用 JY901S_ReadIsr()。
  */
uint8_t JY901S_ReadOnce(void)
{
    return (JY901S_ReadIsr() != 0u) ? 0u : 1u;
}

/**
  * @brief  中断内读取的统计
  */
uint32_t JY901S_GetOkCount(void)
{
    return jy_ok_total;
}

uint32_t JY901S_GetFailCount(void)
{
    return jy_fail_total;
}

uint32_t JY901S_GetBadCount(void)
{
    return jy_bad_total;
}

void JY901S_ResetStats(void)
{
    jy_ok_total   = 0u;
    jy_fail_total = 0u;
    jy_bad_total  = 0u;
    jy_fail_cnt   = 0u;
}

/**
  * @brief  轮询读取(主循环里反复调用)
  * @retval 0 = 本次成功读到数据, 1 = 读取失败, 2 = 还没到读取时刻
  * @note   内部按 JY901S_POLL_INTERVAL_MS 限频, 默认 20ms 读一次。
  *         因为主循环跑得很快, 这样写可以让调用方不用自己管理定时,
  *         又能保证每次拿到的都是最新数据。
  */
uint8_t JY901S_Poll(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  ret;

    /* 第一次调用不等待, 立刻读一帧, 缩短上电后的数据准备时间 */
    if (!jy_first_poll && ((int32_t)(now - jy_next_poll_tick) < 0))
    {
        return 2u;
    }
    jy_first_poll = 0;

    ret = JY901S_ReadOnce();

    if (ret == 0u)
    {
        jy_fail_cnt = 0;
        jy_next_poll_tick = HAL_GetTick() + JY901S_POLL_INTERVAL_MS;
    }
    else
    {
        /* 连续失败后降低重试频率, 避免主循环一直卡在 I2C 超时上 */
        if (jy_fail_cnt < 255u)
        {
            jy_fail_cnt++;
        }
        jy_next_poll_tick = HAL_GetTick() +
            ((jy_fail_cnt >= 5u) ? JY901S_POLL_RETRY_MS : JY901S_POLL_INTERVAL_MS);
    }

    return ret;
}

/**
  * @brief  航向清零
  * @note   把"当前朝向"记为 0 度基准, 之后读到/用到的 Yaw
  *         都是相对这个方向的角度。起跑前调用一次即可。
  */
void JY901S_ZeroYaw(void)
{
    JY901S_BeginUpdate();
    jy_zero_base = jy_yaw_unwrapped_raw;
    jy_update_yaw_outputs();
    JY901S_EndUpdate();
}

/**
  * @brief  设置模块的数据回传频率
  * @param  rate_code  取 JY901S_RRATE_xx, 如 JY901S_RRATE_50HZ
  * @note   写配置寄存器前必须先写 KEY 寄存器解锁(模块的安全机制)。
  *         该设置会保存在模块里, 断电不丢失, 通常只需设置一次。
  *         回传率越高, 闭环转弯的响应越及时, 但 I2C 负载也越大。
  */
void JY901S_SetOutputRate(uint8_t rate_code)
{
    uint8_t cmd[2];

    /* 第1步: 写 KEY = 0xB588 解锁 */
    cmd[0] = (uint8_t)(JY901S_KEY_UNLOCK & 0xFF);
    cmd[1] = (uint8_t)(JY901S_KEY_UNLOCK >> 8);
    (void)jy_i2c_write_regs(JY901S_REG_KEY, cmd, 2);

    HAL_Delay(2);

    /* 第2步: 写 RRATE 设置回传频率 */
    cmd[0] = rate_code;
    cmd[1] = 0x00;
    (void)jy_i2c_write_regs(JY901S_REG_RRATE, cmd, 2);
}

/**
  * @brief  把模块切换为 6 轴算法(不使用磁力计)
  * @retval 0 = 成功, 1 = 有写操作失败
  * @note   写 AXIS6 寄存器后保存参数。6 轴模式下 Yaw 是相对角度,
  *         不会再被磁力计/磁场干扰慢慢拉回某个参考值。
  */
uint8_t JY901S_SetAlgorithm6Axis(void)
{
    uint8_t cmd[2];
    uint8_t ret = 0u;

    /* 解锁 */
    cmd[0] = (uint8_t)(KEY_UNLOCK & 0xFF);
    cmd[1] = (uint8_t)(KEY_UNLOCK >> 8);
    ret |= jy_i2c_write_regs(KEY, cmd, 2);
    HAL_Delay(2);

    /* AXIS6 = ALGRITHM6 */
    cmd[0] = (uint8_t)ALGRITHM6;
    cmd[1] = 0x00u;
    ret |= jy_i2c_write_regs(AXIS6, cmd, 2);
    HAL_Delay(2);

    /* 再次解锁后保存参数, 断电不丢失 */
    cmd[0] = (uint8_t)(KEY_UNLOCK & 0xFF);
    cmd[1] = (uint8_t)(KEY_UNLOCK >> 8);
    ret |= jy_i2c_write_regs(KEY, cmd, 2);
    HAL_Delay(2);

    cmd[0] = (uint8_t)SAVE_PARAM;
    cmd[1] = 0x00u;
    ret |= jy_i2c_write_regs(SAVE, cmd, 2);
    HAL_Delay(100);

    return ret;
}

/**
  * @brief  加速度/陀螺仪校准
  * @note   调用前模块必须水平静止放置; 内部等待约 3 秒。
  *         校准后会重新清零 Yaw, 由调用者在之后执行 JY901S_ZeroYaw()。
  */
void JY901S_CalibrateAccGyro(void)
{
    uint8_t cmd[2];

    /* 开始校准 */
    cmd[0] = (uint8_t)(KEY_UNLOCK & 0xFF);
    cmd[1] = (uint8_t)(KEY_UNLOCK >> 8);
    (void)jy_i2c_write_regs(KEY, cmd, 2);
    HAL_Delay(2);

    cmd[0] = (uint8_t)CALGYROACC;
    cmd[1] = 0x00u;
    (void)jy_i2c_write_regs(CALSW, cmd, 2);

    /* 保持静止, 等待模块完成校准 */
    HAL_Delay(3000u);

    /* 结束校准 */
    cmd[0] = (uint8_t)(KEY_UNLOCK & 0xFF);
    cmd[1] = (uint8_t)(KEY_UNLOCK >> 8);
    (void)jy_i2c_write_regs(KEY, cmd, 2);
    HAL_Delay(2);

    cmd[0] = (uint8_t)NORMAL;
    cmd[1] = 0x00u;
    (void)jy_i2c_write_regs(CALSW, cmd, 2);
    HAL_Delay(50u);
}
