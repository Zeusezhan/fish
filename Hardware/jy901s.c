/* ============================================================
 * JY901S 九轴姿态传感器驱动 (I2C)
 *
 * 【数据流】
 *   JY901S_Poll() / JY901S_ReadOnce()
 *     -> jy_i2c_read_regs(0x34, buf, 24)   一次读 12 个寄存器
 *     -> 小端解析成 12 个 int16 (AX..Yaw)
 *     -> jy_parse_regs()  换算物理量 + Yaw 解缠绕
 *
 * 【重要说明】
 *   JY901S 的 I2C 读有两种时序, 不同批次的模块固件支持的时序不一样,
 *   所以 jy_i2c_read_regs() 里两条都试, 任一条成功即可, 提高兼容性。
 * ============================================================ */

#include "main.h"
#include "i2c.h"
#include "gpio.h"
#include "jy901s.h"

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
static uint32_t jy_last_poll_tick = 0;
static uint8_t  jy_first_poll = 1;       /* 1 = 第一次调用立刻读, 不等间隔 */

/* ==================== I2C 底层 ==================== */

/**
  * @brief  总线自恢复
  * @note   如果上一次传输异常中断, I2C 外设会停在 BUSY 状态,
  *         之后所有读写都会立即失败且无法自行恢复。
  *         这里检测到非 READY 就复位一次外设, 让后续读写能继续。
  */
static void jy_i2c_check_recover(void)
{
    if (HAL_I2C_GetState(JY901S_I2C_HANDLE) != HAL_I2C_STATE_READY)
    {
        HAL_I2C_DeInit(JY901S_I2C_HANDLE);
        HAL_Delay(2);
        HAL_I2C_Init(JY901S_I2C_HANDLE);
    }
}

/**
  * @brief  从指定寄存器起连续读取若干字节
  * @param  reg  起始寄存器地址
  * @param  buf  接收缓冲区
  * @param  len  要读取的字节数
  * @retval 0 = 成功, 1 = 失败
  * @note   依次尝试两种 I2C 时序, 兼容不同批次的模块固件:
  *           路径A: 寄存器地址 + 重复起始 + 读(STM32 的 Mem_Read 标准时序)
  *           路径B: 先写寄存器地址并 STOP, 再单独发起一次读
  */
static uint8_t jy_i2c_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t reg_tmp = reg;

    jy_i2c_check_recover();

    /* ---- 路径A: 寄存器地址 + 重复起始 + 读 ---- */
    if (HAL_I2C_Mem_Read(JY901S_I2C_HANDLE, (uint16_t)(JY901S_I2C_ADDR << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, buf, len, JY901S_I2C_TIMEOUT) == HAL_OK)
    {
        return 0;
    }

    /* ---- 路径B: 写寄存器地址(带STOP) + 单独读 ---- */
    if (HAL_I2C_Master_Transmit(JY901S_I2C_HANDLE, (uint16_t)(JY901S_I2C_ADDR << 1),
                                &reg_tmp, 1, JY901S_I2C_TIMEOUT) == HAL_OK)
    {
        HAL_Delay(2);
        if (HAL_I2C_Master_Receive(JY901S_I2C_HANDLE, (uint16_t)(JY901S_I2C_ADDR << 1),
                                   buf, len, JY901S_I2C_TIMEOUT) == HAL_OK)
        {
            return 0;
        }
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
  * @brief  把 12 个原始寄存器换算成物理量并更新 jy901s_data
  * @param  reg  12 个原始寄存器值
  * @note   下标约定: 0=AX 1=AY 2=AZ  3=GX 4=GY 5=GZ
  *                   6=HX 7=HY 8=HZ  9=Roll 10=Pitch 11=Yaw
  *
  *         Yaw 解缠绕原理: 模块输出的 Yaw 会在 +180/-180 之间跳变,
  *         直接使用的话, 小车转过 180° 时角度会从 179 突然变成 -179,
  *         PID 会误以为要反转。这里通过检测相邻两帧的差值(超过 180°
  *         说明发生了一次跳变), 累加圈数 jy_wrap_cnt 把角度"拉直",
  *         得到可以无限累计的连续角度。
  */
static void jy_parse_regs(const int16_t *reg)
{
    float raw_yaw;
    float diff;

    /* 加速度, 满量程 ±16g */
    jy901s_data.ax = jy_scale(reg[0], 16.0f);
    jy901s_data.ay = jy_scale(reg[1], 16.0f);
    jy901s_data.az = jy_scale(reg[2], 16.0f);

    /* 角速度, 满量程 ±2000deg/s; gz 乘方向符号, 与 yaw 保持同号(左转为正) */
    jy901s_data.gx = jy_scale(reg[3], 2000.0f);
    jy901s_data.gy = jy_scale(reg[4], 2000.0f);
    jy901s_data.gz = jy_scale(reg[5], 2000.0f) * JY901S_YAW_SIGN;

    /* 角度, 满量程 ±180 */
    jy901s_data.roll  = jy_scale(reg[9], 180.0f);
    jy901s_data.pitch = jy_scale(reg[10], 180.0f);

    /* ---- Yaw 解缠绕 ---- */
    raw_yaw = jy_scale(reg[11], 180.0f);

    if (!jy_has_yaw)
    {
        /* 第一帧: 只记录基准, 不判跳变 */
        jy_has_yaw = 1;
        jy_last_raw_yaw = raw_yaw;
    }

    diff = raw_yaw - jy_last_raw_yaw;
    if (diff > 180.0f)
    {
        jy_wrap_cnt--;      /* 正向越过 +180 跳变到 -180 */
    }
    else if (diff < -180.0f)
    {
        jy_wrap_cnt++;      /* 反向越过 -180 跳变到 +180 */
    }
    jy_last_raw_yaw = raw_yaw;
    jy_yaw_unwrapped_raw = raw_yaw + (float)jy_wrap_cnt * 360.0f;

    jy_update_yaw_outputs();
    jy901s_data.ready = 1;
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

    jy_last_poll_tick = HAL_GetTick();
    jy_first_poll = 1;
}

/**
  * @brief  立即读取一次姿态数据并解析
  * @retval 0 = 成功, 1 = I2C 通信失败(数据保持上一次的值不变)
  * @note   一次读取 0x34~0x3F 共 12 个寄存器(24 字节):
  *         加速度 AX~AZ + 角速度 GX~GZ + 磁场 HX~HZ + 角度 Roll/Pitch/Yaw
  */
uint8_t JY901S_ReadOnce(void)
{
    uint8_t buf[JY901S_REG_BYTES];
    int16_t reg[JY901S_REG_NUM];
    uint8_t i;

    if (jy_i2c_read_regs(JY901S_REG_AX, buf, JY901S_REG_BYTES) != 0)
    {
        return 1;
    }

    for (i = 0; i < JY901S_REG_NUM; i++)
    {
        /* 维特智能 I2C 数据是小端: 低字节在前 */
        reg[i] = (int16_t)(((uint16_t)buf[i * 2 + 1] << 8) | buf[i * 2]);
    }

    jy_parse_regs(reg);
    return 0;
}

/**
  * @brief  轮询读取(主循环里反复调用)
  * @note   内部按 JY901S_POLL_INTERVAL_MS 限频, 默认 20ms 读一次。
  *         因为主循环跑得很快, 这样写可以让调用方不用自己管理定时,
  *         又能保证每次拿到的都是最新数据。
  */
void JY901S_Poll(void)
{
    uint32_t now = HAL_GetTick();

    /* 第一次调用不等待, 立刻读一帧, 缩短上电后的数据准备时间 */
    if (!jy_first_poll && (now - jy_last_poll_tick < JY901S_POLL_INTERVAL_MS))
    {
        return;
    }
    jy_first_poll = 0;
    jy_last_poll_tick = now;

    (void)JY901S_ReadOnce();
}

/**
  * @brief  航向清零
  * @note   把"当前朝向"记为 0 度基准, 之后读到/用到的 Yaw
  *         都是相对这个方向的角度。起跑前调用一次即可。
  */
void JY901S_ZeroYaw(void)
{
    jy_zero_base = jy_yaw_unwrapped_raw;
    jy_update_yaw_outputs();
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
