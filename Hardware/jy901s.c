/* ============================================================
 * JY901S 九轴姿态传感器驱动 (I2C)
 *
 * 【数据流】
 *   JY901S_ReadIsr()                    <-- 只在 TIM3 中断里调用
 *     -> jy_i2c_read_regs(0x39, buf, 14)   一次读 7 个寄存器
 *     -> jy_parse_fast()   小端解析 + 合理性校验
 *     -> 通过校验才写 jy901s_data
 *
 * 【坏帧判据】
 *   通信失败 / 全 0 / 全 FF / 角速度越界 / 航向单拍跳变超限,
 *   命中任何一条就丢弃本拍, jy901s_data 保持上一拍的合法值。
 *   调用方(中断)拿到 0 之后沿用上一拍输出继续闭环, 绝不停摆。
 *
 * 【禁止事项】
 *   本文件里不允许出现 HAL_Delay。TIM3 优先级 5 高于 SysTick 的
 *   默认优先级, 在中断里等 tick 会永久卡死。
 * ============================================================ */

#include "main.h"
#include "i2c.h"
#include "gpio.h"
#include "jy901s.h"

/* 模块解算数据(外部只读) */
JY901S_Data_t jy901s_data;

/* ---- Yaw 解缠绕用的内部变量(存的是模块原始角度, 不带方向符号) ---- */
static uint8_t  jy_has_yaw = 0u;         /* 是否已经收到第一帧 */
static float    jy_last_raw_yaw = 0.0f;  /* 上一帧原始 Yaw, 用来检测跳变 */
static int32_t  jy_wrap_cnt = 0;         /* 越过 ±180 的圈数 */
static float    jy_yaw_unwrapped_raw = 0.0f;  /* 解缠绕后的原始角度 */

/* ---- 软件清零基准: 对外角度 = 解缠绕角度 - 该基准 ---- */
static float    jy_zero_base = 0.0f;

/* ---- 统计 ---- */
static uint32_t jy_update_tick = 0;      /* 最近一次成功解析的时刻 */
static uint32_t jy_ok_total   = 0;       /* 有效帧 */
static uint32_t jy_fail_total = 0;       /* I2C 通信失败 */
static uint32_t jy_bad_total  = 0;       /* 通信成功但被判为坏帧 */

/* ==================== I2C 底层 ==================== */

/**
  * @brief  总线自恢复
  * @note   上一次传输异常中断时, I2C 外设会停在 BUSY 状态, 之后所有
  *         读写都会立即失败。这里检测到非 READY 就复位一次外设。
  *         ★ 这里绝对不能有 HAL_Delay —— 本函数会被定时器中断调用。
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
  * @brief  从指定寄存器起连续读若干字节
  * @retval 0 = 成功, 1 = 失败
  * @note   只用 STM32 标准的 Mem_Read 时序(寄存器地址 + 重复起始 + 读)。
  */
static uint8_t jy_i2c_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    jy_i2c_check_recover();

    if (HAL_I2C_Mem_Read(JY901S_I2C_HANDLE, (uint16_t)(JY901S_I2C_ADDR << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, buf, len, JY901S_I2C_TIMEOUT) == HAL_OK)
    {
        return 0u;
    }

    return 1u;
}

/**
  * @brief  向指定寄存器连续写若干字节(只用于写模块配置)
  * @retval 0 = 成功, 1 = 失败
  */
static uint8_t jy_i2c_write_regs(uint8_t reg, const uint8_t *buf, uint16_t len)
{
    uint8_t frame[JY901S_REG_FAST_BYTES + 1u];
    uint16_t i;

    if (len > JY901S_REG_FAST_BYTES)
    {
        return 1u;
    }

    jy_i2c_check_recover();

    /* 首时序: 寄存器地址 + 重复起始 + 数据 */
    if (HAL_I2C_Mem_Write(JY901S_I2C_HANDLE, (uint16_t)(JY901S_I2C_ADDR << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len,
                          JY901S_I2C_TIMEOUT) == HAL_OK)
    {
        return 0u;
    }

    /* 备用时序: 寄存器地址与数据拼成一帧发 */
    frame[0] = reg;
    for (i = 0u; i < len; i++)
    {
        frame[i + 1u] = buf[i];
    }

    if (HAL_I2C_Master_Transmit(JY901S_I2C_HANDLE, (uint16_t)(JY901S_I2C_ADDR << 1),
                                frame, (uint16_t)(len + 1u), JY901S_I2C_TIMEOUT) == HAL_OK)
    {
        return 0u;
    }

    return 1u;
}

/* ==================== 数据解析 ==================== */

/**
  * @brief  根据解缠绕后的原始角度更新对外的 yaw
  * @note   同时更新两个输出:
  *           yaw_continuous —— 连续角度, 可超过 ±180(闭环用这个)
  *           yaw            —— 折算回 -180~180(显示用)
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
  * @param  range  该量的满量程
  */
static float jy_scale(int16_t raw, float range)
{
    return (float)raw / 32768.0f * range;
}

/**
  * @brief  校验并解析 14 字节原始缓冲区(寄存器 0x39~0x3F)
  * @retval 1 = 这一帧有效且已更新 jy901s_data, 0 = 坏帧
  * @note   校验按"代价从低到高"排列, 发现不对立刻返回,
  *         绝不让坏数据进入 PID。
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
    for (i = 0u; i < JY901S_REG_FAST_BYTES; i++)
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

    /* ---- 1. 取出各个寄存器(小端, 低字节在前) ---- */
    raw_gz    = (int16_t)(((uint16_t)buf[1]  << 8) | buf[0]);    /* 0x39 GZ    */
    raw_roll  = (int16_t)(((uint16_t)buf[9]  << 8) | buf[8]);    /* 0x3D Roll  */
    raw_pitch = (int16_t)(((uint16_t)buf[11] << 8) | buf[10]);   /* 0x3E Pitch */
    raw_yaw   = (int16_t)(((uint16_t)buf[13] << 8) | buf[12]);   /* 0x3F Yaw   */

    gz  = jy_scale(raw_gz, 2000.0f);    /* 角速度满量程 ±2000 deg/s */
    yaw = jy_scale(raw_yaw, 180.0f);    /* 角度满量程 ±180 deg */

    /* ---- 2. 角速度范围 ---- */
    if ((gz > JY901S_MAX_GZ_ABS) || (gz < -JY901S_MAX_GZ_ABS))
    {
        return 0u;
    }

    /* ---- 3. 航向解缠绕 + 单拍跳变校验 ----
     * 模块的 Yaw 在 +180/-180 之间跳变, 先按跳变方向累加圈数把它"拉直",
     * 再看这一拍相对上一拍走了多少度。超过门限说明读到了
     * "半个旧帧 + 半个新帧", 宁可丢这一拍也不能让假角度进 PID。 */
    wrap_cnt = jy_wrap_cnt;

    if (jy_has_yaw == 0u)
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
    jy901s_data.roll  = jy_scale(raw_roll, 180.0f);
    jy901s_data.pitch = jy_scale(raw_pitch, 180.0f);

    jy_update_yaw_outputs();
    jy901s_data.ready = 1u;
    jy_update_tick = HAL_GetTick();

    return 1u;
}

/* ==================== 对外接口 ==================== */

void JY901S_Init(void)
{
    jy901s_data.ready = 0u;
    jy901s_data.gz = 0.0f;
    jy901s_data.roll = 0.0f;
    jy901s_data.pitch = 0.0f;
    jy901s_data.yaw = 0.0f;
    jy901s_data.yaw_continuous = 0.0f;

    jy_has_yaw = 0u;
    jy_last_raw_yaw = 0.0f;
    jy_wrap_cnt = 0;
    jy_yaw_unwrapped_raw = 0.0f;
    jy_zero_base = 0.0f;

    jy_update_tick = HAL_GetTick();
    jy_ok_total   = 0u;
    jy_fail_total = 0u;
    jy_bad_total  = 0u;
}

/**
  * @brief  读一次姿态数据(只在定时器中断里调用)
  * @retval 1 = 这一拍有效, 0 = 无效(调用方沿用上一拍)
  * @note   这是全工程唯一访问 JY901S 的地方。
  *
  *         读 14 字节 @100kHz 约 1.5ms, 加上最坏 3ms 的 HAL 超时,
  *         单次最坏约 4.5ms。20ms 周期下占用不到 25%, 且期间
  *         USART1 中断(优先级 0)仍可抢占, 电机指令不会丢。
  */
uint8_t JY901S_ReadIsr(void)
{
    uint8_t buf[JY901S_REG_FAST_BYTES];

    if (jy_i2c_read_regs(JY901S_REG_FAST_START, buf, JY901S_REG_FAST_BYTES) != 0u)
    {
        jy_fail_total++;
        return 0u;
    }

    if (jy_parse_fast(buf) == 0u)
    {
        jy_bad_total++;
        return 0u;
    }

    jy_ok_total++;
    return 1u;
}

/**
  * @brief  航向清零: 把"当前朝向"记为 0 度基准
  * @note   上电调用一次(TIM3 启动之前)。内部短暂屏蔽 TIM3,
  *         防止中断在计算基准的过程中改掉 jy_yaw_unwrapped_raw。
  */
void JY901S_ZeroYaw(void)
{
    NVIC_DisableIRQ(TIM3_IRQn);

    jy_zero_base = jy_yaw_unwrapped_raw;
    jy_update_yaw_outputs();

    NVIC_EnableIRQ(TIM3_IRQn);
}

/**
  * @brief  设置模块回传频率
  * @note   写配置前必须先写 KEY 寄存器解锁。该设置保存在模块里,
  *         断电不丢失。★ 只能在 TIM3 启动之前调用(阻塞 I2C 写)。
  */
void JY901S_SetOutputRate(uint8_t rate_code)
{
    uint8_t cmd[2];

    /* 第 1 步: 写 KEY = 0xB588 解锁 */
    cmd[0] = (uint8_t)(JY901S_KEY_UNLOCK & 0xFFu);
    cmd[1] = (uint8_t)(JY901S_KEY_UNLOCK >> 8);
    (void)jy_i2c_write_regs(JY901S_REG_KEY, cmd, 2u);

    HAL_Delay(2);

    /* 第 2 步: 写 RRATE 设置回传频率 */
    cmd[0] = rate_code;
    cmd[1] = 0x00u;
    (void)jy_i2c_write_regs(JY901S_REG_RRATE, cmd, 2u);
}

uint32_t JY901S_GetUpdateTick(void)
{
    return jy_update_tick;
}

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
}
