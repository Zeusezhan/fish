#ifndef __JY901S_H
#define __JY901S_H

/*
 * ============================================================
 * JY901S (WT901) 九轴姿态传感器驱动 —— I2C 版
 * 厂商: 维特智能(WitMotion)
 *
 * 【硬件接线】
 *   SCL -> PB6 (I2C1_SCL)
 *   SDA -> PB7 (I2C1_SDA)
 *   VCC -> 3.3V 或 5V,  GND -> GND
 *   ★ 建议 SCL/SDA 各接一个 4.7kΩ 上拉到 3.3V。只用 STM32 内部
 *     约 40kΩ 弱上拉撑 100kHz, 边沿太慢, 是丢帧的主要来源。
 *
 * 【方向约定】
 *   模块水平安装, 箭头(X 轴)指向车头, Z 轴垂直向上:
 *   小车左转(俯视逆时针) -> Yaw 增大, gz 为正。
 *   若实际装反, 把 JY901S_YAW_SIGN 改成 -1.0f 即可, 其余代码不用动。
 *
 * 【设计要点: 读取只发生在定时器中断里】
 *   JY901S_ReadIsr() 是全工程唯一的 I2C 访问点, 只在 TIM3 的 20ms
 *   中断里调用。因为只有这一个上下文碰总线, 所以:
 *     - 不会读到"改了一半"的数据
 *     - 不需要任何互斥量 / 临界区
 *     - 采样时刻严格固定, PID 的 dt 是常数
 *   代价是中断里会阻塞约 1.5ms(最坏 3ms 超时), 所以驱动里
 *   **不允许出现 HAL_Delay** —— TIM3 优先级高于 SysTick, 等 tick 就是死锁。
 * ============================================================
 */

#include "main.h"

/* ---------------- 用户可配置参数 ---------------- */

/* I2C 7 位地址(出厂默认 0x50) */
#define JY901S_I2C_ADDR         0x50

/* I2C 句柄(CubeMX 生成的是 hi2c1) */
#define JY901S_I2C_HANDLE       (&hi2c1)

/* Yaw 方向符号: 左转角度增大用 +1.0f, 相反则用 -1.0f */
#define JY901S_YAW_SIGN         (1.0f)

/* ---- 闭环用的"精简读取区间" ----
 *
 * 姿态闭环只需要两个量: 航向角 Yaw(0x3F) 和 Z 轴角速度 GZ(0x39)。
 * 它们之间隔着磁场和 Roll/Pitch, 所以最小的一段连续区间是
 *   0x39(GZ) ~ 0x3F(Yaw) = 7 个寄存器 = 14 字节
 * 100kHz 下约 1.5ms —— 这个时间直接等于"定时器中断被占用多久",
 * 所以不要再把区间往 0x34(加速度)那一端扩。
 *
 * 区间内各寄存器在缓冲区里的字节偏移:
 *   0x39 GZ    -> buf[0..1]
 *   0x3A~0x3C  -> 磁场, 不使用
 *   0x3D Roll  -> buf[8..9]
 *   0x3E Pitch -> buf[10..11]
 *   0x3F Yaw   -> buf[12..13]
 */
#define JY901S_REG_FAST_START   0x39
#define JY901S_REG_FAST_NUM     7
#define JY901S_REG_FAST_BYTES   (JY901S_REG_FAST_NUM * 2)

/* 单拍航向跳变门限(度)。
 * 20ms 一拍, 车再快也转不过 30 度; 超过就说明这一帧被模块的数据刷新
 * "切"开了(半个旧帧 + 半个新帧), 直接丢弃, 沿用上一拍。 */
#define JY901S_MAX_YAW_STEP     30.0f

/* 角速度合理范围(度/秒)。超出满量程一半的读数必然是坏帧。 */
#define JY901S_MAX_GZ_ABS       1200.0f

/* I2C 单次传输超时(ms)。
 * ★ 它直接等于"定时器中断最多被卡多久", 100kHz 读 14 字节约 1.5ms,
 *   3ms 足够; 设大了会让控制节拍被吃掉。 */
#define JY901S_I2C_TIMEOUT      3

/* ---- 模块配置寄存器 ---- */
#define JY901S_REG_KEY          0x69    /* 解锁寄存器 */
#define JY901S_REG_RRATE        0x03    /* 回传频率 */
#define JY901S_KEY_UNLOCK       0xB588u /* 写入 KEY 才能改配置 */

/* 回传频率取值(写 RRATE 寄存器用), 出厂默认 10Hz */
#define JY901S_RRATE_10HZ       0x06
#define JY901S_RRATE_50HZ       0x08
#define JY901S_RRATE_100HZ      0x09
#define JY901S_RRATE_200HZ      0x0B

/* 模块解算后的数据 */
typedef struct
{
    float gz;                /* Z 轴角速度 deg/s (左转为正), 做微分阻尼用 */
    float roll;              /* 横滚角 deg */
    float pitch;             /* 俯仰角 deg */
    float yaw;               /* 偏航角 deg, 相对清零基准, -180~180 */
    float yaw_continuous;    /* 偏航角 deg, 已解缠绕, 可超过 ±180 */
    volatile uint8_t ready;  /* 是否收到过有效数据 */
} JY901S_Data_t;

/* 模块解算数据, 全局只读。
 *   yaw            相对航向角 -180~180 度, 左转为正
 *   yaw_continuous 连续航向角, 不在 ±180 处跳变(闭环用这个)
 *   gz             Z 轴角速度 deg/s, 左转为正
 *   ready          0 = 还没收到数据, 1 = 数据正常 */
extern JY901S_Data_t jy901s_data;

/* ==================== 对外接口 ==================== */

/* 【初始化】上电调用一次, 只清软件状态, 不阻塞等待模块。 */
void JY901S_Init(void);

/* 【读一帧】★ 只在 TIM3 中断里调用, 全工程唯一的 I2C 访问点。
 * 内部做: 读 14 字节 -> 校验 -> 解缠绕 -> 更新 jy901s_data。
 * 返回: 1 = 这一拍数据有效, 0 = 无效(调用方应沿用上一拍的值)。 */
uint8_t JY901S_ReadIsr(void);

/* 【航向清零】把当前朝向记为 0 度基准。
 * 上电时调用一次(TIM3 启动之前), 之后所有 Yaw 都相对这个方向。 */
void JY901S_ZeroYaw(void);

/* 【设置回传率】rate_code 取 JY901S_RRATE_xx。
 * 该设置写入模块后断电不丢失, 上电设置一次即可。
 * ★ 只能在 TIM3 启动之前调用(内部是阻塞 I2C 写)。 */
void JY901S_SetOutputRate(uint8_t rate_code);

/* 【查询】最近一次成功读到数据的时刻(HAL_GetTick 时基)。
 * 用来判断陀螺仪是否掉线: 与 HAL_GetTick() 相减超过一定时间
 * 就说明 I2C 出问题了, 必须停车。 */
uint32_t JY901S_GetUpdateTick(void);

/* 【诊断】读取统计: 有效帧 / I2C 通信失败 / 通信成功但被判为坏帧 */
uint32_t JY901S_GetOkCount(void);
uint32_t JY901S_GetFailCount(void);
uint32_t JY901S_GetBadCount(void);
void     JY901S_ResetStats(void);

#endif /* __JY901S_H */
