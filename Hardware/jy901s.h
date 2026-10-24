#ifndef __JY901S_H
#define __JY901S_H

/*
 * ============================================================
 * JY901S (WT901 / Y901S) 九轴姿态传感器驱动 —— I2C 版
 * 厂商: 维特智能(WitMotion)
 *
 * 【硬件接线】
 *   SCL -> PB6 (I2C1_SCL)
 *   SDA -> PB7 (I2C1_SDA)
 *   VCC -> 3.3V 或 5V,  GND -> GND
 *   建议 SCL/SDA 各接一个 4.7kΩ 上拉电阻到电源。
 *
 * 【寄存器(与官方 REG.h 一致)】
 *   0x34~0x36 : 加速度 AX/AY/AZ
 *   0x37~0x39 : 角速度 GX/GY/GZ
 *   0x3A~0x3C : 磁场   HX/HY/HZ
 *   0x3D~0x3F : 角度   Roll/Pitch/Yaw
 *   一次连续读 12 个寄存器, 数据为小端(低字节在前)。
 *
 * 【方向约定】
 *   模块水平安装, 箭头(X轴)指向车头, Z轴垂直向上:
 *   小车左转(俯视逆时针) -> Yaw 增大, gz 为正。
 *   若实际装反, 把 JY901S_YAW_SIGN 改成 -1.0f 即可, 其余代码不动。
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

/* 起始寄存器与寄存器个数: 0x34(AX) ~ 0x3F(Yaw) */
#define JY901S_REG_AX           0x34
#define JY901S_REG_NUM          12
#define JY901S_REG_BYTES        (JY901S_REG_NUM * 2)

/* 配置寄存器地址与解锁码(写参数前必须先写 KEY 解锁) */
#define JY901S_REG_KEY          0x69
#define JY901S_REG_RRATE        0x03
#define JY901S_KEY_UNLOCK       0xB588u

/* 回传频率取值(写 RRATE 寄存器用), 出厂默认 10Hz */
#define JY901S_RRATE_10HZ       0x06
#define JY901S_RRATE_50HZ       0x08
#define JY901S_RRATE_100HZ      0x09

/* I2C 单次传输超时(ms)
 * ★ 不能设太大: 读失败时 HAL 会一直等到超时, 主循环(OLED/按键/上报)
 *   都会被卡住。100kHz 下读 24 字节约 2.2ms, 10ms 足够。 */
#define JY901S_I2C_TIMEOUT      10

/* 正常轮询间隔(ms) */
#define JY901S_POLL_INTERVAL_MS 20

/* 连续读失败后的重试间隔(ms): 避免 IMU 掉线时主循环一直被 I2C 超时拖住 */
#define JY901S_POLL_RETRY_MS    200

/* 相邻两次读取的容差(寄存器原始值, 满量程 32768)。
 * 同一个采样点的两次读取只会差几个 LSB(噪声);
 * 跨过模块刷新点的帧会差成百上千。取 400 约等于 0.7° 的航向、
 * 1.2°/s 的角速度、0.2g 的加速度, 足够区分两者又不误杀好帧。
 * 若模块回传率很高(100Hz)仍频繁丢帧, 可适当调大。 */
#define JY901S_BURST_TOLERANCE  400

/* 模块解算后的数据 */
typedef struct
{
    float ax, ay, az;        /* 加速度, 单位 g */
    float gx, gy, gz;        /* 角速度, 单位 deg/s (左转 gz 为正) */
    float roll;              /* 横滚角 deg */
    float pitch;             /* 俯仰角 deg */
    float yaw;               /* 偏航角 deg, 已相对清零, -180~180 */
    float yaw_continuous;    /* 偏航角 deg, 已解缠绕, 可超过 +/-180 */
    volatile uint8_t ready;  /* 是否收到过有效数据 */
} JY901S_Data_t;

/* 模块解算数据, 全局只读, 外部直接访问字段即可:
 *   jy901s_data.yaw            相对航向角 -180~180 度, 左转为正
 *   jy901s_data.yaw_continuous 连续航向角, 不在 ±180 处跳变, 可超过 ±180
 *   jy901s_data.gz             Z 轴角速度 deg/s, 左转为正
 *   jy901s_data.roll / .pitch  横滚角 / 俯仰角 deg
 *   jy901s_data.ax/.ay/.az     三轴加速度 g
 *   jy901s_data.gx/.gy         另两轴角速度 deg/s
 *   jy901s_data.ready          0 = 还没收到数据, 1 = 数据正常 */
extern JY901S_Data_t jy901s_data;

/* ==================== 对外接口 ==================== */

/* 【初始化】上电调用一次。
 * 只做软件层面的准备(清零内部状态), 不阻塞等待模块,
 * 是否真正拿到数据看 jy901s_data.ready。 */
void JY901S_Init(void);

/* 【轮询读取】放在 while(1) 主循环里反复调用即可。
 * 内部按 JY901S_POLL_INTERVAL_MS 限频(默认 20ms 读一次),
 * 调用频率高了也不会增加 I2C 负担。
 * 返回值: 0 = 本次成功读到数据, 1 = 读取失败, 2 = 还没到读取时刻。 */
uint8_t JY901S_Poll(void);

/* 【立即读取】不受轮询间隔限制, 立刻读一次并解析。
 * 返回: 0 = 成功, 1 = I2C 通信失败或整帧数据无效。 */
uint8_t JY901S_ReadOnce(void);

/* 【航向清零】把当前朝向记为 0 度基准。
 * 起跑前调用一次, 之后所有 Yaw 都是相对这个方向的。 */
void JY901S_ZeroYaw(void);

/* 【设置回传频率】rate_code 取 JY901S_RRATE_xx。
 * 出厂 10Hz, 做转弯闭环建议改成 50Hz;
 * 注意: 该设置写入模块后断电不丢失, 只需设置一次。
 * 例: JY901S_SetOutputRate(JY901S_RRATE_50HZ); */
void JY901S_SetOutputRate(uint8_t rate_code);

/* 【设置 6 轴算法】写 AXIS6 并保存参数。返回 0 = 成功。
 * 6 轴模式下 Yaw 是相对角度, 不会被磁力计慢慢拉回某个参考值。 */
uint8_t JY901S_SetAlgorithm6Axis(void);

/* 【加速度/陀螺仪校准】调用前模块必须水平静止, 内部等待约 3 秒。
 * 校准完成后如需重新设定航向基准, 再调用 JY901S_ZeroYaw()。 */
void JY901S_CalibrateAccGyro(void);

/* 【查询】最近一次成功读到数据的时刻(HAL_GetTick 时基)。
 * 用来判断陀螺仪是否掉线: 与 HAL_GetTick() 相减超过一定时间
 * 就说明 I2C 出问题了, 应停止闭环运动。 */
uint32_t JY901S_GetUpdateTick(void);

/* 【诊断】取出并清除"上次读取是否发生过容差比对失败"的标志。
 * 配合 JY901S_ReadOnce() 的返回值可以区分失败类型:
 *   返回 1 且本函数返回 0 -> I2C 传输本身失败(无应答/超时)
 *   返回 1 且本函数返回 1 -> 传输成功但两次数据差太多(跨刷新点/误码) */
uint8_t JY901S_TakeDiagLatch(void);

/* 【诊断】当前生效的 I2C 总线速率(Hz)。
 * 传输失败时驱动会自动降速重试, 所以掉到 100k 以下说明信号质量差。 */
uint32_t JY901S_GetBusSpeed(void);

/* ==================== 中断安全的快速读取 ====================
 *
 * 【为什么需要这一套】
 *   I2C 读取是主循环里"轮询 + 阻塞"的: 一次读 24 字节要 2~3ms。
 *   如果 TIM3 中断在 I2C 传输中途进来读 jy901s_data,
 *   就会和主循环抢同一份数据(读到一半被改), 造成 yaw 跳变。
 *
 * 【做法】
 *   JY901S_ReadOnce() 是驱动里唯一的写入口, 它会在写之前
 *   调 JY901S_BeginUpdate(), 写完之后调 JY901S_EndUpdate()。
 *   中断里先用 JY901S_TryRead() 拿快照, 拿不到就跳过本次运算。
 *
 *   JY901S_BeginUpdate() 期间 TIM3 被屏蔽, 所以中断不会读到
 *   写了一半的数据 —— 代价只有每次读取的几毫秒, 20ms 周期完全够用。
 */

/* 标记"主循环正在写 jy901s_data", 期间屏蔽 TIM3 中断。 */
void JY901S_BeginUpdate(void);

/* 结束写状态, 恢复 TIM3 中断。 */
void JY901S_EndUpdate(void);

/* 【中断里用】取一份一致的快照。
 * 返回: 1 = 读到有效数据(前面已经出现过至少一帧), 0 = 还没数据。
 * 只在中断里调用, 内部不做 I2C 通信。 */
uint8_t JY901S_TryRead(JY901S_Data_t *out);

#endif /* __JY901S_H */
