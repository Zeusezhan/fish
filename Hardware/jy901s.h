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

/* ---- 中断内闭环用的"精简读取区间" ----
 *
 * 姿态闭环只需要两个量: 航向角 Yaw(0x3F) 和 Z 轴角速度 GZ(0x39)。
 * 它们之间隔着磁场和 Roll/Pitch, 所以最小的一段连续区间是
 *   0x39(GZ) ~ 0x3F(Yaw) = 7 个寄存器 = 14 字节
 * 100kHz 下约 1.5ms, 只有原方案(读两遍 24 字节, 约 4.6ms)的三分之一。
 *
 * 剩下这 1.5ms 就是"定时器中断被占用的时间", 直接决定中断能开多快,
 * 所以这个区间不要随意扩大。
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
 * 20ms 一拍, 车再快也不可能转过 30 度; 超过就说明这一帧被模块的
 * 数据刷新"切"开了(半个旧帧 + 半个新帧), 直接丢弃, 沿用上一拍。 */
#define JY901S_MAX_YAW_STEP     30.0f

/* 角速度合理范围(度/秒)。超出满量程的读数必然是坏帧。
 * JY901S 出厂量程 ±2000deg/s, 这里留一半余量做门限。 */
#define JY901S_MAX_GZ_ABS       1200.0f

/* 配置寄存器地址与解锁码(写参数前必须先写 KEY 解锁) */
#define JY901S_REG_KEY          0x69
#define JY901S_REG_RRATE        0x03
#define JY901S_KEY_UNLOCK       0xB588u

/* 回传频率取值(写 RRATE 寄存器用), 出厂默认 10Hz */
#define JY901S_RRATE_10HZ       0x06
#define JY901S_RRATE_50HZ       0x08
#define JY901S_RRATE_100HZ      0x09

/* I2C 单次传输超时(ms)
 * ★ 现在是"在定时器中断里读", 所以这个值直接等于"中断最多被卡多久"。
 *   100kHz 下读 14 字节约 1.5ms, 3ms 足够; 设大了会让控制节拍被吃掉。 */
#define JY901S_I2C_TIMEOUT      3

/* 正常轮询间隔(ms) —— 自检模式下主循环用, 闭环时由 TIM3 的 20ms 节拍驱动 */
#define JY901S_POLL_INTERVAL_MS 20

/* 连续读失败后的重试间隔(ms)。
 * ★ 必须明显小于闭环模块的"IMU 掉线判据"(100ms), 否则出现
 *   "刚失败几次就被判掉线强制停车"的自相矛盾行为 —— 原来这里是
 *   200ms, 比 100ms 的判据还长, 所以 I2C 一抖动车就停在半路。 */
#define JY901S_POLL_RETRY_MS    20

/* 相邻两次读取的容差(寄存器原始值) —— 已废弃, 保留定义只为兼容旧代码。
 * 现在的坏帧判据改用"航向跳变门限 + 角速度范围", 见 JY901S_MAX_YAW_STEP。 */
#define JY901S_BURST_TOLERANCE  400

/* 模块解算后的数据 */
typedef struct
{
    float ax, ay, az;        /* 加速度, 单位 g —— 精简读取后不再更新, 恒为 0 */
    float gx, gy;            /* X/Y 轴角速度 deg/s —— 精简读取后不再更新, 恒为 0 */
    float gz;                /* Z 轴角速度 deg/s (左转 gz 为正) */
    float roll;              /* 横滚角 deg */
    float pitch;             /* 俯仰角 deg */
    float yaw;               /* 偏航角 deg, 已相对清零, -180~180 */
    float yaw_continuous;    /* 偏航角 deg, 已解缠绕, 可超过 +/-180 */
    volatile uint8_t ready;  /* 是否收到过有效数据 */
} JY901S_Data_t;

/* 模块解算数据, 全局只读, 外部直接访问字段即可:
 *   jy901s_data.yaw            相对航向角 -180~180 度, 左转为正
 *   jy901s_data.yaw_continuous 连续航向角, 不在 ±180 处跳变, 可超过 ±180
 *   jy901s_data.gz             Z 轴角速度 deg/s, 左转为正(做微分阻尼用)
 *   jy901s_data.roll / .pitch  横滚角 / 俯仰角 deg(只用于显示和调平)
 *   jy901s_data.ready          0 = 还没收到数据, 1 = 数据正常
 *   (ax/ay/az/gx/gy 为了兼容旧代码保留字段, 精简读取后恒为 0) */
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

/* ==================== 中断内读取(闭环主路径) ====================
 *
 * 【设计要点】
 *   全工程只有这一个地方读 I2C, 而且只在 TIM3 中断里调用。
 *   "只有一个上下文访问总线"本身就消灭了原来所有的并发问题:
 *     - 不会再有"主循环读到一半被中断打断"
 *     - 不会再有 NVIC_DisableIRQ / EnableIRQ 当互斥锁的嵌套问题
 *     - 不会再有"读两次比对失败 -> 拉长重试间隔 -> 被判掉线"的连锁反应
 *
 * 【坏帧判据】代替原来的"读两遍逐字节比对"
 *   1. 整帧全 0x00 / 全 0xFF  -> 总线悬空或模块没应答
 *   2. |GZ| 超过 JY901S_MAX_GZ_ABS -> 必然是坏数据
 *   3. 航向单拍跳变超过 JY901S_MAX_YAW_STEP -> 跨过了模块刷新点
 *   任何一条不满足就丢弃本拍, jy901s_data 保持上一拍的合法值,
 *   闭环照常用上一拍的数据继续算 —— 绝不停摆。
 *
 * 【返回值】1 = 本拍数据有效, 0 = 无效
 */
uint8_t JY901S_ReadIsr(void);

/* 【诊断】中断内读取的统计: 成功 / 通信失败 / 坏帧被丢弃 的次数。
 * 用来判断是"接线信号差"还是"模块刷新率和读取节奏对不上"。 */
uint32_t JY901S_GetOkCount(void);
uint32_t JY901S_GetFailCount(void);
uint32_t JY901S_GetBadCount(void);
void     JY901S_ResetStats(void);

/* ==================== 中断安全的读取快照 ====================
 *
 * JY901S_BeginUpdate() / EndUpdate() 仍然保留, 但只用于上电阶段的
 * 零点和模块配置(那时 TIM3 还没启动)。闭环运行期间不要调用它们。
 */

/* 标记"正在写 jy901s_data", 期间屏蔽 TIM3 中断。 */
void JY901S_BeginUpdate(void);

/* 结束写状态, 恢复 TIM3 中断。 */
void JY901S_EndUpdate(void);

/* 【兜底】取一份数据快照。
 * 返回: 1 = 读到有效数据, 0 = 还没数据。只在中断里调用, 内部不做 I2C 通信。 */
uint8_t JY901S_TryRead(JY901S_Data_t *out);

#endif /* __JY901S_H */
