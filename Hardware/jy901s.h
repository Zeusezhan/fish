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

/* I2C 单次传输超时(ms)。读取失败时尽快返回, 避免阻塞主循环。 */
#define JY901S_I2C_TIMEOUT      10

/* 正常轮询间隔(ms) */
#define JY901S_POLL_INTERVAL_MS 20

/* 连续读取失败后的重试间隔(ms) */
#define JY901S_POLL_RETRY_MS    200

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
 * 调用频率高了也不会增加 I2C 负担。 */
void JY901S_Poll(void);

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

/* 【设置 6 轴算法】禁用磁力计参与姿态解算并保存模块配置。
 * 返回: 0 = 成功, 非 0 = 至少一次 I2C 写入失败。 */
uint8_t JY901S_SetAlgorithm6Axis(void);

/* 【加速度/陀螺仪校准】模块需水平静止放置, 调用期间阻塞约 3 秒。
 * 校准完成后如需重新设定航向基准, 再调用 JY901S_ZeroYaw()。 */
void JY901S_CalibrateAccGyro(void);

#endif /* __JY901S_H */
