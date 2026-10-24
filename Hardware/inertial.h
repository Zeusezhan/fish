#ifndef __INERTIAL_H
#define __INERTIAL_H

/*
 * ============================================================
 * 惯导跑图节点数据(基于四轮 EMM_V5 编码器实测)
 *
 * 【数据来源】
 *   底盘以 50 RPM 运行, 每段开始前都把四轮位置计数清零,
 *   停车后读取 S_CPOS 当前位置脉冲计数, 记录如下:
 *
 *     段 | 动作 | 时长  | M1(右前) | M2(右后) | M3(左前) | M4(左后)
 *     ---+------+-------+----------+----------+----------+----------
 *      1 | 直行 | 3.3 s | -163366  | -163376  |  163364  |  163316
 *      2 | 左移 | 3.8 s | -190748  |  190824  | -190716  |  190860
 *      3 | 直行 | 4.8 s | -245282  | -245226  |  245424  |  245312
 *      4 | 左移 | 4.4 s | -223452  |  223526  | -223504  |  223569
 *      5 | 直行 | 4.9 s | -250764  | -250806  |  250700  |  250712
 *
 * 【符号规律】(已核对, 五段完全一致)
 *   直行: M1/M2 为负, M3/M4 为正
 *   左移: M1/M3 为负, M2/M4 为正
 *   这直接反映了底盘电机当前的方向映射, 不要随意改符号。
 *
 * ★ 这组数据是在"1=右前 2=右后 3=左前 4=左后、MOTOR_x_DIR_INVERT=0,0,1,1"
 *   那套映射下采集的(见 function.c 顶部)。直行时该映射下发 dir=[1,1,0,0],
 *   对应脉冲符号 [-,-,+,+]; 左移时下发 dir=[1,0,1,0], 对应 [-,+,-,+]。
 *   八个数据点与该映射全部自洽 —— 所以以后若改动电机映射,
 *   这张表和下面的数据必须重新采集, 否则编号会整体错位。
 *
 * 【数据质量】
 *   同一段内四轮脉冲数的极差都在 200 以内,
 *   折算速度稳定在约 49500~51200 脉冲/秒, 一致性好。
 *
 * 【单位换算】
 *   脉冲 -> 距离: 需要标定"每米脉冲数"。
 *   用任意直行段即可估算, 例如段1:
 *       速度 = 163356 / 3.3 ≈ 49502 脉冲/秒
 *   目前没有实测位移, 所以模块只提供脉冲数, 不做毫米换算。
 *
 * 【重要限制】
 *   这些数据是"按时间跑出来的结果", 不是"跑到目标脉冲数就停"。
 *   直接用它们做闭环到点判断时, 要注意:
 *   1. 起停加减速过程已被平均进总脉冲, 短距离误差更明显;
 *   2. 电池电压、地面打滑会改变同样的时间对应的脉冲数;
 *   3. 目前只有直行和左移两种动作, 没有右移/后退/转向数据。
 * ============================================================
 */

#include <stdint.h>

/* 每段的最大电机数 */
#define INERTIAL_MOTOR_NUM      4u

/* 实测时使用的运行速度(RPM), 与调试页 MENU_DEBUG_SPEED_RPM 一致 */
#define INERTIAL_TEST_SPEED_RPM 50u

/* 记录时的节点总数 */
#define INERTIAL_SEGMENT_NUM    5u

/* 动作类型 */
typedef enum
{
    INERTIAL_MOVE_FORWARD = 0,  /* 直行 */
    INERTIAL_MOVE_LEFT,         /* 左移 */
    INERTIAL_MOVE_RIGHT,        /* 右移(暂无数据) */
    INERTIAL_MOVE_BACKWARD      /* 后退(暂无数据) */
} InertialMove_t;

/* 单个节点(一段动作)的实测记录 */
typedef struct
{
    InertialMove_t type;        /* 动作类型 */
    uint16_t duration_ms;       /* 实测运行时长(ms) */
    int32_t  encoder[INERTIAL_MOTOR_NUM]; /* M1~M4 的 S_CPOS 计数 */
} InertialSegment_t;

/* 五段实测数据, 按测试顺序排列 */
extern const InertialSegment_t g_inertial_segments[INERTIAL_SEGMENT_NUM];

/* 【查询】按序号取一段数据(0~4), 序号越界返回 0 */
uint8_t Inertial_GetSegment(uint8_t index, InertialSegment_t *out);

/* 【查询】取某段里某个电机的绝对值脉冲数(motor 为 1~4), 越界返回 0 */
uint32_t Inertial_GetMagnitude(uint8_t index, uint8_t motor);

/* 【统计】把某段的四轮绝对值做平均, 得到该段的代表性脉冲数。
 * 用于后续标定"脉冲 -> 距离"或做到达判断。越界返回 0。 */
uint32_t Inertial_GetAverageMagnitude(uint8_t index);

/* 【统计】估算脉冲速率(脉冲/秒), 用四轮绝对值平均除以时长。
 * 越界或时长为 0 返回 0。 */
uint32_t Inertial_GetPulseRate(uint8_t index);

/* 【统计】按动作类型统计平均脉冲速率(脉冲/秒)。
 * 没有该类数据时返回 0。 */
uint32_t Inertial_GetPulseRateByType(InertialMove_t type);

#endif /* __INERTIAL_H */
