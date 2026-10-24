#ifndef __INERTIAL_RUN_H
#define __INERTIAL_RUN_H

#include "inertial.h"

/*
 * ============================================================
 * 惯导自动跑图状态机
 *
 * 【每一段的完整流程】
 *
 *   ZERO ──► ZERO_WAIT ──► COUNTDOWN ──► MOVING
 *                                            │  (时间到)
 *                                            ▼
 *                                         STOPPING     发停车命令, 失败重发
 *                                            │
 *                                            ▼
 *                                         VERIFY       用编码器确认"真的停住了"
 *                                            │            有轮子还在转 -> 单独补停, 重试
 *                                            ▼
 *                                         下一段 / DONE
 *
 * 【为什么停车要单独一个状态 + 编码器确认】
 *   原来停车只调用一次 Car_Stop(), 返回值只写进字符串就完事。
 *   只要有一帧停止命令没送到(串口 BUSY / 驱动器漏帧 / 总线误码),
 *   那个轮子就会带着上一段的速度一直转下去 —— 而且 s_stop_lock
 *   还会禁止后续任何命令, 没有任何代码会去补救。
 *
 *   现在: 停车是"发命令 + 验证结果 + 补发"的闭环。验证手段是
 *   编码器 S_CPOS: 停稳后隔 150ms 读两次位置, 差值超过阈值就说明
 *   那个轮子还在动, 单独给它补一帧"立即停止", 最多重试 3 次。
 *
 * 【姿态闭环】
 *   MOVING 期间每 20ms 下发一次 Car_Mecanum(vx, vy, omega),
 *   其中 omega 来自 TIM3 中断算出的 g_yaw_output。
 *   目标航向 = 本段起跑瞬间的当前航向, 所以每一段都是独立地"保持直行"。
 * ============================================================
 */

typedef enum
{
    INERTIAL_RUN_IDLE = 0,
    INERTIAL_RUN_COUNTDOWN,
    INERTIAL_RUN_MOVING,
    INERTIAL_RUN_STOPPING,
    INERTIAL_RUN_VERIFY,
    INERTIAL_RUN_ZERO,
    INERTIAL_RUN_ZERO_WAIT
} InertialRunState_t;

/* VERIFY 状态内部的两个阶段 */
typedef enum
{
    INERTIAL_VERIFY_WAIT = 0,   /* 等车完全停稳, 之后读第一次位置 */
    INERTIAL_VERIFY_CHECK       /* 再等一段时间读第二次, 比较两次的差值 */
} InertialVerifyPhase_t;

typedef struct
{
    InertialRunState_t state;
    uint8_t segment_index;
    uint8_t running;
    uint8_t heading_hold;       /* 1 = 本段正在做航向闭环 */
    uint8_t stop_retry;         /* 已经重试停车的次数 */
    uint16_t duration_ms;
    uint32_t start_tick;
    InertialMove_t move;
    int32_t position[INERTIAL_MOTOR_NUM];
    uint8_t position_valid[INERTIAL_MOTOR_NUM];
    const char *status;
    const char *result;
} InertialRunInfo_t;

/* 主循环里反复调用 InertialRun_Task(), 它内部完全非阻塞(除了停车时的
 * 串口发送和位置读取, 都有超时上限)。 */
void InertialRun_Init(void);
uint8_t InertialRun_Start(void);
void InertialRun_Cancel(void);
void InertialRun_Task(void);
void InertialRun_GetInfo(InertialRunInfo_t *info);
const char *InertialRun_GetStateName(InertialRunState_t state);

#endif
