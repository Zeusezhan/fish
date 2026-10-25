#ifndef __INERTIAL_RUN_H
#define __INERTIAL_RUN_H

/*
 * ============================================================
 * 惯导跑图 —— 全工程唯一的"跑"逻辑
 *
 * 两种模式共用一套状态机:
 *   IR_MODE_AUTO    自动跑图: 预设 5 段(直行/左移交替), 每段给定时长
 *   IR_MODE_SINGLE  单段调试: 方向 + 时长由界面设定, 只跑一段
 *
 * 【每一段的完整流程】
 *
 *   ZERO ──────► COUNTDOWN ──────► MOVING ──────► STOPPING ──────► VERIFY
 *   清零编码器    1s 倒计时          按时间运行      发 8 帧立即停止    编码器确认停住
 *   使能电机      记目标航向         每 20ms 下发     等队列排空        有轮子还转 -> 单轮补停
 *                开航向环           带航向修正        等 150ms 稳定      最多 3 次
 *
 *   VERIFY 通过后: 还有下一段 -> 回 ZERO;  最后一段 -> DONE
 *
 * 【为什么要 VERIFY 这一步】
 *   停车命令只保证"发出去了", 不保证"驱动器收到了"。丢掉一帧就会出现
 *   "某个轮子一直转", 而且是偶发的。VERIFY 用编码器 S_CPOS 实测验证:
 *   停稳 150ms 后读一次位置作基准, 再隔 150ms 读第二次, 差值超过门限
 *   就说明那台还在转, 单独给它补一帧立即停止。
 * ============================================================
 */

#include <stdint.h>

/* 底盘电机个数(和 Car_ReadPositionCount 的上限一致) */
#define IR_MOTOR_NUM    4u

/* 单段时长允许范围(单段调试用) */
#define IR_SINGLE_MS_MIN    500u
#define IR_SINGLE_MS_MAX    10000u

/* 方向 */
typedef enum
{
    IR_MOVE_FWD = 0,        /* 直行 */
    IR_MOVE_LEFT,           /* 左移 */
    IR_MOVE_RIGHT,          /* 右移 */
    IR_MOVE_BACK,           /* 后退 */
    IR_MOVE_COUNT
} InertialMove_t;

/* 模式 */
typedef enum
{
    IR_MODE_AUTO = 0,
    IR_MODE_SINGLE
} InertialMode_t;

/* 状态 */
typedef enum
{
    IR_IDLE = 0,
    IR_ZERO,                /* 清零编码器 + 使能电机 */
    IR_COUNTDOWN,           /* 起跑倒计时 */
    IR_MOVING,              /* 运行中 */
    IR_STOPPING,            /* 发停车命令 + 等队列排空 + 等稳定 */
    IR_VERIFY               /* 编码器确认是否真的停住 */
} IrState_t;

/* 对外快照(只读), 供界面显示 */
typedef struct
{
    IrState_t   state;
    uint8_t     mode;           /* InertialMode_t */
    uint8_t     index;          /* 当前段号, 0 起 */
    uint8_t     total;          /* 总段数 */
    uint8_t     move;           /* InertialMove_t */
    uint8_t     running;        /* 1 = 正在跑(可以下发速度) */
    uint16_t    duration_ms;    /* 本段时长 */
    uint32_t    elapsed_ms;     /* 本段已运行时长 */
    int32_t     position[IR_MOTOR_NUM];
    uint8_t     position_valid[IR_MOTOR_NUM];
    const char *status;         /* 内部阶段名, 4~8 字符 */
    const char *result;         /* 结束原因, 用于显示 */
} IrInfo_t;

/* 初始化, 上电调用一次 */
void InertialRun_Init(void);

/* 启动自动跑图。返回 1 = 已启动, 0 = 忙 / 陀螺仪没数据 */
uint8_t InertialRun_StartAuto(void);

/* 启动单段调试。duration_ms 会被夹到 IR_SINGLE_MS_MIN~MAX */
uint8_t InertialRun_StartSingle(InertialMove_t move, uint16_t duration_ms);

/* 请求停止。运行中会先走完整的"停车 + 编码器确认"流程, 确认后才真正退出。 */
void InertialRun_Cancel(void);

/* 状态机主函数, 主循环每次都调它。内部不会长时间阻塞(最长的阻塞是
 * 停车确认时读 4 个电机的编码器, 每个最多 40ms, 都在停车状态下发生)。 */
void InertialRun_Task(void);

/* 1 = 状态机不在 IDLE(正在跑, 或正在停车确认) */
uint8_t InertialRun_IsBusy(void);

/* 取一份快照 */
void InertialRun_GetInfo(IrInfo_t *info);

/* 状态名(调试显示用) */
const char *InertialRun_StateName(IrState_t state);

/* 方向名: "FWD" / "LEFT" / "RIGHT" / "BACK" */
const char *InertialRun_MoveName(uint8_t move);

#endif /* __INERTIAL_RUN_H */
