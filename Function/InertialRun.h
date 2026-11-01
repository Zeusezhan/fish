#ifndef __INERTIAL_RUN_H
#define __INERTIAL_RUN_H

/*
 * ============================================================
 * 惯导跑图 —— 全工程唯一的"跑"逻辑
 *
 * 两种模式共用一套状态机:
 *   IR_MODE_AUTO    自动跑图: 5 段直行, 段之间原地转 90 度
 *   IR_MODE_SINGLE  单段调试: 动作 + 时长由界面设定, 只跑一段
 *                   动作可选 直行/后退/原地左转 90 度/原地右转 90 度,
 *                   转向和自动跑图走完全相同的陀螺仪闭环代码路径
 *
 * 【自动跑图的完整路径】
 *
 *   段1 直行 ─► 左转90 ─► 段2 直行 ─► 右转90 ─► 段3 直行
 *        ─► 左转90 ─► 段4 直行 ─► 右转90 ─► 段5 直行 ─► 右转90
 *
 *   全部是"直行 + 原地转向", 已经不再使用横移。
 *   横移时麦轮辊子是侧向滚动, 抓地力只有纵向的约七成, 同样大小的
 *   自转修正压不住横移产生的偏航力矩, 表现就是"左移时修不回来"。
 *   改成"直行 + 转向"之后, 每一段都是纵向受力, 闭环效果稳定得多。
 *
 * 【每一段(直行)的完整流程】
 *
 *   ZERO ─────► COUNTDOWN ─────► MOVING ─────► STOPPING ─────► VERIFY
 *   清零编码器   1s 倒计时         按时间运行     发 8 帧立即停止   编码器确认停住
 *   使能电机     记基准航向        每 20ms 下发    等队列排空       有轮子还转 ->
 *               开航向环          带航向修正       等 150ms 稳定     单轮补停, 最多 3 次
 *
 *   VERIFY 通过后: 还有下一段 -> 回 ZERO;  最后一段 -> DONE
 *
 *   两个航向基准, 分工:
 *     s_home_yaw  = **实测**停稳角, 作下一段直行的目标 → 起跑 err = 0, 起步不偏;
 *     s_ideal_yaw = 理想累计角(0/±90/±180°…), 只用来算转向目标 → 残余不累积。
 *   横向外环按 s_ideal_yaw 积分, 所以段首不猛修, 但车仍会被慢慢拉回理想线。
 *   (单段调试例外: 转向基准取实测当前朝向, 保留"就地转 90 度"的语义。)
 *
 * 【直行纠偏可以不依赖陀螺仪】
 *   InertialRun.c 顶部的 IR_STRAIGHT_YAW_ON 宏:
 *     1(默认) 直行走航向环 + 横向外环, 整个跑图全程要求陀螺仪在线;
 *     0        直行段纯开环(自转量恒为 0), 只有转向段用陀螺仪闭环。
 *              此时单段调试选"直行/后退"不要求陀螺仪有数据。
 *   转向段永远用陀螺仪, 开关只管直行。
 *
 * 【每一段(转向)的完整流程】
 *
 *   ZERO ──► COUNTDOWN ──► TURNING ──► STOPPING ──► VERIFY
 *   清零编码器  1s 倒计时    纯自转,      同上           同上
 *   使能电机    算目标角     闭环转到
 *              (±90 度)     目标角
 *
 *   转向靠陀螺仪闭环, 不是"给个时长让车瞎转":
 *     - 目标角 = 开始转那一刻的航向 ± 90 度 (在 COUNTDOWN 阶段算好)
 *     - 每 20ms 读一次剩余角度, 按剩余量分档给自转速度
 *       (>10 度满速 / 5 度半速 / 近到位降到最低速压静摩擦)
 *     - 剩余角度进入 ±IR_TURN_TOL_DEG 就算到位, 立即停车
 *     - 超过 IR_TURN_TIMEOUT_MS 还没到位(被卡住/陀螺仪不跟)直接报错停机,
 *       绝不能让车一直原地打转
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
#define IR_SINGLE_MS_DEFAULT 2000u

/* 方向。
 *
 * ★ 横移(IR_MOVE_LEFT)只保留"左移"一个方向, 且不进单段调试的轮换表:
 *   它只用于自动跑图里"段 4 右转之后补一小段左移"的位置微调(0.2s 量级),
 *   不是用来跑整段的。横移时麦轮辊子侧向滚动、抓地力只有纵向的约七成,
 *   长距离左移闭环压不住(见 InertialRun.c 文件头), 所以别拿它当主运动。
 *
 * ★ 枚举下标必须和 InertialRun_MoveName() 的名字表逐项对齐, 也决定了
 *   单段调试的轮换顺序(见 IR_SINGLE_MOVE_COUNT)。 */
typedef enum
{
    IR_MOVE_FWD = 0,        /* 直行(自动路径全部用它) */
    IR_MOVE_BACK,           /* 后退(仅单段调试) */
    IR_MOVE_TURN_L,         /* 原地左转 90 度(自动路径 + 单段调试) */
    IR_MOVE_TURN_R,         /* 原地右转 90 度(自动路径 + 单段调试) */
    IR_MOVE_LEFT,           /* 左移(横移), 仅段 4 之后的短距离微调 */
    IR_MOVE_COUNT
} InertialMove_t;

/* 单段调试可选的动作个数: FWD / BACK / TURN_L / TURN_R。
 * IR_MOVE_LEFT 是自动路径专用的短距离微调段, 不进这个轮换表
 * (它必须排在枚举最后, 否则这里的计数会把它算进去)。 */
#define IR_SINGLE_MOVE_COUNT    ((uint8_t)IR_MOVE_LEFT)

/* 单段调试"转向"动作的固定转角(度)。和自动跑图一样是 90 度闭环,
 * 所以单段转到位后车头朝向恰好是自动跑图下一段该有的朝向。 */
#define IR_TURN_SEG_DEG         90.0f

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
    IR_COUNTDOWN,           /* 起跑倒计时(转向段在这里算好目标角) */
    IR_MOVING,              /* 直行/后退中: 平移速度 + 航向修正 */
    IR_TURNING,             /* 原地转向中: 只有自转, 闭环转到目标角度 */
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
    uint16_t    duration_ms;    /* 本段时长; 转向段这里是超时上限 */
    uint32_t    elapsed_ms;     /* 本段已运行时长 */
    float       turn_remain_deg;/* 转向中: 还差多少度(正=还要往左转), 其余状态为 0 */
    int32_t     position[IR_MOTOR_NUM];
    uint8_t     position_valid[IR_MOTOR_NUM];
    const char *status;         /* 内部阶段名, 4~8 字符 */
    const char *result;         /* 结束原因, 用于显示 */
} IrInfo_t;

/* 初始化, 上电调用一次 */
void InertialRun_Init(void);

/* 启动自动跑图: 5 段直行, 段间原地转 90 度。返回 1 = 已启动, 0 = 忙 / 陀螺仪没数据
 * (陀螺仪只在直行段走开环时才可以缺, 见 IR_STRAIGHT_YAW_ON) */
uint8_t InertialRun_StartAuto(void);

/* 启动单段调试。动作接受 FWD / BACK / TURN_L / TURN_R, 非法值退回 FWD。
 *
 * duration_ms 只对直行/后退有效, 会被夹到 IR_SINGLE_MS_MIN~MAX;
 * 转向动作的目标角固定为 ±IR_TURN_SEG_DEG, 由陀螺仪闭环转到位, 时长
 * 不起作用 —— 和自动跑图完全一致, 只是把整条路径缩成"一段来测"。 */
uint8_t InertialRun_StartSingle(InertialMove_t move, uint16_t duration_ms);

/* 请求停止。运行中(含转向中)会先走完整的"停车 + 编码器确认"流程,
 * 确认后才真正退出。 */
void InertialRun_Cancel(void);

/* 状态机主函数, 主循环每次都调它。内部不会长时间阻塞(最长的阻塞是
 * 停车确认时读 4 个电机的编码器, 每个最多 150ms, 都在停车状态下发生)。 */
void InertialRun_Task(void);

/* 1 = 状态机不在 IDLE(正在跑, 或正在停车确认) */
uint8_t InertialRun_IsBusy(void);

/* 取一份快照 */
void InertialRun_GetInfo(IrInfo_t *info);

/* 状态名: "IDLE"/"ZERO"/"WAIT"/"RUN"/"TURN"/"STOP"/"VERIFY" */
const char *InertialRun_StateName(IrState_t state);

/* 方向名: "FWD" / "BACK" / "TURN_L" / "TURN_R" / "LEFT" */
const char *InertialRun_MoveName(uint8_t move);

#endif /* __INERTIAL_RUN_H */
