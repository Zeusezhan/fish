/* ============================================================
 * 惯导跑图状态机(全工程唯一的"跑"逻辑)
 *
 * 【预设路径实测数据】50 RPM, 每段开始前四轮 S_CPOS 清零
 *
 *   段 | 动作 | 时长  | M1(右前) | M2(右后) | M3(左前) | M4(左后)
 *   ---+------+-------+----------+----------+----------+----------
 *    1 | 直行 | 3.3 s | -163366  | -163376  |  163364  |  163316
 *    2 | 左移 | 3.8 s | -190748  |  190824  | -190716  |  190860
 *    3 | 直行 | 4.8 s | -245282  | -245226  |  245424  |  245312
 *    4 | 左移 | 4.4 s | -223452  |  223526  | -223504  |  223569
 *    5 | 直行 | 4.9 s | -250764  | -250806  |  250700  |  250712
 *
 *   符号规律(五段完全一致): 直行 M1/M2 为负、M3/M4 为正;
 *   左移 M1/M3 为负、M2/M4 为正 —— 直接反映底盘的方向映射, 不要随意改。
 *   同一段内四轮脉冲极差都在 200 以内, 速度约 49500~51200 脉冲/秒。
 *
 *   ★ 这组数据是在 "1=右前 2=右后 3=左前 4=左后、DIR_INVERT=0,0,1,1"
 *     那套映射下采的。若改动 function.c 里的电机映射, 本表必须重采。
 * ============================================================ */

#include "main.h"
#include "jy901s.h"
#include "function.h"
#include "InertialRun.h"
#include <string.h>

/* ==================== 可调参数 ==================== */

/* 平移速度(RPM)。和上面采数据时一致, 改它必须重采时长表。 */
#define IR_SPEED_RPM            50

/* 航向修正量限幅(RPM)。横移时 50RPM 平移配 ±30RPM 修正,
 * 轮速落在 20~80 之间, 既有纠偏能力又不会甩尾。 */
#define IR_TURN_MAX             30.0f

/* 运行中下发底盘命令的周期(ms)。每拍都重发同一组速度是故意的:
 * 命令幂等, 驱动器漏收一帧下一拍就补上了, 比"只在变化时发"抗干扰。 */
#define IR_STEER_MS             20u

/* 起跑前倒计时(ms), 给驱动器使能、机械稳定留时间 */
#define IR_LEAD_MS              1000u

/* 停车命令发出后等多久再读第一次位置(ms) */
#define IR_STOP_SETTLE_MS       150u

/* 两次位置读取之间的间隔(ms) */
#define IR_VERIFY_GAP_MS        150u

/* "轮子还在转"的判据(编码器脉冲差值)。
 * 50RPM 时约 49500 脉冲/秒, 150ms 约 7400 脉冲;
 * 真正停住的轮子只有几个脉冲的抖动, 取 40 既能容忍噪声,
 * 又能抓住"以 1RPM 以上速度在转"的情况。 */
#define IR_MOVE_THRESHOLD       40

/* 停车确认最多重试几次(单轮补停) */
#define IR_VERIFY_MAX_RETRY     3u

/* 位置清零后等待(ms), 让驱动器把 S_CPOS 真正写掉 */
#define IR_ZERO_WAIT_MS         300u

/* ==================== 内部类型 ==================== */

typedef struct
{
    uint8_t  move;          /* InertialMove_t */
    uint16_t duration_ms;
} IrPathSeg_t;

typedef enum
{
    IR_VERIFY_WAIT = 0,     /* 读第一次位置作基准 */
    IR_VERIFY_CHECK         /* 再读一次做比较 */
} IrVerifyPhase_t;

/* ==================== 预设路径 ==================== */

static const IrPathSeg_t k_auto_path[] =
{
    { IR_MOVE_FWD,   3300u },
    { IR_MOVE_LEFT,  3800u },
    { IR_MOVE_FWD,   4800u },
    { IR_MOVE_LEFT,  4400u },
    { IR_MOVE_FWD,   4900u }
};

#define IR_AUTO_SEG_NUM ((uint8_t)(sizeof(k_auto_path) / sizeof(k_auto_path[0])))

/* ==================== 状态 ==================== */

static IrState_t         s_state = IR_IDLE;
static IrVerifyPhase_t   s_verify_phase = IR_VERIFY_WAIT;

static uint8_t           s_mode = IR_MODE_AUTO;
static uint8_t           s_total = 0u;
static uint8_t           s_index = 0u;

static uint8_t           s_cur_move = IR_MOVE_FWD;
static uint16_t          s_cur_ms = 0u;

/* 单段调试的设定值 */
static uint8_t           s_single_move = IR_MOVE_FWD;
static uint16_t          s_single_ms = 2000u;

static uint8_t           s_running = 0u;
static uint8_t           s_cancel = 0u;

/* ZERO / STOPPING 这两个"一次性动作"的进度标志 */
static uint8_t           s_action_pushed = 0u;
static uint8_t           s_settle_armed = 0u;

static uint8_t           s_verify_retry = 0u;
static uint8_t           s_verify_reliable = 1u;
static uint8_t           s_stuck_mask = 0u;

static uint32_t          s_deadline = 0u;
static uint32_t          s_start_tick = 0u;
static uint32_t          s_next_steer_tick = 0u;

static int32_t           s_position[IR_MOTOR_NUM];
static uint8_t           s_position_valid[IR_MOTOR_NUM];

static const char       *s_status = "READY";
static const char       *s_result = "NONE";

/* ==================== 小工具 ==================== */

static uint32_t ir_abs32(int32_t v)
{
    if (v >= 0)
    {
        return (uint32_t)v;
    }
    return (uint32_t)(-(v + 1)) + 1u;
}

/* 把航向环算出的浮点修正量四舍五入并限幅成整数 RPM */
static int16_t ir_turn_from_yaw(void)
{
    float u = g_yaw_output;

    if (u > IR_TURN_MAX)
    {
        u = IR_TURN_MAX;
    }
    else if (u < -IR_TURN_MAX)
    {
        u = -IR_TURN_MAX;
    }

    if (u >= 0.0f)
    {
        return (int16_t)(u + 0.5f);
    }
    return (int16_t)(u - 0.5f);
}

static void ir_abort(const char *reason)
{
    s_state = IR_IDLE;
    s_running = 0u;
    s_cancel = 0u;
    s_action_pushed = 0u;
    s_settle_armed = 0u;
    YawLoop_Disable();

    s_status = reason;
    s_result = reason;
}

/* 按当前段的方向, 带着航向修正下发一帧麦轮速度。
 * 惯导跑图永远是闭环的: 没有陀螺仪数据时启动会被拒绝,
 * 运行中掉线会立刻停车, 所以这里不需要"开环"分支。 */
static void ir_apply_move(void)
{
    int16_t turn = ir_turn_from_yaw();
    int16_t spd = IR_SPEED_RPM;

    switch (s_cur_move)
    {
        case IR_MOVE_LEFT:
            Car_Mecanum(0, spd, turn);
            break;

        case IR_MOVE_RIGHT:
            Car_Mecanum(0, (int16_t)(-spd), turn);
            break;

        case IR_MOVE_BACK:
            Car_Mecanum((int16_t)(-spd), 0, turn);
            break;

        case IR_MOVE_FWD:
        default:
            Car_Mecanum(spd, 0, turn);
            break;
    }
}

/* 把当前段号对应的动作/时长装进 s_cur_move / s_cur_ms */
static void ir_load_segment(void)
{
    if (s_mode == IR_MODE_SINGLE)
    {
        s_cur_move = s_single_move;
        s_cur_ms = s_single_ms;
        return;
    }

    if (s_index >= IR_AUTO_SEG_NUM)
    {
        s_index = (uint8_t)(IR_AUTO_SEG_NUM - 1u);
    }

    s_cur_move = k_auto_path[s_index].move;
    s_cur_ms = k_auto_path[s_index].duration_ms;
}

/* 结束运动, 进入停车流程 */
static void ir_begin_stop(const char *status)
{
    /* 顺序很重要: 先关航向环, 再发停车。
     * 反过来的话, 中断可能在停车命令之后又算出一个新的修正量,
     * 主循环下一拍就会把车重新开动。 */
    s_running = 0u;
    YawLoop_Disable();

    s_action_pushed = 0u;
    s_settle_armed = 0u;
    s_verify_retry = 0u;
    s_verify_reliable = 1u;
    s_stuck_mask = 0u;

    s_state = IR_STOPPING;
    s_status = status;
}

/* 读一遍四个轮子的位置计数, 返回 1 = 四个都读到了 */
static uint8_t ir_read_positions(void)
{
    uint8_t i;
    uint8_t all_ok = 1u;

    for (i = 0u; i < IR_MOTOR_NUM; i++)
    {
        s_position_valid[i] = Car_ReadPositionCount((uint8_t)(i + 1u),
                                                    &s_position[i]);
        if (s_position_valid[i] == 0u)
        {
            all_ok = 0u;
        }
    }

    return all_ok;
}

/**
  * @brief  停车确认: 用编码器判断四个轮子是不是真的停住了
  * @note   阶段一: 读一次位置作基准 A
  *         阶段二: 隔 150ms 再读一次, 与 A 逐轮比较, 超过门限就补停那台
  */
static void ir_verify(uint32_t now)
{
    uint8_t i;

    if ((int32_t)(now - s_deadline) < 0)
    {
        return;
    }

    /* ---- 阶段一: 记录基准位置 ---- */
    if (s_verify_phase == IR_VERIFY_WAIT)
    {
        s_verify_reliable = ir_read_positions();

        if (s_verify_reliable == 0u)
        {
            /* 读不到位置就没法确认停车。补发一次停止命令再试,
             * 连续失败就整体报错停机 —— 带着"不知道轮子停没停"的
             * 状态跑下一段是最危险的。 */
            s_verify_retry++;
            if (s_verify_retry > IR_VERIFY_MAX_RETRY)
            {
                ir_abort("POS ERR");
                return;
            }
            Car_Stop();
            s_settle_armed = 0u;
            s_state = IR_STOPPING;
            s_status = "POS RETRY";
            return;
        }

        s_verify_phase = IR_VERIFY_CHECK;
        s_deadline = now + IR_VERIFY_GAP_MS;
        s_status = "VERIFY";
        return;
    }

    /* ---- 阶段二: 再读一次, 和基准比较 ---- */
    s_stuck_mask = 0u;

    for (i = 0u; i < IR_MOTOR_NUM; i++)
    {
        int32_t pos_now = 0;
        int32_t delta;

        if (Car_ReadPositionCount((uint8_t)(i + 1u), &pos_now) == 0u)
        {
            s_verify_reliable = 0u;
            continue;
        }

        delta = pos_now - s_position[i];
        if (ir_abs32(delta) > (uint32_t)IR_MOVE_THRESHOLD)
        {
            s_stuck_mask |= (uint8_t)(1u << i);
        }

        s_position[i] = pos_now;
        s_position_valid[i] = 1u;
    }

    if (s_verify_reliable == 0u)
    {
        s_verify_retry++;
        if (s_verify_retry > IR_VERIFY_MAX_RETRY)
        {
            ir_abort("POS ERR");
            return;
        }
        Car_Stop();
        s_verify_phase = IR_VERIFY_WAIT;
        s_settle_armed = 0u;
        s_state = IR_STOPPING;
        s_status = "POS RETRY";
        return;
    }

    if (s_stuck_mask != 0u)
    {
        /* 有轮子还在转: 单独给它补两帧立即停止 */
        for (i = 0u; i < IR_MOTOR_NUM; i++)
        {
            if ((s_stuck_mask & (uint8_t)(1u << i)) != 0u)
            {
                Car_StopMotor((uint8_t)(i + 1u));
            }
        }

        s_verify_retry++;
        if (s_verify_retry > IR_VERIFY_MAX_RETRY)
        {
            /* 补了三次还在转, 说明不是丢帧而是驱动器本身有问题 */
            Car_Stop();
            ir_abort("WHEEL STUCK");
            return;
        }

        s_verify_phase = IR_VERIFY_WAIT;
        s_settle_armed = 0u;
        s_state = IR_STOPPING;
        s_status = "WHEEL RETRY";
        return;
    }

    /* ---- 四个轮子都确认停住了 ---- */
    if (s_cancel != 0u)
    {
        ir_abort("USER STOP");
        return;
    }

    /* 陀螺仪掉线导致的停车不能"接着跑下一段":
     * 没有航向反馈的惯导跑图毫无意义, 直接终止整趟任务。 */
    if (YawLoop_IsImuAlive() == 0)
    {
        ir_abort("IMU LOST");
        return;
    }

    s_verify_phase = IR_VERIFY_WAIT;
    s_verify_retry = 0u;
    s_status = "STOPPED";

    /* 最后一段: 直接收尾, 不清零位置 —— 那组 S_CPOS 就是这一段的位移 */
    if ((uint8_t)(s_index + 1u) >= s_total)
    {
        s_state = IR_IDLE;
        s_status = "ALL DONE";
        s_result = "DONE";
        return;
    }

    /* 还有下一段: 段号+1, 回 ZERO 清零编码器 */
    s_index++;
    s_action_pushed = 0u;
    s_state = IR_ZERO;
    s_deadline = now;
}

/* ==================== 对外接口 ==================== */

void InertialRun_Init(void)
{
    uint8_t i;

    s_state = IR_IDLE;
    s_verify_phase = IR_VERIFY_WAIT;
    s_mode = IR_MODE_AUTO;
    s_total = 0u;
    s_index = 0u;
    s_cur_move = IR_MOVE_FWD;
    s_cur_ms = 0u;
    s_running = 0u;
    s_cancel = 0u;
    s_action_pushed = 0u;
    s_settle_armed = 0u;
    s_verify_retry = 0u;
    s_verify_reliable = 1u;
    s_stuck_mask = 0u;
    s_deadline = 0u;
    s_start_tick = 0u;
    s_next_steer_tick = 0u;
    s_status = "READY";
    s_result = "NONE";

    for (i = 0u; i < IR_MOTOR_NUM; i++)
    {
        s_position[i] = 0;
        s_position_valid[i] = 0u;
    }
}

/**
  * @brief  内部: 两种模式共用的启动准备
  * @retval 1 = 已启动, 0 = 忙或陀螺仪没数据
  */
static uint8_t ir_start(uint8_t mode, uint8_t total)
{
    uint8_t i;

    if (s_state != IR_IDLE)
    {
        s_status = "BUSY";
        return 0u;
    }

    /* 没有航向反馈就跑惯导是没有意义的, 直接拒绝启动 */
    if (YawLoop_IsImuAlive() == 0)
    {
        s_status = "NO IMU";
        s_result = "NO IMU";
        return 0u;
    }

    s_mode = mode;
    s_total = total;
    s_index = 0u;
    s_cancel = 0u;
    s_running = 0u;
    s_action_pushed = 0u;
    s_settle_armed = 0u;
    s_verify_retry = 0u;
    s_stuck_mask = 0u;

    YawLoop_Disable();

    for (i = 0u; i < IR_MOTOR_NUM; i++)
    {
        s_position_valid[i] = 0u;
    }

    s_status = "START";
    s_result = "RUN";

    /* 先进 ZERO: 清零编码器 + 使能电机, 再倒计时起跑 */
    s_state = IR_ZERO;
    s_deadline = HAL_GetTick();

    return 1u;
}

uint8_t InertialRun_StartAuto(void)
{
    return ir_start(IR_MODE_AUTO, IR_AUTO_SEG_NUM);
}

uint8_t InertialRun_StartSingle(InertialMove_t move, uint16_t duration_ms)
{
    if ((uint8_t)move >= (uint8_t)IR_MOVE_COUNT)
    {
        move = IR_MOVE_FWD;
    }

    if (duration_ms < IR_SINGLE_MS_MIN)
    {
        duration_ms = IR_SINGLE_MS_MIN;
    }
    else if (duration_ms > IR_SINGLE_MS_MAX)
    {
        duration_ms = IR_SINGLE_MS_MAX;
    }

    s_single_move = (uint8_t)move;
    s_single_ms = duration_ms;

    return ir_start(IR_MODE_SINGLE, 1u);
}

void InertialRun_Cancel(void)
{
    if (s_state == IR_IDLE)
    {
        return;
    }

    switch (s_state)
    {
        case IR_MOVING:
            s_cancel = 1u;
            ir_begin_stop("USER STOP");
            break;

        case IR_STOPPING:
        case IR_VERIFY:
            /* 停车流程已经在走了, 让它走完再收尾 */
            s_cancel = 1u;
            break;

        case IR_ZERO:
        case IR_COUNTDOWN:
        default:
            /* 轮子本来就是停着的, 直接收尾 */
            ir_abort("USER STOP");
            break;
    }
}

uint8_t InertialRun_IsBusy(void)
{
    return (s_state != IR_IDLE) ? 1u : 0u;
}

void InertialRun_Task(void)
{
    uint32_t now = HAL_GetTick();

    switch (s_state)
    {
        /* ---- 清零编码器 + 使能电机 ---- */
        case IR_ZERO:
            if (s_action_pushed == 0u)
            {
                if (Car_ResetPositionCounts() == 0u)
                {
                    ir_abort("ZERO ERR");
                    break;
                }
                s_action_pushed = 1u;
                s_deadline = now + IR_ZERO_WAIT_MS;
                s_status = "ZERO";
                break;
            }

            /* 等清零帧真的发出去, 再等驱动器把计数写掉 */
            if (Car_TxBusy() != 0u)
            {
                break;
            }
            if ((int32_t)(now - s_deadline) < 0)
            {
                break;
            }

            s_action_pushed = 0u;

            /* 使能放在倒计时之前, 这样 1 秒的倒计时足够 4 帧使能帧发完,
             * 起跑瞬间不会有"使能还没到"的延迟。 */
            if (Car_SetEnabled(1u) == 0u)
            {
                ir_abort("ENA ERR");
                break;
            }

            ir_load_segment();
            s_state = IR_COUNTDOWN;
            s_deadline = now + IR_LEAD_MS;
            s_status = "READY";
            break;

        /* ---- 起跑 ---- */
        case IR_COUNTDOWN:
            if ((int32_t)(now - s_deadline) < 0)
            {
                break;
            }

            if (s_cancel != 0u)
            {
                ir_abort("USER STOP");
                break;
            }

            /* 起跑前再确认一次陀螺仪还在出数。
             * 惯导跑图必须闭环, 没有反馈宁可不起跑。 */
            if (YawLoop_IsImuAlive() == 0)
            {
                ir_abort("IMU LOST");
                break;
            }

            /* 目标航向 = 本段起跑瞬间的当前航向。
             * 每段都是独立地"保持朝这个方向走", 上一段的漂移不会被带进下一段。 */
            YawLoop_Enable(jy901s_data.yaw_continuous, IR_TURN_MAX);

            ir_apply_move();

            s_running = 1u;
            s_start_tick = now;
            s_next_steer_tick = now + IR_STEER_MS;
            s_status = "RUN";
            s_state = IR_MOVING;
            break;

        /* ---- 运行中 ---- */
        case IR_MOVING:
            if (s_running == 0u)
            {
                break;
            }

            /* 陀螺仪掉线就停车: 没有航向反馈的闭环比开环更危险 */
            if (YawLoop_IsImuAlive() == 0)
            {
                ir_begin_stop("IMU LOST");
                break;
            }

            if ((uint32_t)(now - s_start_tick) >= (uint32_t)s_cur_ms)
            {
                ir_begin_stop("STOPPING");
                break;
            }

            /* 每 20ms 重发一次底盘命令 */
            if ((int32_t)(now - s_next_steer_tick) >= 0)
            {
                s_next_steer_tick = now + IR_STEER_MS;
                ir_apply_move();
            }
            break;

        /* ---- 发停车 + 等队列排空 + 等稳定 ---- */
        case IR_STOPPING:
            if (s_action_pushed == 0u)
            {
                Car_Stop();
                s_action_pushed = 1u;
                s_settle_armed = 0u;
                s_status = "STOPPING";
                break;
            }

            /* 队列里还有帧没发出去, 继续等 */
            if (Car_TxBusy() != 0u)
            {
                break;
            }

            /* 队列排空了, 开始计稳定时间 */
            if (s_settle_armed == 0u)
            {
                s_settle_armed = 1u;
                s_deadline = now + IR_STOP_SETTLE_MS;
                break;
            }

            if ((int32_t)(now - s_deadline) < 0)
            {
                break;
            }

            s_action_pushed = 0u;
            s_settle_armed = 0u;
            s_verify_phase = IR_VERIFY_WAIT;
            s_deadline = now;
            s_state = IR_VERIFY;
            break;

        /* ---- 编码器确认停车 ---- */
        case IR_VERIFY:
            ir_verify(now);
            break;

        case IR_IDLE:
        default:
            break;
    }
}

void InertialRun_GetInfo(IrInfo_t *info)
{
    uint8_t i;

    if (info == NULL)
    {
        return;
    }

    info->state = s_state;
    info->mode = s_mode;
    info->index = s_index;
    info->total = s_total;
    info->move = s_cur_move;
    info->running = s_running;
    info->duration_ms = s_cur_ms;
    info->elapsed_ms = (s_running != 0u) ? (HAL_GetTick() - s_start_tick) : 0u;
    info->status = s_status;
    info->result = s_result;

    for (i = 0u; i < IR_MOTOR_NUM; i++)
    {
        info->position[i] = s_position[i];
        info->position_valid[i] = s_position_valid[i];
    }
}

const char *InertialRun_StateName(IrState_t state)
{
    static const char *const names[] =
    {
        "IDLE", "ZERO", "WAIT", "RUN", "STOP", "VERIFY"
    };

    if ((uint8_t)state >= (uint8_t)(sizeof(names) / sizeof(names[0])))
    {
        return "IDLE";
    }

    return names[(uint8_t)state];
}

const char *InertialRun_MoveName(uint8_t move)
{
    static const char *const names[] =
    {
        "FWD", "LEFT", "RIGHT", "BACK"
    };

    if (move >= (uint8_t)(sizeof(names) / sizeof(names[0])))
    {
        return "FWD";
    }

    return names[move];
}
