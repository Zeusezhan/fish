#include "main.h"
#include "jy901s.h"
#include "function.h"
#include "InertialRun.h"
#include <stdio.h>
#include <string.h>

/* ==================== 参数 ==================== */

/* 每段的平移速度(RPM)。和 inertial.h 里采数据时用的 50RPM 一致,
 * 改这里就必须重新采一遍 g_inertial_segments 的时长, 否则距离会变。 */
#define INERTIAL_RUN_SPEED_RPM          50

/* 航向修正量限幅(RPM)。
 * 横移时 50RPM 的横移速度配 ±30RPM 的修正量, 轮速在 20~80 之间,
 * 既有足够的纠偏能力, 又不会因为差速太大而甩尾。
 * 原来这里是 20, 偏小 —— 横移的偏航耦合本来就比直行大。 */
#define INERTIAL_RUN_TURN_MAX           30.0f

/* 运行中下发底盘命令的周期(ms)。和 TIM3 的中断周期一致。
 * 每拍都重发同一组速度是故意的: 命令是幂等的, 驱动器漏收一帧
 * 下一拍就补上了, 比"只在变化时发"抗干扰得多。 */
#define INERTIAL_RUN_STEER_MS           20u

/* 起跑前倒计时(ms): 给驱动器使能、机械稳定留时间 */
#define INERTIAL_RUN_LEAD_MS            1000u

/* 停车命令发送失败后的重发间隔(ms) */
#define INERTIAL_RUN_STOP_RETRY_MS      20u

/* 停止序列最多尝试几次(先看发送是否成功) */
#define INERTIAL_RUN_STOP_MAX_RETRY     3u

/* 停稳等待(ms): 发完停止命令后等这么久再读第一次位置 */
#define INERTIAL_RUN_STOP_SETTLE_MS     150u

/* 位置比较间隔(ms): 读两次位置, 中间隔这么久 */
#define INERTIAL_RUN_VERIFY_GAP_MS      150u

/* "轮子还在转"的判据(编码器脉冲差值)。
 * 标定依据: 50RPM 时约 49500 脉冲/秒(见 inertial.h),
 * 150ms 就是约 7400 脉冲。停住的轮子只会有几个脉冲的抖动,
 * 取 40 既能容忍噪声, 又能抓住"还在以 1RPM 以上速度转"的情况。 */
#define INERTIAL_RUN_MOVE_THRESHOLD     40

/* 停车确认最多重试几次(单轮补停) */
#define INERTIAL_RUN_VERIFY_MAX_RETRY   3u

/* 位置清零后等待(ms), 让驱动器把 S_CPOS 真正写到 0 */
#define INERTIAL_RUN_ZERO_WAIT_MS       300u

/* ==================== 内部状态 ==================== */

static InertialRunState_t s_state = INERTIAL_RUN_IDLE;
static InertialVerifyPhase_t s_verify_phase = INERTIAL_VERIFY_WAIT;

static uint8_t  s_segment_index = 0u;
static uint8_t  s_running = 0u;
static uint8_t  s_heading_hold = 0u;
static uint8_t  s_cancel_requested = 0u;
static uint8_t  s_stop_retry = 0u;
static uint8_t  s_verify_retry = 0u;
static uint8_t  s_verify_reliable = 1u;
static uint8_t  s_stuck_mask = 0u;
static uint32_t s_deadline = 0u;
static uint32_t s_start_tick = 0u;
static uint32_t s_next_steer_tick = 0u;
static int32_t  s_position[INERTIAL_MOTOR_NUM] = {0};
static uint8_t  s_position_valid[INERTIAL_MOTOR_NUM] = {0};
static const char *s_status = "READY";
static char s_result[20] = "NONE";
static char s_fault[20] = "";

/* ==================== 小工具 ==================== */

static void inertial_run_set_result(const char *result)
{
    size_t len = strlen(result);

    if (len >= sizeof(s_result))
    {
        len = sizeof(s_result) - 1u;
    }
    memcpy(s_result, result, len);
    s_result[len] = '\0';
}

/* 取 int32 绝对值, 回避 INT32_MIN 取负溢出 */
static uint32_t inertial_run_abs32(int32_t v)
{
    if (v >= 0)
    {
        return (uint32_t)v;
    }
    return (uint32_t)(-(v + 1)) + 1u;
}

/* 取 mask 里最低位的电机下标(0~3), 用于报错, 定义在文件末尾 */
static uint8_t inertial_run_stuck_first(uint8_t mask);

/* 把中断算出的浮点修正量四舍五入并限幅成整数 RPM */
static int16_t inertial_run_turn_from_yaw(void)
{
    float u = g_yaw_output;

    if (u > INERTIAL_RUN_TURN_MAX)
    {
        u = INERTIAL_RUN_TURN_MAX;
    }
    else if (u < -INERTIAL_RUN_TURN_MAX)
    {
        u = -INERTIAL_RUN_TURN_MAX;
    }

    if (u >= 0.0f)
    {
        return (int16_t)(u + 0.5f);
    }
    return (int16_t)(u - 0.5f);
}

/* ==================== 动作 ==================== */

/**
  * @brief  按当前段的方向, 带着航向修正下发一帧麦轮速度
  * @note   四种动作统一走 Car_Mecanum(vx, vy, omega):
  *           直行   Car_Mecanum( +V,  0, turn)
  *           后退   Car_Mecanum( -V,  0, turn)
  *           左移   Car_Mecanum(  0, +V, turn)
  *           右移   Car_Mecanum(  0, -V, turn)
  *
  *         逆运动学保证了 omega 只是在平移基础上叠加一个自转分量,
  *         不改变平移速度, 所以"边横移边纠航向"时距离基本不变 ——
  *         这也是为什么按时间分段在加了航向环之后依然成立。
  */
static void inertial_run_apply_move(InertialMove_t move)
{
    int16_t turn = (s_heading_hold != 0u) ? inertial_run_turn_from_yaw() : 0;
    int16_t spd  = (int16_t)INERTIAL_RUN_SPEED_RPM;

    switch (move)
    {
        case INERTIAL_MOVE_LEFT:
            Car_Mecanum(0, spd, turn);
            break;

        case INERTIAL_MOVE_RIGHT:
            Car_Mecanum(0, (int16_t)(-spd), turn);
            break;

        case INERTIAL_MOVE_BACKWARD:
            Car_Mecanum((int16_t)(-spd), 0, turn);
            break;

        case INERTIAL_MOVE_FORWARD:
        default:
            Car_Mecanum(spd, 0, turn);
            break;
    }
}

/**
  * @brief  结束运动, 进入停车流程
  * @note   顺序很关键:  先关航向环 -> 再发停车命令。
  *         反过来的话, 中断可能在停车命令之后又算出一个新的 omega,
  *         主循环下一拍就会把车重新开动。
  */
static void inertial_run_begin_stop(void)
{
    s_running = 0u;
    s_heading_hold = 0u;
    YawLoop_Disable();

    s_stop_retry = 0u;
    s_verify_retry = 0u;
    s_verify_reliable = 1u;
    s_stuck_mask = 0u;

    s_state = INERTIAL_RUN_STOPPING;
    s_deadline = HAL_GetTick();     /* 立刻执行第一次停止 */
    s_status = "STOPPING";
}

/**
  * @brief  读一遍四个轮子的位置计数
  * @retval 1 = 四个都读到了, 0 = 有读失败的
  */
static uint8_t inertial_run_read_positions(void)
{
    uint8_t i;
    uint8_t all_ok = 1u;

    for (i = 0u; i < INERTIAL_MOTOR_NUM; i++)
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

static void inertial_run_abort(const char *reason)
{
    s_state = INERTIAL_RUN_IDLE;
    s_running = 0u;
    s_heading_hold = 0u;
    s_cancel_requested = 0u;
    YawLoop_Disable();

    s_status = reason;
    inertial_run_set_result(reason);
}

static uint8_t inertial_run_start_segment(void)
{
    if (!Car_SetEnabled(1u))
    {
        s_status = "ENA ERR";
        return 0u;
    }

    /* 陀螺仪有数据才开闭环; 没有就老老实实开环跑, 但要在屏上标出来 */
    s_heading_hold = (jy901s_data.ready != 0u) ? 1u : 0u;

    if (s_heading_hold != 0u)
    {
        /* ★ 这一句就是"每段都做姿态闭环"的入口:
         *   目标航向 = 本段起跑瞬间的当前航向。
         *   所以每段都是独立地"保持朝这个方向走", 上一段的漂移不会被
         *   带进下一段 —— 这也是惯导跑图能一段一段拼起来的前提。 */
        YawLoop_Enable(jy901s_data.yaw_continuous, INERTIAL_RUN_TURN_MAX);
    }
    else
    {
        YawLoop_Disable();
    }

    inertial_run_apply_move(g_inertial_segments[s_segment_index].type);

    s_running         = 1u;
    s_start_tick      = HAL_GetTick();
    s_next_steer_tick = s_start_tick + INERTIAL_RUN_STEER_MS;
    s_status          = (s_heading_hold != 0u) ? "RUN" : "RUN NO PID";

    return 1u;
}

/**
  * @brief  运行中每 20ms 刷新一次底盘命令
  * @note   航向环本身在 TIM3 中断里跑, 这里只负责把最新的
  *         g_yaw_output 取出来合成麦轮速度并下发。
  */
static void inertial_run_steer(uint32_t now)
{
    if (s_running == 0u)
    {
        return;
    }

    if ((int32_t)(now - s_next_steer_tick) < 0)
    {
        return;
    }

    /* ---- 陀螺仪掉线保护 ----
     * 运行中如果 I2C 停了, 继续按最后一拍的修正量走会越走越偏,
     * 而且没有任何反馈能把它拉回来。这时唯一正确的动作是停车。 */
    if ((s_heading_hold != 0u) && (YawLoop_IsImuAlive() == 0))
    {
        inertial_run_begin_stop();
        s_status = "IMU LOST";
        return;
    }

    s_next_steer_tick = now + INERTIAL_RUN_STEER_MS;
    inertial_run_apply_move(g_inertial_segments[s_segment_index].type);
}

/**
  * @brief  停车确认: 用编码器判断四个轮子是不是真的停住了
  * @note   两阶段:
  *           第一阶段: 停稳后读一次位置, 记为基准 A
  *           第二阶段: 隔 150ms 再读一次, 和 A 比较
  *         某个轮子的差值超过阈值 -> 它还在转 -> 单独给它补一帧
  *         "立即停止", 然后重新走一遍确认, 最多 3 次。
  *
  *         ★ 这就是解决"跑完一段后有轮子继续转"的关键:
  *           原来停车只发命令、不看结果, 丢一帧就再也无人补救;
  *           现在停车是一个"发命令 -> 看结果 -> 补发"的闭环。
  */
static void inertial_run_verify(uint32_t now)
{
    uint8_t i;

    if ((int32_t)(now - s_deadline) < 0)
    {
        return;
    }

    /* ---- 阶段一: 记录基准位置 ---- */
    if (s_verify_phase == INERTIAL_VERIFY_WAIT)
    {
        s_verify_reliable = inertial_run_read_positions();

        if (s_verify_reliable == 0u)
        {
            /* 位置读不到就没法确认停车。再补发一次停止命令再试,
             * 连续失败就整体报错停机 —— 带着"不知道轮子停没停"的
             * 状态跑下一段是最危险的。 */
            s_verify_retry++;
            if (s_verify_retry > INERTIAL_RUN_VERIFY_MAX_RETRY)
            {
                inertial_run_abort("POS READ ERR");
                return;
            }
            (void)Car_Stop();
            s_deadline = now + INERTIAL_RUN_STOP_SETTLE_MS;
            s_status = "POS RETRY";
            return;
        }

        s_verify_phase = INERTIAL_VERIFY_CHECK;
        s_deadline = now + INERTIAL_RUN_VERIFY_GAP_MS;
        s_status = "VERIFY";
        return;
    }

    /* ---- 阶段二: 再读一次, 和基准比较 ---- */
    s_stuck_mask = 0u;

    for (i = 0u; i < INERTIAL_MOTOR_NUM; i++)
    {
        int32_t pos_now = 0;
        int32_t delta;

        if (Car_ReadPositionCount((uint8_t)(i + 1u), &pos_now) == 0u)
        {
            s_verify_reliable = 0u;
            continue;
        }

        delta = pos_now - s_position[i];
        if (inertial_run_abs32(delta) > (uint32_t)INERTIAL_RUN_MOVE_THRESHOLD)
        {
            s_stuck_mask |= (uint8_t)(1u << i);
        }

        /* 更新成最新值, 顺便给屏幕显示用 */
        s_position[i] = pos_now;
        s_position_valid[i] = 1u;
    }

    if (s_verify_reliable == 0u)
    {
        s_verify_retry++;
        if (s_verify_retry > INERTIAL_RUN_VERIFY_MAX_RETRY)
        {
            inertial_run_abort("POS READ ERR");
            return;
        }
        (void)Car_Stop();
        s_verify_phase = INERTIAL_VERIFY_WAIT;
        s_deadline = now + INERTIAL_RUN_STOP_SETTLE_MS;
        s_status = "POS RETRY";
        return;
    }

    if (s_stuck_mask != 0u)
    {
        /* 有轮子还在转: 单独给它补一帧"立即停止"(snF=0, 不依赖同步帧) */
        for (i = 0u; i < INERTIAL_MOTOR_NUM; i++)
        {
            if ((s_stuck_mask & (uint8_t)(1u << i)) != 0u)
            {
                (void)Car_StopMotor((uint8_t)(i + 1u));
            }
        }

        s_verify_retry++;
        if (s_verify_retry > INERTIAL_RUN_VERIFY_MAX_RETRY)
        {
            /* 补了三次还在转, 说明不是丢帧而是驱动器本身有问题
             * (比如被卡住、参数被改、供电不足)。整体停机并报出电机号。 */
            (void)snprintf(s_fault, sizeof(s_fault), "M%u RUNAWAY",
                           (unsigned)(inertial_run_stuck_first(s_stuck_mask) + 1u));
            (void)Car_Stop();
            inertial_run_abort(s_fault);
            return;
        }

        s_verify_phase = INERTIAL_VERIFY_WAIT;
        s_deadline = now + INERTIAL_RUN_STOP_SETTLE_MS;
        s_status = "WHEEL RETRY";
        return;
    }

    /* ---- 四个轮子都确认停住了 ---- */
    if (s_cancel_requested != 0u)
    {
        inertial_run_abort("USER STOP");
        return;
    }

    s_verify_phase = INERTIAL_VERIFY_WAIT;
    s_verify_retry = 0u;
    s_status = "STOPPED";

    /* 这是最后一段: 直接收尾, 不要清零位置计数 ——
     * 那组 S_CPOS 数据就是这一段跑出来的位移, 屏幕和调试都要看它。 */
    if ((s_segment_index + 1u) >= INERTIAL_SEGMENT_NUM)
    {
        s_state = INERTIAL_RUN_IDLE;
        s_status = "ALL DONE";
        inertial_run_set_result("DONE");
        return;
    }

    /* 还有下一段: 先清零位置计数, 再进入下一段的倒计时 */
    s_segment_index++;
    s_state = INERTIAL_RUN_ZERO;
    s_deadline = now;
}

/* 取 mask 里最低位的电机下标(0~3), 用于报错 */
static uint8_t inertial_run_stuck_first(uint8_t mask)
{
    uint8_t i;

    for (i = 0u; i < INERTIAL_MOTOR_NUM; i++)
    {
        if ((mask & (uint8_t)(1u << i)) != 0u)
        {
            return i;
        }
    }
    return 0u;
}

/* ==================== 对外接口 ==================== */

void InertialRun_Init(void)
{
    uint8_t i;

    s_state = INERTIAL_RUN_IDLE;
    s_verify_phase = INERTIAL_VERIFY_WAIT;
    s_segment_index = 0u;
    s_running = 0u;
    s_heading_hold = 0u;
    s_cancel_requested = 0u;
    s_stop_retry = 0u;
    s_verify_retry = 0u;
    s_verify_reliable = 1u;
    s_stuck_mask = 0u;
    s_deadline = 0u;
    s_start_tick = 0u;
    s_next_steer_tick = 0u;
    s_status = "READY";
    inertial_run_set_result("NONE");

    for (i = 0u; i < INERTIAL_MOTOR_NUM; i++)
    {
        s_position[i] = 0;
        s_position_valid[i] = 0u;
    }
}

/**
  * @brief  启动惯导跑图
  * @retval 1 = 已启动, 0 = 忙或者轮子还没停稳
  * @note   一开始先进 ZERO 状态清零四轮编码器, 再倒计时起跑。
  *         先清零、再跑, 这样每段的编码器读数都是本段的位移,
  *         不是"从上次清零到现在的累计值"。
  */
uint8_t InertialRun_Start(void)
{
    uint8_t i;

    if (s_state != INERTIAL_RUN_IDLE)
    {
        s_status = "BUSY";
        return 0u;
    }

    s_segment_index = 0u;
    s_cancel_requested = 0u;
    s_stop_retry = 0u;
    s_verify_retry = 0u;
    s_verify_reliable = 1u;
    s_stuck_mask = 0u;
    s_running = 0u;
    s_heading_hold = 0u;
    YawLoop_Disable();

    s_status = "AUTO";
    inertial_run_set_result("RUN");

    for (i = 0u; i < INERTIAL_MOTOR_NUM; i++)
    {
        s_position_valid[i] = 0u;
    }

    /* 先清位置计数, 再起跑 */
    s_state = INERTIAL_RUN_ZERO;
    s_deadline = HAL_GetTick();

    return 1u;
}

/**
  * @brief  请求停止
  * @note   运行中按 PE0 会走到这里。
  *         运动中的话不直接复位, 而是先走完整的"停车 + 编码器确认"流程,
  *         确认停稳之后才真正退出 —— 否则按下停止反而可能留下一个
  *         还在转的轮子(这正是原来的问题)。
  */
void InertialRun_Cancel(void)
{
    if (s_state == INERTIAL_RUN_IDLE)
    {
        return;
    }

    switch (s_state)
    {
        case INERTIAL_RUN_MOVING:
            s_cancel_requested = 1u;
            inertial_run_begin_stop();
            break;

        case INERTIAL_RUN_STOPPING:
        case INERTIAL_RUN_VERIFY:
            s_cancel_requested = 1u;
            break;

        case INERTIAL_RUN_COUNTDOWN:
        case INERTIAL_RUN_ZERO:
        case INERTIAL_RUN_ZERO_WAIT:
        default:
            /* 这几种状态下轮子本来就是停着的, 直接收尾 */
            inertial_run_abort("USER STOP");
            break;
    }
}

/**
  * @brief  状态机主函数, 放在主循环里反复调用
  * @note   每一个分支都不会长时间阻塞:
  *           停车序列约 12ms(Car_Stop 两轮冗余发送),
  *           位置读取每个电机最多 40ms,
  *           其余都是纯状态判断。
  *         所以即使一次调用花掉几十毫秒, 按键和显示也不会卡死。
  */
void InertialRun_Task(void)
{
    uint32_t now = HAL_GetTick();

    switch (s_state)
    {
        case INERTIAL_RUN_COUNTDOWN:
            if ((int32_t)(now - s_deadline) < 0)
            {
                break;
            }

            if (s_cancel_requested != 0u)
            {
                inertial_run_abort("USER STOP");
                break;
            }

            if (inertial_run_start_segment())
            {
                s_state = INERTIAL_RUN_MOVING;
            }
            else
            {
                inertial_run_abort("ENA ERR");
            }
            break;

        case INERTIAL_RUN_MOVING:
            if (s_running == 0u)
            {
                /* 已经被 steer 里的掉线保护转成停车流程了 */
                break;
            }

            if ((uint32_t)(now - s_start_tick)
                >= (uint32_t)g_inertial_segments[s_segment_index].duration_ms)
            {
                inertial_run_begin_stop();
            }
            else
            {
                inertial_run_steer(now);
            }
            break;

        case INERTIAL_RUN_STOPPING:
            if ((int32_t)(now - s_deadline) < 0)
            {
                break;
            }

            if (Car_Stop())
            {
                /* 命令都发出去了, 但驱动器有没有收到还得靠编码器验证 */
                s_state = INERTIAL_RUN_VERIFY;
                s_verify_phase = INERTIAL_VERIFY_WAIT;
                s_deadline = now + INERTIAL_RUN_STOP_SETTLE_MS;
                s_status = "STOPPED";
            }
            else
            {
                s_stop_retry++;
                if (s_stop_retry >= INERTIAL_RUN_STOP_MAX_RETRY)
                {
                    inertial_run_abort("STOP TX ERR");
                }
                else
                {
                    s_deadline = now + INERTIAL_RUN_STOP_RETRY_MS;
                    s_status = "STOP RETRY";
                }
            }
            break;

        case INERTIAL_RUN_VERIFY:
            inertial_run_verify(now);
            break;

        case INERTIAL_RUN_ZERO:
            if ((int32_t)(now - s_deadline) < 0)
            {
                break;
            }

            if (!Car_ResetPositionCounts())
            {
                inertial_run_abort("ZERO TX ERR");
                break;
            }

            s_status = "ZERO...";
            s_state = INERTIAL_RUN_ZERO_WAIT;
            s_deadline = now + INERTIAL_RUN_ZERO_WAIT_MS;
            break;

        case INERTIAL_RUN_ZERO_WAIT:
            if ((int32_t)(now - s_deadline) < 0)
            {
                break;
            }

            if (s_cancel_requested != 0u)
            {
                inertial_run_abort("USER STOP");
                break;
            }

            /* 段号在 VERIFY 确认停稳时就已经加过了; 起跑前的第一次
             * 清零不加, 所以这里直接进倒计时。 */
            s_state = INERTIAL_RUN_COUNTDOWN;
            s_deadline = now + INERTIAL_RUN_LEAD_MS;
            break;

        case INERTIAL_RUN_IDLE:
        default:
            break;
    }
}

void InertialRun_GetInfo(InertialRunInfo_t *info)
{
    uint8_t i;
    const InertialSegment_t *segment;

    if (info == NULL)
    {
        return;
    }

    segment = &g_inertial_segments[s_segment_index];
    info->state          = s_state;
    info->segment_index  = s_segment_index;
    info->running        = s_running;
    info->heading_hold   = s_heading_hold;
    info->stop_retry     = (s_stop_retry > s_verify_retry) ? s_stop_retry : s_verify_retry;
    info->duration_ms    = segment->duration_ms;
    info->start_tick     = s_start_tick;
    info->move           = segment->type;
    info->status         = s_status;
    info->result         = s_result;

    for (i = 0u; i < INERTIAL_MOTOR_NUM; i++)
    {
        info->position[i] = s_position[i];
        info->position_valid[i] = s_position_valid[i];
    }
}

const char *InertialRun_GetStateName(InertialRunState_t state)
{
    static const char *const names[] =
    {
        "IDLE", "COUNT", "RUN", "STOP", "VERIFY", "ZERO", "ZEROWT"
    };

    if ((uint8_t)state >= (sizeof(names) / sizeof(names[0])))
    {
        return "IDLE";
    }

    return names[state];
}
