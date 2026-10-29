/* ============================================================
 * 惯导跑图状态机(全工程唯一的"跑"逻辑)
 *
 * 【预设路径实测数据】50 RPM, 每段开始前四轮 S_CPOS 清零
 *
 *   段 | 动作 | 时长  | M1(右前) | M2(右后) | M3(左前) | M4(左后)
 *   ---+------+-------+----------+----------+----------+----------
 *    1 | 直行 | 3.3 s | -163366  | -163376  |  163364  |  163316
 *    3 | 直行 | 4.8 s | -245282  | -245226  |  245424  |  245312
 *    5 | 直行 | 4.9 s | -250764  | -250806  |  250700  |  250712
 *
 *   直行段符号(三段完全一致): M1/M2 为负、M3/M4 为正 —— 直接反映底盘的
 *   方向映射, 不要随意改。同一段内四轮脉冲极差都在 200 以内,
 *   速度约 49500~51200 脉冲/秒。
 *
 *   ★ 原来第 2/4 段是"左移", 那组数据保留在下方注释里以便回溯; 现在
 *     路径已改成"直行 + 原地转 90 度", 不再跑横移段。
 *       2 | 左移 | 3.8 s | -190748 |  190824 | -190716 |  190860
 *       4 | 左移 | 4.4 s | -223452 |  223526 | -223504 |  223569
 *
 *   ★ 这组数据是在 "1=右前 2=右后 3=左前 4=左后、DIR_INVERT=0,0,1,1"
 *     那套映射下采的。若改动 function.c 里的电机映射, 本表必须重采。
 *
 * 【为什么放弃横移】
 *   麦轮横移时辊子在地面上是侧向滚动, 侧向抓地力只有纵向的约七成,
 *   而四轮摩擦不均产生的偏航力矩并不小。同样 30RPM 的自转修正,
 *   直行时压得住, 横移时压不住, 表现就是"左移段怎么调都修不回来"。
 *   改成"直行 + 陀螺仪闭环转向"后, 每一段都是纵向受力, 闭环稳定。
 * ============================================================ */

#include "main.h"
#include "jy901s.h"
#include "function.h"
#include "InertialRun.h"
#include <string.h>

/* ==================== 可调参数 ==================== */

/* 平移速度(RPM), 只用于直行/横移/后退。和上面采数据时一致,
 * 改它必须重采直行时长表(k_auto_path 里的 duration_ms)。 */
#define IR_SPEED_RPM            50

/* 直行时的航向修正量限幅(RPM)。
 * 50RPM 直行配 ±30RPM 修正, 轮速落在 20~80 之间,
 * 既有纠偏能力又不会甩尾。转向段不走这个限幅(见下)。 */
#define IR_TURN_MAX             30.0f

/* ---- 原地转向参数 ----
 *
 * 转向靠陀螺仪闭环完成: 目标角 = 开始转那一刻的航向 ± 90 度, 每 20ms
 * 读一次剩余角度, 用纯自转命令 Car_Mecanum(0, 0, omega) 转过去。
 * ★ 转向不经过航向环的 g_yaw_output, 也不受 IR_TURN_MAX(30) 限幅,
 *   速度由 ir_apply_turn() 按剩余角度直接分档给出。
 *
 * IR_TURN_RPM     转向时的最高自转速度(RPM)。麦轮原地转要克服辊子的
 *                 侧向摩擦, 比直行吃力, 所以给得比平移稍低但起步要够;
 *                 太大容易冲过目标再往回摆。
 * IR_TURN_TOL_DEG 到位容差(度)。剩余角度进这个带就认为转到位, 立即停车。
 *                 ★ 设太小(比如 0.3)会因为陀螺噪声在边界反复进出, 车身
 *                   抖一下才停; 设太大则每段都留几度残余。
 *                 注意这是"每段独立 ±90 度"的判据, 误差不会累积。
 * IR_TURN_TIMEOUT 保护超时(ms)。转 90 度理论只要 1~2 秒, 给足 5 秒;
 *                 超时说明被卡住或陀螺仪不跟, 直接报错停机,
 *                 绝不能让车一直原地打转。 */
#define IR_TURN_RPM             40
#define IR_TURN_TOL_DEG         1.0f
#define IR_TURN_TIMEOUT_MS      5000u

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

/* 路径表的一项。move 可以是 IR_MOVE_FWD 或 IR_MOVE_TURN_L/R。
 * duration_ms 只在直行段有效; 转向段的时长由陀螺仪闭环决定,
 * 这一项填 0, 运行时会被替换成 IR_TURN_TIMEOUT_MS 当超时上限。 */
typedef struct
{
    uint8_t  move;          /* InertialMove_t */
    uint16_t duration_ms;   /* 直行: 运行时长; 转向: 填 0(用超时) */
} IrPathSeg_t;

typedef enum
{
    IR_VERIFY_WAIT = 0,     /* 读第一次位置作基准 */
    IR_VERIFY_CHECK         /* 再读一次做比较 */
} IrVerifyPhase_t;

/* ==================== 预设路径 ==================== */

/* 自动跑图的 5 段全部是"直行", 段与段之间插入 90 度原地转向。
 *
 * 转向动作写成路径里的一项(duration_ms 不使用), 由状态机识别后
 * 走 IR_TURNING 分支 —— 这样路径表一眼能看出完整轨迹:
 *
 *   段1 直行 -> 左转90 -> 段2 直行 -> 右转90 -> 段3 直行
 *            -> 左转90 -> 段4 直行 -> 右转90 -> 段5 直行 -> 右转90
 *
 * ★ 时长是 50RPM 直行实测值, 改 IR_SPEED_RPM 必须重测。
 *   转向不需要填时长, 它靠陀螺仪闭环转到角度为止。 */
static const IrPathSeg_t k_auto_path[] =
{
    { IR_MOVE_FWD,      3300u },
    { IR_MOVE_TURN_L,   0u    },     /* 段1 之后: 左转 90 */
    { IR_MOVE_FWD,      3800u },
    { IR_MOVE_TURN_R,   0u    },     /* 段2 之后: 右转 90 */
    { IR_MOVE_FWD,      4800u },
    { IR_MOVE_TURN_L,   0u    },     /* 段3 之后: 左转 90 */
    { IR_MOVE_FWD,      4400u },
    { IR_MOVE_TURN_R,   0u    },     /* 段4 之后: 右转 90 */
    { IR_MOVE_FWD,      4900u },
    { IR_MOVE_TURN_R,   0u    }      /* 段5 之后: 右转 90 */
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

/* 自动跑图的"基准航向": 第 1 段起跑瞬间的航向。
 * 后面每一段都用它当目标, 而不是用"本段起跑时的当前航向" ——
 * 否则上一段偏掉多少度, 下一段就把那个歪着的方向当成正前方,
 * 误差会逐段累积, 跑完 5 段车头可以偏几十度。 */
static float             s_home_yaw = 0.0f;

/* 转向段的目标角度(绝对航向, 度)。
 * 由 ir_load_segment() 在倒计时阶段算好, IR_TURNING 里闭环收敛到它。 */
static float             s_turn_target = 0.0f;

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

/* 读编码器失败的那台电机(1~4), 0 = 没有。
 * 挂在 IR_ZERO / IR_COUNTDOWN 的状态名上: POS ERR 之后按 PE0 重启,
 * 会先走 ZERO -> 1s 倒计时, 这段时间足够在屏幕上看到是哪一号读不到。 */
static uint8_t           s_pos_fail_motor = 0u;

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

/* 按当前段的方向下发一帧麦轮速度。
 *
 * 直行/横移/后退: 平移速度 + 航向修正叠加。
 * 原地转向: 只发自转分量, 转多少由航向环的误差决定(见 ir_apply_turn)。
 *
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

/**
  * @brief  判断当前段是不是"原地转向"
  */
static uint8_t ir_is_turn_seg(uint8_t move)
{
    return ((move == (uint8_t)IR_MOVE_TURN_L)
            || (move == (uint8_t)IR_MOVE_TURN_R)) ? 1u : 0u;
}

/**
  * @brief  下发一帧纯自转命令
  * @note   转向角速度按剩余误差限幅: 差得多就快转, 快到位就慢转,
  *         避免以 40RPM 冲过目标再掉头来回摆。
  */
static void ir_apply_turn(void)
{
    float remain = g_yaw_target - jy901s_data.yaw_continuous;
    int16_t rpm;

    /* 误差 10 度以上给满转, 5 度时减半, 1.5 度以内用最低转(能压过静摩擦) */
    if (remain > 10.0f)       rpm = IR_TURN_RPM;
    else if (remain < -10.0f) rpm = -IR_TURN_RPM;
    else
    {
        float gain = (float)IR_TURN_RPM / 10.0f;
        float cmd  = remain * gain;

        if ((cmd < 8.0f) && (cmd > 0.0f))   cmd = 8.0f;
        if ((cmd > -8.0f) && (cmd < 0.0f))  cmd = -8.0f;

        rpm = (int16_t)((cmd >= 0.0f) ? (cmd + 0.5f) : (cmd - 0.5f));
    }

    Car_Mecanum(0, 0, rpm);
}

/* 把当前段号对应的动作/时长装进 s_cur_move / s_cur_ms,
 * 并为"转向段"算好目标角度。 */
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

    /* 转向段: 目标角 = 转向开始那一刻的航向 ± 90 度。
     * ★ 必须在真正开始转之前采样, 所以这里用"当前航向"而不是 s_home_yaw;
     *   转向完成后 s_home_yaw 会被同步更新成转完的朝向(见 ir_verify),
     *   下一段直行才不会把车头拧回去。 */
    if (ir_is_turn_seg(s_cur_move) != 0u)
    {
        float delta = (s_cur_move == (uint8_t)IR_MOVE_TURN_L) ? 90.0f : -90.0f;

        s_turn_target = jy901s_data.yaw_continuous + delta;
        s_cur_ms      = IR_TURN_TIMEOUT_MS;   /* 这一段用超时当上限 */
    }
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
            s_pos_fail_motor = (uint8_t)(i + 1u);
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
            /* 读不到位置就没法确认停车。★ 这里不能重发停车命令:
             * Car_Stop() 一次就塞 8 帧, 而现在失败的原因恰恰是队列里
             * 积压着上一拍的速度帧 + 刚才那 8 帧停车帧, 再塞 8 帧只会
             * 让队列更满、下次读位置更容易超时 —— 越重试越堵。
             * 停车帧已经发过 2 轮(幂等), 这里只需等队列排空再读一次。 */
            s_verify_retry++;
            if (s_verify_retry > IR_VERIFY_MAX_RETRY)
            {
                ir_abort("POS ERR");
                return;
            }
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
            s_pos_fail_motor = (uint8_t)(i + 1u);
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
        /* 同上: 阶段二失败也不补发停车帧, 只等队列排空再读。
         * 补发的唯一作用是把队列重新塞满。 */
        s_verify_retry++;
        if (s_verify_retry > IR_VERIFY_MAX_RETRY)
        {
            ir_abort("POS ERR");
            return;
        }
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

    /* 转向段结束后, 基准航向要跟着更新成新的车头朝向。
     * 否则下一段直行时会用转弯前的 s_home_yaw 当目标, 一开跑就
     * 把车头硬拧回原来的方向。 */
    if (ir_is_turn_seg(s_cur_move) != 0u)
    {
        s_home_yaw = jy901s_data.yaw_continuous;
    }

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
    s_pos_fail_motor = 0u;
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

    /* 基准航向等第 1 段真正起跑时再采样(那时车已经摆好、IMU 数据也稳),
     * 这里只清掉上一趟留下的值, 避免误用。 */
    s_home_yaw = 0.0f;

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
    /* 单段调试只允许四个平移方向; 转向动作需要固定 90 度目标,
     * 不在手工调试范围内, 传进来就退回直行。 */
    if ((uint8_t)move >= IR_SINGLE_MOVE_COUNT)
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
        case IR_TURNING:
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

            /* 上一次 POS ERR 是哪一号读不到, 借这段清零/倒计时显示出来,
             * 下次再失败就能直接定位到电机和接线。 */
            if (s_pos_fail_motor != 0u)
            {
                s_status = (s_pos_fail_motor == 1u) ? "ZERO! M1"
                         : (s_pos_fail_motor == 2u) ? "ZERO! M2"
                         : (s_pos_fail_motor == 3u) ? "ZERO! M3"
                         : "ZERO! M4";
            }
            else
            {
                s_status = "READY";
            }
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

            /* 目标航向的基准:
             *   - 第 1 段(countdown 时 s_index == 0 且当前段是直行)采一次基;
             *   - 之后的直行段沿用 s_home_yaw, 它会在转向段结束后被更新成
             *     新的车头朝向(见 ir_verify), 所以每段直行都是"朝着当前
             *     车头方向走", 而不会把车头硬拧回起点朝向。
             *
             * 单段调试(s_index 恒为 0)行为不变: 每次起跑都重新定基准。 */
            if ((s_index == 0u) && (ir_is_turn_seg(s_cur_move) == 0u))
            {
                s_home_yaw = jy901s_data.yaw_continuous;
            }

            /* ---- 转向段: 完全不同的下发方式 ---- */
            if (ir_is_turn_seg(s_cur_move) != 0u)
            {
                /* 目标角就是本段的绝对航向, 直接闭环过去。
                 * 这里不能用 s_home_yaw: 转向的目的就是离开当前朝向。 */
                YawLoop_Enable(s_turn_target, IR_TURN_MAX);

                ir_apply_turn();

                s_running = 1u;
                s_start_tick = now;
                s_next_steer_tick = now + IR_STEER_MS;
                s_status = "TURN";
                s_state = IR_TURNING;
                break;
            }

            YawLoop_Enable(s_home_yaw, IR_TURN_MAX);

            ir_apply_move();

            s_running = 1u;
            s_start_tick = now;
            s_next_steer_tick = now + IR_STEER_MS;
            s_status = "RUN";
            s_state = IR_MOVING;
            break;

        /* ---- 直行/横移中 ---- */
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

        /* ---- 原地转向中 ---- */
        case IR_TURNING:
            if (s_running == 0u)
            {
                break;
            }

            if (YawLoop_IsImuAlive() == 0)
            {
                ir_begin_stop("IMU LOST");
                break;
            }

            /* 到位判据: 剩余角度进容差带就算转完。
             * 只判一个方向(转过目标也算到位), 否则会绕着目标来回摆。 */
            if ((s_turn_target - jy901s_data.yaw_continuous) <= IR_TURN_TOL_DEG
                && (s_turn_target - jy901s_data.yaw_continuous) >= -IR_TURN_TOL_DEG)
            {
                ir_begin_stop("TURNED");
                break;
            }

            /* 超时保护: 被卡住或陀螺仪不跟时不能一直原地转 */
            if ((uint32_t)(now - s_start_tick) >= (uint32_t)s_cur_ms)
            {
                ir_begin_stop("TURN TO");
                break;
            }

            if ((int32_t)(now - s_next_steer_tick) >= 0)
            {
                s_next_steer_tick = now + IR_STEER_MS;
                ir_apply_turn();
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
    info->turn_remain_deg = (s_state == IR_TURNING)
                          ? (s_turn_target - jy901s_data.yaw_continuous)
                          : 0.0f;
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
        "IDLE", "ZERO", "WAIT", "RUN", "TURN", "STOP", "VERIFY"
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
        "FWD", "LEFT", "RIGHT", "BACK", "TURN_L", "TURN_R"
    };

    if (move >= (uint8_t)(sizeof(names) / sizeof(names[0])))
    {
        return "FWD";
    }

    return names[move];
}
