/* ============================================================
 * OLED 显示菜单
 *
 * 【页面结构】
 *   第一页(选择界面) ──确定──> 自动跑图 / 单段调试
 *   调试界面长按 PE2 返回选择页
 *
 * 【按键】按键编号见 key.h: KEY_1=PE0, KEY_2=PE1, KEY_3=PE2
 *   第一页:  PE1(KEY_2) = 下一个    PE0(KEY_1) = 确定
 *   选择页: PE1(KEY_2) 切换选项, PE0(KEY_1) 确认
 *   调试页: PE2(KEY_3) 长按返回选择页
 *
 * 【绘制方式】
 *   每次重绘流程: OLED_ClearBuffer() -> 按页面写显存 -> OLED_Refresh()
 *   用 OLED_ClearBuffer() 而不是 OLED_Clear(), 可以少一次全屏 I2C 传输。
 * ============================================================ */

#include "main.h"
#include "OLED.h"
#include "jy901s.h"
#include "key.h"
#include "Menu.h"
#include "function.h"
#include "../Function/InertialRun.h"
#include <stdio.h>

/* 航向环状态由 main.h 统一声明(main.c 的 TIM3 中断每 20ms 计算一次),
 * 这里不再重复 extern —— 参数改了也不会两边对不上。 */

/* 调试页航向闭环开关: 1 = 启动时记录目标航向并下发纠偏命令。
 * ★ 只有它和 main.c 里的 HAL_TIM_Base_Start_IT(&htim3) 同时打开,
 *   航向环才会真正算出来并下发; 少了任何一个都会退化成开环运动。 */
#define MENU_DEBUG_HEADING_LOOP_ENABLED   1

#define MENU_DEBUG_SPEED_RPM       50

/* 调试运行时的航向修正转速限幅(RPM), 和 InertialRun 保持一致 */
#define MENU_DEBUG_TURN_MAX        30.0f

/* 航向环输出刷新周期(ms), 与 TIM3 的 20ms 运算周期一致 */
#define MENU_DEBUG_STEER_MS        20u

/* ---- 显示刷新节奏 ----
 * OLED 是软件 I2C, 整屏刷一次要十几毫秒, 跑动中刷太勤会把
 * 主循环的 20ms 下发节拍拖乱。所以运行中降频, 停车时恢复正常。 */
#define MENU_REFRESH_RUN_MS        300u

/* 秒数以 0.1 秒为单位保存, 便于用整数精确计时 */
#define MENU_DEBUG_MIN_TENTH_SECONDS    10u     /* 1.0 秒 */
#define MENU_DEBUG_MAX_TENTH_SECONDS    990u    /* 99.0 秒 */
#define MENU_DEBUG_HOLD_MS         1000u
#define MENU_DEBUG_READ_INTERVAL   500u
#define MENU_DEBUG_ZERO_WAIT_MS    100u

/* 清零判定容差: 读回绝对值小于该值就认为已清零 */
#define MENU_DEBUG_ZERO_TOLERANCE  10

typedef enum
{
    MENU_DEBUG_MOVE_FORWARD = 0,
    MENU_DEBUG_MOVE_LEFT,
    MENU_DEBUG_MOVE_RIGHT,
    MENU_DEBUG_MOVE_BACKWARD,
    MENU_DEBUG_MOVE_COUNT
} MenuDebugMove_t;

/* 当前页面 */
typedef enum
{
    MENU_PAGE_MAIN = 0,
    MENU_PAGE_DEBUG
} MenuPage_t;

/* Debug 页当前显示/运行的功能 */
typedef enum
{
    MENU_DEBUG_VIEW_AUTO = 0,       /* 自动跑完惯导五段 */
    MENU_DEBUG_VIEW_SINGLE          /* 单段调试: 可设方向与时长 */
} MenuDebugView_t;

/* 当前页面 */
static MenuPage_t s_page = MENU_PAGE_MAIN;

/* 第一页的光标位置(当前选中的菜单项) */
static uint8_t s_cursor = MENU_ITEM_AUTO;

/* 显示刷新计时 */
static uint32_t s_last_draw_tick = 0;

/* 需要立即重绘的标志: 切换页面/移动光标后置 1, 让界面立刻响应按键 */
static uint8_t s_need_redraw = 1;

static uint16_t s_debug_tenth_seconds = MENU_DEBUG_MIN_TENTH_SECONDS;
static MenuDebugMove_t s_debug_move = MENU_DEBUG_MOVE_FORWARD;
static MenuDebugView_t s_debug_view = MENU_DEBUG_VIEW_AUTO;
static uint16_t s_debug_node = 1u;
static uint8_t s_debug_running = 0u;
static uint32_t s_debug_start_tick = 0u;
static uint32_t s_debug_next_read_tick = 0u;
static uint8_t s_debug_read_valid[4] = {0u, 0u, 0u, 0u};
static int32_t s_debug_position[4] = {0, 0, 0, 0};
static uint8_t s_debug_key1_pending = 0u;
static uint8_t s_debug_start_pending = 0u;
static uint8_t s_debug_key3_pending = 0u;
static uint8_t s_debug_clear_pending = 0u;
static uint8_t s_debug_clear_tx_ok = 0u;
static uint8_t s_debug_read_error = 0u;
static uint8_t s_debug_heading_hold = 0u;
static uint32_t s_debug_next_steer_tick = 0u;

/* 停车状态锁:
 * 0 = 允许下发运动命令
 * 1 = 正在执行停车序列, 期间任何速度/纠偏命令都必须被丢弃
 * 停车序列本身会占用串口十余毫秒, 这期间若主循环再发一帧带速度的
 * 命令, 就会把刚发出的停止帧覆盖掉, 车就会停不住。
 * 所以停车必须先把锁置上, 停完再释放。 */
static volatile uint8_t s_debug_stop_lock = 0u;
static const char *s_debug_status = "READY";

/* 调试页清零函数稍后定义, 这里先声明 */
static void menu_debug_clear_positions(void);

/* 主选择页的菜单项文字
 * (OLED 6x8 字库只有 ASCII, 不能用中文) */
static const char *const s_main_item[MENU_ITEM_NUM] =
{
    "Auto Run",         /* MENU_ITEM_AUTO */
    "Single Seg",       /* MENU_ITEM_SINGLE */
};

/* ==================== 界面绘制 ==================== */

/**
  * @brief  绘制第一页: 选择界面
  * @note   光标所指的那一项, 前面用 '>' 标记
  */
static void menu_draw_main(void)
{
    uint8_t i;
    uint8_t row;

    OLED_ShowString(0, 0, "======= MENU =======");

    for (i = 0; i < MENU_ITEM_NUM; i++)
    {
        row = (uint8_t)(2 + i * 2);

        OLED_ShowChar(0, row, (i == s_cursor) ? '>' : ' ');
        OLED_ShowString(6, row, s_main_item[i]);
    }

    OLED_ShowString(0, 7, "PE1:Next  PE0:OK");
}

/* 取方向名字 */
static const char *menu_move_name(void)
{
    static const char *const name[MENU_DEBUG_MOVE_COUNT] =
    {
        "FWD", "LEFT", "RIGHT", "BACK"
    };

    if ((uint8_t)s_debug_move >= MENU_DEBUG_MOVE_COUNT)
    {
        return "FWD";
    }
    return name[s_debug_move];
}

/* 取自动跑图方向名字 */
static const char *menu_inertial_move_name(InertialMove_t move)
{
    static const char *const name[] =
    {
        "FWD", "LEFT", "RIGHT", "BACK"
    };

    if ((uint8_t)move >= (sizeof(name) / sizeof(name[0])))
    {
        return "FWD";
    }
    return name[move];
}

/* 绘制 M1~M4 四行读数值 */
static void menu_draw_positions(const int32_t position[INERTIAL_MOTOR_NUM],
                                const uint8_t valid[INERTIAL_MOTOR_NUM])
{
    char line[24];
    uint8_t i;

    for (i = 0; i < INERTIAL_MOTOR_NUM; i++)
    {
        if (valid[i])
        {
            (void)snprintf(line, sizeof(line), "M%u:%ld", (unsigned)(i + 1u),
                           (long)position[i]);
        }
        else
        {
            (void)snprintf(line, sizeof(line), "M%u:--", (unsigned)(i + 1u));
        }
        OLED_ShowString(0, (uint8_t)(2u + i), line);
    }
}

/* 第 1 行: 运行时显示剩余时间, 否则显示自定义文本 */
static void menu_draw_timer_line(const char *idle_text)
{
    char line[32];

    if (s_debug_running)
    {
        uint32_t remain = (uint32_t)s_debug_tenth_seconds * 100u
                        - (HAL_GetTick() - s_debug_start_tick);

        (void)snprintf(line, sizeof(line), "T:%u.%uS V:%uRPM",
                       (unsigned)((remain + 99u) / 1000u),
                       (unsigned)(((remain + 99u) % 1000u) / 100u),
                       (unsigned)MENU_DEBUG_SPEED_RPM);
    }
    else
    {
        (void)snprintf(line, sizeof(line), "%s", idle_text);
    }
    OLED_ShowString(0, 1, line);
}

/* 绘制自动跑图页 */
static void menu_draw_debug_auto(void)
{
    char line[24];
    InertialRunInfo_t run;
    uint8_t active;

    InertialRun_GetInfo(&run);
    active = (run.state != INERTIAL_RUN_IDLE);

    if (active)
    {
        (void)snprintf(line, sizeof(line), "AUTO %u/%u %s",
                       (unsigned)(run.segment_index + 1u),
                       (unsigned)INERTIAL_SEGMENT_NUM,
                       menu_inertial_move_name(run.move));
        OLED_ShowString(0, 0, line);

        if (run.running)
        {
            uint32_t elapsed = HAL_GetTick() - run.start_tick;
            uint32_t remain = (elapsed < run.duration_ms)
                            ? (run.duration_ms - elapsed) : 0u;
            (void)snprintf(line, sizeof(line), "T:%u.%uS V:%uRPM",
                           (unsigned)((remain + 99u) / 1000u),
                           (unsigned)(((remain + 99u) % 1000u) / 100u),
                           (unsigned)INERTIAL_TEST_SPEED_RPM);
        }
        else
        {
            (void)snprintf(line, sizeof(line), "SEG %u/%u WAIT",
                           (unsigned)(run.segment_index + 1u),
                           (unsigned)INERTIAL_SEGMENT_NUM);
        }
        OLED_ShowString(0, 1, line);
    }
    else
    {
        OLED_ShowString(0, 0, "-- Auto Run --");
        OLED_ShowString(0, 1, "HOLD PE0: RERUN");
    }

    menu_draw_positions(run.position, run.position_valid);

    /* 第 6 行: 运行 > 阶段 > 结果 */
    if (run.running)
    {
        (void)snprintf(line, sizeof(line), "PE0:STOP %s",
                       run.heading_hold ? "PID" : "OPEN");
        OLED_ShowString(0, 6, line);

        /* 第 7 行: 运行中显示实时航向误差, 调 Kp/Ki 时看这个最直观 */
        if (run.heading_hold)
        {
            (void)snprintf(line, sizeof(line), "Err:%6.1f", (double)YawLoop_GetError());
        }
        else
        {
            (void)snprintf(line, sizeof(line), "NO IMU DATA");
        }
        OLED_ShowString(0, 7, line);
    }
    else if (active)
    {
        (void)snprintf(line, sizeof(line), "ST:%s",
                       InertialRun_GetStateName(run.state));
        OLED_ShowString(0, 6, line);

        /* 第 7 行: 显示本段序号与进度; 停车重试时把次数也打出来 */
        if (run.stop_retry > 0u)
        {
            (void)snprintf(line, sizeof(line), "SEG%u RT:%u",
                           (unsigned)(run.segment_index + 1u),
                           (unsigned)run.stop_retry);
        }
        else
        {
            (void)snprintf(line, sizeof(line), "SEG %u/%u %s",
                           (unsigned)(run.segment_index + 1u),
                           (unsigned)INERTIAL_SEGMENT_NUM,
                           menu_inertial_move_name(run.move));
        }
        OLED_ShowString(0, 7, line);
    }
    else
    {
        (void)snprintf(line, sizeof(line), "RESULT:%s", run.result);
        OLED_ShowString(0, 6, line);
        OLED_ShowString(0, 7, "PE2H:BACK PE0H:GO");
    }
}

/* 绘制单段调试页 */
static void menu_draw_debug_single(void)
{
    char line[32];

    (void)snprintf(line, sizeof(line), "N%03u %-5s %s",
                   (unsigned)s_debug_node, menu_move_name(), s_debug_status);
    OLED_ShowString(0, 0, line);

    (void)snprintf(line, sizeof(line), "T:%u.%uS PE1:+ PE2:-",
                   (unsigned)(s_debug_tenth_seconds / 10u),
                   (unsigned)(s_debug_tenth_seconds % 10u));
    menu_draw_timer_line(line);

    menu_draw_positions(s_debug_position, s_debug_read_valid);

    if (s_debug_running)
    {
        OLED_ShowString(0, 6, "PE0:STOP");
        if (s_debug_heading_hold)
        {
            (void)snprintf(line, sizeof(line), "YawErr:%5.1f",
                           (double)YawLoop_GetError());
            OLED_ShowString(0, 7, line);
        }
        else
        {
            OLED_ShowString(0, 7, "NO PID: raw cmd");
        }
    }
    else
    {
        OLED_ShowString(0, 6, "PE0:DIR HOLD:GO");
        OLED_ShowString(0, 7, "PE1H:ZERO PE2H:BACK");
    }
}

/* 绘制当前调试功能界面 */
static void menu_draw_debug(void)
{
    if (s_debug_view == MENU_DEBUG_VIEW_SINGLE)
    {
        menu_draw_debug_single();
    }
    else
    {
        menu_draw_debug_auto();
    }
}

static void menu_debug_read_positions(uint32_t now)
{
    uint8_t i;
    uint8_t all_zero = 1u;
    uint8_t all_valid = 1u;

    for (i = 0; i < 4u; i++)
    {
        s_debug_read_valid[i] = Car_ReadPositionCount((uint8_t)(i + 1u),
                                                       &s_debug_position[i]);
        if (!s_debug_read_valid[i])
        {
            all_valid = 0u;
        }
        else if ((s_debug_position[i] > MENU_DEBUG_ZERO_TOLERANCE)
                 || (s_debug_position[i] < -MENU_DEBUG_ZERO_TOLERANCE))
        {
            /* 超出容差才算没清零, 小残余量视为已完成 */
            all_zero = 0u;
        }
    }

    if (s_debug_clear_pending)
    {
        s_debug_clear_pending = 0u;
        if (s_debug_clear_tx_ok && all_valid && all_zero)
        {
            if (s_debug_node < 999u)
            {
                s_debug_node++;
            }
            s_debug_status = "ZERO OK";
            s_debug_read_error = 0u;
        }
        else
        {
            s_debug_status = "ZERO ERR";
            s_debug_read_error = (uint8_t)!all_valid;
        }
    }
    else if (!all_valid)
    {
        s_debug_status = "READ ERR";
        s_debug_read_error = 1u;
    }
    else if (s_debug_read_error)
    {
        s_debug_status = s_debug_running ? "RUN" : "STOP";
        s_debug_read_error = 0u;
    }

    s_debug_next_read_tick = now + MENU_DEBUG_READ_INTERVAL;

    /* 停车后的第一次读数完成, 说明停车序列已经彻底结束, 释放状态锁 */
    s_debug_stop_lock = 0u;

    s_need_redraw = 1u;
}

/**
  * @brief  按当前选择的方向发一帧带航向修正的底盘命令
  * @note   航向环使能时, g_yaw_output 始终把车头拉回启动时记录的朝向:
  *           前进/后退 -> Car_Drive(基准速度, 修正量)
  *           左移/右移 -> Car_Mecanum(0, 横移速度, 修正量)
  *         未使能时退化成不带修正的原始命令。
  */
static void menu_debug_apply_move(void)
{
    int16_t turn = 0;

    /* 兜底: 停车锁存在时, 任何调用路径都不允许发速度命令 */
    if (s_debug_stop_lock)
    {
        return;
    }

    if (s_debug_heading_hold)
    {
        /* g_yaw_output 是带小数、带符号的修正转速, 四舍五入后再限幅 */
        if (g_yaw_output > MENU_DEBUG_TURN_MAX)
        {
            turn = (int16_t)MENU_DEBUG_TURN_MAX;
        }
        else if (g_yaw_output < -MENU_DEBUG_TURN_MAX)
        {
            turn = -(int16_t)MENU_DEBUG_TURN_MAX;
        }
        else if (g_yaw_output >= 0.0f)
        {
            turn = (int16_t)(g_yaw_output + 0.5f);
        }
        else
        {
            turn = (int16_t)(g_yaw_output - 0.5f);
        }
    }

    switch (s_debug_move)
    {
        case MENU_DEBUG_MOVE_LEFT:
            if (s_debug_heading_hold)
            {
                Car_Mecanum(0, (int16_t)MENU_DEBUG_SPEED_RPM, turn);
            }
            else
            {
                Car_MoveLeft(MENU_DEBUG_SPEED_RPM);
            }
            break;

        case MENU_DEBUG_MOVE_RIGHT:
            if (s_debug_heading_hold)
            {
                Car_Mecanum(0, -(int16_t)MENU_DEBUG_SPEED_RPM, turn);
            }
            else
            {
                Car_MoveRight(MENU_DEBUG_SPEED_RPM);
            }
            break;

        case MENU_DEBUG_MOVE_BACKWARD:
            if (s_debug_heading_hold)
            {
                Car_Drive(-(int16_t)MENU_DEBUG_SPEED_RPM, turn);
            }
            else
            {
                Car_MoveBackward(MENU_DEBUG_SPEED_RPM);
            }
            break;

        case MENU_DEBUG_MOVE_FORWARD:
        default:
            if (s_debug_heading_hold)
            {
                Car_Drive((int16_t)MENU_DEBUG_SPEED_RPM, turn);
            }
            else
            {
                Car_MoveForward(MENU_DEBUG_SPEED_RPM);
            }
            break;
    }
}

static void menu_debug_start(void)
{
    uint8_t i;

    /* 停车序列没走完之前不允许再次启动 */
    if (s_debug_stop_lock)
    {
        return;
    }

    if (!Car_SetEnabled(1u))
    {
        s_debug_status = "ENA ERR";
        s_need_redraw = 1u;
        return;
    }

    s_debug_heading_hold = (MENU_DEBUG_HEADING_LOOP_ENABLED
                            && jy901s_data.ready) ? 1u : 0u;
    if (s_debug_heading_hold)
    {
        /* 目标航向 = 起跑瞬间的当前航向; 积分器在使能时自动清零 */
        YawLoop_Enable(jy901s_data.yaw_continuous, MENU_DEBUG_TURN_MAX);
    }
    else
    {
        YawLoop_Disable();
    }
    menu_debug_apply_move();
    s_debug_running = 1u;
    s_debug_start_tick = HAL_GetTick();
    s_debug_next_steer_tick = s_debug_start_tick + MENU_DEBUG_STEER_MS;
    s_debug_status = s_debug_heading_hold ? "RUN" : "RUN NO PID";
    for (i = 0; i < 4u; i++)
    {
        s_debug_read_valid[i] = 0u;
    }
    s_need_redraw = 1u;
}

static void menu_debug_stop(void)
{
    /* 1. 先上锁: 从这一刻起, 任何运动/纠偏命令都不会再发出去 */
    s_debug_stop_lock = 1u;

    /* 2. 立刻关掉航向环, 中断不会再算出新的修正量 */
    YawLoop_Disable();
    s_debug_heading_hold = 0u;
    s_debug_running = 0u;

    /* 3. 发停车序列(缓存停止 + 广播同步 + 逐台立即停止, 两轮冗余) */
    s_debug_status = Car_Stop() ? "STOP" : "STOP TX ERR";

    /* 4. 停稳后读取位置计数, 读数期间锁保持, 避免读数与运动命令抢串口 */
    s_debug_next_read_tick = HAL_GetTick() + MENU_DEBUG_ZERO_WAIT_MS;
    s_need_redraw = 1u;
}

/* 运行中按 MENU_DEBUG_STEER_MS 刷新底盘命令, 让航向修正真正下发。
 * 不能每次主循环都发: Car_Mecanum 逐帧等待发送完成, 一次 4~8ms,
 * 每圈都发会把主循环和菜单刷新拖慢。
 *
 * ★ 控制流是"主循环按 20ms 定时下发 + 中断只算不发", 所以即使
 *   中断某一拍没算出新值, 车也会按最后一拍的修正量继续走,
 *   不会因为一次中断丢拍就停在那里等。 */
static void menu_debug_steer(uint32_t now)
{
    /* 停车锁优先: 停车期间绝不能再下发任何速度命令 */
    if (s_debug_stop_lock || !s_debug_running)
    {
        return;
    }

    if ((int32_t)(now - s_debug_next_steer_tick) < 0)
    {
        return;
    }

    /* ---- IMU 掉线保护 ----
     * 跑动中如果陀螺仪读数停了, 继续用最后一拍修正量会越修越偏,
     * 这里直接停车, 避免车带着错误航向乱跑。
     * 判据统一交给 YawLoop_IsImuAlive(), 和 InertialRun 用的是同一套门限,
     * 不会出现"两个模块对掉线的定义不一样"的问题。 */
    if ((s_debug_heading_hold != 0u) && (YawLoop_IsImuAlive() == 0))
    {
        menu_debug_stop();
        s_debug_status = "IMU LOST";
        s_need_redraw = 1u;
        return;
    }

    s_debug_next_steer_tick = now + MENU_DEBUG_STEER_MS;
    menu_debug_apply_move();
}

static void menu_debug_clear_positions(void)
{
    if (s_debug_running || s_debug_clear_pending || s_debug_stop_lock)
    {
        return;
    }

    s_debug_clear_tx_ok = Car_ResetPositionCounts();
    if (!s_debug_clear_tx_ok)
    {
        s_debug_status = "ZERO TX ERR";
        s_need_redraw = 1u;
        return;
    }

    s_debug_clear_pending = 1u;
    s_debug_status = "ZERO...";
    s_debug_next_read_tick = HAL_GetTick() + MENU_DEBUG_ZERO_WAIT_MS;
    s_need_redraw = 1u;
}

/* 判断某键是否已达到长按阈值 */
static uint8_t menu_is_held(uint8_t key)
{
    return (Key_IsDown(key) && (Key_GetHoldMs(key) >= MENU_DEBUG_HOLD_MS))
           ? 1u : 0u;
}

/* 按步长调整秒数(1 = +0.1s, 0 = -0.1s), 带上下限 */
static void menu_adjust_time(uint8_t increase)
{
    if (s_debug_running)
    {
        return;
    }

    if (increase)
    {
        if (s_debug_tenth_seconds < MENU_DEBUG_MAX_TENTH_SECONDS)
        {
            s_debug_tenth_seconds++;
            s_need_redraw = 1u;
        }
    }
    else if (s_debug_tenth_seconds > MENU_DEBUG_MIN_TENTH_SECONDS)
    {
        s_debug_tenth_seconds--;
        s_need_redraw = 1u;
    }
}

static void menu_handle_debug_keys(void)
{
    uint8_t key;
    uint8_t release;

    key = Key_GetPress();

    if (key == KEY_1)
    {
        if (s_debug_running)
        {
            menu_debug_stop();
        }
        else
        {
            s_debug_start_pending = 1u;
        }
    }
    else if (key == KEY_2)
    {
        s_debug_key1_pending = 1u;
    }
    else if (key == KEY_3)
    {
        s_debug_key3_pending = 1u;
    }

    /* ---- 长按动作 ---- */
    if (s_debug_start_pending && menu_is_held(KEY_1))
    {
        s_debug_start_pending = 0u;
        menu_debug_start();
    }

    if (s_debug_key1_pending && menu_is_held(KEY_2))
    {
        s_debug_key1_pending = 0u;
        menu_debug_clear_positions();
    }

    if (s_debug_key3_pending && menu_is_held(KEY_3))
    {
        s_debug_key3_pending = 0u;
        if (!s_debug_running)
        {
            s_page = MENU_PAGE_MAIN;
            s_need_redraw = 1u;
        }
    }

    /* ---- 短按动作 ---- */
    release = Key_GetRelease();
    if ((release == KEY_1) && s_debug_start_pending)
    {
        s_debug_start_pending = 0u;
        if (!s_debug_running && (s_debug_view == MENU_DEBUG_VIEW_SINGLE))
        {
            s_debug_move = (MenuDebugMove_t)((s_debug_move + 1u)
                                           % MENU_DEBUG_MOVE_COUNT);
            s_need_redraw = 1u;
        }
    }
    else if ((release == KEY_2) && s_debug_key1_pending)
    {
        s_debug_key1_pending = 0u;
        menu_adjust_time(1u);
    }
    else if ((release == KEY_3) && s_debug_key3_pending)
    {
        s_debug_key3_pending = 0u;
        menu_adjust_time(0u);
    }
}

static void menu_handle_auto_keys(void)
{
    InertialRunInfo_t run;
    uint8_t key = Key_GetPress();
    uint8_t release;

    InertialRun_GetInfo(&run);
    if (key == KEY_1)
    {
        if (run.state != INERTIAL_RUN_IDLE)
        {
            InertialRun_Cancel();
            s_debug_start_pending = 0u;
            s_need_redraw = 1u;
        }
        else
        {
            s_debug_start_pending = 1u;
        }
    }
    else if (key == KEY_3)
    {
        s_debug_key3_pending = 1u;
    }

    if (s_debug_start_pending && menu_is_held(KEY_1))
    {
        s_debug_start_pending = 0u;
        (void)InertialRun_Start();
        s_need_redraw = 1u;
    }

    InertialRun_GetInfo(&run);
    if (s_debug_key3_pending && menu_is_held(KEY_3)
        && (run.state == INERTIAL_RUN_IDLE))
    {
        s_debug_key3_pending = 0u;
        s_page = MENU_PAGE_MAIN;
        s_need_redraw = 1u;
    }

    release = Key_GetRelease();
    if ((release == KEY_1) && s_debug_start_pending)
    {
        s_debug_start_pending = 0u;
    }
    else if ((release == KEY_3) && s_debug_key3_pending)
    {
        s_debug_key3_pending = 0u;
    }
}

/* ==================== 按键处理 ==================== */

/**
  * @brief  处理一次按键事件, 决定页面怎么跳
  * @param  key  Key_GetPress() 的返回值(KEY_1/KEY_2/KEY_3/KEY_NONE)
  * @note   只有状态真的变化时才置 s_need_redraw, 避免无谓重绘。
  */
static void menu_handle_key(uint8_t key)
{
    if (key == KEY_NONE)
    {
        return;
    }

    if (s_page == MENU_PAGE_MAIN)
    {
        /* ---------- 第一页: 选择界面 ---------- */
        if (key == KEY_2)
        {
            /* PE1: 选择下一个, 到底了绕回第一个 */
            s_cursor++;
            if (s_cursor >= MENU_ITEM_NUM)
            {
                s_cursor = 0;
            }
            s_need_redraw = 1;
        }
        else if (key == KEY_1)
        {
            /* PE0: 直接进入所选功能; 自动跑图状态机由独立模块执行。 */
            s_page = MENU_PAGE_DEBUG;
            s_debug_running = 0u;
            s_debug_clear_pending = 0u;
            s_debug_status = "READY";
            s_debug_next_read_tick = HAL_GetTick();
            s_debug_key1_pending = 0u;
            s_debug_start_pending = 0u;
            s_debug_key3_pending = 0u;
            s_debug_read_valid[0] = 0u;
            s_debug_read_valid[1] = 0u;
            s_debug_read_valid[2] = 0u;
            s_debug_read_valid[3] = 0u;

            if (s_cursor == MENU_ITEM_AUTO)
            {
                s_debug_view = MENU_DEBUG_VIEW_AUTO;
                InertialRun_Init();
                (void)InertialRun_Start();
            }
            else
            {
                s_debug_view = MENU_DEBUG_VIEW_SINGLE;
                s_debug_move = MENU_DEBUG_MOVE_FORWARD;
                s_debug_tenth_seconds = MENU_DEBUG_MIN_TENTH_SECONDS;
            }
            s_need_redraw = 1;
        }
        /* 按 PE2(KEY_3) 不做任何事: 已经在最顶层界面了 */
    }
    else
    {
        /* ---------- 调试功能页: PE2 长按返回选择页 ---------- */
        if (key == KEY_3)
        {
            s_page = MENU_PAGE_MAIN;
            s_need_redraw = 1;
        }
    }
}

/* ==================== 对外接口 ==================== */

/**
  * @brief  初始化菜单
  * @note   上电调用一次, 让界面从第一页开始。
  */
void Menu_Init(void)
{
    s_page = MENU_PAGE_MAIN;
    s_cursor = MENU_ITEM_AUTO;
    s_need_redraw = 1;
    s_last_draw_tick = 0;
    s_debug_view = MENU_DEBUG_VIEW_AUTO;
    s_debug_running = 0u;
    s_debug_clear_pending = 0u;
    s_debug_tenth_seconds = MENU_DEBUG_MIN_TENTH_SECONDS;
    s_debug_move = MENU_DEBUG_MOVE_FORWARD;
    s_debug_node = 1u;
    s_debug_read_error = 0u;
    s_debug_start_pending = 0u;
    s_debug_heading_hold = 0u;
    s_debug_next_steer_tick = 0u;
    s_debug_stop_lock = 0u;
    InertialRun_Init();
}

/**
  * @brief  菜单主函数
  * @note   放到 while(1) 里反复调用(前提是已经调用过 Key_Scan())。
  *
  *         按键是"按一次触发一次", 所以这里的按键处理每次都执行;
  *         绘制则按 MENU_REFRESH_MS 限频, 位置与状态周期刷新。
  *         按键导致页面/光标变化时置 s_need_redraw, 界面立刻更新。
  */
void Menu_Run(void)
{
    uint32_t now;
    uint32_t refresh_ms = MENU_REFRESH_MS;
    uint8_t  running_now = 0u;
    InertialRunInfo_t run_for_draw;

    /* ---- 1. 处理按键 ---- */
    if (s_page == MENU_PAGE_DEBUG)
    {
        if (s_debug_view == MENU_DEBUG_VIEW_AUTO)
        {
            menu_handle_auto_keys();
        }
        else
        {
            menu_handle_debug_keys();
        }
    }
    else
    {
        menu_handle_key(Key_GetPress());
    }

    /* ---- 2. 调试页运行计时与停稳后的编码器读取 ---- */
    now = HAL_GetTick();
    if (s_page == MENU_PAGE_DEBUG)
    {
        if (s_debug_view == MENU_DEBUG_VIEW_AUTO)
        {
            InertialRun_Task();
        }
        else if (s_debug_running
                 && ((now - s_debug_start_tick)
                     >= (uint32_t)s_debug_tenth_seconds * 100u))
        {
            menu_debug_stop();
        }
        else if (s_debug_running)
        {
            menu_debug_steer(now);
        }
        else if ((int32_t)(now - s_debug_next_read_tick) >= 0)
        {
            menu_debug_read_positions(now);
        }
    }

    /* ---- 3. 绘制(限频) ----
     * 车在动的时候把刷新周期放慢: OLED 是软件 I2C, 整屏刷一次要
     * 十几毫秒, 这段时间主循环完全停摆, 20ms 的底盘下发节拍会被拉长。
     * 跑图时少刷几次屏幕, 换来的是航向修正更及时 —— 这笔交换很划算。 */
    now = HAL_GetTick();

    if (s_page == MENU_PAGE_DEBUG)
    {
        if (s_debug_view == MENU_DEBUG_VIEW_AUTO)
        {
            InertialRun_GetInfo(&run_for_draw);
            running_now = run_for_draw.running;
        }
        else
        {
            running_now = s_debug_running;
        }
    }

    refresh_ms = (running_now != 0u) ? MENU_REFRESH_RUN_MS : MENU_REFRESH_MS;

    if (!s_need_redraw && ((now - s_last_draw_tick) < refresh_ms))
    {
        return;
    }
    s_last_draw_tick = now;
    s_need_redraw = 0;

    OLED_ClearBuffer();

    switch (s_page)
    {
        case MENU_PAGE_DEBUG:
            menu_draw_debug();
            break;

        case MENU_PAGE_MAIN:
        default:
            menu_draw_main();
            break;
    }

    OLED_Refresh();
}
