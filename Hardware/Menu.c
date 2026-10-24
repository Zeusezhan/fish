/* ============================================================
 * OLED 显示菜单
 *
 * 【页面结构】
 *   第一页(选择界面) ──确定──> 参数界面 / 模式选择
 *                    <──返回──
 *
 * 【按键】按键编号见 key.h: KEY_1=PE0, KEY_2=PE1, KEY_3=PE2
 *   第一页:  PE1(KEY_2) = 下一个    PE0(KEY_1) = 确定
 *   子界面:  PE2(KEY_3) = 返回
 *   第一页已是最顶层, 按 PE2 不做任何事。
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
#include "inertial.h"
#include <stdio.h>

/* 航向环状态, 由 main.c 的 TIM3 中断每 20ms 计算一次。 */
extern volatile uint8_t g_yaw_loop_enable;
extern volatile float   g_yaw_target;
extern volatile float   g_yaw_output;

/* 调试页航向闭环开关: 1 = 启动时记录目标航向并下发纠偏命令。
 * ★ 只有它和 main.c 里的 HAL_TIM_Base_Start_IT(&htim3) 同时打开,
 *   PD 才会真正算出来并下发; 少了任何一个都会退化成开环运动。 */
#define MENU_DEBUG_HEADING_LOOP_ENABLED   1

#define MENU_DEBUG_SPEED_RPM       50u

/* 调试运行时的航向修正转速限幅(RPM)。 */
#define MENU_DEBUG_TURN_MAX        20.0f

/* 航向环输出刷新周期(ms), 与 TIM3 的 20ms 运算周期一致 */
#define MENU_DEBUG_STEER_MS        20u

/* 跑动中多久没读到新数据就认为陀螺仪掉线(ms), 掉线立即停车 */
#define MENU_DEBUG_IMU_TIMEOUT_MS  100u

/* 秒数以 0.1 秒为单位保存, 便于用整数精确计时 */
#define MENU_DEBUG_MIN_TENTH_SECONDS    10u     /* 1.0 秒 */
#define MENU_DEBUG_MAX_TENTH_SECONDS    990u    /* 99.0 秒 */
#define MENU_DEBUG_HOLD_MS         1000u
#define MENU_DEBUG_READ_INTERVAL   500u
#define MENU_DEBUG_ZERO_WAIT_MS    100u

/* ---- 自动跑图相关 ---- */
/* 每段起步前的等待时间(ms) */
#define MENU_AUTO_LEAD_MS          1000u
/* 段间停稳后等待读数的时间(ms) */
#define MENU_AUTO_SETTLE_MS        300u
/* 等待停车锁释放的上限(ms) */
#define MENU_AUTO_LOCK_TIMEOUT_MS  300u
/* 清零判定容差: 读回绝对值小于该值就认为已清零 */
#define MENU_DEBUG_ZERO_TOLERANCE  10
/* 清零后等待读回校验的超时(ms) */
#define MENU_AUTO_ZERO_TIMEOUT_MS  2000u
/* 清零校验失败时最多重试次数(驱动器可能来不及处理) */
#define MENU_AUTO_ZERO_RETRY       3u

typedef enum
{
    MENU_DEBUG_MOVE_FORWARD = 0,
    MENU_DEBUG_MOVE_LEFT,
    MENU_DEBUG_MOVE_RIGHT,
    MENU_DEBUG_MOVE_BACKWARD,
    MENU_DEBUG_MOVE_COUNT
} MenuDebugMove_t;

/* Debug 页内的二级选择: 先选模式, 再进入对应功能 */
typedef enum
{
    MENU_DEBUG_VIEW_CHOOSE = 0,     /* 二选一: Auto Run / Single Seg */
    MENU_DEBUG_VIEW_AUTO,           /* 自动跑完惯导五段 */
    MENU_DEBUG_VIEW_SINGLE          /* 单段调试: 可设方向与时长 */
} MenuDebugView_t;

/* 自动跑段状态机 */
typedef enum
{
    MENU_AUTO_IDLE = 0,     /* 未启动 */
    MENU_AUTO_COUNTDOWN,    /* 显示段号, 等待起步 */
    MENU_AUTO_RUN,          /* 正在跑该段 */
    MENU_AUTO_SETTLE,       /* 已停车, 等车停稳后读一次 */
    MENU_AUTO_ZERO,         /* 发清零命令 */
    MENU_AUTO_ZERO_WAIT,    /* 等驱动器处理, 再读回校验 */
    MENU_AUTO_DONE          /* 五段跑完 */
} MenuAutoState_t;

/* 当前页面 */
static MenuPage_t s_page = MENU_PAGE_MAIN;

/* 第一页的光标位置(当前选中的菜单项) */
static uint8_t s_cursor = MENU_ITEM_PARAM;

/* 显示刷新计时 */
static uint32_t s_last_draw_tick = 0;

/* 需要立即重绘的标志: 切换页面/移动光标后置 1, 让界面立刻响应按键 */
static uint8_t s_need_redraw = 1;

static uint16_t s_debug_tenth_seconds = MENU_DEBUG_MIN_TENTH_SECONDS;
static MenuDebugMove_t s_debug_move = MENU_DEBUG_MOVE_FORWARD;
static MenuDebugView_t s_debug_view = MENU_DEBUG_VIEW_CHOOSE;
static uint8_t s_debug_choice = 0u;    /* 0=Auto Run, 1=Single Seg */
static MenuAutoState_t s_auto_state = MENU_AUTO_IDLE;
static uint8_t  s_auto_index = 0u;      /* 当前段序号 0~4 */
static uint8_t  s_auto_pending_zero = 0u;
static uint8_t  s_auto_zero_ok = 0u;    /* 清零读回结果: 1=成功 */
static uint8_t  s_auto_retry = 0u;      /* 清零校验重试次数 */
static uint32_t s_auto_zero_worst = 0u; /* 清零后实测到的最大绝对值 */
static char     s_auto_result[20] = "NONE";  /* 中止原因, 不被读数覆盖 */
static uint16_t s_debug_read_count = 0u; /* 读数执行次数, 仅用于诊断 */
static uint32_t s_auto_deadline = 0u;
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

/* 清零函数在自动状态机之后定义, 这里先声明 */
static void menu_debug_clear_positions(void);

/* 第一页的菜单项文字
 * (OLED 6x8 字库只有 ASCII, 不能用中文) */
static const char *const s_main_item[MENU_ITEM_NUM] =
{
    "Parameters",       /* MENU_ITEM_PARAM */
    "Debug",            /* MENU_ITEM_DEBUG */
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

/**
  * @brief  绘制参数界面: 只显示航向角和俯仰角
  */
static void menu_draw_param(void)
{
    OLED_ShowString(0, 0, "--- Parameters ---");

    OLED_ShowString(0, 2, "Yaw:");
    OLED_ShowFloat(30, 2, jy901s_data.yaw, 1);

    OLED_ShowString(0, 4, "Pit:");
    OLED_ShowFloat(30, 4, jy901s_data.pitch, 1);

    OLED_ShowString(0, 7, "PE2:Back");
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

/* 取自动状态名字 */
static const char *menu_auto_state_name(void)
{
    static const char *const name[] =
    {
        "IDLE", "COUNT", "RUN", "SETTLE", "ZERO", "ZEROWAIT", "DONE"
    };

    if ((uint8_t)s_auto_state > (uint8_t)MENU_AUTO_DONE)
    {
        return "IDLE";
    }
    return name[s_auto_state];
}

/* 绘制 M1~M4 四行读数值 */
static void menu_draw_positions(void)
{
    char line[24];
    uint8_t i;

    for (i = 0; i < 4u; i++)
    {
        if (s_debug_read_valid[i])
        {
            (void)snprintf(line, sizeof(line), "M%u:%ld", (unsigned)(i + 1u),
                           (long)s_debug_position[i]);
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

/* 绘制 Debug 页的模式选择界面 */
static void menu_draw_debug_choose(void)
{
    static const char *const item[] = { "Auto Run", "Single Seg" };
    uint8_t i;

    OLED_ShowString(0, 0, "----- DEBUG -----");

    for (i = 0u; i < 2u; i++)
    {
        uint8_t row = (uint8_t)(2u + i * 2u);

        OLED_ShowChar(0, row, (s_debug_choice == i) ? '>' : ' ');
        OLED_ShowString(6, row, item[i]);
    }

    OLED_ShowString(0, 7, "PE1:Next PE0:OK");
}

/* 绘制自动跑图页 */
static void menu_draw_debug_auto(void)
{
    char line[24];
    uint8_t active = (s_auto_state != MENU_AUTO_IDLE);

    if (active)
    {
        (void)snprintf(line, sizeof(line), "AUTO %u/%u %s",
                       (unsigned)(s_auto_index + 1u),
                       (unsigned)INERTIAL_SEGMENT_NUM, menu_move_name());
        OLED_ShowString(0, 0, line);

        (void)snprintf(line, sizeof(line), "SEG %u/%u WAIT",
                       (unsigned)(s_auto_index + 1u),
                       (unsigned)INERTIAL_SEGMENT_NUM);
        menu_draw_timer_line(line);
    }
    else
    {
        OLED_ShowString(0, 0, "-- Auto Run --");
        OLED_ShowString(0, 1, "HOLD PE0 TO START");
    }

    menu_draw_positions();

    /* 第 6 行: 运行 > 阶段 > 结果 */
    if (s_debug_running)
    {
        OLED_ShowString(0, 6, "PE0:STOP");
    }
    else if (active)
    {
        (void)snprintf(line, sizeof(line), "ST:%s", menu_auto_state_name());
        OLED_ShowString(0, 6, line);

        /* 第 7 行: 显示本段序号与进度 */
        (void)snprintf(line, sizeof(line), "SEG %u/%u %s",
                       (unsigned)(s_auto_index + 1u),
                       (unsigned)INERTIAL_SEGMENT_NUM,
                       menu_move_name());
        OLED_ShowString(0, 7, line);
    }
    else
    {
        (void)snprintf(line, sizeof(line), "RESULT:%s", s_auto_result);
        OLED_ShowString(0, 6, line);
        OLED_ShowString(0, 7, "HOLD PE0 TO RERUN");
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

    menu_draw_positions();

    if (s_debug_running)
    {
        OLED_ShowString(0, 6, "PE0:STOP");
        if (s_debug_heading_hold)
        {
            (void)snprintf(line, sizeof(line), "YawErr:%5.1f",
                           (double)(g_yaw_target
                                    - jy901s_data.yaw_continuous));
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

/* 按二级选择分发到对应界面 */
static void menu_draw_debug(void)
{
    switch (s_debug_view)
    {
        case MENU_DEBUG_VIEW_AUTO:   menu_draw_debug_auto();   break;
        case MENU_DEBUG_VIEW_SINGLE: menu_draw_debug_single(); break;
        case MENU_DEBUG_VIEW_CHOOSE:
        default:                     menu_draw_debug_choose(); break;
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
            if (s_auto_pending_zero)
            {
                s_auto_zero_ok = 1u;
                s_auto_pending_zero = 0u;   /* 回填完成, 通知状态机继续 */
            }
        }
        else
        {
            s_debug_status = "ZERO ERR";
            s_debug_read_error = (uint8_t)!all_valid;

            /* 记录清零后实测到的最大绝对值, 便于判断是"没清零"还是"差一点" */
            {
                uint32_t worst = 0u;
                uint8_t  k;

                for (k = 0u; k < 4u; k++)
                {
                    uint32_t m;

                    if (!s_debug_read_valid[k])
                    {
                        continue;
                    }

                    if (s_debug_position[k] >= 0)
                    {
                        m = (uint32_t)s_debug_position[k];
                    }
                    else if (s_debug_position[k] == (-2147483647 - 1))
                    {
                        m = 2147483647u;
                    }
                    else
                    {
                        m = (uint32_t)(-s_debug_position[k]);
                    }

                    if (m > worst)
                    {
                        worst = m;
                    }
                }
                s_auto_zero_worst = worst;
            }

            if (s_auto_pending_zero)
            {
                s_auto_zero_ok = 0u;
                s_auto_pending_zero = 0u;   /* 回填完成(失败) */
            }
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

    if (s_debug_read_count < 9999u)
    {
        s_debug_read_count++;
    }

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
        g_yaw_loop_enable = 0u;
        g_yaw_output = 0.0f;
        g_yaw_target = jy901s_data.yaw_continuous;
        g_yaw_loop_enable = 1u;
    }
    else
    {
        g_yaw_loop_enable = 0u;
        g_yaw_output = 0.0f;
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
    g_yaw_loop_enable = 0u;
    g_yaw_output = 0.0f;
    s_debug_heading_hold = 0u;
    s_debug_running = 0u;

    /* 3. 发停车序列(逐台缓存停止 + 广播同步) */
    s_debug_status = Car_Stop() ? "STOP" : "STOP TX ERR";

    /* 4. 停稳后读取位置计数, 读数期间锁保持, 避免读数与运动命令抢串口 */
    s_debug_next_read_tick = HAL_GetTick() + MENU_DEBUG_ZERO_WAIT_MS;
    s_need_redraw = 1u;
}

/* 运行中按 MENU_DEBUG_STEER_MS 刷新底盘命令, 让航向修正真正下发。
 * 不能每次主循环都发: Car_Mecanum 内部有 4 个 HAL_Delay(2),
 * 每圈都发会把主循环和菜单刷新拖慢。
 *
 * ★ 控制流是"主循环按 20ms 定时下发 + 中断只算不填", 所以即使
 *   PD 中断没有及时算出新值, 车也会按最后一拍的修正量继续走,
 *   不会因为一次中断丢拍就停在那里等。 */
static void menu_debug_steer(uint32_t now)
{
    uint32_t imu_tick;

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
     * 判定依据: 主循环每 20ms 调一次 JY901S_Poll(), 读数停了
     * 就说明 I2C 出问题了。 */
    imu_tick = JY901S_GetUpdateTick();
    if ((jy901s_data.ready == 0u)
        || ((uint32_t)(now - imu_tick) > MENU_DEBUG_IMU_TIMEOUT_MS))
    {
        menu_debug_stop();
        s_debug_status = "IMU LOST";
        s_need_redraw = 1u;
        return;
    }

    s_debug_next_steer_tick = now + MENU_DEBUG_STEER_MS;
    menu_debug_apply_move();
}

/* ==================== 自动跑图状态机 ==================== */

/* 按段序号取对应的调试动作与时长 */
static void menu_auto_load_segment(uint8_t index)
{
    const InertialSegment_t *seg = &g_inertial_segments[index];

    s_debug_tenth_seconds = (uint16_t)(seg->duration_ms / 100u);

    switch (seg->type)
    {
        case INERTIAL_MOVE_LEFT:
            s_debug_move = MENU_DEBUG_MOVE_LEFT;
            break;
        case INERTIAL_MOVE_RIGHT:
            s_debug_move = MENU_DEBUG_MOVE_RIGHT;
            break;
        case INERTIAL_MOVE_BACKWARD:
            s_debug_move = MENU_DEBUG_MOVE_BACKWARD;
            break;
        case INERTIAL_MOVE_FORWARD:
        default:
            s_debug_move = MENU_DEBUG_MOVE_FORWARD;
            break;
    }
}

/* 启动自动跑图 */
static void menu_auto_begin(void)
{
    if (s_debug_stop_lock || s_debug_running)
    {
        return;
    }

    s_auto_index = 0u;
    s_auto_pending_zero = 0u;
    s_auto_zero_ok = 0u;
    s_auto_retry = 0u;
    s_auto_zero_worst = 0u;
    s_debug_node = 1u;
    s_debug_status = "AUTO";
    s_auto_result[0] = 'R';
    s_auto_result[1] = 'U';
    s_auto_result[2] = 'N';
    s_auto_result[3] = '\0';
    s_auto_state = MENU_AUTO_COUNTDOWN;
    menu_auto_load_segment(s_auto_index);
    s_auto_deadline = HAL_GetTick() + MENU_AUTO_LEAD_MS;
    s_debug_read_valid[0] = 0u;
    s_debug_read_valid[1] = 0u;
    s_debug_read_valid[2] = 0u;
    s_debug_read_valid[3] = 0u;
    s_need_redraw = 1u;
}

/* 结束整轮自动跑图, 记录原因(不被读数函数覆盖) */
static void menu_auto_abort(const char *reason)
{
    uint8_t i;

    s_auto_state = MENU_AUTO_IDLE;
    s_auto_pending_zero = 0u;
    s_debug_status = reason;

    for (i = 0u; (i < (sizeof(s_auto_result) - 1u)) && (reason[i] != '\0'); i++)
    {
        s_auto_result[i] = reason[i];
    }
    s_auto_result[i] = '\0';

    s_need_redraw = 1u;
}

/* 自动跑图状态机, 由 Menu_Run() 每次主循环调用。
 *
 * 每段时序(状态机自己掌控, 不依赖主循环的读数分支):
 *   COUNTDOWN --(1s)--> 启动 -> RUN
 *   RUN --(计时到, 主循环已停车)--> SETTLE
 *   SETTLE --(100ms 停稳)--> 自己读一次(释放锁) -> ZERO
 *   ZERO --(锁已释放)--> 发清零 -> ZERO_WAIT
 *   ZERO_WAIT --(100ms)--> 读回校验 -> 成功进下一段
 */
static void menu_auto_step(uint32_t now)
{
    switch (s_auto_state)
    {
        case MENU_AUTO_COUNTDOWN:
            if ((int32_t)(now - s_auto_deadline) < 0)
            {
                break;
            }

            if (s_debug_stop_lock)
            {
                if ((int32_t)(now - s_auto_deadline)
                    >= (int32_t)MENU_AUTO_LOCK_TIMEOUT_MS)
                {
                    menu_auto_abort("LOCK TIMEOUT");
                }
                break;
            }

            menu_debug_start();
            if (s_debug_running)
            {
                s_auto_state = MENU_AUTO_RUN;
                s_need_redraw = 1u;
            }
            else
            {
                menu_auto_abort("ENA ERR");
            }
            break;

        case MENU_AUTO_RUN:
            if (!s_debug_running)
            {
                /* 主循环已完成停车(锁=1), 这里只记账 */
                s_auto_state = MENU_AUTO_SETTLE;
                s_auto_deadline = now + MENU_DEBUG_ZERO_WAIT_MS;
                s_need_redraw = 1u;
            }
            break;

        case MENU_AUTO_SETTLE:
            /* 等 100ms 让车停稳, 然后本阶段自己读一次。
             * 读数函数内部会释放停车锁, 所以这里不能先等锁。 */
            if ((int32_t)(now - s_auto_deadline) < 0)
            {
                break;
            }

            menu_debug_read_positions(now);
            s_auto_state = MENU_AUTO_ZERO;
            s_auto_deadline = now + MENU_AUTO_LOCK_TIMEOUT_MS;
            s_need_redraw = 1u;
            break;

        case MENU_AUTO_ZERO:
            /* 读数已完成, 锁应已释放; 发清零。
             * 发完清零后直接进入下一段, 不做读回校验。 */
            if (s_debug_stop_lock || s_debug_clear_pending || s_debug_running)
            {
                if ((int32_t)(now - s_auto_deadline)
                    >= (int32_t)MENU_AUTO_LOCK_TIMEOUT_MS)
                {
                    menu_auto_abort("ZERO STUCK");
                }
                break;
            }

            menu_debug_clear_positions();

            if (!s_debug_clear_pending)
            {
                menu_auto_abort("ZERO TX ERR");
                break;
            }

            /* 给驱动器留出处理时间, 然后直接切下一段 */
            s_auto_state = MENU_AUTO_ZERO_WAIT;
            s_auto_deadline = now + MENU_AUTO_SETTLE_MS;
            s_need_redraw = 1u;
            break;

        case MENU_AUTO_ZERO_WAIT:
            /* 清零命令已发出, 等一小段让驱动器执行, 然后进入下一段。
             * 不读回校验: 清零是否精确归零不影响跑图流程。 */
            if ((int32_t)(now - s_auto_deadline) < 0)
            {
                break;
            }

            /* 清掉读数模块里可能挂起的清零校验标志, 避免影响后续读数 */
            s_debug_clear_pending = 0u;

            if ((s_auto_index + 1u) < INERTIAL_SEGMENT_NUM)
            {
                s_auto_index++;
                menu_auto_load_segment(s_auto_index);
                s_auto_state = MENU_AUTO_COUNTDOWN;
                s_auto_deadline = now + MENU_AUTO_LEAD_MS;
            }
            else
            {
                s_auto_state = MENU_AUTO_IDLE;
                s_debug_status = "ALL DONE";
                s_auto_result[0] = 'D';
                s_auto_result[1] = 'O';
                s_auto_result[2] = 'N';
                s_auto_result[3] = 'E';
                s_auto_result[4] = '\0';
            }
            s_need_redraw = 1u;
            break;

        case MENU_AUTO_IDLE:
        case MENU_AUTO_DONE:
        default:
            break;
    }
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

    /* ---- 模式选择界面: 只处理上下移动与确认 ---- */
    if (s_debug_view == MENU_DEBUG_VIEW_CHOOSE)
    {
        if (key == KEY_2)
        {
            s_debug_choice = (uint8_t)(1u - s_debug_choice);
            s_need_redraw = 1u;
        }
        else if (key == KEY_1)
        {
            s_need_redraw = 1u;
            if (s_debug_choice == 0u)
            {
                s_debug_view = MENU_DEBUG_VIEW_AUTO;
                s_debug_status = "AUTO";
                menu_auto_begin();
            }
            else
            {
                s_debug_view = MENU_DEBUG_VIEW_SINGLE;
                s_debug_status = "READY";
                s_debug_move = MENU_DEBUG_MOVE_FORWARD;
                s_debug_tenth_seconds = MENU_DEBUG_MIN_TENTH_SECONDS;
            }
        }
        else if (key == KEY_3)
        {
            s_page = MENU_PAGE_MAIN;
            s_need_redraw = 1u;
        }
        return;
    }

    if (key == KEY_1)
    {
        /* 自动跑图中按 PE0 = 终止整轮 */
        if (s_auto_state != MENU_AUTO_IDLE)
        {
            menu_auto_abort("USER STOP");
            if (s_debug_running)
            {
                menu_debug_stop();
            }
            return;
        }

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
        if (s_debug_view == MENU_DEBUG_VIEW_AUTO)
        {
            menu_auto_begin();
        }
        else
        {
            menu_debug_start();
        }
    }

    if (s_debug_key1_pending && menu_is_held(KEY_2))
    {
        s_debug_key1_pending = 0u;
        menu_debug_clear_positions();
    }

    if (s_debug_key3_pending && menu_is_held(KEY_3))
    {
        s_debug_key3_pending = 0u;
        if (!s_debug_running && (s_auto_state == MENU_AUTO_IDLE))
        {
            s_debug_view = MENU_DEBUG_VIEW_CHOOSE;
            s_debug_status = "READY";
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
            /* PE0: 确定, 进入光标所指的界面 */
            if (s_cursor == MENU_ITEM_PARAM)
            {
                s_page = MENU_PAGE_PARAM;
            }
            else
            {
                s_page = MENU_PAGE_DEBUG;
                s_debug_view = MENU_DEBUG_VIEW_CHOOSE;
                s_debug_choice = 0u;
                s_auto_state = MENU_AUTO_IDLE;
                s_auto_index = 0u;
                s_auto_pending_zero = 0u;
                s_auto_zero_ok = 0u;
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
            }
            s_need_redraw = 1;
        }
        /* 按 PE2(KEY_3) 不做任何事: 已经在最顶层界面了 */
    }
    else
    {
        /* ---------- 子界面: 参数界面只响应 PE2 返回 ---------- */
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
    s_cursor = MENU_ITEM_PARAM;
    s_need_redraw = 1;
    s_last_draw_tick = 0;
    s_debug_view = MENU_DEBUG_VIEW_CHOOSE;
    s_debug_choice = 0u;
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
    s_auto_state = MENU_AUTO_IDLE;
    s_auto_index = 0u;
    s_auto_pending_zero = 0u;
    s_auto_zero_ok = 0u;
    s_auto_deadline = 0u;
    s_debug_read_count = 0u;
    s_auto_result[0] = 'N';
    s_auto_result[1] = 'O';
    s_auto_result[2] = 'N';
    s_auto_result[3] = 'E';
    s_auto_result[4] = '\0';
    s_auto_retry = 0u;
    s_auto_zero_worst = 0u;
}

/**
  * @brief  菜单主函数
  * @note   放到 while(1) 里反复调用(前提是已经调用过 Key_Scan())。
  *
  *         按键是"按一次触发一次", 所以这里的按键处理每次都执行;
  *         绘制则按 MENU_REFRESH_MS 限频, 参数界面靠它周期刷新数据。
  *         按键导致页面/光标变化时置 s_need_redraw, 界面立刻更新。
  */
void Menu_Run(void)
{
    uint32_t now;

    /* ---- 1. 处理按键 ---- */
    if (s_page == MENU_PAGE_DEBUG)
    {
        menu_handle_debug_keys();
    }
    else
    {
        menu_handle_key(Key_GetPress());
    }

    /* ---- 2. 调试页运行计时与停稳后的编码器读取 ---- */
    now = HAL_GetTick();
    if (s_page == MENU_PAGE_DEBUG)
    {
        if (s_debug_running
            && ((now - s_debug_start_tick)
                >= (uint32_t)s_debug_tenth_seconds * 100u))
        {
            menu_debug_stop();
        }
        else if (s_debug_running)
        {
            menu_debug_steer(now);
        }
        else if (!s_debug_running
                 && (s_auto_state == MENU_AUTO_IDLE)
                 && ((int32_t)(now - s_debug_next_read_tick) >= 0))
        {
            /* 自动流程自己驱动读数, 这里只在空闲时给单段调试用 */
            menu_debug_read_positions(now);
        }

        /* 自动跑图状态机 */
        menu_auto_step(now);
    }

    /* ---- 2. 绘制(限频) ---- */
    now = HAL_GetTick();
    if (!s_need_redraw && ((now - s_last_draw_tick) < MENU_REFRESH_MS))
    {
        return;
    }
    s_last_draw_tick = now;
    s_need_redraw = 0;

    OLED_ClearBuffer();

    switch (s_page)
    {
        case MENU_PAGE_PARAM:
            menu_draw_param();
            break;

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

/**
  * @brief  查询当前所在的页面
  * @retval 当前页面编号(MENU_PAGE_MAIN / MENU_PAGE_PARAM / MENU_PAGE_DEBUG)
  */
MenuPage_t Menu_GetPage(void)
{
    return s_page;
}
