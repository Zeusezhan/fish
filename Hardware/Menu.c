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
#include <stdio.h>

/* PD 计算启停请求, 由 main.c 在主循环里应用。 */
extern volatile uint8_t g_mode_run_request;

/* 航向环状态, 由 main.c 的 TIM3 中断每 20ms 计算一次。 */
extern volatile uint8_t g_yaw_loop_enable;
extern volatile float   g_yaw_target;
extern volatile float   g_yaw_output;

/* 调试页航向闭环开关: 1 = 启动时记录目标航向并下发纠偏命令。 */
#define MENU_DEBUG_HEADING_LOOP_ENABLED   1

#define MENU_DEBUG_SPEED_RPM       50u

/* 调试运行时的航向修正转速限幅(RPM)。
 * 增益沿用 main.c 的 Kp/Kd, 这里只限住幅值,
 * 避免大误差时横移被自转分量吃掉。 */
#define MENU_DEBUG_TURN_MAX        20.0f

/* 航向环输出刷新周期(ms), 与 TIM3 的 20ms 运算周期一致 */
#define MENU_DEBUG_STEER_MS        20u

/* 秒数以 0.5 秒为单位保存, 便于用整数精确计时 */
#define MENU_DEBUG_MIN_HALF_SECONDS     2u
#define MENU_DEBUG_MAX_HALF_SECONDS     198u
#define MENU_DEBUG_HOLD_MS         1000u
#define MENU_DEBUG_READ_INTERVAL   500u
#define MENU_DEBUG_ZERO_WAIT_MS    100u

typedef enum
{
    MENU_DEBUG_MOVE_FORWARD = 0,
    MENU_DEBUG_MOVE_LEFT,
    MENU_DEBUG_MOVE_RIGHT,
    MENU_DEBUG_MOVE_BACKWARD,
    MENU_DEBUG_MOVE_COUNT
} MenuDebugMove_t;

/* 当前页面 */
static MenuPage_t s_page = MENU_PAGE_MAIN;

/* 第一页的光标位置(当前选中的菜单项) */
static uint8_t s_cursor = MENU_ITEM_PARAM;

/* 显示刷新计时 */
static uint32_t s_last_draw_tick = 0;

/* 需要立即重绘的标志: 切换页面/移动光标后置 1, 让界面立刻响应按键 */
static uint8_t s_need_redraw = 1;

static uint8_t s_debug_half_seconds = MENU_DEBUG_MIN_HALF_SECONDS;
static MenuDebugMove_t s_debug_move = MENU_DEBUG_MOVE_FORWARD;
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

/* 第一页的菜单项文字
 * (OLED 6x8 字库只有 ASCII, 不能用中文) */
static const char *const s_main_item[MENU_ITEM_NUM] =
{
    "Parameters",       /* MENU_ITEM_PARAM */
    "Mode Select",      /* MENU_ITEM_MODE */
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
  * @brief  绘制参数界面: 显示陀螺仪解算出来的数据
  * @note   数据来自 jy901s_data(见 jy901s.h)
  */
static void menu_draw_param(void)
{
    OLED_ShowString(0, 0, "--- Parameters ---");

    /* 姿态角 */
    OLED_ShowString(0, 1, "Yaw:");
    OLED_ShowFloat(30, 1, jy901s_data.yaw, 1);

    OLED_ShowString(0, 2, "Pit:");
    OLED_ShowFloat(30, 2, jy901s_data.pitch, 1);

    OLED_ShowString(0, 3, "Rol:");
    OLED_ShowFloat(30, 3, jy901s_data.roll, 1);

    /* 三轴角速度 deg/s */
    OLED_ShowString(0, 4, "Gx:");
    OLED_ShowFloat(24, 4, jy901s_data.gx, 1);

    OLED_ShowString(0, 5, "Gy:");
    OLED_ShowFloat(24, 5, jy901s_data.gy, 1);

    OLED_ShowString(0, 6, "Gz:");
    OLED_ShowFloat(24, 6, jy901s_data.gz, 1);

    OLED_ShowString(0, 7, "PE2:Back");
}

/**
  * @brief  绘制 PD 监测界面
  * @note   PE0 只启停航向 PD 计算, 不控制底盘电机。
  */
static void menu_draw_mode(void)
{
    OLED_ShowString(0, 0, "--- Mode Select ---");

    if (g_mode_run_request)
    {
        OLED_ShowString(0, 2, "PD: ON");
        OLED_ShowString(0, 4, "PE0: PD OFF");
    }
    else
    {
        OLED_ShowString(0, 2, "PD: OFF");
        OLED_ShowString(0, 4, "PE0: PD ON");
    }

    /* 显示实时航向 */
    OLED_ShowString(0, 5, "Yaw:");
    OLED_ShowFloat(30, 5, jy901s_data.yaw, 1);

    OLED_ShowString(0, 7, "PE2:Back");
}

static void menu_draw_debug(void)
{
    char line[32];
    const char *move_name;
    uint8_t i;
    uint32_t remaining_ms;

    switch (s_debug_move)
    {
        case MENU_DEBUG_MOVE_LEFT:
            move_name = "LEFT";
            break;
        case MENU_DEBUG_MOVE_RIGHT:
            move_name = "RIGHT";
            break;
        case MENU_DEBUG_MOVE_BACKWARD:
            move_name = "BACK";
            break;
        case MENU_DEBUG_MOVE_FORWARD:
        default:
            move_name = "FWD";
            break;
    }

    (void)snprintf(line, sizeof(line), "N%03u %-5s %s",
                   (unsigned int)s_debug_node, move_name, s_debug_status);
    OLED_ShowString(0, 0, line);

    if (s_debug_running)
    {
        remaining_ms = (uint32_t)s_debug_half_seconds * 500u
                     - (HAL_GetTick() - s_debug_start_tick);
        (void)snprintf(line, sizeof(line), "T:%u.%uS V:%uRPM",
                       (unsigned int)((remaining_ms + 499u) / 1000u),
                       (unsigned int)(((remaining_ms % 1000u) >= 500u) ? 5u : 0u),
                       (unsigned int)MENU_DEBUG_SPEED_RPM);
    }
    else
    {
        (void)snprintf(line, sizeof(line), "T:%u.%uS PE1:+ PE2:-",
                       (unsigned int)(s_debug_half_seconds / 2u),
                       (unsigned int)((s_debug_half_seconds % 2u) ? 5u : 0u));
    }
    OLED_ShowString(0, 1, line);

    for (i = 0; i < 4u; i++)
    {
        if (s_debug_read_valid[i])
        {
            (void)snprintf(line, sizeof(line), "M%u:%ld",
                           (unsigned int)(i + 1u),
                           (long)s_debug_position[i]);
        }
        else
        {
            (void)snprintf(line, sizeof(line), "M%u:--",
                           (unsigned int)(i + 1u));
        }
        OLED_ShowString(0, (uint8_t)(2u + i), line);
    }

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
        OLED_ShowString(0, 6, "PE0:DIR H=GO V50");
        OLED_ShowString(0, 7, "PE1H:ZERO PE2H:BACK");
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
        else if (s_debug_position[i] != 0)
        {
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
 * 每圈都发会把主循环和菜单刷新拖慢。 */
static void menu_debug_steer(uint32_t now)
{
    /* 停车锁优先: 停车期间绝不能再下发任何速度命令 */
    if (s_debug_stop_lock || !s_debug_running || !s_debug_heading_hold)
    {
        return;
    }

    if ((int32_t)(now - s_debug_next_steer_tick) < 0)
    {
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

    if (s_debug_start_pending && Key_IsDown(KEY_1)
        && (Key_GetHoldMs(KEY_1) >= MENU_DEBUG_HOLD_MS))
    {
        s_debug_start_pending = 0u;
        menu_debug_start();
    }

    if (s_debug_key1_pending && Key_IsDown(KEY_2)
        && (Key_GetHoldMs(KEY_2) >= MENU_DEBUG_HOLD_MS))
    {
        s_debug_key1_pending = 0u;
        menu_debug_clear_positions();
    }

    if (s_debug_key3_pending && Key_IsDown(KEY_3)
        && (Key_GetHoldMs(KEY_3) >= MENU_DEBUG_HOLD_MS))
    {
        s_debug_key3_pending = 0u;
        if (!s_debug_running)
        {
            s_page = MENU_PAGE_MAIN;
            s_need_redraw = 1u;
        }
    }

    release = Key_GetRelease();
    if ((release == KEY_1) && s_debug_start_pending)
    {
        s_debug_start_pending = 0u;
        if (!s_debug_running)
        {
            s_debug_move = (MenuDebugMove_t)((s_debug_move + 1u)
                                           % MENU_DEBUG_MOVE_COUNT);
            s_need_redraw = 1u;
        }
    }
    else if ((release == KEY_2) && s_debug_key1_pending)
    {
        s_debug_key1_pending = 0u;
        if (!s_debug_running
            && (s_debug_half_seconds < MENU_DEBUG_MAX_HALF_SECONDS))
        {
            s_debug_half_seconds++;
            s_need_redraw = 1u;
        }
    }
    else if ((release == KEY_3) && s_debug_key3_pending)
    {
        s_debug_key3_pending = 0u;
        if (!s_debug_running
            && (s_debug_half_seconds > MENU_DEBUG_MIN_HALF_SECONDS))
        {
            s_debug_half_seconds--;
            s_need_redraw = 1u;
        }
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
            else if (s_cursor == MENU_ITEM_MODE)
            {
                s_page = MENU_PAGE_MODE;
            }
            else
            {
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
            }
            s_need_redraw = 1;
        }
        /* 按 PE2(KEY_3) 不做任何事: 已经在最顶层界面了 */
    }
    else
    {
        /* ---------- 子界面 ---------- */
        if (s_page == MENU_PAGE_MODE)
        {
            /* PE0: 启动 / 停止 切换(这个键在本页当启停开关用) */
            if (key == KEY_1)
            {
                g_mode_run_request = (uint8_t)(!g_mode_run_request);
                s_need_redraw = 1;
            }
            /* PE2: 返回第一页 */
            else if (key == KEY_3)
            {
                s_page = MENU_PAGE_MAIN;
                s_need_redraw = 1;
            }
        }
        else
        {
            /* 参数界面: 只响应 PE2 返回 */
            if (key == KEY_3)
            {
                s_page = MENU_PAGE_MAIN;
                s_need_redraw = 1;
            }
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
    s_debug_running = 0u;
    s_debug_clear_pending = 0u;
    s_debug_half_seconds = MENU_DEBUG_MIN_HALF_SECONDS;
    s_debug_move = MENU_DEBUG_MOVE_FORWARD;
    s_debug_node = 1u;
    s_debug_read_error = 0u;
    s_debug_start_pending = 0u;
    s_debug_heading_hold = 0u;
    s_debug_next_steer_tick = 0u;
    s_debug_stop_lock = 0u;
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
                >= (uint32_t)s_debug_half_seconds * 500u))
        {
            menu_debug_stop();
        }
        else if (s_debug_running)
        {
            menu_debug_steer(now);
        }
        else if (!s_debug_running
                 && ((int32_t)(now - s_debug_next_read_tick) >= 0))
        {
            menu_debug_read_positions(now);
        }
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

        case MENU_PAGE_MODE:
            menu_draw_mode();
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
  * @retval 当前页面编号(MENU_PAGE_MAIN / MENU_PAGE_PARAM / MENU_PAGE_MODE / MENU_PAGE_DEBUG)
  * @note   以后跑图逻辑需要知道"现在是不是在参数页"时可以用到。
  */
MenuPage_t Menu_GetPage(void)
{
    return s_page;
}
