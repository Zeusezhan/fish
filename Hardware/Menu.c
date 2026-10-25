/* ============================================================
 * OLED 界面 + 按键路由(纯 UI, 不含任何跑图逻辑)
 *
 * 跑图状态机在 Function/InertialRun.c 里, 本文件只负责:
 *   1. 把按键翻译成 开始 / 停止 / 调参数 / 换页
 *   2. 从 InertialRun_GetInfo() 取快照画到 OLED
 *
 * 【画面】6x8 字库, 一行最多 21 字符
 *
 *  MAIN(选择页)
 *    === FISH RUNNER ===
 *    >Auto Run
 *     Single Seg
 *    Yaw:  -0.3
 *    IMU 99% F:0 B:0
 *    PE1:Next  PE0:OK
 *
 *  AUTO(自动跑图)
 *    AUTO 2/5 LEFT
 *    T:2.6s E:-0.3
 *    M1  -190748
 *    M2   190824
 *    M3  -190716
 *    M4   190860
 *    RUN
 *    PE0:STOP
 *
 *  SINGLE(单段调试, 空闲)
 *    SINGLE FWD 2.0s
 *    PE1:DIR  PE2:T+
 *    M1  ......
 *    READY
 *    PE0:GO  PE2H:BACK
 * ============================================================ */

#include "main.h"
#include "OLED.h"
#include "jy901s.h"
#include "key.h"
#include "Menu.h"
#include "InertialRun.h"
#include <stdio.h>

/* 长按判定(ms) */
#define MENU_LONG_PRESS_MS      800u

/* 绘制周期(ms): 车在动的时候放慢, 省下的都是主循环时间 */
#define MENU_REFRESH_IDLE_MS    200u
#define MENU_REFRESH_RUN_MS     300u

/* 单段调试的时长步进(ms) */
#define MENU_SINGLE_STEP_MS     500u

/* ==================== 内部状态 ==================== */

static MenuPage_t s_page = MENU_PAGE_MAIN;
static uint8_t    s_cursor = 0u;        /* 0 = Auto Run, 1 = Single Seg */
static uint8_t    s_need_redraw = 1u;
static uint32_t   s_last_draw = 0u;

static uint8_t    s_single_move = (uint8_t)IR_MOVE_FWD;
static uint16_t   s_single_ms = 2000u;

/* PE2 长按动作已经触发过(用来区分长按和短按) */
static uint8_t    s_long_fired = 0u;

/* ==================== 绘制 ==================== */

static void menu_draw_positions(const IrInfo_t *info)
{
    char line[22];
    uint8_t i;

    for (i = 0u; i < IR_MOTOR_NUM; i++)
    {
        if (info->position_valid[i] != 0u)
        {
            (void)snprintf(line, sizeof(line), "M%u %8ld",
                           (unsigned)(i + 1u), (long)info->position[i]);
        }
        else
        {
            (void)snprintf(line, sizeof(line), "M%u       --",
                           (unsigned)(i + 1u));
        }
        OLED_ShowString(0, (uint8_t)(2u + i), line);
    }
}

/* 选择页 */
static void menu_draw_main(void)
{
    char line[22];
    uint32_t ok;
    uint32_t fail;
    uint32_t bad;
    uint32_t total;
    uint32_t permille;

    OLED_ShowString(0, 0, "=== FISH RUNNER ===");

    OLED_ShowChar(0, 2, (s_cursor == 0u) ? '>' : ' ');
    OLED_ShowString(6, 2, "Auto Run");

    OLED_ShowChar(0, 3, (s_cursor == 1u) ? '>' : ' ');
    OLED_ShowString(6, 3, "Single Seg");

    /* 航向角: 上电零点是基准, 左转为正 */
    (void)snprintf(line, sizeof(line), "Yaw:%6.1f", (double)jy901s_data.yaw);
    OLED_ShowString(0, 5, line);

    /* 陀螺仪读取质量: 有效帧占比 + 通信失败数 + 坏帧数 */
    if (jy901s_data.ready == 0u)
    {
        OLED_ShowString(0, 6, "IMU:NO DATA");
    }
    else
    {
        ok = JY901S_GetOkCount();
        fail = JY901S_GetFailCount();
        bad = JY901S_GetBadCount();
        total = ok + fail + bad;
        permille = (total == 0u) ? 1000u : ((ok * 1000u) / total);

        (void)snprintf(line, sizeof(line), "IMU%3u%% F:%lu B:%lu",
                       (unsigned)(permille / 10u),
                       (unsigned long)fail,
                       (unsigned long)bad);
        OLED_ShowString(0, 6, line);
    }

    OLED_ShowString(0, 7, "PE1:Next  PE0:OK");
}

/* 运行页的公共部分: 行 2~5 显示四轮编码器 */
static void menu_draw_run_head(const IrInfo_t *info, uint8_t single)
{
    char line[22];

    if (single != 0u)
    {
        (void)snprintf(line, sizeof(line), "SINGLE %s %.1fs",
                       InertialRun_MoveName(info->move),
                       (double)info->duration_ms / 1000.0);
    }
    else
    {
        (void)snprintf(line, sizeof(line), "AUTO %u/%u %s",
                       (unsigned)(info->index + 1u),
                       (unsigned)info->total,
                       InertialRun_MoveName(info->move));
    }
    OLED_ShowString(0, 0, line);

    if (info->running != 0u)
    {
        /* 剩余时间 + 当前航向误差, 调 Kp/Ki 看这一行最直观 */
        uint32_t remain = (info->elapsed_ms < info->duration_ms)
                        ? (uint32_t)(info->duration_ms - info->elapsed_ms) : 0u;

        (void)snprintf(line, sizeof(line), "T:%u.%us E:%5.1f",
                       (unsigned)(remain / 1000u),
                       (unsigned)((remain % 1000u) / 100u),
                       (double)YawLoop_GetError());
    }
    else
    {
        (void)snprintf(line, sizeof(line), "T:%u.%us",
                       (unsigned)(info->duration_ms / 1000u),
                       (unsigned)((info->duration_ms % 1000u) / 100u));
    }
    OLED_ShowString(0, 1, line);
}

/* 自动跑图页 */
static void menu_draw_auto(void)
{
    IrInfo_t info;

    InertialRun_GetInfo(&info);
    menu_draw_run_head(&info, 0u);
    menu_draw_positions(&info);

    if (info.state == IR_IDLE)
    {
        OLED_ShowString(0, 6, info.result);
        OLED_ShowString(0, 7, "PE0:GO  PE2H:BACK");
    }
    else
    {
        OLED_ShowString(0, 6, InertialRun_StateName(info.state));
        OLED_ShowString(0, 7, "PE0:STOP");
    }
}

/* 单段调试页 */
static void menu_draw_single(void)
{
    IrInfo_t info;

    InertialRun_GetInfo(&info);
    menu_draw_run_head(&info, 1u);
    menu_draw_positions(&info);

    if (info.state == IR_IDLE)
    {
        /* 空闲: 显示当前设定 + 可调项, 第 1 行让给按键提示 */
        char line[22];

        (void)snprintf(line, sizeof(line), "PE1:DIR  PE2:T+");
        OLED_ShowString(0, 1, line);
        OLED_ShowString(0, 6, info.result);
        OLED_ShowString(0, 7, "PE0:GO  PE2H:BACK");
    }
    else
    {
        OLED_ShowString(0, 6, InertialRun_StateName(info.state));
        OLED_ShowString(0, 7, "PE0:STOP");
    }
}

/* ==================== 按键路由 ==================== */

/* PE2 长按是否刚触发(只在按住达到门限的那一拍返回 1) */
static uint8_t menu_long_pe2(void)
{
    if ((s_long_fired == 0u) && (Key_IsDown(KEY_3) != 0u)
        && (Key_GetHoldMs(KEY_3) >= MENU_LONG_PRESS_MS))
    {
        s_long_fired = 1u;
        return 1u;
    }
    return 0u;
}

static void menu_keys_main(uint8_t press, uint8_t release)
{
    (void)release;

    if (press == KEY_2)
    {
        s_cursor = (s_cursor == 0u) ? 1u : 0u;
        s_need_redraw = 1u;
    }
    else if (press == KEY_1)
    {
        s_page = (s_cursor == 0u) ? MENU_PAGE_AUTO : MENU_PAGE_SINGLE;
        s_long_fired = 0u;
        s_need_redraw = 1u;
    }
}

static void menu_keys_auto(uint8_t press, uint8_t release)
{
    /* 长按 PE2 返回选择页, 只在空闲时允许(跑图过程中不能走开) */
    if (menu_long_pe2() != 0u)
    {
        if (InertialRun_IsBusy() == 0u)
        {
            s_page = MENU_PAGE_MAIN;
            s_need_redraw = 1u;
        }
    }

    if (release == KEY_3)
    {
        s_long_fired = 0u;
    }

    /* PE0: 开始 / 停止 */
    if (press == KEY_1)
    {
        if (InertialRun_IsBusy() != 0u)
        {
            InertialRun_Cancel();
        }
        else
        {
            (void)InertialRun_StartAuto();
        }
        s_need_redraw = 1u;
    }
}

static void menu_keys_single(uint8_t press, uint8_t release)
{
    uint8_t busy = InertialRun_IsBusy();

    /* PE2 短按松开: 调时长(空闲时) */
    if (release == KEY_3)
    {
        if ((s_long_fired == 0u) && (busy == 0u))
        {
            s_single_ms = (uint16_t)(s_single_ms + MENU_SINGLE_STEP_MS);
            if (s_single_ms > IR_SINGLE_MS_MAX)
            {
                s_single_ms = IR_SINGLE_MS_MIN;
            }
            s_need_redraw = 1u;
        }
        s_long_fired = 0u;
    }

    /* PE2 长按: 返回选择页(空闲时) */
    if (menu_long_pe2() != 0u)
    {
        if (busy == 0u)
        {
            s_page = MENU_PAGE_MAIN;
            s_need_redraw = 1u;
        }
    }

    /* PE0: 开始 / 停止 */
    if (press == KEY_1)
    {
        if (busy != 0u)
        {
            InertialRun_Cancel();
        }
        else
        {
            (void)InertialRun_StartSingle((InertialMove_t)s_single_move,
                                          s_single_ms);
        }
        s_need_redraw = 1u;
    }

    /* PE1: 换方向(空闲时) */
    if ((press == KEY_2) && (busy == 0u))
    {
        s_single_move = (uint8_t)((s_single_move + 1u) % (uint8_t)IR_MOVE_COUNT);
        s_need_redraw = 1u;
    }
}

/* ==================== 对外接口 ==================== */

void Menu_Init(void)
{
    s_page = MENU_PAGE_MAIN;
    s_cursor = 0u;
    s_need_redraw = 1u;
    s_last_draw = 0u;
    s_single_move = (uint8_t)IR_MOVE_FWD;
    s_single_ms = 2000u;
    s_long_fired = 0u;
}

void Menu_Run(void)
{
    uint8_t press = Key_GetPress();
    uint8_t release = Key_GetRelease();
    uint32_t now;

    switch (s_page)
    {
        case MENU_PAGE_AUTO:
            menu_keys_auto(press, release);
            break;

        case MENU_PAGE_SINGLE:
            menu_keys_single(press, release);
            break;

        case MENU_PAGE_MAIN:
        default:
            menu_keys_main(press, release);
            break;
    }

    /* ---- 绘制(限频) ---- */
    now = HAL_GetTick();
    if ((s_need_redraw == 0u)
        && ((uint32_t)(now - s_last_draw)
            < ((InertialRun_IsBusy() != 0u) ? MENU_REFRESH_RUN_MS
                                            : MENU_REFRESH_IDLE_MS)))
    {
        return;
    }
    s_last_draw = now;
    s_need_redraw = 0u;

    OLED_ClearBuffer();

    switch (s_page)
    {
        case MENU_PAGE_AUTO:
            menu_draw_auto();
            break;

        case MENU_PAGE_SINGLE:
            menu_draw_single();
            break;

        case MENU_PAGE_MAIN:
        default:
            menu_draw_main();
            break;
    }

    OLED_Refresh();
}
