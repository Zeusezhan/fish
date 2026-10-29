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
 *    RUN YAW:  -0.3
 *    PE0:STOP
 *
 *  SINGLE(单段调试)
 *    SINGLE FWD 2.0s
 *    TOTAL 2.0s Y: -0.3
 *    M1  ......
 *    READY
 *    1:+ 2:- 0:DIR
 *
 *  ★ 单段页按键(第 7 行常驻显示, 运行中也不消失):
 *      PE0 短按 = 换方向    PE0 长按 = 开始 / 停止
 *      PE1 短按 = 加时长(按当前步进 0.1s / 1.0s)
 *      PE1 长按 = 切换时长步进 0.1s / 1.0s
 *      PE2 短按 = 减时长(按当前步进)
 *      PE2 长按 = 返回选择页
 *    ★ 每个键的短按/长按含义固定且互不重叠, 长按不会误触发短按动作。
 *    ★ 每次按键都立刻重绘一次并显示 400ms 反馈, 不受 200ms 刷新限频影响:
 *        加时长 -> 第 7 行显示 "[UP] +1.0s"
 *        减时长 -> 第 7 行显示 "[DN] -1.0s"
 *        切步进 -> 第 7 行显示 "STEP 1.0s"
 *      第 0 行(方向 + 秒数)与第 1 行(TOTAL)始终跟着设定值实时变化。
 *  ★ 第 1 行在空闲时显示陀螺仪实时航向 Y, 运行中显示航向误差 E,
 *    两种情况都能看到当前车头角度。
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
#define MENU_SINGLE_STEP_MS     100u

/* 单段调试: 长按 PE1 改步进, 每长按一次在 0.1s / 1.0s 之间轮换 */
#define MENU_SINGLE_STEP_BIG_MS 1000u

/* ==================== 内部状态 ==================== */

static MenuPage_t s_page = MENU_PAGE_MAIN;
static uint8_t    s_cursor = 0u;        /* 0 = Auto Run, 1 = Single Seg */
static uint8_t    s_need_redraw = 1u;
static uint32_t   s_last_draw = 0u;

static uint8_t    s_single_move = (uint8_t)IR_MOVE_FWD;
static uint16_t   s_single_ms = 2000u;

/* 当前时长步进(ms): 0.1s 或 1.0s, 长按 PE1 切换 */
static uint16_t   s_single_step = MENU_SINGLE_STEP_MS;

/* PE2 长按动作已经触发过(用来区分长按和短按) */
static uint8_t    s_long_fired = 0u;

/* PE0 长按动作已经触发过(单段页: 长按 = 返回选择页) */
static uint8_t    s_long_fired_pe0 = 0u;

/* PE1 长按动作已经触发过(单段页: 长按 = 切换时长步进) */
static uint8_t    s_long_fired_pe1 = 0u;

/* ---- 按键反馈(单段页) ----
 * 加/减时长、换方向这类操作本身只改一个数字, 屏幕上不刷新到就看不出来
 * (绘制限频 200ms, 而且按一下可能刚好落在两帧之间)。
 * 这里记下"刚刚发生了什么", 在高优先级的那次立即重绘里把标记画上,
 * 再保持 MENU_FEEDBACK_MS 后自动消失。 */
#define MENU_FEEDBACK_MS        400u

#define MENU_FB_NONE            0u
#define MENU_FB_TIME_UP         1u
#define MENU_FB_TIME_DOWN       2u
#define MENU_FB_DIR             3u
#define MENU_FB_STEP            4u

static uint8_t    s_feedback = MENU_FB_NONE;
static uint32_t   s_feedback_tick = 0u;

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

static void menu_draw_run_state(const IrInfo_t *info)
{
    char line[22];

    (void)snprintf(line, sizeof(line), "%s YAW:%6.1f",
                   InertialRun_StateName(info->state),
                   (double)jy901s_data.yaw);
    OLED_ShowString(0, 6, line);
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
        /* ★ 空闲时显示的是"界面正在编辑的设定值", 不是状态机快照。
         *   IrInfo_t 里的 move/duration_ms 是"本次跑图的实际值", 空闲时
         *   还停在上一次运行的结果上, 跟着它画的话按键改了也看不出来。
         *   运行中才用快照(那才是真的在下发的值)。 */
        const char *name = (info->running != 0u)
                         ? InertialRun_MoveName(info->move)
                         : InertialRun_MoveName(s_single_move);
        double sec = (info->running != 0u)
                   ? ((double)info->duration_ms / 1000.0)
                   : ((double)s_single_ms / 1000.0);

        (void)snprintf(line, sizeof(line), "SINGLE %s %.1fs", name, sec);
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
    else if (single != 0u)
    {
        /* 空闲: 和上面第 0 行同源, 显示界面设定值 + 陀螺仪实时航向 */
        (void)snprintf(line, sizeof(line), "TOTAL %.1fs Y:%5.1f",
                       (double)s_single_ms / 1000.0,
                       (double)jy901s_data.yaw);
    }
    else
    {
        /* 自动跑图空闲: 误差没有意义, 换成陀螺仪实时航向 */
        (void)snprintf(line, sizeof(line), "T:%u.%us Y:%5.1f",
                       (unsigned)(info->duration_ms / 1000u),
                       (unsigned)((info->duration_ms % 1000u) / 100u),
                       (double)jy901s_data.yaw);
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
        menu_draw_run_state(&info);
        OLED_ShowString(0, 7, "PE0:STOP");
    }
}

/* 单段页第 7 行的按键说明。
 * 反馈期间整行替换成醒目的短标记, 让人确认"这次按键真的生效了":
 *   加时长 -> "[UP] +0.1s"      减时长 -> "[DN] -0.1s"
 *   切步进 -> "STEP 0.1s"       其余时候显示常驻按键说明。
 * 最终设定值本身在第 0 行(方向 + 秒数)和第 1 行(TOTAL)同时更新,
 * 所以这里不需要重复数字, 只负责"这次按键被接收了"的即时反馈。 */
static void menu_draw_single_hint(void)
{
    char line[22];

    switch (s_feedback)
    {
        case MENU_FB_TIME_UP:
            (void)snprintf(line, sizeof(line), "[UP] +%u.%us",
                           (unsigned)(s_single_step / 1000u),
                           (unsigned)((s_single_step % 1000u) / 100u));
            break;

        case MENU_FB_TIME_DOWN:
            (void)snprintf(line, sizeof(line), "[DN] -%u.%us",
                           (unsigned)(s_single_step / 1000u),
                           (unsigned)((s_single_step % 1000u) / 100u));
            break;

        case MENU_FB_STEP:
            (void)snprintf(line, sizeof(line), "STEP %u.%us",
                           (unsigned)(s_single_step / 1000u),
                           (unsigned)((s_single_step % 1000u) / 100u));
            break;

        default:
            (void)snprintf(line, sizeof(line), "1:+ 2:- 0:DIR");
            break;
    }

    OLED_ShowString(0, 7, line);
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
        /* 空闲: 第 6 行显示结束原因, 第 7 行显示按键说明 */
        OLED_ShowString(0, 6, info.result);
    }
    else
    {
        menu_draw_run_state(&info);
    }

    /* 第 0 行已经直接跟着 s_single_move / s_single_ms 变, 换方向时
     * 方向名本身就是反馈, 不再需要额外的 "*" 标记。 */
    menu_draw_single_hint();
}

/* ==================== 按键路由 ==================== */

/* 记一次按键反馈: 立即重绘 + 短时保留标记 */
static void menu_feedback(uint8_t kind)
{
    s_feedback = kind;
    s_feedback_tick = HAL_GetTick();
    s_need_redraw = 1u;
}

/* 到时间就撤掉反馈标记 */
static void menu_feedback_tick(void)
{
    if ((s_feedback != MENU_FB_NONE)
        && ((uint32_t)(HAL_GetTick() - s_feedback_tick) >= MENU_FEEDBACK_MS))
    {
        s_feedback = MENU_FB_NONE;
        s_need_redraw = 1u;
    }
}

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

/* PE0 长按是否刚触发。单段页: PE0 短按 = 换方向, 长按 = 开始/停止。 */
static uint8_t menu_long_pe0(void)
{
    if ((s_long_fired_pe0 == 0u) && (Key_IsDown(KEY_1) != 0u)
        && (Key_GetHoldMs(KEY_1) >= MENU_LONG_PRESS_MS))
    {
        s_long_fired_pe0 = 1u;
        return 1u;
    }
    return 0u;
}

/* 单段页时长调整: 超过上限绕回到下限, 保持可循环加 */
static uint8_t menu_single_add_ms(void)
{
    uint32_t next = (uint32_t)s_single_ms + (uint32_t)s_single_step;

    while (next > (uint32_t)IR_SINGLE_MS_MAX)
    {
        next -= (uint32_t)IR_SINGLE_MS_MAX;
        next += (uint32_t)IR_SINGLE_MS_MIN - 1u;
    }
    if (next < (uint32_t)IR_SINGLE_MS_MIN)
    {
        next = (uint32_t)IR_SINGLE_MS_MIN;
    }

    if ((uint16_t)next == s_single_ms)
    {
        return 0u;
    }

    s_single_ms = (uint16_t)next;
    s_need_redraw = 1u;
    return 1u;
}

/* 单段页时长调整: 减到下限以下就绕到上限, 与加时长对称 */
static uint8_t menu_single_sub_ms(void)
{
    uint32_t next;

    if ((uint32_t)s_single_ms <= (uint32_t)IR_SINGLE_MS_MIN)
    {
        /* 已经在下限附近: 绕到上限 */
        next = (uint32_t)IR_SINGLE_MS_MAX;
    }
    else if ((uint32_t)s_single_ms - (uint32_t)s_single_step
             < (uint32_t)IR_SINGLE_MS_MIN)
    {
        /* 减一个步进会跌破下限: 停在 500ms, 不要跳过它 */
        next = (uint32_t)IR_SINGLE_MS_MIN;
    }
    else
    {
        next = (uint32_t)s_single_ms - (uint32_t)s_single_step;
    }

    if ((uint16_t)next == s_single_ms)
    {
        return 0u;
    }

    s_single_ms = (uint16_t)next;
    s_need_redraw = 1u;
    return 1u;
}

/* PE1 长按是否刚触发。PE1 在单段页承担"换方向", 长按改用"换步进",
 * 所以和 PE2 一样需要自己记一次"已经触发过", 否则按住期间会每圈都算。 */
static uint8_t menu_long_pe1(void)
{
    if ((s_long_fired_pe1 == 0u) && (Key_IsDown(KEY_2) != 0u)
        && (Key_GetHoldMs(KEY_2) >= MENU_LONG_PRESS_MS))
    {
        s_long_fired_pe1 = 1u;
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

    /* PE1 短按(按下即生效): 按当前步进加时长 */
    if (press == KEY_2)
    {
        if (busy == 0u)
        {
            (void)menu_single_add_ms();
            menu_feedback(MENU_FB_TIME_UP);
        }
    }

    /* PE1 长按: 切换时长步进 0.1s <-> 1.0s */
    if (menu_long_pe1() != 0u)
    {
        if (busy == 0u)
        {
            s_single_step = (s_single_step == MENU_SINGLE_STEP_BIG_MS)
                          ? (uint16_t)MENU_SINGLE_STEP_MS
                          : (uint16_t)MENU_SINGLE_STEP_BIG_MS;
            menu_feedback(MENU_FB_STEP);
        }
    }

    if (release == KEY_2)
    {
        s_long_fired_pe1 = 0u;
    }

    /* PE2 短按(按下即生效): 按当前步进减时长 */
    if (press == KEY_3)
    {
        if (busy == 0u)
        {
            (void)menu_single_sub_ms();
            menu_feedback(MENU_FB_TIME_DOWN);
        }
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

    if (release == KEY_3)
    {
        s_long_fired = 0u;
    }

    /* PE0 短按: 换方向(避开长按的开始/停止) */
    if (release == KEY_1)
    {
        if ((s_long_fired_pe0 == 0u) && (busy == 0u))
        {
            s_single_move = (uint8_t)((s_single_move + 1u) % IR_SINGLE_MOVE_COUNT);
            menu_feedback(MENU_FB_DIR);
        }
        s_long_fired_pe0 = 0u;
    }

    /* PE0 长按: 开始 / 停止(空闲时开始单段, 运行中取消) */
    if (menu_long_pe0() != 0u)
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
    s_single_step = MENU_SINGLE_STEP_MS;
    s_long_fired = 0u;
    s_long_fired_pe1 = 0u;
    s_long_fired_pe0 = 0u;
    s_feedback = MENU_FB_NONE;
    s_feedback_tick = 0u;
}

void Menu_Run(void)
{
    uint8_t press = Key_GetPress();
    uint8_t release = Key_GetRelease();
    uint32_t now;

    menu_feedback_tick();

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
