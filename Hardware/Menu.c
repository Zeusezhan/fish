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

/* 闭环的"按键启动请求"标志, 定义在 main.c 的 USER CODE BEGIN PV 里。
 * 这里只是把它翻转, 真正的启动/停车由 Heading_UpdateRunState()
 * 在主循环里执行(见 main.c), 避免在本模块里直接操作电机。 */
extern volatile uint8_t g_mode_run_request;

/* 当前页面 */
static MenuPage_t s_page = MENU_PAGE_MAIN;

/* 第一页的光标位置(当前选中的菜单项) */
static uint8_t s_cursor = MENU_ITEM_PARAM;

/* 显示刷新计时 */
static uint32_t s_last_draw_tick = 0;

/* 需要立即重绘的标志: 切换页面/移动光标后置 1, 让界面立刻响应按键 */
static uint8_t s_need_redraw = 1;

/* 第一页的菜单项文字
 * (OLED 6x8 字库只有 ASCII, 不能用中文) */
static const char *const s_main_item[MENU_ITEM_NUM] =
{
    "Parameters",       /* MENU_ITEM_PARAM */
    "Mode Select",      /* MENU_ITEM_MODE */
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
        row = (uint8_t)(2 + i * 2);     /* 第2行、第4行 */

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
  * @brief  绘制模式选择界面
  * @note   这一页现在是一个"启停开关":
  *           PE0 按一次启动航向闭环, 再按一次停止并停车。
  */
static void menu_draw_mode(void)
{
    OLED_ShowString(0, 0, "--- Mode Select ---");

    if (g_mode_run_request)
    {
        OLED_ShowString(0, 2, "Status: RUN");
        OLED_ShowString(0, 4, "PE0: STOP");
    }
    else
    {
        OLED_ShowString(0, 2, "Status: STOP");
        OLED_ShowString(0, 4, "PE0: START");
    }

    /* 顺手显示实时航向, 按 PE0 时能直接看到转向是否正常 */
    OLED_ShowString(0, 5, "Yaw:");
    OLED_ShowFloat(30, 5, jy901s_data.yaw, 1);

    OLED_ShowString(0, 7, "PE2:Back");
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
                s_page = MENU_PAGE_MODE;
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
    menu_handle_key(Key_GetPress());

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

        case MENU_PAGE_MAIN:
        default:
            menu_draw_main();
            break;
    }

    OLED_Refresh();
}

/**
  * @brief  查询当前所在的页面
  * @retval 当前页面编号(MENU_PAGE_MAIN / MENU_PAGE_PARAM / MENU_PAGE_MODE)
  * @note   以后跑图逻辑需要知道"现在是不是在参数页"时可以用到。
  */
MenuPage_t Menu_GetPage(void)
{
    return s_page;
}
