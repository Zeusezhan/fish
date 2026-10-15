#ifndef __MENU_H
#define __MENU_H

/*
 * ============================================================
 * OLED 显示菜单
 *
 * 【页面结构】
 *   第一页  选择界面 MENU_PAGE_MAIN
 *             ├─ Parameters   -> 参数界面(显示陀螺仪数据)
 *             └─ Mode Select  -> 模式选择(暂未实现, 留空)
 *   子界面   参数界面 MENU_PAGE_PARAM
 *            模式界面 MENU_PAGE_MODE
 *
 * 【按键操作】按键定义见 key.h
 *   第一页(选择界面):
 *       PE1 (KEY_2) = 选择下一个        光标往下移, 到底后回到第一个
 *       PE0 (KEY_1) = 确定              进入光标所指的界面
 *   参数界面:
 *       PE2 (KEY_3) = 返回              回到第一页
 *   模式界面(启停开关):
 *       PE0 (KEY_1) = 启动 / 停止       按一次启动航向闭环, 再按一次停止并停车
 *       PE2 (KEY_3) = 返回              回到第一页
 *   第一页已经是最顶层, 所以按 PE2 不做任何事。
 *
 * 【模式页怎么工作】
 *   PE0 不直接开/关闭环, 而是翻转 g_mode_run_request 这个请求标志,
 *   真正的启动/停车由 main.c 的 Heading_UpdateRunState() 在主循环里
 *   执行(因为电机命令内部有 HAL_Delay, 不能在中断里调用)。
 *   这样菜单只管界面和按键, 不直接碰电机逻辑, 分层更清楚。
 *
 * 【使用方法】
 *   1. 上电时调用一次 Menu_Init()
 *   2. 在 while(1) 主循环里反复调用 Menu_Run()
 *      前提是主循环里已经调用了 Key_Scan() 做按键扫描
 *
 * 【注意】
 *   OLED 的 6x8 字库只有 ASCII 字符, 显示不了中文,
 *   所以菜单文字一律用英文(字库外的字符会被显示成空格)。
 * ============================================================
 */

#include "main.h"

/* ---------------- 用户可配置参数 ---------------- */

/* 显示刷新周期(ms): 参数界面要实时看数据, 太快会占用太多 I2C 时间 */
#define MENU_REFRESH_MS     100

/* ------------------------------------------------ */

/* 菜单页面编号 */
typedef enum
{
    MENU_PAGE_MAIN = 0,     /* 第一页: 选择界面 */
    MENU_PAGE_PARAM,        /* 参数界面: 陀螺仪数据 */
    MENU_PAGE_MODE          /* 模式选择: 暂未实现 */
} MenuPage_t;

/* 第一页里可选的菜单项 */
#define MENU_ITEM_PARAM     0   /* 参数界面 */
#define MENU_ITEM_MODE      1   /* 模式选择 */
#define MENU_ITEM_NUM       2   /* 菜单项总数 */

/* 【初始化】上电调用一次, 把页面复位到第一页 */
void Menu_Init(void);

/* 【菜单主函数】放到 while(1) 主循环里反复调用。
 * 内部同时处理两件事:
 *   1. 读取按键事件, 切换页面/移动光标
 *   2. 把当前页面画到 OLED(按 MENU_REFRESH_MS 限频)
 * 前提: 主循环里已经调用了 Key_Scan()。 */
void Menu_Run(void);

/* 【查询当前页面】需要根据界面做不同动作时可以用它判断 */
MenuPage_t Menu_GetPage(void);

#endif /* __MENU_H */
