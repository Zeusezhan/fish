#ifndef __MENU_H
#define __MENU_H

/*
 * ============================================================
 * OLED 显示菜单
 *
 * 【页面结构】
 *   第一页  选择界面 MENU_PAGE_MAIN
 *             ├─ Parameters   -> 参数界面(显示陀螺仪数据)
 *             ├─ Mode Select  -> PD 监测
 *             └─ Debug        -> 定时直行与底盘位置计数
 *   子界面   参数界面 MENU_PAGE_PARAM
 *            模式界面 MENU_PAGE_MODE
 *   调试界面 MENU_PAGE_DEBUG
 *
 * 【按键操作】按键定义见 key.h
 *   第一页(选择界面):
 *       PE1 (KEY_2) = 选择下一个        光标往下移, 到底后回到第一个
 *       PE0 (KEY_1) = 确定              进入光标所指的界面
 *   参数界面:
 *       PE2 (KEY_3) = 返回              回到第一页
 *   模式界面(PD 监测):
 *       PE0 (KEY_1) = 启动 / 停止       只启停航向 PD 计算, 不控制底盘电机
 *       PE2 (KEY_3) = 返回              回到第一页
 *   调试界面: 停车时 PE0 短按循环选择前进/左移/右移/后退, 长按启动;
 *              运行中 PE0 立即停车。PE1/PE2 短按调秒, 长按清零/返回。
 *              启动时若陀螺仪已就绪, 自动把当前航向设为目标,
 *              四种方向都用 g_yaw_output 做航向保持;
 *              陀螺仪未就绪时只发原始命令并在屏幕上提示。
 *              清零后会读回验证, 成功后节点号递增; 秒数范围为 1~99 秒。
 *
 * 【模式页怎么工作】
 *   PE0 翻转 g_mode_run_request, main.c 根据请求启停 PD 计算。
 *   当前该输出只用于观察/串口上报, 不下发电机命令。
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
    MENU_PAGE_MODE,         /* PD 监测与启停 */
    MENU_PAGE_DEBUG         /* 定时直行与底盘位置计数 */
} MenuPage_t;

/* 第一页里可选的菜单项 */
#define MENU_ITEM_PARAM     0   /* 参数界面 */
#define MENU_ITEM_MODE      1   /* 模式选择 */
#define MENU_ITEM_DEBUG     2   /* 底盘调试 */
#define MENU_ITEM_NUM       3   /* 菜单项总数 */

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
