#ifndef __MENU_H
#define __MENU_H

/*
 * ============================================================
 * OLED 界面 + 按键路由
 *
 * 【本文件只做两件事】
 *   1. 按键路由: 根据当前页面把按键翻译成"开始 / 停止 / 调参数 / 换页"
 *   2. 画面绘制: 从 InertialRun_GetInfo() 取快照画到 OLED
 *
 * ★ 这里不包含任何跑图逻辑 —— 状态机、计时、停车、编码器确认
 *   全部在 Function/InertialRun.c 里。本文件只负责"开始/停止"这两个
 *   动作的转发。
 *
 * 【页面结构】
 *
 *   MAIN(选择页)
 *     ├─ Auto Run    -> AUTO 页: 自动跑 5 段
 *     └─ Single Seg  -> SINGLE 页: 单段调试
 *
 * 【按键】
 *   MAIN    : PE1 = 下移光标      PE0 = 进入
 *   AUTO    : PE0 = 开始/停止     PE2 长按 = 返回(空闲时)
 *   SINGLE  : PE0 = 开始/停止     PE1 = 换方向(空闲时)
 *             PE2 短按 = 时长+0.5s(空闲时)   PE2 长按 = 返回(空闲时)
 *
 * 【注意】OLED 的 6x8 字库只有 ASCII, 所以界面文字一律英文。
 *         一行最多 21 个字符。
 * ============================================================
 */

#include <stdint.h>

typedef enum
{
    MENU_PAGE_MAIN = 0,     /* 选择页 */
    MENU_PAGE_AUTO,         /* 自动跑图 */
    MENU_PAGE_SINGLE        /* 单段调试 */
} MenuPage_t;

/* 上电调用一次 */
void Menu_Init(void);

/* 主循环里反复调用(前提是已经调用过 Key_Scan)。
 * 内部同时处理按键和绘制, 绘制按周期限频。 */
void Menu_Run(void);

#endif /* __MENU_H */
