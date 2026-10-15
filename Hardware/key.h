#ifndef __KEY_H
#define __KEY_H

/*
 * ============================================================
 * 按键驱动 (3 个按键, 轮询方式)
 *
 * 【硬件接线】
 *   KEY1 -> PE0
 *   KEY2 -> PE1
 *   KEY3 -> PE2
 *   三个引脚都配置成"上拉输入", 按键另一端接 GND:
 *       松开 -> 引脚被内部上拉为高电平(1)
 *       按下 -> 引脚被按键拉到低电平(0)
 *   (引脚配置由 CubeMX 生成的 MX_GPIO_Init() 完成, 本文件不用再动 GPIO)
 *
 * 【使用方法】
 *   1. 上电时调用一次 Key_Init()
 *   2. 在 while(1) 主循环里反复调用 Key_Scan()  <- 轮询检测就靠它
 *   3. 用下面几个函数取状态:
 *        Key_GetPress()   取"刚按下"事件(读一次就清除, 适合做触发动作)
 *        Key_GetRelease() 取"刚松开"事件
 *        Key_IsDown()     判断某个键当前是不是按住
 *        Key_GetHoldMs()  已经按住多少毫秒(做长按判断用)
 *
 * 【消抖原理】
 *   Key_Scan() 内部每 KEY_SCAN_INTERVAL_MS 毫秒采样一次,
 *   连续 KEY_DEBOUNCE_CNT 次都读到相同电平, 才认为状态真的变了。
 *   机械按键的抖动通常只有几毫秒, 这样就滤掉了。
 * ============================================================
 */

#include "main.h"

/* ---------------- 用户可配置参数 ---------------- */

/* 按键编号(传给 Key_IsDown / Key_GetHoldMs 用) */
#define KEY_1               0
#define KEY_2               1
#define KEY_3               2

/* 按键个数 */
#define KEY_NUM             3

/* 表示"没有按键"的返回值, 用于 Key_GetPress / Key_GetRelease */
#define KEY_NONE            0xFF

/* 扫描周期(ms): Key_Scan() 内部按这个间隔采样一次 */
#define KEY_SCAN_INTERVAL_MS 10

/* 消抖次数: 连续这么多次采样电平一致才认为状态稳定
 * 10ms x 3 = 30ms, 对机械按键足够 */
#define KEY_DEBOUNCE_CNT    3

/* ------------------------------------------------ */

/* 【初始化】上电调用一次, 清零内部状态。
 * 引脚由 MX_GPIO_Init() 配置好, 这里不用重复配置 GPIO。 */
void Key_Init(void);

/* 【轮询扫描】放在 while(1) 主循环里反复调用, 内部自动按 10ms 限频,
 * 调用频率再高也不会影响消抖效果。 */
void Key_Scan(void);

/* 【取按下事件】返回刚被按下的键号(KEY_1/KEY_2/KEY_3),
 * 没有新按下时返回 KEY_NONE。
 * 事件读走即清除, 所以按住不放只会在按下那一刻被读到一次。
 * 一次只能取一个, 想取完所有事件可以这样写:
 *     uint8_t k;
 *     while ((k = Key_GetPress()) != KEY_NONE) { ... } */
uint8_t Key_GetPress(void);

/* 【取松开事件】返回刚被松开的键号, 没有则返回 KEY_NONE。读走即清除 */
uint8_t Key_GetRelease(void);

/* 【查询按住状态】key 传 KEY_1 / KEY_2 / KEY_3。
 * 返回 1 = 正按住, 0 = 已松开(或键号非法)。 */
uint8_t Key_IsDown(uint8_t key);

/* 【查询按住时长】key 传 KEY_1 / KEY_2 / KEY_3。
 * 返回已经按住的毫秒数; 没按住时返回 0。做长按判断用:
 *     if (Key_IsDown(KEY_1) && Key_GetHoldMs(KEY_1) > 1000) { ... 长按 1 秒 ... } */
uint32_t Key_GetHoldMs(uint8_t key);

#endif /* __KEY_H */
