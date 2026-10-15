#ifndef __OLED_H
#define __OLED_H

/* =================================================
 * 0.96寸 OLED (SSD1306, 128x64) 软件I2C驱动
 * =================================================
 * 接线:
 *   OLED SCL -> PD10
 *   OLED SDA -> PD8
 *   OLED VCC -> 3.3V
 *   OLED GND -> GND
 *
 * 引脚已在 gpio.c 中配置为开漏输出+上拉, 无需再配置。
 *
 * 常用调用:
 *   OLED_Init();                       // 上电初始化一次
 *   OLED_Clear();                      // 清屏
 *   OLED_ShowString(0, 0, "Yaw:");     // 第0行第0列显示字符串
 *   OLED_ShowFloat(30, 0, jy901s_data.yaw, 1);  // 显示浮点数(1位小数)
 *   OLED_ShowNum(0, 2, 123, 3);        // 显示整数, 占3位
 *
 * 坐标说明: x为列(0~127), y为行(0~7, 每行8个像素点)
 * =================================================
 */

#include "main.h"

/* 初始化OLED(内部包含软件I2C引脚初始化) */
void OLED_Init(void);

/* 清屏(清空显存 + 刷新到屏幕) */
void OLED_Clear(void);

/* 只清空显存, 不刷新屏幕。
 * 配合 OLED_Refresh() 使用, 比 OLED_Clear() 少一次全屏 I2C 传输,
 * 适合需要频繁重绘的场合(例如菜单)。*/
void OLED_ClearBuffer(void);

/* 刷新显存到屏幕(修改显示内容后调用) */
void OLED_Refresh(void);

/* 在(x列, y行)显示一个字符, size=8为6x8小字体 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr);

/* 在(x列, y行)显示字符串 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);

/* 在(x列, y行)显示整数, len为位数(不足前面补0) */
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len);

/* 在(x列, y行)显示浮点数, dec为小数位数 */
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t dec);

#endif /* __OLED_H */
