#ifndef __WIRELESS_UART_H
#define __WIRELESS_UART_H

/* ============================================================
 * 无线串口 / VOFA+ 波形上报
 *
 * 【用途】用 USART2(PA2=TX, PA3=RX, 9600 8N1)把航向闭环的
 *         目标值和当前值发出去, 在 VOFA+ 上位机里实时看曲线;
 *         同时用同一串口接收 PC 发来的 "kp,kd\n" 在线调参命令。
 *
 * 【接线】USART2 是 3.3V TTL 电平, 不能直接接电脑 USB!
 *         接无线模块(如 HC-05 / ESP-01)或 CH340 USB-TTL 小板:
 *             模块 RXD  <-  PA2 (单片机 TX)
 *             模块 TXD  ->  PA3 (单片机 RX, 接收 PC 发来的 PID 参数)
 *             模块 GND  <-> GND
 *         无线模块的波特率必须也是 9600, 否则收到的全是乱码。
 *
 * 【VOFA+ 设置】数据格式选 "FireWater", 波特率选 9600。
 *         FireWater 是纯文本协议: 一帧就是一串用逗号分成的数字,
 *         以 '\n' 结尾。本文件每帧发 4 个通道:
 *             第 0 通道 = 目标航向角 (度)
 *             第 1 通道 = 当前航向角 (度)
 *             第 2 通道 = 当前 Kp (回显, 用来确认参数已生效)
 *             第 3 通道 = 当前 Kd (回显, 用来确认参数已生效)
 *         两个角度用的是同一套坐标, 叠在一起就能看出"还差多少"。
 *
 * 【在线调 PD】两种方式:
 *   1. 数据发送框手打 "kp,kd\n", 例如 6.0,0.4(两个一起设);
 *   2. 用 VOFA+ 控件: 加两个"滑动条", 分别绑定命令 "P%f\n" 和
 *      "D%f\n", 拖动滑块即自动发送, 不用手动发。
 *   芯片收到后实时更新 g_yaw_kp / g_yaw_kd, 波形立即变化就是生效。
 * ============================================================ */

#include "main.h"

/* ---- 上报周期(ms) ----
 * 别设太小: 每帧约 15 字节在 9600 下要传约 16ms, 周期太短会撞上
 * 上一次没发完 -> HAL_UART_Transmit_DMA 返回 HAL_BUSY -> 波形丢帧。
 * 40ms(25Hz) 时占用率约 40%, 比较稳; 想更快可把波特率提到 115200。 */
#define WIRELESS_REPORT_PERIOD_MS   (40u)

/**
  * @brief  初始化无线串口上报模块
  * @note   在 main() 的初始化段调用一次(须在 MX_USART2_UART_Init() 之后)。
  *         只复位内部计时并启动下行接收, 串口本身已由 MX_USART2_UART_Init() 配好。
  */
void Wireless_Init(void);

/**
  * @brief  启动下行接收(PC -> 芯片): 用中断逐字节接收 VOFA+ 发来的
  *         "kp,kd\n" 命令, 解析成功后实时更新 g_yaw_kp / g_yaw_kd。
  * @note   Wireless_Init() 内部已自动调用, 一般不需要手动调。
  *         接收引脚 PA3 与 USART2 中断由 CubeMX 配置, 无需额外初始化。
  */
void Wireless_StartReceive(void);

/**
  * @brief  上报"目标航向 + 当前航向"(供定时器中断回调调用)
  * @param  target_yaw_deg   目标航向角(度)
  * @param  current_yaw_deg  当前陀螺仪航向角(度)
  * @note   ★ 这个函数是"非阻塞"的, 内部用 DMA 把数据丢出去就返回,
  *           不会占用 CPU 等待, 所以可以放心在中断里调用。
  *         内部带周期限频, 比 WIRELESS_REPORT_PERIOD_MS 更密的调用
  *         会被自动跳过, 所以直接无脑调用即可。
  *         帧格式: "目标,当前,Kp,Kd\n", Kp/Kd 是当前实际生效的 PD 参数。
  */
void Wireless_SendYaw(float target_yaw_deg, float current_yaw_deg);

/**
  * @brief  上报航向误差(度) = 目标角 - 当前角
  * @param  target_yaw_deg   目标航向角(度)
  * @param  current_yaw_deg  当前陀螺仪航向角(度)
  * @note   内部与 Wireless_SendYaw 相同: 每帧发 4 个通道
  *         (目标/当前/Kp/Kd), 两角之差在 VOFA+ 里用两根曲线的差
  *         就能看出来。
  */
void Wireless_SendYawError(float target_yaw_deg, float current_yaw_deg);

/**
  * @brief  按当前实际状态上报(目标航向 + 当前航向 + Kp + Kd)
  * @note   四个值都取自全局变量:
  *             目标  = g_yaw_target
  *             当前  = jy901s_data.yaw_continuous
  *             Kp    = g_yaw_kp
  *             Kd    = g_yaw_kd
  *         已经算好帧内容并做了周期限频, 可以贴到主循环里反复调用,
  *         不会刷爆串口。
  *         ★ 上报不受闭环开关影响: 即使 g_yaw_loop_enable = 0,
  *           只要没到达上传周期就会照常发, 方便上电先验证 IMU 和接线。
  *         由于回传了 Kp/Kd, 在 VOFA+ 里拖动滑块后能立刻看到
  *         对应通道的数值变化 —— 这就是"参数已发送成功"的确认。
  */
void Wireless_ReportYaw(void);

#endif /* __WIRELESS_UART_H */
