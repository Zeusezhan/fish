/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* ==================== 航向姿态环(main.c 实现) ====================
 *
 * TIM3 中断每 20ms 读一次陀螺仪并算一次 PI-D, 结果放在 g_yaw_output。
 * 下面这些函数是状态机(InertialRun / Menu)跟航向环打交道的唯一入口。
 *
 * ★ 想让姿态闭环生效, 必须同时满足两条:
 *   1. main() 里调用了 HAL_TIM_Base_Start_IT(&htim3);
 *   2. 状态机调用了 YawLoop_Enable()。
 *   少任何一条都会退化成开环 —— 原来的代码就是第 1 条被注释掉了,
 *   所以 InertialRun 里"保持航向"其实一直没生效。
 */

/* 使能航向环: target_deg 目标航向(度), out_max_rpm 自转修正量限幅(RPM) */
void  YawLoop_Enable(float target_deg, float out_max_rpm);

/* 关闭航向环(输出清零, IMU 仍在中断里继续读) */
void  YawLoop_Disable(void);

/* 陀螺仪是否还在正常出数: 1 = 正常, 0 = 掉线(运行中必须停车) */
int8_t YawLoop_IsImuAlive(void);

/* 当前航向误差(度), 仅用于显示 */
float YawLoop_GetError(void);

/* TIM3 中断里算出的自转修正量(RPM), 主循环取它当麦轮的 omega */
extern volatile float   g_yaw_output;
extern volatile uint8_t g_yaw_loop_enable;
extern volatile float   g_yaw_target;
extern volatile float   g_yaw_kp;
extern volatile float   g_yaw_ki;
extern volatile float   g_yaw_kd;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STEP_TX_Pin GPIO_PIN_9
#define STEP_TX_GPIO_Port GPIOA
#define STEP_RX_Pin GPIO_PIN_10
#define STEP_RX_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
