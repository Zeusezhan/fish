/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "jy901s.h"
#include "OLED.h"
#include "key.h"
#include "Menu.h"
#include "wireless_uart.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ==================== 航向角 PD 闭环参数 ==================== */

/* PD 参数(上电默认值, 运行中可用 VOFA+ 实时修改):
 * 整定时先只调 Kp —— 从小往大加, 加到"转得够快但开始有点晃";
 * 然后再加 Kd 把晃动压下去。Kd 用的是陀螺仪角速度, 不需要自己差分。
 * 运行时值保存在 g_yaw_kp / g_yaw_kd(见 USER CODE PV), 通过无线串口
 * (USART2)发 "kp,kd\n" 即可修改, 例如在 VOFA+ 发送框里发 "6.0,0.4"。 */
#define YAW_PID_KP_DEFAULT      (4.0f)      /* 比例: 航向误差 1 度给多少转速 */
#define YAW_PID_KD_DEFAULT      (0.35f)     /* 微分: 阻尼项, 压住过冲 */

/* PD 输出限幅(当前只用于观察输出, 不驱动底盘)。 */
#define YAW_PID_DRIVE_OUT_MAX   (60.0f)

/* 航向死区: 误差小于该角度时输出为零 */
#define YAW_PID_TOLERANCE       (2.0f)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ==================== 航向 PD 监测状态 ====================
 * TIM3 中断每 20ms 计算一次 PD 输出, 目前只用于无线串口观察,
 * 不向底盘发送电机命令。
 * ======================================================== */

/* PE0 在 Mode Select 页启停 PD 计算；上电默认关闭。 */
volatile uint8_t g_yaw_loop_enable = 0;

/* Menu 模块通过此标志请求开始/停止 PD 计算。 */
volatile uint8_t g_mode_run_request = 0;

/* 目标航向(度), 相对 JY901S_ZeroYaw() 的清零点 */
volatile float   g_yaw_target = 0.0f;

/* 可在线整定的 PD 参数: VOFA+ 发 "kp,kd\n" 即可实时修改。
 * 初值取上面的 YAW_PID_KP_DEFAULT / YAW_PID_KD_DEFAULT。
 * 中断里直接读, 主循环/接收回调里写, 都是单次 32 位读写, 安全。 */
volatile float   g_yaw_kp = YAW_PID_KP_DEFAULT;
volatile float   g_yaw_kd = YAW_PID_KD_DEFAULT;

/* 中断算出的 PD 输出(仅供观察, 当前不驱动电机)。 */
volatile float   g_yaw_output = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Heading_Task(void);                                        /* 主循环调用: 更新监测与串口上报 */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* OLED only needs the GPIO software-I2C pins, so initialize it before
   * the other peripherals and show a boot message as early as possible. */
  OLED_Init();
  OLED_ShowString(0, 0, "BOOTING...");
  OLED_Refresh();

  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* 初始化 JY901S 姿态模块(I2C 接口, PB6=SCL, PB7=SDA) */
  JY901S_Init();

  OLED_ShowString(0, 0, "JY901S init...");
  OLED_Refresh();

  /* 等模块上电稳定, 然后主动读一帧, 确认它能正常应答 */
  HAL_Delay(500);
  (void)JY901S_ReadOnce();

  if (!jy901s_data.ready)
  {
    /* 读不到数据: 检查 PB6/PB7 接线与 4.7k 上拉电阻 */
    OLED_ShowString(0, 2, "No IMU data!");
    OLED_ShowString(0, 4, "Chk wire+pullup");
    OLED_Refresh();
    HAL_Delay(1500);
  }
  else
  {
    /* 模块正常: 把回传率从出厂 10Hz 提到 50Hz。
     * 必须提 —— 出厂 10Hz 是 100ms 才更新一次角度, 而定时器 20ms 就跑一次
     * PD, 中间几次都在重复算同一个旧数据, 响应会明显滞后、容易振荡。
     * 该设置写进模块后断电不丢失, 只需生效一次。 */
    JY901S_SetOutputRate(JY901S_RRATE_50HZ);
  }

  /* 起跑前把当前朝向清零, 之后读到的 Yaw 都是相对这个方向 */
  JY901S_ZeroYaw();

  /* 初始化按键(PE0=KEY1, PE1=KEY2, PE2=KEY3, 上拉输入, 按下为低电平)
   * 引脚已由 MX_GPIO_Init() 配置好, 这里只复位内部状态 */
  Key_Init();

  /* 初始化 OLED 菜单, 从第一页(选择界面)开始 */
  Menu_Init();

  /* 初始化无线串口上报(VOFA+ 波形), 只复位内部计时,
   * 串口本身已由 MX_USART2_UART_Init() 配好 */
  Wireless_Init();

  /* 启动 TIM3 更新中断(20ms 一次), 航向角 PD 运算在中断回调里做。
   * CubeMX 不会自动生成这一句, 不加的话中断永远不会进来。
   * 调试页有停车状态锁, 停车期间不会再下发速度命令, 可以和闭环共存。 */
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* I2C 轮询读取 JY901S 姿态数据(内部按 20ms 间隔控制频率) */
    JY901S_Poll();

    /* 处理菜单的 PD 监测启停请求, 并上报陀螺仪/PD 数据 */
    Heading_Task();

    /* 按键轮询检测(内部按 10ms 间隔采样并消抖) */
    Key_Scan();

    /* OLED 菜单: 内部处理按键(切换页面/移动光标)并绘制当前页面,
     * 显示按 100ms 限频刷新。显示内容全部由 Menu.c 负责。 */
    Menu_Run();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  定时器中断回调: 航向角 PD 运算
  * @param  htim  触发中断的定时器句柄
  * @note   TIM3 配置成 20ms 一次, 所以这个函数每 20ms 进来一次。
  *
  *         运算流程:
  *           误差 err = 目标航向 - 当前航向
  *           输出 u   = Kp * err - Kd * gz      (gz 是陀螺仪 Z 轴角速度)
  *           限幅后存进 g_yaw_output, 供串口监测, 不驱动底盘
  *
  *         中断内只做数值运算, 不进行阻塞式外设操作。
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    float err;
    float u;
    float out_max;
    float out_min;

    /* 这个回调是所有定时器共用的, 先确认是 TIM3 触发 */
    if (htim->Instance != TIM3)
    {
        return;
    }

    /* 闭环没启动就什么都不做, 保证中断尽可能轻 */
    if (!g_yaw_loop_enable)
    {
        return;
    }

    /* ---- 1. 航向误差 ----
     * 用 yaw_continuous(连续角度), 这样转过 ±180 也不会跳变 */
    err = g_yaw_target - jy901s_data.yaw_continuous;

    /* ---- 2. 航向死区 ----
     * 误差落在容差内时输出归零, 避免小误差造成输出抖动 */
    if ((err <= YAW_PID_TOLERANCE) && (err >= -YAW_PID_TOLERANCE))
    {
        g_yaw_output = 0.0f;
    }
    else
    {
        /* ---- 3. PD 运算 ----
         * 比例项: 误差越大转得越快
         * 微分项: 直接用陀螺仪角速度做阻尼, 不用自己差分, 没有噪声放大。
         *         gz > 0 表示模块检测到左转, 减去它作为阻尼项 */
        u = (g_yaw_kp * err) - (g_yaw_kd * jy901s_data.gz);

        /* 限制观察用输出的幅值; 当前不将该量下发给电机 */
        out_max = YAW_PID_DRIVE_OUT_MAX;
        out_min = 0.0f;

        /* 按误差方向分段限幅: 输出永远指向目标那一侧,
         * 即使微分项再大也不会把方向顶反 */
        if (err > 0.0f)
        {
            if (u > out_max)
            {
                u = out_max;
            }
            else if (u < out_min)
            {
                u = out_min;
            }
        }
        else
        {
            if (u < -out_max)
            {
                u = -out_max;
            }
            else if (u > -out_min)
            {
                u = -out_min;
            }
        }

        g_yaw_output = u;
    }

}

void Heading_Task(void)
{
    static uint8_t s_applied_request = 0;

    if (g_mode_run_request != s_applied_request)
    {
        s_applied_request = g_mode_run_request;
        g_yaw_output = 0.0f;
        if (s_applied_request)
        {
            g_yaw_target = jy901s_data.yaw_continuous;
            g_yaw_loop_enable = 1;
        }
        else
        {
            g_yaw_loop_enable = 0;
        }
    }

    Wireless_ReportYaw();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
