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
#include "function.h"
#include "InertialRun.h"
#include "wireless_uart.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ==================== 航向环(PI-D)参数 ====================
 *
 *   err = 目标航向 - 当前航向
 *   u   = Kp*err + Ki*∫err*dt - Kd*gz        (gz = 陀螺仪 Z 轴角速度)
 *
 * 微分项直接用陀螺仪角速度做阻尼, 不自己差分航向角, 所以不会放大噪声。
 * 积分项用来消掉麦轮四轮摩擦/辊子接触不对称造成的**稳态偏航** ——
 * 纯 PD 只能靠"留一点航向误差"来对抗它, 表现就是横移时车头慢慢歪
 * 过去而且回不来。
 *
 * 【整定顺序】
 *   1. 先只调 Kp: 从小往大加, 加到"修得够快但开始有点晃";
 *   2. 再加 Kd 把晃动压下去;
 *   3. 最后加一点点 Ki 消掉残余的稳态偏航。
 */
#define YAW_KP_DEFAULT          (9.0f)     /* 比例: 误差 1 度给多少 RPM */
#define YAW_KI_DEFAULT          (0.0f)     /* 积分: 单位 RPM/(度*秒) */
#define YAW_KD_DEFAULT          (0.0f)     /* 微分: 阻尼, 作用于 gz */

/* 控制周期(秒), 必须与 TIM3 的 20ms 一致 */
#define YAW_DT_S                (0.02f)

/* 死区(度): 误差小于它时不再积分, 只留比例+阻尼。
 * ★ 别设大 —— 2 度死区在横移时足以让车头越走越歪。 */
#define YAW_DEADBAND_DEG        (0.20f)

/* 积分项输出限幅(RPM) 和 清积分门限(度) */
#define YAW_INT_MAX_RPM         (8.0f)
#define YAW_INT_RESET_DEG       (8.0f)

/* 输出限幅(RPM)。InertialRun 会通过 YawLoop_Enable 覆盖它。 */
#define YAW_OUT_MAX_DEFAULT     (30.0f)

/* IMU 掉线判据(ms): 超过这么久没有新的有效帧就判定掉线, 必须停车。
 * 50RPM 下 200ms 约跑 1.7cm, 再长就纯靠惯性了。 */
#define YAW_IMU_TIMEOUT_MS      (200u)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ==================== 航向环状态 ====================
 *
 * 【数据流】
 *   TIM3 中断(20ms):  JY901S_ReadIsr() -> PI-D -> g_yaw_output
 *   主循环:           取 g_yaw_output 当 omega, 交给 Car_Mecanum()
 *
 * 中断里"只算不发": Car_Mecanum() 只是塞队列, 但航向修正不需要
 * 20ms 级的下发精度, 让它留在主循环里更简单。
 *
 * g_yaw_output 是 32 位 float, Cortex-M4 上单次写入是原子的,
 * 主循环直接读不会读到"写了一半"的值, 所以不需要临界区。
 * ==================================================== */

/* 1 = 中断里每拍计算并更新 g_yaw_output */
volatile uint8_t g_yaw_loop_enable = 0u;

/* 目标航向(度), 相对 JY901S_ZeroYaw() 的清零点 */
volatile float   g_yaw_target = 0.0f;

/* 可在线整定的参数(无线串口可以改) */
volatile float   g_yaw_kp = YAW_KP_DEFAULT;
volatile float   g_yaw_ki = YAW_KI_DEFAULT;
volatile float   g_yaw_kd = YAW_KD_DEFAULT;

/* 中断算出的航向修正量(RPM) */
volatile float   g_yaw_output = 0.0f;

/* 输出限幅(RPM), 由 YawLoop_Enable() 设定 */
volatile float   g_yaw_out_max = YAW_OUT_MAX_DEFAULT;

/* 积分器(只在中断里累加) */
static volatile float s_yaw_integral = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void  YawLoop_Enable(float target_deg, float out_max_rpm);
void  YawLoop_Disable(void);
int8_t YawLoop_IsImuAlive(void);
float YawLoop_GetError(void);

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

  /* ★ SysTick 提到最高优先级。
   *
   * TIM3 的航向环中断(优先级 5)会抢占 SysTick(默认 15)。中断里读一次
   * I2C 要 1.5ms 左右, 最长到 4.5ms, 而 SysTick 周期只有 1ms ——
   * 一次读取就能吃掉 1~4 个 tick。SysTick 是单一位挂起标志, 被屏蔽期间
   * 到期的多个周期只会补一次, HAL_GetTick() 就越跑越慢。
   *
   * 本项目所有分段时长、超时判据都基于 HAL_GetTick(), 时基慢 10% 就意味着
   * "3.3 秒的一段"实际跑 3.6 秒, 跑图必然走位。 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  OLED_Init();
  OLED_ShowString(0, 0, "BOOTING...");
  OLED_Refresh();

  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* MX_USART3_UART_Init() 不再调用: USART3 原本给起重轴(Crane)用, */

  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* ---- 陀螺仪上电初始化(全部在 TIM3 启动之前完成) ---- */
  JY901S_Init();

  OLED_ShowString(0, 0, "IMU INIT...");
  OLED_Refresh();

  HAL_Delay(500);
  (void)JY901S_ReadIsr();     /* 此刻还没有中断, 直接读一次探活 */

  if (jy901s_data.ready == 0u)
  {
    OLED_ShowString(0, 2, "NO IMU DATA");
    OLED_ShowString(0, 4, "CHECK PB6/PB7");
    OLED_ShowString(0, 5, "ADD 4.7k PULLUP");
    OLED_Refresh();
    HAL_Delay(1500);
  }
  else
  {
    /* 回传率设 100Hz(10ms 一帧), 而闭环每 20ms 读一次,
     * 这样每次读到的都是最近 10ms 内的新数据。
     * 该设置保存在模块里断电不丢, 这里每次上电重设一次以防万一。 */
    JY901S_SetOutputRate(JY901S_RRATE_100HZ);
  }

  /* 把当前朝向记为 0 度基准。之后的 Yaw 都是相对这个方向的。 */
  JY901S_ZeroYaw();

  Key_Init();
  InertialRun_Init();
  Menu_Init();

  /* 无线串口: 只用于上位机看波形和在线调 Kp/Ki/Kd, 不参与控制。
   * 内部会启动 USART2 的接收中断。 */
  Wireless_Init();


  /* 打开 FPU 自动/惰性压栈。中断里做浮点运算, 主循环也在做浮点解析,
   * 不打开的话中断抢占主循环时可能破坏 s0-s15 寄存器。 */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  FPU->FPCCR |= FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk;
#endif

  /* ★ 这一行是整个姿态闭环的总开关。
   * 启动后 IMU 只由 TIM3 中断读, 主循环不能再碰 I2C。 */
  (void)HAL_TIM_Base_Start_IT(&htim3);

  /* 上电即启用航向闭环, 以当前朝向作为目标, 无需等待状态机打开。 */
  YawLoop_Enable(jy901s_data.yaw_continuous, YAW_OUT_MAX_DEFAULT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* 1. 按键扫描(内部 10ms 限频, 3 次一致才认) */
    Key_Scan();

    /* 2. 跑图状态机: 计时、下发、停车、编码器确认全在里面 */
    InertialRun_Task();

    /* 3. 推进底盘发送队列。
     *    放在菜单绘制之前 —— 菜单刷一次 OLED 要十几毫秒,
     *    放后面会让电机指令出现不必要的延迟。 */
    Car_TxTask();

    /* 4. 界面(按键路由 + 绘制) */
    Menu_Run();

    Wireless_ReportYaw();


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

/* ==================== 航向环对外接口 ====================
 *
 * 状态机(InertialRun)只通过这四个函数跟航向环打交道。
 * 不要去直接写 g_yaw_loop_enable / g_yaw_target —— 那样容易出现
 * "先使能再改目标"的中间态, 让中断算出一拍错误的输出。
 */

/**
  * @brief  使能航向环
  * @param  target_deg   目标航向(度), 一般就是起跑瞬间的当前航向
  * @param  out_max_rpm  自转修正量的限幅(RPM), <=0 时用默认值
  * @note   整组状态在关中断下更新(只有几个时钟周期), 所以中断要么
  *         看到"完全没使能", 要么看到"目标和参数都已就绪"。
  */
void YawLoop_Enable(float target_deg, float out_max_rpm)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_yaw_loop_enable = 0u;
    g_yaw_output      = 0.0f;
    s_yaw_integral    = 0.0f;
    g_yaw_target      = target_deg;
    g_yaw_out_max     = (out_max_rpm > 0.0f) ? out_max_rpm : YAW_OUT_MAX_DEFAULT;
    g_yaw_loop_enable = 1u;
    __set_PRIMASK(primask);
}

/**
  * @brief  关闭航向环
  * @note   关闭后中断不再更新 g_yaw_output(保持为 0), 但 IMU 仍然
  *         每 20ms 读一次 —— 下次起跑时数据是热的, 不用等 I2C 重新稳定。
  */
void YawLoop_Disable(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_yaw_loop_enable = 0u;
    g_yaw_output      = 0.0f;
    s_yaw_integral    = 0.0f;
    __set_PRIMASK(primask);
}

/**
  * @brief  陀螺仪是否还在正常出数
  * @retval 1 = 正常, 0 = 掉线
  * @note   运行中返回 0 必须立刻停车: 没有航向反馈的闭环比开环更危险。
  */
int8_t YawLoop_IsImuAlive(void)
{
    if (jy901s_data.ready == 0u)
    {
        return 0;
    }

    if ((uint32_t)(HAL_GetTick() - JY901S_GetUpdateTick()) > YAW_IMU_TIMEOUT_MS)
    {
        return 0;
    }

    return 1;
}

/**
  * @brief  当前航向误差(度), 只用于显示
  */
float YawLoop_GetError(void)
{
    return g_yaw_target - jy901s_data.yaw_continuous;
}

/**
  * @brief  定时器中断回调: 读 IMU + 航向闭环运算(TIM3, 20ms 一次)
  * @note   顺序:
  *           1. 读 I2C(全工程唯一访问点)
  *           2. 没使能闭环 -> 输出清零返回
  *           3. 本拍没读到有效数据 -> 沿用上一拍输出, 不积分
  *           4. err = target - yaw
  *           5. u = Kp*err + ∫ - Kd*gz, 限幅后写入 g_yaw_output
  *
  *         ★ 中断里绝对不能出现 HAL_Delay: TIM3 优先级 5 高于
  *           SysTick 的默认优先级, 在里面等 tick 会永久卡死。
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    float err;
    float u;
    float integral;
    float out_max;

    /* 这个回调是所有定时器共用的, 先确认是 TIM3 触发 */
    if (htim->Instance != TIM3)
    {
        return;
    }

    /* ---- 1. 读 IMU ----
     * 读失败或坏帧时 jy901s_data 仍是上一拍的合法值,
     * 所以下面照样可以继续闭环, 只是这一拍不积分。 */
    {
        uint8_t ok = JY901S_ReadIsr();

        if (g_yaw_loop_enable == 0u)
        {
            g_yaw_output   = 0.0f;
            s_yaw_integral = 0.0f;
            return;
        }

        if ((ok == 0u) || (jy901s_data.ready == 0u))
        {
            /* 数据过期: 保持上一拍的修正量, 不重新计算 */
            return;
        }
    }

    /* ---- 2. 航向误差(用连续角度, 转过 ±180 也不跳变) ---- */
    err = g_yaw_target - jy901s_data.yaw_continuous;

    /* ---- 3. 积分器 ----
     * 死区内不累加; 大误差(被撞歪/刚起跑)直接清零, 免得老账一直反着顶。 */
    integral = s_yaw_integral;

    if ((err > YAW_INT_RESET_DEG) || (err < -YAW_INT_RESET_DEG))
    {
        integral = 0.0f;
    }
    else if ((err > YAW_DEADBAND_DEG) || (err < -YAW_DEADBAND_DEG))
    {
        integral += (err * g_yaw_ki * YAW_DT_S);

        if (integral > YAW_INT_MAX_RPM)
        {
            integral = YAW_INT_MAX_RPM;
        }
        else if (integral < -YAW_INT_MAX_RPM)
        {
            integral = -YAW_INT_MAX_RPM;
        }
    }

    s_yaw_integral = integral;

    /* ---- 4. PI-D ---- */
    u = (g_yaw_kp * err) + integral - (g_yaw_kd * jy901s_data.gz);

    /* ---- 5. 对称限幅 ----
     * 不做"按误差方向单边截断": 那等于把阻尼项在反向时也砍掉,
     * 车一到目标附近就会反复过冲。 */
    out_max = g_yaw_out_max;

    if (u > out_max)
    {
        u = out_max;
    }
    else if (u < -out_max)
    {
        u = -out_max;
    }

    g_yaw_output = u;
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
