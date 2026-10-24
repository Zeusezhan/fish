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
#include "function.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ==================== 航向角闭环(姿态环)参数 ====================
 *
 * 【控制律】PI-D
 *     err = 目标航向 - 当前航向
 *     u   = Kp*err + Ki*∫err dt - Kd*gz
 *   其中 gz 是陀螺仪 Z 轴角速度(左转为正), 直接拿来做阻尼,
 *   不需要自己差分航向角, 因此没有"差分放大噪声"的问题。
 *
 * 【为什么加积分 I】原来只有 PD, 稳态误差消不掉。
 *   麦克纳姆轮横移时四个轮子的实际摩擦/辊子接触并不对称, 会持续
 *   产生一个偏航力矩; 纯 PD 只能靠"留一点航向误差"来对抗它,
 *   表现就是"横移时车头慢慢歪过去, 而且歪了就回不来"。
 *   加上很小的 Ki 就能把这个稳态误差积掉。
 *
 * 【整定顺序】
 *   1. 先只调 Kp: 从小往大加, 加到"修得够快但开始有点晃";
 *   2. 再加 Kd 把晃动压下去(Kd 直接作用于陀螺仪角速度);
 *   3. 最后加一点点 Ki 消掉剩余的稳态偏航, 别一次加太多。
 */
#define YAW_PID_KP_DEFAULT      (4.0f)      /* 比例: 航向误差 1 度给多少转速 */
#define YAW_PID_KI_DEFAULT      (0.30f)     /* 积分: 消稳态误差, 单位 RPM/(度*秒) */
#define YAW_PID_KD_DEFAULT      (0.35f)     /* 微分: 阻尼项, 压住过冲 */

/* 控制周期(秒), 必须与 TIM3 的 20ms 一致。
 * 积分累加 "err * Ki * dt" 用的是它。 */
#define YAW_PID_DT_S            (0.02f)

/* 积分清零门限(度): 误差超过这个值(比如被撞歪、刚开始跑)时,
 * 把积分器清掉, 避免积分饱和后长时间反向顶。
 * ★ 别设太大: 一段惯导只有 3~5 秒, 误差一旦超过 10 度, 积分很容易
 *   在几秒内冲到限幅, 等车回正时它还在拼命往老方向顶。 */
#define YAW_PID_INT_RESET_DEG   (8.0f)

/* 积分项输出限幅(RPM): 积分最多贡献多少转速, 防止饱和。
 * 这个值应该明显小于总限幅(30RPM), 让比例项始终占主导。 */
#define YAW_PID_INT_MAX         (8.0f)

/* 死区(度): 误差小于它就只留阻尼项, 不再积分。
 * ★ 千万别设大 —— 原来这里是 2.0 度, 等于"2 度以内的偏差全部放弃",
 *   而 turn 又是整数 RPM, 实际能纠的最小偏差约 0.25 度。
 *   2 度死区在横移时足以让车头越走越歪。 */
#define YAW_PID_TOLERANCE       (0.15f)

/* 输出限幅(RPM): 航向修正最多给多少自转分量。
 * 横移时 50RPM 的横移速度配 ±30RPM 的修正量, 轮速在 20~80 之间,
 * 既有修正能力又不会甩尾。 */
#define YAW_OUT_MAX_DEFAULT     (30.0f)

/* IMU 掉线判据(ms): 超过这么久没读到有效数据就判定掉线, 必须停车。
 * ★ 必须大于 JY901S_POLL_RETRY_MS 的重试间隔, 否则正常抖动会被
 *   误判成掉线(原代码 100ms 判据 vs 200ms 重试间隔就是反的)。 */
#define YAW_IMU_TIMEOUT_MS      (150u)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ==================== 航向闭环状态 ====================
 *
 * 【数据流】
 *   TIM3 中断(20ms):
 *       JY901S_ReadIsr()      读 I2C, 校验, 更新 jy901s_data
 *       PI-D 计算  ->  g_yaw_output
 *   主循环:
 *       状态机(InertialRun / Menu)决定 vx / vy,
 *       再取 g_yaw_output 当 omega, 合成麦轮速度后下发串口。
 *
 *   中断里"只算不发"的原因: fish_Step_All_Control() 一次要 4~8ms,
 *   放中断里会把整个系统拖死; 而串口指令本身不需要 20ms 的精度,
 *   晚几毫秒下发对航向没有影响。
 *
 * 【变量可见性】
 *   g_yaw_output 是 32 位 float, Cortex-M4 上单次写入是原子的,
 *   主循环直接读不会读到"写了一半"的值, 所以不需要临界区。
 * ==================================================== */

/* 航向环使能: 1 = 中断里每拍计算并更新 g_yaw_output */
volatile uint8_t g_yaw_loop_enable = 0;

/* 目标航向(度), 相对 JY901S_ZeroYaw() 的清零点 */
volatile float   g_yaw_target = 0.0f;

/* 可在线整定的参数(无线串口可以改, 见 wireless_uart.c) */
volatile float   g_yaw_kp = YAW_PID_KP_DEFAULT;
volatile float   g_yaw_ki = YAW_PID_KI_DEFAULT;
volatile float   g_yaw_kd = YAW_PID_KD_DEFAULT;

/* 中断算出的航向修正量(RPM), 主循环取它当 omega */
volatile float   g_yaw_output = 0.0f;

/* 输出限幅(RPM), 由运行模式决定(横移/直行可以不一样) */
volatile float   g_yaw_out_max = YAW_OUT_MAX_DEFAULT;

/* 积分器状态。g_yaw_integral_clear 置 1 后, 中断在下一拍把它清零,
 * 用于每段起步、切换模式时复位。 */
volatile float   g_yaw_integral = 0.0f;
volatile uint8_t g_yaw_integral_clear = 1u;

/* 中断最近一拍 IMU 读取是否成功(1 = 成功)。
 * 主循环用它 + JY901S_GetUpdateTick() 判断陀螺仪是否掉线。 */
volatile uint8_t g_imu_ok_last = 0u;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void YawLoop_Enable(float   target_deg, float out_max_rpm);
void YawLoop_Disable(void);
int8_t YawLoop_IsImuAlive(void);
float YawLoop_GetError(void);
void Gyro_SelfTest(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 陀螺仪自检模式: 1 = 上电后只做陀螺仪测试, 不进菜单、不发任何电机命令,
 * 也不启动 TIM3 中断。用来单独验证 JY901S 能不能正常工作; 调通后改回 0。 */
#define GYRO_SELFTEST_ENABLE    0

/* 无线串口上报开关。开了之后可以在上位机上实时看航向/误差,
 * 也能在线改 Kp/Ki/Kd(协议见 wireless_uart.c)。不需要时置 0。 */
#define WIRELESS_REPORT_ENABLE  1

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

  /* ★ SysTick 优先级提到最高。
   *
   * TIM3 的航向环中断(优先级 5)会抢占 SysTick(默认优先级 15)。
   * 中断里读一次 I2C 要 1.5ms 左右, 最长可能到 4.5ms(等 HAL 超时),
   * 而 SysTick 的周期只有 1ms —— 一次 I2C 读就能吃掉 1~4 个 tick。
   * SysTick 是单一位挂起标志, 被屏蔽期间到期的多个周期只会'补一次',
   * 结果就是 HAL_GetTick() 越跑越慢。
   *
   * 本项目所有分段时长、超时判据都基于 HAL_GetTick(), 时基变慢
   * 会让"3.3 秒的一段"实际跑 3.6 秒 —— 跑图必然走位。
   *
   * 把 SysTick 提到和 USART1 一样的最高优先级, 它就能随时抢占 TIM3,
   * 时基恢复准确, 而 I2C 读只是被短暂打断(不影响传输)。 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

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

  JY901S_Init();

  OLED_ShowString(0, 0, "JY901S init...");
  OLED_Refresh();

  HAL_Delay(500);
  (void)JY901S_ReadOnce();

#if GYRO_SELFTEST_ENABLE
  /* 自检模式下把回传率设成 100Hz, 和闭环时一致, 方便判断丢帧率;
   * 不清零航向, 这样屏幕上直接看到模块的绝对航向。 */
  if (jy901s_data.ready)
  {
    JY901S_SetOutputRate(JY901S_RRATE_100HZ);
  }

  Gyro_SelfTest();
#else
  if (!jy901s_data.ready)
  {
    OLED_ShowString(0, 2, "No IMU data!");
    OLED_ShowString(0, 4, "Chk wire+pullup");
    OLED_Refresh();
    HAL_Delay(1500);
  }
  else
  {
    JY901S_SetOutputRate(JY901S_RRATE_100HZ);
  }

  /* 起跑前把当前朝向清零, 之后读到的 Yaw 都是相对这个方向。
   * 必须在 TIM3 启动之前做 —— 它内部会短暂屏蔽 TIM3 中断。 */
  JY901S_ZeroYaw();
#endif
  Key_Init();
  Menu_Init();

#if WIRELESS_REPORT_ENABLE
  /* 无线串口: 只用于上位机观察航向/误差和在线调 Kp/Ki/Kd, 不参与控制 */
  Wireless_Init();
#endif

  /* 打开 FPU 自动/惰性压栈。
   * TIM3 中断里做浮点运算, 主循环也在做浮点解析; 不打开的话,
   * 中断抢占主循环时可能破坏 s0-s15 寄存器, 造成偶发数据跳变。 */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  FPU->FPCCR |= FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk;
#endif

#if !GYRO_SELFTEST_ENABLE
  /* ★★ 这一行是整个姿态闭环的总开关 ★★
   *
   * 它启动 TIM3 的 20ms 中断, 中断里做两件事:
   *   1. 读 JY901S(唯一的 I2C 访问点)
   *   2. 算航向 PI-D, 把修正量写进 g_yaw_output
   *
   * 原来这一行是被注释掉的, 所以就算 InertialRun 里
   * s_heading_hold = 1、g_yaw_loop_enable = 1, g_yaw_output 也恒为 0,
   * 惯导跑图实际是纯开环 —— 直行能凑合, 横移必然走不直。
   *
   * 必须放在所有 I2C 初始化和 ZeroYaw() 之后: 一旦启动,
   * IMU 就只由中断读了, 主循环不能再碰。
   */
  (void)HAL_TIM_Base_Start_IT(&htim3);

  YawLoop_Disable();    /* 上电默认不闭环, 等状态机使能 */
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if GYRO_SELFTEST_ENABLE
    Gyro_SelfTest();
#else
    /* 主循环只做三件事:
     *   1. 按键扫描(内部 10ms 限频)
     *   2. 菜单 + 状态机(InertialRun_Task 在里面被调用)
     *   3. 无线上报(内部 40ms 限频)
     *
     * ★ 这里已经不再调用 JY901S_Poll() 了 —— IMU 由 TIM3 中断读,
     *   主循环碰 I2C 会和中断抢总线。 */
    Key_Scan();

    Menu_Run();

#if WIRELESS_REPORT_ENABLE
    Wireless_ReportYaw();
#endif
#endif
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
 * 状态机(InertialRun / Menu 的单段调试)通过这三个函数控制航向环,
 * 不要直接去写 g_yaw_loop_enable / g_yaw_target —— 那样容易出现
 * "先开环再改目标"的中间态, 让中断算出一拍错误的输出。
 */

/**
  * @brief  使能航向环, 并指定目标航向和输出限幅
  * @param  target_deg   目标航向(度), 一般就是起跑瞬间的当前航向
  * @param  out_max_rpm  自转修正量的限幅(RPM)
  * @note   整个过程在关中断下完成(只有几个时钟周期), 所以中断
  *         要么看到"完全没使能", 要么看到"目标和参数都已就绪",
  *         不会读到改了一半的状态。
  */
void YawLoop_Enable(float target_deg, float out_max_rpm)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_yaw_loop_enable = 0u;
    g_yaw_output      = 0.0f;
    g_yaw_integral    = 0.0f;
    g_yaw_target      = target_deg;
    g_yaw_out_max     = (out_max_rpm > 0.0f) ? out_max_rpm : YAW_OUT_MAX_DEFAULT;
    g_yaw_loop_enable = 1u;
    __set_PRIMASK(primask);
}

/**
  * @brief  关闭航向环
  * @note   关闭后中断不再更新 g_yaw_output(会保持为 0),
  *         但 IMU 仍然每 20ms 读一次 —— 这样下次起跑时数据是热的,
  *         不需要等 I2C 重新稳定。
  */
void YawLoop_Disable(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_yaw_loop_enable = 0u;
    g_yaw_output      = 0.0f;
    g_yaw_integral    = 0.0f;
    __set_PRIMASK(primask);
}

/**
  * @brief  判断陀螺仪是否还在正常出数
  * @retval 1 = 正常, 0 = 掉线
  * @note   判据是"最近一次成功解析的时刻"距今有没有超过 YAW_IMU_TIMEOUT_MS。
  *         运行中这个函数返回 0 就必须立刻停车 —— 没有航向反馈的
  *         闭环比开环更危险。
  */
int8_t YawLoop_IsImuAlive(void)
{
    uint32_t now;

    if (!jy901s_data.ready)
    {
        return 0;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - JY901S_GetUpdateTick()) > YAW_IMU_TIMEOUT_MS)
    {
        return 0;
    }

    return 1;
}

/**
  * @brief  当前航向误差(度), 仅用于显示
  */
float YawLoop_GetError(void)
{
    return g_yaw_target - jy901s_data.yaw_continuous;
}

/**
  * @brief  陀螺仪自检: 只读不动(自检模式下由主循环反复调用)
  * @note   1. 只驱动 JY901S_Poll(), 不发电机的任何命令,
  *            所以可以彻底排除"陀螺仪影响电机通信"的怀疑;
  *         2. 屏幕显示驱动的统计: OK = 有效帧, FAIL = I2C 通信失败,
  *            BAD = 通信成功但被判为坏帧(全0/全FF/角速度越界/航向跳变)。
  *            OK 占比低于 95% 就该怀疑接线和上拉电阻了;
  *         3. 按 PE0 把当前朝向记为 0 度基准, 之后直接看
  *            左转 90° 时 Yaw 是不是接近 +90;
  *         4. 按 PE1 重新开始统计。
  */
void Gyro_SelfTest(void)
{
    static uint32_t last_draw = 0;
    static uint32_t ok_prev   = 0;
    static uint32_t fail_prev = 0;
    static uint32_t bad_prev  = 0;
    static uint32_t ok_show   = 0;
    static uint32_t fail_show = 0;
    static uint32_t bad_show  = 0;
    uint32_t now;
    uint32_t ok_now;
    uint32_t fail_now;
    uint32_t bad_now;
    uint8_t  key;
    char     line[24];

    /* 只读, 不动电机: 这一路代码完全不碰 USART1 */
    (void)JY901S_Poll();
    Key_Scan();

    key = Key_GetPress();
    if (key == KEY_1)
    {
        /* 把当前朝向记为 0 度: 之后屏幕上的 Yaw 就是相对角,
         * 左转 90 度应接近 +90, 右转 90 度应接近 -90。 */
        JY901S_ZeroYaw();
    }
    else if (key == KEY_2)
    {
        JY901S_ResetStats();
        ok_prev   = 0;
        fail_prev = 0;
        bad_prev  = 0;
        ok_show   = 0;
        fail_show = 0;
        bad_show  = 0;
    }

    now = HAL_GetTick();
    if ((now - last_draw) < 200u)
    {
        return;
    }
    last_draw = now;

    /* 记录统计窗口的增量, 这样屏幕上是"最近 200ms"的数字, 不是累计值 */
    ok_now   = JY901S_GetOkCount();
    fail_now = JY901S_GetFailCount();
    bad_now  = JY901S_GetBadCount();
    ok_show   = ok_now   - ok_prev;
    fail_show = fail_now - fail_prev;
    bad_show  = bad_now  - bad_prev;
    ok_prev   = ok_now;
    fail_prev = fail_now;
    bad_prev  = bad_now;

    OLED_ClearBuffer();

    if (!jy901s_data.ready)
    {
        OLED_ShowString(0, 0, "JY901S SELFTEST");
        OLED_ShowString(0, 2, "No data from IMU");
        OLED_ShowString(0, 4, "Check PB6/PB7 4k7");
        OLED_ShowString(0, 6, "Then PE1:recount");
        OLED_ShowString(0, 7, "PE1:recount");
        OLED_Refresh();
        return;
    }

    /* Yaw 是相对 PE0 清零点的角度, 直接用来验证 90 度转向 */
    (void)snprintf(line, sizeof(line), "Yaw:%8.1f", (double)jy901s_data.yaw);
    OLED_ShowString(0, 0, line);

    (void)snprintf(line, sizeof(line), "Gz:%8.1f", (double)jy901s_data.gz);
    OLED_ShowString(0, 1, line);

    (void)snprintf(line, sizeof(line), "Roll:%6.1f", (double)jy901s_data.roll);
    OLED_ShowString(0, 2, line);

    (void)snprintf(line, sizeof(line), "Pit:%7.1f", (double)jy901s_data.pitch);
    OLED_ShowString(0, 3, line);

    /* 最近 200ms 内的读取质量 */
    (void)snprintf(line, sizeof(line), "OK:%lu F:%lu B:%lu",
                   (unsigned long)ok_show,
                   (unsigned long)fail_show,
                   (unsigned long)bad_show);
    OLED_ShowString(0, 5, line);

    OLED_ShowString(0, 6, "F=I2C B=badframe");
    OLED_ShowString(0, 7, "PE0:zero PE1:rst");

    OLED_Refresh();
}

/**
  * @brief  定时器中断回调: 读 IMU + 航向闭环运算(TIM3, 20ms 一次)
  * @param  htim  触发中断的定时器句柄
  * @note   ★ 这是整个姿态闭环的心脏, 每 20ms 执行一次, 顺序是:
  *
  *           1. 读 I2C(唯一访问点) —— JY901S_ReadIsr()
  *           2. 算误差 err = 目标航向 - 当前航向
  *           3. PI-D: u = Kp*err + Ki*∫err*dt - Kd*gz
  *           4. 限幅后写进 g_yaw_output
  *
  *         主循环只负责把 g_yaw_output 当 omega 合成到麦轮速度上,
  *         再通过串口下发 —— "中断算、主循环发" 是刻意的分工:
  *         fish_Step_All_Control() 一次要 4~8ms, 放进中断会把整个
  *         系统拖死, 而它晚几毫秒执行对航向几乎没有影响。
  *
  *         ★ 中断里绝对不要出现 HAL_Delay:
  *           TIM3 优先级 5 高于 TIM3 之外的大部分中断, 在里面等 tick
  *           就是等一个不会到来的事件(所以 I2C 驱动里所有 HAL_Delay 都删了)。
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
     * 全工程唯一的 I2C 访问点。读失败或坏帧时 jy901s_data 仍是上一拍
     * 的合法值, 所以下面照样可以用它继续闭环, 只是这一拍不积分。 */
    g_imu_ok_last = JY901S_ReadIsr();

    /* ---- 2. 没使能闭环: 输出清零, 直接返回 ----
     * 这就是原来缺的那一步: 不检查 g_yaw_loop_enable, 中断会一直算,
     * 而主循环又不知道该不该用, 两边永远对不上。 */
    if (!g_yaw_loop_enable)
    {
        g_yaw_output   = 0.0f;
        g_yaw_integral = 0.0f;
        return;
    }

    /* ---- 3. 上一拍没读到新数据 ----
     * 沿用上一拍的输出(保持修正量不跳变), 不积分、不重新计算,
     * 避免在数据过期的时刻算出一个假的大误差。 */
    if ((g_imu_ok_last == 0u) || (jy901s_data.ready == 0u))
    {
        return;
    }

    /* ---- 4. 航向误差 ----
     * 用 yaw_continuous(连续角度), 转过 ±180 也不会跳变 */
    err = g_yaw_target - jy901s_data.yaw_continuous;

    /* ---- 5. 大误差时清积分 ----
     * 比如刚起跑、被人推歪、或者撞墙之后, 积分器里存的老账已经没有
     * 意义了, 留着只会让它长时间反向顶, 反而更难回正。 */
    if (g_yaw_integral_clear != 0u)
    {
        g_yaw_integral = 0.0f;
        g_yaw_integral_clear = 0u;
    }
    else if ((err > YAW_PID_INT_RESET_DEG) || (err < -YAW_PID_INT_RESET_DEG))
    {
        g_yaw_integral = 0.0f;
    }

    /* ---- 6. 积分项 ----
     * 死区外的误差才累加, 避免在极小误差上把积分器慢慢堆起来。
     * 累加完立即限幅(抗积分饱和), 防止大误差时积分越积越大、
     * 回正时冲过头。 */
    integral = g_yaw_integral;

    if ((err > YAW_PID_TOLERANCE) || (err < -YAW_PID_TOLERANCE))
    {
        integral += (err * g_yaw_ki * YAW_PID_DT_S);

        if (integral > YAW_PID_INT_MAX)
        {
            integral = YAW_PID_INT_MAX;
        }
        else if (integral < -YAW_PID_INT_MAX)
        {
            integral = -YAW_PID_INT_MAX;
        }
    }

    g_yaw_integral = integral;

    /* ---- 7. PI-D 运算 ----
     * 比例: 误差越大修得越快
     * 积分: 消掉麦轮不对称带来的稳态偏航
     * 微分: 直接用陀螺仪角速度做阻尼, 不用自己差分航向角,
     *       所以不会放大噪声。gz > 0 表示正在左转, 减去它起阻尼作用。 */
    u = (g_yaw_kp * err) + integral - (g_yaw_kd * jy901s_data.gz);

    /* ---- 8. 限幅 ----
     * 这里只做"对称限幅", 不再像原来那样按误差方向单边截断 ——
     * 单边截断等于把微分阻尼项在反向时也砍掉, 车一到目标附近就会
     * 反复过冲。阻尼项必须有正负两个方向才起得到作用。 */
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
