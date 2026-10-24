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

/* ==================== 航向角 PD 闭环参数 ==================== */

/* PD 参数(上电默认值, 运行中可改):
 * 整定时先只调 Kp —— 从小往大加, 加到"修得够快但开始有点晃";
 * 然后再加 Kd 把晃动压下去。Kd 用的是陀螺仪角速度, 不需要自己差分。 */
#define YAW_PID_KP_DEFAULT      (4.0f)      /* 比例: 航向误差 1 度给多少转速 */
#define YAW_PID_KD_DEFAULT      (0.35f)     /* 微分: 阻尼项, 压住过冲 */

/* 航向死区: 误差小于该角度时输出为零 */
#define YAW_PID_TOLERANCE       (2.0f)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ==================== 航向 PD 状态 ====================
 * TIM3 中断每 20ms 计算一次 PD 输出;
 * 由 Menu 发请求, 主循环 Heading_ApplyOutput() 真正下发电机命令。
 * ==================================================== */

/* PE0 在 Mode Select 页启停 PD 计算；上电默认关闭。 */
volatile uint8_t g_yaw_loop_enable = 0;

/* Menu 模块通过此标志请求开始/停止 PD 计算。 */
volatile uint8_t g_mode_run_request = 0;

/* 目标航向(度), 相对 JY901S_ZeroYaw() 的清零点 */
volatile float   g_yaw_target = 0.0f;

/* 可在线整定的 PD 参数。 */
volatile float   g_yaw_kp = YAW_PID_KP_DEFAULT;
volatile float   g_yaw_kd = YAW_PID_KD_DEFAULT;

/* 中断算出的 PD 输出(RPM)。 */
volatile float   g_yaw_output = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Heading_Task(void);                                        /* 主循环调用: 更新监测与串口上报 */
void Heading_ApplyOutput(void);                                 /* 主循环调用: 下发输出 */
void Heading_UpdateRunState(void);                              /* 主循环调用: 执行启动/停车请求 */
void Gyro_SelfTest(void);                                       /* 陀螺仪单独自检, 不碰电机 */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 陀螺仪自检模式: 1 = 上电后只做陀螺仪测试, 不进菜单、不发任何电机命令。
 * 用来单独验证 JY901S 能不能正常工作; 调通后改回 0。 */
#define GYRO_SELFTEST_ENABLE    0

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

  JY901S_Init();

  OLED_ShowString(0, 0, "JY901S init...");
  OLED_Refresh();

  HAL_Delay(500);
  (void)JY901S_ReadOnce();

#if GYRO_SELFTEST_ENABLE
  /* 自检模式下只把回传率设成 50Hz, 方便和 20ms 的读取节奏对上;
   * 不清零航向, 这样屏幕上直接看到模块的绝对航向。 */
  if (jy901s_data.ready)
  {
    JY901S_SetOutputRate(JY901S_RRATE_50HZ);
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
    /* 6 轴算法和校准结果已经保存在模块里, 不需要每次上电重复执行 */
    JY901S_SetOutputRate(JY901S_RRATE_50HZ);
  }

  /* 起跑前把当前朝向清零, 之后读到的 Yaw 都是相对这个方向 */
  JY901S_ZeroYaw();
#endif
  Key_Init();
#if GYRO_SELFTEST_ENABLE
  /* 自检模式只用 PE0/PE1, 不初始化菜单, 避免误触发电机命令 */
#else
  Menu_Init();
#endif

  /* 初始化无线串口上报(VOFA+ 波形), 只复位内部计时,
   * 串口本身已由 MX_USART2_UART_Init() 配好 */
  //Wireless_Init();

  /* 打开 FPU 自动/惰性压栈。
   * TIM3 中断里做浮点 PD, 主循环也在做浮点解析; 不打开的话,
   * 中断抢占主循环时可能破坏 s0-s15 寄存器, 造成偶发数据跳变。 */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  FPU->FPCCR |= FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk;
#endif

  /* 启动 TIM3 更新中断(20ms 一次), 航向角 PD 运算在中断回调里做。
   * CubeMX 不会自动生成这一句, 不加的话中断永远不会进来。 */
  /* 当前只调试底盘运动, 暂时不启动航向 PD 中断;
   * 需要"边调试边闭环"时把下面这行取消注释, 并把 Menu.c 里的
   * MENU_DEBUG_HEADING_LOOP_ENABLED 改为 1。 */
  /* HAL_TIM_Base_Start_IT(&htim3); */

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
    JY901S_Poll();

    /* 处理菜单的 PD 监测启停请求, 并上报陀螺仪/PD 数据 */
    //Heading_Task();

    /* VOFA+ 下行命令("P6.0"/"D0.4"/"kp,kd")由 USART2 接收中断
     * 在 HAL_UART_RxCpltCallback() 里自动解析, 这里不需要轮询。 */
    Wireless_ReportYaw();

    Key_Scan();
    Menu_Run();
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

/* ==================== 航向角 PD 闭环参数 ==================== */

/* --- 原地转向模式的输出限幅(RPM) ---
 * 下限: 步进电机转速太低是转不动的, 给一个能转起来的最小值
 * 上限: 防止误差大时冲太快甩过头 */
#define YAW_PID_TURN_OUT_MIN    (20.0f)
#define YAW_PID_TURN_OUT_MAX    (80.0f)

/* --- 边走边修模式的输出限幅(RPM) ---
 * 这时的输出是叠加在前进/横移速度上的"左右差速修正", 不能太大,
 * 否则车会甩尾。 */
#define YAW_PID_DRIVE_OUT_MAX   (60.0f)

/* 电机命令下发间隔(ms):
 * fish_Step_All_Control() 内部有好几个 HAL_Delay(2), 一次调用要 8ms 以上,
 * 所以下发必须限频, 不能每 20ms 都发一次。 */
#define YAW_CMD_INTERVAL_MS     (30u)

#define MODE_START_YAW_TARGET   (0.0f)

/* 启动后停车前的"前进基准速度(RPM)":
 *   0 = 原地靠 PD 保持航向(不前进, 最安全, 适合先验证航向保持)
 *   非 0 = 边走边修(车会持续前进, 测试前先把车架空或留出跑道)
 *
 * ★ base != 0 时不会自动停: 一边前进一边靠 PD 保持航向,
 *   除非按 PE0 停车, 否则会一直往前走。 */
#define MODE_START_DRIVE_BASE   (0)

/* 前进基准速度(RPM):
 *   = 0  -> 原地转向模式(输出有下限, 保证转得动)
 *   != 0 -> 边走边修模式(输出是差速修正量, 叠加在这个基准速度上) */
volatile int16_t g_drive_base = 0;

/* 有新结果的标志: 中断算完置 1, 主循环下发或丢弃后清 0 */
volatile uint8_t g_yaw_output_ready = 0;

/* 电机当前是否已经停车, 防止在目标点反复发 Car_Stop() */
static uint8_t s_motor_stopped = 1;

/**
  * @brief  陀螺仪自检: 只读不动(自检模式下由主循环反复调用)
  * @note   1. 只驱动 JY901S_Poll(), 不发电机的任何命令,
  *            所以可以彻底排除"陀螺仪影响电机通信"的怀疑;
  *         2. 屏幕同时显示 I2C 读取成功率, 用来区分
  *            "接线/上拉有问题" 和 "数据解析有问题";
  *         3. 按 PE0 把当前朝向记为 0 度基准, 之后直接看
  *            左转 90° 时 Yaw 是不是接近 +90;
  *         4. 按 PE1 重新开始统计读取成功率。
  */
void Gyro_SelfTest(void)
{
    static uint32_t last_draw = 0;
    static uint32_t ok_cnt    = 0;
    static uint32_t try_cnt   = 0;
    static uint32_t tol_cnt   = 0;
    uint32_t now;
    uint8_t  key;
    uint8_t  ret;
    char     line[24];

    /* 只读, 不动电机: 这一路代码完全不碰 USART1 */
    ret = JY901S_Poll();
    Key_Scan();

    if (ret != 2u)          /* 2 = 还没到 20ms 的读取时刻, 不计数 */
    {
        /* 先取诊断标志: 它只反映本次 JY901S_ReadOnce 的结果 */
        uint8_t tol = JY901S_TakeDiagLatch();

        try_cnt++;
        if (ret == 0u)
        {
            ok_cnt++;
        }
        else if (tol)
        {
            /* 传输成功但两次数据差太多: 跨刷新点或误码 */
            tol_cnt++;
        }
    }

    key = Key_GetPress();
    if (key == KEY_1)
    {
        /* 把当前朝向记为 0 度: 之后屏幕上的 Yaw 就是相对角,
         * 左转 90 度应接近 +90, 右转 90 度应接近 -90。 */
        JY901S_ZeroYaw();
    }
    else if (key == KEY_2)
    {
        ok_cnt  = 0;
        try_cnt = 0;
        tol_cnt = 0;
    }

    now = HAL_GetTick();
    if ((now - last_draw) < 200u)
    {
        return;
    }
    last_draw = now;

    OLED_ClearBuffer();

    if (!jy901s_data.ready)
    {
        (void)snprintf(line, sizeof(line), "OK:%lu/%lu TOL:%lu",
                       (unsigned long)ok_cnt, (unsigned long)try_cnt,
                       (unsigned long)tol_cnt);
        OLED_ShowString(0, 0, "JY901S SELFTEST");
        OLED_ShowString(0, 1, line);
        (void)snprintf(line, sizeof(line), "Bus:%luHz",
                       (unsigned long)JY901S_GetBusSpeed());
        OLED_ShowString(0, 2, line);
        OLED_ShowString(0, 4, "No data from IMU");
        OLED_ShowString(0, 6, "Check PB6/PB7 4k7");
        OLED_ShowString(0, 7, "PE1:recount");
    }
    else
    {
        /* Yaw 是相对 PE0 清零点的角度, 直接用来验证 90 度转向 */
        (void)snprintf(line, sizeof(line), "Yaw:%7.1f", (double)jy901s_data.yaw);
        OLED_ShowString(0, 0, line);

        (void)snprintf(line, sizeof(line), "Gz:%7.1f", (double)jy901s_data.gz);
        OLED_ShowString(0, 1, line);

        (void)snprintf(line, sizeof(line), "Roll:%6.1f", (double)jy901s_data.roll);
        OLED_ShowString(0, 2, line);

        (void)snprintf(line, sizeof(line), "Pit:%7.1f", (double)jy901s_data.pitch);
        OLED_ShowString(0, 3, line);

        /* Z 轴加速度: 水平静止时应在 1.0g 附近, 可用来验证模块装平了 */
        (void)snprintf(line, sizeof(line), "Az:%7.2fg", (double)jy901s_data.az);
        OLED_ShowString(0, 4, line);

        /* OK = 成功/总次数, TOL = 因"两次差太多"被丢弃的次数 */
        (void)snprintf(line, sizeof(line), "OK:%lu/%lu TOL:%lu",
                       (unsigned long)ok_cnt, (unsigned long)try_cnt,
                       (unsigned long)tol_cnt);
        OLED_ShowString(0, 5, line);

        /* 总线速率: 传输失败时 HAL 会自动降速, 低于 100k 说明信号差 */
        (void)snprintf(line, sizeof(line), "Bus:%luHz",
                       (unsigned long)JY901S_GetBusSpeed());
        OLED_ShowString(0, 6, line);

        OLED_ShowString(0, 7, "PE0:zero PE1:rst");
    }

    OLED_Refresh();
}

/**
  * @brief  定时器中断回调: 航向角 PD 运算
  * @param  htim  触发中断的定时器句柄
  * @note   TIM3 配置成 20ms 一次, 所以这个函数每 20ms 进来一次。
  *
  *         运算流程:
  *           误差 err = 目标航向 - 当前航向
  *           输出 u   = Kp * err - Kd * gz      (gz 是陀螺仪 Z 轴角速度)
  *           限幅后存进 g_yaw_output, 由主循环下发电机命令
  *
  *         中断内只做数值运算, 不进行阻塞式外设操作。
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    JY901S_Data_t imu;
    float err;
    float u;
    float out_max;
    float out_min;

    /* 这个回调是所有定时器共用的, 先确认是 TIM3 触发 */
    if (htim->Instance != TIM3)
    {
        return;
    }

    /* ---- 0. 取一份一致的 IMU 快照 ----
     * 主循环每次更新 jy901s_data 都会屏蔽本中断, 所以拿到的
     * yaw/gz 一定来自同一帧, 不会读到改了一半的数据。 */
    if (!JY901S_TryRead(&imu))
    {
        g_yaw_output = 0.0f;
        return;
    }

    /* ---- 1. 航向误差 ----
     * 用 yaw_continuous(连续角度), 这样转过 ±180 也不会跳变 */
    err = g_yaw_target - imu.yaw_continuous;

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
        u = (g_yaw_kp * err) - (g_yaw_kd * imu.gz);

        /* 限制输出幅值; 是否下发电机由主循环决定 */
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

/**
  * @brief  把中断算出的航向输出下发给电机(主循环里反复调用)
  * @note   内部做了三件事:
  *           1. 限频 30ms —— fish_Step_All_Control() 一次要 8ms 以上
  *           2. (基准速度, 修正量) 没变化就不重发 —— 电机收到后会保持
  *           3. 走 Car_Drive(), 也就是麦轮的"前进 + 自转修正"合成
  *
  *         两种工况都由 Car_Drive() 统一处理, 不需要分支:
  *           原地转向: base = 0,   turn = 输出  -> 右轮正转/左轮反转, 原地打转
  *           边走边修: base = 转速, turn = 输出  -> 左右轮差速, 边跑边纠偏
  *           已经到位(base = 0 且 turn = 0)     -> 停车
  */
void Heading_ApplyOutput(void)
{
    static uint32_t last_cmd_tick = 0;
    uint32_t now;
    int16_t  base;
    int16_t  turn;

    /* ---- 处理菜单(模式选择页)发来的启动/停车请求 ---- */
    Heading_UpdateRunState();

    /* ---- 电机没启动: PD 照常计算, 但结果不下发 ----
     * 清掉 ready, 避免停止期间积压的结果在下次启动时被误发。 */
    if (!g_mode_run_request)
    {
        g_yaw_output_ready = 0;
        return;
    }

    /* 中断还没有算出新结果, 直接返回 */
    if (!g_yaw_output_ready)
    {
        return;
    }

    /* 限频: 上一次下发还没过 30ms 就先不发 */
    now = HAL_GetTick();
    if ((now - last_cmd_tick) < YAW_CMD_INTERVAL_MS)
    {
        return;
    }

    last_cmd_tick      = now;
    g_yaw_output_ready = 0;

    base = g_drive_base;
    turn = (int16_t)g_yaw_output;   /* float 转整数, 顺带滤掉小于 1 转的抖动 */

    if ((base == 0) && (turn == 0))
    {
        /* 原地转向模式 + 已经到位 -> 停车(只在第一次进入时发) */
        if (!s_motor_stopped)
        {
            s_motor_stopped = 1;
            Car_Stop();
        }
    }
    else
    {
        /* 麦轮: 前进 + 自转修正 */
        s_motor_stopped = 0;
        Car_Drive(base, turn);
    }
}

/**
  * @brief  启动电机(内部使用, 只能在主循环里调用)
  * @note   PD 运算本来就在中断里一直跑, 这里只准备电机启动参数。
  *         g_yaw_output_ready 置 1 是为了让主循环下一拍立刻把 PD 输出发出去。
  */
static void heading_do_start(void)
{
    g_yaw_target       = MODE_START_YAW_TARGET;
    g_drive_base       = MODE_START_DRIVE_BASE;
    g_yaw_output       = 0.0f;
    g_yaw_output_ready = 1;                    /* 强制下发一次初始命令 */
    s_motor_stopped    = 0;

    Car_Drive(g_drive_base, 0);
}

/**
  * @brief  停止电机(内部使用, 只能在主循环里调用)
  * @note   PD 运算不受影响, 仍会继续更新 g_yaw_output。
  */
static void heading_do_stop(void)
{
    g_drive_base       = 0;
    g_yaw_output       = 0.0f;
    g_yaw_output_ready = 0;
    s_motor_stopped    = 1;

    Car_Stop();
}

/**
  * @brief  处理菜单(模式选择页)发来的启动/停车请求(主循环里反复调用)
  * @note   和菜单模块之间靠 g_mode_run_request 这个标志通信:
  *             标志变化 -> 执行启动或停车
  *         上一状态记在模块内部, 所以反复调用不会重复动作,
  *         只在标志真正翻转的那一次才干活。
  */
void Heading_UpdateRunState(void)
{
    static uint8_t s_applied = 0;   /* 已经执行过的请求值(上电默认停车) */

    if (g_mode_run_request == s_applied)
    {
        return;                     /* 没有新请求 */
    }

    s_applied = g_mode_run_request;

    if (s_applied)
    {
        /* IMU 没读到数据时禁止启动:
         * 否则 yaw 一直是 0, 目标 90° 会让车开环一直转。
         * 把请求清回 0, 让菜单状态也回到 STOP。 */
        if (!jy901s_data.ready)
        {
            g_mode_run_request = 0;
            s_applied = 0;
            return;
        }

        heading_do_start();
    }
    else
    {
        heading_do_stop();
    }
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
