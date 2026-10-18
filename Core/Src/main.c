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
#include "Crane.h"
#include "function.h"
#include "jy901s.h"
#include "OLED.h"
#include "key.h"
#include "Menu.h"
#include "wireless_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- 行程标定测试开关 ----
 * 0 = 关闭(正常运行完整程序)
 * 1 = 标定 z 轴(上下轴, 地址1)
 * 2 = 标定 xy 轴(前后轴, 地址2)
 * 标定时: 上电使能对应轴 -> 走 CRANE_CALIB_PULSE 个脉冲 -> 停。
 * 用直尺量出滑台实际位移(mm)后:
 *     PULSE_PER_MM = CRANE_CALIB_PULSE / 实测位移mm
 * 把结果填回 Crane.h 的 CRANE_*_PULSE_PER_MM。
 * ★ 标定完记得把 CRANE_CALIB_AXIS 改回 0。 */
#define CRANE_CALIB_AXIS    0
#define CRANE_CALIB_PULSE   6400u   /* 3200=1圈(16细分), 建议6400=2圈更准 */

/* ==================== 航向角 PD 闭环参数 ==================== */

/* PD 参数(上电默认值, 运行中可用 VOFA+ 实时修改):
 * 整定时先只调 Kp —— 从小往大加, 加到"转得够快但开始有点晃";
 * 然后再加 Kd 把晃动压下去。Kd 用的是陀螺仪角速度, 不需要自己差分。
 * 运行时值保存在 g_yaw_kp / g_yaw_kd(见 USER CODE PV), 通过无线串口
 * (USART2)发 "kp,kd\n" 即可修改, 例如在 VOFA+ 发送框里发 "6.0,0.4"。 */
#define YAW_PID_KP_DEFAULT      (4.0f)      /* 比例: 航向误差 1 度给多少转速 */
#define YAW_PID_KD_DEFAULT      (0.35f)     /* 微分: 阻尼项, 压住过冲 */

/* --- 原地转向模式的输出限幅(RPM) ---
 * 下限: 步进电机转速太低是转不动的, 给一个能转起来的最小值
 * 上限: 防止误差大时冲太快甩过头 */
#define YAW_PID_TURN_OUT_MIN    (20.0f)
#define YAW_PID_TURN_OUT_MAX    (80.0f)

/* --- 边走边修模式的输出限幅(RPM) ---
 * 这时的输出是叠加在前进速度上的"左右差速修正", 不能太大,
 * 否则车会甩尾。而且刻意不设下限 —— 误差小修正就小, 走起来才平顺。 */
#define YAW_PID_DRIVE_OUT_MAX   (60.0f)

/* 到位判定: 航向误差小于这个角度就认为到了, 输出直接归零 */
#define YAW_PID_TOLERANCE       (2.0f)

/* 电机命令下发间隔(ms):
 * fish_Step_All_Control() 内部有好几个 HAL_Delay(2), 一次调用要 8ms 以上,
 * 所以下发必须限频, 不能每 20ms 都发一次。 */
#define YAW_CMD_INTERVAL_MS     (30u)

/* 菜单"Mode Select"页按下 PE0 启动闭环时用的目标航向(度),
 * 相对 JY901S_ZeroYaw() 的清零点。
 *
 * 为什么设成 90° 而不是 0°:
 *   启动瞬间当前航向是 0°, 目标取 0° 的话"误差 = 0", 处在
 *   YAW_PID_TOLERANCE 容差内, 输出直接归零 —— 电机不会动, 看起来
 *   像"按了没反应"。取 90° 就会产生 90° 的误差, 启动后车会原地
 *   转到 90° 再停住, 最适合用来验证整条链路。 */
#define MODE_START_YAW_TARGET   (90.0f)

/* 启动后停车前的"前进基准速度(RPM)":
 *   0 = 原地靠 PD 保持航向(不前进, 最安全, 适合先验证航向保持)
 *   非 0 = 边走边修(车会持续前进, 测试前先把车架空或留出跑道)
 *
 * ★ base != 0 时不会自动停: 一边前进一边靠 PD 保持航向,
 *   除非按 PE0 停车, 否则会一直往前走。 */
#define MODE_START_DRIVE_BASE   (0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ==================== 航向角 PD 闭环的运行状态 ====================
 *
 * 分工:
 *   TIM3 中断(20ms) -> 只做 PD 运算, 结果存进 g_yaw_output
 *   主循环          -> 调用 Heading_ApplyOutput() 把结果下发给电机
 *
 * 两种工作模式(由 g_drive_base 区分):
 *   原地转向:  base = 0       (输出有下限, 保证步进电机转得动)
 *   边走边修:  base != 0     (输出是左右差速修正量, 叠加在 base 上)
 *
 * 为什么中断里不发电机命令:
 *   电机命令函数(Car_Drive / Car_Stop ...)内部有 HAL_Delay(),
 *   而 HAL_Delay 靠 SysTick 计时, SysTick 优先级(15)比 TIM3(5)低,
 *   在中断里等待 SysTick 会永久卡死。所以中断只算不发。
 * ================================================================= */

/* 闭环总开关: 1 = 定时器中断开始运算。
 * 上电默认 0, 车不会自己动。启动前先把 g_yaw_target / g_drive_base /
 * g_yaw_output 设好再置 1; 要停就清 0, 然后停车。 */
volatile uint8_t g_yaw_loop_enable = 0;

/* 闭环的"按键启动请求"标志(菜单和闭环之间的通信变量):
 *   Menu 模块(主循环)按下/再按下 PE0 时把它翻转,
 *   Heading_ApplyOutput()(也在主循环)看到变化后执行真正的启动/停车。
 *
 * 为什么要绕这一道: 启动/停车最后要调用 Car_Drive() / Car_Stop(),
 * 它们内部有 HAL_Delay(), 只能放在主循环里执行; 而且不直接跨模块
 * 写 g_yaw_loop_enable, 是为了避免和中断抢同一个变量。
 *   0 = 请求停车(上电默认), 1 = 请求启动 */
volatile uint8_t g_mode_run_request = 0;

/* 目标航向(度), 相对 JY901S_ZeroYaw() 的清零点 */
volatile float   g_yaw_target = 0.0f;

/* 可在线整定的 PD 参数: VOFA+ 发 "kp,kd\n" 即可实时修改。
 * 初值取上面的 YAW_PID_KP_DEFAULT / YAW_PID_KD_DEFAULT。
 * 中断里直接读, 主循环/接收回调里写, 都是单次 32 位读写, 安全。 */
volatile float   g_yaw_kp = YAW_PID_KP_DEFAULT;
volatile float   g_yaw_kd = YAW_PID_KD_DEFAULT;

/* 前进基准速度(RPM):
 *   = 0  -> 原地转向模式(输出有下限, 保证转得动)
 *   != 0 -> 边走边修模式(输出是差速修正量, 叠加在这个基准速度上) */
volatile int16_t g_drive_base = 0;

/* 中断算出来的输出(转向修正量, RPM):
 *   > 0 = 需要往左修正, < 0 = 需要往右修正, 绝对值 = 转速 */
volatile float   g_yaw_output = 0.0f;

/* 有新结果的标志: 中断算完置 1, 主循环下发完清 0 */
volatile uint8_t g_yaw_output_ready = 0;

/* 上一次下发给电机的 (基准速度, 修正量), 主循环专用, 中断不碰 */
static int16_t s_last_base = 0;
static int16_t s_last_turn = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Heading_ApplyOutput(void);                                 /* 主循环调用: 下发输出 */
void Heading_UpdateRunState(void);                              /* 主循环调用: 执行启动/停车请求 */

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* 初始化 OLED(软件I2C, PD10=SCL, PD8=SDA), 标定时也用来显示提示 */
  OLED_Init();
  OLED_Clear();

#if (CRANE_CALIB_AXIS == 0)
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
#endif /* CRANE_CALIB_AXIS == 0 */

  /* 使能塔吊对应轴(标定/正常都用) */
#if (CRANE_CALIB_AXIS == 1)
  Crane_Vert_Init();                        /* 标定 z 轴: 使能上下轴(地址1) */
#elif (CRANE_CALIB_AXIS == 2)
  Crane_Hori_Init();                        /* 标定 xy 轴: 使能前后轴(地址2) */
#else
  /* 正常模式: 上电使能上下轴(竖直, 地址1), 只使能不动作。
   * 前后轴(地址2)待配合抓取功能时再启用。 */
  Crane_Vert_Init();
#endif

  /* ---- 行程标定: 走固定脉冲数, 量实际位移 ---- */
#if (CRANE_CALIB_AXIS == 1)
  OLED_ShowString(0, 0, "Calib Z...");
  OLED_Refresh();
  HAL_Delay(1000);                          /* 等驱动器稳定 */
  Crane_Vert_MovePulse(CRANE_CALIB_PULSE);  /* z 轴走 CRANE_CALIB_PULSE 个脉冲 */
  HAL_Delay(3000);                          /* 等它走完, 再量位移 */
#elif (CRANE_CALIB_AXIS == 2)
  OLED_ShowString(0, 0, "Calib XY...");
  OLED_Refresh();
  HAL_Delay(1000);                          /* 等驱动器稳定 */
  Crane_Hori_MovePulse(CRANE_CALIB_PULSE);  /* xy 轴走 CRANE_CALIB_PULSE 个脉冲 */
  HAL_Delay(3000);                          /* 等它走完, 再量位移 */
#endif

  /* 启动 TIM3 更新中断(20ms 一次), 航向角 PD 运算在中断回调里做。
   * CubeMX 不会自动生成这一句, 不加的话中断永远不会进来。 */
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if (CRANE_CALIB_AXIS == 0)
    /* I2C 轮询读取 JY901S 姿态数据(内部按 20ms 间隔控制频率) */
    JY901S_Poll();

    /* 航向闭环输出下发: 定时器中断只负责"算", 电机命令在这里"发"
     * (内部限频 30ms, 没有新结果时不发) */
    Heading_ApplyOutput();

    /* 按键轮询检测(内部按 10ms 间隔采样并消抖) */
    Key_Scan();

    /* OLED 菜单: 内部处理按键(切换页面/移动光标)并绘制当前页面,
     * 显示按 100ms 限频刷新。显示内容全部由 Menu.c 负责。 */
    Menu_Run();
#endif /* CRANE_CALIB_AXIS == 0 */
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
  *           限幅后存进 g_yaw_output, 由主循环下发
  *
  *         ★ 中断里绝对不能调用 HAL_Delay() 和任何内部用到它的函数
  *           (Car_Drive / Car_Stop / fish_Step_All_Control ...)。
  *           因为 HAL_Delay 靠 SysTick 累加计时, 而 SysTick 优先级是 15
  *           (最低), 比 TIM3 的 5 低, 一旦在中断里等 SysTick 就永久卡死。
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

    /* ---- 2. 到位判定 ----
     * 误差落在容差内就认为到了, 输出归零;
     * 否则会因为"输出下限"一直在目标点附近来回抖 */
    if ((err <= YAW_PID_TOLERANCE) && (err >= -YAW_PID_TOLERANCE))
    {
        g_yaw_output = 0.0f;
    }
    else
    {
        /* ---- 3. PD 运算 ----
         * 比例项: 误差越大转得越快
         * 微分项: 直接用陀螺仪角速度做阻尼, 不用自己差分, 没有噪声放大。
         *         gz > 0 表示车正在左转, 减去它就能提前减速, 防止冲过头 */
        u = (g_yaw_kp * err) - (g_yaw_kd * jy901s_data.gz);

        /* ---- 4. 限幅 ----
         * 原地转向: 必须给下限, 转速太低步进电机根本转不起来
         * 边走边修: 不要下限 —— 误差小修正就小, 走起来才平顺不摇摆 */
        if (g_drive_base == 0)
        {
            out_max = YAW_PID_TURN_OUT_MAX;
            out_min = YAW_PID_TURN_OUT_MIN;
        }
        else
        {
            out_max = YAW_PID_DRIVE_OUT_MAX;
            out_min = 0.0f;
        }

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

    /* 告诉主循环"有新结果可以下发了" */
    g_yaw_output_ready = 1;
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
  *
  *         另外还顺手做了 VOFA+ 的波形上报(见函数末尾)。
  *         上报放在这里而不是中断里, 是为了和闭环开关解耦:
  *         不管闭环开没开, 主循环一直在跑, 波形就一直有。
  */
void Heading_ApplyOutput(void)
{
    static uint32_t last_cmd_tick = 0;
    uint32_t now;
    int16_t  base;
    int16_t  turn;

    /* ---- 处理菜单(模式选择页)发来的启动/停车请求 ----
     * 放在最前面, 这样按下 PE0 后本周期就能把电机命令发出去。
     * 内部自带状态记忆, 没有新请求时开销极小。 */
    Heading_UpdateRunState();

    /* ---- VOFA+ 波形上报 ----
     * 内部自带周期限频。放在这里而不是中断里, 是为了和闭环开关解耦:
     * 不管闭环开没开, 波形一直有, 方便先验证 IMU 和接线。 */
    Wireless_ReportYaw();

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

    /* 和上次下发的完全一样, 不用重复发 */
    if ((base == s_last_base) && (turn == s_last_turn))
    {
        return;
    }
    s_last_base = base;
    s_last_turn = turn;

    if ((base == 0) && (turn == 0))
    {
        /* 原地转向模式 + 已经到位 -> 停车 */
        Car_Stop();
    }
    else
    {
        /* 麦轮: 前进 + 自转修正 */
        Car_Drive(base, turn);
    }
}

/**
  * @brief  执行"启动航向闭环"(内部使用, 只能在主循环里调用)
  * @note   开中断之前先把参数和输出全部准备好, 避免中断读到半套状态。
  *         g_yaw_output_ready 置 1 是为了让下面的 Car_Drive() 立刻
  *         把初始速度发出去 —— 否则输出恰好等于上次发过的值时会跳过。
  */
static void heading_do_start(void)
{
    g_yaw_loop_enable  = 0;                    /* 先关中断运算, 改参数 */
    g_yaw_target       = MODE_START_YAW_TARGET;
    g_drive_base       = MODE_START_DRIVE_BASE;
    g_yaw_output       = 0.0f;
    s_last_base        = 0;
    s_last_turn        = 0;
    g_yaw_output_ready = 1;                    /* 强制下发一次初始命令 */
    g_yaw_loop_enable  = 1;                    /* 开闸, 中断开始 PD 运算 */

    Car_Drive(g_drive_base, 0);
}

/**
  * @brief  执行"停止航向闭环并停车"(内部使用, 只能在主循环里调用)
  */
static void heading_do_stop(void)
{
    g_yaw_loop_enable  = 0;
    g_drive_base       = 0;
    g_yaw_output       = 0.0f;
    g_yaw_output_ready = 0;
    s_last_base        = 0;
    s_last_turn        = 0;

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
