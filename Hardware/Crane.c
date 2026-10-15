#include "main.h"      /* HAL_Delay 等 */
#include "usart.h"     /* huart3 */
#include <stdbool.h>
#include "Crane.h"

/*
 * USART3 指令发送层：把电机驱动器的指令帧通过 DMA 发出。
 * DMA 发送是非阻塞的，发送完成前仍会访问缓冲区，因此缓冲区必须
 * 保存在静态存储区，不能使用函数返回后就失效的局部数组。
 */

/* 发送电机使能/关闭指令。state=true 为使能，snF=false 表示不要求驱动器回传应答。 */
static void Crane_Tx_En(uint8_t addr, bool state, bool snF)
{
    static uint8_t cmd[6];

    cmd[0] = addr;
    cmd[1] = 0xF3;
    cmd[2] = 0xAB;
    cmd[3] = (uint8_t)state;
    cmd[4] = snF;
    cmd[5] = 0x6B;

    HAL_UART_Transmit_DMA(&huart3, cmd, 6);
}

/*
 * 发送位置运动指令。
 * dir 为方向，vel 为速度，acc 为加速度参数，clk 为运动脉冲数；
 * raF=true 表示绝对位置，raF=false 表示相对位移，snF=false 表示不请求应答。
 */
static void Crane_Tx_Pos(uint8_t addr, uint8_t dir, uint16_t vel,
                         uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    static uint8_t cmd[13];

    cmd[0]  = addr;
    cmd[1]  = 0xFD;
    cmd[2]  = dir;
    cmd[3]  = (uint8_t)(vel >> 8);
    cmd[4]  = (uint8_t)(vel >> 0);
    cmd[5]  = acc;
    cmd[6]  = (uint8_t)(clk >> 24);
    cmd[7]  = (uint8_t)(clk >> 16);
    cmd[8]  = (uint8_t)(clk >> 8);
    cmd[9]  = (uint8_t)(clk >> 0);
    cmd[10] = raF;
    cmd[11] = snF;
    cmd[12] = 0x6B;

    HAL_UART_Transmit_DMA(&huart3, cmd, 13);
}

/* 发送立即停止指令；snF=false 表示不请求驱动器应答。 */
static void Crane_Tx_Stop(uint8_t addr, bool snF)
{
    static uint8_t cmd[5];

    cmd[0] = addr;
    cmd[1] = 0xFE;
    cmd[2] = 0x98;
    cmd[3] = snF;
    cmd[4] = 0x6B;

    HAL_UART_Transmit_DMA(&huart3, cmd, 5);
}

/* 发送回原点指令；o_mode 选择驱动器的回零方式。 */
static void Crane_Tx_Origin(uint8_t addr, uint8_t o_mode, bool snF)
{
    static uint8_t cmd[5];

    cmd[0] = addr;
    cmd[1] = 0x9A;
    cmd[2] = o_mode;
    cmd[3] = snF;
    cmd[4] = 0x6B;

    HAL_UART_Transmit_DMA(&huart3, cmd, 5);
}

/* 将毫米位置限制在有效行程 [0, max_mm]，避免下发超出配置行程的目标。 */
static int32_t Crane_Clamp_mm(int32_t mm, int32_t max_mm)
{
    if (mm < 0)     { mm = 0; }
    if (mm > max_mm){ mm = max_mm; }
    return mm;
}

/* 将毫米换算为驱动器需要的脉冲数；先限制行程，再按脉冲/毫米系数四舍五入。 */
static uint32_t Crane_MmToPulse(int32_t mm, int32_t max_mm, float pulse_per_mm)
{
    mm = Crane_Clamp_mm(mm, max_mm);
    return (uint32_t)((float)mm * pulse_per_mm + 0.5f);
}

/*
 * 上下轴最近一次记录的目标位置（毫米），不是电机反馈的实际位置。
 * USART3 当前仅用于发送，没有解析驱动器的位置回传，因此 GetPos 返回此记录值。
 */
static int32_t s_vert_target_mm = 0;

/* 使能上下轴电机，并将软件记录的位置初始化为 0；不会执行机械回零。 */
void Crane_Vert_Init(void)
{
    Crane_Tx_En(CRANE_VERT_ADDR, true, false);
    HAL_Delay(2);
    s_vert_target_mm = 0;
}

/* 命令上下轴移动到绝对位置（毫米）；目标会被限制在 [0, 最大行程]。 */
void Crane_Vert_MoveTo(int32_t pos_mm)
{
    pos_mm = Crane_Clamp_mm(pos_mm, CRANE_VERT_MAX_MM);

    Crane_Tx_Pos(CRANE_VERT_ADDR, CRANE_VERT_DIR_POS, CRANE_VERT_RPM,
                 CRANE_VERT_ACC,
                 Crane_MmToPulse(pos_mm, CRANE_VERT_MAX_MM, CRANE_VERT_PULSE_PER_MM),
                 true, false);   /* raF=true: 绝对位置 */

    s_vert_target_mm = pos_mm;
}

/*
 * 命令上下轴相对当前位置移动 delta_mm 毫米。
 * 根据限幅后的目标计算真实位移、方向和脉冲数；若位移为零则不发送指令。
 * 当前位置以软件记录的目标为基准，不代表实时测量值。
 */
void Crane_Vert_MoveRel(int32_t delta_mm)
{
    int32_t  target, real_delta;
    uint8_t  dir;
    uint32_t pulse;

    if (delta_mm == 0) { return; }

    target = Crane_Clamp_mm(s_vert_target_mm + delta_mm, CRANE_VERT_MAX_MM);
    real_delta = target - s_vert_target_mm;
    if (real_delta == 0) { return; }

    dir   = (real_delta > 0) ? CRANE_VERT_DIR_POS : (uint8_t)(!CRANE_VERT_DIR_POS);
    pulse = (uint32_t)((float)(real_delta > 0 ? real_delta : -real_delta)
                       * CRANE_VERT_PULSE_PER_MM + 0.5f);

    Crane_Tx_Pos(CRANE_VERT_ADDR, dir, CRANE_VERT_RPM,
                 CRANE_VERT_ACC, pulse, false, false);  /* raF=false: 相对 */

    s_vert_target_mm = target;
}

/* 向上下轴驱动器发送停止指令。 */
void Crane_Vert_Stop(void)
{
    Crane_Tx_Stop(CRANE_VERT_ADDR, false);
}

/*
 * 请求上下轴按驱动器的模式 3 回原点（单圈限位开关回零）。
 * 需要将限位开关接到驱动器原点输入；此函数不会等待回零完成，
 * 也不会更新软件记录的位置。
 */
void Crane_Vert_Home(void)
{
    Crane_Tx_Origin(CRANE_VERT_ADDR, 3, false);
}

/* 返回上下轴最近一次记录的目标位置（毫米），不是编码器/驱动器实测位置。 */
int32_t Crane_Vert_GetPos_mm(void)
{
    return s_vert_target_mm;
}

/* 前后轴最近一次记录的目标位置（毫米），不代表电机反馈的实际位置。 */
static int32_t s_hori_target_mm = 0;

/* 使能前后轴电机，并将软件记录的位置初始化为 0；不会执行机械回零。 */
void Crane_Hori_Init(void)
{
    Crane_Tx_En(CRANE_HORI_ADDR, true, false);
    HAL_Delay(2);
    s_hori_target_mm = 0;
}

/* 命令前后轴移动到绝对位置（毫米）；目标会被限制在 [0, 最大行程]。 */
void Crane_Hori_MoveTo(int32_t pos_mm)
{
    pos_mm = Crane_Clamp_mm(pos_mm, CRANE_HORI_MAX_MM);

    Crane_Tx_Pos(CRANE_HORI_ADDR, CRANE_HORI_DIR_POS, CRANE_HORI_RPM,
                 CRANE_HORI_ACC,
                 Crane_MmToPulse(pos_mm, CRANE_HORI_MAX_MM, CRANE_HORI_PULSE_PER_MM),
                 true, false);

    s_hori_target_mm = pos_mm;
}

/*
 * 命令前后轴相对当前位置移动 delta_mm 毫米。
 * 根据限幅后的目标计算真实位移、方向和脉冲数；若位移为零则不发送指令。
 * 当前位置以软件记录的目标为基准，不代表实时测量值。
 */
void Crane_Hori_MoveRel(int32_t delta_mm)
{
    int32_t  target, real_delta;
    uint8_t  dir;
    uint32_t pulse;

    if (delta_mm == 0) { return; }

    target = Crane_Clamp_mm(s_hori_target_mm + delta_mm, CRANE_HORI_MAX_MM);
    real_delta = target - s_hori_target_mm;
    if (real_delta == 0) { return; }

    dir   = (real_delta > 0) ? CRANE_HORI_DIR_POS : (uint8_t)(!CRANE_HORI_DIR_POS);
    pulse = (uint32_t)((float)(real_delta > 0 ? real_delta : -real_delta)
                       * CRANE_HORI_PULSE_PER_MM + 0.5f);

    Crane_Tx_Pos(CRANE_HORI_ADDR, dir, CRANE_HORI_RPM,
                 CRANE_HORI_ACC, pulse, false, false);

    s_hori_target_mm = target;
}

/* 向前后轴驱动器发送停止指令。 */
void Crane_Hori_Stop(void)
{
    Crane_Tx_Stop(CRANE_HORI_ADDR, false);
}

/*
 * 请求前后轴按驱动器的模式 3 回原点（单圈限位开关回零）。
 * 需要将限位开关接到驱动器原点输入；此函数不会等待回零完成，
 * 也不会更新软件记录的位置。
 */
void Crane_Hori_Home(void)
{
    Crane_Tx_Origin(CRANE_HORI_ADDR, 3, false);
}

/* 返回前后轴最近一次记录的目标位置（毫米），不是编码器/驱动器实测位置。 */
int32_t Crane_Hori_GetPos_mm(void)
{
    return s_hori_target_mm;
}
