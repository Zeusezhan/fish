/* ============================================================
 * 无线串口 / VOFA+ 波形上报
 *
 * 【实现要点】
 *   1. 用 USART2 + DMA 发送, 全程非阻塞, 可以在定时器中断里调用。
 *   2. 帧格式是 VOFA+ 的 "FireWater": 逗号分隔的 ASCII 数字 + '\n'。
 *   3. 自己实现浮点转字符串, 不依赖 printf / sprintf,
 *      省掉 MicroLIB 和新库的浮点格式化开销, 中断里跑得更轻。
 *   4. 周期性启动 DMA 前必须等上一次发完, 否则返回 HAL_BUSY 丢帧。
 *
 * 【帧格式】(上行: 芯片 -> PC)
 *       "12.34,-5.60,4.00,0.35\n"
 *         ^通道0   ^通道1   ^通道2  ^通道3
 *         目标角   当前角    Kp      Kd
 * 【下行格式】(PC -> 芯片, 在线调 PI-D)
 *       "kp,kd\n"     例如 "6.0,0.4\n"    (数据发送框手打, 两个一起设)
 *       "kp,ki,kd\n"  例如 "6.0,0.3,0.4\n" (三个一起设)
 *       "P6.0\n"      只改 Kp            (VOFA+ 滑动条发 "P%f\n")
 *       "I0.3\n"      只改 Ki            (VOFA+ 滑动条发 "I%f\n")
 *       "D0.4\n"      只改 Kd            (VOFA+ 滑动条发 "D%f\n")
 * ============================================================ */

#include "wireless_uart.h"

/* ---- 闭环参数(定义在 main.c) ---- */
extern volatile float g_yaw_target;
extern volatile float g_yaw_kp;
extern volatile float g_yaw_ki;
extern volatile float g_yaw_kd;
#include "usart.h"
#include "jy901s.h"

/* ---- 上报通道数:
 *   0 = 目标航向, 1 = 当前航向, 2 = 当前 Kp, 3 = 当前 Kd, 4 = 航向误差
 *
 * 为什么要有误差通道: 调 PD 时最关心的是"误差收敛得快不快、有没有
 * 来回摆", 直接看误差曲线比看两条几乎重合的角度曲线直观得多。
 *
 * 说明: 目标/当前/Kp/Kd 之外再单独发误差, 是为了在 VOFA+ 里能
 * 把误差单独放大显示; 误差 = 目标 - 当前, 也可以在 VOFA+ 里用
 * 表达式算, 但占用一个通道更省事。 */
#define WIRELESS_CHANNELS       (5u)

/* ---- 单帧最大字节数 ----
 * 5 个浮点数(各最多约 8 字节) + 逗号 + 结束符, 64 字节绰绰有余 */
#define WIRELESS_FRAME_MAX      (64u)

/* ---- 下行(PC -> 芯片)命令缓冲 ----
 * 协议: "kp,kd\n", 例如 "6.0,0.4"。32 字节足够, 多出来的直接丢弃。 */
#define WIRELESS_RX_BUF_MAX     (32u)

/* ---- 小数位数: 2 位对应角度 0.01°, 分辨率足够看波形 ---- */
#define WIRELESS_DECIMALS       (2u)

/* ---- VOFA+ FireWater 要求的帧结束符 ---- */
#define WIRELESS_FRAME_END      '\n'

/* DMA 发送缓冲区: HAL_UART_Transmit_DMA 期间由 DMA 持续读取,
 * 必须一直有效, 所以放成静态全局, 不能是局部变量。 */
static uint8_t s_tx_buf[WIRELESS_FRAME_MAX];

/* 上一次上报的时间戳, 用于限频 */
static uint32_t s_last_report_tick = 0;

/* ---- 下行接收状态(PC -> 芯片) ---- */
static uint8_t  s_rx_byte;                     /* 单字节接收缓冲(中断写入) */
static char     s_rx_buf[WIRELESS_RX_BUF_MAX]; /* 一行命令缓冲 */
static uint8_t  s_rx_len = 0;                  /* 已收到的字节数 */

/**
  * @brief  无符号整数转十进制字符串
  * @param  buf  输出缓冲区
  * @param  val  待转换的值
  * @retval 写入的字符个数
  */
static uint8_t wireless_utoa(char *buf, uint32_t val)
{
    char    tmp[10];
    uint8_t len = 0;
    uint8_t i;

    if (val == 0u)
    {
        buf[0] = '0';
        return 1u;
    }

    while (val > 0u)
    {
        tmp[len++] = (char)('0' + (val % 10u));
        val /= 10u;
    }

    /* 上面是从低位往高位取的, 这里倒序写回 */
    for (i = 0u; i < len; i++)
    {
        buf[i] = tmp[len - 1u - i];
    }

    return len;
}

/**
  * @brief  把一个浮点数按 FireWater 格式追加进缓冲区
  * @param  buf       输出缓冲区
  * @param  pos       当前写入位置(输入输出)
  * @param  val       待转换的浮点数
  * @param  decimals  保留的小数位数
  * @note   buf 必须足够大: 最多 "-" + 10 位整数 + "." + decimals 位小数
  */
static void wireless_append_float(char *buf, uint8_t *pos, float val, uint8_t decimals)
{
    uint32_t scale    = 1u;
    uint32_t rounded;
    uint32_t int_part;
    uint32_t frac_part;
    uint8_t  i;

    for (i = 0u; i < decimals; i++)
    {
        scale *= 10u;
    }

    /* NaN / Inf 会让下面的强制转换行为未定义, 直接按 0 上报 */
    if ((val != val) || (val > 1.0e9f) || (val < -1.0e9f))
    {
        val = 0.0f;
    }

    if (val < 0.0f)
    {
        buf[(*pos)++] = '-';
        val = -val;
    }

    /* 四舍五入到 decimals 位小数, 再拆成整数部分和小数部分,
     * 全程整数运算, 避免浮点取整误差 */
    rounded   = (uint32_t)((val * (float)scale) + 0.5f);
    int_part  = rounded / scale;
    frac_part = rounded % scale;

    *pos += wireless_utoa(&buf[*pos], int_part);

    if (decimals > 0u)
    {
        buf[(*pos)++] = '.';

        /* 小数部分要补前导零, 例如 0.05 不能写成 0.5 */
        for (i = 0u; i < decimals; i++)
        {
            uint32_t divisor = 1u;
            uint8_t  j;

            for (j = 0u; j < (decimals - 1u - i); j++)
            {
                divisor *= 10u;
            }

            buf[(*pos)++] = (char)('0' + ((frac_part / divisor) % 10u));
        }
    }
}

/**
  * @brief  初始化无线串口上报模块
  */
void Wireless_Init(void)
{
    s_last_report_tick = HAL_GetTick();
    s_rx_len = 0;
    Wireless_StartReceive();
}

/**
  * @brief  启动下行接收(PC -> 芯片)
  * @note   用中断逐字节接收, 每收一个字节 HAL 会回调
  *          HAL_UART_RxCpltCallback(), 在回调里完成组行和解析。
  */
void Wireless_StartReceive(void)
{
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1u);
}

/**
  * @brief  解析一个浮点数字符串(不依赖 strtof, 中断里跑得轻)
  * @param  s     字符串
  * @param  len   字符串有效长度
  * @param  idx   解析起点(输入输出, 成功后指向数字后的位置)
  * @param  out   解析结果
  * @retval 1 成功, 0 失败(一个数字都没有)
  * @note   支持前导空格 / 正负号 / 小数点, 不支持科学计数法。
  */
static uint8_t wireless_parse_float(const char *s, uint8_t len, uint8_t *idx, float *out)
{
    uint8_t i      = *idx;
    uint8_t digits = 0u;
    float   sign   = 1.0f;
    float   int_part  = 0.0f;
    float   frac_part = 0.0f;
    float   frac_scale = 1.0f;

    while ((i < len) && ((s[i] == ' ') || (s[i] == '\t')))
    {
        i++;
    }

    if ((i < len) && ((s[i] == '-') || (s[i] == '+')))
    {
        if (s[i] == '-')
        {
            sign = -1.0f;
        }
        i++;
    }

    while ((i < len) && (s[i] >= '0') && (s[i] <= '9'))
    {
        int_part = (int_part * 10.0f) + (float)(s[i] - '0');
        i++;
        digits++;
    }

    if ((i < len) && (s[i] == '.'))
    {
        i++;
        while ((i < len) && (s[i] >= '0') && (s[i] <= '9'))
        {
            frac_scale *= 10.0f;
            frac_part   = (frac_part * 10.0f) + (float)(s[i] - '0');
            i++;
            digits++;
        }
    }

    if (digits == 0u)
    {
        return 0u;
    }

    *out = sign * (int_part + (frac_part / frac_scale));
    *idx = i;
    return 1u;
}

/**
  * @brief  参数限幅, 防止手滑发个超大值把车搞飞
  * @param  v   输入值
  * @param  hi  上限(下限固定为 0)
  */
static float wireless_clamp(float v, float hi)
{
    if (v < 0.0f)
    {
        return 0.0f;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

/**
  * @brief  解析一条下行命令, 解析成功就写进全局 PI-D 参数
  * @note   支持这几种写法:
  *             "kp,kd"   例如 "6.0,0.4"  两个一起设(数据发送框手打)
  *             "kp,ki,kd" 例如 "6.0,0.3,0.4" 三个一起设
  *             "P<值>"   例如 "P6.0"     只改 Kp(VOFA+ 滑动条发 "P%f\n")
  *             "I<值>"   例如 "I0.3"     只改 Ki(VOFA+ 滑动条发 "I%f\n")
  *             "D<值>"   例如 "D0.4"     只改 Kd(VOFA+ 滑动条发 "D%f\n")
  */
static void wireless_parse_frame(char *buf, uint8_t len)
{
    uint8_t idx = 0u;
    float   kp;
    float   ki;
    float   kd;

    buf[len] = '\0';    /* 补上字符串结尾, 方便下面的解析 */

    /* 跳过前导空白 */
    while ((idx < len) && ((buf[idx] == ' ') || (buf[idx] == '\t')))
    {
        idx++;
    }

    /* 单参数命令: P / I / D 开头 */
    if ((buf[idx] == 'P') || (buf[idx] == 'p'))
    {
        idx++;
        /* 允许 "P=6.0" / "P:6.0" 这类写法, 跳过分隔符 */
        while ((idx < len) && ((buf[idx] == '=') || (buf[idx] == ':') || (buf[idx] == ' ')))
        {
            idx++;
        }
        if (wireless_parse_float(buf, len, &idx, &kp))
        {
            g_yaw_kp = wireless_clamp(kp, 100.0f);
        }
        return;
    }

    if ((buf[idx] == 'I') || (buf[idx] == 'i'))
    {
        idx++;
        while ((idx < len) && ((buf[idx] == '=') || (buf[idx] == ':') || (buf[idx] == ' ')))
        {
            idx++;
        }
        if (wireless_parse_float(buf, len, &idx, &ki))
        {
            g_yaw_ki = wireless_clamp(ki, 10.0f);
        }
        return;
    }

    if ((buf[idx] == 'D') || (buf[idx] == 'd'))
    {
        idx++;
        while ((idx < len) && ((buf[idx] == '=') || (buf[idx] == ':') || (buf[idx] == ' ')))
        {
            idx++;
        }
        if (wireless_parse_float(buf, len, &idx, &kd))
        {
            g_yaw_kd = wireless_clamp(kd, 10.0f);
        }
        return;
    }

    /* 双参数命令: "kp,kd" */
    if (!wireless_parse_float(buf, len, &idx, &kp))
    {
        return;
    }

    /* 跳过数字后面的分隔符(逗号/空格) */
    while ((idx < len) && ((buf[idx] == ',') || (buf[idx] == ' ') || (buf[idx] == '\t')))
    {
        idx++;
    }

    if (!wireless_parse_float(buf, len, &idx, &kd))
    {
        return;
    }

    /* 三个值按 kp,ki,kd 解析, 两个值保持 kp,kd 兼容格式。 */
    ki = g_yaw_ki;
    if ((idx < len) && ((buf[idx] == ',') || (buf[idx] == ' ')))
    {
        uint8_t idx3 = idx;
        float third;

        while ((idx3 < len) && ((buf[idx3] == ',') || (buf[idx3] == ' ')
                                || (buf[idx3] == '\t')))
        {
            idx3++;
        }

        if (wireless_parse_float(buf, len, &idx3, &third))
        {
            ki = kd;
            kd = third;
        }
    }

    g_yaw_kp = wireless_clamp(kp, 100.0f);
    g_yaw_ki = wireless_clamp(ki, 10.0f);
    g_yaw_kd = wireless_clamp(kd, 10.0f);
}

/**
  * @brief  串口接收完成回调(HAL 弱函数, 这里覆盖实现)
  * @note   每收到 1 个字节进一次: 攒成一行, 遇到 '\n'/'\r' 就解析。
  *         只处理 USART2; 其它串口(USART1 步进电机)不受影响。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t ch;

    if (huart->Instance != USART2)
    {
        return;
    }

    ch = s_rx_byte;

    if ((ch == '\n') || (ch == '\r'))
    {
        /* 一帧结束, 有内容才解析(空行直接忽略) */
        if (s_rx_len > 0u)
        {
            wireless_parse_frame(s_rx_buf, s_rx_len);
            s_rx_len = 0u;
        }
    }
    else if ((ch >= ' ') && (ch <= '~'))
    {
        /* 只收可打印字符; 满了就先解析, 防止缓冲区溢出 */
        if (s_rx_len >= (WIRELESS_RX_BUF_MAX - 1u))
        {
            wireless_parse_frame(s_rx_buf, s_rx_len);
            s_rx_len = 0u;
        }
        s_rx_buf[s_rx_len++] = (char)ch;
    }

    /* 重新武装下一个字节 */
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1u);
}

/**
  * @brief  串口错误回调: 出错(如偶尔的帧错误)后重新武装接收,
  *         否则接收会停在错误态, 再也收不到新命令。
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1u);
    }
}

/**
  * @brief  上报"目标航向 + 当前航向"(供定时器中断回调调用)
  */
void Wireless_SendYaw(float target_yaw_deg, float current_yaw_deg)
{
    uint32_t now = HAL_GetTick();
    uint8_t  pos = 0u;

    /* ---- 限频: 没到上报周期就直接返回, 中断尽量短 ---- */
    if ((now - s_last_report_tick) < WIRELESS_REPORT_PERIOD_MS)
    {
        return;
    }

    /* ---- 上一次 DMA 还没发完就别启动新的 ----
     * 不检查的话 HAL_UART_Transmit_DMA 会直接返回 HAL_BUSY,
     * 那一帧就丢了, 波形上表现为随机掉点。
     * 正在发送时也 deliberately 不更新时间戳, 这样下一轮会尽快补上。 */
    if (huart2.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    s_last_report_tick = now;

    /* ---- 组帧: "目标,当前,Kp,Kd,误差\n" ----
     * 把当前 Kp/Kd 也回传, 拖动滑块后能立刻在波形上看到参数有没有生效;
     * 误差通道用来观察收敛速度和是否来回摆。 */
    wireless_append_float((char *)s_tx_buf, &pos, target_yaw_deg, WIRELESS_DECIMALS);
    s_tx_buf[pos++] = ',';
    wireless_append_float((char *)s_tx_buf, &pos, current_yaw_deg, WIRELESS_DECIMALS);
    s_tx_buf[pos++] = ',';
    wireless_append_float((char *)s_tx_buf, &pos, g_yaw_kp, WIRELESS_DECIMALS);
    s_tx_buf[pos++] = ',';
    wireless_append_float((char *)s_tx_buf, &pos, g_yaw_kd, WIRELESS_DECIMALS);
    s_tx_buf[pos++] = ',';
    wireless_append_float((char *)s_tx_buf, &pos,
                          target_yaw_deg - current_yaw_deg, WIRELESS_DECIMALS);
    s_tx_buf[pos++] = WIRELESS_FRAME_END;

    /* ---- 非阻塞发送, 立刻返回 ---- */
    (void)HAL_UART_Transmit_DMA(&huart2, s_tx_buf, (uint16_t)pos);
}

/**
  * @brief  上报航向误差(度) = 目标角 - 当前角
  */
void Wireless_SendYawError(float target_yaw_deg, float current_yaw_deg)
{
    Wireless_SendYaw(target_yaw_deg, current_yaw_deg);
}

/**
  * @brief  按当前实际状态上报(目标航向 + 当前航向)
  */
void Wireless_ReportYaw(void)
{
    /* 直接取全局变量, 所以本文件需要 extern 声明 g_yaw_target。
     * jy901s_data 由 jy901s.h 提供。 */
    Wireless_SendYaw(g_yaw_target, jy901s_data.yaw_continuous);
}
