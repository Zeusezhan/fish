/* ============================================================
 * 麦轮底盘执行器层
 *
 * 【Emm_V5 帧格式速查】(USART1, 115200 8N1, 四台共用一条总线)
 *
 *   使能       addr  F3  AB  en  00  6B                      (6 字节)
 *   速度模式   addr  F6  dir vH  vL  acc snF 6B               (8 字节)
 *   立即停止   addr  FE  98  snF 6B                           (5 字节)
 *   广播同步   00    FF  66  6B                               (4 字节)
 *   读位置     addr  36  6B                                   (3 字节)
 *     应答     addr  36  sign v3 v2 v1 v0 6B                  (8 字节)
 *   位置清零   addr  0A  6D  6B                               (4 字节)
 *
 *   dir : 0 = CW, 1 = CCW         acc : 加减速档位 0~255
 *   snF : 0 = 立即执行, 1 = 缓存等待广播同步
 *   每帧最后一个字节 0x6B 是校验字节, 不是累加和。
 *
 * 【发送调度】
 *   所有帧先进环形队列, 由 Car_TxTask() 逐帧用 DMA 发出。
 *   这样做同时解决三件事:
 *     1. 不会出现 HAL_BUSY 丢帧(全工程只有一个发送者)
 *     2. 主循环不会被串口阻塞(原来一次 Car_Mecanum 要阻塞 4~8ms)
 *     3. 帧与帧之间有确定的空闲间隔, 给驱动器留出解析时间
 * ============================================================ */

#include "main.h"
#include "usart.h"
#include "function.h"
#include <string.h>

/* ==================== 可调参数 ==================== */

/* 加减速档位(0~255), 越大加速越急 */
#define CAR_ACC                 5u

/* 驱动器允许的最高转速(RPM) */
#define CAR_MAX_RPM             5000

/* 轮子方向: Emm_V5 的 dir 字段, 0 = CW, 1 = CCW */
#define WHEEL_DIR_FORWARD       1u
#define WHEEL_DIR_BACKWARD      0u

/* 电机物理安装方向翻转表: 1 = 该轮安装方向与其它轮相反。
 * 实测左侧两个轮子(3、4 号)反向安装, 所以整体取反。
 * ★ 这张表和 Car_Mecanum() 里的 speed[] 顺序必须配套,
 *   只改一边必然跑偏。 */
#define MOTOR_1_DIR_INVERT      0u      /* 右前 */
#define MOTOR_2_DIR_INVERT      0u      /* 右后 */
#define MOTOR_3_DIR_INVERT      1u      /* 左前 */
#define MOTOR_4_DIR_INVERT      1u      /* 左后 */

/* ==================== 发送队列 ==================== */

/* 队列深度。一次最长的突发是停车(8 帧), 留足余量。 */
#define CAR_TX_QUEUE_LEN        16u

/* 单帧最大字节数 */
#define CAR_TX_FRAME_MAX        16u

/* 帧间最小间隔(ms)。驱动器解析一帧需要时间, 背靠背连着发容易丢帧。 */
#define CAR_FRAME_GAP_MS        1u

/* 读位置的超时预算: 每字节 / 整体 */
#define CAR_POS_BYTE_TIMEOUT_MS 5u
#define CAR_POS_TOTAL_MS        40u

typedef struct
{
    uint8_t len;
    uint8_t data[CAR_TX_FRAME_MAX];
} CarTxFrame_t;

static CarTxFrame_t      s_txq[CAR_TX_QUEUE_LEN];
static uint8_t           s_tx_head = 0u;    /* 生产者(主循环)写 */
static uint8_t           s_tx_tail = 0u;    /* 消费者(主循环)写 */
static volatile uint8_t  s_tx_inflight = 0u;/* 1 = DMA 正在发, 完成中断清 0 */
static volatile uint8_t  s_tx_finished = 0u;/* 1 = 上一帧发完, 待主循环回收 */
static uint32_t          s_tx_gap_tick = 0u;/* 上一帧完成时刻, 用来控制帧间隔 */
static uint32_t          s_tx_errors = 0u;  /* 队列满 / DMA 启动失败 */

/* ==================== 队列内部函数 ==================== */

/**
  * @brief  把一帧塞进发送队列
  * @retval 1 = 入队成功, 0 = 队列满或长度非法(已计入 s_tx_errors)
  * @note   只有主循环会调用它, 所以不需要临界区。
  */
static uint8_t car_tx_push(const uint8_t *data, uint8_t len)
{
    uint8_t next = (uint8_t)((s_tx_head + 1u) % CAR_TX_QUEUE_LEN);

    if ((data == NULL) || (len == 0u) || (len > CAR_TX_FRAME_MAX)
        || (next == s_tx_tail))
    {
        s_tx_errors++;
        return 0u;
    }

    memcpy(s_txq[s_tx_head].data, data, len);
    s_txq[s_tx_head].len = len;
    s_tx_head = next;

    return 1u;
}

/**
  * @brief  DMA 发送完成中断(覆盖 HAL 的弱函数)
  * @note   这里只置两个标志, 队列指针一律由主循环维护,
  *         所以发送路径上不需要任何临界区。
  *         ★ 必须过滤 USART1: 无线串口(USART2)也用 DMA 发送。
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        s_tx_inflight = 0u;
        s_tx_finished = 1u;
    }
}

void Car_TxTask(void)
{
    uint32_t now;

    /* 上一帧还没发完, 什么都不做 */
    if (s_tx_inflight != 0u)
    {
        return;
    }

    /* 回收上一帧: 只有回收之后, 那个槽位才允许被生产者复用 */
    if (s_tx_finished != 0u)
    {
        s_tx_finished = 0u;
        s_tx_tail = (uint8_t)((s_tx_tail + 1u) % CAR_TX_QUEUE_LEN);
        s_tx_gap_tick = HAL_GetTick();
    }

    /* 队列空 */
    if (s_tx_tail == s_tx_head)
    {
        return;
    }

    /* 帧间隔: 给驱动器留出解析上一帧的时间。
     * s_tx_gap_tick 初值为 0, 上电后 (now - 0) 必然超过间隔, 首帧不受影响。 */
    now = HAL_GetTick();
    if ((uint32_t)(now - s_tx_gap_tick) < CAR_FRAME_GAP_MS)
    {
        return;
    }

    if (HAL_UART_Transmit_DMA(&huart1, s_txq[s_tx_tail].data,
                              s_txq[s_tx_tail].len) == HAL_OK)
    {
        s_tx_inflight = 1u;
    }
    else
    {
        /* 启动失败: 丢掉这一帧, 继续下一帧, 不能卡死队列 */
        s_tx_errors++;
        s_tx_tail = (uint8_t)((s_tx_tail + 1u) % CAR_TX_QUEUE_LEN);
    }
}

uint8_t Car_TxBusy(void)
{
    if ((s_tx_inflight != 0u) || (s_tx_finished != 0u))
    {
        return 1u;
    }

    return (s_tx_tail != s_tx_head) ? 1u : 0u;
}

uint8_t Car_TxFlush(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (Car_TxBusy() != 0u)
    {
        Car_TxTask();

        if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms)
        {
            return 0u;
        }
    }

    return 1u;
}

uint32_t Car_TakeTxErrors(void)
{
    uint32_t v = s_tx_errors;

    s_tx_errors = 0u;
    return v;
}

/* ==================== 帧构造 ==================== */

static uint8_t car_send_enable(uint8_t addr, uint8_t en)
{
    uint8_t f[6];

    f[0] = addr;
    f[1] = 0xF3;
    f[2] = 0xAB;
    f[3] = en;
    f[4] = 0x00;
    f[5] = 0x6B;

    return car_tx_push(f, 6u);
}

static uint8_t car_send_vel(uint8_t addr, uint8_t dir, uint16_t vel)
{
    uint8_t f[8];

    f[0] = addr;
    f[1] = 0xF6;
    f[2] = dir;
    f[3] = (uint8_t)(vel >> 8);
    f[4] = (uint8_t)(vel & 0x00FFu);
    f[5] = CAR_ACC;
    f[6] = 1u;              /* snF=1: 先缓存, 等广播同步一起启动 */
    f[7] = 0x6B;

    return car_tx_push(f, 8u);
}

static uint8_t car_send_stop_immediate(uint8_t addr)
{
    uint8_t f[5];

    f[0] = addr;
    f[1] = 0xFE;
    f[2] = 0x98;
    f[3] = 0u;              /* snF=0: 收到即停, 不依赖同步帧 */
    f[4] = 0x6B;

    return car_tx_push(f, 5u);
}

static uint8_t car_send_sync(void)
{
    uint8_t f[4];

    f[0] = 0x00;            /* 地址 0x00 = 广播 */
    f[1] = 0xFF;
    f[2] = 0x66;
    f[3] = 0x6B;

    return car_tx_push(f, 4u);
}

/* ==================== 对外接口 ==================== */

void Car_Mecanum(int16_t vx, int16_t vy, int16_t omega)
{
    static const uint8_t dir_invert[CAR_MOTOR_NUM] =
    {
        MOTOR_1_DIR_INVERT,
        MOTOR_2_DIR_INVERT,
        MOTOR_3_DIR_INVERT,
        MOTOR_4_DIR_INVERT
    };

    int32_t  speed[CAR_MOTOR_NUM];
    uint8_t  i;

    /* 麦轮逆运动学(x 前进, y 左移, omega 逆时针为正):
     *
     *   1 右前 = vx + vy + omega
     *   2 右后 = vx - vy + omega
     *   3 左前 = vx - vy - omega
     *   4 左后 = vx + vy - omega
     *
     * 自洽性检查: 纯 omega 时右侧两轮前进、左侧两轮后退 -> 左转;
     *             纯 vy 时右前/左后前进、右后/左前后退 -> 整车左移。 */
    speed[0] = (int32_t)vx + (int32_t)vy + (int32_t)omega;   /* 右前 */
    speed[1] = (int32_t)vx - (int32_t)vy + (int32_t)omega;   /* 右后 */
    speed[2] = (int32_t)vx - (int32_t)vy - (int32_t)omega;   /* 左前 */
    speed[3] = (int32_t)vx + (int32_t)vy - (int32_t)omega;   /* 左后 */

    for (i = 0u; i < CAR_MOTOR_NUM; i++)
    {
        uint8_t  dir;
        uint32_t mag;

        if (speed[i] >= 0)
        {
            dir = (dir_invert[i] != 0u) ? WHEEL_DIR_BACKWARD : WHEEL_DIR_FORWARD;
            mag = (uint32_t)speed[i];
        }
        else
        {
            dir = (dir_invert[i] != 0u) ? WHEEL_DIR_FORWARD : WHEEL_DIR_BACKWARD;
            mag = (uint32_t)(-speed[i]);
        }

        if (mag > CAR_MAX_RPM)
        {
            mag = CAR_MAX_RPM;
        }

        (void)car_send_vel((uint8_t)(i + 1u), dir, (uint16_t)mag);
    }

    /* 四台都用 snF=1 缓存起来了, 最后一条广播同步让它们同时启动 ——
     * 启动必须同时, 否则车会先歪一下再走。 */
    (void)car_send_sync();
}

void Car_Stop(void)
{
    uint8_t round;
    uint8_t i;

    /* 只发"立即停止"(snF=0), 而且完全不发广播同步帧。
     *
     * ★ 为什么不能靠同步帧停车
     *   缓存式停止要求驱动器"先收到自己那帧停止、再收到同步帧"。
     *   只要任意一帧在驱动器侧丢了, 它的缓存里就还是上一拍的速度命令,
     *   随后到来的同步帧会把那条**旧速度重新执行** —— 那个轮子反而
     *   被"唤醒", 继续按原速度跑。这就是"1 号电机偶尔停不下来"的
     *   一个确定机制, 而且是间歇性的, 极难复现。
     *
     *   立即停止帧收到即停, 和同步帧没有任何关系。
     *
     * 两轮冗余: 第一轮万一某台丢了, 第二轮补上。停止命令是幂等的,
     * 重复发没有副作用。 */
    for (round = 0u; round < 2u; round++)
    {
        for (i = 1u; i <= CAR_MOTOR_NUM; i++)
        {
            (void)car_send_stop_immediate(i);
        }
    }
}

void Car_StopMotor(uint8_t motor_id)
{
    if ((motor_id < 1u) || (motor_id > CAR_MOTOR_NUM))
    {
        return;
    }

    (void)car_send_stop_immediate(motor_id);
    (void)car_send_stop_immediate(motor_id);
}

uint8_t Car_SetEnabled(uint8_t enabled)
{
    uint8_t i;
    uint8_t ok = 1u;

    for (i = 1u; i <= CAR_MOTOR_NUM; i++)
    {
        if (car_send_enable(i, (enabled != 0u) ? 1u : 0u) == 0u)
        {
            ok = 0u;
        }
    }

    return ok;
}

uint8_t Car_ResetPositionCounts(void)
{
    uint8_t f[4];
    uint8_t i;
    uint8_t ok = 1u;

    for (i = 1u; i <= CAR_MOTOR_NUM; i++)
    {
        f[0] = i;
        f[1] = 0x0A;
        f[2] = 0x6D;
        f[3] = 0x6B;

        if (car_tx_push(f, 4u) == 0u)
        {
            ok = 0u;
        }
    }

    return ok;
}

/**
  * @brief  清空底盘串口接收缓冲里的残留字节
  * @note   驱动器对每条命令都会回一帧 ACK(addr + 0x02 + status + 0x6B),
  *         连发十几条之后 RX 里会堆一堆 ACK。读位置前先冲掉,
  *         否则搜索应答帧会一直撞在 ACK 上。
  */
static void car_uart_flush_rx(void)
{
    uint8_t guard = 0u;

    while ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET) && (guard < 64u))
    {
        (void)huart1.Instance->DR;      /* 读 DR 即清 RXNE */
        guard++;
    }

    __HAL_UART_CLEAR_OREFLAG(&huart1);
}

uint8_t Car_ReadPositionCount(uint8_t motor_id, int32_t *position)
{
    uint8_t  f[3];
    uint8_t  b;
    uint8_t  prev = 0u;
    uint8_t  tail[6];
    uint8_t  found = 0u;
    uint8_t  i;
    uint32_t start_tick;
    uint32_t magnitude;

    if ((motor_id < 1u) || (motor_id > CAR_MOTOR_NUM) || (position == NULL))
    {
        return 0u;
    }

    /* 先把队列里积压的帧发完并冲掉它们产生的 ACK,
     * 这样下面收到的第一个有效帧就是本次查询的应答。 */
    (void)Car_TxFlush(CAR_POS_TOTAL_MS);
    car_uart_flush_rx();

    f[0] = motor_id;
    f[1] = 0x36;
    f[2] = 0x6B;

    if (car_tx_push(f, 3u) == 0u)
    {
        return 0u;
    }

    /* 查询帧真的发出去之后再开始收 */
    if (Car_TxFlush(CAR_POS_TOTAL_MS) == 0u)
    {
        return 0u;
    }

    start_tick = HAL_GetTick();

    /* ---- 用"addr 后紧跟 0x36"做滑窗同步, 逐字节跳过 ACK 与噪声 ---- */
    for (i = 0u; i < 64u; i++)
    {
        if ((uint32_t)(HAL_GetTick() - start_tick) > CAR_POS_TOTAL_MS)
        {
            return 0u;
        }

        if (HAL_UART_Receive(&huart1, &b, 1u, CAR_POS_BYTE_TIMEOUT_MS) != HAL_OK)
        {
            return 0u;
        }

        if ((prev == motor_id) && (b == 0x36u))
        {
            found = 1u;
            break;
        }

        prev = b;
    }

    if (found == 0u)
    {
        return 0u;
    }

    /* ---- 收 6 字节尾部: sign + 4 字节数值(高字节先发) + 0x6B ---- */
    if (HAL_UART_Receive(&huart1, tail, sizeof(tail), CAR_POS_BYTE_TIMEOUT_MS)
        != HAL_OK)
    {
        return 0u;
    }

    if ((tail[5] != 0x6B) || (tail[0] > 1u))
    {
        return 0u;
    }

    magnitude = ((uint32_t)tail[1] << 24)
              | ((uint32_t)tail[2] << 16)
              | ((uint32_t)tail[3] << 8)
              | (uint32_t)tail[4];

    if (tail[0] == 0u)
    {
        if (magnitude > 0x7FFFFFFFu)
        {
            return 0u;
        }
        *position = (int32_t)magnitude;
    }
    else
    {
        if (magnitude > 0x80000000u)
        {
            return 0u;
        }
        *position = (magnitude == 0x80000000u)
                  ? (-2147483647 - 1)
                  : -(int32_t)magnitude;
    }

    return 1u;
}
