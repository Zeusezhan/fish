#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "Emm_V5.h"
#include "function.h"

/* ---- 轮子方向约定 ---- */
/* Emm_V5 的 dir: 0 = CW, 1 = CCW。 */
#define WHEEL_DIR_FORWARD       1
#define WHEEL_DIR_BACKWARD      0

/* 【电机地址与本车轮子的实际对应关系】
 *   1 = 右前, 2 = 右后, 3 = 左前, 4 = 左后
 *
 * 【历史记录 —— 不要随意改这套映射】
 *   10.23 版本(commit 73dad5c) 用下面这套映射, 前进/后退/左移/右移/
 *   自转全部实测正确。清理合并冲突时曾误用远程分支另一套映射
 *   (1=左后 2=左前 3=右后 4=右前), 导致前进变后退、右移变左移。
 *   现已恢复为本车实测的版本。
 *
 *   注意: 下面的宏必须和 Car_Mecanum() 里的 speed[] 注释配套使用,
 *         它们共同描述"哪个编号是哪个轮子"。只改一边必然跑偏。
 *
 * 【自检方法】
 *   架空车, 在菜单 Debug → Single Seg 里选 FWD 跑 1 秒:
 *     四个轮子都应朝车头方向转。
 *   若只有一个轮子反, 单独改那一个 MOTORx_DIR_INVERT;
 *   若四个全反, 才把 WHEELx_DIR_FORWARD 四个一起取反。 */
#define WHEEL1_DIR_FORWARD      WHEEL_DIR_FORWARD   /* 右前 */
#define WHEEL2_DIR_FORWARD      WHEEL_DIR_FORWARD   /* 右后 */
#define WHEEL3_DIR_FORWARD      WHEEL_DIR_FORWARD   /* 左前 */
#define WHEEL4_DIR_FORWARD      WHEEL_DIR_FORWARD   /* 左后 */

/* 个别轮子的安装方向相反, 需要单独翻转。
 * 实测: 3/4 号(左侧两个轮子)与 1/2 号(右侧)安装方向相反,
 * 所以左侧轮子要整体取反, 才能和右侧一起把车往前推。 */
#define MOTOR_1_DIR_INVERT      0   /* 右前: 不翻转 */
#define MOTOR_2_DIR_INVERT      0   /* 右后: 不翻转 */
#define MOTOR_3_DIR_INVERT      1   /* 左前: 翻转 */
#define MOTOR_4_DIR_INVERT      1   /* 左后: 翻转 */

/* 电机允许的最高转速(RPM), 由 Emm_V5 驱动器限制 */
#define CAR_MAX_RPM             5000

/* 加减速档位, 传给 Emm_V5 的 acc 参数(0 表示直接换向, 越大加速越急) */
#define CAR_ACC                 5

/**
  * @brief  进入 / 退出"底盘命令临界区"
  * @note   Emm_V5 的速度命令用 DMA 发送, 而命令缓冲区是函数内的局部数组。
  *         如果一条命令的 DMA 还没有发完, 又一次进到这里(例如 TIM3
  *         中断里的上报逻辑插进来), 缓冲区就会失效, 发出去的是乱码帧,
  *         那个电机就"收不到命令"。
  *
  *         临界区期间屏蔽 TIM3 中断, 保证同一时刻只有一条底盘命令
  *         序列在拼帧/发送。串口的传输完成中断仍允许, 发送不会卡死。
  *
  *         ★ 临界区内不要放 HAL_Delay: 只包住"打包 + 启动 DMA"这一小段。
  */
static void car_cmd_enter(void)
{
    NVIC_DisableIRQ(TIM3_IRQn);
}

static void car_cmd_exit(void)
{
    NVIC_EnableIRQ(TIM3_IRQn);
}

/**
  * @brief  向底盘串口发一帧命令(阻塞发送, 单帧发送期间屏蔽 TIM3)
  * @note   ★ 这是"电机收不到停止命令"的根因之一, 不要去掉保护。
  *
  *         TIM3(20ms) 中断里如果去发 VOFA+/无线数据, 就可能正好插在
  *         Car_Stop() 这几帧之间。一旦某一帧(典型是地址 1 的第一帧)
  *         被别的数据盖掉, 该驱动器就不会进入"缓存停车"状态,
  *         最后一帧广播同步时它照旧按老速度跑 —— 表现就是
  *         "1 号电机收不到停止命令"。
  */
static HAL_StatusTypeDef car_uart_send(const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef st;

    car_cmd_enter();
    st = HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 20);
    car_cmd_exit();

    return st;
}

/**
  * @brief  底层: 分别设定四个电机的方向和速度, 最后广播同步启动
  * @param  dir1~dir4  各电机方向: 0 = 后退, 1 = 前进
  * @param  vel1~vel4  各电机转速 RPM(0~5000)
  * @note   1. 先用 snF=true 把速度"缓存"在各驱动器里, 不立即执行;
  *         2. 最后用广播地址 0x00 发同步触发命令, 四个轮子同时启动,
  *            这样才不会出现"前两个轮子先动、后两个慢一步"的跑偏。
  *         ⚠ 内部有 4 个 HAL_Delay(2), 一次调用阻塞 8ms 以上,
  *           绝对不能在任何中断里调用。
  */
void fish_Step_All_Control(uint8_t dir1, uint16_t vel1,
                           uint8_t dir2, uint16_t vel2,
                           uint8_t dir3, uint16_t vel3,
                           uint8_t dir4, uint16_t vel4)
{
    /* 1. 分别设定四个电机速度, snF=true: 命令先缓存, 等待同步触发 */
    car_cmd_enter();
    Emm_V5_Vel_Control(1, dir1, vel1, CAR_ACC, true);
    car_cmd_exit();
    HAL_Delay(2);
    car_cmd_enter();
    Emm_V5_Vel_Control(2, dir2, vel2, CAR_ACC, true);
    car_cmd_exit();
    HAL_Delay(2);
    car_cmd_enter();
    Emm_V5_Vel_Control(3, dir3, vel3, CAR_ACC, true);
    car_cmd_exit();
    HAL_Delay(2);
    car_cmd_enter();
    Emm_V5_Vel_Control(4, dir4, vel4, CAR_ACC, true);
    car_cmd_exit();
    HAL_Delay(2);

    /* 2. 广播同步触发(地址 0x00 为广播地址), 四个电机同时转 */
    car_cmd_enter();
    Emm_V5_Synchronous_motion(0x00);
    car_cmd_exit();
}

/**
  * @brief  核心: 麦轮逆运动学, 前进 / 横移 / 自转任意组合
  * @param  vx     前进速度 RPM, 正数前进, 负数后退
  * @param  vy     左移速度 RPM, 正数左移, 负数右移
  * @param  omega  自转速度 RPM, 正数逆时针(原地左转), 负数顺时针
  * @note   四轮速度可能为负, 负值表示该轮要反转,
  *         这里统一换算成 (方向, 绝对值转速) 再交给底层。
  */
void Car_Mecanum(int16_t vx, int16_t vy, int16_t omega)
{
    int16_t  speed[4];      /* 各轮带符号的前进速度 */
    uint8_t  dir[4];        /* 换算后的方向 */
    uint16_t rpm[4];        /* 换算后的转速绝对值 */
    static const uint8_t dir_invert[4] =
    {
        MOTOR_1_DIR_INVERT,
        MOTOR_2_DIR_INVERT,
        MOTOR_3_DIR_INVERT,
        MOTOR_4_DIR_INVERT
    };
    uint8_t  i;

    /* 下标 0~3 依次对应电机 1~4, 实际接线:
     *   1 = 右前, 2 = 右后, 3 = 左前, 4 = 左后
     * 麦轮逆运动学(x 前进, y 左移, omega 逆时针为正):
     *   右前 = vx + vy + omega
     *   右后 = vx - vy + omega
     *   左前 = vx - vy - omega
     *   左后 = vx + vy - omega
     *
     * ★ 这套顺序是和上面的 WHEELx_DIR_FORWARD / MOTORx_DIR_INVERT
     *   配套实测出来的, 修改前请先看文件顶部的说明。 */
    speed[0] = vx + vy + omega;     /* 1 右前 */
    speed[1] = vx - vy + omega;     /* 2 右后 */
    speed[2] = vx - vy - omega;     /* 3 左前 */
    speed[3] = vx + vy - omega;     /* 4 左后 */

    for (i = 0; i < 4; i++)
    {
        if (speed[i] >= 0)
        {
            dir[i] = dir_invert[i] ? WHEEL_DIR_BACKWARD : WHEEL_DIR_FORWARD;
            rpm[i] = (uint16_t)speed[i];
        }
        else
        {
            dir[i] = dir_invert[i] ? WHEEL_DIR_FORWARD : WHEEL_DIR_BACKWARD;
            rpm[i] = (uint16_t)(-speed[i]);
        }

        /* 限幅, 防止超过驱动器允许的最高转速 */
        if (rpm[i] > CAR_MAX_RPM)
        {
            rpm[i] = CAR_MAX_RPM;
        }
    }

    fish_Step_All_Control(dir[0], rpm[0], dir[1], rpm[1],
                          dir[2], rpm[2], dir[3], rpm[3]);
}

/**
  * @brief  边走边修航向: 前进 + 自转修正
  * @param  base_speed  前进基准速度 RPM, >0 前进, <0 后退, =0 则退化成原地转向
  * @param  turn        转向修正量 RPM, >0 往左修, <0 往右修
  * @note   代入逆运动学(vy = 0)可以看出来:
  *           右前 = 右后 = base + turn
  *           左后 = 左前 = base - turn
  *         也就是"右侧轮加速、左侧轮减速" -> 车头往左偏,
  *         正好用来抵消航向误差(误差为正表示车头偏右, 需要往左修)。
  *
  *         这个函数同时覆盖了两种工况:
  *           base = 150, turn = 0   -> 直行
  *           base = 150, turn = 20  -> 直行中往左修正
  *           base = 0,   turn = 100 -> 原地左转
  */
void Car_Drive(int16_t base_speed, int16_t turn)
{
    Car_Mecanum(base_speed, 0, turn);
}

/**
  * @brief  直行前进
  */
void Car_MoveForward(uint16_t vel)
{
    Car_Mecanum((int16_t)vel, 0, 0);
}

/**
  * @brief  直行后退
  */
void Car_MoveBackward(uint16_t vel)
{
    Car_Mecanum(-(int16_t)vel, 0, 0);
}

/**
  * @brief  左移(车头方向不变, 整车横着往左走)
  */
void Car_MoveLeft(uint16_t vel)
{
    Car_Mecanum(0, (int16_t)vel, 0);
}

/**
  * @brief  右移(车头方向不变, 整车横着往右走)
  */
void Car_MoveRight(uint16_t vel)
{
    Car_Mecanum(0, -(int16_t)vel, 0);
}

/**
  * @brief  原地左转(俯视逆时针)
  */
void Car_TurnLeft(uint16_t vel)
{
    Car_Mecanum(0, 0, (int16_t)vel);
}

/**
  * @brief  原地右转(俯视顺时针)
  */
void Car_TurnRight(uint16_t vel)
{
    Car_Mecanum(0, 0, -(int16_t)vel);
}

/**
  * @brief  停车: 四台先缓存停止动作, 再用广播同步同时停住
  * @retval 1 表示所有帧全部发送成功, 0 表示至少一条发送失败
  * @note   ⚠ 内部逐条阻塞发送, 不能在中断里调用。
  *
  *         四帧停止命令都带 snF=1, 只缓存不立即执行;
  *         最后用广播地址 0x00 发同步触发, 四台拿到同一条帧,
  *         因此停止动作是同时发生的, 车不会先偏一下。
  *
  *         每一帧都用 car_uart_send() 发送(发送期间屏蔽 TIM3),
  *         保证 4 帧停止命令 + 1 帧同步触发不会被别处的上报数据插入。
  *         这里不用 Emm_V5_Stop_Now(), 因为它走 DMA 发送、函数
  *         立刻返回, 紧接着的同步帧可能抢在停止帧前面发出去。
  */
uint8_t Car_Stop(void)
{
    uint8_t motor_id;
    uint8_t stop_cmd[5];
    uint8_t sync_cmd[4];
    uint8_t ok = 1u;

    /* 广播同步帧: 地址 0x00 + 0xFF 0x66 + 结束符 */
    sync_cmd[0] = 0x00;
    sync_cmd[1] = 0xFF;
    sync_cmd[2] = 0x66;
    sync_cmd[3] = 0x6B;

    for (motor_id = 1u; motor_id <= 4u; motor_id++)
    {
        stop_cmd[0] = motor_id;
        stop_cmd[1] = 0xFE;
        stop_cmd[2] = 0x98;
        stop_cmd[3] = 1u;           /* snF=1: 先缓存, 等广播同步 */
        stop_cmd[4] = 0x6B;

        if (car_uart_send(stop_cmd, sizeof(stop_cmd)) != HAL_OK)
        {
            ok = 0u;
        }
        HAL_Delay(2);
    }

    /* ★ 关键: 同步帧之前必须给驱动器留出"处理完最后一帧"的时间。
     *
     *   HAL_Delay(2) 只能保证数据从 STM32 发出去了, 不能保证驱动器
     *   已经把它写进内部的同步缓存。115200 下一帧 5 字节约 0.43ms,
     *   4 号帧发完到同步帧发出之间几乎没有间隙; 如果驱动器正在
     *   处理自身的中断/写入, 就会漏掉这帧停止命令 —— 而漏掉的
     *   往往就是最先收到、看起来"最来得及"的 1 号电机。
     *
     *   这正是"跑很多次才偶发一次地址 1 收不到停止"的来源。
     *   这里额外等 5ms, 让四台都完成缓存写入再广播同步。 */
    HAL_Delay(5);

    /* 广播同步: 四台收到同一条帧, 同时执行缓存里的停止动作.
     * 只发一次 —— 以前补发第二次会让驱动器多同步一次, 干扰下段运动。 */
    if (car_uart_send(sync_cmd, sizeof(sync_cmd)) != HAL_OK)
    {
        ok = 0u;
    }

    return ok;
}

/**
  * @brief  使能或失能四个底盘电机
  * @retval 1 表示四条命令均发送成功, 0 表示发送失败
  */
uint8_t Car_SetEnabled(uint8_t enabled)
{
    uint8_t motor_id;
    uint8_t cmd[6];

    for (motor_id = 1; motor_id <= 4; motor_id++)
    {
        cmd[0] = motor_id;
        cmd[1] = 0xF3;
        cmd[2] = 0xAB;
        cmd[3] = enabled ? 1u : 0u;
        cmd[4] = 0;
        cmd[5] = 0x6B;

        if (car_uart_send(cmd, sizeof(cmd)) != HAL_OK)
        {
            return 0;
        }
        HAL_Delay(2);
    }

    return 1;
}

/**
  * @brief  读取某个电机的当前位置脉冲计数
  * @note   EMM V5 的 0x36 响应帧为:
  *         地址 + 0x36 + 符号 + 32 位数值(高字节先发) + 0x6B
  * @retval 1 表示响应有效, 0 表示超时或帧无效
  */
uint8_t Car_ReadPositionCount(uint8_t motor_id, int32_t *position)
{
    uint8_t cmd[3];
    uint8_t header_byte;
    uint8_t command_byte;
    uint8_t response_tail[6];
    uint8_t scan_count;
    uint32_t magnitude;

    if ((motor_id < 1u) || (motor_id > 4u) || (position == NULL))
    {
        return 0;
    }

    cmd[0] = motor_id;
    cmd[1] = 0x36;
    cmd[2] = 0x6B;

    if (car_uart_send(cmd, sizeof(cmd)) != HAL_OK)
    {
        return 0;
    }

    /* Skip any stale acknowledgements and synchronize on this motor's 0x36 reply. */
    for (scan_count = 0; scan_count < 32u; scan_count++)
    {
        if (HAL_UART_Receive(&huart1, &header_byte, 1, 25) != HAL_OK)
        {
            return 0;
        }

        if (header_byte != motor_id)
        {
            continue;
        }

        if (HAL_UART_Receive(&huart1, &command_byte, 1, 10) != HAL_OK)
        {
            return 0;
        }

        if (command_byte != 0x36)
        {
            continue;
        }

        if (HAL_UART_Receive(&huart1, response_tail, sizeof(response_tail), 20) != HAL_OK)
        {
            return 0;
        }

        if ((response_tail[5] != 0x6B) || (response_tail[0] > 1u))
        {
            return 0;
        }

        magnitude = ((uint32_t)response_tail[1] << 24)
                  | ((uint32_t)response_tail[2] << 16)
                  | ((uint32_t)response_tail[3] << 8)
                  | (uint32_t)response_tail[4];

        if (response_tail[0] == 0u)
        {
            if (magnitude > 0x7FFFFFFFu)
            {
                return 0;
            }
            *position = (int32_t)magnitude;
        }
        else
        {
            if (magnitude > 0x80000000u)
            {
                return 0;
            }
            *position = (magnitude == 0x80000000u)
                      ? (-2147483647 - 1)
                      : -(int32_t)magnitude;
        }

        return 1;
    }

    return 0;
}

/**
  * @brief  将 1~4 号电机的当前位置清零
  * @retval 1 表示清零命令均发送成功, 0 表示发送失败
  */
uint8_t Car_ResetPositionCounts(void)
{
    uint8_t motor_id;
    uint8_t cmd[4];

    for (motor_id = 1; motor_id <= 4; motor_id++)
    {
        cmd[0] = motor_id;
        cmd[1] = 0x0A;
        cmd[2] = 0x6D;
        cmd[3] = 0x6B;

        if (car_uart_send(cmd, sizeof(cmd)) != HAL_OK)
        {
            return 0;
        }
        HAL_Delay(2);
    }

    return 1;
}
