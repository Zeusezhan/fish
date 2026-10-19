#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "Emm_V5.h"
#include "function.h"

/* ---- 每个轮子的“正转”方向 ----
 * Emm_V5 的 dir: 0 = CW, 1 = CCW。
 * 电机地址与轮子的实际对应关系(实测):
 *   1 = 左后, 2 = 左前, 3 = 右后, 4 = 右前。
 *
 * 自检方法(架空, 启动 target=90 原地左转):
 *   左后/左前应向后, 右后/右前应向前。
 *   如果四个轮子都朝同一个方向, 说明 dir 约定整体反了,
 *   把四个宏一起改成 0 即可; 只有某一个轮子反, 就单独改那一个。 */
/* 实测结果: dir=1 时, 1/2(左侧)向后, 3/4(右侧)向前。
 * 所以左侧的前进方向是 dir=0, 右侧的前进方向是 dir=1。 */
#define WHEEL1_DIR_FORWARD      0   /* 左后 */
#define WHEEL2_DIR_FORWARD      0   /* 左前 */
#define WHEEL3_DIR_FORWARD      1   /* 右后 */
#define WHEEL4_DIR_FORWARD      1   /* 右前 */

static const uint8_t s_wheel_fwd_dir[4] =
{
    WHEEL1_DIR_FORWARD, WHEEL2_DIR_FORWARD, WHEEL3_DIR_FORWARD, WHEEL4_DIR_FORWARD
};

/* 电机允许的最高转速(RPM), 由 Emm_V5 驱动器限制 */
#define CAR_MAX_RPM             5000

/* 加减速档位, 传给 Emm_V5 的 acc 参数(0 表示直接换向, 越大加速越急) */
#define CAR_ACC                 5

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
    Emm_V5_Vel_Control(1, dir1, vel1, CAR_ACC, true);
    HAL_Delay(2);
    Emm_V5_Vel_Control(2, dir2, vel2, CAR_ACC, true);
    HAL_Delay(2);
    Emm_V5_Vel_Control(3, dir3, vel3, CAR_ACC, true);
    HAL_Delay(2);
    Emm_V5_Vel_Control(4, dir4, vel4, CAR_ACC, true);
    HAL_Delay(2);

    /* 2. 广播同步触发(地址 0x00 为广播地址), 四个电机同时转 */
    Emm_V5_Synchronous_motion(0x00);
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
    uint8_t  i;

    /* 下标 0~3 依次对应电机 1~4, 实际接线:
     *   1 = 左后, 2 = 左前, 3 = 右后, 4 = 右前
     * 麦轮逆运动学(x 前进, y 左移, omega 逆时针为正):
     *   左后 = vx + vy - omega
     *   左前 = vx - vy - omega
     *   右后 = vx - vy + omega
     *   右前 = vx + vy + omega */
    speed[0] = vx + vy - omega;     /* 1 左后 */
    speed[1] = vx - vy - omega;     /* 2 左前 */
    speed[2] = vx - vy + omega;     /* 3 右后 */
    speed[3] = vx + vy + omega;     /* 4 右前 */

    for (i = 0; i < 4; i++)
    {
        if (speed[i] >= 0)
        {
            dir[i] = s_wheel_fwd_dir[i];
            rpm[i] = (uint16_t)speed[i];
        }
        else
        {
            dir[i] = (uint8_t)(!s_wheel_fwd_dir[i]);
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
  * @brief  停车: 四个电机立即停止, 最后广播同步触发
  * @note   ⚠ 内部有 4 个 HAL_Delay(2), 不能在中断里调用。
  */
void Car_Stop(void)
{
    Emm_V5_Stop_Now(1, true);
    HAL_Delay(2);
    Emm_V5_Stop_Now(2, true);
    HAL_Delay(2);
    Emm_V5_Stop_Now(3, true);
    HAL_Delay(2);
    Emm_V5_Stop_Now(4, true);
    HAL_Delay(2);

    Emm_V5_Synchronous_motion(0x00);
}
