#ifndef __FUNCTION_H
#define __FUNCTION_H

/*
 * ============================================================
 * 麦轮底盘运动控制
 *
 * 【电机布局】(已确认)
 *            车头
 *      3(左前)      1(右前)
 *      4(左后)      2(右后)
 *
 * 【本文件提供的接口】
 *   底层:  fish_Step_All_Control()  直接给四个电机设速度并同步启动
 *   核心:  Car_Mecanum()            麦轮逆运动学, 前进/横移/自转任意组合
 *   常用:  Car_Drive()              边走边修航向(前进 + 自转修正)
 *   语义:  Car_MoveForward / MoveBackward / MoveLeft / MoveRight
 *          Car_TurnLeft / Car_TurnRight   (原地转向)
 *          Car_Stop
 * ============================================================
 */

#include "Emm_V5.h"

/* 【底层】分别设定四个电机的方向和速度, 最后广播同步启动。
 * dir  : 0 = 后退, 1 = 前进(见 function.c 里的方向约定)
 * vel  : 转速 RPM, 范围 0~5000
 * 注意 : 内部有 4 个 HAL_Delay(2), 一次调用阻塞 8ms 以上,
 *        绝对不能在中断里调用。 */
void fish_Step_All_Control(uint8_t dir1, uint16_t vel1,
                           uint8_t dir2, uint16_t vel2,
                           uint8_t dir3, uint16_t vel3,
                           uint8_t dir4, uint16_t vel4);

/* 【核心】麦轮逆运动学, 三个量任意组合:
 * vx    : 前进速度(RPM), 正数前进, 负数后退
 * vy    : 左移速度(RPM), 正数左移, 负数右移
 * omega : 自转速度(RPM), 正数逆时针(原地左转), 负数顺时针
 * 例: Car_Mecanum(150, 0, 0)     直行
 *     Car_Mecanum(0, 150, 0)     原地左移(横着走)
 *     Car_Mecanum(0, 0, 100)     原地左转
 *     Car_Mecanum(150, 0, 30)    边走边往左偏 */
void Car_Mecanum(int16_t vx, int16_t vy, int16_t omega);

/* 【常用】边走边修航向: 前进 + 自转修正
 * base_speed : 前进基准速度(RPM), >0 前进, <0 后退, =0 就是原地转向
 * turn       : 转向修正量(RPM), >0 往左修, <0 往右修
 * 说明: turn > 0 时右侧轮比左侧轮快, 车头会向左偏,
 *       正好用来抵消"车头偏右"的航向误差。 */
void Car_Drive(int16_t base_speed, int16_t turn);

/* 【语义接口】下面是常用动作的封装, 内部都是调用 Car_Mecanum */
void Car_MoveForward(uint16_t vel);     /* 直行前进 */
void Car_MoveBackward(uint16_t vel);    /* 直行后退 */
void Car_MoveLeft(uint16_t vel);        /* 左移(横着往左) */
void Car_MoveRight(uint16_t vel);       /* 右移(横着往右) */
void Car_TurnLeft(uint16_t vel);        /* 原地左转(逆时针) */
void Car_TurnRight(uint16_t vel);       /* 原地右转(顺时针) */

/* 【停车】四个电机立即停止并同步触发。
 * 返回 1 表示全部停止帧发送成功, 0 表示至少一条失败。
 * 注意: 内部逐条阻塞发送, 不能在中断里调用。 */
uint8_t Car_Stop(void);

/* 【调试】使能/失能四个底盘电机。返回 1 表示所有命令均已发送。 */
uint8_t Car_SetEnabled(uint8_t enabled);

/* 【调试】读取指定电机的当前位置脉冲计数(S_CPOS)。
 * 返回 1 表示收到有效响应；返回 0 表示串口超时或响应帧无效。 */
uint8_t Car_ReadPositionCount(uint8_t motor_id, int32_t *position);

/* 【调试】将 1~4 号电机的当前位置清零。返回 1 表示清零命令均已发送。 */
uint8_t Car_ResetPositionCounts(void);

#endif /* __FUNCTION_H */
