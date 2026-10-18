#ifndef __CRANE_H
#define __CRANE_H

/*
 * ============================================================
 * 塔吊驱动(统一封装): 上下 + 前后
 *
 * 串口  : USART3(与小车四轮的 USART1 完全分开, 两条总线地址独立编号)
 * 协议  : 张大头 Emm_V5.0 指令帧, 校验字节 0x6B
 * 地址  : 上下 = 1, 前后 = 2
 * 模式  : 位置模式(绝对定位), 适合"走到指定位置停住并保持"
 *
 * 注意  : USART3 上地址 1/2 与 USART1 上小车四轮的 1~4 不冲突,
 *         因为是两条独立串口总线。但别把塔吊接到 USART1 上。
 * ============================================================
 */

#include <stdint.h>

/* ==================== 上下轴(竖直, 地址1) ==================== */
#define CRANE_VERT_ADDR          1
#define CRANE_VERT_DIR_POS       1       /* 1=上, 0=下(实测反了改这里) */
#define CRANE_VERT_RPM           50u     /* 转速上限(RPM), 首次测试先用低速 */
#define CRANE_VERT_ACC           5u      /* 加减速(0=直接换向, 越大越急) */
#define CRANE_VERT_MAX_MM        150     /* 行程上限(mm), 原点=0 在最下, 按实际滑轨 150mm */
#define CRANE_VERT_PULSE_PER_MM  10.0f   /* mm->脉冲系数 ★ 待标定(实测后修改) */

/* ==================== 前后轴(水平, 地址2) ==================== */
#define CRANE_HORI_ADDR          2
#define CRANE_HORI_DIR_POS       1       /* 1=前, 0=后(实测反了改这里) */
#define CRANE_HORI_RPM           300u
#define CRANE_HORI_ACC           5u
#define CRANE_HORI_MAX_MM        80      /* 行程上限(mm), 原点=0 在最后, 按实际滑轨 80mm */
#define CRANE_HORI_PULSE_PER_MM  10.0f   /* mm->脉冲系数 ★ 待标定(实测后修改) */

/* ==================== 上下轴接口 ==================== */
void     Crane_Vert_Init(void);                 /* 上电使能(只保持不动) */
void     Crane_Vert_MoveTo(int32_t pos_mm);     /* 绝对定位(mm) */
void     Crane_Vert_MoveRel(int32_t delta_mm);  /* 相对移动(mm) */
void     Crane_Vert_Stop(void);                 /* 立即停车 */
void     Crane_Vert_Run(uint8_t dir);           /* 速度模式连续转动(直到 Stop) */
void     Crane_Vert_MovePulse(uint32_t clk);    /* 相对走 clk 脉冲(行程标定用) */
void     Crane_Vert_Home(void);                 /* 回原点(需接限位开关) */
int32_t  Crane_Vert_GetPos_mm(void);            /* 读当前位置(mm) */

/* ==================== 前后轴接口 ==================== */
void     Crane_Hori_Init(void);                 /* 上电使能(只保持不动) */
void     Crane_Hori_MoveTo(int32_t pos_mm);     /* 绝对定位(mm) */
void     Crane_Hori_MoveRel(int32_t delta_mm);  /* 相对移动(mm) */
void     Crane_Hori_Stop(void);                 /* 立即停车 */
void     Crane_Hori_Run(uint8_t dir);           /* 速度模式连续转动(直到 Stop) */
void     Crane_Hori_MovePulse(uint32_t clk);    /* 相对走 clk 脉冲(行程标定用) */
void     Crane_Hori_Home(void);                 /* 回原点(需接限位开关) */
int32_t  Crane_Hori_GetPos_mm(void);            /* 读当前位置(mm) */

#endif /* __CRANE_H */
