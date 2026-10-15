/* ============================================================
 * 按键驱动 (3 个按键, 轮询 + 消抖)
 *
 * 【本文件的实现思路】
 *   每个按键保存 4 个状态:
 *       last    —— 上一次采样到的电平
 *       cnt     —— 连续读到相同电平的次数
 *       stable  —— 消抖之后的稳定状态
 *       evt     —— 待读取的事件标志(按下 / 松开)
 *   Key_Scan() 每 10ms 采样一次:
 *       采样值和上次相同 -> cnt 累加
 *       采样值和上次不同 -> cnt 清零, 重新开始计数
 *       当 cnt 达到阈值且稳定状态和当前电平不一致时, 才更新 stable
 *       并产生一次事件。这样就把抖动滤掉了。
 *
 * 【电平约定】上拉输入, 按下接 GND
 *       1 = 松开(高)
 *       0 = 按下(低)
 * ============================================================ */

#include "main.h"
#include "key.h"

/* ---- 按键引脚表, 顺序必须和 KEY_1/KEY_2/KEY_3 一一对应 ---- */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} KeyPin_t;

static const KeyPin_t s_key_pin[KEY_NUM] =
{
    { GPIOE, GPIO_PIN_0 },   /* KEY_1 -> PE0 */
    { GPIOE, GPIO_PIN_1 },   /* KEY_2 -> PE1 */
    { GPIOE, GPIO_PIN_2 },   /* KEY_3 -> PE2 */
};

/* ---- 每个按键的运行时状态 ---- */
typedef struct
{
    uint8_t  stable;        /* 消抖后的稳定状态: 1 = 松开, 0 = 按下 */
    uint8_t  last;          /* 上一次采样到的电平 */
    uint8_t  cnt;           /* 连续读到相同电平的次数 */
    uint8_t  press_evt;     /* 待处理的"按下"事件: 1 = 有新按下没被取走 */
    uint8_t  release_evt;   /* 待处理的"松开"事件 */
    uint32_t down_tick;     /* 按下那一刻的时间戳, 用来算按住时长 */
} KeyState_t;

static KeyState_t s_key[KEY_NUM];

/* 上一次扫描的时间戳, 用于给 Key_Scan() 限频 */
static uint32_t s_last_scan_tick = 0;

/**
  * @brief  读某个按键的原始电平
  * @param  index  按键下标 0 ~ KEY_NUM-1
  * @retval 1 = 松开(高电平), 0 = 按下(低电平)
  */
static uint8_t key_read_raw(uint8_t index)
{
    if (HAL_GPIO_ReadPin(s_key_pin[index].port, s_key_pin[index].pin) == GPIO_PIN_SET)
    {
        return 1;   /* 上拉为高 -> 按键松开 */
    }

    return 0;       /* 被按键拉到地 -> 按键按下 */
}

/**
  * @brief  初始化按键驱动
  * @note   GPIO 已经在 MX_GPIO_Init() 里配置成上拉输入了,
  *         这里只把内部状态复位成"全部松开"。
  */
void Key_Init(void)
{
    uint8_t i;

    for (i = 0; i < KEY_NUM; i++)
    {
        s_key[i].stable      = 1;                   /* 上电默认都是松开的 */
        s_key[i].last        = 1;
        s_key[i].cnt         = KEY_DEBOUNCE_CNT;    /* 直接算作已稳定 */
        s_key[i].press_evt   = 0;
        s_key[i].release_evt = 0;
        s_key[i].down_tick   = 0;
    }

    s_last_scan_tick = HAL_GetTick();
}

/**
  * @brief  轮询扫描所有按键(消抖 + 事件生成)
  * @note   放到 while(1) 主循环里反复调用即可。
  *         内部按 KEY_SCAN_INTERVAL_MS 限频, 所以即使主循环
  *         跑得很快, 也不会因为采样过快而导致消抖失效。
  */
void Key_Scan(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  i;

    /* ---- 限频: 每 KEY_SCAN_INTERVAL_MS 毫秒才真正采样一次 ---- */
    if (now - s_last_scan_tick < KEY_SCAN_INTERVAL_MS)
    {
        return;
    }
    s_last_scan_tick = now;

    for (i = 0; i < KEY_NUM; i++)
    {
        uint8_t raw = key_read_raw(i);

        if (raw == s_key[i].last)
        {
            /* 和上次采样一样, 连续次数累加(封顶即可) */
            if (s_key[i].cnt < KEY_DEBOUNCE_CNT)
            {
                s_key[i].cnt++;
            }
        }
        else
        {
            /* 电平跳了, 说明可能刚按下/刚松开或正在抖动, 重新计数 */
            s_key[i].last = raw;
            s_key[i].cnt  = 0;
        }

        /* ---- 连续 KEY_DEBOUNCE_CNT 次电平一致, 且和稳定状态不同
         *      -> 确认状态真的变了, 更新并产生事件 ---- */
        if ((s_key[i].cnt >= KEY_DEBOUNCE_CNT) && (s_key[i].stable != raw))
        {
            s_key[i].stable = raw;

            if (raw == 0)
            {
                /* 松开 -> 按下 */
                s_key[i].press_evt = 1;
                s_key[i].down_tick = now;
            }
            else
            {
                /* 按下 -> 松开 */
                s_key[i].release_evt = 1;
            }
        }
    }
}

/**
  * @brief  取"刚按下"事件
  * @retval 刚被按下的键号(KEY_1/KEY_2/KEY_3), 没有则返回 KEY_NONE
  * @note   事件读走即清除, 所以按住不放只会返回一次。
  */
uint8_t Key_GetPress(void)
{
    uint8_t i;

    for (i = 0; i < KEY_NUM; i++)
    {
        if (s_key[i].press_evt)
        {
            s_key[i].press_evt = 0;     /* 取走就清掉 */
            return i;
        }
    }

    return KEY_NONE;
}

/**
  * @brief  取"刚松开"事件
  * @retval 刚被松开的键号, 没有则返回 KEY_NONE。读走即清除。
  */
uint8_t Key_GetRelease(void)
{
    uint8_t i;

    for (i = 0; i < KEY_NUM; i++)
    {
        if (s_key[i].release_evt)
        {
            s_key[i].release_evt = 0;
            return i;
        }
    }

    return KEY_NONE;
}

/**
  * @brief  查询某个按键当前是否按住
  * @param  key  KEY_1 / KEY_2 / KEY_3
  * @retval 1 = 按住, 0 = 松开或键号非法
  */
uint8_t Key_IsDown(uint8_t key)
{
    if (key >= KEY_NUM)
    {
        return 0;
    }

    return (s_key[key].stable == 0) ? 1 : 0;
}

/**
  * @brief  查询某个按键已经按住了多久
  * @param  key  KEY_1 / KEY_2 / KEY_3
  * @retval 按住的毫秒数; 没按住返回 0
  */
uint32_t Key_GetHoldMs(uint8_t key)
{
    if (key >= KEY_NUM)
    {
        return 0;
    }

    if (s_key[key].stable != 0)
    {
        return 0;   /* 当前是松开状态 */
    }

    /* 从按下那一刻算到现在 */
    return HAL_GetTick() - s_key[key].down_tick;
}
