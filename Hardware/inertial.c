#include "inertial.h"

/* 五段实测数据: 50 RPM, 每段开始前四轮计数清零。
 * 数据说明与符号规律见 inertial.h 顶部注释。 */
const InertialSegment_t g_inertial_segments[INERTIAL_SEGMENT_NUM] =
{
    /* 段1: 直行 3.3 s */
    { INERTIAL_MOVE_FORWARD, 3300u, { -163366, -163376,  163364,  163316 } },

    /* 段2: 左移 3.8 s */
    { INERTIAL_MOVE_LEFT,    3800u, { -190748,  190824, -190716,  190860 } },

    /* 段3: 直行 4.8 s */
    { INERTIAL_MOVE_FORWARD, 4800u, { -245282, -245226,  245424,  245312 } },

    /* 段4: 左移 4.4 s */
    { INERTIAL_MOVE_LEFT,    4400u, { -223452,  223526, -223504,  223569 } },

    /* 段5: 直行 4.9 s */
    { INERTIAL_MOVE_FORWARD, 4900u, { -250764, -250806,  250700,  250712 } },
};

/* 取绝对值且不依赖 64 位除法, 回避 INT32_MIN 取负溢出 */
static uint32_t inertial_abs(int32_t value)
{
    if (value >= 0)
    {
        return (uint32_t)value;
    }

    if (value == (-2147483647 - 1))
    {
        return 2147483647u;
    }

    return (uint32_t)(-value);
}

uint8_t Inertial_GetSegment(uint8_t index, InertialSegment_t *out)
{
    if ((index >= INERTIAL_SEGMENT_NUM) || (out == 0))
    {
        return 0u;
    }

    *out = g_inertial_segments[index];
    return 1u;
}

uint32_t Inertial_GetMagnitude(uint8_t index, uint8_t motor)
{
    if ((index >= INERTIAL_SEGMENT_NUM)
        || (motor < 1u)
        || (motor > INERTIAL_MOTOR_NUM))
    {
        return 0u;
    }

    return inertial_abs(g_inertial_segments[index].encoder[motor - 1u]);
}

uint32_t Inertial_GetAverageMagnitude(uint8_t index)
{
    uint32_t sum = 0u;
    uint8_t  i;

    if (index >= INERTIAL_SEGMENT_NUM)
    {
        return 0u;
    }

    for (i = 0u; i < INERTIAL_MOTOR_NUM; i++)
    {
        sum += inertial_abs(g_inertial_segments[index].encoder[i]);
    }

    return sum / INERTIAL_MOTOR_NUM;
}

uint32_t Inertial_GetPulseRate(uint8_t index)
{
    uint32_t duration_ms;

    if (index >= INERTIAL_SEGMENT_NUM)
    {
        return 0u;
    }

    duration_ms = (uint32_t)g_inertial_segments[index].duration_ms;
    if (duration_ms == 0u)
    {
        return 0u;
    }

    /* (脉冲数 x 1000) / 毫秒 = 脉冲/秒。
     * 单段最大约 250000 脉冲, 乘 1000 后约 2.5e8, 不会溢出 uint32。 */
    return (Inertial_GetAverageMagnitude(index) * 1000u) / duration_ms;
}

uint32_t Inertial_GetPulseRateByType(InertialMove_t type)
{
    uint32_t rate_sum = 0u;
    uint32_t rate;
    uint8_t  count = 0u;
    uint8_t  i;

    for (i = 0u; i < INERTIAL_SEGMENT_NUM; i++)
    {
        if (g_inertial_segments[i].type != type)
        {
            continue;
        }

        rate = Inertial_GetPulseRate(i);
        if (rate != 0u)
        {
            rate_sum += rate;
            count++;
        }
    }

    if (count == 0u)
    {
        return 0u;
    }

    return rate_sum / count;
}
