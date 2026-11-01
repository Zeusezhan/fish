/* ============================================================
 * 惯导跑图状态机(全工程唯一的"跑"逻辑)
 *
 * 【预设路径实测数据】50 RPM, 每段开始前四轮 S_CPOS 清零
 *
 *   段 | 动作 | 时长  | M1(右前) | M2(右后) | M3(左前) | M4(左后)
 *   ---+------+-------+----------+----------+----------+----------
 *    1 | 直行 | 3.3 s | -163366  | -163376  |  163364  |  163316
 *    3 | 直行 | 4.8 s | -245282  | -245226  |  245424  |  245312
 *    5 | 直行 | 4.9 s | -250764  | -250806  |  250700  |  250712
 *
 *   直行段符号(三段完全一致): M1/M2 为负、M3/M4 为正 —— 直接反映底盘的
 *   方向映射, 不要随意改。同一段内四轮脉冲极差都在 200 以内,
 *   速度约 49500~51200 脉冲/秒。
 *
 *   ★ 原来第 2/4 段是"左移", 那组数据保留在下方注释里以便回溯; 现在
 *     路径已改成"直行 + 原地转 90 度", 不再跑横移段。
 *       2 | 左移 | 3.8 s | -190748 |  190824 | -190716 |  190860
 *       4 | 左移 | 4.4 s | -223452 |  223526 | -223504 |  223569
 *
 *   ★ 这组数据是在 "1=右前 2=右后 3=左前 4=左后、DIR_INVERT=0,0,1,1"
 *     那套映射下采的。若改动 function.c 里的电机映射, 本表必须重采。
 *
 * 【为什么放弃横移】
 *   麦轮横移时辊子在地面上是侧向滚动, 侧向抓地力只有纵向的约七成,
 *   而四轮摩擦不均产生的偏航力矩并不小。同样 30RPM 的自转修正,
 *   直行时压得住, 横移时压不住, 表现就是"左移段怎么调都修不回来"。
 *   改成"直行 + 陀螺仪闭环转向"后, 每一段都是纵向受力, 闭环稳定。
 * ============================================================ */

#include "main.h"
#include "jy901s.h"
#include "function.h"
#include "InertialRun.h"
#include <string.h>

/* ==================== 可调参数 ==================== */

/* 平移速度(RPM), 只用于直行/后退。和上面采数据时一致,
 * 改它必须重采直行时长表(k_auto_path 里的 duration_ms)。 */
#define IR_SPEED_RPM            50

/* 横移(左移)用的平移速度(RPM)。比直行慢: 段 4 之后只是 0.2s 的
 * 位置微调, 车速太快一下就走过头, 而且横移本身抓地力差, 慢一点稳。
 * 时长按 0.2s 反推, 改这个值必须重算 k_auto_path 里那一项。 */
#define IR_SIDE_RPM             30

/* 直行时的航向修正量限幅(RPM) 统一用 main.h 的 YAW_OUT_MAX_RPM,
 * 本文件不再单独定义 —— 原来这里的 IR_TURN_MAX 和 main.c 的
 * YAW_OUT_MAX_DEFAULT 是同一个 30, 分两处容易改漏。

* ---- 原地转向参数 ----
 *
 * 转向靠陀螺仪闭环完成: 目标角 = 基准航向 ± 90 度(AUTO 模式下基准是
 * 理想累计角, 单段调试用实测当前朝向, 见 ir_turn_base_yaw), 每 20ms
 * 读一次剩余角度, 用纯自转命令 Car_Mecanum(0, 0, omega) 转过去。
 * ★ 转向不经过航向环的 g_yaw_output, 也不受 YAW_OUT_MAX_RPM(30) 限幅,
 *   速度由 ir_apply_turn() 按剩余角度直接分档给出。
 *
 * IR_TURN_RPM     转向时的最高自转速度(RPM)。麦轮原地转要克服辊子的
 *                 侧向摩擦, 比直行吃力, 所以给得比平移稍低但起步要够;
 *                 太大容易冲过目标再往回摆。
 * IR_TURN_MIN_RPM 接近目标时的最低自转速度(RPM)。
 *                 ★ 这一项是"能不能转动"的生死线: 实测降到 4RPM 时麦轮
 *                   原地转会卡在离目标零点几度的地方不动, 一直转不到容差
 *                   带里(现在没有超时, 会一直磨下去)。8RPM 是实测能可靠
 *                   转起来的下限, 别再往下调 —— 想让残余更小应该收紧
 *                   IR_TURN_TOL_DEG 之外的其他环节(比如直行段修正的
 *                   软启动), 不是降这一项。
 * IR_TURN_TOL_DEG 到位容差(度)。剩余角度进这个带就认为转到位, 立即停车。
 *                 ★ 要和 IR_TURN_MIN_RPM 配套: 保底转速在一个 20ms 控制
 *                   周期内转过的角度必须明显小于容差, 否则会在边界来回冲。
 *                   容差 1.0° + 保底 8RPM 是实测稳定的一组; 容差收到 0.5°
 *                   时必须同时把保底降到能转动的极限, 而这两条互相打架,
 *                   所以 0.5° 那一版不可用, 已回退。
 *                 ★ 设太大则每段都留残余, 直行段首要花时间修。
 *                 注意这是"每段独立 ±90 度"的绝对判据, 误差不会累积。
 *                 注意: 转向**没有时间上限**, 只认角度。转不到位就一直转,
 *                 直到角度进容差带为止(靠 IMU LOST 兜底, 见下)。
 *                 这是有意为之: 超时停车会让车停在错误角度却继续跑下一段,
 *                 比多转一会儿更危险。 */
#define IR_TURN_RPM             40
#define IR_TURN_MIN_RPM         8
#define IR_TURN_TOL_DEG         0.5f

/* 转向"提前停"补偿(度)。
 *
 * 【解决什么问题】
 *   判到位那一刻, 陀螺仪读数是 20ms 前的(模块 100Hz 输出、我们 50Hz 读),
 *   加上停车指令要过发送队列、驱动器还要按 acc 减速, 车实际会**转过头**
 *   一两度。转向本身看不出来, 但下一段直行一开跑就带着这个反向误差:
 *   左转冲过了头 → 车头比目标更左 → 控制器往右猛修 → 看起来就是
 *   "起步往右偏", 修完车已经横移, 再靠横向外环慢慢拉回来。
 *
 * 【怎么定】
 *   跑一趟, 看直行段刚开始那一瞬的 E:。过冲 2 度时 E: 一上来就是 −2 左右。
 *   把这个数填进来, 车就会在离目标还差这么多度时提前发停车, 靠惯性正好停住。
 *   默认 0 = 不补偿(保持原行为)。填大了会停不到位, 表现为 E: 一上来是正的。
 *
 * 【和容差的关系】
 *   实际停车判据 = IR_TURN_TOL_DEG + IR_TURN_LEAD_DEG。 */
#define IR_TURN_LEAD_DEG        0.0f

/* ==================== 转向段航向跳变抑制 ====================
 *
 * 【解决什么问题】
 *   JY901S 的航向是"磁力计参与解算"的结果, 不是纯陀螺积分。转向时电机
 *   电流最大、磁场最脏, 偶发某一拍 yaw_continuous 会整圈跳变(±360)或
 *   跳几十度。闭环一旦看到这种假值, 剩余角度会瞬间变成 ±270°/±450°,
 *   车就按这个假目标继续转 —— 表现是"该转 90 度, 却转到了 260 度"。
 *
 * 【为什么能判出来】
 *   转向时真实角速度最多 IR_TURN_RPM 对应的 240°/s, 控制拍是 20ms,
 *   每拍真跳最多约 5 度。留足余量取 60 度: 超过 60 度的单拍变化不可能是
 *   真转动, 只能是坏数据。
 *
 * 【怎么处理】
 *   丢弃这一拍**不直接丢弃**: 用上一拍的好值 + gz 积分补一小段(最多
 *   IR_YAW_HOLD_MAX_MS), 让闭环既不跳变、也不至于在坏数据期间"冻结
 *   不动"。gz 满量程 ±2000°/s, 受磁场影响比 yaw 小得多, 短时间外推是可靠的。
 *   连续坏数据超过 IR_YAW_HOLD_MAX_MS 就不再外推(保持最后好值),
 *   交给"IMU 掉线/超时"保护收场, 不让它无穷积分。
 *
 * 【作用范围】
 *   只在 IR_TURNING 状态生效。直行段、启动采样都不受影响 —— 那些地方
 *   单拍真变化本来就小, 加了抑制反而可能吞掉正常的航向更新。 */
#define IR_YAW_JUMP_LIMIT_DEG   60.0f
#define IR_YAW_HOLD_MAX_MS      100u    /* 最多外推这么久, 约 5 个控制拍 */

/* ==================== 横向位置外环(横偏补偿) ====================
 *
 * 【为什么需要它】
 *   航向环只管方向, 不管位置: 车头偏 1 度, 跑 1 米就横移约 17mm。
 *   车头被拉回理想角之后, 车是**平行于理想线**在走, 不会自己拐回原来那条线
 *   —— 这就是"拉回没让跑偏回到正轨"的原因, 不是参数没调好。
 *
 * 【原理】
 *   横偏 y ≈ −v · ∫(理想航向 − 实际航向) dt
 *   所以把目标航向挪一个量, 就能把车拐回原线:
 *       目标 = 理想航向 + K · ∫(理想航向 − 实际航向) dt
 *
 *   ★ 积分的是"相对**理想航向**"的误差, 不是相对当前目标 ——
 *     用目标去积, 一旦目标被偏移, 误差就自己变小、积分停止累积,
 *     结果变成斜着平行走, 永远回不了线。这是这条外环最容易写错的地方。
 *
 * 【整定】
 *   闭环后横偏按 e^(-K·t) 衰减, 时间常数 = 1/K 秒:
 *     K = 0.25 → 约 4 秒回到线上(比内环慢 4~8 倍, 稳定)
 *     K = 0.40 → 约 2.5 秒, 更快但可能和内环抢节奏, 表现是画龙
 *   段长只有 3~5 秒, 所以 K 取 0.25~0.35 比较合适。
 *
 * 【安全】
 *   IR_XT_MAX_DEG 限制最大补偿角: 估计跑飞时最多把目标挪这么多度,
 *   不会把车拐到赛道外。IR_XT_ENABLE 置 0 即可整条外环失效。 */
#define IR_XT_ENABLE            (1)
#define IR_XT_GAIN              (0.25f)     /* 1/秒 */
#define IR_XT_MAX_DEG           (8.0f)      /* 最大补偿角(度) */
#define IR_XT_DT                (0.02f)     /* 与外环刷新周期一致(秒) */
#define IR_XT_ACC_MAX           (IR_XT_MAX_DEG / IR_XT_GAIN)  /* 积分上限(度·秒) */

/* ==================== 直行段纠偏方式 ====================
 *
 * 【为什么要有这个开关】
 *   转向段必须用陀螺仪闭环(目标角 − 实测角), 没有替代方案;
 *   但直行段的纠偏只是"锦上添花", 可以完全不用陀螺仪。
 *   想验证"光靠机械能不能走直"时把它置 0 即可。
 *
 * IR_STRAIGHT_YAW_ON  = 1(默认) 直行段照旧走航向环 + 横向外环,
 *                               起点(COUNTDOWN)和运行中(IR_MOVING)都要求
 *                               陀螺仪在线, 掉线立即停车。
 * IR_STRAIGHT_YAW_ON  = 0       直行段**完全开环**: 自转量恒为 0,
 *                               不使能航向环, 也不检查陀螺仪在线
 *                               (单段直行时陀螺仪没数据也能起跑)。
 *
 * ★ 转向段不受本开关影响: 转向永远要求陀螺仪在线, 目标角基准仍然
 *   走 s_ideal_yaw(见 ir_turn_base_yaw), 所以"停在 ±90° 绝对角"的
 *   语义不变。
 * ★ 关掉之后 s_xt_accum 不再更新, 横向位置补偿整条失效, 直行的
 *   水平度完全取决于四轮摩擦是否一致 —— 这是预期行为, 不是 bug。 */
#define IR_STRAIGHT_YAW_ON      (0)

/* 运行中下发底盘命令的周期(ms)。每拍都重发同一组速度是故意的:
 * 命令幂等, 驱动器漏收一帧下一拍就补上了, 比"只在变化时发"抗干扰。 */
#define IR_STEER_MS             20u

/* 起跑前倒计时(ms), 给驱动器使能、机械稳定留时间 */
#define IR_LEAD_MS              1000u

/* 停车命令发出后等多久再读第一次位置(ms) */
#define IR_STOP_SETTLE_MS       150u

/* 两次位置读取之间的间隔(ms) */
#define IR_VERIFY_GAP_MS        150u

/* "轮子还在转"的判据(编码器脉冲差值)。
 * 50RPM 时约 49500 脉冲/秒, 150ms 约 7400 脉冲;
 * 真正停住的轮子只有几个脉冲的抖动, 取 40 既能容忍噪声,
 * 又能抓住"以 1RPM 以上速度在转"的情况。 */
#define IR_MOVE_THRESHOLD       40

/* 停车确认最多重试几次(单轮补停) */
#define IR_VERIFY_MAX_RETRY     3u

/* 位置清零后等待(ms), 让驱动器把 S_CPOS 真正写掉 */
#define IR_ZERO_WAIT_MS         300u

/* ==================== 内部类型 ==================== */

/* 路径表的一项。move 可以是 IR_MOVE_FWD / IR_MOVE_LEFT / IR_MOVE_TURN_L/R。
 * duration_ms 在直行/横移段有效; 转向段的时长由陀螺仪闭环决定,
 * 这一项填 0 且不起作用: 转向段不限时, 只靠角度判到位。 */
typedef struct
{
    uint8_t  move;          /* InertialMove_t */
    uint16_t duration_ms;   /* 直行: 运行时长; 转向: 填 0(用超时) */
} IrPathSeg_t;

typedef enum
{
    IR_VERIFY_WAIT = 0,     /* 读第一次位置作基准 */
    IR_VERIFY_CHECK         /* 再读一次做比较 */
} IrVerifyPhase_t;

/* ==================== 预设路径 ==================== */

/* 自动跑图的 5 段全部是"直行", 段与段之间插入 90 度原地转向。
 *
 * 转向动作写成路径里的一项(duration_ms 不使用), 由状态机识别后
 * 走 IR_TURNING 分支 —— 这样路径表一眼能看出完整轨迹:
 *
 *   段1 直行 -> 左转90 -> 段2 直行
 *            -> 右移0.5s -> 右转90 -> 左移0.7s
 *            -> 段3 直行
 *            -> 后移0.6s -> 前移0.3s -> 左转90
 *            -> 段4 直行 -> 右转90 -> 段5 直行 -> 右转90
 *
 * ★ 横移/后退/前移这些"微调段"都是为了把车身摆正在赛道上, 它们全部
 *   靠时长决定距离(不走陀螺仪闭环), 所以时长要按实测标定, 换地面
 *   或改 IR_SIDE_RPM / IR_SPEED_RPM 都要重测。
 *
 * ★ 时长是 50RPM 直行实测值, 改 IR_SPEED_RPM 必须重测。
 *   转向不需要填时长, 它靠陀螺仪闭环转到角度为止。 */
static const IrPathSeg_t k_auto_path[] =
{
    { IR_MOVE_FWD,      3400u },//直行
    { IR_MOVE_TURN_L,   0u    },     
    { IR_MOVE_FWD,      3100u },//直行
    { IR_MOVE_RIGHT,    600u  },//右移     
    { IR_MOVE_TURN_R,   0u    },     
    { IR_MOVE_LEFT,     1310u },//左移     
    { IR_MOVE_FWD,      5100u },//直行
//    { IR_MOVE_BACK,     400u  },     /* 段3 之后: 后移 0.6s */
//    { IR_MOVE_FWD,      800u  },     /* 段3 之后: 前进 0.3s */
    { IR_MOVE_TURN_L,   0u    },     
	{ IR_MOVE_RIGHT,    600u  }, //右移
    { IR_MOVE_FWD,      4400u },//直行
	{ IR_MOVE_RIGHT,    800u  },//右移
    { IR_MOVE_TURN_R,   0u    },     
    { IR_MOVE_FWD,      4700u },//直行
    { IR_MOVE_TURN_R,   0u    },     /* 段5 之后: 右转 90 */
	{ IR_MOVE_LEFT,     500u  },//左移
};

#define IR_AUTO_SEG_NUM ((uint8_t)(sizeof(k_auto_path) / sizeof(k_auto_path[0])))

/* ==================== 状态 ==================== */

static IrState_t         s_state = IR_IDLE;
static IrVerifyPhase_t   s_verify_phase = IR_VERIFY_WAIT;

static uint8_t           s_mode = IR_MODE_AUTO;
static uint8_t           s_total = 0u;
static uint8_t           s_index = 0u;

static uint8_t           s_cur_move = IR_MOVE_FWD;
static uint16_t          s_cur_ms = 0u;

/* 单段调试的设定值 */
static uint8_t           s_single_move = IR_MOVE_FWD;
static uint16_t          s_single_ms = 2000u;

static uint8_t           s_running = 0u;
static uint8_t           s_cancel = 0u;

/* ==================== 两个航向基准, 分工不同 ====================
 *
 * s_ideal_yaw  理想累计航向(0 → ±90 → ±180 …)。**只用来算转向目标**,
 *              保证每次转到的都是绝对理想角, 转向残余不逐段累积。
 *
 * s_home_yaw   直行段的目标航向, 取"转向停稳后的**实测**角"。
 *              所以下一段起跑瞬间 err = 0, 车不会被当场拧一下 ——
 *              这就是"起步就偏"这个现象的根, 取实测就没有了。
 *
 * ★ 两者不能合成一个:
 *     都用实测 → 转向目标 = 实测 ± 90, 残余逐段累积(会越跑越歪);
 *     都用理想 → 直行目标比实测差一两度, 起步就要修(会"歪一下再回正")。
 *
 * ★ 横向外环仍按 s_ideal_yaw 积分, 所以车会被慢慢拉回理想线:
 *   段首不猛修, 但最终位置还是回到线上。 */
static float             s_ideal_yaw = 0.0f;
static float             s_home_yaw  = 0.0f;

/* 横向位置外环的累积量(度·秒): ∫(理想航向 − 实际航向) dt。
 * 每段起跑时清零 —— 每段都是一条新的理想线, 横偏从 0 算起。 */
static float             s_xt_accum = 0.0f;

/* 转向段的目标角度(绝对航向, 度)。
 * 由 ir_load_segment() 在倒计时阶段算好, IR_TURNING 里闭环收敛到它。
 * 自动跑图和单段调试的转向都走这一套, 行为完全一致。 */
static float             s_turn_target = 0.0f;

/* ---- 转向段航向跳变抑制的运行时状态(见 IR_YAW_JUMP_LIMIT_DEG) ----
 * s_yaw_good      上一次被判定为"可信"的 yaw_continuous
 * s_yaw_good_tick 采到 s_yaw_good 的时刻(HAL_GetTick)
 * s_yaw_hold      本拍是否处于"坏数据, 用外推值顶着"的状态, 仅用于诊断 */
static float             s_yaw_good      = 0.0f;
static uint32_t          s_yaw_good_tick = 0u;
static uint8_t           s_yaw_hold      = 0u;

/* 1 = 当前转向段的目标角还没定, 等起转那一拍再采基准航向 */
static uint8_t           s_turn_pending = 0u;

/* ZERO / STOPPING 这两个"一次性动作"的进度标志 */
static uint8_t           s_action_pushed = 0u;
static uint8_t           s_settle_armed = 0u;

static uint8_t           s_verify_retry = 0u;
static uint8_t           s_verify_reliable = 1u;
static uint8_t           s_stuck_mask = 0u;

static uint32_t          s_deadline = 0u;
static uint32_t          s_start_tick = 0u;
static uint32_t          s_next_steer_tick = 0u;

static int32_t           s_position[IR_MOTOR_NUM];
static uint8_t           s_position_valid[IR_MOTOR_NUM];

static const char       *s_status = "READY";
static const char       *s_result = "NONE";

/* 读编码器失败的那台电机(1~4), 0 = 没有。
 * 挂在 IR_ZERO / IR_COUNTDOWN 的状态名上: POS ERR 之后按 PE0 重启,
 * 会先走 ZERO -> 1s 倒计时, 这段时间足够在屏幕上看到是哪一号读不到。 */
static uint8_t           s_pos_fail_motor = 0u;

/* ==================== 小工具 ==================== */

static uint32_t ir_abs32(int32_t v)
{
    if (v >= 0)
    {
        return (uint32_t)v;
    }
    return (uint32_t)(-(v + 1)) + 1u;
}

/* 把航向环算出的浮点修正量四舍五入并限幅成整数 RPM */
static int16_t ir_turn_from_yaw(void)
{
    float u = g_yaw_output;

    if (u > YAW_OUT_MAX_RPM)
    {
        u = YAW_OUT_MAX_RPM;
    }
    else if (u < -YAW_OUT_MAX_RPM)
    {
        u = -YAW_OUT_MAX_RPM;
    }

    if (u >= 0.0f)
    {
        return (int16_t)(u + 0.5f);
    }
    return (int16_t)(u - 0.5f);
}

static void ir_abort(const char *reason)
{
    s_state = IR_IDLE;
    s_running = 0u;
    s_cancel = 0u;
    s_action_pushed = 0u;
    s_settle_armed = 0u;
    s_turn_pending = 0u;
    YawLoop_Disable();

    s_status = reason;
    s_result = reason;
}

/* 按当前段的方向下发一帧麦轮速度。
 *
 * 直行/后退: 平移速度 + 航向修正叠加。
 * 原地转向不走这里: 只发自转分量, 转多少由剩余角度决定(见 ir_apply_turn)。
 *
 * 转向永远是闭环的, 启动时一定要求陀螺仪在线。
 * 直行段的纠偏由 IR_STRAIGHT_YAW_ON 决定: 关掉时自转量恒为 0,
 * 变成纯开环直行(见参数说明)。 */
static void ir_apply_move(void)
{
#if (IR_STRAIGHT_YAW_ON)
    int16_t turn = ir_turn_from_yaw();
#else
    int16_t turn = 0;
#endif
    int16_t spd = IR_SPEED_RPM;

    switch (s_cur_move)
    {
        case IR_MOVE_LEFT:
            /* 左移: vy 取正(见 Car_Mecanum 的逆运动学, 纯 +vy 即整车左移)。
             * vy 是唯一的横向命令来源, 横移段不要再叠 turn: 麦轮同时
             * 横移+自转时辊子会打滑, 30RPM 的修正量基本是白给。
             * 传 0 让四个轮子严格按 ±vy 平均分配。 */
            Car_Mecanum(0, IR_SIDE_RPM, 0);
            break;

        case IR_MOVE_RIGHT:
            Car_Mecanum(0, (int16_t)(-IR_SIDE_RPM), 0);
            break;

        case IR_MOVE_BACK:
            Car_Mecanum((int16_t)(-spd), 0, turn);
            break;

        case IR_MOVE_FWD:
        default:
            Car_Mecanum(spd, 0, turn);
            break;
    }
}

/**
  * @brief  判断当前段是不是"原地转向"
  */
static uint8_t ir_is_turn_seg(uint8_t move)
{
    return ((move == (uint8_t)IR_MOVE_TURN_L)
            || (move == (uint8_t)IR_MOVE_TURN_R)) ? 1u : 0u;
}

/**
  * @brief  由起始航向算出转向段的目标角(绝对航向)
  * @param  move   IR_MOVE_TURN_L / IR_MOVE_TURN_R
  * @param  start  开始转那一刻的航向(yaw_continuous)
  * @retval 目标航向(度)
  * @note   自动跑图和单段调试共用同一套公式, 这样单段测出来的转向
  *         行为和整趟跑图里段间转向完全一致。左转为正, 所以左转 +90、
  *         右转 -90。
  */
static float ir_turn_target_from(uint8_t move, float start)
{
    float delta = (move == (uint8_t)IR_MOVE_TURN_L)
                ? IR_TURN_SEG_DEG
                : -IR_TURN_SEG_DEG;

    return start + delta;
}

/**
  * @brief  取一个"可信"的转向航向, 把单拍跳变挡在外面
  * @retval 用于闭环计算的航向(度)
  * @note   只在 IR_TURNING 阶段有意义的取样方式, 见 IR_YAW_JUMP_LIMIT_DEG。
  *         判定为坏数据时, 用上一拍好值 + gz 积分外推, 外推时长不超过
  *         IR_YAW_HOLD_MAX_MS; 超出后保持最后好值, 让超时保护去收场。
  */
static float ir_turn_yaw_now(void)
{
    float    now_yaw = jy901s_data.yaw_continuous;
    uint32_t now     = HAL_GetTick();
    float    dt_s;
    float    est;

    if (s_yaw_good_tick == 0u)
    {
        /* 本轮还没建立基准: 第一次调用直接采信当前值 */
        s_yaw_good      = now_yaw;
        s_yaw_good_tick = now;
        s_yaw_hold      = 0u;
        return now_yaw;
    }

    dt_s = (float)(uint32_t)(now - s_yaw_good_tick) * 0.001f;

    /* 正常情况: 单拍变化在门限内, 直接采信并刷新基准 */
    if ((now_yaw - s_yaw_good <= IR_YAW_JUMP_LIMIT_DEG)
        && (now_yaw - s_yaw_good >= -IR_YAW_JUMP_LIMIT_DEG))
    {
        s_yaw_good      = now_yaw;
        s_yaw_good_tick = now;
        s_yaw_hold      = 0u;
        return now_yaw;
    }

    /* 跳变: 用 gz 外推顶着。gz 是 deg/s, 左转为正, 和航向同号。
     * 外推上限 IR_YAW_HOLD_MAX_MS, 防止坏数据期间积分跑飞。 */
    s_yaw_hold = 1u;

    if ((uint32_t)(now - s_yaw_good_tick) > IR_YAW_HOLD_MAX_MS)
    {
        dt_s = (float)IR_YAW_HOLD_MAX_MS * 0.001f;
    }

    est = s_yaw_good + jy901s_data.gz * dt_s;

    /* 外推值本身也要过一遍门限, 免得 dt 抖动时又给出一个离谱值 */
    if ((est - s_yaw_good > IR_YAW_JUMP_LIMIT_DEG)
        || (est - s_yaw_good < -IR_YAW_JUMP_LIMIT_DEG))
    {
        float lim = (est > s_yaw_good) ? IR_YAW_JUMP_LIMIT_DEG
                                       : -IR_YAW_JUMP_LIMIT_DEG;
        est = s_yaw_good + lim;
    }

    return est;
}

/**
  * @brief  下发一帧纯自转命令
  * @note   转向角速度按剩余误差限幅: 差得多就快转, 快到位就慢转,
  *         避免以 40RPM 冲过目标再掉头来回摆。
  */
static void ir_apply_turn(void)
{
    float remain = g_yaw_target - ir_turn_yaw_now();
    int16_t rpm;

    /* 误差 10 度以上给满转; 10 度以内按比例降速(5 度约半速);
     * 接近目标时保留 IR_TURN_MIN_RPM 的保底转, 压过静摩擦才转得动。
     * 保底必须和 IR_TURN_TOL_DEG 配套: 保底转速一步能转过的角度要小于
     * 容差, 否则会在边界反复冲过、来回抖(见文件头参数说明)。 */
    if (remain > 10.0f)       rpm = IR_TURN_RPM;
    else if (remain < -10.0f) rpm = -IR_TURN_RPM;
    else
    {
        float gain = (float)IR_TURN_RPM / 10.0f;
        float cmd  = remain * gain;
        float minr = (float)IR_TURN_MIN_RPM;

        if ((cmd < minr) && (cmd > 0.0f))    cmd = minr;
        if ((cmd > -minr) && (cmd < 0.0f))   cmd = -minr;

        rpm = (int16_t)((cmd >= 0.0f) ? (cmd + 0.5f) : (cmd - 0.5f));
    }

    Car_Mecanum(0, 0, rpm);
}

/* 转向目标角的基准航向(理想基准)。
 *
 * AUTO : 返回 s_ideal_yaw —— 理想累计航向, 由理想累加得到
 *        (0° → ±90° → ±180° …), 不是实测停住角。这样转向目标永远是
 *        理想的绝对角, 每一次转向残余不会被当成新基准往下传。
 *        ★ 注意这里不能用 s_home_yaw —— 它已经是实测角了, 拿它累加
 *          就等于"以实际为基准转 90", 误差会一段段滚下去。
 * SINGLE: 返回实测当前朝向, 保留"就地转 90 度"的调试语义。 */
static float ir_turn_base_yaw(void)
{
    if (s_mode == IR_MODE_AUTO)
    {
        return s_ideal_yaw;
    }
    return jy901s_data.yaw_continuous;
}

/* 把当前段号对应的动作/时长装进 s_cur_move / s_cur_ms,
 * 并为"转向段"算好目标角度。 */
static void ir_load_segment(uint8_t countdown_pending)
{
    if (s_mode == IR_MODE_SINGLE)
    {
        s_cur_move = s_single_move;
        s_cur_ms = s_single_ms;
        /* 转向动作没有"时长"可言, 也不再有超时上限: 只靠角度判到位。
         * 这里把时长清零, 界面显示和内部判据都按"不限时"处理。 */
        if (ir_is_turn_seg(s_cur_move) != 0u)
        {
            s_cur_ms = 0u;
        }
    }
    else
    {
        if (s_index >= IR_AUTO_SEG_NUM)
        {
            s_index = (uint8_t)(IR_AUTO_SEG_NUM - 1u);
        }

        s_cur_move = k_auto_path[s_index].move;
        s_cur_ms = k_auto_path[s_index].duration_ms;

        if (ir_is_turn_seg(s_cur_move) != 0u)
        {
            s_cur_ms = 0u;      /* 转向段不限时 */
        }
    }

    /* 转向段: 目标角 = 基准航向 ± 90 度。
     * ★ AUTO 模式下基准是"理想基准"(见 ir_turn_base_yaw), 不是实测当前角 ——
     *   实测角带着上一段的转向残余(容差 ±1°), 用它累加会让误差逐段放大。
     * ★ 起转那一拍再定目标, 因为倒计时 1 秒内车可能被碰、陀螺也在漂。 */
    if (ir_is_turn_seg(s_cur_move) != 0u)
    {
        if (countdown_pending != 0u)
        {
            /* 正常起跑: 倒计时还没开始, 现在就是"转之前", 直接算目标。 */
            s_turn_target = ir_turn_target_from(s_cur_move,
                                                ir_turn_base_yaw());
        }
        else
        {
            /* 倒计时期间陀螺仪还会继续走一点, 所以先留一个待采样的标记,
             * 真正起转的那一拍(COUNTDOWN 结束)再采基准, 和"起跑瞬间才
             * 定基准航向"完全一致。 */
            s_turn_pending = 1u;
        }
    }
    else
    {
        s_turn_pending = 0u;
    }
}

/* 结束运动, 进入停车流程 */
static void ir_begin_stop(const char *status)
{
    /* 顺序很重要: 先关航向环, 再发停车。
     * 反过来的话, 中断可能在停车命令之后又算出一个新的修正量,
     * 主循环下一拍就会把车重新开动。 */
    s_running = 0u;
    YawLoop_Disable();

    s_action_pushed = 0u;
    s_settle_armed = 0u;
    s_verify_retry = 0u;
    s_verify_reliable = 1u;
    s_stuck_mask = 0u;

    s_state = IR_STOPPING;
    s_status = status;
}

/* 读一遍四个轮子的位置计数, 返回 1 = 四个都读到了 */
static uint8_t ir_read_positions(void)
{
    uint8_t i;
    uint8_t all_ok = 1u;

    for (i = 0u; i < IR_MOTOR_NUM; i++)
    {
        s_position_valid[i] = Car_ReadPositionCount((uint8_t)(i + 1u),
                                                    &s_position[i]);
        if (s_position_valid[i] == 0u)
        {
            all_ok = 0u;
            s_pos_fail_motor = (uint8_t)(i + 1u);
        }
    }

    return all_ok;
}

/**
  * @brief  停车确认: 用编码器判断四个轮子是不是真的停住了
  * @note   阶段一: 读一次位置作基准 A
  *         阶段二: 隔 150ms 再读一次, 与 A 逐轮比较, 超过门限就补停那台
  */
static void ir_verify(uint32_t now)
{
    uint8_t i;

    if ((int32_t)(now - s_deadline) < 0)
    {
        return;
    }

    /* ---- 阶段一: 记录基准位置 ---- */
    if (s_verify_phase == IR_VERIFY_WAIT)
    {
        s_verify_reliable = ir_read_positions();

        if (s_verify_reliable == 0u)
        {
            /* 读不到位置就没法确认停车。★ 这里不能重发停车命令:
             * Car_Stop() 一次就塞 8 帧, 而现在失败的原因恰恰是队列里
             * 积压着上一拍的速度帧 + 刚才那 8 帧停车帧, 再塞 8 帧只会
             * 让队列更满、下次读位置更容易超时 —— 越重试越堵。
             * 停车帧已经发过 2 轮(幂等), 这里只需等队列排空再读一次。 */
            s_verify_retry++;
            if (s_verify_retry > IR_VERIFY_MAX_RETRY)
            {
                ir_abort("POS ERR");
                return;
            }
            s_settle_armed = 0u;
            s_state = IR_STOPPING;
            s_status = "POS RETRY";
            return;
        }

        s_verify_phase = IR_VERIFY_CHECK;
        s_deadline = now + IR_VERIFY_GAP_MS;
        s_status = "VERIFY";
        return;
    }

    /* ---- 阶段二: 再读一次, 和基准比较 ---- */
    s_stuck_mask = 0u;

    for (i = 0u; i < IR_MOTOR_NUM; i++)
    {
        int32_t pos_now = 0;
        int32_t delta;

        if (Car_ReadPositionCount((uint8_t)(i + 1u), &pos_now) == 0u)
        {
            s_verify_reliable = 0u;
            s_pos_fail_motor = (uint8_t)(i + 1u);
            continue;
        }

        delta = pos_now - s_position[i];
        if (ir_abs32(delta) > (uint32_t)IR_MOVE_THRESHOLD)
        {
            s_stuck_mask |= (uint8_t)(1u << i);
        }

        s_position[i] = pos_now;
        s_position_valid[i] = 1u;
    }

    if (s_verify_reliable == 0u)
    {
        /* 同上: 阶段二失败也不补发停车帧, 只等队列排空再读。
         * 补发的唯一作用是把队列重新塞满。 */
        s_verify_retry++;
        if (s_verify_retry > IR_VERIFY_MAX_RETRY)
        {
            ir_abort("POS ERR");
            return;
        }
        s_verify_phase = IR_VERIFY_WAIT;
        s_settle_armed = 0u;
        s_state = IR_STOPPING;
        s_status = "POS RETRY";
        return;
    }

    if (s_stuck_mask != 0u)
    {
        /* 有轮子还在转: 单独给它补两帧立即停止 */
        for (i = 0u; i < IR_MOTOR_NUM; i++)
        {
            if ((s_stuck_mask & (uint8_t)(1u << i)) != 0u)
            {
                Car_StopMotor((uint8_t)(i + 1u));
            }
        }

        s_verify_retry++;
        if (s_verify_retry > IR_VERIFY_MAX_RETRY)
        {
            /* 补了三次还在转, 说明不是丢帧而是驱动器本身有问题 */
            Car_Stop();
            ir_abort("WHEEL STUCK");
            return;
        }

        s_verify_phase = IR_VERIFY_WAIT;
        s_settle_armed = 0u;
        s_state = IR_STOPPING;
        s_status = "WHEEL RETRY";
        return;
    }

    /* ---- 四个轮子都确认停住了 ---- */
    if (s_cancel != 0u)
    {
        ir_abort("USER STOP");
        return;
    }

    /* 陀螺仪掉线导致的停车不能"接着跑下一段":
     * 没有航向反馈的惯导跑图毫无意义, 直接终止整趟任务。 */
    if (YawLoop_IsImuAlive() == 0)
    {
        ir_abort("IMU LOST");
        return;
    }

    s_verify_phase = IR_VERIFY_WAIT;
    s_verify_retry = 0u;
    s_status = "STOPPED";

    /* 转向段结束后更新两个基准, 各管一件事:
     *
     *   s_ideal_yaw = 理想目标角(90°/180°…) —— **只用来算下一次转向的目标**,
     *                 这样每次转到的都是绝对理想角, 转向残余不会逐段累积。
     *
     *   s_home_yaw  = **实测停稳角** —— 下一段直行的目标。起跑瞬间 err = 0,
     *                 车不会被当场拧一下, 也就没有"起步就偏"。
     *
     * ★ 为什么不能只用一个变量:
     *   若两个都用实测, 转向目标 = 实测 ± 90, 残余逐段累积(老的毛病);
     *   若两个都用理想, 直行目标比实测差 1~2°, 起步就要修(上一版的现象)。
     *   拆开才能"转向不累积 + 直行不偏"。
     * ★ 横向外环(s_xt_accum)仍然按 s_ideal_yaw 积分, 所以它会把车慢慢
     *   拉回理想线 —— 段首不猛修, 但最终位置还是对的。
     * ★ 单段调试也走这一支: 一次只测一段, 更新了也没副作用。 */
    if (ir_is_turn_seg(s_cur_move) != 0u)
    {
        s_ideal_yaw = s_turn_target;

        /* 停稳角只在"最后那一段确实是可信读数"时才采信: 抑制器还在
         * 外推(坏数据未恢复)就用外推值, 免得把假角当成下一段的直行目标。 */
        s_home_yaw = ir_turn_yaw_now();
    }

    /* 最后一段: 直接收尾, 不清零位置 —— 那组 S_CPOS 就是这一段的位移 */
    if ((uint8_t)(s_index + 1u) >= s_total)
    {
        s_state = IR_IDLE;
        s_status = "ALL DONE";
        s_result = "DONE";
        return;
    }

    /* 还有下一段: 段号+1, 回 ZERO 清零编码器 */
    s_index++;
    s_action_pushed = 0u;
    s_state = IR_ZERO;
    s_deadline = now;
}

/* ==================== 对外接口 ==================== */

void InertialRun_Init(void)
{
    uint8_t i;

    s_state = IR_IDLE;
    s_verify_phase = IR_VERIFY_WAIT;
    s_mode = IR_MODE_AUTO;
    s_total = 0u;
    s_index = 0u;
    s_cur_move = IR_MOVE_FWD;
    s_cur_ms = 0u;
    s_single_move = IR_MOVE_FWD;
    s_single_ms = IR_SINGLE_MS_DEFAULT;
    s_running = 0u;
    s_cancel = 0u;
    s_action_pushed = 0u;
    s_settle_armed = 0u;
    s_verify_retry = 0u;
    s_verify_reliable = 1u;
    s_stuck_mask = 0u;
    s_pos_fail_motor = 0u;
    s_turn_target = 0.0f;
    s_turn_pending = 0u;
    s_deadline = 0u;
    s_start_tick = 0u;
    s_next_steer_tick = 0u;
    s_status = "READY";
    s_result = "NONE";

    for (i = 0u; i < IR_MOTOR_NUM; i++)
    {
        s_position[i] = 0;
        s_position_valid[i] = 0u;
    }
}

/* 启动时(ir_start / START 命令)要不要拦下"陀螺仪没数据"?
 *   1 = 直接拒绝启动。AUTO 一定含转向段, 只是第一段可能还是直行,
 *       所以不能等到 COUNTDOWN 再报错, 这里就要拦;
 *   0 = 放行。只有"单段 + 直行开环"能走到这里 —— 这一趟压根不用
 *       陀螺仪, 没有数据也没关系。 */
static uint8_t ir_need_imu_at_start(void)
{
    return ((s_mode == IR_MODE_AUTO) || (ir_is_turn_seg(s_single_move) != 0u))
           ? 1u : 0u;
}

/**
  * @brief  内部: 两种模式共用的启动准备
  * @retval 1 = 已启动, 0 = 忙或陀螺仪没数据
  */
static uint8_t ir_start(uint8_t mode, uint8_t total)
{
    uint8_t i;

    if (s_state != IR_IDLE)
    {
        s_status = "BUSY";
        return 0u;
    }

    /* 需要航向反馈却没有数据时拒绝启动(见 ir_need_imu_at_start) */
    if ((ir_need_imu_at_start() != 0u) && (YawLoop_IsImuAlive() == 0))
    {
        s_status = "NO IMU";
        s_result = "NO IMU";
        return 0u;
    }

    s_mode = mode;
    s_total = total;
    s_index = 0u;
    s_cancel = 0u;
    s_running = 0u;
    s_action_pushed = 0u;
    s_settle_armed = 0u;
    s_verify_retry = 0u;
    s_stuck_mask = 0u;
    s_turn_pending = 0u;

    /* 基准航向等第 1 段真正起跑时再采样(那时车已经摆好、IMU 数据也稳),
     * 这里只清掉上一趟留下的值, 避免误用。 */
    s_home_yaw  = 0.0f;
    s_ideal_yaw = 0.0f;

    YawLoop_Disable();

    for (i = 0u; i < IR_MOTOR_NUM; i++)
    {
        s_position_valid[i] = 0u;
    }

    s_status = "START";
    s_result = "RUN";

    /* 先进 ZERO: 清零编码器 + 使能电机, 再倒计时起跑 */
    s_state = IR_ZERO;
    s_deadline = HAL_GetTick();

    return 1u;
}

uint8_t InertialRun_StartAuto(void)
{
    return ir_start(IR_MODE_AUTO, IR_AUTO_SEG_NUM);
}

uint8_t InertialRun_StartSingle(InertialMove_t move, uint16_t duration_ms)
{
    /* 单段调试允许 直行 / 后退 / 原地左转 90 度 / 原地右转 90 度。
     * 越界(含自动路径专用的 IR_MOVE_LEFT)一律退回直行。 */
    if ((uint8_t)move >= IR_SINGLE_MOVE_COUNT)
    {
        move = IR_MOVE_FWD;
    }

    if (duration_ms < IR_SINGLE_MS_MIN)
    {
        duration_ms = IR_SINGLE_MS_MIN;
    }
    else if (duration_ms > IR_SINGLE_MS_MAX)
    {
        duration_ms = IR_SINGLE_MS_MAX;
    }

    s_single_move = (uint8_t)move;
    s_single_ms = duration_ms;

    return ir_start(IR_MODE_SINGLE, 1u);
}

void InertialRun_Cancel(void)
{
    if (s_state == IR_IDLE)
    {
        return;
    }

    switch (s_state)
    {
        case IR_MOVING:
        case IR_TURNING:
            s_cancel = 1u;
            ir_begin_stop("USER STOP");
            break;

        case IR_STOPPING:
        case IR_VERIFY:
            /* 停车流程已经在走了, 让它走完再收尾 */
            s_cancel = 1u;
            break;

        case IR_ZERO:
        case IR_COUNTDOWN:
        default:
            /* 轮子本来就是停着的, 直接收尾 */
            ir_abort("USER STOP");
            break;
    }
}

uint8_t InertialRun_IsBusy(void)
{
    return (s_state != IR_IDLE) ? 1u : 0u;
}

void InertialRun_Task(void)
{
    uint32_t now = HAL_GetTick();

    switch (s_state)
    {
        /* ---- 清零编码器 + 使能电机 ---- */
        case IR_ZERO:
            if (s_action_pushed == 0u)
            {
                if (Car_ResetPositionCounts() == 0u)
                {
                    ir_abort("ZERO ERR");
                    break;
                }
                s_action_pushed = 1u;
                s_deadline = now + IR_ZERO_WAIT_MS;
                s_status = "ZERO";
                break;
            }

            /* 等清零帧真的发出去, 再等驱动器把计数写掉 */
            if (Car_TxBusy() != 0u)
            {
                break;
            }
            if ((int32_t)(now - s_deadline) < 0)
            {
                break;
            }

            s_action_pushed = 0u;

            /* 使能放在倒计时之前, 这样 1 秒的倒计时足够 4 帧使能帧发完,
             * 起跑瞬间不会有"使能还没到"的延迟。 */
            if (Car_SetEnabled(1u) == 0u)
            {
                ir_abort("ENA ERR");
                break;
            }

            ir_load_segment(1u);
            s_state = IR_COUNTDOWN;
            s_deadline = now + IR_LEAD_MS;

            /* 上一次 POS ERR 是哪一号读不到, 借这段清零/倒计时显示出来,
             * 下次再失败就能直接定位到电机和接线。 */
            if (s_pos_fail_motor != 0u)
            {
                s_status = (s_pos_fail_motor == 1u) ? "ZERO! M1"
                         : (s_pos_fail_motor == 2u) ? "ZERO! M2"
                         : (s_pos_fail_motor == 3u) ? "ZERO! M3"
                         : "ZERO! M4";
            }
            else
            {
                s_status = "READY";
            }
            break;

        /* ---- 起跑 ---- */
        case IR_COUNTDOWN:
            if ((int32_t)(now - s_deadline) < 0)
            {
                break;
            }

            if (s_cancel != 0u)
            {
                ir_abort("USER STOP");
                break;
            }

            /* 起跑前再确认一次陀螺仪还在出数。
             * 转向段 / 直行闭环时这些数据是必需的, 没有就宁可不起跑。 */
            if ((YawLoop_IsImuAlive() == 0)
                && ((ir_is_turn_seg(s_cur_move) != 0u)
                    || (IR_STRAIGHT_YAW_ON != 0)))
            {
                ir_abort("IMU LOST");
                break;
            }

            /* 航向基准:
             *   - 第 1 段(countdown 时 s_index == 0 且当前段是直行)采一次基,
             *     把当时朝向同时定为"直行目标"和"理想 0 点";
             *   - 之后每段由 ir_verify 更新: 直行目标取**实测停稳角**(起步
             *     不偏), 转向基准取**理想角**(残余不累积)。
             *
             * 单段调试(s_index 恒为 0)行为不变: 每次起跑都重新定基准。
             * ★ 基准就是转向的目标角, 所以陀螺仪没数据时也必须采:
             *   单段直行在 IR_STRAIGHT_YAW_ON = 0 下允许无陀螺仪起跑,
             *   但此时 s_ideal_yaw 若仍为 0, 接下来第一次转向就会转错方向。 */
            if ((s_index == 0u) && (ir_is_turn_seg(s_cur_move) == 0u))
            {
                s_home_yaw  = jy901s_data.yaw_continuous;
                s_ideal_yaw = s_home_yaw;          /* 起点即理想 0 点 */
            }

            /* ---- 转向段: 完全不同的下发方式 ---- */
            if (ir_is_turn_seg(s_cur_move) != 0u)
            {
                /* 倒计时阶段只留了"待采样"标记, 起转这一拍才把目标角定下来:
                 * 这 1 秒里车可能被人碰了一下、陀螺仪也可能还在缓慢漂移。
                 * 基准仍走 ir_turn_base_yaw(): AUTO 用理想基准, SINGLE 用实测。 */
                if (s_turn_pending != 0u)
                {
                    s_turn_target = ir_turn_target_from(s_cur_move,
                                                        ir_turn_base_yaw());
                    s_turn_pending = 0u;
                }

                /* 目标角就是本段的绝对航向(理想角), 直接闭环过去。
                 * 这里不能用 s_home_yaw: 转向的目的就是离开当前朝向。 */
                YawLoop_Enable(s_turn_target, YAW_OUT_MAX_RPM);

                /* 跳变抑制器重新建立基准: 停了一整段 + 倒计时, 上一段
                 * 留下的 s_yaw_good 已经过期, 不能拿来当"上一拍好值"。
                 * 用此刻的实测值当起点, 之后每 20ms 和它比。 */
                s_yaw_good      = jy901s_data.yaw_continuous;
                s_yaw_good_tick = now;
                s_yaw_hold      = 0u;

                ir_apply_turn();

                s_running = 1u;
                s_start_tick = now;
                s_next_steer_tick = now + IR_STEER_MS;
                s_status = "TURN";
                s_state = IR_TURNING;
                break;
            }

            /* 新的一段 = 一条新的理想线: 横偏估计从 0 起算 */
            s_xt_accum = 0.0f;

#if (IR_STRAIGHT_YAW_ON)
            YawLoop_Enable(s_home_yaw, YAW_OUT_MAX_RPM);
#else
            /* 直行开环: 航向环必须关掉。否则 s_home_yaw=0(上一趟清的值)
             * 会被当成目标, 直行一开始就被拧一个大自转量。 */
            YawLoop_Disable();
#endif

            ir_apply_move();

            s_running = 1u;
            s_start_tick = now;
            s_next_steer_tick = now + IR_STEER_MS;
            s_status = "RUN";
            s_state = IR_MOVING;
            break;

        /* ---- 直行/后退中 ---- */
        case IR_MOVING:
            if (s_running == 0u)
            {
                break;
            }

            /* 陀螺仪掉线就停车: 没有航向反馈的闭环比开环更危险。
             * 直行开环(IR_STRAIGHT_YAW_ON = 0)时不纠偏, 也就不需要它,
             * 照常跑到时长结束 —— 车不会因为陀螺仪偶发掉线横在赛道上。 */
#if (IR_STRAIGHT_YAW_ON)
            if (YawLoop_IsImuAlive() == 0)
            {
                ir_begin_stop("IMU LOST");
                break;
            }
#endif

            if ((uint32_t)(now - s_start_tick) >= (uint32_t)s_cur_ms)
            {
                ir_begin_stop("STOPPING");
                break;
            }

            /* 每 20ms 重发一次底盘命令 */
            if ((int32_t)(now - s_next_steer_tick) >= 0)
            {
                s_next_steer_tick = now + IR_STEER_MS;

#if (IR_XT_ENABLE && IR_STRAIGHT_YAW_ON)
                /* ---- 横向位置外环: 用航向误差的积分估计横偏 ----
                 * 积的是"**理想航向** − 实际航向", 不是"目标 − 实际",
                 * 也不是"直行目标 − 实际":
                 *   - 用目标积: 目标一偏移, 误差自己就小了, 积分停止累积;
                 *   - 用直行目标(实测角)积: 段首那点残余被当成"没有偏",
                 *     车会一直斜着走、永远回不到理想线。
                 * 只有相对理想线积分, 才能把"转向残余造成的横移"算进去,
                 * 再慢慢把车拐回线上。
                 * ★ 只对前进段生效: 后退时速度方向相反, 航向偏差造成的
                 *   横移方向也相反, 同一套符号会把车越拐越偏。 */
                if (s_cur_move == (uint8_t)IR_MOVE_FWD)
                {
                    s_xt_accum += (s_ideal_yaw - jy901s_data.yaw_continuous)
                                  * IR_XT_DT;

                    if (s_xt_accum > IR_XT_ACC_MAX)
                    {
                        s_xt_accum = IR_XT_ACC_MAX;
                    }
                    else if (s_xt_accum < -IR_XT_ACC_MAX)
                    {
                        s_xt_accum = -IR_XT_ACC_MAX;
                    }

                    /* 补偿角从**直行目标(实测角)**起叠 —— 段首 bias=0,
                     * 所以起跑瞬间 err 仍是 0, 不会猛修; 之后 bias 慢慢
                     * 长大, 车才被温和地拐回理想线。 */
                    YawLoop_SetTarget(s_home_yaw + IR_XT_GAIN * s_xt_accum);
                }
#endif

                ir_apply_move();
            }
            break;

        /* ---- 原地转向中 ---- */
        case IR_TURNING:
            if (s_running == 0u)
            {
                break;
            }

            if (YawLoop_IsImuAlive() == 0)
            {
                ir_begin_stop("IMU LOST");
                break;
            }

            /* 到位判据: 剩余角度进"容差 + 提前停补偿"带就算转完。
             * ★ 加 IR_TURN_LEAD_DEG 是因为停车有延迟(陀螺 20ms 滞后 +
             *   指令排队 + 驱动器减速), 车实际会转过头一两度。提前发停车
             *   让惯性正好停在目标上, 下一段直行才不会带着反向误差起步。
             * ★ 仍然只判一个方向(转过目标也算到位), 否则会绕着目标来回摆。
             * ★ 转向没有时间上限: 只有这一条判据能把车停下来。转不到位
             *   就一直转; 陀螺仪掉线由上面的 IMU LOST 兜底。 */
            {
                float stop_win = IR_TURN_TOL_DEG + IR_TURN_LEAD_DEG;
                float now_yaw  = ir_turn_yaw_now();

                if ((s_turn_target - now_yaw <= stop_win)
                    && (s_turn_target - now_yaw >= -stop_win))
                {
                    ir_begin_stop("TURNED");
                    break;
                }
            }

            if ((int32_t)(now - s_next_steer_tick) >= 0)
            {
                s_next_steer_tick = now + IR_STEER_MS;
                ir_apply_turn();
            }
            break;

        /* ---- 发停车 + 等队列排空 + 等稳定 ---- */
        case IR_STOPPING:
            if (s_action_pushed == 0u)
            {
                Car_Stop();
                s_action_pushed = 1u;
                s_settle_armed = 0u;
                s_status = "STOPPING";
                break;
            }

            /* 队列里还有帧没发出去, 继续等 */
            if (Car_TxBusy() != 0u)
            {
                break;
            }

            /* 队列排空了, 开始计稳定时间 */
            if (s_settle_armed == 0u)
            {
                s_settle_armed = 1u;
                s_deadline = now + IR_STOP_SETTLE_MS;
                break;
            }

            if ((int32_t)(now - s_deadline) < 0)
            {
                break;
            }

            s_action_pushed = 0u;
            s_settle_armed = 0u;
            s_verify_phase = IR_VERIFY_WAIT;
            s_deadline = now;
            s_state = IR_VERIFY;
            break;

        /* ---- 编码器确认停车 ---- */
        case IR_VERIFY:
            ir_verify(now);
            break;

        case IR_IDLE:
        default:
            break;
    }
}

void InertialRun_GetInfo(IrInfo_t *info)
{
    uint8_t i;

    if (info == NULL)
    {
        return;
    }

    info->state = s_state;
    info->mode = s_mode;
    info->index = s_index;
    info->total = s_total;
    info->move = s_cur_move;
    info->running = s_running;
    info->duration_ms = s_cur_ms;
    info->elapsed_ms = (s_running != 0u) ? (HAL_GetTick() - s_start_tick) : 0u;
    info->turn_remain_deg = (s_state == IR_TURNING)
                          ? (s_turn_target - jy901s_data.yaw_continuous)
                          : 0.0f;
    info->turn_yaw_hold = s_yaw_hold;
    info->status = s_status;
    info->result = s_result;

    for (i = 0u; i < IR_MOTOR_NUM; i++)
    {
        info->position[i] = s_position[i];
        info->position_valid[i] = s_position_valid[i];
    }
}

const char *InertialRun_StateName(IrState_t state)
{
    static const char *const names[] =
    {
        "IDLE", "ZERO", "WAIT", "RUN", "TURN", "STOP", "VERIFY"
    };

    if ((uint8_t)state >= (uint8_t)(sizeof(names) / sizeof(names[0])))
    {
        return "IDLE";
    }

    return names[(uint8_t)state];
}

const char *InertialRun_MoveName(uint8_t move)
{
    static const char *const names[] =
    {
        "FWD", "BACK", "TURN_L", "TURN_R", "LEFT", "RIGHT"
    };

    if (move >= (uint8_t)(sizeof(names) / sizeof(names[0])))
    {
        return "FWD";
    }

    return names[move];
}
