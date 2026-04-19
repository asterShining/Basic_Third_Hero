#include "shoot_private.h"

#include "bsp_log.h"
#include "bsp_usart.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#define SHOOT_VOFA_TX_INTERVAL_MS 10.0f
#define SHOOT_VOFA_TX_BUFFER_BYTES 512u

// VOFA 运行时只保存“当前目标、当前事务计时和最近一帧发送节拍”，目的是把量化记录状态收口到独立文件里，避免这些观测变量扩散进主控制逻辑。
typedef struct {
    float target_inner_mps;
    float target_outer_mps;
    float request_accept_time_ms;
    float feed_start_time_ms;
    float shot_finish_time_ms;
    float last_ready_wait_ms;
    float last_feed_ms;
    float last_recover_ms;
    uint8_t last_shot_valid;
    uint32_t tx_drop_count;
    float last_tx_time_ms;
} ShootVofaRuntime_s;

// 这里直接复用 BSP 的串口实例结构体，但只填写发送侧句柄，作用是沿用现有 DMA 发送和 ready 判定接口；
// 原因是 VOFA 只需要发，不需要注册新的接收服务，更不应该去抢占当前空闲串口的 RX DMA。
static USARTInstance shoot_vofa_usart = { 0 };
// 这里保留一块静态发送缓冲区，作用是 DMA 发送期间数据地址必须稳定存在；
// 原因是若把 FireWater 文本放在栈上，函数返回后 DMA 还没发完就会读到脏数据。
static char shoot_vofa_tx_buffer[SHOOT_VOFA_TX_BUFFER_BYTES] = { 0 };
// 这里把 VOFA 运行时状态集中保存在独立静态区，作用是单发多阶段计时与串口限频都需要跨拍记忆。
static ShootVofaRuntime_s shoot_vofa_runtime = { 0 };

/**
 * @brief 把浮点量格式化成 VOFA 使用的字符串
 * @param out 缓冲区
 * @param value 待格式化的值
 */
static void ShootVofaFormatFloat(char *out, float value)
{
    if (out == NULL)
        return;

    // 这里统一复用现有 Float2Str，作用是避免在嵌入式固件里直接启用 `%f` 带来的体积和性能开销；
    // 原因是当前项目日志侧已经约定浮点全部先转字符串，VOFA 文本输出保持同一套做法最稳。
    Float2Str(out, value);
}

/**
 * @brief 计算当前内圈平均掉速量并换算成 m/s
 * @return 当前内圈平均掉速，单位 m/s
 */
static float ShootVofaGetInnerDipAvgMps(void)
{
    float dip_avg_deg = 0.0f;

    if (single_fire.state != SF_FEEDING && single_fire.state != SF_RETRYING)
        return 0.0f;

    // 这里直接复用控制链维护的掉速基线，作用是让 VOFA 看到的掉速量与单发锁角判据完全同源；
    // 原因是若再走另一套基线，波形能看但和实际控制决策对不上，诊断价值会明显下降。
    dip_avg_deg =
        ((dip_control.inner_left_baseline - fabsf(GetMotorSpeedAps(friction_inner_left))) +
         (dip_control.inner_right_baseline - fabsf(GetMotorSpeedAps(friction_inner_right))) +
         (dip_control.inner_down_baseline - fabsf(GetMotorSpeedAps(friction_inner_down)))) *
        0.3333333f;

    if (dip_avg_deg < 0.0f)
        dip_avg_deg = 0.0f;

    return SpeedAps2Mps(dip_avg_deg);
}

/**
 * @brief 计算当前外圈平均掉速量并换算成 m/s
 * @return 当前外圈平均掉速，单位 m/s
 */
static float ShootVofaGetOuterDipAvgMps(void)
{
    float dip_avg_deg = 0.0f;

    if (single_fire.state != SF_FEEDING && single_fire.state != SF_RETRYING)
        return 0.0f;

    if (!HasOuterFrictionWheel())
        return 0.0f;

    // 这里对外圈掉速采用与内圈相同的基线差分，作用是后续可以直接比较“内圈先掉多少、外圈跟着掉多少”；
    // 原因是单发稳定性问题常常不是有没有掉速，而是两级摩擦轮掉速是否同步、是否恢复得回来。
    dip_avg_deg =
        ((dip_control.outer_left_baseline - fabsf(GetMotorSpeedAps(friction_outer_left))) +
         (dip_control.outer_right_baseline - fabsf(GetMotorSpeedAps(friction_outer_right))) +
         (dip_control.outer_down_baseline - fabsf(GetMotorSpeedAps(friction_outer_down)))) *
        0.3333333f;

    if (dip_avg_deg < 0.0f)
        dip_avg_deg = 0.0f;

    return SpeedAps2Mps(dip_avg_deg);
}

/**
 * @brief 初始化 VOFA 发送链路
 */
void ShootVofa_Init(void)
{
    memset(&shoot_vofa_runtime, 0, sizeof(shoot_vofa_runtime));
    memset(shoot_vofa_tx_buffer, 0, sizeof(shoot_vofa_tx_buffer));

    // 这里把输出口固定到 USART1，作用是与当前 UART6 上的 VT03/图像桥接彻底隔离；
    // 原因是用户已经明确要求保留 UART6 现有链路，VOFA 不能再去争用同一个物理串口。
    shoot_vofa_usart.usart_handle = &huart1;
    shoot_vofa_usart.recv_buff_size = 0u;
}

/**
 * @brief 同步当前拍的摩擦轮目标线速度
 * @param inner_mps 内圈目标线速度
 * @param outer_mps 外圈目标线速度
 */
void ShootVofa_UpdateTarget(float inner_mps, float outer_mps)
{
    // 这里保留上层原始线速度命令，作用是 VOFA 曲线直接显示“期望多少 m/s”而不是把角速度目标再反算一次；
    // 原因是反算会混入打滑补偿和 trim，用户看目标值时更关心命令语义而不是控制器内部量纲。
    shoot_vofa_runtime.target_inner_mps = inner_mps;
    shoot_vofa_runtime.target_outer_mps = outer_mps;
}

/**
 * @brief 记录一次单发事务被状态机接纳的时刻
 * @param current_time_ms 当前系统时间
 */
void ShootVofa_OnSingleFireAccepted(float current_time_ms)
{
    // 这里把事务起点定义为“进入待速态”的这一拍，作用是 ready_wait 只度量真正等待摩擦轮就绪的时间；
    // 原因是更早的按键边沿可能经历去抖和边沿锁存，不适合作为控制链内部等待时间的起算点。
    shoot_vofa_runtime.request_accept_time_ms = current_time_ms;
    shoot_vofa_runtime.feed_start_time_ms = 0.0f;
    shoot_vofa_runtime.shot_finish_time_ms = 0.0f;
}

/**
 * @brief 记录一次单发事务首次开始送弹的时刻
 * @param current_time_ms 当前系统时间
 */
void ShootVofa_OnFeedStart(float current_time_ms)
{
    if (shoot_vofa_runtime.feed_start_time_ms == 0.0f)
        shoot_vofa_runtime.feed_start_time_ms = current_time_ms;

    if (shoot_vofa_runtime.request_accept_time_ms > 0.0f) {
        // 这里仅在第一次真正进入送弹时结算 ready_wait，作用是把等待到速耗时与后续补发耗时彻底分离；
        // 原因是用户排查“点一下为什么时快时慢”时，首先要区分是卡在待速还是卡在送弹事务本身。
        shoot_vofa_runtime.last_ready_wait_ms = current_time_ms - shoot_vofa_runtime.request_accept_time_ms;
        shoot_vofa_runtime.request_accept_time_ms = 0.0f;
    }
}

/**
 * @brief 锁存最近一发的送弹耗时和成功标志
 * @param current_time_ms 当前系统时间
 * @param shot_success 是否确认出弹
 */
void ShootVofa_OnShotFinish(float current_time_ms, uint8_t shot_success)
{
    if (shoot_vofa_runtime.feed_start_time_ms > 0.0f)
        shoot_vofa_runtime.last_feed_ms = current_time_ms - shoot_vofa_runtime.feed_start_time_ms;
    else
        shoot_vofa_runtime.last_feed_ms = 0.0f;

    // 这里把最近一发是否成功单独锁存下来，作用是 VOFA 保存波形回放时能直接看出哪几笔事务是空发或超时；
    // 原因是仅靠 fire_count 递增无法识别失败事务，后续做统计时会把失败样本误吞掉。
    shoot_vofa_runtime.last_shot_valid = shot_success;
    shoot_vofa_runtime.feed_start_time_ms = 0.0f;

    if (shot_success != 0u) {
        shoot_vofa_runtime.shot_finish_time_ms = current_time_ms;
    } else {
        shoot_vofa_runtime.shot_finish_time_ms = 0.0f;
        shoot_vofa_runtime.last_recover_ms = 0.0f;
    }
}

/**
 * @brief 记录最近一发的回速耗时
 * @param current_time_ms 当前系统时间
 */
void ShootVofa_OnRecoverDone(float current_time_ms)
{
    if (shoot_vofa_runtime.shot_finish_time_ms > 0.0f)
        shoot_vofa_runtime.last_recover_ms = current_time_ms - shoot_vofa_runtime.shot_finish_time_ms;
    else
        shoot_vofa_runtime.last_recover_ms = 0.0f;

    // 这里在 recover 结算后清掉瞬态起点，作用是下一发事务必须重新开始计时；
    // 原因是回速完成已经标记上一发彻底收口，继续保留旧时间戳只会污染后续事务。
    shoot_vofa_runtime.shot_finish_time_ms = 0.0f;
}

/**
 * @brief 清空当前未完成事务的瞬态计时
 */
void ShootVofa_ResetTransactionState(void)
{
    // 这里故意只清空“尚未完成事务”的瞬态时间戳，作用是保留最近一次已经收口事务的量化结果；
    // 原因是操作者停火或急停后通常还要回看上一发数据，不能因为中途 abort 就把最近一次有效结果一起抹掉。
    shoot_vofa_runtime.request_accept_time_ms = 0.0f;
    shoot_vofa_runtime.feed_start_time_ms = 0.0f;
    shoot_vofa_runtime.shot_finish_time_ms = 0.0f;
}

/**
 * @brief 按 100Hz 节拍通过 FireWater 协议发送一帧 VOFA 数据
 * @param current_time_ms 当前系统时间
 */
void ShootVofa_SendFrameIfDue(float current_time_ms)
{
    char t_ms_str[20];
    char target_inner_str[20];
    char target_outer_str[20];
    char inner_left_str[20];
    char inner_right_str[20];
    char inner_down_str[20];
    char outer_left_str[20];
    char outer_right_str[20];
    char outer_down_str[20];
    char inner_avg_str[20];
    char outer_avg_str[20];
    char inner_dip_str[20];
    char outer_dip_str[20];
    char ready_wait_str[20];
    char feed_ms_str[20];
    char recover_ms_str[20];
    int tx_len;

    if ((current_time_ms - shoot_vofa_runtime.last_tx_time_ms) < SHOOT_VOFA_TX_INTERVAL_MS)
        return;

    // 这里无论本次是否真正发出去都先推进节拍，作用是把输出频率硬限制在 100Hz；
    // 原因是 DMA 忙时若不推进时间窗，下一拍又会立刻重试，最终尝试频率会升到控制周期频率而不是目标观测频率。
    shoot_vofa_runtime.last_tx_time_ms = current_time_ms;

    if (shoot_vofa_usart.usart_handle == NULL)
        return;

    if (USARTIsReady(&shoot_vofa_usart) == 0u) {
        shoot_vofa_runtime.tx_drop_count++;
        return;
    }

    ShootVofaFormatFloat(t_ms_str, current_time_ms);
    ShootVofaFormatFloat(target_inner_str, shoot_vofa_runtime.target_inner_mps);
    ShootVofaFormatFloat(target_outer_str, shoot_vofa_runtime.target_outer_mps);
    ShootVofaFormatFloat(inner_left_str, SpeedAps2Mps(GetMotorSpeedAps(friction_inner_left)));
    ShootVofaFormatFloat(inner_right_str, SpeedAps2Mps(GetMotorSpeedAps(friction_inner_right)));
    ShootVofaFormatFloat(inner_down_str, SpeedAps2Mps(GetMotorSpeedAps(friction_inner_down)));
    ShootVofaFormatFloat(outer_left_str, SpeedAps2Mps(GetMotorSpeedAps(friction_outer_left)));
    ShootVofaFormatFloat(outer_right_str, SpeedAps2Mps(GetMotorSpeedAps(friction_outer_right)));
    ShootVofaFormatFloat(outer_down_str, SpeedAps2Mps(GetMotorSpeedAps(friction_outer_down)));
    ShootVofaFormatFloat(inner_avg_str, SpeedAps2Mps(GetInnerFrictionAvgSpeed()));
    ShootVofaFormatFloat(outer_avg_str, SpeedAps2Mps(GetOuterFrictionAvgSpeed()));
    ShootVofaFormatFloat(inner_dip_str, ShootVofaGetInnerDipAvgMps());
    ShootVofaFormatFloat(outer_dip_str, ShootVofaGetOuterDipAvgMps());
    ShootVofaFormatFloat(ready_wait_str, shoot_vofa_runtime.last_ready_wait_ms);
    ShootVofaFormatFloat(feed_ms_str, shoot_vofa_runtime.last_feed_ms);
    ShootVofaFormatFloat(recover_ms_str, shoot_vofa_runtime.last_recover_ms);

    // 这里固定用 FireWater 文本帧输出，作用是让串口抓包、VOFA 波形和人工排查三种场景共用同一份数据；
    // 原因是当前字段既有连续量也有状态量，文本协议在 100Hz 下带宽完全够用，而且临时增删字段最方便。
    tx_len = snprintf(
        shoot_vofa_tx_buffer,
        sizeof(shoot_vofa_tx_buffer),
        "%s,%d,%u,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%u,%u\r\n",
        t_ms_str,
        (int)single_fire.state,
        (unsigned int)single_fire.fire_count,
        target_inner_str,
        target_outer_str,
        inner_left_str,
        inner_right_str,
        inner_down_str,
        outer_left_str,
        outer_right_str,
        outer_down_str,
        inner_avg_str,
        outer_avg_str,
        inner_dip_str,
        outer_dip_str,
        ready_wait_str,
        feed_ms_str,
        recover_ms_str,
        (unsigned int)shoot_vofa_runtime.last_shot_valid,
        (unsigned int)shoot_feedback_data.empty_flag);

    if (tx_len <= 0)
        return;

    if ((size_t)tx_len >= sizeof(shoot_vofa_tx_buffer))
        tx_len = (int)(sizeof(shoot_vofa_tx_buffer) - 1u);

    USARTSend(&shoot_vofa_usart, (uint8_t *)shoot_vofa_tx_buffer, (uint16_t)tx_len, USART_TRANSFER_DMA);
}
