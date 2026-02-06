/**
 * @file shoot_debug.c
 * @brief 发射机构调试模块实现
 * @note 集中管理所有发射相关的调试逻辑
 *       包括摩擦轮调试、堵转调试、单发调试、掉速抓拍等功能
 */

#include "shoot_debug.h"
#include "bsp_dwt.h"
#include <math.h>

// ==================== 全局调试变量定义 ====================
// 摩擦轮调试数据 (供调试器观测和手动覆盖)
static FrictionWheelDebug_s friction_debug = { 0 };

// 堵转调试数据 (供调试器观测堵转状态机)
static StallDebug_s stall_debug = { 0 };

// 单发调试数据 (供调试器观测单发状态机)
static SingleFireDebug_s sf_debug = { 0 };

// 掉速抓拍数据 (记录最近一次弹丸经过时6个电机的掉速情况)
static BulletDipSnapshot_s dip_snapshot = { 0 };

// 掉速历史记录数组 (保存最近N发用于统计分析)
static BulletDipSnapshot_s dip_history[DIP_HISTORY_SIZE] = { 0 };
static uint8_t dip_history_index = 0;

// 摩擦轮半径 (单位: 米), 例如 30mm = 0.03m (与 shoot.h 保持一致)
#define FRICTION_WHEEL_RADIUS_LOCAL 0.03f
// 弧度转角度系数
#define RAD_2_DEGREE_LOCAL 57.295779513f

// 掉速检测内部状态 (用于记录基准速度和检测时序)
static struct {
    float baseline_inner_left; // 送弹开始时的内圈左基准速度 (deg/s)
    float baseline_inner_right; // 送弹开始时的内圈右基准速度 (deg/s)
    float baseline_inner_down; // 送弹开始时的内圈下基准速度 (deg/s)
    float baseline_outer_left; // 送弹开始时的外圈左基准速度 (deg/s)
    float baseline_outer_right; // 送弹开始时的外圈右基准速度 (deg/s)
    float baseline_outer_down; // 送弹开始时的外圈下基准速度 (deg/s)
    float inner_dip_time; // 内圈首次检测到掉速的时间 (ms)
    float outer_dip_time; // 外圈首次检测到掉速的时间 (ms)
    uint8_t inner_dipping; // 内圈是否正在掉速 (1=是)
    uint8_t outer_dipping; // 外圈是否正在掉速 (1=是)
    uint8_t snapshot_taken; // 本次发射是否已抓拍 (防止重复抓拍)
} dip_detector = { 0 };

// ==================== 调试指针接口实现 ====================

/**
 * @brief 获取摩擦轮调试结构体指针
 * @return 指向摩擦轮调试数据的指针
 */
FrictionWheelDebug_s *ShootDebug_GetFrictionPtr(void)
{
    return &friction_debug;
}

/**
 * @brief 获取堵转调试结构体指针
 * @return 指向堵转调试数据的指针
 */
StallDebug_s *ShootDebug_GetStallPtr(void)
{
    return &stall_debug;
}

/**
 * @brief 获取单发调试结构体指针
 * @return 指向单发调试数据的指针
 */
SingleFireDebug_s *ShootDebug_GetSingleFirePtr(void)
{
    return &sf_debug;
}

/**
 * @brief 获取掉速抓拍结构体指针
 * @return 指向最近一次掉速抓拍数据的指针
 */
BulletDipSnapshot_s *ShootDebug_GetDipSnapshotPtr(void)
{
    return &dip_snapshot;
}

/**
 * @brief 获取掉速历史记录数组指针
 * @param out_index 输出当前历史记录索引
 * @return 指向掉速历史记录数组的指针
 */
BulletDipSnapshot_s *ShootDebug_GetDipHistoryPtr(uint8_t *out_index)
{
    if (out_index != NULL) {
        *out_index = dip_history_index;
    }
    return dip_history;
}

// ==================== 辅助函数 ====================

/**
 * @brief 辅助函数：将角速度转换为线速度 (deg/s -> m/s)
 * @note 不包含打滑补偿，反映电机轴的理论线速度
 */
static float SpeedAps2Mps_Local(float speed_aps)
{
    // 公式: (角速度 / (180/PI)) * 半径
    // v = w * r
    return (speed_aps / RAD_2_DEGREE_LOCAL) * FRICTION_WHEEL_RADIUS_LOCAL;
}

// ==================== 调试功能函数实现 ====================

/**
 * @brief 更新摩擦轮调试数据 (将电机反馈的角速度转换为线速度)
 * @note 此函数由 shoot.c 周期性调用，传入6个电机的实时速度
 */
void ShootDebug_UpdateFrictionInfo(float inner_left_aps, float inner_right_aps, float inner_down_aps,
                                   float outer_left_aps, float outer_right_aps, float outer_down_aps)
{
    friction_debug.inner_left_mps = SpeedAps2Mps_Local(inner_left_aps);
    friction_debug.inner_right_mps = SpeedAps2Mps_Local(inner_right_aps);
    friction_debug.inner_down_mps = SpeedAps2Mps_Local(inner_down_aps);
    friction_debug.outer_left_mps = SpeedAps2Mps_Local(outer_left_aps);
    friction_debug.outer_right_mps = SpeedAps2Mps_Local(outer_right_aps);
    friction_debug.outer_down_mps = SpeedAps2Mps_Local(outer_down_aps);
}

/**
 * @brief 记录6个电机的基准速度 (在送弹开始时调用)
 * @note 基准速度用于后续计算掉速量, 取电机速度的绝对值
 */
void ShootDebug_RecordDipBaseline(float inner_left_aps, float inner_right_aps, float inner_down_aps,
                                  float outer_left_aps, float outer_right_aps, float outer_down_aps)
{
    // 记录6个电机的当前速度作为基准 (取绝对值)
    dip_detector.baseline_inner_left = fabsf(inner_left_aps);
    dip_detector.baseline_inner_right = fabsf(inner_right_aps);
    dip_detector.baseline_inner_down = fabsf(inner_down_aps);
    dip_detector.baseline_outer_left = fabsf(outer_left_aps);
    dip_detector.baseline_outer_right = fabsf(outer_right_aps);
    dip_detector.baseline_outer_down = fabsf(outer_down_aps);

    // 复位检测状态
    dip_detector.inner_dipping = 0;
    dip_detector.outer_dipping = 0;
    dip_detector.inner_dip_time = 0;
    dip_detector.outer_dip_time = 0;
    dip_detector.snapshot_taken = 0;
}

/**
 * @brief 检查是否已完成本次抓拍 (防止重复抓拍)
 * @return 1=已抓拍, 0=未抓拍
 */
uint8_t ShootDebug_IsSnapshotTaken(void)
{
    return dip_detector.snapshot_taken;
}

/**
 * @brief 执行掉速抓拍 (当检测到掉速时调用)
 * @note 记录弹丸经过瞬间6个电机各自的掉速情况
 *       只在首次检测到掉速时抓拍一次, 避免重复
 */
void ShootDebug_TakeDipSnapshot(float inner_left_aps, float inner_right_aps, float inner_down_aps,
                                float outer_left_aps, float outer_right_aps, float outer_down_aps,
                                uint16_t fire_count)
{
    // 防止本次发射重复抓拍
    if (dip_detector.snapshot_taken)
        return;
    dip_detector.snapshot_taken = 1;

    float current_time = DWT_GetTimeline_ms();

    // --- 填充抓拍时间戳 ---
    dip_snapshot.snapshot_time_ms = current_time;
    dip_snapshot.shot_index = fire_count + 1; // +1 因为还没累加

    // --- 记录基准速度 ---
    dip_snapshot.baseline_inner_left = dip_detector.baseline_inner_left;
    dip_snapshot.baseline_inner_right = dip_detector.baseline_inner_right;
    dip_snapshot.baseline_inner_down = dip_detector.baseline_inner_down;
    dip_snapshot.baseline_outer_left = dip_detector.baseline_outer_left;
    dip_snapshot.baseline_outer_right = dip_detector.baseline_outer_right;
    dip_snapshot.baseline_outer_down = dip_detector.baseline_outer_down;

    // --- 记录掉速瞬间速度 (当前速度, 取绝对值) ---
    dip_snapshot.dip_inner_left = fabsf(inner_left_aps);
    dip_snapshot.dip_inner_right = fabsf(inner_right_aps);
    dip_snapshot.dip_inner_down = fabsf(inner_down_aps);
    dip_snapshot.dip_outer_left = fabsf(outer_left_aps);
    dip_snapshot.dip_outer_right = fabsf(outer_right_aps);
    dip_snapshot.dip_outer_down = fabsf(outer_down_aps);

    // --- 计算各电机掉速量 (baseline - dip, 正值表示掉速) ---
    dip_snapshot.delta_inner_left = dip_snapshot.baseline_inner_left - dip_snapshot.dip_inner_left;
    dip_snapshot.delta_inner_right = dip_snapshot.baseline_inner_right - dip_snapshot.dip_inner_right;
    dip_snapshot.delta_inner_down = dip_snapshot.baseline_inner_down - dip_snapshot.dip_inner_down;
    dip_snapshot.delta_outer_left = dip_snapshot.baseline_outer_left - dip_snapshot.dip_outer_left;
    dip_snapshot.delta_outer_right = dip_snapshot.baseline_outer_right - dip_snapshot.dip_outer_right;
    dip_snapshot.delta_outer_down = dip_snapshot.baseline_outer_down - dip_snapshot.dip_outer_down;

    // --- 统计分析 ---
    // 内圈平均掉速
    dip_snapshot.delta_inner_avg = (dip_snapshot.delta_inner_left +
                                    dip_snapshot.delta_inner_right +
                                    dip_snapshot.delta_inner_down) /
                                   3.0f;
    // 外圈平均掉速
    dip_snapshot.delta_outer_avg = (dip_snapshot.delta_outer_left +
                                    dip_snapshot.delta_outer_right +
                                    dip_snapshot.delta_outer_down) /
                                   3.0f;
    // 左右掉速差异: (左侧平均) - (右侧平均)
    // 正值表示左侧掉速更多 (弹丸偏右), 负值表示右侧掉速更多 (弹丸偏左)
    float left_avg = (dip_snapshot.delta_inner_left + dip_snapshot.delta_outer_left) / 2.0f;
    float right_avg = (dip_snapshot.delta_inner_right + dip_snapshot.delta_outer_right) / 2.0f;
    dip_snapshot.delta_left_right_diff = left_avg - right_avg;

    // 统计有效掉速的电机数量 (掉速量超过噪声阈值)
    dip_snapshot.valid_dip_count = 0;
    if (dip_snapshot.delta_inner_left > DIP_NOISE_THRESHOLD)
        dip_snapshot.valid_dip_count++;
    if (dip_snapshot.delta_inner_right > DIP_NOISE_THRESHOLD)
        dip_snapshot.valid_dip_count++;
    if (dip_snapshot.delta_inner_down > DIP_NOISE_THRESHOLD)
        dip_snapshot.valid_dip_count++;
    if (dip_snapshot.delta_outer_left > DIP_NOISE_THRESHOLD)
        dip_snapshot.valid_dip_count++;
    if (dip_snapshot.delta_outer_right > DIP_NOISE_THRESHOLD)
        dip_snapshot.valid_dip_count++;
    if (dip_snapshot.delta_outer_down > DIP_NOISE_THRESHOLD)
        dip_snapshot.valid_dip_count++;
}

/**
 * @brief 验证掉速快照的有效性并保存到历史记录
 * @note 有效判定条件:
 *       1. 有足够数量的电机同时掉速 (>= DIP_MIN_MOTOR_COUNT)
 *       2. 掉速量足够大 (超过噪声阈值)
 *       3. 内外圈掉速时间一致 (弹丸物理上连续通过)
 */
void ShootDebug_ValidateAndSave(void)
{
    // 默认有效
    dip_snapshot.is_valid_shot = 1;
    dip_snapshot.validity_reason = DIP_VALID;

    // --- 检查1: 有效掉速电机数量 ---
    // 真实弹丸会同时挤压多个摩擦轮, 导致多个电机同时掉速
    // 噪声通常只影响单个电机或不同步
    if (dip_snapshot.valid_dip_count < DIP_MIN_MOTOR_COUNT) {
        dip_snapshot.is_valid_shot = 0;
        dip_snapshot.validity_reason = DIP_INVALID_TOO_FEW_MOTORS;
        goto save_history;
    }

    // --- 检查2: 掉速量是否足够大 ---
    // 内圈和外圈的平均掉速都要超过噪声阈值
    // 只有其中一个超过也可能是有效的 (弹丸可能只经过一级就被发射)
    float max_avg_dip = (dip_snapshot.delta_inner_avg > dip_snapshot.delta_outer_avg) ?
                            dip_snapshot.delta_inner_avg :
                            dip_snapshot.delta_outer_avg;
    if (max_avg_dip < DIP_NOISE_THRESHOLD) {
        dip_snapshot.is_valid_shot = 0;
        dip_snapshot.validity_reason = DIP_INVALID_TOO_SMALL;
        goto save_history;
    }

    // --- 检查3: 内外圈掉速时间一致性 ---
    // 弹丸物理上是连续通过内圈再到外圈的, 时间差应该很小
    // 如果时间差过大, 说明可能是随机噪声而非真实弹丸
    if (dip_detector.inner_dip_time > 0 && dip_detector.outer_dip_time > 0) {
        float time_diff = fabsf(dip_detector.outer_dip_time - dip_detector.inner_dip_time);
        if (time_diff > DIP_TIME_COHERENCE_MS) {
            dip_snapshot.is_valid_shot = 0;
            dip_snapshot.validity_reason = DIP_INVALID_INCOHERENT;
            goto save_history;
        }
    }

    // 通过所有检查, 确认为有效发射

save_history:
    // 保存到历史记录 (循环覆盖, 始终保留最近 DIP_HISTORY_SIZE 条记录)
    dip_history[dip_history_index] = dip_snapshot;
    dip_history_index = (dip_history_index + 1) % DIP_HISTORY_SIZE;
}

/**
 * @brief 更新单发调试信息
 */
void ShootDebug_UpdateSingleFireInfo(SingleFireState_e state, float baseline, float outer_baseline,
                                     float current, float current_outer, float loader_speed,
                                     uint8_t is_dipping, uint8_t trigger_edge,
                                     uint16_t fire_count, uint16_t timeout_count,
                                     float feed_start, float brake_start)
{
    sf_debug.state = state;
    sf_debug.baseline_speed = baseline;
    sf_debug.outer_baseline_speed = outer_baseline;
    sf_debug.current_speed = current;
    sf_debug.current_outer_speed = current_outer;
    sf_debug.speed_diff = baseline - current;
    sf_debug.loader_speed = loader_speed;
    sf_debug.is_dipping = is_dipping;
    sf_debug.trigger_edge = trigger_edge;
    sf_debug.fire_count = fire_count;
    sf_debug.feed_timeout_count = timeout_count;
    sf_debug.feed_start_time = feed_start;
    sf_debug.brake_start_time = brake_start;
}

/**
 * @brief 更新堵转调试信息
 */
void ShootDebug_UpdateStallInfo(LoaderStallState_e state, int16_t current_abs, float speed_abs,
                                uint8_t is_stalled, uint8_t reverse_count, float reverse_target)
{
    stall_debug.state = state;
    stall_debug.current_abs = current_abs;
    stall_debug.speed_abs = speed_abs;
    stall_debug.is_stalled = is_stalled;
    stall_debug.reverse_count = reverse_count;
    stall_debug.reverse_target_angle = reverse_target;
}
