/**
 * @file shoot_debug.h
 * @brief 发射机构调试模块头文件
 * @note 集中管理所有发射相关的调试结构体、变量和函数
 *       通过函数接口访问调试数据，避免过多 extern 声明
 */

#ifndef SHOOT_DEBUG_H
#define SHOOT_DEBUG_H

#include <stdint.h>
#include "robot_def.h"

// ==================== 摩擦轮调试结构体 ====================
// 用于调试器实时观测和手动覆盖摩擦轮速度
typedef struct {
    // 控制部分 (Debug Control)
    uint8_t override_enable; // [调试] 开启手动覆盖模式 (1=开启, 0=关闭)
    float target_inner_mps; // [调试] 手动设置内圈目标线速度 (m/s)
    float target_outer_mps; // [调试] 手动设置外圈目标线速度 (m/s)

    // 反馈部分 (Feedback)
    float inner_left_mps; // 内圈左实际线速度 (m/s)
    float inner_right_mps; // 内圈右实际线速度 (m/s)
    float inner_down_mps; // 内圈下实际线速度 (m/s)
    float outer_left_mps; // 外圈左实际线速度 (m/s)
    float outer_right_mps; // 外圈右实际线速度 (m/s)
    float outer_down_mps; // 外圈下实际线速度 (m/s)
} FrictionWheelDebug_s;

// ==================== 堵转调试结构体 ====================
// 堵转检测状态枚举
typedef enum {
    STALL_NORMAL = 0, // 正常状态
    STALL_DETECTING, // 疑似堵转,消抖中
    STALL_REVERSING, // 确认堵转,正在反转
    STALL_RECOVERY // 反转完成,恢复中
} LoaderStallState_e;

// 堵转调试信息结构体 (全局可观测)
typedef struct {
    LoaderStallState_e state; // 当前堵转状态机状态
    int16_t current_abs; // 当前电流绝对值 (用于调试观察)
    float speed_abs; // 当前速度绝对值 (用于调试观察)
    uint8_t is_stalled; // 是否处于堵转状态 (1=堵转, 0=正常)
    uint8_t reverse_count; // 连续反转次数
    float reverse_target_angle; // 反转目标角度
} StallDebug_s;

// ==================== 单发调试结构体 ====================
// 简化的单发状态枚举 (基于机械限位 + 掉速检测)
typedef enum {
    SF_IDLE = 0, // 空闲, 等待触发
    SF_FEEDING, // 速度环送弹中, 监测摩擦轮掉速
    SF_BRAKING, // 检测到掉速, 反向制动中
    SF_COOLDOWN, // 制动完成, 冷却等待
} SingleFireState_e;

// 单发调试信息结构体 (全局可观测, 用于调试器实时监控)
typedef struct {
    SingleFireState_e state; // 当前单发状态机状态
    float baseline_speed; // 触发时记录的基准摩擦轮速度 (deg/s) (内圈)
    float outer_baseline_speed; // 触发时记录的基准摩擦轮速度 (deg/s) (外圈)
    float current_speed; // 当前内圈摩擦轮平均速度 (deg/s)
    float current_outer_speed; // 当前外圈摩擦轮平均速度 (deg/s)
    float speed_diff; // 速度差 (baseline - current), 正值表示掉速
    float loader_speed; // 拨盘当前速度 (deg/s)
    float feed_start_time; // 送弹开始时间 (ms)
    float brake_start_time; // 制动开始时间 (ms)
    uint8_t is_dipping; // 是否检测到掉速 (1=掉速中)
    uint8_t trigger_edge; // 是否检测到触发边沿 (1=边沿触发)
    uint16_t fire_count; // 累计发射弹丸计数
    uint16_t feed_timeout_count; // 送弹超时计数 (可能缺弹)
} SingleFireDebug_s;

// ==================== 有效掉速识别参数 ====================
// 正常电机速度波动范围 (deg/s), 低于此值视为噪声而非弹丸掉速
#define DIP_NOISE_THRESHOLD 500.0f
// 有效掉速的最小电机数量, 至少N个电机同时掉速才认为是真实弹丸
#define DIP_MIN_MOTOR_COUNT 2
// 内外圈掉速时间差上限 (ms), 超过此值认为是异常波动而非弹丸通过
#define DIP_TIME_COHERENCE_MS 50.0f
// 有效掉速持续时间窗口 (ms), 在此时间内持续低于基准才确认
#define DIP_SUSTAIN_TIME_MS 10.0f
// 历史记录缓冲区大小 (保存最近N发的数据用于分析)
#define DIP_HISTORY_SIZE 10

// ==================== 弹丸掉速抓拍结构体 ====================
// 用于记录弹丸经过瞬间6个电机各自的掉速情况
typedef struct {
    // --- 抓拍时间戳 ---
    float snapshot_time_ms; // 抓拍时刻 (ms)
    uint16_t shot_index; // 第几发弹丸 (累计计数)

    // --- 基准速度 (发射前稳态, deg/s, 取绝对值) ---
    float baseline_inner_left; // 内圈左基准
    float baseline_inner_right; // 内圈右基准
    float baseline_inner_down; // 内圈下基准
    float baseline_outer_left; // 外圈左基准
    float baseline_outer_right; // 外圈右基准
    float baseline_outer_down; // 外圈下基准

    // --- 掉速瞬间速度 (deg/s, 取绝对值) ---
    float dip_inner_left; // 内圈左掉速时速度
    float dip_inner_right; // 内圈右掉速时速度
    float dip_inner_down; // 内圈下掉速时速度
    float dip_outer_left; // 外圈左掉速时速度
    float dip_outer_right; // 外圈右掉速时速度
    float dip_outer_down; // 外圈下掉速时速度

    // --- 掉速量 (baseline - dip, 正值表示掉速, deg/s) ---
    float delta_inner_left; // 内圈左掉速量
    float delta_inner_right; // 内圈右掉速量
    float delta_inner_down; // 内圈下掉速量
    float delta_outer_left; // 外圈左掉速量
    float delta_outer_right; // 外圈右掉速量
    float delta_outer_down; // 外圈下掉速量

    // --- 统计分析 ---
    float delta_inner_avg; // 内圈三电机掉速平均值 (deg/s)
    float delta_outer_avg; // 外圈三电机掉速平均值 (deg/s)
    float delta_left_right_diff; // 左侧与右侧掉速差异 (用于判断弹道偏移)
    uint8_t valid_dip_count; // 有效掉速的电机数量 (超过阈值的)

    // --- 有效性判定 ---
    uint8_t is_valid_shot; // 是否为有效发射 (1=有效, 0=可能误触发)
    uint8_t validity_reason; // 有效性判定原因代码 (见下方枚举)
} BulletDipSnapshot_s;

// 有效性判定原因枚举
typedef enum {
    DIP_VALID = 0, // 有效发射
    DIP_INVALID_TOO_FEW_MOTORS, // 掉速电机数不足
    DIP_INVALID_TOO_SMALL, // 掉速量太小 (噪声)
    DIP_INVALID_INCOHERENT, // 内外圈掉速时间不一致
    DIP_INVALID_TOO_SHORT, // 掉速持续时间太短
} DipValidityReason_e;

// ==================== 调试接口函数声明 ====================
// 使用函数接口获取调试数据指针，避免直接 extern 声明

/**
 * @brief 获取摩擦轮调试结构体指针
 * @return 指向摩擦轮调试数据的指针
 */
FrictionWheelDebug_s *ShootDebug_GetFrictionPtr(void);

/**
 * @brief 获取堵转调试结构体指针
 * @return 指向堵转调试数据的指针
 */
StallDebug_s *ShootDebug_GetStallPtr(void);

/**
 * @brief 获取单发调试结构体指针
 * @return 指向单发调试数据的指针
 */
SingleFireDebug_s *ShootDebug_GetSingleFirePtr(void);

/**
 * @brief 获取掉速抓拍结构体指针
 * @return 指向最近一次掉速抓拍数据的指针
 */
BulletDipSnapshot_s *ShootDebug_GetDipSnapshotPtr(void);

/**
 * @brief 获取掉速历史记录数组指针
 * @param out_index 输出当前历史记录索引
 * @return 指向掉速历史记录数组的指针
 */
BulletDipSnapshot_s *ShootDebug_GetDipHistoryPtr(uint8_t *out_index);

// ==================== 调试功能函数声明 ====================
// 供 shoot.c 调用的调试相关处理函数

/**
 * @brief 更新摩擦轮调试数据 (将电机反馈的角速度转换为线速度)
 * @param inner_left_aps 内圈左电机角速度 (deg/s)
 * @param inner_right_aps 内圈右电机角速度 (deg/s)
 * @param inner_down_aps 内圈下电机角速度 (deg/s)
 * @param outer_left_aps 外圈左电机角速度 (deg/s)
 * @param outer_right_aps 外圈右电机角速度 (deg/s)
 * @param outer_down_aps 外圈下电机角速度 (deg/s)
 */
void ShootDebug_UpdateFrictionInfo(float inner_left_aps, float inner_right_aps, float inner_down_aps,
                                   float outer_left_aps, float outer_right_aps, float outer_down_aps);

/**
 * @brief 记录6个电机的基准速度 (在送弹开始时调用)
 * @param inner_left_aps 内圈左电机角速度 (deg/s)
 * @param inner_right_aps 内圈右电机角速度 (deg/s)
 * @param inner_down_aps 内圈下电机角速度 (deg/s)
 * @param outer_left_aps 外圈左电机角速度 (deg/s)
 * @param outer_right_aps 外圈右电机角速度 (deg/s)
 * @param outer_down_aps 外圈下电机角速度 (deg/s)
 */
void ShootDebug_RecordDipBaseline(float inner_left_aps, float inner_right_aps, float inner_down_aps,
                                  float outer_left_aps, float outer_right_aps, float outer_down_aps);

/**
 * @brief 执行掉速抓拍 (当检测到掉速时调用)
 * @param inner_left_aps 内圈左当前速度 (deg/s)
 * @param inner_right_aps 内圈右当前速度 (deg/s)
 * @param inner_down_aps 内圈下当前速度 (deg/s)
 * @param outer_left_aps 外圈左当前速度 (deg/s)
 * @param outer_right_aps 外圈右当前速度 (deg/s)
 * @param outer_down_aps 外圈下当前速度 (deg/s)
 * @param fire_count 当前发射计数
 */
void ShootDebug_TakeDipSnapshot(float inner_left_aps, float inner_right_aps, float inner_down_aps,
                                float outer_left_aps, float outer_right_aps, float outer_down_aps,
                                uint16_t fire_count);

/**
 * @brief 验证掉速快照的有效性并保存到历史记录
 */
void ShootDebug_ValidateAndSave(void);

/**
 * @brief 检查是否已完成本次抓拍 (防止重复抓拍)
 * @return 1=已抓拍, 0=未抓拍
 */
uint8_t ShootDebug_IsSnapshotTaken(void);

/**
 * @brief 更新单发调试信息
 * @param state 当前状态
 * @param baseline 内圈基准速度
 * @param outer_baseline 外圈基准速度
 * @param current 内圈当前速度
 * @param current_outer 外圈当前速度
 * @param loader_speed 拨盘速度
 * @param is_dipping 是否掉速
 * @param trigger_edge 触发边沿
 * @param fire_count 发射计数
 * @param timeout_count 超时计数
 * @param feed_start 送弹开始时间
 * @param brake_start 制动开始时间
 */
void ShootDebug_UpdateSingleFireInfo(SingleFireState_e state, float baseline, float outer_baseline,
                                     float current, float current_outer, float loader_speed,
                                     uint8_t is_dipping, uint8_t trigger_edge,
                                     uint16_t fire_count, uint16_t timeout_count,
                                     float feed_start, float brake_start);

/**
 * @brief 更新堵转调试信息
 * @param state 堵转状态
 * @param current_abs 电流绝对值
 * @param speed_abs 速度绝对值
 * @param is_stalled 是否堵转
 * @param reverse_count 反转计数
 * @param reverse_target 反转目标角度
 */
void ShootDebug_UpdateStallInfo(LoaderStallState_e state, int16_t current_abs, float speed_abs,
                                uint8_t is_stalled, uint8_t reverse_count, float reverse_target);

#endif // SHOOT_DEBUG_H
