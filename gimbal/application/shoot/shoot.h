#ifndef SHOOT_H
#define SHOOT_H

#include "robot_def.h"
#include "shoot_debug.h"
#include <stdint.h>

// 摩擦轮半径 (单位: 米), 例如 30mm = 0.03m
#define FRICTION_WHEEL_RADIUS 0.03f
// 打滑补偿系数 (需要实测微调, 通常在 1.0 - 1.2 之间)
#define SLIP_COMPENSATION 1.00f

// [新增] 摩擦轮软启动步长 (deg/loop)
// 假设 200Hz 控制频率，15m/s (约28000dps)
// 设为 150.0f 表示约 1秒 达到满速
#define FRICTION_RAMP_STEP 100.0f

// [新增] 摩擦轮前馈控制参数
#define FRICTION_FEEDFORWARD_CURRENT 500 // 前馈电流值
#define FRICTION_FEEDFORWARD_TIME 500 // 前馈持续时间 (ms)

// ==================== 堵转检测参数 ====================
// 堵转检测电流阈值 (raw值, M3508满量程16384, 设为 ~80% 高阈值使堵转处理更激烈)
#define STALL_CURRENT_THRESHOLD 15000
// 堵转检测速度阈值 (deg/s), 低于此值且电流高则判定为堵转
#define STALL_SPEED_THRESHOLD 400.0f
// 堵转检测消抖时间 (ms), 持续满足条件才确认堵转
#define STALL_DETECT_TIME 1500
// 反转角度 (deg), 约为1/2颗弹丸角度,足够解卡但用户无感
#define REVERSE_ANGLE 2 * ONE_BULLET_DELTA_ANGLE
// 反转持续时间 (ms)
#define REVERSE_TIME 500
// 恢复等待时间 (ms), 反转后等待稳定再继续供弹
#define RECOVERY_TIME 80
// 连续反转次数上限, 超过则认为卡死,停止尝试
#define MAX_REVERSE_COUNT 5

// ==================== 单发控制参数 ====================
// 送弹速度 (deg/s), 中速稳定推弹, 给检测留足时间
#define SF_FEED_SPEED 13000.0f
// 制动速度 (deg/s), 负值反向制动, 抵消惯性防止第二颗进入
#define SF_BRAKE_SPEED -1000.0f
// 制动持续时间 (ms), 反向制动的持续时长
#define SF_BRAKE_TIME 100
// 送弹超时时间 (ms), 超时未检测到掉速则认为缺弹或卡弹
#define SF_FEED_TIMEOUT 9500

// ==================== 发射确认检测参数 ====================
// 掉速检测阈值 (deg/s), 内圈摩擦轮速度下降超过此值认为有弹丸通过
#define FRICTION_SPEED_DIP_THRESHOLD 1000.0f
// 回升检测阈值 (deg/s), 与目标速度差小于此值认为回升完成
#define FRICTION_SPEED_RECOVER_THRESHOLD 100.0f
// [新增] 射速就绪阈值 (deg/s), 实际速度与目标速度误差小于此值才允许发射
#define SHOOT_SPEED_READY_THRESHOLD 500.0f

// 堵转检测状态结构体 (运行时数据, 仅供 shoot.c 内部使用)
static struct {
    LoaderStallState_e state; // 当前状态
    float detect_start_time; // 开始检测堵转的时间戳 (ms)
    float reverse_start_time; // 开始反转的时间戳 (ms)
    float recovery_start_time; // 开始恢复的时间戳 (ms)
    float reverse_target_angle; // 反转目标角度
    uint8_t reverse_count; // 连续反转次数
    loader_mode_e saved_mode; // 保存的原发射模式
} stall_handler = { 0 };

// 单发控制状态结构体 (运行时数据, 仅供 shoot.c 内部使用)
static struct {
    SingleFireState_e state; // 当前状态
    float baseline_speed; // 触发时的基准摩擦轮速度 (内圈)
    float outer_baseline_speed; // 触发时的基准摩擦轮速度 (外圈)
    float feed_start_time; // 送弹开始时间戳 (ms)
    float brake_start_time; // 制动开始时间戳 (ms)
    float cooldown_start_time; // 冷却开始时间戳 (ms)
    uint16_t fire_count; // 累计发射计数
    uint16_t feed_timeout_count; // 送弹超时计数
} single_fire = { 0 };

// ==================== 单发触发边沿检测 ====================
// 用于确保每次触发只响应一次，防止持续按住导致多发
typedef struct {
    loader_mode_e last_mode; // 上一次的拨盘模式 (用于边沿检测)
    uint8_t trigger_consumed; // 触发是否已被消费 (1=已消费,等待复位)
    uint8_t pending_fire; // 是否有待处理的发射请求 (1=有)
} FireTrigger_s;

static FireTrigger_s fire_trigger = { .last_mode = LOAD_STOP, .trigger_consumed = 0, .pending_fire = 0 };

/**
 * @brief 发射初始化,会被RobotInit()调用
 *
 */
void ShootInit();

/**
 * @brief 发射任务
 *
 */
void ShootTask();

#endif // SHOOT_H