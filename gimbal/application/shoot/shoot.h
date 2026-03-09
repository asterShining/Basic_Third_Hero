#ifndef SHOOT_H
#define SHOOT_H

#include "robot_def.h"
#include "shoot_debug.h"
#include <stdint.h>

// 摩擦轮半径 (单位: 米), 例如 30mm = 0.03m
#define FRICTION_WHEEL_RADIUS 0.03f
// 打滑补偿系数 (需要实测微调, 通常在 1.0 - 1.2 之间)
#define SLIP_COMPENSATION 1.20f

// [新增] 摩擦轮软启动步长 (deg/loop)
// 假设 200Hz 控制频率，15m/s (约28000dps)
// 设为 150.0f 表示约 1秒 达到满速
#define FRICTION_RAMP_STEP 100.0f

// [新增] 摩擦轮前馈控制参数
#define FRICTION_FEEDFORWARD_CURRENT 500 // 前馈电流值
#define FRICTION_FEEDFORWARD_TIME 500 // 前馈持续时间 (ms)

// ==================== 堵转检测参数 ====================
// 堵转检测电流阈值 (raw值), 取较高阈值以减少正常发射时的误触发
// 原因是当前主要诉求是别让防堵转频繁打断单发事务，因此先回到更保守的触发电流
#define STALL_CURRENT_THRESHOLD 15000
// 堵转检测速度阈值 (deg/s), 只有速度明显塌到较低水平才认为疑似堵转
// 原因是阈值过高会把正常咬弹和负载波动误判成卡弹，破坏拨盘相位一致性
#define STALL_SPEED_THRESHOLD 400.0f
// 堵转检测消抖时间 (ms), 延长确认时间以降低误判反转概率
// 原因是本轮优先减少误触发而不是追求最激进的解卡响应
#define STALL_DETECT_TIME 600
// 反转角度 (deg), 约为 1/2 颗弹丸角度, 足够解卡但尽量不把正常节拍打乱
#define REVERSE_ANGLE (0.5f * ONE_BULLET_DELTA_ANGLE)
// 反转持续时间 (ms)
#define REVERSE_TIME 500
// 恢复等待时间 (ms), 保持现有恢复节拍避免在本轮同时改变过多行为
// 原因是这次重点是避免误触发和保留单发事务，不额外改恢复手感
#define RECOVERY_TIME 50
// 连续反转次数上限, 超过则认为卡死,停止尝试
#define MAX_REVERSE_COUNT 5

// ==================== 单发控制参数 ====================
// 单发固定截止行程 (单位: 发), 采用略小于整发的保守值提升防连发容错
// 原因是拨盘到固定相位即收口比“继续追整发再等掉速”更不容易把下一发带出来
#define SF_RUSH_BULLET_COUNT 0.95f
// 拨盘电机总角度对应的一发角度 (deg), 需要乘减速比, 因为 total_angle 是电机转子多圈角度
#define LOADER_MOTOR_ANGLE_PER_BULLET (ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER)
// 单发截止总角度 (deg), 用于把每次单发统一锁到固定拨盘相位
#define SF_RUSH_ANGLE (SF_RUSH_BULLET_COUNT * LOADER_MOTOR_ANGLE_PER_BULLET)
// 单发恒电流起步的目标电流 (raw), 保守起步先保证相位一致性而不是极限推弹
// 原因是开环电流过大会放大惯性过冲, 直接降低固定相位截止的防连发容错
#define SF_STARTUP_CURRENT_REF 10000.0f
// 单发恒电流阶段的最长持续时间 (ms), 超时仍未到截止角则立即收口防止拖泥带水
// 原因是单发不再依赖掉速等待或补发重试, 这里要给编码器异常和机械异常一个硬上限
#define SF_STARTUP_TIMEOUT 300.0f

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
    uint8_t single_fire_interrupted; // 单发事务是否被堵转流程打断
    SingleFireState_e single_fire_state_before_stall; // 堵转前的单发状态, 用于恢复原事务语义
    float single_fire_pause_start_time; // 堵转打断开始时间, 用于补偿单发计时
} stall_handler = { 0 };

// 单发控制状态结构体 (运行时数据, 仅供 shoot.c 内部使用)
static struct {
    SingleFireState_e state; // 当前状态
    float baseline_speed; // 触发时的基准摩擦轮速度 (内圈)
    float outer_baseline_speed; // 触发时的基准摩擦轮速度 (外圈)
    float rush_start_angle; // 冲刺起始角度, 用于调试观察本次发射从哪里开始
    float rush_target_angle; // 冲刺目标角度, 用于给位置环施加大误差换取起步力矩
    float lock_target_angle; // 锁止目标角度, 用于在掉速瞬间冻结当前位置防止多送
    float shot_start_time; // 本次单发事务开始时间戳 (ms), 用于限制整次有限重试的总时长
    float feed_start_time; // 送弹开始时间戳 (ms)
    float retry_start_time; // 补发等待起始时间戳 (ms), 用于按固定频率释放下一次补步
    float brake_start_time; // 锁止开始时间戳 (ms), 保留原字段名以减少调试结构变更范围
    float cooldown_start_time; // 冷却开始时间戳 (ms)
    uint8_t retry_count; // 已执行的补发次数, 用于实现有限重试而不是无限卷弹
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
