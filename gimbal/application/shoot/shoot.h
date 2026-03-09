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
// 堵转检测电流阈值 (raw值, M3508满量程16384, 设为 ~80% 高阈值使堵转处理更激烈)
#define STALL_CURRENT_THRESHOLD 15000
// 堵转检测速度阈值 (deg/s), 低于此值且电流高则判定为堵转
#define STALL_SPEED_THRESHOLD 400.0f
// 堵转检测消抖时间 (ms), 持续满足条件才确认堵转
#define STALL_DETECT_TIME 1500
// 反转角度 (deg), 约为 1/2 颗弹丸角度, 足够解卡但尽量不把正常节拍打乱
#define REVERSE_ANGLE (0.5f * ONE_BULLET_DELTA_ANGLE)
// 反转持续时间 (ms)
#define REVERSE_TIME 500
// 恢复等待时间 (ms), 反转后等待稳定再继续供弹
#define RECOVERY_TIME 80
// 连续反转次数上限, 超过则认为卡死,停止尝试
#define MAX_REVERSE_COUNT 5

// ==================== 单发控制参数 ====================
// 单发冲刺行程 (单位: 发), 直接给出大位置误差, 让位置环一开始就把速度环顶到高输出
#define SF_RUSH_BULLET_COUNT 1.5f
// 拨盘电机总角度对应的一发角度 (deg), 需要乘减速比, 因为 total_angle 是电机转子多圈角度
#define LOADER_MOTOR_ANGLE_PER_BULLET (ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER)
// 单发冲刺总角度 (deg), 用于位置环大步进推弹
#define SF_RUSH_ANGLE (SF_RUSH_BULLET_COUNT * LOADER_MOTOR_ANGLE_PER_BULLET)
// 单发冲刺到位容差 (deg), 用于在掉速丢失时尽快收口, 避免持续追一个过远目标
#define SF_RUSH_REACHED_TOLERANCE (0.10f * LOADER_MOTOR_ANGLE_PER_BULLET)
// 单发固定终点步距 (单位: 发), 作用是让单发最终停在固定机械节距附近
// 原因是用户希望弱化摩擦轮掉速对停盘点的影响, 改为主要由机械步距决定停盘位置
#define SF_SINGLE_TARGET_BULLET_COUNT 1.4f
// 切入最终收口的提前量 (单位: 发), 作用是在接近固定终点时尽早切回终点目标
// 原因是拨盘位置环 Kp 较高, 若一直追冲刺远目标容易把惯性能量带过头
#define SF_FINAL_APPROACH_BULLET_COUNT 0.25f
// 固定终点到位容差 (deg), 作用是统一判定“本轮固定步距已完成”
// 原因是单发现在要先追远目标再收口, 需要独立于冲刺容差的终点完成条件
#define SF_FINAL_TARGET_TOLERANCE (0.10f * LOADER_MOTOR_ANGLE_PER_BULLET)
// 补发频率 (Hz), 初次冲刺未发现掉速时按该节拍继续位置环补步, 便于逐颗寻找弹丸
#define SF_RETRY_RATE_HZ 8.0f
// 兼容保留的补发步距宏, 作用是让历史代码若继续引用时仍与新的固定终点步距一致
// 原因是本次改造要求补发也按同一固定机械步距推进, 不能再回到旧的 1.5 发语义
#define SF_RETRY_STEP_BULLET_COUNT SF_SINGLE_TARGET_BULLET_COUNT
// 有限重试上限, 超过后判定本次未成功出弹并锁止, 防止空仓时无休止卷弹
#define SF_RETRY_MAX_COUNT 5u
// 补发间隔 (ms), 由补发频率直接换算, 便于状态机按绝对时间节拍触发下一步
#define SF_RETRY_INTERVAL_MS (1000.0f / SF_RETRY_RATE_HZ)
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
    float rush_start_angle; // 冲刺起始角度, 用于调试观察本次发射从哪里开始
    float rush_target_angle; // 冲刺目标角度, 用于给位置环施加大误差换取起步力矩
    float final_target_angle; // 本轮固定终点角度, 用于把单发停盘点约束在固定机械步距附近
    float lock_target_angle; // 锁止目标角度, 用于在掉速瞬间冻结当前位置防止多送
    float shot_start_time; // 本次单发事务开始时间戳 (ms), 用于限制整次有限重试的总时长
    float feed_start_time; // 送弹开始时间戳 (ms)
    float retry_start_time; // 补发等待起始时间戳 (ms), 用于按固定频率释放下一次补步
    float brake_start_time; // 锁止开始时间戳 (ms), 保留原字段名以减少调试结构变更范围
    float cooldown_start_time; // 冷却开始时间戳 (ms)
    uint8_t dip_confirmed; // 本轮是否已通过掉速校验确认出弹, 用于把掉速降级为确认信号而非停盘信号
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
