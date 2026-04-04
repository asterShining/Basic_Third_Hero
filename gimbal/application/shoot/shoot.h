#ifndef SHOOT_H
#define SHOOT_H

#include "robot_def.h"
#include "shoot_debug.h"
#include <stdint.h>

// 摩擦轮半径 (单位: 米), 例如 30mm = 0.03m
#define FRICTION_WHEEL_RADIUS 0.03f
// 打滑补偿系数 (需要实测微调, 通常在 1.0 - 1.2 之间)
#define SLIP_COMPENSATION 1.00f

// What: 定义摩擦轮升速斜率 (deg/s^2)；Why: 用户要求把预热再放慢一点，这里下调升速斜率以拉长到稳态速度的时间，同时减轻上电瞬间电流冲击。
#define FRICTION_RAMP_UP_RATE_DPS_PER_S 60000.0f
// What: 定义摩擦轮停转斜率 (deg/s^2)；Why: 关摩擦轮要比升速更快回零，避免停火后长时间拖转。
#define FRICTION_RAMP_DOWN_RATE_DPS_PER_S 160000.0f
// What: 定义摩擦轮控制斜坡允许使用的最大 dt (s)；Why: 任务抖动或断点恢复时必须限幅，避免一次循环跳过整段 ramp。
#define FRICTION_RAMP_MAX_DT_S 0.02f

// ==================== 摩擦轮独立速度微调 (Trim) ====================
// 每个摩擦轮相对命令速度的偏置量 (单位: m/s)
// 正值 = 在该轮目标基础上加速; 负值 = 减速
// 用于补偿装配偏差或机械打滑差异，使各轮弹丸出口线速度一致
// 内圈: 负责主要加速 (第一级)
#define FRICTION_TRIM_INNER_LEFT 0.5f // 内圈左轮速度偏置 (m/s)
#define FRICTION_TRIM_INNER_RIGHT 0.0f // 内圈右轮速度偏置 (m/s)
#define FRICTION_TRIM_INNER_DOWN 0.2f // 内圈下轮速度偏置 (m/s)
// 外圈: 负责稳速/微加速 (第二级)
#define FRICTION_TRIM_OUTER_LEFT 0.5f // 外圈左轮速度偏置 (m/s)
#define FRICTION_TRIM_OUTER_RIGHT 0.0f // 外圈右轮速度偏置 (m/s)
#define FRICTION_TRIM_OUTER_DOWN 0.2f // 外圈下轮速度偏置 (m/s)

// [新增] 摩擦轮前馈控制参数
#define FRICTION_FEEDFORWARD_CURRENT 500 // 前馈电流值
#define FRICTION_FEEDFORWARD_TIME 100 // 前馈持续时间 (ms)

// ==================== 堵转检测参数 ====================
// 堵转检测电流阈值 (raw值, M3508满量程16384, 设为 ~80% 高阈值使堵转处理更激烈)
#define STALL_CURRENT_THRESHOLD 15000
// 堵转检测速度阈值 (deg/s), 低于此值且电流高则判定为堵转
#define STALL_SPEED_THRESHOLD 400.0f
// 堵转检测消抖时间 (ms), 持续满足条件才确认堵转
#define STALL_DETECT_TIME 1500
#define REVERSE_ANGLE (0.1f * ONE_BULLET_DELTA_ANGLE)
// 反转持续时间 (ms)
#define REVERSE_TIME 500
// 恢复等待时间 (ms), 反转后等待稳定再继续供弹
#define RECOVERY_TIME 80
// 连续反转次数上限, 超过则认为卡死,停止尝试
#define MAX_REVERSE_COUNT 5

// ==================== 单发控制参数 ====================
// What: 定义单发首发冲刺步距 (单位: 发)；Why: 上一版 1.2 发在弱供弹工况下空发偏多，因此小幅回补到 1.35 发，让首发更容易把弹丸稳定送入摩擦轮而又不直接回到双发风险很高的旧值。
#define SF_RUSH_BULLET_COUNT 1.65f
// 拨盘电机总角度对应的一发角度 (deg), 需要乘减速比, 因为 total_angle 是电机转子多圈角度
#define LOADER_MOTOR_ANGLE_PER_BULLET (ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER)
// 单发冲刺总角度 (deg), 用于位置环大步进推弹
#define SF_RUSH_ANGLE (SF_RUSH_BULLET_COUNT * LOADER_MOTOR_ANGLE_PER_BULLET)
// 单发冲刺到位容差 (deg), 用于在掉速丢失时尽快收口, 避免持续追一个过远目标
#define SF_RUSH_REACHED_TOLERANCE (0.20f * LOADER_MOTOR_ANGLE_PER_BULLET)
// 补发频率 (Hz), 初次冲刺未发现掉速时按该节拍继续位置环补步, 便于逐颗寻找弹丸
#define SF_RETRY_RATE_HZ 9.0f
// What: 定义单发补发步距 (单位: 发)；Why: 空发现象明显时一次 0.25 发补步过小，因此上调到 0.35 发，在不放开多次补发的前提下提高补发有效性。
#define SF_RETRY_STEP_BULLET_COUNT 0.35f
// What: 定义单发补发上限；Why: 当前目标是防双发优先，因此只保留 1 次小补发，不再允许多次累加推进量。
#define SF_RETRY_MAX_COUNT 3u
// 补发间隔 (ms), 由补发频率直接换算, 便于状态机按绝对时间节拍触发下一步
#define SF_RETRY_INTERVAL_MS (1000.0f / SF_RETRY_RATE_HZ)
// What: 定义整次单发事务的总超时 (ms)；Why: 首发和最多一次小补发都必须在短时间内收口，不能让状态机长时间霸占拨盘控制权。
#define SF_TRANSACTION_TIMEOUT 400.0f
// What: 定义最小有效掉速行程 (单位: 发)；Why: 上一版 0.25 发允许过早把弱咬弹识别成成功，这会带来空发，因此回调到 0.30 发以减少过早锁角。
#define SF_MIN_VALID_DIP_PROGRESS_BULLET 0.30f
// What: 定义平均掉速连续稳定拍数；Why: 单电机噪声和瞬时扰动较多，平均掉速需要跨两个控制周期确认后再锁角更稳。
#define SF_DIP_AVG_STABLE_CYCLES 2u
// What: 定义发射成功后的回速稳定拍数；Why: 略微收紧回速判定，让上一发完全恢复后才允许下一次单发进入，进一步压住连续点射时的多发风险。
#define FRICTION_RECOVER_STABLE_CYCLES 4u

// ==================== 发射确认检测参数 ====================
// 掉速检测阈值 (deg/s), 内圈摩擦轮速度下降超过此值认为有弹丸通过
// What: 适度降低掉速门槛；Why: 当前更需要让首颗弹丸的真实掉速更早被识别到，从而尽快锁角，减少漏检后继续送弹。
#define FRICTION_SPEED_DIP_THRESHOLD 900.0f
// What: 定义外圈辅助确认阈值 (deg/s)；Why: 当内圈只出现“较弱但连续”的平均掉速时，必须让外圈也给出一定幅度的掉速佐证，才能减少把半咬弹误判成成功所导致的空发。
#define OUTER_DIP_CONFIRM_THRESHOLD 650.0f
// 回升检测阈值 (deg/s), 与目标速度差小于此值认为回升完成
#define FRICTION_SPEED_RECOVER_THRESHOLD 400.0f
// [新增] 射速就绪阈值 (deg/s), 实际速度与目标速度误差小于此值才允许发射
#define SHOOT_SPEED_READY_THRESHOLD 500.0f
// What: 定义单发重武装保护时间 (ms)；Why: VT03 扳机抖动、鼠标回弹和链路伪边沿都可能把一次点击拆成两次触发，需要一小段保护窗口挡住伪双击。
#define SF_TRIGGER_REARM_GUARD_MS 100.0f

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
    float lock_target_angle; // 锁止目标角度, 用于在掉速瞬间冻结当前位置防止多送
    float shot_start_time; // 本次单发事务开始时间戳 (ms), 用于限制整次有限重试的总时长
    float feed_start_time; // 送弹开始时间戳 (ms)
    float retry_start_time; // 补发等待起始时间戳 (ms), 用于按固定频率释放下一次补步
    float brake_start_time; // 锁止开始时间戳 (ms), 保留原字段名以减少调试结构变更范围
    float cooldown_start_time; // 冷却开始时间戳 (ms)
    uint8_t retry_count; // 已执行的补发次数, 用于实现有限重试而不是无限卷弹
    uint8_t inner_dip_stable_count; // What: 记录平均掉速连续稳定拍数；Why: 内圈平均掉速要跨多个控制周期确认，不能把单拍波动直接当成有效出弹。
    uint8_t recover_stable_count; // What: 记录摩擦轮回速稳定拍数；Why: 发射成功后必须连续多拍恢复到稳态，下一发才允许进入。
    uint16_t fire_count; // 累计发射计数
    uint16_t feed_timeout_count; // 送弹超时计数
} single_fire = { 0 };

// ==================== 单发触发边沿检测 ====================
// 用于确保每次触发只响应一次，防止持续按住导致多发
typedef struct {
    loader_mode_e last_mode; // 上一次的拨盘模式 (用于边沿检测)
    uint8_t trigger_consumed; // 触发是否已被消费 (1=已消费,等待复位)
    uint8_t pending_fire; // What: 是否缓存了待处理单发请求；Why: 单发请求至少要保留到状态机真正消费它的那一拍，不能在入口瞬时丢掉。
    float last_accept_time_ms; // What: 记录最近一次真正接受单发请求的时间；Why: 只有知道上次有效触发时刻，才能对抖动边沿施加最小间隔保护。
} FireTrigger_s;

static FireTrigger_s fire_trigger = { .last_mode = LOAD_STOP, .trigger_consumed = 0, .pending_fire = 0, .last_accept_time_ms = 0.0f };

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
