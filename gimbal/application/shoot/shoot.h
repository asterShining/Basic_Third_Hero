#ifndef SHOOT_H
#define SHOOT_H

#include "robot_def.h"
#include <stdint.h>

typedef struct
{
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

// 全局摩擦轮调试变量 (可在调试器中观测和修改)
extern FrictionWheelDebug_s friction_debug;
// 摩擦轮半径 (单位: 米), 例如 30mm = 0.03m
#define FRICTION_WHEEL_RADIUS 0.03f
// 打滑补偿系数 (需要实测微调, 通常在 1.0 - 1.2 之间)
#define SLIP_COMPENSATION 1.03f

// [新增] 摩擦轮软启动步长 (deg/loop)
// 假设 200Hz 控制频率，15m/s (约28000dps)
// 设为 150.0f 表示约 1秒 达到满速
#define FRICTION_RAMP_STEP 150.0f

// [新增] 摩擦轮前馈控制参数
#define FRICTION_FEEDFORWARD_CURRENT 500 // 前馈电流值
#define FRICTION_FEEDFORWARD_TIME 50 // 前馈持续时间 (ms)

// ==================== 堵转检测参数 ====================
// 堵转检测电流阈值 (raw值, M3508满量程16384, 设为 ~80% 高阈值使堵转处理更激烈)
#define STALL_CURRENT_THRESHOLD 14500
// 堵转检测速度阈值 (deg/s), 低于此值且电流高则判定为堵转
#define STALL_SPEED_THRESHOLD 500.0f
// 堵转检测消抖时间 (ms), 持续满足条件才确认堵转
#define STALL_DETECT_TIME 100
// 反转角度 (deg), 约为1/2颗弹丸角度,足够解卡但用户无感
#define REVERSE_ANGLE ONE_BULLET_DELTA_ANGLE
// 反转持续时间 (ms)
#define REVERSE_TIME 200
// 恢复等待时间 (ms), 反转后等待稳定再继续供弹
#define RECOVERY_TIME 80
// 连续反转次数上限, 超过则认为卡死,停止尝试
#define MAX_REVERSE_COUNT 5

// ==================== 预紧力矩参数 ====================
// // 预紧目标弹丸数 (设置拨盘目标为2发距离, 让电流提前建立)
// #define PRETENSION_BULLET_COUNT 2
// // 检测到发射后重置偏移量 (回退到1发位置, 防止连发)
// #define PRETENSION_RESET_OFFSET 1

// ==================== 单发控制参数 ====================
// 送弹速度 (deg/s), 中速稳定推弹, 给检测留足时间
#define SF_FEED_SPEED 5000.0f
// 制动速度 (deg/s), 负值反向制动, 抵消惯性防止第二颗进入
#define SF_BRAKE_SPEED -1000.0f
// 制动持续时间 (ms), 反向制动的持续时长
#define SF_BRAKE_TIME 1000
// 送弹超时时间 (ms), 超时未检测到掉速则认为缺弹或卡弹
#define SF_FEED_TIMEOUT 9500

// ==================== 发射确认检测参数 ====================
// 掉速检测阈值 (deg/s), 内圈摩擦轮速度下降超过此值认为有弹丸通过
// 使用内圈检测是因为弹丸先接触内圈, 信号更早, 能更有效防止多发
#define FRICTION_SPEED_DIP_THRESHOLD 800.0f
// 回升检测阈值 (deg/s), 与目标速度差小于此值认为回升完成
#define FRICTION_SPEED_RECOVER_THRESHOLD 100.0f

// ==================== 有效掉速识别参数 ====================
// 正常电机速度波动范围 (deg/s), 低于此值视为噪声而非弹丸掉速
#define DIP_NOISE_THRESHOLD 500.0f
// 有效掉速的最小电机数量, 至少N个电机同时掉速才认为是真实弹丸
#define DIP_MIN_MOTOR_COUNT 2
// 内外圈掉速时间差上限 (ms), 超过此值认为是异常波动而非弹丸通过
#define DIP_TIME_COHERENCE_MS 50.0f
// 有效掉速持续时间窗口 (ms), 在此时间内持续低于基准才确认
#define DIP_SUSTAIN_TIME_MS 10.0f

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

// 全局掉速抓拍变量声明 (最近一次发射的快照)
extern BulletDipSnapshot_s dip_snapshot;

// 历史记录缓冲区大小 (保存最近N发的数据用于分析)
#define DIP_HISTORY_SIZE 10
// 全局历史记录数组声明
extern BulletDipSnapshot_s dip_history[DIP_HISTORY_SIZE];
extern uint8_t dip_history_index;

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

// 全局堵转调试变量声明 (可在调试器中观测)
extern StallDebug_s stall_debug;

// 堵转检测状态结构体
static struct {
    LoaderStallState_e state; // 当前状态
    float detect_start_time; // 开始检测堵转的时间戳 (ms)
    float reverse_start_time; // 开始反转的时间戳 (ms)
    float recovery_start_time; // 开始恢复的时间戳 (ms)
    float reverse_target_angle; // 反转目标角度
    uint8_t reverse_count; // 连续反转次数
    loader_mode_e saved_mode; // 保存的原发射模式
} stall_handler = { 0 };

// ==================== 单发状态机 ====================
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
    float current_outer_speed; // [新增] 当前外圈摩擦轮平均速度 (deg/s)
    float speed_diff; // 速度差 (baseline - current), 正值表示掉速
    float loader_speed; // 拨盘当前速度 (deg/s)
    float feed_start_time; // 送弹开始时间 (ms)
    float brake_start_time; // 制动开始时间 (ms)
    uint8_t is_dipping; // 是否检测到掉速 (1=掉速中)
    uint8_t trigger_edge; // 是否检测到触发边沿 (1=边沿触发)
    uint16_t fire_count; // 累计发射弹丸计数
    uint16_t feed_timeout_count; // 送弹超时计数 (可能缺弹)
} SingleFireDebug_s;

// 全局单发调试变量声明 (可在调试器中观测)
extern SingleFireDebug_s sf_debug;

// 单发控制状态结构体 (运行时数据)
static struct {
    SingleFireState_e state; // 当前状态
    float baseline_speed; // 触发时的基准摩擦轮速度 (内圈)
    float outer_baseline_speed; // [新增] 触发时的基准摩擦轮速度 (外圈)
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