#ifndef SHOOT_H
#define SHOOT_H

#include "robot_def.h"
#include <stdint.h>

typedef struct
{
    float inner_left; // 内圈左 (m/s)
    float inner_right; // 内圈右 (m/s)
    float inner_down; // 内圈下 (m/s)
    float outer_left; // 外圈左 (m/s)
    float outer_right; // 外圈右 (m/s)
    float outer_down; // 外圈下 (m/s)
} ShootDebugSpeed_s;
// 摩擦轮半径 (单位: 米), 例如 30mm = 0.03m
#define FRICTION_WHEEL_RADIUS 0.03f
// 打滑补偿系数 (需要实测微调, 通常在 1.0 - 1.2 之间)
#define SLIP_COMPENSATION 1.05f

// [新增] 摩擦轮软启动步长 (deg/loop)
// 假设 200Hz 控制频率，15m/s (约28000dps)
// 设为 150.0f 表示约 1秒 达到满速
#define FRICTION_RAMP_STEP 150.0f

// ==================== 堵转检测参数 ====================
// 堵转检测电流阈值 (raw值, M3508满量程16384, 设为 ~80% 高阈值使堵转处理更激烈)
#define STALL_CURRENT_THRESHOLD 13000
// 堵转检测速度阈值 (deg/s), 低于此值且电流高则判定为堵转
#define STALL_SPEED_THRESHOLD 100.0f
// 堵转检测消抖时间 (ms), 持续满足条件才确认堵转
#define STALL_DETECT_TIME 100
// 反转角度 (deg), 约为1/2颗弹丸角度,足够解卡但用户无感
#define REVERSE_ANGLE (ONE_BULLET_DELTA_ANGLE / 2.5f)
// 反转持续时间 (ms)
#define REVERSE_TIME 120
// 恢复等待时间 (ms), 反转后等待稳定再继续供弹
#define RECOVERY_TIME 80
// 连续反转次数上限, 超过则认为卡死,停止尝试
#define MAX_REVERSE_COUNT 3

// ==================== 预紧力矩参数 ====================
// // 预紧目标弹丸数 (设置拨盘目标为2发距离, 让电流提前建立)
// #define PRETENSION_BULLET_COUNT 2
// // 检测到发射后重置偏移量 (回退到1发位置, 防止连发)
// #define PRETENSION_RESET_OFFSET 1

// ==================== 发射确认检测参数 ====================
// 掉速检测阈值 (deg/s), 内圈摩擦轮速度下降超过此值认为有弹丸通过
// 使用内圈检测是因为弹丸先接触内圈, 信号更早, 能更有效防止多发
#define FRICTION_SPEED_DIP_THRESHOLD 300.0f
// 回升检测阈值 (deg/s), 与目标速度差小于此值认为回升完成
#define FRICTION_SPEED_RECOVER_THRESHOLD 100.0f
// 拨盘位置误差阈值 (deg), 小于此值认为拨盘到位
#define LOADER_POSITION_THRESHOLD 8.0f
// 发射确认超时时间 (ms), 等待摩擦轮掉速的最大时间
#define FIRE_DETECT_TIMEOUT 300
// 拨盘到位超时时间 (ms), 等待拨盘转到位的最大时间
#define LOADER_ARRIVE_TIMEOUT 200

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

// 发射确认状态枚举
// 用于追踪单发弹丸从拨盘推出到摩擦轮检测的完整流程
typedef enum {
    FIRE_IDLE = 0, // 空闲/等待发射指令
    FIRE_LOADING, // 拨盘正在旋转, 等待到位
    FIRE_WAIT_DIP, // 拨盘已到位, 等待摩擦轮掉速
    FIRE_WAIT_RECOVER, // 检测到掉速, 等待回升 (拨盘已锁定)
    FIRE_CONFIRMED, // 发射确认成功
    FIRE_EMPTY, // 缺弹 (拨盘到位但无掉速)
} FireDetectState_e;

// [新增] 发射检测调试信息结构体 (全局可观测)
// 用于在调试器中实时观测摩擦轮单发检测状态机的工作情况
typedef struct {
    FireDetectState_e state; // 当前发射检测状态机状态
    float baseline_speed; // 发射前基准摩擦轮速度 (deg/s)
    float current_speed; // 当前内圈摩擦轮平均速度 (deg/s)
    float speed_diff; // 速度差 (baseline - current), 正值表示掉速
    float loader_target_angle; // 拨盘目标角度 (deg)
    float loader_actual_angle; // 拨盘实际角度 (deg)
    float loader_error; // 拨盘位置误差 (deg)
    uint8_t is_dipping; // 是否检测到掉速 (1=掉速中, 0=正常)
    uint8_t is_recovered; // 速度是否回升 (1=已回升, 0=未回升)
    uint8_t loader_locked; // 拨盘锁定标志 (1=锁定, 0=解锁)
    uint8_t empty_flag; // 缺弹标志 (1=缺弹, 0=正常)
    uint8_t trigger_consumed; // 触发是否已消费 (1=已消费)
    uint16_t fire_count; // 已确认发射计数
} FireDebug_s;

// [新增] 全局发射检测调试变量声明 (可在调试器中观测)
extern FireDebug_s fire_debug;

// 发射确认状态结构体
// 用于管理发射检测状态机的所有运行时数据
static struct {
    FireDetectState_e state; // 当前状态
    float loader_target_angle; // 拨盘目标角度
    float baseline_speed; // 发射前的基准摩擦轮速度 (deg/s)
    float fire_start_time; // 开始发射的时间戳 (ms)
    float loader_arrive_time; // 拨盘到位的时间戳 (ms)
    uint8_t bullet_fired_flag; // 发射确认标志位 (1=已确认发射)
    uint8_t empty_flag; // 缺弹标志位 (1=检测到缺弹)
    uint8_t loader_locked; // 拨盘锁定标志 (防多发核心机制)
    uint16_t fire_count; // 已确认发射计数
} fire_detector = { 0 };

// ==================== 单发触发边沿检测 ====================
// 用于确保每次触发只响应一次，防止持续按住导致多发
typedef struct {
    loader_mode_e last_mode; // 上一次的拨盘模式 (用于边沿检测)
    uint8_t trigger_consumed; // 触发是否已被消费 (1=已消费,等待复位)
    uint8_t pending_fire; // 是否有待处理的发射请求 (1=有)
} FireTrigger_s;

static FireTrigger_s fire_trigger = { .last_mode = LOAD_STOP, .trigger_consumed = 0, .pending_fire = 0 };

// 空仓加速参数
#define EMPTY_SPEEDUP_RATIO 1.5f // 空仓状态下加速倍率
#define EMPTY_RETRY_ANGLE (ONE_BULLET_DELTA_ANGLE * 0.5f) // 空仓时继续推进的角度
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