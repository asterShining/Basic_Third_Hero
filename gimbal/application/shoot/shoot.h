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
// 堵转检测电流阈值 (raw值, M3508满量程16384, 设为 ~60% 为稳妥值)
#define STALL_CURRENT_THRESHOLD 10000
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

// ==================== 发射确认检测参数 ====================
// 掉速检测阈值 (deg/s), 内圈摩擦轮速度下降超过此值认为有弹丸通过
// 使用内圈检测是因为弹丸先接触内圈, 信号更早, 能更有效防止多发
#define FRICTION_SPEED_DIP_THRESHOLD 400.0f
// 回升检测阈值 (deg/s), 与目标速度差小于此值认为回升完成
#define FRICTION_SPEED_RECOVER_THRESHOLD 200.0f
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

/**
 * @brief 设置摩擦轮速度
 *
 * @param speed_mps 速度 m/s
 */
void ShootSetSpeed(float speed_mps);

/**
 * @brief 获取发射确认标志
 * @return 1 表示最近一次发射已确认, 0 表示未确认
 */
uint8_t ShootGetFiredFlag(void);

/**
 * @brief 获取缺弹标志
 * @return 1 表示检测到缺弹, 0 表示正常
 */
uint8_t ShootGetEmptyFlag(void);

/**
 * @brief 清除缺弹标志 (需要手动调用复位)
 */
void ShootClearEmptyFlag(void);

/**
 * @brief 获取已确认发射计数
 * @return 累计确认发射的弹丸数量
 */
uint16_t ShootGetFireCount(void);

/**
 * @brief 检查拨盘是否被锁定 (防多发机制)
 * @return 1 表示锁定中, 应拒绝新的发射指令
 */
uint8_t ShootIsLoaderLocked(void);

#endif // SHOOT_H