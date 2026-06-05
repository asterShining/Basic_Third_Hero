#ifndef SHOOT_H
#define SHOOT_H

#include "robot_def.h"
#include "shoot_debug.h"
#include <stdint.h>

// 摩擦轮半径 (单位: 米), 例如 30mm = 0.03m
#define FRICTION_WHEEL_RADIUS 0.03f
// 打滑补偿系数 (需要实测微调, 通常在 1.0 - 1.2 之间)
#define SLIP_COMPENSATION 1.01f

// 定义摩擦轮升速斜率 (deg/s^2)，目的是用户要求把预热再放慢一点，这里下调升速斜率以拉长到稳态速度的时间，同时减轻上电瞬间电流冲击。
#define FRICTION_RAMP_UP_RATE_DPS_PER_S 60000.0f
// 定义摩擦轮停转斜率 (deg/s^2)，目的是关摩擦轮要比升速更快回零，避免停火后长时间拖转。
#define FRICTION_RAMP_DOWN_RATE_DPS_PER_S 160000.0f
// 定义摩擦轮控制斜坡允许使用的最大 dt (s)，目的是任务抖动或断点恢复时必须限幅，避免一次循环跳过整段 ramp。
#define FRICTION_RAMP_MAX_DT_S 0.02f

// ==================== 摩擦轮独立速度微调 (Trim) ====================
// 每个摩擦轮相对命令速度的偏置量 (单位: m/s)
// 正值 = 在该轮目标基础上加速; 负值 = 减速
// 用于补偿装配偏差或机械打滑差异，使各轮弹丸出口线速度一致
// 内圈: 负责主要加速 (第一级)
#define FRICTION_TRIM_INNER_LEFT 0.0f // 内圈左轮速度偏置 (m/s)
#define FRICTION_TRIM_INNER_RIGHT 0.0f // 内圈右轮速度偏置 (m/s)
#define FRICTION_TRIM_INNER_DOWN 0.0f // 内圈下轮速度偏置 (m/s)
// 外圈: 负责稳速/微加速 (第二级)
#define FRICTION_TRIM_OUTER_LEFT 0.0f // 外圈左轮速度偏置 (m/s)
#define FRICTION_TRIM_OUTER_RIGHT 0.0f // 外圈右轮速度偏S置 (m/s)
#define FRICTION_TRIM_OUTER_DOWN 0.0f // 外圈下轮速度偏置 (m/s)

// [新增] 摩擦轮前馈控制参数
#define FRICTION_FEEDFORWARD_CURRENT 500 // 前馈电流值
#define FRICTION_FEEDFORWARD_TIME 100 // 前馈持续时间 (ms)

// ==================== 拨弹盘线性速度前馈参数 ====================
// 前馈增益 (单位: 1/s)，将位置误差 (deg) 线性映射为速度补偿 (deg/s)，
// 注入速度环参考值入口，让速度环感知前馈并配合加速，避免与 PID 对抗；
// 当前单发已经改成完整一发行程强推，前馈增益需要比早期保守估算更大，目的是在限制位变紧或弹丸阻力偏大时让速度环更早拿到足够推进参考。
#define LOADER_FF_GAIN 8.0f
// 上限同步抬高但仍低于直接打满速度环的激进值，目的是增强起步推力的同时保留速度环 PID 和电流限幅的调节余量，避免单发末端过冲过大。
#define LOADER_FF_MAX_SPEED 14000.0f

// ==================== 堵转检测参数 ====================
// 堵转检测电流阈值 (raw值, M3508满量程16384, 设为 ~80% 高阈值使堵转处理更激烈)
#define STALL_CURRENT_THRESHOLD 15000
// 堵转检测速度阈值 (deg/s), 低于此值且电流高则判定为堵转
#define STALL_SPEED_THRESHOLD 400.0f
// 堵转检测消抖时间 (ms), 持续满足条件才确认堵转
#define STALL_DETECT_TIME 1500
// 自动解卡兜底反转使用完整一发弹位，目的是没有单发起点缓存的模式也按当前机械弹位退回一个明确位置，而不是只短退导致解卡不足。
#define REVERSE_ANGLE ONE_BULLET_DELTA_ANGLE
// 反转持续时间 (ms)
#define REVERSE_TIME 500
// 恢复等待时间 (ms), 反转后等待稳定再继续供弹
#define RECOVERY_TIME 80
// 连续反转次数上限, 超过则认为卡死,停止尝试
#define MAX_REVERSE_COUNT 5

// ==================== 单发控制参数 ====================
// 定义单发固定送弹步距 (单位: 发)，目的是每次触发只让拨弹盘输出端走完 ONE_BULLET_DELTA_ANGLE 对应的一发机械行程，掉速只参与出弹计数，不再提前截断拨盘目标。
#define SF_RUSH_BULLET_COUNT 1.0f
// 拨盘电机总角度对应的一发角度 (deg), 需要乘减速比, 因为 total_angle 是电机转子多圈角度
#define LOADER_MOTOR_ANGLE_PER_BULLET (ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER)
// 单发目标总角度 (deg)，用于位置环按固定一发机械行程推弹，实际输出端角度由 ONE_BULLET_DELTA_ANGLE 决定。
#define SF_RUSH_ANGLE (SF_RUSH_BULLET_COUNT * LOADER_MOTOR_ANGLE_PER_BULLET)
// 单发到位容差 (deg)，用于判断固定一发机械行程已经基本完成；这里收紧到 0.03 发是为了避免过早收口导致拨盘输出端明显少走角度。
#define SF_RUSH_REACHED_TOLERANCE (0.03f * LOADER_MOTOR_ANGLE_PER_BULLET)
// 定义整次单发事务的总超时 (ms)，目的是单发限制位变紧后仍快速收口，即使强推不到位也不能让拨盘长时间顶住机构。
#define SF_TRANSACTION_TIMEOUT 300.0f
// 定义最小有效掉速行程 (单位: 发)，目的是忽略刚起步阶段的摩擦轮扰动，只有拨盘确实推进到可能咬弹的位置后才允许把掉速记为一发。
#define SF_MIN_VALID_DIP_PROGRESS_BULLET 0.30f
// 定义平均掉速连续稳定拍数，目的是单电机噪声和瞬时扰动较多，平均掉速需要跨两个控制周期确认后再累计发射计数。
#define SF_DIP_AVG_STABLE_CYCLES 2u
// 定义发射成功后的回速稳定拍数，目的是略微收紧回速判定，让上一发完全恢复后才允许下一次单发进入，进一步压住连续点射时的多发风险。
#define FRICTION_RECOVER_STABLE_CYCLES 4u

// ==================== 发射确认检测参数 ====================
// 掉速检测阈值 (deg/s), 内圈摩擦轮速度下降超过此值认为有弹丸通过
// 掉速门槛只服务发射计数，目的是保留弹丸数统计能力，同时避免阈值变化再次影响拨盘固定一发送弹行程。
#define FRICTION_SPEED_DIP_THRESHOLD 900.0f
// 定义外圈辅助确认阈值 (deg/s)，目的是当内圈只出现“较弱但连续”的平均掉速时，必须让外圈也给出一定幅度的掉速佐证，减少把半咬弹误记为真实出弹。
#define OUTER_DIP_CONFIRM_THRESHOLD 650.0f
// 回升检测阈值 (deg/s), 与目标速度差小于此值认为回升完成
#define FRICTION_SPEED_RECOVER_THRESHOLD 400.0f
// [新增] 射速就绪阈值 (deg/s), 实际速度与目标速度误差小于此值才允许发射
#define SHOOT_SPEED_READY_THRESHOLD 500.0f
// 定义单发重武装保护时间 (ms)，目的是VT03 扳机抖动、鼠标回弹和链路伪边沿都可能把一次点击拆成两次触发，需要一小段保护窗口挡住伪双击。
#define SF_TRIGGER_REARM_GUARD_MS 100.0f

// 堵转、单发状态机和触发边沿锁存都属于 shoot 模块内部运行时状态，目的是拆分成多个 `.c` 后这些状态必须集中放进私有头，公共头只保留真正对外可见的接口与参数。

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
