#ifndef GIMBAL_PRIVATE_H
#define GIMBAL_PRIVATE_H

#include "gimbal.h"
#include "dmmotor.h"
#include "motor_def.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "gimbal_pitch_cali.h"
#include "video_link_motor.h"
#include "bsp_dwt.h"
#include "arm_math.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// 这里直接填入 2026-04-18 这轮“新一批” pitch 标定数据经 /home/aster/robo-misc/python_pitch.py 同款最小二乘重新计算后得到的一阶余弦系数，目的是固件里的重力前馈公式要与这次最新离线拟合模型严格同符号、同尺度，不能继续沿用上一轮旧数据的结果。
// 这次拟合前仍按同一套规则先剔除了明显坏点：目标力矩格式错位的 x.99 行，以及 |Target_Torque-Real_Torque| > 0.25Nm 的异常行，目的是把 RTT 导出里的错位样本和明显失真样本挡在拟合外面，避免重力前馈被脏数据拉偏。
#define PITCH_GRAVITY_COEFFICIENT_K1 5.8370f
// 这里同步填入同一批新数据清洗后有效样本解出的正弦修正项，目的是枪管重心不完全落在纯余弦项上，若这项仍保留上一轮旧值，高低俯仰两侧的补偿误差会重新变得不对称。
#define PITCH_GRAVITY_COEFFICIENT_K2 8.9759f
// 这里保留同一轮新数据拟合出的常值偏置，目的是当前机构静态平衡点明显不在零力附近，若忽略这项，整段姿态都会残留同方向的恒定欠补或过补。
#define PITCH_GRAVITY_OFFSET 5.8526f
// 这里给 Pitch 目标角加速度前馈一个保守初值，目的是快速抬头/压头起停时先帮速度环分担一点惯量负担，但在未上车细调前不能给得太猛。
#define PITCH_INERTIA_FEEDFORWARD_K 0.015f
// 这里给 Pitch 目标角速度相关阻力补偿一个低幅值初值，目的是连续运动时先减掉一部分黏性负载，又不至于一上来就把匀速段顶得发硬。
#define PITCH_VISCOUS_FEEDFORWARD_K 0.080f
// 这里给 Pitch 低速起转补一小段平滑静摩擦前馈，目的是细微抬头和压头时减少“推不动”的涩感，但仍把幅值压在较保守范围。
#define PITCH_STATIC_FEEDFORWARD_K 0.120f
// 这里用平滑带宽控制 Pitch 静摩擦项的过零柔和度，目的是慢速过零时不要像硬 sign 一样突然翻转，避免目标点附近细碎抖动。
#define PITCH_STATIC_SMOOTH_BAND_RAD_S 0.180f
// 这里沿用仓库里历史上用过的小陀螺 Pitch 抗扰量级作为保守起点，目的是先给高速 Yaw 旋转工况一层离心补偿，同时尽量不偏离你之前验证过的结构。
#define PITCH_CENTRIFUGAL_FEEDFORWARD_K 0.090f

// 定义 Yaw 轴科氏项前馈初始系数，目的是复合甩头时先补一层轻量耦合，减少仅靠误差环追赶带来的卡顿。
#define YAW_CORIOLIS_FEEDFORWARD_K 0.10f
// 定义 Yaw 轴惯量前馈基础系数，目的是目标 Yaw 加速度变化时，先补一层基础转动惯量，减轻位置环和速度环的追赶负担。
#define YAW_INERTIA_BASE 0.010f
// 定义 Yaw 轴随 Pitch 姿态变化的附加载荷惯量系数，目的是枪管和载荷抬起后，Yaw 有效惯量会变化，需要随 Pitch 姿态做一阶补偿。
#define YAW_INERTIA_PITCH_COS2_GAIN 0.006f
// 定义 Yaw 轴黏性摩擦前馈系数，目的是云台持续匀速甩头时，先补掉一部分速度相关阻力，让跟手性更稳定。
#define YAW_VISCOUS_FEEDFORWARD_K 0.025f
// 定义 Yaw 轴静摩擦前馈幅值，目的是起转和低速换向最容易被静摩擦拖住，补一小段定值可以减轻发涩感。
#define YAW_STATIC_FEEDFORWARD_K 0.18f
// 定义 Yaw 静摩擦前馈死区，目的是参考速度太小时不应持续注入静摩擦补偿，否则停稳附近更容易自己来回拧。
#define YAW_STATIC_FEEDFORWARD_DEADBAND_RAD_S 0.10f

// 定义前馈角速度低通时间常数，目的是关节速率直接进入动力学耦合项对噪声很敏感，必须先做轻度滤波抑制抖动。
#define GIMBAL_FEEDFORWARD_RATE_LPF_RC 0.020f
// 定义 Pitch 目标速度估计低通时间常数，目的是 Pitch 参考角同样来自离散输入，直接求导会放大量化噪声，必须先平滑再参与动力学前馈。
#define PITCH_REF_RATE_LPF_RC 0.020f
// 定义 Pitch 目标加速度估计低通时间常数，目的是二次求导比速度更容易尖峰，先做保守滤波才能把惯量项控制在可用范围。
#define PITCH_REF_ACC_LPF_RC 0.030f
// 定义 Yaw 目标速度估计低通时间常数，目的是参考角度离散求导会放大量化噪声，先滤波后再生成惯量和摩擦前馈更稳。
#define YAW_REF_RATE_LPF_RC 0.015f
// 定义 Yaw 目标加速度估计低通时间常数，目的是二次求导最容易尖峰，必须再滤一次才能给惯量前馈使用。
#define YAW_REF_ACC_LPF_RC 0.025f
// 定义云台前馈周期后备值，目的是DWT 首拍或异常值不能直接拿去更新滤波器，否则会把耦合项瞬间放大。
#define GIMBAL_FEEDFORWARD_DT_FALLBACK 0.005f
// 定义 Pitch 参考导数状态的跳变复位阈值，目的是切模式、贴当前姿态或外部目标大步跳变时，不允许沿用旧导数去制造假惯量尖峰。
#define PITCH_REF_DERIV_RESET_THRESHOLD_DEG 6.0f
// 定义 Yaw 参考导数状态的跳变复位阈值，目的是切源、贴齐当前姿态或外部大步跳目标时，直接求导会产生假加速度尖峰。
#define YAW_REF_DERIV_RESET_THRESHOLD_DEG 10.0f
// 这里把 Pitch 前馈总限幅同步抬到略高于本轮静态标定最大实测力矩的位置，目的是新拟合的重力项在大仰角已经接近 12Nm，若仍卡在旧的 7.5Nm，会在高角度长期被截断，导致“参数换了但实车托不住”的假象。
#define PITCH_FEEDFORWARD_LIMIT 12.0f
// 定义 Yaw 前馈输出限幅，目的是当前 Yaw 只加动态耦合项，先用更保守的上限保证复合运动不过激。
#define YAW_FEEDFORWARD_LIMIT 3.0f

// 定义 Pitch 机械上限，目的是云台初始化后要立即给 DM 电机同一份机构保护范围，避免文件拆分后限位来源分叉。
#define PITCH_MECH_LIMIT_MAX 0.08f
// 定义 Pitch 机械下限，目的是冲坡或姿态突变时仍需由底层限位保护枪管不撞下极限。
#define PITCH_MECH_LIMIT_MIN -0.967f
// 定义 yaw 速度环积分系数，目的是小陀螺属于持续扰动场景，只靠 P 项容易留下稳态偏差，因此补一小段 Ki 来慢慢顶住漂移。
#define YAW_SPEED_PID_KI 0.12f
// 定义 yaw 速度环静止死区(rad/s)，目的是陀螺仪静止时也会有零偏和噪声，必须把极小误差吞掉，防止积分攒久后突然抽动。
#define YAW_SPEED_PID_DEADBAND_RAD 0.015f
// 定义 yaw 速度环积分限幅，目的是即使进入积分，也只允许积累少量修正，避免静摩擦被一次性打穿导致云台突跳。
#define YAW_SPEED_PID_INTEGRAL_LIMIT 0.6f
// 定义 yaw 速度环变速积分主区间(rad/s)，目的是误差较大时由 P 项主导，误差较小时才逐步放开积分，减少小陀螺切换和停车瞬间的堆积。
#define YAW_SPEED_PID_COEF_A_RAD 0.18f
// 定义 yaw 速度环全积分阈值(rad/s)，目的是只有误差足够小的时候才允许满积分，用来补静态偏差而不是放大动态扰动。
#define YAW_SPEED_PID_COEF_B_RAD 0.03f

// 云台 IMU、电机与命令反馈缓存会在初始化、主任务和拆分 helper 之间共同读写，目的是拆成多个编译单元后必须继续共用同一份运行时状态。
extern attitude_t *gimba_IMU_data;
extern DMMotorInstance *yaw_motor;
extern DJIMotorInstance *pitch_motor;
extern Gimbal_Upload_Data_s gimbal_feedback_data;
extern Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;

// 下面这些内部 helper 只服务 gimbal 模块内部拆分，目的是保持私有头统一声明，既能跨文件复用，又不污染公共头接口。
float NormalizeAngleTo360(float angle_deg);
void UpdateGimbalCurrentFeedforward(uint8_t gimbal_mode_changed,
                                    uint8_t yaw_motor_online,
                                    uint8_t yaw_motor_online_changed,
                                    uint8_t pitch_motor_online,
                                    uint8_t pitch_motor_online_changed);

#endif // GIMBAL_PRIVATE_H
