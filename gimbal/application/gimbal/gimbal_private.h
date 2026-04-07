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

// 保留 Pitch 重力补偿拟合的一阶余弦系数，目的是这是当前实机已经验证可用的主补偿项，拆文件后仍必须共用同一组标定参数。
#define PITCH_GRAVITY_COEFFICIENT_K1 -1.406f
// 保留 Pitch 重力补偿拟合的一阶正弦系数，目的是当前枪管重心偏置已经体现在这一项中，拆分后不能让不同文件各自维护不同版本。
#define PITCH_GRAVITY_COEFFICIENT_K2 -0.6058f
// 保留 Pitch 重力补偿的常值偏置，目的是实机静态平衡点并不在零位，常值项必须和拟合系数一起使用。
#define PITCH_GRAVITY_OFFSET -5.0816f

// 定义 Pitch 轴离心项前馈初始系数，目的是先给小陀螺工况一个保守补偿起点，后续只需围绕这一处做上车标定。
#define PITCH_CENTRIFUGAL_FEEDFORWARD_K 0.09f
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
// 定义 Yaw 目标速度估计低通时间常数，目的是参考角度离散求导会放大量化噪声，先滤波后再生成惯量和摩擦前馈更稳。
#define YAW_REF_RATE_LPF_RC 0.015f
// 定义 Yaw 目标加速度估计低通时间常数，目的是二次求导最容易尖峰，必须再滤一次才能给惯量前馈使用。
#define YAW_REF_ACC_LPF_RC 0.025f
// 定义云台前馈周期后备值，目的是DWT 首拍或异常值不能直接拿去更新滤波器，否则会把耦合项瞬间放大。
#define GIMBAL_FEEDFORWARD_DT_FALLBACK 0.005f
// 定义 Yaw 参考导数状态的跳变复位阈值，目的是切源、贴齐当前姿态或外部大步跳目标时，直接求导会产生假加速度尖峰。
#define YAW_REF_DERIV_RESET_THRESHOLD_DEG 10.0f
// 定义 Pitch 前馈总输出限幅，目的是重力项之外新增耦合项后必须保留硬保护，防止未标定参数直接把扭矩顶满。
#define PITCH_FEEDFORWARD_LIMIT 7.5f
// 定义 Yaw 前馈输出限幅，目的是当前 Yaw 只加动态耦合项，先用更保守的上限保证复合运动不过激。
#define YAW_FEEDFORWARD_LIMIT 3.0f

// 定义 Pitch 机械上限，目的是云台初始化后要立即给 DM 电机同一份机构保护范围，避免文件拆分后限位来源分叉。
#define PITCH_MECH_LIMIT_MAX 0.08f
// 定义 Pitch 机械下限，目的是冲坡或姿态突变时仍需由底层限位保护枪管不撞下极限。
#define PITCH_MECH_LIMIT_MIN -0.967f
// 定义 yaw 速度环积分系数，目的是小陀螺属于持续扰动场景，只靠 P 项容易留下稳态偏差，因此补一小段 Ki 来慢慢顶住漂移。
#define YAW_SPEED_PID_KI 0.03f
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
extern DMMotorInstance *pitch_motor;
extern Gimbal_Upload_Data_s gimbal_feedback_data;
extern Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;

// 下面这些内部 helper 只服务 gimbal 模块内部拆分，目的是保持私有头统一声明，既能跨文件复用，又不污染公共头接口。
float NormalizeAngleTo360(float angle_deg);
void ResetPIDRuntimeState(PIDInstance *pid);
void ResetYawMotorRuntimeState(void);
void ResetPitchMotorRuntimeState(void);
void UpdateGimbalCurrentFeedforward(uint8_t gimbal_mode_changed,
                                    uint8_t yaw_motor_online,
                                    uint8_t yaw_motor_online_changed,
                                    uint8_t pitch_motor_online,
                                    uint8_t pitch_motor_online_changed);

#endif // GIMBAL_PRIVATE_H
