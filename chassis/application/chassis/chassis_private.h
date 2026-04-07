#ifndef CHASSIS_PRIVATE_H
#define CHASSIS_PRIVATE_H

// 继续沿用底盘应用内部默认启用超电的编译开关，目的是拆成多个编译单元后必须保证所有底盘私有实现看到同一份超电条件，否则功率链路会出现“部分文件有超电、部分文件无超电”的行为分叉。
#define USE_SUPER_CAP

#include "chassis.h"
#include "can.h"
#include "motor_def.h"
#include "robot_def.h"
#include "power_control.h"
#include "dmmotor.h"
#include "dji_motor.h"
#include "message_center.h"
#include "referee_task.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "referee_UI.h"
#include "user_lib.h"
#include "arm_math.h"

#include <math.h>
#include <stdint.h>

#ifdef CHASSIS_BOARD
// 双板构型下私有实现需要直接访问 CANComm 和底盘板 IMU 类型，目的是这些类型只在底盘应用内部共享，没有必要暴露到公共头。
#include "can_comm.h"
#include "ins_task.h"
// 在编译期校验底盘反馈结构体尺寸，目的是双板通信仍需保证单帧 CAN 可以装下完整反馈数据，拆文件后也不能丢掉这个硬约束。
_Static_assert(sizeof(Chassis_Upload_Data_s) <= CAN_COMM_MAX_BUFFSIZE,
               "Chassis_Upload_Data_s exceeds CAN_COMM_MAX_BUFFSIZE");
// 在编译期校验底盘控制结构体尺寸，目的是新增上岛和跟随字段后仍必须保证命令帧不会溢出 CANComm 缓冲区。
_Static_assert(sizeof(Chassis_Ctrl_Cmd_s) <= CAN_COMM_MAX_BUFFSIZE,
               "Chassis_Ctrl_Cmd_s exceeds CAN_COMM_MAX_BUFFSIZE");
#endif

#ifdef USE_SUPER_CAP
// 超电实例和相关 helper 只服务底盘应用内部，目的是公共头不需要因此额外引入供电链路实现细节。
#include "super_cap.h"
#endif

#ifdef USE_ISLAND_ACTION
// 上岛辅助机构只由底盘应用任务调度，目的是私有头集中包含后，入口文件和任务文件都能共享同一份条件依赖。
#include "island_action.h"
#endif

/* 根据 robot_def.h 中的 macro 自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f) // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f) // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define DEFAULT_TEST_POWER 80.0f // 调试用的基础功率
#define LIFT_DIAL_MAX_SPEED_DPS 65.0f // 定义抬升拨轮映射后的最大调节角速度，目的是提高抬升跟手性同时保持位置环仍在可控范围内
#define LIFT_RELATIVE_MIN_ANGLE -20.0f // 定义相对进入点的最小抬升角，目的是给机构保留回落空间并避免误操作顶到底部极限
#define LIFT_RELATIVE_MAX_ANGLE 80.0f // 定义相对进入点的最大抬升角，目的是用保守软件限位先保护机构，后续可按实车行程再放宽
#define CHASSIS_TASK_DT_FALLBACK 0.005f // 定义底盘任务积分后备周期，目的是DWT 异常时仍按 200Hz 近似积分，避免抬升目标突变
#define FOLLOW_TRANSITION_HOLD_TICKS 20u // 定义小陀螺退跟随后接管保持拍数，目的是约 100ms 的刹停窗口足够先卸掉残余自旋，再进入回正环能明显减少反向抽动。
#define FOLLOW_TRANSITION_EXIT_WZ_DPS 120.0f // 定义接管阶段允许提前退出的底盘角速度阈值，目的是余旋已经很小时没必要继续硬刹，尽早恢复回正能减少体感拖滞。
#define FOLLOW_TRANSITION_BRAKE_KD 1.6f // 定义接管阶段只看车体角速度的阻尼增益，目的是先用纯阻尼把小陀螺残余动量压下，避免偏角环和前馈同时抢控制权。
#define FOLLOW_TRANSITION_MAX_WZ 900.0f // 定义接管阶段底盘角速度输出上限，目的是先刹停阶段只需要中等强度制动，限幅后更不容易把轮速环再次顶入饱和。
#define FOLLOW_ANGLE_FILTER_ALPHA 0.18f // 定义跟随偏角的一阶滤波系数，目的是退出接管时先把机械回弹和单圈角小抖动滤掉，避免刚恢复 P 项就来回翻向。
#define FOLLOW_OUTPUT_SLEW_DPS_PER_S 30000.0f // 定义跟随输出角速度的变化率上限，目的是保留足够快的接管制动，同时避免控制律切段时输出一步跳变。

// ==========================================
// 小陀螺模式配置
// ==========================================
#define VARIABLE_SPIN_ENABLED 1 // 保留小陀螺的轻微变速效果，目的是贴边吃功率时仍保留一点扰动，降低被针对时的运动可预测性。
#define SPIN_BASE_INIT_SPEED 1400.0f // 定义小陀螺进入时的初始基础角速度，目的是首拍就给到中高转速，避免每次进小陀螺都要从很低速度慢慢爬升。
#define SPIN_BASE_MIN_SPEED 900.0f // 定义功率闭环允许维持的最小基础角速度，目的是即使功率余量吃紧也保留稳定自旋，不让姿态突然塌掉。
#define SPIN_TOP_MAX_SPEED 3200.0f // 提高小陀螺基础角速度上限，目的是让功率控制器在合法范围内有足够目标可追，才能把可用功率真正吃满。
#define SPIN_POWER_TARGET_BASE_RATIO 0.96f // 定义小陀螺平均功率目标比例，目的是让整段变速逻辑围绕贴边功率运行，而不是回到过去那种明显留手的保守状态。
#define SPIN_POWER_TARGET_WAVE_AMPLITUDE 0.025f // 定义变速波形对功率目标比例的调制幅度，目的是高速段略多吃一点功率、低速段略收一点，变速节奏会更明显但仍留有安全余量。
#define SPIN_POWER_TRACK_GAIN 10.0f // 定义功率误差到角速度修正的比例系数，目的是让小陀螺能跟着功率余量快速抬速，但不过分激进导致来回抽动。
#define SPIN_POWER_STEP_UP_MAX 20.0f // 限制单周期最大提速量，目的是200Hz 任务下给平滑爬升，避免目标角速度阶跃过大把轮速环瞬间顶饱和。
#define SPIN_POWER_STEP_DOWN_MAX 35.0f // 限制单周期最大降速量，目的是超功率边缘时更快回收旋转目标，优先守住不超功率底线。
#define SPIN_POWER_DEADBAND_W 2.0f // 定义功率误差死区，目的是吃满附近直接忽略微小波动，减少由裁判功率抖动带来的转速抖动。
#define SPIN_WAVE_SCALE_AMPLITUDE 0.15f // 定义小陀螺角速度包络波动幅度，目的是把快慢节奏拉得更开一些，让变速逻辑在场上是明显可感知的。
#define SPIN_WAVE_REFRESH_MIN_MS 180u // 定义无节奏变速目标的最短刷新时间，目的是保证变速方向不会切得过快，避免底盘体感变成抖动。
#define SPIN_WAVE_REFRESH_MAX_MS 520u // 定义无节奏变速目标的最长刷新时间，目的是让目标保持时间也带随机性，避免形成“固定拍点”。
#define SPIN_WAVE_SMOOTH_ALPHA 0.06f // 定义当前波形向随机目标逼近的平滑系数，目的是让无节奏变速保持连续过渡，不出现突兀阶跃。
#define TRANSLATION_PRIORITY_RATIO 0.6f // 定义平移优先系数，目的是贴边吃功率时仍优先保证平移手感，避免横移一给就把整车拖死。
#define DEFAULT_TEST_BUFFER_ENERGY_J 60.0f // 定义无裁判系统时的默认缓冲能量，目的是场下调车也要能走通超电功率策略，不能因为没裁判就永远进不到激进分支。
#define CHASSIS_SUPER_CAP_BONUS_LOW_W 35.0f // 定义超电低能量档的附加功率，目的是电容电量不高时也先给一小档放电，让体感尽快从“没反应”变成“有帮助”。
#define CHASSIS_SUPER_CAP_BONUS_MID_W 60.0f // 定义超电中能量档的附加功率，目的是电容进入可用区后直接给明显增益，体现比基础模式更激进的输出。
#define CHASSIS_SUPER_CAP_BONUS_HIGH_W 75.0f // 定义超电高能量档的附加功率，目的是电容和裁判缓冲都充足时允许更猛地放电，把爆发优势真正打出来。
#define CHASSIS_SUPER_CAP_ROTATE_EXTRA_W 10.0f // 定义小陀螺工况额外附加的超电功率，目的是自旋时功率起伏最大，需要再补一档预算才能让实际转速更敢放。
#define CHASSIS_SUPER_CAP_MIN_PERCENT 15.0f // 定义超电开始介入的最低电量百分比，目的是把起放门槛从保守值下探，让超电更早参与而不是一直等到很满才出手。
#define CHASSIS_SUPER_CAP_MID_PERCENT 35.0f // 定义超电中档功率的电量阈值，目的是分档控制比单阈值更容易同时兼顾激进体感和低电量保护。
#define CHASSIS_SUPER_CAP_HIGH_PERCENT 60.0f // 定义超电高档功率的电量阈值，目的是只有电容余量明显充足时才拉到最高 bonus，避免长期贴顶后一下子掉空。
#define CHASSIS_SUPER_CAP_MIN_BUFFER_J 15.0f // 定义超电开始介入的最小裁判缓冲能量，目的是即使电容电量一般，只要裁判缓冲还厚，也允许先上低档爆发。
#define CHASSIS_SUPER_CAP_MID_BUFFER_J 35.0f // 定义超电中档功率的裁判缓冲阈值，目的是把裁判缓冲一起纳入决策，避免只看电容百分比导致机会窗口利用不足。
#define CHASSIS_SUPER_CAP_HIGH_BUFFER_J 60.0f // 定义超电高档功率的裁判缓冲阈值，目的是缓冲和电容都高时直接进入最猛档，让整车更敢吃功率。
#define CHASSIS_TOTAL_POWER_LIMIT_MAX_W 165.0f // 定义超电介入后的底盘总功率硬上限，目的是用户要求更激进，就把总预算再往上抬一档，但仍保留硬上限避免完全失控。

// 下面这些状态在底盘入口任务和拆分后的 helper 之间共同维护，目的是多个编译单元围绕同一拍控制命令、功率状态和接管锁存协同工作时，必须继续共用同一份运行时数据。
#ifdef CHASSIS_BOARD
extern CANCommInstance *chasiss_can_comm;
extern attitude_t *Chassis_IMU_data;
#endif
extern Chassis_Ctrl_Cmd_s chassis_cmd_recv;
extern referee_info_t *referee_data;
extern Referee_Interactive_info_t ui_data;
#ifdef USE_SUPER_CAP
extern SuperCapInstance *cap;
#endif
extern DJIMotorInstance *motor_lf;
extern DJIMotorInstance *motor_rf;
extern DJIMotorInstance *motor_lb;
extern DJIMotorInstance *motor_rb;
extern float chassis_vx;
extern float chassis_vy;
extern uint8_t last_follow_transition_request;
extern uint8_t last_follow_brake_request;
extern uint8_t follow_transition_ticks;
extern float follow_angle_err_filtered;
extern float follow_wz_cmd_limited;
extern uint32_t follow_control_dwt_cnt;

// 下面这些函数虽然只在底盘模块内部使用，但拆分后要跨多个 `.c` 互相调用，目的是集中在私有头声明，能避免把内部实现泄漏到公共接口。
#ifdef USE_SUPER_CAP
float GetAggressiveSuperCapBonus(float buffer_energy_j, uint8_t chassis_output_allowed);
#endif
float OptimizedSpinSpeed(float base_target_wz, float vx_cmd, float vy_cmd);
float GetAdaptiveSpinBase(float power_limit);
void ResetAdaptiveSpinBase(void);
float GetFollowControlDt(void);
void ResetFollowControlState(void);
void StartFollowTransition(float current_angle_err);
float ApplyFollowCommandSlew(float target_wz, float dt_s);
void MecanumCalculate(void);
void HybridCalculate(void);
void LimitChassisOutput(void);
void EstimateSpeed(void);
void RefereeUIUpdateData(void);

#endif // CHASSIS_PRIVATE_H
