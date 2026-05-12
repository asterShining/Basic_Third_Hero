#ifndef SHOOT_PRIVATE_H
#define SHOOT_PRIVATE_H

#include "shoot.h"
#include "motor_def.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

#include <math.h>

// 堵转状态机运行时数据只在 shoot 模块内部共享，目的是拆成多个编译单元后必须集中定义一份，不能再让公共头生成多个独立副本。
typedef struct {
    LoaderStallState_e state;
    float detect_start_time;
    float reverse_start_time;
    float recovery_start_time;
    float reverse_target_angle;
    uint8_t reverse_count;
    loader_mode_e saved_mode;
} ShootStallHandler_s;

// 单发状态机运行时数据只在 shoot 模块内部共享，目的是固定角度送弹、掉速计数和锁角保持都依赖同一份跨拍状态，拆文件后仍必须共用。
typedef struct {
    SingleFireState_e state;
    float baseline_speed;
    float outer_baseline_speed;
    float rush_start_angle;
    float rush_target_angle;
    float lock_target_angle;
    float shot_start_time;
    float feed_start_time;
    float retry_start_time;
    float brake_start_time;
    float cooldown_start_time;
    float increment_target_angle; // 当前增量步进的目标角度，每次只推进一小步后检查掉速，掉速则立即停止形成物理事件闭环
    uint8_t retry_count;
    uint8_t inner_dip_stable_count;
    uint8_t recover_stable_count;
    uint8_t shot_counted; // 本次固定一发送弹事务是否已经由摩擦轮掉速计入 fire_count，目的是把“计数”和“拨盘停止”彻底解耦并防止持续掉速重复计数。
    uint16_t fire_count;
    uint16_t feed_timeout_count;
} SingleFireRuntime_s;

// 单发控制的掉速基线由控制链单独维护，目的是出弹计数不能依赖调试模块内部存储，而要有自己可控的一份同拍数据。
typedef struct {
    float inner_left_baseline;
    float inner_right_baseline;
    float inner_down_baseline;
    float outer_left_baseline;
    float outer_right_baseline;
    float outer_down_baseline;
} DipControlRuntime_s;

// 单发触发边沿锁存只服务 shoot 模块内部，目的是用户边沿请求必须先在入口缓存，再由状态机决定何时真正消费。
typedef struct {
    loader_mode_e last_mode;
    uint8_t trigger_consumed;
    uint8_t pending_fire;
    float last_accept_time_ms;
} FireTrigger_s;

// 下面这些是拆分后多个 `.c` 需要共享的运行时状态，目的是这些状态在初始化、摩擦轮控制、单发状态机和堵转处理之间会被同拍反复读写。
extern float current_inner_deg;
extern float current_outer_deg;
extern float target_inner_left_deg_abs;
extern float target_inner_right_deg_abs;
extern float target_inner_down_deg_abs;
extern float target_outer_left_deg_abs;
extern float target_outer_right_deg_abs;
extern float target_outer_down_deg_abs;
extern uint32_t friction_ramp_dwt_cnt;
extern float ff_inner_left;
extern float ff_inner_right;
extern float ff_inner_down;
extern float ff_outer_left;
extern float ff_outer_right;
extern float ff_outer_down;
extern float ff_loader;
extern DJIMotorInstance *loader;
extern DJIMotorInstance *friction_inner_down;
extern DJIMotorInstance *friction_inner_left;
extern DJIMotorInstance *friction_inner_right;
extern DJIMotorInstance *friction_outer_down;
extern DJIMotorInstance *friction_outer_left;
extern DJIMotorInstance *friction_outer_right;
extern Publisher_t *shoot_pub;
extern Shoot_Ctrl_Cmd_s shoot_cmd_recv;
extern Subscriber_t *shoot_sub;
extern Shoot_Upload_Data_s shoot_feedback_data;
extern float hibernate_time;
extern float dead_time;
extern DipControlRuntime_s dip_control;
extern ShootStallHandler_s stall_handler;
extern SingleFireRuntime_s single_fire;
extern FireTrigger_s fire_trigger;

// 下面这些内部函数会在拆分后的多个 `.c` 之间相互调用，目的是保持私有头统一声明，既能共享内部能力，又不污染公共接口。
float SpeedMps2Degs(float speed_mps);
float SpeedAps2Mps(float speed_aps);
float GetMotorSpeedAps(const DJIMotorInstance *motor);
uint8_t HasOuterFrictionWheel(void);
float LoaderOutputAngleToMotorAngle(float output_angle_deg);
float LoaderBulletCountToMotorAngle(float bullet_count);
float LoaderBulletRateToMotorSpeed(float bullet_rate);
void UpdateFrictionTargetAbs(float inner_left_ref, float inner_right_ref, float inner_down_ref,
                             float outer_left_ref, float outer_right_ref, float outer_down_ref);
void LoaderSetSpeedRef(float speed_ref);
void LoaderSetAngleRef(float angle_ref);
void SetFrictionFeedforward(float inner_ff, float outer_ff);
void UpdateLoaderFeedforward(void);
void SetMotorEnableIfReady(DJIMotorInstance *motor, uint8_t enable);
void HoldLoaderIdlePosition(void);
uint8_t IsAllFrictionStableAgainstTarget(float threshold);
float GetInnerFrictionAvgSpeed(void);
float GetOuterFrictionAvgSpeed(void);
void RecordDipBaseline(void);
void TakeDipSnapshot(void);
void ValidateAndSaveDipSnapshot(void);
void ShootSetSpeedDual(float inner_mps, float outer_mps);
void UpdateFrictionDebugInfo(void);
void AbortSingleFire(void);
uint8_t SingleFireIsRetryActive(void);
void HandleSingleFire(uint8_t trigger_active);
loader_mode_e HandleLoaderStall(loader_mode_e current_mode);

#endif // SHOOT_PRIVATE_H
