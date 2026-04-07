#include "island_action.h"

#ifdef USE_ISLAND_ACTION // [条件编译] 仅在启用上岛机构时编译整个实现，目的是禁用时此翻译单元为空，不产生任何符号
#include "bsp_dwt.h"
#include "controller.h" // 引入通用PID模块，目的是自动调平仍沿用现有 pitch 速度闭环，不额外新建控制器实现。
#include "dji_motor.h"
#include "dmmotor.h"

// 前履带电机实例 (DM 3519)
static DMMotorInstance *front_track_motor_left;
static DMMotorInstance *front_track_motor_right;

// 后抬升电机实例 (DJI 3508)
static DJIMotorInstance *lift_motor_left;
static DJIMotorInstance *lift_motor_right;

// ==================== 抬升相关宏 ====================
#define LIFT_DIAL_MAX_SPEED_DPS 12500.0f // 定义手动抬升拨轮映射的最大目标角速度，目的是保持当前已经加快后的手动抬升速度不回退。
#define LIFT_STALL_TARGET_SPEED_THRESHOLD_DPS 300.0f // 定义抬升堵转检测的最小目标速度，目的是近零命令下的保持抖动不应被误判成撞限位。
#define LIFT_STALL_SPEED_THRESHOLD_DPS 50.0f // 定义抬升“几乎不转”的速度阈值，目的是3508撞到机械限位时通常会掉到接近零速。
#define LIFT_STALL_CURRENT_THRESHOLD_RAW 7000.0f // 定义抬升堵转检测的电流阈值，目的是必须同时满足高负载，才能把慢速重载和真堵转区分开。
#define LIFT_STALL_CONFIRM_TIME_S 0.20f // 定义抬升堵转持续确认时间，目的是过滤启动瞬态和短时碰撞，避免误触发保持或报码。
#define LIFT_CONTROL_DT_FALLBACK_S 0.005f // 定义辅助机构控制的后备周期，目的是DWT异常时仍按200Hz近似累计时间，不让首次判定跳变。
#define LIFT_AUTO_LEVEL_MAX_SPEED_DPS 12500.0f // 定义自动调平PID的输出限幅，目的是继续沿用当前抬升速度上限，不再单独压小自动调平响应。
#define LIFT_AUTO_LEVEL_DEADBAND_DEG 0.3f // 收窄自动调平pitch死区，目的是用户反馈闭环太慢，小角度误差也应尽快开始修正。
#define LIFT_AUTO_LEVEL_PID_KP 650.0f // 提高自动调平比例系数，目的是让 pitch 偏差一出现就给出更强的抬升速度响应。
#define LIFT_AUTO_LEVEL_PID_KI 8.0f // 适度提高自动调平积分系数，目的是更快消除坡面上的残余稳态误差，而不是长期挂着小角度偏差。
#define LIFT_AUTO_LEVEL_PID_KD 20.0f // 降低自动调平微分系数，目的是减少对快速变化测量的保守抑制，避免闭环被过大的D项拖慢。
#define LIFT_AUTO_LEVEL_PID_INTEGRAL_LIMIT 4000.0f // 放宽自动调平积分限幅，目的是提高持续坡面场景下的纠偏余量，但仍受整体输出限幅保护。
#define LIFT_RETRACT_SPEED_DPS (-15000.0f) // 定义更快的快速收腿目标速度且修正为反向，目的是实车确认当前快速收腿方向反了，最小修复就是只翻转收腿固定速度符号。

// ==================== 前履带堵转相关宏 ====================
#define TRACK_STALL_SPEED_THRESHOLD 0.5f // 定义前履带“几乎不转”的速度阈值，目的是DM电机速度反馈单位是rad/s，卡住时接近零速。
#define TRACK_STALL_TORQUE_THRESHOLD 8.0f // 定义前履带堵转扭矩阈值，目的是DM3519高扭矩低速度同时出现才说明履带真被卡住。
#define TRACK_STALL_CONFIRM_TIME_S 0.15f // 定义前履带堵转确认时间，目的是过滤瞬态碰撞和启动冲击，避免正常负载下被过早停机。

typedef struct {
    float stall_duration_s; // 累计当前方向的连续堵转时长，目的是只有持续顶死达到阈值才认为是真堵转。
} IslandMotorStallState_s;

// ==================== 前履带堵转状态变量 ====================
static IslandMotorStallState_s track_stall_left;
static IslandMotorStallState_s track_stall_right;
static uint8_t track_stall_latched = 0u; // 记录前履带是否已因堵转进入停机锁存，目的是堵转后必须保持停机，避免前履带在误判边缘来回正反切换。

// ==================== 抬升状态变量 ====================
static IslandMotorStallState_s lift_stall_left;
static IslandMotorStallState_s lift_stall_right;
static float lift_hold_angle_left = 0.0f; // 保存左后抬升保持角度，目的是位置环保持必须锁住各自的当前高度而不是用平均值。
static float lift_hold_angle_right = 0.0f; // 保存右后抬升保持角度，目的是双电机存在装配误差，必须独立锁位。
static uint8_t lift_retract_bottom_latched = 0u; // 记录快速收腿是否已经撞到底部，目的是上层持续发 `LIFT_RETRACT` 时不能每拍重复顶底限位。
static lift_mode_e last_lift_mode = LIFT_OFF; // 记录上一拍抬升模式，目的是只有在模式切换边沿才需要切外环和重置局部状态。
static uint32_t lift_control_dwt_cnt = 0u; // 记录辅助机构控制节拍计数器，目的是堵转确认逻辑依赖真实控制周期，不能用不稳定的拍间隔累加。

// 底盘pitch自动调平PID实例，目的是继续复用现有速度环架构，只调整参数来提高自动调平响应。
static PIDInstance lift_pitch_pid;

/**
 * @brief 读取单个抬升电机的统一机械角度
 */
static float GetLiftMotorMechanicalAngle(const DJIMotorInstance *motor)
{
    float mechanical_angle = motor->measure.total_angle;

    // 把反装电机的反馈统一到同一机械坐标系，目的是位置保持和堵转判断都必须基于统一方向比较左右两侧。
    if (motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        mechanical_angle = -mechanical_angle;

    return mechanical_angle;
}

/**
 * @brief 读取单个抬升电机的统一机械角速度
 */
static float GetLiftMotorMechanicalSpeed(const DJIMotorInstance *motor)
{
    float mechanical_speed = motor->measure.speed_aps;

    // 把反装电机的速度反馈统一到同一机械坐标系，目的是左右抬升电机的独立堵转判定必须在同方向下进行。
    if (motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        mechanical_speed = -mechanical_speed;

    return mechanical_speed;
}

/**
 * @brief 清空单个电机的堵转状态
 */
static void ResetSingleMotorStallState(IslandMotorStallState_s *stall_state)
{
    // 清空单个电机的连续堵转时间，目的是条件解除后下次堵转要重新累计确认，不能沿用上一轮残量。
    stall_state->stall_duration_s = 0.0f;
}

/**
 * @brief 更新单个电机的堵转状态
 */
static uint8_t UpdateMotorStallState(IslandMotorStallState_s *stall_state,
                                     uint8_t stall_condition_met,
                                     float dt_s,
                                     float confirm_time_s)
{
    if (stall_condition_met != 0u) {
        stall_state->stall_duration_s += dt_s;
        if (stall_state->stall_duration_s >= confirm_time_s)
            return 1u;
        return 0u;
    }

    ResetSingleMotorStallState(stall_state);
    return 0u;
}

/**
 * @brief 重置后抬升堵转状态
 */
static void ResetLiftStallState(void)
{
    // 同时清空左右抬升的堵转累计，目的是切模式、退出保持或解除撞限位后都应从干净状态重新判定。
    ResetSingleMotorStallState(&lift_stall_left);
    ResetSingleMotorStallState(&lift_stall_right);
}

/**
 * @brief 重置前履带电机堵转状态
 */
static void ResetTrackMotorStallState(void)
{
    // 同时清空左右履带的堵转累计，目的是前履带重新启动或解除堵转后必须从零重新确认堵转。
    ResetSingleMotorStallState(&track_stall_left);
    ResetSingleMotorStallState(&track_stall_right);
}

/**
 * @brief 重置前履带整体堵转状态
 */
static void ResetTrackStallState(void)
{
    // 一次性清空履带堵转累计和停机锁存，目的是只有前履带明确关闭或整机停机后，才允许下一次重新启动履带。
    ResetTrackMotorStallState();
    track_stall_latched = 0u;
}

/**
 * @brief 锁存当前后抬升保持角度
 */
static void CaptureLiftHoldPosition(void)
{
    // 分别锁存左右抬升当前位置，目的是双电机切位置环保持时必须各锁各的真实机械位置。
    lift_hold_angle_left = GetLiftMotorMechanicalAngle(lift_motor_left);
    lift_hold_angle_right = GetLiftMotorMechanicalAngle(lift_motor_right);
}

/**
 * @brief 进入后抬升位置保持模式
 */
static void EnterLiftHoldModeFromCurrentPosition(void)
{
    // 锁存当前位置并切到位置环保持，目的是抬升撞限位或松手时需要在同一拍内止住，不能多打一拍速度命令。
    CaptureLiftHoldPosition();
    DJIMotorOuterLoop(lift_motor_left, ANGLE_LOOP);
    DJIMotorOuterLoop(lift_motor_right, ANGLE_LOOP);
    DJIMotorEnable(lift_motor_left);
    DJIMotorEnable(lift_motor_right);
    DJIMotorSetRef(lift_motor_left, lift_hold_angle_left);
    DJIMotorSetRef(lift_motor_right, lift_hold_angle_right);
    ResetLiftStallState();
}

/**
 * @brief 下发后抬升保持参考
 */
static void ApplyLiftHoldRef(void)
{
    // 保持模式每拍续写位置参考，目的是防止其他模式残留值覆盖掉当前位置保持目标。
    DJIMotorEnable(lift_motor_left);
    DJIMotorEnable(lift_motor_right);
    DJIMotorSetRef(lift_motor_left, lift_hold_angle_left);
    DJIMotorSetRef(lift_motor_right, lift_hold_angle_right);
}

/**
 * @brief 按当前目标速度更新左右抬升电机的独立堵转状态
 */
static uint8_t DetectLiftStall(float target_speed, float dt_s)
{
    uint8_t left_stalled;
    uint8_t right_stalled;
    uint8_t enable_detection = (uint8_t)(fabsf(target_speed) >= LIFT_STALL_TARGET_SPEED_THRESHOLD_DPS);

    if (enable_detection == 0u) {
        ResetLiftStallState();
        return 0u;
    }

    left_stalled = UpdateMotorStallState(
        &lift_stall_left,
        (uint8_t)(fabsf(GetLiftMotorMechanicalSpeed(lift_motor_left)) <= LIFT_STALL_SPEED_THRESHOLD_DPS &&
                  fabsf((float)lift_motor_left->measure.real_current) >= LIFT_STALL_CURRENT_THRESHOLD_RAW),
        dt_s,
        LIFT_STALL_CONFIRM_TIME_S);
    right_stalled = UpdateMotorStallState(
        &lift_stall_right,
        (uint8_t)(fabsf(GetLiftMotorMechanicalSpeed(lift_motor_right)) <= LIFT_STALL_SPEED_THRESHOLD_DPS &&
                  fabsf((float)lift_motor_right->measure.real_current) >= LIFT_STALL_CURRENT_THRESHOLD_RAW),
        dt_s,
        LIFT_STALL_CONFIRM_TIME_S);

    // 任一侧确认堵转就认为整体抬升机构到达极限，目的是机械结构是刚性联动，一侧顶死后继续推只会让另一侧受力打架。
    return (uint8_t)(left_stalled || right_stalled);
}

/**
 * @brief 根据履带堵转状态决定正常速度还是停机保护
 */
static float TrackStallProtectControl(float target_speed, float dt_s)
{
    if (track_stall_latched != 0u) {
        // 堵转锁存后持续输出零速，目的是必须保持停机等待人工重新开关履带，避免原地来回抽动。
        return 0.0f;
    }

    if (UpdateMotorStallState(
            &track_stall_left,
            (uint8_t)(fabsf(front_track_motor_left->measure.velocity) <= TRACK_STALL_SPEED_THRESHOLD &&
                      fabsf(front_track_motor_left->measure.torque) >= TRACK_STALL_TORQUE_THRESHOLD &&
                      fabsf(target_speed) > 0.5f),
            dt_s,
            TRACK_STALL_CONFIRM_TIME_S) != 0u ||
        UpdateMotorStallState(
            &track_stall_right,
            (uint8_t)(fabsf(front_track_motor_right->measure.velocity) <= TRACK_STALL_SPEED_THRESHOLD &&
                      fabsf(front_track_motor_right->measure.torque) >= TRACK_STALL_TORQUE_THRESHOLD &&
                      fabsf(target_speed) > 0.5f),
            dt_s,
            TRACK_STALL_CONFIRM_TIME_S) != 0u) {
        // 任一侧履带确认堵转后锁存停机，目的是现场已经出现反复正反切换，最小可靠方案就是直接停住等待人工重启。
        ResetTrackMotorStallState();
        track_stall_latched = 1u;
        return 0.0f;
    }

    return target_speed;
}

void IslandActionInit(void)
{
    Motor_Init_Config_s front_track_motor_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 1.2f,
                .Ki = 0.3f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 6.0f,
                .MaxOut = 19.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = MOTOR_TYPE_NONE,
    };
    Motor_Init_Config_s lift_motor_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 12.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 4000.0f,
                .MaxOut = 2500.0f,
            },
            .speed_PID = {
                .Kp = 4.5f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 4000.0f,
                .MaxOut = 16000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    PID_Init_Config_s pitch_pid_config = {
        .Kp = LIFT_AUTO_LEVEL_PID_KP,
        .Ki = LIFT_AUTO_LEVEL_PID_KI,
        .Kd = LIFT_AUTO_LEVEL_PID_KD,
        .MaxOut = LIFT_AUTO_LEVEL_MAX_SPEED_DPS,
        .DeadBand = LIFT_AUTO_LEVEL_DEADBAND_DEG,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = LIFT_AUTO_LEVEL_PID_INTEGRAL_LIMIT,
    };

    front_track_motor_config.can_init_config.tx_id = 0x07;
    front_track_motor_config.can_init_config.rx_id = 0x08;
    front_track_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    front_track_motor_left = DMMotorInit(&front_track_motor_config);

    front_track_motor_config.can_init_config.tx_id = 0x09;
    front_track_motor_config.can_init_config.rx_id = 0x10;
    front_track_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    front_track_motor_right = DMMotorInit(&front_track_motor_config);

    lift_motor_config.can_init_config.tx_id = 6;
    lift_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    lift_motor_left = DJIMotorInit(&lift_motor_config);

    lift_motor_config.can_init_config.tx_id = 5;
    lift_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    lift_motor_right = DJIMotorInit(&lift_motor_config);

    // 初始化自动调平PID为更快响应参数，目的是当前闭环结构够用，直接调大P、收窄死区比改协议和新控制器更稳妥。
    PIDInit(&lift_pitch_pid, &pitch_pid_config);
    DWT_GetDeltaT(&lift_control_dwt_cnt);
}

void IslandActionControl(const Chassis_Ctrl_Cmd_s *cmd_recv, float chassis_pitch_deg)
{
    float dt_s;
    lift_mode_e applied_lift_mode = cmd_recv->lift_mode;

    if (front_track_motor_left == NULL || front_track_motor_right == NULL ||
        lift_motor_left == NULL || lift_motor_right == NULL)
        return;

    dt_s = DWT_GetDeltaT(&lift_control_dwt_cnt);
    if (dt_s <= 0.0f || dt_s > 0.05f)
        dt_s = LIFT_CONTROL_DT_FALLBACK_S;

    if (cmd_recv->lift_mode != LIFT_RETRACT && lift_retract_bottom_latched != 0u) {
        // 只有真正离开收腿模式后才清掉“已到底”的锁存，目的是上层保持中位持续发 `LIFT_RETRACT` 时不能反复重新顶底。
        lift_retract_bottom_latched = 0u;
    }

    if (cmd_recv->front_track_mode == FRONT_TRACK_ON) {
        float actual_track_speed;

        DMMotorEnable(front_track_motor_left);
        DMMotorEnable(front_track_motor_right);
        actual_track_speed = TrackStallProtectControl(cmd_recv->front_track_speed_ref, dt_s);
        DMMotorSetRef(front_track_motor_left, actual_track_speed);
        DMMotorSetRef(front_track_motor_right, actual_track_speed);
    } else {
        DMMotorSetRef(front_track_motor_left, 0.0f);
        DMMotorSetRef(front_track_motor_right, 0.0f);
        DMMotorStop(front_track_motor_left);
        DMMotorStop(front_track_motor_right);
        ResetTrackStallState(); // 履带关闭时重置堵转与反转状态，目的是下次重新开启时必须从干净的正常履带状态起步。
    }

    switch (cmd_recv->lift_mode) {
    case LIFT_ADJUST: {
        float target_speed = -(cmd_recv->lift_dial_input) * LIFT_DIAL_MAX_SPEED_DPS;

        if (last_lift_mode != LIFT_ADJUST) {
            // 手动调节模式固定使用速度环，目的是拨轮本质是速度型输入，直接映射速度环最符合当前人机接口。
            DJIMotorOuterLoop(lift_motor_left, SPEED_LOOP);
            DJIMotorOuterLoop(lift_motor_right, SPEED_LOOP);
            ResetLiftStallState();
        }

        if (DetectLiftStall(target_speed, dt_s) != 0u) {
            // 任一侧确认撞限位就切当前位置保持，目的是防止继续顶死机构，同时保留当前后抬升高度。
            EnterLiftHoldModeFromCurrentPosition();
            applied_lift_mode = LIFT_HOLD;
            break;
        }

        DJIMotorEnable(lift_motor_left);
        DJIMotorEnable(lift_motor_right);
        DJIMotorSetRef(lift_motor_left, target_speed);
        DJIMotorSetRef(lift_motor_right, target_speed);
        break;
    }

    case LIFT_AUTO_LEVEL: {
        float auto_speed;

        if (last_lift_mode != LIFT_AUTO_LEVEL) {
            // 自动调平继续走抬升速度环，目的是保持现有架构，只通过更积极的PID参数提高响应速度。
            DJIMotorOuterLoop(lift_motor_left, SPEED_LOOP);
            DJIMotorOuterLoop(lift_motor_right, SPEED_LOOP);
            ResetLiftStallState();
        }

        auto_speed = PIDCalculate(&lift_pitch_pid, chassis_pitch_deg, 0.0f);
        if (DetectLiftStall(auto_speed, dt_s) != 0u) {
            EnterLiftHoldModeFromCurrentPosition();
            applied_lift_mode = LIFT_HOLD;
            break;
        }

        DJIMotorEnable(lift_motor_left);
        DJIMotorEnable(lift_motor_right);
        DJIMotorSetRef(lift_motor_left, auto_speed);
        DJIMotorSetRef(lift_motor_right, auto_speed);
        break;
    }

    case LIFT_RETRACT: {
        if (lift_retract_bottom_latched != 0u) {
            // 已经确认收到底后继续保持当前位置位置环，目的是用户要求堵转后切位置环，避免到底后电机直接松掉。
            if (last_lift_mode != LIFT_HOLD)
                EnterLiftHoldModeFromCurrentPosition();
            else
                ApplyLiftHoldRef();
            applied_lift_mode = LIFT_HOLD;
            ResetLiftStallState();
            break;
        }

        if (last_lift_mode != LIFT_RETRACT) {
            // 快速收腿模式切回速度环，目的是仍然沿用现有收腿速度控制链路，不改双板协议和模式语义。
            DJIMotorOuterLoop(lift_motor_left, SPEED_LOOP);
            DJIMotorOuterLoop(lift_motor_right, SPEED_LOOP);
            ResetLiftStallState();
        }

        if (DetectLiftStall(LIFT_RETRACT_SPEED_DPS, dt_s) != 0u) {
            // 收腿撞底后立刻切当前位置位置环并锁存“已到底”，目的是到底时既要停止继续下压，也要维持当前腿位不回弹。
            EnterLiftHoldModeFromCurrentPosition();
            lift_retract_bottom_latched = 1u;
            applied_lift_mode = LIFT_HOLD;
            break;
        }

        DJIMotorEnable(lift_motor_left);
        DJIMotorEnable(lift_motor_right);
        DJIMotorSetRef(lift_motor_left, LIFT_RETRACT_SPEED_DPS);
        DJIMotorSetRef(lift_motor_right, LIFT_RETRACT_SPEED_DPS);
        break;
    }

    case LIFT_HOLD:
        if (last_lift_mode != LIFT_HOLD)
            EnterLiftHoldModeFromCurrentPosition();
        else
            ApplyLiftHoldRef();
        break;

    case LIFT_OFF:
    default:
        ResetLiftStallState(); // 抬升关闭时清掉堵转累计，目的是非工作态重新进来不应继承旧的限位计时。
        DJIMotorSetRef(lift_motor_left, 0.0f);
        DJIMotorSetRef(lift_motor_right, 0.0f);
        DJIMotorStop(lift_motor_left);
        DJIMotorStop(lift_motor_right);
        break;
    }

    last_lift_mode = applied_lift_mode;
}

void IslandActionStop(void)
{
    if (front_track_motor_left != NULL) {
        DMMotorSetRef(front_track_motor_left, 0.0f);
        DMMotorStop(front_track_motor_left);
    }
    if (front_track_motor_right != NULL) {
        DMMotorSetRef(front_track_motor_right, 0.0f);
        DMMotorStop(front_track_motor_right);
    }
    if (lift_motor_left != NULL) {
        lift_hold_angle_left = GetLiftMotorMechanicalAngle(lift_motor_left);
        DJIMotorSetRef(lift_motor_left, lift_hold_angle_left);
        DJIMotorStop(lift_motor_left);
    }
    if (lift_motor_right != NULL) {
        lift_hold_angle_right = GetLiftMotorMechanicalAngle(lift_motor_right);
        DJIMotorSetRef(lift_motor_right, lift_hold_angle_right);
        DJIMotorStop(lift_motor_right);
    }

    // 急停或零力时同步清掉辅助机构的局部状态，目的是下一次恢复控制必须从完全干净的辅助机构状态开始。
    ResetLiftStallState();
    ResetTrackStallState();
    lift_retract_bottom_latched = 0u;
    last_lift_mode = LIFT_OFF;
}

#endif // USE_ISLAND_ACTION
