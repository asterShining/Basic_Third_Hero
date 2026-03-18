#include "island_action.h"
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "dmmotor.h"

// 前履带电机实例 (DM 3519)
static DMMotorInstance *front_track_motor_left;
static DMMotorInstance *front_track_motor_right;

// 后抬升电机实例 (DJI 3508)
static DJIMotorInstance *lift_motor_left;
static DJIMotorInstance *lift_motor_right;

// 相关控制参数和宏
#define LIFT_DIAL_MAX_SPEED_DPS 12500.0f // What: 继续提高后抬升拨轮映射后的最大目标角速度；Why: 当前执行链路实际走速度环，直接放大目标速度是加快后轮抬升的最小实现。
#define LIFT_STALL_TARGET_SPEED_THRESHOLD_DPS 300.0f // What: 规定进入堵转检测时的最小目标速度；Why: 过滤近零拨轮输入，避免无效微小命令触发堵转计时。
#define LIFT_STALL_SPEED_THRESHOLD_DPS 120.0f // What: 规定“几乎不转”的速度阈值；Why: 机械撞限位时3508通常会掉到接近零速，用它区分正常运动与堵转。
#define LIFT_STALL_CURRENT_THRESHOLD_RAW 5000.0f // What: 规定堵转检测时的最小电流阈值；Why: 必须同时满足高负载，避免中途摩擦增大被误判成机械限位。
#define LIFT_STALL_CONFIRM_TIME_S 0.20f // What: 规定堵转持续确认时间；Why: 只在持续顶死时才切位置环保持，滤掉启动瞬态和短时碰撞。
#define LIFT_CONTROL_DT_FALLBACK_S 0.005f // What: 定义抬升控制的后备周期；Why: DWT异常时仍按约200Hz估算堵转累计时间，避免首次判定跳变。

static float lift_hold_angle_left = 0.0f; // What: 保存左后抬升保持角度；Why: 命中限位或松手后要立刻切位置环锁住当前位置。
static float lift_hold_angle_right = 0.0f; // What: 保存右后抬升保持角度；Why: 左右两侧必须各自锁位，不能再用平均值互相打架。
static float lift_stall_duration_s = 0.0f; // What: 累计本次连续堵转时长；Why: 只在持续顶死达到阈值后才切位置环保持，避免误判时频繁抢模式。
static lift_mode_e last_lift_mode = LIFT_OFF; // What: 记录上一拍抬升模式；Why: 只在模式切换瞬间切环和锁定当前位置，避免每拍重复改闭环引入抖动。
static uint32_t lift_control_dwt_cnt = 0u; // What: 记录抬升控制节拍计数器；Why: 用真实周期累计堵转确认时间，避免依赖任务频率的隐式假设。

/**
 * @brief 读取单个抬升电机的统一机械角度
 */
static float GetLiftMotorMechanicalAngle(const DJIMotorInstance *motor)
{
    float mechanical_angle = motor->measure.total_angle;
    // DJIMotor底层在闭环时：pid_measure直接等于total_angle。
    // 如果电机设置了REVERSE，其pid_ref会在首环前被乘以-1。
    // 这意味着：对于被定义为REVERSE的电机，如果要让它转到物理的x度，
    // 我们如果传给SetRef的值是t，底层解算会变成目标是-t，实际去向-t度。
    // 但物理装配如果它反装，它正转其实是另一个方向。
    // 但是这里需要获得的是统一坐标系下的当前高度（便于锁定期给SetRef）。

    // 底层逻辑: target_ref = REF * (REVERSE ? -1 : 1)
    // error = target_ref - total_angle
    // 当静止时: REF * (REVERSE ? -1 : 1) == total_angle
    // REF == total_angle * (REVERSE ? -1 : 1)

    // 因此为了得出一个统一的 REF（使得两个电机接收相同的 REF 就能保持高度），
    // 反向电机的 REF 应该是其 total_angle 取反。
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

    // What: 把反装电机的原始测速统一映射到同一机械坐标系；Why: 后续限位判断和堵转判定都必须基于“统一方向”才能比较左右两侧。
    if (motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        mechanical_speed = -mechanical_speed;

    return mechanical_speed;
}

/**
 * @brief 重置后抬升堵转计时
 */
static void ResetLiftStallState(void)
{
    // What: 清空本轮堵转累计时长；Why: 方向切换、松手或退出保持后都不应沿用上一轮的堵转计时残量。
    lift_stall_duration_s = 0.0f;
}

/**
 * @brief 锁存当前后抬升保持角度
 */
static void CaptureLiftHoldPosition(void)
{
    // What: 分别锁存左右后抬升当前机械角度；Why: 双电机机构在切保持时必须按各自当前位置刹住，不能再用平均值互相拉扯。
    lift_hold_angle_left = GetLiftMotorMechanicalAngle(lift_motor_left);
    lift_hold_angle_right = GetLiftMotorMechanicalAngle(lift_motor_right);
}

/**
 * @brief 进入后抬升位置保持模式
 */
static void EnterLiftHoldModeFromCurrentPosition(void)
{
    // What: 先锁存当前位置，再切到位置环并立刻下发保持参考；Why: 命中限位或拨轮松手时要在同一拍内停住，避免多打一拍速度指令继续顶限位。
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
    // What: 在保持模式持续重发左右位置参考；Why: 位置环切入后仍需每拍续写参考，防止被其它模式残留值打断。
    DJIMotorEnable(lift_motor_left);
    DJIMotorEnable(lift_motor_right);
    DJIMotorSetRef(lift_motor_left, lift_hold_angle_left);
    DJIMotorSetRef(lift_motor_right, lift_hold_angle_right);
}

/**
 * @brief 判断后抬升是否进入持续堵转
 */
static uint8_t DetectLiftStallForHold(float target_speed, float dt_s)
{
    float avg_abs_speed;
    float left_abs_current;
    float right_abs_current;

    // What: 只在明确有调节命令时才做堵转检测；Why: 保持态和近零命令下的微小振动不应累积成机械撞限位。
    if (fabsf(target_speed) < LIFT_STALL_TARGET_SPEED_THRESHOLD_DPS) {
        ResetLiftStallState();
        return 0u;
    }

    avg_abs_speed = 0.5f * (fabsf(GetLiftMotorMechanicalSpeed(lift_motor_left)) +
                            fabsf(GetLiftMotorMechanicalSpeed(lift_motor_right)));
    left_abs_current = fabsf((float)lift_motor_left->measure.real_current);
    right_abs_current = fabsf((float)lift_motor_right->measure.real_current);

    // What: 用“低速度 + 双侧高电流”联合判定堵转；Why: 单看速度会把重载慢速误判成堵转，单看电流又会把加速瞬态误判成撞限位。
    if (avg_abs_speed <= LIFT_STALL_SPEED_THRESHOLD_DPS &&
        left_abs_current >= LIFT_STALL_CURRENT_THRESHOLD_RAW &&
        right_abs_current >= LIFT_STALL_CURRENT_THRESHOLD_RAW) {
        lift_stall_duration_s += dt_s;
    } else {
        ResetLiftStallState();
    }

    return lift_stall_duration_s >= LIFT_STALL_CONFIRM_TIME_S;
}

void IslandActionInit(void)
{
    // ================= 前履带 DM 3519 初始化 =================
    Motor_Init_Config_s front_track_motor_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 0.9f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 6.0f,
                .MaxOut = 18.0f,
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

    front_track_motor_config.can_init_config.tx_id = 0x07;
    front_track_motor_config.can_init_config.rx_id = 0x08;
    front_track_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    front_track_motor_left = DMMotorInit(&front_track_motor_config);

    front_track_motor_config.can_init_config.tx_id = 0x09;
    front_track_motor_config.can_init_config.rx_id = 0x10;
    front_track_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    front_track_motor_right = DMMotorInit(&front_track_motor_config);

    // ================= 后抬升 DJI 3508 初始化 =================
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
            .outer_loop_type = SPEED_LOOP, // 默认速度环，按需切换
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = M3508,
    };

    lift_motor_config.can_init_config.tx_id = 6;
    lift_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    lift_motor_left = DJIMotorInit(&lift_motor_config);

    lift_motor_config.can_init_config.tx_id = 5;
    lift_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    lift_motor_right = DJIMotorInit(&lift_motor_config);

    DWT_GetDeltaT(&lift_control_dwt_cnt);
}

void IslandActionControl(const Chassis_Ctrl_Cmd_s *cmd_recv)
{
    float dt_s;
    lift_mode_e applied_lift_mode = cmd_recv->lift_mode;

    if (front_track_motor_left == NULL || front_track_motor_right == NULL ||
        lift_motor_left == NULL || lift_motor_right == NULL)
        return;

    // What: 每拍读取真实控制周期；Why: 堵转保持依赖持续时间，不能偷偷假设任务频率永远固定不漂移。
    dt_s = DWT_GetDeltaT(&lift_control_dwt_cnt);
    if (dt_s <= 0.0f || dt_s > 0.05f)
        dt_s = LIFT_CONTROL_DT_FALLBACK_S;

    // 1. 履带控制：如果处于开启态，直接按传下来的速度闭环运行
    if (cmd_recv->front_track_mode == FRONT_TRACK_ON) {
        DMMotorEnable(front_track_motor_left);
        DMMotorEnable(front_track_motor_right);
        DMMotorSetRef(front_track_motor_left, cmd_recv->front_track_speed_ref);
        DMMotorSetRef(front_track_motor_right, cmd_recv->front_track_speed_ref);
    } else {
        DMMotorSetRef(front_track_motor_left, 0.0f);
        DMMotorSetRef(front_track_motor_right, 0.0f);
        DMMotorStop(front_track_motor_left);
        DMMotorStop(front_track_motor_right);
    }

    // 2. 抬升控制：严格按命令侧下发的模式执行
    switch (cmd_recv->lift_mode) {
    case LIFT_ADJUST: {
        float target_speed = -(cmd_recv->lift_dial_input) * LIFT_DIAL_MAX_SPEED_DPS;

        // What: 调高/下降模式固定使用速度环；Why: 拨轮是速度型输入，统一从这一条链路走可以保证上/下双向都由同一套控制逻辑处理。
        if (last_lift_mode != LIFT_ADJUST) {
            DJIMotorOuterLoop(lift_motor_left, SPEED_LOOP);
            DJIMotorOuterLoop(lift_motor_right, SPEED_LOOP);
            ResetLiftStallState();
        }

        // What: 连续满足堵转条件后立即切到当前位置位置环；Why: 用户要求取消自动保存零点，只在撞限位当次停住机构。
        if (DetectLiftStallForHold(target_speed, dt_s) != 0u) {
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

    case LIFT_HOLD:
        // What: 保持模式在进入时锁存左右当前位置；Why: 抬升双电机存在背隙和微小装配误差，保持时必须继续各锁各的位置而不是重新求平均。
        if (last_lift_mode != LIFT_HOLD)
            EnterLiftHoldModeFromCurrentPosition();
        else
            ApplyLiftHoldRef();
        break;

    case LIFT_OFF:
    default:
        // What: 关闭模式直接停止3508输出；Why: 退出上岛会话后不应继续后台保持，避免辅助机构在非工作态仍然对机构施力。
        ResetLiftStallState();
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
        // What: 停机前记录左后抬升当前位置；Why: 停止态恢复到保持态时能从最近一次真实位置重新接管，不沿用陈旧参考。
        lift_hold_angle_left = GetLiftMotorMechanicalAngle(lift_motor_left);
        DJIMotorSetRef(lift_motor_left, lift_hold_angle_left);
        DJIMotorStop(lift_motor_left);
    }
    if (lift_motor_right != NULL) {
        // What: 停机前记录右后抬升当前位置；Why: 左右电机独立锁位，不能把右侧停机参考继续绑到左侧平均角度上。
        lift_hold_angle_right = GetLiftMotorMechanicalAngle(lift_motor_right);
        DJIMotorSetRef(lift_motor_right, lift_hold_angle_right);
        DJIMotorStop(lift_motor_right);
    }
    ResetLiftStallState();
    last_lift_mode = LIFT_OFF;
}
