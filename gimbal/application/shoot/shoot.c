#include "shoot.h"
#include "master_process.h"
#include "motor_def.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

static float current_inner_deg = 0.0f;
static float current_outer_deg = 0.0f;

static DJIMotorInstance *loader; // 拨盘电机
static DJIMotorInstance *friction_inner_down, *friction_inner_left, *friction_inner_right; // 内部摩擦轮电机
static DJIMotorInstance *friction_outer_down, *friction_outer_left, *friction_outer_right; // 外部摩擦轮电机
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

ShootDebugSpeed_s shoot_debug_speed;

// [新增] 全局堵转调试变量定义 (用于调试器实时观测堵转状态机状态)
StallDebug_s stall_debug = { 0 };

// [新增] 全局单发调试变量定义 (用于调试器实时观测单发状态机状态)
SingleFireDebug_s sf_debug = { 0 };

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

void ShootInit()
{
    // 内摩擦轮
    Motor_Init_Config_s friction_config_inner = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 7.2, // 20
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 16000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508
    };
    // 外摩擦轮
    Motor_Init_Config_s friction_config_outer = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 8,
                .Ki = 1,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 16000,
            },

        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508
    };
    // 第一级，内摩擦轮初始化
    friction_config_inner.can_init_config.tx_id = 1; // 上摩擦轮,改txid和方向就行
    friction_config_inner.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_inner_down = DJIMotorInit(&friction_config_inner);

    friction_config_inner.can_init_config.tx_id = 2; // 左摩擦轮,改txid和方向就行
    friction_config_inner.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_inner_left = DJIMotorInit(&friction_config_inner);

    friction_config_inner.can_init_config.tx_id = 3; // 右摩擦轮,改txid和方向就行
    friction_config_inner.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_inner_right = DJIMotorInit(&friction_config_inner);

    // // 第二级，外摩擦轮初始化
    friction_config_outer.can_init_config.tx_id = 4; // 上摩擦轮,改txid和方向就行
    friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_outer_down = DJIMotorInit(&friction_config_outer);

    friction_config_outer.can_init_config.tx_id = 5; // 左摩擦轮,改txid和方向就行
    friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_outer_left = DJIMotorInit(&friction_config_outer);

    friction_config_outer.can_init_config.tx_id = 6; // 右摩擦轮,改txid和方向就行
    friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_outer_right = DJIMotorInit(&friction_config_outer);
    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 6.3, // 10
                .Ki = 0.0,
                .Kd = 0,
                .MaxOut = 13000, // 角度环输出限幅 (deg/s), 提高以允许更大力矩

            },
            .speed_PID = {
                .Kp = 5.6, // 10
                .Ki = 1.0, // 1
                .Kd = 0.0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 14000,
            },

        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M3508 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}
/**
 * @brief 辅助函数：将线速度转换为角速度
 */
float SpeedMps2Degs(float speed_mps)
{
    if (speed_mps <= 0.0f)
        return 0.0f;
    if (speed_mps > 20.0f)
        speed_mps = 20.0f; // 安全限幅

    // 公式: (线速度 / 半径) * (180/PI) * 补偿系数
    return (speed_mps / FRICTION_WHEEL_RADIUS) * RAD_2_DEGREE * SLIP_COMPENSATION;
}

/**
 * @brief 分别设置两级摩擦轮的速度
 * @param inner_mps 第一级（内圈）目标射速 m/s
 * @param outer_mps 第二级（外圈）目标射速 m/s
 */
void ShootSetSpeedDual(float inner_mps, float outer_mps)
{
    // 1. 计算各自的目标角速度 (deg/s)
    float target_inner_deg = SpeedMps2Degs(inner_mps);
    float target_outer_deg = SpeedMps2Degs(outer_mps);

    // [新增] 软启动斜坡算法 (Ramp Logic)
    // Inner
    float diff_inner = target_inner_deg - current_inner_deg;
    if (diff_inner > FRICTION_RAMP_STEP)
        current_inner_deg += FRICTION_RAMP_STEP;
    else if (diff_inner < -FRICTION_RAMP_STEP)
        current_inner_deg -= FRICTION_RAMP_STEP;
    else
        current_inner_deg = target_inner_deg;

    // Outer
    float diff_outer = target_outer_deg - current_outer_deg;
    if (diff_outer > FRICTION_RAMP_STEP)
        current_outer_deg += FRICTION_RAMP_STEP;
    else if (diff_outer < -FRICTION_RAMP_STEP)
        current_outer_deg -= FRICTION_RAMP_STEP;
    else
        current_outer_deg = target_outer_deg;

    // 2. 设置第一级（内圈3个电机）- 负责主要加速
    // 注意：你的 shoot.c 初始化了 down, left, right 三个电机
    DJIMotorSetRef(friction_inner_left, current_inner_deg);
    DJIMotorSetRef(friction_inner_right, current_inner_deg);
    DJIMotorSetRef(friction_inner_down, current_inner_deg);

    // 3. 设置第二级（外圈3个电机）- 负责稳速/微加速
    DJIMotorSetRef(friction_outer_left, current_outer_deg);
    DJIMotorSetRef(friction_outer_right, current_outer_deg);
    DJIMotorSetRef(friction_outer_down, current_outer_deg);
}
/**
 * @brief 将电机反馈的角速度(deg/s)转换为线速度(m/s)用于调试
 * @note 不包含打滑补偿，反映的是摩擦轮表面的物理线速度
 */
static void UpdateDebugSpeed(void)
{
// 定义局部宏简化代码
#define CALC_MPS(motor) ((motor->measure.speed_aps / RAD_2_DEGREE) * FRICTION_WHEEL_RADIUS)

    // 注意：这里保留了正负号，方便观察电机是否反转。
    // 如果你只想看数值大小，可以使用 fabsf() 函数取绝对值。
    if (friction_inner_left)
        shoot_debug_speed.inner_left = CALC_MPS(friction_inner_left);
    if (friction_inner_right)
        shoot_debug_speed.inner_right = CALC_MPS(friction_inner_right);
    if (friction_inner_down)
        shoot_debug_speed.inner_down = CALC_MPS(friction_inner_down);

    if (friction_outer_left)
        shoot_debug_speed.outer_left = CALC_MPS(friction_outer_left);
    if (friction_outer_right)
        shoot_debug_speed.outer_right = CALC_MPS(friction_outer_right);
    if (friction_outer_down)
        shoot_debug_speed.outer_down = CALC_MPS(friction_outer_down);
}

/**
 * @brief 检测拨盘电机是否堵转
 * @return 1 表示当前处于堵转状态（高电流+低转速），0 表示正常
 * @note 使用电机反馈的实际电流和速度进行判断
 */
static uint8_t IsLoaderStalled(void)
{
    // 获取电机反馈数据: |real_current| > 阈值 且 |speed_aps| < 阈值
    int16_t current_abs = (loader->measure.real_current > 0) ?
                              loader->measure.real_current :
                              -loader->measure.real_current;
    float speed_abs = (loader->measure.speed_aps > 0.0f) ?
                          loader->measure.speed_aps :
                          -loader->measure.speed_aps;

    // [新增] 更新全局调试变量的电流和速度信息 (供调试器实时观测)
    stall_debug.current_abs = current_abs;
    stall_debug.speed_abs = speed_abs;
    stall_debug.is_stalled = (current_abs > STALL_CURRENT_THRESHOLD) &&
                             (speed_abs < STALL_SPEED_THRESHOLD);

    return stall_debug.is_stalled;
}

/**
 * @brief 获取内圈摩擦轮平均速度 (deg/s)
 * @return 三个内圈摩擦轮速度的平均绝对值
 * @note 使用内圈检测是因为弹丸先接触内圈, 信号更早, 防多发效果更好
 */
static float GetInnerFrictionAvgSpeed(void)
{
    // 计算三个内圈摩擦轮的平均绝对速度
    // 取绝对值是因为电机方向可能不同, 我们只关心速度大小
    float avg = (fabsf(friction_inner_left->measure.speed_aps) +
                 fabsf(friction_inner_right->measure.speed_aps) +
                 fabsf(friction_inner_down->measure.speed_aps)) /
                3.0f;
    return avg;
}

/**
 * @brief 检测内圈摩擦轮是否发生掉速
 * @return 1 表示检测到掉速 (有弹丸通过), 0 表示正常
 * @note 通过与发射前记录的基准速度对比来判断是否掉速
 */
static uint8_t IsFrictionDipping(void)
{
    float current_speed = GetInnerFrictionAvgSpeed();
    // 与基准速度对比, 下降超过阈值则认为掉速 (有弹丸正在通过)
    return (single_fire.baseline_speed - current_speed) > FRICTION_SPEED_DIP_THRESHOLD;
}

/**
 * @brief 堵转检测与自动反转处理状态机
 * @param current_mode 当前的发射模式
 * @return 经过堵转处理后的发射模式 (可能被临时替换为反转模式)
 * @note 该函数实现无感反转逻辑,在检测到堵转时自动反转一小段角度
 */
static loader_mode_e HandleLoaderStall(loader_mode_e current_mode)
{
    float current_time = DWT_GetTimeline_ms();

    // 仅在发射模式下进行堵转检测 (单发/二连发/三连发/连发)
    // LOAD_STOP 和 LOAD_REVERSE 模式不进行检测
    // 仅在连发模式下进行通用堵转检测
    // 单发模式由 HandleSingleFire 自行处理超时与异常
    uint8_t is_shooting = (current_mode == LOAD_2_BULLET) ||
                          (current_mode == LOAD_3_BULLET) ||
                          (current_mode == LOAD_BURSTFIRE);

// [新增] 宏定义: 返回前统一更新调试状态 (避免代码重复)
#define RETURN_WITH_DEBUG(mode)                  \
    do {                                         \
        stall_debug.state = stall_handler.state; \
        return (mode);                           \
    } while (0)

    switch (stall_handler.state) {
    case STALL_NORMAL:
        // 正常状态: 检测是否进入堵转
        if (is_shooting && IsLoaderStalled()) {
            // 检测到疑似堵转,进入消抖阶段
            stall_handler.state = STALL_DETECTING;
            stall_handler.detect_start_time = current_time;
            stall_handler.saved_mode = current_mode; // 保存当前模式
        }
        // 如果当前是主动反转或停止,重置反转计数
        if (current_mode == LOAD_REVERSE || current_mode == LOAD_STOP) {
            stall_handler.reverse_count = 0;
        }
        RETURN_WITH_DEBUG(current_mode); // 正常模式不修改

    case STALL_DETECTING:
        // 消抖阶段: 持续检测堵转状态
        if (!IsLoaderStalled()) {
            // 堵转消失,恢复正常
            stall_handler.state = STALL_NORMAL;
            RETURN_WITH_DEBUG(current_mode);
        }
        // 检查是否超过消抖时间
        if ((current_time - stall_handler.detect_start_time) >= STALL_DETECT_TIME) {
            // 确认堵转,进入反转阶段
            stall_handler.state = STALL_REVERSING;
            stall_handler.reverse_start_time = current_time;
            stall_handler.reverse_target_angle = loader->measure.total_angle - REVERSE_ANGLE;
            stall_handler.reverse_count++;

            // [新增] 更新全局调试变量的反转信息
            stall_debug.reverse_count = stall_handler.reverse_count;
            stall_debug.reverse_target_angle = stall_handler.reverse_target_angle;

            // 设定反转目标角度 (在 ShootTask 的 switch 之前生效)
            DJIMotorOuterLoop(loader, ANGLE_LOOP);
            DJIMotorSetRef(loader, stall_handler.reverse_target_angle);
        }
        RETURN_WITH_DEBUG(current_mode); // 消抖中暂不修改

    case STALL_REVERSING:
        // 反转阶段: 等待反转完成
        if ((current_time - stall_handler.reverse_start_time) >= REVERSE_TIME) {
            // 反转时间结束,进入恢复期
            stall_handler.state = STALL_RECOVERY;
            stall_handler.recovery_start_time = current_time;
        }
        // 返回 LOAD_STOP 跳过正常的发射逻辑,由状态机控制电机
        RETURN_WITH_DEBUG(LOAD_STOP);

    case STALL_RECOVERY:
        // 恢复阶段: 等待稳定后恢复发射
        if ((current_time - stall_handler.recovery_start_time) >= RECOVERY_TIME) {
            // 恢复期结束
            if (stall_handler.reverse_count >= MAX_REVERSE_COUNT) {
                // 连续反转达到上限,停止发射,需要人工干预
                stall_handler.state = STALL_NORMAL;
                stall_handler.reverse_count = 0;
                RETURN_WITH_DEBUG(LOAD_STOP); // 返回停止,等待新指令
            }
            // 恢复正常状态,继续之前的发射模式
            stall_handler.state = STALL_NORMAL;
            RETURN_WITH_DEBUG(stall_handler.saved_mode);
        }
        RETURN_WITH_DEBUG(LOAD_STOP); // 恢复期也返回 STOP

    default:
        stall_handler.state = STALL_NORMAL;
        RETURN_WITH_DEBUG(current_mode);
    }

// 取消宏定义, 避免污染全局命名空间
#undef RETURN_WITH_DEBUG
}

/**
 * @brief 单发处理逻辑 (基于摩擦轮入口限位 + 速度环 + 反向制动)
 * @param trigger_active 是否触发 (边沿信号)
 */
static void HandleSingleFire(uint8_t trigger_active)
{
    float current_time = DWT_GetTimeline_ms();

    // [新增] 更新全局调试变量
    sf_debug.state = single_fire.state;
    sf_debug.baseline_speed = single_fire.baseline_speed;
    sf_debug.current_speed = GetInnerFrictionAvgSpeed();
    // sf_debug.speed_diff = single_fire.baseline_speed - sf_debug.current_speed; // 放在下面计算
    sf_debug.loader_speed = loader->measure.speed_aps;
    sf_debug.is_dipping = IsFrictionDipping(); // 使用 single_fire.baseline_speed
    sf_debug.speed_diff = single_fire.baseline_speed - sf_debug.current_speed;
    sf_debug.trigger_edge = trigger_active;
    sf_debug.fire_count = single_fire.fire_count;
    sf_debug.feed_timeout_count = single_fire.feed_timeout_count;
    sf_debug.brake_start_time = single_fire.brake_start_time;
    sf_debug.feed_start_time = single_fire.feed_start_time;

    switch (single_fire.state) {
    case SF_IDLE:
        if (trigger_active) {
            // 触发: 记录基准速度,进入送弹
            single_fire.baseline_speed = GetInnerFrictionAvgSpeed();
            single_fire.feed_start_time = current_time;
            single_fire.state = SF_FEEDING;

            // 速度环 - 中速送弹
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, SF_FEED_SPEED);
        } else {
            // 确保停止
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, 0);
        }
        break;

    case SF_FEEDING:
        // 监测掉速
        if (IsFrictionDipping()) {
            // 掉速 -> 立即反向制动
            single_fire.state = SF_BRAKING;
            single_fire.brake_start_time = current_time;

            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, SF_BRAKE_SPEED); // 反向
        } else if ((current_time - single_fire.feed_start_time) > SF_FEED_TIMEOUT) {
            // 超时 (缺弹或卡住) -> 回到空闲
            single_fire.state = SF_IDLE;
            single_fire.feed_timeout_count++;

            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, 0);
        } else {
            // 保持送弹
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, SF_FEED_SPEED);
        }
        break;

    case SF_BRAKING:
        if ((current_time - single_fire.brake_start_time) > SF_BRAKE_TIME) {
            // 制动结束 -> 进入冷却
            single_fire.state = SF_COOLDOWN;
            single_fire.cooldown_start_time = current_time;

            // 停止电机
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, 0);

            // 计数
            single_fire.fire_count++;
        } else {
            // 保持制动
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, SF_BRAKE_SPEED);
        }
        break;

    case SF_COOLDOWN:
        // 冷却时间 80ms (参考原 SF_COOLDOWN_TIME 建议，这里直接用宏或硬编码)
        if (current_time - single_fire.cooldown_start_time > 80) {
            single_fire.state = SF_IDLE;
        }
        // 保持停止
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, 0);
        break;
    }
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
        current_inner_deg = 0.0f;
        current_outer_deg = 0.0f;
        DJIMotorStop(friction_inner_left);
        DJIMotorStop(friction_inner_right);
        DJIMotorStop(friction_outer_left);
        DJIMotorStop(friction_outer_right);
        DJIMotorStop(friction_inner_down);
        DJIMotorStop(friction_outer_down);
        DJIMotorStop(loader);
    } else // 恢复运行
    {
        DJIMotorEnable(friction_inner_left);
        DJIMotorEnable(friction_inner_right);
        DJIMotorEnable(friction_outer_left);
        DJIMotorEnable(friction_outer_right);
        DJIMotorEnable(friction_inner_down);
        DJIMotorEnable(friction_outer_down);
        DJIMotorEnable(loader);
    }

    // [新增] 堵转检测与自动反转处理
    // 该函数会在堵转时自动替换发射模式为反转/停止状态
    loader_mode_e actual_load_mode = HandleLoaderStall(shoot_cmd_recv.load_mode);

    // 若不在休眠状态,根据实际发射模式进行拨盘电机参考值设定和模式切换
    switch (actual_load_mode) {
    // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0); // 同时设定参考值为0,这样停止的速度最快
        // [新增] 当模式切换到STOP时, 复位触发状态, 允许下一次触发
        fire_trigger.trigger_consumed = 0;
        fire_trigger.pending_fire = 0;
        // [新增] 复位单发状态机
        single_fire.state = SF_IDLE;
        break;
    // 单发模式
    case LOAD_1_BULLET:
        // [新增] 边沿触发检测
        {
            uint8_t measure_trigger = 0;
            // 只有当上一次是LOAD_STOP时才认为是新的触发
            if (fire_trigger.last_mode != LOAD_1_BULLET && !fire_trigger.trigger_consumed) {
                measure_trigger = 1;
                fire_trigger.trigger_consumed = 1; // 标记消费
            }
            // 调用单发逻辑处理 (包含速度环控制和制动)
            HandleSingleFire(measure_trigger);
        }
        break;
    // 三连发
    case LOAD_3_BULLET:
        if (hibernate_time + dead_time > DWT_GetTimeline_ms())
            break;
        DJIMotorOuterLoop(loader, ANGLE_LOOP);
        DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE);
        hibernate_time = DWT_GetTimeline_ms();
        dead_time = 300;
        break;
    // 二连发模式 (英雄专用)
    case LOAD_2_BULLET:
        if (hibernate_time + dead_time > DWT_GetTimeline_ms())
            break;
        DJIMotorOuterLoop(loader, ANGLE_LOOP);
        DJIMotorSetRef(loader, loader->measure.total_angle + 2 * ONE_BULLET_DELTA_ANGLE);
        hibernate_time = DWT_GetTimeline_ms();
        dead_time = 200;
        break;
    // 连发模式
    case LOAD_BURSTFIRE:
        // 在连发模式下复位单发标志位，确保切回单发时可立即触发一次
        fire_trigger.trigger_consumed = 0;
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
        // DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorOuterLoop(loader, ANGLE_LOOP); // 切换到角度环
        DJIMotorSetRef(loader, loader->measure.total_angle - ONE_BULLET_DELTA_ANGLE / 2.0f); // 控制量减少一发弹丸的一半角度
        hibernate_time = DWT_GetTimeline_ms(); // 记录触发指令的时间
        dead_time = 150; // 完成1发弹丸发射的时间
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }

    // [新增] 更新上一次的发射模式, 用于下一次边沿检测
    fire_trigger.last_mode = actual_load_mode;

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON) {
        // 示例：根据不同的模式设置不同的分级速度
        switch (shoot_cmd_recv.bullet_speed) {
        case BIG_AMU_12:
            // 目标12m/s：一级给11.5，二级给12.0
            ShootSetSpeedDual(11.5f, 12.0f);
            break;
        case BIG_AMU_16:
            // 目标16.5m/s：一级给16.0，二级给16.5
            ShootSetSpeedDual(16.0f, 16.5f);
            break;
        default:
            // 调试默认值 (可以根据你的串口调试实时修改这两个值)
            ShootSetSpeedDual(15.5f, 15.8f);
            break;
        }
    } else {
        // 关闭摩擦轮
        ShootSetSpeedDual(0.0f, 0.0f);
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE) {
        //...
    } else if (shoot_cmd_recv.lid_mode == LID_OPEN) {
        //...
    }
    // 更新调试信息
    UpdateDebugSpeed();

    // [新增] 更新反馈数据, 供外部模块订阅
    shoot_feedback_data.fire_count = single_fire.fire_count;
    shoot_feedback_data.bullet_fired_flag = (single_fire.state == SF_COOLDOWN); // 简单映射
    shoot_feedback_data.empty_flag = (single_fire.feed_timeout_count > 0);

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
