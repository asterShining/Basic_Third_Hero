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

// [新增] 6个摩擦轮单独的前馈变量
static float ff_inner_left = 0.0f;
static float ff_inner_right = 0.0f;
static float ff_inner_down = 0.0f;
static float ff_outer_left = 0.0f;
static float ff_outer_right = 0.0f;
static float ff_outer_down = 0.0f;

static DJIMotorInstance *loader; // 拨盘电机
static DJIMotorInstance *friction_inner_down, *friction_inner_left, *friction_inner_right; // 内部摩擦轮电机
static DJIMotorInstance *friction_outer_down, *friction_outer_left, *friction_outer_right; // 外部摩擦轮电机
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// [调试] 结构体指针变量，便于调试器直接观测 (通过函数接口获取)
static FrictionWheelDebug_s *p_friction_debug;
static StallDebug_s *p_stall_debug;
static SingleFireDebug_s *p_sf_debug;
static BulletDipSnapshot_s *p_dip_snapshot;

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

void ShootInit()
{
    // 内摩擦轮配置模板
    Motor_Init_Config_s friction_config_inner = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 9.7, // 9.3
                .Ki = 0.9, // 1.3
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 16000,
            },
            // [修改] 下面会针对每个电机单独赋值
            .current_feedforward_ptr = NULL,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .feedforward_flag = CURRENT_FEEDFORWARD, // [新增] 开启电流前馈
        },
        .motor_type = M3508
    };
    // 外摩擦轮配置模板
    Motor_Init_Config_s friction_config_outer = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 9.3, // 8
                .Ki = 0.9, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_feedforward_ptr = NULL,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .feedforward_flag = CURRENT_FEEDFORWARD, // [新增] 开启电流前馈
        },

        .motor_type = M3508
    };

    // 第一级，内摩擦轮初始化
    // 1. 下
    friction_config_inner.can_init_config.tx_id = 1;
    friction_config_inner.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_config_inner.controller_param_init_config.current_feedforward_ptr = &ff_inner_down; // 绑定独立前馈
    friction_inner_down = DJIMotorInit(&friction_config_inner);

    // 2. 左 (反转)
    friction_config_inner.can_init_config.tx_id = 2;
    friction_config_inner.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_config_inner.controller_param_init_config.current_feedforward_ptr = &ff_inner_left; // 绑定独立前馈
    friction_inner_left = DJIMotorInit(&friction_config_inner);

    // 3. 右
    friction_config_inner.can_init_config.tx_id = 3;
    friction_config_inner.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_config_inner.controller_param_init_config.current_feedforward_ptr = &ff_inner_right; // 绑定独立前馈
    friction_inner_right = DJIMotorInit(&friction_config_inner);

    // 第二级，外摩擦轮初始化
    // 1. 下
    friction_config_outer.can_init_config.tx_id = 4;
    friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_config_outer.controller_param_init_config.current_feedforward_ptr = &ff_outer_down; // 绑定独立前馈
    friction_outer_down = DJIMotorInit(&friction_config_outer);

    // 2. 左 (反转)
    friction_config_outer.can_init_config.tx_id = 5;
    friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_config_outer.controller_param_init_config.current_feedforward_ptr = &ff_outer_left; // 绑定独立前馈
    friction_outer_left = DJIMotorInit(&friction_config_outer);

    // 3. 右
    friction_config_outer.can_init_config.tx_id = 6;
    friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_config_outer.controller_param_init_config.current_feedforward_ptr = &ff_outer_right; // 绑定独立前馈
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
                .Kp = 1.3, // 10
                .Ki = 0.0,
                .Kd = 0.01,
                .MaxOut = 9000, // 角度环输出限幅 (deg/s), 提高以允许更大力矩

            },
            .speed_PID = {
                .Kp = 3.5, // 10S
                .Ki = 0.0, // 1
                .Kd = 0.0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 16100,
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

    // [调试] 初始化调试指针 (获取 shoot_debug.c 中的实际结构体地址)
    // 原因: 这些指针必须在初始化时赋值，否则 Ozone 调试器读到的是 NULL
    p_friction_debug = ShootDebug_GetFrictionPtr();
    p_stall_debug = ShootDebug_GetStallPtr();
    p_sf_debug = ShootDebug_GetSingleFirePtr();
    p_dip_snapshot = ShootDebug_GetDipSnapshotPtr();
}
/**
 * @brief 辅助函数：将线速度转换为角速度
 */
/**
 * @brief 辅助函数：将线速度转换为角速度 (m/s -> deg/s)
 */
float SpeedMps2Degs(float speed_mps)
{
    if (speed_mps == 0.0f)
        return 0.0f;

    // 限制最大输入，防止异常值
    if (speed_mps > 20.0f)
        speed_mps = 20.0f;
    if (speed_mps < -20.0f)
        speed_mps = -20.0f;

    // 公式: (线速度 / 半径) * (180/PI) * 补偿系数
    // v = w * r  =>  w = v / r
    return (speed_mps / FRICTION_WHEEL_RADIUS) * RAD_2_DEGREE * SLIP_COMPENSATION;
}

/**
 * @brief 辅助函数：将角速度转换为线速度 (deg/s -> m/s)
 * @note 不包含打滑补偿，反映电机轴的理论线速度
 */
float SpeedAps2Mps(float speed_aps)
{
    // 公式: (角速度 / (180/PI)) * 半径
    // v = w * r
    return (speed_aps / RAD_2_DEGREE) * FRICTION_WHEEL_RADIUS;
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
 * @brief 更新调试数据 (将电机反馈的角速度转换为线速度)
 * @note 调用调试模块的接口函数
 */
static void UpdateFrictionDebugInfo(void)
{
    // 调用调试模块接口，传入6个电机的实时速度
    ShootDebug_UpdateFrictionInfo(
        friction_inner_left ? friction_inner_left->measure.speed_aps : 0,
        friction_inner_right ? friction_inner_right->measure.speed_aps : 0,
        friction_inner_down ? friction_inner_down->measure.speed_aps : 0,
        friction_outer_left ? friction_outer_left->measure.speed_aps : 0,
        friction_outer_right ? friction_outer_right->measure.speed_aps : 0,
        friction_outer_down ? friction_outer_down->measure.speed_aps : 0);
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

    // 计算是否堵转
    uint8_t is_stalled = (current_abs > STALL_CURRENT_THRESHOLD) &&
                         (speed_abs < STALL_SPEED_THRESHOLD);

    // 更新调试模块的堵转信息 (通过接口函数)
    // 注: 此处只更新电流/速度/is_stalled，状态机状态由 HandleLoaderStall 更新
    StallDebug_s *p = ShootDebug_GetStallPtr();
    p->current_abs = current_abs;
    p->speed_abs = speed_abs;
    p->is_stalled = is_stalled;

    return is_stalled;
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
    // [优化] 使用乘法代替除法: / 3.0f -> * 0.3333333f
    float avg = (fabsf(friction_inner_left->measure.speed_aps) +
                 fabsf(friction_inner_right->measure.speed_aps) +
                 fabsf(friction_inner_down->measure.speed_aps)) *
                0.3333333f;
    return avg;
}

/**
 * @brief 获取外圈摩擦轮平均速度 (deg/s)
 * @return 三个外圈摩擦轮速度的平均绝对值
 */
static float GetOuterFrictionAvgSpeed(void)
{
    // [优化] 使用乘法代替除法
    float avg = (fabsf(friction_outer_left->measure.speed_aps) +
                 fabsf(friction_outer_right->measure.speed_aps) +
                 fabsf(friction_outer_down->measure.speed_aps)) *
                0.3333333f;
    return avg;
}

/**
 * @brief 检测内圈摩擦轮是否发生掉速
 * @return 1 表示检测到掉速 (有弹丸通过), 0 表示正常
 * @note 通过与发射前记录的基准速度对比来判断是否掉速
 */
/**
 * @brief 检测摩擦轮是否发生掉速 (双重检测: 内圈 OR 外圈)
 * @return 1 表示检测到掉速, 0 表示正常
 */
static uint8_t IsFrictionDipping(void)
{
    // 仅使用外圈摩擦轮进行掉速检测
    // 原因: 外圈是弹丸最后经过的一级, 检测到掉速即确认弹丸已完全发射出去, 计数更准确
    float current_outer = GetOuterFrictionAvgSpeed();

    // 外圈掉速判断: 基准速度与当前速度之差超过阈值, 认为弹丸正在通过
    uint8_t outer_dip = (single_fire.outer_baseline_speed - current_outer) > FRICTION_SPEED_DIP_THRESHOLD;

    return outer_dip;
}

/**
 * @brief 记录6个电机的基准速度 (在送弹开始时调用)
 * @note 调用调试模块接口
 */
static void RecordDipBaseline(void)
{
    // 调用调试模块接口，传入6个电机的实时速度
    ShootDebug_RecordDipBaseline(
        friction_inner_left->measure.speed_aps,
        friction_inner_right->measure.speed_aps,
        friction_inner_down->measure.speed_aps,
        friction_outer_left->measure.speed_aps,
        friction_outer_right->measure.speed_aps,
        friction_outer_down->measure.speed_aps);
}

/**
 * @brief 执行掉速抓拍 (当检测到掉速时调用)
 * @note 调用调试模块接口
 */
static void TakeDipSnapshot(void)
{
    // 检查是否已抓拍，避免重复
    if (ShootDebug_IsSnapshotTaken())
        return;

    // 调用调试模块接口，传入6个电机的实时速度和发射计数
    ShootDebug_TakeDipSnapshot(
        friction_inner_left->measure.speed_aps,
        friction_inner_right->measure.speed_aps,
        friction_inner_down->measure.speed_aps,
        friction_outer_left->measure.speed_aps,
        friction_outer_right->measure.speed_aps,
        friction_outer_down->measure.speed_aps,
        single_fire.fire_count);
}

/**
 * @brief 验证掉速快照的有效性并保存到历史记录
 * @note 调用调试模块接口
 */
static void ValidateAndSaveDipSnapshot(void)
{
    ShootDebug_ValidateAndSave();
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
    // 在所有发射模式下进行堵转检测 (单发/二连发/三连发/连发)
    // 检测到堵转时自动反转解卡
    uint8_t is_shooting = (current_mode == LOAD_1_BULLET) || // [修复] 添加单发模式，使防堵转逻辑生效
                          (current_mode == LOAD_2_BULLET) ||
                          (current_mode == LOAD_3_BULLET) ||
                          (current_mode == LOAD_BURSTFIRE);

// [重构] 宏定义: 返回前统一更新调试状态 (通过接口获取指针)
#define RETURN_WITH_DEBUG(mode)                                \
    do {                                                       \
        ShootDebug_GetStallPtr()->state = stall_handler.state; \
        return (mode);                                         \
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

            // [重构] 更新调试模块的反转信息 (通过接口获取指针)
            StallDebug_s *p_stall = ShootDebug_GetStallPtr();
            p_stall->reverse_count = stall_handler.reverse_count;
            p_stall->reverse_target_angle = stall_handler.reverse_target_angle;

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
 * @brief 检测摩擦轮转速是否就绪 (均在误差范围内)
 * @return 1=就绪, 0=未就绪
 */
static uint8_t ShootIsSpeedReady(void)
{
    // 如果目标速度为0, 直接认为就绪 (避免无法停止)
    if (current_inner_deg == 0.0f && current_outer_deg == 0.0f)
        return 1;

    // 检查内圈3个电机
    if (fabsf(friction_inner_left->measure.speed_aps - current_inner_deg) > SHOOT_SPEED_READY_THRESHOLD)
        return 0;
    if (fabsf(friction_inner_right->measure.speed_aps - current_inner_deg) > SHOOT_SPEED_READY_THRESHOLD)
        return 0;
    if (fabsf(friction_inner_down->measure.speed_aps - current_inner_deg) > SHOOT_SPEED_READY_THRESHOLD)
        return 0;

    // 检查外圈3个电机
    if (fabsf(friction_outer_left->measure.speed_aps - current_outer_deg) > SHOOT_SPEED_READY_THRESHOLD)
        return 0;
    if (fabsf(friction_outer_right->measure.speed_aps - current_outer_deg) > SHOOT_SPEED_READY_THRESHOLD)
        return 0;
    if (fabsf(friction_outer_down->measure.speed_aps - current_outer_deg) > SHOOT_SPEED_READY_THRESHOLD)
        return 0;

    return 1;
}

/**
 * @brief 单发处理逻辑 (基于摩擦轮入口限位 + 速度环 + 反向制动)
 * @param trigger_active 是否触发 (边沿信号)
 */
static void HandleSingleFire(uint8_t trigger_active)
{
    float current_time = DWT_GetTimeline_ms();

    // [重构] 更新单发调试信息 (通过接口获取指针)
    SingleFireDebug_s *p_sf = ShootDebug_GetSingleFirePtr();
    float inner_speed = GetInnerFrictionAvgSpeed();
    float outer_speed = GetOuterFrictionAvgSpeed();

    p_sf->state = single_fire.state;
    p_sf->baseline_speed = single_fire.baseline_speed;
    p_sf->outer_baseline_speed = single_fire.outer_baseline_speed;
    p_sf->current_speed = inner_speed;
    p_sf->current_outer_speed = outer_speed;
    p_sf->loader_speed = loader->measure.speed_aps;
    p_sf->is_dipping = IsFrictionDipping();
    p_sf->speed_diff = single_fire.baseline_speed - inner_speed;
    p_sf->trigger_edge = trigger_active;
    p_sf->fire_count = single_fire.fire_count;
    p_sf->feed_timeout_count = single_fire.feed_timeout_count;
    p_sf->brake_start_time = single_fire.brake_start_time;
    p_sf->feed_start_time = single_fire.feed_start_time;

    switch (single_fire.state) {
    case SF_IDLE:
        if (trigger_active) {
            // [修改] 触发后先进入等待状态，确保转速稳定
            single_fire.state = SF_WAIT_SPEED;
            // 记录触发时间作为等待起始(可选，用于超时判断，这里暂复用 feed_start_time)
            single_fire.feed_start_time = current_time;
        } else {
            // 确保停止
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, 0);
        }
        break;

    case SF_WAIT_SPEED:
        // [新增] 等待转速就绪
        // 如果转速达标 OR 等待超时(500ms防止死锁) -> 开始推弹
        if (ShootIsSpeedReady() || (current_time - single_fire.feed_start_time > 500)) {
            // 转入正式推弹
            single_fire.state = SF_FEEDING;
            single_fire.feed_start_time = current_time; // 重置推弹开始时间

            // 记录双重基准速度
            single_fire.baseline_speed = GetInnerFrictionAvgSpeed();
            single_fire.outer_baseline_speed = GetOuterFrictionAvgSpeed();

            // [抓拍] 记录6电机基准速度
            RecordDipBaseline();

            // 复位前馈
            ff_inner_left = 0.0f;
            ff_inner_right = 0.0f;
            ff_inner_down = 0.0f;
            ff_outer_left = 0.0f;
            ff_outer_right = 0.0f;
            ff_outer_down = 0.0f;

            // 启动推弹
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, SF_FEED_SPEED);
        } else {
            // 继续等待, 保持停止
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, 0);
        }
        break;

    case SF_FEEDING:
        // [新增] 前馈控制逻辑 (Updated to 6 individual variables)
        // 在送弹初期的短时间内, 注入额外电流
        if ((current_time - single_fire.feed_start_time) < FRICTION_FEEDFORWARD_TIME) {
            // 这里可以针对每个电机单独设置方向 +/-
            // 目前假设底层电机模块处理了 MOTOR_DIRECTION_REVERSE, 所以这里都给正值 (Forward Current)
            ff_inner_left = FRICTION_FEEDFORWARD_CURRENT; // 使用宏定义
            ff_inner_right = FRICTION_FEEDFORWARD_CURRENT; //
            ff_inner_down = FRICTION_FEEDFORWARD_CURRENT; //
            ff_outer_left = 0.0f;
            ff_outer_right = 0.0f;
            ff_outer_down = 0.0f;
        } else {
            ff_inner_left = 0.0f;
            ff_inner_right = 0.0f;
            ff_inner_down = 0.0f;
            ff_outer_left = 0.0f;
            ff_outer_right = 0.0f;
            ff_outer_down = 0.0f;
        }

        // 如果电机因为前馈加速了, 基准线也要跟着涨, 否则检测不到掉速
        { // 增加大括号限制作用域
            float current_inner = GetInnerFrictionAvgSpeed();
            if (current_inner > single_fire.baseline_speed) {
                single_fire.baseline_speed = current_inner;
            }
            float current_outer = GetOuterFrictionAvgSpeed();
            if (current_outer > single_fire.outer_baseline_speed) {
                single_fire.outer_baseline_speed = current_outer;
            }
        }

        // [调试] 同时更新调试模块的基准速度 (Peak Hold), 确保调试数据准确
        ShootDebug_UpdatePeakBaseline(
            friction_inner_left->measure.speed_aps,
            friction_inner_right->measure.speed_aps,
            friction_inner_down->measure.speed_aps,
            friction_outer_left->measure.speed_aps,
            friction_outer_right->measure.speed_aps,
            friction_outer_down->measure.speed_aps);

        // 监测掉速 (IsFrictionDipping 已更新为双重检测)
        if (IsFrictionDipping()) {
            // [抓拍] 检测到掉速, 立即抓拍6电机掉速数据
            TakeDipSnapshot();

            // 掉速 -> 立即反向制动
            single_fire.state = SF_BRAKING;
            single_fire.brake_start_time = current_time;

            // 立即停止所有前馈
            ff_inner_left = 0.0f;
            ff_inner_right = 0.0f;
            ff_inner_down = 0.0f;
            ff_outer_left = 0.0f;
            ff_outer_right = 0.0f;
            ff_outer_down = 0.0f;

            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, SF_BRAKE_SPEED); // 反向
        } else if ((current_time - single_fire.feed_start_time) > SF_FEED_TIMEOUT) {
            // 超时 (缺弹或卡住) -> 回到空闲
            single_fire.state = SF_IDLE;
            single_fire.feed_timeout_count++;

            // 停止所有前馈
            ff_inner_left = 0.0f;
            ff_inner_right = 0.0f;
            ff_inner_down = 0.0f;
            ff_outer_left = 0.0f;
            ff_outer_right = 0.0f;
            ff_outer_down = 0.0f;

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

            // [重构] 验证掉速有效性并保存到历史 (调用合并后的接口)
            ValidateAndSaveDipSnapshot();

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

    // [重构] 摩擦轮控制逻辑 (包含调试覆盖)
    // 优先级: 调试覆盖 > 正常指令
    FrictionWheelDebug_s *p_fric = ShootDebug_GetFrictionPtr();
    if (p_fric->override_enable) {
        // 调试模式: 直接使用 debug 结构体中的目标速度
        ShootSetSpeedDual(p_fric->target_inner_mps, p_fric->target_outer_mps);
    } else if (shoot_cmd_recv.friction_mode == FRICTION_ON) {
        // 正常模式: 根据不同的弹速等级设置不同的分级速度
        switch (shoot_cmd_recv.bullet_speed) {
        case BIG_AMU_12:
            // 目标12m/s：一级给11.5，二级给12.0
            ShootSetSpeedDual(11.5f, 12.0f);
            break;
        case BIG_AMU_16:
            // 目标16.5m/s：一级给16.0，二级给16.8
            ShootSetSpeedDual(15.5f, 16.5f);
            break;
        default:
            // 默认值
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

    // [优化] 降低调试信息更新频率 (Downsampling)
    // 只有每10ms更新一次调试数据, 避免每次循环(1ms)都进行浮点除法运算导致任务超时
    // E:[freeRTOS] MOTOR Task DELAY! dt = 1084 us
    static uint32_t debug_update_count = 0;
    if (debug_update_count++ % 10 == 0) {
        UpdateFrictionDebugInfo();
    }

    // [新增] 更新反馈数据, 供外部模块订阅
    shoot_feedback_data.fire_count = single_fire.fire_count;
    shoot_feedback_data.bullet_fired_flag = (single_fire.state == SF_COOLDOWN); // 简单映射
    shoot_feedback_data.empty_flag = (single_fire.feed_timeout_count > 0);

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
