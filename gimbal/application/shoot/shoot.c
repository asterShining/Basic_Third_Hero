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
// [新增] 单发掉速锁存标志, 作用是把“本次是否观察到有效出弹”与实时掉速瞬态解耦
// 原因是单发现在按固定角度截止, 掉速只负责记账而不再决定何时停机
static uint8_t single_fire_dip_detected = 0;

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
                .Kp = 8.7, // 8.7
                .Ki = 0.5, // 0.5
                .Kd = 0,
                .DeadBand = 10.0f, // [新增] 死区: ±20 deg/s 以内视为零速, 配合 Iout 清零防止停止时自转
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
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
                .Kp = 8.3, // 8.3
                .Ki = 0.5, // 0.5
                .Kd = 0,
                .DeadBand = 10.0f, // [新增] 死区: ±20 deg/s 以内视为零速, 配合 Iout 清零防止停止时自转
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
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

    // // 第二级，外摩擦轮初始化
    // // 1. 下
    // friction_config_outer.can_init_config.tx_id = 4;
    // friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    // friction_config_outer.controller_param_init_config.current_feedforward_ptr = &ff_outer_down; // 绑定独立前馈
    // friction_outer_down = DJIMotorInit(&friction_config_outer);

    // // 2. 左 (反转)
    // friction_config_outer.can_init_config.tx_id = 5;
    // friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    // friction_config_outer.controller_param_init_config.current_feedforward_ptr = &ff_outer_left; // 绑定独立前馈
    // friction_outer_left = DJIMotorInit(&friction_config_outer);

    // // 3. 右
    // friction_config_outer.can_init_config.tx_id = 6;
    // friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    // friction_config_outer.controller_param_init_config.current_feedforward_ptr = &ff_outer_right; // 绑定独立前馈
    // friction_outer_right = DJIMotorInit(&friction_config_outer);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 这里把位置环 Kp 和输出上限一起抬高，作用是把 2.5 发量级的大角度误差直接转换成高速度给定；
                // 原因是 DJI 串级控制里位置环输出先喂给速度环，若这里不够大，拨盘就无法在起步瞬间打出满力矩冲刺。
                .Kp = 19.0f,
                .Ki = 0.0,
                .Kd = 0.0f,
                .MaxOut = 40000,

            },
            .speed_PID = {
                .Kp = 3.1, // 3.1
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
 * @brief 读取电机实时角速度
 * @param motor 电机实例
 * @return 电机实时角速度，空指针时返回 0
 * @note 这里统一做空指针保护，原因是当前外圈摩擦轮可能被裁剪为未安装配置，直接解引用会把射击任务打崩。
 */
static float GetMotorSpeedAps(const DJIMotorInstance *motor)
{
    if (motor == NULL)
        return 0.0f;

    return motor->measure.speed_aps;
}

/**
 * @brief 判断外圈摩擦轮是否完整存在
 * @return 1 表示外圈三电机都已初始化，0 表示当前平台没有完整外圈
 * @note 这里单独做能力检测，原因是掉速检测和就绪判定需要根据硬件配置自动降级，避免把不存在的外圈当成故障。
 */
static uint8_t HasOuterFrictionWheel(void)
{
    return (friction_outer_left != NULL) &&
           (friction_outer_right != NULL) &&
           (friction_outer_down != NULL);
}

/**
 * @brief 将拨盘输出端角度换算为电机 total_angle 单位
 * @param output_angle_deg 拨盘输出端角度，单位为 deg
 * @return 电机转子总角度，单位为 deg
 * @note total_angle 是电机转子多圈角度而非拨盘输出角度，所以位置环目标必须乘减速比，否则会少转约 19 倍。
 */
static float LoaderOutputAngleToMotorAngle(float output_angle_deg)
{
    return output_angle_deg * REDUCTION_RATIO_LOADER * 2;
}

/**
 * @brief 将“多少发弹丸的机械行程”换算为电机 total_angle 单位
 * @param bullet_count 以“发”为单位的拨盘行程
 * @return 电机转子总角度，单位为 deg
 * @note 用弹丸数量描述位置目标更符合射击机构语义，原因是单发、二连发、反转都天然按“几发”的机械节距定义。
 */
static float LoaderBulletCountToMotorAngle(float bullet_count)
{
    return bullet_count * LOADER_MOTOR_ANGLE_PER_BULLET;
}

/**
 * @brief 给拨盘电机下发速度环目标
 * @param speed_ref 目标角速度，单位为 deg/s
 * @note 统一封装速度环切换和设定值下发，原因是单发/停机/连发都会复用，避免多处切环遗漏。
 */
static void LoaderSetSpeedRef(float speed_ref)
{
    DJIMotorOuterLoop(loader, SPEED_LOOP);
    DJIMotorSetRef(loader, speed_ref);
}

/**
 * @brief 给拨盘电机下发位置环目标
 * @param angle_ref 目标 total_angle，单位为 deg
 * @note 统一封装位置环切换和设定值下发，原因是单发冲刺、掉速锁角、二三连发都必须严格使用同一角度语义。
 */
static void LoaderSetAngleRef(float angle_ref)
{
    DJIMotorOuterLoop(loader, ANGLE_LOOP);
    DJIMotorSetRef(loader, angle_ref);
}

/**
 * @brief 给拨盘电机下发开环电流目标
 * @param current_ref 目标电流指令，单位为 raw
 * @note 这里统一封装恒电流起步入口，作用是让单发起步力矩固定；
 *       原因是位置环大误差起步会让每次咬弹时机随阻力波动，破坏拨盘相位一致性。
 */
static void LoaderSetCurrentRef(float current_ref)
{
    if (current_ref > 16000.0f)
        current_ref = 16000.0f;
    else if (current_ref < -16000.0f)
        current_ref = -16000.0f;

    DJIMotorOuterLoop(loader, OPEN_LOOP);
    DJIMotorSetRef(loader, current_ref);
}

/**
 * @brief 统一设置摩擦轮前馈
 * @param inner_ff 内圈前馈电流
 * @param outer_ff 外圈前馈电流
 * @note 这里把 6 个前馈量集中写入，原因是单发状态机有多处清零/注入动作，手写六次很容易漏改。
 */
static void SetFrictionFeedforward(float inner_ff, float outer_ff)
{
    ff_inner_left = inner_ff;
    ff_inner_right = inner_ff;
    ff_inner_down = inner_ff;
    ff_outer_left = outer_ff;
    ff_outer_right = outer_ff;
    ff_outer_down = outer_ff;
}

/**
 * @brief 安全地下发摩擦轮参考值
 * @param motor 电机实例
 * @param ref 目标角速度
 * @note 这里对可选电机做空指针保护，原因是英雄当前代码允许裁剪外圈，空指针直接写 ref 会触发 HardFault。
 */
static void SetFrictionRefIfReady(DJIMotorInstance *motor, float ref)
{
    if (motor != NULL)
        DJIMotorSetRef(motor, ref);
}

/**
 * @brief 安全地启停电机
 * @param motor 电机实例
 * @param enable 1 使能，0 停止
 * @note 统一封装空指针保护，原因是 ShootTask 会批量启停所有摩擦轮，当前硬件配置可能并不具备全部 6 个电机。
 */
static void SetMotorEnableIfReady(DJIMotorInstance *motor, uint8_t enable)
{
    if (motor == NULL)
        return;

    if (enable)
        DJIMotorEnable(motor);
    else
        DJIMotorStop(motor);
}

/**
 * @brief 中止单发状态机
 * @note 这里同时清空前馈和挂起触发，原因是切换到二连发/连发/反转时必须把单发遗留状态完全收口，避免旧状态抢控制权。
 */
static void AbortSingleFire(void)
{
    single_fire.state = SF_IDLE;
    single_fire.retry_count = 0;
    single_fire.shot_start_time = 0.0f;
    single_fire.feed_start_time = 0.0f;
    single_fire.retry_start_time = 0.0f;
    single_fire_dip_detected = 0;
    fire_trigger.pending_fire = 0;
    SetFrictionFeedforward(0.0f, 0.0f);
}

/**
 * @brief 结束本次单发事务并锁住固定相位目标
 * @param current_time 当前系统时间 (ms)
 * @param shot_success 1 表示确认出弹, 0 表示未确认出弹
 * @param lock_target_angle 需要锁住的目标 total_angle
 * @note 这里统一收口成功/失败两条分支，作用是确保每次都锁回同一个截止相位；
 *       原因是若按当前位置收口，掉速检测时刻的抖动会直接变成下一发的相位误差。
 */
static void FinishSingleFire(float current_time, uint8_t shot_success, float lock_target_angle)
{
    single_fire.state = SF_LOCKING;
    single_fire.brake_start_time = current_time;
    single_fire.lock_target_angle = lock_target_angle;
    SetFrictionFeedforward(0.0f, 0.0f);

    if (shot_success) {
        single_fire.fire_count++;
    } else {
        // 这里继续沿用 feed_timeout_count 记未确认出弹，作用是复用现有反馈通路；
        // 原因是本次改动聚焦单发相位一致性，不额外扩展上层消息结构。
        single_fire.feed_timeout_count++;
    }

    LoaderSetAngleRef(single_fire.lock_target_angle);
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
    SetFrictionRefIfReady(friction_inner_left, current_inner_deg);
    SetFrictionRefIfReady(friction_inner_right, current_inner_deg);
    SetFrictionRefIfReady(friction_inner_down, current_inner_deg);

    // 3. 设置第二级（外圈3个电机）- 负责稳速/微加速
    SetFrictionRefIfReady(friction_outer_left, current_outer_deg);
    SetFrictionRefIfReady(friction_outer_right, current_outer_deg);
    SetFrictionRefIfReady(friction_outer_down, current_outer_deg);
}
/**
 * @brief 更新调试数据 (将电机反馈的角速度转换为线速度)
 * @note 调用调试模块的接口函数
 */
static void UpdateFrictionDebugInfo(void)
{
    // 调用调试模块接口，传入6个电机的实时速度
    ShootDebug_UpdateFrictionInfo(
        GetMotorSpeedAps(friction_inner_left),
        GetMotorSpeedAps(friction_inner_right),
        GetMotorSpeedAps(friction_inner_down),
        GetMotorSpeedAps(friction_outer_left),
        GetMotorSpeedAps(friction_outer_right),
        GetMotorSpeedAps(friction_outer_down));
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
    float avg = (fabsf(GetMotorSpeedAps(friction_inner_left)) +
                 fabsf(GetMotorSpeedAps(friction_inner_right)) +
                 fabsf(GetMotorSpeedAps(friction_inner_down))) *
                0.3333333f;
    return avg;
}

/**
 * @brief 获取外圈摩擦轮平均速度 (deg/s)
 * @return 三个外圈摩擦轮速度的平均绝对值
 */
static float GetOuterFrictionAvgSpeed(void)
{
    // 外圈缺失时退化为内圈平均速度，作用是让掉速检测与发射等待仍可工作；
    // 原因是当前工程允许只装内圈摩擦轮，强依赖外圈会让单发状态机永远等不到有效检测。
    if (!HasOuterFrictionWheel())
        return GetInnerFrictionAvgSpeed();

    // [优化] 使用乘法代替除法
    float avg = (fabsf(GetMotorSpeedAps(friction_outer_left)) +
                 fabsf(GetMotorSpeedAps(friction_outer_right)) +
                 fabsf(GetMotorSpeedAps(friction_outer_down))) *
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
        GetMotorSpeedAps(friction_inner_left),
        GetMotorSpeedAps(friction_inner_right),
        GetMotorSpeedAps(friction_inner_down),
        GetMotorSpeedAps(friction_outer_left),
        GetMotorSpeedAps(friction_outer_right),
        GetMotorSpeedAps(friction_outer_down));
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
        GetMotorSpeedAps(friction_inner_left),
        GetMotorSpeedAps(friction_inner_right),
        GetMotorSpeedAps(friction_inner_down),
        GetMotorSpeedAps(friction_outer_left),
        GetMotorSpeedAps(friction_outer_right),
        GetMotorSpeedAps(friction_outer_down),
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
            // 反转目标统一换算到电机 total_angle 单位，作用是让位置环反转量与机械半发节距一致；
            // 原因是 total_angle 统计的是转子角度，不乘减速比时反转角度会明显不足。
            stall_handler.reverse_target_angle = loader->measure.total_angle -
                                                 LoaderOutputAngleToMotorAngle(REVERSE_ANGLE);
            stall_handler.reverse_count++;

            // [重构] 更新调试模块的反转信息 (通过接口获取指针)
            StallDebug_s *p_stall = ShootDebug_GetStallPtr();
            p_stall->reverse_count = stall_handler.reverse_count;
            p_stall->reverse_target_angle = stall_handler.reverse_target_angle;

            // 设定反转目标角度 (在 ShootTask 的 switch 之前生效)
            LoaderSetAngleRef(stall_handler.reverse_target_angle);
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
    float target_inner_abs = fabsf(current_inner_deg);
    float target_outer_abs = fabsf(current_outer_deg);

    // 如果目标速度为0, 直接认为就绪 (避免无法停止)
    if (target_inner_abs == 0.0f && target_outer_abs == 0.0f)
        return 1;

    // 用绝对值比较转速是否达标，作用是兼容反装电机；
    // 原因是左摩擦轮在反转安装时反馈速度为负，直接和正目标做差会导致就绪判定永远失败。
    if (fabsf(fabsf(GetMotorSpeedAps(friction_inner_left)) - target_inner_abs) > SHOOT_SPEED_READY_THRESHOLD)
        return 0;
    if (fabsf(fabsf(GetMotorSpeedAps(friction_inner_right)) - target_inner_abs) > SHOOT_SPEED_READY_THRESHOLD)
        return 0;
    if (fabsf(fabsf(GetMotorSpeedAps(friction_inner_down)) - target_inner_abs) > SHOOT_SPEED_READY_THRESHOLD)
        return 0;

    // 外圈存在时再参与判定，作用是让单发等待逻辑适配不同级数的摩擦轮；
    // 原因是无外圈平台不应被一个不存在的速度反馈永远卡住。
    if (HasOuterFrictionWheel()) {
        if (fabsf(fabsf(GetMotorSpeedAps(friction_outer_left)) - target_outer_abs) > SHOOT_SPEED_READY_THRESHOLD)
            return 0;
        if (fabsf(fabsf(GetMotorSpeedAps(friction_outer_right)) - target_outer_abs) > SHOOT_SPEED_READY_THRESHOLD)
            return 0;
        if (fabsf(fabsf(GetMotorSpeedAps(friction_outer_down)) - target_outer_abs) > SHOOT_SPEED_READY_THRESHOLD)
            return 0;
    }

    return 1;
}

/**
 * @brief 单发处理逻辑 (基于“恒电流起步 + 固定角度截止”的固定相位单发)
 * @param trigger_active 是否触发 (边沿信号)
 * @note 单发一旦触发就会自保持执行到锁角完成，原因是遥控拨杆通常是瞬时动作，不能要求操作者一直压住指令。
 */
static void HandleSingleFire(uint8_t trigger_active)
{
    float current_time = DWT_GetTimeline_ms();
    SingleFireDebug_s *p_sf = ShootDebug_GetSingleFirePtr();
    float inner_speed = GetInnerFrictionAvgSpeed();
    float outer_speed = GetOuterFrictionAvgSpeed();

    // 触发边沿在真正进入单发状态机时再消费，作用是防止堵转恢复期间误吞一次遥控触发。
    if (trigger_active) {
        fire_trigger.pending_fire = 0;
    }

    switch (single_fire.state) {
    case SF_IDLE:
        if (trigger_active) {
            // 先进入待速阶段并锁住当前位置，作用是等摩擦轮恢复到稳态再发；
            // 原因是固定相位单发仍然怕摩擦轮未就绪时提前咬弹，导致出弹时刻和相位一起漂移。
            single_fire.state = SF_WAIT_SPEED;
            single_fire.retry_count = 0;
            single_fire.shot_start_time = current_time;
            single_fire.feed_start_time = current_time;
            single_fire.retry_start_time = current_time;
            single_fire_dip_detected = 0;
            single_fire.lock_target_angle = loader->measure.total_angle;
            LoaderSetAngleRef(single_fire.lock_target_angle);
        } else {
            LoaderSetSpeedRef(0.0f);
        }
        break;

    case SF_WAIT_SPEED:
        if (ShootIsSpeedReady() || (current_time - single_fire.feed_start_time > 500.0f)) {
            // 进入送弹阶段时预先算出固定截止目标，作用是让单发无论是否漏检掉速都收口到同一相位；
            // 原因是“锁当前位置”会把掉速触发时刻的波动直接映射成下一发的机械初相。
            single_fire.state = SF_FEEDING;
            single_fire.feed_start_time = current_time;
            single_fire.rush_start_angle = loader->measure.total_angle;
            single_fire.rush_target_angle = single_fire.rush_start_angle + SF_RUSH_ANGLE;
            single_fire.lock_target_angle = single_fire.rush_target_angle;
            single_fire.baseline_speed = inner_speed;
            single_fire.outer_baseline_speed = outer_speed;
            single_fire_dip_detected = 0;

            RecordDipBaseline();
            SetFrictionFeedforward(0.0f, 0.0f);
            LoaderSetCurrentRef(SF_STARTUP_CURRENT_REF);
        } else {
            // 待速期间保持当前位置锁定，作用是防止供弹盘在等待时被反扭矩拖走；
            // 原因是固定相位截止依赖一个稳定起点，等待期滑动会让同样的截止角对应不同弹丸相位。
            LoaderSetAngleRef(single_fire.lock_target_angle);
        }
        break;

    case SF_FEEDING:
        if ((current_time - single_fire.feed_start_time) < FRICTION_FEEDFORWARD_TIME) {
            // 在冲刺初段给摩擦轮额外前馈，作用是让咬弹瞬间的速度塌陷更可控；
            // 原因是拨盘改成位置环后起步扭矩更猛，摩擦轮若不提前补能量，掉速幅值会放大且恢复更慢。
            SetFrictionFeedforward(FRICTION_FEEDFORWARD_CURRENT, 0.0f);
        } else {
            SetFrictionFeedforward(0.0f, 0.0f);
        }

        // 基准线随峰值更新，作用是把前馈带来的正常提速吸收进去；
        // 原因是掉速检测依赖“基准 - 当前”，若基准不跟峰值走就会把正常加速误判成未掉速。
        if (inner_speed > single_fire.baseline_speed) {
            single_fire.baseline_speed = inner_speed;
        }
        if (outer_speed > single_fire.outer_baseline_speed) {
            single_fire.outer_baseline_speed = outer_speed;
        }

        ShootDebug_UpdatePeakBaseline(
            GetMotorSpeedAps(friction_inner_left),
            GetMotorSpeedAps(friction_inner_right),
            GetMotorSpeedAps(friction_inner_down),
            GetMotorSpeedAps(friction_outer_left),
            GetMotorSpeedAps(friction_outer_right),
            GetMotorSpeedAps(friction_outer_down));

        if (!single_fire_dip_detected && IsFrictionDipping()) {
            // 这里仅锁存“已观察到有效掉速”，作用是把出弹确认和停机条件解耦；
            // 原因是单发现在由固定截止角收口，不能再为了等掉速把拨盘继续往前推。
            TakeDipSnapshot();
            ValidateAndSaveDipSnapshot();
            single_fire_dip_detected = 1;
        }

        if (loader->measure.total_angle >= single_fire.rush_target_angle) {
            // 到达固定截止角后立刻切回位置环锁定目标，作用是把每一发都收敛到同一机械相位；
            // 原因是单发真正要稳定的是“停在哪”，而不是“何时第一次看到掉速”。
            FinishSingleFire(current_time, single_fire_dip_detected, single_fire.rush_target_angle);
        } else if ((current_time - single_fire.feed_start_time) > SF_STARTUP_TIMEOUT) {
            // 超时仍未到截止角也强制收口，作用是给机械阻滞和编码器异常一个硬上限；
            // 原因是单发不再允许等待掉速或继续补发，否则会重新引入节拍拖延和多发风险。
            FinishSingleFire(current_time, 0, single_fire.rush_target_angle);
        } else {
            LoaderSetCurrentRef(SF_STARTUP_CURRENT_REF);
        }
        break;

    case SF_RETRYING:
        // 这里兼容旧状态枚举值，作用是防止调试器手改状态或残留值导致 switch 漏处理；
        // 原因是主流程已删除有限重试逻辑，但保留枚举可减少调试结构改动范围。
        single_fire.state = SF_LOCKING;
        LoaderSetAngleRef(single_fire.lock_target_angle);
        break;

    case SF_LOCKING:
        if (trigger_active) {
            // 在锁角态收到下一次触发时只重启待速，不先退回速度环，作用是保证两发之间的机械相位连续；
            // 原因是当前位置已经是上一发真实出弹点，直接从这里叠加下一次冲刺最不容易积累角度误差。
            single_fire.state = SF_WAIT_SPEED;
            single_fire.retry_count = 0;
            single_fire.shot_start_time = current_time;
            single_fire.feed_start_time = current_time;
            single_fire.retry_start_time = current_time;
            single_fire_dip_detected = 0;
            single_fire.lock_target_angle = loader->measure.total_angle;
        }
        LoaderSetAngleRef(single_fire.lock_target_angle);
        break;
    }

    // 调试信息在状态机执行后再写回，作用是让调试器看到当前循环的最新状态；
    // 原因是单发状态会在一次调用中切换，先写调试会让观察值总是滞后一拍。
    p_sf->state = single_fire.state;
    p_sf->baseline_speed = single_fire.baseline_speed;
    p_sf->outer_baseline_speed = single_fire.outer_baseline_speed;
    p_sf->current_speed = GetInnerFrictionAvgSpeed();
    p_sf->current_outer_speed = GetOuterFrictionAvgSpeed();
    p_sf->loader_speed = loader->measure.speed_aps;
    p_sf->is_dipping = IsFrictionDipping();
    p_sf->speed_diff = single_fire.baseline_speed - p_sf->current_speed;
    p_sf->trigger_edge = trigger_active;
    p_sf->retry_count = 0;
    p_sf->fire_count = single_fire.fire_count;
    p_sf->feed_timeout_count = single_fire.feed_timeout_count;
    p_sf->brake_start_time = single_fire.brake_start_time;
    p_sf->feed_start_time = single_fire.feed_start_time;
    p_sf->retry_start_time = 0.0f;
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    static uint16_t last_report_fire_count = 0;

    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 先对单发触发做边沿锁存，作用是把遥控器瞬时拨杆转换成一次完整的单发事务；
    // 原因是单发改成位置环冲刺后必须执行到“掉速锁角”收口，不能依赖 LOAD_1_BULLET 电平持续存在。
    if (shoot_cmd_recv.load_mode != LOAD_1_BULLET) {
        fire_trigger.trigger_consumed = 0;
    } else if (fire_trigger.last_mode != LOAD_1_BULLET && !fire_trigger.trigger_consumed) {
        fire_trigger.pending_fire = 1;
        fire_trigger.trigger_consumed = 1;
    }

    loader_mode_e requested_load_mode = shoot_cmd_recv.load_mode;

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
        current_inner_deg = 0.0f;
        current_outer_deg = 0.0f;
        requested_load_mode = LOAD_STOP;
        fire_trigger.pending_fire = 0;
        fire_trigger.trigger_consumed = 0;
        AbortSingleFire();
        SetMotorEnableIfReady(friction_inner_left, 0);
        SetMotorEnableIfReady(friction_inner_right, 0);
        SetMotorEnableIfReady(friction_outer_left, 0);
        SetMotorEnableIfReady(friction_outer_right, 0);
        SetMotorEnableIfReady(friction_inner_down, 0);
        SetMotorEnableIfReady(friction_outer_down, 0);
        SetMotorEnableIfReady(loader, 0);
    } else // 恢复运行
    {
        SetMotorEnableIfReady(friction_inner_left, 1);
        SetMotorEnableIfReady(friction_inner_right, 1);
        SetMotorEnableIfReady(friction_outer_left, 1);
        SetMotorEnableIfReady(friction_outer_right, 1);
        SetMotorEnableIfReady(friction_inner_down, 1);
        SetMotorEnableIfReady(friction_outer_down, 1);
        SetMotorEnableIfReady(loader, 1);
    }

    // 当单发已经触发但遥控器电平回到 STOP 时，仍然把它作为单发事务送进堵转状态机；
    // 原因是位置环冲刺阶段若中途不再参与堵转检测，卡弹时就会失去自动解卡能力。
    loader_mode_e stall_input_mode = requested_load_mode;
    if ((requested_load_mode == LOAD_STOP) &&
        (fire_trigger.pending_fire || single_fire.state == SF_WAIT_SPEED || single_fire.state == SF_FEEDING)) {
        stall_input_mode = LOAD_1_BULLET;
    }

    // 该函数会在堵转时自动替换发射模式为反转/停止状态
    loader_mode_e actual_load_mode = HandleLoaderStall(stall_input_mode);

    // 若不在休眠状态,根据实际发射模式进行拨盘电机参考值设定和模式切换
    switch (actual_load_mode) {
    // 停止拨盘
    case LOAD_STOP:
        if ((single_fire.state != SF_IDLE || fire_trigger.pending_fire) && stall_handler.state == STALL_NORMAL) {
            HandleSingleFire(fire_trigger.pending_fire);
        } else {
            AbortSingleFire();
            LoaderSetSpeedRef(0.0f);
        }
        break;
    // 单发模式
    case LOAD_1_BULLET:
        HandleSingleFire(fire_trigger.pending_fire);
        break;
    // 三连发
    case LOAD_3_BULLET:
        AbortSingleFire();
        if (hibernate_time + dead_time > DWT_GetTimeline_ms())
            break;
        LoaderSetAngleRef(loader->measure.total_angle + LoaderBulletCountToMotorAngle(3.0f));
        hibernate_time = DWT_GetTimeline_ms();
        dead_time = 300;
        break;
    // 二连发模式 (英雄专用)
    case LOAD_2_BULLET:
        AbortSingleFire();
        if (hibernate_time + dead_time > DWT_GetTimeline_ms())
            break;
        LoaderSetAngleRef(loader->measure.total_angle + LoaderBulletCountToMotorAngle(2.0f));
        hibernate_time = DWT_GetTimeline_ms();
        dead_time = 200;
        break;
    // 连发模式
    case LOAD_BURSTFIRE:
        // 在连发模式下复位单发标志位，确保切回单发时可立即触发一次
        AbortSingleFire();
        fire_trigger.trigger_consumed = 0;
        LoaderSetSpeedRef(shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
        AbortSingleFire();
        LoaderSetAngleRef(loader->measure.total_angle - LoaderBulletCountToMotorAngle(0.5f));
        hibernate_time = DWT_GetTimeline_ms(); // 记录触发指令的时间
        dead_time = 150; // 完成1发弹丸发射的时间
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }

    // 这里保存原始遥控指令而不是实际执行模式，作用是让边沿检测只对用户动作敏感；
    // 原因是堵转状态机会临时把模式改成 STOP，若记录 actual_load_mode 会把一次拨杆动作误拆成多次触发。
    fire_trigger.last_mode = requested_load_mode;

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
            ShootSetSpeedDual(11.0f, 12.0f);
            break;
        case BIG_AMU_16:
            // 目标16.5m/s：一级给15.5，二级给16.5
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

    // 用 fire_count 边沿生成 bullet_fired_flag，作用是把“成功发射”变成单周期脉冲；
    // 原因是单发现在会长时间停留在锁角态，不能再用状态枚举直接映射“本周期已发射”。
    shoot_feedback_data.fire_count = single_fire.fire_count;
    shoot_feedback_data.bullet_fired_flag = (single_fire.fire_count != last_report_fire_count);
    last_report_fire_count = single_fire.fire_count;
    shoot_feedback_data.empty_flag = (single_fire.feed_timeout_count > 0);

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
