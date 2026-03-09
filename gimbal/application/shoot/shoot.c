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
    return output_angle_deg * REDUCTION_RATIO_LOADER;
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
    single_fire.final_target_angle = 0.0f;
    single_fire.dip_confirmed = 0;
    fire_trigger.pending_fire = 0;
    SetFrictionFeedforward(0.0f, 0.0f);
}

/**
 * @brief 进入单发有限重试等待态
 * @param current_time 当前系统时间 (ms)
 * @note 这里先锁住当前位置再等待下一次补步，作用是让重试节拍由时间控制而不是由电机惯性决定；
 *       原因是用户要求“按一定频率位置环转动”，若不先锁住当前位置，补发间隔会随机械阻力漂移。
 */
static void EnterSingleFireRetryWait(float current_time)
{
    single_fire.state = SF_RETRYING;
    // 在固定步距结束后锁住固定终点，作用是让等待补发期间的位置语义保持一致；
    // 原因是当前需求要求单发优先按固定机械节距停盘，而不是锁在掉速发生瞬间的随机位置。
    single_fire.lock_target_angle = single_fire.final_target_angle;
    single_fire.retry_start_time = current_time;
    SetFrictionFeedforward(0.0f, 0.0f);
    LoaderSetAngleRef(single_fire.lock_target_angle);
}

/**
 * @brief 结束本次单发事务并锁住当前位置
 * @param current_time 当前系统时间 (ms)
 * @param shot_success 1 表示确认出弹, 0 表示未确认出弹
 * @note 这里统一收口成功/失败两条分支，作用是确保锁角、前馈清零、计数更新始终一致；
 *       原因是单发现在包含初次冲刺和有限补发，多处手写收尾容易出现某个分支漏清状态。
 */
static void FinishSingleFire(float current_time, uint8_t shot_success)
{
    single_fire.state = SF_LOCKING;
    single_fire.brake_start_time = current_time;
    // 到达固定终点附近时优先锁到固定终点，作用是让单发收口位置更一致；
    // 原因是本次改造要弱化掉速对停盘点的影响，但超时/异常时仍需允许锁住当前位置以避免继续顶弹。
    single_fire.lock_target_angle = loader->measure.total_angle;
    if (fabsf(single_fire.final_target_angle - loader->measure.total_angle) < SF_FINAL_TARGET_TOLERANCE) {
        single_fire.lock_target_angle = single_fire.final_target_angle;
    }
    SetFrictionFeedforward(0.0f, 0.0f);

    if (shot_success) {
        single_fire.fire_count++;
    } else {
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
 * @brief 启动一轮“固定终点为主”的单发送弹
 * @param current_time 当前系统时间 (ms)
 * @param inner_speed 当前内圈平均速度 (deg/s)
 * @param outer_speed 当前外圈平均速度 (deg/s)
 * @note 这里统一初始化固定终点、冲刺远目标和掉速基线，作用是让首发与补发共用同一套位置语义；
 *       原因是单发与重试都必须按同一固定步距推进，否则停盘点会再次被掉速时刻间接主导。
 */
static void StartSingleFireFeed(float current_time, float inner_speed, float outer_speed)
{
    float single_step_angle = LoaderBulletCountToMotorAngle(SF_SINGLE_TARGET_BULLET_COUNT);
    float rush_step_angle = LoaderBulletCountToMotorAngle(SF_RUSH_BULLET_COUNT);
    float rush_command_angle = (rush_step_angle > single_step_angle) ? rush_step_angle : single_step_angle;

    single_fire.state = SF_FEEDING;
    single_fire.feed_start_time = current_time;
    single_fire.rush_start_angle = loader->measure.total_angle;
    single_fire.final_target_angle = single_fire.rush_start_angle + single_step_angle;
    single_fire.rush_target_angle = single_fire.rush_start_angle + rush_command_angle;
    single_fire.lock_target_angle = single_fire.rush_start_angle;
    single_fire.baseline_speed = inner_speed;
    single_fire.outer_baseline_speed = outer_speed;
    single_fire.dip_confirmed = 0;

    // 每轮送弹前重录基线并清掉上轮抓拍状态，作用是让掉速确认只对当前步距负责；
    // 原因是单发现在允许多轮固定节拍补发，若沿用旧快照会把上一轮噪声带进这一轮确认。
    RecordDipBaseline();
    SetFrictionFeedforward(0.0f, 0.0f);
    LoaderSetAngleRef(single_fire.rush_target_angle);
}

/**
 * @brief 获取当前单发应下发的位置目标
 * @return 本周期拨盘位置环目标角度 (deg)
 * @note 接近固定终点时切回终点目标，作用是让拨盘先猛推再稳收口；
 *       原因是位置环 Kp 较大，若整轮都追远目标，即使已确认出弹也容易把惯性继续送到下一发。
 */
static float GetSingleFireCommandAngle(void)
{
    float remaining_angle = single_fire.final_target_angle - loader->measure.total_angle;
    float final_approach_angle = LoaderBulletCountToMotorAngle(SF_FINAL_APPROACH_BULLET_COUNT);

    if (remaining_angle <= final_approach_angle) {
        return single_fire.final_target_angle;
    }

    return single_fire.rush_target_angle;
}

/**
 * @brief 判断本轮单发是否已经到达固定终点
 * @return 1 表示已到达固定终点附近, 0 表示仍需继续推进
 * @note 用独立终点容差统一判断一轮固定步距是否完成，作用是把“到位”与“掉速确认”解耦；
 *       原因是本次改造要求单发先按机械步距走完，再根据掉速确认决定成功还是继续补发。
 */
static uint8_t IsSingleFireFinalTargetReached(void)
{
    return fabsf(single_fire.final_target_angle - loader->measure.total_angle) < SF_FINAL_TARGET_TOLERANCE;
}

/**
 * @brief 尝试把原始掉速升级为“有效出弹确认”
 * @return 1 表示已确认有效出弹, 0 表示尚未确认
 * @note 这里复用现有抓拍和校验接口，作用是让掉速只负责确认出弹而不直接控制停盘；
 *       原因是简单阈值掉速容易受噪声影响，改造成辅助信号后必须先过一次有效性校验。
 */
static uint8_t TryConfirmSingleFireDip(void)
{
    if (single_fire.dip_confirmed) {
        return 1;
    }

    if (!IsFrictionDipping()) {
        return 0;
    }

    // 同一轮送弹只抓拍并校验一次，作用是避免持续掉速时重复写入同一帧历史记录；
    // 原因是单发现在不会在首个掉速点立刻结束，若每个周期都重复校验会把一次事件误记成多次观测。
    if (!ShootDebug_IsSnapshotTaken()) {
        TakeDipSnapshot();
        ValidateAndSaveDipSnapshot();
    }

    if ((p_dip_snapshot != NULL) && p_dip_snapshot->is_valid_shot) {
        single_fire.dip_confirmed = 1;
    }

    return single_fire.dip_confirmed;
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
 * @brief 单发处理逻辑 (基于摩擦轮掉速的“位置环冲刺 + 掉速锁角”)
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
            // 原因是位置环冲刺起步非常猛，若摩擦轮还没回速，会把进弹误差直接放大成多发风险。
            single_fire.state = SF_WAIT_SPEED;
            single_fire.retry_count = 0;
            single_fire.shot_start_time = current_time;
            single_fire.feed_start_time = current_time;
            single_fire.retry_start_time = current_time;
            single_fire.lock_target_angle = loader->measure.total_angle;
            single_fire.final_target_angle = loader->measure.total_angle;
            single_fire.dip_confirmed = 0;
            LoaderSetAngleRef(single_fire.lock_target_angle);
        } else {
            LoaderSetSpeedRef(0.0f);
        }
        break;

    case SF_WAIT_SPEED:
        if (ShootIsSpeedReady() || (current_time - single_fire.feed_start_time > 500.0f)) {
            // 待速结束后统一启动固定步距送弹，作用是让首发和补发都共享同一套终点/冲刺目标生成逻辑；
            // 原因是用户要求单发按固定位置转动，不能让不同分支各自生成目标后再慢慢漂出两套语义。
            StartSingleFireFeed(current_time, inner_speed, outer_speed);
        } else {
            // 待速期间保持当前位置锁定，作用是防止供弹盘在等待时被反扭矩拖走；
            // 原因是下一次冲刺目标是相对当前位置叠加的，一旦等待期滑动，整个单发节距就会漂移。
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

        // 原始掉速仅用于确认“是否真的出弹”，作用是不再让单发停盘点由掉速瞬间直接决定；
        // 原因是本次需求要把拨盘收口位置固定到机械步距附近，同时保留掉速对成功计数的判据价值。
        TryConfirmSingleFireDip();

        if ((current_time - single_fire.shot_start_time) > SF_FEED_TIMEOUT) {
            // 整次事务超时后直接锁死并记失败，作用是保证有限重试有总时长上限；
            // 原因是空仓或掉速传感异常时，即使每次补发都有限，也不能让单发状态机一直占着控制权。
            FinishSingleFire(current_time, 0);
        } else if (IsSingleFireFinalTargetReached()) {
            // 到达固定终点后再根据掉速确认决定成功/补发，作用是把“停盘位置”和“出弹确认”彻底解耦；
            // 原因是单发现在要优先按固定机械步距完成，再决定这一步是否已经成功把弹送出。
            if (single_fire.dip_confirmed) {
                FinishSingleFire(current_time, 1);
            } else if (single_fire.retry_count < SF_RETRY_MAX_COUNT) {
                EnterSingleFireRetryWait(current_time);
            } else {
                FinishSingleFire(current_time, 0);
            }
        } else {
            LoaderSetAngleRef(GetSingleFireCommandAngle());
        }
        break;

    case SF_RETRYING:
        if (TryConfirmSingleFireDip()) {
            // 等待补发期间若才确认出弹，也按成功收口，作用是兼容掉速信号相对机械动作略滞后的情况；
            // 原因是此时拨盘已经锁在固定终点附近，迟到确认不应再触发额外推进。
            FinishSingleFire(current_time, 1);
        } else if ((current_time - single_fire.shot_start_time) > SF_FEED_TIMEOUT) {
            FinishSingleFire(current_time, 0);
        } else if ((current_time - single_fire.retry_start_time) >= SF_RETRY_INTERVAL_MS) {
            // 到达补发节拍后再走一轮相同固定步距，作用是把“频率”落实成固定时间间隔的离散位置步进；
            // 原因是用户要求没出弹时继续按固定位置补发，不能再回到 1.5 发大步进的旧语义。
            single_fire.retry_count++;
            StartSingleFireFeed(current_time, inner_speed, outer_speed);
        } else {
            LoaderSetAngleRef(single_fire.lock_target_angle);
        }
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
            single_fire.lock_target_angle = loader->measure.total_angle;
            single_fire.final_target_angle = loader->measure.total_angle;
            single_fire.dip_confirmed = 0;
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
    p_sf->final_target_angle = single_fire.final_target_angle;
    p_sf->is_dipping = IsFrictionDipping();
    p_sf->dip_confirmed = single_fire.dip_confirmed;
    p_sf->speed_diff = single_fire.baseline_speed - p_sf->current_speed;
    p_sf->trigger_edge = trigger_active;
    p_sf->retry_count = single_fire.retry_count;
    p_sf->fire_count = single_fire.fire_count;
    p_sf->feed_timeout_count = single_fire.feed_timeout_count;
    p_sf->brake_start_time = single_fire.brake_start_time;
    p_sf->feed_start_time = single_fire.feed_start_time;
    p_sf->retry_start_time = single_fire.retry_start_time;
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
