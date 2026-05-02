#include "shoot_private.h"
#include "vofa_debug.h"

// 当前内圈 ramp 输出需要跨控制周期保存，目的是摩擦轮软启动改成时间型斜坡后，不能每拍都从 0 重新推进。
float current_inner_deg = 0.0f;
// 当前外圈 ramp 输出需要跨控制周期保存，目的是关枪和开枪都要基于上一拍状态平滑推进。
float current_outer_deg = 0.0f;
// 缓存内圈左轮最终目标速度绝对值，目的是ready/recover 判定必须盯住真正的稳态目标，而不是 ramp 中间值。
float target_inner_left_deg_abs = 0.0f;
// 缓存内圈右轮最终目标速度绝对值，目的是方向相反的电机在 ready 判定里仍然只关心速度幅值。
float target_inner_right_deg_abs = 0.0f;
// 缓存内圈下轮最终目标速度绝对值，目的是单发回速判定要覆盖全部已安装摩擦轮。
float target_inner_down_deg_abs = 0.0f;
// 缓存外圈左轮最终目标速度绝对值，目的是外圈作为稳速级也要参与 ready/recover 判定。
float target_outer_left_deg_abs = 0.0f;
// 缓存外圈右轮最终目标速度绝对值，目的是调试和单发恢复逻辑都需要知道真正的目标幅值。
float target_outer_right_deg_abs = 0.0f;
// 缓存外圈下轮最终目标速度绝对值，目的是外圈存在时必须与其余轮子一起进入判定。
float target_outer_down_deg_abs = 0.0f;
// 缓存摩擦轮 ramp 的 DWT 计数器，目的是升速改成时间型斜坡后，每轮都要用真实 dt 计算步进量。
uint32_t friction_ramp_dwt_cnt = 0u;

// 6 个摩擦轮的电流前馈量需要跨函数共享，目的是初始化要把它们绑进电机控制器，而单发状态机会在运行时动态改写。
float ff_inner_left = 0.0f;
float ff_inner_right = 0.0f;
float ff_inner_down = 0.0f;
float ff_outer_left = 0.0f;
float ff_outer_right = 0.0f;
float ff_outer_down = 0.0f;
// 拨弹盘线性速度前馈变量 (单位: deg/s)，由 shoot 逻辑每拍根据位置误差计算后写入，
// 电机控制器通过 speed_feedforward_ptr 读取并叠加到速度环参考值入口，
// 速度环能感知这个前馈并配合加速，避免电流前馈注入时速度环与前馈对抗的问题。
float ff_loader = 0.0f;

// 拨盘电机句柄是 shoot 应用的核心执行器，目的是初始化、单发、堵转和主任务都围绕同一实例工作。
DJIMotorInstance *loader = NULL;
// 内圈三个摩擦轮句柄需要在摩擦轮控制和掉速检测里共享，目的是trim、掉速基线和 ready 判定都离不开它们。
DJIMotorInstance *friction_inner_down = NULL;
DJIMotorInstance *friction_inner_left = NULL;
DJIMotorInstance *friction_inner_right = NULL;
// 外圈三个摩擦轮句柄同样需要在摩擦轮控制和掉速检测里共享，目的是英雄平台支持外圈裁剪，因此统一用句柄是否为空表示硬件能力。
DJIMotorInstance *friction_outer_down = NULL;
DJIMotorInstance *friction_outer_left = NULL;
DJIMotorInstance *friction_outer_right = NULL;

// 发射反馈发布者在初始化和主任务里都要复用，目的是shoot 应用每拍都要把最新火力状态推回 cmd。
Publisher_t *shoot_pub = NULL;
// 发射命令缓存保存 cmd 层最新下发的控制意图，目的是主任务要围绕这份命令决定拨盘和摩擦轮动作。
Shoot_Ctrl_Cmd_s shoot_cmd_recv = { 0 };
// 发射命令订阅者在初始化后持续复用，目的是主任务每拍都要从同一个订阅实例取命令。
Subscriber_t *shoot_sub = NULL;
// 发射反馈缓存保存本拍准备回传给 cmd 的状态，目的是fire_count、empty_flag 和 bullet_fired_flag 都要通过同一消息体上送。
Shoot_Upload_Data_s shoot_feedback_data = { 0 };

// 连发和反转都依赖跨拍保存的冷却时序，目的是这些动作不是单拍完成，必须记住上一轮动作的时间窗。
float hibernate_time = 0.0f;
float dead_time = 0.0f;

// 单发控制的掉速基线必须跨拍保存，目的是送弹过程中的峰值更新和后续出弹计数都基于同一份控制基线。
DipControlRuntime_s dip_control = { 0 };
// 堵转状态机必须跨拍保存当前阶段与时间戳，目的是自动解卡由“检测 -> 反转 -> 恢复”三段时序构成。
ShootStallHandler_s stall_handler = { 0 };
// 单发状态机必须跨拍保存事务状态，目的是待速、固定 90 度送弹、回速和锁角都不是单拍逻辑。
SingleFireRuntime_s single_fire = { 0 };
// 单发触发边沿缓存必须跨拍保存，目的是用户边沿请求要先缓存住，再由状态机在合适时机消费。
FireTrigger_s fire_trigger = { .last_mode = LOAD_STOP, .trigger_consumed = 0u, .pending_fire = 0u, .last_accept_time_ms = 0.0f };

// 上电软件初始位锁存延时给 DJI 电机留出反馈刷新窗口，目的是不能在反馈还停留在零初始化值时就把 0 度误当成拨弹盘真实初始位。
#define LOADER_INITIAL_LOCK_DELAY_MS 100.0f
// 初始位锁存要求拨弹盘反馈速度已经接近静止，目的是避免机器人上电瞬间外力拨动或反馈滤波过渡时锁到中间态。
#define LOADER_INITIAL_LOCK_SPEED_THRESHOLD 50.0f

static uint8_t loader_initial_position_locked = 0u;
static float loader_initial_position_angle = 0.0f;
static float loader_initial_position_start_time = 0.0f;

// 这些调试结构体指针仅用于在初始化时把调试视图固定到真实状态区，目的是Ozone 直接看指针时需要拿到一份稳定地址。
static FrictionWheelDebug_s *p_friction_debug = NULL;
static StallDebug_s *p_stall_debug = NULL;
static SingleFireDebug_s *p_sf_debug = NULL;
static BulletDipSnapshot_s *p_dip_snapshot = NULL;

void ShootInit(void)
{
    Motor_Init_Config_s friction_config_inner = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 7.7,
                .Ki = 0.0,
                .Kd = 0,
                .DeadBand = 10.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 16000,
            },
            .current_feedforward_ptr = NULL,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .feedforward_flag = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508
    };
    Motor_Init_Config_s friction_config_outer = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 7.3,
                .Ki = 0.0,
                .Kd = 0,
                .DeadBand = 10.0f,
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
            .feedforward_flag = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508
    };
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 这里把位置环 Kp 和输出上限一起抬高，作用是把大角度误差直接转换成高速度给定；
                // 原因是 DJI 串级控制里位置环输出先喂给速度环，若这里不够大，拨盘就无法在起步瞬间打出满力矩冲刺。
                .Kp = 14.0f,
                .Ki = 0.0,
                .Kd = 0.0f,
                .MaxOut = 30000,
            },
            .speed_PID = {
                .Kp = 3.1,
                .Ki = 0.0,
                .Kd = 0.0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 16100,
            },
            // 绑定拨弹盘速度前馈变量，电机控制器每拍通过此指针读取前馈速度并叠加到速度环参考值入口
            .speed_feedforward_ptr = &ff_loader,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            // 启用速度前馈标志，让 DJIMotorControl 在速度环计算前叠加 ff_loader 到速度参考值上
            .feedforward_flag = SPEED_FEEDFORWARD,
        },
        .motor_type = M3508
    };

    // 内圈三个摩擦轮按固定 CAN ID 和方向初始化，目的是前馈指针和方向标志都要在注册前绑定到各自电机实例。
    friction_config_inner.can_init_config.tx_id = 1;
    friction_config_inner.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_config_inner.controller_param_init_config.current_feedforward_ptr = &ff_inner_down;
    friction_inner_down = DJIMotorInit(&friction_config_inner);

    friction_config_inner.can_init_config.tx_id = 2;
    friction_config_inner.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_config_inner.controller_param_init_config.current_feedforward_ptr = &ff_inner_left;
    friction_inner_left = DJIMotorInit(&friction_config_inner);

    friction_config_inner.can_init_config.tx_id = 3;
    friction_config_inner.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_config_inner.controller_param_init_config.current_feedforward_ptr = &ff_inner_right;
    friction_inner_right = DJIMotorInit(&friction_config_inner);

    // 外圈三个摩擦轮同样在初始化阶段绑定独立前馈地址，目的是单发状态机后续要能分别驱动内外圈前馈。
    friction_config_outer.can_init_config.tx_id = 4;
    friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_config_outer.controller_param_init_config.current_feedforward_ptr = &ff_outer_down;
    friction_outer_down = DJIMotorInit(&friction_config_outer);

    friction_config_outer.can_init_config.tx_id = 5;
    friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_config_outer.controller_param_init_config.current_feedforward_ptr = &ff_outer_left;
    friction_outer_left = DJIMotorInit(&friction_config_outer);

    friction_config_outer.can_init_config.tx_id = 6;
    friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_config_outer.controller_param_init_config.current_feedforward_ptr = &ff_outer_right;
    friction_outer_right = DJIMotorInit(&friction_config_outer);

    // 拨盘电机仍按原来的串级配置初始化，目的是这次拆分不改变拨盘控制律和 CAN 接线。
    loader = DJIMotorInit(&loader_config);
    // 从拨盘电机注册完成后开始计时，目的是后续软件初始位锁存只依赖本模块自己的启动窗口，而不是全局上电时间。
    loader_initial_position_start_time = DWT_GetTimeline_ms();

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

    // 初始化调试指针，便于调试器直接观测真实状态区，目的是若不在初始化阶段固定这些地址，Ozone 里看到的可能是空指针。
    p_friction_debug = ShootDebug_GetFrictionPtr();
    p_stall_debug = ShootDebug_GetStallPtr();
    p_sf_debug = ShootDebug_GetSingleFirePtr();
    p_dip_snapshot = ShootDebug_GetDipSnapshotPtr();
    // 初始化摩擦轮 ramp 的时间基准，目的是第一次进入时间型斜坡时不能拿到异常大的 dt，否则会把目标一步跳满。
    DWT_GetDeltaT(&friction_ramp_dwt_cnt);

    // 初始化 VOFA+ 调试输出模块，清零通道缓冲并准备 USART1 DMA 发送
    VofaDebugInit();
}

/**
 * @brief 锁存拨弹盘上电软件初始位
 * @param current_time_ms 当前系统时间，单位为 ms
 */
static void UpdateLoaderInitialPosition(float current_time_ms)
{
    if (loader_initial_position_locked != 0u || loader == NULL)
        return;

    if ((current_time_ms - loader_initial_position_start_time) < LOADER_INITIAL_LOCK_DELAY_MS) {
        // 启动保护窗口内只保持速度为零，目的是等待第一批 CAN 反馈和速度滤波稳定，避免位置环抢先拉动拨弹盘。
        LoaderSetSpeedRef(0.0f);
        return;
    }

    if (loader->dt <= 0.0f) {
        // 至少收到过一次 DJI 电机反馈后 dt 才会被刷新，目的是不能只凭结构体默认值或 daemon 初始计数就锁软件初始位。
        LoaderSetSpeedRef(0.0f);
        return;
    }

    if (loader->daemon == NULL || DaemonIsOnline(loader->daemon) == 0u) {
        // 拨盘电机尚未确认在线时不能锁初始位，目的是离线状态下 total_angle 可能已经不再代表真实当前位置。
        LoaderSetSpeedRef(0.0f);
        return;
    }

    if (fabsf(loader->measure.speed_aps) > LOADER_INITIAL_LOCK_SPEED_THRESHOLD) {
        // 反馈速度未稳定时继续等待，目的是把“上电当前位置”定义成静止弹位，而不是运动过程中的瞬时角度。
        LoaderSetSpeedRef(0.0f);
        return;
    }

    loader_initial_position_angle = loader->measure.total_angle;
    loader_initial_position_locked = 1u;
    // 单发状态机的锁角目标也从同一个初始位开始，目的是后续空闲保持、正常单发和堵转回退都围绕同一套弹位坐标工作。
    single_fire.lock_target_angle = loader_initial_position_angle;
    single_fire.rush_start_angle = loader_initial_position_angle;
    single_fire.rush_target_angle = loader_initial_position_angle;
    LoaderSetAngleRef(loader_initial_position_angle);
}

/**
 * @brief 空闲时保持拨弹盘在已知弹位
 */
void HoldLoaderIdlePosition(void)
{
    if (loader == NULL)
        return;

    if (loader_initial_position_locked == 0u) {
        // 尚未锁定软件初始位前不能使用位置环追目标，目的是避免把未知反馈当成绝对弹位导致上电误动作。
        LoaderSetSpeedRef(0.0f);
        return;
    }

    LoaderSetAngleRef(single_fire.lock_target_angle);
}

/* 机器人发射机构控制核心任务 */
void ShootTask(void)
{
    static uint16_t last_report_fire_count = 0;
    float current_time_ms = DWT_GetTimeline_ms();
    loader_mode_e raw_load_mode;
    loader_mode_e requested_load_mode;
    loader_mode_e stall_input_mode;
    loader_mode_e actual_load_mode;
    FrictionWheelDebug_s *p_fric;

    // 从 cmd 获取控制数据；原因是 shoot 应用本拍的全部动作都必须围绕最新一帧的发射命令展开。
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    raw_load_mode = shoot_cmd_recv.load_mode;
    UpdateLoaderInitialPosition(current_time_ms);

    // 先对单发触发做边沿锁存，目的是单发要完整执行固定 90 度位置目标，不能依赖 `LOAD_1_BULLET` 电平持续存在。
    if (loader_initial_position_locked == 0u) {
        fire_trigger.pending_fire = 0u;
        fire_trigger.trigger_consumed = (uint8_t)(raw_load_mode == LOAD_1_BULLET);
    } else if (raw_load_mode != LOAD_1_BULLET) {
        fire_trigger.trigger_consumed = 0;
    } else if (fire_trigger.last_mode != LOAD_1_BULLET && !fire_trigger.trigger_consumed) {
        uint8_t single_fire_can_accept_request =
            (uint8_t)(single_fire.state == SF_IDLE || single_fire.state == SF_LOCKING);
        uint8_t trigger_guard_elapsed =
            (uint8_t)((current_time_ms - fire_trigger.last_accept_time_ms) >= SF_TRIGGER_REARM_GUARD_MS);

        // 只允许在单发状态机已经完全收口时接收新的单发请求，目的是若上一发还在待速、送弹或回速阶段就继续收边沿，会把一次点击排成两发。
        if (single_fire_can_accept_request != 0u && trigger_guard_elapsed != 0u) {
            fire_trigger.pending_fire = 1u;
            // 只有真正接受这次请求时才刷新最近接受时间，目的是防抖窗口应该围绕“有效触发”建立，不能被被拒绝的伪边沿不断往后推迟。
            fire_trigger.last_accept_time_ms = current_time_ms;
        }

        // 无论本次边沿是否被接受都立即标记为已消费，目的是同一次扳机抖动或鼠标回弹只能贡献一次判定。
        fire_trigger.trigger_consumed = 1u;
    }

    requested_load_mode = raw_load_mode;
    if (loader_initial_position_locked == 0u) {
        // 软件初始位未锁定前统一禁止拨弹盘执行发射模式，目的是所有相对 90 度送弹都必须建立在明确的弹位坐标上。
        requested_load_mode = LOAD_STOP;
    }

    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
        current_inner_deg = 0.0f;
        current_outer_deg = 0.0f;
        // 急停时同步清空最终目标缓存，目的是ready/recover 判定不能继续拿上一次开火目标当作当前目标。
        UpdateFrictionTargetAbs(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        requested_load_mode = LOAD_STOP;
        fire_trigger.pending_fire = 0u;
        fire_trigger.trigger_consumed = 0u;
        AbortSingleFire();
        SetMotorEnableIfReady(friction_inner_left, 0u);
        SetMotorEnableIfReady(friction_inner_right, 0u);
        SetMotorEnableIfReady(friction_outer_left, 0u);
        SetMotorEnableIfReady(friction_outer_right, 0u);
        SetMotorEnableIfReady(friction_inner_down, 0u);
        SetMotorEnableIfReady(friction_outer_down, 0u);
        SetMotorEnableIfReady(loader, 0u);
    } else {
        // 恢复运行时统一重新使能全部已安装电机，目的是发射链复位后不能沿用上一次急停留下的失能状态。
        SetMotorEnableIfReady(friction_inner_left, 1u);
        SetMotorEnableIfReady(friction_inner_right, 1u);
        SetMotorEnableIfReady(friction_outer_left, 1u);
        SetMotorEnableIfReady(friction_outer_right, 1u);
        SetMotorEnableIfReady(friction_inner_down, 1u);
        SetMotorEnableIfReady(friction_outer_down, 1u);
        SetMotorEnableIfReady(loader, 1u);
    }

    // 固定 90 度事务待速和送弹期间即使上层已经回到 STOP，也仍把请求送进堵转状态机，目的是单发事务尚未收口时自动解卡仍然有意义。
    stall_input_mode = requested_load_mode;
    if ((requested_load_mode == LOAD_STOP) &&
        !SingleFireIsRetryActive() &&
        (single_fire.state == SF_WAIT_SPEED || single_fire.state == SF_FEEDING)) {
        stall_input_mode = LOAD_1_BULLET;
    }

    actual_load_mode = HandleLoaderStall(stall_input_mode);

    if (stall_handler.state == STALL_REVERSING || stall_handler.state == STALL_RECOVERY) {
        // 堵转反转和恢复窗口内由堵转状态机独占拨弹盘目标，目的是防止主发射 switch 再把目标覆盖成速度 0 或继续送弹。
        AbortSingleFire();
        single_fire.lock_target_angle = stall_handler.reverse_target_angle;
        LoaderSetAngleRef(stall_handler.reverse_target_angle);
    } else {
        switch (actual_load_mode) {
        case LOAD_STOP:
            // 非自保持阶段收到 STOP 时直接中止状态机，目的是安全停拨优先于继续维持历史残留状态。
            if (!SingleFireIsRetryActive() &&
                (single_fire.state != SF_IDLE || fire_trigger.pending_fire) &&
                stall_handler.state == STALL_NORMAL) {
                HandleSingleFire(fire_trigger.pending_fire);
            } else {
                AbortSingleFire();
                HoldLoaderIdlePosition();
            }
            break;

        case LOAD_1_BULLET:
            HandleSingleFire(fire_trigger.pending_fire);
            break;

        case LOAD_3_BULLET:
            AbortSingleFire();
            if (hibernate_time + dead_time > DWT_GetTimeline_ms())
                break;
            single_fire.lock_target_angle = loader->measure.total_angle + LoaderBulletCountToMotorAngle(3.0f);
            LoaderSetAngleRef(single_fire.lock_target_angle);
            hibernate_time = DWT_GetTimeline_ms();
            dead_time = 300.0f;
            break;

        case LOAD_2_BULLET:
            AbortSingleFire();
            if (hibernate_time + dead_time > DWT_GetTimeline_ms())
                break;
            single_fire.lock_target_angle = loader->measure.total_angle + LoaderBulletCountToMotorAngle(2.0f);
            LoaderSetAngleRef(single_fire.lock_target_angle);
            hibernate_time = DWT_GetTimeline_ms();
            dead_time = 200.0f;
            break;

        case LOAD_BURSTFIRE:
            // 连发模式切入时复位单发消费标志，目的是后续切回单发时应该能立即重新接受一次新的边沿请求。
            AbortSingleFire();
            single_fire.lock_target_angle = loader->measure.total_angle;
            fire_trigger.trigger_consumed = 0u;
            // 连发速度统一按单发机械节距换算，目的是不能再保留与单发位置目标冲突的历史常量。
            LoaderSetSpeedRef(LoaderBulletRateToMotorSpeed(shoot_cmd_recv.shoot_rate));
            break;

        case LOAD_REVERSE:
            AbortSingleFire();
            // 手动反转同样按完整一发弹位执行，目的是用户触发反转时能明确回到上一个 90 度弹位，而不是停在半发中间位置。
            single_fire.lock_target_angle = loader->measure.total_angle - LoaderBulletCountToMotorAngle(1.0f);
            LoaderSetAngleRef(single_fire.lock_target_angle);
            hibernate_time = DWT_GetTimeline_ms();
            dead_time = 150.0f;
            break;

        default:
            while (1)
                ;
        }
    }

    // 保存原始遥控指令而不是实际执行模式，目的是堵转状态机会临时把模式改成 STOP，边沿检测只能对用户原始动作敏感。
    fire_trigger.last_mode = raw_load_mode;

    p_fric = ShootDebug_GetFrictionPtr();
    if (p_fric->override_enable) {
        // 调试模式: 直接使用 debug 结构体中的目标速度；原因是调参时要能绕开上层弹速档位逻辑。
        ShootSetSpeedDual(p_fric->target_inner_mps, p_fric->target_outer_mps);
    } else if (shoot_cmd_recv.friction_mode == FRICTION_ON) {
        // 正常模式: 根据不同的弹速等级设置不同的分级速度；原因是当前双级摩擦轮靠“内圈先加速、外圈后稳速”来匹配目标弹速。
        switch (shoot_cmd_recv.bullet_speed) {
        case BIG_AMU_12:
            ShootSetSpeedDual(11.0f, 11.7f);
            break;
        case BIG_AMU_16:
            ShootSetSpeedDual(16.2f, 16.2f);
            break;
        default:
            ShootSetSpeedDual(15.2f, 16.3f);
            break;
        }
    } else {
        ShootSetSpeedDual(0.0f, 0.0f);
    }

    if (shoot_cmd_recv.lid_mode == LID_CLOSE) {
        // 当前版本未接入弹舱盖执行器，目的是本次拆文件不额外补硬件功能，只保留原占位分支。
    } else if (shoot_cmd_recv.lid_mode == LID_OPEN) {
        // 当前版本未接入弹舱盖执行器，目的是保持原占位行为，避免在拆分过程中引入新动作。
    }

    // 降低调试信息更新频率到约 10ms 一次，目的是每个循环都做多次浮点运算会占掉高频任务预算。
    static uint32_t debug_update_count = 0u;
    if (debug_update_count++ % 10u == 0u) {
        UpdateFrictionDebugInfo();
    }

    // 用 fire_count 边沿生成 `bullet_fired_flag`，目的是单发现在会在锁角态停很久，不能再用状态枚举直接映射“本周期已发射”。
    shoot_feedback_data.fire_count = single_fire.fire_count;
    shoot_feedback_data.bullet_fired_flag = (single_fire.fire_count != last_report_fire_count);
    last_report_fire_count = single_fire.fire_count;
    shoot_feedback_data.empty_flag = (single_fire.feed_timeout_count > 0u);

    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);

    // 填充 VOFA+ 调试通道并发送 JustFloat 帧，复用 shoot_debug 已有数据源避免重复计算
    {
        FrictionWheelDebug_s *p_vofa_fric = ShootDebug_GetFrictionPtr();
        SingleFireDebug_s *p_vofa_sf = ShootDebug_GetSingleFirePtr();

        // ch0~ch5: 6 个摩擦轮实际线速度 (m/s)，观测摩擦轮升速、稳态和掉速全过程
        VofaDebugSetChannel(0, p_vofa_fric->inner_left_mps);
        VofaDebugSetChannel(1, p_vofa_fric->inner_right_mps);
        VofaDebugSetChannel(2, p_vofa_fric->inner_down_mps);
        VofaDebugSetChannel(3, p_vofa_fric->outer_left_mps);
        VofaDebugSetChannel(4, p_vofa_fric->outer_right_mps);
        VofaDebugSetChannel(5, p_vofa_fric->outer_down_mps);

        // ch6: 拨盘电机速度 (deg/s)，观测送弹冲刺和堵转反转的速度响应
        VofaDebugSetChannel(6, p_vofa_sf->loader_speed);

        // ch7: 内圈掉速差 (baseline - current)，正值表示弹丸正在通过内圈摩擦轮
        VofaDebugSetChannel(7, p_vofa_sf->speed_diff);

        // ch8: 外圈掉速差 (outer_baseline - current_outer)，正值表示弹丸正在通过外圈摩擦轮
        VofaDebugSetChannel(8, p_vofa_sf->outer_baseline_speed - p_vofa_sf->current_outer_speed);

        // ch9: 累计发射计数，每跳一格代表成功检测到一发弹丸
        VofaDebugSetChannel(9, (float)p_vofa_sf->fire_count);

        // ch10~ch15: 6 个电机各自的实时掉速量 (baseline - |current|, deg/s)
        // 从 dip_control 取送弹前锁存的基线，减去当前瞬时速度绝对值，正值代表该轮正在被弹丸减速
        VofaDebugSetChannel(10, dip_control.inner_left_baseline - fabsf(GetMotorSpeedAps(friction_inner_left)));
        VofaDebugSetChannel(11, dip_control.inner_right_baseline - fabsf(GetMotorSpeedAps(friction_inner_right)));
        VofaDebugSetChannel(12, dip_control.inner_down_baseline - fabsf(GetMotorSpeedAps(friction_inner_down)));
        VofaDebugSetChannel(13, dip_control.outer_left_baseline - fabsf(GetMotorSpeedAps(friction_outer_left)));
        VofaDebugSetChannel(14, dip_control.outer_right_baseline - fabsf(GetMotorSpeedAps(friction_outer_right)));
        VofaDebugSetChannel(15, dip_control.outer_down_baseline - fabsf(GetMotorSpeedAps(friction_outer_down)));

        VofaDebugSend();
    }
}
