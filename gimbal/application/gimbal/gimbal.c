#include "gimbal_private.h"
#include "motor_def.h"

// 云台 IMU、电机和命令反馈缓存会在初始化、主任务和拆分 helper 间共同使用，目的是拆成多个编译单元后仍必须围绕同一份状态协同工作。
attitude_t *gimba_IMU_data = NULL;
DMMotorInstance *yaw_motor = NULL;
DJIMotorInstance *pitch_motor = NULL;
static Publisher_t *gimbal_pub = NULL; // 云台应用消息发布者，目的是反馈只在入口文件统一推送，不需要对外暴露。
static Subscriber_t *gimbal_sub = NULL; // cmd 控制消息订阅者，目的是主任务入口负责消费，不需要跨文件共享。
Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给 cmd 的云台状态，目的是主任务末尾统一打包和发送反馈。
Gimbal_Ctrl_Cmd_s gimbal_cmd_recv; // 来自 cmd 的控制信息，目的是主任务和前馈 helper 都要读取同一拍控制目

/**
 * @brief 初始化云台
 *
 */
void GimbalInit(void)
{
    static attitude_t gimbal_default_attitude = { 0 };
    Motor_Init_Config_s yaw_config;
    Motor_Init_Config_s pitch_config;

    // 先初始化 IMU 并缓存姿态数据指针，目的是yaw 与 pitch 都使用 IMU 作为外部反馈源，后续电机初始化必须拿到稳定指针。
    gimba_IMU_data = INS_Init();
    if (gimba_IMU_data == NULL) {
        // INS 初始化失败时回退到静态零姿态，目的是至少要避免把空指针直接塞进电机控制链造成野指针访问。
        gimba_IMU_data = &gimbal_default_attitude;
    }

    // 配置 yaw 电机的角度环和速度环，目的是yaw 当前采用 IMU 角度与角速度双环控制，这里保留原有参数不改行为。
    yaw_config = (Motor_Init_Config_s){
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x01,
            .rx_id = 0x03,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.67,
                .Ki = 0,
                .Kd = 0.01,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ErrorHandle,
                .IntegralLimit = 7,
                .MaxOut = 21,
            },
            .speed_PID = {
                .Kp = 1.32,
                .Ki = YAW_SPEED_PID_KI,
                .Kd = 0,
                .DeadBand = YAW_SPEED_PID_DEADBAND_RAD,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate,
                .IntegralLimit = YAW_SPEED_PID_INTEGRAL_LIMIT,
                .CoefA = YAW_SPEED_PID_COEF_A_RAD,
                .CoefB = YAW_SPEED_PID_COEF_B_RAD,
                .MaxOut = 15,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = J8006,
    };

    pitch_config = (Motor_Init_Config_s){
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                // pitch 只保留速度环，目的是复刻旧工程“输入直接生成速度目标，速度 PID 输出 C620 电流”的链路；角度安全边界放到 GimbalTask 里用 IMU 实际角兜底处理。
                .Kp = 5.2,
                .Ki = 1.2,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 6000,
                .MaxOut = 16000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            // 速度环使用电机自身 speed_aps 反馈，目的是保持控制量和反馈量都在 degree/s 量纲内，避免把 IMU Gyro 的 rad/s 与 DJI 速度环混用。
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            .feedforward_flag = FEEDFORWARD_NONE,
        },
        .motor_type = M3508,
    };

    // 初始化 yaw 和 pitch 电机实例，目的是 yaw 继续使用 DM 电机链路，pitch 切换到 DJI 3508 链路但上层目标仍由同一套云台任务下发。
    yaw_motor = DMMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);

    // 注册云台反馈发布者和命令订阅者，目的是当前 cmd 与 gimbal 仍通过消息中心通信，拆文件不改变数据流入口。
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

    // 在云台初始化阶段统一拉起图传固定电机，目的是该电机生命周期从属于云台模块，且必须在 CAN 就绪后初始化。
    VideoLinkMotorInit();
}

/**
 * @brief 触发一次 pitch 标定状态机
 *
 */
void GimbalStartPitchCalibration(void)
{
    // Pitch 已经临时切换为 DJI 3508，旧标定状态机依赖 DM MIT 开环力矩与 DM 扭矩反馈，继续启动会把 3508 误当成 DM 力矩源使用，因此在 3508 迁移阶段先保持入口无动作。
    return;
}

/**
 * @brief 查询 pitch 标定是否正在运行
 *
 * @return uint8_t 1:标定运行中 0:标定空闲
 */
uint8_t GimbalPitchCalibrationActive(void)
{
    // Pitch 3508 当前没有接入新的 DJI 标定流程，旧 DM 标定链被暂停后应始终向上层报告空闲，避免 cmd 层误以为 pitch 控制权被占用。
    return 0u;
}

/* 机器人云台控制核心任务,后续考虑只保留 IMU 控制,不再需要电机的反馈 */
void GimbalTask(void)
{
    static gimbal_mode_e last_gimbal_mode = GIMBAL_ZERO_FORCE; // 记录上一拍云台模式，目的是前馈和模式边沿逻辑仍要基于真实切换事件工作，不能因为删除 PID 清零链就把边沿判断也一起丢掉。
    static uint8_t last_yaw_motor_online = 0u; // 记录 yaw 电机上一拍在线状态，目的是前馈和反馈冻结逻辑仍要识别掉线/复活边沿，不能再把这个状态只理解成“是否触发 PID 清零”。
    static uint8_t last_pitch_motor_online = 0u; // 记录 pitch 电机上一拍在线状态，目的是 pitch 前馈滤波状态和反馈发布同样依赖在线边沿，去掉 PID 清零后这里依然必须保留。
    static float yaw_motor_single_round_cache_deg = 0.0f; // 缓存最后一次可信的 yaw 单圈机械角，目的是电机离线时继续发布这个值，底盘跟随不会被脏反馈带偏。
    uint8_t yaw_motor_online = 0u;
    uint8_t pitch_motor_online = 0u;
    uint8_t yaw_motor_online_changed = 0u;
    uint8_t pitch_motor_online_changed = 0u;
    uint8_t gimbal_mode_changed = 0u;
    float pitch_speed_ref;
    float current_pitch_deg;
    float pitch_limit_distance_deg;

    // 先获取本拍最新云台控制命令，目的是后续在线状态边沿处理、模式切换和前馈计算都必须基于同一拍目标执行。
    if (gimbal_sub != NULL) {
        SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    } else {
        // 若消息订阅失败则回退到全零命令，目的是至少要保证任务在异常初始化场景下不会沿用上一拍脏命令。
        memset(&gimbal_cmd_recv, 0, sizeof(gimbal_cmd_recv));
    }

    // 采样 yaw/pitch 两个电机当前是否在线，目的是掉线与复活边沿仍要驱动前馈状态收口和反馈冻结策略。
    if (yaw_motor != NULL && DMMotorIsOnline(yaw_motor) != 0u) {
        yaw_motor_online = 1u;
    }
    if (pitch_motor != NULL && pitch_motor->daemon != NULL && DaemonIsOnline(pitch_motor->daemon) != 0u) {
        pitch_motor_online = 1u;
    }

    if (yaw_motor_online != last_yaw_motor_online) {
        // yaw 电机在线状态变化时仅上报边沿，目的是当前这轮明确去掉手动 PID 清状态链，但前馈与反馈发布仍需要知道什么时候发生了掉线/复活。
        yaw_motor_online_changed = 1u;
        last_yaw_motor_online = yaw_motor_online;
    }
    if (pitch_motor_online != last_pitch_motor_online) {
        // pitch 电机在线状态变化时同样只保留边沿信息，目的是恢复链路继续依赖在线状态贴齐目标，但不再额外手搓 PID 内部状态。
        pitch_motor_online_changed = 1u;
        last_pitch_motor_online = pitch_motor_online;
    }

    // 速度环模式下 pitch 的安全边界必须看 IMU 实际角，而不是只看 cmd 的角度目标；这样即使上层目标或输入源异常，也不能在撞限位方向继续给速度。
    current_pitch_deg = (gimba_IMU_data != NULL) ? gimba_IMU_data->Pitch : 0.0f;
    pitch_speed_ref = gimbal_cmd_recv.pitch_speed_ref;
    if (pitch_speed_ref > PITCH_SPEED_REF_MAX_DPS) {
        // cmd 层已经做过速度限幅，但 gimbal 是离电机最近的最后一道保护；这里再次夹紧上限，避免异常发布者把速度参考直接推满。
        pitch_speed_ref = PITCH_SPEED_REF_MAX_DPS;
    } else if (pitch_speed_ref < -PITCH_SPEED_REF_MAX_DPS) {
        // 下压方向同样在底层兜底，目的是无论上层来源如何变化，最终进入 C620 的速度参考都不能越过统一配置。
        pitch_speed_ref = -PITCH_SPEED_REF_MAX_DPS;
    }
    if (pitch_speed_ref > 0.0f) {
        // 上抬方向只根据距离上限的剩余角度做处理，目的是接近上限时提前把高速命令平滑收下来，同时不影响操作者反向下压退出限位。
        pitch_limit_distance_deg = PITCH_MAX_ANGLE - current_pitch_deg;
        if (pitch_limit_distance_deg <= 0.0f) {
            // 实际 pitch 已到或越过上抬软件限位时，只禁止继续往上抬的速度命令，反向速度仍放行，目的是不会把机构卡死在限位边界。
            pitch_speed_ref = 0.0f;
        } else if (pitch_limit_distance_deg < PITCH_LIMIT_RAMP_ZONE_DEG) {
            // 距离上限不足 3 度时按剩余距离线性缩小速度参考，目的是让速度环越接近边界越慢，降低高速撞限位和突兀切零的风险。
            pitch_speed_ref *= pitch_limit_distance_deg / PITCH_LIMIT_RAMP_ZONE_DEG;
        }
    } else if (pitch_speed_ref < 0.0f) {
        // 下压方向使用同一套剩余角度斜坡，目的是上下限手感一致，并且只削减继续压向下限的速度。
        pitch_limit_distance_deg = current_pitch_deg - PITCH_MIN_ANGLE;
        if (pitch_limit_distance_deg <= 0.0f) {
            // 实际 pitch 已到或越过下压软件限位时，只禁止继续下压的速度命令，反向脱离限位必须保留，避免操作者无法把枪管拉回安全区。
            pitch_speed_ref = 0.0f;
        } else if (pitch_limit_distance_deg < PITCH_LIMIT_RAMP_ZONE_DEG) {
            // 距离下限不足 3 度时保留负号只缩小幅值，目的是让下压接近边界时同样线性减速，而不是在限位点才硬切为零。
            pitch_speed_ref *= pitch_limit_distance_deg / PITCH_LIMIT_RAMP_ZONE_DEG;
        }
    }

    if (gimbal_cmd_recv.gimbal_mode != last_gimbal_mode) {
        // 模式切换时只保留边沿标记，目的是前馈和其余运行时逻辑仍需要知道模式发生了变化，但不再借这个边沿去强行清空 PID 内部状态。
        gimbal_mode_changed = 1u;
        last_gimbal_mode = gimbal_cmd_recv.gimbal_mode;
    }

    // 根据当前模式使能或停止云台电机，并把最终目标下发到底层，目的是主任务仍保留最核心的模式 switch，便于直接核对控制链顺序。
    switch (gimbal_cmd_recv.gimbal_mode) {
    case GIMBAL_ZERO_FORCE:
        if (yaw_motor != NULL) {
            DMMotorStop(yaw_motor);
        }
        if (pitch_motor != NULL) {
            DJIMotorStop(pitch_motor);
        }
        // 零力模式下同步停掉图传固定电机，目的是云台主执行器已经失能时，图传电机也不应继续运动。
        VideoLinkMotorDisable();
        break;

    case GIMBAL_GYRO_MODE:
        if (yaw_motor != NULL) {
            DMMotorEnable(yaw_motor);
        }
        if (pitch_motor != NULL) {
            DJIMotorEnable(pitch_motor);
        }
        if (yaw_motor != NULL) {
            // yaw 目标角继续直接采用 cmd 侧整理后的多圈目标，目的是本轮拆分只调整文件结构，不改原有多圈控制语义。
            DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
        }
        if (pitch_motor != NULL) {
            // pitch 已切成速度环，输入参考直接来自 cmd 汇总出的瞬态角速度；方向统一交给 DJI 电机配置里的 MOTOR_DIRECTION_REVERSE 处理，避免应用层再取反导致速度语义和 IMU 限位方向互相打架。
            DJIMotorSetRef(pitch_motor, pitch_speed_ref);
        }
        // 陀螺仪模式下使能图传固定电机，目的是云台正常工作时图传随动机构也应保持工作。
        VideoLinkMotorEnable();
        break;

    case GIMBAL_FREE_MODE:
        if (yaw_motor != NULL) {
            DMMotorEnable(yaw_motor);
            // 自由模式先关闭旧的速度前馈位，目的是保持与原逻辑一致，避免这一路旧标志和当前电流前馈语义混用。
            yaw_motor->motor_settings.feedforward_flag &= ~SPEED_FEEDFORWARD;
        }
        if (pitch_motor != NULL) {
            DJIMotorEnable(pitch_motor);
        }
        if (yaw_motor != NULL) {
            DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
        }
        if (pitch_motor != NULL) {
            // 自由模式仍然沿用同一条 pitch 速度环，目的是只改变底盘/云台模式语义，不再为 pitch 额外分叉一套角度控制。
            DJIMotorSetRef(pitch_motor, pitch_speed_ref);
        }
        // 自由模式下也保持图传固定电机工作，目的是该机构从属于“云台有力”状态，而不是某一种具体姿态模式。
        VideoLinkMotorEnable();
        break;

    default:
        break;
    }

    // 在模式输出已经确定后统一更新 Pitch/Yaw 电流前馈，目的是前馈依赖本拍最终模式、在线状态和参考角，必须在这些量稳定后再计算。
    UpdateGimbalCurrentFeedforward(gimbal_mode_changed,
                                   yaw_motor_online,
                                   yaw_motor_online_changed,
                                   pitch_motor_online,
                                   pitch_motor_online_changed);

    // 每周期执行图传电机状态机，目的是该状态机需要持续检测堵转并切换状态，必须跟随云台任务周期运行。
    VideoLinkMotorTask(current_pitch_deg);
    // Pitch 3508 迁移阶段暂停旧 DM 力矩标定状态机，目的是常规 pitch 控制只保留“IMU 软件限位 + DJI 速度环”，不再让 DM 专用开环力矩流程接管新电机。

    // 只有 yaw 电机当前在线时才刷新单圈机械角缓存，目的是掉线后继续沿用最近一次可信值，底盘跟随不会被脏反馈带偏。
    if (yaw_motor_online != 0u && yaw_motor != NULL) {
        float yaw_rad = yaw_motor->measure.position;

        // 将单圈硬件位置统一换算到 0~360 度，目的是固定对正角 YAW_CHASSIS_ALIGN_DEG 也是 0~360 度坐标，二者必须落在同一坐标系里比较。
        yaw_motor_single_round_cache_deg = NormalizeAngleTo360(yaw_rad * RAD_2_DEGREE);
    }

    // 统一打包反馈结构，目的是cmd 层后续的偏角计算、电机在线状态判断和姿态同步都依赖这同一份反馈快照。
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor_single_round_cache_deg;
    gimbal_feedback_data.yaw_motor_online = yaw_motor_online;
    gimbal_feedback_data.pitch_motor_online = pitch_motor_online;
    if (gimba_IMU_data != NULL) {
        gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    } else {
        memset(&gimbal_feedback_data.gimbal_imu_data, 0, sizeof(gimbal_feedback_data.gimbal_imu_data));
    }

    if (gimbal_pub != NULL) {
        // 在任务尾部统一推送本拍云台反馈，目的是保持 cmd 侧始终消费到同一拍整理完成的状态。
        PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
    }
}
