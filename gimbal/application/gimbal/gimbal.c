#include "gimbal_private.h"

// 云台 IMU、电机和命令反馈缓存会在初始化、主任务和拆分 helper 间共同使用，目的是拆成多个编译单元后仍必须围绕同一份状态协同工作。
attitude_t *gimba_IMU_data = NULL;
DMMotorInstance *yaw_motor = NULL;
DMMotorInstance *pitch_motor = NULL;
static Publisher_t *gimbal_pub = NULL; // 云台应用消息发布者，目的是反馈只在入口文件统一推送，不需要对外暴露。
static Subscriber_t *gimbal_sub = NULL; // cmd 控制消息订阅者，目的是主任务入口负责消费，不需要跨文件共享。
Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给 cmd 的云台状态，目的是主任务末尾统一打包和发送反馈。
Gimbal_Ctrl_Cmd_s gimbal_cmd_recv; // 来自 cmd 的控制信息，目的是主任务和前馈 helper 都要读取同一拍控制目标。
static GimbalCali_Handler_t pitch_cali_handler; // 标定句柄暂时继续保留原位置，目的是这轮只做文件拆分，不顺手改动历史标定链路。
static BMI088Instance *bmi088; // 云台 IMU 调试句柄继续保留原位置，目的是不扩散未使用调试接口，避免超出本轮拆分范围。

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

    // 配置 pitch 电机的角度环和速度环，目的是pitch 同样走 IMU 外环控制，并保留当前已经验证过的负向速度环参数。
    pitch_config = (Motor_Init_Config_s){
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x14,
            .rx_id = 0x15,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 1.32,
                .Ki = 0.0,
                .Kd = 0.03,
                .DeadBand = 0.0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 19,
            },
            .speed_PID = {
                .Kp = 2.7,
                .Ki = 1.81,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5,
                .MaxOut = 19,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[1],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = J4310,
    };

    // 初始化 yaw 和 pitch 电机实例，目的是云台当前所有控制逻辑最终都围绕这两个 DM 电机展开。
    yaw_motor = DMMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&pitch_config);
    // if (pitch_motor != NULL) {
    //     // 给 pitch 电机立即配置机械限位，目的是防止后续任何模式下目标角异常时直接撞限位。
    //     pitch_motor->pos_limit_max = PITCH_MECH_LIMIT_MAX;
    //     pitch_motor->pos_limit_min = PITCH_MECH_LIMIT_MIN;
    // }

    // 注册云台反馈发布者和命令订阅者，目的是当前 cmd 与 gimbal 仍通过消息中心通信，拆文件不改变数据流入口。
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

    // 在云台初始化阶段统一拉起图传固定电机，目的是该电机生命周期从属于云台模块，且必须在 CAN 就绪后初始化。
    VideoLinkMotorInit();

    // 在云台初始化时同步把 pitch 标定状态机归零，目的是 VT03 新入口会直接复用这份句柄，不能继续保留“声明了但未初始化”的历史状态。
    GimbalCali_Init(&pitch_cali_handler);
}

/**
 * @brief 触发一次 pitch 标定状态机
 *
 */
void GimbalStartPitchCalibration(void)
{
    // 只有在 pitch 电机实例已经就绪时才允许启动标定，目的是状态机第一步就要接管该电机，空指针场景下不能假装启动成功。
    if (pitch_motor == NULL) {
        return;
    }

    // 这里统一由云台模块内部转发启动请求，目的是外部模块不需要知道标定句柄放在哪里，也不应该直接改它的状态。
    GimbalCali_Start(&pitch_cali_handler);
}

/**
 * @brief 查询 pitch 标定是否正在运行
 *
 * @return uint8_t 1:标定运行中 0:标定空闲
 */
uint8_t GimbalPitchCalibrationActive(void)
{
    // 只要状态机不在空闲态，就视为 pitch 标定仍在占用控制权，目的是 `robot_cmd` 需要用这一位统一屏蔽重复触发和摇杆覆写。
    return (uint8_t)(pitch_cali_handler.state != CALI_STATE_IDLE);
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
    uint8_t pitch_cali_active = 0u;
    float pitch_ref;
    float current_pitch_deg;

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
    if (pitch_motor != NULL && DMMotorIsOnline(pitch_motor) != 0u) {
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

    // 先锁存本拍最终要给 pitch 的参考角，目的是当前恢复链已经简化成单纯的目标同步，这里不再额外夹带任何 PID 状态操作。
    pitch_ref = gimbal_cmd_recv.pitch;
    // 在模式切换前先读取一次标定占用位，目的是本拍若已经进入 pitch 标定，后面的模式 switch 就必须跳过常规 pitch 目标下发，避免两条控制链互相覆盖。
    pitch_cali_active = GimbalPitchCalibrationActive();

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
            DMMotorStop(pitch_motor);
        }
        // 零力模式下同步停掉图传固定电机，目的是云台主执行器已经失能时，图传电机也不应继续运动。
        VideoLinkMotorDisable();
        break;

    case GIMBAL_GYRO_MODE:
        if (yaw_motor != NULL) {
            DMMotorEnable(yaw_motor);
        }
        if (pitch_motor != NULL) {
            DMMotorEnable(pitch_motor);
        }
        if (yaw_motor != NULL) {
            // yaw 目标角继续直接采用 cmd 侧整理后的多圈目标，目的是本轮拆分只调整文件结构，不改原有多圈控制语义。
            DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
        }
        if (pitch_motor != NULL && pitch_cali_active == 0u) {
            // 只有在 pitch 标定空闲时才允许常规控制链写入 pitch 参考角，目的是标定状态机运行期间必须独占 `pitch_motor` 的开环力矩输出。
            DMMotorSetRef(pitch_motor, pitch_ref);
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
            DMMotorEnable(pitch_motor);
        }
        if (yaw_motor != NULL) {
            DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
        }
        if (pitch_motor != NULL && pitch_cali_active == 0u) {
            // 自由模式同样要让位给标定状态机，目的是这轮新增的是“pitch 标定独占控制权”，不能只在陀螺仪模式里屏蔽。
            DMMotorSetRef(pitch_motor, pitch_ref);
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
    current_pitch_deg = (gimba_IMU_data != NULL) ? gimba_IMU_data->Pitch : 0.0f;
    VideoLinkMotorTask(current_pitch_deg);
    if (pitch_cali_active != 0u) {
        if (gimbal_cmd_recv.gimbal_mode == GIMBAL_ZERO_FORCE) {
            // 一旦用户主动切回零力或安全链把云台打进零力，就立即中止本次标定并恢复原配置，目的是零力优先级必须高于标定流程。
            GimbalCali_Abort(&pitch_cali_handler, pitch_motor);
        } else {
            // 只有云台仍处于有力模式时才推进标定状态机，目的是让标定流程在安全前提下独占 `pitch_motor`，同时不影响 yaw 的正常闭环。
            (void)GimbalCali_Update(&pitch_cali_handler, pitch_motor, current_pitch_deg);
        }
    }

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
