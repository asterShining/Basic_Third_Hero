#include "gimbal.h"
#include "dmmotor.h"
#include "motor_def.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "gimbal_pitch_cali.h"
#include "video_link_motor.h" // [新增] 图传固定电机模块; Why: 在云台中统一管理图传电机的生命周期

// // 顶部宏定义区域
// #define PITCH_GRAVITY_COEFFICIENT_K1 -15.2383f
// #define PITCH_GRAVITY_COEFFICIENT_K2 -0.2070f
// // 新增 Offset 宏 (注意保留负号)
// #define PITCH_GRAVITY_OFFSET -16.1574f

// 更新为拟合结果
#define PITCH_GRAVITY_COEFFICIENT_K1 -1.406f
#define PITCH_GRAVITY_COEFFICIENT_K2 -0.6058f
// 新增 Offset 宏 (注意保留负号)
#define PITCH_GRAVITY_OFFSET -5.0816f

// What: 定义 Pitch 轴离心项前馈初始系数；Why: 先给小陀螺工况一个保守补偿起点，后续只需围绕这一处做上车标定。
#define PITCH_CENTRIFUGAL_FEEDFORWARD_K 0.05f

// What: 定义 Yaw 轴科氏项前馈初始系数；Why: 复合甩头时先补一层轻量耦合，减少仅靠误差环追赶带来的卡顿。
#define YAW_CORIOLIS_FEEDFORWARD_K 0.06f

// What: 定义前馈角速度低通时间常数；Why: 关节速率直接进入动力学耦合项对噪声很敏感，必须先做轻度滤波抑制抖动。
#define GIMBAL_FEEDFORWARD_RATE_LPF_RC 0.020f
// What: 定义云台前馈周期后备值；Why: DWT 首拍或异常值不能直接拿去更新滤波器，否则会把耦合项瞬间放大。
#define GIMBAL_FEEDFORWARD_DT_FALLBACK 0.005f
// What: 定义 Pitch 前馈总输出限幅；Why: 重力项之外新增耦合项后必须保留硬保护，防止未标定参数直接把扭矩顶满。
#define PITCH_FEEDFORWARD_LIMIT 7.5f
// What: 定义 Yaw 前馈输出限幅；Why: 当前 Yaw 只加动态耦合项，先用更保守的上限保证复合运动不过激。
#define YAW_FEEDFORWARD_LIMIT 3.0f

#define PITCH_MECH_LIMIT_MAX 0.08f // 上极限
#define PITCH_MECH_LIMIT_MIN -0.967f // 下极限
// What: 定义 yaw 速度环积分系数；Why: 小陀螺属于持续扰动场景，只靠 P 项容易留下稳态偏差，因此补一小段 Ki 来慢慢顶住漂移。
#define YAW_SPEED_PID_KI 0.03f
// What: 定义 yaw 速度环静止死区(rad/s)；Why: 陀螺仪静止时也会有零偏和噪声，必须把极小误差吞掉，防止积分攒久后突然抽动。
#define YAW_SPEED_PID_DEADBAND_RAD 0.015f
// What: 定义 yaw 速度环积分限幅；Why: 即使进入积分，也只允许积累少量修正，避免静摩擦被一次性打穿导致云台突跳。
#define YAW_SPEED_PID_INTEGRAL_LIMIT 0.6f
// What: 定义 yaw 速度环变速积分主区间(rad/s)；Why: 误差较大时由 P 项主导，误差较小时才逐步放开积分，减少小陀螺切换和停车瞬间的堆积。
#define YAW_SPEED_PID_COEF_A_RAD 0.18f
// What: 定义 yaw 速度环全积分阈值(rad/s)；Why: 只有误差足够小的时候才允许满积分，用来补静态偏差而不是放大动态扰动。
#define YAW_SPEED_PID_COEF_B_RAD 0.03f

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DMMotorInstance *yaw_motor, *pitch_motor;

static Publisher_t *gimbal_pub; // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub; // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv; // 来自cmd的控制信息
static GimbalCali_Handler_t pitch_cali_handler; // 定义标定句柄

static BMI088Instance *bmi088; // 云台IMU

/**
 * @brief 将角度归一化到 [0, 360)
 *
 * @param angle_deg 原始角度
 * @return float 归一化后的角度
 */
static float NormalizeAngleTo360(float angle_deg)
{
    // What: 统一做 yaw 单圈角度归一化；Why: 电机离线/复活后的机械角输出必须始终落在同一坐标系里，避免不同调用点各写一套逻辑后出现偏差。
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

/**
 * @brief 清空单个 PID 的运行时状态
 *
 * @param pid 需要复位的 PID 实例
 */
static void ResetPIDRuntimeState(PIDInstance *pid)
{
    if (pid == NULL) {
        return;
    }

    // What: 仅清空 PID 的误差、积分和微分历史；Why: 模式切换后最怕沿用旧工况的残留积分，导致云台恢复静止时突然自己扭一下。
    pid->Measure = 0.0f;
    pid->Last_Measure = 0.0f;
    pid->Err = 0.0f;
    pid->Last_Err = 0.0f;
    pid->Last_ITerm = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->ITerm = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->Ref = 0.0f;
    pid->ERRORHandler.ERRORCount = 0u;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
    DWT_GetDeltaT(&pid->DWT_CNT);
}

/**
 * @brief 清空 yaw 电机角度环和速度环的运行时状态
 *
 */
static void ResetYawMotorRuntimeState(void)
{
    // What: 在 yaw 电机掉线/复活边沿统一清空角度环和速度环历史；Why: 电机失能期间反馈与目标会脱钩，残留状态会在重新上电时把头突然拉偏。
    if (yaw_motor == NULL) {
        return;
    }

    ResetPIDRuntimeState(&yaw_motor->angle_PID);
    ResetPIDRuntimeState(&yaw_motor->speed_PID);
}

void GimbalCalibrate(void)
{
    if (yaw_motor == NULL) {
        return;
    }

    // What: 统一通过云台模块向 yaw DM 电机发送硬件零点校准指令；Why: 上层只关心“触发校零”，不应直接拿底层电机实例改零点，避免后续接口再次分叉。
    DMMotorCaliEncoder(yaw_motor);
}

/**
 * @brief 获取云台前馈计算周期
 *
 * @return float 当前周期，单位 s
 */
static float GetGimbalFeedforwardDt(void)
{
    static uint32_t gimbal_ff_dwt_cnt = 0u;
    float dt_s = DWT_GetDeltaT(&gimbal_ff_dwt_cnt);

    // What: 对前馈使用的 dt 做异常钳制；Why: 首拍和调度抖动可能给出异常大周期，直接参与滤波会把速率耦合项污染掉。
    if (dt_s <= 0.0f || dt_s > 0.05f) {
        dt_s = GIMBAL_FEEDFORWARD_DT_FALLBACK;
    }
    return dt_s;
}

/**
 * @brief 一阶低通更新
 *
 * @param last_output 上一拍输出
 * @param input 当前输入
 * @param rc 时间常数
 * @param dt_s 当前周期
 * @return float 本拍输出
 */
static float FirstOrderLowPass(float last_output, float input, float rc, float dt_s)
{
    if (rc <= 0.0f || dt_s <= 0.0f) {
        return input;
    }

    // What: 使用一阶低通平滑关节速率；Why: 动力学前馈要尽量吃到真实运动趋势，但不能把高频噪声直接变成扭矩抖动。
    return input * dt_s / (rc + dt_s) + last_output * rc / (rc + dt_s);
}

/**
 * @brief 对称限幅
 *
 * @param value 原始值
 * @param limit 对称上限
 * @return float 限幅后的结果
 */
static float ClampSymmetric(float value, float limit)
{
    // What: 统一做正负对称限幅；Why: 前馈是辅助项，不允许在任一方向上越过预设安全边界。
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void GimbalInit()
{
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    /* 防御性处理：若 INS_Init 返回 NULL，使用静态默认值避免野指针 */
    static attitude_t _gimbal_default_attitude = { 0 };
    if (gimba_IMU_data == NULL) {
        gimba_IMU_data = &_gimbal_default_attitude;
    }
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x01,
            .rx_id = 0x03,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.67, // 0.71
                .Ki = 0,
                .Kd = 0,

                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ErrorHandle,
                .IntegralLimit = 7,

                .MaxOut = 21,
            },
            .speed_PID = {
                .Kp = 2.1, // 2.1
                .Ki = YAW_SPEED_PID_KI,
                .Kd = 0,
                .DeadBand = YAW_SPEED_PID_DEADBAND_RAD,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate,
                .IntegralLimit = YAW_SPEED_PID_INTEGRAL_LIMIT,
                .CoefA = YAW_SPEED_PID_COEF_A_RAD,
                .CoefB = YAW_SPEED_PID_COEF_B_RAD,
                .MaxOut = 10,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = J8006
    };
    // PITCH  冲坡的时候，由于陀螺仪反馈，容易撞击到下限位
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x14,
            .rx_id = 0x15,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.92,
                .Ki = 0.0,
                .Kd = 0.0,
                .DeadBand = 0.0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 5,
            },
            .speed_PID = {
                // 此处为速度环参数，均为负数
                .Kp = -6.84,
                .Ki = -0.23,
                .Kd = 0, // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5,
                .MaxOut = 15,
            },
            // [轴互换后] Pitch 字段现在就是物理 Pitch 轴数据
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[1]),
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
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DMMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&pitch_config);
    if (pitch_motor) {
        pitch_motor->pos_limit_max = PITCH_MECH_LIMIT_MAX;
        pitch_motor->pos_limit_min = PITCH_MECH_LIMIT_MIN;
    }

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

    /* 如果注册失败，置为 NULL 并在使用处判空 */
    if (gimbal_pub == NULL) {
        /* optional logging */
    }
    if (gimbal_sub == NULL) {
        /* optional logging */
    }

    // [新增] 初始化图传固定电机(M2006, CAN2 ID7)
    // Why: 在云台初始化中统一注册, 确保CAN外设就绪后再初始化电机
    // VideoLinkMotorInit();

    // GimbalCali_Init(&pitch_cali_handler);
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    static gimbal_mode_e last_gimbal_mode = GIMBAL_ZERO_FORCE; // What: 记录上一拍云台模式；Why: 只有在模式切换边沿才需要清空 yaw 速度环积分，避免每拍都把 Ki 的作用抹掉。
    static uint8_t last_yaw_motor_online = 0u; // What: 记录 yaw 电机上一拍在线状态；Why: 只在掉线/复活边沿清一次 PID，避免正常运行时反复抹掉控制状态。
    static float yaw_motor_single_round_cache_deg = 0.0f; // What: 缓存最后一次可信的 yaw 单圈机械角；Why: 电机离线时继续发布这个值，底盘跟随不会被脏反馈带偏。
    static float pitch_ff_storage = 0.0f; // What: 保存 Pitch 电流前馈输出；Why: 驱动侧通过指针异步读取，必须保证引用对象跨周期持续有效。
    static float yaw_ff_storage = 0.0f; // What: 保存 Yaw 电流前馈输出；Why: 让 Yaw 动态耦合补偿可以和 Pitch 一样稳定挂到 DM 前馈接口。
    static float pitch_rate_filtered = 0.0f; // What: 缓存滤波后的 Pitch 关节速率；Why: 科氏耦合直接吃原始速度会更抖，必须跨周期保留滤波状态。
    static float yaw_rate_filtered = 0.0f; // What: 缓存滤波后的 Yaw 关节速率；Why: 离心项对速率平方更敏感，先滤波才能避免噪声被放大。
    uint8_t yaw_motor_online = 0u;
    uint8_t pitch_motor_online = 0u;
    float ff_dt_s = GetGimbalFeedforwardDt();
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    if (gimbal_sub) {
        SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    } else {
        memset(&gimbal_cmd_recv, 0, sizeof(gimbal_cmd_recv));
    }
    if (yaw_motor != NULL && DMMotorIsOnline(yaw_motor) != 0u) {
        yaw_motor_online = 1u;
    }
    if (pitch_motor != NULL && DMMotorIsOnline(pitch_motor) != 0u) {
        pitch_motor_online = 1u;
    }
    if (yaw_motor_online != last_yaw_motor_online) {
        // What: yaw 电机在线状态变化时立即清掉控制残留；Why: 复活重新上电后的第一拍不能继续带着掉线前的历史误差工作。
        ResetYawMotorRuntimeState();
        last_yaw_motor_online = yaw_motor_online;
    }
    if (gimbal_cmd_recv.yaw_pid_reset_request != 0u) {
        if (yaw_motor != NULL) {
            // What: 收到 cmd 侧的小陀螺切换复位请求时清空 yaw 速度环状态；Why: 进入/退出小陀螺只改了 chassis_mode，不会触发 gimbal_mode 边沿，必须靠显式请求来消掉残留积分。
            ResetPIDRuntimeState(&yaw_motor->speed_PID);
            ResetPIDRuntimeState(&yaw_motor->angle_PID);
        }
    }
    if (gimbal_cmd_recv.gimbal_mode != last_gimbal_mode) {
        if (yaw_motor != NULL) {
            // What: 模式切换时清空 yaw 速度环运行时状态；Why: 小陀螺、自由和零力切换后若沿用旧积分，静止下来时很容易突然乱动一下。
            ResetYawMotorRuntimeState();
        }
        last_gimbal_mode = gimbal_cmd_recv.gimbal_mode;
    }

    // ... (省略部分注释代码) ...

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode) {
    // 停止
    case GIMBAL_ZERO_FORCE:
        if (yaw_motor) {
            DMMotorStop(yaw_motor);
        }
        if (pitch_motor)
            DMMotorStop(pitch_motor);
        VideoLinkMotorDisable(); // [新增] 零力模式下停止图传电机; Why: 避免零力模式下电机继续转动
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        if (yaw_motor) {
            DMMotorEnable(yaw_motor);

            // [新增] 应用底盘速度前馈
            // 收到的是底盘真实角速度(deg/s), 赋值给电机速度前馈
            // 注意方向：根据物理模型修正前馈方向
        }
        if (pitch_motor)
            DMMotorEnable(pitch_motor);

        if (yaw_motor)
            DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        if (pitch_motor)
            DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        VideoLinkMotorEnable(); // [新增] 陀螺仪模式下使能图传电机; Why: 云台正常工作时才允许图传电机运动
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        if (yaw_motor) {
            DMMotorEnable(yaw_motor);
            // 自由模式下可能不需要此特定前馈，或者需要根据实际情况决定
            // 暂时关闭前馈以保安全
            yaw_motor->motor_settings.feedforward_flag &= ~SPEED_FEEDFORWARD;
        }
        if (pitch_motor)
            DMMotorEnable(pitch_motor);

        if (yaw_motor)
            DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        if (pitch_motor)
            DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        VideoLinkMotorEnable(); // [新增] 自由模式下使能图传电机; Why: 与陀螺仪模式一致, 云台工作时图传电机应运动
        break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    if (gimbal_cmd_recv.gimbal_mode != GIMBAL_ZERO_FORCE) {
        float pitch_rad = (gimba_IMU_data ? gimba_IMU_data->Pitch : 0.0f) * DEGREE_2_RAD;
        float pitch_rate_rel = (gimba_IMU_data ? gimba_IMU_data->Gyro[1] : 0.0f);
        float yaw_rate_rel = (gimba_IMU_data ? gimba_IMU_data->Gyro[2] : 0.0f);
        float pitch_sin = arm_sin_f32(pitch_rad);
        float pitch_cos = arm_cos_f32(pitch_rad);
        float pitch_coupling = pitch_sin * pitch_cos;
        float gravity_ff;
        float centrifugal_ff;
        float coriolis_ff;

        // What: 优先用关节自身测速作为动力学前馈输入；Why: IMU 角速度更接近绝对运动，直接拿来算耦合项容易把底盘旋转也算进去。
        if (pitch_motor_online != 0u && pitch_motor != NULL) {
            pitch_rate_rel = pitch_motor->measure.velocity;
        }
        if (yaw_motor_online != 0u && yaw_motor != NULL) {
            yaw_rate_rel = yaw_motor->measure.velocity;
        }

        pitch_rate_filtered = FirstOrderLowPass(pitch_rate_filtered,
                                                pitch_rate_rel,
                                                GIMBAL_FEEDFORWARD_RATE_LPF_RC,
                                                ff_dt_s);
        yaw_rate_filtered = FirstOrderLowPass(yaw_rate_filtered,
                                              yaw_rate_rel,
                                              GIMBAL_FEEDFORWARD_RATE_LPF_RC,
                                              ff_dt_s);

        // What: 保留现有 Pitch 重力补偿拟合式；Why: 这是当前已经验证可用的主补偿项，新增耦合项只是在其上做增量补偿。
        gravity_ff = -(PITCH_GRAVITY_COEFFICIENT_K1 * pitch_cos - PITCH_GRAVITY_COEFFICIENT_K2 * pitch_sin) + PITCH_GRAVITY_OFFSET;
        // What: 增加 Pitch 离心项补偿；Why: 底盘或 Yaw 高速旋转时，枪管会上下“发飘”，提前补力矩能减轻 Pitch 跟随滞后。
        centrifugal_ff = PITCH_CENTRIFUGAL_FEEDFORWARD_K * yaw_rate_filtered * yaw_rate_filtered * pitch_coupling;
        // What: 增加 Yaw 科氏耦合项补偿；Why: Yaw 与 Pitch 复合快速运动时会出现转速突变和卡顿，需要给 Yaw 一层动态前馈卸掉误差环压力。
        coriolis_ff = -YAW_CORIOLIS_FEEDFORWARD_K * yaw_rate_filtered * pitch_rate_filtered * pitch_coupling;

        pitch_ff_storage = ClampSymmetric(gravity_ff + centrifugal_ff, PITCH_FEEDFORWARD_LIMIT);
        yaw_ff_storage = ClampSymmetric(coriolis_ff, YAW_FEEDFORWARD_LIMIT);

        if (pitch_motor != NULL) {
            // What: 每拍重绑 Pitch 前馈指针；Why: 保持驱动端始终读取最新的静态存储，并在电机离线时立即撤掉前馈标志。
            pitch_motor->current_feedforward_ptr = &pitch_ff_storage;
            if (pitch_motor_online != 0u) {
                pitch_motor->motor_settings.feedforward_flag |= CURRENT_FEEDFORWARD;
            } else {
                pitch_motor->motor_settings.feedforward_flag &= ~CURRENT_FEEDFORWARD;
            }
        }
        if (yaw_motor != NULL) {
            // What: 为 Yaw 挂载电流前馈；Why: 让复合运动时的耦合补偿直接进入最内层力矩通道，而不是继续堆给位置/速度环硬抗。
            yaw_motor->current_feedforward_ptr = &yaw_ff_storage;
            if (yaw_motor_online != 0u) {
                yaw_motor->motor_settings.feedforward_flag |= CURRENT_FEEDFORWARD;
            } else {
                yaw_motor->motor_settings.feedforward_flag &= ~CURRENT_FEEDFORWARD;
            }
        }
    } else {
        // What: 零力模式下同步清空前馈输出和滤波状态；Why: 退出失能后若沿用旧动态项，恢复第一拍会带着过期力矩直接冲击机构。
        pitch_ff_storage = 0.0f;
        yaw_ff_storage = 0.0f;
        pitch_rate_filtered = 0.0f;
        yaw_rate_filtered = 0.0f;
        if (pitch_motor) {
            pitch_motor->motor_settings.feedforward_flag &= ~CURRENT_FEEDFORWARD;
        }
        if (yaw_motor) {
            yaw_motor->motor_settings.feedforward_flag &= ~CURRENT_FEEDFORWARD;
        }
    }

    // [新增] 每周期执行图传电机状态机
    // Why: 状态机需要周期性检测堵转并切换状态, 放在重力补偿之后保证电机控制逻辑的完整执行
    // VideoLinkMotorTask();

    // 设置反馈数据,主要是imu和yaw的ecd
    if (yaw_motor_online != 0u && yaw_motor != NULL) {
        float yaw_rad = yaw_motor->measure.position; // What: 直接读取 yaw 电机当前帧的单圈硬件位置；Why: 固定绝对编码值方案要避开 total_round 复活误判，不能再把多圈累计量混进 offset。
        yaw_motor_single_round_cache_deg = NormalizeAngleTo360(yaw_rad * RAD_2_DEGREE); // What: 将单圈硬件位置统一换算到 0~360 度；Why: 固定对正角 YAW_CHASSIS_ALIGN_DEG 也是 0~360 度坐标，二者必须落在同一坐标系里比较。
    }
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor_single_round_cache_deg;
    gimbal_feedback_data.yaw_motor_online = yaw_motor_online;
    /* 防御性拷贝 IMU 数据 */
    if (gimba_IMU_data)
        gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    else
        memset(&gimbal_feedback_data.gimbal_imu_data, 0, sizeof(gimbal_feedback_data.gimbal_imu_data));

    // 推送消息
    if (gimbal_pub) {
        PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
    }
}
