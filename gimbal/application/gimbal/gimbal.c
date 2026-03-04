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

#define PITCH_MECH_LIMIT_MAX 0.08f // 上极限
#define PITCH_MECH_LIMIT_MIN -0.967f // 下极限

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DMMotorInstance *yaw_motor, *pitch_motor;

static Publisher_t *gimbal_pub; // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub; // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv; // 来自cmd的控制信息
static GimbalCali_Handler_t pitch_cali_handler; // 定义标定句柄

static BMI088Instance *bmi088; // 云台IMU
void GimbalCalibrate()
{
    if (yaw_motor != NULL) {
        DMMotorCaliEncoder(yaw_motor);
    }
    // if (pitch_motor != NULL) {
    //     DMMotorCaliEncoder(pitch_motor);
    // }
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
                .Kp = 0.71, // 0.71
                .Ki = 0,
                .Kd = 0,

                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ErrorHandle,
                .IntegralLimit = 7,

                .MaxOut = 21,
            },
            .speed_PID = {
                .Kp = 2.4, // 2.1
                .Ki = 0.0, // 0.1 //最好增加速度环ki,小陀螺的时候可以抑制云台偏移
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3,
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
            .tx_id = 0x05,
            .rx_id = 0x06,
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

    // GimbalCali_Init(&pitch_cali_handler);
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    if (gimbal_sub) {
        SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    } else {
        memset(&gimbal_cmd_recv, 0, sizeof(gimbal_cmd_recv));
    }
    // [新增] 静态变量: 用于存储前馈值 (必须是static，因为指针会被传递给电机驱动)
    static float pitch_ff_storage = 0.0f;
    static float yaw_ff_storage = 0.0f; // [新增] Yaw轴前馈存储

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
        break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    if (gimbal_cmd_recv.gimbal_mode != GIMBAL_ZERO_FORCE) {
        float k1_val, k2_val;

        k1_val = PITCH_GRAVITY_COEFFICIENT_K1;
        k2_val = PITCH_GRAVITY_COEFFICIENT_K2;

        // B. 获取当前 Pitch 角度 (弧度制)
        // [轴互换后] Pitch 字段直接对应物理 Pitch 轴
        float pitch_rad = (gimba_IMU_data ? gimba_IMU_data->Pitch : 0.0f) * DEGREE_2_RAD;

        // C. 计算补偿力矩
        // 匹配拟合模型: T = -k1*cos + k2*sin + offset
        float gravity_ff = -(k1_val * arm_cos_f32(pitch_rad) - k2_val * arm_sin_f32(pitch_rad)) + PITCH_GRAVITY_OFFSET;

        // D. 应用前馈
        pitch_ff_storage = gravity_ff; // 更新静态变量
        if (pitch_motor) {
            pitch_motor->current_feedforward_ptr = &pitch_ff_storage; // 更新指针 (防防御性编程，尽管Init时可能已赋值)
            pitch_motor->motor_settings.feedforward_flag |= CURRENT_FEEDFORWARD; // 开启前馈标志位
        }

    } else {
        // 停止模式下清除前馈，防止切回时突变
        if (pitch_motor) {
            pitch_motor->motor_settings.feedforward_flag &= ~CURRENT_FEEDFORWARD;
        }
        pitch_ff_storage = 0.0f;
    }

    // 设置反馈数据,主要是imu和yaw的ecd
    // 1. 获取 Yaw 电机当前的连续累计弧度值 (解决 ±12.5 rad 跳变与 2PI 不匹配的问题)
    float yaw_rad = 0.0f;
    if (yaw_motor) {
        yaw_rad = yaw_motor->measure.total_angle; // 使用 total_angle 替代 position，避免多圈溢出问题
    }

    // 2. 将弧度转换为角度 ( 1 rad ≈ 57.3 deg )
    float yaw_deg = yaw_rad * RAD_2_DEGREE;

    // 3. 将角度归一化到 0 ~ 360 度 (对应单圈角度)
    while (yaw_deg < 0.0f)
        yaw_deg += 360.0f;
    while (yaw_deg >= 360.0f)
        yaw_deg -= 360.0f;

    // 4. 赋值给反馈数据
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_deg;
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
