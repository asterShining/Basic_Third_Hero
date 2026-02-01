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

// gimbal.c 顶部宏定义区域
// 更新为拟合结果
#define PITCH_GRAVITY_COEFFICIENT_K1 -15.2383f
#define PITCH_GRAVITY_COEFFICIENT_K2 -0.2070f
// 新增 Offset 宏 (注意保留负号)
#define PITCH_GRAVITY_OFFSET -16.1574f
static attitude_t *gimba_IMU_data; // 云台IMU数据
static DMMotorInstance *yaw_motor, *pitch_motor;

static Publisher_t *gimbal_pub; // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub; // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv; // 来自cmd的控制信息
static GimbalCali_Handler_t pitch_cali_handler; // 定义标定句柄

static BMI088Instance *bmi088; // 云台IMU
void GimbalCalibrateYaw()
{
    if (yaw_motor != NULL) {
        DMMotorCaliEncoder(yaw_motor);
    }
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
                .Kp = 0.72, // 0.52
                .Ki = 0,
                .Kd = 0,

                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ErrorHandle,
                .IntegralLimit = 7,

                .MaxOut = 20,
            },
            .speed_PID = {
                .Kp = 2.1, // 1.2
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3,
                .MaxOut = 5,
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
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x05,
            .rx_id = 0x06,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.0,
                .Ki = 0.0,
                .Kd = 0.0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 5,
            },
            .speed_PID = {
                .Kp = 0.0,
                .Ki = 0.0,
                .Kd = 0, // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5,
                .MaxOut = 7,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = J4310,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    // yaw_motor = DMMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

    /* 如果注册失败，置为 NULL 并在使用处判空 */
    if (gimbal_pub == NULL) {
        /* optional logging */
    }
    if (gimbal_sub == NULL) {
        /* optional logging */
    }

    GimbalCali_Init(&pitch_cali_handler);
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
    if (gimbal_cmd_recv.gimbal_mode == GIMBAL_CALI_MODE) {
        // 如果当前是空闲状态，则开始标定
        if (pitch_cali_handler.state == CALI_STATE_IDLE) {
            GimbalCali_Start(&pitch_cali_handler);
        }
    }

    // 获取当前Roll角度 (根据你的描述，Pitch轴对应IMU的Roll)
    float current_imu_roll = (gimba_IMU_data ? gimba_IMU_data->Roll : 0.0f);

    // 调用更新函数，如果正在标定(返回1)，则跳过后面的正常控制逻辑
    if (GimbalCali_Update(&pitch_cali_handler, pitch_motor, current_imu_roll)) {
        // 正在标定中...
        // 此时不要执行下面的 switch(gimbal_mode) 逻辑，防止冲突
        // 也不要推送反馈消息，或者仅推送标定状态
        osDelay(2);
        return;
    }

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode) {
    // 停止
    case GIMBAL_ZERO_FORCE:
        if (yaw_motor)
            DMMotorStop(yaw_motor);
        if (pitch_motor)
            DMMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        if (yaw_motor)
            DMMotorEnable(yaw_motor);
        if (pitch_motor)
            DMMotorEnable(pitch_motor);

        if (yaw_motor)
            DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        if (pitch_motor)
            DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        if (yaw_motor)
            DMMotorEnable(yaw_motor);
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
    // ...
    if (gimbal_cmd_recv.gimbal_mode != GIMBAL_ZERO_FORCE) {
        float k1_val, k2_val;

        k1_val = PITCH_GRAVITY_COEFFICIENT_K1;
        k2_val = PITCH_GRAVITY_COEFFICIENT_K2;

        // B. 获取当前 Pitch 角度 (弧度制)
        // 务必确认 gimba_IMU_data->Roll 对应的是 Pitch 轴的物理运动
        float pitch_rad = (gimba_IMU_data ? gimba_IMU_data->Roll : 0.0f) * DEGREE_2_RAD;

        // C. 计算补偿力矩
        // 公式: T_motor = -T_gravity = -(K1*cos + K2*sin)
        // 物理含义:
        //  K1*cos: 抵消主重力矩 (重心在水平轴上的分量)
        //  K2*sin: 抵消重心偏移带来的非正弦畸变
        // 公式变形以匹配拟合模型: T = -k1*cos + k2*sin + offset
        // 注意：原代码是 -(k1*cos - k2*sin) = -k1*cos + k2*sin，正好匹配前两项
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
    // 1. 获取 Yaw 电机当前的弧度值 (DM电机反馈的是弧度)
    float yaw_rad = 0.0f;
    if (yaw_motor) {
        yaw_rad = yaw_motor->measure.position;
    }

    // 2. 将弧度转换为角度 ( 1 rad ≈ 57.3 deg )
    float yaw_deg = yaw_rad * RAD_2_DEGREE;

    // 3. 将角度归一化到 0 ~ 360 度 (对应单圈角度)
    // DM电机的 position 可能是多圈的 (例如 720度, -50度等)，我们需要把它变成 0-360
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
