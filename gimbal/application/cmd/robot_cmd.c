// app
#include "can.h"
#include "robot_def.h"
#include "robot_cmd.h"
#include "gimbal.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
#include "buzzer.h"
#include "remote_control.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include <stdint.h>
#include <stdbool.h>

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

#define RC_TRIGGER_TH 500

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信

#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub; // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data; // 视觉发送数据

static Publisher_t *gimbal_cmd_pub; // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub; // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send; // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub; // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub; // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send; // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态
static BuzzzerInstance *hint_buzzer;

#define RC_DEADZONE 10.0f // 遥控器摇杆死区阈值

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
// 定义一个静态变量来保存上一次的开关状态，初始化为下（急停/停止状态）
static uint16_t last_switch_right = RC_SW_DOWN;

// --- 新增的静态变量，用于长按计时 ---
static uint32_t inner_eight_cnt = 0; // 内八计时器
static uint32_t outer_eight_cnt = 0; // 外八计时器
static uint8_t cali_triggered = 0; // 触发状态：0-无，1-内八触发，2-外八触发
void RobotCMDInit()
{
    // BMI088_Init_Config_s bmi088_config = {
    //     .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
    //     .work_mode = BMI088_BLOCK_TRIGGER_MODE,
    //     .spi_acc_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOA,
    //         .cs_pin = GPIO_PIN_4,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .acc_int_config = {
    //         .GPIOx = GPIOC,
    //         .GPIO_Pin = GPIO_PIN_4,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .spi_gyro_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOB,
    //         .cs_pin = GPIO_PIN_0,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .gyro_int_config = {
    //         .GPIO_Pin = GPIO_PIN_5,
    //         .GPIOx = GPIOC,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .heat_pwm_config = {
    //         .htim = &htim10,
    //         .channel = TIM_CHANNEL_1,
    //         .period = 1,
    //     },
    //     .heat_pid_config = {
    //         .Kp = 0.5,
    //         .Ki = 0,
    //         .Kd = 0,
    //         .DeadBand = 0.1,
    //         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //         .IntegralLimit = 100,
    //         .MaxOut = 100,
    //     },
    // };
    // bmi088_test = BMI088Register(&bmi088_config);
    rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    // vision_recv_data = VisionInit(&huart1); // 视觉通信串口
    Buzzer_config_s hint_config = {
        .alarm_level = ALARM_LEVEL_MEDIUM, // 优先级
        .octave = OCTAVE_5, // 音调 (SoFreq)
        .loudness = 0.5f, // 音量 (0.0 ~ 1.0)
    };
    hint_buzzer = BuzzerRegister(&hint_config);

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x012,
            .rx_id = 0x011,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .daemon_count = 200,
    };
    cmd_can_comm = CANCommInit(&comm_conf);
    LOGINFO("[can_comm] Chassis_Upload_Data_s size: %d", sizeof(Chassis_Upload_Data_s));
    LOGINFO("[can_comm] Chassis_Ctrl_Cmd_s size: %d", sizeof(Chassis_Ctrl_Cmd_s));
    LOGINFO("[can_comm] CAN_COMM_MAX_BUFFSIZE: %d", CAN_COMM_MAX_BUFFSIZE);
    // 【新增调试日志】
    if (cmd_can_comm != NULL) {
        LOGINFO("[GIMBAL_DEBUG] CAN Comm Init Success! TxID: %d, RxID: %d", cmd_can_comm->can_ins->tx_id, cmd_can_comm->can_ins->rx_id);
    } else {
        LOGERROR("[GIMBAL_DEBUG] CAN Comm Init Failed!");
    }
#endif // GIMBAL_BOARD
    // gimbal_cmd_send.pitch = 0;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
// static void CalcOffsetAngle()
// {
//     // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
//     static float angle;
//     angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
// #if YAW_ECD_GREATER_THAN_4096 // 如果大于180度
//     if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else if (angle > 180.0f + YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
//     else
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
// #else // 小于180度
//     if (angle > YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
// #endif
// }

static void CalcOffsetAngle()
{
    // 获取你在 gimbal.c 中计算出的 0~360 度角度
    float gimbal_angle = gimbal_fetch_data.yaw_motor_single_round_angle;

    // DM电机通常上电归零，所以偏移量设为0；如果需要机械对齐，可修改 YAW_ALIGN_ANGLE
    float align_offset = 0.0f;
    // 或者保留宏定义： float align_offset = YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI; (需确保宏转换正确)

    // 1. 计算原始偏差
    float error = gimbal_angle - align_offset;

    // 2. 归一化到 0~360
    while (error < 0.0f)
        error += 360.0f;
    while (error >= 360.0f)
        error -= 360.0f;

    // 3. 转换为 -180 ~ +180 范围 (最短路径逻辑)
    if (error > 180.0f) {
        chassis_cmd_send.offset_angle = error - 360.0f; // 例如 350 -> -10
    } else {
        chassis_cmd_send.offset_angle = error; // 例如 10 -> 10
    }
}
/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    // if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    // {
    robot_state = ROBOT_STOP;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    // }
    // 遥控器右侧开关为[上],恢复正常运行
    // if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    //     robot_state = ROBOT_READY;
    //     shoot_cmd_send.shoot_mode = SHOOT_ON;
    //     LOGINFO("[CMD] reinstate, robot ready");
    // // }
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    // 1. 获取当前开关状态
    uint16_t current_switch_right = rc_data[TEMP].rc.switch_right;
    uint16_t current_switch_left = rc_data[TEMP].rc.switch_left;

    // --- 状态机逻辑 ---

    // [下] 急停模式
    if (switch_is_down(current_switch_left)) {
        // --- 杆位判定 ---
        // 内八：左摇杆(右下)，右摇杆(左下) -> ↘ ↙
        bool is_inner_eight = (rc_data[TEMP].rc.rocker_l_ > RC_TRIGGER_TH) &&
                              (rc_data[TEMP].rc.rocker_l1 < -RC_TRIGGER_TH) &&
                              (rc_data[TEMP].rc.rocker_r_ < -RC_TRIGGER_TH) &&
                              (rc_data[TEMP].rc.rocker_r1 < -RC_TRIGGER_TH);

        // 外八：左摇杆(左下)，右摇杆(右下) -> ↙ ↘
        bool is_outer_eight = (rc_data[TEMP].rc.rocker_l_ < -RC_TRIGGER_TH) &&
                              (rc_data[TEMP].rc.rocker_l1 < -RC_TRIGGER_TH) &&
                              (rc_data[TEMP].rc.rocker_r_ > RC_TRIGGER_TH) &&
                              (rc_data[TEMP].rc.rocker_r1 < -RC_TRIGGER_TH);

        // --- 逻辑 1：内八 (2.5秒) -> 校准电机零点 ---
        if (is_inner_eight) {
            if (cali_triggered == 0) { // 只有在未触发状态下才计时
                inner_eight_cnt++;
                if (inner_eight_cnt >= 500) { // 200Hz * 2.5s = 500次
                    // 1. 执行电机校准
                    GimbalCalibrateYaw();

                    // 2. 蜂鸣器提示 (高音 Octave 6)
                    if (hint_buzzer != NULL) {
                        hint_buzzer->octave = OCTAVE_6; // 设置为高音
                        AlarmSetStatus(hint_buzzer, ALARM_ON);
                    }

                    cali_triggered = 1; // 标记为内八已触发，防止重复执行
                }
            }
        } else {
            inner_eight_cnt = 0; // 只要摇杆没有保持住，计时器立马清零
        }

        // --- 逻辑 2：外八 (2.5秒) -> 校准陀螺仪 ---
        if (is_outer_eight) {
            if (cali_triggered == 0) {
                outer_eight_cnt++;
                if (outer_eight_cnt >= 500) { // 200Hz * 2.5s = 500次
                    if (hint_buzzer != NULL) {
                        hint_buzzer->octave = OCTAVE_4; // 设置为中音
                        AlarmSetStatus(hint_buzzer, ALARM_ON);
                    }

                    // 1. 执行陀螺仪校准 (调用 INS_Init 重新初始化姿态)
                    chassis_cmd_send.calibrate_imu = 1;
                    INS_Calibrate();

                    if (hint_buzzer != NULL) {
                        AlarmSetStatus(hint_buzzer, ALARM_OFF);
                        // 【关键】手动刷新一次任务，确保蜂鸣器立即停止
                    }

                    cali_triggered = 2; // 标记为外八已触发
                }
            }
        } else {
            outer_eight_cnt = 0; // 摇杆松开清零
        }

        // --- 逻辑 3：复位 ---
        // 当摇杆都回中(或不满足条件) 且 之前处于触发状态时，关闭蜂鸣器
        if (!is_inner_eight && !is_outer_eight && cali_triggered != 0) {
            if (hint_buzzer != NULL) {
                AlarmSetStatus(hint_buzzer, ALARM_OFF);
            }
            cali_triggered = 0; // 重置触发标志，允许下一次触发
        }
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
    }
    // [上] 底盘无力，云台能够转动
    else if (switch_is_up(current_switch_right)) {
        // 如果是从[下]或其他模式刚刚切换到[中]
        if (!switch_is_up(last_switch_right)) {
            // 无扰切换：将目标角度重置为当前实际角度
            gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
            gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
        }

        robot_state = ROBOT_READY;
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }
    // [中] 底盘跟随云台模式
    else if (switch_is_mid(current_switch_right)) {
        // 核心修改：检测是否刚刚切入[上]档位
        if (!switch_is_mid(last_switch_right)) {
            // 【无扰切换执行】
            // 将云台控制的"目标值"强行设定为当前的"反馈值"
            // 这样PID的误差(Error)在这一瞬间为0，避免云台疯转
            gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
            gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
        }

        robot_state = ROBOT_READY;
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }

    // --- 1. 提取原始数据并转为浮点数 ---
    float rocker_lx = (float)rc_data[TEMP].rc.rocker_l_; // 左摇杆 X (云台Yaw)
    float rocker_ly = (float)rc_data[TEMP].rc.rocker_l1; // 左摇杆 Y (云台Pitch)
    float rocker_rx = (float)rc_data[TEMP].rc.rocker_r_; // 右摇杆 X (底盘左右)
    float rocker_ry = (float)rc_data[TEMP].rc.rocker_r1; // 右摇杆 Y (底盘前后)

    // --- 2. 死区处理逻辑 ---
    // 如果数值在 -RC_DEADZONE 到 +RC_DEADZONE 之间，则强制归零
    if (rocker_lx > -RC_DEADZONE && rocker_lx < RC_DEADZONE)
        rocker_lx = 0;
    if (rocker_ly > -RC_DEADZONE && rocker_ly < RC_DEADZONE)
        rocker_ly = 0;
    if (rocker_rx > -RC_DEADZONE && rocker_rx < RC_DEADZONE)
        rocker_rx = 0;
    if (rocker_ry > -RC_DEADZONE && rocker_ry < RC_DEADZONE)
        rocker_ry = 0;

    // --- 3. 使用过滤后的数据进行控制 ---

    // 云台控制量计算 (仅在非急停状态下累加)
    if (!switch_is_down(current_switch_right)) {
        // 使用处理后的 rocker_lx 和 rocker_ly
        gimbal_cmd_send.yaw -= 0.001f * rocker_lx;
        gimbal_cmd_send.pitch += 0.0001f * rocker_ly;
    }

    // 底盘参数
    // 右手系 x正向前进 y正向右移
    // 使用处理后的 rocker_rx 和 rocker_ry
    chassis_cmd_send.vy = 10.0f * rocker_rx; // _水平方向
    chassis_cmd_send.vx = 10.0f * rocker_ry; // 1数值方向

    // 发射参数
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],弹舱打开
        ; // 弹舱舵机控制,待添加servo_motor模块,开启
    else {
        {
            // 弹舱舵机控制,待添加servo_motor模块,关闭
        };
    }
    if (rc_data[TEMP].rc.dial < -200) // 向上超过100,打开摩擦轮
        chassis_cmd_send.gimbal_cmd_wz = 4000;
    else if (rc_data[TEMP].rc.dial > 200)
        chassis_cmd_send.gimbal_cmd_wz = -4000;
    else
        chassis_cmd_send.gimbal_cmd_wz = 0;

    // // 摩擦轮控制,拨轮向上打为负,向下为正
    // if (rc_data[TEMP].rc.dial < -100) // 向上超过100,打开摩擦轮
    //     shoot_cmd_send.friction_mode = FRICTION_ON;
    // else
    //     shoot_cmd_send.friction_mode = FRICTION_OFF;
    // // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    // if (rc_data[TEMP].rc.dial < -500)
    //     shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    // else
    //     shoot_cmd_send.load_mode = LOAD_STOP;
    // // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    // shoot_cmd_send.shoot_rate = 8;

    last_switch_right = current_switch_right; // 更新上一次开关状态
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    // 如果觉得键盘太快，可以乘以 0.5f (半速)
    float key_scale = 1.0f;

    // W-S 控制前后，A-D 控制左右
    chassis_cmd_send.vx = (rc_data[TEMP].key[KEY_PRESS].w - rc_data[TEMP].key[KEY_PRESS].s) * key_scale;
    chassis_cmd_send.vy = (rc_data[TEMP].key[KEY_PRESS].a - rc_data[TEMP].key[KEY_PRESS].d) * key_scale;

    gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
    {
    case 0:
        shoot_cmd_send.bullet_speed = 15;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = 18;
        break;
    default:
        shoot_cmd_send.bullet_speed = 30;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4) // E键设置发射模式
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    case 1:
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        break;
    case 2:
        shoot_cmd_send.load_mode = LOAD_3_BULLET;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    default:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 40;
        break;
    case 1:
        chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:

        break;

    default:

        break;
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    // if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
    RemoteControlSet();
    // else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
    //     MouseKeySet();

    // EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
    chassis_cmd_send.gimbal_gyro_z = gimbal_fetch_data.gimbal_imu_data.Gyro[2] * RAD_2_DEGREE; // 假设原始是弧度，转成度
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
