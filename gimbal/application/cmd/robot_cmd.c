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
#include "auto_gimbal.h"
#include "std_cmd.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include <stdint.h>
#include <stdbool.h>

#define LOAD_TRIGGER_DELAY 500

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

#define RC_TRIGGER_TH 500
// Yaw 死区漂移低通锁定系数: 0.90~0.98，值越小锁定越快，0.95 约 0.5s 平滑锁定
#define YAW_DRIFT_LOCK_COEF 0.95f

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
// What: 编译期校验底盘反馈结构体尺寸；Why: 防止新增键鼠字段后悄悄超过单帧CAN通信上限
_Static_assert(sizeof(Chassis_Upload_Data_s) <= CAN_COMM_MAX_BUFFSIZE,
               "Chassis_Upload_Data_s exceeds CAN_COMM_MAX_BUFFSIZE");
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

#define RC_DEADZONE 5.0f // 遥控器摇杆死区阈值

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
// 定义一个静态变量来保存上一次的开关状态，初始化为下（急停/停止状态）
static uint16_t last_switch_right = RC_SW_DOWN;
// 记录当前用于底盘跟随的 Yaw 对齐基准角。
// 当前设计默认信任 DM 已保存的硬件零点，因此开机直接以 0 度作为软件对齐基准。
static float yaw_align_offset_deg = 0.0f;

// --- 新增的静态变量，用于长按计时 ---
static uint32_t inner_eight_cnt = 0; // 内八计时器
static uint32_t outer_eight_cnt = 0; // 外八计时器

static uint8_t cali_triggered = 0; // 触发状态：0-无，1-内八触发，2-外八触发
// [新增] 自瞄状态变量
static AutoAim_State_e auto_aim_state = AUTO_AIM_IDLE;

void RobotCMDInit()
{
    // 开机直接按 DM 硬件零点工作，不回退到机械安装角。
    yaw_align_offset_deg = 0.0f;
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

    // 使用当前生效的对齐基准角计算 offset。
    // 默认信任 DM 硬件零点，因此软件基准开机就是 0；手动校零后仍保持围绕 0 闭环。
    float align_offset = yaw_align_offset_deg;

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
    shoot_cmd_send.friction_mode = FRICTION_OFF;
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

    float current_real_pitch = gimbal_fetch_data.gimbal_imu_data.Pitch; // [轴互换后] Pitch 字段即物理 Pitch

    // --- 状态机逻辑 ---

    // [下] 急停模式
    if (switch_is_down(current_switch_right)) {
        EmergencyHandler();
        static uint8_t cali_triggered = 0;
        bool is_inner_eight = (rc_data[TEMP].rc.rocker_l_ > RC_TRIGGER_TH) && // 左摇杆向右
                              (rc_data[TEMP].rc.rocker_l1 < -RC_TRIGGER_TH) && // 左摇杆向下
                              (rc_data[TEMP].rc.rocker_r_ < -RC_TRIGGER_TH) && // 右摇杆向左
                              (rc_data[TEMP].rc.rocker_r1 < -RC_TRIGGER_TH); // 右摇杆向下

        if (is_inner_eight) {
            if (cali_triggered == 0) {
                // 1. 调用校准
                GimbalCalibrate();
                // 手动 DM 校零后，将底盘跟随基准切到运行时零位。
                // 原因是当前 yaw_motor_single_round_angle 会围绕 0 反馈，继续减旧机械角会让校零看起来“没生效”。
                yaw_align_offset_deg = 0.0f;
                // 同步刷新云台目标到当前 IMU 姿态，避免退出急停后沿用旧参考值导致瞬时回拉。
                gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
                gimbal_cmd_send.pitch = current_real_pitch;

                // 2. 【新增】开启蜂鸣器提示
                if (hint_buzzer != NULL) {
                    AlarmSetStatus(hint_buzzer, ALARM_ON);
                }

                cali_triggered = 1;
            }
        } else {
            // 摇杆回中后，关闭蜂鸣器并重置触发位
            if (cali_triggered == 1) {
                if (hint_buzzer != NULL) {
                    AlarmSetStatus(hint_buzzer, ALARM_OFF);
                }
                cali_triggered = 0;
            }
        }
    }
    // [上] 底盘无力，云台能够转动
    else if (switch_is_up(current_switch_right)) {
        // 无扰切换判断
        if (!switch_is_up(last_switch_right)) {
            if (!switch_is_up(last_switch_right)) {
                gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
            }
        }

        robot_state = ROBOT_READY;
        // [用户要求] 注释掉原有逻辑 (User request: Comment out original logic)
        // chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        // gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;

        // [新增] 小陀螺模式配置 (New Configuration: Little Top Mode)
        // 这里明确下发 CHASSIS_ROTATE，作用是让底盘侧进入小陀螺分支。
        // 之前该行被注释后，发送出去的一直是 CHASSIS_NO_FOLLOW，所以底盘永远不会自旋。
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        // 云台切换至陀螺仪模式以保持世界坐标系下的稳定瞄准
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

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
        float yaw_sensitivity = 0.001f;
        float pitch_sensitivity = 0.0003f;

        // [新增] 拨轮微调模式
        // 当左侧拨轮向下拨动超过 100 时，进入微调模式 (灵敏度降低)
        if (rc_data[TEMP].rc.dial > 100) {
            yaw_sensitivity *= 0.3f; // 降低 YAW 灵敏度至 30%
            pitch_sensitivity *= 0.3f; // 降低 PITCH 灵敏度至 30%
        }

        // 使用处理后的 rocker_lx 和 rocker_ly
        gimbal_cmd_send.yaw -= yaw_sensitivity * rocker_lx;
        gimbal_cmd_send.pitch += pitch_sensitivity * rocker_ly;

        // ==================== [新增] Yaw 死区零偏漂移限位 ====================
        // 功能: 当摇杆 X 轴在死区内 (rocker_lx == 0) 时，陀螺仪零偏会让 YawTotalAngle
        //       持续缓慢变化，但 gimbal_cmd_send.yaw（目标值）却保持不动，
        //       导致 PID 误差不断积累，云台被强迫追赶漂移，表现为"飘"。
        // 原理: 在无输入期间，将目标值以低通方式软锁定到当前 IMU 反馈值，
        //       使目标值跟随真实漂移，消除误差累积。
        //       低通系数 YAW_DRIFT_LOCK_COEF 越小，锁定越快（0.0 = 立即锁定，会突变）；
        //       设为 0.95 可让云台在放手后约 0.5s 内平滑锁定到当前姿态，无扰动感。
        if (rocker_lx == 0.0f) {
            // 将目标值向当前 IMU 真实 yaw 缓慢拉拢，防止零偏积分积累误差
            gimbal_cmd_send.yaw = YAW_DRIFT_LOCK_COEF * gimbal_cmd_send.yaw +
                                  (1.0f - YAW_DRIFT_LOCK_COEF) * gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        }

        // ==================== [新增] 软件限幅逻辑 ====================

        // 1. Pitch 轴限幅 (最重要，防止撞击)
        if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE) {
            gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
        } else if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE) {
            gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
        }
    }

    // ==========================================
    // [新增] 自瞄逻辑集成
    // ==========================================
    // 触发条件: 右拨杆为[上] (GIMBAL_GYRO_MODE)
    // 且 用户按下了自瞄按键 (这里假设是 PC端的鼠标右键 或者 遥控器的某个组合键，目前默认在 GYRO_MODE 下常开自瞄检测)
    // 或者，我们可以定义一个特定的开关逻辑

    // // 修正: 只有在 GIMBAL_GYRO_MODE 下才允许自瞄介入
    if (switch_is_up(current_switch_right)) {
        if (gimbal_cmd_send.gimbal_mode == GIMBAL_GYRO_MODE) {
            // 1. 获取当前云台反馈
            // 注意: 我们在 robot_def.h 中定义了 VISION_YAW_AXIS 和 VISION_PITCH_AXIS
            // 确保使用宏定义的轴向, 保持逻辑统一
            float current_yaw_total = gimbal_fetch_data.gimbal_imu_data.VISION_YAW_AXIS * VISION_YAW_SIGN;

            // [轴互换后] Pitch 字段直接对应物理 Pitch 轴
            float current_pitch = gimbal_fetch_data.gimbal_imu_data.VISION_PITCH_AXIS * VISION_PITCH_SIGN;

            // 2. 运行自瞄逻辑
            // 如果识别到目标，cmd_yaw/pitch 会被更新为目标值
            // 如果未识别到，cmd_yaw/pitch 保持 RemoteControl 计算出的手动值
            auto_aim_state = AutoGimbalRun(
                vision_recv_data,
                current_yaw_total,
                current_pitch,
                &gimbal_cmd_send.yaw,
                &gimbal_cmd_send.pitch);

            // 如果进入自瞄跟踪状态，可以覆盖底盘模式为跟随云台 (可选)
            if (auto_aim_state == AUTO_AIM_TRACKING) {
                // chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
                // (保持小陀螺或跟随，视 tactical 需求而定，暂不强制修改底盘模式)
            }
        } else {
            auto_aim_state = AUTO_AIM_IDLE;
        }
    }

    // 底盘参数
    // 右手系 x正向前进 y正向右移
    // 使用处理后的 rocker_rx 和 rocker_ry
    // [修复] 将遥控器摇杆值(-660~+660)正确映射为底盘速度(m/s)单位
    // 归一化公式: (rocker_value / 660.0f) * MAX_SPEED
    // 最大平地速度由 robot_def.h 中的 MAX_CHASSIS_VX/VY_SPEED 定义 (默认 6.0 m/s)
    // chassis_cmd_send.vx = (rocker_ry / 660.0f) * MAX_CHASSIS_VX_SPEED; // 前后速度 (m/s)
    // chassis_cmd_send.vy = (rocker_rx / 660.0f) * MAX_CHASSIS_VY_SPEED; // 左右速度 (m/s)
    chassis_cmd_send.vx = 10.0f * rocker_ry; // 竖直方向,发送给vx
    chassis_cmd_send.vy = 10.0f * rocker_rx; // 水平方向

    // 发射参数
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],弹舱打开
        ; // 弹舱舵机控制,待添加servo_motor模块,开启
    else {
        {
            // 弹舱舵机控制,待添加servo_motor模块,关闭
        };
    }

    static uint8_t friction_switch_state = 0; // 0-关, 1-开
    static uint16_t last_switch_left = RC_SW_DOWN;

    // 【修改点1】定义默认发射模式
    // 0:单发, 1:二连发, 2:连发
    // 您可以在这里修改初始值，或者通过键盘 Key_E 来修改它
    static uint8_t fire_mode_state = 0;

// [新增] 连发射频
#define BURST_FIRE_RATE 8.0f // 连发每秒8发，可自行调整

    // ==============================================================
    // [新增] 安全逻辑: 仅在非急停状态下允许开启摩擦轮和射击
    // ==============================================================
    if (!switch_is_down(current_switch_right)) {
        // ==============================================================
        // 逻辑 A: 摩擦轮开关控制 (左拨杆 中 -> 上)
        // ==============================================================
        // 检测从【中】拨到【上】的瞬间 (上升沿)
        if (switch_is_up(current_switch_left) && switch_is_mid(last_switch_left)) {
            // 无论当前是什么模式，直接取反状态
            friction_switch_state = !friction_switch_state;

            // 可选：为了安全，每次关闭摩擦轮时，重置发射模式为单发
            /*
            if (friction_switch_state == 0) {
                fire_mode_state = 0;
            }
            */
        }

        // ==============================================================
        // 逻辑 B: 执行发射逻辑
        // ==============================================================
        if (friction_switch_state == 1) {
            // 1. 开启摩擦轮
            shoot_cmd_send.friction_mode = FRICTION_ON;
            shoot_cmd_send.bullet_speed = BIG_AMU_12;

            // 2. 处理开火指令 (左拨杆 -> 下)
            // 只有在摩擦轮开启时，拨到下面才有效

            switch (fire_mode_state) {
            case 0: // 【单发模式】
                if (switch_is_down(current_switch_left)) {
                    shoot_cmd_send.load_mode = LOAD_1_BULLET;
                } else {
                    shoot_cmd_send.load_mode = LOAD_STOP;
                    shoot_cmd_send.shoot_rate = 0.0f;
                }
                break;

            case 1:
                if (switch_is_down(current_switch_left)) {
                    shoot_cmd_send.load_mode = LOAD_2_BULLET;
                } else {
                    shoot_cmd_send.load_mode = LOAD_STOP;
                }
                break;

            case 2: // 【连发模式】
                // 逻辑: 只要拨杆保持在 [下]，就持续开火 (电平触发)
                if (switch_is_down(current_switch_left)) {
                    shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
                    shoot_cmd_send.shoot_rate = BURST_FIRE_RATE;
                } else {
                    shoot_cmd_send.load_mode = LOAD_STOP;
                }
                break;

            default:
                shoot_cmd_send.load_mode = LOAD_STOP;
                break;
            }
        } else {
            // --- 摩擦轮关闭状态 ---
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.load_mode = LOAD_STOP;
            shoot_cmd_send.shoot_rate = 0.0f;
        }

    } else {
        // [新增] 急停状态下的强制关闭
        friction_switch_state = 0; // 强制关闭摩擦轮开关状态
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        shoot_cmd_send.shoot_rate = 0.0f;
    }

    // 更新历史状态
    last_switch_left = current_switch_left;
    last_switch_right = current_switch_right;
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
    StdCmdApplyRefereeKeyMouseOverlay(&chassis_fetch_data.referee_keymouse,
                                      robot_state,
                                      &chassis_cmd_send,
                                      &gimbal_cmd_send); // What: 通过std_cmd模块叠加裁判WSAD/鼠标；Why: 让主程序只保留状态机和模块调用
    // else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
    //     StdCmdApplyRemoteMouseKey(rc_data, &chassis_cmd_send, &gimbal_cmd_send);

    // EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // ======================== 视觉通信发送 ========================
    // 注: 四元数和姿态由 ins_task.c 的 INS_Task() 以 1kHz 频率直接设置
    //     (VisionSetQuaternion + VisionSetAltitude), 使用 EKF 解算的真实四元数
    // 此处只需补充机器人状态并触发发送
    VisionSetStatus(
        (uint8_t)gimbal_cmd_send.gimbal_mode, // 当前模式
        (float)shoot_cmd_send.bullet_speed, // 弹速
        shoot_fetch_data.fire_count // 累计发弹数
    );
    VisionSend(); // 通过 USB VCP 发送给上位机

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
