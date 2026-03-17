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
#include "remote_control.h"
#include "video_link_km.h"

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
// 鼠标灵敏度按当前车体手感分别调整，作用是让 yaw 更跟手、pitch 方向与实际操控一致；
// 原因是用户反馈 yaw 偏慢且 pitch 反向，因此这里同时上调 yaw 系数并翻转 pitch 符号。
#define MOUSE_YAW_SENSITIVITY_DEG (0.00011f * RAD_2_DEGREE)
#define MOUSE_PITCH_SENSITIVITY_DEG (0.000018f * RAD_2_DEGREE)
// 键盘底盘指令沿用当前摇杆分支的量纲，作用是让 WSAD 和摇杆给到底盘的速度处于同一数量级；
// 原因是底盘侧现在仍按“摇杆值×10”的历史量纲做解算，直接发 6.0f 会小到几乎看不出运动。
#define KEYBOARD_CHASSIS_CMD_SCALE 6600.0f
// 键盘平移斜坡加速率，作用是把 WSAD 从阶跃输入改成渐变输入；
// 原因是当前满幅键盘指令一拍就冲到目标值，底盘体感会像“瞬间踹一脚”。
#define KEYBOARD_CHASSIS_RAMP_UP_PER_SEC 22000.0f
// 键盘平移斜坡减速率，作用是松键后更快回零但仍保持平滑；
// 原因是只做慢加速会让停车拖泥带水，因此减速单独设得略快。
#define KEYBOARD_CHASSIS_RAMP_DOWN_PER_SEC 30000.0f
// 键盘平移状态归零阈值，作用是避免斜坡尾巴长期残留极小数值；
// 原因是 CAN 下发浮点微小残值也会让底盘低速爬行，看起来像没停干净。
#define KEYBOARD_CHASSIS_CMD_EPSILON 1.0f
// 键鼠长按连发速率单独定义，作用是让鼠标连发不依赖遥控拨杆分支；
// 原因是本轮需要让键鼠和遥控器并行控制，不能复用函数内局部宏。
#define MOUSE_BURST_FIRE_RATE 8.0f

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
static RC_ctrl_t *video_link_data; // 图传键鼠数据,初始化时返回
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
// What: 保存底盘跟随使用的 yaw 软件对齐基准角；Why: 直接信任 DM 已保存的硬件零点，避免开机又减一次机械安装角导致零点偏移。
static float yaw_align_offset_deg = 0.0f;

// --- 新增的静态变量，用于长按计时 ---
static uint32_t inner_eight_cnt = 0; // 内八计时器
static uint32_t outer_eight_cnt = 0; // 外八计时器

static uint8_t cali_triggered = 0; // 触发状态：0-无，1-内八触发，2-外八触发
// [新增] 自瞄状态变量
static AutoAim_State_e auto_aim_state = AUTO_AIM_IDLE;
// 键鼠射击锁存状态，作用是记住“鼠标已请求开火后保持摩擦轮开启”；
// 原因是本轮只接 mouse.press_l，没有单独停摩擦轮按键，必须靠锁存保证后续连发稳定。
static uint8_t mouse_fire_friction_latched = 0;
// 鼠标左键上一拍电平，作用是检测单发上升沿；
// 原因是 `shoot` 应用的单发事务是边沿触发，不能把电平直接长期送成 `LOAD_1_BULLET`。
static uint8_t mouse_left_last = 0;
// 鼠标是否已进入长按连发，作用是区分短按单发和长按连发；
// 原因是松手后需要自动退出 burst，而不能影响遥控器本身的发射命令。
static uint8_t mouse_left_burst_active = 0;
// 鼠标左键按下时间戳，作用是计算长按阈值；
// 原因是任务存在调度抖动，用绝对时间比按周期计数更稳。
static uint32_t mouse_left_press_start_ms = 0;
// 记录图传链路上一次在线状态，作用是做离线边沿清零；
// 原因是图传优先级高于 DBUS 键鼠，断链时必须主动丢弃旧的鼠标锁存。
static uint8_t last_video_link_online = 0;
// 键盘摩擦轮开关键上一拍电平，作用是检测 F 键上升沿；
// 原因是摩擦轮开关应该是“按一下切换一次”，而不是按住期间每拍反复翻转。
static uint8_t keyboard_friction_toggle_last = 0;
// 键盘平移当前平滑输出，作用是保存斜坡状态跨周期延续；
// 原因是 `PrepareControlCommandBase()` 每拍都会清零瞬态命令，平滑状态必须独立保存。
static float keyboard_vx_smoothed = 0.0f;
static float keyboard_vy_smoothed = 0.0f;
// 键盘平移上一次更新时间戳，作用是按真实控制周期计算每拍允许变化量；
// 原因是任务调度并非绝对恒定 5ms，直接写死步长会让不同负载下手感漂移。
static uint32_t keyboard_ramp_last_ms = 0;

/**
 * @brief 将 pitch 目标统一限幅到机构安全范围
 *
 */
static void LimitGimbalPitchTarget()
{
    // 这里统一约束 pitch 目标，作用是让遥控器和鼠标共用同一份限位；
    // 原因是两个输入源都会改写 pitch，分散限位容易出现一边忘记限位导致撞机构。
    if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE) {
        gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
    } else if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE) {
        gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
    }
}

/**
 * @brief 清空键鼠射击锁存状态
 *
 */
static void ResetMouseFireState()
{
    // 这里把键鼠开火状态全部清零，作用是急停后不残留摩擦轮和连发锁存；
    // 原因是键鼠改成并行输入后，若不清状态，恢复出急停时会把上一次鼠标意图带回来。
    mouse_fire_friction_latched = 0;
    mouse_left_last = 0;
    mouse_left_burst_active = 0;
    mouse_left_press_start_ms = 0;
    keyboard_friction_toggle_last = 0;
}

/**
 * @brief 清空键盘平移斜坡状态
 *
 */
static void ResetKeyboardMotionState()
{
    // 这里把键盘平移斜坡状态清零，作用是急停/断链后不保留上一拍的加减速尾巴；
    // 原因是平滑状态本身是跨周期记忆量，不主动复位就会在恢复控制时带出历史速度。
    keyboard_vx_smoothed = 0.0f;
    keyboard_vy_smoothed = 0.0f;
    keyboard_ramp_last_ms = 0u;
}

/**
 * @brief 按斜坡限速把当前值推进到目标值
 *
 * @param current 当前输出
 * @param target 目标输出
 * @param dt_s 本拍时间
 * @return float 平滑后的输出
 */
static float RampKeyboardAxis(float current, float target, float dt_s)
{
    float max_step;

    // 这里根据“加速/减速/反向”场景选择不同步长，作用是让起步柔和但松键更干脆；
    // 原因是底盘平移最怕的就是起步突兀和停车拖尾，两种手感不能共用一套斜率。
    if (target == 0.0f || (current > 0.0f && target < current) || (current < 0.0f && target > current)) {
        max_step = KEYBOARD_CHASSIS_RAMP_DOWN_PER_SEC * dt_s;
    } else {
        max_step = KEYBOARD_CHASSIS_RAMP_UP_PER_SEC * dt_s;
    }

    if (target > current + max_step) {
        current += max_step;
    } else if (target < current - max_step) {
        current -= max_step;
    } else {
        current = target;
    }

    if (current < KEYBOARD_CHASSIS_CMD_EPSILON && current > -KEYBOARD_CHASSIS_CMD_EPSILON && target == 0.0f) {
        current = 0.0f;
    }
    return current;
}

/**
 * @brief 选择当前生效的键鼠输入源
 *
 * @return const RC_ctrl_t* 当前键鼠输入源
 */
static const RC_ctrl_t *GetActiveMouseKeySource(void)
{
    // 这里优先返回图传键鼠数据，作用是让官方图传链路覆盖 DBUS 内嵌键鼠；
    // 原因是用户明确要求把 2026 图传控制入口接到云台板，当前实现必须以图传为主源。
    if (video_link_data != NULL && VideoLinkKMIsOnline() && VideoLinkKMHasValidFrame()) {
        return &video_link_data[TEMP];
    }
    return &rc_data[TEMP];
}

/**
 * @brief 每周期先把控制量恢复到安全基线
 *
 */
static void PrepareControlCommandBase()
{
    // 这里先清理瞬态控制量，作用是阻断上一拍残留的速度和发射命令；
    // 原因是键鼠接入后同一份指令会被多源叠加，若不先回到安全基线会出现“松手后还在沿用旧命令”。
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = 0.0f;
    chassis_cmd_send.gimbal_cmd_wz = 0.0f;
    chassis_cmd_send.calibrate_imu = 0;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;

    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;

    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.shoot_rate = 0.0f;

    auto_aim_state = AUTO_AIM_IDLE;
}

void RobotCMDInit()
{
    // What: 开机时把 yaw 软件对齐基准直接设为 0 度；Why: 让底盘跟随从第一拍就围绕 DM 硬件零点闭环，避免首帧回到旧机械角。
    yaw_align_offset_deg = 0.0f;
    rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    // 这里在 USART1 初始化图传键鼠，作用是按 921600 串口接入官方图传链路；
    // 原因是当前工程视觉走 VCP，USART1 空闲，正好可作为云台板图传入口。
    video_link_data = VideoLinkKMInit(&huart1);
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

    // What: 使用当前生效的 yaw 软件基准计算底盘跟随偏角；Why: 手动校零后继续围绕 0 度闭环，防止 offset 仍引用旧机械安装角。
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
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;

        // [新增] 小陀螺模式配置 (New Configuration: Little Top Mode)
        // 这里明确下发 CHASSIS_ROTATE，作用是让底盘侧进入小陀螺分支。
        // 之前该行被注释后，发送出去的一直是 CHASSIS_NO_FOLLOW，所以底盘永远不会自旋。
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        // 云台切换至陀螺仪模式以保持世界坐标系下的稳定瞄准
        // gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

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

        LimitGimbalPitchTarget();
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

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    const RC_ctrl_t *mouse_key_source = GetActiveMouseKeySource();
    uint8_t video_link_online = (video_link_data != NULL) && VideoLinkKMIsOnline() && VideoLinkKMHasValidFrame();
    uint16_t key_bits = mouse_key_source->key[KEY_PRESS].keys;
    uint8_t friction_toggle_pressed = (uint8_t)((key_bits >> Key_F) & 0x1u);
    int8_t keyboard_vx = (int8_t)((key_bits >> Key_W) & 0x1u) - (int8_t)((key_bits >> Key_S) & 0x1u);
    int8_t keyboard_vy = (int8_t)((key_bits >> Key_D) & 0x1u) - (int8_t)((key_bits >> Key_A) & 0x1u);
    uint8_t mouse_left_pressed = mouse_key_source->mouse.press_l;
    uint32_t now_ms = DWT_GetTimeline_ms();
    float dt_s = 0.005f;
    float keyboard_target_vx = (float)keyboard_vx * KEYBOARD_CHASSIS_CMD_SCALE;
    float keyboard_target_vy = (float)keyboard_vy * KEYBOARD_CHASSIS_CMD_SCALE;

    // 这里检测图传离线边沿，作用是第一时间清掉键鼠锁存和残留发射意图；
    // 原因是图传断链后会回退到 DBUS 源，若不在切换瞬间清理状态，旧图传命令会被继续沿用。
    if (last_video_link_online && !video_link_online) {
        ResetMouseFireState();
        ResetKeyboardMotionState();
    }
    last_video_link_online = video_link_online;

    if (keyboard_ramp_last_ms != 0u) {
        uint32_t dt_ms = now_ms - keyboard_ramp_last_ms;
        if (dt_ms > 0u && dt_ms < 20u) {
            dt_s = (float)dt_ms * 0.001f;
        }
    }
    keyboard_ramp_last_ms = now_ms;

    // 遇到急停或零力模式时直接清空键鼠锁存，作用是确保任何鼠标残留命令都不会越过急停；
    // 原因是键鼠现在和遥控器并行输入，停机优先级必须最高。
    if (robot_state == ROBOT_STOP ||
        gimbal_cmd_send.gimbal_mode == GIMBAL_ZERO_FORCE ||
        chassis_cmd_send.chassis_mode == CHASSIS_ZERO_FORCE) {
        ResetMouseFireState();
        ResetKeyboardMotionState();
        return;
    }

    // 这里把键盘目标值通过斜坡推进到输出，作用是让起步和停车都不是阶跃命令；
    // 原因是底盘动力链对阶跃输入很敏感，直接满量纲切换会表现成前后左右猛冲。
    keyboard_vx_smoothed = RampKeyboardAxis(keyboard_vx_smoothed, keyboard_target_vx, dt_s);
    keyboard_vy_smoothed = RampKeyboardAxis(keyboard_vy_smoothed, keyboard_target_vy, dt_s);

    if (keyboard_vx != 0 || keyboard_vx_smoothed != 0.0f) {
        chassis_cmd_send.vx = keyboard_vx_smoothed;
    }
    if (keyboard_vy != 0 || keyboard_vy_smoothed != 0.0f) {
        chassis_cmd_send.vy = keyboard_vy_smoothed;
    }

    // 鼠标在遥控器目标上继续叠加云台增量，作用是保留遥控微调同时给电脑端快速修正；
    // 原因是旧工程的键鼠本来就是通过 DBUS 叠加进来，本轮需要恢复这种混控手感。
    gimbal_cmd_send.yaw -= (float)mouse_key_source->mouse.x * MOUSE_YAW_SENSITIVITY_DEG;
    gimbal_cmd_send.pitch += (float)mouse_key_source->mouse.y * MOUSE_PITCH_SENSITIVITY_DEG;
    LimitGimbalPitchTarget();

    // 这里把 F 定义成键鼠摩擦轮开关键，作用是给电脑端一个显式的预热/关闭入口；
    // 原因是当前版本只有鼠标左键会隐式拉起摩擦轮，没有独立关断键，调试和行进中都不方便。
    if (friction_toggle_pressed && !keyboard_friction_toggle_last) {
        if (mouse_fire_friction_latched) {
            ResetMouseFireState();
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.load_mode = LOAD_STOP;
            shoot_cmd_send.shoot_rate = 0.0f;
        } else {
            mouse_fire_friction_latched = 1;
            mouse_left_burst_active = 0;
            mouse_left_press_start_ms = now_ms;
        }
    }
    keyboard_friction_toggle_last = friction_toggle_pressed;

    // 鼠标首次请求开火时锁存摩擦轮开启，作用是让后续短按/长按都不需要额外按键预热；
    // 原因是你本轮只要求接入 mouse.press_l，不能再依赖 F/Q/E 之类的旧调试键位。
    if (mouse_left_pressed && !mouse_left_last) {
        mouse_fire_friction_latched = 1;
        mouse_left_burst_active = 0;
        mouse_left_press_start_ms = now_ms;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.bullet_speed = BIG_AMU_16;
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
    } else if (mouse_left_pressed && ((now_ms - mouse_left_press_start_ms) >= LOAD_TRIGGER_DELAY)) {
        mouse_left_burst_active = 1;
    } else if (!mouse_left_pressed) {
        mouse_left_burst_active = 0;
    }

    // 只要键鼠曾经请求过开火，就持续维持摩擦轮开启，作用是让鼠标短按之后可立即继续点射或转连发；
    // 原因是当前版本没有显式“关闭键鼠摩擦轮”按键，掉电/急停前应保持 ready 状态。
    if (mouse_fire_friction_latched) {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        if (shoot_cmd_send.bullet_speed == BULLET_SPEED_NONE) {
            shoot_cmd_send.bullet_speed = BIG_AMU_16;
        }
    }

    // 长按进入连发，作用是让 mouse.press_l 兼顾点射和持续火力；
    // 原因是 `shoot` 应用对 `LOAD_BURSTFIRE` 是电平语义，必须在长按期间持续下发。
    if (mouse_left_burst_active) {
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        shoot_cmd_send.shoot_rate = MOUSE_BURST_FIRE_RATE;
    }

    mouse_left_last = mouse_left_pressed;
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

    // 每拍先清理一次瞬态控制量，作用是让后续遥控器和键鼠都从同一安全基线开始叠加；
    // 原因是当前 `robot_cmd` 已经不是互斥控制源，直接沿用上拍结果会产生残留指令。
    PrepareControlCommandBase();
    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 先执行遥控器基础映射，再叠加键鼠输入，作用是实现“同周期混控”；
    // 原因是用户要求键盘鼠标和遥控器能够同时控制，而不是走左拨杆互斥切换。
    RemoteControlSet();
    MouseKeySet();

    // EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // ======================== 视觉通信发送 ========================
    // 注: 四元数和姿态由 ins_task.c 的 INS_Task() 以 1kHz 频率直接设置
    //     (VisionSetQuaternion + VisionSetAltitude), 使用 EKF 解算的真实四元数
    // 此处只需补充机器人状态并触发发送
    // VisionSetStatus(
    //     (uint8_t)gimbal_cmd_send.gimbal_mode, // 当前模式
    //     (float)shoot_cmd_send.bullet_speed, // 弹速
    //     shoot_fetch_data.fire_count // 累计发弹数
    // );
    // VisionSend(); // 通过 USB VCP 发送给上位机

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
