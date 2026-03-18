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
// What: 定义遥控器连发射频；Why: 保持当前遥控器发射手感，不把这次上岛迁移扩散成发射参数重调。
#define BURST_FIRE_RATE 8.0f
// What: 定义抬升拨轮归一化满量程；Why: 与DBUS拨轮量程保持一致，让双板两侧都使用同一套输入解释。
#define AUX_DIAL_INPUT_MAX 660.0f
// What: 定义抬升拨轮死区；Why: 机械中位抖动和手指轻碰都不应该持续积分抬升目标。
#define AUX_DIAL_INPUT_DEADZONE 50.0f
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
// What: 在编译期校验底盘反馈结构体尺寸；Why: 双板通信仍需保证单帧CAN可以装下完整反馈。
_Static_assert(sizeof(Chassis_Upload_Data_s) <= CAN_COMM_MAX_BUFFSIZE,
               "Chassis_Upload_Data_s exceeds CAN_COMM_MAX_BUFFSIZE");
// What: 在编译期校验底盘控制结构体尺寸；Why: 新增上岛字段后必须继续保证命令帧不会溢出CANComm缓冲区。
_Static_assert(sizeof(Chassis_Ctrl_Cmd_s) <= CAN_COMM_MAX_BUFFSIZE,
               "Chassis_Ctrl_Cmd_s exceeds CAN_COMM_MAX_BUFFSIZE");
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
static uint16_t last_switch_left = RC_SW_DOWN; // What: 保存左拨杆上一拍状态；Why: 上岛逻辑依赖中位切换边沿，不能把持续保持误判成重复触发。
// What: 保存底盘跟随使用的 yaw 软件对齐基准角；Why: 直接信任 DM 已保存的硬件零点，避免开机又减一次机械安装角导致零点偏移。
static float yaw_align_offset_deg = 0.0f;
static uint8_t friction_switch_state = 0u; // What: 记录遥控器摩擦轮锁存状态；Why: 左拨杆上拨需要做一次一切换而不是按住就反复翻转。
static uint8_t fire_mode_state = 0u; // What: 记录当前遥控器发射模式；Why: 保留现有单发/二连发/连发切换状态，不把上岛迁移变成发射重构。
static uint8_t front_track_switch_state = 0u; // What: 记录前履带开关状态；Why: 上岛辅助机构需要跨控制周期保持启停状态。
static uint8_t lift_mode_switch_state = 0u; // What: 记录抬升模式锁存状态；Why: 拨轮松手后仍需保持当前位置，而不是自动退出抬升会话。
static uint8_t lift_retract_active = 0u; // What: 记录快速收腿状态；Why: 拨动到中间时触发收腿，持续下发直到底盘内部撞底层限位停止。

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

void RobotCMDInit()
{
    // What: 开机时把 yaw 软件对齐基准直接设为 0 度；Why: 让底盘跟随从第一拍就围绕 DM 硬件零点闭环，避免首帧回到旧机械角。
    yaw_align_offset_deg = 0.0f;
    rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    // What: 把图传键鼠接到 USART6；Why: 当前 DJI C 板外部标注的 UART1 接口对应 PG14/PG9 这组 USART6 引脚。
    video_link_data = VideoLinkKMInit(&huart6);
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
    // What: 保留双板通信初始化日志；Why: 这些原本就是排障入口，不该因为快速收腿需求被我删掉。
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
 * @brief 将 pitch 目标统一限幅到机构安全范围
 *
 */
static void LimitGimbalPitchTarget()
{
    // What: 统一约束 pitch 目标；Why: 遥控器与键鼠都会改写 pitch，必须共用一套限位避免撞机构。
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
    // What: 一次性清空键鼠开火锁存；Why: 急停或链路切换后不能把旧的摩擦轮/连发意图带到下一拍。
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
    // What: 清空键盘平移斜坡缓存；Why: 急停或断链后不应继续沿用上一拍的速度尾巴。
    keyboard_vx_smoothed = 0.0f;
    keyboard_vy_smoothed = 0.0f;
    keyboard_ramp_last_ms = 0u;
}

/**
 * @brief 将拨轮原始值归一化为抬升输入
 *
 * @param raw_dial 遥控器原始拨轮值
 * @return float 归一化后的抬升输入
 */
static float NormalizeLiftDialInput(int16_t raw_dial)
{
    float normalized = -((float)raw_dial) / AUX_DIAL_INPUT_MAX;
    float deadzone = AUX_DIAL_INPUT_DEADZONE / AUX_DIAL_INPUT_MAX;

    // What: 统一把拨轮上拨解释成正向抬升并做死区与限幅；Why: 底盘侧只消费稳定标准化命令，避免双板两边各自解释方向和抖动。
    if (normalized > -deadzone && normalized < deadzone)
        return 0.0f;

    LIMIT_MIN_MAX(normalized, -1.0f, 1.0f);
    return normalized;
}

/**
 * @brief 返回当前有效的键鼠控制源
 */
static const RC_ctrl_t *GetActiveMouseKeySource(void)
{
    // What: 恢复原有图传优先取数方式；Why: 快速收腿不应改变键鼠输入源的指针语义。
    if (video_link_data != NULL && VideoLinkKMIsOnline() && VideoLinkKMHasValidFrame()) {
        return &video_link_data[TEMP];
    }
    return &rc_data[TEMP];
}

/**
 * @brief 清空上岛辅助机构锁存状态
 */
static void ResetChassisAuxState()
{
    // What: 一次性复位前履带、抬升和快速收腿状态及下发值；Why: 急停或退出上岛时不能沿用上一拍辅助机构命令继续动作。
    front_track_switch_state = 0u;
    lift_mode_switch_state = 0u;
    lift_retract_active = 0u;
    chassis_cmd_send.front_track_mode = FRONT_TRACK_OFF;
    chassis_cmd_send.lift_mode = LIFT_OFF;
    chassis_cmd_send.front_track_speed_ref = 0.0f;
    chassis_cmd_send.lift_dial_input = 0.0f;
}

/**
 * @brief 每拍先清空瞬态控制量
 */
static void PrepareControlCommandBase()
{
    // What: 恢复原有瞬态命令清零基线；Why: 快速收腿只需要叠加辅助机构命令，不该删掉其它原本每拍回零的控制字段。
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = 0.0f;
    chassis_cmd_send.gimbal_cmd_wz = 0.0f;
    chassis_cmd_send.calibrate_imu = 0;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.cap_mode = SUPER_CAP_OFF;
    chassis_cmd_send.chassis_speed_buff = 0;
    chassis_cmd_send.front_track_mode = FRONT_TRACK_OFF;
    chassis_cmd_send.front_track_speed_ref = 0.0f;
    chassis_cmd_send.lift_mode = LIFT_OFF;
    chassis_cmd_send.lift_dial_input = 0.0f;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    shoot_cmd_send.load_mode = LOAD_STOP;
    shoot_cmd_send.shoot_rate = 0.0f;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    auto_aim_state = AUTO_AIM_IDLE;
}

/**
 * @brief 按斜坡限速把当前值推进到目标值
 */
static float RampKeyboardAxis(float current, float target, float dt_s)
{
    float delta = target - current;
    float max_step;

    // What: 按当前是加速还是减速选择不同步长；Why: 起步要柔和，松键和反向要更干脆。
    if (target == 0.0f || (current > 0.0f && target < current) || (current < 0.0f && target > current))
        max_step = KEYBOARD_CHASSIS_RAMP_DOWN_PER_SEC * dt_s;
    else
        max_step = KEYBOARD_CHASSIS_RAMP_UP_PER_SEC * dt_s;

    if (delta > max_step)
        delta = max_step;
    else if (delta < -max_step)
        delta = -max_step;

    current += delta;
    if (fabsf(current) < KEYBOARD_CHASSIS_CMD_EPSILON && target == 0.0f)
        current = 0.0f;

    return current;
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 */
static void CalcOffsetAngle()
{
    float error = gimbal_fetch_data.yaw_motor_single_round_angle - yaw_align_offset_deg;

    // What: 把云台相对底盘的偏角统一折叠到[-180, 180]；Why: 底盘跟随只需要最短转向误差，不能直接拿0~360度做闭环。
    while (error < -180.0f)
        error += 360.0f;
    while (error > 180.0f)
        error -= 360.0f;

    chassis_cmd_send.offset_angle = error;
}

/**
 * @brief 紧急停止处理
 */
static void EmergencyHandler()
{
    // What: 恢复原有急停清状态逻辑；Why: 快速收腿不能越过急停，摩擦轮和辅助机构锁存都必须一起清掉。
    robot_state = ROBOT_STOP;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    friction_switch_state = 0u;
    shoot_cmd_send.shoot_rate = 0.0f;
    ResetChassisAuxState();
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 */
static void RemoteControlSet()
{
    uint16_t current_switch_right = rc_data[TEMP].rc.switch_right;
    uint16_t current_switch_left = rc_data[TEMP].rc.switch_left;
    uint8_t left_mid_to_up = (uint8_t)(switch_is_up(current_switch_left) && switch_is_mid(last_switch_left));
    uint8_t left_mid_to_down = (uint8_t)(switch_is_down(current_switch_left) && switch_is_mid(last_switch_left));
    uint8_t left_up_to_mid = (uint8_t)(switch_is_mid(current_switch_left) && switch_is_up(last_switch_left));
    float rocker_lx;
    float rocker_ly;
    float rocker_rx;
    float rocker_ry;
    float current_real_pitch = gimbal_fetch_data.gimbal_imu_data.Pitch; // What: 读取当前真实 pitch 反馈；Why: 内八校准后要把云台目标同步回当前姿态，避免退出急停时瞬时回拉。

    // What: 右拨杆下位进入急停与校准检测；Why: 保持原有安全优先级，不让辅助机构逻辑覆盖急停。
    if (switch_is_down(current_switch_right)) {
        bool is_inner_eight;

        EmergencyHandler();

        is_inner_eight = (rc_data[TEMP].rc.rocker_l_ > RC_TRIGGER_TH) &&
                         (rc_data[TEMP].rc.rocker_l1 < -RC_TRIGGER_TH) &&
                         (rc_data[TEMP].rc.rocker_r_ < -RC_TRIGGER_TH) &&
                         (rc_data[TEMP].rc.rocker_r1 < -RC_TRIGGER_TH);

        if (is_inner_eight) {
            if (cali_triggered == 0u) {
                GimbalCalibrate();
                // What: 手动校零后把底盘跟随零位同步回运行时基准；Why: 否则 offset 还会沿用旧零点，看起来像校零没有生效。
                yaw_align_offset_deg = 0.0f;
                // What: 校零完成后同步刷新云台目标到当前姿态；Why: 退出急停时不能让云台去追旧目标角。
                gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
                gimbal_cmd_send.pitch = current_real_pitch;
                if (hint_buzzer != NULL)
                    AlarmSetStatus(hint_buzzer, ALARM_ON);
                cali_triggered = 1u;
            }
        } else if (cali_triggered != 0u) {
            if (hint_buzzer != NULL)
                AlarmSetStatus(hint_buzzer, ALARM_OFF);
            cali_triggered = 0u;
        }
    } else if (switch_is_up(current_switch_right)) {
        // What: 右拨杆上位进入小陀螺 + 云台陀螺仪模式；Why: 保持当前车体的上位驾驶习惯不变。
        if (!switch_is_up(last_switch_right))
            gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;

        robot_state = ROBOT_READY;
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    } else {
        // What: 右拨杆中位进入底盘跟随云台模式；Why: 保留当前主驾驶模式切换语义。
        if (!switch_is_mid(last_switch_right))
            gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;

        robot_state = ROBOT_READY;
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }

    rocker_lx = (float)rc_data[TEMP].rc.rocker_l_;
    rocker_ly = (float)rc_data[TEMP].rc.rocker_l1;
    rocker_rx = (float)rc_data[TEMP].rc.rocker_r_;
    rocker_ry = (float)rc_data[TEMP].rc.rocker_r1;

    // What: 对四路摇杆统一做死区处理；Why: 中位抖动不应该继续驱动云台和底盘。
    if (rocker_lx > -RC_DEADZONE && rocker_lx < RC_DEADZONE)
        rocker_lx = 0.0f;
    if (rocker_ly > -RC_DEADZONE && rocker_ly < RC_DEADZONE)
        rocker_ly = 0.0f;
    if (rocker_rx > -RC_DEADZONE && rocker_rx < RC_DEADZONE)
        rocker_rx = 0.0f;
    if (rocker_ry > -RC_DEADZONE && rocker_ry < RC_DEADZONE)
        rocker_ry = 0.0f;

    if (!switch_is_down(current_switch_right)) {
        float yaw_sensitivity = 0.001f;
        float pitch_sensitivity = 0.0003f;

        if (rc_data[TEMP].rc.dial > 100) {
            yaw_sensitivity *= 0.3f;
            pitch_sensitivity *= 0.3f;
        }

        // What: 用左摇杆累加云台目标角；Why: 保持当前“目标角积分式”手感，不重写云台控制接口。
        gimbal_cmd_send.yaw -= yaw_sensitivity * rocker_lx;
        gimbal_cmd_send.pitch += pitch_sensitivity * rocker_ly;

        // What: 摇杆松手时把 yaw 目标缓慢锁回当前反馈；Why: 消除陀螺仪零偏导致的长期误差积累与放手漂移。
        if (rocker_lx == 0.0f) {
            gimbal_cmd_send.yaw = YAW_DRIFT_LOCK_COEF * gimbal_cmd_send.yaw +
                                  (1.0f - YAW_DRIFT_LOCK_COEF) * gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        }

        LimitGimbalPitchTarget();
    }

    if (switch_is_up(current_switch_right)) {
        if (gimbal_cmd_send.gimbal_mode == GIMBAL_GYRO_MODE) {
            float current_yaw_total = gimbal_fetch_data.gimbal_imu_data.VISION_YAW_AXIS * VISION_YAW_SIGN;
            float current_pitch = gimbal_fetch_data.gimbal_imu_data.VISION_PITCH_AXIS * VISION_PITCH_SIGN;

            // What: 只有陀螺仪模式才允许自瞄覆写目标角；Why: 避免自由模式下视觉和手控同时争抢同一套目标值。
            auto_aim_state = AutoGimbalRun(
                vision_recv_data,
                current_yaw_total,
                current_pitch,
                &gimbal_cmd_send.yaw,
                &gimbal_cmd_send.pitch);
        } else {
            auto_aim_state = AUTO_AIM_IDLE;
        }
    } else {
        auto_aim_state = AUTO_AIM_IDLE;
    }

    // What: 底盘遥控量继续沿用当前历史量纲；Why: 底盘侧还按“摇杆值×10”解释，不能在这一轮同时改单位。
    chassis_cmd_send.vx = 10.0f * rocker_ry;
    chassis_cmd_send.vy = 10.0f * rocker_rx;

    if (!switch_is_down(current_switch_right)) {
        // What: 左拨杆中位上拨优先处理当前功能域；Why: 同一个拨杆同时复用摩擦轮和上岛辅助机构，必须按已锁存状态解释边沿。
        if (left_mid_to_up) {
            if (friction_switch_state != 0u) {
                friction_switch_state = 0u;
            } else if (front_track_switch_state != 0u) {
                if (lift_retract_active != 0u) {
                    lift_retract_active = 0u;
                    lift_mode_switch_state = 1u;
                } else {
                    lift_mode_switch_state = !lift_mode_switch_state;
                }
            } else {
                friction_switch_state = 1u;
            }
        }

        // What: 左拨杆中位下拨仍然只负责前履带开关；Why: 保持用户要求恢复的原始交互，不把收腿入口塞到下拨上。
        if (friction_switch_state == 0u && left_mid_to_down) {
            front_track_switch_state = !front_track_switch_state;
            if (front_track_switch_state == 0u) {
                lift_mode_switch_state = 0u;
                lift_retract_active = 0u;
            }
        }

        if (front_track_switch_state == 0u) {
            // What: 前履带关闭时同步退出抬升与收腿会话；Why: 上岛辅助机构必须先有前履带作为前置状态。
            lift_mode_switch_state = 0u;
            lift_retract_active = 0u;
        }

        // What: 左拨杆从上位回到中位时触发快速收腿；Why: `lift_retract_active` 已预留但未接线，本轮把它挂到用户指定的拨杆动作上。
        if (left_up_to_mid &&
            friction_switch_state == 0u &&
            front_track_switch_state != 0u &&
            lift_mode_switch_state != 0u) {
            lift_retract_active = 1u;
        }

        if (friction_switch_state == 1u) {
            // What: 摩擦轮开启后才解析左拨杆下位发射；Why: 保持原有发射安全门控，不让辅助机构复用伤到发射逻辑。
            shoot_cmd_send.friction_mode = FRICTION_ON;
            shoot_cmd_send.bullet_speed = BIG_AMU_12;

            switch (fire_mode_state) {
            case 0:
                if (switch_is_down(current_switch_left)) {
                    shoot_cmd_send.load_mode = LOAD_1_BULLET;
                } else {
                    shoot_cmd_send.load_mode = LOAD_STOP;
                    shoot_cmd_send.shoot_rate = 0.0f;
                }
                break;

            case 1:
                if (switch_is_down(current_switch_left))
                    shoot_cmd_send.load_mode = LOAD_2_BULLET;
                else
                    shoot_cmd_send.load_mode = LOAD_STOP;
                break;

            case 2:
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
        }

        chassis_cmd_send.front_track_mode = front_track_switch_state ? FRONT_TRACK_ON : FRONT_TRACK_OFF;
        if (front_track_switch_state != 0u) {
            float lift_dial_input = NormalizeLiftDialInput(rc_data[TEMP].rc.dial);

            chassis_cmd_send.front_track_speed_ref = FRONT_TRACK_SPEED_REF_DEFAULT; // What: 前履带开启时下发显式配置速度；Why: 让用户继续只改宏就能调履带速度。
            chassis_cmd_send.lift_dial_input = 0.0f;

            if (lift_retract_active != 0u) {
                // What: 快速收腿期间强制下发 `LIFT_RETRACT`；Why: 持续保持中位时也要让底盘侧一直执行收腿，直到内部撞底停机。
                chassis_cmd_send.lift_mode = LIFT_RETRACT;
            } else {
                // What: 非收腿时恢复原始抬升拨轮语义；Why: 手动拨轮松手后必须回到位置保持，而不是在 `robot_cmd` 里擅自切成陀螺仪自动调平。
                chassis_cmd_send.lift_dial_input = lift_dial_input;
                chassis_cmd_send.lift_mode = (lift_mode_switch_state == 0u) ? LIFT_OFF : (lift_dial_input == 0.0f ? LIFT_HOLD : LIFT_ADJUST);
            }
        }
    } else {
        // What: 急停时强制清空所有辅助机构锁存；Why: 解除急停后必须从安全基线重新进入，而不是继续沿用旧会话。
        friction_switch_state = 0u;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        shoot_cmd_send.shoot_rate = 0.0f;
        ResetChassisAuxState();
    }

    last_switch_left = current_switch_left; // What: 记录左拨杆历史状态；Why: 自动收腿依赖 `up -> mid` 边沿，不能只看当前电平。
    last_switch_right = current_switch_right; // What: 记录右拨杆历史状态；Why: 云台无扰切换只应在模式边沿触发一次。
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
