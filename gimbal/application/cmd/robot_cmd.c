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
#include "user_lib.h"
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
// What: 定义 yaw 目标软锁允许生效的最大实际角速度；Why: 云台快速旋转或底盘开小陀螺时不能再把目标值往当前姿态拉，否则会把扰动误当成目标更新。
#define YAW_DRIFT_LOCK_GYRO_TH_DPS 8.0f
// What: 定义遥控器连发射频；Why: 保持当前遥控器发射手感，不把这次上岛迁移扩散成发射参数重调。
#define BURST_FIRE_RATE 8.0f
// What: 定义前履带默认速度参考；Why: 先打通履带速度闭环链路，后续实车调速只需改这一处。
#define FRONT_TRACK_SPEED_REF_DEFAULT 10.0f
// What: 定义左拨杆上档时的默认抬升输入；Why: 用户希望前履带开启后拨杆推到上档就能直接看到后轮抬升响应。
#define LIFT_SWITCH_UP_INPUT_DEFAULT 0.6f
// What: 定义抬升拨轮归一化满量程；Why: 与DBUS拨轮量程保持一致，让双板两侧都使用同一套输入解释。
#define AUX_DIAL_INPUT_MAX 660.0f
// What: 定义抬升拨轮死区；Why: 机械中位抖动和手指轻碰都不应该持续积分抬升目标。
#define AUX_DIAL_INPUT_DEADZONE 50.0f
// 鼠标灵敏度按当前车体手感分别调整，作用是让 yaw 更跟手、pitch 方向与实际操控一致；
// 原因是用户反馈 yaw 偏慢且 pitch 反向，因此这里同时上调 yaw 系数并翻转 pitch 符号。
#define MOUSE_YAW_SENSITIVITY_DEG (0.00012f * RAD_2_DEGREE) // What: 小幅上调键鼠 yaw 灵敏度；Why: 用户要求更跟手，但这次只做轻微增益，避免一次加太多后瞄准发飘。
// What: 进一步上调键鼠 Pitch 灵敏度；Why: 用户这次明确要求把鼠标 Pitch 再提一档，因此在现值基础上加 30%，让抬头压头更跟手但仍保留可控余量。
#define MOUSE_PITCH_SENSITIVITY_DEG (0.00002925f * RAD_2_DEGREE)

// What: 定义遥控器 Pitch 灵敏度；Why: 让 DT7 和 VT03 共用一套更高一点的 Pitch 手感，而不是在函数体里留裸值难以统一调整。
#define REMOTE_PITCH_SENSITIVITY 0.000375f
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
// What: 定义 VT03 的 C 挡编码；Why: 挡位映射需要显式拆开，避免再次把 C/N 合并后改坏小陀螺逻辑。
#define VT03_MODE_SW_C 0u
// What: 定义 VT03 的 N 挡编码；Why: 用命名常量替代裸值，后续调整挡位语义时不容易改漏。
#define VT03_MODE_SW_N 1u
// What: 定义 VT03 的 S 挡编码；Why: 当前 S 挡仍然保留自由模式，显式命名能让分支语义更清楚。
#define VT03_MODE_SW_S 2u
// What: 定义 VT03 模式未初始化标记；Why: 需要把“首次接入 VT03”与真实模式挡位区分开，避免刚上线就误判成换挡。
#define VT03_MODE_SW_INVALID 0xFFu
// What: 定义 VT03 Pause 在零力态下触发 yaw 校零的长按时长；Why: 需要保留短按零力入口，同时用足够长的按压门槛避免误触直接改掉 DM 零点。
#define VT03_PAUSE_CALIB_HOLD_MS 2000u
// What: 定义 yaw PID 复位请求保持拍数；Why: `message_center` 队列深度只有 1，边沿请求只发一拍有可能被下一拍覆盖，因此需要短暂保持几拍确保 gimbal 侧必然收到。
#define YAW_PID_RESET_HOLD_TICKS 4u
// What: 定义底盘跟随接管请求保持拍数；Why: 双板任务相位可能错开，只发单拍容易被覆盖，保持几拍才能确保底盘板收到“小陀螺退跟随”边沿。
#define FOLLOW_TRANSITION_REQUEST_HOLD_TICKS 4u
// What: 定义一键掉头完成时允许的云台 yaw 误差；Why: 目标角附近保留少量容差可以避开 IMU 噪声和离散控制带来的抖动，不必苛求完全零误差。
#define TURNBACK_COMPLETE_YAW_ERROR_DEG 6.0f
// What: 定义一键掉头完成判据需要连续满足的拍数；Why: 连续若干拍稳定满足比单拍命中更稳，能避免刚好掠过阈值就被误判为完成。
#define TURNBACK_COMPLETE_STABLE_TICKS 20u
// What: 定义一键掉头的最长执行时间；Why: 若机构阻塞或姿态估计异常，超时后必须退出执行态，避免一直霸占 yaw 目标。
#define TURNBACK_TIMEOUT_MS 3000u

typedef enum {
    CONTROL_SOURCE_NONE = 0,
    CONTROL_SOURCE_VT03,
    CONTROL_SOURCE_DT7,
} ControlSource_e;

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
// What: 保存底盘跟随使用的 yaw 软件对齐基准角；Why: 当前链路已统一围绕 DM 硬件零点闭环，保留独立变量是为了后续手动校零时仍有单一基准可改。
static float yaw_align_offset_deg = YAW_CHASSIS_ALIGN_DEG;
// What: 记录在底盘跟随模式里被视为“正前方”的真实云台-底盘夹角；Why: V 键只让云台转动时，反转后的朝向仍要被认定为新前方，因此必须把这份语义独立保存。
static float follow_front_offset_deg = 0.0f;
static uint8_t friction_switch_state = 0u; // What: 记录遥控器摩擦轮锁存状态；Why: 左拨杆上拨需要做一次一切换而不是按住就反复翻转。
static uint8_t fire_mode_state = 0u; // What: 记录当前遥控器发射模式；Why: 保留现有单发/二连发/连发切换状态，不把上岛迁移变成发射重构。
#ifdef USE_ISLAND_ACTION
static uint8_t front_track_switch_state = 0u; // What: 记录前履带开关状态；Why: 上岛辅助机构需要跨控制周期保持启停状态。
#endif
// What: 记录 DT7 内八校零是否已经在本次组合里触发；Why: 校零指令属于重动作，按住组合期间只能发一次，避免重复改零点。
static uint8_t cali_triggered = 0u;

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
// 键盘小陀螺开关键上一拍电平，作用是检测 X 键上升沿；
// 原因是用户要求 X 键按一次切一次，不能按住期间每拍重复翻转小陀螺状态。
static uint8_t keyboard_spin_toggle_last = 0u;
// 键盘自由模式开关键上一拍电平，作用是检测 B 键上升沿；
// 原因是自由模式需要做锁存切换，不能把按住期间的持续电平误判成多次切换。
static uint8_t keyboard_free_toggle_last = 0u;
// 键盘一键掉头键上一拍电平，作用是检测 V 键上升沿；
// 原因是 V 现在是一次性动作触发键，必须只在真正按下沿启动一次新的掉头任务，不能把按住期间的持续电平误判成重复触发。
static uint8_t keyboard_turnback_toggle_last = 0u;
// 键盘 UI 刷新键上一拍电平，作用是检测 G 键上升沿；
// 原因是 UI 整页重建只能发一次请求，不能在按住期间每拍都重复触发底盘板删页重画。
static uint8_t keyboard_ui_refresh_last = 0u;
// 键盘小陀螺锁存状态，作用是让 X 键一次切换后持续保持小陀螺；
// 原因是用户明确要求 X 键直接作为小陀螺开关，而不是按住才生效。
static uint8_t keyboard_spin_mode_latched = 0u;
// 键盘自由模式锁存状态，作用是让 B 键一次切换后持续保持“云台自由 + 底盘自由”；
// 原因是用户要求按一下进入该模式，而不是全程按住某个键保持。
static uint8_t keyboard_free_mode_latched = 0u;
// 键盘一键掉头执行态，作用是标记当前这次“转头 180 度”是否仍在执行；
// 原因是掉头是一次性动作而不是模式切换，执行完成或被打断后都必须释放 yaw 目标强制覆盖。
static uint8_t keyboard_turnback_active = 0u;
// 键盘一键掉头目标 yaw，作用是跨周期保存这次掉头要追踪的累计角目标；
// 原因是掉头参考已经改成“当前云台朝向反向 180 度”，必须把算出的累计角目标单独保存，后续每拍都稳定追踪同一个终点。
static float keyboard_turnback_target_yaw = 0.0f;
// 键盘一键掉头完成后要提交的新“前方参考角”，作用是跨周期保存这次掉头结束后底盘跟随应认定的正前方；
// 原因是用户要求 V 键期间底盘不转，但动作完成后又要把反转后的云台朝向作为新的前方，因此必须把待提交参考单独记住。
static float keyboard_turnback_follow_front_target_deg = 0.0f;
// 键盘一键掉头开始时间戳，作用是做掉头超时保护；
// 原因是机构或传感器异常时不能一直停留在执行态，必须在超时后释放目标强制覆盖。
static uint32_t keyboard_turnback_start_ms = 0u;
// 键盘一键掉头完成稳定拍数，作用是对完成判据做时间去抖；
// 原因是单拍命中阈值容易受噪声影响，连续若干拍满足才更接近“真正已经完成云台 180 度翻转”。
static uint8_t keyboard_turnback_stable_ticks = 0u;
// 键盘平移当前平滑输出，作用是保存斜坡状态跨周期延续；
// 原因是 `PrepareControlCommandBase()` 每拍都会清零瞬态命令，平滑状态必须独立保存。
static float keyboard_vx_smoothed = 0.0f;
static float keyboard_vy_smoothed = 0.0f;
// 键盘平移上一次更新时间戳，作用是按真实控制周期计算每拍允许变化量；
// 原因是任务调度并非绝对恒定 5ms，直接写死步长会让不同负载下手感漂移。
static uint32_t keyboard_ramp_last_ms = 0;
// What: 记录当前主控输入源；Why: 需要在 VT03 主控、DT7 回退和双离线零力之间做统一仲裁。
static ControlSource_e current_control_source = CONTROL_SOURCE_NONE;
// What: 记录 VT03 右自定义键上一拍电平；Why: 自定义键是电平输入，必须在 cmd 层自行做上升沿锁存切换。
static uint8_t vt03_fn_right_last = 0u;
// What: 记录 VT03 扳机上一拍电平；Why: 单发拨弹只能响应上升沿，不能把扳机电平直接长期送进发射状态机。
static uint8_t vt03_trigger_last = 0u;
// What: 记录 VT03 Pause 上一拍电平；Why: Pause 现在同时承担短按零力和长按校准，必须靠边沿区分按下、保持和释放三个阶段。
static uint8_t vt03_pause_last = 0u;
// What: 锁存 VT03 Pause 零力状态；Why: 长按校准必须在零力中完成，短按进入零力后也需要在松手后继续保持失能状态。
static uint8_t vt03_pause_zero_force_latched = 0u;
// What: 记录 VT03 Pause 本次按下起始时刻；Why: 长按校零要按真实毫秒时间判定，不能依赖任务周期拍数估算。
static uint32_t vt03_pause_press_start_ms = 0u;
// What: 标记 VT03 Pause 本次按压是否从零力态开始；Why: 只有“已经零力”时的长按才允许触发 yaw 校零，防止有力态误长按直接改零点。
static uint8_t vt03_pause_press_started_in_zero_force = 0u;
// What: 标记 VT03 Pause 本次按压是否已经触发过长按校零；Why: 同一次按住里只能发送一次 DM 校零指令，避免重复动作。
static uint8_t vt03_pause_longpress_handled = 0u;
// What: 标记 VT03 首次接管后是否仍需默认进入零力；Why: 用户要求上电后 VT03 不自动使能，但该默认锁存只应在第一次接管时生效，后续重连不应反复触发。
static uint8_t vt03_boot_zero_force_pending = 1u;
// What: 记录 VT03 上一拍模式挡位；Why: VT03 换挡时需要把云台目标同步到当前姿态，避免切挡瞬间跳变。
static uint8_t vt03_mode_sw_last = VT03_MODE_SW_INVALID;
// What: 记录 yaw PID 复位请求剩余保持拍数；Why: 小陀螺切换是边沿事件，做成多拍保持可以避免调度先后不同导致 gimbal 漏掉这一拍复位请求。
static uint8_t yaw_pid_reset_hold_ticks = 0u;
// What: 记录底盘跟随接管请求剩余保持拍数；Why: 退出小陀螺进跟随时，底盘板必须稳定收到边沿才能进入先刹停再回正的接管窗口。
static uint8_t follow_transition_request_hold_ticks = 0u;
// What: 记录 yaw 电机上一拍在线状态；Why: 复活边沿需要把目标贴回当前姿态，常态下则不应反复打断控制。
static uint8_t last_yaw_motor_online = 0u;
// What: 缓存最后一次可信的真实云台-底盘夹角；Why: yaw 电机离线时继续沿用这个值，比用脏机械角实时重算更安全。
static float last_valid_offset_angle = 0.0f;
// What: 缓存最后一次可信的底盘跟随误差；Why: yaw 电机离线时虚拟前方闭环也必须冻结到最近可信值，不能重新解出一份假误差驱动底盘。
static float last_valid_follow_offset_angle = 0.0f;
// What: 记录上一拍最终下发到底盘的模式；Why: 底盘模式会被遥控器、VT03 和键鼠锁存多次覆盖，只有在最终结果层面检测边沿才不会漏掉小陀螺退跟随。
static chassis_mode_e last_effective_chassis_mode = CHASSIS_ZERO_FORCE;
static void ResetChassisAuxState(void);
static void SyncGimbalTargetToCurrentAttitude(void);
static void RequestYawSpeedPIDReset(void);
static void RequestFollowTransition(void);
static void SyncGimbalTargetAndRequestYawReset(void);
static void ResetVT03PausePressState(void);
static void TriggerVT03YawCalibration(void);




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
    // 这里仅清空键鼠发射相关锁存，作用是关闭摩擦轮或急停后不残留开火意图；
    // 原因是 F 键手动关摩擦轮也会复用这个函数，不能顺带把其它键鼠模式锁存一起清掉。
    mouse_fire_friction_latched = 0;
    mouse_left_last = 0;
    mouse_left_burst_active = 0;
    mouse_left_press_start_ms = 0;
    keyboard_friction_toggle_last = 0;
}

/**
 * @brief 清空键盘一键掉头状态
 *
 */
static void ClearKeyboardTurnbackState()
{
    // What: 一次性清空掉头执行态、目标角、新前方向候选值和完成计时；Why: 被模式切换、切源或急停打断后绝不能继续沿用旧掉头任务驱动云台，或在异常结束后误提交新的前方向语义。
    keyboard_turnback_active = 0u;
    keyboard_turnback_target_yaw = 0.0f;
    keyboard_turnback_follow_front_target_deg = 0.0f;
    keyboard_turnback_start_ms = 0u;
    keyboard_turnback_stable_ticks = 0u;
}

/**
 * @brief 清空键鼠控制锁存状态
 *
 */
static void ResetMouseControlLatchState()
{
    // What: 一次性清空键鼠发射、小陀螺、自由模式和一键掉头状态；Why: 急停、断链和切源后不应继续沿用上一拍的火力或模式意图。
    ResetMouseFireState();
    keyboard_spin_toggle_last = 0u;
    keyboard_free_toggle_last = 0u;
    keyboard_turnback_toggle_last = 0u;
    keyboard_ui_refresh_last = 0u;
    keyboard_spin_mode_latched = 0u;
    keyboard_free_mode_latched = 0u;
    ClearKeyboardTurnbackState();
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
 * @brief 对摇杆量应用统一死区
 *
 * @param value 原始摇杆量
 * @return float 死区处理后的摇杆量
 */
static float ApplyRCDeadzone(float value)
{
    // What: 统一复用同一套摇杆死区；Why: VT03 和 DT7 都应该保持相同的中位抖动抑制手感。
    if (value > -RC_DEADZONE && value < RC_DEADZONE) {
        return 0.0f;
    }
    return value;
}

/**
 * @brief 将单圈目标角映射为距离当前累计角最近的累计角目标
 *
 * @param current_angle_deg 当前累计角度
 * @param target_angle_norm_deg 目标单圈角度
 * @return float 最近的累计角目标
 */
static float FindClosestCumulativeAngle(float current_angle_deg, float target_angle_norm_deg)
{
    float target_norm = target_angle_norm_deg;
    float current_round;
    float target_angle_deg;
    float diff_deg;

    // What: 先把输入目标约束回单圈范围；Why: 上层只关心“反向 180 度”这类单圈语义，统一归一化后再映射到累计角可以避免跨圈歧义。
    while (target_norm > 180.0f) {
        target_norm -= 360.0f;
    }
    while (target_norm <= -180.0f) {
        target_norm += 360.0f;
    }

    // What: 先按当前累计角所在的最近圈数生成一个初始目标；Why: 云台 yaw 现在按 IMU 累计角闭环，必须把单圈语义落到最近圈才能避免突然多转整圈。
    current_round = roundf(current_angle_deg / 360.0f);
    target_angle_deg = current_round * 360.0f + target_norm;
    diff_deg = target_angle_deg - current_angle_deg;

    // What: 若初始目标离当前姿态超过半圈，则补偿一整圈到更近的等价目标；Why: 这样最终执行的一定是机械上最近的那一个等价角，而不是远端同姿态目标。
    if (diff_deg > 180.0f) {
        target_angle_deg -= 360.0f;
    } else if (diff_deg < -180.0f) {
        target_angle_deg += 360.0f;
    }

    return target_angle_deg;
}

/**
 * @brief 将云台目标同步到当前姿态
 *
 */
static void SyncGimbalTargetToCurrentAttitude()
{
    // What: 在控制源切换或模式切换时把云台目标同步到当前反馈；Why: 避免切源后继续追旧目标导致云台突然跳转。
    gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
    LimitGimbalPitchTarget();
}

/**
 * @brief 请求 gimbal 侧复位 yaw 速度环运行时状态
 *
 */
static void RequestYawSpeedPIDReset(void)
{
    // What: 本拍立即拉高 yaw PID 复位请求并启动多拍保持；Why: 小陀螺切换边沿如果只发单拍，可能被下一拍普通命令覆盖，导致 gimbal 侧漏复位。
    gimbal_cmd_send.yaw_pid_reset_request = 1u;
    yaw_pid_reset_hold_ticks = YAW_PID_RESET_HOLD_TICKS;
}

/**
 * @brief 请求底盘侧进入“小陀螺退跟随”接管窗口
 *
 */
static void RequestFollowTransition(void)
{
    // What: 本拍立即拉高底盘接管请求并启动多拍保持；Why: 退出小陀螺到跟随是边沿事件，保持几拍才能避免双板调度相位错开时底盘漏收。
    chassis_cmd_send.follow_transition_request = 1u;
    follow_transition_request_hold_ticks = FOLLOW_TRANSITION_REQUEST_HOLD_TICKS;
}

/**
 * @brief 同步云台目标并请求复位 yaw 速度环
 *
 */
static void SyncGimbalTargetAndRequestYawReset(void)
{
    // What: 在模式切换边沿同时贴齐当前姿态并清 yaw 速度环残留状态；Why: 只同步目标不能消掉残余积分，底盘停止后仍可能把云台头慢慢拽歪。
    SyncGimbalTargetToCurrentAttitude();
    RequestYawSpeedPIDReset();
}

/**
 * @brief 清空 VT03 Pause 本次按压会话状态
 *
 */
static void ResetVT03PausePressState(void)
{
    // What: 清空 VT03 Pause 的长按计时和已处理标记；Why: 松手、切源或断链后本次按压语义已经失效，继续沿用会把下一次短按误判成长按。
    vt03_pause_press_start_ms = 0u;
    vt03_pause_press_started_in_zero_force = 0u;
    vt03_pause_longpress_handled = 0u;
    if (hint_buzzer != NULL) {
        // What: 清会话时同步关闭校零提示蜂鸣器；Why: 蜂鸣器只应反映当前一次校零动作，不能跨按压会话残留鸣叫。
        AlarmSetStatus(hint_buzzer, ALARM_OFF);
    }
}

/**
 * @brief 执行一次 VT03 的 yaw 轴零点校准
 *
 */
static void TriggerVT03YawCalibration(void)
{
    // What: 统一封装 VT03 长按 Pause 的 yaw 校零动作；Why: 校零后还要同步软件零位和云台目标，集中处理能避免恢复后漏掉复位链路。
    GimbalCalibrate();
    yaw_align_offset_deg = 0.0f;
    // What: yaw 硬件零点被重标后同步把虚拟前方参考复位到 0；Why: 旧的前方向参考依赖上一次零位坐标系，继续保留会让跟随误差整体偏移。
    follow_front_offset_deg = 0.0f;
    SyncGimbalTargetAndRequestYawReset();
    if (hint_buzzer != NULL) {
        // What: 校零真正触发后开启蜂鸣器提示；Why: 现场操作需要一个明确反馈来确认 DM 零点指令已经发出。
        AlarmSetStatus(hint_buzzer, ALARM_ON);
    }
}

/**
 * @brief 判断 VT03 是否已经具备主控资格
 *
 * @return uint8_t 1:可用 0:不可用
 */
static uint8_t IsVideoLinkControlReady(void)
{
    // What: 统一封装 VT03 主控可用条件；Why: 主控仲裁和键鼠取源都必须依赖同一套在线且有有效帧的判断。
    return (uint8_t)((video_link_data != NULL) && VideoLinkKMIsOnline() && VideoLinkKMHasValidFrame());
}

/**
 * @brief 获取当前主控输入源
 *
 * @return ControlSource_e 当前主控输入源
 */
static ControlSource_e GetActiveControlSource(void)
{
    // What: 固定采用 VT03 优先、DT7 回退、双离线零力的顺序；Why: 用户要求 VT03 遥控器恢复主控，DT7 只在断链时兜底。
    if (IsVideoLinkControlReady()) {
        return CONTROL_SOURCE_VT03;
    }
    if (RemoteControlIsOnline()) {
        return CONTROL_SOURCE_DT7;
    }
    return CONTROL_SOURCE_NONE;
}

/**
 * @brief 将键鼠边沿状态对齐到当前输入源的当前电平
 *
 * @param mouse_key_source 当前输入源的键鼠数据视图
 */
static void SyncMouseKeyEdgeState(const RC_ctrl_t *mouse_key_source)
{
    uint16_t key_bits = 0u;

    if (mouse_key_source != NULL) {
        key_bits = mouse_key_source->key[KEY_PRESS].keys;
        mouse_left_last = mouse_key_source->mouse.press_l;
    } else {
        mouse_left_last = 0u;
    }

    // What: 切换输入源时对齐键盘 F 键和鼠标左键边沿历史；Why: 否则切源首拍会把“已经按住”的键误判成新的上升沿。
    keyboard_friction_toggle_last = (uint8_t)((key_bits >> Key_F) & 0x1u);
    // What: 同步对齐 B 键边沿历史；Why: 主控切换时若用户正按着 B，不应在切源首拍被误判成新的自由模式切换。
    keyboard_free_toggle_last = (uint8_t)((key_bits >> Key_B) & 0x1u);
    // What: 同步对齐 V 键边沿历史；Why: 主控切换时若用户正按着 V，不应在切源首拍被误判成新的掉头触发。
    keyboard_turnback_toggle_last = (uint8_t)((key_bits >> Key_V) & 0x1u);
    // What: 同步对齐 G 键边沿历史；Why: 主控切换时用户若正按着 G，不应该在切源首拍误触发一次 UI 全量刷新。
    keyboard_ui_refresh_last = (uint8_t)((key_bits >> Key_G) & 0x1u);
    // What: 同步对齐 X 键边沿历史；Why: 主控切换时若用户正按着 X，不应在切源首拍被误判成新的小陀螺切换。
    keyboard_spin_toggle_last = (uint8_t)((key_bits >> Key_X) & 0x1u);
}

/**
 * @brief 处理主控输入源切换时的锁存复位和边沿同步
 *
 * @param new_source 新的主控输入源
 */
static void HandleControlSourceSwitch(ControlSource_e new_source)
{
    const VideoLinkKM_RemoteState_s *video_link_remote_state = VideoLinkKMGetRemoteState();
    const RC_ctrl_t *mouse_key_source = NULL;
    uint8_t need_sync_on_source_switch = (uint8_t)(new_source != CONTROL_SOURCE_NONE);

    if (new_source == current_control_source) {
        return;
    }

    // What: 切换主控源时统一清空所有跨周期锁存；Why: 不同链路的边沿语义不同，沿用旧状态会直接造成误开火或残留运动。
    ResetMouseControlLatchState();
    ResetKeyboardMotionState();
    ResetChassisAuxState();
    friction_switch_state = 0u;
    shoot_cmd_send.shoot_rate = 0.0f;
    auto_aim_state = AUTO_AIM_IDLE;
    // What: 切换主控源时同步清掉 DT7 校零锁存；Why: 内八组合已经失效，继续沿用旧触发状态会让蜂鸣器残留或下次组合无法重新触发。
    cali_triggered = 0u;
    // What: 切换主控源时只清理 Pause 本次按压会话，不清零力锁存；Why: VT03 短暂掉帧或切回 DT7 时若把失能锁存一起清掉，链路恢复后机器人会自己突然重新使能。
    ResetVT03PausePressState();
    vt03_pause_last = 0u;
    vt03_mode_sw_last = VT03_MODE_SW_INVALID;

    if (new_source == CONTROL_SOURCE_DT7) {
        last_switch_left = rc_data[TEMP].rc.switch_left;
        last_switch_right = rc_data[TEMP].rc.switch_right;
        vt03_fn_right_last = 0u;
        vt03_trigger_last = 0u;
        mouse_key_source = &rc_data[TEMP];
    } else if (new_source == CONTROL_SOURCE_VT03) {
        last_switch_left = RC_SW_DOWN;
        last_switch_right = RC_SW_DOWN;
        if (video_link_remote_state != NULL) {
            vt03_fn_right_last = video_link_remote_state->fn_right_button_down;
            vt03_trigger_last = video_link_remote_state->trigger_button_down;
            vt03_pause_last = video_link_remote_state->pause_button_down;
            vt03_mode_sw_last = video_link_remote_state->mode_sw;
        } else {
            vt03_fn_right_last = 0u;
            vt03_trigger_last = 0u;
            vt03_pause_last = 0u;
        }
        if (video_link_data != NULL) {
            mouse_key_source = &video_link_data[TEMP];
        }
        if (vt03_boot_zero_force_pending != 0u) {
            // What: VT03 第一次真正接管时先锁进零力；Why: 用户要求 VT03 上电后默认失能，必须等操作者明确按一次 Pause 才解除。
            vt03_pause_zero_force_latched = 1u;
            vt03_boot_zero_force_pending = 0u;
            // What: 首次接管默认零力时同步清空 Pause 按压会话；Why: 若沿用接管瞬间的电平或旧计时，可能把第一次短按误判成释放或长按。
            ResetVT03PausePressState();
            // What: 首次接管默认零力时不做目标同步与 PID 复位；Why: 当前拍本来就要保持失能，额外下发恢复链只会制造无意义的边沿扰动。
            need_sync_on_source_switch = 0u;
        }
    } else {
        last_switch_left = RC_SW_DOWN;
        last_switch_right = RC_SW_DOWN;
        vt03_fn_right_last = 0u;
        vt03_trigger_last = 0u;
    }

    SyncMouseKeyEdgeState(mouse_key_source);
    last_video_link_online = IsVideoLinkControlReady();

    if (need_sync_on_source_switch != 0u) {
        SyncGimbalTargetAndRequestYawReset();
    }

    current_control_source = new_source;
}

#ifdef USE_ISLAND_ACTION
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
#endif

/**
 * @brief 清空上岛辅助机构状态
 *
 */
static void ResetChassisAuxState()
{
    // What: 一次性复位前履带与抬升状态及下发值；Why: 急停或退出上岛时不能沿用上一拍辅助机构命令继续动作。
#ifdef USE_ISLAND_ACTION
    front_track_switch_state = 0u;
    chassis_cmd_send.front_track_mode = FRONT_TRACK_OFF;
    chassis_cmd_send.lift_mode = LIFT_OFF;
    chassis_cmd_send.front_track_speed_ref = 0.0f;
    chassis_cmd_send.lift_dial_input = 0.0f;
#endif
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
    // What: 键鼠输入跟随当前主控源取数；Why: VT03 在线时需要完整接管图传键鼠，断链后再无缝回退到 DBUS。
    if (current_control_source == CONTROL_SOURCE_VT03 && video_link_data != NULL) {
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
    chassis_cmd_send.cap_mode = SUPER_CAP_OFF;
    chassis_cmd_send.chassis_speed_buff = 0;
    chassis_cmd_send.follow_offset_angle = 0.0f; // What: 每拍先清空虚拟前方跟随误差；Why: 该值后续必须由当拍真实姿态重新计算，不能沿用上一拍残留结果。
    chassis_cmd_send.gimbal_pitch_deg = 0.0f; // What: 每拍先清空下发到底盘的云台pitch实测值；Why: 若后续链路异常或本拍未完成赋值，底盘 UI 不应沿用上一拍旧姿态。
    chassis_cmd_send.friction_on = 0u; // What: 每拍先清空摩擦轮状态位；Why: 断链或状态机切换时优先回到关闭显示，避免底盘 UI 继续误报摩擦轮开启。
    chassis_cmd_send.ui_refresh_request = 0u; // What: 每拍默认清空 UI 刷新请求；Why: 该请求是一次性边沿语义，不能像锁存状态一样跨周期保留。
    chassis_cmd_send.follow_transition_request = 0u; // What: 每拍默认清空底盘跟随接管请求；Why: 该请求只表达一次模式边沿，不能像持续模式值一样长期保持。
    chassis_cmd_send.follow_brake_request = 0u; // What: 每拍默认清空底盘跟随刹停请求；Why: 该请求只在 V 掉头执行窗口内有效，不能像模式值一样跨周期残留。
#ifdef USE_ISLAND_ACTION
    chassis_cmd_send.front_track_mode = FRONT_TRACK_OFF;
    chassis_cmd_send.lift_mode = LIFT_OFF;
    chassis_cmd_send.front_track_speed_ref = 0.0f;
    chassis_cmd_send.lift_dial_input = 0.0f;
#endif

    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    gimbal_cmd_send.yaw_pid_reset_request = 0u;

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
    // What: 开机时把虚拟前方参考也置回 0 度；Why: 默认前方必须与云台和底盘的物理回中方向一致，不能带入上次运行期的语义。
    follow_front_offset_deg = 0.0f;
    // What: 开机时把 VT03 首次接管默认零力标志置位；Why: 用户要求 VT03 上电后先失能，必须等第一次接管时自动锁进零力。
    vt03_boot_zero_force_pending = 1u;
    last_yaw_motor_online = 0u;
    last_valid_offset_angle = 0.0f;
    last_valid_follow_offset_angle = 0.0f;
    last_effective_chassis_mode = CHASSIS_ZERO_FORCE;
    follow_transition_request_hold_ticks = 0u;
    rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    // What: 将 VT03 图传链路恢复到 USART6；Why: 当前实车接线走的是云台板 USART6，挂到 USART1 会导致 VT03 遥控和键鼠都收不到有效帧。
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
    if (gimbal_fetch_data.yaw_motor_online == 0u) {
        // What: yaw 电机离线时同时冻结真实夹角和虚拟前方跟随误差；Why: 掉电或复活抖动阶段的机械角不可信，继续重算只会同时污染底盘平移坐标变换和跟随闭环。
        chassis_cmd_send.offset_angle = last_valid_offset_angle;
        chassis_cmd_send.follow_offset_angle = last_valid_follow_offset_angle;
        return;
    }

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

    // 3. 转换为 -180 ~ +180 范围并带有翻转边界死区迟滞 (Hysteresis)
    // Why: 避免刚从小陀螺/失能退出或受外力时，偏移角在 ±180 度边缘疯狂越界引起连续极性翻转，导致底盘在原地进行全力震荡抽搐。
    static float last_offset_angle = 0.0f;
    float current_offset;

    // 默认最短路径归一化
    if (error > 180.0f) {
        current_offset = error - 360.0f; // 例如 350 -> -10
    } else {
        current_offset = error; // 例如 10 -> 10
    }

    // 防翻转拉扯：如果与上一拍差值巨大(即跨越了 ±180° 切线)并在远端发生跳变，暂借方向以平滑驶离 180° 禁区
    if (current_offset > 170.0f && last_offset_angle < -170.0f) {
        current_offset -= 360.0f; 
    } else if (current_offset < -170.0f && last_offset_angle > 170.0f) {
        current_offset += 360.0f; 
    }

    chassis_cmd_send.offset_angle = current_offset;
    // What: 用真实夹角减去当前“虚拟前方”参考，得到底盘跟随专用误差；Why: V 键掉头后新的前方语义只应该影响跟随闭环，不应该改写真实夹角给平移坐标变换使用。
    chassis_cmd_send.follow_offset_angle = theta_format(current_offset - follow_front_offset_deg);
    last_offset_angle = current_offset;

    // What: 仅在 yaw 电机在线时刷新最后有效真实夹角和跟随误差缓存；Why: 后续离线冻结必须拿最近一帧可信结果，而不是默认值或抖动值。
    last_valid_offset_angle = chassis_cmd_send.offset_angle;
    last_valid_follow_offset_angle = chassis_cmd_send.follow_offset_angle;
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
    shoot_cmd_send.shoot_rate = 0.0f;
    friction_switch_state = 0u;
    // What: 零力出口统一清空键鼠和遥控锁存；Why: 否则恢复有力后会把暂停前的旧输入当成当前意图继续执行。
    ResetMouseControlLatchState();
    ResetKeyboardMotionState();
    ResetChassisAuxState();
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
static void ApplyRemoteGimbalStickControl(float rocker_lx, float rocker_ly, int16_t dial_input)
{
    float yaw_sensitivity = 0.001f;
    float pitch_sensitivity = REMOTE_PITCH_SENSITIVITY;
    float yaw_gyro_dps = gimbal_fetch_data.gimbal_imu_data.Gyro[2] * RAD_2_DEGREE;

    if (dial_input > 100) {
        yaw_sensitivity *= 0.3f;
        pitch_sensitivity *= 0.3f;
    }

    // What: 统一复用当前积分式云台手感；Why: 本轮只恢复 VT03 主控，不应该顺带重调云台控制参数。
    gimbal_cmd_send.yaw -= yaw_sensitivity * rocker_lx;
    gimbal_cmd_send.pitch += pitch_sensitivity * rocker_ly;

    // What: 仅在低角速度且非小陀螺工况下，才把 yaw 目标缓慢拉向当前反馈；Why: 原逻辑在小陀螺时会把真实扰动当成“新目标”写回去，表现成云台越转越歪。
    if (rocker_lx == 0.0f &&
        chassis_cmd_send.chassis_mode != CHASSIS_ROTATE &&
        chassis_cmd_send.chassis_mode != CHASSIS_FOLLOW_GIMBAL_YAW &&
        yaw_gyro_dps < YAW_DRIFT_LOCK_GYRO_TH_DPS &&
        yaw_gyro_dps > -YAW_DRIFT_LOCK_GYRO_TH_DPS) {
        gimbal_cmd_send.yaw = YAW_DRIFT_LOCK_COEF * gimbal_cmd_send.yaw +
                              (1.0f - YAW_DRIFT_LOCK_COEF) * gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    }

    LimitGimbalPitchTarget();
}

/**
 * @brief 按当前遥控器手感更新底盘平移指令
 *
 * @param rocker_rx 右摇杆横向
 * @param rocker_ry 右摇杆纵向
 */
static void ApplyRemoteChassisStickControl(float rocker_rx, float rocker_ry)
{
    // What: 继续沿用当前底盘历史量纲；Why: 底盘侧仍按“摇杆值乘 10”解释，不能在恢复 VT03 时再顺手改单位。
    chassis_cmd_send.vx = 10.0f * rocker_ry;
    chassis_cmd_send.vy = 10.0f * rocker_rx;
}

/**
 * @brief 控制输入为 DT7 遥控器时的模式和控制量设置
 *
 */
static void RemoteControlSetDT7(void)
{
    // 1. 获取当前开关状态
    uint16_t current_switch_right = rc_data[TEMP].rc.switch_right;
    uint16_t current_switch_left = rc_data[TEMP].rc.switch_left;
    uint8_t dt7_cali_combo_active = 0u;
    uint8_t left_mid_to_up = (uint8_t)(switch_is_up(current_switch_left) && switch_is_mid(last_switch_left));
    uint8_t left_mid_to_down = (uint8_t)(switch_is_down(current_switch_left) && switch_is_mid(last_switch_left));
    float rocker_lx;
    float rocker_ly;
    float rocker_rx;
    float rocker_ry;

    // What: 每拍先清空辅助机构瞬态命令；Why: 上岛开关是锁存状态，但履带速度和抬升拨轮输入都不应该沿用旧值。
#ifdef USE_ISLAND_ACTION
    chassis_cmd_send.front_track_mode = front_track_switch_state ? FRONT_TRACK_ON : FRONT_TRACK_OFF;
    chassis_cmd_send.lift_mode = LIFT_OFF;
    chassis_cmd_send.front_track_speed_ref = 0.0f;
    chassis_cmd_send.lift_dial_input = 0.0f;
#endif

    // --- 状态机逻辑 ---

    // [下] 急停模式
    if (switch_is_down(current_switch_right)) {
        uint8_t is_inner_eight;

        EmergencyHandler();
        is_inner_eight = (uint8_t)(switch_is_down(current_switch_left) &&
                                   (rc_data[TEMP].rc.rocker_l_ > RC_TRIGGER_TH) &&
                                   (rc_data[TEMP].rc.rocker_l1 < -RC_TRIGGER_TH) &&
                                   (rc_data[TEMP].rc.rocker_r_ < -RC_TRIGGER_TH) &&
                                   (rc_data[TEMP].rc.rocker_r1 < -RC_TRIGGER_TH));
        if (is_inner_eight != 0u) {
            dt7_cali_combo_active = 1u;
            if (cali_triggered == 0u) {
                // What: 在 DT7 急停态内八组合下触发一次 yaw DM 校零；Why: 恢复历史零点设置入口，同时只允许每次组合发送一次指令避免重复改零点。
                GimbalCalibrate();
                yaw_align_offset_deg = 0.0f;
                // What: DT7 触发 yaw 校零时同步复位虚拟前方参考；Why: 物理零位坐标系一旦重建，旧前方向参考会立刻失效，继续沿用只会把底盘跟随带偏。
                follow_front_offset_deg = 0.0f;
                SyncGimbalTargetAndRequestYawReset();
                if (hint_buzzer != NULL) {
                    // What: DT7 校零触发时开启蜂鸣器提示；Why: 操作者需要立即知道本次内八已经成功进入校零链路。
                    AlarmSetStatus(hint_buzzer, ALARM_ON);
                }
                cali_triggered = 1u;
            }
        }
    }
    // [上] 底盘无力，云台能够转动
    else if (switch_is_up(current_switch_right)) {
        if (!switch_is_up(last_switch_right)) {
            // What: DT7 右拨杆模式发生变化时立即取消一键掉头状态；Why: 用户已经通过物理挡位给出新的模式意图，旧掉头任务不应继续抢控制权。
            ClearKeyboardTurnbackState();
            SyncGimbalTargetAndRequestYawReset();
        }

        robot_state = ROBOT_READY;
        // What: 上档恢复为小陀螺 + 云台陀螺仪模式；Why: 保持现有 DT7 主驾驶语义，也避免顶部逻辑和 VT03 分支分叉两套行为。
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }
    // [中] 底盘跟随云台模式
    else if (switch_is_mid(current_switch_right)) {
        if (!switch_is_mid(last_switch_right)) {
            // What: DT7 右拨杆模式发生变化时立即取消一键掉头状态；Why: 从其它模式切进跟随属于新的操控会话，旧掉头任务必须立刻失效。
            ClearKeyboardTurnbackState();
            SyncGimbalTargetAndRequestYawReset();
        }

        robot_state = ROBOT_READY;
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }

    if (dt7_cali_combo_active == 0u && cali_triggered != 0u) {
        // What: 内八组合失效后立即清掉 DT7 校零锁存与提示音；Why: 只有完整重新摆出组合才允许再次触发，避免切模式后蜂鸣器残留。
        if (hint_buzzer != NULL) {
            AlarmSetStatus(hint_buzzer, ALARM_OFF);
        }
        cali_triggered = 0u;
    }

    rocker_lx = ApplyRCDeadzone((float)rc_data[TEMP].rc.rocker_l_);
    rocker_ly = ApplyRCDeadzone((float)rc_data[TEMP].rc.rocker_l1);
    rocker_rx = ApplyRCDeadzone((float)rc_data[TEMP].rc.rocker_r_);
    rocker_ry = ApplyRCDeadzone((float)rc_data[TEMP].rc.rocker_r1);

    // 云台控制量计算 (仅在非急停状态下累加)
    if (!switch_is_down(current_switch_right)) {
        ApplyRemoteGimbalStickControl(rocker_lx, rocker_ly, rc_data[TEMP].rc.dial);
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

    ApplyRemoteChassisStickControl(rocker_rx, rocker_ry);

    // 发射参数
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],弹舱打开
        ; // 弹舱舵机控制,待添加servo_motor模块,开启
    else {
        {
            // 弹舱舵机控制,待添加servo_motor模块,关闭
        };
    }

    // ==============================================================
    // [新增] 安全逻辑: 仅在非急停状态下允许开启摩擦轮、前履带和抬升
    // ==============================================================
    if (!switch_is_down(current_switch_right)) {
        if (left_mid_to_up) {
            // What: 左拨杆上翻优先处理当前功能域；Why: 同一个拨杆要在摩擦轮与上岛辅助机构之间复用，必须按当前锁存状态解释边沿。
            if (friction_switch_state != 0u) {
                friction_switch_state = 0u;
            } else {
#ifdef USE_ISLAND_ACTION
                if (front_track_switch_state == 0u)
#endif
                {
                    friction_switch_state = 1u;
                }
            }
        }

#ifdef USE_ISLAND_ACTION
        if (friction_switch_state == 0u && left_mid_to_down) {
            // What: 仅在摩擦轮关闭时响应履带切换；Why: 保留原有左拨杆下拨开火语义，不让上岛逻辑抢占发射入口。
            front_track_switch_state = !front_track_switch_state;
        }
#endif

        if (friction_switch_state == 1u) {
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

#ifdef USE_ISLAND_ACTION
        chassis_cmd_send.front_track_mode = front_track_switch_state ? FRONT_TRACK_ON : FRONT_TRACK_OFF;
        if (front_track_switch_state != 0u) {
            float lift_dial_input = NormalizeLiftDialInput(rc_data[TEMP].rc.dial);

            // What: 履带开启时下发更保守的固定速度与抬升输入；Why: 先把前履带速度降下来，同时让左拨杆上档就能直接触发后轮抬升。
            chassis_cmd_send.front_track_speed_ref = FRONT_TRACK_SPEED_REF_DEFAULT;
            if (switch_is_up(current_switch_left)) {
                if (lift_dial_input == 0.0f)
                    lift_dial_input = LIFT_SWITCH_UP_INPUT_DEFAULT;

                // What: 左拨杆上档时直接进入抬升调高；Why: 用户操作上希望上档立刻有反应，不需要再依赖隐藏锁存或额外拨轮动作。
                chassis_cmd_send.lift_dial_input = lift_dial_input;
                chassis_cmd_send.lift_mode = LIFT_ADJUST;
            } else {
                // What: 左拨杆离开上档后保持当前位置；Why: 后轮抬起后应稳定保持高度，避免松手就回落。
                chassis_cmd_send.lift_dial_input = 0.0f;
                chassis_cmd_send.lift_mode = LIFT_HOLD;
            }
        }
#endif

    } else {
        // What: 急停时强制清空所有遥控器锁存状态；Why: 解除急停后必须从安全基线重新进入，而不是继续沿用旧会话。
        friction_switch_state = 0u;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        shoot_cmd_send.shoot_rate = 0.0f;
        ResetChassisAuxState();
    }

    // 更新历史状态
    last_switch_left = current_switch_left;
    last_switch_right = current_switch_right;
}

/**
 * @brief 处理 VT03 遥控器发射逻辑
 *
 * @param video_link_remote_state VT03 遥控器状态
 */
static void ApplyVT03ShootLogic(const VideoLinkKM_RemoteState_s *video_link_remote_state)
{
    uint8_t fn_right_pressed;
    uint8_t trigger_pressed;

    if (video_link_remote_state == NULL) {
        return;
    }

    fn_right_pressed = video_link_remote_state->fn_right_button_down;
    trigger_pressed = video_link_remote_state->trigger_button_down;

    if (fn_right_pressed && !vt03_fn_right_last) {
        // What: 使用 VT03 右自定义键切换摩擦轮；Why: VT03 遥控器没有 DT7 的拨杆边沿接口，需要在 cmd 层用自定义键承接启停。
        friction_switch_state = (uint8_t)!friction_switch_state;
    }

    if (friction_switch_state != 0u) {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.bullet_speed = BIG_AMU_12;

        // What: 扳机只响应单发上升沿；Why: 用户要求 VT03 遥控器保持“扳机点一下打一发”的安全语义。
        if (trigger_pressed && !vt03_trigger_last) {
            shoot_cmd_send.load_mode = LOAD_1_BULLET;
        } else {
            shoot_cmd_send.load_mode = LOAD_STOP;
        }
    } else {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        shoot_cmd_send.shoot_rate = 0.0f;
    }

    vt03_fn_right_last = fn_right_pressed;
    vt03_trigger_last = trigger_pressed;
}

/**
 * @brief 控制输入为 VT03 遥控器时的模式和控制量设置
 *
 */
static void RemoteControlSetVT03(void)
{
    const VideoLinkKM_RemoteState_s *video_link_remote_state = VideoLinkKMGetRemoteState();
    uint8_t pause_pressed;
    uint32_t now_ms;
    float rocker_lx;
    float rocker_ly;
    float rocker_rx;
    float rocker_ry;

    if (video_link_remote_state == NULL || video_link_data == NULL) {
        // What: VT03 状态视图失效时只复位 Pause 本次按压会话；Why: 图传临时掉帧不应把已经锁住的零力状态顺手解除，否则链路恢复时会出现“自己突然使能”。
        ResetVT03PausePressState();
        vt03_pause_last = 0u;
        EmergencyHandler();
        return;
    }

    pause_pressed = video_link_remote_state->pause_button_down;
    now_ms = (uint32_t)DWT_GetTimeline_ms();
    if (pause_pressed && !vt03_pause_last) {
        // What: Pause 按下沿启动一次新的长按会话；Why: 需要区分“按下进入零力”“零力中继续长按校零”和“松手退出零力”三种不同语义。
        vt03_pause_press_start_ms = now_ms;
        vt03_pause_longpress_handled = 0u;
        if (vt03_pause_zero_force_latched != 0u) {
            // What: 记录本次 Pause 是在零力态中按下；Why: 只有已经零力时的长按才允许进入 yaw 校零，防止有力态误触。
            vt03_pause_press_started_in_zero_force = 1u;
        } else {
            // What: 非零力态按下 Pause 时立即进入零力；Why: 保留 VT03 的短按失能入口，同时禁止第一次按住就直接跨过零力触发校零。
            vt03_pause_press_started_in_zero_force = 0u;
            vt03_pause_zero_force_latched = 1u;
            vt03_fn_right_last = video_link_remote_state->fn_right_button_down;
            vt03_trigger_last = video_link_remote_state->trigger_button_down;
            vt03_mode_sw_last = video_link_remote_state->mode_sw;
        }
    } else if (pause_pressed &&
               vt03_pause_press_started_in_zero_force != 0u &&
               vt03_pause_longpress_handled == 0u &&
               (now_ms - vt03_pause_press_start_ms) >= VT03_PAUSE_CALIB_HOLD_MS) {
        // What: 零力中长按达到阈值后只触发一次 yaw 校零；Why: DM 零点指令是重动作，同一次按住里重复发送没有收益且会增加风险。
        TriggerVT03YawCalibration();
        vt03_pause_longpress_handled = 1u;
    } else if (!pause_pressed && vt03_pause_last) {
        if (vt03_pause_press_started_in_zero_force != 0u && vt03_pause_longpress_handled == 0u) {
            // What: 零力中短按并松开时退出零力；Why: 用户需要在不校零的情况下，通过一次短按恢复正常有力控制。
            vt03_pause_zero_force_latched = 0u;
            vt03_fn_right_last = video_link_remote_state->fn_right_button_down;
            vt03_trigger_last = video_link_remote_state->trigger_button_down;
            vt03_mode_sw_last = video_link_remote_state->mode_sw;
            SyncGimbalTargetAndRequestYawReset();
        }
        // What: Pause 松开沿统一结束当前按压会话；Why: 下一次按键必须重新从 0 开始计时，蜂鸣器也应在此时关闭。
        ResetVT03PausePressState();
    }
    vt03_pause_last = pause_pressed;

    if (vt03_pause_zero_force_latched != 0u) {
        // What: Pause 锁存零力期间持续更新 VT03 边沿历史；Why: 用户在零力时改挡位或按住扳机，恢复时不应该被当成新的边沿事件。
        vt03_fn_right_last = video_link_remote_state->fn_right_button_down;
        vt03_trigger_last = video_link_remote_state->trigger_button_down;
        vt03_mode_sw_last = video_link_remote_state->mode_sw;
        EmergencyHandler();
        return;
    }

    if (vt03_mode_sw_last != video_link_remote_state->mode_sw) {
        // What: VT03 挡位变化时立即取消一键掉头状态；Why: 操作者已经显式切了主控模式，旧掉头任务不应继续绑死跟随和 yaw 目标。
        ClearKeyboardTurnbackState();
        SyncGimbalTargetAndRequestYawReset();
    }

    robot_state = ROBOT_READY;
    if (video_link_remote_state->mode_sw == VT03_MODE_SW_C) {
        // What: VT03 的 C 挡进入小陀螺 + 云台陀螺仪模式；Why: 用户明确要求切到 C 挡时直接进入小陀螺。
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    } else if (video_link_remote_state->mode_sw == VT03_MODE_SW_N) {
        // What: VT03 的 N 挡进入底盘跟随云台；Why: 保留常规驾驶模式，让 N 挡承担正常跟随操控。
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    } else if (video_link_remote_state->mode_sw == VT03_MODE_SW_S) {
        // What: VT03 的 S 挡进入底盘自由 + 云台自由；Why: 保持遥控器 S 挡作为“完全手动自由控制”的直觉语义。
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    } else {
        // What: 异常挡位兜底回到底盘跟随云台；Why: 协议层虽然已限幅，但上层仍保留保守回退，避免异常值直接打到自由或小陀螺。
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    auto_aim_state = AUTO_AIM_IDLE;

    rocker_lx = ApplyRCDeadzone((float)video_link_data[TEMP].rc.rocker_l_);
    rocker_ly = ApplyRCDeadzone((float)video_link_data[TEMP].rc.rocker_l1);
    rocker_rx = ApplyRCDeadzone((float)video_link_data[TEMP].rc.rocker_r_);
    rocker_ry = ApplyRCDeadzone((float)video_link_data[TEMP].rc.rocker_r1);

    ApplyRemoteGimbalStickControl(rocker_lx, rocker_ly, video_link_data[TEMP].rc.dial);
    ApplyRemoteChassisStickControl(rocker_rx, rocker_ry);

    // What: VT03 遥控分支显式关闭当前未接到图传遥控器上的辅助机构；Why: 本轮只恢复主驾驶和发射，不让上岛逻辑误吃到图传状态。
    ResetChassisAuxState();
    ApplyVT03ShootLogic(video_link_remote_state);
    vt03_mode_sw_last = video_link_remote_state->mode_sw;
}

/**
 * @brief 根据当前主控输入源设置模式和控制量
 *
 * @param control_source 当前主控输入源
 */
static void RemoteControlSet(ControlSource_e control_source)
{
    if (control_source == CONTROL_SOURCE_VT03) {
        RemoteControlSetVT03();
    } else if (control_source == CONTROL_SOURCE_DT7) {
        RemoteControlSetDT7();
    } else {
        EmergencyHandler();
    }
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    const RC_ctrl_t *mouse_key_source = GetActiveMouseKeySource();
    uint8_t video_link_online = IsVideoLinkControlReady();
    uint16_t key_bits = mouse_key_source->key[KEY_PRESS].keys;
    uint8_t friction_toggle_pressed = (uint8_t)((key_bits >> Key_F) & 0x1u);
    uint8_t free_toggle_pressed = (uint8_t)((key_bits >> Key_B) & 0x1u);
    uint8_t turnback_toggle_pressed = (uint8_t)((key_bits >> Key_V) & 0x1u);
    uint8_t ui_refresh_pressed = (uint8_t)((key_bits >> Key_G) & 0x1u);
    uint8_t spin_toggle_pressed = (uint8_t)((key_bits >> Key_X) & 0x1u);
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
        ResetMouseControlLatchState();
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
        ResetMouseControlLatchState();
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

    if (free_toggle_pressed && !keyboard_free_toggle_last) {
        // What: B 键显式切模式前先取消一键掉头状态；Why: 用户已经给出新的自由模式意图，旧掉头任务和强制跟随锁存都不应继续霸占控制权。
        ClearKeyboardTurnbackState();
        // What: 用 B 键上升沿翻转“云台自由 + 底盘自由”锁存；Why: 用户希望按一下直接进入该模式，而不是依赖遥控器档位或持续按住按键。
        keyboard_free_mode_latched = (uint8_t)!keyboard_free_mode_latched;
        // What: 进入自由模式时主动清掉小陀螺锁存；Why: 自由模式与小陀螺控制语义互斥，保留旧锁存会导致模式争抢。
        if (keyboard_free_mode_latched != 0u) {
            keyboard_spin_mode_latched = 0u;
        }
        // What: 自由模式切换边沿统一贴齐当前姿态并清 yaw 残留；Why: 避免模式切换后继续追旧目标导致云台突跳。
        SyncGimbalTargetAndRequestYawReset();
    }
    keyboard_free_toggle_last = free_toggle_pressed;

    if (ui_refresh_pressed && !keyboard_ui_refresh_last) {
        // What: 用 G 键上升沿触发一次底盘 UI 全量刷新；Why: 复用现有底盘 UI 重建入口，比临时拼局部刷新包更稳。
        chassis_cmd_send.ui_refresh_request = 1u;
    }
    keyboard_ui_refresh_last = ui_refresh_pressed;

    if (spin_toggle_pressed && !keyboard_spin_toggle_last) {
        // What: X 键显式切模式前先取消一键掉头状态；Why: 用户已经要求转入小陀螺，旧掉头任务和强制跟随锁存必须立刻失效，避免两个模式同时抢控制权。
        ClearKeyboardTurnbackState();
        // What: 用 X 键上升沿翻转键鼠小陀螺锁存；Why: 用户要求按一次切一次，而不是按住期间临时进入小陀螺。
        keyboard_spin_mode_latched = (uint8_t)!keyboard_spin_mode_latched;
        // What: 进入小陀螺时主动退出自由模式锁存；Why: 两种模式都要最终覆盖底盘/云台状态，互斥处理能避免分支互相打架。
        if (keyboard_spin_mode_latched != 0u) {
            keyboard_free_mode_latched = 0u;
        }
        // What: 小陀螺开关切换时同步云台目标到当前姿态；Why: 避免在自由/跟随/小陀螺之间切换时继续追旧目标产生瞬时跳变。
        SyncGimbalTargetAndRequestYawReset();
    }
    keyboard_spin_toggle_last = spin_toggle_pressed;

    if (turnback_toggle_pressed && !keyboard_turnback_toggle_last) {
        if (gimbal_fetch_data.yaw_motor_online != 0u &&
            keyboard_spin_mode_latched == 0u &&
            keyboard_free_mode_latched == 0u &&
            chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
            float current_yaw_total = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
            float reverse_gimbal_yaw_norm;

            // What: V 键触发的是一次性掉头动作，因此先清掉旧掉头会话残留；Why: 连续多次按 V 时每次都应该从当前姿态重新计算新的 180 度目标，而不是继承旧会话的计时和稳定计数。
            ClearKeyboardTurnbackState();
            // What: 启动掉头前先把目标贴齐当前姿态并清 yaw 速度环残留；Why: 这样新的 180 度动作会从当前真实反馈平滑起步，而不是叠着旧目标或旧积分直接猛转。
            SyncGimbalTargetAndRequestYawReset();

            // What: 直接以当前云台世界系朝向作为掉头参考；Why: 用户现在要的是“当前看到哪就以该方向反向 180 度作为新前方”，而不是先回到底盘中线再反向。
            reverse_gimbal_yaw_norm = theta_format(current_yaw_total + 180.0f);
            // What: 把“当前云台朝向反向 180 度”的单圈目标映射成距离当前累计角最近的累计角；Why: 云台 yaw 闭环吃的是累计角，仍要选最近等价目标来避免无意义的跨圈。
            keyboard_turnback_target_yaw = FindClosestCumulativeAngle(current_yaw_total, reverse_gimbal_yaw_norm);
            // What: 记录这次掉头完成后应提交的新前方向参考；Why: 用户要求底盘在掉头期间不动，但完成后又要把反转后的云台朝向认定为新的前方。
            keyboard_turnback_follow_front_target_deg = theta_format(chassis_cmd_send.offset_angle + 180.0f);
            // What: 掉头触发拍立即进入执行态；Why: 用户要的是按一次立刻开始云台 180 度掉头，而不是等待下一拍再起动作。
            keyboard_turnback_active = 1u;
            keyboard_turnback_start_ms = now_ms;
            keyboard_turnback_stable_ticks = 0u;
            // What: 触发拍直接把 yaw 目标切到本次掉头目标；Why: 这样同一拍下发出去的就是新的 180 度参考，不需要再等下一控制周期才起动作。
            gimbal_cmd_send.yaw = keyboard_turnback_target_yaw;
        }
    }
    keyboard_turnback_toggle_last = turnback_toggle_pressed;

    if (keyboard_turnback_active != 0u) {
        float yaw_error_deg = fabsf(theta_format(keyboard_turnback_target_yaw - gimbal_fetch_data.gimbal_imu_data.YawTotalAngle));

        // What: 一键掉头执行期间持续请求底盘进入“只阻尼刹停”子状态；Why: 当前用户反馈的前段跟转来自跟随环历史量和云台角速度前馈，这里必须让底盘暂时完全退出追随云台。
        chassis_cmd_send.follow_brake_request = 1u;
        // What: 一键掉头执行期间强制把跟随误差压成 0；Why: 用户明确要求整个动作只有云台转动，底盘不能因为看到真实夹角变大就跟着补转。
        chassis_cmd_send.follow_offset_angle = 0.0f;
        // What: 同步把最近可信跟随误差也更新为 0；Why: 若 yaw 在掉头中途离线，底盘侧冻结时也必须继续保持“不跟着转”的结果。
        last_valid_follow_offset_angle = 0.0f;

        // What: yaw 电机若在执行途中离线就立刻取消本次掉头；Why: 失去可靠反馈后已经无法确认云台真实到位情况，继续执行或提交新前方都会把整车语义带偏。
        if (gimbal_fetch_data.yaw_motor_online == 0u) {
            ClearKeyboardTurnbackState();
        }
        // What: 一键掉头执行态次优先检查超时；Why: 机构阻塞或姿态估计异常时不能一直霸占目标强制覆盖，超时后必须退出并保留旧前方语义。
        else if ((now_ms - keyboard_turnback_start_ms) >= TURNBACK_TIMEOUT_MS) {
            ClearKeyboardTurnbackState();
        }
        // What: 只有云台目标误差足够小，才累计“完成稳定拍数”；Why: 这次需求明确只让云台自己转到位，底盘并不参与完成判据。
        else if (yaw_error_deg < TURNBACK_COMPLETE_YAW_ERROR_DEG) {
            if (keyboard_turnback_stable_ticks < TURNBACK_COMPLETE_STABLE_TICKS) {
                keyboard_turnback_stable_ticks++;
            }
            // What: 连续多拍稳定满足完成条件后提交新的前方向参考并退出执行态；Why: 这样可以滤掉短暂穿越阈值的假到位，只让真正完成的掉头改写后续前方语义。
            if (keyboard_turnback_stable_ticks >= TURNBACK_COMPLETE_STABLE_TICKS) {
                follow_front_offset_deg = keyboard_turnback_follow_front_target_deg;
                ClearKeyboardTurnbackState();
            }
        } else {
            // What: 只要任一完成条件被破坏，就重新开始稳定计数；Why: 真正完成必须连续稳定，而不是零星几拍偶然接近目标。
            keyboard_turnback_stable_ticks = 0u;
        }
    }

    // 鼠标首次请求开火时锁存摩擦轮开启，作用是让后续短按/长按都不需要额外按键预热；
    // 原因是你本轮只要求接入 mouse.press_l，不能再依赖 F/Q/E 之类的旧调试键位。
    if (mouse_left_pressed && !mouse_left_last) {
        mouse_fire_friction_latched = 1;
        mouse_left_burst_active = 0;
        mouse_left_press_start_ms = now_ms;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.bullet_speed = BIG_AMU_12;
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
            shoot_cmd_send.bullet_speed = BIG_AMU_12;
        }
    }

    // 长按进入连发，作用是让 mouse.press_l 兼顾点射和持续火力；
    // 原因是 `shoot` 应用对 `LOAD_BURSTFIRE` 是电平语义，必须在长按期间持续下发。
    if (mouse_left_burst_active) {
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        shoot_cmd_send.shoot_rate = MOUSE_BURST_FIRE_RATE;
    }

    if (keyboard_turnback_active != 0u) {
        // What: 一键掉头执行期间只强制保持云台工作在陀螺仪模式；Why: 掉头动作本身只属于云台 yaw 目标覆盖，不应该顺带改写底盘模式导致底盘跟着转。
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        // What: 仅在掉头执行态内持续覆盖 yaw 目标；Why: 执行完成后应恢复普通跟随下的手动 yaw 控制，而不是继续把云台锁死在旧目标上。
        gimbal_cmd_send.yaw = keyboard_turnback_target_yaw;
    } else if (keyboard_spin_mode_latched != 0u) {
        // What: X 锁存生效后在键鼠层最终覆盖成小陀螺 + 陀螺仪模式；Why: 用户要求 X 优先于当前遥控器挡位，直到再次按 X 或被安全链清锁。
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    } else if (keyboard_free_mode_latched != 0u) {
        // What: B 锁存生效后最终覆盖成底盘自由平移 + 云台自由控制；Why: 用户要求键盘提供一个不依赖遥控器档位的自由模式快捷入口。
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    }

    mouse_left_last = mouse_left_pressed;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    ControlSource_e active_control_source;

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
    if (gimbal_fetch_data.yaw_motor_online != 0u && last_yaw_motor_online == 0u) {
        // What: yaw 电机复活第一拍就把目标贴回当前姿态；Why: 死亡期间若仍保留旧 yaw 目标，恢复后云台会试图回拉到旧参考导致头发歪。
        SyncGimbalTargetAndRequestYawReset();
    }
    last_yaw_motor_online = gimbal_fetch_data.yaw_motor_online;

    // 每拍先清理一次瞬态控制量，作用是让后续遥控器和键鼠都从同一安全基线开始叠加；
    // 原因是当前 `robot_cmd` 已经不是互斥控制源，直接沿用上拍结果会产生残留指令。
    PrepareControlCommandBase();
    if (yaw_pid_reset_hold_ticks != 0u) {
        // What: 在请求后的若干拍持续下发 yaw PID 复位位；Why: Pub/Sub 队列深度为 1，持续几拍才能避免 producer 先跑两次把边沿消息顶掉。
        gimbal_cmd_send.yaw_pid_reset_request = 1u;
        yaw_pid_reset_hold_ticks--;
    }
    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    // What: 底盘跟随始终围绕既定零位计算偏角；Why: 上电/重连时若把“当前角度”重新写成零位，会直接把真实偏角清掉，表现成无法自动回正甚至越跟越歪。
    CalcOffsetAngle();
    active_control_source = GetActiveControlSource();
    HandleControlSourceSwitch(active_control_source);

    // What: VT03 Pause 锁存后，即使主控短暂切到 DT7 或 NONE 也必须继续零力；Why: 用户按下暂停就是明确的失能意图，不能因为链路抖动或回退仲裁而自动恢复使能。
    if (vt03_pause_zero_force_latched != 0u && active_control_source != CONTROL_SOURCE_VT03) {
        EmergencyHandler();
    }
    // What: 先收口主遥控源，再叠加当前主控对应的键鼠输入；Why: 需要让 VT03 遥控器和电脑控制同链路共存，同时保留 DT7 断链回退。
    else if (active_control_source == CONTROL_SOURCE_NONE) {
        EmergencyHandler();
    } else {
        RemoteControlSet(active_control_source);
        MouseKeySet();
    }
    if (last_effective_chassis_mode == CHASSIS_ROTATE &&
        chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
        // What: 仅在最终有效模式从小陀螺切到底盘跟随时请求接管窗口；Why: 只有最终输出层的边沿才覆盖了遥控器、VT03 与键鼠锁存的全部竞争结果。
        RequestFollowTransition();
    }
    last_effective_chassis_mode = chassis_cmd_send.chassis_mode;

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
    if (chassis_cmd_send.chassis_mode != CHASSIS_ZERO_FORCE) {
        // What: 机器人处于有力模式时默认请求超电介入；Why: 用户要求平时超电常开，且底盘侧额外功率策略依赖 cap_mode，必须在非零力态持续置位。
        chassis_cmd_send.cap_mode = SUPER_CAP_ON;
    }
    if (follow_transition_request_hold_ticks != 0u) {
        // What: 在请求后的若干拍持续下发底盘跟随接管位；Why: 双板命令缓冲只保留最新帧，持续几拍才能避免边沿请求被覆盖丢失。
        chassis_cmd_send.follow_transition_request = 1u;
        follow_transition_request_hold_ticks--;
    }
    chassis_cmd_send.gimbal_gyro_z = gimbal_fetch_data.gimbal_imu_data.Gyro[2] * RAD_2_DEGREE; // 假设原始是弧度，转成度
    chassis_cmd_send.gimbal_pitch_deg = gimbal_fetch_data.gimbal_imu_data.Pitch; // What: 把云台实际pitch姿态随底盘命令一起下发；Why: 底盘裁判 UI 的俯仰滑块必须跟随机构真实位置实时移动。
    chassis_cmd_send.friction_on = (shoot_cmd_send.friction_mode == FRICTION_ON) ? 1u : 0u; // What: 把摩擦轮真实启停状态显式下发到底盘；Why: fric 指示要反映上板最终发射使能，而不是底盘侧猜测。

#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
