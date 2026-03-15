/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "can.h"
#include "motor_def.h"
#include "robot_def.h"
#include "power_control.h"
#include "dmmotor.h"
#ifdef USE_SUPER_CAP
#include "super_cap.h" // [条件编译] 仅在启用超电时包含此头文件
#endif
#include "message_center.h"
#include "referee_task.h"
#include "usart.h"
#include <arm_math.h> // for fabsf
#include "buzzer.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f) // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f) // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define DEFAULT_TEST_POWER 300.0f // 调试用的基础功率
#define REFEREE_KEYMOUSE_TIMEOUT_MS 100u // What: 定义键鼠超时窗口；Why: 100ms内无新帧就清零，避免粘键
#define REFEREE_MOUSE_DELTA_LIMIT 660 // What: 限制单帧鼠标增量；Why: 与旧DBUS鼠标尺度对齐，防止重连时大跳变
#define LIFT_DIAL_MAX_SPEED_DPS 65.0f // What: 定义抬升拨轮映射后的最大调节角速度；Why: 提高拨轮调高响应速度，让抬升更跟手，但仍保持位置环可控范围
#define LIFT_RELATIVE_MIN_ANGLE -20.0f // What: 定义相对进入点的最小抬升角；Why: 给机构保留回落空间，同时避免误操作撞下限
#define LIFT_RELATIVE_MAX_ANGLE 80.0f // What: 定义相对进入点的最大抬升角；Why: 先用保守软件限位保护机构，后续再按实车行程放宽
#define CHASSIS_TASK_DT_FALLBACK 0.005f // What: 定义底盘任务积分后备周期；Why: DWT异常时仍按200Hz近似积分，避免抬升目标突变

typedef struct
{
    uint16_t last_x_position; // What: 保存上一帧裁判鼠标绝对X；Why: 用于把0x0306绝对坐标转换成cmd层增量
    uint16_t last_y_position; // What: 保存上一帧裁判鼠标绝对Y；Why: 用于把0x0306绝对坐标转换成cmd层增量
    uint32_t last_update_tick_ms; // What: 保存上一帧键鼠专属时间戳；Why: 只在新键鼠帧到达时更新增量，避免重复消费旧值
    uint8_t baseline_ready; // What: 标记是否已经建立鼠标增量基准；Why: 重连首帧必须归零，避免云台瞬时抽动
} Referee_KeyMouse_Sync_s;

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
// What: 编译期校验底盘上传结构体尺寸；Why: 防止新增键鼠字段后悄悄超过单帧CAN通信上限
_Static_assert(sizeof(Chassis_Upload_Data_s) <= CAN_COMM_MAX_BUFFSIZE,
               "Chassis_Upload_Data_s exceeds CAN_COMM_MAX_BUFFSIZE");
// What: 编译期校验底盘接收结构体尺寸；Why: 新增履带/抬升控制字段后仍要保证单帧CAN通信可以完整传输
_Static_assert(sizeof(Chassis_Ctrl_Cmd_s) <= CAN_COMM_MAX_BUFFSIZE,
               "Chassis_Ctrl_Cmd_s exceeds CAN_COMM_MAX_BUFFSIZE");
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub; // 用于发布底盘的数据
static Subscriber_t *chassis_sub; // 用于订阅底盘的控制命令
#endif // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv; // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static referee_info_t *referee_data = { NULL }; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

#ifdef USE_SUPER_CAP
static SuperCapInstance *cap = { NULL }; // [条件编译] 超级电容实例指针
#endif
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
static DMMotorInstance *front_track_motor_left, *front_track_motor_right; // What: 保存前履带DM实例；Why: 底盘任务需要直接下发速度闭环参考并处理启停
static DJIMotorInstance *lift_motor_left, *lift_motor_right; // What: 保存后抬升3508实例；Why: 底盘任务需要基于真实角度执行双环位置控制

// 方案二，陀螺仪针对全麦纠偏PID
static PIDInstance yaw_lock_pid; // 航向锁定专用PID
static float lock_target_yaw = 0.0f; // 锁定的目标角度
static uint8_t is_manual_rotating = 0; // 标记是否正在手动旋转

// 底盘蜂鸣器初始化 [!code ++]
static BuzzzerInstance *chassis_buzzer = NULL; // [!code ++]
static uint8_t last_cali_flag = 0; // 上一次的校准标志位

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy; // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

static float real_vx = 0.0f; // 真实前进速度 m/s
static float real_vy = 0.0f; // 真实横移速度 m/s
static float real_wz = 0.0f; // 真实旋转速度 deg/s
static Referee_KeyMouse_Sync_s referee_keymouse_sync; // What: 保存裁判键鼠增量转换状态；Why: 将绝对坐标安全适配成增量输入
static float lift_target_angle = 0.0f; // What: 保存抬升机构当前机械目标角；Why: 退出抬升模式后仍需保持最后高度而不是失控回落
static float lift_entry_angle = 0.0f; // What: 保存本次进入抬升模式时的机械零位；Why: 软件限位以进入点为参考，避免依赖固定机械绝对零点
static uint8_t lift_ref_ready = 0u; // What: 标记抬升位置环是否已经建立参考；Why: 等待首帧编码器反馈后再切角度环，避免上电乱冲
static lift_mode_e last_lift_mode = LIFT_OFF; // What: 保存上一帧抬升模式；Why: 通过边沿检测在进入抬升模式时重建当前零位
static uint32_t lift_control_dt_cnt = 0u; // What: 保存抬升积分时间戳；Why: 用DWT获取真实任务周期，避免拨轮调高受调度抖动影响

// [!code ++]
// ==========================================
// [新增] 小陀螺模式配置
// ==========================================
#define VARIABLE_SPIN_ENABLED 1 // 1: 启用变速小陀螺; 0: 启用优化后的匀速小陀螺
#define SPIN_TOP_MAX_SPEED 2000.0f // 小陀螺最大旋转速度 (deg/s) 降低转速以防电机过度饱和发生偏航漂移
#define TRANSLATION_PRIORITY_RATIO 0.6f // 平移优先系数 (0~1)，越大则平移时旋转降速越明显

/**
 * @brief 计算小陀螺的旋转目标速度 (核心优化逻辑：平移优先)
 *
 * @param base_target_wz 基础目标旋转速度 (如果是变速模式，这里输入的是随时间变化的值)
 * @param vx_cmd 当前的前进指令 (来自摇杆解析，范围对应最大如 6600)
 * @param vy_cmd 当前的横移指令 (来自摇杆解析，范围对应最大如 6600)
 * @return float 最终的旋转速度 target_wz
 */
static float OptimizedSpinSpeed(float base_target_wz, float vx_cmd, float vy_cmd)
{
    // 1. 计算当前的平移需求幅度
    float trans_speed = sqrtf(vx_cmd * vx_cmd + vy_cmd * vy_cmd);

    // 2. 归一化平移强度 (0.0 ~ 1.0)
    // 根据 robot_cmd.c 中的摇杆解算逻辑，推杆满位时 vx/vy 数值可达 6600。
    // 将阈值修正为 6600.0f 以匹配当前的指令单位尺度，恢复摇杆的线性与平移灵敏度。
    const float MAX_TRANS_SPEED_REF = 6600.0f;
    float trans_ratio = trans_speed / MAX_TRANS_SPEED_REF;
    if (trans_ratio > 1.0f)
        trans_ratio = 1.0f;

    // 3. 计算旋转缩放系数
    // 当 trans_ratio = 0 (静止) -> scale = 1.0 (全速旋转)
    // 当 trans_ratio = 1 (全速平移) -> scale = (1 - TRANSLATION_PRIORITY_RATIO) (例如 0.4)
    // 这样保证了全速移动时，旋转速度不会完全降为0（保留一点小陀螺效果），但主要功率留给平移
    float spin_scale = 1.0f - (trans_ratio * TRANSLATION_PRIORITY_RATIO);

    // 4. 应用缩放
    return base_target_wz * spin_scale;
}

/**
 * @brief 生成变速小陀螺的波形
 *
 * @return float 当前时刻的基础旋转速度
 */
static float GetVariableSpinBase()
{
    // 使用 HAL_GetTick() 获取时间 (ms)
    uint32_t current_time = HAL_GetTick();

    // 周期设计：
    // 正弦波: A * sin(2*pi*f*t) + Offset
    // 设周期 T = 2秒 (f=0.5Hz)
    // 为避免电机在小陀螺时转速溢出，我们将振幅和均值同比下调
    float time_sec = current_time / 1000.0f;
    const float SPIN_PERIOD = 2.0f; // 周期 2秒
    const float SPIN_AMP = 150.0f; // 波动幅度 (降为原来的1/10)
    const float SPIN_OFFSET = 450.0f; // 基础均值

    // 2 * PI * f * t = 2 * PI * (1/T) * t
    float phase = 2.0f * PI * (1.0f / SPIN_PERIOD) * time_sec;

    // 生成波形
    float wave = arm_sin_f32(phase) * SPIN_AMP + SPIN_OFFSET;

    return wave;
}

static int16_t ClampRefereeMouseDelta(int32_t raw_delta)
{
    // What: 将裁判鼠标单帧位移限幅到旧DBUS鼠标范围；Why: 统一手感并抑制重连/跳帧带来的异常大增量
    if (raw_delta > REFEREE_MOUSE_DELTA_LIMIT)
        return REFEREE_MOUSE_DELTA_LIMIT;
    if (raw_delta < -REFEREE_MOUSE_DELTA_LIMIT)
        return -REFEREE_MOUSE_DELTA_LIMIT;
    return (int16_t)raw_delta;
}

static void ClearRefereeKeyMouseUpload(void)
{
    // What: 清空当前待上传的裁判键鼠状态；Why: 超时或未接线时必须显式归零，避免cmd层继续使用旧输入
    chassis_feedback_data.referee_keymouse.key_value = 0u;
    chassis_feedback_data.referee_keymouse.mouse_dx = 0;
    chassis_feedback_data.referee_keymouse.mouse_dy = 0;
    chassis_feedback_data.referee_keymouse.mouse_left = 0u;
    chassis_feedback_data.referee_keymouse.mouse_right = 0u;
    chassis_feedback_data.referee_keymouse.online = 0u;
}

static void UpdateRefereeKeyMouseUpload(void)
{
    uint32_t now_tick = HAL_GetTick();
    uint32_t custom_client_tick = 0u;
    const ext_custom_client_data_t *custom_client = NULL;

    // What: 每个控制周期先把鼠标增量归零；Why: 增量只允许消费一次，避免在cmd层重复施加同一帧位移
    chassis_feedback_data.referee_keymouse.mouse_dx = 0;
    chassis_feedback_data.referee_keymouse.mouse_dy = 0;

    if (referee_data == NULL) {
        ClearRefereeKeyMouseUpload();
        referee_keymouse_sync.baseline_ready = 0u;
        return;
    }

    custom_client_tick = RefereeGetCustomClientLastUpdateTick();
    if (RefereeGetCustomClientFrameCount() == 0u) {
        ClearRefereeKeyMouseUpload();
        referee_keymouse_sync.baseline_ready = 0u;
        return;
    }

    if ((uint32_t)(now_tick - custom_client_tick) > REFEREE_KEYMOUSE_TIMEOUT_MS) {
        ClearRefereeKeyMouseUpload();
        referee_keymouse_sync.baseline_ready = 0u;
        return;
    }

    custom_client = &referee_data->CustomClientData;
    chassis_feedback_data.referee_keymouse.online = 1u;
    chassis_feedback_data.referee_keymouse.key_value = custom_client->key_value;
    chassis_feedback_data.referee_keymouse.mouse_left = (uint8_t)(custom_client->mouse_left != 0u);
    chassis_feedback_data.referee_keymouse.mouse_right = (uint8_t)(custom_client->mouse_right != 0u);

    if (custom_client_tick != referee_keymouse_sync.last_update_tick_ms ||
        referee_keymouse_sync.baseline_ready == 0u) {
        if (referee_keymouse_sync.baseline_ready != 0u) {
            chassis_feedback_data.referee_keymouse.mouse_dx =
                ClampRefereeMouseDelta((int32_t)custom_client->x_position - (int32_t)referee_keymouse_sync.last_x_position);
            chassis_feedback_data.referee_keymouse.mouse_dy =
                ClampRefereeMouseDelta((int32_t)custom_client->y_position - (int32_t)referee_keymouse_sync.last_y_position);
        }

        referee_keymouse_sync.last_x_position = custom_client->x_position;
        referee_keymouse_sync.last_y_position = custom_client->y_position;
        referee_keymouse_sync.last_update_tick_ms = custom_client_tick;
        referee_keymouse_sync.baseline_ready = 1u;
    }
}

/**
 * @brief 读取单个抬升电机的统一机械角度。
 * @param motor 目标抬升电机实例指针。
 * @return 已按安装方向折算后的机械角度，单位与 `total_angle` 保持一致。
 * @note What: 把左右两台安装方向不同的 3508 编码器角度统一换算到同一机械正方向。
 * @note Why: 双电机抬升必须共享同一个位置目标，若直接使用各自原始反馈，反装电机会把同一高度解释成相反方向，导致位置环相互对冲。
 */
static float GetLiftMotorMechanicalAngle(const DJIMotorInstance *motor)
{
    float mechanical_angle = motor->measure.total_angle;

    // What: 将带反向安装的3508角度折算回统一机械正方向；Why: 左右抬升电机必须共享同一个目标角，不能直接使用各自原始编码器符号
    if (motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        mechanical_angle = -mechanical_angle;

    return mechanical_angle;
}

/**
 * @brief 判断抬升电机是否已经收到可信的编码器反馈。
 * @return `1u` 表示至少收到过一侧有效反馈，`0u` 表示当前仍不能建立位置参考。
 * @note What: 通过 `ecd` 和 `total_round` 是否脱离上电初值来判断 3508 反馈是否就绪。
 * @note Why: 位置外环依赖真实当前位置，若在首帧反馈到来前直接闭环，会把默认零值误当成机械零位，造成上电瞬时回拉。
 */
static uint8_t IsLiftFeedbackReady(void)
{
    if (lift_motor_left == NULL || lift_motor_right == NULL)
        return 0u;

    // What: 用编码器首帧反馈作为位置环启用条件；Why: 3508上电瞬间total_angle默认是0，过早切位置环会把抬升误拉回假零点
    return (uint8_t)(lift_motor_left->measure.ecd != 0u || lift_motor_right->measure.ecd != 0u ||
                     lift_motor_left->measure.total_round != 0 || lift_motor_right->measure.total_round != 0);
}

/**
 * @brief 计算双抬升电机当前的平均机械角度。
 * @return 左右抬升电机在统一方向下的平均角度。
 * @note What: 对两台抬升电机的机械角进行平均，得到机构整体当前高度的代表值。
 * @note Why: 双边联动机构允许存在少量装配误差，直接取平均值比只信任单侧反馈更稳健，也能减小左右编码器微小偏差对目标重建的影响。
 */
static float GetLiftAverageMechanicalAngle(void)
{
    return 0.5f * (GetLiftMotorMechanicalAngle(lift_motor_left) + GetLiftMotorMechanicalAngle(lift_motor_right));
}

/**
 * @brief 将抬升目标角与模式进入参考角同步到当前位置。
 * @return 无。
 * @note What: 同时刷新 `lift_target_angle` 和 `lift_entry_angle`，把当前位置作为当前 session 的控制起点。
 * @note Why: 抬升限位采用“相对本次进入模式的零位”策略，进入模式、急停恢复或失能后都需要重建参考，否则旧目标会跨 session 延续并引发突跳。
 */
static void SyncLiftTargetToCurrent(void)
{
    lift_target_angle = GetLiftAverageMechanicalAngle();
    lift_entry_angle = lift_target_angle;
}

/**
 * @brief 在抬升反馈就绪后建立位置环参考并切换到角度外环。
 * @return 无。
 * @note What: 检测到首帧可信反馈后，初始化抬升目标角、记录当前进入参考角，并把两台 3508 的外环切到角度模式。
 * @note Why: 抬升初始化阶段必须先静止等待真实编码器值，等参考可靠后再闭位置环，才能避免上电时因假零点造成的突发回零动作。
 */
static void EnsureLiftReferenceReady(void)
{
    if (lift_ref_ready != 0u || IsLiftFeedbackReady() == 0u)
        return;

    // What: 收到首帧有效反馈后再切到角度外环；Why: 这样抬升上电先静止，等当前位置可信后再进入位置保持
    lift_ref_ready = 1u;
    SyncLiftTargetToCurrent();
    DJIMotorOuterLoop(lift_motor_left, ANGLE_LOOP);
    DJIMotorOuterLoop(lift_motor_right, ANGLE_LOOP);
    DJIMotorSetRef(lift_motor_left, lift_target_angle);
    DJIMotorSetRef(lift_motor_right, lift_target_angle);
}

/**
 * @brief 安全停用前履带与抬升辅助机构。
 * @return 无。
 * @note What: 将前履带 DM 速度参考清零并停机，同时让抬升在当前位置重建目标后停机，最后清空抬升模式边沿状态。
 * @note Why: 急停或零力模式下不仅要切断输出，还要把抬升保持目标更新到当前实际位置，否则重新使能时会追旧目标；履带也必须同步清零避免残余速度指令继续发送。
 */
static void StopAuxActuators(void)
{
    if (front_track_motor_left != NULL) {
        DMMotorSetRef(front_track_motor_left, 0.0f);
        DMMotorStop(front_track_motor_left);
    }
    if (front_track_motor_right != NULL) {
        DMMotorSetRef(front_track_motor_right, 0.0f);
        DMMotorStop(front_track_motor_right);
    }

    if (lift_ref_ready != 0u && lift_motor_left != NULL && lift_motor_right != NULL)
        SyncLiftTargetToCurrent();

    if (lift_motor_left != NULL) {
        DJIMotorSetRef(lift_motor_left, lift_target_angle);
        DJIMotorStop(lift_motor_left);
    }
    if (lift_motor_right != NULL) {
        DJIMotorSetRef(lift_motor_right, lift_target_angle);
        DJIMotorStop(lift_motor_right);
    }

    // What: 急停后把上一帧抬升模式清零；Why: 解除急停时应先以当前位置重建零位，避免沿用旧session限位窗口突然补偿
    last_lift_mode = LIFT_OFF;
}

/**
 * @brief 按当前底盘命令控制前履带 DM3519 速度闭环。
 * @return 无。
 * @note What: 根据 `front_track_mode` 对两台前履带 DM 执行启停和速度参考下发，关闭时立即给零并停机。
 * @note Why: 前履带的主要需求是速度闭环且目标值可调，单独收口成函数能让它与麦轮底盘功率分配解耦，减少后续调速或换 CAN ID 时的改动面。
 */
static void ControlFrontTrackMotors(void)
{
    if (front_track_motor_left == NULL || front_track_motor_right == NULL)
        return;

    if (chassis_cmd_recv.front_track_mode == FRONT_TRACK_ON) {
        // What: 前履带开启时给两台DM同一机械速度参考；Why: 方向差异交给电机反向标志处理，应用层只维护一套履带速度目标
        DMMotorEnable(front_track_motor_left);
        DMMotorEnable(front_track_motor_right);
        DMMotorSetRef(front_track_motor_left, chassis_cmd_recv.front_track_speed_ref);
        DMMotorSetRef(front_track_motor_right, chassis_cmd_recv.front_track_speed_ref);
    } else {
        DMMotorSetRef(front_track_motor_left, 0.0f);
        DMMotorSetRef(front_track_motor_right, 0.0f);
        DMMotorStop(front_track_motor_left);
        DMMotorStop(front_track_motor_right);
    }
}

/**
 * @brief 按当前辅助模式控制后抬升 3508 的双环位置闭环。
 * @return 无。
 * @note What: 在反馈就绪后维护抬升 session 参考角，按拨轮输入积分目标高度，执行相对限位，并将最终角度参考下发到双 3508。
 * @note Why: 抬升要求“拨轮改高度、松手保持当前位置”，因此需要位置外环和速度内环配合；把积分、建零和限位集中到这里，能保证模式切换、调度抖动和双板命令更新都不会破坏位置保持行为。
 */
static void ControlLiftMotors(void)
{
    float dt;
    lift_mode_e requested_lift_mode;

    if (lift_motor_left == NULL || lift_motor_right == NULL)
        return;

    EnsureLiftReferenceReady();
    if (lift_ref_ready == 0u)
        return;

    requested_lift_mode = (chassis_cmd_recv.front_track_mode == FRONT_TRACK_ON) ? chassis_cmd_recv.lift_mode : LIFT_OFF;
    if (last_lift_mode == LIFT_OFF && requested_lift_mode != LIFT_OFF)
        SyncLiftTargetToCurrent();

    dt = DWT_GetDeltaT(&lift_control_dt_cnt);
    if (dt <= 0.0f || dt > 0.05f)
        dt = CHASSIS_TASK_DT_FALLBACK;

    if (requested_lift_mode == LIFT_ADJUST)
        lift_target_angle += chassis_cmd_recv.lift_dial_input * LIFT_DIAL_MAX_SPEED_DPS * dt;

    // What: 抬升限位以本次进入模式的当前位置为零点；Why: 不依赖固定机械绝对零位，调试阶段改装机构后也不会立刻失效
    LIMIT_MIN_MAX(lift_target_angle, lift_entry_angle + LIFT_RELATIVE_MIN_ANGLE, lift_entry_angle + LIFT_RELATIVE_MAX_ANGLE);

    DJIMotorEnable(lift_motor_left);
    DJIMotorEnable(lift_motor_right);
    DJIMotorSetRef(lift_motor_left, lift_target_angle);
    DJIMotorSetRef(lift_motor_right, lift_target_angle);
    last_lift_mode = requested_lift_mode;
}

void ChassisInit()
{
    Chassis_IMU_data = INS_Init();
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 4.2, // 4.5 3.7
                .Ki = 0.0, // 0.2
                .Kd = 0.05, // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
                .Output_LPF_RC = 0.1,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 设置为开环，电机设定值由下面的功率控制设定，不走普通的pid
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    // 使用功率控制的电机需要使用PowerControlInit()函数初始化,因为电机的控制方式不同
    chassis_motor_config.can_init_config.can_handle = &hcan2;
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL; //
    motor_lf = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle = &hcan2;
    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle = &hcan2;
    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle = &hcan2;
    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = PowerControlInit(&chassis_motor_config);

    Motor_Init_Config_s front_track_motor_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 0.9f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 6.0f,
                .MaxOut = 18.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = MOTOR_TYPE_NONE,
    };

    // What: 前履带使用DM3519速度闭环模板初始化；Why: 复用现有DM封装，只把速度目标作为外部可调参数暴露给cmd层
    front_track_motor_config.can_init_config.tx_id = 0x07;
    front_track_motor_config.can_init_config.rx_id = 0x08;
    front_track_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    front_track_motor_left = DMMotorInit(&front_track_motor_config);

    front_track_motor_config.can_init_config.tx_id = 0x09;
    front_track_motor_config.can_init_config.rx_id = 0x10;
    front_track_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    front_track_motor_right = DMMotorInit(&front_track_motor_config);

    Motor_Init_Config_s lift_motor_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 12.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 4000.0f,
                .MaxOut = 2500.0f,
            },
            .speed_PID = {
                .Kp = 4.5f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 4000.0f,
                .MaxOut = 16000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = M3508,
    };

    // What: 抬升3508先以上电零速度方式初始化；Why: 等待首帧编码器反馈后再切位置环，避免当前位置未知时上电回零乱冲
    lift_motor_config.can_init_config.tx_id = 6;
    lift_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    lift_motor_left = DJIMotorInit(&lift_motor_config);

    lift_motor_config.can_init_config.tx_id = 5;
    lift_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    lift_motor_right = DJIMotorInit(&lift_motor_config);

    ClearRefereeKeyMouseUpload(); // What: 上电先清空键鼠上传缓存；Why: 避免CAN另一侧在首帧前读到脏数据
    referee_keymouse_sync.last_x_position = 0u;
    referee_keymouse_sync.last_y_position = 0u;
    referee_keymouse_sync.last_update_tick_ms = 0u;
    referee_keymouse_sync.baseline_ready = 0u;
    // referee_data = RefereeInit(&huart6); // What: 初始化底盘板裁判接收；Why: 双板方案下0x0306需要先在底盘侧解析再经CAN回传
    PowerControl_EnableSlopeComp(0);

#ifdef USE_SUPER_CAP
    // [条件编译] 超级电容初始化配置
    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x061, // 超级电容默认接收id
            .rx_id = 0x051, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }
    };
    cap = SuperCapInit(&cap_conf); // 初始化超级电容模块
#endif // USE_SUPER_CAP

    // 底盘蜂鸣器初始化 [!code ++]
    Buzzer_config_s buzzer_config = {
        .alarm_level = ALARM_LEVEL_ABOVE_MEDIUM,
        .octave = OCTAVE_4, // 使用中音，区别于云台的高音
        .loudness = 0.5f, // 音量
    };
    chassis_buzzer = BuzzerRegister(&buzzer_config); // [!code ++]
    // 底盘跟随云台

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x011,
            .rx_id = 0x012,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
        .daemon_count = 200,
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化

    LOGINFO("[can_comm] Chassis_Upload_Data_s size: %d", sizeof(Chassis_Upload_Data_s));
    LOGINFO("[can_comm] Chassis_Ctrl_Cmd_s size: %d", sizeof(Chassis_Ctrl_Cmd_s));
    LOGINFO("[can_comm] CAN_COMM_MAX_BUFFSIZE: %d", CAN_COMM_MAX_BUFFSIZE);
    // 【新增调试日志】
    if (chasiss_can_comm != NULL) {
        LOGINFO("[CHASSIS_DEBUG] CAN Comm Init Success! TxID: %d, RxID: %d", chasiss_can_comm->can_ins->tx_id, chasiss_can_comm->can_ins->rx_id);
    } else {
        LOGERROR("[CHASSIS_DEBUG] CAN Comm Init Failed!");
    }
#endif // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    // 标准 X 型麦轮解算公式 (所有轮子 Forward 均为 +vx)
    // 假设右手系：x前, y左, z逆时针

    // LF: 前进(+vx), 左移需后转(+vy), 左转需后转(+wz)
    vt_lf = chassis_vx + chassis_vy + chassis_cmd_recv.wz * LF_CENTER;

    // RF: 前进(+vx), 左移需前转(-vy), 左转需前转(-wz)
    vt_rf = chassis_vx - chassis_vy - chassis_cmd_recv.wz * RF_CENTER;

    // LB: 前进(+vx), 左移需前转(+vy), 左转需后转(-wz) (注意：左后为了向左平移，实际上是要向前转的，但在标准受力分析中，这里的符号取决于你的y定义。通常为 + -)
    // 修正：X型布局向左平移：左前向后(+)，左后向前(+)。
    // 等等，如果归一化了，我们再推导一次：
    // 向左平移(vy>0):
    //   LF(A轮): 需向后转 -> +vy
    //   RF(B轮): 需向前转 -> -vy
    //   LB(B轮): 需向前转 -> +vy (注意：X型左后轮向前转产生向左的分力)
    //   RB(A轮): 需向后转 -> -vy

    vt_lb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx - chassis_vy + chassis_cmd_recv.wz * RB_CENTER;
}

// 针对全麦和全向轮构型，方案一，前轮麦轮，后轮全向轮结算。方案二，利用陀螺仪强力纠正侧向漂移
static void HybridCalculate()
{
    // 前轮 (麦轮)：系数 = 轮距 + 轴距
    vt_lf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;

    // 计算后轮所需的旋转线速度分量
    float rear_rot_spd = chassis_cmd_recv.wz * HALF_TRACK_WIDTH * DEGREE_2_RAD;
    // 只有纵向速度 vy 参与，vy 对全向轮无效
    vt_lb = chassis_vy - rear_rot_spd;
    vt_rb = chassis_vy + rear_rot_spd; // 左右轮旋转项符号相反，形成力偶
}
/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
#ifdef USE_SUPER_CAP
    // [条件编译] 超级电容功率控制逻辑
    if (cap) {
        // 1. 发送能量缓冲 (告诉超电当前裁判系统里还有多少缓冲能量)
        // 保持原样，发送实时buffer是正确的，超电板会根据这个决定是否全力充电
        cap->tx_msg.refereeEnergyBuffer = referee_data->PowerHeatData.buffer_energy;

        // 2. 发送功率限制
        float referee_limit = referee_data->GameRobotState.chassis_power_limit;

        float safe_limit = referee_limit;
        if (safe_limit < 30.0f)
            safe_limit = 30.0f; // 兆底防止过低

        // 无论是否开启爆发模式，给超电的永远是"合法的电池功率上限"
        cap->tx_msg.refereePowerLimit = (uint16_t)safe_limit;

        // 3. DCDC 开关逻辑 (保持你原有的逻辑，稍作优化)
        // 裁判系统允许底盘输出 && 超电在线 && 无关键错误
        if (referee_data->GameRobotState.power_management_chassis_output != 0 &&
            cap->is_online &&
            !SuperCapIsOutputDisabled(cap)) // 使用 super_cap.c 里的辅助函数判断错误
        {
            // 电量充足时开启 DCDC
            if (cap->rx_msg.capEnergyPercent > 30) {
                cap->tx_msg.enableDCDC = 1;
            } else {
                // 低电量保护
                cap->tx_msg.enableDCDC = 0;
            }
        } else {
            cap->tx_msg.enableDCDC = 0;
        }
        static uint32_t error_toggle_tick = 0;
        if (cap->rx_msg.errorCode != 0) {
            uint32_t now = HAL_GetTick();
            if (error_toggle_tick == 0)
                error_toggle_tick = now;
            uint32_t elapsed = (now - error_toggle_tick) % 4000; // 4秒周期
            if (elapsed < 2000) {
                cap->tx_msg.enableDCDC = 0; // 前2秒关
            } else {
                cap->tx_msg.enableDCDC = 1; // 后2秒开
            }
        } else {
            error_toggle_tick = 0; // 错误消除，重置
        }

        /* 发送 CAN 消息 */
        SuperCapSend(cap);
    }
#endif // USE_SUPER_CAP

    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}
/**
 * @brief 根据电机反馈和底盘IMU数据，融合计算底盘实际运动速度
 *
 * 融合策略：
 * 1. 旋转速度(wz)：直接使用IMU陀螺仪数据，比电机逆解算更精确
 * 2. 线速度(vx/vy)：电机编码器逆解算为主，IMU加速度积分为辅，互补滤波融合
 *
 * @note 此函数应以固定频率(如1kHz)被调用，以保证积分精度
 */
// static void EstimateSpeed()
// {
//     // ===== 1. 获取时间间隔 (秒) =====
//     // 假设 ChassisTask 以固定 1ms 周期运行
//     const float dt = 0.001f; // 1ms，若实际周期不同可使用 DWT 获取

//     // ===== 2. 从 IMU 直接获取旋转角速度 (更精确) =====
//     // Chassis_IMU_data->Gyro[2] 是 Z 轴角速度 (rad/s)
//     // 转换为 deg/s 以与其他变量统一
//     float imu_wz_dps = Chassis_IMU_data->Gyro[Z] * RAD_2_DEGREE; // Z 轴角速度 (deg/s)

//     // ===== 3. 电机编码器逆运动学解算 =====
//     // 获取电机转速 (度/秒)，并根据电机安装方向进行符号修正
//     float v_lf = motor_lf->measure.speed_aps; // LF: NORMAL，保持原样
//     float v_rf = -motor_rf->measure.speed_aps; // RF: REVERSE，取反
//     float v_lb = motor_lb->measure.speed_aps; // LB: NORMAL，保持原样
//     float v_rb = -motor_rb->measure.speed_aps; // RB: REVERSE，取反

//     // 麦轮逆运动学公式 (由正解反推):
//     // vx = (v_lf + v_rf + v_lb + v_rb) / 4
//     // vy = (-v_lf + v_rf + v_lb - v_rb) / 4  (X型麦轮)
//     // wz = (-v_lf + v_rf - v_lb + v_rb) / 4 / (R_wheel / geometry_sum)
//     float avg_vx_raw = (v_lf + v_rf + v_lb + v_rb) / 4.0f; // 电机坐标系下的 vx (deg/s)
//     float avg_vy_raw = (-v_lf + v_rf + v_lb - v_rb) / 4.0f; // 电机坐标系下的 vy (deg/s)
//     float avg_wz_raw = (-v_lf + v_rf - v_lb + v_rb) / 4.0f; // 电机逆解算的 wz (deg/s)

//     // 几何参数：用于从电机转速转换到底盘速度
//     float geometry_sum = HALF_TRACK_WIDTH + HALF_WHEEL_BASE; // 半轮距 + 半轴距 (m)
//     float wheel_radius_ratio = RADIUS_WHEEL * DEGREE_2_RAD; // 轮子半径 (m) * (rad/deg)

//     // 电机编码器解算的线速度 (m/s)
//     float encoder_vx = avg_vx_raw * wheel_radius_ratio; // 前后方向速度 (m/s)
//     float encoder_vy = avg_vy_raw * wheel_radius_ratio; // 左右方向速度 (m/s)

//     // 电机编码器解算的旋转速度 (deg/s)
//     float encoder_wz = avg_wz_raw * (RADIUS_WHEEL / geometry_sum);

//     // ===== 4. IMU 加速度积分获取线速度 (辅助) =====
//     // Chassis_IMU_data->Accel[X/Y] 是机体系加速度 (m/s²)
//     // 需要去除重力分量影响，这里假设底盘基本水平
//     static float imu_vx = 0.0f; // IMU 积分得到的 vx
//     static float imu_vy = 0.0f; // IMU 积分得到的 vy

//     // 获取机体系加速度，并进行积分
//     // 注意：需要根据 IMU 安装方向调整坐标轴对应关系
//     float accel_x = Chassis_IMU_data->Accel[X]; // 前后方向加速度 (m/s²)
//     float accel_y = Chassis_IMU_data->Accel[Y]; // 左右方向加速度 (m/s²)

//     // 对加速度进行积分得到速度增量
//     imu_vx += accel_x * dt;
//     imu_vy += accel_y * dt;

//     // ===== 5. 互补滤波融合 =====
//     // 使用互补滤波融合编码器和 IMU 数据
//     // 编码器：低频准确（无漂移），高频噪声大
//     // IMU 积分：高频响应好，但有累积漂移
//     // 策略：以编码器为主，IMU 积分作为高频补偿，并持续向编码器值收敛
//     const float alpha_linear = 0.05f; // 线速度融合系数 (编码器权重较高)
//     const float alpha_wz = 0.8f; // 角速度融合系数 (IMU 权重较高，因为陀螺仪精度高)

//     // 让 IMU 积分值向编码器值收敛，防止累积漂移
//     const float drift_correction = 0.02f; // 漂移修正系数
//     imu_vx = imu_vx * (1.0f - drift_correction) + encoder_vx * drift_correction;
//     imu_vy = imu_vy * (1.0f - drift_correction) + encoder_vy * drift_correction;

//     // 融合线速度：编码器为主 + IMU 高频补偿
//     static float last_vx = 0.0f, last_vy = 0.0f;
//     float fused_vx = encoder_vx * (1.0f - alpha_linear) + imu_vx * alpha_linear;
//     float fused_vy = encoder_vy * (1.0f - alpha_linear) + imu_vy * alpha_linear;

//     // 融合角速度：IMU 陀螺仪为主，编码器为辅 (陀螺仪精度更高)
//     static float last_wz = 0.0f;
//     float fused_wz = imu_wz_dps * alpha_wz + encoder_wz * (1.0f - alpha_wz);

//     // ===== 6. 低通滤波 (平滑输出) =====
//     const float lpf_alpha = 0.3f; // 滤波系数，越小越平滑但滞后
//     real_vx = (1.0f - lpf_alpha) * last_vx + lpf_alpha * fused_vx;
//     real_vy = (1.0f - lpf_alpha) * last_vy + lpf_alpha * fused_vy;
//     real_wz = (1.0f - lpf_alpha) * last_wz + lpf_alpha * fused_wz;

//     // 保存上一次的值
//     last_vx = real_vx;
//     last_vy = real_vy;
//     last_wz = real_wz;
// }

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    float gimbal_wz = 0.0f;
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD
    gimbal_wz = chassis_cmd_recv.gimbal_gyro_z; // What: 使用最新一帧云台角速度前馈；Why: 避免先读旧值再收新命令导致跟随支路固定滞后一个控制周期

    /* 功率控制策略 */
    // 1. 获取裁判系统功率限制 (原始最大值)
    float referee_power_limit;

    // 【修复】先判断指针是否为空
    // 如果没有初始化裁判系统（referee_data == NULL），直接使用测试功率
    if (referee_data == NULL) {
        referee_power_limit = DEFAULT_TEST_POWER; // 强制使用测试功率
    } else {
        // 只有指针有效时才去读取
        referee_power_limit = referee_data->GameRobotState.chassis_power_limit;
        // 即使有指针，如果读出来是0（裁判系统刚启动），也用测试功率兜底
        if (referee_power_limit < 1.0f) {
            referee_power_limit = DEFAULT_TEST_POWER;
        }
    }

    // 2. [新增] 平地/坡道功率缩放策略
    // 平地：使用裁判系统限制的 75%，节省缓冲能量
    // 坡道：使用裁判系统限制的 100%，全力冲坡
    float final_power_limit;
    if (fabsf(Chassis_IMU_data->Pitch) > CHASSIS_SLOPE_THRESHOLD) {
        // 坡道模式：使用 100% 功率
        final_power_limit = referee_power_limit * 1.00f;
    } else {
        // 平地模式：使用 75% 功率
        final_power_limit = referee_power_limit * 1.0f;
    }

    // 3. [条件编译] 超电爆发功率策略
    // 判断是否可以爆发 (电容模式开启 或 坡道检测触发 + 电容在线 + 电量充足 + DCDC已使能)
#ifdef USE_SUPER_CAP
    if (cap && cap->is_online &&
        (chassis_cmd_recv.cap_mode == SUPER_CAP_ON || fabsf(Chassis_IMU_data->Pitch) > CHASSIS_SLOPE_THRESHOLD) &&
        cap->rx_msg.capEnergyPercent > 30 &&
        cap->tx_msg.enableDCDC == 1) {
        // 允许爆发，电机功率上限 = 裁判限制 + 电容贡献(30W-40W)
        final_power_limit += 35.0f;
    }
#endif // USE_SUPER_CAP

    // 4. 最终限幅保护
    if (final_power_limit > 300.0f)
        final_power_limit = 300.0f; // 物理极限
    // 5. 设置给底盘功率控制算法 (这个函数控制电机的电流)
    SetPowerLimit(final_power_limit);

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
        StopAuxActuators();
    } else { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode) {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: {
        const float follow_yaw_kp = 8.0f; // What: 跟随模式位置环比例增益；Why: 直接按角度误差生成回正速度，比二次项更线性且更容易调到“快但不炸”
        const float follow_yaw_kd = 0.18f; // What: 跟随模式底盘角速度阻尼增益；Why: 使用底盘真实角速度做D项，专门抑制回中穿越和反向摆动
        const float follow_yaw_kff = 0.85f; // What: 跟随模式云台角速度前馈增益；Why: 云台先动时提前拉动底盘，减少纯靠角度误差追赶带来的滞后
        const float follow_yaw_deadband = 0.8f; // What: 跟随模式角度死区；Why: 回中附近直接清零小误差，避免机械间隙和噪声触发来回抖动
        const float follow_yaw_max_wz = 600.0f; // What: 跟随模式角速度输出上限；Why: 防止大角度时给电机速度环过猛目标，降低饱和后再过冲的概率
        float chassis_wz = Chassis_IMU_data->Gyro[Z] * RAD_2_DEGREE; // What: 读取底盘当前真实角速度；Why: D项必须基于被控对象自身速度才能形成真实阻尼
        float angle_err = chassis_cmd_recv.offset_angle; // What: 缓存当前底盘相对云台的角度误差；Why: 便于在进入控制律前统一做死区处理

        if (fabsf(angle_err) < follow_yaw_deadband) {
            angle_err = 0.0f; // What: 清零死区内误差；Why: 小角度时让前馈和阻尼接管，避免位置项在零点附近反复翻转
        }

        chassis_cmd_recv.wz = -follow_yaw_kp * angle_err - follow_yaw_kd * chassis_wz - follow_yaw_kff * gimbal_wz; // What: 生成底盘跟随云台的角速度指令；Why: 用P保证回中速度、用D抑制过冲、用前馈减少跟随滞后
        LIMIT_MIN_MAX(chassis_cmd_recv.wz, -follow_yaw_max_wz, follow_yaw_max_wz); // What: 限制跟随模式角速度输出；Why: 避免外环瞬时给出过大目标把电机内环推入饱和
        break;
    }
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动
                         // [修改] 优化小陀螺逻辑：区分变速/匀速，并统一应用平移优先策略
#if VARIABLE_SPIN_ENABLED
                         // 1. 变速小陀螺
    {
        float base_wz = GetVariableSpinBase(); // 获取随时间变化的基础速度
        chassis_cmd_recv.wz = OptimizedSpinSpeed(base_wz, chassis_cmd_recv.vx, chassis_cmd_recv.vy);
    }
#else
                         // 2. 优化后的匀速小陀螺
    {
        float const_target = SPIN_TOP_MAX_SPEED; // 固定最大速度
        chassis_cmd_recv.wz = OptimizedSpinSpeed(const_target, chassis_cmd_recv.vx, chassis_cmd_recv.vy);
    }
#endif
    break;
    default:
        break;
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    // 修正旋转矩阵：底盘为右手系（X向前方，Y向右方），且 offset_angle 以逆时针方向为正。
    // 因此在分解目标云台向上的平移速度(vx, vy)时，sin 项的符号需要与标准矩阵相反，避免出现推杆前进而横移的现象。
    chassis_vx = chassis_cmd_recv.vx * cos_theta + chassis_cmd_recv.vy * sin_theta;
    chassis_vy = -chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    if (chassis_cmd_recv.chassis_mode == CHASSIS_NO_FOLLOW) {
        // ChassisHeadLock();
    }

    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();
    // HybridCalculate();
    PowerControl_UpdateIMU(Chassis_IMU_data->Pitch * DEGREE_2_RAD,
                           Chassis_IMU_data->Roll * DEGREE_2_RAD);

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    if (chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE) {
        // What: 在底盘主运动解算后独立控制履带与抬升；Why: 这两套执行机构不参与麦轮功率分配，独立更新能减少耦合风险
        ControlFrontTrackMotors();
        ControlLiftMotors();
    }

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    // EstimateSpeed();

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;
    UpdateRefereeKeyMouseUpload(); // What: 刷新裁判键鼠上传内容；Why: 让云台cmd层直接复用现有底盘反馈链路拿到键鼠输入

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}
