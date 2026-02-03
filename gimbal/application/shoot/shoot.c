#include "shoot.h"
#include "master_process.h"
#include "motor_def.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
// 摩擦轮半径 (单位: 米), 例如 30mm = 0.03m
#define FRICTION_WHEEL_RADIUS 0.03f
// 打滑补偿系数 (需要实测微调, 通常在 1.0 - 1.2 之间)
#define SLIP_COMPENSATION 1.05f

// [新增] 摩擦轮软启动步长 (deg/loop)
// 假设 200Hz 控制频率，15m/s (约28000dps)
// 设为 150.0f 表示约 1秒 达到满速
#define FRICTION_RAMP_STEP 150.0f

// ==================== 堵转检测参数 ====================
// 堵转检测电流阈值 (raw值, M3508满量程16384, 设为 ~60% 为稳妥值)
#define STALL_CURRENT_THRESHOLD 10000
// 堵转检测速度阈值 (deg/s), 低于此值且电流高则判定为堵转
#define STALL_SPEED_THRESHOLD 100.0f
// 堵转检测消抖时间 (ms), 持续满足条件才确认堵转
#define STALL_DETECT_TIME 100
// 反转角度 (deg), 约为1/2颗弹丸角度,足够解卡但用户无感
#define REVERSE_ANGLE (ONE_BULLET_DELTA_ANGLE / 2.5f)
// 反转持续时间 (ms)
#define REVERSE_TIME 120
// 恢复等待时间 (ms), 反转后等待稳定再继续供弹
#define RECOVERY_TIME 80
// 连续反转次数上限, 超过则认为卡死,停止尝试
#define MAX_REVERSE_COUNT 3

// ==================== 发射确认检测参数 ====================
// 掉速检测阈值 (deg/s), 内圈摩擦轮速度下降超过此值认为有弹丸通过
// 使用内圈检测是因为弹丸先接触内圈, 信号更早, 能更有效防止多发
#define FRICTION_SPEED_DIP_THRESHOLD 400.0f
// 回升检测阈值 (deg/s), 与目标速度差小于此值认为回升完成
#define FRICTION_SPEED_RECOVER_THRESHOLD 200.0f
// 拨盘位置误差阈值 (deg), 小于此值认为拨盘到位
#define LOADER_POSITION_THRESHOLD 8.0f
// 发射确认超时时间 (ms), 等待摩擦轮掉速的最大时间
#define FIRE_DETECT_TIMEOUT 300
// 拨盘到位超时时间 (ms), 等待拨盘转到位的最大时间
#define LOADER_ARRIVE_TIMEOUT 200

// 堵转检测状态枚举
typedef enum {
    STALL_NORMAL = 0, // 正常状态
    STALL_DETECTING, // 疑似堵转,消抖中
    STALL_REVERSING, // 确认堵转,正在反转
    STALL_RECOVERY // 反转完成,恢复中
} LoaderStallState_e;

// 堵转检测状态结构体
static struct {
    LoaderStallState_e state; // 当前状态
    float detect_start_time; // 开始检测堵转的时间戳 (ms)
    float reverse_start_time; // 开始反转的时间戳 (ms)
    float recovery_start_time; // 开始恢复的时间戳 (ms)
    float reverse_target_angle; // 反转目标角度
    uint8_t reverse_count; // 连续反转次数
    loader_mode_e saved_mode; // 保存的原发射模式
} stall_handler = { 0 };

// 发射确认状态枚举
// 用于追踪单发弹丸从拨盘推出到摩擦轮检测的完整流程
typedef enum {
    FIRE_IDLE = 0, // 空闲/等待发射指令
    FIRE_LOADING, // 拨盘正在旋转, 等待到位
    FIRE_WAIT_DIP, // 拨盘已到位, 等待摩擦轮掉速
    FIRE_WAIT_RECOVER, // 检测到掉速, 等待回升 (拨盘已锁定)
    FIRE_CONFIRMED, // 发射确认成功
    FIRE_EMPTY, // 缺弹 (拨盘到位但无掉速)
} FireDetectState_e;

// 发射确认状态结构体
// 用于管理发射检测状态机的所有运行时数据
static struct {
    FireDetectState_e state; // 当前状态
    float loader_target_angle; // 拨盘目标角度
    float baseline_speed; // 发射前的基准摩擦轮速度 (deg/s)
    float fire_start_time; // 开始发射的时间戳 (ms)
    float loader_arrive_time; // 拨盘到位的时间戳 (ms)
    uint8_t bullet_fired_flag; // 发射确认标志位 (1=已确认发射)
    uint8_t empty_flag; // 缺弹标志位 (1=检测到缺弹)
    uint8_t loader_locked; // 拨盘锁定标志 (防多发核心机制)
    uint16_t fire_count; // 已确认发射计数
} fire_detector = { 0 };

static float current_inner_deg = 0.0f;
static float current_outer_deg = 0.0f;

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *loader; // 拨盘电机
static DJIMotorInstance *friction_inner_down, *friction_inner_left, *friction_inner_right; // 内部摩擦轮电机
static DJIMotorInstance *friction_outer_down, *friction_outer_left, *friction_outer_right; // 外部摩擦轮电机
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

ShootDebugSpeed_s shoot_debug_speed;

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

void ShootInit()
{
    // 内摩擦轮
    Motor_Init_Config_s friction_config_inner = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, // 20
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 16000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508
    };
    // 外摩擦轮
    Motor_Init_Config_s friction_config_outer = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 8,
                .Ki = 1,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 16000,
            },

        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508
    };
    // 第一级，内摩擦轮初始化
    friction_config_inner.can_init_config.tx_id = 1; // 上摩擦轮,改txid和方向就行
    friction_config_inner.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_inner_down = DJIMotorInit(&friction_config_inner);

    friction_config_inner.can_init_config.tx_id = 2; // 左摩擦轮,改txid和方向就行
    friction_config_inner.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_inner_left = DJIMotorInit(&friction_config_inner);

    friction_config_inner.can_init_config.tx_id = 3; // 右摩擦轮,改txid和方向就行
    friction_config_inner.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_inner_right = DJIMotorInit(&friction_config_inner);

    // // 第二级，外摩擦轮初始化
    friction_config_outer.can_init_config.tx_id = 4; // 上摩擦轮,改txid和方向就行
    friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_outer_down = DJIMotorInit(&friction_config_outer);

    friction_config_outer.can_init_config.tx_id = 5; // 左摩擦轮,改txid和方向就行
    friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_outer_left = DJIMotorInit(&friction_config_outer);

    friction_config_outer.can_init_config.tx_id = 6; // 右摩擦轮,改txid和方向就行
    friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_outer_right = DJIMotorInit(&friction_config_outer);
    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 2.2, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 400,
            },
            .speed_PID = {
                .Kp = 6.5, // 10
                .Ki = 0.9, // 1
                .Kd = 0.0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 14000,
            },

        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M3508 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}
/**
 * @brief 辅助函数：将线速度转换为角速度
 */
float SpeedMps2Degs(float speed_mps)
{
    if (speed_mps <= 0.0f)
        return 0.0f;
    if (speed_mps > 20.0f)
        speed_mps = 20.0f; // 安全限幅

    // 公式: (线速度 / 半径) * (180/PI) * 补偿系数
    return (speed_mps / FRICTION_WHEEL_RADIUS) * RAD_2_DEGREE * SLIP_COMPENSATION;
}

/**
 * @brief 分别设置两级摩擦轮的速度
 * @param inner_mps 第一级（内圈）目标射速 m/s
 * @param outer_mps 第二级（外圈）目标射速 m/s
 */
void ShootSetSpeedDual(float inner_mps, float outer_mps)
{
    // 1. 计算各自的目标角速度 (deg/s)
    float target_inner_deg = SpeedMps2Degs(inner_mps);
    float target_outer_deg = SpeedMps2Degs(outer_mps);

    // [新增] 软启动斜坡算法 (Ramp Logic)
    // Inner
    float diff_inner = target_inner_deg - current_inner_deg;
    if (diff_inner > FRICTION_RAMP_STEP)
        current_inner_deg += FRICTION_RAMP_STEP;
    else if (diff_inner < -FRICTION_RAMP_STEP)
        current_inner_deg -= FRICTION_RAMP_STEP;
    else
        current_inner_deg = target_inner_deg;

    // Outer
    float diff_outer = target_outer_deg - current_outer_deg;
    if (diff_outer > FRICTION_RAMP_STEP)
        current_outer_deg += FRICTION_RAMP_STEP;
    else if (diff_outer < -FRICTION_RAMP_STEP)
        current_outer_deg -= FRICTION_RAMP_STEP;
    else
        current_outer_deg = target_outer_deg;

    // 2. 设置第一级（内圈3个电机）- 负责主要加速
    // 注意：你的 shoot.c 初始化了 down, left, right 三个电机
    DJIMotorSetRef(friction_inner_left, current_inner_deg);
    DJIMotorSetRef(friction_inner_right, current_inner_deg);
    DJIMotorSetRef(friction_inner_down, current_inner_deg);

    // 3. 设置第二级（外圈3个电机）- 负责稳速/微加速
    DJIMotorSetRef(friction_outer_left, current_outer_deg);
    DJIMotorSetRef(friction_outer_right, current_outer_deg);
    DJIMotorSetRef(friction_outer_down, current_outer_deg);
}
/**
 * @brief 将电机反馈的角速度(deg/s)转换为线速度(m/s)用于调试
 * @note 不包含打滑补偿，反映的是摩擦轮表面的物理线速度
 */
static void UpdateDebugSpeed(void)
{
// 定义局部宏简化代码
#define CALC_MPS(motor) ((motor->measure.speed_aps / RAD_2_DEGREE) * FRICTION_WHEEL_RADIUS)

    // 注意：这里保留了正负号，方便观察电机是否反转。
    // 如果你只想看数值大小，可以使用 fabsf() 函数取绝对值。
    if (friction_inner_left)
        shoot_debug_speed.inner_left = CALC_MPS(friction_inner_left);
    if (friction_inner_right)
        shoot_debug_speed.inner_right = CALC_MPS(friction_inner_right);
    if (friction_inner_down)
        shoot_debug_speed.inner_down = CALC_MPS(friction_inner_down);

    if (friction_outer_left)
        shoot_debug_speed.outer_left = CALC_MPS(friction_outer_left);
    if (friction_outer_right)
        shoot_debug_speed.outer_right = CALC_MPS(friction_outer_right);
    if (friction_outer_down)
        shoot_debug_speed.outer_down = CALC_MPS(friction_outer_down);
}

/**
 * @brief 检测拨盘电机是否堵转
 * @return 1 表示当前处于堵转状态（高电流+低转速），0 表示正常
 * @note 使用电机反馈的实际电流和速度进行判断
 */
static uint8_t IsLoaderStalled(void)
{
    // 获取电机反馈数据: |real_current| > 阈值 且 |speed_aps| < 阈值
    int16_t current_abs = (loader->measure.real_current > 0) ?
                              loader->measure.real_current :
                              -loader->measure.real_current;
    float speed_abs = (loader->measure.speed_aps > 0.0f) ?
                          loader->measure.speed_aps :
                          -loader->measure.speed_aps;

    return (current_abs > STALL_CURRENT_THRESHOLD) &&
           (speed_abs < STALL_SPEED_THRESHOLD);
}

/**
 * @brief 获取内圈摩擦轮平均速度 (deg/s)
 * @return 三个内圈摩擦轮速度的平均绝对值
 * @note 使用内圈检测是因为弹丸先接触内圈, 信号更早, 防多发效果更好
 */
static float GetInnerFrictionAvgSpeed(void)
{
    // 计算三个内圈摩擦轮的平均绝对速度
    // 取绝对值是因为电机方向可能不同, 我们只关心速度大小
    float avg = (fabsf(friction_inner_left->measure.speed_aps) +
                 fabsf(friction_inner_right->measure.speed_aps) +
                 fabsf(friction_inner_down->measure.speed_aps)) /
                3.0f;
    return avg;
}

/**
 * @brief 检测内圈摩擦轮是否发生掉速
 * @return 1 表示检测到掉速 (有弹丸通过), 0 表示正常
 * @note 通过与发射前记录的基准速度对比来判断是否掉速
 */
static uint8_t IsFrictionDipping(void)
{
    float current_speed = GetInnerFrictionAvgSpeed();
    // 与基准速度对比, 下降超过阈值则认为掉速 (有弹丸正在通过)
    return (fire_detector.baseline_speed - current_speed) > FRICTION_SPEED_DIP_THRESHOLD;
}

/**
 * @brief 检测内圈摩擦轮速度是否回升
 * @return 1 表示速度已回升 (弹丸已离开), 0 表示仍在恢复
 * @note 速度回升表示弹丸已经完全通过摩擦轮
 */
static uint8_t IsFrictionRecovered(void)
{
    float current_speed = GetInnerFrictionAvgSpeed();
    // 与目标速度对比, 差值小于阈值则认为回升完成
    return (current_inner_deg - current_speed) < FRICTION_SPEED_RECOVER_THRESHOLD;
}

/**
 * @brief 检测拨盘是否到达目标位置
 * @return 1 表示拨盘已到位, 0 表示仍在旋转
 * @note 用于判断供弹动作是否完成
 */
static uint8_t IsLoaderInPosition(void)
{
    float error = fabsf(loader->measure.total_angle - fire_detector.loader_target_angle);
    return error < LOADER_POSITION_THRESHOLD;
}

/**
 * @brief 检查拨盘是否被锁定 (防多发机制)
 * @return 1 表示锁定中, 应拒绝新的发射指令; 0 表示未锁定
 * @note 该函数供外部模块调用, 用于在检测到发射时锁定拨盘
 */
uint8_t ShootIsLoaderLocked(void)
{
    return fire_detector.loader_locked;
}

/**
 * @brief 堵转检测与自动反转处理状态机
 * @param current_mode 当前的发射模式
 * @return 经过堵转处理后的发射模式 (可能被临时替换为反转模式)
 * @note 该函数实现无感反转逻辑,在检测到堵转时自动反转一小段角度
 */
static loader_mode_e HandleLoaderStall(loader_mode_e current_mode)
{
    float current_time = DWT_GetTimeline_ms();

    // 仅在发射模式下进行堵转检测 (单发/二连发/三连发/连发)
    // LOAD_STOP 和 LOAD_REVERSE 模式不进行检测
    uint8_t is_shooting = (current_mode == LOAD_1_BULLET) ||
                          (current_mode == LOAD_2_BULLET) ||
                          (current_mode == LOAD_3_BULLET) ||
                          (current_mode == LOAD_BURSTFIRE);

    switch (stall_handler.state) {
    case STALL_NORMAL:
        // 正常状态: 检测是否进入堵转
        if (is_shooting && IsLoaderStalled()) {
            // 检测到疑似堵转,进入消抖阶段
            stall_handler.state = STALL_DETECTING;
            stall_handler.detect_start_time = current_time;
            stall_handler.saved_mode = current_mode; // 保存当前模式
        }
        // 如果当前是主动反转或停止,重置反转计数
        if (current_mode == LOAD_REVERSE || current_mode == LOAD_STOP) {
            stall_handler.reverse_count = 0;
        }
        return current_mode; // 正常模式不修改

    case STALL_DETECTING:
        // 消抖阶段: 持续检测堵转状态
        if (!IsLoaderStalled()) {
            // 堵转消失,恢复正常
            stall_handler.state = STALL_NORMAL;
            return current_mode;
        }
        // 检查是否超过消抖时间
        if ((current_time - stall_handler.detect_start_time) >= STALL_DETECT_TIME) {
            // 确认堵转,进入反转阶段
            stall_handler.state = STALL_REVERSING;
            stall_handler.reverse_start_time = current_time;
            stall_handler.reverse_target_angle = loader->measure.total_angle - REVERSE_ANGLE;
            stall_handler.reverse_count++;

            // 设定反转目标角度 (在 ShootTask 的 switch 之前生效)
            DJIMotorOuterLoop(loader, ANGLE_LOOP);
            DJIMotorSetRef(loader, stall_handler.reverse_target_angle);
        }
        return current_mode; // 消抖中暂不修改

    case STALL_REVERSING:
        // 反转阶段: 等待反转完成
        if ((current_time - stall_handler.reverse_start_time) >= REVERSE_TIME) {
            // 反转时间结束,进入恢复期
            stall_handler.state = STALL_RECOVERY;
            stall_handler.recovery_start_time = current_time;
        }
        // 返回 LOAD_STOP 跳过正常的发射逻辑,由状态机控制电机
        return LOAD_STOP;

    case STALL_RECOVERY:
        // 恢复阶段: 等待稳定后恢复发射
        if ((current_time - stall_handler.recovery_start_time) >= RECOVERY_TIME) {
            // 恢复期结束
            if (stall_handler.reverse_count >= MAX_REVERSE_COUNT) {
                // 连续反转达到上限,停止发射,需要人工干预
                stall_handler.state = STALL_NORMAL;
                stall_handler.reverse_count = 0;
                return LOAD_STOP; // 返回停止,等待新指令
            }
            // 恢复正常状态,继续之前的发射模式
            stall_handler.state = STALL_NORMAL;
            return stall_handler.saved_mode;
        }
        return LOAD_STOP; // 恢复期也返回 STOP

    default:
        stall_handler.state = STALL_NORMAL;
        return current_mode;
    }
}

/**
 * @brief 发射确认检测状态机
 * @param load_mode 当前的拨盘模式
 * @note 该函数应在 ShootTask 的拨盘控制之后调用
 *       检测流程: 拨盘到位 -> 内圈掉速 -> 锁定拨盘 -> 速度回升 -> 确认发射
 *       使用内圈检测是因为弹丸先接触内圈, 信号更早, 防多发效果更好
 */
static void HandleFireDetection(loader_mode_e load_mode)
{
    float current_time = DWT_GetTimeline_ms();

    // 仅在单发/二连发/三连发模式下进行检测
    // 连发模式不进行这种检测, 因为连发时掉速信号会重叠
    uint8_t is_single_fire = (load_mode == LOAD_1_BULLET) ||
                             (load_mode == LOAD_2_BULLET) ||
                             (load_mode == LOAD_3_BULLET);

    switch (fire_detector.state) {
    case FIRE_IDLE:
        // 空闲状态: 检测到发射指令时启动检测流程
        if (is_single_fire && !fire_detector.loader_locked) {
            fire_detector.state = FIRE_LOADING;
            // 记录拨盘目标角度, 用于判断拨盘是否到位
            fire_detector.loader_target_angle = loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE;
            // 记录发射前的摩擦轮基准速度, 用于后续掉速比较
            fire_detector.baseline_speed = GetInnerFrictionAvgSpeed();
            fire_detector.fire_start_time = current_time;
            fire_detector.bullet_fired_flag = 0;
        }
        break;

    case FIRE_LOADING:
        // 等待拨盘到位: 拨盘电机正在旋转, 将弹丸推向摩擦轮
        if (IsLoaderInPosition()) {
            // 拨盘已到位, 弹丸应该即将接触摩擦轮
            fire_detector.state = FIRE_WAIT_DIP;
            fire_detector.loader_arrive_time = current_time;
        } else if ((current_time - fire_detector.fire_start_time) > LOADER_ARRIVE_TIMEOUT) {
            // 拨盘到位超时 -> 可能卡弹, 交给现有的堵转处理逻辑 (HandleLoaderStall)
            fire_detector.state = FIRE_IDLE;
        }
        break;

    case FIRE_WAIT_DIP:
        // 等待内圈摩擦轮掉速: 弹丸接触摩擦轮会导致转速下降
        if (IsFrictionDipping()) {
            // 检测到掉速, 说明弹丸正在通过摩擦轮
            // 立即锁定拨盘, 防止在弹丸完全离开前供入下一发
            fire_detector.state = FIRE_WAIT_RECOVER;
            fire_detector.loader_locked = 1; // 🔒 锁定拨盘, 防止多发
        } else if ((current_time - fire_detector.loader_arrive_time) > FIRE_DETECT_TIMEOUT) {
            // 超时无掉速 -> 缺弹 (拨盘转动了但没有弹丸被推出)
            fire_detector.state = FIRE_EMPTY;
            fire_detector.empty_flag = 1;
        }
        break;

    case FIRE_WAIT_RECOVER:
        // 等待内圈摩擦轮速度回升: 弹丸离开后摩擦轮速度会恢复
        if (IsFrictionRecovered()) {
            // 速度回升, 弹丸已完全离开摩擦轮, 发射确认成功
            fire_detector.state = FIRE_CONFIRMED;
            fire_detector.bullet_fired_flag = 1;
            fire_detector.fire_count++;
            fire_detector.loader_locked = 0; // 🔓 解锁拨盘, 允许下一发
        } else if ((current_time - fire_detector.loader_arrive_time) > FIRE_DETECT_TIMEOUT) {
            // 超时未回升 -> 仍视为发射成功 (弹丸可能卡在枪管中)
            fire_detector.state = FIRE_CONFIRMED;
            fire_detector.bullet_fired_flag = 1;
            fire_detector.fire_count++;
            fire_detector.loader_locked = 0; // 🔓 解锁拨盘
        }
        break;

    case FIRE_CONFIRMED:
    case FIRE_EMPTY:
        // 终态: 等待状态复位
        // 当不再是单发模式时 (如切换到 LOAD_STOP), 复位状态机准备下一次检测
        if (!is_single_fire) {
            fire_detector.state = FIRE_IDLE;
            fire_detector.bullet_fired_flag = 0;
            fire_detector.loader_locked = 0;
            // empty_flag 保持, 需要通过 ShootClearEmptyFlag() 手动复位
        }
        break;

    default:
        // 异常状态恢复
        fire_detector.state = FIRE_IDLE;
        fire_detector.loader_locked = 0;
        break;
    }
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
        current_inner_deg = 0.0f;
        current_outer_deg = 0.0f;
        DJIMotorStop(friction_inner_left);
        DJIMotorStop(friction_inner_right);
        DJIMotorStop(friction_outer_left);
        DJIMotorStop(friction_outer_right);
        DJIMotorStop(friction_inner_down);
        DJIMotorStop(friction_outer_down);
        DJIMotorStop(loader);
    } else // 恢复运行
    {
        DJIMotorEnable(friction_inner_left);
        DJIMotorEnable(friction_inner_right);
        DJIMotorEnable(friction_outer_left);
        DJIMotorEnable(friction_outer_right);
        DJIMotorEnable(friction_inner_down);
        DJIMotorEnable(friction_outer_down);
        DJIMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    if (hibernate_time + dead_time > DWT_GetTimeline_ms())
        return;

    // [新增] 堵转检测与自动反转处理
    // 该函数会在堵转时自动替换发射模式为反转/停止状态
    loader_mode_e actual_load_mode = HandleLoaderStall(shoot_cmd_recv.load_mode);

    // 若不在休眠状态,根据实际发射模式进行拨盘电机参考值设定和模式切换
    switch (actual_load_mode) {
    // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0); // 同时设定参考值为0,这样停止的速度最快
        break;
    // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET: // 激活能量机关/干扰对方用,英雄用.
        DJIMotorOuterLoop(loader, ANGLE_LOOP); // 切换到角度环
        DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
        hibernate_time = DWT_GetTimeline_ms(); // 记录触发指令的时间
        dead_time = 150; // 完成1发弹丸发射的时间
        break;
    // 三连发,如果不需要后续可能删除
    case LOAD_3_BULLET:
        DJIMotorOuterLoop(loader, ANGLE_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
        hibernate_time = DWT_GetTimeline_ms(); // 记录触发指令的时间
        dead_time = 300; // 完成3发弹丸发射的时间
        break;
    // 二连发模式 (英雄专用)
    case LOAD_2_BULLET:
        DJIMotorOuterLoop(loader, ANGLE_LOOP); // 切换到角度环
        DJIMotorSetRef(loader, loader->measure.total_angle + 2 * ONE_BULLET_DELTA_ANGLE); // 增加2发
        hibernate_time = DWT_GetTimeline_ms(); // 记录触发指令的时间
        dead_time = 200; // 完成2发弹丸发射的时间
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
        // DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorOuterLoop(loader, ANGLE_LOOP); // 切换到角度环
        DJIMotorSetRef(loader, loader->measure.total_angle - ONE_BULLET_DELTA_ANGLE / 2.0f); // 控制量减少一发弹丸的一半角度
        hibernate_time = DWT_GetTimeline_ms(); // 记录触发指令的时间
        dead_time = 150; // 完成1发弹丸发射的时间
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON) {
        // 示例：根据不同的模式设置不同的分级速度
        switch (shoot_cmd_recv.bullet_speed) {
        case BIG_AMU_12:
            // 目标12m/s：一级给11.5，二级给12.0
            ShootSetSpeedDual(11.5f, 12.0f);
            break;
        case BIG_AMU_16:
            // 目标16.5m/s：一级给16.0，二级给16.5
            ShootSetSpeedDual(16.0f, 16.5f);
            break;
        default:
            // 调试默认值 (可以根据你的串口调试实时修改这两个值)
            ShootSetSpeedDual(15.5f, 15.8f);
            break;
        }
    } else {
        // 关闭摩擦轮
        ShootSetSpeedDual(0.0f, 0.0f);
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE) {
        //...
    } else if (shoot_cmd_recv.lid_mode == LID_OPEN) {
        //...
    }
    // 更新调试信息
    UpdateDebugSpeed();

    // [新增] 发射确认检测状态机
    // 检测流程: 拨盘到位 -> 内圈掉速 -> 锁定拨盘 -> 速度回升 -> 确认发射
    HandleFireDetection(actual_load_mode);

    // [新增] 更新反馈数据, 供外部模块订阅
    shoot_feedback_data.bullet_fired_flag = fire_detector.bullet_fired_flag;
    shoot_feedback_data.empty_flag = fire_detector.empty_flag;
    shoot_feedback_data.fire_count = fire_detector.fire_count;

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}

/**
 * @brief 获取发射确认标志
 * @return 1 表示最近一次发射已确认, 0 表示未确认
 * @note 该标志在每次发射确认后置位, 在下一次发射开始时清除
 */
uint8_t ShootGetFiredFlag(void)
{
    return fire_detector.bullet_fired_flag;
}

/**
 * @brief 获取缺弹标志
 * @return 1 表示检测到缺弹, 0 表示正常
 * @note 该标志需要通过 ShootClearEmptyFlag() 手动清除
 */
uint8_t ShootGetEmptyFlag(void)
{
    return fire_detector.empty_flag;
}

/**
 * @brief 清除缺弹标志
 * @note 在补充弹药后调用此函数以复位缺弹状态
 */
void ShootClearEmptyFlag(void)
{
    fire_detector.empty_flag = 0;
}

/**
 * @brief 获取已确认发射计数
 * @return 累计确认发射的弹丸数量
 * @note 该计数从开机后累加, 不会自动清零
 */
uint16_t ShootGetFireCount(void)
{
    return fire_detector.fire_count;
}