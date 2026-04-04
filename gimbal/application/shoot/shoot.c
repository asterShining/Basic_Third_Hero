#include "shoot.h"
#include "master_process.h"
#include "motor_def.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include <math.h>

static float current_inner_deg = 0.0f;
static float current_outer_deg = 0.0f;
// What: 缓存摩擦轮最终目标转速绝对值；Why: 单发 ready/recover 判定必须对齐最终稳态目标，而不能对齐 ramp 过程中的中间目标。
static float target_inner_left_deg_abs = 0.0f;
static float target_inner_right_deg_abs = 0.0f;
static float target_inner_down_deg_abs = 0.0f;
static float target_outer_left_deg_abs = 0.0f;
static float target_outer_right_deg_abs = 0.0f;
static float target_outer_down_deg_abs = 0.0f;
// What: 缓存摩擦轮 ramp 的 DWT 计数器；Why: 升速改成时间型斜坡后，每轮都要用真实 dt 计算步进量。
static uint32_t friction_ramp_dwt_cnt = 0;

// [新增] 6个摩擦轮单独的前馈变量
static float ff_inner_left = 0.0f;
static float ff_inner_right = 0.0f;
static float ff_inner_down = 0.0f;
static float ff_outer_left = 0.0f;
static float ff_outer_right = 0.0f;
static float ff_outer_down = 0.0f;

static DJIMotorInstance *loader; // 拨盘电机
static DJIMotorInstance *friction_inner_down, *friction_inner_left, *friction_inner_right; // 内部摩擦轮电机
static DJIMotorInstance *friction_outer_down, *friction_outer_left, *friction_outer_right; // 外部摩擦轮电机
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// [调试] 结构体指针变量，便于调试器直接观测 (通过函数接口获取)
static FrictionWheelDebug_s *p_friction_debug;
static StallDebug_s *p_stall_debug;
static SingleFireDebug_s *p_sf_debug;
static BulletDipSnapshot_s *p_dip_snapshot;

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

// What: 记录单发控制使用的内圈掉速基线；Why: 生产控制要以内圈最先咬弹的掉速为准，不能继续只靠外圈晚确认来刹车。
static struct {
    float inner_left_baseline;
    float inner_right_baseline;
    float inner_down_baseline;
    float outer_left_baseline;
    float outer_right_baseline;
    float outer_down_baseline;
} dip_control = { 0 };

// What: 为后续辅助函数提供前置声明；Why: 单发状态机重构后有多处工具函数前后复用，显式声明比依赖定义顺序更稳妥。
static float GetInnerFrictionAvgSpeed(void);
static float GetOuterFrictionAvgSpeed(void);
static void RecordDipBaseline(void);

void ShootInit()
{
    // 内摩擦轮配置模板
    Motor_Init_Config_s friction_config_inner = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 7.7, // 8.7
                .Ki = 0.0, // 0.5
                .Kd = 0,
                .DeadBand = 10.0f, // [新增] 死区: ±20 deg/s 以内视为零速, 配合 Iout 清零防止停止时自转
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 16000,
            },
            // [修改] 下面会针对每个电机单独赋值
            .current_feedforward_ptr = NULL,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .feedforward_flag = CURRENT_FEEDFORWARD, // [新增] 开启电流前馈
        },
        .motor_type = M3508
    };
    // 外摩擦轮配置模板
    Motor_Init_Config_s friction_config_outer = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 7.3, // 8.3
                .Ki = 0.0, // 0.0
                .Kd = 0,
                .DeadBand = 10.0f, // [新增] 死区: ±20 deg/s 以内视为零速, 配合 Iout 清零防止停止时自转
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 15000,
            },
            .current_feedforward_ptr = NULL,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .feedforward_flag = CURRENT_FEEDFORWARD, // [新增] 开启电流前馈
        },

        .motor_type = M3508
    };

    // 第一级，内摩擦轮初始化
    // 1. 下
    friction_config_inner.can_init_config.tx_id = 1;
    friction_config_inner.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_config_inner.controller_param_init_config.current_feedforward_ptr = &ff_inner_down; // 绑定独立前馈
    friction_inner_down = DJIMotorInit(&friction_config_inner);

    // 2. 左 (反转)
    friction_config_inner.can_init_config.tx_id = 2;
    friction_config_inner.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_config_inner.controller_param_init_config.current_feedforward_ptr = &ff_inner_left; // 绑定独立前馈
    friction_inner_left = DJIMotorInit(&friction_config_inner);

    // 3. 右
    friction_config_inner.can_init_config.tx_id = 3;
    friction_config_inner.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_config_inner.controller_param_init_config.current_feedforward_ptr = &ff_inner_right; // 绑定独立前馈
    friction_inner_right = DJIMotorInit(&friction_config_inner);

    // 第二级，外摩擦轮初始化
    // 1. 下
    friction_config_outer.can_init_config.tx_id = 4;
    friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_config_outer.controller_param_init_config.current_feedforward_ptr = &ff_outer_down; // 绑定独立前馈
    friction_outer_down = DJIMotorInit(&friction_config_outer);

    // 2. 左 (反转)
    friction_config_outer.can_init_config.tx_id = 5;
    friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_config_outer.controller_param_init_config.current_feedforward_ptr = &ff_outer_left; // 绑定独立前馈
    friction_outer_left = DJIMotorInit(&friction_config_outer);

    // 3. 右
    friction_config_outer.can_init_config.tx_id = 6;
    friction_config_outer.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_config_outer.controller_param_init_config.current_feedforward_ptr = &ff_outer_right; // 绑定独立前馈
    friction_outer_right = DJIMotorInit(&friction_config_outer);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 这里把位置环 Kp 和输出上限一起抬高，作用是把 2.5 发量级的大角度误差直接转换成高速度给定；
                // 原因是 DJI 串级控制里位置环输出先喂给速度环，若这里不够大，拨盘就无法在起步瞬间打出满力矩冲刺。
                .Kp = 14.0f,
                .Ki = 0.0,
                .Kd = 0.0f,
                .MaxOut = 30000,

            },
            .speed_PID = {
                .Kp = 3.1, // 3.1
                .Ki = 0.0, // 1
                .Kd = 0.0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 16100,
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

    // [调试] 初始化调试指针 (获取 shoot_debug.c 中的实际结构体地址)
    // 原因: 这些指针必须在初始化时赋值，否则 Ozone 调试器读到的是 NULL
    p_friction_debug = ShootDebug_GetFrictionPtr();
    p_stall_debug = ShootDebug_GetStallPtr();
    p_sf_debug = ShootDebug_GetSingleFirePtr();
    p_dip_snapshot = ShootDebug_GetDipSnapshotPtr();
    // What: 初始化摩擦轮 ramp 的时间基准；Why: 第一次进入时间型斜坡时不能拿到异常大的 dt，否则会把目标一步跳满。
    DWT_GetDeltaT(&friction_ramp_dwt_cnt);
}
/**
 * @brief 辅助函数：将线速度转换为角速度
 */
/**
 * @brief 辅助函数：将线速度转换为角速度 (m/s -> deg/s)
 */
float SpeedMps2Degs(float speed_mps)
{
    if (speed_mps == 0.0f)
        return 0.0f;

    // 限制最大输入，防止异常值
    if (speed_mps > 20.0f)
        speed_mps = 20.0f;
    if (speed_mps < -20.0f)
        speed_mps = -20.0f;

    // 公式: (线速度 / 半径) * (180/PI) * 补偿系数
    // v = w * r  =>  w = v / r
    return (speed_mps / FRICTION_WHEEL_RADIUS) * RAD_2_DEGREE * SLIP_COMPENSATION;
}

/**
 * @brief 辅助函数：将角速度转换为线速度 (deg/s -> m/s)
 * @note 不包含打滑补偿，反映电机轴的理论线速度
 */
float SpeedAps2Mps(float speed_aps)
{
    // 公式: (角速度 / (180/PI)) * 半径
    // v = w * r
    return (speed_aps / RAD_2_DEGREE) * FRICTION_WHEEL_RADIUS;
}

/**
 * @brief 读取电机实时角速度
 * @param motor 电机实例
 * @return 电机实时角速度，空指针时返回 0
 * @note 这里统一做空指针保护，原因是当前外圈摩擦轮可能被裁剪为未安装配置，直接解引用会把射击任务打崩。
 */
static float GetMotorSpeedAps(const DJIMotorInstance *motor)
{
    if (motor == NULL)
        return 0.0f;

    return motor->measure.speed_aps;
}

/**
 * @brief 判断外圈摩擦轮是否完整存在
 * @return 1 表示外圈三电机都已初始化，0 表示当前平台没有完整外圈
 * @note 这里单独做能力检测，原因是掉速检测和就绪判定需要根据硬件配置自动降级，避免把不存在的外圈当成故障。
 */
static uint8_t HasOuterFrictionWheel(void)
{
    return (friction_outer_left != NULL) &&
           (friction_outer_right != NULL) &&
           (friction_outer_down != NULL);
}

/**
 * @brief 将拨盘输出端角度换算为电机 total_angle 单位
 * @param output_angle_deg 拨盘输出端角度，单位为 deg
 * @return 电机转子总角度，单位为 deg
 * @note total_angle 是电机转子多圈角度而非拨盘输出角度，所以位置环目标必须乘减速比，否则会少转约 19 倍。
 */
static float LoaderOutputAngleToMotorAngle(float output_angle_deg)
{
    return output_angle_deg * REDUCTION_RATIO_LOADER;
}

/**
 * @brief 将“多少发弹丸的机械行程”换算为电机 total_angle 单位
 * @param bullet_count 以“发”为单位的拨盘行程
 * @return 电机转子总角度，单位为 deg
 * @note 用弹丸数量描述位置目标更符合射击机构语义，原因是单发、二连发、反转都天然按“几发”的机械节距定义。
 */
static float LoaderBulletCountToMotorAngle(float bullet_count)
{
    return bullet_count * LOADER_MOTOR_ANGLE_PER_BULLET;
}

/**
 * @brief 将射频换算为拨盘电机速度参考
 * @param bullet_rate 发射频率，单位为 bullet/s
 * @return 拨盘电机速度参考，单位为 deg/s
 * @note 这里统一使用单发机械节距做换算，作用是让单发、连发和二连发共享同一套几何语义；
 *       原因是旧代码里残留 `/8` 常量，会把实际“每发角度”与单发位置环目标拆成两套定义。
 */
static float LoaderBulletRateToMotorSpeed(float bullet_rate)
{
    return bullet_rate * LOADER_MOTOR_ANGLE_PER_BULLET;
}

/**
 * @brief 约束摩擦轮控制 dt
 * @param dt_s 原始时间间隔，单位为 s
 * @return 约束后的 dt
 * @note 这里对时间步长做下限和上限保护，作用是让时间型 ramp 在调度抖动下仍保持可控；
 *       原因是 `RobotTask` 不是硬实时周期，直接使用异常 dt 会把斜坡推进量放大成突跳。
 */
static float ClampFrictionRampDt(float dt_s)
{
    if (dt_s <= 0.0f)
        return 0.005f;
    if (dt_s > FRICTION_RAMP_MAX_DT_S)
        return FRICTION_RAMP_MAX_DT_S;

    return dt_s;
}

/**
 * @brief 按给定加减速度把当前参考推进到目标参考
 * @param current_ref 当前参考值
 * @param target_ref 目标参考值
 * @param up_rate 上升斜率，单位为 deg/s^2
 * @param down_rate 下降斜率，单位为 deg/s^2
 * @param dt_s 时间步长，单位为 s
 * @return 新的参考值
 * @note 这里统一封装升降速限幅，作用是让开摩擦轮和关摩擦轮都走同一套时间语义；
 *       原因是用户反馈斜坡过慢，后续只需要改速率常量就能整体调整响应。
 */
static float RampFrictionRef(float current_ref, float target_ref, float up_rate, float down_rate, float dt_s)
{
    float max_delta;
    float diff = target_ref - current_ref;

    if (diff == 0.0f)
        return target_ref;

    if (fabsf(target_ref) > fabsf(current_ref))
        max_delta = up_rate * dt_s;
    else
        max_delta = down_rate * dt_s;

    if (diff > max_delta)
        return current_ref + max_delta;
    if (diff < -max_delta)
        return current_ref - max_delta;

    return target_ref;
}

/**
 * @brief 缓存摩擦轮最终目标转速绝对值
 * @param inner_left_ref 内圈左最终目标
 * @param inner_right_ref 内圈右最终目标
 * @param inner_down_ref 内圈下最终目标
 * @param outer_left_ref 外圈左最终目标
 * @param outer_right_ref 外圈右最终目标
 * @param outer_down_ref 外圈下最终目标
 * @note 这里把最终目标和实时 ramp 输出分开管理，作用是让 ready/recover 判定盯住真正的稳态速度；
 *       原因是若只盯住 ramp 中间值，摩擦轮在升速早期也会被误判成“已经 ready”。
 */
static void UpdateFrictionTargetAbs(float inner_left_ref, float inner_right_ref, float inner_down_ref,
                                    float outer_left_ref, float outer_right_ref, float outer_down_ref)
{
    target_inner_left_deg_abs = friction_inner_left != NULL ? fabsf(inner_left_ref) : 0.0f;
    target_inner_right_deg_abs = friction_inner_right != NULL ? fabsf(inner_right_ref) : 0.0f;
    target_inner_down_deg_abs = friction_inner_down != NULL ? fabsf(inner_down_ref) : 0.0f;
    target_outer_left_deg_abs = friction_outer_left != NULL ? fabsf(outer_left_ref) : 0.0f;
    target_outer_right_deg_abs = friction_outer_right != NULL ? fabsf(outer_right_ref) : 0.0f;
    target_outer_down_deg_abs = friction_outer_down != NULL ? fabsf(outer_down_ref) : 0.0f;
}

/**
 * @brief 给拨盘电机下发速度环目标
 * @param speed_ref 目标角速度，单位为 deg/s
 * @note 统一封装速度环切换和设定值下发，原因是单发/停机/连发都会复用，避免多处切环遗漏。
 */
static void LoaderSetSpeedRef(float speed_ref)
{
    DJIMotorOuterLoop(loader, SPEED_LOOP);
    DJIMotorSetRef(loader, speed_ref);
}

/**
 * @brief 给拨盘电机下发位置环目标
 * @param angle_ref 目标 total_angle，单位为 deg
 * @note 统一封装位置环切换和设定值下发，原因是单发冲刺、掉速锁角、二三连发都必须严格使用同一角度语义。
 */
static void LoaderSetAngleRef(float angle_ref)
{
    DJIMotorOuterLoop(loader, ANGLE_LOOP);
    DJIMotorSetRef(loader, angle_ref);
}

/**
 * @brief 统一设置摩擦轮前馈
 * @param inner_ff 内圈前馈电流
 * @param outer_ff 外圈前馈电流
 * @note 这里把 6 个前馈量集中写入，原因是单发状态机有多处清零/注入动作，手写六次很容易漏改。
 */
static void SetFrictionFeedforward(float inner_ff, float outer_ff)
{
    ff_inner_left = inner_ff;
    ff_inner_right = inner_ff;
    ff_inner_down = inner_ff;
    ff_outer_left = outer_ff;
    ff_outer_right = outer_ff;
    ff_outer_down = outer_ff;
}

/**
 * @brief 安全地下发摩擦轮参考值
 * @param motor 电机实例
 * @param ref 目标角速度
 * @note 这里对可选电机做空指针保护，原因是英雄当前代码允许裁剪外圈，空指针直接写 ref 会触发 HardFault。
 */
static void SetFrictionRefIfReady(DJIMotorInstance *motor, float ref)
{
    if (motor != NULL)
        DJIMotorSetRef(motor, ref);
}

/**
 * @brief 安全地启停电机
 * @param motor 电机实例
 * @param enable 1 使能，0 停止
 * @note 统一封装空指针保护，原因是 ShootTask 会批量启停所有摩擦轮，当前硬件配置可能并不具备全部 6 个电机。
 */
static void SetMotorEnableIfReady(DJIMotorInstance *motor, uint8_t enable)
{
    if (motor == NULL)
        return;

    if (enable)
        DJIMotorEnable(motor);
    else
        DJIMotorStop(motor);
}

/**
 * @brief 判断单个摩擦轮当前速度是否回到目标附近
 * @param motor 电机实例
 * @param target_abs 目标速度绝对值，单位为 deg/s
 * @param threshold 允许误差，单位为 deg/s
 * @return 1 表示已回到目标附近，0 表示仍未稳定
 * @note 这里统一对实际速度和目标都取绝对值，作用是兼容反装电机；
 *       原因是左侧摩擦轮方向相反，但 ready/recover 语义只关心速度幅值。
 */
static uint8_t IsFrictionMotorStable(const DJIMotorInstance *motor, float target_abs, float threshold)
{
    if (motor == NULL)
        return 1;

    return fabsf(fabsf(GetMotorSpeedAps(motor)) - target_abs) <= threshold;
}

/**
 * @brief 判断全部已安装摩擦轮是否达到最终目标附近
 * @param threshold 允许误差，单位为 deg/s
 * @return 1 表示全部稳定，0 表示仍有电机未到位
 * @note 这里统一盯住最终稳态目标，作用是防止 ramp 尚未跑满时就把摩擦轮误判成 ready；
 *       原因是单发触发安全性依赖“最终射速 ready”，而不是“当前 ramp 中间值能跟上”。
 */
static uint8_t IsAllFrictionStableAgainstTarget(float threshold)
{
    if (!IsFrictionMotorStable(friction_inner_left, target_inner_left_deg_abs, threshold))
        return 0;
    if (!IsFrictionMotorStable(friction_inner_right, target_inner_right_deg_abs, threshold))
        return 0;
    if (!IsFrictionMotorStable(friction_inner_down, target_inner_down_deg_abs, threshold))
        return 0;
    if (!IsFrictionMotorStable(friction_outer_left, target_outer_left_deg_abs, threshold))
        return 0;
    if (!IsFrictionMotorStable(friction_outer_right, target_outer_right_deg_abs, threshold))
        return 0;
    if (!IsFrictionMotorStable(friction_outer_down, target_outer_down_deg_abs, threshold))
        return 0;

    return 1;
}

/**
 * @brief 记录单发控制使用的内圈掉速基线
 * @note 这里单独维护控制基线，作用是让生产控制不依赖调试模块内部结构；
 *       原因是调试快照允许慢慢验证，而真正刹车必须走最短路径。
 */
static void RecordControlDipBaseline(void)
{
    dip_control.inner_left_baseline = fabsf(GetMotorSpeedAps(friction_inner_left));
    dip_control.inner_right_baseline = fabsf(GetMotorSpeedAps(friction_inner_right));
    dip_control.inner_down_baseline = fabsf(GetMotorSpeedAps(friction_inner_down));
    dip_control.outer_left_baseline = fabsf(GetMotorSpeedAps(friction_outer_left));
    dip_control.outer_right_baseline = fabsf(GetMotorSpeedAps(friction_outer_right));
    dip_control.outer_down_baseline = fabsf(GetMotorSpeedAps(friction_outer_down));
}

/**
 * @brief 以峰值保持方式更新单发控制基线
 * @note 这里把基线跟随到喂弹过程中的真实峰值，作用是吸收前馈和惯性带来的正常加速；
 *       原因是掉速判定依赖“基线 - 当前”，若基线不更新就会把正常升速误算成掉速不足。
 */
static void UpdateControlDipPeakBaseline(void)
{
    float current_abs;

    current_abs = fabsf(GetMotorSpeedAps(friction_inner_left));
    if (current_abs > dip_control.inner_left_baseline)
        dip_control.inner_left_baseline = current_abs;

    current_abs = fabsf(GetMotorSpeedAps(friction_inner_right));
    if (current_abs > dip_control.inner_right_baseline)
        dip_control.inner_right_baseline = current_abs;

    current_abs = fabsf(GetMotorSpeedAps(friction_inner_down));
    if (current_abs > dip_control.inner_down_baseline)
        dip_control.inner_down_baseline = current_abs;

    current_abs = fabsf(GetMotorSpeedAps(friction_outer_left));
    if (current_abs > dip_control.outer_left_baseline)
        dip_control.outer_left_baseline = current_abs;

    current_abs = fabsf(GetMotorSpeedAps(friction_outer_right));
    if (current_abs > dip_control.outer_right_baseline)
        dip_control.outer_right_baseline = current_abs;

    current_abs = fabsf(GetMotorSpeedAps(friction_outer_down));
    if (current_abs > dip_control.outer_down_baseline)
        dip_control.outer_down_baseline = current_abs;
}

/**
 * @brief 计算当前喂弹步的推进量
 * @return 当前推进的机械行程，单位为 bullet
 * @note 这里把电机 total_angle 统一换算回“发”的语义，作用是让掉速门槛直接按机械节距表达；
 *       原因是用户关心的是“已经推进了多少发的距离”，不是转子角度。
 */
static float GetCurrentFeedProgressBullet(void)
{
    float progress_angle = loader->measure.total_angle - single_fire.rush_start_angle;

    if (progress_angle < 0.0f)
        progress_angle = 0.0f;

    return progress_angle / LOADER_MOTOR_ANGLE_PER_BULLET;
}

/**
 * @brief 以内圈基线计算当前掉速电机数量和平均掉速
 * @param out_avg_dip 输出内圈平均掉速，单位为 deg/s
 * @return 当前超过掉速阈值的内圈电机数量
 * @note 这里单独返回“数量 + 平均值”两种特征，作用是同时兼顾强咬弹和弱但持续掉速两种真实发射形态；
 *       原因是鹅颈弹链工况变化大，只看单一特征容易漏检或误检。
 */
static uint8_t GetInnerDipMetrics(float *out_avg_dip)
{
    uint8_t dip_count = 0;
    float dip_left = dip_control.inner_left_baseline - fabsf(GetMotorSpeedAps(friction_inner_left));
    float dip_right = dip_control.inner_right_baseline - fabsf(GetMotorSpeedAps(friction_inner_right));
    float dip_down = dip_control.inner_down_baseline - fabsf(GetMotorSpeedAps(friction_inner_down));

    if (out_avg_dip != NULL)
        *out_avg_dip = (dip_left + dip_right + dip_down) * 0.3333333f;

    if (dip_left > FRICTION_SPEED_DIP_THRESHOLD)
        dip_count++;
    if (dip_right > FRICTION_SPEED_DIP_THRESHOLD)
        dip_count++;
    if (dip_down > FRICTION_SPEED_DIP_THRESHOLD)
        dip_count++;

    return dip_count;
}

/**
 * @brief 以外圈基线计算当前掉速电机数量和平均掉速
 * @param out_avg_dip 输出外圈平均掉速，单位为 deg/s
 * @return 当前超过阈值的外圈电机数量
 * @note 这里仅把外圈作为辅助确认来源，作用是在内圈只出现“较弱但连续”的掉速时再补一层真实性佐证；
 *       原因是外圈物理位置更靠后，不能拿来做第一时间刹车，但可以帮助过滤掉半咬弹造成的假成功。
 */
static uint8_t GetOuterDipMetrics(float *out_avg_dip)
{
    uint8_t dip_count = 0;
    float dip_left;
    float dip_right;
    float dip_down;

    if (!HasOuterFrictionWheel()) {
        if (out_avg_dip != NULL)
            *out_avg_dip = 0.0f;
        return 0;
    }

    dip_left = dip_control.outer_left_baseline - fabsf(GetMotorSpeedAps(friction_outer_left));
    dip_right = dip_control.outer_right_baseline - fabsf(GetMotorSpeedAps(friction_outer_right));
    dip_down = dip_control.outer_down_baseline - fabsf(GetMotorSpeedAps(friction_outer_down));

    if (out_avg_dip != NULL)
        *out_avg_dip = (dip_left + dip_right + dip_down) * 0.3333333f;

    if (dip_left > OUTER_DIP_CONFIRM_THRESHOLD)
        dip_count++;
    if (dip_right > OUTER_DIP_CONFIRM_THRESHOLD)
        dip_count++;
    if (dip_down > OUTER_DIP_CONFIRM_THRESHOLD)
        dip_count++;

    return dip_count;
}

/**
 * @brief 判断当前是否应该以内圈掉速立即锁角
 * @return 1 表示应该锁角，0 表示继续喂弹
 * @note 这里把“至少两路明显掉速”与“平均掉速连续两拍”合并成生产控制条件，作用是更早刹住拨盘；
 *       原因是外圈掉速确认虽然更晚更完整，但对防多发来说已经太迟。
 */
static uint8_t ShouldLockByInnerDip(void)
{
    float inner_avg_dip = 0.0f;
    float outer_avg_dip = 0.0f;
    uint8_t dip_count;
    uint8_t outer_dip_count;

    if (GetCurrentFeedProgressBullet() < SF_MIN_VALID_DIP_PROGRESS_BULLET) {
        single_fire.inner_dip_stable_count = 0;
        return 0;
    }

    dip_count = GetInnerDipMetrics(&inner_avg_dip);
    if (dip_count >= DIP_MIN_MOTOR_COUNT) {
        single_fire.inner_dip_stable_count = 0;
        return 1;
    }

    outer_dip_count = GetOuterDipMetrics(&outer_avg_dip);

    if (inner_avg_dip > FRICTION_SPEED_DIP_THRESHOLD) {
        if (single_fire.inner_dip_stable_count < 0xFFu)
            single_fire.inner_dip_stable_count++;
    } else {
        single_fire.inner_dip_stable_count = 0;
    }

    if (single_fire.inner_dip_stable_count < SF_DIP_AVG_STABLE_CYCLES)
        return 0;

    if (!HasOuterFrictionWheel()) {
        // What: 外圈不存在时保留原有平均掉速确认链路；Why: 当前工程允许裁剪外圈配置，辅助确认不能让无外圈平台完全失去单发能力。
        return 1;
    }

    // What: 对“弱但连续”的内圈平均掉速增加外圈辅助确认；Why: 这样可以保留早期锁角优势，同时减少半咬弹或扰动被误判成成功带来的空发。
    return (outer_dip_count >= 1u) || (outer_avg_dip > OUTER_DIP_CONFIRM_THRESHOLD);
}

/**
 * @brief 接受一个待处理单发请求并进入待速态
 * @param current_time 当前系统时间，单位为 ms
 * @note 这里把边沿请求显式缓存并在状态机内消费，作用是让鼠标和 VT03 的瞬时点击不会因为状态切换被吞掉；
 *       原因是新的 ready/recover 策略会让请求延后执行，不能再在入口处立刻清空。
 */
static void AcceptPendingSingleFireRequest(float current_time)
{
    fire_trigger.pending_fire = 0;
    single_fire.state = SF_WAIT_SPEED;
    single_fire.retry_count = 0;
    single_fire.shot_start_time = 0.0f;
    single_fire.feed_start_time = 0.0f;
    single_fire.retry_start_time = 0.0f;
    single_fire.inner_dip_stable_count = 0;
    single_fire.recover_stable_count = 0;
    single_fire.lock_target_angle = loader->measure.total_angle;
    single_fire.brake_start_time = current_time;
    LoaderSetAngleRef(single_fire.lock_target_angle);
}

/**
 * @brief 中止单发状态机
 * @note 这里同时清空前馈和挂起触发，原因是切换到二连发/连发/反转时必须把单发遗留状态完全收口，避免旧状态抢控制权。
 */
static void AbortSingleFire(void)
{
    single_fire.state = SF_IDLE;
    single_fire.retry_count = 0;
    single_fire.shot_start_time = 0.0f;
    single_fire.feed_start_time = 0.0f;
    single_fire.retry_start_time = 0.0f;
    single_fire.inner_dip_stable_count = 0;
    single_fire.recover_stable_count = 0;
    fire_trigger.pending_fire = 0;
    SetFrictionFeedforward(0.0f, 0.0f);
}

/**
 * @brief 判断单发事务是否处于“可被 STOP 打断”的补发阶段
 * @return 1 表示当前已经进入补发等待或补发送弹，0 表示仍处于首发自保持阶段
 * @note 这里统一识别补发等待和补发步进两种状态，作用是让 STOP 指令在补发期间立即停拨；
 *       原因是首发冲刺需要保证一次触发完整收口，而补发只是附加尝试，安全优先级高于补发完成率。
 */
static uint8_t SingleFireIsRetryActive(void)
{
    if (single_fire.state == SF_RETRYING)
        return 1;

    if (single_fire.state == SF_FEEDING && single_fire.retry_count > 0)
        return 1;

    return 0;
}

/**
 * @brief 开始一次单发喂弹尝试
 * @param current_time 当前系统时间 (ms)
 * @param feed_bullet_count 本次喂弹步距，单位为 bullet
 * @param reset_transaction_timer 1 表示重置整次事务计时，0 表示沿用已有计时
 * @note 这里把首发和补发共用的准备动作收敛到一个入口，作用是保证基线、前馈和位置目标始终同步更新；
 *       原因是当前单发包含首发和补发两条路径，散落手写很容易把某个步骤漏掉。
 */
static void BeginSingleFireFeedAttempt(float current_time, float feed_bullet_count, uint8_t reset_transaction_timer)
{
    single_fire.state = SF_FEEDING;
    single_fire.feed_start_time = current_time;
    if (reset_transaction_timer) {
        single_fire.shot_start_time = current_time;
    }
    single_fire.rush_start_angle = loader->measure.total_angle;
    single_fire.rush_target_angle = single_fire.rush_start_angle +
                                    LoaderBulletCountToMotorAngle(feed_bullet_count);
    single_fire.lock_target_angle = single_fire.rush_start_angle;
    single_fire.baseline_speed = GetInnerFrictionAvgSpeed();
    single_fire.outer_baseline_speed = GetOuterFrictionAvgSpeed();
    single_fire.inner_dip_stable_count = 0;

    RecordDipBaseline();
    RecordControlDipBaseline();
    SetFrictionFeedforward(0.0f, 0.0f);
    LoaderSetAngleRef(single_fire.rush_target_angle);
}

/**
 * @brief 更新回速稳定计数并返回是否已经恢复
 * @return 1 表示摩擦轮已连续稳定恢复，0 表示仍需等待
 * @note 这里要求摩擦轮连续多拍回到最终目标附近，作用是把“刚反弹回来”的瞬态与“真正恢复稳态”区分开；
 *       原因是高频点射时下一发过早进入会直接放大多发风险。
 */
static uint8_t ShootIsSpeedRecovered(void)
{
    if (IsAllFrictionStableAgainstTarget(FRICTION_SPEED_RECOVER_THRESHOLD)) {
        if (single_fire.recover_stable_count < 0xFFu)
            single_fire.recover_stable_count++;
    } else {
        single_fire.recover_stable_count = 0;
    }

    return single_fire.recover_stable_count >= FRICTION_RECOVER_STABLE_CYCLES;
}

/**
 * @brief 进入单发有限重试等待态
 * @param current_time 当前系统时间 (ms)
 * @note 这里先锁住当前位置再等待下一次补步，作用是让重试节拍由时间控制而不是由电机惯性决定；
 *       原因是用户要求“按一定频率位置环转动”，若不先锁住当前位置，补发间隔会随机械阻力漂移。
 */
static void EnterSingleFireRetryWait(float current_time)
{
    single_fire.state = SF_RETRYING;
    single_fire.lock_target_angle = loader->measure.total_angle;
    single_fire.retry_start_time = current_time;
    single_fire.inner_dip_stable_count = 0;
    SetFrictionFeedforward(0.0f, 0.0f);
    LoaderSetAngleRef(single_fire.lock_target_angle);
}

/**
 * @brief 结束本次单发事务并锁住当前位置
 * @param current_time 当前系统时间 (ms)
 * @param shot_success 1 表示确认出弹, 0 表示未确认出弹
 * @note 这里统一收口成功/失败两条分支，作用是确保锁角、前馈清零、计数更新始终一致；
 *       原因是单发现在包含初次冲刺和有限补发，多处手写收尾容易出现某个分支漏清状态。
 */
static void FinishSingleFire(float current_time, uint8_t shot_success)
{
    single_fire.brake_start_time = current_time;
    single_fire.lock_target_angle = loader->measure.total_angle;
    single_fire.inner_dip_stable_count = 0;
    single_fire.recover_stable_count = 0;
    // What: 单发事务结束时主动清空挂起请求；Why: 当前策略明确禁止“上一发执行过程中顺延排队下一发”，否则一次点击仍可能被拆成连续两发。
    fire_trigger.pending_fire = 0;
    SetFrictionFeedforward(0.0f, 0.0f);

    if (shot_success) {
        // What: 发射成功后先进入回速等待；Why: 新一发必须建立在摩擦轮已经恢复稳态的前提上，不能刚咬完一颗又立刻继续推。
        single_fire.state = SF_WAIT_RECOVER;
        single_fire.fire_count++;
    } else {
        // What: 发射失败后直接回到锁角保持；Why: 空仓或未确认出弹时不应继续自动卷弹，只等待下一次明确触发。
        single_fire.state = SF_LOCKING;
        single_fire.feed_timeout_count++;
    }

    LoaderSetAngleRef(single_fire.lock_target_angle);
}

/**
 * @brief 分别设置两级摩擦轮的速度
 * @param inner_mps 第一级（内圈）目标射速 m/s
 * @param outer_mps 第二级（外圈）目标射速 m/s
 */
void ShootSetSpeedDual(float inner_mps, float outer_mps)
{
    float dt_s = ClampFrictionRampDt(DWT_GetDeltaT(&friction_ramp_dwt_cnt));
    float target_inner_deg = SpeedMps2Degs(inner_mps);
    float target_outer_deg = SpeedMps2Degs(outer_mps);
    float target_inner_left_ref;
    float target_inner_right_ref;
    float target_inner_down_ref;
    float target_outer_left_ref;
    float target_outer_right_ref;
    float target_outer_down_ref;
    float ref_inner_base;
    float ref_outer_base;

    // What: 按真实 dt 推进摩擦轮升降速；Why: 这样 ready 时间由“秒”决定，不再由任务循环次数偶然决定。
    current_inner_deg = RampFrictionRef(current_inner_deg, target_inner_deg,
                                        FRICTION_RAMP_UP_RATE_DPS_PER_S,
                                        FRICTION_RAMP_DOWN_RATE_DPS_PER_S,
                                        dt_s);
    current_outer_deg = RampFrictionRef(current_outer_deg, target_outer_deg,
                                        FRICTION_RAMP_UP_RATE_DPS_PER_S,
                                        FRICTION_RAMP_DOWN_RATE_DPS_PER_S,
                                        dt_s);

    target_inner_left_ref = target_inner_deg != 0.0f ? target_inner_deg + SpeedMps2Degs(FRICTION_TRIM_INNER_LEFT) : 0.0f;
    target_inner_right_ref = target_inner_deg != 0.0f ? target_inner_deg + SpeedMps2Degs(FRICTION_TRIM_INNER_RIGHT) : 0.0f;
    target_inner_down_ref = target_inner_deg != 0.0f ? target_inner_deg + SpeedMps2Degs(FRICTION_TRIM_INNER_DOWN) : 0.0f;
    target_outer_left_ref = target_outer_deg != 0.0f ? target_outer_deg + SpeedMps2Degs(FRICTION_TRIM_OUTER_LEFT) : 0.0f;
    target_outer_right_ref = target_outer_deg != 0.0f ? target_outer_deg + SpeedMps2Degs(FRICTION_TRIM_OUTER_RIGHT) : 0.0f;
    target_outer_down_ref = target_outer_deg != 0.0f ? target_outer_deg + SpeedMps2Degs(FRICTION_TRIM_OUTER_DOWN) : 0.0f;
    UpdateFrictionTargetAbs(target_inner_left_ref, target_inner_right_ref, target_inner_down_ref,
                            target_outer_left_ref, target_outer_right_ref, target_outer_down_ref);

    // 2. 设置第一级（内圈3个电机）- 负责主要加速
    // 仅在基础速度非零（摩擦轮已开启）时才叠加各轮独立 Trim 偏置；
    // 原因: 若基础速度为 0（摩擦轮关闭）时仍叠加非零 Trim，电机会持续转动，与关闭意图相悖。
    ref_inner_base = current_inner_deg; // 经软启动斜坡后的内圈基础目标角速度
    SetFrictionRefIfReady(friction_inner_left, ref_inner_base != 0.0f ? ref_inner_base + SpeedMps2Degs(FRICTION_TRIM_INNER_LEFT) : 0.0f);
    SetFrictionRefIfReady(friction_inner_right, ref_inner_base != 0.0f ? ref_inner_base + SpeedMps2Degs(FRICTION_TRIM_INNER_RIGHT) : 0.0f);
    SetFrictionRefIfReady(friction_inner_down, ref_inner_base != 0.0f ? ref_inner_base + SpeedMps2Degs(FRICTION_TRIM_INNER_DOWN) : 0.0f);

    // 3. 设置第二级（外圈3个电机）- 负责稳速/微加速
    // 同理，基础速度为 0 时直接下发 0，不叠加 Trim
    ref_outer_base = current_outer_deg; // 经软启动斜坡后的外圈基础目标角速度
    SetFrictionRefIfReady(friction_outer_left, ref_outer_base != 0.0f ? ref_outer_base + SpeedMps2Degs(FRICTION_TRIM_OUTER_LEFT) : 0.0f);
    SetFrictionRefIfReady(friction_outer_right, ref_outer_base != 0.0f ? ref_outer_base + SpeedMps2Degs(FRICTION_TRIM_OUTER_RIGHT) : 0.0f);
    SetFrictionRefIfReady(friction_outer_down, ref_outer_base != 0.0f ? ref_outer_base + SpeedMps2Degs(FRICTION_TRIM_OUTER_DOWN) : 0.0f);
}
/**
 * @brief 更新调试数据 (将电机反馈的角速度转换为线速度)
 * @note 调用调试模块的接口函数
 */
static void UpdateFrictionDebugInfo(void)
{
    // 调用调试模块接口，传入6个电机的实时速度
    ShootDebug_UpdateFrictionInfo(
        GetMotorSpeedAps(friction_inner_left),
        GetMotorSpeedAps(friction_inner_right),
        GetMotorSpeedAps(friction_inner_down),
        GetMotorSpeedAps(friction_outer_left),
        GetMotorSpeedAps(friction_outer_right),
        GetMotorSpeedAps(friction_outer_down));
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

    // 计算是否堵转
    uint8_t is_stalled = (current_abs > STALL_CURRENT_THRESHOLD) &&
                         (speed_abs < STALL_SPEED_THRESHOLD);

    // 更新调试模块的堵转信息 (通过接口函数)
    // 注: 此处只更新电流/速度/is_stalled，状态机状态由 HandleLoaderStall 更新
    StallDebug_s *p = ShootDebug_GetStallPtr();
    p->current_abs = current_abs;
    p->speed_abs = speed_abs;
    p->is_stalled = is_stalled;

    return is_stalled;
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
    // [优化] 使用乘法代替除法: / 3.0f -> * 0.3333333f
    float avg = (fabsf(GetMotorSpeedAps(friction_inner_left)) +
                 fabsf(GetMotorSpeedAps(friction_inner_right)) +
                 fabsf(GetMotorSpeedAps(friction_inner_down))) *
                0.3333333f;
    return avg;
}

/**
 * @brief 获取外圈摩擦轮平均速度 (deg/s)
 * @return 三个外圈摩擦轮速度的平均绝对值
 */
static float GetOuterFrictionAvgSpeed(void)
{
    // 外圈缺失时退化为内圈平均速度，作用是让掉速检测与发射等待仍可工作；
    // 原因是当前工程允许只装内圈摩擦轮，强依赖外圈会让单发状态机永远等不到有效检测。
    if (!HasOuterFrictionWheel())
        return GetInnerFrictionAvgSpeed();

    // [优化] 使用乘法代替除法
    float avg = (fabsf(GetMotorSpeedAps(friction_outer_left)) +
                 fabsf(GetMotorSpeedAps(friction_outer_right)) +
                 fabsf(GetMotorSpeedAps(friction_outer_down))) *
                0.3333333f;
    return avg;
}

/**
 * @brief 检测内圈摩擦轮是否发生掉速
 * @return 1 表示检测到掉速 (有弹丸通过), 0 表示正常
 * @note 通过与发射前记录的基准速度对比来判断是否掉速
 */
/**
 * @brief 检测摩擦轮是否发生掉速 (双重检测: 内圈 OR 外圈)
 * @return 1 表示检测到掉速, 0 表示正常
 */
static uint8_t IsFrictionDipping(void)
{
    float current_inner = GetInnerFrictionAvgSpeed();
    float current_outer = GetOuterFrictionAvgSpeed();
    uint8_t inner_dip = (single_fire.baseline_speed - current_inner) > FRICTION_SPEED_DIP_THRESHOLD;

    // What: 调试观测同时保留内外圈掉速；Why: 生产控制已经改成以内圈优先，但调试仍需要看到外圈是否同步通过。
    uint8_t outer_dip = (single_fire.outer_baseline_speed - current_outer) > FRICTION_SPEED_DIP_THRESHOLD;

    return inner_dip || outer_dip;
}

/**
 * @brief 记录6个电机的基准速度 (在送弹开始时调用)
 * @note 调用调试模块接口
 */
static void RecordDipBaseline(void)
{
    // 调用调试模块接口，传入6个电机的实时速度
    ShootDebug_RecordDipBaseline(
        GetMotorSpeedAps(friction_inner_left),
        GetMotorSpeedAps(friction_inner_right),
        GetMotorSpeedAps(friction_inner_down),
        GetMotorSpeedAps(friction_outer_left),
        GetMotorSpeedAps(friction_outer_right),
        GetMotorSpeedAps(friction_outer_down));
}

/**
 * @brief 执行掉速抓拍 (当检测到掉速时调用)
 * @note 调用调试模块接口
 */
static void TakeDipSnapshot(void)
{
    // 检查是否已抓拍，避免重复
    if (ShootDebug_IsSnapshotTaken())
        return;

    // 调用调试模块接口，传入6个电机的实时速度和发射计数
    ShootDebug_TakeDipSnapshot(
        GetMotorSpeedAps(friction_inner_left),
        GetMotorSpeedAps(friction_inner_right),
        GetMotorSpeedAps(friction_inner_down),
        GetMotorSpeedAps(friction_outer_left),
        GetMotorSpeedAps(friction_outer_right),
        GetMotorSpeedAps(friction_outer_down),
        single_fire.fire_count);
}

/**
 * @brief 验证掉速快照的有效性并保存到历史记录
 * @note 调用调试模块接口
 */
static void ValidateAndSaveDipSnapshot(void)
{
    ShootDebug_ValidateAndSave();
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
    // 在所有发射模式下进行堵转检测 (单发/二连发/三连发/连发)
    // 检测到堵转时自动反转解卡
    uint8_t is_shooting = (current_mode == LOAD_1_BULLET) || // [修复] 添加单发模式，使防堵转逻辑生效
                          (current_mode == LOAD_2_BULLET) ||
                          (current_mode == LOAD_3_BULLET) ||
                          (current_mode == LOAD_BURSTFIRE);

// [重构] 宏定义: 返回前统一更新调试状态 (通过接口获取指针)
#define RETURN_WITH_DEBUG(mode)                                \
    do {                                                       \
        ShootDebug_GetStallPtr()->state = stall_handler.state; \
        return (mode);                                         \
    } while (0)

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
        RETURN_WITH_DEBUG(current_mode); // 正常模式不修改

    case STALL_DETECTING:
        // 消抖阶段: 持续检测堵转状态
        if (!IsLoaderStalled()) {
            // 堵转消失,恢复正常
            stall_handler.state = STALL_NORMAL;
            RETURN_WITH_DEBUG(current_mode);
        }
        // 检查是否超过消抖时间
        if ((current_time - stall_handler.detect_start_time) >= STALL_DETECT_TIME) {
            // 确认堵转,进入反转阶段
            stall_handler.state = STALL_REVERSING;
            stall_handler.reverse_start_time = current_time;
            // 反转目标统一换算到电机 total_angle 单位，作用是让位置环反转量与机械半发节距一致；
            // 原因是 total_angle 统计的是转子角度，不乘减速比时反转角度会明显不足。
            stall_handler.reverse_target_angle = loader->measure.total_angle -
                                                 LoaderOutputAngleToMotorAngle(REVERSE_ANGLE);
            stall_handler.reverse_count++;

            // [重构] 更新调试模块的反转信息 (通过接口获取指针)
            StallDebug_s *p_stall = ShootDebug_GetStallPtr();
            p_stall->reverse_count = stall_handler.reverse_count;
            p_stall->reverse_target_angle = stall_handler.reverse_target_angle;

            // 设定反转目标角度 (在 ShootTask 的 switch 之前生效)
            LoaderSetAngleRef(stall_handler.reverse_target_angle);
        }
        RETURN_WITH_DEBUG(current_mode); // 消抖中暂不修改

    case STALL_REVERSING:
        // 反转阶段: 等待反转完成
        if ((current_time - stall_handler.reverse_start_time) >= REVERSE_TIME) {
            // 反转时间结束,进入恢复期
            stall_handler.state = STALL_RECOVERY;
            stall_handler.recovery_start_time = current_time;
        }
        // 返回 LOAD_STOP 跳过正常的发射逻辑,由状态机控制电机
        RETURN_WITH_DEBUG(LOAD_STOP);

    case STALL_RECOVERY:
        // 恢复阶段: 等待稳定后恢复发射
        if ((current_time - stall_handler.recovery_start_time) >= RECOVERY_TIME) {
            // 恢复期结束
            if (stall_handler.reverse_count >= MAX_REVERSE_COUNT) {
                // 连续反转达到上限,停止发射,需要人工干预
                stall_handler.state = STALL_NORMAL;
                stall_handler.reverse_count = 0;
                RETURN_WITH_DEBUG(LOAD_STOP); // 返回停止,等待新指令
            }
            // 恢复正常状态,继续之前的发射模式
            stall_handler.state = STALL_NORMAL;
            RETURN_WITH_DEBUG(stall_handler.saved_mode);
        }
        RETURN_WITH_DEBUG(LOAD_STOP); // 恢复期也返回 STOP

    default:
        stall_handler.state = STALL_NORMAL;
        RETURN_WITH_DEBUG(current_mode);
    }

// 取消宏定义, 避免污染全局命名空间
#undef RETURN_WITH_DEBUG
}

/**
 * @brief 检测摩擦轮转速是否就绪 (均在误差范围内)
 * @return 1=就绪, 0=未就绪
 */
static uint8_t ShootIsSpeedReady(void)
{
    // What: 目标速度为 0 时直接视为就绪；Why: 关闭摩擦轮或异常回退分支不应该被 ready 判定反向卡住。
    if (target_inner_left_deg_abs == 0.0f &&
        target_inner_right_deg_abs == 0.0f &&
        target_inner_down_deg_abs == 0.0f &&
        target_outer_left_deg_abs == 0.0f &&
        target_outer_right_deg_abs == 0.0f &&
        target_outer_down_deg_abs == 0.0f)
        return 1;

    return IsAllFrictionStableAgainstTarget(SHOOT_SPEED_READY_THRESHOLD);
}

/**
 * @brief 单发处理逻辑 (基于摩擦轮掉速的“位置环冲刺 + 掉速锁角”)
 * @param trigger_active 是否触发 (边沿信号)
 * @note 单发一旦触发就会自保持执行到锁角完成，原因是遥控拨杆通常是瞬时动作，不能要求操作者一直压住指令。
 */
static void HandleSingleFire(uint8_t trigger_active)
{
    float current_time = DWT_GetTimeline_ms();
    SingleFireDebug_s *p_sf = ShootDebug_GetSingleFirePtr();
    float inner_speed = GetInnerFrictionAvgSpeed();
    float outer_speed = GetOuterFrictionAvgSpeed();
    uint8_t has_pending_request = fire_trigger.pending_fire != 0u;

    switch (single_fire.state) {
    case SF_IDLE:
        if (has_pending_request) {
            // What: 空闲态有待处理请求时进入待速；Why: 鼠标和 VT03 都是边沿输入，请求必须先缓存住，再等摩擦轮真正 ready 后执行。
            AcceptPendingSingleFireRequest(current_time);
        } else {
            LoaderSetSpeedRef(0.0f);
        }
        break;

    case SF_WAIT_SPEED:
        if (ShootIsSpeedReady()) {
            // What: 只有最终目标速度 ready 后才允许首发冲刺；Why: 直接去掉旧版“500ms 没到速也硬发”的行为，避免半热态送弹带来的多发和首发无力。
            BeginSingleFireFeedAttempt(current_time, SF_RUSH_BULLET_COUNT, 1u);
        } else {
            // What: 待速期间只锁当前位置不再前推；Why: 用户反馈“点一下先动一下”就是旧前推逻辑带来的预拨感。
            LoaderSetAngleRef(single_fire.lock_target_angle);
        }
        break;

    case SF_FEEDING:
        if ((current_time - single_fire.feed_start_time) < FRICTION_FEEDFORWARD_TIME) {
            // 在冲刺初段给摩擦轮额外前馈，作用是让咬弹瞬间的速度塌陷更可控；
            // 原因是拨盘改成位置环后起步扭矩更猛，摩擦轮若不提前补能量，掉速幅值会放大且恢复更慢。
            SetFrictionFeedforward(FRICTION_FEEDFORWARD_CURRENT, 400.0f);
        } else {
            SetFrictionFeedforward(0.0f, 0.0f);
        }

        // 基准线随峰值更新，作用是把前馈带来的正常提速吸收进去；
        // 原因是掉速检测依赖“基准 - 当前”，若基准不跟峰值走就会把正常加速误判成未掉速。
        if (inner_speed > single_fire.baseline_speed) {
            single_fire.baseline_speed = inner_speed;
        }
        if (outer_speed > single_fire.outer_baseline_speed) {
            single_fire.outer_baseline_speed = outer_speed;
        }
        UpdateControlDipPeakBaseline();

        ShootDebug_UpdatePeakBaseline(
            GetMotorSpeedAps(friction_inner_left),
            GetMotorSpeedAps(friction_inner_right),
            GetMotorSpeedAps(friction_inner_down),
            GetMotorSpeedAps(friction_outer_left),
            GetMotorSpeedAps(friction_outer_right),
            GetMotorSpeedAps(friction_outer_down));

        if (ShouldLockByInnerDip()) {
            // What: 一旦内圈确认咬弹就立即锁角；Why: 内圈是弹丸最早通过的位置，用它刹车比等外圈确认更能压住多发。
            TakeDipSnapshot();
            ValidateAndSaveDipSnapshot();
            FinishSingleFire(current_time, 1);
        } else if ((single_fire.shot_start_time > 0.0f) &&
                   ((current_time - single_fire.shot_start_time) > SF_TRANSACTION_TIMEOUT)) {
            // What: 整次单发事务超过总时限后直接判失败；Why: 首发加补发都必须在短时间内收口，不能拖成持续卷弹。
            FinishSingleFire(current_time, 0);
        } else if (fabsf(single_fire.rush_target_angle - loader->measure.total_angle) < SF_RUSH_REACHED_TOLERANCE) {
            // What: 当前一步走到位但仍未确认出弹时进入等待补发；Why: 用户希望优先“这次打出去”，但也只接受有限次中等补步。
            if (single_fire.retry_count < SF_RETRY_MAX_COUNT) {
                EnterSingleFireRetryWait(current_time);
            } else {
                FinishSingleFire(current_time, 0);
            }
        } else {
            LoaderSetAngleRef(single_fire.rush_target_angle);
        }
        break;

    case SF_RETRYING:
        if (ShouldLockByInnerDip()) {
            // What: 补发等待期间若晚到掉速也按成功收口；Why: 掉速与机械到位并不同相，不能因为进入等待态就丢弃这次有效发射。
            TakeDipSnapshot();
            ValidateAndSaveDipSnapshot();
            FinishSingleFire(current_time, 1);
        } else if ((single_fire.shot_start_time > 0.0f) &&
                   ((current_time - single_fire.shot_start_time) > SF_TRANSACTION_TIMEOUT)) {
            FinishSingleFire(current_time, 0);
        } else if ((current_time - single_fire.retry_start_time) >= SF_RETRY_INTERVAL_MS) {
            single_fire.retry_count++;
            // What: 补发按固定节拍给一个中等步距；Why: 对鹅颈弹链要保留足够推进行程，但又不能重新回到一次冲很多发的旧策略。
            BeginSingleFireFeedAttempt(current_time, SF_RETRY_STEP_BULLET_COUNT, 0u);
        } else {
            LoaderSetAngleRef(single_fire.lock_target_angle);
        }
        break;

    case SF_WAIT_RECOVER:
        if (ShootIsSpeedRecovered()) {
            // What: 回速完成后统一退回锁角保持；Why: 当前需求是防双发优先，因此即便恢复期间出现新点击，也不能在这一拍自动续上一发。
            single_fire.state = SF_LOCKING;
        }
        LoaderSetAngleRef(single_fire.lock_target_angle);
        break;

    case SF_LOCKING:
        if (has_pending_request) {
            // What: 锁角态存在缓存请求时重新进入待速；Why: 这样单发请求的消费点只在状态机内，行为更可预测。
            AcceptPendingSingleFireRequest(current_time);
        }
        LoaderSetAngleRef(single_fire.lock_target_angle);
        break;
    }

    // 调试信息在状态机执行后再写回，作用是让调试器看到当前循环的最新状态；
    // 原因是单发状态会在一次调用中切换，先写调试会让观察值总是滞后一拍。
    p_sf->state = single_fire.state;
    p_sf->baseline_speed = single_fire.baseline_speed;
    p_sf->outer_baseline_speed = single_fire.outer_baseline_speed;
    p_sf->current_speed = GetInnerFrictionAvgSpeed();
    p_sf->current_outer_speed = GetOuterFrictionAvgSpeed();
    p_sf->loader_speed = loader->measure.speed_aps;
    p_sf->is_dipping = IsFrictionDipping();
    p_sf->speed_diff = single_fire.baseline_speed - p_sf->current_speed;
    p_sf->trigger_edge = trigger_active;
    p_sf->retry_count = single_fire.retry_count;
    p_sf->fire_count = single_fire.fire_count;
    p_sf->feed_timeout_count = single_fire.feed_timeout_count;
    p_sf->brake_start_time = single_fire.brake_start_time;
    p_sf->feed_start_time = single_fire.feed_start_time;
    p_sf->retry_start_time = single_fire.retry_start_time;
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    static uint16_t last_report_fire_count = 0;
    float current_time_ms = DWT_GetTimeline_ms();

    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 先对单发触发做边沿锁存，作用是把遥控器瞬时拨杆转换成一次完整的单发事务；
    // 原因是单发改成位置环冲刺后必须执行到“掉速锁角”收口，不能依赖 LOAD_1_BULLET 电平持续存在。
    if (shoot_cmd_recv.load_mode != LOAD_1_BULLET) {
        fire_trigger.trigger_consumed = 0;
    } else if (fire_trigger.last_mode != LOAD_1_BULLET && !fire_trigger.trigger_consumed) {
        uint8_t single_fire_can_accept_request =
            (uint8_t)(single_fire.state == SF_IDLE || single_fire.state == SF_LOCKING);
        uint8_t trigger_guard_elapsed =
            (uint8_t)((current_time_ms - fire_trigger.last_accept_time_ms) >= SF_TRIGGER_REARM_GUARD_MS);

        // What: 只允许在单发状态机已经完全收口时接收新的单发请求；Why: 若上一发还在待速、送弹、补发或回速阶段就继续收边沿，会把一次点击排成两发。
        if (single_fire_can_accept_request != 0u && trigger_guard_elapsed != 0u) {
            fire_trigger.pending_fire = 1;
            // What: 只有真正接受这次请求时才刷新最近接受时间；Why: 防抖窗口应该围绕“有效触发”建立，不能被一个被拒绝的伪边沿不断往后推迟。
            fire_trigger.last_accept_time_ms = current_time_ms;
        }

        // What: 无论本次边沿是否被接受都立即标记为已消费；Why: 同一次扳机抖动或鼠标回弹只能贡献一次判定，后续必须先释放回 STOP 才允许重新武装。
        fire_trigger.trigger_consumed = 1;
    }

    loader_mode_e requested_load_mode = shoot_cmd_recv.load_mode;

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
        current_inner_deg = 0.0f;
        current_outer_deg = 0.0f;
        // What: 急停时同步清空最终目标缓存；Why: ready/recover 判定不能继续拿上一次开火目标当作当前目标。
        UpdateFrictionTargetAbs(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        requested_load_mode = LOAD_STOP;
        fire_trigger.pending_fire = 0;
        fire_trigger.trigger_consumed = 0;
        AbortSingleFire();
        SetMotorEnableIfReady(friction_inner_left, 0);
        SetMotorEnableIfReady(friction_inner_right, 0);
        SetMotorEnableIfReady(friction_outer_left, 0);
        SetMotorEnableIfReady(friction_outer_right, 0);
        SetMotorEnableIfReady(friction_inner_down, 0);
        SetMotorEnableIfReady(friction_outer_down, 0);
        SetMotorEnableIfReady(loader, 0);
    } else // 恢复运行
    {
        SetMotorEnableIfReady(friction_inner_left, 1);
        SetMotorEnableIfReady(friction_inner_right, 1);
        SetMotorEnableIfReady(friction_outer_left, 1);
        SetMotorEnableIfReady(friction_outer_right, 1);
        SetMotorEnableIfReady(friction_inner_down, 1);
        SetMotorEnableIfReady(friction_outer_down, 1);
        SetMotorEnableIfReady(loader, 1);
    }

    // 当单发已经触发但遥控器电平回到 STOP 时，仅在首发待速/首发冲刺阶段继续送进堵转状态机；
    // 原因是首发事务尚未收口时仍需保留自动解卡，而补发属于附加尝试，必须允许用户立即停拨。
    loader_mode_e stall_input_mode = requested_load_mode;
    if ((requested_load_mode == LOAD_STOP) &&
        !SingleFireIsRetryActive() &&
        (single_fire.state == SF_WAIT_SPEED || single_fire.state == SF_FEEDING)) {
        stall_input_mode = LOAD_1_BULLET;
    }

    // 该函数会在堵转时自动替换发射模式为反转/停止状态
    loader_mode_e actual_load_mode = HandleLoaderStall(stall_input_mode);

    // 若不在休眠状态,根据实际发射模式进行拨盘电机参考值设定和模式切换
    switch (actual_load_mode) {
    // 停止拨盘
    case LOAD_STOP:
        // 补发期间收到 STOP 时直接中止状态机，作用是切断后续补步；
        // 原因是补发继续卷弹会扩大误触风险，安全停拨要优先于“补发补到底”。
        if (!SingleFireIsRetryActive() &&
            (single_fire.state != SF_IDLE || fire_trigger.pending_fire) &&
            stall_handler.state == STALL_NORMAL) {
            HandleSingleFire(fire_trigger.pending_fire);
        } else {
            AbortSingleFire();
            LoaderSetSpeedRef(0.0f);
        }
        break;
    // 单发模式
    case LOAD_1_BULLET:
        HandleSingleFire(fire_trigger.pending_fire);
        break;
    // 三连发
    case LOAD_3_BULLET:
        AbortSingleFire();
        if (hibernate_time + dead_time > DWT_GetTimeline_ms())
            break;
        LoaderSetAngleRef(loader->measure.total_angle + LoaderBulletCountToMotorAngle(3.0f));
        hibernate_time = DWT_GetTimeline_ms();
        dead_time = 300;
        break;
    // 二连发模式 (英雄专用)
    case LOAD_2_BULLET:
        AbortSingleFire();
        if (hibernate_time + dead_time > DWT_GetTimeline_ms())
            break;
        LoaderSetAngleRef(loader->measure.total_angle + LoaderBulletCountToMotorAngle(2.0f));
        hibernate_time = DWT_GetTimeline_ms();
        dead_time = 200;
        break;
    // 连发模式
    case LOAD_BURSTFIRE:
        // 在连发模式下复位单发标志位，确保切回单发时可立即触发一次
        AbortSingleFire();
        fire_trigger.trigger_consumed = 0;
        // What: 连发速度统一按单发机械节距换算；Why: 不能再保留与单发位置目标冲突的 `/8` 常量，否则不同模式会对“一发角度”产生两套定义。
        LoaderSetSpeedRef(LoaderBulletRateToMotorSpeed(shoot_cmd_recv.shoot_rate));
        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
        AbortSingleFire();
        LoaderSetAngleRef(loader->measure.total_angle - LoaderBulletCountToMotorAngle(0.5f));
        hibernate_time = DWT_GetTimeline_ms(); // 记录触发指令的时间
        dead_time = 150; // 完成1发弹丸发射的时间
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }

    // 这里保存原始遥控指令而不是实际执行模式，作用是让边沿检测只对用户动作敏感；
    // 原因是堵转状态机会临时把模式改成 STOP，若记录 actual_load_mode 会把一次拨杆动作误拆成多次触发。
    fire_trigger.last_mode = requested_load_mode;

    // [重构] 摩擦轮控制逻辑 (包含调试覆盖)
    // 优先级: 调试覆盖 > 正常指令
    FrictionWheelDebug_s *p_fric = ShootDebug_GetFrictionPtr();
    if (p_fric->override_enable) {
        // 调试模式: 直接使用 debug 结构体中的目标速度
        ShootSetSpeedDual(p_fric->target_inner_mps, p_fric->target_outer_mps);
    } else if (shoot_cmd_recv.friction_mode == FRICTION_ON) {
        // 正常模式: 根据不同的弹速等级设置不同的分级速度
        switch (shoot_cmd_recv.bullet_speed) {
        case BIG_AMU_12:
            // 目标12m/s：一级给11.5，二级给12.0
            ShootSetSpeedDual(11.0f, 11.7f);
            break;
        case BIG_AMU_16:
            // 目标16.5m/s：一级给15.5，二级给16.5
            ShootSetSpeedDual(14.5f, 16.5f);
            break;
        default:
            // 默认值
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

    // [优化] 降低调试信息更新频率 (Downsampling)
    // 只有每10ms更新一次调试数据, 避免每次循环(1ms)都进行浮点除法运算导致任务超时
    // E:[freeRTOS] MOTOR Task DELAY! dt = 1084 us
    static uint32_t debug_update_count = 0;
    if (debug_update_count++ % 10 == 0) {
        UpdateFrictionDebugInfo();
    }

    // 用 fire_count 边沿生成 bullet_fired_flag，作用是把“成功发射”变成单周期脉冲；
    // 原因是单发现在会长时间停留在锁角态，不能再用状态枚举直接映射“本周期已发射”。
    shoot_feedback_data.fire_count = single_fire.fire_count;
    shoot_feedback_data.bullet_fired_flag = (single_fire.fire_count != last_report_fire_count);
    last_report_fire_count = single_fire.fire_count;
    shoot_feedback_data.empty_flag = (single_fire.feed_timeout_count > 0);

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
