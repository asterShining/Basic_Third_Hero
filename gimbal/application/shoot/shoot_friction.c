#include "shoot_private.h"

/**
 * @brief 辅助函数：将线速度转换为角速度 (m/s -> deg/s)
 */
float SpeedMps2Degs(float speed_mps)
{
    if (speed_mps == 0.0f)
        return 0.0f;

    // 限制最大输入，防止异常值；原因是射速链路偶发异常值时不能直接把目标推到不受控的超大速度。
    if (speed_mps > 20.0f)
        speed_mps = 20.0f;
    if (speed_mps < -20.0f)
        speed_mps = -20.0f;

    // 公式: (线速度 / 半径) * (180/PI) * 补偿系数；原因是控制器内部统一以 deg/s 工作。
    return (speed_mps / FRICTION_WHEEL_RADIUS) * RAD_2_DEGREE * SLIP_COMPENSATION;
}

/**
 * @brief 辅助函数：将角速度转换为线速度 (deg/s -> m/s)
 * @note 不包含打滑补偿，反映电机轴的理论线速度
 */
float SpeedAps2Mps(float speed_aps)
{
    // 公式: (角速度 / (180/PI)) * 半径；原因是调试显示更关心近似线速度而不是电机角速度。
    return (speed_aps / RAD_2_DEGREE) * FRICTION_WHEEL_RADIUS;
}

/**
 * @brief 读取电机实时角速度
 * @param motor 电机实例
 * @return 电机实时角速度，空指针时返回 0
 */
float GetMotorSpeedAps(const DJIMotorInstance *motor)
{
    if (motor == NULL)
        return 0.0f;

    return motor->measure.speed_aps;
}

/**
 * @brief 判断外圈摩擦轮是否完整存在
 * @return 1 表示外圈三电机都已初始化，0 表示当前平台没有完整外圈
 */
uint8_t HasOuterFrictionWheel(void)
{
    return (friction_outer_left != NULL) &&
           (friction_outer_right != NULL) &&
           (friction_outer_down != NULL);
}

/**
 * @brief 将拨盘输出端角度换算为电机 total_angle 单位
 * @param output_angle_deg 拨盘输出端角度，单位为 deg
 * @return 电机转子总角度，单位为 deg
 */
float LoaderOutputAngleToMotorAngle(float output_angle_deg)
{
    return output_angle_deg * REDUCTION_RATIO_LOADER;
}

/**
 * @brief 将“多少发弹丸的机械行程”换算为电机 total_angle 单位
 * @param bullet_count 以“发”为单位的拨盘行程
 * @return 电机转子总角度，单位为 deg
 */
float LoaderBulletCountToMotorAngle(float bullet_count)
{
    return bullet_count * LOADER_MOTOR_ANGLE_PER_BULLET;
}

/**
 * @brief 将射频换算为拨盘电机速度参考
 * @param bullet_rate 发射频率，单位为 bullet/s
 * @return 拨盘电机速度参考，单位为 deg/s
 */
float LoaderBulletRateToMotorSpeed(float bullet_rate)
{
    return bullet_rate * LOADER_MOTOR_ANGLE_PER_BULLET;
}

/**
 * @brief 约束摩擦轮控制 dt
 * @param dt_s 原始时间间隔，单位为 s
 * @return 约束后的 dt
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
 */
void UpdateFrictionTargetAbs(float inner_left_ref, float inner_right_ref, float inner_down_ref,
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
 */
void LoaderSetSpeedRef(float speed_ref)
{
    DJIMotorOuterLoop(loader, SPEED_LOOP);
    DJIMotorSetRef(loader, speed_ref);
}

/**
 * @brief 给拨盘电机下发位置环目标
 * @param angle_ref 目标 total_angle，单位为 deg
 */
void LoaderSetAngleRef(float angle_ref)
{
    DJIMotorOuterLoop(loader, ANGLE_LOOP);
    DJIMotorSetRef(loader, angle_ref);
}

/**
 * @brief 统一设置摩擦轮前馈
 * @param inner_ff 内圈前馈电流
 * @param outer_ff 外圈前馈电流
 */
void SetFrictionFeedforward(float inner_ff, float outer_ff)
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
 */
void SetMotorEnableIfReady(DJIMotorInstance *motor, uint8_t enable)
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
 */
uint8_t IsAllFrictionStableAgainstTarget(float threshold)
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

    // 按真实 dt 推进摩擦轮升降速，目的是这样 ready 时间由“秒”决定，不再由任务循环次数偶然决定。
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

    // 内圈负责主要加速，因此先把软启动后的内圈基础目标下发给三个内圈电机，目的是trim 必须叠在经过 ramp 的基础目标上，而不是叠在稳态目标上。
    ref_inner_base = current_inner_deg;
    SetFrictionRefIfReady(friction_inner_left, ref_inner_base != 0.0f ? ref_inner_base + SpeedMps2Degs(FRICTION_TRIM_INNER_LEFT) : 0.0f);
    SetFrictionRefIfReady(friction_inner_right, ref_inner_base != 0.0f ? ref_inner_base + SpeedMps2Degs(FRICTION_TRIM_INNER_RIGHT) : 0.0f);
    SetFrictionRefIfReady(friction_inner_down, ref_inner_base != 0.0f ? ref_inner_base + SpeedMps2Degs(FRICTION_TRIM_INNER_DOWN) : 0.0f);

    // 外圈负责稳速与微加速，因此同样按软启动后的外圈基础目标单独叠加 trim，目的是基础速度为 0 时必须直接下发 0，不能让 trim 在关枪时把外圈偷偷带起来。
    ref_outer_base = current_outer_deg;
    SetFrictionRefIfReady(friction_outer_left, ref_outer_base != 0.0f ? ref_outer_base + SpeedMps2Degs(FRICTION_TRIM_OUTER_LEFT) : 0.0f);
    SetFrictionRefIfReady(friction_outer_right, ref_outer_base != 0.0f ? ref_outer_base + SpeedMps2Degs(FRICTION_TRIM_OUTER_RIGHT) : 0.0f);
    SetFrictionRefIfReady(friction_outer_down, ref_outer_base != 0.0f ? ref_outer_base + SpeedMps2Degs(FRICTION_TRIM_OUTER_DOWN) : 0.0f);
}

/**
 * @brief 更新调试数据 (将电机反馈的角速度转换为线速度)
 */
void UpdateFrictionDebugInfo(void)
{
    // 调用调试模块接口，传入 6 个电机的实时速度；原因是调试观测需要看到全部摩擦轮的同拍速度，而不是只看均值。
    ShootDebug_UpdateFrictionInfo(
        GetMotorSpeedAps(friction_inner_left),
        GetMotorSpeedAps(friction_inner_right),
        GetMotorSpeedAps(friction_inner_down),
        GetMotorSpeedAps(friction_outer_left),
        GetMotorSpeedAps(friction_outer_right),
        GetMotorSpeedAps(friction_outer_down));
}

/**
 * @brief 获取内圈摩擦轮平均速度 (deg/s)
 * @return 三个内圈摩擦轮速度的平均绝对值
 */
float GetInnerFrictionAvgSpeed(void)
{
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
float GetOuterFrictionAvgSpeed(void)
{
    // 外圈缺失时退化为内圈平均速度，作用是让掉速检测与发射等待仍可工作；
    // 原因是当前工程允许只装内圈摩擦轮，强依赖外圈会让单发状态机永远等不到有效检测。
    if (!HasOuterFrictionWheel())
        return GetInnerFrictionAvgSpeed();

    float avg = (fabsf(GetMotorSpeedAps(friction_outer_left)) +
                 fabsf(GetMotorSpeedAps(friction_outer_right)) +
                 fabsf(GetMotorSpeedAps(friction_outer_down))) *
                0.3333333f;
    return avg;
}

/**
 * @brief 记录 6 个电机的基准速度 (在送弹开始时调用)
 */
void RecordDipBaseline(void)
{
    // 调用调试模块接口，传入 6 个电机的实时速度；原因是后续掉速快照需要有同一拍的基线做对照。
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
 */
void TakeDipSnapshot(void)
{
    if (ShootDebug_IsSnapshotTaken())
        return;

    // 调用调试模块接口，传入 6 个电机的实时速度和发射计数；原因是同一拍内必须把速度与发射编号绑定起来。
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
 */
void ValidateAndSaveDipSnapshot(void)
{
    ShootDebug_ValidateAndSave();
}

/**
 * @brief 根据拨弹盘位置误差计算线性速度前馈
 *        只在 SF_FEEDING 状态（固定一发位置环送弹）期间输出有效前馈，
 *        其余状态清零以避免锁角保持时前馈干扰 PID 稳态。
 *        前馈注入速度环参考值入口（SPEED_FEEDFORWARD），速度环能感知
 *        这个额外速度需求并配合输出电流，不会出现与 PID 对抗的问题。
 */
void UpdateLoaderFeedforward(void)
{
    float angle_error;

    // 非送弹状态下前馈清零，避免锁角或空闲时前馈干扰 PID 稳态保持
    if (single_fire.state != SF_FEEDING) {
        ff_loader = 0.0f;
        return;
    }

    // 改用增量目标而非总行程目标，前馈力度与当前10度步进匹配而非远端目标
    angle_error = single_fire.increment_target_angle - loader->measure.total_angle;

    // 负误差表示已经超调到位，不应反向施加前馈，直接清零
    if (angle_error < 0.0f)
        angle_error = 0.0f;

    // 线性映射：前馈速度 (deg/s) = 增益 (1/s) × 位置误差 (deg)，
    // 误差越大速度补偿越大，到位时自然衰减为零
    ff_loader = LOADER_FF_GAIN * angle_error;

    // 限幅保护：防止冲刺起步时大误差导致速度参考值跳变过猛
    if (ff_loader > LOADER_FF_MAX_SPEED)
        ff_loader = LOADER_FF_MAX_SPEED;
}
