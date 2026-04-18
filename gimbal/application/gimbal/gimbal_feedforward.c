#include "gimbal_private.h"

// 下面这些状态只服务云台前馈链内部，目的是它们既需要跨周期保持，又不属于主任务模式控制的公共语义，因此收口在前馈实现文件里最安全。
static float pitch_ff_storage = 0.0f; // 保存 Pitch 电流前馈输出，目的是驱动侧通过指针异步读取，必须保证引用对象跨周期持续有效。
static float yaw_ff_storage = 0.0f; // 保存 Yaw 电流前馈输出，目的是让 Yaw 动态耦合补偿可以和 Pitch 一样稳定挂到 DM 前馈接口。
static float pitch_rate_filtered = 0.0f; // 缓存滤波后的 Pitch 关节速率，目的是科氏耦合直接吃原始速度会更抖，必须跨周期保留滤波状态。
static float yaw_rate_filtered = 0.0f; // 缓存滤波后的 Yaw 关节速率，目的是离心项对速率平方更敏感，先滤波才能避免噪声被放大。
static float last_yaw_ref_rad = 0.0f; // 缓存上一拍 Yaw 参考角，目的是Yaw 惯量前馈需要对目标角做求导，必须保留上一个参考值。
static float yaw_ref_rate_filtered = 0.0f; // 缓存滤波后的 Yaw 参考角速度，目的是黏性摩擦和静摩擦前馈都应基于平滑的目标运动趋势工作。
static float yaw_ref_acc_filtered = 0.0f; // 缓存滤波后的 Yaw 参考角加速度，目的是惯量前馈依赖加速度，直接使用二次差分噪声会过大。

/**
 * @brief 将浮点数映射为符号位
 *
 * @param value 输入值
 * @return float 正返回 1，负返回 -1，零返回 0
 */
static float GetFloatSign(float value)
{
    // 统一获取符号位，目的是静摩擦前馈只关心方向，不应该在各个调用点重复写一套判断。
    if (value > 0.0f) {
        return 1.0f;
    }
    if (value < 0.0f) {
        return -1.0f;
    }
    return 0.0f;
}

/**
 * @brief 复位 Yaw 参考导数状态
 *
 * @param yaw_ref_rad 当前参考角，单位 rad
 */
static void ResetYawReferenceDerivativeState(float yaw_ref_rad)
{
    // 在模式切换、贴目标和大步跳目标时同步清空参考导数状态，目的是参考角本身可能被直接重置，若继续沿用旧导数会制造假的惯量和摩擦前馈。
    last_yaw_ref_rad = yaw_ref_rad;
    yaw_ref_rate_filtered = 0.0f;
    yaw_ref_acc_filtered = 0.0f;
}

/**
 * @brief 获取云台前馈计算周期
 *
 * @return float 当前周期，单位 s
 */
static float GetGimbalFeedforwardDt(void)
{
    static uint32_t gimbal_ff_dwt_cnt = 0u;
    float dt_s = DWT_GetDeltaT(&gimbal_ff_dwt_cnt);

    // 对前馈使用的 dt 做异常钳制，目的是首拍和调度抖动可能给出异常大周期，直接参与滤波会把速率耦合项污染掉。
    if (dt_s <= 0.0f || dt_s > 0.05f) {
        dt_s = GIMBAL_FEEDFORWARD_DT_FALLBACK;
    }
    return dt_s;
}

/**
 * @brief 一阶低通更新
 *
 * @param last_output 上一拍输出
 * @param input 当前输入
 * @param rc 时间常数
 * @param dt_s 当前周期
 * @return float 本拍输出
 */
static float FirstOrderLowPass(float last_output, float input, float rc, float dt_s)
{
    if (rc <= 0.0f || dt_s <= 0.0f) {
        return input;
    }

    // 使用一阶低通平滑关节速率，目的是动力学前馈要尽量吃到真实运动趋势，但不能把高频噪声直接变成扭矩抖动。
    return input * dt_s / (rc + dt_s) + last_output * rc / (rc + dt_s);
}

/**
 * @brief 对称限幅
 *
 * @param value 原始值
 * @param limit 对称上限
 * @return float 限幅后的结果
 */
static float ClampSymmetric(float value, float limit)
{
    // 统一做正负对称限幅，目的是前馈是辅助项，不允许在任一方向上越过预设安全边界。
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

/**
 * @brief 更新云台 Pitch/Yaw 电流前馈
 *
 * @param gimbal_mode_changed 本拍是否发生模式切换
 * @param yaw_motor_online yaw 电机是否在线
 * @param yaw_motor_online_changed yaw 电机在线状态是否发生边沿变化
 * @param pitch_motor_online pitch 电机是否在线
 * @param pitch_motor_online_changed pitch 电机在线状态是否发生边沿变化
 */
void UpdateGimbalCurrentFeedforward(uint8_t gimbal_mode_changed,
                                    uint8_t yaw_motor_online,
                                    uint8_t yaw_motor_online_changed,
                                    uint8_t pitch_motor_online,
                                    uint8_t pitch_motor_online_changed)
{
    if (gimbal_cmd_recv.gimbal_mode != GIMBAL_ZERO_FORCE) {
        float ff_dt_s = GetGimbalFeedforwardDt();
        float yaw_ref_rad = gimbal_cmd_recv.yaw * DEGREE_2_RAD;
        float pitch_rad = (gimba_IMU_data != NULL ? gimba_IMU_data->Pitch : 0.0f) * DEGREE_2_RAD;
        float pitch_rate_rel = (gimba_IMU_data != NULL ? gimba_IMU_data->Gyro[1] : 0.0f);
        float yaw_rate_rel = (gimba_IMU_data != NULL ? gimba_IMU_data->Gyro[2] : 0.0f);
        float pitch_sin = arm_sin_f32(pitch_rad);
        float pitch_cos = arm_cos_f32(pitch_rad);
        float pitch_coupling = pitch_sin * pitch_cos;
        float yaw_ref_rate_raw = 0.0f;
        float yaw_ref_acc_raw = 0.0f;
        float yaw_ref_delta_deg = fabsf((yaw_ref_rad - last_yaw_ref_rad) * RAD_2_DEGREE);
        float gravity_ff;
        float coriolis_ff;
        float yaw_inertia_ff;
        float yaw_viscous_ff;
        float yaw_static_ff;
        float yaw_inertia_total;
        float last_yaw_ref_rate_filtered = yaw_ref_rate_filtered;

        if (pitch_motor_online_changed != 0u) {
            // pitch 复活边沿同步清零关节速率滤波状态，目的是旧的速度滤波缓存跨掉线周期已经不可信，继续拿来算前馈会在恢复瞬间制造额外冲击。
            pitch_rate_filtered = 0.0f;
        }

        if (gimbal_mode_changed != 0u ||
            yaw_motor_online_changed != 0u ||
            yaw_ref_delta_deg > YAW_REF_DERIV_RESET_THRESHOLD_DEG) {
            // 在模式切换、在线状态变化和大步跳目标时重置 Yaw 参考导数，目的是这些场景下目标角通常不是连续变化，继续求导只会制造假前馈尖峰。
            ResetYawReferenceDerivativeState(yaw_ref_rad);
        } else {
            // 在非跳变工况下对 Yaw 目标角做求导，目的是不改双板协议也能估出参考速度和加速度，用来补齐 Yaw 力控的惯量与摩擦前馈。
            yaw_ref_rate_raw = (yaw_ref_rad - last_yaw_ref_rad) / ff_dt_s;
            yaw_ref_rate_filtered = FirstOrderLowPass(yaw_ref_rate_filtered,
                                                      yaw_ref_rate_raw,
                                                      YAW_REF_RATE_LPF_RC,
                                                      ff_dt_s);
            // 基于滤波后的参考速度继续求导得到参考加速度，目的是先滤后求导比直接二次差分更稳，不容易把输入量化噪声放大成扭矩尖峰。
            yaw_ref_acc_raw = (yaw_ref_rate_filtered - last_yaw_ref_rate_filtered) / ff_dt_s;
            yaw_ref_acc_filtered = FirstOrderLowPass(yaw_ref_acc_filtered,
                                                     yaw_ref_acc_raw,
                                                     YAW_REF_ACC_LPF_RC,
                                                     ff_dt_s);
            last_yaw_ref_rad = yaw_ref_rad;
        }

        // 优先用关节自身测速作为动力学前馈输入，目的是IMU 角速度更接近绝对运动，直接拿来算耦合项容易把底盘旋转也算进去。
        if (pitch_motor_online != 0u && pitch_motor != NULL) {
            pitch_rate_rel = pitch_motor->measure.velocity;
        }
        if (yaw_motor_online != 0u && yaw_motor != NULL) {
            yaw_rate_rel = yaw_motor->measure.velocity;
        }

        pitch_rate_filtered = FirstOrderLowPass(pitch_rate_filtered,
                                                pitch_rate_rel,
                                                GIMBAL_FEEDFORWARD_RATE_LPF_RC,
                                                ff_dt_s);
        yaw_rate_filtered = FirstOrderLowPass(yaw_rate_filtered,
                                              yaw_rate_rel,
                                              GIMBAL_FEEDFORWARD_RATE_LPF_RC,
                                              ff_dt_s);

        // 当前 Pitch 电流前馈只保留重力补偿，目的是先把高仰角静态持位问题收敛到最基础的重力模型上，避免再叠加小陀螺耦合项把现象搅混。
        gravity_ff = -(PITCH_GRAVITY_COEFFICIENT_K1 * pitch_cos - PITCH_GRAVITY_COEFFICIENT_K2 * pitch_sin) + PITCH_GRAVITY_OFFSET;
        // 增加 Yaw 科氏耦合项补偿，目的是Yaw 与 Pitch 复合快速运动时会出现转速突变和卡顿，需要给 Yaw 一层动态前馈卸掉误差环压力。
        coriolis_ff = -YAW_CORIOLIS_FEEDFORWARD_K * yaw_rate_filtered * pitch_rate_filtered * pitch_coupling;
        // 根据当前 Pitch 姿态修正 Yaw 有效惯量，目的是枪管姿态变化会改变 Yaw 负载分布，只用常数惯量会让不同俯仰角下的补偿不一致。
        yaw_inertia_total = YAW_INERTIA_BASE + YAW_INERTIA_PITCH_COS2_GAIN * pitch_cos * pitch_cos;
        // 生成 Yaw 惯量前馈，目的是目标 Yaw 加速度变化时先补主体力矩，可以减少大动作起停时的跟随滞后。
        yaw_inertia_ff = yaw_inertia_total * yaw_ref_acc_filtered;
        // 生成 Yaw 黏性摩擦前馈，目的是匀速甩头阶段先补掉一部分速度相关阻力，让速度环不必长期靠积分硬顶。
        yaw_viscous_ff = YAW_VISCOUS_FEEDFORWARD_K * yaw_ref_rate_filtered;
        // 生成 Yaw 静摩擦前馈，目的是低速起转最容易被静摩擦拖住，给一点定值补偿可以改善“发涩”的第一下。
        if (fabsf(yaw_ref_rate_filtered) > YAW_STATIC_FEEDFORWARD_DEADBAND_RAD_S) {
            yaw_static_ff = YAW_STATIC_FEEDFORWARD_K * GetFloatSign(yaw_ref_rate_filtered);
        } else {
            yaw_static_ff = 0.0f;
        }

        // Pitch 前馈输出现在只由重力项构成，目的是让驱动端收到的补偿力矩与静态标定一一对应，后续出现高位点头时能直接围绕重力项本身排查。
        pitch_ff_storage = ClampSymmetric(gravity_ff, PITCH_FEEDFORWARD_LIMIT);
        yaw_ff_storage = ClampSymmetric(yaw_inertia_ff + yaw_viscous_ff + yaw_static_ff + coriolis_ff,
                                        YAW_FEEDFORWARD_LIMIT);

        if (pitch_motor != NULL) {
            // 每拍重绑 Pitch 前馈指针，目的是保持驱动端始终读取最新的静态存储，并在电机离线时立即撤掉前馈标志。
            pitch_motor->current_feedforward_ptr = &pitch_ff_storage;
            if (pitch_motor_online != 0u) {
                pitch_motor->motor_settings.feedforward_flag |= CURRENT_FEEDFORWARD;
            } else {
                pitch_motor->motor_settings.feedforward_flag &= ~CURRENT_FEEDFORWARD;
            }
        }
        if (yaw_motor != NULL) {
            // 为 Yaw 挂载电流前馈，目的是让复合运动时的耦合补偿直接进入最内层力矩通道，而不是继续堆给位置/速度环硬抗。
            yaw_motor->current_feedforward_ptr = &yaw_ff_storage;
            if (yaw_motor_online != 0u) {
                yaw_motor->motor_settings.feedforward_flag |= CURRENT_FEEDFORWARD;
            } else {
                yaw_motor->motor_settings.feedforward_flag &= ~CURRENT_FEEDFORWARD;
            }
        }
    } else {
        // 零力模式下同步清空前馈输出和滤波状态，目的是退出失能后若沿用旧动态项，恢复第一拍会带着过期力矩直接冲击机构。
        pitch_ff_storage = 0.0f;
        yaw_ff_storage = 0.0f;
        pitch_rate_filtered = 0.0f;
        yaw_rate_filtered = 0.0f;
        // 零力模式同步清零 Yaw 参考导数，目的是重新使能后的第一拍必须从当前目标重新起算，不能带着零力前的旧参考速度和加速度。
        ResetYawReferenceDerivativeState(gimbal_cmd_recv.yaw * DEGREE_2_RAD);
        if (pitch_motor != NULL) {
            pitch_motor->motor_settings.feedforward_flag &= ~CURRENT_FEEDFORWARD;
        }
        if (yaw_motor != NULL) {
            yaw_motor->motor_settings.feedforward_flag &= ~CURRENT_FEEDFORWARD;
        }
    }
}
