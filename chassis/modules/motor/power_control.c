#include "power_control.h"
#include "general_def.h"
#include "robot_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include <math.h> // 需要用到 sin, cos, tan, sqrt

#define TORQUE_COEF 0.0003662109375f // (20/16384)*(0.3), 电机转矩系数，与电流和转矩相关
#define POWER_COEF 187.0f / 3591.0f / 9.55f // 电机机械功率系数，与力矩和转速相关，注意框架中速度单位为aps
const float K1[4] = { 1.23e-07, 1.23e-07, 1.23e-07, 1.23e-07 };
const float K2[4] = { 1.453e-07, 1.453e-07, 1.453e-07, 1.453e-07 };
const float constant[4] = { 4.081f, 4.081f, 4.081f, 4.081f };

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = { NULL }; // 会在control任务中遍历该指针数组进行pid计算
static float initial_torque[4]; // 电机输出轴实际转矩，单位N·m
static float A, B, C; // 测试用
static float power_control_out[4], initial_give_power[4]; // 电机输出功率
static float chassis_max_power, chassis_power, initial_total_power = 0.0f;

// ==========================================
// [新增] 坡道补偿相关变量
// ==========================================
static float chassis_pitch = 0.0f;
static float chassis_roll = 0.0f;
static uint8_t slope_comp_enable = 0; // 默认关闭，需要在初始化或任务中开启
static float slope_feedforward_current[4] = { 0.0f }; // 存储计算出的前馈电流
static float pid_gain_scale[4] = { 1.0f, 1.0f, 1.0f, 1.0f }; // PID增益缩放系数

// 力控前馈只根据底盘任务已经生成的速度参考做微分，目的是作为速度 PID 的辅助电流而不是替代外层速度闭环。
static uint8_t force_ff_enable = 1u;
static uint8_t force_ff_ready = 0u;
static uint8_t force_ff_active = 0u;
static uint32_t force_ff_dwt_cnt = 0u;
static float force_ff_last_vx = 0.0f;
static float force_ff_last_vy = 0.0f;
static float force_ff_last_wz = 0.0f;
static float force_ff_ax = 0.0f;
static float force_ff_ay = 0.0f;
static float force_ff_alpha_z = 0.0f;
static float force_ff_current[4] = { 0.0f };

static float PowerControlLimitFloat(float value, float min_value, float max_value)
{
    // 小型限幅函数专门服务本文件内的浮点模型量，目的是避免把带赋值副作用的 LIMIT_MIN_MAX 宏嵌进复杂表达式后降低可读性。
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void PowerControl_ResetForceFeedforward(void)
{
    // 清空历史参考和滤波状态，目的是零力、禁用或重新进入控制时不把停机前的速度差分误当成一次巨大加速度。
    force_ff_ready = 0u;
    force_ff_active = 0u;
    force_ff_last_vx = 0.0f;
    force_ff_last_vy = 0.0f;
    force_ff_last_wz = 0.0f;
    force_ff_ax = 0.0f;
    force_ff_ay = 0.0f;
    force_ff_alpha_z = 0.0f;
    for (uint8_t i = 0; i < 4u; i++) {
        force_ff_current[i] = 0.0f;
    }
    DWT_GetDeltaT(&force_ff_dwt_cnt);
}

void PowerControl_EnableForceFeedforward(uint8_t enable)
{
    // 使能开关只决定是否参与输出；关闭时立即清空状态，目的是调参或故障回退时前馈不会继续残留到下一拍。
    force_ff_enable = (enable != 0u) ? 1u : 0u;
    if (force_ff_enable == 0u) {
        PowerControl_ResetForceFeedforward();
    }
}

void PowerControl_UpdateForceFeedforward(float vx_ref, float vy_ref, float wz_ref, uint8_t active)
{
    float dt_s;
    float vx_mps;
    float vy_mps;
    float wz_rps;
    float raw_ax;
    float raw_ay;
    float raw_alpha_z;
    float force_x;
    float force_y;
    float torque_z;
    float geometry_radius;
    float drive_eff;
    float wheel_force[4];

    if (force_ff_enable == 0u || active == 0u) {
        PowerControl_ResetForceFeedforward();
        return;
    }

    dt_s = DWT_GetDeltaT(&force_ff_dwt_cnt);
    if (dt_s < 0.001f || dt_s > 0.05f) {
        // DWT 首拍、任务阻塞或异常调度都会让差分失真，这里回退到标称周期，避免前馈电流被异常周期放大或压没。
        dt_s = CHASSIS_FORCE_FF_DT_FALLBACK;
    }

    vx_mps = vx_ref * CHASSIS_FORCE_FF_LINEAR_REF_TO_MPS;
    vy_mps = vy_ref * CHASSIS_FORCE_FF_LINEAR_REF_TO_MPS;
    wz_rps = wz_ref * DEGREE_2_RAD;

    if (force_ff_ready == 0u) {
        // 首次进入只建立历史点，不立即输出前馈，目的是避免上电或模式切换第一拍把已有速度参考当成从零阶跃。
        force_ff_ready = 1u;
        force_ff_active = 1u;
        force_ff_last_vx = vx_mps;
        force_ff_last_vy = vy_mps;
        force_ff_last_wz = wz_rps;
        for (uint8_t i = 0; i < 4u; i++) {
            force_ff_current[i] = 0.0f;
        }
        return;
    }

    raw_ax = (vx_mps - force_ff_last_vx) / dt_s;
    raw_ay = (vy_mps - force_ff_last_vy) / dt_s;
    raw_alpha_z = (wz_rps - force_ff_last_wz) / dt_s;

    raw_ax = PowerControlLimitFloat(raw_ax, -CHASSIS_FORCE_FF_ACCEL_LIMIT, CHASSIS_FORCE_FF_ACCEL_LIMIT);
    raw_ay = PowerControlLimitFloat(raw_ay, -CHASSIS_FORCE_FF_ACCEL_LIMIT, CHASSIS_FORCE_FF_ACCEL_LIMIT);
    raw_alpha_z = PowerControlLimitFloat(raw_alpha_z, -CHASSIS_FORCE_FF_ALPHA_LIMIT, CHASSIS_FORCE_FF_ALPHA_LIMIT);

    // 对加速度前馈做低通，目的是减少遥控和双板通信量化抖动造成的轮组电流噪声，同时仍保留阶跃指令的提前出力。
    force_ff_ax += (raw_ax - force_ff_ax) * CHASSIS_FORCE_FF_FILTER_ALPHA;
    force_ff_ay += (raw_ay - force_ff_ay) * CHASSIS_FORCE_FF_FILTER_ALPHA;
    force_ff_alpha_z += (raw_alpha_z - force_ff_alpha_z) * CHASSIS_FORCE_FF_FILTER_ALPHA;

    force_x = ROBOT_MASS * force_ff_ax;
    force_y = ROBOT_MASS * force_ff_ay;
    torque_z = ROBOT_YAW_INERTIA * force_ff_alpha_z;
    geometry_radius = HALF_WHEEL_BASE_M + HALF_TRACK_WIDTH_M;
    drive_eff = CHASSIS_DRIVETRAIN_EFF;
    if (drive_eff < 0.2f) {
        // 传动效率如果被误配到过小会把电流反向放大，这里给硬下限，保证调参错误不会直接造成离谱前馈。
        drive_eff = 0.2f;
    }

    // 四麦轮布局采用当前麦轮速度解算的同一套符号：LF/RB 与 RF/LB 在横移和旋转项上成对相反，保证前馈方向与速度 PID 输出可以同向相加。
    wheel_force[0] = 0.25f * (force_x + force_y + torque_z / geometry_radius);
    wheel_force[1] = 0.25f * (force_x - force_y - torque_z / geometry_radius);
    wheel_force[2] = 0.25f * (force_x - force_y + torque_z / geometry_radius);
    wheel_force[3] = 0.25f * (force_x + force_y - torque_z / geometry_radius);

    for (uint8_t i = 0; i < 4u; i++) {
        float wheel_torque = wheel_force[i] * RADIUS_WHEEL_M;
        float motor_current = wheel_torque * TORQUE_2_CURRENT_COEF * CHASSIS_FORCE_FF_GAIN / drive_eff;
        force_ff_current[i] = PowerControlLimitFloat(motor_current, -CHASSIS_FORCE_FF_MAX_CURRENT, CHASSIS_FORCE_FF_MAX_CURRENT);
    }

    force_ff_active = 1u;
    force_ff_last_vx = vx_mps;
    force_ff_last_vy = vy_mps;
    force_ff_last_wz = wz_rps;
}
/**
 * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can*3group)can_instance专门负责发送
 *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
 *
 * @note  因为只用于发送,所以不需要在bsp_can中注册
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
static CANInstance sender_assignment[6] = {
    [0] = { .can_handle = &hcan1, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = { 0 } },
    [1] = { .can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = { 0 } },
    [2] = { .can_handle = &hcan1, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = { 0 } },
    [3] = { .can_handle = &hcan2, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = { 0 } },
    [4] = { .can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = { 0 } },
    [5] = { .can_handle = &hcan2, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = { 0 } },
};

void PowerControl_UpdateIMU(float pitch_rad, float roll_rad)
{
    chassis_pitch = pitch_rad;
    chassis_roll = roll_rad;
}

void PowerControl_EnableSlopeComp(uint8_t enable)
{
    slope_comp_enable = enable;
}

float PowerControlGetChassisPower(void)
{
    // What: 向 UI 暴露功率控制内部维护的实时功率估计；Why: 超电离线时仍需要一个本地可信功率源显示到底盘选手端上。
    return chassis_power;
}

/**
 * @brief 计算坡道补偿前馈和大坡度 PID 载荷分配
 *
 */
static void CalculateSlopeCompensation(void)
{
    float slope_threshold_rad = CHASSIS_SLOPE_THRESHOLD * DEGREE_2_RAD;
    float uphill_angle_rad = 0.0f;
    float drive_eff = CHASSIS_DRIVETRAIN_EFF;

    // 每拍先恢复平地默认值，目的是坡道条件退出后前馈和 PID 缩放必须立即回到零/一，不能沿用上一拍坡道状态。
    for (int i = 0; i < 4; i++) {
        pid_gain_scale[i] = 1.0f;
        slope_feedforward_current[i] = 0.0f;
    }

    if (idx < 4)
        return;

    if (drive_eff < 0.2f) {
        // 传动效率被误配得过低会把理论扭矩放大成危险电流，这里给硬下限，保证参数异常时前馈仍处在可控范围。
        drive_eff = 0.2f;
    }

    if (chassis_pitch < -slope_threshold_rad) {
        float slope_force_x;
        float wheel_force_x;
        float wheel_torque;
        float motor_current;

        // 当前项目注释约定 pitch 为负表示上坡，因此 9 度只作为启用门槛，实际前馈仍按当前真实坡度代入 PPT 低配版公式 f=m*g*sin(beta_s)。
        uphill_angle_rad = -chassis_pitch;
        // PPT 低配版只补偿重力沿坡分量；这里把该牵引力当作底盘 x 正方向目标力，再平均分到四个麦轮。
        slope_force_x = ROBOT_MASS * GRAVITY_ACC * sinf(uphill_angle_rad) * CHASSIS_SLOPE_FF_GAIN;
        wheel_force_x = slope_force_x * 0.25f;
        wheel_torque = wheel_force_x * RADIUS_WHEEL_M;
        motor_current = wheel_torque * TORQUE_2_CURRENT_COEF / drive_eff;
        motor_current = PowerControlLimitFloat(motor_current, 0.0f, CHASSIS_SLOPE_FF_MAX_CURRENT);

        for (int i = 0; i < 4; i++) {
            // 低配版暂不做重力投影点权重分配，四轮使用同一份上坡牵引前馈，后续若前轮打滑再升级到 PPT 高配版。
            slope_feedforward_current[i] = motor_current;
        }
    }

    // 0. 阈值判断 (例如 25度 ~ 30度)
    // 小于此角度不进行干预，使用默认 PID
    if (fabsf(chassis_pitch) < CHASSIS_SLOPE_PID_DISTRIBUTION_THRESHOLD * DEGREE_2_RAD) {
        return;
    }

    // 1. 计算重心偏移 (无负号，方向正确)
    // Pitch负(上坡) -> tan负 -> 重心后移 -> 后轮近
    float shift_gain = 1.0f;
    float cog_shift_x = ROBOT_COG_H * tanf(chassis_pitch) * shift_gain;

    // 投影点安全限幅 (防止数值爆炸)
    float half_wb = 0.225f; // 半轴距
    float safe_margin = 0.02f;
    if (cog_shift_x < -(half_wb - safe_margin))
        cog_shift_x = -(half_wb - safe_margin);
    if (cog_shift_x > (half_wb - safe_margin))
        cog_shift_x = (half_wb - safe_margin);

    // 2. 定义轮子坐标 (保持不变)
#ifdef HALF_WHEEL_BASE_M
    const float wb = HALF_WHEEL_BASE_M;
    const float tw = HALF_TRACK_WIDTH_M;
#else
    const float wb = HALF_WHEEL_BASE / 1000.0f;
    const float tw = HALF_TRACK_WIDTH / 1000.0f;
#endif

    // 0:LF, 1:RF, 2:RB, 3:LB
    const float wheel_pos_x[4] = { wb, wb, -wb, -wb };
    const float wheel_pos_y[4] = { -tw, tw, tw, -tw };

    float weights[4];
    float total_weight = 0.0f;

    // 3. 计算权重 (平方反比，对距离非常敏感)
    for (int i = 0; i < 4; i++) {
        float dx = wheel_pos_x[i] - cog_shift_x;
        float dy = wheel_pos_y[i] - 0.0f; // 忽略侧倾

        float d_sq = dx * dx + dy * dy;
        if (d_sq < 0.0001f)
            d_sq = 0.0001f;

        weights[i] = 1.0f / d_sq;
        total_weight += weights[i];
    }

    // 4. 计算 PID 缩放系数
    for (int i = 0; i < 4; i++) {
        float eta = weights[i] / total_weight;

        // 平均分配时 eta = 0.25。 Scale = eta * 4.0
        float scale = eta * 4.0f;

        // 【强力优化】：针对大坡度的特殊处理
        // 如果是前轮 (LF/RF)，且坡度很大，强制压得更低
        // 这样前轮几乎处于“随动”状态，绝不打滑
        if (i == 0 || i == 1) {
            // 如果 scale 算出来是 0.3，我们再给它打个折，确保不空转
            scale *= 0.5f;
        }

        // 限幅：最低给 0.05 (保留一点点力维持编码器)，最高给 3.0
        if (scale < 0.05f)
            scale = 0.05f;
        if (scale > 3.0f)
            scale = 3.0f;

        pid_gain_scale[i] = scale;
    }
}

/**
 * @brief 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 */
static uint8_t sender_enable_flag[6] = { 0 };

/**
 * @brief 设置底盘的最大功率限制
 *
 * @param power_limit 限制的功率值
 */
void SetPowerLimit(float power_limit)
{
    chassis_max_power = power_limit;
}

/**
 * @brief 返回当前底盘功率控制模块正在使用的总功率预算
 *
 * @return float 当前生效的总功率上限
 */
float PowerControlGetPowerLimit(void)
{
    // 直接返回功率控制模块内部锁存的上限，目的是让超电下发链路复用同一份预算，避免两条功率链各自维护不同的目标值。
    return chassis_max_power;
}

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static void MotorSenderGrouping(DJIMotorInstance *motor, CAN_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id - 1; // 下标从零开始,先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (motor->motor_type) {
    case M2006:
    case M3508:
        if (motor_id < 4) // 根据ID分组
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 1 : 4;
        } else {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
        }

        // 计算接收id并设置分组发送id
        config->rx_id = 0x200 + motor_id + 1; // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        // 检查是否发生id冲突
        for (size_t i = 0; i < idx; ++i) {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id) {
                LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
                while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
                    LOGERROR("[dji_motor] id [%d], can_bus [%d]", config->rx_id, can_bus);
            }
        }
        break;

    case GM6020:
        if (motor_id < 4) {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
        } else {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 2 : 5;
        }

        config->rx_id = 0x204 + motor_id + 1; // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        for (size_t i = 0; i < idx; ++i) {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id) {
                LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
                while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
                    LOGERROR("[dji_motor] id [%d], can_bus [%d]", config->rx_id, can_bus);
            }
        }
        break;

    default: // other motors should not be registered here
        while (1)
            LOGERROR("[dji_motor]You must not register other motors using the API of DJI motor."); // 其他电机不应该在这里注册
    }
}

/**
 * @todo  是否可以简化多圈角度的计算？
 * @brief 根据返回的can_instance对反馈报文进行解析
 *
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
static void DecodeDJIMotor(CANInstance *_instance)
{
    // 这里对can instance的id进行了强制转换,从而获得电机的instance实例地址
    // _instance指针指向的id是对应电机instance的地址,通过强制转换为电机instance的指针,再通过->运算符访问电机的成员motor_measure,最后取地址获得指针
    uint8_t *rxbuff = _instance->rx_buff;
    DJIMotorInstance *motor = (DJIMotorInstance *)_instance->id;
    DJI_Motor_Measure_s *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销

    DaemonReload(motor->daemon);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];

    // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

static void DJIMotorLostCallback(void *motor_ptr)
{
    DJIMotorInstance *motor = (DJIMotorInstance *)motor_ptr;
    uint16_t can_bus = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
    LOGWARNING("[dji_motor] Motor lost, can bus [%d] , id [%d]", can_bus, motor->motor_can_instance->tx_id);
}

// 电机初始化,返回一个电机实例
DJIMotorInstance *PowerControlInit(Motor_Init_Config_s *config)
{
    DJIMotorInstance *instance = (DJIMotorInstance *)malloc(sizeof(DJIMotorInstance));
    memset(instance, 0, sizeof(DJIMotorInstance));

    // motor basic setting 电机基本设置
    instance->motor_type = config->motor_type; // 6020 or 2006 or 3508
    instance->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等

    // motor controller init 电机控制器初始化
    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    // 后续增加电机前馈控制器(速度和电流)

    // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
    MotorSenderGrouping(instance, &config->can_init_config);

    // 注册电机到CAN总线
    config->can_init_config.can_module_callback = DecodeDJIMotor; // set callback
    config->can_init_config.id = instance; // set id,eq to address(it is identity)
    instance->motor_can_instance = CANRegister(&config->can_init_config);

    // 注册守护线程
    Daemon_Init_Config_s daemon_config = {
        .callback = DJIMotorLostCallback,
        .owner_id = instance,
        .reload_count = 2, // 20ms未收到数据则丢失
    };
    instance->daemon = DaemonRegister(&daemon_config);

    DJIMotorEnable(instance);
    dji_motor_instance[idx++] = instance;
    return instance;
}

// 为所有电机实例计算三环PID,发送控制报文
void PowerControl()
{
    // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
    uint8_t group, num; // 电机组号和组内编号
    int16_t set; // 电机控制CAN发送设定值
    DJIMotorInstance *motor;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller; // 电机控制器
    DJI_Motor_Measure_s *measure; // 电机测量值
    float pid_measure, pid_ref; // 电机PID测量值和设定值
    initial_total_power = 0.0f;
    // 1. 计算坡道前馈和 PID 分配系数
    if (slope_comp_enable) {
        CalculateSlopeCompensation();
    } else {
        for (int i = 0; i < 4; i++) {
            // 关闭坡道补偿时同步清空前馈，目的是调参或故障回退时不会继续输出上一拍坡道电流。
            pid_gain_scale[i] = 1.0f;
            slope_feedforward_current[i] = 0.0f;
        }
    }
    // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
    for (size_t i = 0; i < idx; ++i) // idx实际上是4个
    { // 减小访存开销,先保存指针引用
        motor = dji_motor_instance[i];
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // 设置反转

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))) {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->speed_aps / 6.0f; // 电机的速度单位是度每秒,转换为rpm
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);

            // 速度 PID 仍然是主闭环，坡道载荷分配只缩放 PID 主输出，目的是先保持原有稳定性和防打滑策略。
            pid_ref *= pid_gain_scale[i];
            if (slope_feedforward_current[i] > 0.0f) {
                float slope_current_ref = slope_feedforward_current[i];
                if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
                    // 反装电机的速度参考已经在进入速度环前翻向，坡道前馈也必须同步翻向，才能保证四个轮子都在补偿底盘 x 正方向牵引力。
                    slope_current_ref *= -1.0f;
                }
                // 坡道前馈按 PPT 低配版补偿重力沿坡分量，并在功率估算前叠加，目的是让上坡提前给力但仍受后级总功率限制统一裁剪。
                pid_ref += slope_current_ref;
            }
            if (force_ff_active != 0u) {
                float force_current_ref = force_ff_current[i];
                if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
                    // 反装电机的速度参考在进入 PID 前已经翻向，前馈电流必须使用同一方向约定，否则会在右前和左后轮上抵消主 PID 输出。
                    force_current_ref *= -1.0f;
                }
                // 力控前馈在功率估算前叠加到电流指令，目的是让起步和换向提前给轮端力，同时继续交给后面的总功率模型统一裁剪。
                pid_ref += force_current_ref * pid_gain_scale[i];
            }

            initial_torque[i] = pid_ref;
            power_control_out[i] = pid_ref;
            A = K1[i] * initial_torque[i] * initial_torque[i];
            B = K2[i] * pid_measure * pid_measure;
            C = POWER_COEF * pid_measure * initial_torque[i] * TORQUE_COEF;
            initial_give_power[i] = A + B + C + constant[i];
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        // 获取最终输出
        power_control_out[i] = pid_ref;
    }
    for (uint8_t i = 0; i < idx; i++) {
        if (initial_give_power[i] < 0) {
            continue;
        }
        initial_total_power += initial_give_power[i];
    }
    if (initial_total_power > chassis_max_power) {
        float ratio = chassis_max_power / initial_total_power; // 根据允许的最大功率进行放缩
        for (uint8_t i = 0; i < idx; i++) {
            motor = dji_motor_instance[i];
            measure = &motor->measure;
            pid_measure = measure->speed_aps / 6.0f; // 电机的速度单位是度每秒,转换为rpm
            initial_give_power[i] *= ratio;
            if (initial_give_power[i] < 0) // 功率小于零，表明依靠发电减速，因此不算入功率统计
            {
                continue;
            }
            float a = K1[i];
            float b = TORQUE_COEF * POWER_COEF * pid_measure;
            float c = K2[i] * pid_measure * pid_measure - initial_give_power[i] + constant[i];
            float discriminant = b * b - 4.0f * a * c;
            if (discriminant < 0.0f) {
                // 前馈或模型误差可能让二次方程在当前功率目标下无实根，直接夹到零判别式可以避免 sqrt 产生 NaN 后污染 CAN 输出。
                discriminant = 0.0f;
            }
            if (power_control_out[i] > 0) {
                power_control_out[i] = (-b + sqrtf(discriminant)) / (2 * a);
                if (power_control_out[i] > 15000) {
                    power_control_out[i] = 15000;
                }
            } else {
                power_control_out[i] = (-b - sqrtf(discriminant)) / (2 * a);
                if (power_control_out[i] < -15000) {
                    power_control_out[i] = -15000;
                }
            }
        }
    }
    chassis_power = 0.0f; // 先清零
    for (uint8_t i = 0; i < idx; i++) {
        // initial_give_power[i] 在此时已经是乘以 ratio 之后的值了
        // 只统计正功（消耗电池能量），不统计发电（负功）
        if (initial_give_power[i] > 0) {
            chassis_power += initial_give_power[i];
        }
    }

    for (uint8_t i = 0; i < idx; i++) {
        motor = dji_motor_instance[i];
        set = (int16_t)power_control_out[i];
        group = motor->sender_group;
        num = motor->message_num;
        sender_assignment[group].tx_buff[2 * num] = (uint8_t)(set >> 8); // 低八位
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff); // 高八位

        // 若该电机处于停止状态,直接将buff置零
        if (motor->stop_flag == MOTOR_STOP)
            memset(sender_assignment[group].tx_buff + 2 * num, 0, sizeof(uint16_t));
    }

    // 遍历flag,检查是否要发送这一帧报文
    for (size_t i = 0; i < 6; ++i) {
        if (sender_enable_flag[i]) {
            CANTransmit(&sender_assignment[i], 1);
        }
    }
}
