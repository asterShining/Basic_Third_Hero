#include "chassis_private.h"

// 下面这些状态只服务小陀螺变速与贴边功率控制，目的是它们只在 spin helper 内部流转，没有必要继续占用主任务文件的可读空间。
static float spin_power_base_wz = SPIN_BASE_INIT_SPEED; // 缓存小陀螺基础角速度闭环状态，目的是通过跨周期累积调节把实际功率稳定贴到上限附近。
static float spin_wave_current = 0.0f; // 缓存当前无节奏变速波形值，目的是通过连续状态平滑逼近随机目标，避免每拍直接跳变。
static float spin_wave_target = 0.0f; // 缓存当前随机变速目标，目的是让一段时间内的快慢趋势保持一致，而不是完全白噪声式乱跳。
static uint32_t spin_wave_next_refresh_tick = 0u; // 记录下次刷新随机目标的时间戳，目的是让每次目标切换间隔本身也不固定，进一步去掉节奏感。
static uint32_t spin_wave_rng_state = 0x13572468u; // 保存轻量级伪随机状态，目的是裸机/RTOS 环境下不用标准库随机数也能稳定生成无节奏变速序列。

/**
 * @brief 生成一个 -1.0 ~ 1.0 的伪随机值
 *
 * @return float 归一化的伪随机值
 */
static float GetSpinRandomSignedUnit(void)
{
    // 使用 xorshift 更新随机状态，目的是算法开销极低，适合底盘高频任务里生成“无节奏但可控”的目标。
    spin_wave_rng_state ^= spin_wave_rng_state << 13;
    spin_wave_rng_state ^= spin_wave_rng_state >> 17;
    spin_wave_rng_state ^= spin_wave_rng_state << 5;

    // 将整数随机态映射到对称区间，目的是后续同时调功率目标和角速度包络时需要一个中心在零点的有符号变量。
    return ((float)(spin_wave_rng_state & 0xFFFFu) / 32767.5f) - 1.0f;
}

/**
 * @brief 生成归一化的小陀螺无节奏变速波形
 *
 * @return float 当前时刻的波形值，范围约为 -1.0 ~ 1.0
 */
static float GetVariableSpinWave(void)
{
    // 使用系统节拍驱动随机目标刷新，目的是只在到期时切换趋势，平时保持平滑逼近，才能兼顾无节奏与可控性。
    uint32_t current_time = HAL_GetTick();

    if (spin_wave_next_refresh_tick == 0u || (int32_t)(current_time - spin_wave_next_refresh_tick) >= 0) {
        uint32_t refresh_span = SPIN_WAVE_REFRESH_MAX_MS - SPIN_WAVE_REFRESH_MIN_MS;
        float random_wave = GetSpinRandomSignedUnit();

        // 刷新下一段随机变速目标，目的是让快慢段的方向和幅度都不固定，避免被人听节奏或看轨迹读出来。
        spin_wave_target = random_wave;
        // 随机化下一次刷新间隔，目的是即使目标幅度相近，切换时刻也不固定，进一步打散周期性。
        spin_wave_next_refresh_tick = current_time + SPIN_WAVE_REFRESH_MIN_MS + (spin_wave_rng_state % (refresh_span + 1u));
    }

    // 让当前波形缓慢追向随机目标，目的是速度变化需要连续，不能因为目标随机就让输出也随机抽动。
    spin_wave_current += (spin_wave_target - spin_wave_current) * SPIN_WAVE_SMOOTH_ALPHA;
    LIMIT_MIN_MAX(spin_wave_current, -1.0f, 1.0f);
    return spin_wave_current;
}

#ifdef USE_SUPER_CAP
/**
 * @brief 计算超电能给到底盘的激进 bonus
 *
 * @param buffer_energy_j 当前裁判缓冲能量
 * @param chassis_output_allowed 当前拍是否允许底盘输出
 * @return float 额外附加功率
 */
float GetAggressiveSuperCapBonus(float buffer_energy_j, uint8_t chassis_output_allowed)
{
    float cap_percent;
    float bonus = 0.0f;

    // 统一封装超电激进功率加成决策，目的是超电阈值、错误保护和模式判断分散写在任务里很容易互相打架，抽成单函数更不容易改坏。
    if (cap == NULL || cap->is_online == 0u || chassis_output_allowed == 0u) {
        return 0.0f;
    }

    if (SuperCapGetErrorCode(cap) != 0u || SuperCapIsOutputDisabled(cap) != 0u) {
        // 超电报真实错误或输出被禁用时直接不给 bonus，目的是此时继续放大底盘功率预算只会制造“指令很猛但电源不给”的假象。
        return 0.0f;
    }

    if (chassis_cmd_recv.cap_mode != SUPER_CAP_ON &&
        fabsf(Chassis_IMU_data->Pitch) <= CHASSIS_SLOPE_THRESHOLD) {
        // 非常规爆发模式且不在坡道时不介入激进 bonus，目的是让超电加成仍受上层意图约束，避免全工况都顶着最高功率跑。
        return 0.0f;
    }

    cap_percent = SuperCapGetEnergyPercent(cap);
    if (cap_percent >= CHASSIS_SUPER_CAP_HIGH_PERCENT ||
        buffer_energy_j >= CHASSIS_SUPER_CAP_HIGH_BUFFER_J) {
        bonus = CHASSIS_SUPER_CAP_BONUS_HIGH_W;
    } else if (cap_percent >= CHASSIS_SUPER_CAP_MID_PERCENT ||
               buffer_energy_j >= CHASSIS_SUPER_CAP_MID_BUFFER_J) {
        bonus = CHASSIS_SUPER_CAP_BONUS_MID_W;
    } else if (cap_percent >= CHASSIS_SUPER_CAP_MIN_PERCENT ||
               buffer_energy_j >= CHASSIS_SUPER_CAP_MIN_BUFFER_J) {
        bonus = CHASSIS_SUPER_CAP_BONUS_LOW_W;
    }

    if (bonus > 0.0f && chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE) {
        // 小陀螺工况额外叠一档 bonus，目的是自旋时轮组功率波动最大，只靠通用 bonus 往往还不够把转速真正托起来。
        bonus += CHASSIS_SUPER_CAP_ROTATE_EXTRA_W;
    }

    return bonus;
}
#endif // USE_SUPER_CAP

/**
 * @brief 计算小陀螺的旋转目标速度
 *
 * @param base_target_wz 基础目标角速度
 * @param vx_cmd 当前前进指令
 * @param vy_cmd 当前横移指令
 * @return float 平移优先缩放后的角速度
 */
float OptimizedSpinSpeed(float base_target_wz, float vx_cmd, float vy_cmd)
{
    float trans_speed = sqrtf(vx_cmd * vx_cmd + vy_cmd * vy_cmd);
    const float MAX_TRANS_SPEED_REF = 6600.0f;
    float trans_ratio = trans_speed / MAX_TRANS_SPEED_REF;

    // 将平移需求归一化到 0.0 ~ 1.0，目的是这样才能把不同方向的推杆量统一映射成一套旋转缩放逻辑。
    if (trans_ratio > 1.0f) {
        trans_ratio = 1.0f;
    }

    // 平移越大就越多让出旋转预算，目的是用户要求激进但仍要保住横移和前后机动手感，不能让小陀螺把移动完全抢没。
    return base_target_wz * (1.0f - (trans_ratio * TRANSLATION_PRIORITY_RATIO));
}

/**
 * @brief 按实时功率余量自适应调整小陀螺基础角速度
 *
 * @param power_limit 当前底盘允许使用的总功率上限
 * @return float 已贴边调节后的基础角速度
 */
float GetAdaptiveSpinBase(float power_limit)
{
    float measured_power = PowerControlGetChassisPower();
    float target_power_ratio = SPIN_POWER_TARGET_BASE_RATIO;
    float power_error;
    float wz_delta = 0.0f;

    // 读取上一控制拍估算的底盘功率，目的是功率限幅已经在电机层闭环完成，直接复用其估计值即可形成外层吃满控制。
#if VARIABLE_SPIN_ENABLED
    float spin_wave = GetVariableSpinWave();
    // 让变速波形直接调制功率目标比例，目的是高速段会主动索取更多合法功率，确保变速逻辑不仅体现在目标值上，也体现在真实输出上。
    target_power_ratio += spin_wave * SPIN_POWER_TARGET_WAVE_AMPLITUDE;
#else
    float spin_wave = 0.0f;
#endif
    LIMIT_MIN_MAX(target_power_ratio, 0.90f, 0.99f);
    power_error = power_limit * target_power_ratio - measured_power;

    // 仅在偏离目标较明显时修正基础角速度，目的是避免已经吃满附近时还被功率噪声推着来回抽动。
    if (fabsf(power_error) > SPIN_POWER_DEADBAND_W) {
        wz_delta = power_error * SPIN_POWER_TRACK_GAIN;
        LIMIT_MIN_MAX(wz_delta, -SPIN_POWER_STEP_DOWN_MAX, SPIN_POWER_STEP_UP_MAX);
    }

    // 将功率误差积分到基础角速度状态上，目的是让小陀螺能随着余量逐步抬速，直到后级功率控制开始稳定限幅。
    spin_power_base_wz += wz_delta;
    LIMIT_MIN_MAX(spin_power_base_wz, SPIN_BASE_MIN_SPEED, SPIN_TOP_MAX_SPEED);

#if VARIABLE_SPIN_ENABLED
    {
        float variable_spin_scale = 1.0f + spin_wave * SPIN_WAVE_SCALE_AMPLITUDE;
        float variable_spin_wz = spin_power_base_wz * variable_spin_scale;

        // 对附加了明显变速包络的目标再做一次限幅，目的是即使波峰阶段主动索取更多功率，也不能让目标角速度越过软件安全边界。
        LIMIT_MIN_MAX(variable_spin_wz, SPIN_BASE_MIN_SPEED, SPIN_TOP_MAX_SPEED);
        return variable_spin_wz;
    }
#else
    return spin_power_base_wz;
#endif
}

/**
 * @brief 退出小陀螺时复位内部自适应状态
 *
 */
void ResetAdaptiveSpinBase(void)
{
    // 在退出小陀螺时恢复基础角速度状态，目的是避免上一次贴边学到的高转速在下次切入时直接带来过猛的瞬时冲击。
    spin_power_base_wz = SPIN_BASE_INIT_SPEED;
    // 在退出小陀螺时清空无节奏变速状态，目的是下次进入重新生成一段新的随机趋势，避免固定沿用上一段未走完的节奏。
    spin_wave_current = 0.0f;
    spin_wave_target = 0.0f;
    spin_wave_next_refresh_tick = 0u;
}
