#include "auto_gimbal.h"
#include "user_lib.h"
#include <math.h>

// 将弧度转换为角度
#define RAD_TO_DEG 57.295779513f

/**
 * @brief 计算最短路径的目标角度 (处理 0/360 跳变和多圈累计)
 *
 * @param current_angle 当前累计角度 (单位: 度, 可能 > 360 或 < 0)
 * @param target_angle_norm 归一化的目标角度 (单位: 度, 范围 [-180, 180] 或 [0, 360])
 * @return float 最接近当前角度的目标累计角度
 */
static float FindClosestAngle(float current_angle, float target_angle_norm)
{
    // 1. 将目标角度限制在 [-180, 180] 范围内
    // float_mod 是自定义或 math.h 的实现，这里手动实现简单的转换
    float target_norm = target_angle_norm;
    while (target_norm > 180.0f)
        target_norm -= 360.0f;
    while (target_norm <= -180.0f)
        target_norm += 360.0f;

    // 2. 计算当前角度相对于 0 位的圈数 (以 360 度为一个周期)
    // current_round_angle 是去掉圈数后的当前角度，范围接近 [-180, 180]
    // round(current / 360) 计算当前是第几圈
    float current_round = roundf(current_angle / 360.0f);

    // 3. 初步目标值 = 圈数 * 360 + 目标角度
    float target = current_round * 360.0f + target_norm;

    // 4. 二次校验：确保 diff 在 [-180, 180] 之间
    // 虽然理论上上面的计算已经很接近了，但为了保险起见
    float diff = target - current_angle;
    if (diff > 180.0f) {
        target -= 360.0f;
    } else if (diff < -180.0f) {
        target += 360.0f;
    }

    return target;
}

void AutoGimbalInit(void)
{
    // 初始化自瞄参数 (如有必要)
}

AutoAim_State_e AutoGimbalRun(Vision_Recv_s *vis_recv, float current_yaw, float current_pitch, float *cmd_yaw, float *cmd_pitch)
{
    // 0. 安全检查
    if (vis_recv == NULL || cmd_yaw == NULL || cmd_pitch == NULL) {
        return AUTO_AIM_IDLE;
    }

    // 1. 检查上位机是否发来控制指令 (control=1)
    if (vis_recv->control) {
        // --- 目标 Pitch 计算 ---
        // 上位机发来的是弧度，需转换为角度
        // 注意: 直接使用上位机的 Pitch，下位机执行端已经做了重力补偿和限位保护
        // 但这里需要映射到下位机的 Pitch 定义域 (通常是 -20 ~ 30 度)
        float target_pitch_deg = vis_recv->pitch * RAD_TO_DEG;

        // --- 目标 Yaw 计算 (最短路径处理) ---
        // 上位机发来的 Yaw 是 [-PI, PI] 的绝对角度
        float vision_yaw_deg = vis_recv->yaw * RAD_TO_DEG;

        // 计算最短路径目标值，解决 0/360 跳变和多圈问题
        float target_yaw_deg = FindClosestAngle(current_yaw, vision_yaw_deg);

        // --- 赋值输出 ---
        *cmd_yaw = target_yaw_deg;
        *cmd_pitch = target_pitch_deg;

        return AUTO_AIM_TRACKING;
    } else {
        // --- 目标丢失或未启用 ---
        // 保持手动控制逻辑产生的值 (即不修改 cmd_yaw/cmd_pitch)
        // 用户可以通过遥控器/键鼠自由控制
        return AUTO_AIM_IDLE;
    }
}
