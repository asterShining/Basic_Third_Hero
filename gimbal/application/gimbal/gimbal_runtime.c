#include "gimbal_private.h"

/**
 * @brief 将角度归一化到 [0, 360)
 *
 * @param angle_deg 原始角度
 * @return float 归一化后的角度
 */
float NormalizeAngleTo360(float angle_deg)
{
    // 统一做 yaw 单圈角度归一化，目的是电机离线/复活后的机械角输出必须始终落在同一坐标系里，避免不同调用点各写一套逻辑后出现偏差。
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

/**
 * @brief 触发一次 yaw 轴 DM 零点校准
 *
 */
void GimbalCalibrate(void)
{
    if (yaw_motor == NULL) {
        return;
    }

    // 统一通过云台模块向 yaw DM 电机发送硬件零点校准指令，目的是上层只关心“触发校零”，不应直接拿底层电机实例改零点，避免后续接口再次分叉。
    DMMotorCaliEncoder(yaw_motor);
}
