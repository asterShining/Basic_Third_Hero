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
 * @brief 清空单个 PID 的运行时状态
 *
 * @param pid 需要复位的 PID 实例
 */
void ResetPIDRuntimeState(PIDInstance *pid)
{
    if (pid == NULL) {
        return;
    }

    // 仅清空 PID 的误差、积分和微分历史，目的是模式切换后最怕沿用旧工况的残留积分，导致云台恢复静止时突然自己扭一下。
    pid->Measure = 0.0f;
    pid->Last_Measure = 0.0f;
    pid->Err = 0.0f;
    pid->Last_Err = 0.0f;
    pid->Last_ITerm = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->ITerm = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->Ref = 0.0f;
    pid->ERRORHandler.ERRORCount = 0u;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
    DWT_GetDeltaT(&pid->DWT_CNT);
}

/**
 * @brief 清空 yaw 电机角度环和速度环的运行时状态
 *
 */
void ResetYawMotorRuntimeState(void)
{
    // 在 yaw 电机掉线/复活边沿统一清空角度环和速度环历史，目的是电机失能期间反馈与目标会脱钩，残留状态会在重新上电时把头突然拉偏。
    if (yaw_motor == NULL) {
        return;
    }

    ResetPIDRuntimeState(&yaw_motor->angle_PID);
    ResetPIDRuntimeState(&yaw_motor->speed_PID);
}

/**
 * @brief 清空 pitch 电机角度环和速度环的运行时状态
 *
 */
void ResetPitchMotorRuntimeState(void)
{
    // 在 pitch 电机掉线/复活边沿和强制回零请求时统一清空角度环与速度环历史，目的是pitch 死亡前残留的误差和积分若继续带到恢复后，会把枪口重新拽回旧角度。
    if (pitch_motor == NULL) {
        return;
    }

    ResetPIDRuntimeState(&pitch_motor->angle_PID);
    ResetPIDRuntimeState(&pitch_motor->speed_PID);
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
