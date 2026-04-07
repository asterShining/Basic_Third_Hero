#include "chassis_private.h"

/**
 * @brief 获取跟随控制当前拍的真实时间步长
 *
 * @return float 当前拍 dt，单位为 s
 */
float GetFollowControlDt(void)
{
    float dt_s = DWT_GetDeltaT(&follow_control_dwt_cnt);

    // 为跟随输出限斜率获取本拍真实周期，目的是RTOS 调度和中断负载会带来轻微抖动，按真实 dt 换算比写死 5ms 更稳。
    if (dt_s <= 0.0f || dt_s > 0.05f) {
        dt_s = CHASSIS_TASK_DT_FALLBACK; // DWT 异常时回退到任务标称周期，目的是防止计时首拍或异常值把斜率限幅直接放大到不可控。
    }
    return dt_s;
}

/**
 * @brief 复位跟随控制相关运行时状态
 *
 */
void ResetFollowControlState(void)
{
    // 退出跟随相关工况时统一清空接管、滤波和限斜率状态，目的是下次再进跟随必须从当前姿态重新接管，不能沿用上一次的历史输出记忆。
    follow_transition_ticks = 0u;
    follow_angle_err_filtered = 0.0f;
    follow_wz_cmd_limited = 0.0f;
    last_follow_transition_request = 0u;
    last_follow_brake_request = 0u;
    DWT_GetDeltaT(&follow_control_dwt_cnt); // 顺手重置跟随控制时间基准，目的是避免长时间不在跟随模式时下一次进入拿到异常大的 dt。
}

/**
 * @brief 启动小陀螺退跟随后接管窗口
 *
 * @param current_angle_err 当前虚拟前方偏角误差
 */
void StartFollowTransition(float current_angle_err)
{
    // 在小陀螺退跟随边沿启动底盘接管窗口，目的是先把滤波状态贴到当前偏角，再给固定刹停窗口，能避免退出首拍就被旧自旋余量拉着来回抽。
    follow_transition_ticks = FOLLOW_TRANSITION_HOLD_TICKS;
    follow_angle_err_filtered = current_angle_err;
    follow_wz_cmd_limited = 0.0f;
    DWT_GetDeltaT(&follow_control_dwt_cnt); // 接管起点重置时间基准，目的是让随后的斜率限制从稳定起点开始计算，而不是沿用模式外的旧周期。
}

/**
 * @brief 按斜率限制推进跟随输出
 *
 * @param target_wz 目标角速度
 * @param dt_s 当前拍 dt
 * @return float 限斜率后的角速度输出
 */
float ApplyFollowCommandSlew(float target_wz, float dt_s)
{
    float max_delta = FOLLOW_OUTPUT_SLEW_DPS_PER_S * dt_s;
    float delta = target_wz - follow_wz_cmd_limited;

    // 给跟随输出统一加变化率限制，目的是控制律在“纯阻尼刹停”和“正常回正”之间切段时，若直接阶跃切换很容易再次激发底盘来回摆动。
    LIMIT_MIN_MAX(delta, -max_delta, max_delta);
    follow_wz_cmd_limited += delta;
    return follow_wz_cmd_limited;
}
