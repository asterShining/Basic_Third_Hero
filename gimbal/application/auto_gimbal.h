#ifndef AUTO_GIMBAL_H
#define AUTO_GIMBAL_H

#include "master_process.h"
#include "robot_def.h"

// 自瞄状态枚举
typedef enum {
    AUTO_AIM_IDLE = 0, // 空闲 (未识别到目标/自瞄未启用)
    AUTO_AIM_TRACKING, // 跟踪中 (识别到目标)
    AUTO_AIM_SEARCHING, // 搜索中 (暂未使用，后续可扩展)
} AutoAim_State_e;

/**
 * @brief 自瞄初始化
 */
void AutoGimbalInit(void);

/**
 * @brief 自瞄控制逻辑运行
 *
 * @param vis_recv      视觉接收数据指针
 * @param current_yaw   当前云台 Yaw 角度 (Total Angle, deg)
 * @param current_pitch 当前云台 Pitch 角度 (deg, 注意轴向映射)
 * @param cmd_yaw       [输出] 计算后的目标 Yaw (deg)
 * @param cmd_pitch     [输出] 计算后的目标 Pitch (deg)
 * @return AutoAim_State_e 当前自瞄状态
 */
AutoAim_State_e AutoGimbalRun(Vision_Recv_s *vis_recv, float current_yaw, float current_pitch, float *cmd_yaw, float *cmd_pitch);

#endif // AUTO_GIMBAL_H
