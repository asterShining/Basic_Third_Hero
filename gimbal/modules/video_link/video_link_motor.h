#ifndef VIDEO_LINK_MOTOR_H
#define VIDEO_LINK_MOTOR_H

#include "stdint.h"

/**
 * @brief 初始化图传固定电机
 * Why: 统一配置M2006电机，注册CAN及PID参数
 */
void VideoLinkMotorInit(void);

/**
 * @brief 使能图传固定电机
 * Why: 退出零力模式时启用闭环控制
 */
void VideoLinkMotorEnable(void);

/**
 * @brief 失能图传固定电机
 * Why: 进入零力模式时安全切断电机输出
 */
void VideoLinkMotorDisable(void);

/**
 * @brief 图传固定电机每周期控制任务
 * Why: 根据云台的当前Pitch角度调整图传相机的目标角度点，避免由于高仰角干涉
 * 
 * @param pitch_angle_deg 当前Pitch轴角度(度)
 */
void VideoLinkMotorTask(float pitch_angle_deg);

#endif // VIDEO_LINK_MOTOR_H
