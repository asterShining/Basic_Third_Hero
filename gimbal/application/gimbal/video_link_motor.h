/**
 * @file video_link_motor.h
 * @brief 图传固定电机模块头文件
 *        使用M2006电机(CAN2)实现图传镜头角度固定:
 *        位置环驱动到目标角度 → 堵转检测 → 停止发力锁定
 */

#ifndef VIDEO_LINK_MOTOR_H
#define VIDEO_LINK_MOTOR_H

/**
 * @brief 初始化图传固定电机, 注册M2006到CAN2
 *        在GimbalInit()中调用
 */
void VideoLinkMotorInit(void);

/**
 * @brief 图传固定电机状态机任务
 *        在GimbalTask()末尾周期调用
 */
void VideoLinkMotorTask(void);

/**
 * @brief 使能图传电机, 从IDLE进入MOVING状态
 *        在云台非零力模式(GYRO/FREE)时调用
 */
void VideoLinkMotorEnable(void);

/**
 * @brief 停止图传电机, 回到IDLE状态
 *        在云台零力模式(ZERO_FORCE)时调用
 */
void VideoLinkMotorDisable(void);

#endif // VIDEO_LINK_MOTOR_H
