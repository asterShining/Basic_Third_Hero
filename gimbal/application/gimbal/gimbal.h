#ifndef GIMBAL_H
#define GIMBAL_H

/**
 * @brief 初始化云台,会被RobotInit()调用
 *
 */
void GimbalInit();

/**
 * @brief 云台任务
 *
 */
void GimbalTask();

/**
 * @brief 触发一次 yaw 轴 DM 零点校准
 *
 * @note What: 对外暴露统一的 yaw 校零入口；Why: `robot_cmd` 需要复用同一条底层链路恢复历史零点设置逻辑，避免跨模块直接操作电机实例。
 */
void GimbalCalibrate(void);

#endif // GIMBAL_H
