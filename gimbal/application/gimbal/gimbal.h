#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

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
 * @note 对外暴露统一的 yaw 校零入口，目的是`robot_cmd` 需要复用同一条底层链路恢复历史零点设置逻辑，避免跨模块直接操作电机实例。
 */
void GimbalCalibrate(void);

/**
 * @brief 触发一次 pitch 标定状态机
 *
 * @note 对外只暴露“开始标定”语义，目的是 `robot_cmd` 只负责决定何时进入标定，不应直接持有或修改云台内部标定句柄。
 */
void GimbalStartPitchCalibration(void);

/**
 * @brief 查询 pitch 标定是否正在运行
 *
 * @return uint8_t 1:标定运行中 0:标定空闲
 */
uint8_t GimbalPitchCalibrationActive(void);

#endif // GIMBAL_H
