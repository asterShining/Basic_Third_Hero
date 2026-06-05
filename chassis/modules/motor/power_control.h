/**
 * @file dji_motor.h
 * @author neozng
 * @brief DJI智能电机头文件
 * @version 0.2
 * @date 2022-11-01
 *
 * @todo  1. 给不同的电机设置不同的低通滤波器惯性系数而不是统一使用宏
          2. 为M2006和M3508增加开环的零位校准函数,并在初始化时调用(根据用户配置决定是否调用)

 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"
#include "dji_motor.h"

DJIMotorInstance *PowerControlInit(Motor_Init_Config_s *config);

/**
 * @brief 电机功率控制,此时电机根据电机功率模型进行控制，不是直接的pid控制
 *
 * @param motor 电机实例指针
 * @param power 功率值
 */
void PowerControl(void);

/**
 * @brief 设置电机功率限制
 *
 * @param power_limit 功率限制值
 */
void SetPowerLimit(float power_limit);

/**
 * @brief 读取当前底盘功率控制模块采用的总功率预算
 *
 * @return float 本拍生效的底盘总功率上限
 */
float PowerControlGetPowerLimit(void);

/**
 * @brief 更新底盘姿态信息用于坡道力补偿 (需要在底盘任务中定时调用)
 * @param pitch_rad 俯仰角 (弧度)
 * @param roll_rad  横滚角 (弧度)
 */
void PowerControl_UpdateIMU(float pitch_rad, float roll_rad);

/**
 * @brief 更新底盘力控前馈使用的速度参考
 *
 * @param vx_ref 底盘坐标系前后速度参考，沿用当前底盘任务的速度指令量纲
 * @param vy_ref 底盘坐标系横移速度参考，沿用当前底盘任务的速度指令量纲
 * @param wz_ref 底盘旋转速度参考，单位为 deg/s
 * @param active 非零时根据参考变化计算前馈，零力或停机时传 0 清空前馈历史
 */
void PowerControl_UpdateForceFeedforward(float vx_ref, float vy_ref, float wz_ref, uint8_t active);

/**
 * @brief 使能或失能力控前馈
 *
 * @param enable 1:开启, 0:关闭
 */
void PowerControl_EnableForceFeedforward(uint8_t enable);

/**
 * @brief 清空力控前馈内部状态
 */
void PowerControl_ResetForceFeedforward(void);

/**
 * @brief 使能/失能 坡道力矩补偿
 * @param enable 1:开启, 0:关闭
 */
void PowerControl_EnableSlopeComp(uint8_t enable);

/**
 * @brief 获取底盘功率控制模块估算的实时功率
 *
 * @return float 当前底盘功率估计值
 */
// What: 对外暴露底盘功率估计读取接口；Why: UI 在超电离线时仍需显示一份来自本地控制器的实时功率值。
float PowerControlGetChassisPower(void);
#endif // !DJI_MOTOR_H
