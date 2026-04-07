#ifndef ISLAND_ACTION_H
#define ISLAND_ACTION_H

#include "robot_def.h"

#ifdef USE_ISLAND_ACTION // [条件编译] 仅在启用上岛机构时暴露接口声明，目的是禁用时头文件为空壳，避免链接未定义符号

/**
 * @brief 初始化上岛机构的电机（前履带DM电机，后抬升3508电机）
 */
void IslandActionInit(void);

/**
 * @brief 控制上岛机构动作
 * @param cmd_recv 接收到的底盘控制指令
 * @param chassis_pitch_deg 底盘IMU当前pitch角度(度)，用于自动调平闭环
 */
void IslandActionControl(const Chassis_Ctrl_Cmd_s *cmd_recv, float chassis_pitch_deg);

/**
 * @brief 紧急停止或底盘模式为ZERO_FORCE时停止辅助机构
 */
void IslandActionStop(void);

#endif // USE_ISLAND_ACTION

#endif // ISLAND_ACTION_H
