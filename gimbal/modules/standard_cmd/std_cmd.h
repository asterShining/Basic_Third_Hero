#ifndef STD_CMD_H
#define STD_CMD_H

#include "robot_def.h"
#include "remote_control.h"

/**
 * @brief 将裁判0x0306键鼠输入叠加为标准底盘/云台控制命令
 *
 * @param referee_keymouse 裁判键鼠输入
 * @param robot_state 机器人当前运行状态
 * @param chassis_cmd 底盘标准命令
 * @param gimbal_cmd 云台标准命令
 */
void StdCmdApplyRefereeKeyMouseOverlay(const Referee_KeyMouse_Data_s *referee_keymouse,
                                       Robot_Status_e robot_state,
                                       Chassis_Ctrl_Cmd_s *chassis_cmd,
                                       Gimbal_Ctrl_Cmd_s *gimbal_cmd);

/**
 * @brief 将遥控器自带键鼠输入叠加为标准底盘/云台控制命令
 *
 * @param rc_data 遥控器完整输入
 * @param chassis_cmd 底盘标准命令
 * @param gimbal_cmd 云台标准命令
 */
void StdCmdApplyRemoteMouseKey(const RC_ctrl_t *rc_data,
                               Chassis_Ctrl_Cmd_s *chassis_cmd,
                               Gimbal_Ctrl_Cmd_s *gimbal_cmd);

#endif // STD_CMD_H
