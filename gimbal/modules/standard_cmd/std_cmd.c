#include "std_cmd.h"

#define STD_CMD_KEYBOARD_CMD_MAX 6600.0f // What: 定义键盘叠加的满量程指令；Why: 对齐当前底盘摇杆缩放后的±6600控制尺度
#define STD_CMD_MOUSE_DIVISOR 660.0f // What: 统一鼠标增量归一化分母；Why: 复用现有DBUS键鼠手感，降低联调成本
#define STD_CMD_MOUSE_GAIN 10.0f // What: 统一鼠标增量增益；Why: 保持重构前后的云台视角手感一致

static float StdCmdClampChassisOverlay(float value)
{
    // What: 将叠加后的底盘指令限幅到当前控制尺度；Why: 防止双输入同向叠加后打爆底盘速度预算
    if (value > STD_CMD_KEYBOARD_CMD_MAX)
        return STD_CMD_KEYBOARD_CMD_MAX;
    if (value < -STD_CMD_KEYBOARD_CMD_MAX)
        return -STD_CMD_KEYBOARD_CMD_MAX;
    return value;
}

static void StdCmdClampGimbalPitch(Gimbal_Ctrl_Cmd_s *gimbal_cmd)
{
    // What: 对叠加后的pitch目标再次执行机械限幅；Why: 避免键鼠增量绕过上层摇杆分支里的安全角度保护
    if (gimbal_cmd->pitch > PITCH_MAX_ANGLE)
        gimbal_cmd->pitch = PITCH_MAX_ANGLE;
    else if (gimbal_cmd->pitch < PITCH_MIN_ANGLE)
        gimbal_cmd->pitch = PITCH_MIN_ANGLE;
}

static void StdCmdApplyMoveLookOverlay(const Key_t *key_state,
                                       int16_t mouse_x,
                                       int16_t mouse_y,
                                       Chassis_Ctrl_Cmd_s *chassis_cmd,
                                       Gimbal_Ctrl_Cmd_s *gimbal_cmd)
{
    // What: 统一处理WASD平移与鼠标视角叠加；Why: 让裁判键鼠和遥控器键鼠共用一套标准命令映射逻辑
    chassis_cmd->vx = StdCmdClampChassisOverlay(chassis_cmd->vx + (float)(key_state->w - key_state->s) * STD_CMD_KEYBOARD_CMD_MAX);
    chassis_cmd->vy = StdCmdClampChassisOverlay(chassis_cmd->vy + (float)(key_state->a - key_state->d) * STD_CMD_KEYBOARD_CMD_MAX);
    gimbal_cmd->yaw += (float)mouse_x / STD_CMD_MOUSE_DIVISOR * STD_CMD_MOUSE_GAIN;
    gimbal_cmd->pitch += (float)mouse_y / STD_CMD_MOUSE_DIVISOR * STD_CMD_MOUSE_GAIN;
    StdCmdClampGimbalPitch(gimbal_cmd);
}

void StdCmdApplyRefereeKeyMouseOverlay(const Referee_KeyMouse_Data_s *referee_keymouse,
                                       Robot_Status_e robot_state,
                                       Chassis_Ctrl_Cmd_s *chassis_cmd,
                                       Gimbal_Ctrl_Cmd_s *gimbal_cmd)
{
    Key_t referee_key_state = { 0 };

    // What: 校验输入指针与运行状态；Why: 让模块保持无状态且不绕过上层急停/失联控制
    if (referee_keymouse == NULL || chassis_cmd == NULL || gimbal_cmd == NULL)
        return;
    if (robot_state != ROBOT_READY || referee_keymouse->online == 0u)
        return;

    // What: 直接复用现有Key_t位域解析裁判键值；Why: 避免重复维护第二套键位枚举
    referee_key_state.keys = referee_keymouse->key_value;
    StdCmdApplyMoveLookOverlay(&referee_key_state,
                               referee_keymouse->mouse_dx,
                               referee_keymouse->mouse_dy,
                               chassis_cmd,
                               gimbal_cmd);
}

void StdCmdApplyRemoteMouseKey(const RC_ctrl_t *rc_data,
                               Chassis_Ctrl_Cmd_s *chassis_cmd,
                               Gimbal_Ctrl_Cmd_s *gimbal_cmd)
{
    // What: 将遥控器自带键鼠转换为同一套标准运动命令；Why: 为后续恢复左拨杆切换或回放旧DBUS键鼠场景保留统一入口
    if (rc_data == NULL || chassis_cmd == NULL || gimbal_cmd == NULL)
        return;

    StdCmdApplyMoveLookOverlay(&rc_data[TEMP].key[KEY_PRESS],
                               rc_data[TEMP].mouse.x,
                               rc_data[TEMP].mouse.y,
                               chassis_cmd,
                               gimbal_cmd);
}
