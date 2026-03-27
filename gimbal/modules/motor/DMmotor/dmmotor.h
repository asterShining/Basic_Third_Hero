#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4

#define DM_P_MIN (-12.5f)
#define DM_P_MAX 12.5f
#define DM_V_MIN (-45.0f)
#define DM_V_MAX 45.0f
#define DM_T_MIN (-18.0f)
#define DM_T_MAX 18.0f

typedef enum {
    DM_ERR_NONE = 0x0,
    DM_ERR_OVER_VOLTAGE = 0x8, // 超压
    DM_ERR_UNDER_VOLTAGE = 0x9, // 欠压
    DM_ERR_OVER_CURRENT = 0xA, // 过流
    DM_ERR_MOS_OVERHEAT = 0xB // MOS过温
} DM_Motor_Error_e;
typedef struct
{
    uint8_t id;
    uint8_t state;
    DM_Motor_Error_e err_code; // [新增] 存储解析出的错误码
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;
    float T_Rotor;
    int32_t total_round;
    float total_angle;
} DM_Motor_Measure_s;


typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
} DMMotor_Send_s;
typedef struct
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;
    float pos_limit_min; // 最小机械角度 (如 -0.6)
    float pos_limit_max; // 最大机械角度 (如 0.75)
    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance *motor_daemon;
    uint32_t lost_cnt;
    uint8_t enable_cmd_cnt; // [新增] 用于保活指令降频计数
} DMMotorInstance;

typedef enum {
    DM_CMD_MOTOR_MODE = 0xfc, // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd, // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
} DMMotor_Mode_e;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);
void DMMotorControlInit();
void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);
uint8_t DMMotorIsOnline(DMMotorInstance *motor);
#endif // !DMMOTOR
