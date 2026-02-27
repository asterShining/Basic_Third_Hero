#include "gimbal_pitch_cali.h"
#include "string.h"

// 内部使用的字符串Buffer，用于Float2Str
static char log_buff_t[20];
static char log_buff_a[20];

void GimbalCali_Init(GimbalCali_Handler_t *handler)
{
    if (handler == NULL)
        return;
    memset(handler, 0, sizeof(GimbalCali_Handler_t));
    handler->state = CALI_STATE_IDLE;
}

void GimbalCali_Start(GimbalCali_Handler_t *handler)
{
    if (handler == NULL)
        return;
    if (handler->state == CALI_STATE_IDLE || handler->state == CALI_STATE_DONE) {
        handler->state = CALI_STATE_INIT;
        LOGINFO("[CALI] Calibration Started...");
    }
}

uint8_t GimbalCali_Update(GimbalCali_Handler_t *handler, DMMotorInstance *motor, float current_pitch_deg)
{
    if (handler == NULL || motor == NULL)
        return 0;

    // 如果处于空闲状态，直接返回0，不影响外部正常控制
    if (handler->state == CALI_STATE_IDLE)
        return 0;

    switch (handler->state) {
    case CALI_STATE_INIT:
        // 1. 备份电机原有配置
        memcpy(&handler->original_setting, &motor->motor_settings, sizeof(Motor_Control_Setting_s));
        handler->original_ref = motor->pid_ref;

        // 2. 强制切换为开环力矩模式 (Close Loop Type = 0)
        // 这样 DMMotorTask 中: set_torque = pid_ref，不会经过PID计算
        motor->motor_settings.outer_loop_type = 0; // 无外环
        motor->motor_settings.close_loop_type = 0; // 无闭环 (直接透传Ref到Torque)
        motor->motor_settings.feedforward_flag = 0; // 关闭前馈叠加

        DMMotorEnable(motor);

        // 3. 初始化变量
        handler->current_torque = CALI_TORQUE_START;
        handler->timer_cnt = 0;

        // 4. 打印表头 (方便Excel处理)
        LOGINFO("DATA_START: Target_Torque(Nm), Angle_Pitch(Deg), Real_Torque(Nm)");

        handler->state = CALI_STATE_RAMP;
        break;

    case CALI_STATE_RAMP:
        // 设置电机目标力矩
        DMMotorSetRef(motor, handler->current_torque);

        // 进入稳定等待
        handler->timer_cnt = 0;
        handler->state = CALI_STATE_STABILIZE;
        break;

    case CALI_STATE_STABILIZE:
        // 维持力矩，等待云台物理晃动停止
        DMMotorSetRef(motor, handler->current_torque); // 保持发送

        handler->timer_cnt += CALI_TASK_PERIOD_MS;
        if (handler->timer_cnt >= CALI_STABLE_TIME_MS) {
            // 准备进入记录阶段，清空滤波器
            memset(handler->angle_buffer, 0, sizeof(handler->angle_buffer));
            handler->timer_cnt = 0;
            handler->state = CALI_STATE_RECORD;
        }
        break;

    case CALI_STATE_RECORD:
        DMMotorSetRef(motor, handler->current_torque);

        // 采集数据并滤波 (利用 user_lib 中的 AverageFilter)
        // 注意：这里为了简单，每次都计算一次平均值，实际是取最后一次的窗口平均
        handler->avg_angle = AverageFilter(current_pitch_deg, handler->angle_buffer, 50);

        handler->timer_cnt += CALI_TASK_PERIOD_MS;
        if (handler->timer_cnt >= CALI_RECORD_TIME_MS) {
            // 记录时间结束，打印当前数据点
            // 将浮点转字符串
            Float2Str(log_buff_t, handler->current_torque);
            Float2Str(log_buff_a, handler->avg_angle);

            // 为了验证控制效果，也可以打印电机实际反馈的力矩(motor->measure.torque)
            char log_buff_real_t[20];
            Float2Str(log_buff_real_t, motor->measure.torque);

            // 格式: T, Angle, RealT
            LOGINFO("%s, %s, %s", log_buff_t, log_buff_a, log_buff_real_t);

            handler->state = CALI_STATE_NEXT;
        }
        break;

    case CALI_STATE_NEXT:
        // 增加力矩
        handler->current_torque += CALI_TORQUE_STEP;

        // 检查是否结束
        if (handler->current_torque > CALI_TORQUE_END) {
            handler->state = CALI_STATE_DONE;
        } else {
            handler->state = CALI_STATE_RAMP;
        }
        break;

    case CALI_STATE_DONE:
        // 1. 力矩归零
        DMMotorSetRef(motor, 0.0f);

        // 2. 恢复电机原有配置
        memcpy(&motor->motor_settings, &handler->original_setting, sizeof(Motor_Control_Setting_s));
        DMMotorSetRef(motor, handler->original_ref);

        LOGINFO("[CALI] Calibration Done.");
        handler->state = CALI_STATE_IDLE;
        break;

    default:
        handler->state = CALI_STATE_IDLE;
        break;
    }

    return 1; // 返回1表示正在占用控制权
}
