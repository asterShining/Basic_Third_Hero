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
    // 初始化时显式清零“已备份原配置”标记，目的是后续若标定还没真正接管电机就被安全链打断，恢复逻辑必须知道当前没有有效备份可用。
    handler->config_backed_up = 0u;
    // 初始化时同步清掉导出会话状态和样本计数，目的是 pitch 标定导出协议要求一轮标定只对应一轮文件，不能把旧会话残留带到下一轮。
    handler->export_started = 0u;
    handler->sample_count = 0u;
    handler->state = CALI_STATE_IDLE;
}

void GimbalCali_Start(GimbalCali_Handler_t *handler)
{
    if (handler == NULL)
        return;
    if (handler->state == CALI_STATE_IDLE || handler->state == CALI_STATE_DONE) {
        // 每次重新开始前先把备份有效位置回无效，目的是旧一轮标定留下的恢复快照不能被下一轮中途异常直接复用。
        handler->config_backed_up = 0u;
        // 每次重新开始前同时清掉导出开始标记和历史行数，目的是上位机脚本必须把本轮 begin/header/row/end 当成一份全新的 CSV 会话。
        handler->export_started = 0u;
        handler->sample_count = 0u;
        handler->state = CALI_STATE_INIT;
        // 这里把启动日志明确成 pitch 标定，目的是 VT03 新入口和历史 yaw 校零已经同时存在，现场看日志时必须一眼分清进入的是哪条链路。
        LOGINFO("[PITCH_CALI] Calibration started.");
    }
}

void GimbalCali_Abort(GimbalCali_Handler_t *handler, DMMotorInstance *motor)
{
    if (handler == NULL || motor == NULL)
        return;

    if (handler->state == CALI_STATE_IDLE)
        return;

    // 中止时先把输出力矩拉回零，目的是用户切回零力或链路故障后不能继续保留上一拍的开环力矩命令。
    DMMotorSetRef(motor, 0.0f);
    // 中止时恢复标定前的电机闭环配置，目的是后续重新进入正常云台控制时必须回到原来的角度/速度环语义。
    if (handler->config_backed_up != 0u) {
        // 只有确认已经备份过原配置，才能把闭环参数和参考值写回，目的是防止刚触发标定就被打断时把未初始化内存误写进电机配置。
        memcpy(&motor->motor_settings, &handler->original_setting, sizeof(Motor_Control_Setting_s));
        // 同步把参考值恢复成标定前那一拍的原始目标，目的是恢复有力后不能沿用中止前的标定力矩。
        DMMotorSetRef(motor, handler->original_ref);
    }
    if (handler->export_started != 0u) {
        // 只有导出会话真正开始过，才向 RTT 发出 abort 标记，目的是上位机脚本要据此丢弃 `.partial` 文件，但不能被孤立的中止日志误触发。
        LOGWARNING("PITCH_CALI_EXPORT_ABORT|rows=%u|status=aborted", (unsigned int)handler->sample_count);
    }
    // 清空运行期缓存并回到空闲态，目的是下一次重新进标定时必须从第一步重新备份和采样，不能带着旧窗口数据续跑。
    handler->timer_cnt = 0;
    handler->current_torque = 0.0f;
    handler->avg_angle = 0.0f;
    handler->config_backed_up = 0u;
    handler->export_started = 0u;
    handler->sample_count = 0u;
    memset(handler->angle_buffer, 0, sizeof(handler->angle_buffer));
    handler->state = CALI_STATE_IDLE;

    // 这里显式输出中止日志，目的是现场排查“为什么标定突然停了”时能直接看出是安全链打断，而不是误以为流程跑完了。
    LOGWARNING("[PITCH_CALI] Calibration aborted.");
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
        // 一旦备份完成就把标志位置 1，目的是后续无论正常结束还是异常中止，都可以安全地按这份快照恢复原控制配置。
        handler->config_backed_up = 1u;

        // 2. 强制切换为开环力矩模式 (Close Loop Type = 0)
        // 这样 DMMotorTask 中: set_torque = pid_ref，不会经过PID计算
        motor->motor_settings.outer_loop_type = 0; // 无外环
        motor->motor_settings.close_loop_type = 0; // 无闭环 (直接透传Ref到Torque)
        motor->motor_settings.feedforward_flag = 0; // 关闭前馈叠加

        DMMotorEnable(motor);

        // 3. 初始化变量
        handler->current_torque = CALI_TORQUE_START;
        handler->timer_cnt = 0;
        handler->sample_count = 0u;
        handler->export_started = 1u;

        // 4. 向 RTT 发出导出会话开始标记，目的是上位机脚本要据此创建 `.partial` 文件并把后续这一轮数据写进同一份 CSV。
        LOGINFO("PITCH_CALI_EXPORT_BEGIN|axis=pitch|format=csv|version=1");
        // 紧接着输出 CSV 表头协议行，目的是上位机不需要硬编码字段顺序，直接按固件当前导出的列定义落盘即可。
        LOGINFO("PITCH_CALI_EXPORT_HEADER|Target_Torque(Nm),Angle_Pitch(Deg),Real_Torque(Nm)");

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

            // 这里按“协议前缀 + 原始 CSV 行”输出当前样本，目的是上位机脚本既能稳定识别导出数据，又能把后半段内容原样写进 CSV 文件。
            LOGINFO("PITCH_CALI_EXPORT_ROW|%s,%s,%s", log_buff_t, log_buff_a, log_buff_real_t);
            // 每成功导出一行就递增样本计数，目的是结束或中止时要把实际发送过的行数反馈给上位机做完整性校验。
            handler->sample_count++;

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
        if (handler->config_backed_up != 0u) {
            // 只有在本轮确实完成过配置备份时才恢复原闭环，目的是让“刚启动就被打断”的异常路径和正常结束路径共享同一份安全保护。
            memcpy(&motor->motor_settings, &handler->original_setting, sizeof(Motor_Control_Setting_s));
            DMMotorSetRef(motor, handler->original_ref);
        }

        if (handler->export_started != 0u) {
            // 导出会话正常结束时显式发出 end 标记和行数，目的是上位机脚本只有看到这一行后才把 `.partial` 原子改名成正式 CSV。
            LOGINFO("PITCH_CALI_EXPORT_END|rows=%u|status=done", (unsigned int)handler->sample_count);
        }
        // 这里把完成日志同样改成 pitch 专用前缀，目的是和上面的启动/中止日志保持同一语义域，现场筛日志时不再与其它校准动作混淆。
        LOGINFO("[PITCH_CALI] Calibration done.");
        handler->config_backed_up = 0u;
        handler->export_started = 0u;
        handler->sample_count = 0u;
        handler->state = CALI_STATE_IDLE;
        break;

    default:
        handler->state = CALI_STATE_IDLE;
        break;
    }

    return 1; // 返回1表示正在占用控制权
}
