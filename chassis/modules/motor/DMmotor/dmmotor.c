#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"
#include <math.h>

// What: 固定 DM 控制任务周期为 5ms；Why: 在当前 1kHz RTOS tick 下把主控制发送频率压到 200Hz，从而连带把反馈触发频率降到约 200Hz，降低底盘 CAN 负载
#define DM_CONTROL_TASK_PERIOD_MS 5U
// What: 固定低扭矩判定阈值为 1.0Nm；Why: 复用现有软失能保活逻辑，同时把阈值集中管理，避免后续改频率时漏改分支常量
#define DM_ENABLE_KEEPALIVE_TORQUE_THRESHOLD 1.0f
// What: 低扭矩场景下每 25ms 发送一次模式保活帧；Why: 主链路降到 200Hz 后继续优先降低总线负载，不额外把保活帧抬回 200Hz
#define DM_ENABLE_KEEPALIVE_PERIOD_MS 25U
// What: 由周期常量推导保活计数门限；Why: 保证任务周期再调整时，保活节拍会随之联动，避免再次写死循环次数
#define DM_ENABLE_KEEPALIVE_CYCLE_COUNT (DM_ENABLE_KEEPALIVE_PERIOD_MS / DM_CONTROL_TASK_PERIOD_MS)

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7); // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}
void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    // DM电机通常不需要像DJI那样检查指针越界，因为结构体是一样的
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp;
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure);

    DaemonReload(motor->motor_daemon);

    // ================= [新增] 解析 Byte 0: ID 和 ERR =================
    // 格式: MST_ID ID | ERR<<4 (即高4位为ERR，低4位为ID)
    uint8_t raw_err = (rxbuff[0] >> 4) & 0x0F;
    uint8_t feedback_id = rxbuff[0] & 0x0F;

    measure->id = feedback_id; // 更新反馈ID
    measure->err_code = (DM_Motor_Error_e)raw_err;

    // // [可选] 如果发现错误，打印日志 (依赖 bsp_log.h)
    // if (measure->err_code != DM_ERR_NONE) {
    //     LOGWARNING("[dm_motor] Error Detected! ID:%d, Code:0x%X", feedback_id, raw_err);
    // }
    // ===============================================================

    measure->last_position = measure->position;

    // 原有逻辑: Byte 1-2 位置
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    // 原有逻辑: Byte 3-4 速度 (VEL[11:4] | VEL[3:0])
    // 现有代码逻辑是正确的: (Byte3 << 4) | (Byte4 >> 4)
    tmp = (uint16_t)((rxbuff[3] << 4) | (rxbuff[4] >> 4));
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    // 原有逻辑: Byte 4-5 扭矩 (T[11:8] | T[7:0])
    // 现有代码逻辑是正确的: ((Byte4 & 0x0F) << 8) | Byte5
    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];

    // 多圈角度逻辑
    //  计算位置差值
    float diff = measure->position - measure->last_position;

    // 设定阈值，通常为量程的一半。DM电机量程跨度为 25 (12.5 - (-12.5))
    // 如果差值突变超过 12.5，说明发生了过零溢出
    if (diff < -12.5f) {
        // 从正最大值跳变到负最小值 (例如 12 -> -12)，说明正向转过了一圈
        measure->total_round++;
    } else if (diff > 12.5f) {
        // 从负最小值跳变到正最大值 (例如 -12 -> 12)，说明反向转过了一圈
        measure->total_round--;
    }

    // 计算连续的多圈角度
    // total_angle = 圈数 * 每圈弧度 + 当前单圈角度
    measure->total_angle = (float)measure->total_round * (DM_P_MAX - DM_P_MIN) + measure->position;
}

static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    uint16_t can_bus = motor->motor_can_instace->can_handle == &hcan1 ? 1 : 2;
    LOGWARNING("[dm_motor] Motor lost, can bus [%d] , id [%d]", can_bus, motor->motor_can_instace->tx_id);
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
    motor->measure.total_round = 0;
    motor->measure.total_angle = 0;
    motor->measure.position = 0; // 理论上校零后电机反馈也是0
    motor->measure.last_position = 0;
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    // Do not recalibrate on boot: this project trusts the DM driver's saved hardware zero.
    // DMMotorCaliEncoder(motor);
    DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor) // 不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

//@Todo: 目前只实现了力控，更多位控PID等请自行添加
void DMMotorTask(void const *argument)
{
    float pid_ref;
    float set_torque;
    float angle_feedback;
    float speed_feedback;
    float torque_feedback; // 对应 DJI 的 real_current
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    // DM_Motor_Measure_s *measure = &motor->measure;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    // CANInstance *motor_can = motor->motor_can_instace;
    // uint16_t tmp;
    DMMotor_Send_s motor_send_mailbox;
    while (1) {
        // ================= [新增] 自动监测与快速复位逻辑 =================
        if (motor->stop_flag == MOTOR_ENALBED) {
            // What: 在低扭矩阶段继续发送模式保活帧；Why: DM 可能在轻载时进入软失能，保活帧用于维持在线但不追求更高总线占用
            if (fabs(motor->measure.torque) < DM_ENABLE_KEEPALIVE_TORQUE_THRESHOLD) {
                motor->enable_cmd_cnt++;
                if (motor->enable_cmd_cnt >= DM_ENABLE_KEEPALIVE_CYCLE_COUNT) {
                    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
                    motor->enable_cmd_cnt = 0;
                }
            } else {
                motor->enable_cmd_cnt = 0;
            }
        }
        // ===============================================================
        // ================= 1. 反馈源选择与处理 =================
        // 角度反馈
        if (setting->angle_feedback_source == OTHER_FEED && motor->other_angle_feedback_ptr)
            angle_feedback = *motor->other_angle_feedback_ptr;
        else
            angle_feedback = motor->measure.position;

        // 速度反馈
        if (setting->speed_feedback_source == OTHER_FEED && motor->other_speed_feedback_ptr)
            speed_feedback = *motor->other_speed_feedback_ptr;
        else
            speed_feedback = motor->measure.velocity;

        // 力矩/电流反馈 (通常使用电机自带反馈)
        torque_feedback = motor->measure.torque;

        // 反馈方向处理
        if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) {
            angle_feedback *= -1;
            speed_feedback *= -1;
            torque_feedback *= -1;
        }

        // ================= 2. 串级 PID 计算 =================
        pid_ref = motor->pid_ref; // 获取最外层设定值 (如：目标角度 或 目标速度)

        // 处理电机方向反转 (注意：DJI代码是在进入PID计算前处理ref的反转)
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;

        // --- 位置环 ---
        // 启用条件：配置了位置闭环 且 最外层控制模式是位置模式
        if ((setting->close_loop_type & ANGLE_LOOP) && (setting->outer_loop_type == ANGLE_LOOP)) {
            pid_ref = PIDCalculate(&motor->angle_PID, angle_feedback, pid_ref);
            // 此时 pid_ref 变成了 目标速度
        }

        // --- 速度环 ---
        // 启用条件：配置了速度闭环 且 最外层是(位置 或 速度)模式
        if ((setting->close_loop_type & SPEED_LOOP) && (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))) {
            // 速度前馈：直接叠加到目标速度上
            if ((setting->feedforward_flag & SPEED_FEEDFORWARD) && motor->speed_feedforward_ptr)
                pid_ref += *motor->speed_feedforward_ptr;

            pid_ref = PIDCalculate(&motor->speed_PID, speed_feedback, pid_ref);
            // 此时 pid_ref 变成了 目标力矩/电流
        }

        // --- 力矩(电流)环 ---
        // 启用条件：配置了电流闭环 (只要配了就跑，通常作为最内环)
        // 这里的逻辑与 DJI 保持一致：先加前馈，再进 PID

        // 力矩前馈
        if ((setting->feedforward_flag & CURRENT_FEEDFORWARD) && motor->current_feedforward_ptr)
            pid_ref += *motor->current_feedforward_ptr;

        if (setting->close_loop_type & CURRENT_LOOP) {
            // 在 MCU 侧运行力矩 PID (输入：目标力矩 vs 实际力矩，输出：更底层的控制量，这里仍视为力矩指令)
            pid_ref = PIDCalculate(&motor->current_PID, torque_feedback, pid_ref);
        }

        // 最终输出赋值
        set_torque = pid_ref;

        // ================= 3. 输出限幅与发送 =================

        // 再次处理反转标志对最终输出的影响(如果上面在入口处处理了ref，这里就不需要了，
        // 但为了保险起见，如果这是力矩模式直接设定，可能需要反转。
        // *对比 DJI 代码*：DJI 在循环开始处 `if (reverse) pid_ref *= -1;`，之后全程传递。
        // 所以这里不需要再次反转 set_torque，除非你想实现特殊的逻辑。
        // 保持与 DJI 一致，上面入口处已处理。
        // ================= [新增] 机械限位保护 (Hard Limit) =================
        // 只有当限位值不为0时才启用保护 (防止影响其他没设置限位的电机)
        if (motor->pos_limit_max != 0.0f || motor->pos_limit_min != 0.0f) {
            float curr_pos = motor->measure.position;

            // 情况1: 超过上极限，且力矩是向上的(正) -> 掐断力矩 (允许输出负力矩拉回来)
            // 注意：这里假设 正力矩 = 向正位置运动。如果你的电机反了，这里逻辑要反。
            // DM电机通常符合右手定则：Torque > 0 -> Position 增加
            if (curr_pos > motor->pos_limit_max && set_torque > 0.0f) {
                set_torque = 0.0f;
                // 可选: 给一个微小的反向阻尼 let it dampen? 不，0最安全，让重力拉回来
            }

            // 情况2: 低于下极限，且力矩是向下的(负) -> 掐断力矩
            if (curr_pos < motor->pos_limit_min && set_torque < 0.0f) {
                set_torque = 0.0f;
            }
        }
        // ====================================

        // 限制力矩范围 (安全保护)
        LIMIT_MIN_MAX(set_torque, DM_T_MIN, DM_T_MAX);

        // MIT 模式报文构建
        // 我们使用纯力矩控制模式(MIT模式的特例)，将 P_des, V_des, Kp, Kd 设为 0
        motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
        motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
        motor_send_mailbox.Kp = 0;
        motor_send_mailbox.Kd = 0;
        motor_send_mailbox.torque_des = float_to_uint(set_torque, DM_T_MIN, DM_T_MAX, 12);

        // 停止模式处理
        if (motor->stop_flag == MOTOR_STOP) {
            motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
        }

        // 填入 CAN 发送缓冲区
        motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

        CANTransmit(motor->motor_can_instace, 4);

        // What: 控制任务按 5ms 固定节拍运行；Why: 主控制帧和由其触发的电机反馈都需要统一降到约 200Hz
        osDelay(DM_CONTROL_TASK_PERIOD_MS);
    }
}
void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++) {
        char dm_id_buff[2] = { 0 };
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}
