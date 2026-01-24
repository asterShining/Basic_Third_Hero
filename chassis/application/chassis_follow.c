#include "chassis_follow.h"
#include <stdlib.h> // for malloc, free
#include <string.h> // for memset
#include <math.h> // for fabsf

/* ---------------- 初始化接口 ---------------- */
ChassisFollowInstance *ChassisFollowInit(ChassisFollow_Config_s *config)
{
    // 1. 参数检查
    if (config == NULL) {
        return NULL;
    }

    // 2. 申请内存空间
    ChassisFollowInstance *instance = (ChassisFollowInstance *)malloc(sizeof(ChassisFollowInstance));

    // 3. 内存申请失败检查
    if (instance == NULL) {
        return NULL; // 内存不足，返回空指针
    }

    // 4. 清空内存，防止野数据
    memset(instance, 0, sizeof(ChassisFollowInstance));

    // 5. 保存配置参数
    instance->config = *config;
    instance->feed_forward_gain = 0.0f; // 默认开启全额前馈
    instance->enable = 1;

    // 6. 初始化内部 位置环 PID
    PID_Init_Config_s angle_conf = {
        .Kp = config->angle_pid.kp,
        .Ki = config->angle_pid.ki,
        .Kd = config->angle_pid.kd,
        .MaxOut = config->angle_pid.max_out,
        .DeadBand = config->deadzone_angle,
        .IntegralLimit = config->angle_pid.IntegralLimit, // 经验值：位置环积分不需要太大 //100
        // 使用梯形积分、积分限幅、微分先行等优化
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .Output_LPF_RC = 0.0f, // 位置环一般不需要滤波
    };
    PIDInit(&instance->angle_pid_inst, &angle_conf);

    // 7. 初始化内部 速度环 PID
    PID_Init_Config_s speed_conf = {
        .Kp = config->speed_pid.kp,
        .Ki = config->speed_pid.ki,
        .Kd = config->speed_pid.kd,
        .MaxOut = config->speed_pid.max_out,
        .IntegralLimit = config->speed_pid.IntegralLimit, // 经验值：速度环积分可以适当大一些 //3000
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .Output_LPF_RC = 0.3f, // 速度环增加少量滤波防抖
    };
    PIDInit(&instance->speed_pid_inst, &speed_conf);

    // 8. 返回创建好的实例指针
    return instance;
}

/* ---------------- 重置接口 ---------------- */
void ChassisFollowReset(ChassisFollowInstance *instance)
{
    if (instance == NULL)
        return;

    // 清除 PID 的中间状态 (积分项、上一次误差等)
    // 假设你的 PID 库没有专门的 Reset 函数，手动清除关键字段
    instance->angle_pid_inst.Iout = 0;
    instance->angle_pid_inst.Output = 0;

    instance->speed_pid_inst.Iout = 0;
    instance->speed_pid_inst.Output = 0;
}

float ChassisFollowCalc(ChassisFollowInstance *instance, float angle_error, float gimbal_wz, float chassis_wz)
{
    // 1. 安全检查
    if (instance == NULL || instance->enable == 0) {
        return 0.0f;
    }

    // 将误差限制在 -180 到 180 度之间
    if (angle_error > 180.0f) {
        angle_error -= 360.0f;
    } else if (angle_error < -180.0f) {
        angle_error += 360.0f;
    }

    // 2. 位置环计算 (Step 1: Position Loop)
    // 目标是消除角度误差 (Set Point = 0, Feedback = Error)
    // 注意：如果 angle_error > 0 表示底盘滞后，需要正向转动去追，则传入 (0, -error) 或调整 PID 符号
    // 这里假设：error = Target - Current，PID(Target, Current) -> PID(0, -error)
    float follow_speed_ref = PIDCalculate(&instance->angle_pid_inst, 0.0f, -angle_error);

    // 3. 前馈融合 (Step 2: Feed Forward)
    // 底盘总目标速度 = 追赶速度(PID计算值) + 云台当前转速(前馈值)
    float total_speed_ref = follow_speed_ref + (gimbal_wz * instance->feed_forward_gain);

    // 4. 速度环计算 (Step 3: Speed Loop)
    // 目标速度 vs 实际底盘速度
    float output = PIDCalculate(&instance->speed_pid_inst, total_speed_ref, chassis_wz);

    return output;
}