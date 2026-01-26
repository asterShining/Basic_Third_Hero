#ifndef CHASSIS_FOLLOW_H
#define CHASSIS_FOLLOW_H

#include "controller.h" // 引用你的PID库
#include "stdint.h"

// --- 配置结构体 (用户初始化时填写的参数) ---
typedef struct {
    float deadzone_angle; // 角度死区 (度)

    // 位置环 PID 参数配置
    struct {
        float kp;
        float ki;
        float kd;
        float max_out; // 位置环最大输出 (度/秒)
        float IntegralLimit; // 位置环积分限幅
    } angle_pid;

    // 速度环 PID 参数配置
    struct {
        float kp;
        float ki;
        float kd;
        float max_out; // 速度环最大输出 (电流/电压)
        float IntegralLimit; // 速度环积分限幅
    } speed_pid;
    float feed_forward_gain; // 前馈增益

    uint8_t enable;

} ChassisFollow_Config_s;

// --- 实例结构体 (对外隐藏具体实现细节，用户通常只持有这个指针) ---
typedef struct {
    // 内部集成的两个PID实例
    PIDInstance angle_pid_inst;
    PIDInstance speed_pid_inst;

    // 保存一份配置副本，方便运行时查看或在线调试
    ChassisFollow_Config_s config;

    float feed_forward_gain; // 前馈增益 (默认为1.0)
    uint8_t enable; // 模块使能标志

} ChassisFollowInstance;

/**
 * @brief 初始化跟随模块，申请内存空间
 * @param config 配置参数指针
 * @return ChassisFollowInstance* 返回分配好的实例指针，如果失败返回NULL
 */
ChassisFollowInstance *ChassisFollowInit(ChassisFollow_Config_s *config);

/**
 * @brief 计算跟随输出
 * @param instance     跟随器实例指针
 * @param angle_error  云台与底盘的角度误差 (度)
 * @param gimbal_wz    云台当前的绝对角速度 (前馈量, 度/秒)
 * @param chassis_wz   底盘当前的绝对角速度 (反馈量, 度/秒)
 * @return float       底盘电机的旋转控制量
 */
float ChassisFollowCalc(ChassisFollowInstance *instance, float angle_error, float gimbal_wz, float chassis_wz);

/**
 * @brief 重置跟随器状态 (清除PID积分等)
 * @param instance 实例指针
 */
void ChassisFollowReset(ChassisFollowInstance *instance);

#endif // CHASSIS_FOLLOW_H