#ifndef GIMBAL_PITCH_CALI_H
#define GIMBAL_PITCH_CALI_H

#include "dmmotor.h"
#include "user_lib.h"
#include "bsp_log.h"

/* ================== 用户配置宏 ================== */
// 标定力矩范围 (单位: Nm, 根据电机型号调整，DM4310峰值约7-10Nm，标定通常不需要跑满)
#define CALI_TORQUE_START 0.0f
#define CALI_TORQUE_END 5.2f // ⚠️注意：根据负载重量调整，不要设太大防止打到限位
#define CALI_TORQUE_STEP 0.05f // 力矩步长

// 采样时间配置 (单位: ms)
// 每次增加力矩后等待稳定的时间
#define CALI_STABLE_TIME_MS 1500
// 稳定后采集数据的窗口时间
#define CALI_RECORD_TIME_MS 500
// 任务循环调用周期 (假设GimbalTask是2ms一次，这里填2)
#define CALI_TASK_PERIOD_MS 2

/* ================== 结构体定义 ================== */
typedef enum {
    CALI_STATE_IDLE, // 空闲
    CALI_STATE_INIT, // 初始化：备份电机配置，切换模式
    CALI_STATE_RAMP, // 爬坡：增加力矩
    CALI_STATE_STABILIZE, // 稳定：等待物理晃动停止
    CALI_STATE_RECORD, // 记录：采集多帧数据取平均
    CALI_STATE_NEXT, // 决策：继续还是结束
    CALI_STATE_DONE, // 完成：恢复电机配置
} Cali_State_e;

typedef struct {
    // 状态机变量
    Cali_State_e state;
    uint32_t timer_cnt; // 计时器
    float current_torque; // 当前目标力矩

    // 原始配置备份 (用于标定结束后恢复)
    Motor_Control_Setting_s original_setting;
    float original_ref;

    // 采样滤波相关
    float angle_buffer[50]; // 滑动滤波缓冲区
    float avg_angle; // 计算出的平均角度

} GimbalCali_Handler_t;

/**
 * @brief 初始化标定模块
 * @param handler 句柄指针
 */
void GimbalCali_Init(GimbalCali_Handler_t *handler);

/**
 * @brief 开始一次自动标定
 * @param handler 句柄指针
 */
void GimbalCali_Start(GimbalCali_Handler_t *handler);

/**
 * @brief 标定状态机更新函数 (需在GimbalTask循环中持续调用)
 * * @param handler 句柄指针
 * @param motor   Pitch电机实例
 * @param current_pitch_deg 当前IMU的Pitch轴角度(单位:度)
 * @return uint8_t 1: 正在标定中(屏蔽原控制逻辑) 0: 未标定(正常运行)
 */
uint8_t GimbalCali_Update(GimbalCali_Handler_t *handler, DMMotorInstance *motor, float current_pitch_deg);

#endif // GIMBAL_PITCH_CALI_H
