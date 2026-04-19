#ifndef GIMBAL_PITCH_CALI_H
#define GIMBAL_PITCH_CALI_H

#include "dmmotor.h"
#include "user_lib.h"
#include "bsp_log.h"

/* ================== 用户配置宏 ================== */
// 标定力矩范围 (单位: Nm)，这里按现场确认改成从负力矩扫到正力矩，目的是一次采完整段 pitch 载荷曲线，避免旧逻辑只看正向力矩导致拟合区间不完整。
#define CALI_TORQUE_START (-3.2f)
// 这里保留当前现场要求的正向终点 12.4Nm，目的是离线拟合脚本和实车采样必须严格使用同一段力矩区间，避免两边数据不可比。
#define CALI_TORQUE_END 14.1f
// 这里把力矩步长改成 0.1Nm，目的是对齐这次人工指定的采样密度，避免继续沿用旧步长导致样本数量和停留时间都超出预期。
#define CALI_TORQUE_STEP 0.1f

// 采样时间配置 (单位: ms)
// 每次增加力矩后等待稳定的时间
#define CALI_STABLE_TIME_MS 1500
// 稳定后采集数据的窗口时间
#define CALI_RECORD_TIME_MS 500
// 定义标定状态机默认周期，目的是当前 GimbalTask 标称运行在 5ms，继续沿用旧的 2ms 假设会让稳定等待时间整体失真。
#define CALI_TASK_PERIOD_MS 5

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
    uint8_t config_backed_up; // 标记是否已经完成过原配置备份，目的是异常中止时只能在备份有效后才能恢复，避免把未初始化内存写回电机配置。
    uint8_t export_started; // 标记当前这轮导出协议是否已经发出 begin，目的是中止或完成时只能结束真正开始过的导出会话，避免脚本误收孤立尾标记。
    uint16_t sample_count; // 记录当前这轮已经导出的有效数据行数，目的是导出结束和异常中止时都要给上位机一个可核对的行数统计。

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
 * @brief 强制中止一次自动标定并恢复电机原配置
 * @param handler 句柄指针
 * @param motor Pitch电机实例
 */
void GimbalCali_Abort(GimbalCali_Handler_t *handler, DMMotorInstance *motor);

/**
 * @brief 标定状态机更新函数 (需在GimbalTask循环中持续调用)
 * * @param handler 句柄指针
 * @param motor   Pitch电机实例
 * @param current_pitch_deg 当前IMU的物理 Pitch 角度(单位:度)
 * @return uint8_t 1: 正在标定中(屏蔽原控制逻辑) 0: 未标定(正常运行)
 */
uint8_t GimbalCali_Update(GimbalCali_Handler_t *handler, DMMotorInstance *motor, float current_pitch_deg);

#endif // GIMBAL_PITCH_CALI_H
