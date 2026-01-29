#ifndef GIMBAL_CALIBRATION_H
#define GIMBAL_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "user_lib.h"
#include "arm_math.h"
#include "dmmotor.h"
#include "ins_task.h"

// 标定配置宏
#define CALI_SAMPLE_COUNT 12 /* 采样点数 (建议增加一点) */
#define CALI_STABLE_TIME 1500 /* 稳定等待时间 (ms) - 稍微加长 */
#define CALI_GYRO_LIMIT 0.02f /* 判定静止的角速度阈值 */

// 标定状态枚举
typedef enum {
    CALI_STATE_IDLE,
    CALI_STATE_NEXT_STEP,
    CALI_STATE_WAIT_STABLE,
    CALI_STATE_SAMPLE,
    CALI_STATE_CALCULATE,
    CALI_STATE_COMPLETE,
    CALI_STATE_ERROR
} CaliState_e;

/* 标定控制结构体 */
typedef struct {
    CaliState_e state;
    uint32_t timer;
    uint8_t current_idx;

    /* 预设的测试力矩序列 (根据负载调整范围) */
    float torque_seq[CALI_SAMPLE_COUNT];

    /* 采集到的数据 (A矩阵: [cos, sin], Y向量: [torque])
       最小二乘法公式: (X^T * X) * Beta = X^T * Y
       这里我们构建 simplified matrix:
       M * [K1, K2]^T = B
       其中 M 是 2x2 矩阵, B 是 2x1 向量 */
    float sum_cos2; /* M[0][0] */
    float sum_sin2; /* M[1][1] */
    float sum_sc; /* M[0][1] = M[1][0] */
    float sum_Tcos; /* B[0] */
    float sum_Tsin; /* B[1] */

    float result_k1;
    float result_k2;
} GimbalCali_t;

/* 全局实例在 gimbal_calibration.c 中定义 */
extern GimbalCali_t g_cali;

/* 初始化标定参数（在 gimbal.c 中调用以开始标定） */
void Gimbal_Calibration_Init(void);

/* 标定任务处理函数（在 GimbalTask 循环中调用） */
void Gimbal_Calibration_Handler(DMMotorInstance *motor, const attitude_t *imu);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_CALIBRATION_H */
