#include "gimbal_calibration.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "general_def.h"
// ================== 标定功能函数实现 ==================
char str_k1[16];
char str_k2[16];
#define CALI_TEST_MAX_TORQUE 3.0f
/* 全局标定实例定义在 gimbal.c 中 (在头文件声明为 extern) */

/**
 * @brief 初始化标定参数
 */
void Gimbal_Calibration_Init(void)
{
    memset(&g_cali, 0, sizeof(g_cali));

    // 计算步长：总跨度 / (点数 - 1)
    float range = 2.0f * CALI_TEST_MAX_TORQUE;
    float step = range / (float)(CALI_SAMPLE_COUNT - 1);

    for (int i = 0; i < CALI_SAMPLE_COUNT; i++) {
        // 从负最大力矩 到 正最大力矩
        g_cali.torque_seq[i] = -CALI_TEST_MAX_TORQUE + (i * step);
    }

    g_cali.state = CALI_STATE_NEXT_STEP;
}

/**
 * @brief 最小二乘法解算 (封装了 user_lib 的矩阵运算)
 * 方程: T = -K1*cos(theta) + K2*sin(theta)
 * 线性化: y = A*x1 + B*x2  (A=-K1, B=K2)
 */
static void Gimbal_Calibration_Solve(void)
{
    // 1. 定义矩阵数据缓存 (使用栈内存，避免频繁malloc)
    float32_t mat_M_data[4];
    float32_t mat_B_data[2];
    float32_t mat_Res_data[2]; // 结果 [A, B]^T
    float32_t mat_InvM_data[4];

    mat M, B, Res, InvM;

    // 2. 初始化 arm_math 矩阵实例
    arm_mat_init_f32(&M, 2, 2, mat_M_data);
    arm_mat_init_f32(&B, 2, 1, mat_B_data);
    arm_mat_init_f32(&Res, 2, 1, mat_Res_data);
    arm_mat_init_f32(&InvM, 2, 2, mat_InvM_data);

    // 3. 填充数据 (构建正规方程 Normal Equation)
    // | sum_c2  sum_sc | * | A | = | sum_Tc |
    // | sum_sc  sum_s2 |   | B |   | sum_Ts |

    mat_M_data[0] = g_cali.sum_cos2;
    mat_M_data[1] = g_cali.sum_sc;
    mat_M_data[2] = g_cali.sum_sc;
    mat_M_data[3] = g_cali.sum_sin2;

    mat_B_data[0] = g_cali.sum_Tcos;
    mat_B_data[1] = g_cali.sum_Tsin;

    // 4. 矩阵运算: Res = inv(M) * B
    if (arm_mat_inverse_f32(&M, &InvM) == ARM_MATH_SUCCESS) {
        arm_mat_mult_f32(&InvM, &B, &Res);

        // 5. 提取结果
        // 我们设定的模型是: T = A*cos + B*sin
        // 对应的物理公式是: T = -K1*cos + K2*sin
        // 所以: K1 = -A, K2 = B
        g_cali.result_k1 = -mat_Res_data[0];
        g_cali.result_k2 = mat_Res_data[1];

        g_cali.state = CALI_STATE_COMPLETE;
    } else {
        // 矩阵不可逆 (通常是因为采集的角度太集中，没有覆盖足够的范围)
        g_cali.state = CALI_STATE_ERROR;
    }
}

/**
 * @brief 标定任务处理函数 (在 GimbalTask 中循环调用)
 */
void Gimbal_Calibration_Handler(DMMotorInstance *motor, const attitude_t *imu)
{
    switch (g_cali.state) {
    case CALI_STATE_IDLE:
        // 等待指令触发，外部调用 Init 后会跳出此状态
        break;

    case CALI_STATE_NEXT_STEP:
        if (g_cali.current_idx >= CALI_SAMPLE_COUNT) {
            g_cali.state = CALI_STATE_CALCULATE;
        } else {
            // 设定开环力矩 (注意：dmmotor 需要支持纯力矩模式)
            DMMotorOuterLoop(motor, CURRENT_LOOP); // 确保这是纯电流/力矩环
            DMMotorSetRef(motor, g_cali.torque_seq[g_cali.current_idx]);
            g_cali.timer = HAL_GetTick();
            g_cali.state = CALI_STATE_WAIT_STABLE;
        }
        break;

    case CALI_STATE_WAIT_STABLE:
        // 等待超时 且 角速度足够小
        if ((HAL_GetTick() - g_cali.timer > CALI_STABLE_TIME) &&
            (fabsf(imu->Gyro[2]) < CALI_GYRO_LIMIT)) { // 假设 Gyro[2] 是 roll 轴
            g_cali.state = CALI_STATE_SAMPLE;
        }
        break;

    case CALI_STATE_SAMPLE: {
        float theta = imu->Roll * DEGREE_2_RAD; // 务必确认 IMU 对应轴
        float T = g_cali.torque_seq[g_cali.current_idx];
        float c = arm_cos_f32(theta); // 使用 arm_math 优化函数
        float s = arm_sin_f32(theta);

        // 累加矩阵元素 (在线构建正规方程，省去存储所有采样点)
        g_cali.sum_cos2 += c * c;
        g_cali.sum_sin2 += s * s;
        g_cali.sum_sc += s * c;
        g_cali.sum_Tcos += T * c;
        g_cali.sum_Tsin += T * s;

        g_cali.current_idx++;
        g_cali.state = CALI_STATE_NEXT_STEP;
    } break;

    case CALI_STATE_CALCULATE:
        DMMotorStop(motor); // 安全停机
        Gimbal_Calibration_Solve();
        break;

    case CALI_STATE_COMPLETE:
        // 定义两个临时字符串缓存，长度根据需要调整

        // 转换浮点数为字符串
        Float2Str(str_k1, g_cali.result_k1);
        Float2Str(str_k2, g_cali.result_k2);

        // 使用 %s 打印字符串
        LOGINFO("Cali Done: K1=%s, K2=%s", str_k1, str_k2);
        DMMotorStop(motor);
        break;

    case CALI_STATE_ERROR:
        DMMotorStop(motor);
        break;
    }
}