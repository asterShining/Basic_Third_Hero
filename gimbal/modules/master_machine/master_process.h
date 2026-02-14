/**
 * @file    master_process.h
 * @brief   视觉上位机 (sp_vision_25) 通信模块 — 头文件
 * @details 适配同济大学 sp_vision 上位机通信协议,支持以下通信方式:
 *          - VISION_USE_VCP:  USB 虚拟串口 (默认, 推荐)
 *          - VISION_USE_UART: 硬件串口
 *          - VISION_USE_CAN:  CAN 总线
 *          在 robot_def.h 中通过宏定义切换。
 * @version 2.0
 * @date    2026-02-15
 */

#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "sp_vision_protocol.h"

/* ======================== 缓冲区大小定义 ======================== */
/* 串口/VCP模式: 接收大小为 VisionToGimbal 结构体大小, 发送大小为 GimbalToVision 结构体大小 */
#define VISION_RECV_SIZE  sizeof(SP_VisionToGimbal_t)
#define VISION_SEND_SIZE  sizeof(SP_GimbalToVision_t)

/* ======================== 兼容性类型定义 ======================== */
/* 以下类型由 robot_def.h 中的 Shoot_Ctrl_Cmd_s / Chassis_Upload_Data_s 使用 */

typedef enum {
    COLOR_NONE = 0,
    COLOR_BLUE = 1,
    COLOR_RED  = 2,
} Enemy_Color_e;

typedef enum {
    BULLET_SPEED_NONE = 0,
    BIG_AMU_12   = 12,
    SMALL_AMU_15 = 15,
    BIG_AMU_16   = 16,
    SMALL_AMU_20 = 20,
    SMALL_AMU_30 = 30,
} Bullet_Speed_e;

/* ======================== 接收数据结构 ======================== */

/**
 * @brief 从视觉上位机接收的控制数据 (解析后)
 * @note  CAN模式和串口模式解析后的数据统一存储在此结构体中
 */
#pragma pack(1)
typedef struct
{
    /* 控制模式 (由上位机决定)
     * CAN模式:  control=data[0], shoot=data[1]
     * 串口模式: mode 字段 (0=不控制, 1=控制不开火, 2=控制且开火) */
    uint8_t control;        // 是否启用自瞄控制 (1=启用)
    uint8_t shoot;          // 是否开火 (1=开火)

    /* 视觉解算的目标角度 (弧度) */
    float yaw;              // yaw 轴目标角度或偏移 (rad)
    float pitch;            // pitch 轴目标角度或偏移 (rad)

    /* 串口模式额外数据 (CAN模式下为0) */
    float yaw_vel;          // yaw 角速度前馈 (rad/s)
    float yaw_acc;          // yaw 角加速度前馈 (rad/s²)
    float pitch_vel;        // pitch 角速度前馈 (rad/s)
    float pitch_acc;        // pitch 角加速度前馈 (rad/s²)

    /* CAN模式额外数据 */
    float horizon_distance; // 水平距离 (m), 用于弹道补偿
} Vision_Recv_s;
#pragma pack()

/* ======================== 发送数据结构 ======================== */

/**
 * @brief 发送给视觉上位机的机器人状态数据
 * @note  应用层 (gimbal_task, cmd_task) 负责填充此结构体
 */
#pragma pack(1)
typedef struct
{
    /* IMU 四元数 (wxyz 顺序, 与上位机 GimbalToVision.q 一致) */
    float q[4];             // q[0]=w, q[1]=x, q[2]=y, q[3]=z

    /* 云台角度和角速度 (串口模式需要) */
    float yaw;              // yaw 角度 (rad)
    float yaw_vel;          // yaw 角速度 (rad/s)
    float pitch;            // pitch 角度 (rad)
    float pitch_vel;        // pitch 角速度 (rad/s)

    /* 机器人状态 */
    float bullet_speed;     // 实时弹速 (m/s)
    uint16_t bullet_count;  // 累计发弹计数
    uint8_t mode;           // 当前模式 (SP_Vision_Mode_e)
    uint8_t shoot_mode;     // 射击模式 (SP_Shoot_Mode_e), CAN模式使用
} Vision_Send_s;
#pragma pack()

/* ======================== 外部接口 ======================== */

/**
 * @brief  初始化视觉通信模块
 * @param  _handle 串口句柄 (仅 VISION_USE_UART 模式使用, 其他模式传 NULL)
 * @return 接收数据结构体指针, 应用层保存此指针以读取视觉数据
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief  发送数据给视觉上位机
 * @note   应在主循环或定时任务中定期调用
 *         调用前需确保 send_data 中的数据已更新
 */
void VisionSend(void);

/**
 * @brief  设置发送数据的 IMU 四元数
 * @param  q 四元数数组 [w, x, y, z]
 */
void VisionSetQuaternion(const float *q);

/**
 * @brief  设置发送数据的云台姿态 (用于串口模式的额外数据)
 * @param  yaw       yaw 角度 (rad)
 * @param  yaw_vel   yaw 角速度 (rad/s)
 * @param  pitch     pitch 角度 (rad)
 * @param  pitch_vel pitch 角速度 (rad/s)
 */
void VisionSetAltitude(float yaw, float pitch, float yaw_vel, float pitch_vel);

/**
 * @brief  设置发送数据的机器人状态标志位
 * @param  mode         当前模式 (SP_Vision_Mode_e)
 * @param  bullet_speed 实时弹速 (m/s)
 * @param  bullet_count 累计发弹计数
 */
void VisionSetStatus(uint8_t mode, float bullet_speed, uint16_t bullet_count);

/**
 * @brief  检查视觉通信是否在线
 * @return 1=在线, 0=离线
 */
uint8_t VisionIsOnline(void);

#endif // !MASTER_PROCESS_H