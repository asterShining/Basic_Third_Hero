/**
 * @file sp_protocol.h
 * @brief SP视觉通信协议定义 (适配 sp_vision_25 上位机)
 *
 * 协议特性:
 * - 帧头: 'SP' (0x53 0x50)
 * - 校验: CRC16
 * - 字节序: 小端
 */

#ifndef SP_PROTOCOL_H
#define SP_PROTOCOL_H

#include <stdint.h>

/* ==================== 帧头定义 ==================== */
#define SP_FRAME_HEAD_0 0x53 // 'S'
#define SP_FRAME_HEAD_1 0x50 // 'P'

/* ==================== 帧大小定义 ==================== */
#define SP_TX_FRAME_SIZE 43 // GimbalToVision 帧长度
#define SP_RX_FRAME_SIZE 30 // VisionToGimbal 帧长度

/* ==================== 工作模式定义 (下位机 -> 上位机) ==================== */
typedef enum {
    SP_MODE_IDLE = 0, // 空闲模式
    SP_MODE_AUTO_AIM = 1, // 自动瞄准模式
    SP_MODE_SMALL_BUFF = 2, // 小符模式 (英雄不使用)
    SP_MODE_BIG_BUFF = 3 // 大符模式 (英雄不使用)
} SP_WorkMode_e;

/* ==================== 控制模式定义 (上位机 -> 下位机) ==================== */
typedef enum {
    SP_CTRL_IDLE = 0, // 不控制, 纯观测模式
    SP_CTRL_AIM = 1, // 控制云台, 不开火
    SP_CTRL_FIRE = 2 // 控制云台并开火
} SP_CtrlMode_e;

/* ==================== 下位机 -> 上位机 数据帧 ==================== */
/**
 * @brief 云台状态数据帧, 发送给上位机
 *
 * 该结构体使用 packed 属性确保内存布局与通信协议一致
 * 帧大小: 43 字节
 */
#pragma pack(push, 1)
typedef struct {
    uint8_t head[2]; // 帧头 'SP' (0x53 0x50), 用于帧同步

    uint8_t mode; // 工作模式: SP_WorkMode_e
                  // 0-空闲, 1-自瞄, 2-小符, 3-大符

    float q[4]; // 姿态四元数 (顺序: w,x,y,z 标量在前)
                // 用于上位机进行坐标变换

    float yaw; // Yaw轴当前角度 (单位: 度)
    float yaw_vel; // Yaw轴当前角速度 (单位: 度/秒)

    float pitch; // Pitch轴当前角度 (单位: 度)
    float pitch_vel; // Pitch轴当前角速度 (单位: 度/秒)

    float bullet_speed; // 弹丸速度 (单位: m/s)
                        // 英雄42mm弹丸典型值: 10-16 m/s

    uint16_t bullet_count; // 累计发弹数量, 用于单发检测

    uint16_t crc16; // CRC16 校验值
                    // 计算范围: head[0] 到 bullet_count
} SP_GimbalToVision_t;
#pragma pack(pop)

/* ==================== 上位机 -> 下位机 数据帧 ==================== */
/**
 * @brief 视觉控制指令帧, 从上位机接收
 *
 * 帧大小: 30 字节
 */
#pragma pack(push, 1)
typedef struct {
    uint8_t head[2]; // 帧头 'SP' (0x53 0x50)

    uint8_t mode; // 控制模式: SP_CtrlMode_e
                  // 0-不控制, 1-控制不开火, 2-控制并开火

    float yaw; // 目标 Yaw 角度 (单位: 度)
    float yaw_vel; // Yaw 角速度前馈 (单位: 度/秒)
    float yaw_acc; // Yaw 角加速度前馈 (单位: 度/秒²)

    float pitch; // 目标 Pitch 角度 (单位: 度)
    float pitch_vel; // Pitch 角速度前馈 (单位: 度/秒)
    float pitch_acc; // Pitch 角加速度前馈 (单位: 度/秒²)

    uint16_t crc16; // CRC16 校验值
} SP_VisionToGimbal_t;
#pragma pack(pop)

/* ==================== 协议接口函数 ==================== */

/**
 * @brief 初始化发送帧 (填充帧头)
 * @param tx_frame 待初始化的发送帧指针
 */
void SP_InitTxFrame(SP_GimbalToVision_t *tx_frame);

/**
 * @brief 更新发送帧的姿态四元数
 * @param tx_frame 发送帧指针
 * @param w 四元数标量部分
 * @param x 四元数向量x
 * @param y 四元数向量y
 * @param z 四元数向量z
 */
void SP_SetQuaternion(SP_GimbalToVision_t *tx_frame, float w, float x, float y, float z);

/**
 * @brief 更新发送帧的云台状态
 * @param tx_frame 发送帧指针
 * @param yaw      Yaw角度 (度)
 * @param yaw_vel  Yaw角速度 (度/秒)
 * @param pitch    Pitch角度 (度)
 * @param pitch_vel Pitch角速度 (度/秒)
 */
void SP_SetGimbalState(SP_GimbalToVision_t *tx_frame,
                       float yaw, float yaw_vel,
                       float pitch, float pitch_vel);

/**
 * @brief 更新发送帧的弹道信息
 * @param tx_frame     发送帧指针
 * @param bullet_speed 弹速 (m/s)
 * @param bullet_count 累计发弹数
 */
void SP_SetBulletInfo(SP_GimbalToVision_t *tx_frame,
                      float bullet_speed, uint16_t bullet_count);

/**
 * @brief 设置发送帧的工作模式
 * @param tx_frame 发送帧指针
 * @param mode     工作模式 (SP_WorkMode_e)
 */
void SP_SetMode(SP_GimbalToVision_t *tx_frame, SP_WorkMode_e mode);

/**
 * @brief 计算并填充发送帧的CRC16校验
 * @param tx_frame 发送帧指针
 */
void SP_CalcTxCRC(SP_GimbalToVision_t *tx_frame);

/**
 * @brief 解析接收帧
 * @param rx_buf   接收缓冲区
 * @param len      缓冲区长度
 * @param rx_frame 解析结果输出
 * @return 1-解析成功, 0-解析失败(帧头不匹配或CRC错误)
 */
uint8_t SP_ParseRxFrame(const uint8_t *rx_buf, uint16_t len, SP_VisionToGimbal_t *rx_frame);

/**
 * @brief 校验CRC16
 * @param data 数据指针
 * @param len  数据长度 (包含CRC16字段)
 * @return 1-校验通过, 0-校验失败
 */
uint8_t SP_CheckCRC16(const uint8_t *data, uint16_t len);

#endif // SP_PROTOCOL_H
