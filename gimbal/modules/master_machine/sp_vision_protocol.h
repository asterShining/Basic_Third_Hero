/**
 * @file    sp_vision_protocol.h
 * @brief   同济大学 sp_vision 上位机通信协议定义
 * @details 本文件定义了与 sp_vision_25 上位机完全匹配的数据结构和协议接口。
 *          支持两种通信方式:
 *          1. CAN 总线 (io/cboard): int16 缩放编码
 *          2. 串口/USB虚拟串口 (io/gimbal): SP帧头 + CRC16-CCITT 校验
 * @version 1.0
 * @date    2026-02-15
 */

#ifndef SP_VISION_PROTOCOL_H
#define SP_VISION_PROTOCOL_H

#include <stdint.h>

/* ======================== CAN 协议定义 ======================== */

/* CAN ID 配置 (与上位机 standard3.yaml 中的配置一致) */
#define SP_CAN_ID_QUATERNION 0x100 // 下位机→上位机: 发送 IMU 四元数
#define SP_CAN_ID_BULLET_SPEED 0x101 // 下位机→上位机: 发送弹速/模式/射击模式
#define SP_CAN_ID_VISION_CMD 0xFF // 上位机→下位机: 接收视觉控制命令

/* CAN 数据缩放因子 */
#define SP_CAN_QUAT_SCALE 10000.0f // 四元数/角度缩放因子 (×1e4)
#define SP_CAN_BULLET_SCALE 100.0f // 弹速缩放因子 (×1e2)
#define SP_CAN_ANGLE_SCALE 10000.0f // yaw/pitch角度缩放因子 (×1e4)

/* 上位机模式定义 (与 io/cboard.hpp 中的 Mode 枚举一致) */
typedef enum {
    SP_MODE_IDLE = 0, // 空闲/待机
    SP_MODE_AUTO_AIM = 1, // 自瞄模式
    SP_MODE_SMALL_BUFF = 2, // 小能量机关
    SP_MODE_BIG_BUFF = 3, // 大能量机关
    SP_MODE_OUTPOST = 4, // 前哨站模式
} SP_Vision_Mode_e;

/* 射击模式定义 (与 io/cboard.hpp 中的 ShootMode 枚举一致) */
typedef enum {
    SP_SHOOT_LEFT = 0, // 左射击
    SP_SHOOT_RIGHT = 1, // 右射击
    SP_SHOOT_BOTH = 2, // 双管射击
} SP_Shoot_Mode_e;

/* ======================== 串口协议定义 ======================== */

/* 串口帧头标识 (与 io/gimbal.hpp 中的 head 字段一致) */
#define SP_SERIAL_HEADER_0 'S'
#define SP_SERIAL_HEADER_1 'P'

/**
 * @brief 下位机→上位机的串口数据包 (与 io/gimbal.hpp 中的 GimbalToVision 完全一致)
 * @note  必须使用 __attribute__((packed)) 保证内存布局与上位机一致
 * @note  总长度应 <= 64 字节
 */
typedef struct __attribute__((packed)) {
    uint8_t head[2]; // 帧头: {'S', 'P'}
    uint8_t mode; // 模式: 0=空闲, 1=自瞄, 2=小符, 3=大符
    float q[4]; // IMU 四元数 (wxyz 顺序)
    float yaw; // yaw 轴角度 (rad)
    float yaw_vel; // yaw 轴角速度 (rad/s)
    float pitch; // pitch 轴角度 (rad)
    float pitch_vel; // pitch 轴角速度 (rad/s)
    float bullet_speed; // 当前弹速 (m/s)
    uint16_t bullet_count; // 累计发弹计数
    uint16_t crc16; // CRC16-CCITT 校验
} SP_GimbalToVision_t;

/**
 * @brief 上位机→下位机的串口数据包 (与 io/gimbal.hpp 中的 VisionToGimbal 完全一致)
 * @note  必须使用 __attribute__((packed)) 保证内存布局与上位机一致
 */
typedef struct __attribute__((packed)) {
    uint8_t head[2]; // 帧头: {'S', 'P'}
    uint8_t mode; // 模式: 0=不控制, 1=控制云台但不开火, 2=控制云台且开火
    float yaw; // yaw 目标角度/偏移 (rad)
    float yaw_vel; // yaw 角速度前馈 (rad/s)
    float yaw_acc; // yaw 角加速度前馈 (rad/s²)
    float pitch; // pitch 目标角度/偏移 (rad)
    float pitch_vel; // pitch 角速度前馈 (rad/s)
    float pitch_acc; // pitch 角加速度前馈 (rad/s²)
    uint16_t crc16; // CRC16-CCITT 校验
} SP_VisionToGimbal_t;

/* 编译期验证结构体大小 (与上位机 static_assert 对应) */
_Static_assert(sizeof(SP_GimbalToVision_t) <= 64, "SP_GimbalToVision_t too large");
_Static_assert(sizeof(SP_VisionToGimbal_t) <= 64, "SP_VisionToGimbal_t too large");

/* ======================== 协议函数接口 ======================== */

/**
 * @brief  计算 CRC16-CCITT 校验值 (与上位机 tools/crc.cpp 中的 get_crc16 完全一致)
 * @param  data 数据指针
 * @param  len  数据长度
 * @return CRC16 校验值
 */
uint16_t SP_CRC16_CCITT(const uint8_t *data, uint32_t len);

/**
 * @brief  校验 CRC16 (数据末尾2字节为CRC16, 小端序)
 * @param  data 含CRC16的完整数据
 * @param  len  完整数据长度 (含CRC16的2字节)
 * @return 1=校验通过, 0=校验失败
 */
uint8_t SP_CRC16_Verify(const uint8_t *data, uint32_t len);

/**
 * @brief  打包下位机→上位机的串口数据帧
 * @param  tx_data 待发送的结构体指针 (帧头和CRC16会自动填充)
 * @note   调用后 tx_data 中的 head 和 crc16 字段会被自动填充
 */
void SP_Serial_Pack(SP_GimbalToVision_t *tx_data);

/**
 * @brief  解析上位机→下位机的串口数据帧
 * @param  rx_buf   接收缓冲区
 * @param  rx_data  解析结果输出
 * @return 1=解析成功, 0=帧头或CRC校验失败
 */
uint8_t SP_Serial_Unpack(const uint8_t *rx_buf, SP_VisionToGimbal_t *rx_data);

/* ======================== CAN 辅助函数 ======================== */

/**
 * @brief  将 float 编码为 int16 并写入 CAN 数据的高低字节
 * @param  buf   CAN data 数组中的起始位置 (长度至少2字节)
 * @param  value 待编码的浮点值
 * @param  scale 缩放因子 (例如 10000.0f)
 */
void SP_CAN_EncodeInt16(uint8_t *buf, float value, float scale);

/**
 * @brief  从 CAN 数据的高低字节解码为 float
 * @param  buf   CAN data 数组中的起始位置
 * @param  scale 缩放因子
 * @return 解码后的浮点值
 */
float SP_CAN_DecodeInt16(const uint8_t *buf, float scale);

#endif // SP_VISION_PROTOCOL_H
