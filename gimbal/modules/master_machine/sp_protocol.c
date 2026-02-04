/**
 * @file sp_protocol.c
 * @brief SP视觉通信协议实现
 *
 * 实现帧的编码、解码和CRC校验功能
 */

#include "sp_protocol.h"
#include "crc16.h"
#include <string.h>

/* ==================== 内部函数 ==================== */

/**
 * @brief 计算CRC16校验值
 * @param data 数据指针
 * @param len  数据长度 (不包含CRC16字段)
 * @return CRC16值
 */
static uint16_t SP_GetCRC16(const uint8_t *data, uint16_t len)
{
    // 使用项目已有的crc_16函数
    return crc_16(data, len);
}

/* ==================== 发送帧操作函数 ==================== */

/**
 * @brief 初始化发送帧
 *
 * 功能: 填充帧头字段为 'SP'
 * 原因: 确保每次发送前帧头正确, 避免错误的协议识别
 */
void SP_InitTxFrame(SP_GimbalToVision_t *tx_frame)
{
    // 清空整个结构体, 避免残留数据
    memset(tx_frame, 0, sizeof(SP_GimbalToVision_t));

    // 设置帧头标识符
    tx_frame->head[0] = SP_FRAME_HEAD_0; // 'S' = 0x53
    tx_frame->head[1] = SP_FRAME_HEAD_1; // 'P' = 0x50
}

/**
 * @brief 设置姿态四元数
 *
 * 功能: 填充四元数字段, 用于上位机坐标变换
 * 原因: 视觉系统需要云台姿态实现相机到世界坐标系的转换
 */
void SP_SetQuaternion(SP_GimbalToVision_t *tx_frame, float w, float x, float y, float z)
{
    tx_frame->q[0] = w; // 标量部分
    tx_frame->q[1] = x; // 向量x
    tx_frame->q[2] = y; // 向量y
    tx_frame->q[3] = z; // 向量z
}

/**
 * @brief 设置云台状态
 *
 * 功能: 填充云台角度和角速度信息
 * 原因: 提供给上位机进行弹道解算的当前状态和速度前馈
 */
void SP_SetGimbalState(SP_GimbalToVision_t *tx_frame,
                       float yaw, float yaw_vel,
                       float pitch, float pitch_vel)
{
    tx_frame->yaw = yaw; // 当前Yaw角度 (度)
    tx_frame->yaw_vel = yaw_vel; // 当前Yaw角速度 (度/秒)
    tx_frame->pitch = pitch; // 当前Pitch角度 (度)
    tx_frame->pitch_vel = pitch_vel; // 当前Pitch角速度 (度/秒)
}

/**
 * @brief 设置弹道信息
 *
 * 功能: 填充弹速和发弹计数
 * 原因: 弹速用于弹道解算, 发弹计数用于单发检测
 */
void SP_SetBulletInfo(SP_GimbalToVision_t *tx_frame,
                      float bullet_speed, uint16_t bullet_count)
{
    tx_frame->bullet_speed = bullet_speed; // 42mm弹丸速度 (m/s)
    tx_frame->bullet_count = bullet_count; // 累计发弹数
}

/**
 * @brief 设置工作模式
 *
 * 功能: 设置当前工作模式
 * 原因: 告知上位机采用何种视觉处理算法
 */
void SP_SetMode(SP_GimbalToVision_t *tx_frame, SP_WorkMode_e mode)
{
    tx_frame->mode = (uint8_t)mode;
}

/**
 * @brief 计算并填充发送帧的CRC16校验
 *
 * 功能: 计算整帧数据的CRC16并填入帧尾
 * 原因: 确保数据传输的完整性, 上位机可据此判断帧是否损坏
 */
void SP_CalcTxCRC(SP_GimbalToVision_t *tx_frame)
{
    // 计算范围: 从head[0]到bullet_count (不包括crc16字段)
    uint16_t crc_len = sizeof(SP_GimbalToVision_t) - sizeof(tx_frame->crc16);
    tx_frame->crc16 = SP_GetCRC16((const uint8_t *)tx_frame, crc_len);
}

/* ==================== 接收帧操作函数 ==================== */

/**
 * @brief 校验CRC16
 *
 * 功能: 验证接收帧的CRC16是否正确
 * 原因: 确保接收数据的完整性, 丢弃损坏的帧
 */
uint8_t SP_CheckCRC16(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len < 3) {
        return 0; // 数据太短, 无法校验
    }

    // CRC16位于帧尾 (最后2字节, 小端序)
    uint16_t received_crc = (uint16_t)(data[len - 2]) | ((uint16_t)(data[len - 1]) << 8);

    // 计算不包含CRC字段的数据的CRC
    uint16_t calculated_crc = SP_GetCRC16(data, len - 2);

    return (received_crc == calculated_crc) ? 1 : 0;
}

/**
 * @brief 解析接收帧
 *
 * 功能: 从接收缓冲区解析上位机控制指令
 * 原因: 将原始字节流转换为结构化数据供应用层使用
 *
 * @return 1-成功, 0-失败(帧头不匹配或CRC错误)
 */
uint8_t SP_ParseRxFrame(const uint8_t *rx_buf, uint16_t len, SP_VisionToGimbal_t *rx_frame)
{
    // 检查帧长度
    if (len < SP_RX_FRAME_SIZE) {
        return 0; // 数据长度不足
    }

    // 检查帧头
    if (rx_buf[0] != SP_FRAME_HEAD_0 || rx_buf[1] != SP_FRAME_HEAD_1) {
        return 0; // 帧头不匹配
    }

    // 校验CRC16
    if (!SP_CheckCRC16(rx_buf, SP_RX_FRAME_SIZE)) {
        return 0; // CRC校验失败
    }

    // 复制数据到输出结构体
    memcpy(rx_frame, rx_buf, sizeof(SP_VisionToGimbal_t));

    return 1; // 解析成功
}
