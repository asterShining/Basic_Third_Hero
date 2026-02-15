/**
 * @file    sp_vision_protocol.c
 * @brief   同济大学 sp_vision 上位机通信协议实现
 * @details CRC16-CCITT 查表法实现 + 串口帧打包/解包 + CAN 辅助编解码
 *          CRC16 查表与上位机 tools/crc.cpp 完全一致 (多项式 0x8408, 初始值 0xFFFF)
 * @version 1.0
 * @date    2026-02-15
 */

#include "sp_vision_protocol.h"
#include "memory.h"
#include "bsp_log.h"

/* ======================== CRC16-CCITT 查表 ======================== */
/* 与上位机 sp_vision_25/tools/crc.cpp 中的 CRC16_TABLE 完全一致 */
static const uint16_t CRC16_CCITT_TABLE[256] = {
    0x0000,
    0x1189,
    0x2312,
    0x329b,
    0x4624,
    0x57ad,
    0x6536,
    0x74bf,
    0x8c48,
    0x9dc1,
    0xaf5a,
    0xbed3,
    0xca6c,
    0xdbe5,
    0xe97e,
    0xf8f7,
    0x1081,
    0x0108,
    0x3393,
    0x221a,
    0x56a5,
    0x472c,
    0x75b7,
    0x643e,
    0x9cc9,
    0x8d40,
    0xbfdb,
    0xae52,
    0xdaed,
    0xcb64,
    0xf9ff,
    0xe876,
    0x2102,
    0x308b,
    0x0210,
    0x1399,
    0x6726,
    0x76af,
    0x4434,
    0x55bd,
    0xad4a,
    0xbcc3,
    0x8e58,
    0x9fd1,
    0xeb6e,
    0xfae7,
    0xc87c,
    0xd9f5,
    0x3183,
    0x200a,
    0x1291,
    0x0318,
    0x77a7,
    0x662e,
    0x54b5,
    0x453c,
    0xbdcb,
    0xac42,
    0x9ed9,
    0x8f50,
    0xfbef,
    0xea66,
    0xd8fd,
    0xc974,
    0x4204,
    0x538d,
    0x6116,
    0x709f,
    0x0420,
    0x15a9,
    0x2732,
    0x36bb,
    0xce4c,
    0xdfc5,
    0xed5e,
    0xfcd7,
    0x8868,
    0x99e1,
    0xab7a,
    0xbaf3,
    0x5285,
    0x430c,
    0x7197,
    0x601e,
    0x14a1,
    0x0528,
    0x37b3,
    0x263a,
    0xdecd,
    0xcf44,
    0xfddf,
    0xec56,
    0x98e9,
    0x8960,
    0xbbfb,
    0xaa72,
    0x6306,
    0x728f,
    0x4014,
    0x519d,
    0x2522,
    0x34ab,
    0x0630,
    0x17b9,
    0xef4e,
    0xfec7,
    0xcc5c,
    0xddd5,
    0xa96a,
    0xb8e3,
    0x8a78,
    0x9bf1,
    0x7387,
    0x620e,
    0x5095,
    0x411c,
    0x35a3,
    0x242a,
    0x16b1,
    0x0738,
    0xffcf,
    0xee46,
    0xdcdd,
    0xcd54,
    0xb9eb,
    0xa862,
    0x9af9,
    0x8b70,
    0x8408,
    0x9581,
    0xa71a,
    0xb693,
    0xc22c,
    0xd3a5,
    0xe13e,
    0xf0b7,
    0x0840,
    0x19c9,
    0x2b52,
    0x3adb,
    0x4e64,
    0x5fed,
    0x6d76,
    0x7cff,
    0x9489,
    0x8500,
    0xb79b,
    0xa612,
    0xd2ad,
    0xc324,
    0xf1bf,
    0xe036,
    0x18c1,
    0x0948,
    0x3bd3,
    0x2a5a,
    0x5ee5,
    0x4f6c,
    0x7df7,
    0x6c7e,
    0xa50a,
    0xb483,
    0x8618,
    0x9791,
    0xe32e,
    0xf2a7,
    0xc03c,
    0xd1b5,
    0x2942,
    0x38cb,
    0x0a50,
    0x1bd9,
    0x6f66,
    0x7eef,
    0x4c74,
    0x5dfd,
    0xb58b,
    0xa402,
    0x9699,
    0x8710,
    0xf3af,
    0xe226,
    0xd0bd,
    0xc134,
    0x39c3,
    0x284a,
    0x1ad1,
    0x0b58,
    0x7fe7,
    0x6e6e,
    0x5cf5,
    0x4d7c,
    0xc60c,
    0xd785,
    0xe51e,
    0xf497,
    0x8028,
    0x91a1,
    0xa33a,
    0xb2b3,
    0x4a44,
    0x5bcd,
    0x6956,
    0x78df,
    0x0c60,
    0x1de9,
    0x2f72,
    0x3efb,
    0xd68d,
    0xc704,
    0xf59f,
    0xe416,
    0x90a9,
    0x8120,
    0xb3bb,
    0xa232,
    0x5ac5,
    0x4b4c,
    0x79d7,
    0x685e,
    0x1ce1,
    0x0d68,
    0x3ff3,
    0x2e7a,
    0xe70e,
    0xf687,
    0xc41c,
    0xd595,
    0xa12a,
    0xb0a3,
    0x8238,
    0x93b1,
    0x6b46,
    0x7acf,
    0x4854,
    0x59dd,
    0x2d62,
    0x3ceb,
    0x0e70,
    0x1ff9,
    0xf78f,
    0xe606,
    0xd49d,
    0xc514,
    0xb1ab,
    0xa022,
    0x92b9,
    0x8330,
    0x7bc7,
    0x6a4e,
    0x58d5,
    0x495c,
    0x3de3,
    0x2c6a,
    0x1ef1,
    0x0f78,
};

/**
 * @brief  计算 CRC16-CCITT 校验值
 * @note   算法与上位机 tools/crc.cpp 中的 get_crc16() 完全一致:
 *         初始值 0xFFFF, 每字节与 CRC 低8位异或后查表, CRC 右移8位再异或表值
 */
uint16_t SP_CRC16_CCITT(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF; // 初始值与上位机一致
    uint8_t i;

    while (len--) {
        i = (crc ^ (*data++)) & 0x00FF;
        crc = (crc >> 8) ^ CRC16_CCITT_TABLE[i];
    }

    return crc;
}

/**
 * @brief  校验含 CRC16 的完整数据帧
 * @note   CRC16 存储在数据末尾2字节, 小端序 (低字节在前, 高字节在后)
 *         与上位机 tools/crc.cpp 中的 check_crc16() 一致
 */
uint8_t SP_CRC16_Verify(const uint8_t *data, uint32_t len)
{
    if (len <= 2)
        return 0;
    // 从数据末尾提取 CRC16 (小端序: data[len-2] 是低字节, data[len-1] 是高字节)
    uint16_t crc_received = (uint16_t)(data[len - 1] << 8) | data[len - 2];
    uint16_t crc_calculated = SP_CRC16_CCITT(data, len - 2);
    return (crc_received == crc_calculated) ? 1 : 0;
}

/* ======================== 串口帧操作 ======================== */

/**
 * @brief  打包下位机→上位机的串口数据帧
 * @note   自动填充帧头 'S','P' 和 CRC16 校验
 *         CRC16 计算范围: 从 head[0] 到 bullet_count 末尾 (即整个结构体除CRC16字段外)
 */
void SP_Serial_Pack(SP_GimbalToVision_t *tx_data)
{
    // 填充帧头
    tx_data->head[0] = SP_SERIAL_HEADER_0;
    tx_data->head[1] = SP_SERIAL_HEADER_1;

    // 计算 CRC16 (不包含最后的 crc16 字段本身)
    uint32_t crc_len = sizeof(SP_GimbalToVision_t) - sizeof(tx_data->crc16);
    tx_data->crc16 = SP_CRC16_CCITT((const uint8_t *)tx_data, crc_len);
}

/**
 * @brief  解析上位机→下位机的串口数据帧
 * @note   先检查帧头 'S','P', 再校验 CRC16, 最后 memcpy 到输出结构体
 */
uint8_t SP_Serial_Unpack(const uint8_t *rx_buf, SP_VisionToGimbal_t *rx_data)
{
    // 1. 检查帧头
    if (rx_buf[0] != SP_SERIAL_HEADER_0 || rx_buf[1] != SP_SERIAL_HEADER_1) {
        // 日志: 帧头不匹配, 输出收到的实际字节
        LOGWARNING("[SP_Unpack] header fail: got 0x%02X 0x%02X, expect 'S' 'P'",
                   rx_buf[0], rx_buf[1]);
        return 0;
    }

    // 2. CRC16 校验 (整个结构体, 含末尾的CRC16字段)
    if (!SP_CRC16_Verify(rx_buf, sizeof(SP_VisionToGimbal_t))) {
        // 日志: CRC 校验失败, 输出收到的CRC和计算的CRC
        uint16_t crc_recv = (uint16_t)(rx_buf[sizeof(SP_VisionToGimbal_t) - 1] << 8) | rx_buf[sizeof(SP_VisionToGimbal_t) - 2];
        uint16_t crc_calc = SP_CRC16_CCITT(rx_buf,
                                           sizeof(SP_VisionToGimbal_t) - 2);
        LOGWARNING("[SP_Unpack] CRC fail: recv=0x%04X, calc=0x%04X, len=%d",
                   crc_recv, crc_calc, sizeof(SP_VisionToGimbal_t));
        return 0;
    }

    // 3. 拷贝数据
    memcpy(rx_data, rx_buf, sizeof(SP_VisionToGimbal_t));
    return 1;
}

/* ======================== CAN 辅助函数 ======================== */

/**
 * @brief  将 float 编码为 int16 并写入2字节缓冲区 (高字节在前)
 * @note   编码方式与上位机 io/cboard.cpp 中的 send() 函数一致:
 *         data[0] = (int16_t)(value * scale) >> 8  (高字节)
 *         data[1] = (int16_t)(value * scale)       (低字节)
 */
void SP_CAN_EncodeInt16(uint8_t *buf, float value, float scale)
{
    int16_t encoded = (int16_t)(value * scale);
    buf[0] = (uint8_t)(encoded >> 8); // 高字节
    buf[1] = (uint8_t)(encoded & 0xFF); // 低字节
}

/**
 * @brief  从2字节缓冲区解码为 float (高字节在前)
 * @note   解码方式与上位机 io/cboard.cpp 中的 callback() 函数一致:
 *         int16_t raw = (buf[0] << 8) | buf[1];
 *         float value = raw / scale;
 */
float SP_CAN_DecodeInt16(const uint8_t *buf, float scale)
{
    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    return (float)raw / scale;
}
