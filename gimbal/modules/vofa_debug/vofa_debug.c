/**
 * @file vofa_debug.c
 * @brief VOFA+ JustFloat 协议调试输出模块实现
 * @note 直接使用 HAL DMA 发送，不走 BSP USARTRegister 流程，
 *       避免占用接收回调槽位，也避免在纯 TX 场景下引入不必要的 DMA 接收初始化。
 */

#include "vofa_debug.h"
#include "usart.h"
#include <string.h>

// JustFloat 协议帧尾标识，等价于 IEEE754 的 NaN (0x7F800000 小端: 00 00 80 7F)
// VOFA+ 用这 4 字节作为帧分隔符，解析器靠它来定界每一帧的结束位置
static const uint8_t VOFA_TAIL[4] = { 0x00u, 0x00u, 0x80u, 0x7Fu };

// 通道数据缓冲，每拍填充完所有通道后一次性拼帧发送
static float vofa_channels[VOFA_CHANNEL_COUNT];

// DMA 发送缓冲，大小 = 通道数 × 4字节 + 4字节帧尾
// 必须用独立缓冲而不是直接指向 vofa_channels，避免 DMA 传输过程中上层修改通道值导致数据撕裂
static uint8_t vofa_tx_buf[VOFA_CHANNEL_COUNT * sizeof(float) + sizeof(VOFA_TAIL)];

void VofaDebugInit(void)
{
    // 清零通道缓冲，确保首帧发送时不会输出随机值
    memset(vofa_channels, 0, sizeof(vofa_channels));
    memset(vofa_tx_buf, 0, sizeof(vofa_tx_buf));
}

void VofaDebugSetChannel(uint8_t ch, float val)
{
    // 防止越界写入，超出通道范围的调用直接丢弃
    if (ch < VOFA_CHANNEL_COUNT) {
        vofa_channels[ch] = val;
    }
}

void VofaDebugSend(void)
{
    // 若上一帧 DMA 尚未发完则跳过本次发送，绝不阻塞控制任务
    // 在 200Hz 控制频率下偶尔丢一帧对波形观测没有可见影响
    if (huart1.gState != HAL_UART_STATE_READY) {
        return;
    }

    // 将通道浮点数组拷贝到 DMA 发送缓冲前段，隔离 DMA 传输期间的数据修改
    memcpy(vofa_tx_buf, vofa_channels, VOFA_CHANNEL_COUNT * sizeof(float));

    // 在数据末尾追加 JustFloat 帧尾，VOFA+ 上位机靠这 4 字节来判断一帧结束
    memcpy(&vofa_tx_buf[VOFA_CHANNEL_COUNT * sizeof(float)], VOFA_TAIL, sizeof(VOFA_TAIL));

    // 触发 DMA 发送，帧总长度 = 通道数据 + 帧尾
    HAL_UART_Transmit_DMA(&huart1, vofa_tx_buf, (uint16_t)sizeof(vofa_tx_buf));
}
