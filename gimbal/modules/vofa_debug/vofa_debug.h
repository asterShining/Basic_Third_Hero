/**
 * @file vofa_debug.h
 * @brief VOFA+ JustFloat 协议调试输出模块
 * @note 通过空闲的 USART1 (4-pin, PA9=TX) 以 JustFloat 二进制协议
 *       向 VOFA+ 上位机实时输出浮点调试通道，用于波形观测和参数调试。
 *       只做 TX 发送，不注册 BSP 的 USART 接收回调，不占用 DEVICE_USART_CNT 槽位。
 */

#ifndef VOFA_DEBUG_H
#define VOFA_DEBUG_H

#include <stdint.h>

// VOFA JustFloat 通道数量，每个通道占 4 字节 float，帧尾固定 4 字节
// 当前规划 16 通道：6 摩擦轮线速度 + 拨盘速度 + 内外圈掉速差 + 发射计数 + 6 单电机掉速
#define VOFA_CHANNEL_COUNT 16u

/**
 * @brief 初始化 VOFA 调试模块
 *        内部清零通道缓冲，不改变 USART1 的硬件配置（CubeMX 已完成初始化）
 */
void VofaDebugInit(void);

/**
 * @brief 设置指定通道的浮点值
 * @param ch  通道索引 (0 ~ VOFA_CHANNEL_COUNT-1)
 * @param val 要写入的浮点值
 */
void VofaDebugSetChannel(uint8_t ch, float val);

/**
 * @brief 将当前所有通道数据打包为 JustFloat 帧并通过 USART1 DMA 发送
 *        若 USART1 TX 仍忙则本次跳过，不阻塞调用方
 */
void VofaDebugSend(void);

#endif // VOFA_DEBUG_H
