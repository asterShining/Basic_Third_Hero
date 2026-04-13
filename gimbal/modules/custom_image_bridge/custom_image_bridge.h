#ifndef CUSTOM_IMAGE_BRIDGE_H
#define CUSTOM_IMAGE_BRIDGE_H

/**
 * @brief 初始化“上位机原始 H.264 -> 云台板 -> 0x0310”图像桥接模块
 *
 */
void CustomImageBridgeInit(void);

/**
 * @brief 图像桥接周期任务，负责 USB 包出队、300B 聚合和 USART6 TX 下发
 *
 */
void CustomImageBridgeTask(void);

#endif // CUSTOM_IMAGE_BRIDGE_H
