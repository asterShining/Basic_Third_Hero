#ifndef CUSTOM_IMAGE_BRIDGE_H
#define CUSTOM_IMAGE_BRIDGE_H

/**
 * @brief 初始化“上位机原始 H.264 字节流 -> 云台板 -> 0x0310”桥接模块
 *
 * 模块只负责把 USB CDC 收到的原始视频字节流按 300B 固定块排队，
 * 再通过 USART6 以 0x0310 自定义数据帧发给图传链路，不参与图像解码和控制逻辑。
 */
void CustomImageBridgeInit(void);

/**
 * @brief 图像桥接周期任务，负责 USB 原始字节流出队、300B 分块和 USART6 TX 下发
 *
 * 任务侧不再解析 inner packet，也不再假设 JPEG 预览协议。
 * 只要上位机按顺序持续写入原始 H.264 字节流，这里就会持续按 300B 固定块透传到 0x0310。
 */
void CustomImageBridgeTask(void);

#endif // CUSTOM_IMAGE_BRIDGE_H
