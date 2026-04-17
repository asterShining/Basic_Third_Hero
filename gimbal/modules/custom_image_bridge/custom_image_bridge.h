#ifndef CUSTOM_IMAGE_BRIDGE_H
#define CUSTOM_IMAGE_BRIDGE_H

/**
 * @brief 初始化“上位机单包 JPEG 预览 inner packet v3 -> 云台板 -> 0x0310”桥接模块
 *
 * 这里的职责不是做视频重组，也不是做图像解码，而是把上位机送来的固定 300B
 * inner packet 先校验头字段与 payload CRC，再原样装入裁判系统 0x0310 自定义数据帧。
 * 这样可以把协议边界收紧在下位机入口，尽早发现转发链路上的错位、截断和脏数据。
 */
void CustomImageBridgeInit(void);

/**
 * @brief 图像桥接周期任务，负责 USB 包出队、300B packet 对齐校验和 USART6 TX 下发
 *
 * 任务侧会持续从 USB 原始流里重同步 `0x44 0x4C 0x03` 包头，只保留通过完整字段校验的
 * 300B 预览包，再按 0x0310 固定 payload 长度逐包透传到图传链路，避免旧多包视频语义残留。
 */
void CustomImageBridgeTask(void);

#endif // CUSTOM_IMAGE_BRIDGE_H
