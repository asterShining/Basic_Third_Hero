# SP 视觉通信协议 (下位机模块)

## 概述

本模块实现英雄机器人云台板与 sp_vision_25 视觉系统的串口通信。

## 协议规格

| 参数     | 值                  |
| -------- | ------------------- |
| 帧头     | `'SP'` (0x53 0x50) |
| 校验     | CRC16              |
| 通信方式 | USB VCP / UART     |

## 数据帧

### 下位机 → 上位机 (43字节)

```c
typedef struct {
    uint8_t head[2];        // 'SP'
    uint8_t mode;           // 0-空闲, 1-自瞄
    float q[4];             // 四元数 (wxyz)
    float yaw, yaw_vel;     // Yaw角度/角速度
    float pitch, pitch_vel; // Pitch角度/角速度
    float bullet_speed;     // 弹速 (m/s)
    uint16_t bullet_count;  // 发弹计数
    uint16_t crc16;
} SP_GimbalToVision_t;
```

### 上位机 → 下位机 (30字节)

```c
typedef struct {
    uint8_t head[2];        // 'SP'
    uint8_t mode;           // 0-不控制, 1-控制, 2-开火
    float yaw, yaw_vel, yaw_acc;
    float pitch, pitch_vel, pitch_acc;
    uint16_t crc16;
} SP_VisionToGimbal_t;
```

## 使用方法

### 初始化

```c
#include "master_process.h"

Vision_Recv_s *vision_data;

void Init(void) {
    vision_data = VisionInit(NULL);  // VCP模式
}
```

### 发送数据

```c
void Task(void) {
    VisionSetMode(VISION_MODE_AUTO_AIM);
    VisionSetQuaternion(q[0], q[1], q[2], q[3]);
    VisionSetGimbalState(yaw, yaw_vel, pitch, pitch_vel);
    VisionSetBulletInfo(bullet_speed, fire_count);
    VisionSend();
}
```

### 接收数据

```c
void Task(void) {
    if (vision_data->mode == 2) {
        gimbal_cmd.yaw = vision_data->yaw;
        gimbal_cmd.pitch = vision_data->pitch;
        shoot_cmd.fire = 1;
    }
}
```

### 测试模式

```c
VisionTestMode();  // 启用测试模式
// 下位机发送递增yaw (0-359)
// 上位机数据回显到日志
```

## 文件结构

| 文件              | 描述           |
| ----------------- | -------------- |
| sp_protocol.h     | 协议数据结构   |
| sp_protocol.c     | 编解码实现     |
| master_process.h  | 上层接口       |
| master_process.c  | 通信实现       |