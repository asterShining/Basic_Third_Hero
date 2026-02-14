# master_process — 视觉上位机通信模块

> 适配同济大学 sp_vision_25 上位机通信协议

## 概述

本模块负责下位机与视觉上位机 (`sp_vision_25`) 的双向通信，支持三种通信方式：

| 宏定义              | 通信方式        | 对应上位机模块         |
|:------------------|:-------------|:----------------|
| `VISION_USE_VCP`  | USB 虚拟串口（默认） | `io/gimbal`     |
| `VISION_USE_UART` | 硬件串口         | `io/gimbal`     |
| `VISION_USE_CAN`  | CAN 总线       | `io/cboard`     |

在 `robot_def.h` 中选择启用哪个宏。

---

## 串口/VCP 协议 (SP协议)

### 帧格式

**下位机→上位机 (`SP_GimbalToVision_t`, 38字节):**

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|:--|:--|:--|:--|:--|
| 0 | 2 | head | u8[2] | 帧头 `'S','P'` |
| 2 | 1 | mode | u8 | 机器人模式 (0=空闲, 1=自瞄, 2=小符, 3=大符) |
| 3 | 16 | q[4] | float×4 | IMU 四元数 (w,x,y,z) |
| 19 | 4 | yaw | float | yaw 角度 (rad) |
| 23 | 4 | yaw_vel | float | yaw 角速度 (rad/s) |
| 27 | 4 | pitch | float | pitch 角度 (rad) |
| 31 | 4 | pitch_vel | float | pitch 角速度 (rad/s) |
| 35 | 4 | bullet_speed | float | 弹速 (m/s) |
| 39 | 2 | bullet_count | u16 | 累计发弹数 |
| 41 | 2 | crc16 | u16 | CRC16-CCITT (小端) |

**上位机→下位机 (`SP_VisionToGimbal_t`, 29字节):**

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|:--|:--|:--|:--|:--|
| 0 | 2 | head | u8[2] | 帧头 `'S','P'` |
| 2 | 1 | mode | u8 | 0=不控制, 1=控制不开火, 2=控制+开火 |
| 3 | 4 | yaw | float | 目标 yaw (rad) |
| 7 | 4 | yaw_vel | float | yaw 角速度前馈 (rad/s) |
| 11 | 4 | yaw_acc | float | yaw 角加速度前馈 (rad/s²) |
| 15 | 4 | pitch | float | 目标 pitch (rad) |
| 19 | 4 | pitch_vel | float | pitch 角速度前馈 (rad/s) |
| 23 | 4 | pitch_acc | float | pitch 角加速度前馈 (rad/s²) |
| 27 | 2 | crc16 | u16 | CRC16-CCITT (小端) |

### CRC16 校验

- **多项式**: CRC16-CCITT (反转形式, 0x8408)
- **初始值**: 0xFFFF
- **校验范围**: 从 `head[0]` 到 CRC16 字段之前的所有字节
- **存储方式**: 小端序 (低字节在前)
- **与上位机 `tools/crc.cpp` 完全相同的查表实现**

---

## CAN 协议

### CAN ID 定义

| CAN ID | 方向 | 内容 |
|:--|:--|:--|
| `0x100` | 下位机→上位机 | IMU 四元数 |
| `0x101` | 下位机→上位机 | 弹速/模式/射击模式 |
| `0xFF`  | 上位机→下位机 | 控制命令 |

### 数据编码

所有数据均使用 **int16 缩放编码**，大端序 (高字节在前):

| 数据类型 | 缩放因子 | 精度 |
|:--|:--|:--|
| 四元数分量 | ×10000 | 0.0001 |
| yaw/pitch角度 | ×10000 | 0.0001 rad |
| 弹速 | ×100 | 0.01 m/s |

### CAN 帧格式

**四元数帧 (0x100):**
```
[x_h, x_l, y_h, y_l, z_h, z_l, w_h, w_l]
```

**状态帧 (0x101):**
```
[bs_h, bs_l, mode, shoot_mode, 0, 0, 0, 0]
```

**命令帧 (0xFF, 上位机→下位机):**
```
[control, shoot, yaw_h, yaw_l, pitch_h, pitch_l, dist_h, dist_l]
```

---

### 上位机 → 下位机 (30字节)

```c
// 初始化 (UART 模式传串口句柄, VCP/CAN 模式传 NULL)
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

// 发送数据 (在主循环/定时任务中调用)
void VisionSend(void);

// 设置 IMU 四元数
void VisionSetQuaternion(const float *q);

// 设置云台姿态
void VisionSetAltitude(float yaw, float pitch, float yaw_vel, float pitch_vel);

// 设置机器人状态
void VisionSetStatus(uint8_t mode, float bullet_speed, uint16_t bullet_count);

// 检查是否在线
uint8_t VisionIsOnline(void);
```

## 文件结构

| 文件 | 说明 |
|:--|:--|
| `master_process.h` | 模块头文件，定义收发结构体和外部接口 |
| `master_process.c` | 模块实现，三种通信模式的初始化/收发逻辑 |
| `sp_vision_protocol.h` | 协议定义，与上位机完全匹配的数据包结构体 |
| `sp_vision_protocol.c` | 协议实现，CRC16-CCITT + 串口打包/解包 + CAN编解码 |
