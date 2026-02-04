# SP 通信协议 - AU电控组

## 协议概述

SP 通信协议用于英雄机器人云台板（下位机）与 sp_vision_25 视觉系统（上位机）之间的数据交换。

| 参数     | 值                    |
| -------- | --------------------- |
| 帧头     | `'S'` `'P'` (0x53 0x50) |
| 校验     | CRC16                 |
| 通信方式 | USB VCP / UART        |
| 波特率   | 115200 (UART模式)     |

---

## 数据帧结构

### 下位机 → 上位机 (43字节)

| 偏移 | 长度 | 字段名        | 类型     | 说明                           |
| ---- | ---- | ------------- | -------- | ------------------------------ |
| 0    | 2    | head          | uint8[2] | 帧头 'SP' (0x53, 0x50)         |
| 2    | 1    | mode          | uint8    | 工作模式 (见下表)              |
| 3    | 16   | q[4]          | float[4] | 姿态四元数 (w,x,y,z)           |
| 19   | 4    | yaw           | float    | Yaw角度 (度)                   |
| 23   | 4    | yaw_vel       | float    | Yaw角速度 (度/秒)              |
| 27   | 4    | pitch         | float    | Pitch角度 (度)                 |
| 31   | 4    | pitch_vel     | float    | Pitch角速度 (度/秒)            |
| 35   | 4    | bullet_speed  | float    | 弹速 (m/s)                     |
| 39   | 2    | bullet_count  | uint16   | 累计发弹数                     |
| 41   | 2    | crc16         | uint16   | CRC16校验                      |

**mode 字段定义:**
- `0` - 空闲模式
- `1` - 自瞄模式

---

### 上位机 → 下位机 (30字节)

| 偏移 | 长度 | 字段名    | 类型     | 说明                           |
| ---- | ---- | --------- | -------- | ------------------------------ |
| 0    | 2    | head      | uint8[2] | 帧头 'SP' (0x53, 0x50)         |
| 2    | 1    | mode      | uint8    | 控制模式 (见下表)              |
| 3    | 4    | yaw       | float    | 目标Yaw角度 (度)               |
| 7    | 4    | yaw_vel   | float    | Yaw角速度前馈 (度/秒)          |
| 11   | 4    | yaw_acc   | float    | Yaw角加速度前馈 (度/秒²)       |
| 15   | 4    | pitch     | float    | 目标Pitch角度 (度)             |
| 19   | 4    | pitch_vel | float    | Pitch角速度前馈 (度/秒)        |
| 23   | 4    | pitch_acc | float    | Pitch角加速度前馈 (度/秒²)     |
| 27   | 2    | crc16     | uint16   | CRC16校验                      |

**mode 字段定义:**
- `0` - 不控制 (视觉丢失目标)
- `1` - 控制云台，不开火
- `2` - 控制云台并开火

---

## 下位机数据处理

### 接收数据流程

```
接收USB/UART数据
        ↓
检查帧头 == 'SP'?  ──否──→ 丢弃
        ↓是
校验CRC16 ──失败──→ 丢弃
        ↓通过
解析mode字段
        ↓
┌───────┴───────┐
│   mode = 0    │  → 视觉离线，使用手动控制
│   mode = 1    │  → 使用yaw/pitch控制云台
│   mode = 2    │  → 控制云台 + 触发开火
└───────────────┘
```

### 接收数据使用

```c
// 在 RobotCMDTask() 中读取视觉数据
if (vision_recv_data->mode != 0) {
    // 使用视觉目标覆盖手动控制
    gimbal_cmd_send.yaw = vision_recv_data->yaw;
    gimbal_cmd_send.pitch = vision_recv_data->pitch;
    
    // 使用前馈提高跟踪性能
    gimbal_cmd_send.yaw_feedforward = vision_recv_data->yaw_vel;
    
    // mode=2 时自动开火
    if (vision_recv_data->mode == 2) {
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
    }
}
```

---

## 上位机数据处理

### 接收数据流程

```
接收USB/UART数据
        ↓
检查帧头 == 'SP'?  ──否──→ 丢弃
        ↓是
校验CRC16 ──失败──→ 丢弃
        ↓通过
解析云台状态
        ↓
┌─────────────────────────────┐
│ 提取四元数 → 坐标系变换     │
│ 提取yaw/pitch → 弹道解算    │
│ 提取bullet_speed → 落点补偿 │
└─────────────────────────────┘
        ↓
目标检测 + 预测
        ↓
计算控制指令 → 发送给下位机
```

### 发送数据逻辑

```python
# 上位机 Python 伪代码
if target_detected:
    if fire_allowed and aim_ready:
        tx_frame.mode = 2      # 控制并开火
    else:
        tx_frame.mode = 1      # 仅控制
    tx_frame.yaw = target_yaw
    tx_frame.pitch = target_pitch
    tx_frame.yaw_vel = target_yaw_vel    # 速度前馈
    tx_frame.yaw_acc = target_yaw_acc    # 加速度前馈
else:
    tx_frame.mode = 0          # 不控制
```

---

## CRC16 校验

使用多项式 `0xA001` 的 CRC16 算法：

```c
uint16_t crc_16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}
```

校验范围：从 `head[0]` 到 `crc16` 前一个字节。

---

## 通信时序

```
下位机 (200Hz)                    上位机 (100Hz)
     │                                  │
     │──── GimbalToVision (43B) ───────→│
     │                                  │ 目标检测
     │                                  │ 弹道解算
     │←─── VisionToGimbal (30B) ────────│
     │                                  │
     │ 云台控制                          │
     │ 发射控制                          │
     │                                  │
```

---

## 调试建议

1. **测试模式**: 调用 `VisionTestMode()` 发送递增的 yaw 值 (0-359)
2. **日志监控**: 开启 RTT Viewer 查看接收帧解析结果
3. **帧计数**: 调用 `VisionGetTestStats()` 获取有效帧数量
