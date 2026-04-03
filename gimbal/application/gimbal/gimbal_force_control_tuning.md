# 云台力控参数说明与调参手册

## 1. 当前控制架构

当前云台不是纯开环扭矩控制，而是：

- **前馈主导**：模型先给出 Pitch / Yaw 所需的主要补偿力矩
- **串级 PID 收尾**：位置环和速度环只负责消化模型误差、静差和残余扰动

代码入口：

- 前馈主逻辑：[gimbal.c](./gimbal.c)
- 遥控器与键鼠灵敏度：[../cmd/robot_cmd.c](../cmd/robot_cmd.c)

当前各轴力控项如下：

- **Pitch 轴**
  - 重力前馈
  - 离心项前馈
- **Yaw 轴**
  - 惯量前馈
  - 黏性摩擦前馈
  - 静摩擦前馈
  - 科氏耦合前馈

## 2. 参数总表

### 2.1 Pitch 输入灵敏度

| 参数 | 位置 | 作用 | 调大效果 | 调小效果 |
| --- | --- | --- | --- | --- |
| `REMOTE_PITCH_SENSITIVITY` | `robot_cmd.c` | DT7 / VT03 遥控 Pitch 灵敏度 | 抬头压头更快 | 手感更稳但更肉 |
| `MOUSE_PITCH_SENSITIVITY_DEG` | `robot_cmd.c` | 键鼠 Pitch 灵敏度 | 鼠标抬头更跟手 | 微调更稳但慢 |

建议：

- 先只小幅调，步长控制在 `10% ~ 15%`
- 一旦出现明显超调，先回退灵敏度，不要先怀疑前馈

### 2.2 Pitch 力控参数

| 参数 | 作用 | 调大效果 | 调小效果 | 典型现象 |
| --- | --- | --- | --- | --- |
| `PITCH_GRAVITY_COEFFICIENT_K1` | Pitch 主重力项 | 大角度支撑更强 | 大角度更容易下坠 | 俯仰大角度时托不住或顶太狠 |
| `PITCH_GRAVITY_COEFFICIENT_K2` | Pitch 非对称修正项 | 一侧姿态补偿更强 | 一侧姿态补偿更弱 | 抬头准但低头不准，或反过来 |
| `PITCH_GRAVITY_OFFSET` | Pitch 整体偏置 | 所有姿态整体更抬 | 所有姿态整体更沉 | 所有角度都一起偏上或偏下 |
| `PITCH_CENTRIFUGAL_FEEDFORWARD_K` | Pitch 离心项 | 小陀螺/高速 Yaw 时 Pitch 抗飘更强 | 小陀螺时 Pitch 更容易被甩偏 | 开小陀螺后枪口抬头或低头 |
| `PITCH_FEEDFORWARD_LIMIT` | Pitch 前馈总限幅 | 补偿更敢给 | 更安全但可能感觉“没加进去” | 前馈效果弱或顶满过猛 |

### 2.3 Yaw 力控参数

| 参数 | 作用 | 调大效果 | 调小效果 | 典型现象 |
| --- | --- | --- | --- | --- |
| `YAW_INERTIA_BASE` | Yaw 基础惯量前馈 | 大动作起停更跟手 | 起停更依赖 PID 追赶 | 快速甩头启动/刹停发肉 |
| `YAW_INERTIA_PITCH_COS2_GAIN` | Pitch 姿态相关的 Yaw 惯量修正 | 不同 Pitch 角度下的 Yaw 一致性更强 | 抬头/低头后 Yaw 手感变化更明显 | 平视时准，抬头后手感变了 |
| `YAW_VISCOUS_FEEDFORWARD_K` | Yaw 黏性摩擦前馈 | 匀速甩头更顺 | 匀速阶段更依赖积分维持 | 高速甩头中段发涩 |
| `YAW_STATIC_FEEDFORWARD_K` | Yaw 静摩擦前馈 | 起转更果断 | 起转更容易卡住 | 第一口转不起来，过零换向发涩 |
| `YAW_STATIC_FEEDFORWARD_DEADBAND_RAD_S` | 静摩擦死区 | 更不容易停稳附近乱补偿 | 更早注入静摩擦补偿 | 停稳附近自拧或低速迟滞 |
| `YAW_CORIOLIS_FEEDFORWARD_K` | Yaw 科氏耦合项 | 复合动作更顺 | 复合动作更卡 | 甩头同时抬头/低头时掉速或顿挫 |
| `YAW_FEEDFORWARD_LIMIT` | Yaw 前馈总限幅 | 力控效果更明显 | 更保守更安全 | 效果弱或一给就猛 |

### 2.4 Yaw 参考导数估计参数

| 参数 | 作用 | 调大效果 | 调小效果 | 典型现象 |
| --- | --- | --- | --- | --- |
| `YAW_REF_RATE_LPF_RC` | Yaw 参考速度低通 | 更稳、更不抖 | 更灵、更容易吃到噪声 | 起停细碎抖动或感觉跟手慢半拍 |
| `YAW_REF_ACC_LPF_RC` | Yaw 参考加速度低通 | 惯量前馈更平滑 | 惯量前馈更激进 | 起停瞬间是否有尖峰感 |
| `YAW_REF_DERIV_RESET_THRESHOLD_DEG` | 跳变复位阈值 | 更容易识别大步跳目标并清零导数 | 更少复位、连续性更强 | 切模式、贴目标后是否冲一下 |

### 2.5 PID 参数

| 参数 | 作用 | 调大效果 | 调小效果 |
| --- | --- | --- | --- |
| `yaw angle_PID.Kp` | Yaw 外环刚度 | 跟得更紧 | 更稳但更肉 |
| `yaw speed_PID.Kp` | Yaw 内环刚度 | 更硬、更快 | 更柔但更依赖前馈 |
| `yaw speed_PID.Ki` | Yaw 匀速静差修正 | 静差更小 | 不容易积累后突然拧 |
| `pitch angle_PID.Kp` | Pitch 外环刚度 | 更快 | 更稳 |
| `pitch speed_PID.Kp` | Pitch 内环刚度 | 更硬 | 更柔 |
| `pitch speed_PID.Ki` | Pitch 静差修正 | 更能托住小误差 | 更不容易累积残留积分 |

## 3. 推荐调参顺序

### 步骤 1：锁死 Pitch 重力项

前提：你已经确认当前重力补偿是正确的。

此时先不要再动：

- `PITCH_GRAVITY_COEFFICIENT_K1`
- `PITCH_GRAVITY_COEFFICIENT_K2`
- `PITCH_GRAVITY_OFFSET`

### 步骤 2：调 Pitch 离心项

操作：

- 固定一个非零 Pitch 角，建议 `10° ~ 25°`
- 开 / 关小陀螺
- 观察枪口是否明显抬头或低头

调法：

- 开小陀螺后偏移更小：方向对，继续小步加
- 开小陀螺后偏移更大：方向错，直接反号
- 起转 / 停转瞬间变抖：值过大，回退一点

### 步骤 3：调 Yaw 基础力控

此阶段先只看纯 Yaw 大动作，不同时做剧烈 Pitch。

顺序：

1. `YAW_INERTIA_BASE`
2. `YAW_VISCOUS_FEEDFORWARD_K`
3. `YAW_STATIC_FEEDFORWARD_K`

现象判断：

- 启动和刹停发肉：加 `YAW_INERTIA_BASE`
- 匀速中段发涩：加 `YAW_VISCOUS_FEEDFORWARD_K`
- 起转第一下不利索、换向卡：加 `YAW_STATIC_FEEDFORWARD_K`
- 停稳附近来回拧：减 `YAW_STATIC_FEEDFORWARD_K` 或增大 `YAW_STATIC_FEEDFORWARD_DEADBAND_RAD_S`

### 步骤 4：调 Yaw 复合运动力控

操作：

- 高速甩头
- 同时快速抬头 / 低头

主要参数：

- `YAW_CORIOLIS_FEEDFORWARD_K`
- `YAW_INERTIA_PITCH_COS2_GAIN`

现象判断：

- 复合动作掉速、顿一下：先调 `YAW_CORIOLIS_FEEDFORWARD_K`
- 平视时效果好，抬头后 Yaw 手感明显变：再调 `YAW_INERTIA_PITCH_COS2_GAIN`

### 步骤 5：最后减 PID

只有当前馈已经明显有效后，才动 PID。

建议顺序：

1. 先减 `angle_PID.Kp`
2. 再看要不要减 `speed_PID.Kp`
3. 最后再减 `Ki`

理由：

- 前馈已经把大部分力矩工作做了
- 如果 PID 还维持旧强度，很容易出现高频发硬、过冲和停稳后抽动

## 4. 症状对照表

### 症状：Pitch 更灵了，但一拉就过头

优先调整：

- 降 `REMOTE_PITCH_SENSITIVITY`
- 降 `MOUSE_PITCH_SENSITIVITY_DEG`
- 如仍明显，轻降 `pitch angle_PID.Kp`

### 症状：Pitch 在小陀螺时还是会上下飘

优先调整：

- `PITCH_CENTRIFUGAL_FEEDFORWARD_K`

### 症状：Yaw 快速启动还是有点肉

优先调整：

- `YAW_INERTIA_BASE`

### 症状：Yaw 匀速甩头中段发涩

优先调整：

- `YAW_VISCOUS_FEEDFORWARD_K`

### 症状：Yaw 起转第一下卡住，过了某个点才突然动

优先调整：

- `YAW_STATIC_FEEDFORWARD_K`

### 症状：Yaw 停稳附近自己来回轻拧

优先调整：

- 减 `YAW_STATIC_FEEDFORWARD_K`
- 增 `YAW_STATIC_FEEDFORWARD_DEADBAND_RAD_S`
- 必要时增 `YAW_REF_RATE_LPF_RC`

### 症状：切模式或贴齐目标后，Yaw 会突然顶一下

优先调整：

- 增 `YAW_REF_DERIV_RESET_THRESHOLD_DEG`
- 增 `YAW_REF_ACC_LPF_RC`

### 症状：复合动作时 Yaw 掉速或顿挫

优先调整：

- `YAW_CORIOLIS_FEEDFORWARD_K`

### 症状：前馈明显起作用了，但整机发硬、容易抖

优先调整：

- 降 `yaw angle_PID.Kp`
- 降 `pitch angle_PID.Kp`
- 必要时再降 `speed_PID.Kp`

## 5. 推荐步长

建议每次只改一个参数。

推荐步长：

- 灵敏度参数：`10% ~ 15%`
- 重力拟合参数：`5% ~ 10%`
- 离心 / 科氏 / 黏性摩擦参数：`10% ~ 20%`
- 静摩擦参数：每次 `0.02 ~ 0.05`
- 限幅参数：每次 `0.3 ~ 0.8`
- LPF 时间常数：每次 `0.003 ~ 0.005`
- PID `Kp`：每次 `5% ~ 10%`
- PID `Ki`：先减半，再决定是否继续降到 0

## 6. 建议观察量

如果后续需要加低频日志，最值得观察的是：

- `pitch_ff_storage`
- `yaw_ff_storage`
- `yaw_ref_rate_filtered`
- `yaw_ref_acc_filtered`
- `yaw_motor->measure.velocity`
- `pitch_motor->measure.velocity`
- `yaw_motor->measure.torque`
- `pitch_motor->measure.torque`

## 7. 安全规则

- 一次只改一个参数
- 先前馈，后 PID
- 先把方向调对，再谈大小
- 任何时候如果“现象立刻更糟”，先怀疑符号，再怀疑模型
- 前馈限幅不要一开始就放太大
- 切模式、切源、掉线复活时若出现冲击，优先看参考导数复位是否充分

## 8. 当前默认认知

- Pitch 重力补偿已经基本可用
- 小陀螺切跟随问题已单独解决
- 当前这版 Yaw 力控重点不在“静止时存在感”，而在“大动作起停”和“Yaw + Pitch 复合动作”时体现效果
