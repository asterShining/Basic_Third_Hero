# 简化超电UI显示 — 用能量百分比替换状态字符串

## Context

当前超电UI在选手端上显示三部分：
- 状态圆环 `on_3`（绿色=在线可用，紫红色=不可用）
- 大号 "c" 标签（font 31，坐标 198,661）
- 小字号状态字符串（font 20，坐标 242,661）— 显示 OFF/STB/OUT/FLT/CUT

用户认为小字号的五态文本在比赛场景下难以快速辨识。超电能量百分比数据（`SuperCapGetEnergyPercent()` 返回 0.0-100.0%）已从超电板通过 CAN 获取并解析到 `SuperCap_Rx_Msg_s.capEnergyPercent`，但**从未在 UI 上展示**。

## Work Objectives

1. 移除小字号(20)的超电状态字符串 OFF/STB/OUT/FLT/CUT
2. 在 "c" 标签**下方**新增超电能量百分比显示（格式如 "85%"）
3. 保留状态圆环 `on_3` 作为在线/离线的一眼绿色/红色指示

## Guardrails

- **MUST**: 不改变其他 UI 元素（friction、power、pitch、body 等）的位置和渲染逻辑
- **MUST**: 不改变 0x0301 协议发送时序和分包策略
- **MUST**: 超电离线时百分比显示 "0%"，不使用残留数据
- **MUST NOT**: 引入编译警告
- **MUST NOT**: 删除或修改 `SuperCapGetEnergyPercent()` API 接口

## Data Flow（变化后）

```
SuperCap_Rx_Msg_s.capEnergyPercent (uint8_t, 0-255)
  → SuperCapGetEnergyPercent(cap) 返回 0.0-100.0 (float)
    → chassis_ui.c: ui_data.cap_energy_percent  ← 新增字段
      → UIDisplaySnapshot_t.cap_energy_percent_int ← 新增字段(0-100)
        → UIBuildStrings() 格式化为 "XX%" 字符串
          → UICharDraw() → 0x0301 协议包 → 选手端显示
```

## Task Flow

### Step 1: 在数据结构中增加能量百分比字段

**文件**: `chassis/modules/referee/rm_referee.h`

- 在 `Referee_Interactive_info_t` 结构体中 `cap_on` 字段之后，新增：
  ```c
  float cap_energy_percent; // 超电能量百分比(0.0-100.0)，目的是直接承载SuperCapGetEnergyPercent()的返回值供UI层消费
  ```

**文件**: `chassis/modules/referee/referee_task.c`

- 在 `UIDisplaySnapshot_t` 结构体中 `cap_state` 字段附近，新增：
  ```c
  uint8_t cap_energy_percent_int; // 超电能量百分比整数值(0-100)，目的是格式化为百分比字符串时避免浮点运算
  ```
- 在 `UIBuildDisplaySnapshot()` 中将 `interactive_data->cap_energy_percent` 四舍五入赋值给 `snapshot->cap_energy_percent_int`

**Acceptance Criteria**:
- `Referee_Interactive_info_t` 含 `float cap_energy_percent` 字段
- `UIDisplaySnapshot_t` 含 `uint8_t cap_energy_percent_int` 字段
- 编译通过

---

### Step 2: 在 chassis_ui.c 中采集能量百分比

**文件**: `chassis/application/chassis/chassis_ui.c`

在 `#ifdef USE_SUPER_CAP` 块中（`SuperCapIsOnline` 分支内）：
- 在设置 `ui_data.chassis_power_w` 之后，新增：
  ```c
  ui_data.cap_energy_percent = SuperCapGetEnergyPercent(cap);
  ```

在超电离线分支（函数末尾的 else 块，约第 85-88 行）：
- 在现有 `ui_data.cap_state = UI_CAP_STATE_OFF; ui_data.cap_on = 0u;` 之后，新增：
  ```c
  ui_data.cap_energy_percent = 0.0f;
  ```

**Acceptance Criteria**:
- 在线时 `ui_data.cap_energy_percent` 等于 `SuperCapGetEnergyPercent()` 返回值
- 离线时 `ui_data.cap_energy_percent` 为 0.0f
- 无编译警告

---

### Step 3: 替换状态字符串为百分比显示（referee_task.c 核心修改）

**文件**: `chassis/modules/referee/referee_task.c`

#### 3a. 新增百分比位置宏定义

在现有 `UI_LABEL_CAP_STATE_*` 宏（约第 169-172 行）之后新增：

```c
// 超电能量百分比放在大号 `c` 标签正下方，目的是形成"标签在上、数值在下"的紧凑纵向布局，不再沿用旧的水平排列状态字方案。
#define UI_LABEL_CAP_PERCENT_X    UI_LABEL_CAP_X        // 198, 与 "c" 标签左对齐
#define UI_LABEL_CAP_PERCENT_Y    698u                  // "c" 标签(661)下方，保留约37单位的纵向间距
#define UI_LABEL_CAP_PERCENT_FONT 20u                   // 与 F 旁档位字号一致，保证左侧列信息层级统一
#define UI_LABEL_CAP_PERCENT_WIDTH 2u
```

#### 3b. 重命名枚举和变量（状态字 → 百分比）

| 旧名称 | 新名称 |
|--------|--------|
| `UI_STRING_CAP_STATE` | `UI_STRING_CAP_PERCENT` |
| `ui_runtime.last_cap_state_string` | `ui_runtime.last_cap_energy_string` |
| `ui_runtime.cap_state_string_dirty` | `ui_runtime.cap_energy_string_dirty` |
| `ui_runtime.cap_state_string_retry_count` | `ui_runtime.cap_energy_string_retry_count` |
| `UIRefreshCapStateString()` | `UIRefreshCapEnergyPercent()` |
| `UISendDirtyCapStateString()` | `UISendDirtyCapEnergyPercent()` |
| `cap_state_text` (局部变量) | `cap_percent_text` |

#### 3c. 修改 UIBuildStrings()

替换现有的状态字映射逻辑（约第 856-875 行）和绘制调用（约第 893-895 行）：

- 移除 `const char *cap_state_text = "OFF";` 及整个 if-else 状态映射块
- 新增百分比格式化逻辑：
  ```c
  char cap_percent_text[8] = "0%";
  if (snapshot != NULL && snapshot->cap_energy_percent_int <= 100u) {
      snprintf(cap_percent_text, sizeof(cap_percent_text), "%u%%", (unsigned int)snapshot->cap_energy_percent_int);
  }
  ```
- 将 `UICharDraw` 的绘制目标从 `UI_STRING_CAP_STATE` 改为 `UI_STRING_CAP_PERCENT`，坐标改为 `UI_LABEL_CAP_PERCENT_X/Y`

#### 3d. 修改 UIRefreshCapEnergyPercent()（原 UIRefreshCapStateString）

- 脏检查条件从比较状态枚举字符串改为比较百分比整数值
- `UIBuildStrings()` 调用保持不变（每次调用都会重绘所有字符串，但只刷新对应的 dirty 标志）

#### 3e. 更新所有引用

- `UIRuntimeReset()`: 将 `last_cap_state_string` 改为 `last_cap_energy_string`
- `UIStartInitCycle()`: 将 `cap_state_string_dirty/retry_count` 改为 `cap_energy_string_dirty/retry_count`
- `UIFinishInitCycle()`: 同上
- `UIProcessRuntimeUpdate()`: 调用 `UIRefreshCapEnergyPercent()` 并检查 `cap_energy_string_dirty`
- `UISendDirtyCapEnergyPercent()`: 对 `ui_strings[UI_STRING_CAP_PERCENT]` 进行 `UICharRefresh`

**Acceptance Criteria**:
- 状态字符串 OFF/STB/OUT/FLT/CUT 不再出现于代码中
- "c" 标签下方显示 "XX%" 格式百分比
- 超电离线时显示 "0%"
- on_3 圆环状态行为不变
- 百分比变化时触发字符刷新（原有重试机制不变）

---

### Step 4: 编译验证

在 `chassis/` 目录下执行编译，确认零错误零警告：

```bash
cd chassis/build && cmake -G Ninja -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug .. && ninja -j$(nproc)
```

**Acceptance Criteria**:
- `ninja` 返回 0，无 error 无 warning
- 链接成功，生成 ELF 文件

## Success Criteria

1. 选手端 UI 上超电区域：on_3 圆环 + "c" 标签 + 下方百分比数字（如 "85%"）
2. 不再显示 OFF/STB/OUT/FLT/CUT 五种状态文本
3. 超电离线时百分比显示 "0%"，与其他 UI 元素无重叠
4. 所有修改局限在 3 个文件内，不影响其他模块

## 风险点

- **百分比显示与 "r" 标签(Y=733)纵向距离足够**：百分比在 Y=698（font 20），"r" 标签在 Y=733（font 30），间距约 35 单位，无重叠风险。
- **UI 字符串对象 ID "cst" 需改为新的 identifier**：确保客户端不会因为复用同一 ID 而产生旧状态字残留。建议改为 "cpe"（cap percent energy）。
- **`UI_STRING_COUNT` 不变**：只是重命名现有枚举项，不增删，无需修改计数。
