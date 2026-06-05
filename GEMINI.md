# RoboMaster Basic Third Hero 架构与开发指南

## 1. 项目概览 (Project Overview)
本项目是 RoboMaster 英雄机器人的控制代码库。工程基于 STM32F407 芯片，并采用了清晰的双板/双控制流架构：
- **`chassis/` (底盘)**：负责移动底盘解算、通信及电容控制。
- **`gimbal/` (云台)**：负责云台姿态解算、射击（摩擦轮与拨弹盘）、视觉通信及裁判系统接入。

**技术栈与框架：**
- **语言**：C 语言
- **构建系统**：CMake + ARM GCC (`gcc-arm-none-eabi`)
- **系统调度**：FreeRTOS
- **代码结构**：严格遵守分层架构，从底层向上依次为：BSP（板级支持包） -> 模块层 (Modules) -> 应用层 (Application)。

## 2. 构建与运行 (Building and Running)
本项目设计为基于 **VSCode + CMake + Ozone** 的现代化嵌入式开发工作流。

### 常用命令与操作
- **构建 (Build)**: 
  使用 VSCode 的 CMake Tools 插件直接配置和生成，或者在终端执行 `cmake` 和 `make` (配合 `CMakePresets.json`)。
  *(建议在对应的 `chassis` 或 `gimbal` 目录下创建 `build/` 文件夹后执行 `cmake .. && make -j`)*
- **烧录与调试**: 
  - **运行**: 提供 `run.sh` 脚本用于快速执行烧录（通常依赖于 J-Link）。
  - **JLink 烧录**: 提供 `jlink.sh` 脚本用于 JLink 命令行烧录。
  - **高级调试**: 推荐使用 [Ozone](https://www.segger.com/products/development-tools/ozone-j-link-debugger/) 连接 `.mxproject` 或编译产出的 ELF 文件进行图形化和数据流调试。

## 3. 射击控制与拨弹盘逻辑指南 (Shoot Logic & Tuning Guide)
云台项目中的发射控制（射击逻辑）位于 `gimbal/application/shoot/` 目录下。该模块涉及双层摩擦轮（内圈与外圈）和拨弹盘的位置与速度协同。

### 3.1 拨弹盘控制逻辑分析 (The Dip Lock Strategy)
当前的拨弹盘 (`loader`) 逻辑是**高度可行且非常先进**的，其核心是基于“**掉速锁角 (Dip Lock)**”与“**冲刺等待 (Rush + Wait)**”机制：
1. **触发冲刺 (Rush)**：在接收到单发请求并确认摩擦轮已达速 (`SHOOT_SPEED_READY_THRESHOLD`) 后，拨弹盘执行一次预设的大步距冲刺 (`SF_RUSH_BULLET_COUNT`)，将弹丸推入摩擦轮。
2. **掉速检测 (Dip Detection)**：代码实时记录摩擦轮的速度峰值基线（`single_fire.baseline_speed`）。当弹丸挤过摩擦轮时，电机由于负载突增会发生掉速（转速下降）。
3. **立刻锁角 (Immediate Lock)**：一旦内圈或外圈检测到的掉速超过设定阈值 (`FRICTION_SPEED_DIP_THRESHOLD` / `OUTER_DIP_CONFIRM_THRESHOLD`)，状态机会立即判定弹丸已成功送入，并立刻将拨弹盘锁死在当前角度（进入 `SF_WAIT_RECOVER`）。
4. **回速等待 (Wait Recover)**：发弹成功后，系统强制等待摩擦轮速度重新恢复 (`FRICTION_SPEED_RECOVER_THRESHOLD`) 才能响应下一次击发。

**与速度不稳定的关联：**
出射速度不稳定确实**与拨弹盘推弹力度和摩擦轮的动态响应直接相关**：
- 如果拨弹盘在推弹时的冲刺过于粗暴（或位置环过软），每次弹丸进入摩擦轮时的初始挤压角度不同，会导致摩擦轮施加给弹丸的动能产生散布。
- 其次，初段拨弹的硬前馈（`FRICTION_FEEDFORWARD_CURRENT`）如果与实际机械阻力不匹配，可能引发摩擦轮速度剧烈振荡。

### 3.2 摩擦轮调参指南
如果你发现弹丸出射速度依然不稳定，请重点关注以下参数和环节的调节：

1. **机械一致性 (Mechanical Consistency)**
   - 检查内外圈四个/六个摩擦轮的对称间隙，确保左右两侧受力绝对平衡。
   - 检查摩擦皮是否磨损。
   - 更换同一批次的弹丸进行测试。

2. **拨弹盘推力调节 (Loader Consistency)**
   - 检查 `LOADER_MOTOR_ANGLE_PER_BULLET` 是否能完美对应机械结构的单发齿距。
   - 微调冲刺步距 `SF_RUSH_BULLET_COUNT` 和位置环 PID，确保拨弹电机在推入瞬间的出力是均匀平滑的。

3. **摩擦轮 PID 与前馈优化 (Friction PID & FF)**
   - 检查速度环 PID：重点加强速度环刚性（尤其是积分 `I` 和前馈 `FF` 的作用），使得摩擦轮在弹丸通过瞬间的掉速不仅要明显（以触发 Dip Lock），而且**恢复要极快且无过冲**。
   - 调整前馈电流：如果冲刺初段摩擦轮由于拨盘的猛推而过度塌陷，请微调 `FRICTION_FEEDFORWARD_CURRENT`（或者持续时间）。

4. **左右修剪补偿 (Trim Compensation)**
   - 代码中支持微调 `FRICTION_TRIM_INNER_LEFT`, `FRICTION_TRIM_INNER_RIGHT` 等。若通过 Ozone 抓取数据发现左右轮存在系统性误差，可微调此参数进行软件拉平，保证摩擦力严格对称。

5. **掉速阈值微调 (Dip Threshold)**
   - `FRICTION_SPEED_DIP_THRESHOLD`（内圈）和 `OUTER_DIP_CONFIRM_THRESHOLD`（外圈）：阈值定得过低，电机震动噪声就会误触发刹车；定得过高，拨盘会推得过深，引发多发连射或卡弹。需要结合 Ozone 数据抓拍，找到最能准确包络一颗弹丸掉速谷底的数值。

## 4. 开发规范 (Development Conventions)
- **DRY 原则**：增加新功能时优先寻找 `bsp/` 和 `modules/` 下已有的工具函数。
- **模块独立**：不同功能间的解耦非常清晰（如射击与云台控制分离）。修改控制逻辑时，切勿跨作用域污染变量。
- **中文注释**：根据全局规范，新增或修改的代码必须添加全中文注释，说明**What (该行做什么)** 和 **Why (为什么这么实现/这么调参)**。
