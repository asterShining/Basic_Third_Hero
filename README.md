# Basic Third Hero

## 项目定位

本仓库是第三代英雄机器人双板控制工程，当前主分支已经合并 `5m_v2` 分支内容。工程以 `STM32F407 + FreeRTOS + CMake + arm-none-eabi-gcc` 为主要工具链，按 `chassis` 底盘板和 `gimbal` 云台板拆成两套可独立编译、独立烧录的固件。

当前版本的核心目标是把英雄机器人场上实战链路收口到稳定的双板架构：云台板负责遥控器、VT03 图传键鼠、云台和发射机构；底盘板负责麦轮底盘、功率控制、超电、裁判系统 UI 和双板反馈。两块板之间通过 `CANComm` 交换 `Chassis_Ctrl_Cmd_s` 与 `Chassis_Upload_Data_s`，避免输入源、执行模块和 UI 状态互相散落。

## 分支合并总结

本次将 `5m_v2` 合入 `main` 后，主分支获得了从早期发射机构、视觉通信、5m 场地逻辑、VT03 接入，到后期力控、图传电机、自定义图像桥接和实战 UI 的完整演进。

主要变化包括：

- 云台控制拆分为输入仲裁、运行态、前馈力控和 pitch 标定等模块，降低 `gimbal.c` 单文件负担。
- 发射机构拆分为摩擦轮控制、单发状态机、堵转处理和调试输出，支持内外圈六摩擦轮和固定角度单发。
- 底盘控制拆分为跟随、小陀螺、运动学、UI 数据聚合和上岛辅助机构，保留功率控制与坡道补偿链路。
- 双板协议删除旧 `master_machine` 目录，改为由 `robot_def.h` 中的公共控制/反馈结构体表达实际需要的数据。
- 新增 VT03 图传键鼠解析、自定义图像桥接、VOFA 调试、pitch 标定导出工具和多份联调说明文档。

## 目录结构

```text
.
├── chassis/                         # 底盘板固件工程
│   ├── application/chassis/          # 底盘应用层：跟随、小陀螺、运动学、UI 聚合、上岛辅助
│   ├── modules/referee/              # 裁判系统解析与 UI 绘制
│   ├── modules/motor/                # DJI / DM / HT / 舵机 / 步进电机与功率控制
│   ├── modules/super_cap/            # 超级电容 CAN 协议与输出状态
│   └── CMakeLists.txt                # 底盘板 CMake 入口
├── gimbal/                           # 云台板固件工程
│   ├── application/cmd/              # 遥控器、VT03、键鼠、云台、底盘和发射命令仲裁
│   ├── application/gimbal/           # 云台 PID、力控前馈、运行态和 pitch 标定
│   ├── application/shoot/            # 发射机构、摩擦轮、拨盘、单发和堵转处理
│   ├── modules/custom_image_bridge/  # USB CDC 到裁判 0x0310 的 H.264 透传桥
│   ├── modules/video_link/           # VT03 键鼠与图传电机链路
│   ├── modules/vofa_debug/           # VOFA+ 调试数据输出
│   └── CMakeLists.txt                # 云台板 CMake 入口
└── tools/
    └── export_pitch_cali_from_rtt.py # 从 RTT 日志导出 pitch 标定数据
```

## 双板职责

### gimbal 云台板

云台板当前定义为 `GIMBAL_BOARD`，入口在 `gimbal/application/robot.c`。初始化顺序为 `BSPInit()`、`RobotCMDInit()`、`CustomImageBridgeInit()`、`GimbalInit()`、`ShootInit()`，随后由 FreeRTOS 拉起 INS、电机、daemon、自定义图像桥接和机器人核心任务。

云台板主要负责：

- 解析 DT7 遥控器和 VT03 图传键鼠输入。
- 在 `robot_cmd` 中统一处理控制源切换、零力、校零、摩擦轮锁存、弹速切换、小陀螺、一键掉头和 UI 刷新请求。
- 通过消息中心向 `gimbal`、`shoot` 发布控制命令，并通过 `CANComm` 向底盘板发送底盘命令。
- 执行云台 yaw / pitch 控制，包含重力、惯量、摩擦、离心和科氏耦合等前馈项。
- 执行发射机构控制，包含六摩擦轮、拨盘位置环单发、掉速检测、堵转恢复和 VOFA 调试。
- 将 USB CDC 输入的 raw H.264 Annex-B 字节流切成裁判系统 `0x0310` 的 300B payload，经 USART6 转发给 VT03 / 裁判链路。

### chassis 底盘板

底盘板当前定义为 `CHASSIS_BOARD`，入口在 `chassis/application/robot.c`。初始化顺序为 `BSPInit()`、`ChassisInit()`，随后创建 INS、电机、daemon、机器人核心和裁判 UI 任务。

底盘板主要负责：

- 通过 `CANComm` 接收云台板下发的底盘控制命令，并上传底盘状态、功率和 UI 所需数据。
- 执行麦克纳姆轮运动学、底盘跟随、小陀螺、跟随接管刹停和输出限斜率。
- 结合裁判系统功率限制、缓冲能量和超电回传，执行底盘功率预算与输出裁剪。
- 根据底盘 pitch 姿态执行坡道前馈，并在启停、急停和换向时提供速度 PID 力控前馈。
- 维护裁判系统 UI，包括 pitch 滑块、摩擦轮状态、弹速档位、功率/超电状态和手动刷新。
- 在启用 `USE_ISLAND_ACTION` 后挂载前履带和后抬升辅助机构。

## 关键功能

### 输入与模式仲裁

`gimbal/application/cmd` 是整车控制源的唯一收口点。当前输入源按 VT03、DT7 和离线零力进行仲裁，避免云台、底盘和发射机构各自直接读取遥控器或键鼠。

常用操作语义：

- VT03 首次接管默认保持零力，需要用户显式解除。
- VT03 `Pause` 短按用于零力锁存，零力态长按用于 yaw 校零。
- 键鼠 `F` 切换摩擦轮，`R` 切换 12 / 16m/s 弹速预选，鼠标左键按上升沿触发单发。
- 键鼠 `X` 切换小陀螺，`B` 切换云台自由模式，`V` 执行一键掉头，`G` 请求底盘 UI 整页刷新。

### 云台力控与 pitch 标定

云台执行层继续只消费 `Gimbal_Ctrl_Cmd_s`，不直接读取输入源。`gimbal_force_control_tuning.md` 记录了 pitch / yaw 前馈参数、PID 参数和推荐调参顺序；`tools/export_pitch_cali_from_rtt.py` 用于从 RTT 日志导出 pitch 标定数据。

当前 pitch 控制特别注意软件限位和速度目标限幅。输入层会在 `PITCH_MIN_ANGLE`、`PITCH_MAX_ANGLE`、`PITCH_SPEED_REF_MAX_DPS` 内统一裁剪，避免不同输入源绕过机构安全边界。

### 发射机构

发射模块位于 `gimbal/application/shoot`，当前以六摩擦轮和拨盘位置环为核心。摩擦轮支持内外圈目标速度、软启动 ramp、前馈和掉速调试；拨盘使用输出端固定角度推进，单发逻辑通过掉速计数与锁角保持解耦，减少持续掉速导致重复计数的问题。

调试相关数据集中在 `shoot_debug` 与 `vofa_debug`，方便 Ozone 和 VOFA+ 同时观察摩擦轮速度、掉速基线、单发状态机和堵转状态。

### 底盘功率、小陀螺与超电

底盘模块位于 `chassis/application/chassis`，当前把跟随控制、小陀螺、运动学和 UI 数据更新拆成多个私有编译单元。小陀螺围绕裁判功率目标做自适应转速，并保留无节奏变速，平移优先级用于避免横移时整车被旋转功率拖死。

超电链路位于 `chassis/modules/super_cap`，底盘策略会结合裁判功率、缓冲能量、超电电量和超电板回传功率上限计算额外预算。DCDC 请求带最小开关保持时间和故障方波重试逻辑，避免临界状态频繁开关。

### 裁判系统与自定义图像

底盘板维护裁判系统解析和 UI 绘制，云台板维护自定义图像桥接。图像桥接固定为：

```text
上位机 raw H.264
-> USB CDC
-> gimbal custom_image_bridge
-> USART6 / VT03
-> 裁判系统 0x0310
-> 客户端 CustomByteBlock
```

该链路不解析 JPEG，不参与键鼠/遥控仲裁，也不修改云台、发射和底盘控制频率。上位机必须输出 `serial.transport_mode=raw_h264`，默认建议 `921600 8N1`、`192x192`、`18fps`、`112kbps`。

## 构建方式

环境要求：

- CMake 3.22 或更高版本
- `arm-none-eabi-gcc`
- Ninja 或 Make

底盘板：

```bash
cmake -S chassis -B build/chassis -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/chassis
```

云台板：

```bash
cmake -S gimbal -B build/gimbal -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/gimbal
```

如果只需要生成 `compile_commands.json` 给 clangd 使用，也可以只执行 configure 阶段。当前两个子工程都已经在 `CMakeLists.txt` 中开启 `CMAKE_EXPORT_COMPILE_COMMANDS`。

## 联调检查点

1. 烧录前确认 `robot_def.h` 里只启用一个板级宏：底盘板启用 `CHASSIS_BOARD`，云台板启用 `GIMBAL_BOARD`。
2. 检查双板 `CANComm` 的 `tx_id / rx_id` 是否互补，且 `Chassis_Ctrl_Cmd_s` 与 `Chassis_Upload_Data_s` 未超过 `CAN_COMM_MAX_BUFFSIZE`。
3. 云台板先看 DT7 / VT03 是否在线，再看 `robot_cmd` 是否正确进入零力、陀螺仪、自由模式或小陀螺模式。
4. 发射机构先空载验证摩擦轮目标速度、拨盘初始位锁存和单发状态机，再装弹验证掉速计数。
5. 底盘板先断开超电按默认裁判功率验证麦轮方向，再接入裁判系统和超电验证功率预算。
6. 裁判 UI 优先验证整页刷新、pitch 滑块、摩擦轮状态、弹速档位和超电状态是否随真实数据变化。
7. 自定义图像桥接联调时观察 `[img_bridge]` 日志，重点看 `usb_pkt_total`、`tx0310_total`、`pending_drop` 和 `uart_tx_error_count`。

## 重要文档

- [gimbal/遥控器与电脑操控接入迁移说明.md](gimbal/遥控器与电脑操控接入迁移说明.md)
- [gimbal/自定义图像桥接接入说明.md](gimbal/自定义图像桥接接入说明.md)
- [gimbal/application/gimbal/gimbal_force_control_tuning.md](gimbal/application/gimbal/gimbal_force_control_tuning.md)
- [chassis/通信协议.md](chassis/通信协议.md)
- [chassis/application/chassis/chassis.md](chassis/application/chassis/chassis.md)
- [gimbal/application/shoot/shoot.md](gimbal/application/shoot/shoot.md)

## 已知风险与待验证项

- 本次合并覆盖历史提交较多，虽然 Git 合并无冲突，但仍需要在真实双板硬件上验证 CANComm、裁判系统、超电和 VT03 链路。
- `USE_ISLAND_ACTION` 当前默认未打开；启用后必须重新确认前履带、抬升电机 CAN ID、限位和机械行程。
- 云台 pitch、yaw 的力控参数和软件限位与当前机械装配强相关，更换机构或重装 IMU 后必须重新标定。
- 自定义图像链路依赖上位机 raw H.264 输出和客户端 H.264 重组逻辑，任一端切回旧 JPEG 或 inner packet 协议都会失效。
- 仓库内包含 `repomix-output.xml`、`.omc`、`.omg` 等辅助生成文件，它们来自被合并分支；若后续要清理仓库体积，应单独开清理提交，避免和功能合并混在一起。
