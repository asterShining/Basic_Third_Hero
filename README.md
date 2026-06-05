# RoboMaster 英雄机器人双板控制系统

本项目是面向 RoboMaster 英雄机器人的嵌入式控制工程，基于 `STM32F407`、`FreeRTOS`、`CMake` 和 `arm-none-eabi-gcc` 构建。系统采用云台板与底盘板分布式架构，将输入仲裁、云台控制、发射机构、底盘运动、功率管理、裁判系统交互和图传链路拆分为相对独立的模块，并配套跨平台编译、J-Link 烧录和 Ozone 调试流程，适合实车调试、快速迭代和比赛场景下的功能扩展。

## 系统架构

工程分为 `gimbal` 和 `chassis` 两个子工程，分别对应云台板和底盘板。两块控制板通过 CAN 总线进行双板通信：云台板统一接收操作输入并下发底盘控制命令，底盘板回传运动状态、功率状态和裁判系统数据，形成完整的闭环控制链路。

```text
        DT7 / VT03 / 键鼠 / 上位机
                    |
                    v
              gimbal 云台板
   输入仲裁 -> 云台控制 -> 发射机构 -> 图像桥接
                    |
                 CANComm
                    |
                    v
              chassis 底盘板
   麦轮运动 -> 功率控制 -> 超级电容 -> 裁判系统 UI
```

整体设计遵循“输入集中仲裁、执行模块只消费命令、反馈统一回传”的原则。`robot_cmd` 负责整车控制意图的生成，云台、发射和底盘模块只处理各自执行层逻辑，避免遥控器、键鼠、视觉和调试链路直接侵入底层控制代码。

## 主要模块

### 云台板 `gimbal`

云台板负责整车上层控制入口和英雄机器人核心上装执行机构。

- `application/cmd`：统一处理 DT7、VT03 图传键鼠和离线保护逻辑，完成控制源切换、零力保护、校零、小陀螺、一键掉头、摩擦轮锁存和弹速档位选择。
- `application/gimbal`：实现 yaw / pitch 控制，包含串级 PID、姿态反馈、软件限位、pitch 标定和基于模型的前馈补偿。
- `application/shoot`：实现英雄发射机构控制，包含六摩擦轮调速、拨盘位置环、单发状态机、掉速检测、堵转恢复和调试数据输出。
- `modules/video_link`：接入 VT03 图传链路，解析键鼠与自定义控制数据。
- `modules/custom_image_bridge`：将上位机 raw H.264 数据通过 USB CDC 接入，并按裁判系统 `0x0310` 自定义数据链路转发。

### 底盘板 `chassis`

底盘板负责移动底盘、功率约束和裁判系统显示链路。

- `application/chassis`：实现麦克纳姆轮运动学、底盘跟随、小陀螺、跟随接管、坡道补偿和底盘状态聚合。
- `modules/motor`：封装 DJI、DM、HT、舵机等电机驱动，并提供底盘功率控制入口。
- `modules/super_cap`：接入超级电容模块，根据裁判功率限制、缓冲能量和电容状态动态调整底盘输出预算。
- `modules/referee`：解析裁判系统数据并绘制客户端 UI，显示云台 pitch、摩擦轮状态、弹速档位、功率和超电状态。
- `application/chassis/island_action`：预留上岛辅助机构控制逻辑，支持前履带和后抬升机构的条件编译接入。

## 技术特点

- **双板分布式控制**：云台板和底盘板职责清晰，通过 `CANComm` 传输结构化命令与反馈，降低模块耦合。
- **多输入源统一仲裁**：DT7、VT03、键鼠和离线状态统一进入 `robot_cmd`，执行层不直接读取输入设备。
- **云台模型前馈控制**：在 PID 闭环外叠加重力、惯量、摩擦、离心和耦合项补偿，提高云台大角度和动态动作下的响应一致性。
- **英雄发射机构状态机**：发射模块将摩擦轮稳速、拨盘送弹、单发计数、掉速判断和堵转恢复拆分处理，提升调试可观测性。
- **底盘功率闭环约束**：结合裁判系统、超级电容和底盘运动状态进行功率预算分配，兼顾小陀螺、平移手感和功率安全。
- **裁判系统与图传扩展**：支持客户端 UI 绘制和 `0x0310` 自定义图像数据桥接，为操作手提供更完整的场上信息。
- **跨平台开发调试链**：基于 CMake 和 Arm GNU Toolchain 在 Windows / Linux 环境下统一构建，通过 J-Link 脚本烧录固件，并配合 Ozone 进行断点调试、变量观察、曲线分析和 RTT 日志查看。

## 工程结构

```text
.
├── chassis/                         # 底盘板工程
│   ├── application/chassis/          # 底盘应用层
│   ├── modules/motor/                # 电机与功率控制
│   ├── modules/referee/              # 裁判系统与 UI
│   ├── modules/super_cap/            # 超级电容模块
│   └── CMakeLists.txt
├── gimbal/                           # 云台板工程
│   ├── application/cmd/              # 输入仲裁与整车命令生成
│   ├── application/gimbal/           # 云台控制
│   ├── application/shoot/            # 发射机构
│   ├── modules/custom_image_bridge/  # 自定义图像桥接
│   ├── modules/video_link/           # VT03 图传链路
│   └── CMakeLists.txt
└── tools/                            # 调试和数据处理脚本
```

## 构建方式

工程采用 CMake 管理底盘板和云台板两个固件子工程，配合 `arm-none-eabi-gcc` 完成交叉编译。烧录脚本会根据运行环境选择 `JLink.exe` 或 `JLinkExe`，便于在 Windows、Linux 或类 Unix 终端中复用同一套流程。

需要安装：

- `CMake >= 3.22`
- `arm-none-eabi-gcc`
- `Ninja` 或 `Make`
- `SEGGER J-Link` 工具链
- `SEGGER Ozone` 调试器

构建底盘板：

```bash
cmake -S chassis -B build/chassis -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/chassis
```

构建云台板：

```bash
cmake -S gimbal -B build/gimbal -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/gimbal
```

烧录时进入对应子工程目录，执行项目内置的 J-Link 脚本。脚本默认使用 `STM32F407IGHx`、`SWD` 和 `4000 kHz` 连接参数，并将 `build/basic_framework.elf` 写入目标板。

```bash
cd chassis
./jlink.sh

cd ../gimbal
./jlink.sh
```

调试时使用 Ozone 加载对应子工程生成的 `basic_framework.elf`，结合 J-Link / SWD 连接目标板，可直接观察任务运行状态、控制变量、摩擦轮调试结构体、云台姿态反馈和 RTT 日志。工程内保留了 `VSCode+Ozone使用方法.md`，用于说明完整的编辑、编译、烧录和可视化调试工作流。

## 项目亮点

该工程不是单一功能 demo，而是一套围绕真实英雄机器人实车调试形成的完整控制系统。项目覆盖遥控输入、云台姿态、发射机构、底盘运动、裁判系统、超级电容、图传扩展和调试工具链等多个 RoboMaster 关键子系统，重点解决多控制源协同、双板通信、执行层解耦、功率受限运动控制、跨平台构建烧录和比赛可视化信息反馈等实际工程问题。
