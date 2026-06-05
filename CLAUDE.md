# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Flash

Each sub-project (`chassis/`, `gimbal/`) is an independent CMake project targeting STM32F407xx (Cortex-M4, hard float). Build from within the sub-project directory.

```bash
# Configure and build (chassis or gimbal)
cd chassis   # or gimbal
mkdir -p build && cd build
cmake -G Ninja -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug ..
ninja -j$(nproc)

# Or use the run.sh wrapper which auto-detects config changes:
./run.sh

# Flash via J-Link
./jlink.sh
```

**Build presets** (for VSCode CMake Tools): `Debug`, `Release`, `RelWithDebInfo`, `MinSizeRel`. The `Debug` preset uses `-Og -g -gdwarf-2`; `Release` uses `-Ofast -flto`.

**Prerequisites**: `arm-none-eabi-gcc` on PATH, Ninja, J-Link tools.

## Architecture

### Dual-Board Design

- **`chassis/`** — Chassis MCU: motion control (Mecanum wheel kinematics), super capacitor, referee system communication. Defined by `CHASSIS_BOARD` macro.
- **`gimbal/`** — Gimbal MCU: pitch/yaw control, shoot control (friction wheels + loader), referee system, vision communication, remote control input. Defined by `GIMBAL_BOARD` macro.
- **`ONE_BOARD`** — Single-board mode controlling both chassis and gimbal from one MCU.

The board type is configured by exactly one `#define` in `application/robot_def.h` — the other two must be commented out.

### Layered Architecture (bottom-up)

1. **BSP** (`bsp/`) — Hardware abstraction: CAN, USART, ADC, I²C, SPI, PWM, GPIO, DWT (delay timer), flash, logging. `bsp_init.h` provides the single entry point `BSPInit()`.

2. **Modules** (`modules/`) — Reusable hardware-agnostic components:
   - **`message_center/`** — Pub-sub message bus used for all inter-module communication (no global variables). Subscribe/publish by topic name.
   - **`motor/`** — Motor drivers: DJI (M3508, M2006), DM, step/servo motors.
   - **`imu/`**, `BMI088/`, `ist8310/` — IMU sensor fusion (EKF in `ins_task`).
   - **`referee/`** — RoboMaster referee system protocol.
   - **`can_comm/`** — CAN communication layer between boards.
   - **`daemon/`** — Watchdog/health monitoring tasks.
   - **`remote/`**, `standard_cmd/` — Remote control and command protocols.
   - **`super_cap/`** — Super capacitor power management.
   - **`algorithm/`** — Math/control algorithms (filters, PID, ramp).
   - **Gimbal-specific**: `vofa_debug/` (Vofa+ graphing), `video_link/` (video feed), `custom_image_bridge/` (H.264 stream bypass).

3. **Application** (`application/`) — Top-level robot behaviors, parallel tasks communicating via pub-sub:
   - **`robot_cmd/`** — The "brain": ingests raw control inputs (remote, vision, keymouse) and produces quantitative control targets for other apps.
   - **`gimbal/`** — Pitch/yaw axis control with PID + feedforward.
   - **`shoot/`** — Friction wheel and loader (bullet feeder) control with dip-lock firing strategy.
   - **`chassis/`** — Mecanum kinematics, power limiting, UI display.

4. **Middleware** (`Middlewares/`) — Third-party: FreeRTOS, SEGGER RTT.

### Entry Point Flow

`main()` → `RobotInit()` → `BSPInit()` → init each app's `*Init()` → `OSTaskInit()` (creates FreeRTOS tasks) → RTOS scheduler starts.
`RobotTask()` is called periodically (by a FreeRTOS task) and dispatches to each app's `*Task()` function.

### Inter-App Communication

Applications are **parallel and decoupled** — they never include each other's headers. To share data, an app **publishes** a topic to `message_center`; interested apps **subscribe** to that topic. Shared structs (with `#pragma pack(1)`) are defined in `robot_def.h`.

## Key Configuration File

`application/robot_def.h` is the single robot configuration file:
- Board type: `CHASSIS_BOARD` / `GIMBAL_BOARD` / `ONE_BOARD`
- Robot dimensions: wheel base, track width, wheel radius, reduction ratios, CoG
- Gimbal limits: pitch min/max angles, encoder offsets
- Shooter: bullet delta angle, loader reduction ratio
- Feature flags: `USE_SUPER_CAP`, `USE_ISLAND_ACTION`
- Physical model: mass, CoG height (for power/ramp compensation)

**The two `robot_def.h` files (chassis vs gimbal) differ** — the gimbal copy has IMU axis remapping (`GIMBAL_PITCH_AXIS`, etc.) and video_link defines that the chassis copy lacks.

## Coding Conventions

- **Language standard**: C11 (`gnu11`)
- **Comments**: All new/modified code requires detailed Chinese comments. Prohibited: `What:`/`Why:` label-style comments. Required: natural engineering Chinese explaining business intent, design rationale, boundary conditions.
- **Style**: 4-space indent, `AfterFunction: true` brace wrapping, see `.clang-format` for full rules.
- **DRY + minimal changes**: Reuse existing BSP/module functions. Only touch the minimum lines needed.
- **Data alignment**: Shared structs use `#pragma pack(1)` to prevent unaligned access faults.

## Debugging

- Debug output via **SEGGER RTT** (not serial) — `Middlewares/Third_Party/SEGGER/RTT/`
- Visual monitoring via **Vofa+** (gimbal only, `modules/vofa_debug/`)
- For graphical debugging: **Ozone** debugger with the built ELF file
- `tools/export_pitch_cali_from_rtt.py` — Extract pitch calibration data from RTT logs
