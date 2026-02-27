# Repository Guidelines

## Project Structure & Module Organization
- This repository contains two STM32 firmware targets: `chassis/` and `gimbal/` (each is a standalone CMake project).
- In each target:
  - `application/`: robot behavior and task-level logic
  - `modules/`: hardware-agnostic drivers/algorithms
  - `bsp/`: board support wrappers over STM32 HAL
  - `Src/`, `Inc/`, `Drivers/`, `Middlewares/`: CubeMX-generated and vendor code
- Prefer changing `application/`, `modules/`, and `bsp/`. In generated files (for example `Src/main.c`, `Src/freertos.c`), only edit inside `/* USER CODE BEGIN */` blocks.
- Design and usage references are in `.Doc/` and per-component markdown files (for example `bsp/*.md`).

## Build, Test, and Development Commands
- Prerequisites: `arm-none-eabi-gcc`, `cmake`, `ninja`, and optional `JLinkExe` for flashing.
- Build with helper script (from target directory):
  - `./run.sh`
- Build with CMake presets:
  - `cmake --preset Debug`
  - `cmake --build --preset Debug`
- Flash to board (from target directory):
  - `./jlink.sh`
- Build artifacts are generated under `build/` (for example `build/basic_framework.elf`).

## Coding Style & Naming Conventions
- Language standard: C11/`gnu11`; format with repository `.clang-format` (4-space indentation, no tabs).
- Function naming: PascalCase verb phrases (example: `SetMotorControl`).
- Variable naming: lowercase snake_case (example: `gimbal_recv_cmd`).
- Type naming: use `_t` for simple data types and `_s` for richer config/structure types.
- Keep HAL calls in `bsp/`; upper layers should use BSP/module interfaces.
- When adding or changing a module/BSP, update its matching `.md` usage documentation.

## Testing Guidelines
- No dedicated unit-test framework is currently configured.
- Minimum validation: clean Debug builds for both `chassis/` and `gimbal/`.
- Required on-target checks: startup, key task scheduling, and affected communication paths (CAN/UART/USB as applicable).
- Record board mode and macros used in validation (`ONE_BOARD`, `GIMBAL_BOARD`, `CHASSIS_BOARD`).

## Commit & Pull Request Guidelines
- Existing history favors short Chinese summaries; merge commits and “待测试” notes are common.
- Recommended commit style: `<scope>: <summary>` plus optional test state.
  - Example: `gimbal: 调整pitch前馈限幅（待实车测试）`
- PRs should include:
  - changed directories/modules
  - hardware setup and board mode
  - verification steps/results
  - protocol/parameter impacts and follow-up risks

## Safety & Configuration Tips
- Do not add blocking delays in critical sections/interrupt-disabled regions; prefer DWT-based timing utilities.
- For dual-board operation, check CAN ID conflicts and bus load before field testing.
