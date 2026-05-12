qv# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RoboMaster robot control firmware for STM32 (F4/H7), built with CMake + arm-none-eabi-gcc cross-compiler. Uses FreeRTOS for task scheduling and the HAL library for hardware access.

## Build Commands

```bash
# Configure (from project root, using Ninja generator)
cmake -G Ninja -B cmake-build-debug -DbuildType=Debug

# Build
cmake --build cmake-build-debug

# Or with CMake presets (for STM32 builds)
cmake --build cmake-build-debug-stm32
```

Build outputs: `.elf`, `.hex`, `.bin` in the build directory.

CI/build configuration is in `.vscode/settings.json` — Ninja generator, build directory `cmake-build-debug`, `buildType=Debug`.

## Robot & MCU Selection

Set in `CMakeLists.txt`:

- **`MCU_TYPE`**: `stm32-f4` or `stm32-h7`
- **`ROBOT_TYPE`**: `infantry_steering_gimbal`, `infantry_mecanum`, `infantry_steering`, `infantry_wheel_legged`, `infantry_wheel_legged_demo`, `hero_mecanum`, `sentry_mecanum`, `engineering`, `dart`, `debug`

Each robot type has its own directory under `UserApp/robot/<ROBOT_TYPE>/` containing `robot.c`, `robot.h`, `robot.cmake`, and `robot_config.h`.

The `robot.cmake` for each robot selects which component implementations to compile:
```cmake
set(CHASSIS_TYPE chassis_steering)
set(GIMBAL_TYPE gimbal_standard)
set(SHOOT_TYPE shoot_standard)
```

## Architecture

Three-layer design:

### Hardware (`Hardware/`)
CMSIS + STM32 HAL drivers, startup assembly, linker scripts. Per-MCU subdirectory (`stm32-f4/`, `stm32-h7/`). Each has its own `.cmake` file setting `MCU_FLAGS`, `LINKER_SCRIPT`, `DSP_NAME`, and `ASM_SOURCES`.

### Bsp — Board Support Package (`Bsp/`)
Thin wrappers over HAL peripherals: `can`, `dwt` (cycle timer), `flash`, `gpio`, `iic`, `log`, `pwm`, `spi`, `usart`, `usb`. `BSPInit()` in `bsp_init.h` initializes DWT and logging — called before RTOS starts. Other peripherals are initialized lazily when their instance is first used.

### Modules (`Modules/`)
Device-level drivers and utilities:
- `motor/` — DJI motor drivers (M3508, M2006, GM6020) + DM motors + servo motors. `motor_task.c` is the RTOS motor control task.
- `imu/` — BMI088 IMU driver + INS attitude estimation (`ins_task.c`)
- `algorithm/` — PID controllers, Kalman filter, QuaternionEKF, CRC, user math libs
- `can_comm/` — Inter-board CAN communication for dual-board setups
- `remote/` — Remote control (DJI DR16 DBUS protocol) + VT13 new remote
- `referee/` — RoboMaster referee system protocol
- `master_machine/` — Communication with upper computer (Seasky/SRM protocol, navigator)

- `daemon/` — Watchdog/monitoring task
- `super_cap/` — Super capacitor power management
- `alarm/` — Buzzer

### UserApp (`UserApp/`)
Robot application layer, organized as:

- **`os_task.c`** — FreeRTOS task creation (MotorTask at highest priority for 1kHz IMU, DaemonTask at 100Hz, RobotTask at 200-500Hz, optional UITask)
- **`components/`** — Reusable robot components, each with per-kinematics implementations:
  - `chassis/` — `chassis_mecanum/`, `chassis_steering/`, `chassis_rabbit/`, `chassis_wheel_legged/`
  - `gimbal/` — `gimbal_standard/`, `gimbal_steering_infantry/`
  - `shoot/` — `shoot_standard/`
  - `gantry/`, `grab/`
- **`robot/<ROBOT_TYPE>/`** — Per-robot wiring: `robot_config.h` (motor CAN IDs, PID tunings, mechanical parameters), `robot.c` (init and main control loop orchestrating all components), `robot.h` (shared types like `RobotInstance`, `CanComm_Pack`)

## Dual-Board Architecture

The system supports single-board and dual-board configurations via compile-time defines:
- `ONE_BOARD` — All control on one board
- `GIMBAL_BOARD` — Yaw board (gimbal + shoot + vision/remote input)
- `CHASSIS_BOARD` — Chassis board (chassis motors)

Boards communicate via CAN using `can_comm` module with `CanComm_Pack` structs defined per robot. The `#define` for board role is set near the top of `robot.h`.

## Code Style

- **Naming**: `lower_case` for variables, `CamelCase` for functions/structs/enums, `k` prefix for constants
- **Format**: Clang-format with `BasedOnStyle: Microsoft`, format-on-save disabled for C files (manual formatting preferred)
- **Clang-tidy**: `performance-*`, `modernize-*`, `clang-analyzer-*`, `readability-*` enabled (no trailing return type, no nodiscard, no avoid-c-arrays)
- `.h` files are associated as C (not C++), enforced in `.vscode/settings.json`

## Key Patterns


- **Motor abstraction**: All motors use a unified `MotorInstance` with cascaded PID controllers (`controller.c`). Configuration uses designated initializers (`Motor_Init_Config_s`).
- **Robot control flow**: `RobotInit()` creates all component instances → `RobotTask()` runs at 200-500Hz calling `RobotCMDTask()` (remote processing + emergency handling) → `GimbalTask()` → `ShootTask()` → CAN send, then `ChassisTask()`.
- **Emergency stop**: Both remote switches down = power off all actuators. `EmergencyHandler()` in `robot.c`.
- **Memory**: Use `zmalloc()` (FreeRTOS wrapper, zero-initialized) for dynamic allocations during init. Static variables for per-task intermediates to avoid stack allocation overhead.
