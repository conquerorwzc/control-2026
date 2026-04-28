# AGENTS.md — control-2026

## Build

Cross-compile for STM32 (arm-none-eabi-gcc, C11, CMake 3.16+). Ninja recommended.

```bash
cmake -DbuildType=Release -G Ninja -B cmake-build-debug-stm32
cmake --build cmake-build-debug-stm32
```

Outputs: `.elf`, `.hex`, `.bin` in the build directory.

### Selecting robot and MCU

Edit `CMakeLists.txt` (lines 23–37). Uncomment exactly one `MCU_TYPE` and one `ROBOT_TYPE`:

```cmake
set(MCU_TYPE "stm32-f4")           # or stm32-h7
set(ROBOT_TYPE "sentry_omni_chassis")  # ~13 options listed in comments
```

- `MCU_TYPE` picks the toolchain cmake (`Hardware/<type>/<type>.cmake`) and linker script.
- `ROBOT_TYPE` sets the `ROBOT_TYPE_<name>` preprocessor define and includes the robot's `robot.cmake` from `UserApp/robot/<type>/`.

### Build types

| Type | Flags | Notes |
|------|-------|-------|
| Release | `-Ofast` | |
| RelWithDebInfo | `-Ofast -g` | |
| MinSizeRel | `-Os` | |
| Debug | `-Og -g -gdwarf-2` | Adds `-DESC_DEBUG` |

## Architecture

### Layers (bottom-up)

- **Hardware/** — MCU-specific: CMSIS, HAL, FreeRTOS port, linker scripts, startup. Per-MCU subdirectory.
- **Bsp/** — On-chip peripheral wrappers (CAN, GPIO, SPI, USART, PWM, I2C, DWT, etc.). Init functions are named `XXXRegister()` (preferred) or `XXXInit()`.
- **Modules/** — Functional modules (motor drivers, IMU, referee system, message_center, daemon, etc.).
- **UserApp/** — Application layer: robot definitions and shared components.
  - `UserApp/robot/<type>/` — Per-robot entry: `robot.c`, `robot.h`, `robot.cmake`, `robot_config.h`.
  - `UserApp/components/` — Reusable subsystems: chassis, gimbal, shoot, gantry, grab.

### Robot directory structure

Each robot has these 4 files:
- **`robot.cmake`** — Sets `CHASSIS_TYPE`, `GIMBAL_TYPE`, `SHOOT_TYPE` and declares sources. Included by root `CMakeLists.txt`.
- **`robot.h`** — `RobotInstance` struct, enums, `RobotInit()` / `RobotTask()` declarations.
- **`robot.c`** — Init and periodic control logic.
- **`robot_config.h`** — All physical parameters, CAN IDs, PID constants, board type macro.

### Instance-based design

Almost every module follows this pattern:
```c
XXX_Init_Config_s config = { ... };
XXXInstance* instance = XXXInit(&config);
// All operations use the instance pointer
XXXDoSomething(instance, ...);
```

### Pub-Sub message center (critical architecture rule)

Inter-module communication **must** use the `message_center` module. Apps are NOT allowed to directly include each other.

- `SubRegister(name, data_len)` → `Subscriber_t*`
- `PubRegister(name, data_len)` → `Publisher_t*`
- `SubGetMessage(sub, data_ptr)` — returns 0 (no new message) or 1
- `PubPushMessage(pub, data_ptr)` — pushes to all subscribers

Topics are C strings (max 32 chars). Max 12 topics, queue depth 1.

### RTOS tasks

Defined in `UserApp/os_task.c`:

| Task | Priority | Stack | Freq | Function |
|------|----------|-------|------|----------|
| motor | BelowNormal | 256 | ~1kHz | `MotorControlTask()` |
| daemon | Normal | 128 | 100Hz | `DaemonTask()` + `BuzzerTask()` |
| robot | Normal | 1024 | ~1kHz | `RobotTask()` |
| UI | Normal | 512 | — | `#if 0` disabled |

Tasks use `DWT_GetTimeline_ms()` to measure latency; overruns logged with `LOGERROR`.

### Board configuration

Exactly one of these must be defined in `robot_config.h`:
- `ONE_BOARD` — single MCU controls entire robot
- `CHASSIS_BOARD` — chassis-side MCU
- `GIMBAL_BOARD` — gimbal-side MCU

Dual-board communication is via CAN (shared bus — watch out for ID conflicts and load).

## Style & naming

- `.clang-format`: Google style, 120 cols, 4-space indent, no tabs
- `.clangd`: ClangTidy with `performance-*`, `modernize-*`, `clang-analyzer-*`, plus identifier naming rules.

### Naming conventions (enforced by ClangTidy — different from typical C)

| Element | Convention | Example |
|---------|-----------|---------|
| Functions, methods | `CamelCase` | `RobotInit()`, `PubPushMessage()` |
| Structs, enums, classes | `CamelCase` | `RobotInstance`, `Robot_Mode_e` |
| Variables, members | `lower_case` | `robot_mode`, `control_mode` |
| Private/protected members | `lower_case` + `_` suffix | `data_`, `count_` |
| Constants | `k` prefix + `CamelCase` | `kMaxSpeed` |
| Namespaces | `lower_case` | |

- Instance type names end with `Instance` or `_t`: `ChassisInstance`, `Subscriber_t`.
- Init config structs end with `_Init_Config_s`.
- Include guards: `#pragma once` preferred.
- CAN communication structs MUST use `#pragma pack(1)` … `#pragma pack()`.

## Conditional compilation patterns

- `#ifdef ROBOT_TYPE_<NAME>` — robot-specific code paths
- `#ifdef STM32F407xx` / `#elifdef STM32H7` — MCU-specific code
- `#ifdef ONE_BOARD` / `#ifdef CHASSIS_BOARD` / `#ifdef GIMBAL_BOARD` — board configuration
- `#ifdef ESC_DEBUG` — debug-only blocks
- `#ifdef USE_DUAL_RC` / `#elifdef USE_DUAL_RC_NEW` — remote type
- `#pragma message(...)` patters used for config-change warnings

## Commit style

Conventional-commits-like prefixes: `[feat]`, `[fix]`, `[perf]`, `[others]`. Chinese in commit messages is normal.

## Documentation

Internal docs are per-layer `.md` files (mostly in Chinese):
- `Bsp/bsp.md` — BSP layer design
- `Modules/module.md` — Module layer notes
- `UserApp/application.md` — Application layer architecture (52 lines, covers pub-sub model, task frequencies, dual-board setup)
- `UserApp/APP层应用编写指引.md` — Chinese app dev guide (70 lines)

No CI, no pre-commit hooks, no other instruction files exist.
