# Repository Guidelines

## Project Structure & Module Organization

This is an STM32 RoboMaster firmware project written in C11 and built with CMake for `arm-none-eabi-gcc`. Keep hardware-specific code in `Hardware/stm32-f4` or `Hardware/stm32-h7`; board/peripheral drivers belong in `Bsp/` (CAN, GPIO, SPI, USART, etc.). Reusable device, communication, and control logic lives in `Modules/`. Robot-level behavior and FreeRTOS task integration live in `UserApp/`, with each robot variant under `UserApp/robot/<robot_name>/`. Reference material is in `Doc/`; generated build outputs belong in `cmake-build-*` and must not be committed.

## Build, Test, and Development Commands

Configure with Ninja and select the target through CMake cache variables:

```sh
cmake -S . -B cmake-build-debug -G Ninja -DMCU_TYPE=stm32-f4 -DCMAKE_BUILD_TYPE=Debug
cmake --build cmake-build-debug
```

The build produces `control-2026.elf`, `.bin`, `.hex`, and `.map` in the build directory. The default robot is selected by `ROBOT_TYPE` in `CMakeLists.txt`; change it deliberately and ensure the matching `UserApp/robot/<name>/robot.cmake` exists. Use the OpenOCD launch/debug configurations in `.vscode/` or scripts in `Debug/gbd_server/` when flashing hardware.

## Coding Style & Naming Conventions

Format C and header files with the repository `.clang-format` (Google base style, 120-column limit). Use 4-space indentation and braces consistent with surrounding code. Name files by layer and function: BSP files use `bsp_<peripheral>.c/.h`; modules use descriptive lowercase names; robot configurations use `robot_config.h`. Keep public declarations in headers, include the corresponding header first where practical, and avoid cross-layer dependencies. Applications should exchange data through `Modules/message_center` rather than directly including peer application internals.

## Testing Guidelines

There is currently no host-side automated test suite. Treat a clean cross-compile as the required baseline, then test changed behavior on the intended MCU/robot configuration. For control, CAN, or task changes, record the board type, robot variant, task rate, and observed result in the PR. Do not add generated artifacts as test evidence.

## Commit & Pull Request Guidelines

Recent history uses concise Conventional Commit-style prefixes, often with Chinese scopes, e.g. `feat(底盘控制): 增加补偿` or `fix: 修复双板通信`. Use `feat`, `fix`, `refactor`, `chore`, `test`, or `wip` where appropriate. PRs should state the selected MCU and robot target, summarize hardware-impacting changes, link relevant issues, and include build output or logs; attach screenshots only for debugger/UI configuration changes.
