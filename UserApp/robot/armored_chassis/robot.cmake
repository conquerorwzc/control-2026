# 重装底盘使用 STM32H7 主控，由顶层工程加载本文件时选择对应的板级配置。
set(MCU_TYPE "stm32-h7")

# 让框架找到本车型的 robot.h，并将本车型的 robot.c 加入构建。
include_directories(${CMAKE_CURRENT_LIST_DIR})
list(APPEND SOURCES "${CMAKE_CURRENT_LIST_DIR}/robot.c")