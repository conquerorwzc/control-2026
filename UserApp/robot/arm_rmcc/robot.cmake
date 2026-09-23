# 只编译本目录：arm_rmcc 不需要 components/ 下的任何组件。
#
# 为什么不复用 gimbal_standard：它的语义是"把 IMU 稳定在目标姿态"（内部会
# 调 INS_Init()），并且用常规级联闭环维持姿态 —— 而场地臂要能被参赛者的
# 机械臂反向推动，闭环维持姿态正好与需求相反。复用它会逼着改 module，
# 而本机器人被要求不修改其它层。
#
# RMCC 是唯一开发源：c/*.c/h 先在 RMCC 里改并过测试，再拷进本目录。

# ---- MCU 选择 ----------------------------------------------------------
# 顶层 CMakeLists 在本文件之前已经 set(MCU_TYPE "stm32-f4")，所以必须在这里
# 覆盖 —— 本仓库里多板型机器人（infantry_wheel_legged_sjtu）就是这么做的。
#
# 本机器人只用 H7，没有多板型需求，所以 BOARD_TYPE 只作为外部覆写口存在，
# 默认即 ONE_BOARD，写法与既有机器人保持一致：
#     cmake ... -DBOARD_TYPE=ONE_BOARD
if (NOT DEFINED BOARD_TYPE)
    set(BOARD_TYPE "ONE_BOARD")
endif ()

if (BOARD_TYPE STREQUAL "ONE_BOARD")
    set(MCU_TYPE "stm32-h7")
else ()
    message(FATAL_ERROR
        "Unknown BOARD_TYPE '${BOARD_TYPE}' (arm_rmcc 只支持 ONE_BOARD / stm32-h7)")
endif ()

add_compile_definitions(${BOARD_TYPE})

# ---- 平台断言 ----------------------------------------------------------
# arm_rmcc 依赖 H7 的两个条件：DM-J4310 走 FDCAN（hcan1..3 由 bsp_can.h 映射
# 到 hfdcan1..3），J-Link VCOM 走 USART1（H7 上唯一已初始化的 8N1 串口）。
if (NOT "${MCU_TYPE}" STREQUAL "stm32-h7")
    message(FATAL_ERROR
        "arm_rmcc 需要 MCU_TYPE=stm32-h7（当前 '${MCU_TYPE}'）："
        "DM 电机走 FDCAN，数据上报走 USART1，F4 上都不成立。")
endif ()

include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})

file(GLOB ROBOT_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
)

list(APPEND SOURCES ${ROBOT_SOURCES})
