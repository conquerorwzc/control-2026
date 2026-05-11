# Set the type of chassis, gimbal and shooter modules for infantry wheel legged
set(CHASSIS_TYPE chassis_wheel_legged_sjtu)
set(GIMBAL_TYPE gimbal_standard)
set(SHOOT_TYPE shoot_standard)

# 开发板类型: 由 -DBOARD_TYPE 从外部传入 (tasks.json 的 boardType 选项)
# 若未指定则默认为 CHASSIS_BOARD
if(NOT DEFINED BOARD_TYPE)
    set(BOARD_TYPE "CHASSIS_BOARD")
endif()

if(BOARD_TYPE STREQUAL "ONE_BOARD")
    set(MCU_TYPE "stm32-h7")
elseif(BOARD_TYPE STREQUAL "GIMBAL_BOARD")
    set(MCU_TYPE "stm32-f4")
elseif(BOARD_TYPE STREQUAL "CHASSIS_BOARD")
    set(MCU_TYPE "stm32-h7")
else()
    message(FATAL_ERROR "Unknown BOARD_TYPE '${BOARD_TYPE}' (expected ONE_BOARD / GIMBAL_BOARD / CHASSIS_BOARD)")
endif()

add_compile_definitions(${BOARD_TYPE})

# 控制链路选择
# add_compile_definitions(USE_RC_CTRL) #遥控器链路
add_compile_definitions(USE_OCD_CTRL) #图传链路

# 摩擦轮数量
add_compile_definitions(FRICTION_NUM=2)

# Include directories for header file searching
include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/shoot/${SHOOT_TYPE})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/gimbal/${GIMBAL_TYPE})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/chassis/${CHASSIS_TYPE})

# Define source files for the robot application
file(GLOB_RECURSE ROBOT_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/chassis/${CHASSIS_TYPE}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/gimbal/${GIMBAL_TYPE}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/shoot/${SHOOT_TYPE}/*.c"
)

# Add the robot source files to the global SOURCES list
list(APPEND SOURCES ${ROBOT_SOURCES})
