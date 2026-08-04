# 同心五连杆 FK 真机观测：不编入其他底盘、云台、发射机构。
#
# robot/ 只放本车硬件适配和标定配置；机构通用代码在 component 中。
# tests/ 是 host 单测，绝不加入 MCU 固件源文件。
set(WHEEL_LEGGED_PARALLEL_LEG_COMPONENT_DIR
        "${CMAKE_SOURCE_DIR}/UserApp/components/chassis/chassis_wheel_legged_double_closed_loop")
set(WHEEL_LEGGED_LQR_EXPORT_DIR
        "${CMAKE_SOURCE_DIR}/Utils/Simulink/wheel_legged_double_closed_loop_lqr")

include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})
include_sub_directories_recursively(${WHEEL_LEGGED_PARALLEL_LEG_COMPONENT_DIR})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/gimbal/gimbal_standard)
include_directories(${WHEEL_LEGGED_LQR_EXPORT_DIR})

file(GLOB ROBOT_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
        "${WHEEL_LEGGED_PARALLEL_LEG_COMPONENT_DIR}/*.c"
        "${WHEEL_LEGGED_PARALLEL_LEG_COMPONENT_DIR}/kinematics/*.c"
        "${WHEEL_LEGGED_PARALLEL_LEG_COMPONENT_DIR}/transmission/*.c"
)

list(APPEND SOURCES ${ROBOT_SOURCES})
