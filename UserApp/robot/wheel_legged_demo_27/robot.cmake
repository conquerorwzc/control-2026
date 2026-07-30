# 双闭环腿 FK 真机观测：不编入其他底盘、云台、发射机构。
#
# robot/ 只放本车硬件适配和标定配置；机构通用代码在 component 中。
# tests/ 是 host 单测，绝不加入 MCU 固件源文件。
set(DOUBLE_CLOSED_LOOP_LEG_COMPONENT_DIR
        "${CMAKE_SOURCE_DIR}/UserApp/components/chassis/chassis_wheel_legged_double_closed_loop")

include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})
include_sub_directories_recursively(${DOUBLE_CLOSED_LOOP_LEG_COMPONENT_DIR})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/gimbal/gimbal_standard)

file(GLOB ROBOT_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
        "${DOUBLE_CLOSED_LOOP_LEG_COMPONENT_DIR}/*.c"
        "${DOUBLE_CLOSED_LOOP_LEG_COMPONENT_DIR}/kinematics/*.c"
        "${DOUBLE_CLOSED_LOOP_LEG_COMPONENT_DIR}/transmission/*.c"
)

list(APPEND SOURCES ${ROBOT_SOURCES})
