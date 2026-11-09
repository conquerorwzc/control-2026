set(MCU_TYPE "stm32-h7")
add_compile_definitions(CHASSIS_BOARD FRICTION_NUM=2)
include_directories(${CMAKE_CURRENT_LIST_DIR})
include_directories(${CMAKE_SOURCE_DIR}/UserApp/robot/wheel_leg_rl/generated)
list(APPEND SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/robot.c"
    "${CMAKE_CURRENT_LIST_DIR}/sysid_signal.c"
    "${CMAKE_SOURCE_DIR}/UserApp/components/wheelleg_bootstrap/wheelleg_tasks.c"
)
# 输入边界需要可靠地拒绝 NaN/Inf，不能被根工程 -Ofast 的假设消除。
set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/sysid_signal.c PROPERTIES COMPILE_OPTIONS "-fno-fast-math")
