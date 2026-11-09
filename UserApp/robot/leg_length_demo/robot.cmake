set(MCU_TYPE "stm32-h7")
add_compile_definitions(CHASSIS_BOARD FRICTION_NUM=2)
include_directories(${CMAKE_CURRENT_LIST_DIR})
list(APPEND SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/robot.c"
    "${CMAKE_CURRENT_LIST_DIR}/length_control.c"
    "${CMAKE_CURRENT_LIST_DIR}/tasks.c"
)
set_source_files_properties(
    "${CMAKE_CURRENT_LIST_DIR}/robot.c" "${CMAKE_CURRENT_LIST_DIR}/length_control.c"
    PROPERTIES COMPILE_OPTIONS "-fno-fast-math"
)
