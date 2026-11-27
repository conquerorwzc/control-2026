set(MCU_TYPE "stm32-h7")
add_compile_definitions(CHASSIS_BOARD FRICTION_NUM=2)
set(LEG_LENGTH_DIR "${CMAKE_SOURCE_DIR}/UserApp/components/chassis/leg_length_control")
include_directories("${CMAKE_CURRENT_LIST_DIR}" "${LEG_LENGTH_DIR}")
list(APPEND SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/robot.c"
    "${LEG_LENGTH_DIR}/length_control.c"
    "${CMAKE_CURRENT_LIST_DIR}/tasks.c"
)
set_source_files_properties(
    "${CMAKE_CURRENT_LIST_DIR}/robot.c" "${LEG_LENGTH_DIR}/length_control.c"
    PROPERTIES COMPILE_OPTIONS "-fno-fast-math"
)
