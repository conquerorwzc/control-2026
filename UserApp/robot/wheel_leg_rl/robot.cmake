set(MCU_TYPE "stm32-h7")
add_compile_definitions(CHASSIS_BOARD FRICTION_NUM=2 WHEELLEG_BOOTSTRAP)
include_directories(${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/generated)
list(APPEND SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/robot.c"
    "${CMAKE_SOURCE_DIR}/UserApp/components/wheelleg_bootstrap/wheelleg_tasks.c"
)
