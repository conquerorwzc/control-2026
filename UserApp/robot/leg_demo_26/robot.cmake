set(MCU_TYPE "stm32-h7")

set(CHASSIS_TYPE chassis_wheel_legged_sjtu)

include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/chassis/${CHASSIS_TYPE})

file(GLOB_RECURSE ROBOT_SOURCES
        CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/chassis/${CHASSIS_TYPE}/parallel_leg/*.c"
)

list(APPEND SOURCES ${ROBOT_SOURCES})
