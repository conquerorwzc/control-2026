# 三摩擦轮发射测试机器人，不包含底盘和云台组件。
set(SHOOT_TYPE shoot_dm_single)
add_compile_definitions(FRICTION_NUM=3)

include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/shoot/${SHOOT_TYPE})

file(GLOB ROBOT_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/shoot/${SHOOT_TYPE}/*.c"
)

list(APPEND SOURCES ${ROBOT_SOURCES})
