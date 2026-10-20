# 四轴云台 + J4310 拨弹盘 + 三只 M2006 摩擦轮。
set(GIMBAL_TYPE gimbal_four_axis)
set(SHOOT_TYPE shoot_dm_single)
add_compile_definitions(FRICTION_NUM=3)

include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/gimbal/${GIMBAL_TYPE})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/shoot/${SHOOT_TYPE})

file(GLOB ROBOT_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/gimbal/${GIMBAL_TYPE}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/shoot/${SHOOT_TYPE}/*.c"
)
list(APPEND SOURCES ${ROBOT_SOURCES})
