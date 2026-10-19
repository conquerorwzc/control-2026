# 拨弹盘(DM4310)测试机器人:只依赖遥控器模块与DM电机模块,不包含底盘/云台/发射组件
# Include directories for header file searching
set(SHOOT_TYPE shoot_dm_single)
add_compile_definitions(FRICTION_NUM=3)
include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/shoot/${SHOOT_TYPE})
# Define source files for the robot application
file(GLOB ROBOT_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
)

# Add the robot source files to the global SOURCES list
list(APPEND SOURCES ${ROBOT_SOURCES})
