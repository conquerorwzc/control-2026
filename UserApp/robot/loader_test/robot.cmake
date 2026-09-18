# 拨弹盘(M2006+C610)测试机器人:只依赖遥控器模块与DJI电机模块,不包含底盘/云台/发射组件
# Include directories for header file searching
include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})

# Define source files for the robot application
file(GLOB ROBOT_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
)

# Add the robot source files to the global SOURCES list
list(APPEND SOURCES ${ROBOT_SOURCES})
