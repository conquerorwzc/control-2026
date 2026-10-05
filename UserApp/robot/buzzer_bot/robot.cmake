# buzzer_bot：蜂鸣器音乐演示机器人，不挂载任何机构组件
# Include directories for header file searching
include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})

# Define source files for the robot application
file(GLOB ROBOT_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
)

# Add the robot source files to the global SOURCES list
list(APPEND SOURCES ${ROBOT_SOURCES})
