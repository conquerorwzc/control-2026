# 飞镖机器人: 只有发射机构 dart_shoot(同步带 3508 x2 + yaw 2006 + 扳机位置 3508 + PWM 舵机)
set(DART_SHOOT_TYPE dart_shoot)

include_directories(${CMAKE_CURRENT_LIST_DIR})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/${DART_SHOOT_TYPE})

file(GLOB ROBOT_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/${DART_SHOOT_TYPE}/*.c"
)

list(APPEND SOURCES ${ROBOT_SOURCES})
