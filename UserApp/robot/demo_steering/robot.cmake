# 四舵轮 demo: 只使用舵轮底盘组件, 不引入云台/发射机构
set(CHASSIS_TYPE chassis_steering)
# 云台与发射暂不使用, 因此不设置 GIMBAL_TYPE / SHOOT_TYPE,
# 也就不会把 gimbal_standard / shoot_standard 的源码加入编译.
# 后续需要云台时取消下面两行的注释, 并在 robot.h 中按需包含对应头文件.
# set(GIMBAL_TYPE gimbal_standard)
# set(SHOOT_TYPE shoot_standard)

# Include directories for header file searching
include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/chassis/${CHASSIS_TYPE})

# Define source files for the robot application
file(GLOB_RECURSE ROBOT_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/chassis/${CHASSIS_TYPE}/*.c"
)

# Add the robot source files to the global SOURCES list
list(APPEND SOURCES ${ROBOT_SOURCES})
