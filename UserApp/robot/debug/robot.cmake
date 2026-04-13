# Set the type of chassis, gimbal and shooter modules for infantry robot
set(CHASSIS_TYPE chassis_mecanum)
set(GIMBAL_TYPE gimbal_standard)
set(SHOOT_TYPE shoot_standard)
add_compile_definitions(FRICTION_NUM=3)
# Include directories for header file searching
include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/custom_controller)


# Define source files for the robot application
file(GLOB ROBOT_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/custom_controller/*.c"
)

# Add the robot source files to the global SOURCES list
list(APPEND SOURCES ${ROBOT_SOURCES})