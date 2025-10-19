# Set the type of chassis, gimbal and shooter modules for infantry robot
set(CHASSIS_TYPE chassis_mecanum)
set(GIMBAL_TYPE gimbal_standard)
set(SHOOT_TYPE shoot_standard)

# Include directories for header file searching
include_directories(${CMAKE_CURRENT_LIST_DIR})
include_directories(${CMAKE_SOURCE_DIR}/UserApp/components/shoot/shoot_standard)
include_directories(${CMAKE_SOURCE_DIR}/UserApp/components/gimbal/gimbal_standard)
include_directories(${CMAKE_SOURCE_DIR}/UserApp/components/chassis/chassis_mecanum)

# Define source files for the robot application
set(ROBOT_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/robot.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/chassis/${CHASSIS_TYPE}/chassis.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/gimbal/${GIMBAL_TYPE}/gimbal.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/shoot/${SHOOT_TYPE}/shoot.c"
)

# Add the robot source files to the global SOURCES list
list(APPEND SOURCES ${ROBOT_SOURCES})