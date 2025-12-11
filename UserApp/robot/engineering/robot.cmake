# Set the type of chassis, gimbal and shooter modules for infantry robot
set(CHASSIS_TYPE chassis_mecanum)
set(Grab_TYPE 5-DOF)
set(GANTRY_TYPE "")
add_compile_definitions(FRICTION_NUM=3)

# Include directories for header file searching
include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/grab/${Grab_TYPE})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/gantry/${GANTRY_TYPE})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/chassis/${CHASSIS_TYPE})

# Define source files for the robot application
file(GLOB_RECURSE ROBOT_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/chassis/${CHASSIS_TYPE}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/grab/${Grab_TYPE}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/gantry/${GANTRY_TYPE}/*.c"
)

# Add the robot source files to the global SOURCES list
list(APPEND SOURCES ${ROBOT_SOURCES})