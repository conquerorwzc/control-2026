# Include directories for header file searching
include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})

# Define source files for the robot application
file(GLOB ROBOT_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
)

# Add the robot source files to the global SOURCES list
list(APPEND SOURCES ${ROBOT_SOURCES})
add_compile_definitions(ROBOT_TARGET_CONVEYOR)

list(FILTER SOURCES EXCLUDE REGEX "Modules/master_machine/master_process\\.c$")
list(FILTER SOURCES EXCLUDE REGEX "Modules/referee/referee_task\\.c$")
list(FILTER SOURCES EXCLUDE REGEX "Modules/referee/rm_referee\\.c$")
list(FILTER SOURCES EXCLUDE REGEX "Modules/referee/referee_UI\\.c$")