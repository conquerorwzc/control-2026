set(MCU_TYPE "stm32-h7")
set(CHASSIS_DIR "${CMAKE_SOURCE_DIR}/UserApp/components/chassis/chassis_wheelleg_demo")
set(LEG_LENGTH_DIR "${CMAKE_SOURCE_DIR}/UserApp/components/chassis/leg_length_control")
include_directories(BEFORE "${CMAKE_CURRENT_LIST_DIR}" "${CHASSIS_DIR}" "${LEG_LENGTH_DIR}")
add_compile_definitions(CHASSIS_BOARD FRICTION_NUM=2)
set(SYSID_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/robot.c"
    "${CMAKE_CURRENT_LIST_DIR}/vofa_watch.c"
    "${CHASSIS_DIR}/chassis.c"
    "${LEG_LENGTH_DIR}/length_control.c"
)
list(APPEND SOURCES ${SYSID_SOURCES})
set_source_files_properties(${SYSID_SOURCES} PROPERTIES COMPILE_OPTIONS "-fno-fast-math")
foreach(setting SYSID_EXPERIMENT SYSID_TEST_SIDES SYSID_HIP_PATTERN SYSID_TRIAL_DIRECTION HIP_SWEEP_AMPLITUDE_RAD)
    if(DEFINED ${setting})
        set_property(SOURCE "${CMAKE_CURRENT_LIST_DIR}/robot.c" APPEND PROPERTY COMPILE_DEFINITIONS "${setting}=${${setting}}")
    endif()
endforeach()
