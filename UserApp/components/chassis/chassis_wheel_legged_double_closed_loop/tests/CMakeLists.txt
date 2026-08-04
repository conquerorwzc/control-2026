cmake_minimum_required(VERSION 3.20)

# 以下工程仅用于主机端单测，不参与单片机固件构建。
project(parallel_leg_tests C)

set(CONTROLLER_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../../Modules/algorithm/controller")
set(LQR_EXPORT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../../Utils/Simulink/wheel_legged_double_closed_loop_lqr")

add_executable(parallel_leg_tests
    test_main.c
    parallel_leg_test.c
    hip_odometry_test.c
    joint_transmission_test.c
    length_control_test.c
    chassis_lqr_test.c
    controller_dwt_stub.c
    ../chassis_length_control.c
    ../chassis_lqr.c
    ${CONTROLLER_DIR}/controller.c
    ../kinematics/parallel_leg.c
    ../kinematics/hip_odometry.c
    ../transmission/joint_transmission.c
)

target_include_directories(parallel_leg_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CONTROLLER_DIR}
    ${LQR_EXPORT_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/mocks
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../kinematics
    ${CMAKE_CURRENT_LIST_DIR}/../transmission
)

target_compile_features(parallel_leg_tests PRIVATE c_std_11)
target_compile_options(parallel_leg_tests PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(parallel_leg_tests PRIVATE m)
