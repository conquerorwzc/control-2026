cmake_minimum_required(VERSION 3.20)

# 以下工程仅用于主机端单测，不参与单片机固件构建。
project(double_closed_loop_leg_tests C)

add_executable(double_closed_loop_leg_tests
    test_main.c
    double_closed_loop_leg_test.c
    hip_odometry_test.c
    joint_transmission_test.c
    ../kinematics/double_closed_loop_leg.c
    ../kinematics/hip_odometry.c
    ../transmission/joint_transmission.c
)

target_include_directories(double_closed_loop_leg_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../kinematics
    ${CMAKE_CURRENT_LIST_DIR}/../transmission
)

target_compile_features(double_closed_loop_leg_tests PRIVATE c_std_11)
target_compile_options(double_closed_loop_leg_tests PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(double_closed_loop_leg_tests PRIVATE m)
