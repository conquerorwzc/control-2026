include_directories(
        "${CMAKE_CURRENT_LIST_DIR}"
        "${CMAKE_CURRENT_LIST_DIR}/spring_loader"
        "${CMAKE_CURRENT_LIST_DIR}/latch_release"
)

list(APPEND SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/robot.c"
        "${CMAKE_CURRENT_LIST_DIR}/robot_cmd.c"
        "${CMAKE_CURRENT_LIST_DIR}/spring_loader/spring_loader.c"
        "${CMAKE_CURRENT_LIST_DIR}/latch_release/latch_release.c"
)