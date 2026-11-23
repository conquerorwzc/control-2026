
set(MCU_TYPE "stm32-h7") #采用DM MC02(H723)控制板

# 机械臂执行组件
set(Grab_TYPE grab_6dof)

# 头文件搜索路径
include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/grab/${Grab_TYPE})

# 本目录 + 组件源文件加入全局编译列表
file(GLOB ROBOT_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/grab/${Grab_TYPE}/*.c"
)

list(APPEND SOURCES ${ROBOT_SOURCES})

