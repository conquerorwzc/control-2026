# ============================================================
# heavy_three_friction_launcher —— 发射机构 + 云台台架测试目标
# 只编译「云台 + 发射」两个组件，不编译底盘、不编译超级电容
# 选型方式: 根 CMakeLists.txt 的 ROBOT_TYPE 指向本目录
# ============================================================

# 组件选型: 云台用标准云台, 发射用标准发射(三摩擦轮)
set(GIMBAL_TYPE gimbal_standard)
set(SHOOT_TYPE  shoot_standard)

# 三摩擦轮 + 旧版 DJI 遥控(DBUS, 走 USART3)
add_compile_definitions(FRICTION_NUM=3)
add_compile_definitions(USE_DUAL_RC)

# 头文件搜索路径
include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/gimbal/${GIMBAL_TYPE})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/shoot/${SHOOT_TYPE})

# 源文件: 本目录 + 云台组件 + 发射组件
file(GLOB ROBOT_SOURCES
        CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/gimbal/${GIMBAL_TYPE}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/shoot/${SHOOT_TYPE}/*.c"
)

list(APPEND SOURCES ${ROBOT_SOURCES})
