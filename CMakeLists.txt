cmake_minimum_required(VERSION 3.16)
# 指定编译平台/架构与语言标准, 推荐指定Ninja为构建工具,可以加快编译速度(相比make)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
# 指定工具链
set(CMAKE_C_COMPILER_FORCED TRUE) # skip compiler test
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 指定工程名称和语言类型
project(control-2026 C ASM)

# 选择构建类型
if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif() # Debug Release RelWithDebInfo MinSizeRel

# 默认仍构建麦轮步兵，也允许通过 -DROBOT_TYPE=<目录名> 选择其他机器人。
set(ROBOT_TYPE "infantry_mecanum" CACHE STRING "Robot configuration under UserApp/robot")

# 其余已修复机器人预留配置；需要时取消对应一行的注释即可。
# set(ROBOT_TYPE "sentry_omni_chassis")
# set(ROBOT_TYPE "sentry_omni_gimbal")
# set(ROBOT_TYPE "infantry_wheel_legged_sjtu")

set(ROBOT_CMAKE "${CMAKE_SOURCE_DIR}/UserApp/robot/${ROBOT_TYPE}/robot.cmake")
if (NOT EXISTS "${ROBOT_CMAKE}")
    message(FATAL_ERROR "Unknown ROBOT_TYPE '${ROBOT_TYPE}': ${ROBOT_CMAKE} does not exist")
endif ()

# 除明确在 robot.cmake 中覆盖的多板型机器人外，现有机器人均使用 STM32F4。
set(MCU_TYPE "stm32-f4")
# set(MCU_TYPE "stm32-h7")  # H7 主控预留；轮腿项目通常由 robot.cmake 根据 BOARD_TYPE 自动选择

# 递归包含头文件的函数
function(include_sub_directories_recursively root_dir)
    if (IS_DIRECTORY ${root_dir})               # 当前路径是一个目录吗，是的话就加入到包含目录
        message("include dir: " ${root_dir})
        include_directories(${root_dir})
    endif ()

    file(GLOB ALL_SUB RELATIVE ${root_dir} ${root_dir}/*) # 获得当前目录下的所有文件，让如ALL_SUB列表中
    foreach (sub ${ALL_SUB})
        if (IS_DIRECTORY ${root_dir}/${sub})
            include_sub_directories_recursively(${root_dir}/${sub}) # 对子目录递归调用，包含
        endif ()
    endforeach ()
endfunction()

# robot.cmake 根据 BOARD_TYPE 设置 MCU_TYPE，因此必须在 hardware cmake 之前 include
include(${ROBOT_CMAKE})

message(STATUS "Robot type: ${ROBOT_TYPE}")
message(STATUS "MCU type: ${MCU_TYPE}")

include(${CMAKE_SOURCE_DIR}/Hardware/${MCU_TYPE}/${MCU_TYPE}.cmake)

# Generic compiler settings for optimization and basic link lib
add_compile_options(-pipe ${MCU_FLAGS} -Wall -fmessage-length=0 # basic options
        -ffunction-sections -fdata-sections -fno-common # optimize options
)
add_link_options(-pipe ${MCU_FLAGS} -T${LINKER_SCRIPT} -Wl,--no-warn-rwx-segments # close RWX warning
        -lm -lc # lib options
        -Wl,--gc-sections -flto -specs=nano.specs # optimize options
        -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map -Wl,--cref -Wl,--print-memory-usage # map options
) # if your executable is too large , try option '-s' to strip symbols

include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Hardware/${MCU_TYPE})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Bsp)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Modules)
include_directories(${CMAKE_SOURCE_DIR}/UserApp)

file(GLOB_RECURSE BASE_SOURCES
        "Hardware/${MCU_TYPE}/*.c"
        "Bsp/*.c"
        "Modules/*.c"
        "UserApp/os_task.c"
)

# 合并后同时存在两套不兼容的 VT13 驱动，按机器人选择实际使用的一套。
list(FILTER BASE_SOURCES EXCLUDE REGEX "[/\\\\](VT13[/\\\\]VT13|remote[/\\\\]new_RC_VT13)\\.c$")
if (ROBOT_TYPE STREQUAL "infantry_wheel_legged_sjtu" OR ROBOT_TYPE STREQUAL "infantry_six_wheel")
    list(APPEND BASE_SOURCES "${CMAKE_SOURCE_DIR}/Modules/VT13/VT13.c")
elseif (ROBOT_TYPE STREQUAL "infantry_mecanum" OR
        ROBOT_TYPE STREQUAL "sentry_omni_chassis" OR
        ROBOT_TYPE STREQUAL "sentry_omni_gimbal")
    list(APPEND BASE_SOURCES "${CMAKE_SOURCE_DIR}/Modules/remote/new_RC_VT13.c")
endif ()

# 两套超电驱动导出同名 API，不能同时参与链接。
list(FILTER BASE_SOURCES EXCLUDE REGEX "[/\\\\](super_cap[/\\\\]super_cap|super_cap_HKUST[/\\\\]super_cap_HKUST)\\.c$")
if (ROBOT_TYPE STREQUAL "infantry_mecanum" OR ROBOT_TYPE STREQUAL "sentry_omni_chassis")
    list(APPEND BASE_SOURCES "${CMAKE_SOURCE_DIR}/Modules/super_cap_HKUST/super_cap_HKUST.c")
else ()
    list(APPEND BASE_SOURCES "${CMAKE_SOURCE_DIR}/Modules/super_cap/super_cap.c")
endif ()

list(APPEND SOURCES ${BASE_SOURCES})

# Build types
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g -gdwarf-2)
    add_definitions(-DESC_DEBUG) # ESC Debug
endif ()

set_source_files_properties(${ASM_SOURCES} PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

# build binary and hex file
add_executable(${PROJECT_NAME}.elf ${SOURCES} ${ASM_SOURCES} ${LINKER_SCRIPT})
target_link_libraries(${PROJECT_NAME}.elf ${DSP_NAME} m) # link DSP lib and math lib

add_custom_command(
        TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
        COMMENT "Building hex & bin file...\nEXCUTABLE SIZE:"
        COMMAND ${SIZE} ${PROJECT_NAME}.elf
)
