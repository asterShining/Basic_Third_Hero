cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME basic_framework)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

file(GLOB_RECURSE SOURCES
    "application/*.c"
    "application/chassis/*.c"
    "application/shoot/*.c"
    "application/gimbal/*.c"
    "application/cmd/*.c"
    "bsp/*.c"
    "bsp/can/*.c"
    "bsp/usart/*.c"
    "bsp/adc/*.c"
    "bsp/iic/*.c"
    "bsp/flash/*.c"
    "bsp/pwm/*.c"
    "bsp/spi/*.c"
    "bsp/log/*.c"
    "bsp/usb/*.c"
    "bsp/gpio/*.c"
    "bsp/dwt/*.c"
    "modules/*.c"
    "modules/alarm/*.c"
    "modules/algorithm/*.c"
    "modules/bluetooth/*.c"
    "modules/BMI088/*.c"
    "modules/can_comm/*.c"
    "modules/daemon/*.c"
    "modules/encoder/*.c"
    "modules/imu/*.c"
    "modules/ist8310/*.c"
    "modules/master_machine/*.c"
    "modules/message_center/*.c"
    "modules/motor/DJImotor/*.c"
    "modules/motor/DMmotor/*.c"
    "modules/motor/servo_motor/*.c"
    "modules/motor/step_motor/*.c"
    "modules/HTmotor/*.c"
    "modules/oled/*.c"
    "modules/referee/*.c"
    "modules/remote/*.c"
    "modules/standard_cmd/*.c"
    "modules/super_cap/*.c"
    "modules/TFminiPlus/*.c"
    "modules/unicomm/*.c"
    "Middlewares/Third_Party/SEGGER/RTT/*.c"
    "Middlewares/Third_Party/SEGGER/Config/*.c"
    
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${SOURCES}
)

# Add include paths

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

    # application subdirectories
    application
    application/gimbal
    application/chassis
    application/cmd
    application/shoot

    # bsp
    bsp
    bsp/adc
    bsp/can
    bsp/dwt
    bsp/gpio
    bsp/flash
    bsp/usart
    bsp/iic
    bsp/pwm
    bsp/log
    bsp/usb
    bsp/spi

    # modules
    modules
    modules/alarm
    modules/algorithm
    modules/bluetooth
    modules/BMI088
    modules/can_comm
    modules/daemon
    modules/encoder
    modules/imu
    modules/ist8310
    modules/master_machine
    modules/message_center
    modules/motor
    modules/motor/DJImotor
    modules/motor/DMmotor
    modules/motor/HTmotor
    modules/motor/LKmotor
    modules/motor/servo_motor
    modules/motor/step_motor
    modules/oled
    modules/referee
    modules/remote
    modules/standard_cmd
    modules/super_cap

    modules/TFminiPlus
    modules/unicomm

    Middlewares/Third_Party/SEGGER/Config
    Middlewares/Third_Party/SEGGER/RTT
)
# 编译器选项
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE

    # MCU 相关选项
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16

    # C标准
    -std=gnu11

    # 警告选项
    -Wall
    -Wextra
    -Wno-unused-parameter # 这个应该抑制未使用参数警告
    -Wno-unused-variable
    -Wno-pedantic
    -Wno-type-limits
    -Wno-pragma-messages
    -Wno-missing-braces # 可考虑添加：抑制缺少大括号警告

    # 优化选项
    $<$<CONFIG:Debug>:-Og -g -gdwarf-2>
    $<$<CONFIG:Release>:-Ofast -flto>

    # 其他选项
    -fdata-sections
    -ffunction-sections
    -fno-common
    -fstack-usage
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    USE_HAL_DRIVER
    STM32F407xx
    ARM_MATH_CM4
)
# 链接器选项
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE

    -Wl,-Map=${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.map # 生成内存映射文件
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
