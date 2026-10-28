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
set(CMAKE_PROJECT_NAME robotmaster_c_tennis)

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

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    application/CAN_receive.c
    application/INS_task.c
    application/calibrate_task.c
    application/chassis_task.c
    application/chassis_behaviour.c
    application/detect_task.c
    application/led_flow_task.c
    application/test_task.c
    application/remote_control.c
    application/host_comm_task.c
    application/host_protocol.c
    application/host_command.c
    components/algorithm/CRC8_CRC16.c
    bsp/boards/bsp_host_uart.c
    bsp/boards/bsp_buzzer.c
    bsp/boards/bsp_can.c
    bsp/boards/bsp_delay.c
    bsp/boards/bsp_flash.c
    bsp/boards/bsp_imu_pwm.c
    bsp/boards/bsp_led.c
    bsp/boards/bsp_spi.c
    bsp/boards/bsp_rc.c
    components/controller/pid.c
    components/algorithm/AHRS.c
    components/algorithm/AHRS_middleware.c
    components/algorithm/MahonyAHRS.c
    components/algorithm/user_lib.c
    components/devices/BMI088driver.c
    components/devices/BMI088Middleware.c
    components/devices/ist8310driver.c
    components/devices/ist8310driver_middleware.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    application
    bsp/boards
    components/controller
    components/algorithm
    components/devices
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Enable floating-point formatting with newlib-nano printf/snprintf.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)


# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
