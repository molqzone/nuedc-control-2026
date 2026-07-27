cmake_minimum_required(VERSION 3.20)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(ti_mspm0_libxr_dev LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

set(LIBXR_LOG_LEVEL -1)
set(LIBXR_LOG_OUTPUT_LEVEL -1)

include("${CMAKE_SOURCE_DIR}/cmake/LibXR.CMake")

set(FIRMWARE_SOURCES
    "${CMAKE_SOURCE_DIR}/User/main.c"
    "${CMAKE_SOURCE_DIR}/User/app_main.cpp"
    "${CMAKE_SOURCE_DIR}/User/src/scheduler.cpp"
    "${CMAKE_SOURCE_DIR}/User/src/motion_control.cpp"
    "${CMAKE_SOURCE_DIR}/User/src/imu.cpp"
    "${CMAKE_SOURCE_DIR}/User/src/key_led.cpp"
    "${CMAKE_SOURCE_DIR}/User/src/line_sensors.cpp"
    "${CMAKE_SOURCE_DIR}/User/src/drivebase.cpp"
    "${CMAKE_SOURCE_DIR}/User/src/display.cpp"
    "${CMAKE_SOURCE_DIR}/User/src/console.cpp"
    "${CMAKE_SOURCE_DIR}/User/src/zigbee.cpp"
    "${CMAKE_SOURCE_DIR}/Core/syscalls_stub.c"
    "${MSPM0_STARTUP_FILE}"
    "${SYSCFG_C_FILE}")

add_executable(${PROJECT_NAME}.elf ${FIRMWARE_SOURCES})

target_include_directories(${PROJECT_NAME}.elf PRIVATE
    "${CMAKE_SOURCE_DIR}/Core"
    "${CMAKE_SOURCE_DIR}/User"
    "${CMAKE_SOURCE_DIR}/User/src"
    ${MSPM0_COMMON_INCLUDE_DIRS})

target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
    LIBXR_DEFAULT_SCALAR=double
    LIBXR_PRINTF_BUFFER_SIZE=128
    LIBXR_LOG_LEVEL=${LIBXR_LOG_LEVEL}
    LIBXR_LOG_OUTPUT_LEVEL=${LIBXR_LOG_OUTPUT_LEVEL}
    XR_LOG_MESSAGE_MAX_LEN=64
    ${MSPM0_COMMON_COMPILE_DEFINITIONS})

target_compile_options(${PROJECT_NAME}.elf PRIVATE
    ${MSPM0_ARCH_COMPILE_OPTIONS}
    -ffunction-sections
    -fdata-sections
    -Os
    -g
    "@${SYSCFG_OUTPUT_DIR}/device.opt"
    $<$<COMPILE_LANGUAGE:C>:-std=gnu11>
    $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++20>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fpermissive>)

target_link_libraries(${PROJECT_NAME}.elf PRIVATE xr m)
target_compile_options(xr PRIVATE
    -ffunction-sections
    -fdata-sections
    -Os)
target_link_options(${PROJECT_NAME}.elf PRIVATE
    -mcpu=cortex-m0plus
    -mthumb
    -mfloat-abi=soft
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,-e,Reset_Handler
    -Wl,--gc-sections
    -Wl,--no-warn-rwx-segment
    "-Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map"
    "-T${SYSCFG_LINKER_FILE}")

find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy REQUIRED)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
        $<TARGET_FILE:${PROJECT_NAME}.elf>
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
        $<TARGET_FILE:${PROJECT_NAME}.elf>
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin)
