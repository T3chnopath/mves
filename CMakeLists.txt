cmake_minimum_required(VERSION 3.22)
project(MVES)

enable_language(C ASM)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(${TARGET} STREQUAL "Camera")
    set(MCU "STM32H563")
else()
    set(MCU "STM32H503")
endif()

add_subdirectory(manticore)
add_subdirectory(${TARGET})
