cmake_minimum_required(VERSION 3.22)
project(MVES)

enable_language(C ASM)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(BUILD_TARGET STREQUAL "MTuSCModule")
    set(MCU "STM32H563")
    add_compile_definitions(HSE_VALUE=24000000)
else()
   message(FATAL_ERROR "No target specified! Use ./scripts/build.sh DeploymentModule | .. ")
endif()

add_subdirectory(manticore)
add_subdirectory(${BUILD_TARGET})
