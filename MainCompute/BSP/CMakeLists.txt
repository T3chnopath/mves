# Create BSP Library
add_library(BSP
    bsp_main_compute.c
)

target_link_libraries(BSP HAL)

# Include header
target_include_directories(BSP PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/
)