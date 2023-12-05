# Create BSP Library
add_library(BSP
    bsp_mtusc.c
)

target_link_libraries(BSP Common MCU_Support)

# Include header
target_include_directories(BSP PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/
)