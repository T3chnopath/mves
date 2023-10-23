# Create BSP Library
add_library(BSP
    bsp_deployment.c
)

target_link_libraries(BSP Common)

# Include header
target_include_directories(BSP PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/
)