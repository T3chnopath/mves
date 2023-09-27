# Create BSP Library
add_library(BSP
    bsp_deployment.c
    bsp_imu.c
    bsp_motor.c
)

target_link_libraries(BSP HAL)

# Include header
target_include_directories(BSP PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/
)