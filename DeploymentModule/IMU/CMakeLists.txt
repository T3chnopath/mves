# Create IMU Library
add_library(IMU
    imu.c
)

target_link_libraries(IMU Deployment Common MCU_Support)

# Include header
target_include_directories(IMU PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/
)