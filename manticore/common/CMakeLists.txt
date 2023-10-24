add_library(Common INTERFACE)

add_subdirectory(mcan)
add_subdirectory(utility)
add_subdirectory(dc_motor)
add_subdirectory(servo)

target_link_libraries(Common INTERFACE 
    Utility 
    DC_Motor
    MCAN
    Servo
)