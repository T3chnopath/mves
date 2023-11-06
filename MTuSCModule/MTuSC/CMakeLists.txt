# Create Handler Library
add_library(MTUSC
    mtusc.c
)

target_link_libraries(MTUSC BSP Common threadx)

# Include header
target_include_directories(MTUSC PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/
)