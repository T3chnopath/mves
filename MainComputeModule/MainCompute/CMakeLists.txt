# Create Handler Library
add_library(MainCompute
    maincompute.c
)

target_link_libraries(MainCompute BSP Common threadx)

# Include header
target_include_directories(MainCompute PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/
)