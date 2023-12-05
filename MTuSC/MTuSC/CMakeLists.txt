# Create Handler Library
add_library(MTuSC
    mtusc.c
)

target_link_libraries(MTuSC BSP Common threadx)

# Include header
target_include_directories(MTuSC PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/
)