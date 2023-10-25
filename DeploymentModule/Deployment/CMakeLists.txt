# Create Handler Library
add_library(Deployment
    deployment.c
)

target_link_libraries(Deployment Common threadx)

# Include header
target_include_directories(Deployment PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/
)