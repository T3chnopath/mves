add_subdirectory(IT_Handler)
add_subdirectory(BSP)
set(BOARD "MainCompute")

add_executable(${BOARD}.elf
    main.c
    ${STARTUP_SOURCE}
    ${IT_HANDLER_SOURCE}
)

target_link_options(${BOARD}.elf PRIVATE
    -T ${LINKERSRIPT}
)

target_include_directories(${BOARD}.elf PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/
)

target_link_libraries(${BOARD}.elf
    threadx
    CMSIS
    HAL
    IT_Handler
    BSP
)