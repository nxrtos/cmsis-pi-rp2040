cmake_minimum_required(VERSION 3.10.0)
project(test_code)

set(SOURCE ${CMAKE_CURRENT_LIST_DIR}/main.c)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/CMSIS)

add_executable(${PROJECT_NAME} ${SOURCE})
target_link_libraries(${PROJECT_NAME} CMSIS CMSIS_DEVICE_RP2XX)
target_link_options(${PROJECT_NAME} PRIVATE
    -Wl,--script=${CMAKE_CURRENT_LIST_DIR}/gcc_arm.ld)

set(OUTPUT_HEX ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex)
set(OUTPUT_BIN ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_target(${PROJECT_NAME}_hex ALL DEPENDS ${OUTPUT_HEX})
add_custom_command(OUTPUT ${OUTPUT_HEX} COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${OUTPUT_HEX})

add_custom_target(${PROJECT_NAME}_bin ALL DEPENDS ${OUTPUT_BIN})
add_custom_command(OUTPUT ${OUTPUT_BIN} COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}> ${OUTPUT_BIN})