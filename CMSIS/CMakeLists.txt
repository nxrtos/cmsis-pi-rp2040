cmake_minimum_required(VERSION 3.10.0)
project(CMSIS)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/Device/Raspberry/RP2XXX)

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE CMSIS_DEVICE_RP2XX boot_stage2_asm)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR}/Core/Include)