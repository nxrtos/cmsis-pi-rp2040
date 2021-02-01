cmake_minimum_required(VERSION 3.10.0)
project(CMSIS_DEVICE_RP2XX)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/Source/boot_stage2)

file(GLOB SOURCE 
    ${CMAKE_CURRENT_LIST_DIR}/Source/*.c
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/*.c)

add_library(${PROJECT_NAME} ${SOURCE})

target_include_directories(${PROJECT_NAME} 
    PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Include
        ${CMAKE_CURRENT_LIST_DIR}/../../../Driver/Include
    PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../../Core/Include)

target_compile_definitions(${PROJECT_NAME}
    PUBLIC "-DRP2040")