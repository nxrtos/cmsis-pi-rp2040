cmake_minimum_required(VERSION 3.10.0)
project(boot_stage2)

set(SOURCE ${CMAKE_CURRENT_LIST_DIR}/boot_stage2.c)
set(LD_FILE ${CMAKE_CURRENT_LIST_DIR}/boot_stage2.ld)

add_executable(${PROJECT_NAME} ${SOURCE})

target_compile_options(${PROJECT_NAME} PRIVATE
    -Os)

target_link_options(${PROJECT_NAME} PRIVATE
    -nostartfiles
    -Wl,--script=${LD_FILE}
    -Wl,--entry=_stage2_boot)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../../Include
    ${CMAKE_CURRENT_LIST_DIR}/../../../../../Core/Include)

set(ORIGINAL_BIN ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin)
set(ORIGINAL_HEX ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex)
set(ORIGINAL_ELF ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.elf)
set(ORIGINAL_PAD ${CMAKE_BINARY_DIR}/${PROJECT_NAME}_pad.S)

add_custom_target(${PROJECT_NAME}_elf ALL DEPENDS ${ORIGINAL_ELF})
add_custom_command(OUTPUT ${ORIGINAL_ELF} 
    COMMAND cp $<TARGET_FILE:${PROJECT_NAME}> ${ORIGINAL_ELF} 
    DEPENDS ${PROJECT_NAME})

add_custom_target(${PROJECT_NAME}_bin ALL DEPENDS ${ORIGINAL_BIN})
add_custom_command(OUTPUT ${ORIGINAL_BIN} 
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}> ${ORIGINAL_BIN}
    DEPENDS ${PROJECT_NAME})

add_custom_target(${PROJECT_NAME}_hex ALL DEPENDS ${ORIGINAL_HEX})
add_custom_command(OUTPUT ${ORIGINAL_HEX} 
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${ORIGINAL_HEX}
        DEPENDS ${PROJECT_NAME})

add_custom_target(${PROJECT_NAME}_pad ALL DEPENDS ${ORIGINAL_PAD})
add_dependencies(${PROJECT_NAME}_pad ${PROJECT_NAME}_bin)
add_custom_command(OUTPUT ${ORIGINAL_PAD} 
    COMMAND ${CMAKE_CURRENT_LIST_DIR}/pad_checksum -s 0xffffffff ${ORIGINAL_BIN} ${ORIGINAL_PAD}
    DEPENDS ${PROJECT_NAME})

add_library(${PROJECT_NAME}_asm INTERFACE)
add_dependencies(${PROJECT_NAME}_asm ${PROJECT_NAME}_elf ${PROJECT_NAME}_bin ${PROJECT_NAME}_pad)
target_link_libraries(${PROJECT_NAME}_asm INTERFACE ${ORIGINAL_PAD})