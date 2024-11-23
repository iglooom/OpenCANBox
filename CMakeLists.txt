cmake_minimum_required(VERSION 3.9)
project(opencanbox C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

add_compile_options(-mthumb -mcpu=cortex-m3 -mthumb-interwork -mtune=cortex-m0 -mno-unaligned-access)
add_compile_options(-Wall -lm -lc -lnosys --specs=nosys.specs --specs=nano.specs -ffunction-sections -fsigned-char -fdata-sections -fno-common -fno-strict-aliasing -fmessage-length=0)

# Choose target device here
set(DEVICE STM32F105RBT6)
set(PLATFORM_ARCH STM32F1)

# Set STM32 SoC specific variables
add_definitions(-DSTM32F1 -DSTM32F105RBT6)

# Compiler
include(cmake/toolchain.cmake)

# Include additional compilation flags
# include(cmake/warnings.cmake)

# Include libopencm3
include(cmake/libopencm3.cmake)

add_library(stm32f1 STATIC IMPORTED)
set_property(TARGET stm32f1 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${LIBOPENCM3_DIR}/include)
set_property(TARGET stm32f1 PROPERTY IMPORTED_LOCATION ${LIBOPENCM3_DIR}/lib/libopencm3_stm32f1.a)

target_link_directories(stm32f1 INTERFACE ${LIBOPENCM3_DIR}/lib)

# Include FreeRTOS
include(cmake/rtos.cmake)

include_directories(
        ${CMAKE_SOURCE_DIR}/inc
        ${LIBOPENCM3_DIR}/include
        ${FREERTOS_DIR}/include
        ${FREERTOS_DIR}/config
)

#add_subdirectory(src)

#add_definitions(-DDEBUG)

file(GLOB_RECURSE SOURCES "src/*.*" "inc/*.*" "libs/*.*")

if (BOOTLOADER)
    MESSAGE(STATUS "Compile for Bootloader")
    set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/linkerbl.ld)
else()
    set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/linker.ld)
endif()

add_link_options(-nostartfiles -lc -lnosys -mthumb -mcpu=cortex-m3)
add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map,--cref,--gc-sections)
add_link_options(-T ${LINKER_SCRIPT})

add_link_options(-specs=rdimon.specs)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

add_dependencies(${PROJECT_NAME}.elf libopencm3)
target_link_libraries(${PROJECT_NAME}.elf opencm3_stm32f1 freertos)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
