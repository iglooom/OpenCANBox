set(LIBOPENCM3_DIR ${CMAKE_SOURCE_DIR}/libopencm3)
add_compile_options(-mthumb -mcpu=cortex-m3 -mthumb-interwork -mtune=cortex-m3)

# Make sure that git submodule is initialized and updated
if (NOT EXISTS ${LIBOPENCM3_DIR}/Makefile)
    message(FATAL_ERROR "libopencm3 submodule not found. Initialize with 'git submodule update --init' in the source directory")
endif()

set(LIBOPENCM3_DIR ${CMAKE_SOURCE_DIR}/libopencm3)
add_custom_target(libopencm3 make TARGETS=stm32/f1 WORKING_DIRECTORY ${LIBOPENCM3_DIR})
#add_custom_target(libopencm3
#        COMMAND make -j8 PREFIX=${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX} TARGETS=stm32/f0 all
#        WORKING_DIRECTORY ${LIBOPENCM3_DIR})

link_directories(${LIBOPENCM3_DIR}/lib)
include_directories(${LIBOPENCM3_DIR}/include)

set(LIBOPENCM3_LIB opencm3_stm32f0)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --static -nostartfiles ")

set(LIBOPENCM3_LINKER_FLAGS " --static -nostartfiles ")

set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${LIBOPENCM3_LIB})

set(EXTERNAL_DEPENDENCIES ${EXTERNAL_DEPENDENCIES} libopencm3)

set(EXTERNAL_EXECUTABLES ${EXTERNAL_EXECUTABLES} opencm3.c)

