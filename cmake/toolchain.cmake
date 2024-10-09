# Use ARM-GCC for crosscompiling
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_STANDARD 11)

find_program(ARM_CC arm-none-eabi-gcc)
find_program(ARM_CXX arm-none-eabi-g++)
find_program(ARM_OBJCOPY arm-none-eabi-objcopy)
find_program(ARM_SIZE arm-none-eabi-size)

set(COMPILER_OPTIONS "-mthumb -mcpu=cortex-m0 -mthumb-interwork")

SET(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_C_COMPILER ${ARM_CC})
set(CMAKE_CXX_COMPILER ${ARM_CXX})
