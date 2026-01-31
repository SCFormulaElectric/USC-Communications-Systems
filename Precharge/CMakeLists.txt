cmake_minimum_required(VERSION 3.16)
# --- ADD THESE 4 LINES ---
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 1. Force the toolchain compilers (assuming they are in your PATH)
set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "arm-none-eabi-g++")
set(CMAKE_ASM_COMPILER "arm-none-eabi-gcc")

# 2. Fix "Compiler Not Able to Compile" error
# CMake tries to link a test program during setup. 
# Since we haven't set up the linker script yet, this fails. 
# This tells CMake to only test COMPILING, not LINKING.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
# -------------------------
project(precharge_c011 C CXX ASM)

# --- Toolchain Configuration ---
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Start with empty defaults to avoid host system flags
set(CMAKE_C_FLAGS "")
set(CMAKE_CXX_FLAGS "")

# Hardware specific flags for Cortex-M4 with FPU
set(MCU_FLAGS "-mcpu=cortex-m0plus -mthumb -mfloat-abi=soft")
add_compile_definitions(STM32C011xx)

# Compiler flags
set(COMMON_FLAGS "${MCU_FLAGS} -Wall -Wextra -O0 -g3 -ffunction-sections -fdata-sections")
set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++17 -fno-rtti -fno-exceptions -fno-threadsafe-statics")
set(CMAKE_ASM_FLAGS "${MCU_FLAGS}")

# Linker flags
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/linker/STM32C011F6P6_FLASH.ld)

# Add "-Wl,-Map=${PROJECT_NAME}.map" to the end
set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} -T ${LINKER_SCRIPT} -Wl,--gc-sections -Wl,--print-memory-usage --specs=nano.specs -Wl,-Map=${PROJECT_NAME}.map")

# --- Sources and Includes ---
include_directories(
    include
    include/cmsis
)

set(SOURCES
    src/main.cpp
    src/system_stm32c0xx.c
    startup/startup_stm32c011xx.s
)

# --- Targets ---
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Command to create a .bin file for flashing
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMENT "Generating binary file..."
)

add_custom_target(flash
    COMMAND st-flash write ${PROJECT_NAME}.bin 0x08000000
    DEPENDS ${PROJECT_NAME}.elf
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Flashing ${PROJECT_NAME}.bin to Nucleo board..."
)