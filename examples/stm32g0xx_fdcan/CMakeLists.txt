cmake_minimum_required(VERSION 3.19)

project(stm32g0xx_fdcan)
enable_language(C CXX ASM)

# Linker and startup goes here
set(linker_script_SRC ${CMAKE_CURRENT_LIST_DIR}/STM32G0C1VETX_FLASH.ld)

# Application, HAL driver and CANopenNode sources
file(GLOB_RECURSE files_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/Core/Src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/Core/Startup/*.s
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32G0xx_HAL_Driver/Src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/../../CANopenNode_STM32/*.c
)
file(GLOB_RECURSE canopennode_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/../../CANopenNode/*.c
)

# Ignore the actual example folder in the CanOpenNode object
list(FILTER canopennode_SRCS EXCLUDE REGEX "/CANopenNode/example/")

# Merge the sources
set(files_SRCS ${files_SRCS} ${canopennode_SRCS})

# Include directories
set(include_DIRS ${include_DIRS}
    ${CMAKE_CURRENT_LIST_DIR}/Core/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32G0xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Device/ST/STM32G0xx/Include
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_LIST_DIR}/../../CANopenNode
    ${CMAKE_CURRENT_LIST_DIR}/../../CANopenNode_STM32
)

# Symbols for compilation
set(symbols_SYMB ${symbols_SYMB}
    "DEBUG"
    "STM32G0C1xx"
    "USE_HAL_DRIVER"
)

# Core MCU flags, CPU, instruction set and FPU setup
# Set the global compilation flags for all units
include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/compiler-flags-cortex-m0p.cmake)

# Add executable
add_executable(${CMAKE_PROJECT_NAME})
target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC ${symbols_SYMB})
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${include_DIRS})
target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${files_SRCS})

# Setup linker parameters
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T${linker_script_SRC}
    ${cpu_PARAMS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map

    -Wl,--start-group
    -lc
    -lm

    -lstdc++
    -lsupc++
    -Wl,--end-group
    -Wl,-z,max-page-size=8 # Allow good software remapping across address space (with proper GCC section making)
    -Wl,--print-memory-usage
)

# Output files in the build folder
set(BUILD_OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/../../_build_outputs)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
)
