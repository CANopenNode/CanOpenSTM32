cmake_minimum_required(VERSION 3.22)

message("Entering ${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt")

#
# Core project settings
#
set(PROJ_PATH                       ${CMAKE_CURRENT_SOURCE_DIR})
project(CANOpenNode-STM32H735G-DK-FreeRTOS)
enable_language(C CXX ASM)
message("Build type: "              ${CMAKE_BUILD_TYPE})

# Setup compiler settings
set(CMAKE_C_STANDARD                11)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)
set(CMAKE_CXX_STANDARD              20)
set(CMAKE_CXX_STANDARD_REQUIRED     ON)
set(CMAKE_CXX_EXTENSIONS            ON)

#
# Core MCU flags, CPU, instruction set and FPU setup
#
set(CPU_PARAMETERS
    -mthumb
    # Other parameters
    # -mcpu, -mfloat, -mfloat-abi, ...
    -mcpu=cortex-m7
	-mfpu=fpv5-d16
	-mfloat-abi=hard
)

# Set linker script
set(linker_script_SRC               ${PROJ_PATH}/STM32H735IGKX_FLASH.ld)
set(EXECUTABLE                      ${CMAKE_PROJECT_NAME})

#
# Source files
#
set(linked_SRCS
    ${PROJ_PATH}/../CANopenNode/CANopen.c
	${PROJ_PATH}/../CANopenNode/301/CO_Emergency.c
	${PROJ_PATH}/../CANopenNode/304/CO_GFC.c
	${PROJ_PATH}/../CANopenNode/301/CO_HBconsumer.c
	${PROJ_PATH}/../CANopenNode/303/CO_LEDs.c
	${PROJ_PATH}/../CANopenNode/305/CO_LSSmaster.c
	${PROJ_PATH}/../CANopenNode/305/CO_LSSslave.c
	${PROJ_PATH}/../CANopenNode/301/CO_NMT_Heartbeat.c
	${PROJ_PATH}/../CANopenNode/301/CO_ODinterface.c
	${PROJ_PATH}/../CANopenNode/301/CO_PDO.c
	${PROJ_PATH}/../CANopenNode/301/CO_SDOclient.c
	${PROJ_PATH}/../CANopenNode/301/CO_SDOserver.c
	${PROJ_PATH}/../CANopenNode/304/CO_SRDO.c
	${PROJ_PATH}/../CANopenNode/301/CO_SYNC.c
	${PROJ_PATH}/../CANopenNode/301/CO_TIME.c
	${PROJ_PATH}/../CANopenNode/301/CO_fifo.c
	${PROJ_PATH}/../CANopenNode/309/CO_gateway_ascii.c
	${PROJ_PATH}/../CANopenNode/storage/CO_storage.c
	${PROJ_PATH}/../CANopenNode/extra/CO_trace.c
	${PROJ_PATH}/../CANopenNode/301/crc16-ccitt.c
	${PROJ_PATH}/../libs/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
	${PROJ_PATH}/../libs/FreeRTOS/Source/croutine.c
	${PROJ_PATH}/../libs/FreeRTOS/Source/event_groups.c
	${PROJ_PATH}/../libs/FreeRTOS/Source/portable/MemMang/heap_4.c
	${PROJ_PATH}/../libs/FreeRTOS/Source/list.c
	${PROJ_PATH}/../libs/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
	${PROJ_PATH}/../libs/FreeRTOS/Source/queue.c
	${PROJ_PATH}/../libs/FreeRTOS/Source/stream_buffer.c
	${PROJ_PATH}/../libs/FreeRTOS/Source/tasks.c
	${PROJ_PATH}/../libs/FreeRTOS/Source/timers.c
	${PROJ_PATH}/../source/CO_driver_stm32h735g-dk.c
	${PROJ_PATH}/../source/OD.c
	${PROJ_PATH}/../source/comm_stm32h735g-dk.c
	${PROJ_PATH}/../libs/lwmem/lwmem/src/lwmem/lwmem.c
	${PROJ_PATH}/../libs/lwmem/lwmem/src/system/lwmem_sys_cmsis_os.c
	${PROJ_PATH}/../libs/lwprintf/lwprintf/src/lwprintf/lwprintf.c
	${PROJ_PATH}/../libs/lwprintf/lwprintf/src/system/lwprintf_sys_cmsis_os.c
	${PROJ_PATH}/../libs/lwrb/lwrb/src/lwrb/lwrb.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_dma.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_exti.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_gpio.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_rcc.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usart.c
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_utils.c
)
set(source_folder_CANopenNode_SRCS 
	)

set(source_folder_FreeRTOS_SRCS 
	)

set(source_folder_Src_SRCS 
	${PROJ_PATH}/Src/main.c
	${PROJ_PATH}/Src/stm32h7xx_it.c
	${PROJ_PATH}/Src/syscalls.c
	${PROJ_PATH}/Src/sysmem.c
	${PROJ_PATH}/Src/system_stm32h7xx.c)

set(source_folder_Startup_SRCS 
	${PROJ_PATH}/Startup/startup_stm32h735igkx.s)

set(source_folder_lwmem_SRCS 
	)

set(source_folder_lwprintf_SRCS 
	)

set(source_folder_lwrb_SRCS 
	)

set(source_folder_stm32_hal_driver_SRCS 
	)

# Source files in the same path as ".cproject/.project" but nowhere included in file descriptions
set(all_project_dir_SRCS
    
)

#
# Include directories
#
set(include_c_DIRS
    ${PROJ_PATH}/Inc
	${PROJ_PATH}/../libs/FreeRTOS/Source/portable/GCC/ARM_CM4F
	${PROJ_PATH}/../libs/FreeRTOS/Source/include
	${PROJ_PATH}/../libs/FreeRTOS/Source/CMSIS_RTOS_V2
	${PROJ_PATH}/../include
	${PROJ_PATH}/../libs/lwrb/lwrb/src/include
	${PROJ_PATH}/../libs/lwprintf/lwprintf/src/include
	${PROJ_PATH}/../libs/lwmem/lwmem/src/include
	${PROJ_PATH}/../CANopenNode
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Inc
	${PROJ_PATH}/../drivers/STM32H7xx_HAL_Driver/Inc/Legacy
	${PROJ_PATH}/../drivers/CMSIS/Device/ST/STM32H7xx/Include
	${PROJ_PATH}/../drivers/CMSIS/Include
)
set(include_cxx_DIRS
    
)
set(include_asm_DIRS
    
)

#
# Symbols definition
#
set(symbols_c_SYMB
    "DEBUG"
	"STM32H7xx"
	"LWPRINTF_CFG_OS=1"
	"LWMEM_CFG_OS=1"
	"USE_OS"
	"LWPRINTF_IGNORE_USER_OPTS"
	"LWMEM_IGNORE_USER_OPTS"
	"HSE_VALUE=25000000"
	"USE_HAL_DRIVER"
	"USE_FULL_LL_DRIVER"
	"STM32H735xx"
)
set(symbols_cxx_SYMB
    
)
set(symbols_asm_SYMB
    "DEBUG"
)

#
# Link directories setup
# Must be before executable is added
#
set(link_DIRS
    
)
link_directories(${EXECUTABLE} ${link_DIRS})

#
# Executable files
#
add_executable(${EXECUTABLE}
    ${linked_SRCS}
    ${all_project_dir_SRCS} 
	${source_folder_CANopenNode_SRCS}
	${source_folder_FreeRTOS_SRCS}
	${source_folder_Src_SRCS}
	${source_folder_Startup_SRCS}
	${source_folder_lwmem_SRCS}
	${source_folder_lwprintf_SRCS}
	${source_folder_lwrb_SRCS}
	${source_folder_stm32_hal_driver_SRCS})

#
# Add linked libraries for linker
#
set(link_LIBS
    
)
target_link_libraries(${EXECUTABLE} ${link_LIBS})

#
# Project symbols
#
target_compile_definitions(${EXECUTABLE} PRIVATE
    # Language specific only
    $<$<COMPILE_LANGUAGE:C>: ${symbols_c_SYMB}>
    $<$<COMPILE_LANGUAGE:CXX>: ${symbols_cxx_SYMB}>
    $<$<COMPILE_LANGUAGE:ASM>: ${symbols_asm_SYMB}>

    # Configuration specific
    $<$<CONFIG:Debug>:
        DEBUG
    >
    $<$<CONFIG:Release>: >
)

#
# Add include paths for each of the compiler
#
target_include_directories(${EXECUTABLE} PRIVATE
    # Language specific only
    $<$<COMPILE_LANGUAGE:C>: ${include_c_DIRS}>
    $<$<COMPILE_LANGUAGE:CXX>: ${include_cxx_DIRS}>
    $<$<COMPILE_LANGUAGE:ASM>: ${include_asm_DIRS}>

    # Configuration specific
    $<$<CONFIG:Debug>: >
    $<$<CONFIG:Release>: >
)

# Compiler and linker options
target_compile_options(${EXECUTABLE} PRIVATE
    ${CPU_PARAMETERS}
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    $<$<COMPILE_LANGUAGE:C>:

    >
    $<$<COMPILE_LANGUAGE:CXX>:
        #-Wno-volatile
        #-Wold-style-cast
        #-Wuseless-cast
        #-Wsuggest-override
    >
    $<$<COMPILE_LANGUAGE:ASM>:

    >
    $<$<CONFIG:Debug>:
        -Og -g3 -ggdb
    >
    $<$<CONFIG:Release>:
        -Og -g0
    >
)

# Setup linker parameters
target_link_options(${EXECUTABLE} PRIVATE
    -T${MCU_LINKER_SCRIPT}${linker_script_SRC}
    ${CPU_PARAMETERS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    --specs=nosys.specs
    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -lsupc++
    -Wl,--end-group
    -Wl,--print-memory-usage
)

# Execute post-build to print size
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
)

# Convert output to hex and binary
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
)

# Conver to bin file -> add conditional check?
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin
)

message("Exiting ${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt")