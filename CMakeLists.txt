cmake_minimum_required(VERSION 3.20)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(ARM_TOOLCHAIN_HINTS
    "D:/ide/Renesas/rzn/e2studio_v2025-10_fsp_v3.1.0/toolchains/gcc_arm/arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi/bin"
    "D:/ide/Renesas/rz/e2studio_v2026-04.1_fsp_v4.1.0/toolchains/gcc_arm/arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi/bin"
)

find_program(ARM_NONE_EABI_GCC arm-none-eabi-gcc HINTS ${ARM_TOOLCHAIN_HINTS})
find_program(ARM_NONE_EABI_OBJCOPY arm-none-eabi-objcopy HINTS ${ARM_TOOLCHAIN_HINTS})
find_program(ARM_NONE_EABI_SIZE arm-none-eabi-size HINTS ${ARM_TOOLCHAIN_HINTS})

if(NOT ARM_NONE_EABI_GCC)
    message(FATAL_ERROR "arm-none-eabi-gcc was not found. Add the Renesas ARM GCC toolchain bin directory to PATH or update ARM_TOOLCHAIN_HINTS.")
endif()

set(CMAKE_C_COMPILER "${ARM_NONE_EABI_GCC}" CACHE FILEPATH "ARM GCC C compiler" FORCE)

project(RZN2L_lwIP_Ethercat_Master_rev0300 C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

set(TARGET_NAME RZN2L_lwIP_Ethercat_Master_rev0300)
set(E2STUDIO_OUTPUT_NAME RZN2L_RSK_lwIP_single)
set(E2STUDIO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/project/rzn2l_rsk/lwip_single/e2studio")
set(FSP_LINKER_DIR "${E2STUDIO_DIR}/script")
set(FSP_LINKER_SCRIPT "${FSP_LINKER_DIR}/fsp_xspi0_boot.ld")
set(FSP_CMAKE_LINKER_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/fsp_xspi0_boot_cmake.ld")
set(E2STUDIO_DEBUG_DIR "${E2STUDIO_DIR}/Debug")
set(FSP_MEMORY_REGIONS "${E2STUDIO_DEBUG_DIR}/memory_regions.ld")

if(NOT EXISTS "${FSP_LINKER_SCRIPT}")
    message(FATAL_ERROR "Linker script not found: ${FSP_LINKER_SCRIPT}")
endif()

if(NOT EXISTS "${FSP_MEMORY_REGIONS}")
    message(FATAL_ERROR "Memory regions file not found: ${FSP_MEMORY_REGIONS}")
endif()

file(READ "${FSP_LINKER_SCRIPT}" FSP_LINKER_SCRIPT_CONTENT)

function(_add_cmake_object_patterns original)
    foreach(cmake_suffix IN ITEMS ".c.o" ".c.obj")
        string(REPLACE ".o(" "${cmake_suffix}(" cmake_object "${original}")
        string(REPLACE
            "${original}"
            "${original}\n        ${cmake_object}"
            FSP_LINKER_SCRIPT_CONTENT
            "${FSP_LINKER_SCRIPT_CONTENT}")
    endforeach()
    set(FSP_LINKER_SCRIPT_CONTENT "${FSP_LINKER_SCRIPT_CONTENT}" PARENT_SCOPE)
endfunction()

foreach(section IN ITEMS ".text*" ".data*" ".bss*" "COMMON")
    _add_cmake_object_patterns("*/startup_core.o(${section})")
    _add_cmake_object_patterns("*/system_core.o(${section})")
    _add_cmake_object_patterns("*/startup.o(${section})")
    _add_cmake_object_patterns("*/system.o(${section})")
    _add_cmake_object_patterns("*/all*/bsp_*.o(${section})")
    _add_cmake_object_patterns("*/r_ioport.o(${section})")
endforeach()

foreach(section IN ITEMS ".rodata*")
    _add_cmake_object_patterns("*/bsp_mpu_core.o(${section})")
    _add_cmake_object_patterns("*/bsp_mmu_core.o(${section})")
    _add_cmake_object_patterns("*/bsp_register_protection.o(${section})")
    _add_cmake_object_patterns("*/bsp_tzc400.o(${section})")
endforeach()

file(WRITE "${FSP_CMAKE_LINKER_SCRIPT}" "${FSP_LINKER_SCRIPT_CONTENT}")

set(COMMON_COMPILE_OPTIONS
    -mcpu=cortex-r52
    -mthumb
    -mfloat-abi=hard
    -mfpu=neon-fp-armv8
    -fdiagnostics-parseable-fixits
    -Og
    -fmessage-length=0
    -fsigned-char
    -ffunction-sections
    -fdata-sections
    -Wunused
    -Wuninitialized
    -Wall
    -Wextra
    -Wmissing-declarations
    -Wconversion
    -Wpointer-arith
    -Wshadow
    -Wlogical-op
    -Waggregate-return
    -Wfloat-equal
    -Wnull-dereference
    -g
    -gdwarf-4
    -Wno-format-truncation
    -Wno-stringop-overflow
    --param=min-pagesize=0
)

set(COMMON_DEFINITIONS
    _RENESAS_RZN_
    _RZN_CORE=CR52_0
    _RZN_ORDINAL=1
    EC_VER1
    _LWIP_ADD_ON_APP=0
)

set(INCLUDE_DIRS
    "${E2STUDIO_DIR}/rzn/fsp/inc"
    "${E2STUDIO_DIR}/rzn/fsp/inc/api"
    "${E2STUDIO_DIR}/rzn/fsp/inc/instances"
    "${E2STUDIO_DIR}/rzn/fsp/src/bsp/mcu/all/cr"
    "${E2STUDIO_DIR}/rzn/aws/FreeRTOS-Kernel/include"
    "${E2STUDIO_DIR}/rzn/fsp/src/rm_freertos_port/cr"
    "${E2STUDIO_DIR}/rzn/fsp/src/rm_freertos_port"
    "${E2STUDIO_DIR}/rzn/arm/CMSIS_6/CMSIS/Core/Include"
    "${E2STUDIO_DIR}/rzn_gen"
    "${E2STUDIO_DIR}/rzn_cfg/fsp_cfg/bsp"
    "${E2STUDIO_DIR}/rzn_cfg/fsp_cfg"
    "${E2STUDIO_DIR}/rzn_cfg/aws"
    "${E2STUDIO_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/application"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/ether_netif"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/lwip_port"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/lwip_port/lwip_osal/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/lwip_port/lwip_osal/include/arch"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/serial_io"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/oss_deps/lwip"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/oss/lwip/src/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/ethercat/app"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/ethercat/port"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/ethercat/soem/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/ethercat/soem/osal"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/ethercat/soem/oshw"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/ethercat/soem/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/gpt"
)

set(SOURCE_DIRS
    "${E2STUDIO_DIR}/rzn/aws/FreeRTOS-Kernel"
    "${E2STUDIO_DIR}/rzn/aws/FreeRTOS-Kernel/portable/MemMang"
    "${E2STUDIO_DIR}/rzn/board/rzn2l_rsk"
    "${E2STUDIO_DIR}/rzn/fsp/src/bsp/cmsis/Device/RENESAS/Source"
    "${E2STUDIO_DIR}/rzn/fsp/src/bsp/cmsis/Device/RENESAS/Source/cr"
    "${E2STUDIO_DIR}/rzn/fsp/src/bsp/mcu/all"
    "${E2STUDIO_DIR}/rzn/fsp/src/bsp/mcu/all/cr"
    "${E2STUDIO_DIR}/rzn/fsp/src/bsp/mcu/rzn2l"
    "${E2STUDIO_DIR}/rzn/fsp/src/r_ether_phy"
    "${E2STUDIO_DIR}/rzn/fsp/src/r_ether_selector"
    "${E2STUDIO_DIR}/rzn/fsp/src/r_ethsw"
    "${E2STUDIO_DIR}/rzn/fsp/src/r_gmac"
    "${E2STUDIO_DIR}/rzn/fsp/src/r_gpt"
    "${E2STUDIO_DIR}/rzn/fsp/src/r_ioport"
    "${E2STUDIO_DIR}/rzn/fsp/src/r_sci_uart"
    "${E2STUDIO_DIR}/rzn/fsp/src/rm_freertos_port/cr"
    "${E2STUDIO_DIR}/rzn_gen"
    "${E2STUDIO_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/application"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/ether_netif"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/lwip_port"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/lwip_port/lwip_osal/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/serial_io"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/oss/lwip/src/api"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/oss/lwip/src/apps/lwiperf"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/oss/lwip/src/apps/sntp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/oss/lwip/src/core"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/oss/lwip/src/core/ipv4"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/oss/lwip/src/core/ipv6"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/oss/lwip/src/netif"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/oss/lwip/src/netif/ppp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/oss/lwip/src/netif/ppp/polarssl"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/ethercat/app"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/ethercat/port"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/ethercat/soem/osal"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/ethercat/soem/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/renesas/module/ethercat/soem/oshw"
)

set(PROJECT_SOURCES
        common/renesas/module/gpt/gpt.c
        common/renesas/module/gpt/gpt.h
        common/renesas/module/ethercat/app/ethercat_motion.c
        common/renesas/module/ethercat/app/ethercat_motion.h)
foreach(SOURCE_DIR IN LISTS SOURCE_DIRS)
    file(GLOB DIR_SOURCES CONFIGURE_DEPENDS "${SOURCE_DIR}/*.c")
    list(APPEND PROJECT_SOURCES ${DIR_SOURCES})
endforeach()

if(NOT PROJECT_SOURCES)
    message(FATAL_ERROR "No source files were found.")
endif()

add_executable(${TARGET_NAME} ${PROJECT_SOURCES})
set_target_properties(${TARGET_NAME} PROPERTIES OUTPUT_NAME "${E2STUDIO_OUTPUT_NAME}")
set_target_properties(${TARGET_NAME} PROPERTIES
    LINK_DEPENDS "${FSP_CMAKE_LINKER_SCRIPT};${FSP_LINKER_SCRIPT};${FSP_MEMORY_REGIONS}"
)

target_include_directories(${TARGET_NAME} PRIVATE ${INCLUDE_DIRS})
target_compile_definitions(${TARGET_NAME} PRIVATE ${COMMON_DEFINITIONS})
target_compile_options(${TARGET_NAME} PRIVATE ${COMMON_COMPILE_OPTIONS})

target_link_options(${TARGET_NAME} PRIVATE
    -mcpu=cortex-r52
    -mthumb
    -mfloat-abi=hard
    -mfpu=neon-fp-armv8
    -fdiagnostics-parseable-fixits
    -Og
    -fmessage-length=0
    -fsigned-char
    -ffunction-sections
    -fdata-sections
    -Wunused
    -Wuninitialized
    -Wall
    -Wextra
    -Wmissing-declarations
    -Wconversion
    -Wpointer-arith
    -Wshadow
    -Wlogical-op
    -Waggregate-return
    -Wfloat-equal
    -Wnull-dereference
    -g
    -gdwarf-4
    "-T${FSP_CMAKE_LINKER_SCRIPT}"
    "-L${FSP_LINKER_DIR}"
    "-L${E2STUDIO_DEBUG_DIR}"
    -nostartfiles
    -Xlinker --gc-sections
    "-Wl,-Map,${E2STUDIO_OUTPUT_NAME}.map"
    --specs=nano.specs
    -Wl,-esystem_init
    --specs=rdimon.specs
    -Wl,--start-group
    -Wl,--end-group
)

if(ARM_NONE_EABI_OBJCOPY)
    add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
        COMMAND "${ARM_NONE_EABI_OBJCOPY}" -O srec "$<TARGET_FILE:${TARGET_NAME}>" "${E2STUDIO_OUTPUT_NAME}.srec"
        BYPRODUCTS "${E2STUDIO_OUTPUT_NAME}.srec"
        COMMENT "Generating ${E2STUDIO_OUTPUT_NAME}.srec"
    )
endif()

if(ARM_NONE_EABI_SIZE)
    add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
        COMMAND "${ARM_NONE_EABI_SIZE}" --format=berkeley "$<TARGET_FILE:${TARGET_NAME}>"
        COMMENT "Printing target size"
    )
endif()

add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${E2STUDIO_DEBUG_DIR}"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "$<TARGET_FILE:${TARGET_NAME}>" "${E2STUDIO_DEBUG_DIR}/${E2STUDIO_OUTPUT_NAME}.elf"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_CURRENT_BINARY_DIR}/${E2STUDIO_OUTPUT_NAME}.srec" "${E2STUDIO_DEBUG_DIR}/${E2STUDIO_OUTPUT_NAME}.srec"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_CURRENT_BINARY_DIR}/${E2STUDIO_OUTPUT_NAME}.map" "${E2STUDIO_DEBUG_DIR}/${E2STUDIO_OUTPUT_NAME}.map"
    COMMENT "Copying firmware outputs to e2 studio Debug directory"
)
