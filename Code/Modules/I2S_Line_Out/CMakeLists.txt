# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(i2s_line_out C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(i2s_line_out i2s_line_out.c )

pico_set_program_name(i2s_line_out "i2s_line_out")
pico_set_program_version(i2s_line_out "0.1")

# Generate PIO header
pico_generate_pio_header(i2s_line_out ${CMAKE_CURRENT_LIST_DIR}/dmsp_receive.pio)
pico_generate_pio_header(i2s_line_out ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)
pico_generate_pio_header(i2s_line_out ${CMAKE_CURRENT_LIST_DIR}/ws.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(i2s_line_out 0)
pico_enable_stdio_usb(i2s_line_out 1)

# Add the standard library to the build
target_link_libraries(i2s_line_out
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(i2s_line_out PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(i2s_line_out 
        hardware_dma
        hardware_pio
        )

pico_add_extra_outputs(i2s_line_out)

