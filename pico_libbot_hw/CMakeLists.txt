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
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_libbot_hw C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(pico_libbot_hw pico_libbot_hw.cpp)

pico_set_program_name(pico_libbot_hw "pico_libbot_hw")
pico_set_program_version(pico_libbot_hw "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico_libbot_hw 0)
pico_enable_stdio_usb(pico_libbot_hw 1)

# Add the standard library to the build
target_link_libraries(pico_libbot_hw pico_stdlib)

# Add the standard include files to the build
target_include_directories(pico_libbot_hw PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Generate PIO header
pico_generate_pio_header(pico_libbot_hw ${CMAKE_CURRENT_LIST_DIR}/optical_encoder.pio)

# Add library directory into build
add_subdirectory(lib)

# Add any user requested libraries
target_link_libraries(pico_libbot_hw hardware_pio hardware_pwm hardware_spi hardware_gpio RF24)

pico_add_extra_outputs(pico_libbot_hw)

