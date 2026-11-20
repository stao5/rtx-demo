# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(rtx C CXX ASM)

# initialize the sdk
pico_sdk_init()

add_executable(rtx rtx.c)

# must match with executable name
target_link_libraries(  rtx
                        PRIVATE
                        pico_stdlib
                        pico_bootsel_via_double_reset
                        pico_multicore
                        pico_mem_ops
                        hardware_pio
                        hardware_dma
                        hardware_vreg
                        hardware_clocks
                        hardware_adc)

# must match with executable name
pico_add_extra_outputs(rtx)
