# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2026 Florian Frankenberger

cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(bluebridge C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(bluebridge
    src/main.c
    src/lpt.c
    src/ppa.c
    src/zip.c
    src/engine.c
    src/msc.c
    src/shell.c
    src/console.c
    src/led.c
    src/usb_descriptors.c
    src/wdt.c
)

target_include_directories(bluebridge PRIVATE src)
target_compile_options(bluebridge PRIVATE -Wall -Wextra -Wno-unused-parameter)

target_link_libraries(bluebridge
    pico_stdlib
    pico_multicore
    pico_unique_id
    tinyusb_device
    hardware_gpio
    hardware_clocks
    hardware_watchdog
)

# GP0/GP1 are D0/D1 on this board: the default UART stdio must stay off.
# stdio goes over our own CDC interface instead (src/console.c).
pico_enable_stdio_uart(bluebridge 0)
pico_enable_stdio_usb(bluebridge 0)

pico_add_extra_outputs(bluebridge)
