# RP2040 firmware build for the C++ port of the Cart-and-Rail PID lab.
#
# Requires the Raspberry Pi Pico SDK and the arm-none-eabi toolchain:
#
#   cmake -B build -S . -DPICO_SDK_PATH=/path/to/pico-sdk
#   cmake --build build
#
# Produces `build/pid_cart_controller_cpp.uf2`, which you can drag onto the
# Pico's BOOTSEL mass-storage drive.
#
# The host-side unit tests are a separate, SDK-free project: see `tests/`.

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pid_cart_controller_cpp C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

add_executable(pid_cart_controller_cpp src/main.cpp)

target_include_directories(pid_cart_controller_cpp PRIVATE include)

target_link_libraries(pid_cart_controller_cpp
        pico_stdlib
        hardware_adc
        hardware_gpio
        hardware_pwm
)

target_compile_options(pid_cart_controller_cpp PRIVATE -Wall -Wextra)

# `printf` logging over USB CDC (the Rust port uses defmt over RTT instead, so
# it needs no equivalent -- see cpp/README.md). Flip these to use the UART on
# GPIO0/GPIO1 instead if you have a serial adapter wired up.
pico_enable_stdio_usb(pid_cart_controller_cpp 1)
pico_enable_stdio_uart(pid_cart_controller_cpp 0)

# .uf2 (for BOOTSEL flashing) alongside the .elf (for probe-rs/openocd/picotool).
pico_add_extra_outputs(pid_cart_controller_cpp)
