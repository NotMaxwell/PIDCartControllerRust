# Host-side unit tests for the pure control math in `cpp/include/cart`.
#
# Deliberately a standalone CMake project with no pico-sdk dependency, so it
# builds with a plain host compiler -- the counterpart to
# `cargo test --lib --target <host-triple>` on the Rust side.
#
#   cmake -B build -S .
#   cmake --build build
#   ctest --test-dir build --output-on-failure   # or just run ./build/cart_tests

cmake_minimum_required(VERSION 3.13)

project(cart_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_executable(cart_tests
        main.cpp
        test_pid.cpp
        test_rail.cpp
)

target_include_directories(cart_tests PRIVATE ../include .)

if (MSVC)
    target_compile_options(cart_tests PRIVATE /W4)
else ()
    target_compile_options(cart_tests PRIVATE -Wall -Wextra)
endif ()

add_test(NAME cart_tests COMMAND cart_tests)
