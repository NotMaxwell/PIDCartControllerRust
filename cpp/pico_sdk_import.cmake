# Locate the Raspberry Pi Pico SDK and pull in its CMake integration.
#
# This is a trimmed version of the `pico_sdk_import.cmake` shipped with the
# SDK (the git-auto-fetch branch is omitted): point `PICO_SDK_PATH` at a
# checkout you already have, via the environment variable or `-DPICO_SDK_PATH=`.
# You can also just copy the SDK's own `external/pico_sdk_import.cmake` over
# this file if you prefer to track it verbatim.

if (DEFINED ENV{PICO_SDK_PATH} AND (NOT PICO_SDK_PATH))
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
    message("Using PICO_SDK_PATH from environment ('${PICO_SDK_PATH}')")
endif ()

if (NOT PICO_SDK_PATH)
    message(FATAL_ERROR
            "PICO_SDK_PATH is not set. Clone https://github.com/raspberrypi/pico-sdk and either "
            "set the PICO_SDK_PATH environment variable or pass -DPICO_SDK_PATH=<path> to cmake.")
endif ()

get_filename_component(PICO_SDK_PATH "${PICO_SDK_PATH}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
if (NOT EXISTS ${PICO_SDK_PATH})
    message(FATAL_ERROR "Directory '${PICO_SDK_PATH}' not found")
endif ()

set(PICO_SDK_INIT_CMAKE_FILE ${PICO_SDK_PATH}/pico_sdk_init.cmake)
if (NOT EXISTS ${PICO_SDK_INIT_CMAKE_FILE})
    message(FATAL_ERROR
            "Directory '${PICO_SDK_PATH}' does not appear to contain the Raspberry Pi Pico SDK")
endif ()

set(PICO_SDK_PATH ${PICO_SDK_PATH} CACHE PATH "Path to the Raspberry Pi Pico SDK" FORCE)

include(${PICO_SDK_INIT_CMAKE_FILE})
