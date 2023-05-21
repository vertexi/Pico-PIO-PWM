cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# keep disassembler file
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -save-temps=obj")

# # Do not optimize
# set(DCMAKE_BUILD_TYPE Debug)
# set(PICO_DEOPTIMIZED_DEBUG 1)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

pico_sdk_init()

if (TARGET tinyusb_device)
    add_executable(test)

    pico_generate_pio_header(test ${CMAKE_CURRENT_LIST_DIR}/pwm.pio)
    pico_generate_pio_header(test ${CMAKE_CURRENT_LIST_DIR}/clock.pio)
    target_sources(test PRIVATE test.c)

    pico_enable_stdio_usb(test 0)
    pico_enable_stdio_uart(test 1)

    target_link_libraries(test
        pico_stdlib
        hardware_spi
        hardware_adc
		hardware_dma
        hardware_pwm
        hardware_interp
		# For the dummy output:
		hardware_pio
		pico_multicore
        pico_sync
    )
    pico_add_extra_outputs(test)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()