cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico2 CACHE STRING "Board type")
include(pico_sdk_import.cmake)
project(rov_bms C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

set(MICROROS_DIR ${CMAKE_CURRENT_LIST_DIR}/../microros_lib/libmicroros)
if (NOT EXISTS ${MICROROS_DIR}/libmicroros.a)
    message(FATAL_ERROR "Brak ${MICROROS_DIR}/libmicroros.a - uruchom microros_lib/build_lib.sh")
endif()

add_executable(rov_bms main.c microros_transport.c can2040/can2040.c)
pico_generate_pio_header(rov_bms ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)
target_include_directories(rov_bms PRIVATE ${MICROROS_DIR}/include ${CMAKE_CURRENT_LIST_DIR}/can2040)
target_link_directories(rov_bms PRIVATE ${MICROROS_DIR})
target_link_libraries(rov_bms pico_stdlib pico_stdio_usb hardware_pio hardware_irq hardware_clocks microros)
target_compile_options(rov_bms PRIVATE -ffunction-sections -fdata-sections)
set_source_files_properties(can2040/can2040.c PROPERTIES COMPILE_OPTIONS "-O2;-Wno-unused-parameter")

pico_enable_stdio_usb(rov_bms 1)
pico_enable_stdio_uart(rov_bms 0)
target_compile_definitions(rov_bms PRIVATE
    PICO_STDIO_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_DEFAULT_CRLF=0
    PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=20000
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0)
pico_add_extra_outputs(rov_bms)
