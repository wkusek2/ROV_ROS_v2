cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico2 CACHE STRING "Board type")
include(pico_sdk_import.cmake)
project(rov_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# libmicroros zbudowana przez ../microros_lib/build_lib.sh (Cortex-M33 softfp, Humble)
set(MICROROS_DIR ${CMAKE_CURRENT_LIST_DIR}/../microros_lib/libmicroros)
if (NOT EXISTS ${MICROROS_DIR}/libmicroros.a)
    message(FATAL_ERROR "Brak ${MICROROS_DIR}/libmicroros.a - uruchom microros_lib/build_lib.sh")
endif()

add_executable(rov_pico main.c microros_transport.c dshot.c kiss.c imu.c press.c)
pico_generate_pio_header(rov_pico ${CMAKE_CURRENT_LIST_DIR}/dshot_bidir.pio)
target_include_directories(rov_pico PRIVATE ${MICROROS_DIR}/include)
target_link_directories(rov_pico PRIVATE ${MICROROS_DIR})
target_link_libraries(rov_pico
    pico_stdlib pico_multicore pico_stdio_usb
    hardware_pio hardware_dma hardware_adc hardware_uart hardware_irq hardware_clocks hardware_i2c
    microros)
target_compile_options(rov_pico PRIVATE -ffunction-sections -fdata-sections)

# stdio tylko przez USB (to jest transport micro-ROS). UART0/1 zajete przez telemetrie ESC.
pico_enable_stdio_usb(rov_pico 1)
pico_enable_stdio_uart(rov_pico 0)
target_compile_definitions(rov_pico PRIVATE
    PICO_STDIO_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_DEFAULT_CRLF=0
    PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1   # 1200 baud -> BOOTSEL (picotool / flash.sh)
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=20000
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0)
pico_add_extra_outputs(rov_pico)

option(RAM_BUILD "Obraz do SRAM (picotool load -x), gdy flash nie dziala" OFF)
if (RAM_BUILD)
    pico_set_binary_type(rov_pico no_flash)
endif()
