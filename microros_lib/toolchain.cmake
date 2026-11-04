# Toolchain do budowy libmicroros.a dla RP2350 (Cortex-M33, softfp) - flagi takie same jak
# pico-sdk/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake, zeby ABI pasowalo do firmware.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_SYSTEM_PROCESSOR cortex-m33)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if(DEFINED ENV{PICO_TOOLCHAIN_PATH})
    set(TC_BIN "$ENV{PICO_TOOLCHAIN_PATH}/bin/")
else()
    set(TC_BIN "")
endif()
set(CMAKE_C_COMPILER   ${TC_BIN}arm-none-eabi-gcc CACHE FILEPATH "C compiler")
set(CMAKE_CXX_COMPILER ${TC_BIN}arm-none-eabi-g++ CACHE FILEPATH "C++ compiler")
set(CMAKE_AR           ${TC_BIN}arm-none-eabi-ar CACHE FILEPATH "ar")
set(CMAKE_C_COMPILER_WORKS 1 CACHE INTERNAL "")
set(CMAKE_CXX_COMPILER_WORKS 1 CACHE INTERNAL "")

set(FLAGS "-O2 -mcpu=cortex-m33 -mthumb -march=armv8-m.main+fp+dsp -mfloat-abi=softfp -ffunction-sections -fdata-sections -fno-exceptions -nostdlib -D'RCUTILS_LOG_MIN_SEVERITY=RCUTILS_LOG_MIN_SEVERITY_NONE'" CACHE STRING "" FORCE)

set(CMAKE_C_FLAGS_INIT   "-std=c11 ${FLAGS} -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "-std=c++14 ${FLAGS} -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)
