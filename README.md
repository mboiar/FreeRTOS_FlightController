# STM32F411 BlackPill based quadcopter flight controller

## Toolchain
STM32CubeMX 6.14 - STM32CubeCLT 1.18 - GCC  - CMake 3.28 - Ninja

## How to build
In the project folder run: `cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_TOOLCHAIN_FILE=/path/to/gcc-arm-none-eabi.cmake -S . -B build/debug -G Ninja && cmake --build build/debug`

## How to flash
In the project folder run: ``