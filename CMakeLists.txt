cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

set(SOFTWARE_SRC_PATH src)

add_compile_definitions(CM4)
add_compile_definitions(MATRIXCHECK)
add_compile_definitions(ROUNDING)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Release")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -O3")

endif()

# Set the project name
set(CMAKE_PROJECT_NAME BlackPillFlightController)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add CMSIS-DSP sources
set(CMSISCORE ${CMAKE_SOURCE_DIR}/Drivers/CMSIS)
add_subdirectory(${CMAKE_SOURCE_DIR}/CMSIS-DSP/Source bin_dsp)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

file(GLOB_RECURSE SOFTWARE_SRC "${SOFTWARE_SRC_PATH}/*.*")

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${SOFTWARE_SRC}
)

# target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wall -Wextra -Wshadow -Wundef -Wconversion -Wdouble-promotion)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${SOFTWARE_SRC_PATH}
    ${SOFTWARE_SRC_PATH}/Drivers
    ${SOFTWARE_SRC_PATH}/Tasks
    ${SOFTWARE_SRC_PATH}/Core
)

target_include_directories(${CMAKE_PROJECT_NAME} 
    SYSTEM PRIVATE
    ${SOFTWARE_SRC_PATH}/Drivers/mavlink/v2.0
    CMSIS-DSP/Include
)   

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    CMSISDSP
    # Add user defined libraries
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wno-address-of-packed-member)

target_link_options(${PROJECT_NAME} PRIVATE "-Wl,-Map=${PROJECT_NAME}.map")
