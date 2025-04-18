cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME keypad-firmware)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

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

# Raccolta dei file sorgente dei moduli
file(GLOB MODULES_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Modules/*.c"
)

# Debug - Mostra quali moduli sono stati trovati
message("-- Moduli trovati:")
foreach(module ${MODULES_SOURCES})
    message("   - ${module}")
endforeach()

# Configurazione libreria U8G2
add_library(u8g2 STATIC)

# Disabilita l'avviso specifico per le stringhe troppo lunghe
target_compile_options(u8g2 PRIVATE "-Wno-overlength-strings")

# Aggiungi i file sorgente della libreria U8G2
file(GLOB U8G2_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/u8g2/csrc/*.c"
)
target_sources(u8g2 PRIVATE ${U8G2_SOURCES})

# Configura le directory di include per U8G2
target_include_directories(u8g2 PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/u8g2"
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/u8g2/csrc"
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Aggiungi i moduli all'eseguibile
    ${MODULES_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    # Aggiungi la directory dei moduli
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/Modules

    # Add the HAL include path
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
    # Add the u8g2 folder
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/u8g2
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/u8g2/csrc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    u8g2
)
