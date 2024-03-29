# Toolchain Settings
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/toolchain.cmake)

set(CMAKE_VERBOSE_MAKEFILE ON CACHE BOOL "ON" FORCE)

cmake_minimum_required(VERSION 3.27.8)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED true)

project(Proteus LANGUAGES CXX ASM)

add_executable(${PROJECT_NAME}.elf
        Startup/startup_mk60d10.cpp

        Drivers/mcg.cpp
        Drivers/i2c.cpp
        Drivers/spi.cpp
        Drivers/uart.cpp
        Drivers/ff.cpp
        Drivers/SDHC.cpp
        Drivers/lptmr.cpp
        Drivers/adc16.cpp
        Drivers/FEHXBee.cpp
        Drivers/FEHPropeller.cpp
        Drivers/exception_handler.cpp
        Drivers/exception_handler.s

        Libraries/FEHMotor.cpp
        Libraries/FEHUtility.cpp
        Libraries/FEHIO.cpp
        Libraries/FEHBuzzer.cpp
        Libraries/FEHServo.cpp
        Libraries/FEHLCD.cpp
        Libraries/FastLCD.cpp
        Libraries/FEHAccel.cpp
        Libraries/FEHBattery.cpp
        Libraries/FEHRCS.cpp
        Libraries/FEHSD.cpp
        Libraries/FEHRandom.cpp

        FEHProteus.cpp

        main.cpp
)

# Compiler flags
target_compile_options(${PROJECT_NAME}.elf PRIVATE -Wall -Os)

# Add includes
include_directories(. Libraries/ Drivers/ Startup/)

# Add linker script
target_link_options(${PROJECT_NAME}.elf PRIVATE
        -Wl,--print-memory-usage,--gc-sections,-Map,${PROJECT_NAME}.map
        -T${CMAKE_CURRENT_SOURCE_DIR}/Linker/MK60DN512Z_flash.ld
)

# Add FEH scripts
file(COPY Tools/copyS19 Tools/copyS19.bat Tools/sizeGet Tools/sizeGet.bat DESTINATION Tools/)

# Create S19 file from Proteus.elf and invoke the script to copy it to the SD card.
add_custom_target(${PROJECT_NAME}.s19
        DEPENDS ${PROJECT_NAME}.elf
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND arm-none-eabi-size ${PROJECT_NAME}.elf > size.txt
        COMMAND ${CMAKE_OBJCOPY} -O srec --srec-len=40 --srec-forceS3 ${PROJECT_NAME}.elf ${PROJECT_NAME}.s19
        COMMAND Tools/copyS19)