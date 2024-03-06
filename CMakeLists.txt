# Toolchain Settings
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/toolchain.cmake)

cmake_minimum_required(VERSION 3.27.8)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(Proteus LANGUAGES C CXX)
add_executable(${PROJECT_NAME}.elf)

target_sources(${PROJECT_NAME}.elf
        PRIVATE
        Startup/startup_mk60d10.cpp
        Libraries/FEHMotor.cpp
        Drivers/mcg.cpp
        Drivers/i2c.cpp
        Drivers/spi.cpp
        Drivers/uart.cpp
        Drivers/ff.cpp
        Drivers/SDHC.cpp
        Drivers/lptmr.cpp
        FEHProteus.cpp
        Drivers/FEHPropeller.cpp
        Libraries/FEHUtility.cpp
        Libraries/FEHIO.cpp
        Drivers/adc16.cpp
        Libraries/FEHBuzzer.cpp
        Libraries/FEHServo.cpp
        Libraries/FEHLCD.cpp
        Libraries/FastLCD.cpp
        Libraries/FEHAccel.cpp
        Libraries/FEHBattery.cpp
        Drivers/FEHXBee.cpp
        Libraries/FEHRCS.cpp
        Libraries/FEHSD.cpp
        Libraries/FEHRandom.cpp

        main.cpp
)

# Add includes
include_directories(. Libraries/ Drivers/ Startup/)

# Add linker script
target_link_options(${PROJECT_NAME}.elf PRIVATE -Xlinker --gc-sections -T${CMAKE_CURRENT_SOURCE_DIR}/Linker/MK60DN512Z_flash.ld)

# Add FEH scripts
file(COPY Tools/copyS19 Tools/copyS19.bat Tools/sizeGet Tools/sizeGet.bat DESTINATION Tools/)

# Create S19 file from Proteus.elf and copy invoke the script to copy it to the SD card.
add_custom_command(TARGET ${PROJECT_NAME}.elf
        POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND arm-none-eabi-size ${PROJECT_NAME}.elf > size.txt
        COMMAND ${CMAKE_OBJCOPY} ARGS -O srec --srec-len=40 --srec-forceS3 ${PROJECT_NAME}.elf ${PROJECT_NAME}.s19
        COMMAND Tools/sizeGet
        COMMAND Tools/copyS19
)
