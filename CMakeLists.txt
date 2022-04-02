# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "C:/Projects/Pico/pico-sdk")
# set(PICO_SDK_PATH "pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(KT-5 C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

set(PICO_CXX_ENABLE_RTTI 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(KT-5 KT-5.cpp Servo.cpp ADCEngine.cpp SpeedMeasurement.cpp OLED.cpp)

pico_set_program_name(KT-5 "KT-5")
pico_set_program_version(KT-5 "0.1")

pico_enable_stdio_uart(KT-5 0)
pico_enable_stdio_usb(KT-5 1)

# Add the standard library to the build
target_link_libraries(KT-5 pico_stdlib pico_sync)

# Add any user requested libraries
target_link_libraries(KT-5
        hardware_dma
        hardware_timer
        hardware_watchdog
        hardware_clocks
        hardware_gpio
        hardware_pwm
        hardware_irq
        hardware_rtc
        hardware_adc
)

pico_add_extra_outputs(KT-5)

