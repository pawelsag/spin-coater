cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(/home/archspring/git/pico-sdk/pico_sdk_init.cmake)

project(example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w) # Set the board used
set(PICO_PLATFORM rp2040) # Set the board used

pico_sdk_init()

add_executable(hello_pwm
        hello_pwm.cc
        )


# pull in common dependencies and additional pwm hardware support
target_link_libraries(hello_pwm pico_stdlib hardware_pwm)

# create map/bin/hex file etc.
pico_add_extra_outputs(hello_pwm)
pico_enable_stdio_usb(hello_pwm 1)
pico_enable_stdio_uart(hello_pwm 1)

