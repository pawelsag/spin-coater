cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w) # Set the board used
set(PICO_PLATFORM rp2040) # Set the board used

pico_sdk_init()


add_executable(spin_coater
        spin_coater.cc
        )

target_include_directories(spin_coater PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )

target_compile_definitions(spin_coater PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(spin_coater pico_stdlib hardware_pwm pico_cyw43_arch_lwip_threadsafe_background)

# create map/bin/hex file etc.
pico_add_extra_outputs(spin_coater)
pico_enable_stdio_usb(spin_coater 1)
pico_enable_stdio_uart(spin_coater 1)

