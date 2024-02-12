cmake_minimum_required(VERSION 3.13)

# Set required variables
set(PICO_BOARD pico_w)
set(PICO_SDK_PATH ~/pico-sdk)

message("PICO_BOARD is '${PICO_BOARD}'")

if (DEFINED WIFI_SSID)    
    message("Using WIFI_SSID ('${WIFI_SSID}')")
else()
    message("WIFI_SSID not specified")
endif()

if (DEFINED WIFI_PASSWORD)    
    message("WIFI_PASSWORD has been specified")
else()
    message("WIFI_PASSWORD not specified")
endif()

message("=============Running pico_sdk_import.cmake...===============")
include(pico_sdk_import.cmake)
message("=========================DONE===============================")

project(XboxController C CXX ASM)
message("Project name: ${CMAKE_PROJECT_NAME}")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

message("=============Running pico_sdk_init()...=====================")
pico_sdk_init()
message("=========================DONE===============================")

add_executable(${CMAKE_PROJECT_NAME} 
    "src/xbox_controller.c"
    "src/hog_host.c"    
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR} # For btstack config    
)


target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    RUNNING_AS_CLIENT=1
)

pico_enable_stdio_usb(${CMAKE_PROJECT_NAME} 1)
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 0)

pico_add_extra_outputs(${CMAKE_PROJECT_NAME})

target_link_libraries(${CMAKE_PROJECT_NAME} 
    pico_stdlib                 # for core functionality
    pico_cyw43_arch_none        # we need Wifi to access the GPIO, but we don't need anything else
    pico_btstack_ble
    pico_btstack_cyw43
    hardware_pwm
)