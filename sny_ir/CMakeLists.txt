cmake_minimum_required(VERSION 3.16)
include(pico_sdk_import.cmake)
# set(PICO_BOARD pico_w)

# set the project name (change to suit)
project(sny_ir)

pico_sdk_init()

# add_subdirectory()
add_subdirectory(sny_ir_rx_lib)


# include_directories()
include_directories(./sny_ir_rx_lib)

# rest of your project
add_executable(${CMAKE_PROJECT_NAME}
        main.cpp
        )

target_include_directories(${CMAKE_PROJECT_NAME}
        PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
        )

target_link_libraries(${CMAKE_PROJECT_NAME}
        sny_ir_rx_lib
        pico_stdlib
        hardware_pio
        )

# adjust to enable stdio via usb, or uart
pico_enable_stdio_usb(${CMAKE_PROJECT_NAME} 1)
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(${CMAKE_PROJECT_NAME})
