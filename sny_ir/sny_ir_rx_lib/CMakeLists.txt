# build a normal library
#
add_library(sny_ir_rx_lib sny_ir_rx.cpp)

# invoke pio_asm to assemble the state machine program
#
pico_generate_pio_header(sny_ir_rx_lib ${CMAKE_CURRENT_LIST_DIR}/sny_ir_rx.pio)

target_link_libraries(sny_ir_rx_lib PRIVATE
        pico_stdlib
        hardware_pio
        )

# add the `binary` directory so that the generated headers are included in the project
#
target_include_directories (sny_ir_rx_lib PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
	)
