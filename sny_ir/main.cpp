// ****************************************************************
// *  main.cpp for sny_ir project, compatible with SNY
// *  remote control and displays the received content
// *  rev 1 - shabaz - oct 2023
// *  BSD-3-Clause license
// ****************************************************************

#include <stdio.h>
#include "pico/stdlib.h"
#include "sny_ir_rx.h"

int main() {
    stdio_init_all();

    PIO pio = pio0; // select PIO 0 or 1
    uint rx_gpio = 15; // GPIO connected to the IR receiver

    // configure and enable the state machines
    int rx_sm = sny_ir_rx_init(pio, rx_gpio);         // uses one state machine and 9 instructions

    if (rx_sm == -1) {
        printf("ERROR - could not configure PIO!\n");
        return -1;
    }

    // receive frames
    bool first_press = false;
    uint8_t rx_device;
    uint8_t rx_command;
    uint8_t rx_extended_device;
    while (true) {
        // display any frames in the receive FIFO
        while (!pio_sm_is_rx_fifo_empty(pio, rx_sm)) {
            uint32_t rx_frame = pio_sm_get(pio, rx_sm);

            if (sny_ir_rx_decode_frame(rx_frame, &first_press, &rx_device, &rx_command, &rx_extended_device)) {
                if (first_press) {
                    printf("rx 0x%08x: %d.%d, command=%d\n", rx_frame, rx_device, rx_extended_device, rx_command);
                } else {
                    printf("    repeat 0x%08x: %d.%d, command=%d\n", rx_frame, rx_device, rx_extended_device, rx_command);
                }
            } else {
                printf("raw rx: %08x\n", rx_frame);
            }
        }

        sleep_ms(100);

    }
}
