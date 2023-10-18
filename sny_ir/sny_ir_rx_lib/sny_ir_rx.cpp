// sny_ir_rx.cpp
// rev 1 - shabaz - oct 2023
// license: BSD-3-Clause

// SDK types and declarations
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "sny_ir_rx.h"

// import the assembled PIO state machine program
#include "sny_ir_rx.pio.h"

// Claim an unused state machine on the specified PIO and configure it
// to receive SNY protocol frames from the IR receiver connected pin_num.
// Returns: the state machine number on success, otherwise -1
int sny_ir_rx_init(PIO pio, uint pin_num) {

    // disable pull-up and pull-down on gpio pin
    gpio_disable_pulls(pin_num);

    // install the program in the PIO shared instruction space
    uint offset;
    if (pio_can_add_program(pio, &sny_ir_rx_program)) {
        offset = pio_add_program(pio, &sny_ir_rx_program);
    } else {
        return -1;      // the program could not be added
    }

    // claim an unused state machine on this PIO
    int sm = pio_claim_unused_sm(pio, true);
    if (sm == -1) {
        return -1;      // we were unable to claim a state machine
    }

    // configure and enable the state machine
    sny_ir_rx_program_init(pio, sm, offset, pin_num);

    return sm;
}


// sny_ir_rx_decode_frame: decodes the 32-byte frame and returns SNY protocol field values
bool sny_ir_rx_decode_frame(uint32_t frame,
                            bool *first_press,
                            uint8_t *sny_device,
                            uint8_t *sny_command,
                            uint8_t *extended_device) {
    // according to The Internet, the received frame consists of the following fields:
    // 7 bits: Command
    // 5 bits: Device
    // 8 bits: Extended Device
    // each field has the least significant bit first

    // In this implementation, the frame is received as a 32-bit value,
    // with bits 31:24 being the Extended Device,
    // 23:19 being the Device,
    // and 18:12 being the Command,
    // and bit 11 being the first_press flag (1 = first press, 0 = repeat press)

    *extended_device = (uint8_t)((frame >> 24) & 0xff);
    *sny_device = (uint8_t)((frame >> 19) & 0x1f);
    *sny_command = (uint8_t)((frame >> 12) & 0x7f);
    *first_press = (bool)((frame >> 11) & 0x1);

    return true;
}
