#ifndef _SNY_IR_RX_H
#define _SNY_IR_RX_H

// sny_io_rx.h
// rev 1 - shabaz - oct 2023


#include "pico/stdlib.h"
#include "hardware/pio.h"

int sny_ir_rx_init(PIO pio, uint pin_num);
bool sny_ir_rx_decode_frame(uint32_t frame,
                            bool *first_press,
                            uint8_t *sny_device,
                            uint8_t *sny_command,
                            uint8_t *extended_device);


#endif // _SNY_IR_RX_H
