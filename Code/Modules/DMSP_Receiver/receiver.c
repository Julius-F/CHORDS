#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "dmsp_receive.pio.h"  // Include the PIO header

#define DMSP_IN_PIN 20   // Must match the transmitter's DMSP_OUT_PIN

void setup_pio(PIO pio, uint sm, uint offset) {
    dmsp_receive_program_init(pio, sm, offset, DMSP_IN_PIN);
    pio_sm_set_enabled(pio, sm, true);
}

int main() {
    stdio_init_all();
    printf("DMSP Receiver Starting...\n");

    PIO pio = pio0;      // Using the first PIO block
    uint sm = 1;         // Use state machine 1 (avoid conflict with transmitter)
    uint offset = pio_add_program(pio, &dmsp_receive_program);
    setup_pio(pio, sm, offset);

    while (true) {
        if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            uint32_t received_data = pio_sm_get_blocking(pio, sm);
            printf("Received Frame: 0x%08X\n", received_data);
        }
    }
}
