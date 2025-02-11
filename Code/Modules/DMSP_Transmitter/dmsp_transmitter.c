#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "dmsp_transmit.pio.h"

#define GLOBAL_CLK_PIN 16
#define CHANNEL_CTRL_PIN 17
#define DMSP_OUT_PIN 18

int main() {
    stdio_init_all();

    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &dmsp_transmit_program);

    dmsp_transmit_program_init(pio, sm, offset, 
                              GLOBAL_CLK_PIN, CHANNEL_CTRL_PIN, DMSP_OUT_PIN);

    // Send continuous test pattern
    while (true) {
        pio_sm_put_blocking(pio, sm, 0xF0F0F0F0); // Alternating bits
        sleep_ms(64); // Matches 500Hz × 32 bits
    }
}
