// dmsp_transmitter.c - Raspberry Pi Pico (Pico 1) DMSP Transmitter

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "dmsp_transmit.pio.h"  // PIO assembly file

#define GLOBAL_CLK_PIN 16
#define CHANNEL_CTRL_PIN 17
#define DMSP_OUT_PIN 20

void setup_pio(PIO pio, uint sm, uint offset) {
    dmsp_transmit_program_init(pio, sm, offset, GLOBAL_CLK_PIN, CHANNEL_CTRL_PIN, DMSP_OUT_PIN);
    pio_sm_set_enabled(pio, sm, true);
}

int main() {
    stdio_init_all();
    printf("DMSP Transmitter Starting...\n");

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &dmsp_transmit_program);
    setup_pio(pio, sm, offset);

    uint32_t test_data = 0xA5A5A5A5;  // Example test data
    
    while (true) {
        pio_sm_put_blocking(pio, sm, test_data);
        printf("Sent Frame: 0x%08X\n", test_data);
        test_data++;  // Increment test data for debugging
        sleep_ms(25); // Adjust timing if needed
    }
}
