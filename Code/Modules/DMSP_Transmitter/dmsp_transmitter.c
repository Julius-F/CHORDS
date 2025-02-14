#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "dmsp_transmit.pio.h"

// Define GPIO pins
#define GLOBAL_CLK_PIN 14  // GPIO 14 as global clock input
#define CHANNEL_CTRL_PIN 15 // GPIO 15 as sync signal input
#define DMSP_OUT_PIN 16    // GPIO 16 as output for DMSP data

int main() {
    // Initialize the PIO and state machine
    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &dmsp_transmit_program);

    // Configure GPIO pins
    pio_gpio_init(pio, DMSP_OUT_PIN);      // GPIO 16 as output
    pio_gpio_init(pio, GLOBAL_CLK_PIN);    // GPIO 14 as input (global clock)
    pio_gpio_init(pio, CHANNEL_CTRL_PIN);  // GPIO 15 as input (sync signal)

    // Set pin directions
    pio_sm_set_consecutive_pindirs(pio, sm, DMSP_OUT_PIN, 1, true);      // GPIO 16 as output
    pio_sm_set_consecutive_pindirs(pio, sm, GLOBAL_CLK_PIN, 1, false);   // GPIO 14 as input
    pio_sm_set_consecutive_pindirs(pio, sm, CHANNEL_CTRL_PIN, 1, false); // GPIO 15 as input

    // Configure the state machine
    pio_sm_config c = dmsp_transmit_program_get_default_config(offset);
    sm_config_set_out_pins(&c, DMSP_OUT_PIN, 1);  // GPIO 16 as output
    sm_config_set_in_pins(&c, GLOBAL_CLK_PIN);    // GPIO 14 as input (global clock)
    sm_config_set_jmp_pin(&c, CHANNEL_CTRL_PIN);  // GPIO 15 as input (sync signal)
    // Set shift direction to MSB-first
    sm_config_set_out_shift(&c, false, false, 32); // Shift out 32 bits, MSB first
    sm_config_set_in_shift(&c, false, false, 32); // No input shifting

    // Initialize the state machine
    pio_sm_init(pio, sm, offset, &c);

    // Start the state machine
    pio_sm_set_enabled(pio, sm, true);

    // Main loop (push data to TX FIFO whenever it is ready)
    while (true) {
        // Check if the TX FIFO is ready to accept new data
        if (pio_sm_is_tx_fifo_empty(pio, sm)) {
            uint32_t data = 0xAF0AF0A0;  // Example data (replace with actual data)
            pio_sm_put_blocking(pio, sm, data);  // Push data to the TX FIFO
        }
    }
}
