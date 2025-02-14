#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "dmsp_receive.pio.h"

// Define GPIO pins
#define DMSP_IN_PIN 16         // GPIO 16 as input for DMSP data
#define GLOBAL_CLK_PIN 14      // GPIO 14 as input (global clock)
#define CHANNEL_CTRL_PIN 15    // GPIO 15 as input (sync signal)

// Function to initialize the DMSP receiver
void dmsp_receiver_init(PIO pio, uint sm, uint offset) {
    // Configure GPIO pins
    pio_gpio_init(pio, DMSP_IN_PIN);         // GPIO 16 as input
    pio_gpio_init(pio, GLOBAL_CLK_PIN);      // GPIO 14 as input (global clock)
    pio_gpio_init(pio, CHANNEL_CTRL_PIN);    // GPIO 15 as input (sync signal)

    // Set pin directions
    pio_sm_set_consecutive_pindirs(pio, sm, DMSP_IN_PIN, 1, false);      // GPIO 16 as input
    pio_sm_set_consecutive_pindirs(pio, sm, GLOBAL_CLK_PIN, 1, false);   // GPIO 14 as input
    pio_sm_set_consecutive_pindirs(pio, sm, CHANNEL_CTRL_PIN, 1, false); // GPIO 15 as input

    // Configure the state machine
    pio_sm_config c = dmsp_receive_program_get_default_config(offset);
    sm_config_set_in_pins(&c, DMSP_IN_PIN);  // GPIO 16 as input
    sm_config_set_jmp_pin(&c, CHANNEL_CTRL_PIN);  // GPIO 15 as input (sync signal)
    sm_config_set_in_shift(&c, false, false, 32); // Shift in 32 bits, MSB first

    // Initialize the state machine
    pio_sm_init(pio, sm, offset, &c);

    // Start the state machine
    pio_sm_set_enabled(pio, sm, true);
}

// Function to receive a single 32-bit frame
uint32_t dmsp_receive_frame(PIO pio, uint sm) {
    // Pull data from the RX FIFO
    return pio_sm_get_blocking(pio, sm);
}

int main() {
    stdio_init_all();

    // Initialize the PIO and state machine
    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &dmsp_receive_program);

    // Initialize the DMSP receiver
    dmsp_receiver_init(pio, sm, offset);

    printf("DMSP Receiver Initialized\n");

    // Main loop (receive frames and print to serial monitor)
    while (true) {
        // Receive a frame
        uint32_t data = dmsp_receive_frame(pio, sm);

        // Print the received data to the serial monitor
        printf("Received Frame: 0x%08X\n", data);

        // Add a small delay (optional)
        sleep_ms(10);
    }
}
