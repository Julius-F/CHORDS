/*
 * File: global_bus_test.c
 * Author: CHORDS Group
 * Date: February 20, 2025
 * Description:    This code generates a clock signal to syncronize the data being sent between each module. Updated to support 4 channels.
 * Version: 1.1
 */

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "clock.pio.h"

int main() {
    // Initialize the PIO and select a state machine
    PIO pio = pio0;
    uint sm = 0;

    // Load the PIO program into the PIO's instruction memory
    uint offset = pio_add_program(pio, &clock_program);

    // Configure the state machine
    pio_sm_config c = clock_program_get_default_config(offset);

    // Set GPIO 0 and GPIO 1 as outputs
    sm_config_set_set_pins(&c, 0, 2);  // Set pins 0 and 1 as outputs
    pio_gpio_init(pio, 0);             // Initialize GPIO 0
    pio_gpio_init(pio, 1);             // Initialize GPIO 1

    //1.28e6 * 67

    // Set the clock divider for 1.28 MHz
    float div = clock_get_hz(clk_sys) / (1.28e6 * 66);  // 1.28 MHz clock
    sm_config_set_clkdiv(&c, div);

    // Initialize the state machine
    pio_sm_init(pio, sm, offset, &c);

    // Start the state machine
    pio_sm_set_enabled(pio, sm, true);

    // Loop forever
    while (true) {
        tight_loop_contents();
    }
}
