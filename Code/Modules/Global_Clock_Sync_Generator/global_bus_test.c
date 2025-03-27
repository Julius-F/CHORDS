/*
 * File: global_bus_test.c
 * Author: Bjorn Lavik
 * Date: March 26, 2025
 * Description:    This code generates 10 Global Clock and Global Sync signals for the CHORDS Busboard.
 * Version: 2.0
 */

// global_bus_test.c
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "clock.pio.h"

typedef struct {
    uint clock_pin;
    uint sync_pin;
    uint sm;
} clock_gen_t;

int main() {
    stdio_init_all();

    // Use pio0 instance
    PIO pio = pio0;

    // GPIO16 as synchronization trigger (RUN LED)
    const uint sync_trigger_pin = 16;
    gpio_init(sync_trigger_pin);
    gpio_set_dir(sync_trigger_pin, GPIO_OUT);
    gpio_put(sync_trigger_pin, 0);  // Initially low

    // Clock divider calculation (your specified frequency)
    float div = clock_get_hz(clk_sys) / (1.28e6 * 66);

    // Define clock and sync pins per your mapping
    clock_gen_t clock_gens[10] = {
        {3,  2,  0},  // Clock #1
        {5,  4,  1},  // Clock #2
        {21, 22, 2},  // Clock #3
        {20, 19, 3},  // Clock #4
        {18, 17, 4},  // Clock #5
        {6,  7,  5},  // Clock #6
        {8,  9,  6},  // Clock #7
        {10, 11, 7},  // Clock #8
        {13, 12, 8},  // Clock #9
        {15, 14, 9}   // Clock #10
    };

    // Load the PIO program into memory once
    uint offset = pio_add_program(pio, &clock_program);

    // Configure each clock generator state machine
    for (int i = 0; i < 10; i++) {
        clock_gen_t gen = clock_gens[i];

        // Configure SM
        pio_sm_config c = clock_program_get_default_config(offset);
        sm_config_set_set_pins(&c, gen.clock_pin, 2);  // set pins for clock and sync
        sm_config_set_in_pins(&c, sync_trigger_pin);   // pin wait input (RUN LED)

        // Init pins
        pio_gpio_init(pio, gen.clock_pin);
        pio_gpio_init(pio, gen.sync_pin);
        gpio_set_dir(gen.clock_pin, GPIO_OUT);
        gpio_set_dir(gen.sync_pin, GPIO_OUT);

        // Init SM
        sm_config_set_clkdiv(&c, div);
        pio_sm_init(pio, gen.sm, offset, &c);
        pio_sm_restart(pio, gen.sm);
        pio_sm_set_enabled(pio, gen.sm, true);
    }

    sleep_ms(10);  // Wait briefly to ensure all SMs ready at wait instruction

    // Trigger RUN LED pin high, starts all clocks simultaneously
    gpio_put(sync_trigger_pin, 1);

    // Infinite loop
    while (true) {
        tight_loop_contents();
    }
}

