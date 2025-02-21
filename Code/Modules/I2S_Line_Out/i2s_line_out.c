/*
 * File: i2s_line_out.c
 * Author: CHORDS Group
 * Date: February 20, 2025
 * Description:    This program takes the final signal and converts it to and audio output signal. This code utilizes state machines to generate the signal.
 * Version: 1.0
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "dmsp_receive.pio.h"
#include "i2s.pio.h"
#include "ws.pio.h"

// Define GPIO pins
#define DMSP_IN_PIN 16         // GPIO 16 as input for DMSP data
#define GLOBAL_CLK_PIN 14      // GPIO 14 as input (global clock)
#define CHANNEL_CTRL_PIN 15    // GPIO 15 as input (sync signal)
#define BCLK_PIN 10            // GPIO 10 as bit clock output
#define DOUT_PIN 11            // GPIO 11 as data output
#define WS_PIN 12              // GPIO 12 as word select output

// Function to initialize the DMSP receiver
void dmsp_receiver_init(PIO pio, uint sm, uint offset) {
    pio_gpio_init(pio, DMSP_IN_PIN);
    pio_gpio_init(pio, GLOBAL_CLK_PIN);
    pio_gpio_init(pio, CHANNEL_CTRL_PIN);

    pio_sm_set_consecutive_pindirs(pio, sm, DMSP_IN_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, GLOBAL_CLK_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, CHANNEL_CTRL_PIN, 1, false);

    pio_sm_config c = dmsp_receive_program_get_default_config(offset);
    sm_config_set_in_pins(&c, DMSP_IN_PIN);
    sm_config_set_jmp_pin(&c, CHANNEL_CTRL_PIN);
    sm_config_set_in_shift(&c, false, false, 32);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

// Function to initialize the I2S transmitter
void i2s_transmitter_init(PIO pio, uint sm, uint offset) {
    pio_gpio_init(pio, BCLK_PIN);
    pio_gpio_init(pio, DOUT_PIN);

    pio_sm_set_consecutive_pindirs(pio, sm, BCLK_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, DOUT_PIN, 1, true);

    pio_sm_config c = i2s_program_get_default_config(offset);
    sm_config_set_out_pins(&c, DOUT_PIN, 1);
    sm_config_set_sideset_pins(&c, BCLK_PIN);
    sm_config_set_out_shift(&c, false, true, 24);

    // Set the PIO clock divider for the I2S bit clock
    float bit_clock_freq = 39.37e3 * 800;
    float div = 150000000 / bit_clock_freq;
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

// Function to initialize the ws clock
void ws_clock_init(PIO pio, uint sm, uint offset) {
    pio_gpio_init(pio, WS_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, WS_PIN, 1, true);

    // Add the ws program
    pio_sm_config c = ws_program_get_default_config(offset);
    sm_config_set_set_pins(&c, WS_PIN, 1);

    // Initialize the x scratch register to 0
    pio_sm_exec(pio, sm, pio_encode_mov(pio_x, 0));

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}


int main() {
    stdio_init_all();

    // Initialize the PIO and state machines
    PIO pio = pio0;
    uint dmsp_sm = 0;
    uint i2s_sm = 1;
    uint ws_sm = 2;
    uint dmsp_offset = pio_add_program(pio, &dmsp_receive_program);
    uint i2s_offset = pio_add_program(pio, &i2s_program);
    uint ws_offset = pio_add_program(pio, &ws_program);

    // Initialize the DMSP receiver
    dmsp_receiver_init(pio, dmsp_sm, dmsp_offset);

    // Initialize the I2S transmitter
    i2s_transmitter_init(pio, i2s_sm, i2s_offset);

    ws_clock_init(pio, ws_sm, ws_offset);

    printf("DMSP Receiver and I2S Transmitter Initialized\n");

    // Main loop (receive DMSP frames and transmit I2S audio)
    while (true) {
        // Receive a DMSP frame
        uint32_t frame = pio_sm_get_blocking(pio, dmsp_sm);

        // Extract the 24-bit audio sample (LSB of the 32-bit frame)
        //uint32_t audio_sample = frame & 0x00FFFFFF;

        // Transmit the audio sample via I2S
        pio_sm_put_blocking(pio, i2s_sm, frame << 8);
        //there is either a but with the i2s.pio file or the dmsp_receive.pio file where every other frame isnt loaded
        //into the rx fifo. So im sending the same sample twice until i resolve the bug.
        pio_sm_put_blocking(pio, i2s_sm, frame << 8);
    }
}
