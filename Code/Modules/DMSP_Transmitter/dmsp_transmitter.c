/*
 * File: dmps_receiver.c
 * Author: CHORDS Group
 * Date: February 20, 2025
 * Description:    This code will transmites the data being sent between each module. This code utilizes state machines to generate the signal.
 * Version: 1.0
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "dmsp_transmit.pio.h"

// Define GPIO pins
#define GLOBAL_CLK_PIN 14  // GPIO 14 as global clock input
#define CHANNEL_CTRL_PIN 15 // GPIO 15 as sync signal input
#define DMSP_OUT_PIN 16    // GPIO 16 as output for DMSP data

// Number of test frames (0xFA000000 to 0xFA00FFFF)
#define NUM_TEST_FRAMES 65536

// Test data array
uint32_t test_data[NUM_TEST_FRAMES];

// DMA channel
int dma_chan;

// Function to initialize the DMSP transmitter
void dmsp_transmitter_init(PIO pio, uint sm, uint offset) {
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
    sm_config_set_out_shift(&c, false, false, 32); // Shift out 32 bits, MSB first

    // Initialize the state machine
    pio_sm_init(pio, sm, offset, &c);

    // Start the state machine
    pio_sm_set_enabled(pio, sm, true);
}

// Function to generate test data
void generate_test_data() {
    for (int i = 0; i < NUM_TEST_FRAMES; i++) {
        test_data[i] = 0xFA000000 + i;  // Generate frames from 0xFA000000 to 0xFA00FFFF
    }
}

// DMA interrupt handler
void dma_handler() {
    // Clear the interrupt flag
    dma_hw->ints0 = 1u << dma_chan;

    // Reset read address and restart DMA
    dma_channel_set_read_addr(dma_chan, test_data, false);  // Reset read address
    dma_channel_set_trans_count(dma_chan, NUM_TEST_FRAMES, true);  // Restart DMA
}

int main() {
    stdio_init_all();

    // Initialize the PIO and state machine
    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &dmsp_transmit_program);

    // Initialize the DMSP transmitter
    dmsp_transmitter_init(pio, sm, offset);

    // Generate test data
    generate_test_data();

    // Set up DMA
    dma_chan = dma_claim_unused_channel(true);  // Claim a DMA channel
    dma_channel_config c = dma_channel_get_default_config(dma_chan);

    // Configure DMA to read from test_data and write to PIO TX FIFO
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);  // Transfer 32-bit words
    channel_config_set_read_increment(&c, true);             // Increment read address
    channel_config_set_write_increment(&c, false);           // Do not increment write address
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true)); // Use PIO TX FIFO DREQ

    // Remove ring buffer (fix for potential address wrapping issues)
    channel_config_set_ring(&c, false, 0);

    // Enable self-chaining so the DMA loops automatically
    channel_config_set_chain_to(&c, dma_chan);

    // Start DMA transfer with the adjusted transfer count
    dma_channel_configure(
        dma_chan,              // Channel
        &c,                    // Configuration
        &pio->txf[sm],         // Write address (PIO TX FIFO)
        test_data,             // Read address (test_data array)
        NUM_TEST_FRAMES * 2,   // Increased transfer count for testing (doubling)
        true                   // Start immediately
    );

    // Enable DMA interrupt
    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    printf("DMSP Transmitter Initialized\n");

    // Main loop (monitor DMA progress)
    while (true) {
        // Add a small delay (optional)
        sleep_ms(10);
    }
}
