/*
 * File: dmps_receiver.c
 * Author: CHORDS Group
 * Date: February 20, 2025
 * Description:    This code will receive the data being sent between each module.
 * Version: 1.1
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "dmsp_receive.pio.h"

// Define GPIO pins
#define DMSP_IN_PIN 16         // GPIO 16 as input for DMSP data
#define GLOBAL_CLK_PIN 14      // GPIO 14 as input (global clock)
#define CHANNEL_CTRL_PIN 15    // GPIO 15 as input (sync signal)

// Buffer for storing received frames
#define BUFFER_SIZE 512
#define RING_BUFFER_SIZE 1024   // Size of the ring buffer
uint32_t ring_buffer[RING_BUFFER_SIZE]; // Ring buffer
volatile uint32_t ring_buffer_head = 0; // Head index (write position)
volatile uint32_t ring_buffer_tail = 0; // Tail index (read position)

// DMA channel
int dma_channel;

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

// DMA interrupt handler
void dma_handler() {
    // Clear the interrupt
    dma_hw->ints0 = 1u << dma_channel;

    // Update the ring buffer head to reflect the new data written by the DMA
    ring_buffer_head = (ring_buffer_head + BUFFER_SIZE) % RING_BUFFER_SIZE;

    // Debug: Print the number of transfers completed by the DMA
    printf("DMA Transfers: %d\n", dma_channel_hw_addr(dma_channel)->transfer_count);

    // Restart the DMA transfer
    dma_channel_set_write_addr(dma_channel, &ring_buffer[ring_buffer_head], true);
    dma_channel_set_trans_count(dma_channel, BUFFER_SIZE, true);
}

// Function to initialize DMA
void dma_init(PIO pio, uint sm) {
    // Configure DMA channel
    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_channel);

    // Set the transfer size to 32 bits (4 bytes)
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

    // Set the read address to the PIO RX FIFO
    channel_config_set_read_increment(&c, false);  // Don't increment the read address
    channel_config_set_write_increment(&c, true);  // Increment the write address (to the ring buffer)

    // Set the DREQ to PIO RX FIFO (automatic triggering)
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    // Set the write address to the ring buffer
    dma_channel_configure(
        dma_channel,
        &c,
        ring_buffer,           // Write address
        &pio->rxf[sm],         // Read address (PIO RX FIFO)
        BUFFER_SIZE,           // Number of transfers (size of the DMA buffer)
        true                    // Start DMA immediately
    );

    // Enable DMA interrupts
    dma_channel_set_irq0_enabled(dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}

int main() {
    stdio_init_all(); // Initialize stdio for serial communication

    // Initialize the PIO and state machine
    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &dmsp_receive_program);

    // Initialize the DMSP receiver
    dmsp_receiver_init(pio, sm, offset);

    // Initialize DMA
    dma_init(pio, sm);

    printf("DMSP Receiver Initialized with DMA and Ring Buffer\n");

    while (true) {
        // Debug: Print the PIO RX FIFO fill level
        printf("PIO RX FIFO Fill Level: %d\n", pio_sm_get_rx_fifo_level(pio, sm));

        // Process data from the ring buffer
        while (ring_buffer_tail != ring_buffer_head) {
            uint32_t data = ring_buffer[ring_buffer_tail];
            ring_buffer_tail = (ring_buffer_tail + 1) % RING_BUFFER_SIZE; // Increment tail index

            // Print the received data to the serial monitor (optional for debugging)
            printf("Received Data: 0x%08X\n", data);
        }

        // Sleep for a short time to reduce CPU load, adjust this based on performance
        sleep_us(100);  // 100 microseconds, adjust based on your system's requirements
    }
}
