/*
 * File: i2s_line_out_dma.c
 * Author: CHORDS Group
 * Date: February 20, 2025
 * Description: This program takes the final signal and converts it to an audio output signal using DMA for improved performance.
 * Version: 1.2
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
 
 // DMA Buffer Definitions
 #define BUFFER_SIZE 1024
 #define RING_BUFFER_SIZE 2048   // Size of the ring buffer
 static uint32_t ring_buffer[RING_BUFFER_SIZE];
 static volatile uint32_t ring_buffer_head = 0;
 static volatile uint32_t ring_buffer_tail = 0;
 int dma_channel;
 
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
 
 // DMA interrupt handler
 void dma_handler() {
     dma_hw->ints0 = 1u << dma_channel;
     ring_buffer_head = (ring_buffer_head + BUFFER_SIZE) % RING_BUFFER_SIZE;
     dma_channel_set_write_addr(dma_channel, &ring_buffer[ring_buffer_head], true);
     dma_channel_set_trans_count(dma_channel, BUFFER_SIZE, true);
 }
 
 // Function to initialize DMA for DMSP reception
 void dma_init(PIO pio, uint sm) {
     dma_channel = dma_claim_unused_channel(true);
     dma_channel_config c = dma_channel_get_default_config(dma_channel);
     channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
     channel_config_set_read_increment(&c, false);
     channel_config_set_write_increment(&c, true);
     channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
 
     dma_channel_configure(
         dma_channel, &c,
         ring_buffer,   // Destination buffer
         &pio->rxf[sm], // Read from PIO RX FIFO
         BUFFER_SIZE,   // Number of transfers per DMA cycle
         true           // Start immediately
     );
 
     dma_channel_set_irq0_enabled(dma_channel, true);
     irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
     irq_set_enabled(DMA_IRQ_0, true);
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
     float div = 150000000 / (160.17e3 * 399);
     sm_config_set_clkdiv(&c, div);
     pio_sm_init(pio, sm, offset, &c);
     pio_sm_set_enabled(pio, sm, true);
 }
 
 // Initialize the ws clock
 void ws_clock_init(PIO pio, uint sm, uint offset) {
     pio_gpio_init(pio, WS_PIN);
     pio_sm_set_consecutive_pindirs(pio, sm, WS_PIN, 1, true);
     pio_sm_config c = ws_program_get_default_config(offset);
     sm_config_set_set_pins(&c, WS_PIN, 1);
     pio_sm_exec(pio, sm, pio_encode_mov(pio_x, 0));
     pio_sm_init(pio, sm, offset, &c);
     pio_sm_set_enabled(pio, sm, true);
 }
 
 int main() {
    stdio_init_all();

    PIO pio = pio0;
    uint dmsp_sm = 0;
    uint i2s_sm = 1;
    uint ws_sm = 2;
    uint dmsp_offset = pio_add_program(pio, &dmsp_receive_program);
    uint i2s_offset = pio_add_program(pio, &i2s_program);
    uint ws_offset = pio_add_program(pio, &ws_program);

    dmsp_receiver_init(pio, dmsp_sm, dmsp_offset);
    dma_init(pio, dmsp_sm);
    i2s_transmitter_init(pio, i2s_sm, i2s_offset);
    ws_clock_init(pio, ws_sm, ws_offset);

    // Variables for accumulating frame data over 4 channels.
    uint32_t combined_data = 0;
    uint8_t frame_count = 0;  // Keeps track of the number of frames in the current TDM cycle

    while (true) {
        while (ring_buffer_tail != ring_buffer_head) {
            // Retrieve next frame from the DMA ring buffer.
            uint32_t frame = ring_buffer[ring_buffer_tail];
            ring_buffer_tail = (ring_buffer_tail + 1) % RING_BUFFER_SIZE;

            // The frame is expected to have:
            // [ 8-bit header | 24-bit frame data ]
            uint8_t header = (frame >> 24) & 0xFF;
            uint32_t data = frame & 0xFFFFFF;

            // Validate header:
            // 1. The flag bit (bit 7) must be set.
            if ((header & 0x80) == 0) {
                // If flag bit is not set, skip this frame.
                continue;
            }

            // 2. The frame type (lower 4 bits) must be 0xF.
            if ((header & 0x0F) != 0xF) {
                // Invalid frame type; skip frame.
                continue;
            }

            // 3. Extract channel info (bits 6-4). For your test:
            //    channel 1 = 0 (000), channel 2 = 1 (001), channel 3 = 2 (010), channel 4 = 3 (011)
            uint8_t channel = (header >> 4) & 0x07;

            // Check that the frame comes on the expected channel in the TDM cycle.
            // The first frame of the cycle should be channel 0, then channel 1, etc.
            if (channel != frame_count) {
                // Out-of-sequence frame.
                // For simplicity, we reset our accumulation to re-sync.
                frame_count = 0;
                combined_data = 0;
                continue;
            }

            // Accumulate the 24-bit frame data.
            combined_data += data;
            frame_count++;

            // Once we've received 4 frames (one per channel), process the data.
            if (frame_count == 4) {
                // Divide by factor; in this test, it's 1 (no scaling).
                uint32_t output_data = combined_data / 1;

                // Prepare the output frame for I2S.
                // The original transmitter shifted by 8 bits, so we do the same.
                uint32_t i2s_frame = output_data << 8;

                // Send the frame to both left and right channels.
                pio_sm_put_blocking(pio, i2s_sm, i2s_frame);
                pio_sm_put_blocking(pio, i2s_sm, i2s_frame);

                // Reset for the next set of channels.
                frame_count = 0;
                combined_data = 0;
            }
        }
        sleep_us(1);
    }
    return 0;
}
