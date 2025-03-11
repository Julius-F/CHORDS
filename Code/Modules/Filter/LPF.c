/*
 * File: filter.c
 * Author: CHORDS Group
 * Date: March 11, 2025
 * Description: This program takes frequency data sent from the previous module,
 *              applies a low pass filter to it, and outputs filtered data. This version
 *              supports four channels of filtering. Each output frame is tagged with a header
 *              whose lower 4 bits are 'F' (frame type F) and whose upper 4 bits indicate the channel.
 * Version: 1.1
 */

 #include <stdio.h>
 #include <math.h>
 #include "pico/stdlib.h"
 #include "hardware/pio.h"
 #include "hardware/dma.h"
 #include "hardware/adc.h"
 #include "hardware/irq.h"
 #include "hardware/timer.h"
 #include "dmsp_receive.pio.h"
 #include "dmsp_transmit.pio.h"
 
 // Define GPIO pins
 #define DMSP_IN_PIN     16        
 #define GLOBAL_CLK_PIN  14     
 #define CHANNEL_CTRL_PIN 15   
 #define DMSP_OUT_PIN    17       
 #define ANALOG_IN_PIN   26      
 
 #define BUFFER_SIZE         512
 #define RING_BUFFER_SIZE    1024   
 
 // RX ring buffer for incoming frames
 uint32_t rx_ring_buffer[RING_BUFFER_SIZE];
 volatile uint32_t rx_ring_buffer_head = 0;
 volatile uint32_t rx_ring_buffer_tail = 0;
 
 // TX ring buffer for processed frames
 uint32_t tx_ring_buffer[RING_BUFFER_SIZE];
 volatile uint32_t tx_ring_buffer_head = 0;
 volatile uint32_t tx_ring_buffer_tail = 0;
 
 int rx_dma_channel;
 int tx_dma_channel;
 
 // Global variable for the sampled analog value (used to control cutoff)
 volatile uint16_t global_analog_value = 0;
 
 // Timer callback to update the analog sample
 bool analog_sampling_callback(struct repeating_timer *t) {
     global_analog_value = adc_read(); // Sample the analog value
     return true;
 }
 
 //
 // Initialize the DMSP receiver
 //
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
 
     printf("[DEBUG] DMSP Receiver SM %d Enabled\n", sm);
 }
 
 //
 // Initialize the DMSP transmitter
 //
 void dmsp_transmitter_init(PIO pio, uint sm, uint offset) {
     printf("[DEBUG] Initializing DMSP Transmitter on SM %d\n", sm);
 
     pio_gpio_init(pio, DMSP_OUT_PIN);
     pio_gpio_init(pio, GLOBAL_CLK_PIN);
     pio_gpio_init(pio, CHANNEL_CTRL_PIN);
 
     pio_sm_set_consecutive_pindirs(pio, sm, DMSP_OUT_PIN, 1, true);
     pio_sm_set_consecutive_pindirs(pio, sm, GLOBAL_CLK_PIN, 1, false);
     pio_sm_set_consecutive_pindirs(pio, sm, CHANNEL_CTRL_PIN, 1, false);
 
     pio_sm_config c = dmsp_transmit_program_get_default_config(offset);
     sm_config_set_out_pins(&c, DMSP_OUT_PIN, 1);
     sm_config_set_in_pins(&c, GLOBAL_CLK_PIN);
     sm_config_set_jmp_pin(&c, CHANNEL_CTRL_PIN);
     sm_config_set_out_shift(&c, false, false, 32);
 
     pio_sm_init(pio, sm, offset, &c);
     pio_sm_set_enabled(pio, sm, true);
 
     printf("[DEBUG] DMSP Transmitter SM %d Enabled\n", sm);
 }
 
 //
 // DMA interrupt handler for RX channel
 //
 void rx_dma_handler() {
     dma_hw->ints0 = 1u << rx_dma_channel;
     rx_ring_buffer_head = (rx_ring_buffer_head + BUFFER_SIZE) % RING_BUFFER_SIZE;
     dma_channel_set_write_addr(rx_dma_channel, &rx_ring_buffer[rx_ring_buffer_head], true);
     dma_channel_set_trans_count(rx_dma_channel, BUFFER_SIZE, true);
 }
 
 //
 // DMA interrupt handler for TX channel
 //
 void tx_dma_handler() {
     dma_hw->ints1 = 1u << tx_dma_channel;
     tx_ring_buffer_tail = (tx_ring_buffer_tail + BUFFER_SIZE) % RING_BUFFER_SIZE;
     dma_channel_set_read_addr(tx_dma_channel, &tx_ring_buffer[tx_ring_buffer_tail], true);
     dma_channel_set_trans_count(tx_dma_channel, BUFFER_SIZE, true);
 }
 
 //
 // Initialize DMA for DMSP reception
 //
 void rx_dma_init(PIO pio, uint sm) {
     rx_dma_channel = dma_claim_unused_channel(true);
     dma_channel_config c = dma_channel_get_default_config(rx_dma_channel);
     channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
     channel_config_set_read_increment(&c, false);
     channel_config_set_write_increment(&c, true);
     channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
 
     dma_channel_configure(
         rx_dma_channel, &c,
         rx_ring_buffer,
         &pio->rxf[sm],
         BUFFER_SIZE,
         true
     );
 
     dma_channel_set_irq0_enabled(rx_dma_channel, true);
     irq_set_exclusive_handler(DMA_IRQ_0, rx_dma_handler);
     irq_set_enabled(DMA_IRQ_0, true);
 }
 
 //
 // Initialize DMA for DMSP transmission
 //
 void tx_dma_init(PIO pio, uint sm) {
     tx_dma_channel = dma_claim_unused_channel(true);
     dma_channel_config c = dma_channel_get_default_config(tx_dma_channel);
     channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
     channel_config_set_read_increment(&c, true);
     channel_config_set_write_increment(&c, false);
     channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
 
     dma_channel_configure(
         tx_dma_channel, &c,
         &pio->txf[sm],
         tx_ring_buffer,
         BUFFER_SIZE,
         true
     );
 
     dma_channel_set_irq1_enabled(tx_dma_channel, true);
     irq_set_exclusive_handler(DMA_IRQ_1, tx_dma_handler);
     irq_set_enabled(DMA_IRQ_1, true);
 }
 
 //
 // Process incoming frames, apply a biquad low-pass filter per channel,
 // and push out a new frame with a header that uses frame type F (lower 4 bits)
 // and proper channel information (upper 4 bits).
 //
 void process_data() {
     // Calculate filter coefficients based on the current analog value.
     float scale_factor = ((float)global_analog_value + 10.0) / 4106.0f;
     float min_cutoff = 40.0f;      // Minimum cutoff frequency in Hz
     float max_cutoff = 20000.0f;   // Maximum cutoff frequency in Hz
     float cutoff_freq = min_cutoff + scale_factor * (max_cutoff - min_cutoff);
     float sampling_freq = 40000.0f; // Sampling frequency in Hz
     float resonance = 1.5f;         // Q factor
 
     float omega = 2.0f * M_PI * cutoff_freq / sampling_freq;
     float alpha = sin(omega) / (2.0f * resonance);
 
     float b0 = (1.0f - cos(omega)) / 2.0f;
     float b1 = 1.0f - cos(omega);
     float b2 = (1.0f - cos(omega)) / 2.0f;
     float a0 = 1.0f + alpha;
     float a1 = -2.0f * cos(omega);
     float a2 = 1.0f - alpha;
 
     // Normalize coefficients.
     b0 /= a0;
     b1 /= a0;
     b2 /= a0;
     a1 /= a0;
     a2 /= a0;
 
     // Static filter state for each of the four channels.
     static float x1[4] = {0.0f, 0.0f, 0.0f, 0.0f};
     static float x2[4] = {0.0f, 0.0f, 0.0f, 0.0f};
     static float y1[4] = {0.0f, 0.0f, 0.0f, 0.0f};
     static float y2[4] = {0.0f, 0.0f, 0.0f, 0.0f};
 
     // Process each frame in the RX ring buffer.
     while (rx_ring_buffer_tail != rx_ring_buffer_head) {
         // Read next frame.
         uint32_t frame = rx_ring_buffer[rx_ring_buffer_tail];
         rx_ring_buffer_tail = (rx_ring_buffer_tail + 1) % RING_BUFFER_SIZE;
 
         // Extract header (upper 8 bits) and raw 24-bit audio data.
         uint8_t header = (frame >> 24) & 0xFF;
         uint32_t raw_data = frame & 0xFFFFFF;
 
         // Sign-extend the 24-bit audio data to 32 bits.
         int32_t audio_data = raw_data;
         if (raw_data & 0x800000) {
             audio_data |= 0xFF000000;
         }
 
         // Determine which channel this frame belongs to.
         // (Assuming incoming frames use the oscillator header mapping)
         int channel = -1;
         switch (header) {
             case 0x8F: channel = 0; break; // Channel 1
             case 0x9F: channel = 1; break; // Channel 2
             case 0xAF: channel = 2; break; // Channel 3
             case 0xBF: channel = 3; break; // Channel 4
             default:
                 // If header unrecognized, skip this frame.
                 continue;
         }
 
         // Convert the sample to float.
         float input = (float)audio_data;
 
         // Apply the biquad filter for the proper channel.
         float output = b0 * input + b1 * x1[channel] + b2 * x2[channel]
                        - a1 * y1[channel] - a2 * y2[channel];
 
         // Update the channel’s filter state.
         x2[channel] = x1[channel];
         x1[channel] = input;
         y2[channel] = y1[channel];
         y1[channel] = output;
 
         // Convert the filtered output back to an integer.
         int32_t filtered_data = (int32_t)output;
         // Ensure the result fits in 24 bits.
         filtered_data &= 0xFFFFFF;
 
         // Generate a new header for the filtered frame.
         // Here the lower 4 bits are F (frame type F) and the upper nibble encodes the channel.
         uint8_t out_header;
         switch (channel) {
             case 0: out_header = 0x8F; break; // Channel 1
             case 1: out_header = 0x9F; break; // Channel 2
             case 2: out_header = 0xAF; break; // Channel 3
             case 3: out_header = 0xBF; break; // Channel 4
             default: out_header = 0x8F; break;
         }
 
         // Combine the new header and the filtered audio data.
         uint32_t processed_frame = ((uint32_t)out_header << 24) | (filtered_data & 0xFFFFFF);
 
         // Write the processed frame to the TX ring buffer if there is room.
         if (((tx_ring_buffer_head + 1) % RING_BUFFER_SIZE) != tx_ring_buffer_tail) {
             tx_ring_buffer[tx_ring_buffer_head] = processed_frame;
             tx_ring_buffer_head = (tx_ring_buffer_head + 1) % RING_BUFFER_SIZE;
         }
     }
 }
 
 int main() {
     sleep_ms(1000);  // Delay to allow serial monitor connection
     stdio_init_all();
 
     // Initialize ADC for analog sampling (to control cutoff)
     adc_init();
     adc_gpio_init(ANALOG_IN_PIN);
     adc_select_input(0);
 
     PIO pio = pio0;
     uint rx_sm = 0, tx_sm = 1;
     uint rx_offset = pio_add_program(pio, &dmsp_receive_program);
     uint tx_offset = pio_add_program(pio, &dmsp_transmit_program);
 
     printf("[INFO] Initializing DMSP Receiver...\n");
     dmsp_receiver_init(pio, rx_sm, rx_offset);
 
     printf("[INFO] Initializing DMSP Transmitter...\n");
     dmsp_transmitter_init(pio, tx_sm, tx_offset);
 
     printf("[INFO] Initializing DMA for RX and TX...\n");
     rx_dma_init(pio, rx_sm);
     tx_dma_init(pio, tx_sm);
 
     // Set up a repeating timer for ADC sampling (adjust the period as needed)
     struct repeating_timer timer;
     add_repeating_timer_ms(1, analog_sampling_callback, NULL, &timer);
 
     printf("[INFO] Initialization complete. Running main loop.\n");
 
     while (true) {
         process_data();
         sleep_us(1);
     }
 
     return 0;
 }
 
