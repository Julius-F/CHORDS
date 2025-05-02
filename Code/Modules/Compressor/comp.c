/*
 * File: comp.c
 * Author: Bjorn Lavik 
 * Description: DMSP comp with 5-knob support
 *              Knob 1: Threshold
 *              Knob 2: Attack
 *              Knob 3: Release
 *              Knob 4: Ratio
 *              Knob 5: Make Up
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
 
 // Define GPIO pins (updated for new hardware)
 #define DMSP_IN_PIN       3        
 #define GLOBAL_CLK_PIN    7     
 #define CHANNEL_CTRL_PIN  6   
 #define DMSP_OUT_PIN      22       
 #define ADC_SEL           13  // GPIO for demux control
 
 #define BUFFER_SIZE         256
 #define RING_BUFFER_SIZE    1024


 // Lookup tables shared across functions
const int8_t header_to_channel[256] = {
    [0x8F] = 0,
    [0x9F] = 1,
    [0xAF] = 2,
    [0xBF] = 3,
};

const uint8_t channel_to_header[4] = { 0x8F, 0x9F, 0xAF, 0xBF };

 
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

 
 // Global knob values (updated by knob_reading_callback)
 volatile uint16_t current_knob_values[6] = {0};
 
 // --- Knob Reading Functions ---
 
 void read_all_knobs(uint16_t knob_values[6]) {
     // Read first set of knobs (channels 0-2 with SEL=0)
     gpio_put(ADC_SEL, 0);
     sleep_us(10);  // Allow demux to settle
 
     adc_select_input(0);
     knob_values[0] = adc_read();
     adc_select_input(1);
     knob_values[1] = adc_read();
     adc_select_input(2);
     knob_values[2] = adc_read();
 
     // Read second set of knobs (channels 0-2 with SEL=1)
     gpio_put(ADC_SEL, 1);
     sleep_us(10);
 
     adc_select_input(0);
     knob_values[3] = adc_read();
     adc_select_input(1);
     knob_values[4] = adc_read();
     adc_select_input(2);
     knob_values[5] = adc_read();
 }
 
 bool knob_reading_callback(struct repeating_timer *t) {
     uint16_t raw_knobs[6];
     read_all_knobs(raw_knobs);
 
     // Reorder knobs to match physical layout if needed
     uint16_t ordered_knobs[6];
     ordered_knobs[0] = raw_knobs[5];  // Knob 1 (Cutoff)
     ordered_knobs[1] = raw_knobs[4];  // Knob 2 (Resonance/Q)
     ordered_knobs[2] = raw_knobs[1];  // Knob 3
     ordered_knobs[3] = raw_knobs[2];  // Knob 4
     ordered_knobs[4] = raw_knobs[3];  // Knob 5
     ordered_knobs[5] = raw_knobs[0];  // Knob 6
 
     for (int i = 0; i < 6; i++) {
         current_knob_values[i] = ordered_knobs[i];
     }
 
     return true;
 }

 // --- DMSP Initialization (from working code) ---
 
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
 
 void dmsp_transmitter_init(PIO pio, uint sm, uint offset) {
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
 }
 
 // --- DMA Functions (from working code) ---
 
 void rx_dma_handler() {
     dma_hw->ints0 = 1u << rx_dma_channel;
     rx_ring_buffer_head = (rx_ring_buffer_head + BUFFER_SIZE) % RING_BUFFER_SIZE;
     dma_channel_set_write_addr(rx_dma_channel, &rx_ring_buffer[rx_ring_buffer_head], true);
     dma_channel_set_trans_count(rx_dma_channel, BUFFER_SIZE, true);
 }
 
 void tx_dma_handler() {
     dma_hw->ints1 = 1u << tx_dma_channel;
     tx_ring_buffer_tail = (tx_ring_buffer_tail + BUFFER_SIZE) % RING_BUFFER_SIZE;
     dma_channel_set_read_addr(tx_dma_channel, &tx_ring_buffer[tx_ring_buffer_tail], true);
     dma_channel_set_trans_count(tx_dma_channel, BUFFER_SIZE, true);
 }

 
 
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

 
 // --- Filter Processing (updated for knob control) ---
 
 void process_data() {
    // Knob mappings and value normalization
    float knob_threshold = ((float)current_knob_values[0]) / 4096.0f; // Knob 1
    float knob_ratio     = ((float)current_knob_values[1]) / 4096.0f; // Knob 2
    float knob_attack    = ((float)current_knob_values[2]) / 4096.0f; // Knob 3
    float knob_release   = ((float)current_knob_values[3]) / 4096.0f; // Knob 4
    float knob_makeup    = ((float)current_knob_values[4]) / 4096.0f; // Knob 5

    // Scaled parameters
    float threshold_db   = -40.0f + knob_threshold * 40.0f;     // -40 to 0 dB
    float ratio          = 1.0f + knob_ratio * 9.0f;            // 1:1 to 10:1
    float attack_ms      = 1.0f + knob_attack * 49.0f;          // 1–50 ms
    float release_ms     = 10.0f + knob_release * 190.0f;       // 10–200 ms
    float makeup_gain    = powf(10.0f, (knob_makeup * 20.0f) / 20.0f);  // Linear gain

    const float sample_rate = 40000.0f;
    float attack_coeff  = expf(-1.0f / (attack_ms * sample_rate / 1000.0f));
    float release_coeff = expf(-1.0f / (release_ms * sample_rate / 1000.0f));
    const float MAX_VAL = 8388607.0f;
    const float MIN_VAL = -8388608.0f;

    static float envelope[4] = {0};

    uint32_t local_rx_head = rx_ring_buffer_head;
    while (rx_ring_buffer_tail != local_rx_head) {
        uint32_t frame = rx_ring_buffer[rx_ring_buffer_tail];
        rx_ring_buffer_tail = (rx_ring_buffer_tail + 1) % RING_BUFFER_SIZE;

        uint8_t header = frame >> 24;
        int channel = header_to_channel[header];
        if (channel < 0) continue;

        uint32_t raw = frame & 0xFFFFFF;
        int32_t input = raw;
        if (raw & 0x800000) input |= 0xFF000000;

        float in_sample = (float)input;

        // Calculate level
        float level = fabsf(in_sample);
        if (level > envelope[channel]) {
            envelope[channel] = attack_coeff * envelope[channel] + (1.0f - attack_coeff) * level;
        } else {
            envelope[channel] = release_coeff * envelope[channel] + (1.0f - release_coeff) * level;
        }

        // Convert envelope to dB
        float db_level = 20.0f * log10f(envelope[channel] / MAX_VAL + 1e-12f);

        // Gain computation
        float gain = 1.0f;
        if (db_level > threshold_db) {
            float over_db = db_level - threshold_db;
            float compressed_db = over_db / ratio;
            float desired_gain_db = threshold_db + compressed_db - db_level;
            gain = powf(10.0f, desired_gain_db / 20.0f);
        }

        float out_sample = in_sample * gain * makeup_gain;

        // Clamp to 24-bit signed int
        if (out_sample > MAX_VAL) out_sample = MAX_VAL;
        else if (out_sample < MIN_VAL) out_sample = MIN_VAL;

        uint32_t out_frame = ((uint32_t)channel_to_header[channel] << 24) | ((int32_t)out_sample & 0xFFFFFF);
        if (((tx_ring_buffer_head + 1) % RING_BUFFER_SIZE) != tx_ring_buffer_tail) {
            tx_ring_buffer[tx_ring_buffer_head] = out_frame;
            tx_ring_buffer_head = (tx_ring_buffer_head + 1) % RING_BUFFER_SIZE;
        }
    }
}




int main() {
    sleep_ms(1000);  // Allow time for serial connection
    stdio_init_all();

    // Initialize ADC for knob reading
    adc_init();
    adc_gpio_init(26);  // ADC0
    adc_gpio_init(27);  // ADC1
    adc_gpio_init(28);  // ADC2
    adc_select_input(0);

    // Initialize demux control pin
    gpio_init(ADC_SEL);
    gpio_set_dir(ADC_SEL, GPIO_OUT);

    // Initialize PIO and DMA
    PIO pio = pio0;
    uint rx_sm = 0, tx_sm = 1;
    
    // Load programs and initialize state machines
    uint rx_offset = pio_add_program(pio, &dmsp_receive_program);
    uint tx_offset = pio_add_program(pio, &dmsp_transmit_program);
    
    dmsp_receiver_init(pio, rx_sm, rx_offset);
    dmsp_transmitter_init(pio, tx_sm, tx_offset);
    

    // Initialize main audio DMAs
    rx_dma_init(pio, rx_sm);
    tx_dma_init(pio, tx_sm);

    // Start knob reading timer (15ms update rate)
    struct repeating_timer knob_timer;
    add_repeating_timer_ms(15, knob_reading_callback, NULL, &knob_timer);

    printf("System initialized. Starting main loop.\n");

    while (true) {
        process_data();
        tight_loop_contents();
    }
    
    return 0;
}
