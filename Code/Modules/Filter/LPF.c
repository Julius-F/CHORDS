/*
 * File: filter.c
 * Author: CHORDS Group
 * Description: LP HP Blend Filter for CHORDS Module
 *              Knob 1: Cutoff frequency
 *              Knob 2: Resonance/Q
 *              
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
 #define DMSP_IN_PIN       11        
 #define GLOBAL_CLK_PIN    7     
 #define CHANNEL_CTRL_PIN  6   
 #define DMSP_OUT_PIN      22       
 #define ADC_SEL           13  // GPIO for demux control
 
 #define BUFFER_SIZE         1024
 #define RING_BUFFER_SIZE    2048   
 
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
    // --- Coefficient and Parameter Calculation ---

    // Get knob values (atomic read)
    uint16_t knob_cutoff = current_knob_values[1]; // Knob 1: Cutoff frequency
    uint16_t knob_q      = current_knob_values[3]; // Knob 2: Resonance/Q
    uint16_t knob_morph  = current_knob_values[5]; // Knob 5: Morph control (0: LP, 1: HP)

    // Scale knob values (0.0 to 1.0)
    float cutoff_scale = ((float)knob_cutoff) / 4096.0f;
    float q_scale      = ((float)knob_q)     / 4096.0f;
    float morph_scale  = ((float)knob_morph) / 4096.0f;

    // Parameter ranges
    const float min_cutoff = 20.0f;
    const float max_cutoff = 20000.0f;
    const float min_q      = 0.01f;
    const float max_q      = 9.0f;

    // Compute target cutoff frequency and smooth it
    float cutoff_freq_target = min_cutoff + cutoff_scale * (max_cutoff - min_cutoff);
    static float smoothed_cutoff = 20.0f;  // initialize to min_cutoff
    const float smoothing_factor = 0.01f;  // Adjust this for smoother/faster response
    smoothed_cutoff += smoothing_factor * (cutoff_freq_target - smoothed_cutoff);

    float resonance = min_q + q_scale * (max_q - min_q);
    const float sampling_freq = 40000.0f;

    // Precompute trigonometric values
    float omega = 2.0f * M_PI * smoothed_cutoff / sampling_freq;
    float cos_omega = cosf(omega);
    float sin_omega = sinf(omega);
    float alpha = sin_omega / (2.0f * resonance);
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_omega;
    float a2 = 1.0f - alpha;

    // Compute low-pass coefficients (normalized)
    float b0_lp = ((1.0f - cos_omega) / 2.0f) / a0;
    float b1_lp = (1.0f - cos_omega)       / a0;
    float b2_lp = b0_lp;  // same as b0_lp

    // Compute high-pass coefficients (normalized)
    float b0_hp = ((1.0f + cos_omega) / 2.0f) / a0;
    float b1_hp = (-(1.0f + cos_omega))     / a0;
    float b2_hp = b0_hp;  // same as b0_hp

    // Interpolate coefficients based on morph control:
    // morph_scale = 0.0 gives LP; = 1.0 gives HP.
    float t = morph_scale;  // [0,1]
    float b0 = (1.0f - t) * b0_lp + t * b0_hp;
    float b1 = (1.0f - t) * b1_lp + t * b1_hp;
    float b2 = (1.0f - t) * b2_lp + t * b2_hp;
    float a1_norm = a1 / a0;
    float a2_norm = a2 / a0;

    // Define clipping limits for 24-bit data
    const float MAX_VAL = 8388607.0f;
    const float MIN_VAL = -8388608.0f;

    // --- Lookup Tables for Fast Header Mapping ---
    // Maps valid header values to channel indices; invalid entries are -1.
    static const int8_t header_to_channel[256] = {
        [0x8F] = 0,
        [0x9F] = 1,
        [0xAF] = 2,
        [0xBF] = 3,
    };
    // Mapping from channel index back to header for output
    static const uint8_t channel_to_header[4] = { 0x8F, 0x9F, 0xAF, 0xBF };

    // --- Filter State (per channel) ---
    // Keep these static so the state persists between calls.
    static float x1[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    static float x2[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    static float y1[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    static float y2[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // --- Process Ring Buffer Frames ---
    // Local copy of rx_ring_buffer_head to reduce repeated volatile access
    uint32_t local_rx_head = rx_ring_buffer_head;
    while (rx_ring_buffer_tail != local_rx_head) {
        uint32_t frame = rx_ring_buffer[rx_ring_buffer_tail];
        rx_ring_buffer_tail = (rx_ring_buffer_tail + 1) % RING_BUFFER_SIZE;

        uint8_t header = frame >> 24;
        int channel = header_to_channel[header];
        if (channel < 0)
            continue;  // Skip invalid header

        uint32_t raw_data = frame & 0xFFFFFF;
        // Sign-extend 24-bit to 32-bit
        int32_t audio_data = raw_data;
        if (raw_data & 0x800000)
            audio_data |= 0xFF000000;

        // Filter the sample
        float input = (float)audio_data;
        float output = b0 * input + b1 * x1[channel] + b2 * x2[channel]
                     - a1_norm * y1[channel] - a2_norm * y2[channel];

        // Clip output (using branchless-style min/max functions or inline if preferred)
        if (output > MAX_VAL)
            output = MAX_VAL;
        else if (output < MIN_VAL)
            output = MIN_VAL;

        // Update filter state for the channel
        x2[channel] = x1[channel];
        x1[channel] = input;
        y2[channel] = y1[channel];
        y1[channel] = output;

        int32_t filtered_data = (int32_t)output;  // Already clipped
        uint8_t out_header = channel_to_header[channel];
        uint32_t processed_frame = ((uint32_t)out_header << 24) | ((uint32_t)filtered_data & 0xFFFFFF);

        // Queue the processed frame for output if there's space
        if (((tx_ring_buffer_head + 1) % RING_BUFFER_SIZE) != tx_ring_buffer_tail) {
            tx_ring_buffer[tx_ring_buffer_head] = processed_frame;
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
     uint rx_offset = pio_add_program(pio, &dmsp_receive_program);
     uint tx_offset = pio_add_program(pio, &dmsp_transmit_program);
 
     dmsp_receiver_init(pio, rx_sm, rx_offset);
     dmsp_transmitter_init(pio, tx_sm, tx_offset);
     rx_dma_init(pio, rx_sm);
     tx_dma_init(pio, tx_sm);
 
     // Start knob reading timer (100ms update rate)
     struct repeating_timer knob_timer;
     add_repeating_timer_ms(10, knob_reading_callback, NULL, &knob_timer);
 
     printf("System initialized. Starting main loop.\n");
 
     while (true) {
         process_data();
         //sleep_us(1);
     }
 
     return 0;
 }
