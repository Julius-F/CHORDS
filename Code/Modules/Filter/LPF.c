/*
 * File: LPF.c
 * Author: Bjorn Lavik 
 * Description: DMSP filter with 6-knob support (using demux)
 *              Knob 2: Cutoff frequency
 *              Knob 4: Resonance/Q
 *              Knob 6: LP/BP/HP
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
 #define CUTOFF_IN_PIN     10
 
 #define BUFFER_SIZE         512 
 #define RING_BUFFER_SIZE    1024

 #define CUTOFF_INVERT_BUTTON    14      // Button for to toggle "Cutoff In" to inverting/non-inverting mode

 bool cutoff_invert_mode = false;  // Default is non-inverting mode

 bool last_cutoff_invert_button_state = true;
 uint32_t last_cutoff_invert_button_time = 0;
 const uint32_t button_debounce_ms = 100;

 float cutoff_mod[4] = {0};  // modulation input values per channel


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

 // RX ring buffer for incoming Cutoff Frequency Frames
 uint32_t cutoff_rx_ring_buffer[RING_BUFFER_SIZE];
 volatile uint32_t cutoff_rx_ring_buffer_head = 0;
 volatile uint32_t cutoff_rx_ring_buffer_tail = 0;
 
 int rx_dma_channel;
 int tx_dma_channel;
 int cutoff_dma_channel;
 
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

 void poll_buttons() {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Cutoff Invert Button (toggles between inverting/non-inverting mode)
    bool invert_button_state = gpio_get(CUTOFF_INVERT_BUTTON);
    if (!invert_button_state && last_cutoff_invert_button_state &&
        (current_time - last_cutoff_invert_button_time > button_debounce_ms)) {
        last_cutoff_invert_button_time = current_time;
        cutoff_invert_mode = !cutoff_invert_mode;  // Toggle mode
        printf("Cutoff Invert Mode: %s\n", cutoff_invert_mode ? "Inverting" : "Non-Inverting");
    }
    last_cutoff_invert_button_state = invert_button_state;
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
 
 void cutoff_receiver_init(PIO pio, uint sm, uint offset) {
    pio_gpio_init(pio, CUTOFF_IN_PIN);
    pio_gpio_init(pio, GLOBAL_CLK_PIN);
    pio_gpio_init(pio, CHANNEL_CTRL_PIN);

    pio_sm_set_consecutive_pindirs(pio, sm, CUTOFF_IN_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, GLOBAL_CLK_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, CHANNEL_CTRL_PIN, 1, false);

    pio_sm_config c = dmsp_receive_program_get_default_config(offset);
    sm_config_set_in_pins(&c, CUTOFF_IN_PIN);
    sm_config_set_jmp_pin(&c, CHANNEL_CTRL_PIN);
    sm_config_set_in_shift(&c, false, false, 32);

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

 void cutoff_dma_handler() {
    dma_hw->ints2 = 1u << cutoff_dma_channel;
    cutoff_rx_ring_buffer_head = (cutoff_rx_ring_buffer_head + BUFFER_SIZE) % RING_BUFFER_SIZE;
    dma_channel_set_write_addr(cutoff_dma_channel, &cutoff_rx_ring_buffer[cutoff_rx_ring_buffer_head], true);
    dma_channel_set_trans_count(cutoff_dma_channel, BUFFER_SIZE, true);
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

 void cutoff_dma_init(PIO pio, uint sm) {
    cutoff_dma_channel = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(cutoff_dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(
        cutoff_dma_channel, &c,
        cutoff_rx_ring_buffer,
        &pio->rxf[sm],
        BUFFER_SIZE,
        true
    );

    dma_irqn_set_channel_enabled(2, cutoff_dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_2, cutoff_dma_handler);
    irq_set_enabled(DMA_IRQ_2, true);
}

 
 // --- Filter Processing (updated for knob control) ---
 
 void process_data() {
    // Global knob reads
    float knob_cutoff = ((float)current_knob_values[1]) / 4096.0f;
    float knob_q      = ((float)current_knob_values[3]) / 4096.0f;
    float morph_scale = ((float)current_knob_values[5]) / 4096.0f;

    // Parameter ranges
    const float min_cutoff = 15.0f;
    const float max_cutoff = 20000.0f;
    const float min_q      = 0.01f;
    const float max_q      = 15.0f;
    const float sampling_freq = 40000.0f;
    const float smoothing_factor = 0.01f;

    const float MAX_VAL = 8388607.0f;
    const float MIN_VAL = -8388608.0f;

    static float smoothed_cutoff[4] = {200.0f, 200.0f, 200.0f, 200.0f};
    static float b0[4], b1[4], b2[4], a1_norm[4], a2_norm[4];
    static float x1[4] = {0}, x2[4] = {0}, y1[4] = {0}, y2[4] = {0};
    extern float cutoff_mod[4];  // must be defined globally

    float resonance = min_q + knob_q * (max_q - min_q);

    // Recompute filter coefficients per channel
    for (int ch = 0; ch < 4; ch++) {
        float mod = cutoff_mod[ch];  // expected to be 0.0 to 1.0
        float combined = knob_cutoff + mod;
        if (combined > 1.0f) combined = 1.0f;

        float target_cutoff = min_cutoff + combined * (max_cutoff - min_cutoff);
        smoothed_cutoff[ch] += smoothing_factor * (target_cutoff - smoothed_cutoff[ch]);

        float omega = 2.0f * M_PI * smoothed_cutoff[ch] / sampling_freq;
        float cos_omega = cosf(omega);
        float sin_omega = sinf(omega);
        float alpha = sin_omega / (2.0f * resonance);

        float a0 = 1.0f + alpha;
        float a1 = -2.0f * cos_omega;
        float a2 = 1.0f - alpha;

        float b0_lp = ((1.0f - cos_omega) / 2.0f) / a0;
        float b1_lp = (1.0f - cos_omega) / a0;
        float b2_lp = b0_lp;

        float b0_hp = ((1.0f + cos_omega) / 2.0f) / a0;
        float b1_hp = (-(1.0f + cos_omega)) / a0;
        float b2_hp = b0_hp;

        float t = morph_scale;
        b0[ch] = (1.0f - t) * b0_lp + t * b0_hp;
        b1[ch] = (1.0f - t) * b1_lp + t * b1_hp;
        b2[ch] = (1.0f - t) * b2_lp + t * b2_hp;
        a1_norm[ch] = a1 / a0;
        a2_norm[ch] = a2 / a0;
    }

    // Process all incoming frames
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

        float out = b0[channel] * input + b1[channel] * x1[channel] + b2[channel] * x2[channel]
                  - a1_norm[channel] * y1[channel] - a2_norm[channel] * y2[channel];

        if (out > MAX_VAL) out = MAX_VAL;
        else if (out < MIN_VAL) out = MIN_VAL;

        x2[channel] = x1[channel];
        x1[channel] = input;
        y2[channel] = y1[channel];
        y1[channel] = out;

        uint32_t out_frame = ((uint32_t)channel_to_header[channel] << 24) | ((int32_t)out & 0xFFFFFF);
        if (((tx_ring_buffer_head + 1) % RING_BUFFER_SIZE) != tx_ring_buffer_tail) {
            tx_ring_buffer[tx_ring_buffer_head] = out_frame;
            tx_ring_buffer_head = (tx_ring_buffer_head + 1) % RING_BUFFER_SIZE;
        }
    }
}


void process_cutoff_mod() {
    uint32_t local_cutoff_head = cutoff_rx_ring_buffer_head;

    while (cutoff_rx_ring_buffer_tail != local_cutoff_head) {
        uint32_t frame = cutoff_rx_ring_buffer[cutoff_rx_ring_buffer_tail];
        cutoff_rx_ring_buffer_tail = (cutoff_rx_ring_buffer_tail + 1) % RING_BUFFER_SIZE;

        uint8_t header = frame >> 24;

        // Check format: must be 1cccF
        if ((header & 0x0F) != 0x0F) continue;

        int channel = (header >> 4) & 0x07;
        if (channel >= 4) continue;  // Only channels 0-3 supported in this module

        uint32_t raw = frame & 0xFFFFFF;

        // ✅ Treat as unsigned 24-bit (0 to 16777215) mapped to 0.0 – 1.0
        float mod = (float)raw / 16777215.0f;

        // Clamp just in case
        if (mod < 0.0f) mod = 0.0f;
        if (mod > 1.0f) mod = 1.0f;

        // Apply inversion if inverting mode
        if (cutoff_invert_mode) {
            mod = 1.0f - mod;  // Invert the modulation value
        }

        cutoff_mod[channel] = mod;

        // Optional debug
        // printf("Cutoff mod CH%d = %.3f\n", channel, mod);
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

    gpio_init(CUTOFF_INVERT_BUTTON);
    gpio_set_dir(CUTOFF_INVERT_BUTTON, GPIO_IN);
    gpio_pull_up(CUTOFF_INVERT_BUTTON);

    // Initialize demux control pin
    gpio_init(ADC_SEL);
    gpio_set_dir(ADC_SEL, GPIO_OUT);

    // Initialize PIO and DMA
    PIO pio = pio0;
    uint rx_sm = 0, tx_sm = 1, cutoff_sm = 2;
    
    // Load programs and initialize state machines
    uint rx_offset = pio_add_program(pio, &dmsp_receive_program);
    uint tx_offset = pio_add_program(pio, &dmsp_transmit_program);
    
    dmsp_receiver_init(pio, rx_sm, rx_offset);
    dmsp_transmitter_init(pio, tx_sm, tx_offset);
    
    // Initialize cutoff receiver on separate state machine
    cutoff_receiver_init(pio, cutoff_sm, rx_offset);  // Using same receive program
    cutoff_dma_init(pio, cutoff_sm);

    // Initialize main audio DMAs
    rx_dma_init(pio, rx_sm);
    tx_dma_init(pio, tx_sm);

    // Start knob reading timer (15ms update rate)
    struct repeating_timer knob_timer;
    add_repeating_timer_ms(15, knob_reading_callback, NULL, &knob_timer);

    printf("System initialized. Starting main loop.\n");

    while (true) {
        poll_buttons();     // Button state check
        process_cutoff_mod();  // must be called before process_data
        process_data();
        tight_loop_contents();
    }
    
    return 0;
}
