#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "dmsp_receive.pio.h"
#include "dmsp_transmit.pio.h"
#include <math.h>

// --------------------- Pin and Constant Definitions ---------------------
#define DMSP_IN_PIN       10        // Receiver input
#define GLOBAL_CLK_PIN    7     
#define CHANNEL_CTRL_PIN  6   
#define DMSP_OUT_PIN      20        // Transmitter output
#define ADC_SEL           13        // Toggle for analog demux
#define WAVEFORM_BUTTON   15        // Button for waveform switching

#define BUFFER_SIZE       2048
#define RING_BUFFER_SIZE  4096
#define WAVETABLE_SIZE    1024

// --------------------- Waveform Types ---------------------
typedef enum {
    WAVEFORM_SINE,
    WAVEFORM_TRIANGLE,
    WAVEFORM_SQUARE,
    WAVEFORM_SAWTOOTH,
    WAVEFORM_REVERSE_SAWTOOTH,
    WAVEFORM_PULSE,
    WAVEFORM_HALF_RECTIFIED,
    WAVEFORM_FULL_RECTIFIED,
    WAVEFORM_COUNT
} waveform_type_t;

// --------------------- Ring Buffer Globals ---------------------
// TX ring buffer for DMSp transmission
uint32_t ring_buffer[RING_BUFFER_SIZE];
volatile uint32_t ring_buffer_head = 0;
volatile uint32_t ring_buffer_tail = 0;

// RX ring buffer for DMSp reception
uint32_t rx_ring_buffer[RING_BUFFER_SIZE];
volatile uint32_t rx_ring_buffer_head = 0;
volatile uint32_t rx_ring_buffer_tail = 0;

int rx_dma_channel;
int tx_dma_channel;



// --------------------- Wavetable ---------------------
float wavetable[WAVETABLE_SIZE];
volatile waveform_type_t current_waveform = WAVEFORM_SINE;
volatile bool waveform_changed = true;

// Button debouncing variables
volatile uint32_t last_button_time = 0;
const uint32_t button_debounce_ms = 100;

// --------------------- Waveform Generators ---------------------
void generate_sine_wave() {
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
        float phase = (2.0f * M_PI * i) / (float)WAVETABLE_SIZE;
        wavetable[i] = sinf(phase);
    }
}

void generate_triangle_wave() {
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
        float t = (float)i / (float)WAVETABLE_SIZE;
        if (t < 0.5f)
            wavetable[i] = 4.0f * t - 1.0f;  // ramp up from -1 to +1
        else
            wavetable[i] = -4.0f * t + 3.0f; // ramp down from +1 to -1
    }
}

void generate_square_wave() {
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
        float t = (float)i / (float)WAVETABLE_SIZE;
        wavetable[i] = (t < 0.5f) ? 1.0f : -1.0f;
    }
}

void generate_sawtooth_wave() {
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
        float t = (float)i / (float)WAVETABLE_SIZE;
        wavetable[i] = 2.0f * t - 1.0f;
    }
}

void generate_reverse_sawtooth_wave() {
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
        float t = (float)i / (float)WAVETABLE_SIZE;
        wavetable[i] = 1.0f - 2.0f * t;
    }
}

void generate_pulse_wave() {
    const float duty = 0.25f;
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
        float t = (float)i / (float)WAVETABLE_SIZE;
        wavetable[i] = (t < duty) ? 1.0f : -1.0f;
    }
}

void generate_half_rectified_wave() {
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
        float phase = (2.0f * M_PI * i) / (float)WAVETABLE_SIZE;
        float s = sinf(phase);
        wavetable[i] = (s > 0.0f) ? s : 0.0f;
    }
}

void generate_full_rectified_wave() {
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
        float phase = (2.0f * M_PI * i) / (float)WAVETABLE_SIZE;
        wavetable[i] = fabsf(sinf(phase));
    }
}

void init_wavetable() {
    switch(current_waveform) {
        case WAVEFORM_SINE:
            generate_sine_wave();
            break;
        case WAVEFORM_TRIANGLE:
            generate_triangle_wave();
            break;
        case WAVEFORM_SQUARE:
            generate_square_wave();
            break;
        case WAVEFORM_SAWTOOTH:
            generate_sawtooth_wave();
            break;
        case WAVEFORM_REVERSE_SAWTOOTH:
            generate_reverse_sawtooth_wave();
            break;
        case WAVEFORM_PULSE:
            generate_pulse_wave();
            break;
        case WAVEFORM_HALF_RECTIFIED:
            generate_half_rectified_wave();
            break;
        case WAVEFORM_FULL_RECTIFIED:
            generate_full_rectified_wave();
            break;
        default:
            generate_sine_wave();
            break;
    }
    waveform_changed = false;
}

// Button polling function
void poll_waveform_button() {
    static bool last_button_state = true; // Default to high (button not pressed)
    static uint32_t last_poll_time = 0;
    
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_poll_time < 100) return; // Only poll every 100ms
    
    last_poll_time = current_time;
    bool current_button_state = gpio_get(WAVEFORM_BUTTON);
    
    // Check for falling edge (button press)
    if (last_button_state && !current_button_state) {
        // Debounce check
        if (current_time - last_button_time > button_debounce_ms) {
            current_waveform = (current_waveform + 1) % WAVEFORM_COUNT;
            waveform_changed = true;
            printf("Waveform changed to %d\n", current_waveform);
            last_button_time = current_time;
        }
    }
    
    last_button_state = current_button_state;
}

 // Global knob values (updated by knob_reading_callback)
 volatile float current_knob_values[6] = {0.0f};
 
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
    ordered_knobs[4] = raw_knobs[3];  // Knob 5 (Amplitude)
    ordered_knobs[5] = raw_knobs[0];  // Knob 6

    for (int i = 0; i < 6; i++) {
        // Scale to 0.0-1.0 range
        current_knob_values[i] = ordered_knobs[i] / 4095.0f;
    }

    return true;
}

// --------------------- Oscillator Instance Globals ---------------------
// One oscillator per channel (channels 1-4; index 0 = channel 1, etc.)
float phases[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float phaseIncrements[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float amplitudes[4] = {0.0f};  // Initialize to 0

// These are updated via DMSp RX frames for each channel.
volatile uint8_t current_midi_note_arr[4] = {0, 0, 0, 0};
volatile uint32_t current_pitch_bend_arr[4] = {
    (((1 << 17) - 1)) / 2,
    (((1 << 17) - 1)) / 2,
    (((1 << 17) - 1)) / 2,
    (((1 << 17) - 1)) / 2
};

// --------------------- Helper Functions ---------------------
float midi_note_to_frequency(uint8_t midi_note) {
    return 440.0f * powf(2.0f, (((float)midi_note - 69.0f) / 12.0f));
}

float apply_pitch_bend(float base_frequency, uint32_t pitch_bend) {
    float center = (float)(1 << 16);
    float bend = ((float)((int32_t)pitch_bend - center)) / center;
    return base_frequency * powf(2.0f, bend * 2.0f / 12.0f);
}

// Update the oscillator state for instance 'ch'
void update_oscillator(int ch) {
    float target_frequency;
    
    if (current_midi_note_arr[ch] == 0) {
        // When MIDI note 0 is sent, smoothly ramp down to 0Hz
        target_frequency = 0.0f;
        
        // If we're close to 0Hz, just set it to 0 to avoid denormals
        if (phaseIncrements[ch] < (0.001f * WAVETABLE_SIZE / 40000.0f)) {
            phaseIncrements[ch] = 0.0f;
            phases[ch] = 0.0f;  // Reset phase to avoid clicks
            return;
        }
    } else {
        // Normal note processing
        float base_frequency = midi_note_to_frequency(current_midi_note_arr[ch]);
        target_frequency = apply_pitch_bend(base_frequency, current_pitch_bend_arr[ch]);
    }
    
    // Calculate the target phase increment with smoothing
    float target_phaseIncrement = (target_frequency * WAVETABLE_SIZE) / 40000.0f;
    
    // Use a slower smoothing when ramping down to avoid clicks
    float smoothing_factor = (target_phaseIncrement < phaseIncrements[ch]) ? 0.97f : 0.95f;
    phaseIncrements[ch] = (smoothing_factor * phaseIncrements[ch]) + 
                          ((1.0f - smoothing_factor) * target_phaseIncrement);
}
// Generate a sample from oscillator instance 'ch'
float generate_sample_instance(int ch) {
    // Handle zero-frequency case
    if (phaseIncrements[ch] <= 0.0f) {
        // For waveforms that don't naturally go to 0 at phase 0, we need to fade out
        if (current_waveform == WAVEFORM_SQUARE || 
            current_waveform == WAVEFORM_PULSE ||
            current_waveform == WAVEFORM_SAWTOOTH ||
            current_waveform == WAVEFORM_REVERSE_SAWTOOTH) {
            // For these waveforms, return 0 when not playing
            return 0.0f;
        }
        
        // For other waveforms, return their value at phase 0
        return wavetable[0] * amplitudes[ch];
    }
    
    int index = (int)phases[ch];
    float frac = phases[ch] - index;
    float sample = wavetable[index] * (1.0f - frac) +
                   wavetable[(index + 1) % WAVETABLE_SIZE] * frac;
    phases[ch] += phaseIncrements[ch];
    if (phases[ch] >= WAVETABLE_SIZE) {
        phases[ch] -= WAVETABLE_SIZE;
    }
    return sample * amplitudes[ch];
}

// --------------------- DMSP PIO Initialization ---------------------
void dmsp_receiver_init(PIO pio, uint sm, uint offset) {
    printf("[DEBUG] Initializing DMSP Receiver on SM %d\n", sm);
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

// --------------------- DMA Handlers ---------------------
void rx_dma_handler() {
    dma_hw->ints0 = 1u << rx_dma_channel;
    rx_ring_buffer_head = (rx_ring_buffer_head + BUFFER_SIZE) % RING_BUFFER_SIZE;
    dma_channel_set_write_addr(rx_dma_channel, &rx_ring_buffer[rx_ring_buffer_head], true);
    dma_channel_set_trans_count(rx_dma_channel, BUFFER_SIZE, true);
}

void tx_dma_handler() {
    dma_hw->ints1 = 1u << tx_dma_channel;
    ring_buffer_tail = (ring_buffer_tail + BUFFER_SIZE) % RING_BUFFER_SIZE;
    dma_channel_set_read_addr(tx_dma_channel, &ring_buffer[ring_buffer_tail], true);
    dma_channel_set_trans_count(tx_dma_channel, BUFFER_SIZE, true);
}

// --------------------- DMA Initialization ---------------------
void rx_dma_init(PIO pio, uint sm) {
    rx_dma_channel = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(rx_dma_channel);

    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(
        rx_dma_channel, &c,
        rx_ring_buffer,           // Destination buffer
        &pio->rxf[sm],            // Source: RX FIFO of SM 'sm'
        BUFFER_SIZE,              // Transfer count
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
        &pio->txf[sm],           // Destination: TX FIFO of SM 'sm'
        ring_buffer,             // Source: our TX ring buffer
        BUFFER_SIZE,             // Transfer count
        true
    );
    dma_channel_set_irq1_enabled(tx_dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_1, tx_dma_handler);
    irq_set_enabled(DMA_IRQ_1, true);
}

// --------------------- DMSP RX Processing ---------------------
// Process received DMSP frames from the RX ring buffer. We expect the header to
// indicate the channel (0x8F for channel 1, 0x9F for channel 2, 0xAF for channel 3, 0xBF for channel 4).
void process_rx_buffer() {
    while (rx_ring_buffer_tail != rx_ring_buffer_head) {
        uint32_t frame = rx_ring_buffer[rx_ring_buffer_tail];
        rx_ring_buffer_tail = (rx_ring_buffer_tail + 1) % RING_BUFFER_SIZE;
        uint8_t header = frame >> 24;
        uint32_t data = frame & 0xFFFFFF;
        uint8_t note = (data >> 17) & 0x7F;
        uint32_t pitch = data & 0x1FFFF;
        int ch;
        if (header == 0x83) ch = 0;
        else if (header == 0x93) ch = 1;
        else if (header == 0xA3) ch = 2;
        else if (header == 0xB3) ch = 3;
        else continue;
        current_midi_note_arr[ch] = note;
        current_pitch_bend_arr[ch] = pitch;
        // Optional debug:
        // printf("[RX] Channel %d: Note: %d, Pitch Bend: %u\n", ch+1, note, pitch);
    }
}

// --------------------- TX Buffer Filling ---------------------
// Fill the TX ring buffer with oscillator sample data for 4 channels.
// The header cycles through channels 1-4; for each channel the corresponding
// oscillator instance is updated and its sample is generated.
void fill_ring_buffer() {
    // Loop until the TX ring buffer is full.
    while (((ring_buffer_head + 1) % RING_BUFFER_SIZE) != ring_buffer_tail) {
        // Determine the current channel based on the ring_buffer head index.
        // Channels: 1->index 0, 2->index 1, etc.
        int ch = (ring_buffer_head % 4);
        // Update oscillator instance for this channel.
        update_oscillator(ch);
        // Generate sample for this oscillator.
        float sample = generate_sample_instance(ch);
        // Determine header based on channel.
        uint8_t header;
        switch (ch + 1) {
            case 1:
                header = 0x8F;  // Channel 1: 1 (flag) + 000 + 1111
                break;
            case 2:
                header = 0x9F;  // Channel 2: 1 (flag) + 001 + 1111
                break;
            case 3:
                header = 0xAF;  // Channel 3: 1 (flag) + 010 + 1111
                break;
            case 4:
                header = 0xBF;  // Channel 4: 1 (flag) + 011 + 1111
                break;
            default:
                header = 0x8F;
                break;
        }
        int32_t audio_data = (int32_t)(sample * 0x7FFFFF);
        uint32_t frame = ((uint32_t)header << 24) | (audio_data & 0xFFFFFF);
        ring_buffer[ring_buffer_head] = frame;
        ring_buffer_head = (ring_buffer_head + 1) % RING_BUFFER_SIZE;
    }
}

// --------------------- Main Function ---------------------
int main() {
    sleep_ms(1000); // Delay for serial monitor connection
    stdio_init_all();

    // Initialize waveform button with pull-up
    gpio_init(WAVEFORM_BUTTON);
    gpio_set_dir(WAVEFORM_BUTTON, GPIO_IN);
    gpio_pull_up(WAVEFORM_BUTTON);
    
    // Initialize ADC for knob reading
    adc_init();
    adc_gpio_init(26);  // ADC0
    adc_gpio_init(27);  // ADC1
    adc_gpio_init(28);  // ADC2
    adc_select_input(0);

    // Initialize demux control pin
    gpio_init(ADC_SEL);
    gpio_set_dir(ADC_SEL, GPIO_OUT);

    init_wavetable();

    PIO pio = pio0;
    // Use SM0 for RX and SM1 for TX
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


    // Start knob reading timer (100ms update rate)
    struct repeating_timer knob_timer;
    add_repeating_timer_ms(10, knob_reading_callback, NULL, &knob_timer);

    printf("[INFO] Initialization complete. Running main loop.\n");
    while (true) {
        poll_waveform_button();
        if (waveform_changed) {
            init_wavetable();
        }
        
        // Update amplitudes from knobs
        for (int ch = 0; ch < 4; ch++) {
            amplitudes[ch] = current_knob_values[4];  // Now properly scaled 0.0-1.0
        }
        
        process_rx_buffer();
        fill_ring_buffer();
    }

    return 0;
}
