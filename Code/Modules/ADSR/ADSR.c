/*
 * File: ADSR.c
 * Author: Bjorn Lavik 
 * Description: ADSR envelope generator with built in VCA, Buttons for setting ADSR output level
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "math.h"
#include "dmsp_receive.pio.h"
#include "dmsp_transmit.pio.h"

// --------------------- Pin and Constant Definitions ---------------------
#define GLOBAL_CLK_PIN      7     
#define CHANNEL_CTRL_PIN    6 

#define GATE_IN_PIN         10      // Gate for ADSR,       DMSP B
#define ADSR_OUT_PIN        20      // ADSR Envelope Out,   DMSP C
#define VCA_IN_PIN          2       // VCA In for ADSR,     DMSP F
#define VCA_OUT_PIN         12      // VCA Out for ADSR,    DMSP G

#define ADC_SEL             13      // Toggle for analog demux

#define BUFFER_SIZE         512
#define RING_BUFFER_SIZE    1024

#define AMPLITUDE_BUTTON    14      // Button for cycling amplitude of ADSR_OUT, Button 1
#define TIME_BUTTON         15      // Button for triggering ADSR

#define KNOB_ATTACK         0       // Knob 1 (index 0) - Attack Time
#define KNOB_DECAY          1       // Knob 2 (index 1) - Decay Time
#define KNOB_SUSTAIN        4       // Knob 5 (index 4) - Sustain Level
#define KNOB_RELEASE        5       // Knob 6 (index 5) - Release Time

// --------------------- Ring Buffer Globals ---------------------
// TX ring buffers for DMSP transmission
uint32_t adsr_tx_ring_buffer[RING_BUFFER_SIZE];
volatile uint32_t adsr_tx_ring_buffer_head = 0;
volatile uint32_t adsr_tx_ring_buffer_tail = 0;

uint32_t vca_tx_ring_buffer[RING_BUFFER_SIZE];
volatile uint32_t vca_tx_ring_buffer_head = 0;
volatile uint32_t vca_tx_ring_buffer_tail = 0;

// RX ring buffers for DMSP reception
uint32_t gate_rx_ring_buffer[RING_BUFFER_SIZE];
volatile uint32_t gate_rx_ring_buffer_head = 0;
volatile uint32_t gate_rx_ring_buffer_tail = 0;

uint32_t vca_rx_ring_buffer[RING_BUFFER_SIZE];
volatile uint32_t vca_rx_ring_buffer_head = 0;
volatile uint32_t vca_rx_ring_buffer_tail = 0;

int adsr_tx_dma_channel;
int vca_tx_dma_channel;
int gate_rx_dma_channel;
int vca_rx_dma_channel;

// Amplitude button variables
volatile int amplitude_divisors[] = {1, 2, 6, 16, 32};
volatile int current_amp_index = 0;
bool last_amplitude_button_state = true;
uint32_t last_amplitude_button_time = 0;
const uint32_t button_debounce_ms = 100;

// ADSR state arrays per channel
bool gate_inputs[4] = {false, false, false, false}; // Gate DMSP Inputs (e.g. jacks)
bool gate_states[4] = {false, false, false, false}; // Final effective gate states per channel

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

// Update your poll_buttons() function as follows:
void poll_buttons() {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Amplitude mode button (cycles through divisors)
    bool amp_button_state = gpio_get(AMPLITUDE_BUTTON);
    if (!amp_button_state && last_amplitude_button_state && 
        (current_time - last_amplitude_button_time > button_debounce_ms)) {
        last_amplitude_button_time = current_time;
        current_amp_index = (current_amp_index + 1) % 5; // Cycle through 0-4 (divisors 1,2,4,8,16)
        printf("Amplitude divisor: %d\n", amplitude_divisors[current_amp_index]);
    }
    last_amplitude_button_state = amp_button_state;
}

// --------------------- DMSP PIO Initialization ---------------------
void dmsp_receiver_init(PIO pio, uint sm, uint offset, uint dmsp_in_pin) {
    printf("[DEBUG] Initializing DMSP Receiver on SM %d\n", sm);
    pio_gpio_init(pio, dmsp_in_pin);
    pio_gpio_init(pio, GLOBAL_CLK_PIN);
    pio_gpio_init(pio, CHANNEL_CTRL_PIN);

    pio_sm_set_consecutive_pindirs(pio, sm, dmsp_in_pin, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, GLOBAL_CLK_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, CHANNEL_CTRL_PIN, 1, false);

    pio_sm_config c = dmsp_receive_program_get_default_config(offset);
    sm_config_set_in_pins(&c, dmsp_in_pin);
    sm_config_set_jmp_pin(&c, CHANNEL_CTRL_PIN);
    sm_config_set_in_shift(&c, false, false, 32);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
    printf("[DEBUG] DMSP Receiver SM %d Enabled\n", sm);
}

void dmsp_transmitter_init(PIO pio, uint sm, uint offset, uint dmsp_out_pin) {
    printf("[DEBUG] Initializing DMSP Transmitter on SM %d\n", sm);
    pio_gpio_init(pio, dmsp_out_pin);
    pio_gpio_init(pio, GLOBAL_CLK_PIN);
    pio_gpio_init(pio, CHANNEL_CTRL_PIN);

    pio_sm_set_consecutive_pindirs(pio, sm, dmsp_out_pin, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, GLOBAL_CLK_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, CHANNEL_CTRL_PIN, 1, false);

    pio_sm_config c = dmsp_transmit_program_get_default_config(offset);
    sm_config_set_out_pins(&c, dmsp_out_pin, 1);
    sm_config_set_in_pins(&c, GLOBAL_CLK_PIN);
    sm_config_set_jmp_pin(&c, CHANNEL_CTRL_PIN);
    sm_config_set_out_shift(&c, false, false, 32);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
    printf("[DEBUG] DMSP Transmitter SM %d Enabled\n", sm);
}

// --- DMA IRQ Handlers ---

void adsr_tx_dma_handler() {
    dma_hw->ints0 = 1u << adsr_tx_dma_channel;
    adsr_tx_ring_buffer_tail = (adsr_tx_ring_buffer_tail + BUFFER_SIZE) % RING_BUFFER_SIZE;
    dma_channel_set_read_addr(adsr_tx_dma_channel, &adsr_tx_ring_buffer[adsr_tx_ring_buffer_tail], true);
    dma_channel_set_trans_count(adsr_tx_dma_channel, BUFFER_SIZE, true);
}

void vca_tx_dma_handler() {
    dma_hw->ints1 = 1u << vca_tx_dma_channel;
    vca_tx_ring_buffer_tail = (vca_tx_ring_buffer_tail + BUFFER_SIZE) % RING_BUFFER_SIZE;
    dma_channel_set_read_addr(vca_tx_dma_channel, &vca_tx_ring_buffer[vca_tx_ring_buffer_tail], true);
    dma_channel_set_trans_count(vca_tx_dma_channel, BUFFER_SIZE, true);
}

void gate_rx_dma_handler() {
    dma_hw->ints2 = 1u << gate_rx_dma_channel;
    gate_rx_ring_buffer_head = (gate_rx_ring_buffer_head + BUFFER_SIZE) % RING_BUFFER_SIZE;
    dma_channel_set_write_addr(gate_rx_dma_channel, &gate_rx_ring_buffer[gate_rx_ring_buffer_head], true);
    dma_channel_set_trans_count(gate_rx_dma_channel, BUFFER_SIZE, true);
}

void vca_rx_dma_handler() {
    dma_hw->ints3 = 1u << vca_rx_dma_channel;
    vca_rx_ring_buffer_head = (vca_rx_ring_buffer_head + BUFFER_SIZE) % RING_BUFFER_SIZE;
    dma_channel_set_write_addr(vca_rx_dma_channel, &vca_rx_ring_buffer[vca_rx_ring_buffer_head], true);
    dma_channel_set_trans_count(vca_rx_dma_channel, BUFFER_SIZE, true);
}

// --- DMA Initialization Functions ---

void adsr_tx_dma_init(PIO pio, uint sm) {
    adsr_tx_dma_channel = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(adsr_tx_dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));

    dma_channel_configure(
        adsr_tx_dma_channel, &c,
        &pio->txf[sm],
        adsr_tx_ring_buffer,
        BUFFER_SIZE,
        true
    );

    dma_channel_set_irq0_enabled(adsr_tx_dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, adsr_tx_dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}

void vca_tx_dma_init(PIO pio, uint sm) {
    vca_tx_dma_channel = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(vca_tx_dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));

    dma_channel_configure(
        vca_tx_dma_channel, &c,
        &pio->txf[sm],
        vca_tx_ring_buffer,
        BUFFER_SIZE,
        true
    );

    dma_channel_set_irq1_enabled(vca_tx_dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_1, vca_tx_dma_handler);
    irq_set_enabled(DMA_IRQ_1, true);
}

void gate_rx_dma_init(PIO pio, uint sm) {
    gate_rx_dma_channel = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(gate_rx_dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(
        gate_rx_dma_channel, &c,
        gate_rx_ring_buffer,
        &pio->rxf[sm],
        BUFFER_SIZE,
        true
    );

    dma_irqn_set_channel_enabled(2, gate_rx_dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_2, gate_rx_dma_handler);
    irq_set_enabled(DMA_IRQ_2, true);
}

void vca_rx_dma_init(PIO pio, uint sm) {
    vca_rx_dma_channel = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(vca_rx_dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(
        vca_rx_dma_channel, &c,
        vca_rx_ring_buffer,
        &pio->rxf[sm],
        BUFFER_SIZE,
        true
    );

    dma_irqn_set_channel_enabled(3, vca_rx_dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_3, vca_rx_dma_handler);
    irq_set_enabled(DMA_IRQ_3, true);
}

void initialize_pio_and_dma() {
    PIO pio = pio0;

    uint gate_rx_sm = 0, vca_rx_sm = 1, adsr_tx_sm = 2, vca_tx_sm = 3;

    uint rx_offset = pio_add_program(pio, &dmsp_receive_program);
    uint tx_offset = pio_add_program(pio, &dmsp_transmit_program);

    // Initialize receivers
    dmsp_receiver_init(pio, gate_rx_sm, rx_offset, GATE_IN_PIN);
    gate_rx_dma_init(pio, gate_rx_sm);

    dmsp_receiver_init(pio, vca_rx_sm, rx_offset, VCA_IN_PIN);
    vca_rx_dma_init(pio, vca_rx_sm);

    // Initialize transmitters
    dmsp_transmitter_init(pio, adsr_tx_sm, tx_offset, ADSR_OUT_PIN);
    adsr_tx_dma_init(pio, adsr_tx_sm);

    dmsp_transmitter_init(pio, vca_tx_sm, tx_offset, VCA_OUT_PIN);
    vca_tx_dma_init(pio, vca_tx_sm);
}

// --- ADSR Processing Function ---

#define MAX_VAL (8388607.0f)
#define MIN_VAL (-8388608.0f)

void process_adsr_frames() {
    static float adsr_level[4] = {0.0f};
    static enum { IDLE, ATTACK, DECAY, SUSTAIN, RELEASE } adsr_phase[4] = {IDLE};
    static bool was_gate_active[4] = {false}; // Track previous gate state

    uint32_t local_vca_rx_head = vca_rx_ring_buffer_head;

    while (vca_rx_ring_buffer_tail != local_vca_rx_head) {
        uint32_t frame = vca_rx_ring_buffer[vca_rx_ring_buffer_tail];
        vca_rx_ring_buffer_tail = (vca_rx_ring_buffer_tail + 1) % RING_BUFFER_SIZE;
    
        uint8_t header = frame >> 24;
        int channel = (header >> 4) & 0x07;
    
        if ((header & 0x0F) != 0x0F || channel > 3)
            continue;

        int32_t audio_data = (frame & 0x800000) ? (frame | 0xFF000000) : (frame & 0xFFFFFF);
        bool gate_active = gate_states[channel];

        // Detect rising edge - new note on same channel
        if (gate_active && !was_gate_active[channel]) {
            adsr_phase[channel] = ATTACK;
            adsr_level[channel] = 0.0f;
        }
        was_gate_active[channel] = gate_active;

        // Calculate ADSR parameters
        float attack_rate = 1.0f / (powf(current_knob_values[KNOB_ATTACK], 2) * 48000.0f + 1);
        float decay_rate = (1.0f - current_knob_values[KNOB_SUSTAIN]) / (current_knob_values[KNOB_DECAY] * 48000.0f + 1);
        float sustain_level = current_knob_values[KNOB_SUSTAIN];
        float release_rate = adsr_level[channel] / (current_knob_values[KNOB_RELEASE] * 48000.0f + 1);  // Fixed logic

        // ADSR state machine
        switch (adsr_phase[channel]) {
            case IDLE:
                if (gate_active) {
                    adsr_phase[channel] = ATTACK;
                }
                break;

            case ATTACK:
                adsr_level[channel] += attack_rate;
                if (adsr_level[channel] >= 1.0f) {
                    adsr_level[channel] = 1.0f;
                    adsr_phase[channel] = DECAY;
                }
                if (!gate_active) {
                    adsr_phase[channel] = RELEASE;
                }
                break;

            case DECAY:
                adsr_level[channel] -= decay_rate;
                if (adsr_level[channel] <= sustain_level) {
                    adsr_level[channel] = sustain_level;
                    adsr_phase[channel] = SUSTAIN;
                }
                if (!gate_active) {
                    adsr_phase[channel] = RELEASE;
                }
                break;

            case SUSTAIN:
                adsr_level[channel] = sustain_level;
                if (!gate_active) {
                    adsr_phase[channel] = RELEASE;
                }
                break;

            case RELEASE:
                adsr_level[channel] -= release_rate;
                if (adsr_level[channel] <= 0.0f) {
                    adsr_level[channel] = 0.0f;
                    adsr_phase[channel] = IDLE;
                }
                if (gate_active) {
                    adsr_phase[channel] = ATTACK;
                }
                break;
        }

        // Apply envelope to audio
        float processed_audio = adsr_level[channel] * (float)audio_data;
        processed_audio = fmaxf(MIN_VAL, fminf(MAX_VAL, processed_audio));
        
        uint32_t vca_out_frame = (header << 24) | ((uint32_t)(int32_t)processed_audio & 0xFFFFFF);
        uint32_t next_vca_tx_head = (vca_tx_ring_buffer_head + 1) % RING_BUFFER_SIZE;
        if (next_vca_tx_head != vca_tx_ring_buffer_tail) {
            vca_tx_ring_buffer[vca_tx_ring_buffer_head] = vca_out_frame;
            vca_tx_ring_buffer_head = next_vca_tx_head;
        }

        // Send ADSR envelope as digital control voltage
        uint32_t adsr_value_24bit = (uint32_t)((adsr_level[channel] * 16777215.0f) / amplitude_divisors[current_amp_index]);
        uint32_t adsr_out_frame = (header << 24) | (adsr_value_24bit & 0xFFFFFF);
        uint32_t next_adsr_tx_head = (adsr_tx_ring_buffer_head + 1) % RING_BUFFER_SIZE;
        if (next_adsr_tx_head != adsr_tx_ring_buffer_tail) {
            adsr_tx_ring_buffer[adsr_tx_ring_buffer_head] = adsr_out_frame;
            adsr_tx_ring_buffer_head = next_adsr_tx_head;
        }
    }
}


void process_gate_frames() {
    uint32_t local_gate_rx_head = gate_rx_ring_buffer_head;

    while (gate_rx_ring_buffer_tail != local_gate_rx_head) {
        uint32_t frame = gate_rx_ring_buffer[gate_rx_ring_buffer_tail];
        gate_rx_ring_buffer_tail = (gate_rx_ring_buffer_tail + 1) % RING_BUFFER_SIZE;

        uint8_t header = frame >> 24;

        // Corrected channel extraction: 1cccF format
        int channel = (header >> 4) & 0x07;

        if ((header & 0x0F) != 0x0F || channel > 3)
            continue;

        gate_inputs[channel] = (frame & 0xFFFFFF) != 0;
        gate_states[channel] = gate_inputs[channel];
    }
}



int main()  
{
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

    // Initialize buttons with internal pull-ups
    gpio_init(TIME_BUTTON);
    gpio_set_dir(TIME_BUTTON, GPIO_IN);
    gpio_pull_up(TIME_BUTTON);

    gpio_init(AMPLITUDE_BUTTON);
    gpio_set_dir(AMPLITUDE_BUTTON, GPIO_IN);
    gpio_pull_up(AMPLITUDE_BUTTON);

    // Initialize PIO and DMA
    initialize_pio_and_dma();

    // Set up repeating timer for knob reading (e.g., every 10 ms)
    struct repeating_timer knob_timer;
    add_repeating_timer_ms(15, knob_reading_callback, NULL, &knob_timer);

    printf("[INFO] Starting main loop...\n");

    // Main Loop
    while (true)
    {
        poll_buttons();     // Button state check
        process_gate_frames();
        process_adsr_frames();  // ADSR processing
        tight_loop_contents();
    }


    return 0;
}
