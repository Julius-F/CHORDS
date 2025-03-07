#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "dmsp_transmit.pio.h"
#include <math.h>

// Define GPIO pins
#define GLOBAL_CLK_PIN 14
#define CHANNEL_CTRL_PIN 15
#define DMSP_OUT_PIN 16

// Constants
#define SAMPLE_RATE 40000
#define WAVETABLE_SIZE 21504
#define MIDI_NOTE_BITS 7
#define PITCH_BEND_BITS 17
#define MAX_PITCH_BEND (1 << PITCH_BEND_BITS) - 1
#define RING_BUFFER_SIZE 65536

// Wavetable (sine wave)
float wavetable[WAVETABLE_SIZE];

// Persistent phase and phase increment
static float phase = 0.0f;
static float phaseIncrement = 0.0f;

// Current MIDI note and pitch bend
uint8_t current_midi_note = 32;
uint32_t current_pitch_bend = MAX_PITCH_BEND / 2;

// Amplitude control
float amplitude = 0.5f;

// Ring buffer
uint32_t ring_buffer[RING_BUFFER_SIZE];
volatile int ring_buffer_head = 0;
volatile int ring_buffer_tail = 0;

// DMA channel
int dma_chan;

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

/*

// Use a sawtooth from -1.0 to +1.0
void init_wavetable() {
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
        // Simple bipolar saw: -1.0 at i=0 up to nearly +1.0 at i=1023
        wavetable[i] = (2.0f * i / (float)WAVETABLE_SIZE) - 1.0f;
    }
}

*/

void init_wavetable() {
    for (int i = 0; i < WAVETABLE_SIZE; i++) {
        float phase = (2.0f * M_PI * i) / (float)WAVETABLE_SIZE;
        wavetable[i] = sinf(phase);
    }
}



float midi_note_to_frequency(uint8_t midi_note) {
    return 440.0f * powf(2.0f, (float)(midi_note - 69) / 12.0f);
}

float apply_pitch_bend(float base_frequency, uint32_t pitch_bend) {
    float bend = (float)((int32_t)pitch_bend - (MAX_PITCH_BEND / 2)) / (float)(MAX_PITCH_BEND / 2);
    return base_frequency * powf(2.0f, bend * 2.0f / 12.0f);
}

void update_frequency(float new_frequency) {
    float new_phaseIncrement = (new_frequency * WAVETABLE_SIZE) / SAMPLE_RATE;
    phaseIncrement = (0.95f * phaseIncrement) + (0.05f * new_phaseIncrement);
}

float generate_sample() {
    int index = (int)phase;
    float frac = phase - index;
    float sample = (wavetable[index] * (1.0f - frac)) + (wavetable[(index + 1) % WAVETABLE_SIZE] * frac);
    
    phase += phaseIncrement;
    if (phase >= WAVETABLE_SIZE) {
        phase -= WAVETABLE_SIZE;
    }
    return sample * amplitude;
}

void fill_ring_buffer() {
    float base_frequency = midi_note_to_frequency(current_midi_note);
    float frequency = apply_pitch_bend(base_frequency, current_pitch_bend);
    
    update_frequency(frequency);

    while ((ring_buffer_head + 1) % RING_BUFFER_SIZE != ring_buffer_tail) {
        // Determine the current channel (cycles through 1, 2, 3, 4)
        uint8_t channel = (ring_buffer_head % 4) + 1;

        // Construct the frame header based on the channel
        uint8_t header;
        switch (channel) {
            case 1:
                header = 0x8F;  // 1 (flag) + 000 (channel 1) + 1111 (frame type 0xF)
                break;
            case 2:
                header = 0x9F;  // 1 (flag) + 001 (channel 2) + 1111 (frame type 0xF)
                break;
            case 3:
                header = 0xAF;  // 1 (flag) + 010 (channel 3) + 1111 (frame type 0xF)
                break;
            case 4:
                header = 0xBF;  // 1 (flag) + 011 (channel 4) + 1111 (frame type 0xF)
                break;
            default:
                header = 0x8F;  // Default to channel 1 (should not happen)
                break;
        }

        // Generate audio data only for channel 1
        int32_t audio_data = 0x000000;  // Default to no data
        if (channel == 1) {
            float sample = generate_sample();
            audio_data = (int32_t)(sample * 0x7FFFFF);  // Scale to 24-bit signed
        }

        // Combine the header and audio data into a 32-bit word
        uint32_t frame = ((uint32_t)header << 24) | (audio_data & 0xFFFFFF);

        ring_buffer[ring_buffer_head] = frame;
        ring_buffer_head = (ring_buffer_head + 1) % RING_BUFFER_SIZE;
    }
}

void dma_handler() {
    dma_hw->ints0 = 1u << dma_chan;
    ring_buffer_tail = (ring_buffer_tail + RING_BUFFER_SIZE / 2) % RING_BUFFER_SIZE;
    dma_channel_set_read_addr(dma_chan, &ring_buffer[ring_buffer_tail], true);
}

void midi_interpreter(uint8_t* data) {
    if (data[0] == 0xFF) {
        current_midi_note = data[1] & 0x7F;
        current_pitch_bend = (data[2] << 9) | (data[3] << 1) | (data[4] >> 7);
    }
}

int main() {
    stdio_init_all();
    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &dmsp_transmit_program);
    dmsp_transmitter_init(pio, sm, offset);
    init_wavetable();

    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));

    dma_channel_configure(dma_chan, &c, &pio->txf[sm], &ring_buffer[ring_buffer_tail], RING_BUFFER_SIZE / 2, true);
    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    while (true) {
        fill_ring_buffer();
        sleep_us(1); //for tighter loop; adjust if necessary
    }
}
