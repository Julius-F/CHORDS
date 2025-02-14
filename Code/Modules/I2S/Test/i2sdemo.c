#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "i2s_tx.pio.h"  // Include the generated PIO header file

#define SAMPLE_RATE 44100  // 44.1 kHz sample rate
#define SINE_WAVE_FREQ 440  // 440 Hz sine wave (A4 note)
#define AMPLITUDE 0x7FFFFFFF  // Maximum amplitude for 32-bit audio

// Function to generate a sine wave sample
int32_t generate_sine_sample(uint32_t phase) {
    return (int32_t)(sinf(2.0f * M_PI * (float)phase / (float)SAMPLE_RATE) * AMPLITUDE);
}

int main() {
    stdio_init_all();  // Initialize standard I/O

    // Define the GPIO pins for I2S
    const uint data_pin = 0;  // Data pin (e.g., DIN)
    const uint bclk_pin = 1;  // Bit clock pin (BCLK)
    const uint lrclk_pin = 2; // Word select pin (LRCLK)

    // Initialize the PIO and state machine
    PIO pio = pio0;  // Use PIO 0
    uint sm = 0;     // Use state machine 0
    uint offset = pio_add_program(pio, &i2s_tx_program);  // Load the PIO program
    i2s_tx_program_init(pio, sm, offset, data_pin, bclk_pin, lrclk_pin);  // Initialize the I2S TX program

    // Set the clock divider for the PIO state machine
    float div = (float)clock_get_hz(clk_sys) / (SAMPLE_RATE * 64);  // 64 cycles per sample (32 bits x 2 channels)
    pio_sm_set_clkdiv(pio, sm, div);

    // Enable the state machine
    pio_sm_set_enabled(pio, sm, true);

    uint32_t phase = 0;  // Phase accumulator for the sine wave
    while (true) {
        // Generate a sine wave sample for the left and right channels
        int32_t sample = generate_sine_sample(phase);

        // Push the sample to the PIO TX FIFO (left channel)
        pio_sm_put_blocking(pio, sm, sample);

        // Push the same sample to the PIO TX FIFO (right channel)
        pio_sm_put_blocking(pio, sm, sample);

        // Increment the phase accumulator
        phase = (phase + 1) % SAMPLE_RATE;

        // Add a small delay to maintain the sample rate
        sleep_us(1000000 / SAMPLE_RATE);
    }

    return 0;
}
