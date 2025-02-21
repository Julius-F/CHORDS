/*
 * File: Oscillator.c
 * Author: CHORDS Group
 * Date: February 20, 2025
 * Description:    This code creates multiple waveformss; Sine, Sawtooth, Square, Triangle, and Noise, using the table lookup meathod. These waves will be tune to the note A at 440Hz. 
 *                 This program will receive input from a MiDi controller that will input frequancy data. Then send the data to the next module in the chain.
 * Version: 1.0
 */



#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>

#define FREQ 440         // Output frequency (440Hz - A4)
#define SAMPLE_RATE 44100 // Sample rate (44.1kHz)
#define TABLE_SIZE 256    // Number of samples in lookup table
#define MAX_LEVEL 4095    // 12-bit PWM resolution

// GPIO pins for output
#define SINE_GPIO 2
#define SAW_GPIO 3
#define SQUARE_GPIO 4
#define TRIANGLE_GPIO 5
#define NOISE_GPIO 6

// Lookup tables
float sine_table[TABLE_SIZE];
float saw_table[TABLE_SIZE];
float square_table[TABLE_SIZE];
float triangle_table[TABLE_SIZE];
float noise_table[TABLE_SIZE];

// Biquad filter structure
typedef struct {
    float a0, a1, a2, b1, b2;
    float z1, z2;
} Biquad;

// Biquad filter processing
float process_biquad(Biquad *filter, float input) {
    float output = filter->a0 * input + filter->a1 * filter->z1 + filter->a2 * filter->z2 
                   - filter->b1 * filter->z1 - filter->b2 * filter->z2;
    filter->z2 = filter->z1;
    filter->z1 = output;
    return output;
}

// Initialize lookup tables
void init_wave_tables() {
    srand(time(NULL));
    for (int i = 0; i < TABLE_SIZE; i++) {
        float phase = (float)i / TABLE_SIZE * 2.0f * M_PI;
        sine_table[i] = sinf(phase);
        saw_table[i] = 2.0f * (i / (float)TABLE_SIZE) - 1.0f;
        square_table[i] = (i < TABLE_SIZE / 2) ? 1.0f : -1.0f;
        triangle_table[i] = fabsf(2.0f * (i / (float)TABLE_SIZE) - 1.0f) * 2.0f - 1.0f;
        noise_table[i] = ((rand() % MAX_LEVEL) / (float)MAX_LEVEL) * 2.0f - 1.0f;
    }
}

// Configure PWM for a given GPIO pin
void configure_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    
    pwm_set_wrap(slice_num, MAX_LEVEL);
    pwm_set_clkdiv(slice_num, (float)SAMPLE_RATE / (FREQ * (MAX_LEVEL + 1)));
    pwm_set_counter(slice_num, 0);
    pwm_set_enabled(slice_num, true);
}

// Set PWM level
void set_pwm_level(uint gpio, uint16_t level) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), level);
}

// Design a low-pass (18kHz) and high-pass (16kHz) filter
void design_filters(Biquad *lowpass, Biquad *highpass) {
    // Low-pass filter (Butterworth 2nd order, cutoff = 18kHz, Fs = 44.1kHz)
    *lowpass = (Biquad){ 
        .a0 = 0.2929, .a1 = 0.5858, .a2 = 0.2929, 
        .b1 = -0.1716, .b2 = 0.1716, 
        .z1 = 0, .z2 = 0 
    };

    // High-pass filter (Butterworth 2nd order, cutoff = 16kHz, Fs = 44.1kHz)
    *highpass = (Biquad){ 
        .a0 = 0.8192, .a1 = -1.6384, .a2 = 0.8192, 
        .b1 = -1.5610, .b2 = 0.6413, 
        .z1 = 0, .z2 = 0 
    };
}

int main() {
    stdio_init_all();

    // Configure PWM outputs
    configure_pwm(SINE_GPIO);
    configure_pwm(SAW_GPIO);
    configure_pwm(SQUARE_GPIO);
    configure_pwm(TRIANGLE_GPIO);
    configure_pwm(NOISE_GPIO);

    // Initialize waveform lookup tables
    init_wave_tables();

    // Create filters
    Biquad lowpass, highpass;
    design_filters(&lowpass, &highpass);

    uint32_t index = 0;
    uint32_t step_size = (TABLE_SIZE * FREQ) / SAMPLE_RATE;

    while (true) {
        // Read values from lookup tables
        float sine_wave = sine_table[index];
        float saw_wave = saw_table[index];
        float square_wave = square_table[index];
        float triangle_wave = triangle_table[index];
        float noise_wave = noise_table[rand() % TABLE_SIZE]; // Pick a random value for noise

        // Apply Low-pass filter (18kHz)
        float sine_lp = process_biquad(&lowpass, sine_wave);
        float saw_lp = process_biquad(&lowpass, saw_wave);
        float square_lp = process_biquad(&lowpass, square_wave);
        float triangle_lp = process_biquad(&lowpass, triangle_wave);
        float noise_lp = process_biquad(&lowpass, noise_wave);

        // Apply High-pass filter (16kHz)
        float sine_hp = process_biquad(&highpass, sine_wave);
        float saw_hp = process_biquad(&highpass, saw_wave);
        float square_hp = process_biquad(&highpass, square_wave);
        float triangle_hp = process_biquad(&highpass, triangle_wave);
        float noise_hp = process_biquad(&highpass, noise_wave);

        // Convert filtered values to PWM range
        uint16_t sine_pwm = (uint16_t)((sine_lp + 1.0f) / 2.0f * MAX_LEVEL);
        uint16_t saw_pwm = (uint16_t)((saw_lp + 1.0f) / 2.0f * MAX_LEVEL);
        uint16_t square_pwm = (uint16_t)((square_lp + 1.0f) / 2.0f * MAX_LEVEL);
        uint16_t triangle_pwm = (uint16_t)((triangle_lp + 1.0f) / 2.0f * MAX_LEVEL);
        uint16_t noise_pwm = (uint16_t)((noise_lp + 1.0f) / 2.0f * MAX_LEVEL);

        // Send to PWM (Choose either lowpass or highpass output)
        set_pwm_level(SINE_GPIO, sine_pwm);
        set_pwm_level(SAW_GPIO, saw_pwm);
        set_pwm_level(SQUARE_GPIO, square_pwm);
        set_pwm_level(TRIANGLE_GPIO, triangle_pwm);
        set_pwm_level(NOISE_GPIO, noise_pwm);

        // Increment index
        index += step_size;
        if (index >= TABLE_SIZE) {
            index -= TABLE_SIZE;
        }

        // Delay to maintain sample rate
        sleep_us(1000000 / SAMPLE_RATE);
    }

    return 0;
}
