#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

#define POTENTIOMETER_PIN 26 // ADC0
#define SIGNAL_INPUT_PIN 27  // ADC1
#define PWM_OUTPUT_PIN 15   // GPIO for PWM output

// Sampling rate for ADC
#define SAMPLE_RATE 20000 // 20 kHz

// Variables for low-pass filter
float previous_output = 0.0;

void init_adc() {
    adc_init();
    adc_gpio_init(POTENTIOMETER_PIN);
    adc_gpio_init(SIGNAL_INPUT_PIN);
}

void init_pwm(uint slice_num, uint channel) {
    gpio_set_function(PWM_OUTPUT_PIN, GPIO_FUNC_PWM);
    pwm_set_wrap(slice_num, 255); // 8-bit resolution
    pwm_set_chan_level(slice_num, channel, 0);
    pwm_set_enabled(slice_num, true);
}

float lowpass_filter(float input, float alpha) {
    float output = alpha * input + (1 - alpha) * previous_output;
    previous_output = output;
    return output;
}

int main() {
    stdio_init_all();

    // Initialize ADC and PWM
    init_adc();
    uint slice_num = pwm_gpio_to_slice_num(PWM_OUTPUT_PIN);
    uint channel = pwm_gpio_to_channel(PWM_OUTPUT_PIN);
    init_pwm(slice_num, channel);

    while (true) {
        // Read potentiometer value to determine cutoff frequency
        adc_select_input(0); // Select ADC0 (Potentiometer)
        uint16_t pot_value = adc_read();
        float alpha = (float)pot_value / 4095.0; // Normalize to [0, 1]

        // Read input signal
        adc_select_input(1); // Select ADC1 (Signal Input)
        uint16_t input_signal = adc_read();
        float normalized_input = (float)input_signal / 4095.0; // Normalize input to [0, 1]

        // Apply low-pass filter
        float filtered_signal = lowpass_filter(normalized_input, alpha);

        // Output the filtered signal using PWM
        uint16_t pwm_value = (uint16_t)(filtered_signal * 255); // Scale to 8-bit resolution
        pwm_set_chan_level(slice_num, channel, pwm_value);

        // Sleep to maintain the sample rate
        sleep_us(1000000 / SAMPLE_RATE);
    }

    return 0;
}
