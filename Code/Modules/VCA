#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <stdio.h>

int main() {
    // Initialize stdio for output
    stdio_init_all();

    // Initialize the ADC hardware
    adc_init();

    // Define GPIO pins for the analog inputs (26, 27, 28, and 29 are ADC pins on Pico)
    adc_gpio_init(26); // ADC0
    adc_gpio_init(27); // ADC1
    adc_gpio_init(28); // ADC2
    adc_gpio_init(29); // ADC3

    // ADC channel mapping
    const uint8_t adc_channels[4] = {0, 1, 2, 3};
    const int max_adc_value = 4095; // 12-bit ADC resolution
    int channel;
    int dmspdataout[4] = {0,0,0,0};
    int dmspdatain;

   /*while (1) {
        // Read and calculate percentage for each channel
        for (int i = 0; i < 4; i++) {
            adc_select_input(adc_channels[i]); // Select the current ADC channel
            uint16_t raw_value = adc_read(); // Read raw ADC value
            float percentage = (raw_value / (float)max_adc_value) * 100.0f;

            printf("ADC Channel %d: Raw Value = %u, Percentage = %.2f%%\n", 
                    adc_channels[i], raw_value, percentage);
        }

        printf("\n"); // Add spacing between readings
        sleep_ms(1000); // Wait for 1 second before next reading
    }
    */
   int i;
   float percent;
   int lastchannel;
   while (1) {
    if (lastchannel != channel) {
    i = channel;
    adc_select_input(adc_channels[i]); 
    uint16_t raw_value = adc_read();
    percent = raw_value / (float)max_adc_value;
    dmspdataout[i] = dmspdatain * percent; 
    lastchannel = channel;
   }
   }
    return 0;
}
