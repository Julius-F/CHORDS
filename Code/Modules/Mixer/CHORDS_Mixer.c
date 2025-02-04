#include "pico/stdlib.h"
#include <stdio.h>

/*
int main() {


    int channel;
    int mixerout;
    int dmspdatain;
    bool activechannels[4] = {0,0,0,0};
    int channelvalues[4] = {0,0,0,0};
    int channelcounter[4]= {0,0,0,0};

   while (1) {
    if (channel == 1);
    {
        activechannels[0] = 1;
        channelvalues[0] = dmspdatain;
        channelcounter[0] = 0;
    }
    elif (channel == 2);
    {
        activechannels[1] = 1;
        channelvalues[1] = dmspdatain;
        channelcounter[1] = 0;   
    }
    elif (channel == 3);
    {
        activechannels[2] = 1;
        channelvalues[2] = dmspdatain;
        channelcounter[2] = 0;   
    }
    elif (channel == 4);
    {
        activechannels[3] = 1;
        channelvalues[3] = dmspdatain;
        channelcounter[3] = 0;   
    }

    


   }
   }
    
}
*/

// Main code:

#include <stdio.h>

#define NUM_SIGNALS 4
#define THRESHOLD 0.1  // Define a threshold below which signals are ignored

void mix_signals(float signals[], float *output) {
    int valid_count = 0;
    float sum = 0.0;
    
    for (int i = 0; i < NUM_SIGNALS; i++) {
        if (signals[i] > THRESHOLD) { // Only mix signals above threshold
            sum += signals[i];
            valid_count++;
        }
    }
    
    if (valid_count > 0) {
        *output = sum / valid_count;  // Normalize by the number of active signals
    } else {
        *output = 0.0; // No valid signals, output zero
    }
}

int main() {
    float signals[NUM_SIGNALS] = {0.5, 0.05, 0.3, 0.0}; // Example signals
    float output;
    
    mix_signals(signals, &output);
    printf("Mixed Output: %f\n", output);
    
    return 0;
}

// Test code:

void test_mixer() {
    float test_cases[][NUM_SIGNALS] = {
        {0.5, 0.05, 0.3, 0.0},
        {0.2, 0.15, 0.05, 0.0},
        {0.0, 0.0, 0.0, 0.0},
        {0.25, 0.25, 0.25, 0.25},
        {0.12, 0.08, 0.15, 0.09}
    };
    float output;
    int num_tests = sizeof(test_cases) / sizeof(test_cases[0]);
    
    for (int i = 0; i < num_tests; i++) {
        mix_signals(test_cases[i], &output);
        printf("Test %d: Mixed Output = %f\n", i + 1, output);
    }
}

int main() {
    test_mixer();
    return 0;
}

