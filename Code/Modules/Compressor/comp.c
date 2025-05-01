#include <stdint.h>
#include <math.h>
#include <stdbool.h>

// Configuration parameters
#define SAMPLE_RATE 44100     // Hz
#define BUFFER_SIZE 256       // Processing buffer size
#define FFT_SIZE 256         // FFT size for frequency analysis
#define NUM_BANDS 8           // Number of frequency bands to analyze
#define COMP_RATIO 4.0f       // Compression ratio (4:1)
#define THRESHOLD -20.0f      // Compression threshold in dB
#define ATTACK_MS 10.0f       // Attack time in milliseconds
#define RELEASE_MS 100.0f     // Release time in milliseconds
#define MAKEUP_GAIN 6.0f      // Additional output gain in dB

// Global variables
float inputBuffer[BUFFER_SIZE];
float outputBuffer[BUFFER_SIZE];
float fftBuffer[FFT_SIZE * 2]; // Real + Imaginary
float window[FFT_SIZE];
float bandGains[NUM_BANDS];
float prevBandGains[NUM_BANDS];

// Initialize the compressor
void compressorInit() {
    // Initialize window function (Hanning)
    for (int i = 0; i < FFT_SIZE; i++) {
        window[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (FFT_SIZE - 1)));
    }
    
    // Initialize band gains
    for (int i = 0; i < NUM_BANDS; i++) {
        bandGains[i] = 1.0f;
        prevBandGains[i] = 1.0f;
    }
}

// Simple FFT implementation (radix-2)
void fft(float* x, float* y, uint16_t n) {
    uint16_t i, j, k, m;
    float wr, wi, tr, ti;
    
    // Bit-reverse ordering
    j = 0;
    for (i = 0; i < n - 1; i++) {
        if (i < j) {
            tr = x[i];
            ti = y[i];
            x[i] = x[j];
            y[i] = y[j];
            x[j] = tr;
            y[j] = ti;
        }
        
        k = n >> 1;
        while (k <= j) {
            j -= k;
            k >>= 1;
        }
        j += k;
    }
    
    // Butterfly computations
    m = 1;
    while (m < n) {
        for (k = 0; k < m; k++) {
            wr = cosf(M_PI * k / m);
            wi = -sinf(M_PI * k / m);
            
            for (i = k; i < n; i += 2 * m) {
                j = i + m;
                tr = wr * x[j] - wi * y[j];
                ti = wr * y[j] + wi * x[j];
                x[j] = x[i] - tr;
                y[j] = y[i] - ti;
                x[i] += tr;
                y[i] += ti;
            }
        }
        m <<= 1;
    }
}

// Detect peak frequencies and calculate band gains
void analyzeFrequencies(float* input) {
    // Apply window function and prepare FFT buffer
    for (int i = 0; i < FFT_SIZE; i++) {
        fftBuffer[i] = input[i] * window[i]; // Real part
        fftBuffer[i + FFT_SIZE] = 0.0f;      // Imaginary part
    }
    
    // Perform FFT
    fft(fftBuffer, fftBuffer + FFT_SIZE, FFT_SIZE);
    
    // Calculate frequency band energies
    float bandEnergies[NUM_BANDS] = {0};
    int bandWidth = FFT_SIZE / (2 * NUM_BANDS); // Only first half is useful
    
    for (int band = 0; band < NUM_BANDS; band++) {
        int startBin = band * bandWidth;
        int endBin = (band + 1) * bandWidth;
        
        for (int bin = startBin; bin < endBin; bin++) {
            float real = fftBuffer[bin];
            float imag = fftBuffer[bin + FFT_SIZE];
            float power = real * real + imag * imag;
            bandEnergies[band] += power;
        }
        
        // Convert to dB
        bandEnergies[band] = 10.0f * log10f(bandEnergies[band] + 1e-12f);
    }
    
    // Calculate compression for each band
    for (int band = 0; band < NUM_BANDS; band++) {
        if (bandEnergies[band] > THRESHOLD) {
            // Calculate compression amount
            float overThreshold = bandEnergies[band] - THRESHOLD;
            float compressed = overThreshold / COMP_RATIO;
            float desiredGain = THRESHOLD + compressed - bandEnergies[band];
            
            // Convert dB gain to linear
            float gain = powf(10.0f, desiredGain / 20.0f);
            
            // Smooth gain changes with attack/release
            float timeCoeff = (gain < prevBandGains[band]) ? 
                (1.0f - expf(-1.0f / (ATTACK_MS * SAMPLE_RATE / 1000.0f))) : 
                (1.0f - expf(-1.0f / (RELEASE_MS * SAMPLE_RATE / 1000.0f)));
            
            bandGains[band] = prevBandGains[band] + timeCoeff * (gain - prevBandGains[band]);
            prevBandGains[band] = bandGains[band];
        } else {
            bandGains[band] = 1.0f; // No compression
            prevBandGains[band] = 1.0f;
        }
    }
}

// Apply compression to the audio signal
void applyCompression(float* input, float* output, uint16_t size) {
    // Simple time-domain implementation (for real-time processing)
    static float envelope = 0.0f;
    float attackCoeff = expf(-1.0f / (ATTACK_MS * SAMPLE_RATE / 1000.0f));
    float releaseCoeff = expf(-1.0f / (RELEASE_MS * SAMPLE_RATE / 1000.0f));
    float makeupGain = powf(10.0f, MAKEUP_GAIN / 20.0f);
    
    for (int i = 0; i < size; i++) {
        // Calculate signal level (absolute value)
        float level = fabsf(input[i]);
        
        // Update envelope
        if (level > envelope) {
            envelope = attackCoeff * envelope + (1.0f - attackCoeff) * level;
        } else {
            envelope = releaseCoeff * envelope + (1.0f - releaseCoeff) * level;
        }
        
        // Convert to dB
        float dB = 20.0f * log10f(envelope + 1e-12f);
        
        // Apply compression
        float gain = 1.0f;
        if (dB > THRESHOLD) {
            float overThreshold = dB - THRESHOLD;
            float compressed = overThreshold / COMP_RATIO;
            gain = powf(10.0f, (THRESHOLD + compressed - dB) / 20.0f);
        }
        
        // Apply makeup gain and store result
        output[i] = input[i] * gain * makeupGain;
        
        // Optional: Apply band-specific gains (would require filter bank)
        // output[i] *= getBandGainForFrequency(...);
    }
}

// Main processing loop
void processAudio() {
    while (true) {
        // 1. Read audio input (from ADC)
        // readADC(inputBuffer, BUFFER_SIZE);
        
        // 2. Analyze frequencies and update band gains
        analyzeFrequencies(inputBuffer);
        
        // 3. Apply compression
        applyCompression(inputBuffer, outputBuffer, BUFFER_SIZE);
        
        // 4. Write audio output (to DAC)
        // writeDAC(outputBuffer, BUFFER_SIZE);
    }
}

int main() {
    compressorInit();
    processAudio();
    return 0;
}
