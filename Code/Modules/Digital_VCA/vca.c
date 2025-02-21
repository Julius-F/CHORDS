#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "dmsp_receive.pio.h"
#include "dmsp_transmit.pio.h"

// Define GPIO pins
#define DMSP_IN_PIN 16        
#define GLOBAL_CLK_PIN 14     
#define CHANNEL_CTRL_PIN 15   
#define DMSP_OUT_PIN 17       
#define ANALOG_IN_PIN 26      

#define BUFFER_SIZE 512
#define RING_BUFFER_SIZE 1024   

uint32_t rx_ring_buffer[RING_BUFFER_SIZE];
volatile uint32_t rx_ring_buffer_head = 0;
volatile uint32_t rx_ring_buffer_tail = 0;

uint32_t tx_ring_buffer[RING_BUFFER_SIZE];
volatile uint32_t tx_ring_buffer_head = 0;
volatile uint32_t tx_ring_buffer_tail = 0;

int rx_dma_channel;
int tx_dma_channel;

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

// ** Blink LED for Debugging **
void blink_led(int times) {
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    for (int i = 0; i < times; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(200);
        gpio_put(LED_PIN, 0);
        sleep_ms(200);
    }
}

void rx_dma_handler() {
    dma_hw->ints0 = 1u << rx_dma_channel;
    rx_ring_buffer_head = (rx_ring_buffer_head + BUFFER_SIZE) % RING_BUFFER_SIZE;

    dma_channel_set_write_addr(rx_dma_channel, &rx_ring_buffer[rx_ring_buffer_head], true);
    dma_channel_set_trans_count(rx_dma_channel, BUFFER_SIZE, true);

    printf("[DEBUG] RX DMA Transfer completed, head=%d, tail=%d\n", rx_ring_buffer_head, rx_ring_buffer_tail);
}

void tx_dma_handler() {
    dma_hw->ints1 = 1u << tx_dma_channel;
    tx_ring_buffer_tail = (tx_ring_buffer_tail + BUFFER_SIZE) % RING_BUFFER_SIZE;

    dma_channel_set_read_addr(tx_dma_channel, &tx_ring_buffer[tx_ring_buffer_tail], true);
    dma_channel_set_trans_count(tx_dma_channel, BUFFER_SIZE, true);

    printf("[DEBUG] TX DMA Transfer completed, head=%d, tail=%d\n", tx_ring_buffer_head, tx_ring_buffer_tail);
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

    printf("[DEBUG] RX DMA Initialized\n");
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

    dma_channel_set_irq1_enabled(tx_dma_channel, true);  // Using IRQ1 instead of IRQ0
    irq_set_exclusive_handler(DMA_IRQ_1, tx_dma_handler);
    irq_set_enabled(DMA_IRQ_1, true);

    printf("[DEBUG] TX DMA Initialized\n");
}

void process_data() {
    // Read analog input (A0)
    uint16_t analog_value = adc_read();

    // Normalize analog value to a scaling factor (0.0 to 1.0)
    float scale_factor = (float)analog_value / 4095.0f;

    // Process data from the RX ring buffer
    while (rx_ring_buffer_tail != rx_ring_buffer_head) {
        // Read data from the RX ring buffer
        uint32_t data = rx_ring_buffer[rx_ring_buffer_tail];
        rx_ring_buffer_tail = (rx_ring_buffer_tail + 1) % RING_BUFFER_SIZE;

        // Extract the 24-bit audio data (lower 24 bits)
        int32_t audio_data = data & 0xFFFFFF;

        // Apply attenuation (Digital VCA)
        audio_data = (int32_t)(audio_data * scale_factor);

        // Combine with the header (0xFF)
        uint32_t processed_data = 0xFF000000 | (audio_data & 0xFFFFFF);

        // ** Prevent Overflow **
        if (((tx_ring_buffer_head + 1) % RING_BUFFER_SIZE) != tx_ring_buffer_tail) {
            tx_ring_buffer[tx_ring_buffer_head] = processed_data;
            printf("[DEBUG] TX Buffer[%d] = %08X\n", tx_ring_buffer_head, processed_data);
            tx_ring_buffer_head = (tx_ring_buffer_head + 1) % RING_BUFFER_SIZE;
        } else {
            printf("[ERROR] TX Buffer Overflow!\n");
        }
    }
}


int main() {
    sleep_ms(2000); // Delay to ensure the serial monitor can connect
    stdio_init_all();
    
    blink_led(3); // Blink LED three times at startup

    adc_init();
    adc_gpio_init(ANALOG_IN_PIN);
    adc_select_input(0);

    PIO pio = pio0;
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

    printf("[INFO] Initialization complete. Running main loop.\n");

    while (true) {
        process_data();
        sleep_us(1);
    }
}
