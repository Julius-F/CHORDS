#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "dmsp_transmit.pio.h"

#define MIDI_UART       uart1
#define MIDI_RX_PIN     9
#define MIDI_BAUD_RATE  31250

// Shared control pins
#define GLOBAL_CLK_PIN      7     // Clock from master
#define CHANNEL_CTRL_PIN    6     // Channel control from master

// Separate data output pins
#define PITCH_OUT_PIN       20
#define GATE_OUT_PIN        12

#define BUFFER_SIZE     512
#define RING_BUFFER_SIZE 1024

// Default pitch bend (center, no bend) for a 17-bit field.
#define DEFAULT_PITCH_BEND (1 << 16)

// Frame type definitions
#define FRAME_TYPE_MIDI_NOTE 0x3
#define FRAME_TYPE_GATE 0xF

// Ring buffers
uint32_t tx_ring_buffer[RING_BUFFER_SIZE];
volatile uint32_t tx_ring_buffer_head = 0;
volatile uint32_t tx_ring_buffer_tail = 0;

uint32_t gate_tx_ring_buffer[RING_BUFFER_SIZE];
volatile uint32_t gate_tx_ring_buffer_head = 0;
volatile uint32_t gate_tx_ring_buffer_tail = 0;

// DMA channels
int tx_dma_channel;
int gate_tx_dma_channel;

// Channel state
#define NUM_CHANNELS 4
typedef struct {
    bool active;
    uint8_t note;
} ChannelState;

ChannelState channels[NUM_CHANNELS];

void send_dmsp_frame(uint8_t channel, uint8_t midi_note) {
    uint8_t header = 0x80 | ((channel & 0x07) << 4) | FRAME_TYPE_MIDI_NOTE;
    uint32_t data = (((uint32_t)midi_note & 0x7F) << 17) | (DEFAULT_PITCH_BEND & 0x1FFFF);
    uint32_t frame = (((uint32_t)header) << 24) | (data & 0xFFFFFF);
    
    tx_ring_buffer[tx_ring_buffer_head] = frame;
    tx_ring_buffer_head = (tx_ring_buffer_head + 1) % RING_BUFFER_SIZE;
}

void send_gate_frame(uint8_t channel, bool gate_state) {
    uint8_t header = 0x80 | ((channel & 0x07) << 4) | FRAME_TYPE_GATE;
    uint32_t data = gate_state ? 0xFFFFFF : 0x000000;
    uint32_t frame = (((uint32_t)header) << 24) | (data & 0xFFFFFF);
    
    gate_tx_ring_buffer[gate_tx_ring_buffer_head] = frame;
    gate_tx_ring_buffer_head = (gate_tx_ring_buffer_head + 1) % RING_BUFFER_SIZE;
}

uint8_t find_free_channel(void) {
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if (!channels[i].active) return i;
    }
    return 0;
}

void assign_note_to_channel(uint8_t midi_note) {
    uint8_t ch = find_free_channel();
    channels[ch].active = true;
    channels[ch].note = midi_note;
}

void remove_note_from_channel(uint8_t midi_note) {
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if (channels[i].active && channels[i].note == midi_note) {
            channels[i].active = false;
            break;
        }
    }
}

// Initialize both state machines in the same PIO block
void dmsp_transmitters_init(PIO pio, uint pitch_sm, uint gate_sm, uint offset) {
    // Initialize shared pins (only once)
    pio_gpio_init(pio, GLOBAL_CLK_PIN);
    pio_gpio_init(pio, CHANNEL_CTRL_PIN);
    
    // Initialize pitch output
    pio_gpio_init(pio, PITCH_OUT_PIN);
    pio_sm_set_consecutive_pindirs(pio, pitch_sm, PITCH_OUT_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(pio, pitch_sm, GLOBAL_CLK_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio, pitch_sm, CHANNEL_CTRL_PIN, 1, false);

    pio_sm_config pitch_c = dmsp_transmit_program_get_default_config(offset);
    sm_config_set_out_pins(&pitch_c, PITCH_OUT_PIN, 1);
    sm_config_set_in_pins(&pitch_c, GLOBAL_CLK_PIN);
    sm_config_set_jmp_pin(&pitch_c, CHANNEL_CTRL_PIN);
    sm_config_set_out_shift(&pitch_c, false, false, 32);
    pio_sm_init(pio, pitch_sm, offset, &pitch_c);

    // Initialize gate output
    pio_gpio_init(pio, GATE_OUT_PIN);
    pio_sm_set_consecutive_pindirs(pio, gate_sm, GATE_OUT_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(pio, gate_sm, GLOBAL_CLK_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio, gate_sm, CHANNEL_CTRL_PIN, 1, false);

    pio_sm_config gate_c = dmsp_transmit_program_get_default_config(offset);
    sm_config_set_out_pins(&gate_c, GATE_OUT_PIN, 1);
    sm_config_set_in_pins(&gate_c, GLOBAL_CLK_PIN);
    sm_config_set_jmp_pin(&gate_c, CHANNEL_CTRL_PIN);
    sm_config_set_out_shift(&gate_c, false, false, 32);
    pio_sm_init(pio, gate_sm, offset, &gate_c);

    // Start both state machines
    pio_sm_set_enabled(pio, pitch_sm, true);
    pio_sm_set_enabled(pio, gate_sm, true);
}

void __not_in_flash_func(tx_dma_handler)() {
    dma_hw->ints0 = 1u << tx_dma_channel;  // Using IRQ0 for pitch DMA
    tx_ring_buffer_tail = (tx_ring_buffer_tail + BUFFER_SIZE) % RING_BUFFER_SIZE;
    dma_channel_set_read_addr(tx_dma_channel, &tx_ring_buffer[tx_ring_buffer_tail], true);
    dma_channel_set_trans_count(tx_dma_channel, BUFFER_SIZE, true);
}

void __not_in_flash_func(gate_tx_dma_handler)() {
    dma_hw->ints1 = 1u << gate_tx_dma_channel;  // Using IRQ1 for gate DMA
    gate_tx_ring_buffer_tail = (gate_tx_ring_buffer_tail + BUFFER_SIZE) % RING_BUFFER_SIZE;
    dma_channel_set_read_addr(gate_tx_dma_channel, &gate_tx_ring_buffer[gate_tx_ring_buffer_tail], true);
    dma_channel_set_trans_count(gate_tx_dma_channel, BUFFER_SIZE, true);
}

void dma_init(PIO pio, uint pitch_sm, uint gate_sm) {
    // Pitch DMA (using IRQ0)
    tx_dma_channel = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(tx_dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, pitch_sm, true));
    
    dma_channel_configure(
        tx_dma_channel, &c,
        &pio->txf[pitch_sm],
        tx_ring_buffer,
        BUFFER_SIZE,
        true
    );
    dma_channel_set_irq0_enabled(tx_dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, tx_dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Gate DMA (using IRQ1)
    gate_tx_dma_channel = dma_claim_unused_channel(true);
    dma_channel_config gate_c = dma_channel_get_default_config(gate_tx_dma_channel);
    channel_config_set_transfer_data_size(&gate_c, DMA_SIZE_32);
    channel_config_set_read_increment(&gate_c, true);
    channel_config_set_write_increment(&gate_c, false);
    channel_config_set_dreq(&gate_c, pio_get_dreq(pio, gate_sm, true));
    
    dma_channel_configure(
        gate_tx_dma_channel, &gate_c,
        &pio->txf[gate_sm],
        gate_tx_ring_buffer,
        BUFFER_SIZE,
        true
    );
    dma_channel_set_irq1_enabled(gate_tx_dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_1, gate_tx_dma_handler);
    irq_set_enabled(DMA_IRQ_1, true);
}

void handle_midi_command(uint8_t status, uint8_t data1, uint8_t data2) {
    uint8_t command = status & 0xF0;
    if (command == 0x90) { // Note On
        if (data2 > 0) {
            assign_note_to_channel(data1);
        } else {
            remove_note_from_channel(data1);
        }
    } else if (command == 0x80) { // Note Off
        remove_note_from_channel(data1);
    }
}

int main() {
    stdio_init_all();

    // Initialize PIO
    PIO pio = pio0;
    
    // Load program once (shared by both state machines)
    uint offset = pio_add_program(pio, &dmsp_transmit_program);
    
    // Use state machines 0 and 1
    uint pitch_sm = 0;
    uint gate_sm = 1;
    
    // Initialize both transmitters
    dmsp_transmitters_init(pio, pitch_sm, gate_sm, offset);
    
    // Initialize DMA
    dma_init(pio, pitch_sm, gate_sm);

    // MIDI setup
    uart_init(MIDI_UART, MIDI_BAUD_RATE);
    gpio_set_function(MIDI_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(MIDI_RX_PIN);

    // Initialize channels
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        channels[i].active = false;
    }

    uint8_t midi_buffer[3];
    int byte_index = 0;

    while (true) {
        // MIDI processing
        if (uart_is_readable(MIDI_UART)) {
            uint8_t midi_byte = uart_getc(MIDI_UART);
            if (midi_byte & 0x80) {
                midi_buffer[0] = midi_byte;
                byte_index = 1;
            } else if (byte_index > 0 && byte_index < 3) {
                midi_buffer[byte_index++] = midi_byte;
                if (byte_index == 3) {
                    handle_midi_command(midi_buffer[0], midi_buffer[1], midi_buffer[2]);
                    byte_index = 0;
                }
            }
        }
        
        // Buffer management
        uint32_t pitch_free = (tx_ring_buffer_head >= tx_ring_buffer_tail) ?
            RING_BUFFER_SIZE - (tx_ring_buffer_head - tx_ring_buffer_tail) :
            tx_ring_buffer_tail - tx_ring_buffer_head;
            
        uint32_t gate_free = (gate_tx_ring_buffer_head >= gate_tx_ring_buffer_tail) ?
            RING_BUFFER_SIZE - (gate_tx_ring_buffer_head - gate_tx_ring_buffer_tail) :
            gate_tx_ring_buffer_tail - gate_tx_ring_buffer_head;
        
        if (pitch_free >= NUM_CHANNELS && gate_free >= NUM_CHANNELS) {
            for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
                send_dmsp_frame(i, channels[i].active ? channels[i].note : 0);
                send_gate_frame(i, channels[i].active);
            }
        }
        
        sleep_us(1);
    }
    return 0;
}
