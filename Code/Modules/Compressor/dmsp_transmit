.program dmsp_transmit

; Define GPIO pins
.define GLOBAL_CLK_PIN 14  ; GPIO 14 as global clock input
.define CHANNEL_CTRL_PIN 15 ; GPIO 15 as sync signal input
.define DMSP_OUT_PIN 17    ; GPIO 16 as output for DMSP data

; Main loop
.wrap_target
    ; Wait for sync signal to go high (start of a new frame)
    wait 1 GPIO, CHANNEL_CTRL_PIN  ; Wait for GPIO 15 (sync) to go high

    ; Wait for sync signal to go low
    wait 0 GPIO, CHANNEL_CTRL_PIN  ; Wait for GPIO 15 (sync) to go low

    ; Pull 32-bit data from TX FIFO into OSR (Output Shift Register)
    pull                       ; Load data from TX FIFO into OSR

    ; Transmit 32 bits to GPIO 16, synchronized with global clock
    set x, 31                  ; Initialize loop counter (32 bits)
bitloop:
    ; Wait for the rising edge of the global clock
    wait 1 GPIO, GLOBAL_CLK_PIN ; Wait for GPIO 14 (global clock) to go high

    ; Shift out 1 bit to GPIO 16
    out pins, 1                ; Shift out 1 bit to GPIO 16

    ; Wait for the falling edge of the global clock (optional, for clean timing)
    wait 0 GPIO, GLOBAL_CLK_PIN ; Wait for GPIO 14 (global clock) to go low

    ; Decrement x and loop until all 32 bits are sent
    jmp x-- bitloop            ; Decrement x and loop until all 32 bits are sent
.wrap
