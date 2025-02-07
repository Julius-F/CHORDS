#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"

int main() {
    stdio_init_all();

    // Source data
    const char src[] = "Hello, DMA!";
    // Destination buffer
    char dst[sizeof(src)];

    // Claim an unused DMA channel
    int dma_chan = dma_claim_unused_channel(true);

    // Configure the DMA channel
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32); // 8-bit transfers
    channel_config_set_read_increment(&cfg, true);  // Increment read address
    channel_config_set_write_increment(&cfg, true); // Increment write address

    // Set up the DMA channel
    dma_channel_configure(
        dma_chan,
        &cfg,
        dst,        // Destination pointer
        src,        // Source pointer
        sizeof(src),// Number of transfers
        true        // Start immediately
    );

    // Wait for the DMA transfer to complete
    dma_channel_wait_for_finish_blocking(dma_chan);

    // Output the result
    printf("DMA Transfer Complete: %s\n", dst);

    return 0;
}
