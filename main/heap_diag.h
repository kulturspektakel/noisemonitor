#pragma once

// Logs free + largest-contiguous internal heap, with the DMA-capable subset
// broken out separately. WiFi RX buffers and NimBLE controller want DMA;
// fft_table and other DSP buffers don't — comparing the two columns tells you
// whether a DRAM shortfall is in the DMA-only or general region.
// Output is tagged "heap"; the label parameter prefixes the line.
void heap_diag(const char* label);
