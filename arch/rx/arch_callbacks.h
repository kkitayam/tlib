#pragma once

#include <stdint.h>

void arch_init(void);
typedef void (*rx_acknowledge_callback_t)(uint32_t vector, uint32_t fast, uint64_t generation);
void tlib_rx_acknowledge_interrupt(uint32_t vector, uint32_t fast, uint64_t generation);
void tlib_rx_set_acknowledge_callback(rx_acknowledge_callback_t callback);
uint32_t rx_require_acknowledge_callback(void);
