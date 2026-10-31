#pragma once

#include <stdint.h>

uint64_t tlib_rx_get_acc0(void);
uint64_t tlib_rx_get_acc1(void);
void tlib_rx_set_acc0(uint64_t value);
void tlib_rx_set_acc1(uint64_t value);

void tlib_rx_set_interrupt_pending(uint32_t vector, uint32_t pending);
void tlib_rx_set_interrupt_priority(uint32_t vector, uint32_t priority);
void tlib_rx_set_nmi_pending(uint32_t pending);
