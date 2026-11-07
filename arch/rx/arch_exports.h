#pragma once

#include <stdint.h>
#include "arch_callbacks.h"

uint64_t tlib_rx_get_acc0(void);
uint64_t tlib_rx_get_acc1(void);
void tlib_rx_set_acc0(uint64_t value);
void tlib_rx_set_acc1(uint64_t value);
/* Upper 8 bits (bits 64..71) of the 72-bit accumulators. */
uint32_t tlib_rx_get_acc0_high(void);
uint32_t tlib_rx_get_acc1_high(void);
void tlib_rx_set_acc0_high(uint32_t value);
void tlib_rx_set_acc1_high(uint32_t value);
/* Exception vector base register (control register CR 0xD). */
uint32_t tlib_rx_get_extb(void);
void tlib_rx_set_extb(uint32_t value);

/* The ICU publishes its highest candidate. generation identifies the source
 * request, not the publication: a withdrawn and reasserted source needs a
 * new generation. vector is 0..255, level is 1..15, fast is 0 or 1. */
void tlib_rx_set_interrupt(uint32_t vector, uint32_t level, uint32_t fast, uint64_t generation);
void tlib_rx_clear_interrupt(void);
void tlib_rx_set_nmi_pending(uint32_t pending);
/* Register or unregister only while the CPU is stopped. The host retains the
 * callback until all executions have stopped. The callback must acknowledge
 * only the matching source generation and publish the next ICU candidate. */
void tlib_rx_set_interrupt_acknowledge_callback(rx_acknowledge_callback_t callback);
