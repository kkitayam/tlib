#include "arch_exports.h"
#include "cpu.h"
#include "unwind.h"

uint64_t tlib_rx_get_acc0(void)
{
    return rx_get_acc0(cpu);
}

EXC_INT_0(uint64_t, tlib_rx_get_acc0)

uint64_t tlib_rx_get_acc1(void)
{
    return rx_get_acc1(cpu);
}

EXC_INT_0(uint64_t, tlib_rx_get_acc1)

void tlib_rx_set_acc0(uint64_t value)
{
    rx_set_acc0(cpu, value);
}

EXC_VOID_1(tlib_rx_set_acc0, uint64_t, value)

void tlib_rx_set_acc1(uint64_t value)
{
    rx_set_acc1(cpu, value);
}

EXC_VOID_1(tlib_rx_set_acc1, uint64_t, value)

uint32_t tlib_rx_get_acc0_high(void)
{
    return rx_get_acc0_high(cpu);
}

EXC_INT_0(uint32_t, tlib_rx_get_acc0_high)

uint32_t tlib_rx_get_acc1_high(void)
{
    return rx_get_acc1_high(cpu);
}

EXC_INT_0(uint32_t, tlib_rx_get_acc1_high)

void tlib_rx_set_acc0_high(uint32_t value)
{
    rx_set_acc0_high(cpu, value);
}

EXC_VOID_1(tlib_rx_set_acc0_high, uint32_t, value)

void tlib_rx_set_acc1_high(uint32_t value)
{
    rx_set_acc1_high(cpu, value);
}

EXC_VOID_1(tlib_rx_set_acc1_high, uint32_t, value)

uint32_t tlib_rx_get_extb(void)
{
    return rx_get_extb(cpu);
}

EXC_INT_0(uint32_t, tlib_rx_get_extb)

void tlib_rx_set_extb(uint32_t value)
{
    rx_set_extb(cpu, value);
}

EXC_VOID_1(tlib_rx_set_extb, uint32_t, value)

void tlib_rx_set_interrupt(uint32_t vector, uint32_t level, uint32_t fast, uint64_t generation)
{
    if(vector >= RX_INTERRUPT_VECTORS || level == 0 || level > 15 || fast > 1) {
        tlib_abortf("RX: invalid interrupt candidate: vector=%u level=%u fast=%u", vector, level, fast);
        return;
    }
    rx_set_interrupt(cpu, vector, level, fast != 0, generation);
}

EXC_VOID_4(tlib_rx_set_interrupt, uint32_t, vector, uint32_t, level, uint32_t, fast, uint64_t, generation)

void tlib_rx_set_interrupt_acknowledge_callback(rx_acknowledge_callback_t callback)
{
    tlib_rx_set_acknowledge_callback(callback);
}

EXC_VOID_1(tlib_rx_set_interrupt_acknowledge_callback, rx_acknowledge_callback_t, callback)

void tlib_rx_clear_interrupt(void)
{
    rx_clear_interrupt(cpu);
}

EXC_VOID_0(tlib_rx_clear_interrupt)

void tlib_rx_set_nmi_pending(uint32_t pending)
{
    rx_set_nmi_pending(cpu, pending != 0);
}

EXC_VOID_1(tlib_rx_set_nmi_pending, uint32_t, pending)

void tlib_set_single_step(uint32_t enabled)
{
    cpu->singlestep_enabled = enabled;
}

EXC_VOID_1(tlib_set_single_step, uint32_t, enabled)
