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

void tlib_set_single_step(uint32_t enabled)
{
    cpu->singlestep_enabled = enabled;
}

EXC_VOID_1(tlib_set_single_step, uint32_t, enabled)
