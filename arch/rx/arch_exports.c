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

/* External interrupt inputs. The platform supplies the vector table through
 * INTB; the priority of a vector is programmed separately, as in the RX ICU. */
void tlib_rx_set_interrupt_pending(uint32_t vector, uint32_t pending)
{
    rx_set_irq_pending(cpu, vector, pending != 0);
}

EXC_VOID_2(tlib_rx_set_interrupt_pending, uint32_t, vector, uint32_t, pending)

void tlib_rx_set_interrupt_priority(uint32_t vector, uint32_t priority)
{
    rx_set_irq_priority(cpu, vector, priority);
}

EXC_VOID_2(tlib_rx_set_interrupt_priority, uint32_t, vector, uint32_t, priority)

void tlib_rx_set_nmi_pending(uint32_t pending)
{
    rx_set_nmi_pending(cpu, pending != 0);
}

EXC_VOID_1(tlib_rx_set_nmi_pending, uint32_t, pending)

/* Selects the ICU fast-interrupt source (FIR.FVCT/FIEN). The handler address
 * itself is programmed by guest software through FINTV. */
void tlib_rx_set_fast_interrupt_vector(uint32_t vector, uint32_t enabled)
{
    rx_set_fast_interrupt_vector(cpu, vector, enabled != 0);
}

EXC_VOID_2(tlib_rx_set_fast_interrupt_vector, uint32_t, vector, uint32_t, enabled)

void tlib_set_single_step(uint32_t enabled)
{
    cpu->singlestep_enabled = enabled;
}

EXC_VOID_1(tlib_set_single_step, uint32_t, enabled)
