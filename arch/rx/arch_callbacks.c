#include "callbacks.h"
#include "arch_callbacks.h"

DEFAULT_VOID_HANDLER0(void arch_init)

static rx_acknowledge_callback_t acknowledge_callback;

void tlib_rx_set_acknowledge_callback(rx_acknowledge_callback_t callback)
{
    acknowledge_callback = callback;
}

uint32_t rx_require_acknowledge_callback(void)
{
    if(!acknowledge_callback) {
        tlib_abortf("RX: no ICU acknowledgement callback is registered");
        return 0;
    }
    return 1;
}

void tlib_rx_acknowledge_interrupt(uint32_t vector, uint32_t fast, uint64_t generation)
{
    if(!rx_require_acknowledge_callback()) {
        return;
    }
    acknowledge_callback(vector, fast, generation);
}
