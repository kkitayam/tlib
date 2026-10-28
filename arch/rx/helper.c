#include <string.h>
#include <strings.h>

#include "cpu.h"
#include "callbacks.h"
#include "arch_callbacks.h"

/* External interrupt delivery. RX interrupt sources are modelled by the
 * platform; the test harness has none, so nothing is ever pending here. */
int process_interrupt(int interrupt_request, CPUState *env)
{
    (void)interrupt_request;
    (void)env;
    return 0;
}

void cpu_exec_prologue(CPUState *env) { (void)env; }

void cpu_exec_epilogue(CPUState *env) { (void)env; }

void rx_reset_state(CPUState *env)
{
    memset(env->regs, 0, sizeof(env->regs));
    env->pc = 0;
    env->psw = 0;
    env->usp = 0;
    env->isp = 0;
    env->intb = 0;
    env->bpsw = 0;
    env->bpc = 0;
    env->fintv = 0;
    env->fpsw = 0;
    env->acc0 = 0;
    env->acc1 = 0;
    env->cpen = 0;
    env->link_flag = false;
    env->wfi = 0;
    memset(env->banks, 0, sizeof(env->banks));
}

void cpu_reset(CPUState *env)
{
    rx_reset_state(env);
}

int cpu_init(const char *cpu_model)
{
    env->is_v3 = false;
    if(cpu_model == NULL || cpu_model[0] == '\0' || strcasecmp(cpu_model, "rxv2") == 0 ||
       strcasecmp(cpu_model, "rx") == 0 || strcasecmp(cpu_model, "rxv1") == 0) {
        env->is_v3 = false;
    } else if(strcasecmp(cpu_model, "rxv3") == 0) {
        env->is_v3 = true;
    } else {
        return -1;
    }

    cpu_reset(env);
    return 0;
}

void tlib_arch_dispose(void) { }

int cpu_handle_mmu_fault(CPUState *env, target_ulong address, int access_type, int mmu_idx, int no_page_fault,
                         target_phys_addr_t *paddr)
{
    (void)access_type;
    (void)no_page_fault;

    *paddr = address;
    tlb_set_page(env, address & TARGET_PAGE_MASK, address & TARGET_PAGE_MASK,
                 PAGE_READ | PAGE_WRITE | PAGE_EXEC, mmu_idx, TARGET_PAGE_SIZE);
    return TRANSLATE_SUCCESS;
}

target_phys_addr_t cpu_get_phys_page_debug(CPUState *env, target_ulong addr)
{
    (void)env;
    return addr;
}

uint64_t cpu_get_state_for_memory_transaction(CPUState *env, target_ulong addr, int access_type)
{
    (void)env;
    (void)addr;
    (void)access_type;
    return 0;
}

void do_interrupt(CPUState *env)
{
    uint32_t vector = (uint32_t)env->exception_index;
    uint32_t old_psw;

    if(vector == EXCP_WFI) {
        /* WFI is reported to the execution loop by keeping the index. */
        return;
    }

    if(vector >= EXCP_INTERRUPT) {
        /* Execution loop requests are handled by cpu-exec itself. */
        return;
    }

    if(env->interrupt_begin_callback_enabled) {
        tlib_on_interrupt_begin(vector);
    }

    if(env->intb == 0) {
        tlib_abortf("RX: exception %u with no vector table (INTB=0) at PC=0x%08x", vector, env->pc);
        return;
    }

    old_psw = env->psw;

    /* Interrupt/exception entry: PSW.I, PSW.U and PSW.PM are cleared and the
     * old PSW and the return PC are pushed on the interrupt stack. The return
     * PC ends up at the lower address so that RTE can pop PC first. */
    env->psw = old_psw & ~(PSW_I | PSW_U | PSW_PM);
    env->isp = env->isp - 4;
    stl_phys(env->isp, old_psw);
    env->isp = env->isp - 4;
    stl_phys(env->isp, env->pc);

    env->pc = ldl_phys(env->intb + vector * 4);
    env->exception_index = -1;
    env->wfi = 0;

    if(env->interrupt_end_callback_enabled) {
        tlib_on_interrupt_end(vector);
    }
}
