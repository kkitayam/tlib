#include <string.h>
#include <strings.h>

#include "cpu.h"
#include "callbacks.h"
#include "arch_callbacks.h"

/* ------------------------------------------------------------------------- */
/* External interrupts                                                       */
/* ------------------------------------------------------------------------- */

static bool rx_any_interrupt_pending(const CPUState *env)
{
    int vector;

    if(env->nmi_pending) {
        return true;
    }
    for(vector = 0; vector < RX_INTERRUPT_VECTORS; vector++) {
        if(env->irq_pending[vector]) {
            return true;
        }
    }
    return false;
}

/* Keep CPU_INTERRUPT_HARD asserted while any request exists, even when it is
 * currently masked. The execution loop then re-arbitrates after every
 * instruction, so setting PSW.I or lowering PSW.IPL takes effect immediately. */
static void rx_refresh_interrupt_request(CPUState *env)
{
    if(rx_any_interrupt_pending(env)) {
        set_interrupt_pending(env, CPU_INTERRUPT_HARD);
    } else {
        clear_interrupt_pending(env, CPU_INTERRUPT_HARD);
    }
}

/* Highest-priority enabled vector that outranks the current PSW.IPL. Ties go
 * to the lower vector number. Returns -1 when nothing may be taken. */
static int rx_select_interrupt(CPUState *env, uint32_t *priority)
{
    uint32_t current_ipl;
    uint32_t best_priority = 0;
    int best = -1;
    int vector;

    /* The RXv2 manual only masks interrupts with PSW.I; PSW.PM does not stop
     * acceptance. An exception always switches to supervisor mode. */
    if(!(env->psw & PSW_I)) {
        return -1;
    }

    current_ipl = (env->psw & PSW_IPL_MASK) >> PSW_IPL_SHIFT;
    for(vector = 0; vector < RX_INTERRUPT_VECTORS; vector++) {
        uint32_t candidate;

        if(!env->irq_pending[vector]) {
            continue;
        }
        candidate = env->irq_priority[vector];
        if(candidate == 0 || candidate <= current_ipl) {
            continue;
        }
        if(best < 0 || candidate > best_priority ||
           (candidate == best_priority && vector < best)) {
            best = vector;
            best_priority = candidate;
        }
    }

    if(best >= 0) {
        *priority = best_priority;
    }
    return best;
}

bool rx_has_work(CPUState *env)
{
    uint32_t priority;

    if(env->nmi_pending) {
        return true;
    }
    return rx_select_interrupt(env, &priority) >= 0;
}

void rx_set_irq_pending(CPUState *env, uint32_t vector, bool pending)
{
    if(vector >= RX_INTERRUPT_VECTORS) {
        tlib_abortf("RX: interrupt vector %u is out of range", vector);
        return;
    }

    env->irq_pending[vector] = pending;
    rx_refresh_interrupt_request(env);
}

void rx_set_irq_priority(CPUState *env, uint32_t vector, uint32_t priority)
{
    if(vector >= RX_INTERRUPT_VECTORS) {
        tlib_abortf("RX: interrupt vector %u is out of range", vector);
        return;
    }

    /* PSW.IPL is four bits wide, so priorities are clamped to 0..15. */
    if(priority > 15) {
        priority = 15;
    }
    env->irq_priority[vector] = (uint8_t)priority;
}

void rx_set_nmi_pending(CPUState *env, bool pending)
{
    env->nmi_pending = pending;
    rx_refresh_interrupt_request(env);
}

/* Interrupt callbacks are paired with RTE. The stack is intentionally small:
 * nesting deeper than RX_INTERRUPT_STACK_SIZE still executes correctly, but the
 * begin/end hooks are suppressed for the frames that do not fit. */
static bool rx_push_interrupt_record(CPUState *env, uint32_t id)
{
    if(env->interrupt_depth >= RX_INTERRUPT_STACK_SIZE) {
        if(env->interrupt_depth != UINT32_MAX) {
            env->interrupt_depth++;
        }
        return false;
    }

    env->interrupt_stack[env->interrupt_depth] = id;
    env->interrupt_depth++;
    return true;
}

void rx_interrupt_return(CPUState *env)
{
    if(env->interrupt_depth == 0) {
        return;
    }

    env->interrupt_depth--;
    if(env->interrupt_depth < RX_INTERRUPT_STACK_SIZE && env->interrupt_end_callback_enabled) {
        tlib_on_interrupt_end(env->interrupt_stack[env->interrupt_depth]);
    }
}

/* Build the exception frame and enter the handler. new_psw is supplied by the
 * caller because interrupts update PSW.IPL while exceptions do not. */
static void rx_enter_interrupt(CPUState *env, uint32_t id, uint32_t handler, uint32_t new_psw)
{
    uint32_t old_psw = env->psw;
    bool record = env->interrupt_depth < RX_INTERRUPT_STACK_SIZE;

    if(record && env->interrupt_begin_callback_enabled) {
        tlib_on_interrupt_begin(id);
    }

    /* The frame always lives on the interrupt stack. Clearing PSW.U before the
     * pushes makes R0 alias ISP, matching the BRK and exception paths. */
    env->psw = old_psw & ~(PSW_I | PSW_U | PSW_PM);
    env->isp = env->isp - 4;
    stl_phys(env->isp, old_psw);
    env->isp = env->isp - 4;
    stl_phys(env->isp, env->pc);

    env->psw = new_psw;
    rx_push_interrupt_record(env, id);
    env->pc = handler;
    env->exception_index = -1;
    env->wfi = 0;
}

int process_interrupt(int interrupt_request, CPUState *env)
{
    uint32_t handler;
    uint32_t new_psw;
    uint32_t priority;
    int vector;

    if(!(interrupt_request & CPU_INTERRUPT_HARD) || tlib_is_in_debug_mode()) {
        return 0;
    }

    /* NMI ignores PSW.I and PSW.IPL and always uses the fixed reset vector.
     * The manual sets PSW.IPL to 15 on NMI entry. */
    if(env->nmi_pending) {
        env->nmi_pending = false;
        handler = ldl_phys(env->extb + RX_NMI_VECTOR_OFFSET);
        new_psw = (env->psw & ~(PSW_I | PSW_U | PSW_PM | PSW_IPL_MASK)) | PSW_IPL_MASK;
        rx_enter_interrupt(env, RX_NMI_VECTOR_ADDRESS, handler, new_psw);
        rx_refresh_interrupt_request(env);
        return 1;
    }

    vector = rx_select_interrupt(env, &priority);
    if(vector < 0) {
        /* Masked for now. CPU_INTERRUPT_HARD stays asserted so the request is
         * reconsidered after PSW changes. */
        return 0;
    }

    if(env->intb == 0) {
        tlib_abortf("RX: interrupt %d with no vector table (INTB=0)", vector);
        return 0;
    }

    handler = ldl_phys(env->intb + (uint32_t)vector * 4);
    new_psw = (env->psw & ~(PSW_I | PSW_U | PSW_PM | PSW_IPL_MASK)) |
              (priority << PSW_IPL_SHIFT);

    env->irq_pending[vector] = 0;
    rx_enter_interrupt(env, (uint32_t)vector, handler, new_psw);
    rx_refresh_interrupt_request(env);
    return 1;
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
    env->extb = RX_EXTB_RESET;
    env->bpsw = 0;
    env->bpc = 0;
    env->fintv = 0;
    env->fpsw = 0;
    env->acc0 = 0;
    env->acc1 = 0;
    env->cpen = 0;
    env->link_flag = false;
    env->wfi = 0;
    memset(env->irq_pending, 0, sizeof(env->irq_pending));
    memset(env->irq_priority, 0, sizeof(env->irq_priority));
    env->nmi_pending = false;
    memset(env->interrupt_stack, 0, sizeof(env->interrupt_stack));
    env->interrupt_depth = 0;
    clear_interrupt_pending(env, CPU_INTERRUPT_HARD);
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
    uint32_t handler;

    if(vector == EXCP_WFI) {
        /* WFI is reported to the execution loop by keeping the index. */
        return;
    }

    if(vector >= EXCP_INTERRUPT) {
        /* Execution loop requests are handled by cpu-exec itself. */
        return;
    }

    if(vector == EXCP_RX_ILLEGAL_INSTRUCTION) {
        handler = ldl_phys(env->extb + RX_EXCEPTION_UNDEFINED_OFFSET);
    } else if(vector == EXCP_RX_PRIVILEGED_INSTRUCTION) {
        handler = ldl_phys(env->extb + RX_EXCEPTION_PRIVILEGED_OFFSET);
    } else if(env->intb == 0) {
        tlib_abortf("RX: exception %u with no vector table (INTB=0) at PC=0x%08x", vector, env->pc);
        return;
    } else {
        handler = ldl_phys(env->intb + vector * 4);
    }

    /* Interrupt/exception entry: PSW.I, PSW.U and PSW.PM are cleared and the
     * old PSW and the return PC are pushed on the interrupt stack. The return
     * PC ends up at the lower address so that RTE can pop PC first. */
    rx_enter_interrupt(env, vector, handler, env->psw & ~(PSW_I | PSW_U | PSW_PM));
}
