#include <string.h>
#include <strings.h>

#include "cpu.h"
#include "callbacks.h"
#include "arch_callbacks.h"

void rx_store_stats_reset(void)
{
    cpu->store_stats_enabled = true;
    cpu->store_check_count = 0;
    cpu->store_mmio_fast_count = 0;
    cpu->store_range_invalidation_count = 0;
    cpu->store_full_flush_count = 0;
}

uint64_t rx_store_stats_checks(void)
{
    return cpu->store_check_count;
}

uint64_t rx_store_stats_mmio_fast(void)
{
    return cpu->store_mmio_fast_count;
}

uint64_t rx_store_stats_range_invalidations(void)
{
    return cpu->store_range_invalidation_count;
}

uint64_t rx_store_stats_full_flushes(void)
{
    return cpu->store_full_flush_count;
}

static bool rx_tb_is_cached(TranslationBlock *tb)
{
    unsigned hash = tb_phys_hash_func(tb->page_addr[0] + (tb->pc & ~TARGET_PAGE_MASK));
    TranslationBlock *cached = tb_phys_hash[hash];
    while(cached && cached != tb) {
        cached = cached->phys_hash_next;
    }
    return cached != NULL;
}

static __attribute__((always_inline)) inline bool
rx_store_overlaps_tb(CPUState *env, TranslationBlock *tb, tb_page_addr_t physical_page,
                     uint32_t offset, uint32_t length)
{
    uint32_t tb_offset = tb->pc & ~TARGET_PAGE_MASK;
    uint32_t tb_size = tb->size;

    if(tb_size == 1 && tb_offset >= TARGET_PAGE_SIZE - RX_DECODE_PAGE_GUARD) {
        tb_size = RX_DECODE_PAGE_GUARD;
    }

    uint32_t first_length = TARGET_PAGE_SIZE - tb_offset;
    if(first_length > tb_size) {
        first_length = tb_size;
    }
    if(physical_page == tb->page_addr[0] &&
       offset < tb_offset + first_length && offset + length > tb_offset) {
        return true;
    }

    uint32_t second_length = tb_size - first_length;
    if(second_length == 0) {
        return false;
    }

    tb_page_addr_t second_physical = tb->page_addr[1];
    if(second_physical == (tb_page_addr_t)-1) {
        target_ulong second_virtual = (tb->pc & TARGET_PAGE_MASK) + TARGET_PAGE_SIZE;
        second_physical = get_page_addr_code(env, second_virtual, false);
    }
    return second_physical != (tb_page_addr_t)-1 && physical_page == second_physical &&
           offset < second_length;
}

static bool rx_resolve_store_physical(target_ulong address, tb_page_addr_t *physical)
{
    target_ulong translated = virt_to_phys(address, ACCESS_DATA_STORE, 1);
    if(translated == (target_ulong)-1) {
        return false;
    }
    *physical = translated & TARGET_PAGE_MASK;
    return true;
}

static uint32_t rx_store_mmio_result(CPUState *env, TranslationBlock *tb)
{
    if(env->store_stats_enabled) {
        env->store_mmio_fast_count++;
    }
    return tb_invalidated_flag && !rx_tb_is_cached(tb);
}

/* helper_rx_store_check() runs immediately after a successful store.  If that
 * store invalidated its own TLB entry (one-shot mapping, or a host callback
 * that dropped the write mapping), the entry no longer matches, but the iotlb
 * slot still describes the page just accessed.  No other access can have
 * reused the slot in between, so the iotlb marker is still valid here. */
static __attribute__((always_inline)) inline bool
rx_tlb_had_nonexecutable_mmio(CPUState *env, uint32_t mmu_idx, unsigned index)
{
    target_phys_addr_t encoded = env->iotlb[mmu_idx][index] & ~TARGET_PAGE_MASK;
    return encoded > IO_MEM_ROM && encoded != IO_MEM_NOTDIRTY && !(encoded & IO_MEM_EXECUTABLE_IO);
}

uint32_t helper_rx_store_check(CPUState *env, TranslationBlock *tb, target_ulong address, uint32_t size, uint32_t mmu_idx)
{
    target_ulong page = address & TARGET_PAGE_MASK;
    unsigned index = (address >> TARGET_PAGE_BITS) & (CPU_TLB_SIZE - 1);
    target_ulong entry = env->tlb_table[mmu_idx][index].addr_write;
    if(env->store_stats_enabled) {
        env->store_check_count++;
    }

    bool entry_matches = (entry & (TARGET_PAGE_MASK | TLB_INVALID_MASK)) == page;
    bool nonexecutable_mmio = entry_matches && (entry & TLB_MMIO) && !(entry & IO_MEM_EXECUTABLE_IO);
    nonexecutable_mmio |= !entry_matches && rx_tlb_had_nonexecutable_mmio(env, mmu_idx, index);
    if(nonexecutable_mmio &&
       (address & ~TARGET_PAGE_MASK) <= TARGET_PAGE_SIZE - size) {
        return rx_store_mmio_result(env, tb);
    }

    /* Ordinary RAM stores already mark affected TBs dirty in the common path. */
    if(!external_mmu_enabled(env) && env->current_tb == tb && !env->tlib_is_on_memory_access_enabled &&
       !(entry & TLB_MMIO) && (entry & (TARGET_PAGE_MASK | TLB_INVALID_MASK)) == page &&
       (address & ~TARGET_PAGE_MASK) <= TARGET_PAGE_SIZE - size) {
        tb_page_addr_t physical = (page + env->iotlb[mmu_idx][index]) & TARGET_PAGE_MASK;
        return rx_store_overlaps_tb(env, tb, physical, address & ~TARGET_PAGE_MASK, size);
    }

    uint32_t flags = env->current_tb == tb ? 0 : 1;
    uint32_t remaining = size;

    while(remaining) {
        target_ulong page = address & TARGET_PAGE_MASK;
        uint32_t length = TARGET_PAGE_SIZE - (address & ~TARGET_PAGE_MASK);
        unsigned index = (address >> TARGET_PAGE_BITS) & (CPU_TLB_SIZE - 1);
        target_ulong entry = env->tlb_table[mmu_idx][index].addr_write;
        if(length > remaining) {
            length = remaining;
        }
        if((entry & (TARGET_PAGE_MASK | TLB_INVALID_MASK)) != page) {
            tb_page_addr_t physical;
            if(!rx_resolve_store_physical(address, &physical)) {
                flags |= 2;
                break;
            }
            if(rx_store_overlaps_tb(env, tb, physical, address & ~TARGET_PAGE_MASK, length)) {
                flags |= 1;
            }
            if(env->store_stats_enabled) {
                env->store_range_invalidation_count++;
            }
            tb_invalidate_phys_page_range_checked(physical + (address & ~TARGET_PAGE_MASK),
                                                  physical + (address & ~TARGET_PAGE_MASK) + length,
                                                  0, env->tb_broadcast_dirty);
        } else if((entry & TLB_MMIO) && !(entry & IO_MEM_EXECUTABLE_IO)) {
            flags |= rx_store_mmio_result(env, tb);
        } else {
            tb_page_addr_t physical = (page + env->iotlb[mmu_idx][index]) & TARGET_PAGE_MASK;
            if(rx_store_overlaps_tb(env, tb, physical, address & ~TARGET_PAGE_MASK, length)) {
                flags |= 1;
            }
            if(env->store_stats_enabled) {
                env->store_range_invalidation_count++;
            }
            tb_page_addr_t start = physical + (address & ~TARGET_PAGE_MASK);
            tb_invalidate_phys_page_range_checked(start, start + length, 0, env->tb_broadcast_dirty);
        }
        address += length;
        remaining -= length;
    }

    if(tb_invalidated_flag && !rx_tb_is_cached(tb)) {
        flags |= 1;
    }
    return flags;
}

void helper_rx_store_flush(CPUState *env)
{
    if(env->store_stats_enabled) {
        env->store_full_flush_count++;
    }
    tb_flush(env);
    if(env->tb_broadcast_dirty) {
#if UINTPTR_MAX > UINT32_MAX
        tlib_invalidate_tb_in_other_cpus(0, (uintptr_t)UINT32_MAX + 1);
#else
        for(uint64_t page = 0; page <= UINT32_MAX; page += TARGET_PAGE_SIZE) {
            append_dirty_address(page);
        }
        flush_dirty_addresses_list();
#endif
    }
}

/* ------------------------------------------------------------------------- */
/* External interrupts                                                       */
/* ------------------------------------------------------------------------- */

static void rx_refresh_interrupt_request(CPUState *env)
{
    if(env->interrupt_inputs & (RX_IRQ_VALID | RX_IRQ_NMI)) {
        set_interrupt_pending(env, CPU_INTERRUPT_HARD);
    } else {
        clear_interrupt_pending(env, CPU_INTERRUPT_HARD);
    }
}

static int rx_select_interrupt(CPUState *env, uint32_t inputs, uint32_t *priority, bool *fast)
{
    if(!(inputs & RX_IRQ_VALID) || !(env->psw & PSW_I)) {
        return -1;
    }

    *fast = (inputs & RX_IRQ_FAST) != 0;
    *priority = *fast ? 15 : (inputs >> 8) & 15;
    if(*priority <= ((env->psw & PSW_IPL_MASK) >> PSW_IPL_SHIFT)) {
        return -1;
    }
    return inputs & 0xff;
}

bool rx_has_work(CPUState *env)
{
    uint32_t priority;
    bool fast;
    uint32_t inputs;

    pthread_mutex_lock(&env->interrupt_lock);
    inputs = env->interrupt_inputs;
    pthread_mutex_unlock(&env->interrupt_lock);
    if(inputs & RX_IRQ_NMI) {
        return true;
    }
    return rx_select_interrupt(env, inputs, &priority, &fast) >= 0;
}

void rx_set_interrupt(CPUState *env, uint32_t vector, uint32_t level, bool fast, uint64_t generation)
{
    pthread_mutex_lock(&env->interrupt_lock);
    env->interrupt_inputs = (env->interrupt_inputs & ~RX_IRQ_CANDIDATE_MASK) |
                            RX_IRQ_VALID | vector | (level << 8) | (fast ? RX_IRQ_FAST : 0);
    env->interrupt_generation = generation;
    rx_refresh_interrupt_request(env);
    pthread_mutex_unlock(&env->interrupt_lock);
}

void rx_clear_interrupt(CPUState *env)
{
    pthread_mutex_lock(&env->interrupt_lock);
    env->interrupt_inputs &= ~RX_IRQ_CANDIDATE_MASK;
    rx_refresh_interrupt_request(env);
    pthread_mutex_unlock(&env->interrupt_lock);
}

void rx_set_nmi_pending(CPUState *env, bool pending)
{
    pthread_mutex_lock(&env->interrupt_lock);
    if(pending) {
        env->interrupt_inputs |= RX_IRQ_NMI;
    } else {
        env->interrupt_inputs &= ~RX_IRQ_NMI;
    }
    rx_refresh_interrupt_request(env);
    pthread_mutex_unlock(&env->interrupt_lock);
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

void rx_interrupt_begin(CPUState *env, uint32_t id)
{
    bool record = rx_push_interrupt_record(env, id);
    if(record && env->interrupt_begin_callback_enabled) {
        tlib_on_interrupt_begin(id);
    }
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
    stl_phys(env->isp - 4, old_psw);
    stl_phys(env->isp - 8, env->pc);
    env->isp -= 8;
    env->psw = new_psw;
    env->pc = handler;
    env->exception_index = -1;
    env->wfi = 0;
    /* The begin hook observes the completed entry state (PC, PSW, ISP). */
    rx_interrupt_begin(env, id);
}

/* Fast interrupt entry (RXv2 manual 5.5.7): PC and PSW go to BPC and BPSW,
 * nothing is pushed on the stack, and the handler address comes from FINTV. */
static void rx_enter_fast_interrupt(CPUState *env, uint32_t id, uint32_t handler, uint32_t new_psw)
{
    uint32_t old_psw = env->psw;

    env->bpsw = old_psw;
    env->bpc = env->pc;
    env->psw = new_psw;
    env->pc = handler;
    env->exception_index = -1;
    env->wfi = 0;
    /* The begin hook observes the completed entry state (BPC, BPSW, PC). */
    rx_interrupt_begin(env, id);
}

int process_interrupt(int interrupt_request, CPUState *env)
{
    uint32_t handler;
    uint32_t new_psw;
    uint32_t priority;
    uint32_t inputs;
    uint64_t generation;
    bool fast;
    bool nmi;
    int vector;

    if(!(interrupt_request & CPU_INTERRUPT_HARD) || tlib_is_in_debug_mode()) {
        return 0;
    }

    /* NMI ignores PSW.I and PSW.IPL and always uses the fixed reset vector.
     * The manual sets PSW.IPL to 15 on NMI entry. NMI is consumed under the
     * same lock as the candidate snapshot so a maskable request cannot be
     * mistaken for it. */
    pthread_mutex_lock(&env->interrupt_lock);
    inputs = env->interrupt_inputs;
    generation = env->interrupt_generation;
    nmi = (inputs & RX_IRQ_NMI) != 0;
    if(nmi) {
        env->interrupt_inputs &= ~RX_IRQ_NMI;
        rx_refresh_interrupt_request(env);
        pthread_mutex_unlock(&env->interrupt_lock);
        handler = ldl_phys(env->extb + RX_NMI_VECTOR_OFFSET);
        new_psw = (env->psw & ~(PSW_I | PSW_U | PSW_PM | PSW_IPL_MASK)) | PSW_IPL_MASK;
        rx_enter_interrupt(env, RX_HOOK_NMI, handler, new_psw);
        return 1;
    }

    vector = rx_select_interrupt(env, inputs, &priority, &fast);
    if(vector < 0) {
        /* Masked for now. CPU_INTERRUPT_HARD stays asserted so the request is
         * reconsidered after PSW changes. */
        pthread_mutex_unlock(&env->interrupt_lock);
        return 0;
    }
    if(!rx_require_acknowledge_callback()) {
        pthread_mutex_unlock(&env->interrupt_lock);
        return 0;
    }

    /* Select and consume the candidate in one critical section. ICU updates
     * take the same lock, so a request published after the snapshot cannot be
     * mistaken for the one being accepted. If the ICU withdraws the request
     * after this point, the interrupt is still accepted; the generation sent
     * with the acknowledgement lets the ICU detect that case. */
    if(env->interrupt_generation == generation) {
        env->interrupt_inputs &= ~RX_IRQ_CANDIDATE_MASK;
        rx_refresh_interrupt_request(env);
    }
    pthread_mutex_unlock(&env->interrupt_lock);

    if(fast) {
        new_psw = (env->psw & ~(PSW_I | PSW_U | PSW_PM | PSW_IPL_MASK)) |
                  (priority << PSW_IPL_SHIFT);
        rx_enter_fast_interrupt(env, (uint32_t)vector, env->fintv, new_psw);
        tlib_rx_acknowledge_interrupt((uint32_t)vector, 1, generation);
        return 1;
    }

    handler = ldl_phys(env->intb + (uint32_t)vector * 4);
    new_psw = (env->psw & ~(PSW_I | PSW_U | PSW_PM | PSW_IPL_MASK)) |
              (priority << PSW_IPL_SHIFT);

    rx_enter_interrupt(env, (uint32_t)vector, handler, new_psw);
    tlib_rx_acknowledge_interrupt((uint32_t)vector, 0, generation);
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
    env->fpsw = 0x100;
    env->acc0 = 0;
    env->acc1 = 0;
    env->cpen = 0;
    env->link_flag = false;
    env->wfi = 0;
    pthread_mutex_lock(&env->interrupt_lock);
    env->interrupt_inputs = 0;
    env->interrupt_generation = 0;
    rx_refresh_interrupt_request(env);
    pthread_mutex_unlock(&env->interrupt_lock);
    memset(env->interrupt_stack, 0, sizeof(env->interrupt_stack));
    env->interrupt_depth = 0;
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

    if(pthread_mutex_init(&env->interrupt_lock, NULL) != 0) {
        return -1;
    }
    cpu_reset(env);
    return 0;
}

void tlib_arch_dispose(void)
{
    tlib_rx_set_acknowledge_callback(NULL);
    pthread_mutex_destroy(&env->interrupt_lock);
}

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
    } else if(vector == EXCP_RX_FLOATING_POINT) {
        handler = ldl_phys(env->extb + RX_EXCEPTION_FLOAT_OFFSET);
    } else {
        handler = ldl_phys(env->intb + vector * 4);
    }

    /* Interrupt/exception entry: PSW.I, PSW.U and PSW.PM are cleared and the
     * old PSW and the return PC are pushed on the interrupt stack. The return
     * PC ends up at the lower address so that RTE can pop PC first. */
    rx_enter_interrupt(env, RX_HOOK_EXCEPTION_BASE + vector, handler, env->psw & ~(PSW_I | PSW_U | PSW_PM));
}
