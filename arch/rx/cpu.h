#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>
#include <stddef.h>

#include "cpu_registers.h"

#define TARGET_PAGE_BITS            12
#define TARGET_PHYS_ADDR_SPACE_BITS 32
#define TARGET_VIRT_ADDR_SPACE_BITS 32

/*
 * RX has a single, identity mapped address space. Two MMU modes are kept so
 * that the generic softmmu helpers can still select a user/kernel index; both
 * modes map one-to-one.
 */
#define NB_MMU_MODES     2
#define MMU_USER_IDX     0
#define MMU_KERNEL_IDX   1
#define MMU_MODE0_SUFFIX _user
#define MMU_MODE1_SUFFIX _kernel

#define EXCP_RX_ILLEGAL_INSTRUCTION     4
#define EXCP_RX_PRIVILEGED_INSTRUCTION  5
#define EXCP_RX_FLOATING_POINT          6

/* PSW flags */
#define PSW_CARRY    0x00000001
#define PSW_ZERO     0x00000002
#define PSW_SIGN     0x00000004
#define PSW_OVERFLOW 0x00000008
#define PSW_I        0x00010000
#define PSW_U        0x00020000
#define PSW_PM       0x00100000
#define PSW_IPL_MASK 0x0F000000
#define PSW_IPL_SHIFT 24

static inline uint32_t rx_fpsw_normalize(uint32_t fpsw)
{
    fpsw &= 0x7c007dffu;
    return fpsw | ((fpsw & 0x3c000000u) ? 0x80000000u : 0);
}

enum {
    RX_FPU_SUB,
    RX_FPU_CMP,
    RX_FPU_ADD,
    RX_FPU_MUL,
    RX_FPU_DIV,
    RX_FPU_FTOI,
    RX_FPU_ROUND,
    RX_FPU_SQRT = 8,
    RX_FPU_FTOU,
    RX_FPU_ITOF,
    RX_FPU_UTOF,
};

enum {
    RX_DSP_SET,
    RX_DSP_ADD,
    RX_DSP_SUB,
};

#define RX_INTERRUPT_VECTORS     256
#define RX_IRQ_VALID             (1u << 12)
#define RX_IRQ_FAST              (1u << 13)
#define RX_IRQ_NMI               (1u << 14)
#define RX_IRQ_CANDIDATE_MASK    (RX_IRQ_NMI - 1)
/* Exception vectors live at EXTB + a fixed offset (RXv2 manual 1.7.1). EXTB
 * resets to 0xFFFFFF80, so NMI starts at 0xFFFFFFF8. */
#define RX_EXTB_RESET                    0xFFFFFF80u
#define RX_EXCEPTION_PRIVILEGED_OFFSET   0x50u
#define RX_EXCEPTION_UNDEFINED_OFFSET    0x5Cu
#define RX_EXCEPTION_FLOAT_OFFSET        0x64u
#define RX_NMI_VECTOR_OFFSET             0x78u
#define RX_NMI_VECTOR_ADDRESS            (RX_EXTB_RESET + RX_NMI_VECTOR_OFFSET)
#define RX_INTERRUPT_STACK_SIZE          32
#define RX_HOOK_EXCEPTION_BASE           0x100u
#define RX_HOOK_NMI                      0x200u
#define RX_HOOK_INT_BASE                 0x300u
#define RX_HOOK_BRK                      0x400u

typedef struct CPUState {
    uint32_t regs[16];
    uint32_t pc;
    uint32_t psw;
    uint32_t usp;
    uint32_t isp;
    uint32_t intb;
    uint32_t extb;
    uint32_t bpsw;
    uint32_t bpc;
    uint32_t fintv;
    uint32_t fpsw;

    /* Signed 72-bit accumulators. The low 64 bits are exposed through
     * tlib_rx_get_acc0/1, and the upper 8 bits through
     * tlib_rx_get_acc0_high/1 and the ACC0_HIGH/ACC1_HIGH registers. */
    __int128 acc0;
    __int128 acc1;

    uint32_t cpen;
    /* MOVLI/MOVCO link flag */
    bool link_flag;

    /* Candidate and notification flag are updated under interrupt_lock. */
    uint32_t interrupt_inputs;
    uint64_t interrupt_generation;

    /* Vectors of the exception frames currently on the stack. Used only to
     * pair the tlib interrupt begin/end callbacks, which RTE completes. */
    uint32_t interrupt_stack[RX_INTERRUPT_STACK_SIZE];
    uint32_t interrupt_depth;

    /* RXv3 SAVE/RSTR register banks (R1-R15, USP, FPSW, ACC0, ACC1). */
    struct {
        bool valid;
        uint32_t regs[15];
        uint32_t usp;
        uint32_t fpsw;
        __int128 acc0;
        __int128 acc1;
    } banks[16];

    bool is_v3;
    bool singlestep_enabled;

    CPU_COMMON
    bool store_stats_enabled;
    uint64_t store_check_count;
    uint64_t store_mmio_fast_count;
    uint64_t store_range_invalidation_count;
    uint64_t store_full_flush_count;
    pthread_mutex_t interrupt_lock;
} CPUState;

_Static_assert(offsetof(CPUState, interrupt_lock) > offsetof(CPUState, current_tb),
               "RX interrupt mutex must not be included in serialized CPU state");

static inline int cpu_mmu_index(CPUState *env)
{
    /* PSW.PM selects user/supervisor mode.  PSW.U only selects whether R0
     * aliases USP or ISP and must not affect the MMU mode. */
    return (env->psw & PSW_PM) ? MMU_USER_IDX : MMU_KERNEL_IDX;
}

/* PSW direct operand used by SETPSW/CLRPSW.  The defined selectors are
 * 0-3 (C, Z, S, O), 8 (I) and 9 (U).  Reserved selectors are undefined by
 * the ISA; keep the GNU RX simulator behaviour (everything else selects C)
 * so both execution paths stay identical. */
static inline uint32_t rx_psw_flag_bit(unsigned selector)
{
    static const uint8_t flag_bit[16] = {
        0, 1, 2, 3, 0, 0, 0, 0, 16, 17, 0, 0, 0, 0, 0, 0
    };

    return 1u << flag_bit[selector & 15];
}

typedef struct DisasContext DisasContext;

/* Larger than any instruction decoded by the RX interpreter. */
#define RX_DECODE_PAGE_GUARD 16

#include "cpu-all.h"
#include "exec-all.h"

struct DisasContext {
    DisasContextBase base;
    bool stop_after_instruction;
    bool is_v3;
    bool has_store;
    TCGv_i32 store_flags;
};

void rx_store_stats_reset(void);
uint64_t rx_store_stats_checks(void);
uint64_t rx_store_stats_mmio_fast(void);
uint64_t rx_store_stats_range_invalidations(void);
uint64_t rx_store_stats_full_flushes(void);

static inline void cpu_get_tb_cpu_state(CPUState *env, target_ulong *pc, target_ulong *cs_base, int *flags)
{
    *pc = env->pc;
    *cs_base = 0;
    *flags = cpu_mmu_index(env);
}

static inline void cpu_pc_from_tb(CPUState *env, TranslationBlock *tb)
{
    env->pc = tb->pc;
}

void rx_set_interrupt(CPUState *env, uint32_t vector, uint32_t level, bool fast, uint64_t generation);
void rx_clear_interrupt(CPUState *env);
void rx_set_nmi_pending(CPUState *env, bool pending);
bool rx_has_work(CPUState *env);
void rx_interrupt_begin(CPUState *env, uint32_t id);
void rx_interrupt_return(CPUState *env);

static inline bool cpu_has_work(CPUState *env)
{
    return !env->wfi || rx_has_work(env);
}

void rx_reset_state(CPUState *env);
int cpu_init(const char *cpu_model);
void cpu_reset(CPUState *env);
void tlib_arch_dispose(void);

int cpu_handle_mmu_fault(CPUState *env, target_ulong address, int access_type, int mmu_idx, int no_page_fault,
                         target_phys_addr_t *paddr);

/* Decode-only pass used by the translator to find the instruction length.
 * Returns the number of consumed bytes or -1 for an invalid instruction. */
int rx_insn_length(CPUState *env, uint32_t pc);

uint32_t rx_get_active_sp(CPUState *env);
void rx_set_active_sp(CPUState *env, uint32_t value);

uint64_t rx_get_acc0(CPUState *env);
uint64_t rx_get_acc1(CPUState *env);
void rx_set_acc0(CPUState *env, uint64_t value);
void rx_set_acc1(CPUState *env, uint64_t value);
uint32_t rx_get_acc0_high(CPUState *env);
uint32_t rx_get_acc1_high(CPUState *env);
void rx_set_acc0_high(CPUState *env, uint32_t value);
void rx_set_acc1_high(CPUState *env, uint32_t value);
uint32_t rx_get_extb(CPUState *env);
void rx_set_extb(CPUState *env, uint32_t value);
