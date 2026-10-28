#pragma once

#include <stdbool.h>
#include <stdint.h>

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

#define EXCP_RX_ILLEGAL_INSTRUCTION 4

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

typedef struct CPUState {
    uint32_t regs[16];
    uint32_t pc;
    uint32_t psw;
    uint32_t usp;
    uint32_t isp;
    uint32_t intb;
    uint32_t bpsw;
    uint32_t bpc;
    uint32_t fintv;
    uint32_t fpsw;

    /* Signed 72-bit accumulators. Only the low 64 bits are exposed through the
     * generic register interface; see tlib_rx_get_acc0/1. */
    __int128 acc0;
    __int128 acc1;

    uint32_t cpen;
    /* MOVLI/MOVCO link flag */
    bool link_flag;

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
} CPUState;

static inline int cpu_mmu_index(CPUState *env)
{
    return (env->psw & PSW_U) ? MMU_USER_IDX : MMU_KERNEL_IDX;
}

typedef struct DisasContext DisasContext;

#include "cpu-all.h"
#include "exec-all.h"

struct DisasContext {
    DisasContextBase base;
};

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

static inline bool cpu_has_work(CPUState *env)
{
    return !env->wfi;
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
