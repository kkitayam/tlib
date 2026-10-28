#include "cpu.h"
#include "arch_callbacks.h"
#include "tb-helper.h"

/* Global register indexes */
static TCGv cpu_pc;

void translate_init(void)
{
    cpu_pc = tcg_global_mem_new(TCG_AREG0, offsetof(CPUState, pc), "pc");
}

void gen_sync_pc(DisasContext *dc)
{
    tcg_gen_movi_tl(cpu_pc, dc->base.pc);
}

void setup_disas_context(DisasContextBase *base, CPUState *env)
{
    (void)base;
    (void)env;
}

int gen_intermediate_code(CPUState *env, DisasContextBase *base)
{
    int length = rx_insn_length(env, base->pc);
    if(length <= 0) {
        length = 1;
    }

    tcg_gen_insn_start(base->pc);

    TCGv_i32 pc = tcg_const_i32(base->pc);
    gen_helper_rx_exec(cpu_env, pc);
    tcg_temp_free_i32(pc);

    base->tb->size += length;
    base->pc += length;

    if(env->singlestep_enabled) {
        gen_helper_rx_debug(cpu_env);
    }

    /* The helper always updates env->pc, including in the fall-through case,
     * so the block ends here and the execution loop re-enters at the new PC. */
    base->is_jmp = DISAS_JUMP;
    return 1;
}

void gen_intermediate_code_epilogue(CPUState *env, DisasContextBase *base)
{
    (void)env;
    gen_exit_tb_no_chaining(base->tb);
}

int gen_breakpoint(DisasContextBase *base, CPUBreakpoint *bp)
{
    DisasContext *dc = (DisasContext *)base;
    (void)bp;

    gen_sync_pc(dc);
    gen_helper_rx_debug(cpu_env);
    base->is_jmp = DISAS_JUMP;
    return 1;
}

void restore_state_to_opc(CPUState *env, TranslationBlock *tb, target_ulong *data)
{
    (void)tb;
    env->pc = data[0];
}
