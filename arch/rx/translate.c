#include "cpu.h"
#include "arch_callbacks.h"
#include "tb-helper.h"

static TCGv cpu_pc, cpu_psw, cpu_usp, cpu_isp;
static TCGv cpu_regs[16];

void translate_init(void)
{
    static const char *const names[] = {
        "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
        "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15"
    };

    cpu_pc = tcg_global_mem_new(TCG_AREG0, offsetof(CPUState, pc), "pc");
    cpu_psw = tcg_global_mem_new(TCG_AREG0, offsetof(CPUState, psw), "psw");
    cpu_usp = tcg_global_mem_new(TCG_AREG0, offsetof(CPUState, usp), "usp");
    cpu_isp = tcg_global_mem_new(TCG_AREG0, offsetof(CPUState, isp), "isp");
    for(int i = 1; i < 16; i++) {
        cpu_regs[i] = tcg_global_mem_new(TCG_AREG0, offsetof(CPUState, regs[i]), names[i]);
    }
}

static void gen_read_reg(TCGv value, int reg)
{
    if(reg != 0) {
        tcg_gen_mov_tl(value, cpu_regs[reg]);
    } else {
        TCGv mode = tcg_temp_new();
        TCGv zero = tcg_const_tl(0);
        tcg_gen_andi_tl(mode, cpu_psw, PSW_U);
        tcg_gen_movcond_tl(TCG_COND_NE, value, mode, zero, cpu_usp, cpu_isp);
        tcg_temp_free(zero);
        tcg_temp_free(mode);
    }
}

static void gen_write_reg(int reg, TCGv value)
{
    if(reg != 0) {
        tcg_gen_mov_tl(cpu_regs[reg], value);
    } else {
        TCGv mode = tcg_temp_new();
        TCGv zero = tcg_const_tl(0);
        tcg_gen_andi_tl(mode, cpu_psw, PSW_U);
        tcg_gen_movcond_tl(TCG_COND_NE, cpu_usp, mode, zero, value, cpu_usp);
        tcg_gen_movcond_tl(TCG_COND_EQ, cpu_isp, mode, zero, value, cpu_isp);
        tcg_temp_free(zero);
        tcg_temp_free(mode);
    }
}

static void gen_sz_flags(TCGv result)
{
    TCGv flags = tcg_temp_new();
    TCGv temp = tcg_temp_new();
    tcg_gen_setcondi_tl(TCG_COND_EQ, flags, result, 0);
    tcg_gen_shli_tl(flags, flags, 1);
    tcg_gen_shri_tl(temp, result, 29);
    tcg_gen_andi_tl(temp, temp, PSW_SIGN);
    tcg_gen_or_tl(flags, flags, temp);
    tcg_gen_andi_tl(cpu_psw, cpu_psw, ~(PSW_ZERO | PSW_SIGN));
    tcg_gen_or_tl(cpu_psw, cpu_psw, flags);
    tcg_temp_free(temp);
    tcg_temp_free(flags);
}

static void gen_arithmetic_flags(TCGv dst, TCGv src, TCGv result, bool subtract)
{
    TCGv flags = tcg_temp_new();
    TCGv temp = tcg_temp_new();
    TCGv overflow = tcg_temp_new();

    tcg_gen_setcond_tl(subtract ? TCG_COND_GEU : TCG_COND_LTU,
                       flags, subtract ? dst : result, subtract ? src : dst);
    tcg_gen_xor_tl(overflow, dst, src);
    if(!subtract) {
        tcg_gen_not_tl(overflow, overflow);
    }
    tcg_gen_xor_tl(temp, dst, result);
    tcg_gen_and_tl(overflow, overflow, temp);
    tcg_gen_shri_tl(overflow, overflow, 28);
    tcg_gen_andi_tl(overflow, overflow, PSW_OVERFLOW);
    tcg_gen_or_tl(flags, flags, overflow);
    tcg_gen_andi_tl(cpu_psw, cpu_psw, ~(PSW_CARRY | PSW_OVERFLOW));
    tcg_gen_or_tl(cpu_psw, cpu_psw, flags);
    gen_sz_flags(result);
    tcg_temp_free(overflow);
    tcg_temp_free(temp);
    tcg_temp_free(flags);
}

static void gen_alu(int op, int reg, TCGv src)
{
    TCGv dst = tcg_temp_new();
    TCGv result = tcg_temp_new();
    gen_read_reg(dst, reg);
    switch(op) {
        case 0:
        case 1:
            tcg_gen_sub_tl(result, dst, src);
            gen_arithmetic_flags(dst, src, result, true);
            break;
        case 2:
            tcg_gen_add_tl(result, dst, src);
            gen_arithmetic_flags(dst, src, result, false);
            break;
        case 3:
            tcg_gen_mul_tl(result, dst, src);
            break;
        case 4:
            tcg_gen_and_tl(result, dst, src);
            gen_sz_flags(result);
            break;
        default:
            tcg_gen_or_tl(result, dst, src);
            gen_sz_flags(result);
            break;
    }
    if(op != 1) {
        gen_write_reg(reg, result);
    }
    tcg_temp_free(result);
    tcg_temp_free(dst);
}

static int gen_rx_insn(uint32_t pc)
{
    uint8_t opcode = ldub_code(pc);
    if(opcode == 0x03) {
        return 1;
    }
    if(opcode >= 0x60 && opcode <= 0x66) {
        uint8_t operands = ldub_code(pc + 1);
        TCGv src = tcg_const_tl(operands >> 4);
        if(opcode == 0x66) {
            gen_write_reg(operands & 15, src);
        } else {
            gen_alu(opcode - 0x60, operands & 15, src);
        }
        tcg_temp_free(src);
        return 2;
    }
    if(opcode >= 0x43 && opcode <= 0x57 && (opcode & 3) == 3) {
        uint8_t operands = ldub_code(pc + 1);
        TCGv src = tcg_temp_new();
        gen_read_reg(src, operands >> 4);
        gen_alu((opcode >> 2) & 7, operands & 15, src);
        tcg_temp_free(src);
        return 2;
    }
    if(opcode == 0xef) {
        uint8_t operands = ldub_code(pc + 1);
        TCGv src = tcg_temp_new();
        gen_read_reg(src, operands >> 4);
        gen_write_reg(operands & 15, src);
        tcg_temp_free(src);
        return 2;
    }
    return 0;
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
    tcg_gen_insn_start(base->pc);
    int length = gen_rx_insn(base->pc);
    if(length == 0) {
        length = rx_insn_length(env, base->pc);
        tcg_gen_movi_tl(cpu_pc, base->pc);
        TCGv_i32 pc = tcg_const_i32(base->pc);
        gen_helper_rx_exec(cpu_env, pc);
        tcg_temp_free_i32(pc);
        base->is_jmp = DISAS_JUMP;
    }

    base->tb->size += length;
    base->pc += length;

    if(env->singlestep_enabled) {
        if(base->is_jmp == DISAS_NEXT) {
            tcg_gen_movi_tl(cpu_pc, base->pc);
        }
        gen_helper_rx_debug(cpu_env);
        base->is_jmp = DISAS_JUMP;
    }

    return (base->pc - (base->tb->pc & TARGET_PAGE_MASK)) < TARGET_PAGE_SIZE;
}

void gen_intermediate_code_epilogue(CPUState *env, DisasContextBase *base)
{
    (void)env;
    if(base->is_jmp == DISAS_NEXT) {
        tcg_gen_movi_tl(cpu_pc, base->pc);
    }
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
