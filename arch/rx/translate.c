#include "cpu.h"
#include "arch_callbacks.h"
#include "tb-helper.h"

/* Larger than any instruction decoded by the RX interpreter. */
#define RX_DECODE_PAGE_GUARD 16

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

static void gen_alu_values(int op, TCGv dst, TCGv src, TCGv result)
{
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
        case 5:
            tcg_gen_or_tl(result, dst, src);
            gen_sz_flags(result);
            break;
        case 6:
            tcg_gen_xor_tl(result, dst, src);
            gen_sz_flags(result);
            break;
    }
}

static void gen_alu(int op, int reg, TCGv src)
{
    TCGv dst = tcg_temp_new();
    TCGv result = tcg_temp_new();
    gen_read_reg(dst, reg);
    gen_alu_values(op, dst, src, result);
    if(op != 1) {
        gen_write_reg(reg, result);
    }
    tcg_temp_free(result);
    tcg_temp_free(dst);
}

static void gen_shift_imm(int kind, int count, int src_reg, int dst_reg)
{
    TCGv src = tcg_temp_new();
    TCGv result = tcg_temp_new();
    TCGv flags = tcg_temp_new();
    gen_read_reg(src, src_reg);

    if(count == 0) {
        tcg_gen_movi_tl(flags, 0);
    } else {
        tcg_gen_shri_tl(flags, src, kind == 2 ? 32 - count : count - 1);
        tcg_gen_andi_tl(flags, flags, PSW_CARRY);
    }

    if(kind == 0) {
        tcg_gen_shri_tl(result, src, count);
    } else if(kind == 1) {
        tcg_gen_sari_tl(result, src, count);
    } else {
        tcg_gen_shli_tl(result, src, count);
        TCGv restored = tcg_temp_new();
        tcg_gen_sari_tl(restored, result, count);
        tcg_gen_setcond_tl(TCG_COND_NE, restored, restored, src);
        tcg_gen_shli_tl(restored, restored, 3);
        tcg_gen_or_tl(flags, flags, restored);
        tcg_temp_free(restored);
    }
    tcg_gen_andi_tl(cpu_psw, cpu_psw, ~(PSW_CARRY | (kind == 0 ? 0 : PSW_OVERFLOW)));
    tcg_gen_or_tl(cpu_psw, cpu_psw, flags);
    gen_sz_flags(result);
    gen_write_reg(dst_reg, result);
    tcg_temp_free(flags);
    tcg_temp_free(result);
    tcg_temp_free(src);
}

static void gen_shift_reg(int kind, int count_reg, int dst_reg)
{
    TCGv src = tcg_temp_new(), count = tcg_temp_new();
    TCGv result = tcg_temp_new(), flags = tcg_temp_new();
    TCGv shift = tcg_temp_new(), zero = tcg_const_tl(0);
    gen_read_reg(count, count_reg);
    gen_read_reg(src, dst_reg);
    tcg_gen_andi_tl(count, count, 31);
    if(kind == 2) {
        tcg_gen_neg_tl(shift, count);
    } else {
        tcg_gen_subi_tl(shift, count, 1);
    }
    tcg_gen_andi_tl(shift, shift, 31);
    tcg_gen_shr_tl(flags, src, shift);
    tcg_gen_andi_tl(flags, flags, PSW_CARRY);
    tcg_gen_movcond_tl(TCG_COND_EQ, flags, count, zero, zero, flags);
    if(kind == 0) {
        tcg_gen_shr_tl(result, src, count);
    } else if(kind == 1) {
        tcg_gen_sar_tl(result, src, count);
    } else {
        tcg_gen_shl_tl(result, src, count);
        tcg_gen_sar_tl(shift, result, count);
        tcg_gen_setcond_tl(TCG_COND_NE, shift, shift, src);
        tcg_gen_shli_tl(shift, shift, 3);
        tcg_gen_or_tl(flags, flags, shift);
    }
    tcg_gen_andi_tl(cpu_psw, cpu_psw, ~(PSW_CARRY | (kind == 0 ? 0 : PSW_OVERFLOW)));
    tcg_gen_or_tl(cpu_psw, cpu_psw, flags);
    gen_sz_flags(result);
    gen_write_reg(dst_reg, result);
    tcg_temp_free(zero);
    tcg_temp_free(shift);
    tcg_temp_free(flags);
    tcg_temp_free(result);
    tcg_temp_free(count);
    tcg_temp_free(src);
}

static void gen_rotate(int reg, TCGv count, bool left)
{
    TCGv src = tcg_temp_new(), result = tcg_temp_new(), carry = tcg_temp_new();
    TCGv zero = tcg_const_tl(0);
    gen_read_reg(src, reg);
    if(left) {
        tcg_gen_rotl_tl(result, src, count);
        tcg_gen_andi_tl(carry, result, 1);
    } else {
        tcg_gen_rotr_tl(result, src, count);
        tcg_gen_shri_tl(carry, result, 31);
    }
    tcg_gen_movcond_tl(TCG_COND_EQ, carry, count, zero, zero, carry);
    tcg_gen_andi_tl(cpu_psw, cpu_psw, ~PSW_CARRY);
    tcg_gen_or_tl(cpu_psw, cpu_psw, carry);
    gen_sz_flags(result);
    gen_write_reg(reg, result);
    tcg_temp_free(zero);
    tcg_temp_free(carry);
    tcg_temp_free(result);
    tcg_temp_free(src);
}

static void gen_unary(int op, int reg)
{
    TCGv src = tcg_temp_new(), result = tcg_temp_new(), zero = tcg_const_tl(0);
    gen_read_reg(src, reg);
    if(op == 1) {
        tcg_gen_neg_tl(result, src);
        gen_arithmetic_flags(zero, src, result, true);
    } else if(op == 2) {
        TCGv overflow = tcg_temp_new();
        tcg_gen_neg_tl(result, src);
        tcg_gen_movcond_tl(TCG_COND_LT, result, src, zero, result, src);
        tcg_gen_setcondi_tl(TCG_COND_EQ, overflow, src, INT32_MIN);
        tcg_gen_shli_tl(overflow, overflow, 3);
        tcg_gen_andi_tl(cpu_psw, cpu_psw, ~PSW_OVERFLOW);
        tcg_gen_or_tl(cpu_psw, cpu_psw, overflow);
        gen_sz_flags(result);
        tcg_temp_free(overflow);
    } else {
        TCGv sign = tcg_temp_new(), overflow = tcg_temp_new();
        TCGv max = tcg_const_tl(INT32_MAX), min = tcg_const_tl(INT32_MIN);
        tcg_gen_andi_tl(sign, cpu_psw, PSW_SIGN);
        tcg_gen_andi_tl(overflow, cpu_psw, PSW_OVERFLOW);
        tcg_gen_movcond_tl(TCG_COND_NE, result, sign, zero, max, min);
        tcg_gen_movcond_tl(TCG_COND_NE, result, overflow, zero, result, src);
        tcg_temp_free(min);
        tcg_temp_free(max);
        tcg_temp_free(overflow);
        tcg_temp_free(sign);
    }
    gen_write_reg(reg, result);
    tcg_temp_free(zero);
    tcg_temp_free(result);
    tcg_temp_free(src);
}

static void gen_bit_test(TCGv value)
{
    TCGv zero = tcg_temp_new();
    tcg_gen_setcondi_tl(TCG_COND_EQ, zero, value, 0);
    tcg_gen_setcondi_tl(TCG_COND_NE, value, value, 0);
    tcg_gen_shli_tl(zero, zero, 1);
    tcg_gen_or_tl(value, value, zero);
    tcg_gen_andi_tl(cpu_psw, cpu_psw, ~(PSW_CARRY | PSW_ZERO));
    tcg_gen_or_tl(cpu_psw, cpu_psw, value);
    tcg_temp_free(zero);
}

static void gen_bit_imm(int op, int bit, int reg)
{
    TCGv value = tcg_temp_new();
    uint32_t mask = 1u << bit;
    gen_read_reg(value, reg);
    if(op == 0) {
        tcg_gen_ori_tl(value, value, mask);
        gen_write_reg(reg, value);
    } else if(op == 1) {
        tcg_gen_andi_tl(value, value, ~mask);
        gen_write_reg(reg, value);
    } else {
        tcg_gen_andi_tl(value, value, mask);
        gen_bit_test(value);
    }
    tcg_temp_free(value);
}

static void gen_bit_reg(int op, int bit_reg, int dst_reg)
{
    TCGv bit = tcg_temp_new();
    TCGv mask = tcg_const_tl(1);
    TCGv value = tcg_temp_new();
    gen_read_reg(bit, bit_reg);
    gen_read_reg(value, dst_reg);
    tcg_gen_andi_tl(bit, bit, 31);
    tcg_gen_shl_tl(mask, mask, bit);
    if(op == 0) {
        tcg_gen_or_tl(value, value, mask);
    } else if(op == 1) {
        tcg_gen_not_tl(mask, mask);
        tcg_gen_and_tl(value, value, mask);
    } else if(op == 2) {
        tcg_gen_and_tl(value, value, mask);
        gen_bit_test(value);
    } else {
        tcg_gen_xor_tl(value, value, mask);
    }
    if(op != 2) {
        gen_write_reg(dst_reg, value);
    }
    tcg_temp_free(value);
    tcg_temp_free(mask);
    tcg_temp_free(bit);
}

static void gen_reg_logic(int op, int src_reg, int dst_reg)
{
    TCGv src = tcg_temp_new();
    TCGv result = tcg_temp_new();
    gen_read_reg(src, src_reg);
    if(op == 0x3b) {
        tcg_gen_not_tl(result, src);
    } else {
        gen_read_reg(result, dst_reg);
        if(op == 0x33) {
            tcg_gen_and_tl(result, result, src);
        } else {
            tcg_gen_xor_tl(result, result, src);
        }
    }
    gen_sz_flags(result);
    if(op != 0x33) {
        gen_write_reg(dst_reg, result);
    }
    tcg_temp_free(result);
    tcg_temp_free(src);
}

static void gen_condition(TCGv result, int cond)
{
    TCGv z = tcg_temp_new();
    TCGv s = tcg_temp_new();
    tcg_gen_shri_tl(z, cpu_psw, 1);
    tcg_gen_andi_tl(z, z, 1);
    tcg_gen_shri_tl(s, cpu_psw, 2);
    tcg_gen_andi_tl(s, s, 1);

    switch(cond) {
        case 0: case 1:
            tcg_gen_mov_tl(result, z);
            break;
        case 2: case 3: case 4: case 5:
            tcg_gen_andi_tl(result, cpu_psw, PSW_CARRY);
            if(cond >= 4) {
                tcg_gen_xori_tl(z, z, 1);
                tcg_gen_and_tl(result, result, z);
            }
            break;
        case 6: case 7:
            tcg_gen_mov_tl(result, s);
            break;
        case 8: case 9: case 10: case 11:
            tcg_gen_shri_tl(result, cpu_psw, 3);
            tcg_gen_andi_tl(result, result, 1);
            tcg_gen_xor_tl(result, result, s);
            if(cond >= 10) {
                tcg_gen_or_tl(result, result, z);
            }
            break;
        case 12: case 13:
            tcg_gen_shri_tl(result, cpu_psw, 3);
            tcg_gen_andi_tl(result, result, 1);
            break;
        default:
            tcg_gen_movi_tl(result, cond == 14);
            break;
    }
    if(cond == 1 || cond == 3 || cond == 5 || cond == 6 || cond == 8 || cond == 10 || cond == 13) {
        tcg_gen_xori_tl(result, result, 1);
    }
    tcg_temp_free(s);
    tcg_temp_free(z);
}

static void gen_bit_cond(int bit, int cond, int reg)
{
    TCGv value = tcg_temp_new();
    uint32_t mask = 1u << bit;
    gen_read_reg(value, reg);
    if(cond == 15) {
        tcg_gen_xori_tl(value, value, mask);
    } else {
        TCGv condition = tcg_temp_new(), zero = tcg_const_tl(0);
        TCGv set = tcg_temp_new(), clear = tcg_temp_new();
        gen_condition(condition, cond);
        tcg_gen_ori_tl(set, value, mask);
        tcg_gen_andi_tl(clear, value, ~mask);
        tcg_gen_movcond_tl(TCG_COND_NE, value, condition, zero, set, clear);
        tcg_temp_free(clear);
        tcg_temp_free(set);
        tcg_temp_free(zero);
        tcg_temp_free(condition);
    }
    gen_write_reg(reg, value);
    tcg_temp_free(value);
}

static int gen_branch(DisasContextBase *base, uint8_t opcode)
{
    static const int dsp3[] = { 8, 9, 10, 3, 4, 5, 6, 7 };
    uint32_t pc = base->pc;
    int32_t displacement;
    int length, cond = 14;
    if(opcode == 0x04) {
        uint32_t raw = ldub_code(pc + 1) | (ldub_code(pc + 2) << 8) | (ldub_code(pc + 3) << 16);
        displacement = (int32_t)(raw << 8) >> 8;
        length = 4;
    } else if(opcode >= 0x08 && opcode <= 0x1f) {
        displacement = dsp3[opcode & 7];
        length = 1;
        if(opcode >= 0x10) {
            cond = (opcode >> 3) & 1;
        }
    } else if(opcode >= 0x20 && opcode <= 0x2f) {
        displacement = (int8_t)ldub_code(pc + 1);
        cond = opcode & 15;
        length = 2;
    } else if(opcode == 0x38 || opcode == 0x3a || opcode == 0x3b) {
        uint32_t raw = ldub_code(pc + 1) | (ldub_code(pc + 2) << 8);
        displacement = (int16_t)raw;
        length = 3;
        if(opcode != 0x38) {
            cond = opcode - 0x3a;
        }
    } else {
        return 0;
    }

    if(cond == 14) {
        tcg_gen_movi_tl(cpu_pc, pc + (uint32_t)displacement);
    } else {
        TCGv condition = tcg_temp_new();
        TCGv zero = tcg_const_tl(0);
        TCGv target = tcg_const_tl(pc + (uint32_t)displacement);
        TCGv fallthrough = tcg_const_tl(pc + length);
        gen_condition(condition, cond);
        tcg_gen_movcond_tl(TCG_COND_NE, cpu_pc, condition, zero, target, fallthrough);
        tcg_temp_free(fallthrough);
        tcg_temp_free(target);
        tcg_temp_free(zero);
        tcg_temp_free(condition);
    }
    base->is_jmp = DISAS_JUMP;
    return length;
}

static void gen_load_mem(TCGv value, TCGv address, int size, bool is_signed, int mem_idx)
{
    TCGMemOp op = size == 1 ? (is_signed ? MO_SB : MO_UB) :
                  size == 2 ? (is_signed ? MO_LESW : MO_LEUW) : MO_LEUL;
    tcg_gen_qemu_ld_tl(value, address, mem_idx, op | MO_UNALN);
}

static void gen_store_mem(TCGv value, TCGv address, int size, int mem_idx)
{
    TCGMemOp op = size == 1 ? MO_UB : size == 2 ? MO_LEUW : MO_LEUL;
    tcg_gen_qemu_st_tl(value, address, mem_idx, op | MO_UNALN);
}

static uint32_t read_disp(uint32_t pc, int type, int size)
{
    if(type == 1) {
        return ldub_code(pc) * size;
    }
    if(type == 2) {
        return (ldub_code(pc) | (ldub_code(pc + 1) << 8)) * size;
    }
    return 0;
}

static uint32_t read_immediate(uint32_t pc, int im, int *length, bool sign_extend)
{
    int bytes = im == 0 ? 4 : im;
    uint32_t value = 0;
    for(int i = 0; i < bytes; i++) {
        value |= (uint32_t)ldub_code(pc + i) << (i * 8);
    }
    if(sign_extend && bytes < 4 && (value & (1u << (bytes * 8 - 1)))) {
        value |= UINT32_MAX << (bytes * 8);
    }
    *length += bytes;
    return value;
}

static void gen_address(TCGv address, int reg, uint32_t displacement)
{
    gen_read_reg(address, reg);
    if(displacement) {
        tcg_gen_addi_tl(address, address, displacement);
    }
}

static int gen_mov(DisasContextBase *base, uint8_t opcode)
{
    uint32_t pc = base->pc;
    uint8_t operands;
    TCGv address, value;
    if(opcode >= 0x80 && opcode <= 0xbf) {
        operands = ldub_code(pc + 1);
        int kind = (opcode >> 4) & 7;
        int displacement = ((opcode & 7) << 2) | ((operands >> 6) & 2) | ((operands >> 3) & 1);
        int base_reg = (operands >> 4) & 7;
        int reg = operands & 7;
        bool movu = kind == 3;
        int size = movu ? ((opcode & 8) ? 2 : 1) : 1 << kind;
        bool load = movu || (opcode & 8);
        address = tcg_temp_new();
        value = tcg_temp_new();
        tcg_gen_movi_tl(cpu_pc, pc);
        gen_address(address, base_reg, displacement * size);
        if(load) {
            gen_load_mem(value, address, size, !movu, base->mem_idx);
            gen_write_reg(reg, value);
        } else {
            gen_read_reg(value, reg);
            gen_store_mem(value, address, size, base->mem_idx);
            ((DisasContext *)base)->stop_after_instruction = true;
        }
        tcg_temp_free(value);
        tcg_temp_free(address);
        return 2;
    }
    if(opcode < 0xc0 || opcode > 0xef) {
        return 0;
    }

    operands = ldub_code(pc + 1);
    int size = 1 << ((opcode >> 4) & 3);
    int sd = (opcode >> 2) & 3, ss = opcode & 3;
    int src = operands >> 4, dst = operands & 15;
    int length = 2;
    uint32_t src_disp = 0, dst_disp = 0;
    if(ss == 3 && sd != 3) {
        dst_disp = read_disp(pc + length, sd, size);
        length += sd == 1 ? 1 : sd == 2 ? 2 : 0;
    } else {
        src_disp = read_disp(pc + length, ss, size);
        length += ss == 1 ? 1 : ss == 2 ? 2 : 0;
        dst_disp = read_disp(pc + length, sd, size);
        length += sd == 1 ? 1 : sd == 2 ? 2 : 0;
    }
    address = tcg_temp_new();
    value = tcg_temp_new();
    if(ss != 3 || sd != 3) {
        tcg_gen_movi_tl(cpu_pc, pc);
    }
    if(ss == 3 && sd != 3) {
        gen_read_reg(value, dst);
        gen_address(address, src, dst_disp);
        gen_store_mem(value, address, size, base->mem_idx);
        ((DisasContext *)base)->stop_after_instruction = true;
    } else {
        if(ss == 3) {
            gen_read_reg(value, src);
            if(size == 1) {
                tcg_gen_ext8s_tl(value, value);
            } else if(size == 2) {
                tcg_gen_ext16s_tl(value, value);
            }
        } else {
            gen_address(address, src, src_disp);
            gen_load_mem(value, address, size, true, base->mem_idx);
        }
        if(sd == 3) {
            gen_write_reg(dst, value);
        } else {
            gen_address(address, dst, dst_disp);
            gen_store_mem(value, address, size, base->mem_idx);
            ((DisasContext *)base)->stop_after_instruction = true;
        }
    }
    tcg_temp_free(value);
    tcg_temp_free(address);
    return length;
}

static int gen_imm_alu(uint32_t pc, uint8_t opcode)
{
    if(opcode < 0x70 || opcode > 0x77) {
        return 0;
    }
    uint8_t operands = ldub_code(pc + 1);
    int op = operands >> 4;
    int dst_reg = operands & 15;
    int length = 2;
    if(opcode >= 0x74 && op > 3 && !(opcode == 0x75 && (op == 4 || op == 5))) {
        return 0;
    }
    uint32_t value = read_immediate(pc + length, opcode & 3, &length,
                                    !(opcode == 0x75 && (op == 4 || op == 5)));
    TCGv src = tcg_const_tl(value);
    if(opcode < 0x74) {
        TCGv dst = tcg_temp_new(), result = tcg_temp_new();
        gen_read_reg(dst, operands >> 4);
        gen_alu_values(2, dst, src, result);
        gen_write_reg(dst_reg, result);
        tcg_temp_free(result);
        tcg_temp_free(dst);
    } else if(op == 4) {
        gen_write_reg(dst_reg, src);
    } else {
        gen_alu(op == 0 || op == 5 ? 1 : op + 2, dst_reg, src);
    }
    tcg_temp_free(src);
    return length;
}

static int gen_mov_imm(DisasContextBase *base, uint8_t opcode)
{
    if(opcode < 0xf8 || opcode > 0xfb) {
        return 0;
    }
    uint32_t pc = base->pc;
    uint8_t operands = ldub_code(pc + 1);
    int sd = opcode & 3, reg = operands >> 4;
    int im = (operands >> 2) & 3, sz = operands & 3;
    if(sz == 3) {
        return 0;
    }
    int size = 1 << sz, length = 2;
    uint32_t displacement = read_disp(pc + length, sd, size);
    length += sd == 1 ? 1 : sd == 2 ? 2 : 0;
    bool zero_extend = (im == 1 && sz == 0) || (im == 2 && sz == 1) || (im == 0 && sz == 2);
    uint32_t raw = read_immediate(pc + length, im, &length, !zero_extend);
    TCGv value = tcg_const_tl(raw);
    if(sd == 3) {
        TCGv result = tcg_temp_new();
        if(size == 1) {
            tcg_gen_ext8s_tl(result, value);
        } else if(size == 2) {
            tcg_gen_ext16s_tl(result, value);
        } else {
            tcg_gen_mov_tl(result, value);
        }
        gen_write_reg(reg, result);
        tcg_temp_free(result);
    } else {
        TCGv address = tcg_temp_new();
        tcg_gen_movi_tl(cpu_pc, pc);
        gen_address(address, reg, displacement);
        gen_store_mem(value, address, size, base->mem_idx);
        ((DisasContext *)base)->stop_after_instruction = true;
        tcg_temp_free(address);
    }
    tcg_temp_free(value);
    return length;
}

static int gen_memory_bit(DisasContextBase *base, uint8_t opcode)
{
    uint32_t pc = base->pc;
    int sd, op, reg, length;
    TCGv mask;

    if(opcode >= 0xf0 && opcode <= 0xf7) {
        uint8_t operands = ldub_code(pc + 1);
        sd = opcode & 3;
        if(sd == 3 || (opcode == 0xf4 && (operands & 15) >= 8 && (operands & 15) <= 10)) {
            return 0;
        }
        reg = operands >> 4;
        op = opcode >= 0xf4 ? 2 : (operands & 8 ? 1 : 0);
        mask = tcg_const_tl(1u << (operands & 7));
        length = 2;
    } else if(opcode == 0xfc) {
        uint8_t operation = ldub_code(pc + 1);
        sd = operation & 3;
        if((operation & 0xf0) != 0x60 || sd == 3) {
            return 0;
        }
        uint8_t operands = ldub_code(pc + 2);
        reg = operands >> 4;
        op = (operation >> 2) & 3;
        TCGv bit = tcg_temp_new();
        mask = tcg_const_tl(1);
        gen_read_reg(bit, operands & 15);
        tcg_gen_andi_tl(bit, bit, 7);
        tcg_gen_shl_tl(mask, mask, bit);
        tcg_temp_free(bit);
        length = 3;
    } else {
        return 0;
    }

    uint32_t displacement = read_disp(pc + length, sd, 1);
    length += sd == 1 ? 1 : sd == 2 ? 2 : 0;
    TCGv address = tcg_temp_new(), value = tcg_temp_new();
    tcg_gen_movi_tl(cpu_pc, pc);
    gen_address(address, reg, displacement);
    gen_load_mem(value, address, 1, false, base->mem_idx);
    if(op == 2) {
        tcg_gen_and_tl(value, value, mask);
        gen_bit_test(value);
    } else {
        if(op == 0) {
            tcg_gen_or_tl(value, value, mask);
        } else if(op == 1) {
            tcg_gen_not_tl(mask, mask);
            tcg_gen_and_tl(value, value, mask);
        } else {
            tcg_gen_xor_tl(value, value, mask);
        }
        gen_store_mem(value, address, 1, base->mem_idx);
        ((DisasContext *)base)->stop_after_instruction = true;
    }
    tcg_temp_free(value);
    tcg_temp_free(address);
    tcg_temp_free(mask);
    return length;
}

static int gen_alu_source(DisasContextBase *base, uint8_t opcode)
{
    if(opcode < 0x40 || opcode > 0x5f) {
        return 0;
    }
    uint32_t pc = base->pc;
    uint8_t operands = ldub_code(pc + 1);
    int op = (opcode >> 2) & 7, ss = opcode & 3;
    int src_reg = operands >> 4, dst_reg = operands & 15;
    int size = op >= 6 ? (op == 6 ? 1 : 2) : (ss == 3 ? 4 : 1);
    int length = 2;
    uint32_t displacement = read_disp(pc + length, ss, size);
    length += ss == 1 ? 1 : ss == 2 ? 2 : 0;
    TCGv src = tcg_temp_new();
    if(ss == 3) {
        gen_read_reg(src, src_reg);
        if(op == 6) {
            tcg_gen_ext8u_tl(src, src);
        } else if(op == 7) {
            tcg_gen_ext16u_tl(src, src);
        }
    } else {
        TCGv address = tcg_temp_new();
        tcg_gen_movi_tl(cpu_pc, pc);
        gen_address(address, src_reg, displacement);
        gen_load_mem(src, address, size, false, base->mem_idx);
        tcg_temp_free(address);
    }
    if(op >= 6) {
        gen_write_reg(dst_reg, src);
    } else {
        gen_alu(op, dst_reg, src);
    }
    tcg_temp_free(src);
    return length;
}

static int gen_indexed_mov(DisasContextBase *base, uint8_t opcode)
{
    if(opcode != 0xfe) {
        return 0;
    }
    uint32_t pc = base->pc;
    uint8_t b1 = ldub_code(pc + 1), b2 = ldub_code(pc + 2);
    int mode = b1 >> 6, sz = (b1 >> 4) & 3;
    if(sz == 3 || mode == 2) {
        return 0;
    }
    int size = 1 << sz;
    TCGv address = tcg_temp_new(), index = tcg_temp_new(), value = tcg_temp_new();
    tcg_gen_movi_tl(cpu_pc, pc);
    gen_read_reg(index, b1 & 15);
    gen_read_reg(address, b2 >> 4);
    if(sz != 0) {
        tcg_gen_shli_tl(index, index, sz);
    }
    tcg_gen_add_tl(address, address, index);
    if(mode == 0) {
        gen_read_reg(value, b2 & 15);
        gen_store_mem(value, address, size, base->mem_idx);
        ((DisasContext *)base)->stop_after_instruction = true;
    } else {
        gen_load_mem(value, address, size, mode == 1, base->mem_idx);
        gen_write_reg(b2 & 15, value);
    }
    tcg_temp_free(value);
    tcg_temp_free(index);
    tcg_temp_free(address);
    return 3;
}

static int gen_updated_mov(DisasContextBase *base, uint8_t opcode)
{
    if(opcode != 0xfd) {
        return 0;
    }
    uint32_t pc = base->pc;
    uint8_t operation = ldub_code(pc + 1);
    int sz;
    bool load, pre, is_signed = true;
    if(operation >= 0x20 && operation <= 0x22) {
        sz = operation - 0x20; load = false; pre = false;
    } else if(operation >= 0x24 && operation <= 0x26) {
        sz = operation - 0x24; load = false; pre = true;
    } else if(operation >= 0x28 && operation <= 0x2a) {
        sz = operation - 0x28; load = true; pre = false;
    } else if(operation >= 0x2c && operation <= 0x2e) {
        sz = operation - 0x2c; load = true; pre = true;
    } else if(operation >= 0x38 && operation <= 0x39) {
        sz = operation - 0x38; load = true; pre = false; is_signed = false;
    } else if(operation >= 0x3c && operation <= 0x3d) {
        sz = operation - 0x3c; load = true; pre = true; is_signed = false;
    } else {
        return 0;
    }
    int size = 1 << sz;
    uint8_t operands = ldub_code(pc + 2);
    int base_reg = operands >> 4, data_reg = operands & 15;
    TCGv address = tcg_temp_new(), value = tcg_temp_new();
    tcg_gen_movi_tl(cpu_pc, pc);
    gen_read_reg(address, base_reg);
    if(pre) {
        tcg_gen_subi_tl(address, address, size);
        gen_write_reg(base_reg, address);
    }
    if(load) {
        gen_load_mem(value, address, size, is_signed, base->mem_idx);
        gen_write_reg(data_reg, value);
    } else {
        gen_read_reg(value, data_reg);
        gen_store_mem(value, address, size, base->mem_idx);
        ((DisasContext *)base)->stop_after_instruction = true;
    }
    if(!pre) {
        tcg_gen_addi_tl(address, address, size);
        gen_write_reg(base_reg, address);
    }
    tcg_temp_free(value);
    tcg_temp_free(address);
    return 3;
}

static int gen_indirect_branch(DisasContextBase *base, uint8_t opcode)
{
    uint32_t pc = base->pc;
    int reg = -1, kind = -1, length = 0;
    int32_t displacement = 0;
    if(opcode == 0x02) {
        kind = 0; /* RTS */
        length = 1;
    } else if(opcode == 0x05 || opcode == 0x39) {
        length = opcode == 0x05 ? 4 : 3;
        uint32_t raw = ldub_code(pc + 1) | (ldub_code(pc + 2) << 8);
        if(opcode == 0x05) {
            raw |= ldub_code(pc + 3) << 16;
            displacement = (int32_t)(raw << 8) >> 8;
        } else {
            displacement = (int16_t)raw;
        }
        kind = 1; /* BSR */
    } else if(opcode == 0x7f) {
        uint8_t operands = ldub_code(pc + 1);
        reg = operands & 15;
        length = 2;
        if((operands & 0xf0) == 0x00) kind = 2;      /* JMP */
        else if((operands & 0xf0) == 0x10) kind = 3; /* JSR */
        else if((operands & 0xf0) == 0x40) kind = 4; /* BRA register */
        else if((operands & 0xf0) == 0x50) kind = 5; /* BSR register */
        else return 0;
    } else {
        return 0;
    }

    TCGv target = tcg_temp_new();
    if(kind == 0) {
        TCGv stack = tcg_temp_new();
        tcg_gen_movi_tl(cpu_pc, pc);
        gen_read_reg(stack, 0);
        gen_load_mem(target, stack, 4, false, base->mem_idx);
        tcg_gen_addi_tl(stack, stack, 4);
        gen_write_reg(0, stack);
        tcg_temp_free(stack);
    } else {
        if(kind == 1 || kind == 3 || kind == 5) {
            TCGv stack = tcg_temp_new();
            TCGv return_pc = tcg_const_tl(pc + length);
            tcg_gen_movi_tl(cpu_pc, pc);
            gen_read_reg(stack, 0);
            tcg_gen_subi_tl(stack, stack, 4);
            gen_write_reg(0, stack);
            gen_store_mem(return_pc, stack, 4, base->mem_idx);
            tcg_temp_free(return_pc);
            tcg_temp_free(stack);
        }
        if(kind == 1) {
            tcg_gen_movi_tl(target, pc + (uint32_t)displacement);
        } else {
            gen_read_reg(target, reg);
            if(kind == 4 || kind == 5) {
                tcg_gen_addi_tl(target, target, pc + (kind == 5 ? length : 0));
            }
        }
    }
    tcg_gen_mov_tl(cpu_pc, target);
    tcg_temp_free(target);
    base->is_jmp = DISAS_JUMP;
    return length;
}

static int gen_rx_insn(DisasContextBase *base)
{
    uint32_t pc = base->pc;
    uint8_t opcode = ldub_code(pc);
    int branch_length = gen_branch(base, opcode);
    if(branch_length != 0) {
        return branch_length;
    }
    branch_length = gen_indirect_branch(base, opcode);
    if(branch_length != 0) {
        return branch_length;
    }
    int mov_length = gen_mov(base, opcode);
    if(mov_length != 0) {
        return mov_length;
    }
    int imm_length = gen_imm_alu(pc, opcode);
    if(imm_length != 0) {
        return imm_length;
    }
    int alu_length = gen_alu_source(base, opcode);
    if(alu_length != 0) {
        return alu_length;
    }
    int indexed_length = gen_indexed_mov(base, opcode);
    if(indexed_length != 0) {
        return indexed_length;
    }
    int updated_length = gen_updated_mov(base, opcode);
    if(updated_length != 0) {
        return updated_length;
    }
    imm_length = gen_mov_imm(base, opcode);
    if(imm_length != 0) {
        return imm_length;
    }
    int bit_length = gen_memory_bit(base, opcode);
    if(bit_length != 0) {
        return bit_length;
    }
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
    if(opcode >= 0x68 && opcode <= 0x6d) {
        uint8_t operands = ldub_code(pc + 1);
        int count = ((opcode & 1) << 4) | (operands >> 4);
        gen_shift_imm((opcode - 0x68) / 2, count, operands & 15, operands & 15);
        return 2;
    }
    if(opcode >= 0x78 && opcode <= 0x7d) {
        uint8_t operands = ldub_code(pc + 1);
        int bit = ((opcode & 1) << 4) | (operands >> 4);
        gen_bit_imm((opcode - 0x78) / 2, bit, operands & 15);
        return 2;
    }
    if(opcode == 0x7e) {
        uint8_t operands = ldub_code(pc + 1);
        if((operands & 0xf0) == 0) {
            gen_reg_logic(0x3b, operands & 15, operands & 15);
            return 2;
        }
        if((operands >> 4) <= 3) {
            gen_unary(operands >> 4, operands & 15);
            return 2;
        }
    }
    if(opcode == 0xfc) {
        uint8_t operation = ldub_code(pc + 1);
        if(operation >= 0xe0 && (operation & 3) == 3) {
            uint8_t operands = ldub_code(pc + 2);
            gen_bit_cond((operation >> 2) & 7, operands & 15, operands >> 4);
            return 3;
        }
        if((operation & 0xf3) == 0x63) {
            uint8_t operands = ldub_code(pc + 2);
            gen_bit_reg((operation >> 2) & 3, operands & 15, operands >> 4);
            return 3;
        }
        if(operation == 0x33 || operation == 0x37 || operation == 0x3b) {
            uint8_t operands = ldub_code(pc + 2);
            gen_reg_logic(operation, operands >> 4, operands & 15);
            return 3;
        }
        if(operation == 0x43) {
            uint8_t operands = ldub_code(pc + 2);
            int src_reg = operands >> 4, dst_reg = operands & 15;
            TCGv src = tcg_temp_new(), dst = tcg_temp_new();
            gen_read_reg(src, src_reg);
            gen_read_reg(dst, dst_reg);
            gen_write_reg(dst_reg, src);
            gen_write_reg(src_reg, dst);
            tcg_temp_free(dst);
            tcg_temp_free(src);
            return 3;
        }
        if(operation == 0x13 || operation == 0x17) {
            uint8_t operands = ldub_code(pc + 2);
            TCGv src = tcg_temp_new(), dst = tcg_temp_new();
            gen_read_reg(src, operands >> 4);
            gen_read_reg(dst, operands & 15);
            tcg_gen_movcond_tl(operation == 0x13 ? TCG_COND_GT : TCG_COND_LT, dst, dst, src, dst, src);
            gen_write_reg(operands & 15, dst);
            tcg_temp_free(dst);
            tcg_temp_free(src);
            return 3;
        }
    }
    if(opcode == 0xfd) {
        uint8_t operation = ldub_code(pc + 1);
        if(operation >= 0x60 && operation <= 0x62) {
            uint8_t operands = ldub_code(pc + 2);
            gen_shift_reg(operation - 0x60, operands >> 4, operands & 15);
            return 3;
        }
        if(operation == 0x64 || operation == 0x66) {
            uint8_t operands = ldub_code(pc + 2);
            TCGv count = tcg_temp_new();
            gen_read_reg(count, operands >> 4);
            tcg_gen_andi_tl(count, count, 31);
            gen_rotate(operands & 15, count, operation == 0x66);
            tcg_temp_free(count);
            return 3;
        }
        if(operation >= 0x6c && operation <= 0x6f) {
            uint8_t operands = ldub_code(pc + 2);
            TCGv count = tcg_const_tl(((operation & 1) << 4) | (operands >> 4));
            gen_rotate(operands & 15, count, (operation & 2) != 0);
            tcg_temp_free(count);
            return 3;
        }
        if(operation == 0x65 || operation == 0x67) {
            uint8_t operands = ldub_code(pc + 2);
            TCGv src = tcg_temp_new(), result = tcg_temp_new();
            gen_read_reg(src, operands >> 4);
            if(operation == 0x67) {
                tcg_gen_bswap32_tl(result, src);
            } else {
                tcg_gen_shri_tl(result, src, 8);
                tcg_gen_andi_tl(result, result, 0x00ff00ffu);
                tcg_gen_shli_tl(src, src, 8);
                tcg_gen_andi_tl(src, src, 0xff00ff00u);
                tcg_gen_or_tl(result, result, src);
            }
            gen_write_reg(operands & 15, result);
            tcg_temp_free(result);
            tcg_temp_free(src);
            return 3;
        }
        if(operation >= 0x80 && operation <= 0xdf) {
            uint8_t operands = ldub_code(pc + 2);
            int count = ((operation >> 4) & 1) * 16 + (operation & 15);
            gen_shift_imm((operation - 0x80) / 0x20, count, operands >> 4, operands & 15);
            return 3;
        }
        if(operation >= 0xe0) {
            uint8_t operands = ldub_code(pc + 2);
            gen_bit_cond(operation & 31, operands >> 4, operands & 15);
            return 3;
        }
    }
    if(opcode == 0xff) {
        uint8_t operation = ldub_code(pc + 1);
        int op = operation >> 4;
        if(op == 0 || (op >= 2 && op <= 6)) {
            uint8_t operands = ldub_code(pc + 2);
            TCGv src = tcg_temp_new(), dst = tcg_temp_new(), result = tcg_temp_new();
            gen_read_reg(src, operands >> 4);
            gen_read_reg(dst, operands & 15);
            gen_alu_values(op, dst, src, result);
            gen_write_reg(operation & 15, result);
            tcg_temp_free(result);
            tcg_temp_free(dst);
            tcg_temp_free(src);
            return 3;
        }
    }
    return 0;
}

void gen_sync_pc(DisasContext *dc)
{
    tcg_gen_movi_tl(cpu_pc, dc->base.pc);
}

void setup_disas_context(DisasContextBase *base, CPUState *env)
{
    base->mem_idx = cpu_mmu_index(env);
}

int gen_intermediate_code(CPUState *env, DisasContextBase *base)
{
    tcg_gen_insn_start(base->pc);
    bool near_page_end = (base->pc & (TARGET_PAGE_SIZE - 1)) >= TARGET_PAGE_SIZE - RX_DECODE_PAGE_GUARD;
    int length = near_page_end ? 0 : gen_rx_insn(base);
    if(length == 0) {
        /* Decode near the boundary at runtime so a fetch fault cannot discard
         * earlier instructions or escape while no TB is running. */
        length = near_page_end ? 1 : rx_insn_length(env, base->pc);
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

    return !((DisasContext *)base)->stop_after_instruction &&
           (base->pc - (base->tb->pc & TARGET_PAGE_MASK)) < TARGET_PAGE_SIZE - RX_DECODE_PAGE_GUARD;
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
