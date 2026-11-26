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
    tcg_gen_extract_i32(temp, result, 31, 1);
    tcg_gen_shli_tl(temp, temp, 2);
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
    if(subtract) {
        tcg_gen_xor_tl(overflow, dst, src);
    } else {
        tcg_gen_eqv_tl(overflow, dst, src);
    }
    tcg_gen_xor_tl(temp, dst, result);
    tcg_gen_and_tl(overflow, overflow, temp);
    tcg_gen_extract_i32(overflow, overflow, 31, 1);
    tcg_gen_shli_tl(overflow, overflow, 3);
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

static void gen_rotate_carry(int reg, bool left)
{
    TCGv value = tcg_temp_new(), carry = tcg_temp_new(), old_carry = tcg_temp_new();
    gen_read_reg(value, reg);
    tcg_gen_extract_i32(old_carry, cpu_psw, 0, 1);
    if(left) {
        tcg_gen_extract_i32(carry, value, 31, 1);
        tcg_gen_shli_tl(value, value, 1);
    } else {
        tcg_gen_extract_i32(carry, value, 0, 1);
        tcg_gen_shri_tl(value, value, 1);
        tcg_gen_deposit_z_i32(old_carry, old_carry, 31, 1);
    }
    tcg_gen_or_tl(value, value, old_carry);
    tcg_gen_andi_tl(cpu_psw, cpu_psw, ~PSW_CARRY);
    tcg_gen_or_tl(cpu_psw, cpu_psw, carry);
    gen_sz_flags(value);
    gen_write_reg(reg, value);
    tcg_temp_free(old_carry);
    tcg_temp_free(carry);
    tcg_temp_free(value);
}

static void gen_unary(int op, int src_reg, int dst_reg)
{
    TCGv src = tcg_temp_new(), result = tcg_temp_new(), zero = tcg_const_tl(0);
    gen_read_reg(src, src_reg);
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
    gen_write_reg(dst_reg, result);
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
    gen_read_reg(value, reg);
    if(op == 0) {
        TCGv mask = tcg_const_tl(1u << bit);
        tcg_gen_or_tl(value, value, mask);
        tcg_temp_free(mask);
        gen_write_reg(reg, value);
    } else if(op == 1) {
        TCGv mask = tcg_const_tl(1u << bit);
        tcg_gen_andc_tl(value, value, mask);
        tcg_temp_free(mask);
        gen_write_reg(reg, value);
    } else {
        tcg_gen_extract_i32(value, value, bit, 1);
        gen_bit_test(value);
    }
    tcg_temp_free(value);
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
    tcg_gen_extract_i32(z, cpu_psw, 1, 1);
    tcg_gen_extract_i32(s, cpu_psw, 2, 1);

    switch(cond) {
        case 0: case 1:
            tcg_gen_mov_tl(result, z);
            break;
        case 2: case 3: case 4: case 5:
            tcg_gen_extract_i32(result, cpu_psw, 0, 1);
            if(cond >= 4) {
                tcg_gen_xori_tl(z, z, 1);
                tcg_gen_and_tl(result, result, z);
            }
            break;
        case 6: case 7:
            tcg_gen_mov_tl(result, s);
            break;
        case 8: case 9: case 10: case 11:
            tcg_gen_extract_i32(result, cpu_psw, 3, 1);
            tcg_gen_xor_tl(result, result, s);
            if(cond >= 10) {
                tcg_gen_or_tl(result, result, z);
            }
            break;
        case 12: case 13:
            tcg_gen_extract_i32(result, cpu_psw, 3, 1);
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

static void gen_bit_cond_value(TCGv value, int bit, int cond)
{
    uint32_t mask = 1u << bit;
    if(cond == 15) {
        tcg_gen_xori_tl(value, value, mask);
    } else {
        TCGv condition = tcg_temp_new();
        gen_condition(condition, cond);
        tcg_gen_deposit_i32(value, value, condition, bit, 1);
        tcg_temp_free(condition);
    }
}

static void gen_bit_cond(int bit, int cond, int reg)
{
    TCGv value = tcg_temp_new();
    gen_read_reg(value, reg);
    gen_bit_cond_value(value, bit, cond);
    gen_write_reg(reg, value);
    tcg_temp_free(value);
}


/*
 * Instruction decode cursor.  The layout mirrors the interpreter's RXReader
 * so that the two decoders can be compared case by case.
 */
typedef struct RXDecode {
    DisasContextBase *base;
    uint32_t pc;
    uint32_t length;
    bool pc_synced;
} RXDecode;

static uint8_t rx_next8(RXDecode *d)
{
    uint8_t value = ldub_code(d->pc + d->length);
    d->length += 1;
    return value;
}

static uint16_t rx_next16(RXDecode *d)
{
    uint16_t value = ldub_code(d->pc + d->length);
    value |= (uint16_t)ldub_code(d->pc + d->length + 1) << 8;
    d->length += 2;
    return value;
}

static uint32_t rx_next32(RXDecode *d)
{
    uint32_t value = ldub_code(d->pc + d->length);
    value |= (uint32_t)ldub_code(d->pc + d->length + 1) << 8;
    value |= (uint32_t)ldub_code(d->pc + d->length + 2) << 16;
    value |= (uint32_t)ldub_code(d->pc + d->length + 3) << 24;
    d->length += 4;
    return value;
}

/* Sign-extended immediate, matching rx_read_imm(). */
static uint32_t rx_imm(RXDecode *d, int im)
{
    switch(im) {
        case 0:
            return rx_next32(d);
        case 1:
            return (uint32_t)(int32_t)(int8_t)rx_next8(d);
        case 2:
            return (uint32_t)(int32_t)(int16_t)rx_next16(d);
        default: {
            uint32_t value = rx_next8(d);
            value |= (uint32_t)rx_next8(d) << 8;
            value |= (uint32_t)rx_next8(d) << 16;
            return (uint32_t)((int32_t)(value << 8) >> 8);
        }
    }
}

/* Immediate with an explicit extension mode (MOV #imm needs zero extension
 * when the immediate width matches the destination width). */
static uint32_t rx_imm_ext(RXDecode *d, int im, bool sign_extend)
{
    int bytes = im == 0 ? 4 : im;
    uint32_t value = 0;

    for(int i = 0; i < bytes; i++) {
        value |= (uint32_t)rx_next8(d) << (i * 8);
    }
    if(sign_extend && bytes < 4 && (value & (1u << (bytes * 8 - 1)))) {
        value |= UINT32_MAX << (bytes * 8);
    }
    return value;
}

/* Scaled displacement, matching rx_disp_address(). */
static uint32_t rx_disp(RXDecode *d, int type, int size)
{
    if(type == 1) {
        return rx_next8(d) * (uint32_t)size;
    }
    if(type == 2) {
        return rx_next16(d) * (uint32_t)size;
    }
    return 0;
}

/* The interpreter keeps PC at the instruction start until the instruction
 * completes.  Set it before the first operation that can fault. */
static void rx_sync_pc(RXDecode *d)
{
    if(!d->pc_synced) {
        tcg_gen_movi_tl(cpu_pc, d->pc);
        d->pc_synced = true;
    }
}

static void gen_load_mem(TCGv value, TCGv address, int size, bool is_signed, int mem_idx)
{
    TCGMemOp op = size == 1 ? (is_signed ? MO_SB : MO_UB) :
                  size == 2 ? (is_signed ? MO_LESW : MO_LEUW) : MO_LEUL;
    tcg_gen_qemu_ld_tl(value, address, mem_idx, op | MO_UNALN);
}

static void gen_store_mem(DisasContextBase *base, TCGv value, TCGv address, int size)
{
    DisasContext *dc = (DisasContext *)base;
    TCGMemOp op = size == 1 ? MO_UB : size == 2 ? MO_LEUW : MO_LEUL;

    if(!dc->has_store) {
        dc->store_flags = tcg_temp_local_new_i32();
        tcg_gen_movi_i32(dc->store_flags, 0);
        dc->has_store = true;
    }
    tcg_gen_qemu_st_tl(value, address, base->mem_idx, op | MO_UNALN);
    TCGv_i32 flags = tcg_temp_new_i32();
    TCGv_i32 width = tcg_const_i32(size);
    TCGv_i32 mmu_idx = tcg_const_i32(base->mem_idx);
    TCGv_ptr tb = tcg_const_ptr((tcg_target_long)base->tb);
    gen_helper_rx_store_check(flags, cpu_env, tb, address, width, mmu_idx);
    tcg_gen_or_i32(dc->store_flags, dc->store_flags, flags);
    tcg_temp_free_ptr(tb);
    tcg_temp_free_i32(mmu_idx);
    tcg_temp_free_i32(width);
    tcg_temp_free_i32(flags);
}

static void gen_address_reg(TCGv address, int reg, uint32_t displacement)
{
    gen_read_reg(address, reg);
    if(displacement) {
        tcg_gen_addi_tl(address, address, displacement);
    }
}

static void gen_apply_size(TCGv value, int size, bool is_signed)
{
    if(size == 1) {
        if(is_signed) {
            tcg_gen_ext8s_tl(value, value);
        } else {
            tcg_gen_ext8u_tl(value, value);
        }
    } else if(size == 2) {
        if(is_signed) {
            tcg_gen_ext16s_tl(value, value);
        } else {
            tcg_gen_ext16u_tl(value, value);
        }
    }
}

/* Load an operand described by a displacement type and a register number.
 * Type 3 selects the register itself. */
static void gen_load_disp(RXDecode *d, TCGv value, int type, int reg,
                          int size, bool is_signed)
{
    if(type == 3) {
        gen_read_reg(value, reg);
        gen_apply_size(value, size, is_signed);
    } else {
        TCGv address = tcg_temp_new();
        uint32_t displacement = rx_disp(d, type, size);

        rx_sync_pc(d);
        gen_address_reg(address, reg, displacement);
        gen_load_mem(value, address, size, is_signed, d->base->mem_idx);
        tcg_temp_free(address);
    }
}

/* Store to a displacement operand.  Type 3 selects the register itself. */
static void gen_store_disp(RXDecode *d, int type, int reg, int size,
                           TCGv value, bool is_signed)
{
    if(type == 3) {
        TCGv result = tcg_temp_new();

        tcg_gen_mov_tl(result, value);
        gen_apply_size(result, size, is_signed);
        gen_write_reg(reg, result);
        tcg_temp_free(result);
    } else {
        TCGv address = tcg_temp_new();
        uint32_t displacement = rx_disp(d, type, size);

        rx_sync_pc(d);
        gen_address_reg(address, reg, displacement);
        gen_store_mem(d->base, value, address, size);
        tcg_temp_free(address);
    }
}

static void gen_carry_alu(int reg, TCGv src, bool subtract)
{
    TCGv dst = tcg_temp_new(), result = tcg_temp_new(), carry = tcg_temp_new();
    TCGv flag = tcg_temp_new(), extra = tcg_temp_new();

    gen_read_reg(dst, reg);
    tcg_gen_andi_tl(carry, cpu_psw, PSW_CARRY);
    if(subtract) {
        tcg_gen_xori_tl(carry, carry, 1);
        tcg_gen_sub_tl(result, dst, src);
        tcg_gen_sub_tl(result, result, carry);
        tcg_gen_setcond_tl(TCG_COND_GTU, flag, dst, src);
        tcg_gen_setcond_tl(TCG_COND_EQ, extra, dst, src);
        tcg_gen_xori_tl(carry, carry, 1);
        tcg_gen_and_tl(extra, extra, carry);
        tcg_gen_or_tl(flag, flag, extra);
    } else {
        tcg_gen_add_tl(result, dst, src);
        tcg_gen_setcond_tl(TCG_COND_LTU, flag, result, dst);
        tcg_gen_add_tl(result, result, carry);
        tcg_gen_setcond_tl(TCG_COND_LTU, extra, result, carry);
        tcg_gen_or_tl(flag, flag, extra);
    }
    gen_arithmetic_flags(dst, src, result, subtract);
    tcg_gen_andi_tl(cpu_psw, cpu_psw, ~PSW_CARRY);
    tcg_gen_or_tl(cpu_psw, cpu_psw, flag);
    gen_write_reg(reg, result);
    tcg_temp_free(extra);
    tcg_temp_free(flag);
    tcg_temp_free(carry);
    tcg_temp_free(result);
    tcg_temp_free(dst);
}

static void gen_division(int reg, TCGv src, bool is_signed)
{
    TCGv dst = tcg_temp_new(), divisor = tcg_temp_new(), invalid = tcg_temp_new();
    TCGv result = tcg_temp_new(), zero = tcg_const_tl(0), one = tcg_const_tl(1);

    gen_read_reg(dst, reg);
    tcg_gen_setcondi_tl(TCG_COND_EQ, invalid, src, 0);
    if(is_signed) {
        TCGv overflow = tcg_temp_new(), min = tcg_temp_new();

        tcg_gen_setcondi_tl(TCG_COND_EQ, overflow, src, -1);
        tcg_gen_setcondi_tl(TCG_COND_EQ, min, dst, INT32_MIN);
        tcg_gen_and_tl(overflow, overflow, min);
        tcg_gen_or_tl(invalid, invalid, overflow);
        tcg_temp_free(min);
        tcg_temp_free(overflow);
    }
    tcg_gen_movcond_tl(TCG_COND_NE, divisor, invalid, zero, one, src);
    if(is_signed) {
        tcg_gen_div_tl(result, dst, divisor);
    } else {
        tcg_gen_divu_tl(result, dst, divisor);
    }
    tcg_gen_movcond_tl(TCG_COND_NE, result, invalid, zero, dst, result);
    tcg_gen_andi_tl(cpu_psw, cpu_psw, ~PSW_OVERFLOW);
    tcg_gen_shli_tl(invalid, invalid, 3);
    tcg_gen_or_tl(cpu_psw, cpu_psw, invalid);
    gen_write_reg(reg, result);
    tcg_temp_free(one);
    tcg_temp_free(zero);
    tcg_temp_free(result);
    tcg_temp_free(invalid);
    tcg_temp_free(divisor);
    tcg_temp_free(dst);
}

static void gen_extended_result(int operation, int reg, TCGv src)
{
    TCGv dst = tcg_temp_new(), result = tcg_temp_new();

    gen_read_reg(dst, reg);
    if(operation == 4 || operation == 5) {
        tcg_gen_movcond_tl(operation == 4 ? TCG_COND_GT : TCG_COND_LT,
                           result, dst, src, dst, src);
        gen_write_reg(reg, result);
    } else if(operation == 6 || operation == 7) {
        TCGv high = tcg_temp_new();

        if(operation == 6) {
            tcg_gen_muls2_tl(result, high, dst, src);
        } else {
            tcg_gen_mulu2_tl(result, high, dst, src);
        }
        gen_write_reg(reg, result);
        gen_write_reg((reg + 1) & 15, high);
        tcg_temp_free(high);
    } else if(operation == 14 || operation == 15) {
        TCGv condition = tcg_temp_new(), zero = tcg_const_tl(0);

        gen_condition(condition, operation == 14 ? 0 : 1);
        tcg_gen_movcond_tl(TCG_COND_NE, result, condition, zero, src, dst);
        gen_write_reg(reg, result);
        tcg_temp_free(zero);
        tcg_temp_free(condition);
    } else {
        if(operation == 12 || operation == 16) {
            tcg_gen_and_tl(result, dst, src);
        } else {
            tcg_gen_xor_tl(result, dst, src);
        }
        gen_sz_flags(result);
        if(operation != 12) {
            gen_write_reg(reg, result);
        }
    }
    tcg_temp_free(result);
    tcg_temp_free(dst);
}

static void gen_int_float(int reg, TCGv src, bool is_unsigned, uint32_t pc)
{
    TCGv result = tcg_temp_new();
    TCGv operation = tcg_const_tl(is_unsigned ? RX_FPU_UTOF : RX_FPU_ITOF);
    TCGv zero = tcg_const_tl(0);

    tcg_gen_movi_tl(cpu_pc, pc);
    gen_helper_rx_fpu(result, cpu_env, operation, zero, src);
    gen_write_reg(reg, result);
    tcg_temp_free(zero);
    tcg_temp_free(operation);
    tcg_temp_free(result);
}

static int control_offset(int cr)
{
    switch(cr) {
        case 3: return offsetof(CPUState, fpsw);
        case 4: return offsetof(CPUState, cpen);
        case 8: return offsetof(CPUState, bpsw);
        case 9: return offsetof(CPUState, bpc);
        case 11: return offsetof(CPUState, fintv);
        case 12: return offsetof(CPUState, intb);
        case 13: return offsetof(CPUState, extb);
        default: return -1;
    }
}

static bool control_valid(int cr)
{
    return cr == 0 || cr == 1 || cr == 2 || cr == 10 || control_offset(cr) >= 0;
}

static void gen_read_control(TCGv value, int cr, uint32_t pc)
{
    switch(cr) {
        case 0: tcg_gen_mov_tl(value, cpu_psw); break;
        case 1: tcg_gen_movi_tl(value, pc); break;
        case 2: tcg_gen_mov_tl(value, cpu_usp); break;
        case 10: tcg_gen_mov_tl(value, cpu_isp); break;
        default: tcg_gen_ld_tl(value, cpu_env, control_offset(cr)); break;
    }
}

static void gen_write_control(DisasContextBase *base, int cr, TCGv value)
{
    switch(cr) {
        case 0: {
            TCGv pm = tcg_temp_new(), sanitized = tcg_temp_new(), user_value = tcg_temp_new();
            TCGv preserved = tcg_temp_new();
            TCGv zero = tcg_const_tl(0);

            tcg_gen_andi_tl(pm, cpu_psw, PSW_PM);
            tcg_gen_andi_tl(sanitized, value, ~PSW_PM);
            /* In user mode, writes to PM, I, U and IPL are ignored, so keep
             * the current values instead of clearing them. */
            tcg_gen_andi_tl(preserved, cpu_psw, PSW_I | PSW_U | PSW_IPL_MASK);
            tcg_gen_andi_tl(user_value, sanitized, ~(PSW_I | PSW_U | PSW_IPL_MASK));
            tcg_gen_or_tl(user_value, user_value, preserved);
            tcg_gen_or_tl(user_value, user_value, pm);
            tcg_gen_movcond_tl(TCG_COND_NE, cpu_psw, pm, zero, user_value, sanitized);
            tcg_temp_free(zero);
            tcg_temp_free(preserved);
            tcg_temp_free(user_value);
            tcg_temp_free(sanitized);
            tcg_temp_free(pm);
            ((DisasContext *)base)->stop_after_instruction = true;
            break;
        }
        case 1: break;
        case 2: tcg_gen_mov_tl(cpu_usp, value); break;
        case 3: gen_helper_rx_write_fpsw(cpu_env, value); break;
        case 8: case 9: case 11: case 12: case 13:
            /* In user mode, writes to these control registers are ignored. */
            if(base->mem_idx != MMU_USER_IDX) {
                tcg_gen_st_tl(value, cpu_env, control_offset(cr));
            }
            break;
        case 10:
            if(base->mem_idx != MMU_USER_IDX) {
                tcg_gen_mov_tl(cpu_isp, value);
            }
            break;
        default: tcg_gen_st_tl(value, cpu_env, control_offset(cr)); break;
    }
}

/* 0x40-0x5F: register/memory (unsigned byte) ALU group. */
static void gen_alu_group(RXDecode *d, uint8_t b0)
{
    uint8_t b1 = rx_next8(d);
    int rsrc = (b1 >> 4) & 0xf;
    int rdst = b1 & 0xf;
    int ooo = (b0 >> 2) & 7;
    int ss = b0 & 3;

    if(ooo >= 6) {
        int size = (ooo == 6) ? 1 : 2;
        TCGv value = tcg_temp_new();

        gen_load_disp(d, value, ss, rsrc, size, false);
        gen_store_disp(d, 3, rdst, size, value, false);
        tcg_temp_free(value);
    } else {
        int size = (ss == 3) ? 4 : 1;
        TCGv src = tcg_temp_new();

        gen_load_disp(d, src, ss, rsrc, size, ss == 3);
        gen_alu(ooo, rdst, src);
        tcg_temp_free(src);
    }
}

/* 0x80-0xBF: MOV/MOVU with a 5-bit scaled displacement. */
static void gen_mov_scaled(RXDecode *d, uint8_t b0)
{
    uint8_t b1 = rx_next8(d);
    int kind = (b0 >> 4) & 7;
    int displacement = ((b0 & 7) << 2) | ((b1 >> 6) & 2) | ((b1 >> 3) & 1);
    int base_reg = (b1 >> 4) & 7;
    int reg = b1 & 7;
    bool movu = kind == 3;
    int size = movu ? ((b0 & 8) ? 2 : 1) : (1 << kind);
    bool load = movu || (b0 & 8);
    TCGv address = tcg_temp_new(), value = tcg_temp_new();

    rx_sync_pc(d);
    gen_address_reg(address, base_reg, displacement * size);
    if(load) {
        gen_load_mem(value, address, size, !movu, d->base->mem_idx);
        gen_write_reg(reg, value);
    } else {
        gen_read_reg(value, reg);
        gen_store_mem(d->base, value, address, size);
    }
    tcg_temp_free(value);
    tcg_temp_free(address);
}

/* 0xC0-0xEF: MOV group. */
static bool gen_mov_group(RXDecode *d, uint8_t b0)
{
    int sz = (b0 >> 4) & 3;
    int sd = (b0 >> 2) & 3;
    int ss = b0 & 3;
    uint8_t b1;
    int rsrc, rdst, size;
    TCGv value;

    if(sz == 3) {
        return false;
    }
    b1 = rx_next8(d);
    rsrc = (b1 >> 4) & 0xf;
    rdst = b1 & 0xf;
    size = 1 << sz;

    value = tcg_temp_new();
    if(ss == 3 && sd != 3) {
        gen_read_reg(value, rdst);
        gen_store_disp(d, sd, rsrc, size, value, true);
    } else {
        gen_load_disp(d, value, ss, rsrc, size, true);
        gen_store_disp(d, sd, rdst, size, value, true);
    }
    tcg_temp_free(value);
    return true;
}

static void gen_branch(RXDecode *d, uint8_t opcode)
{
    static const int dsp3[] = { 8, 9, 10, 3, 4, 5, 6, 7 };
    int32_t displacement;
    int cond = 14;

    if(opcode == 0x04) {
        displacement = (int32_t)rx_imm(d, 3);
    } else if(opcode >= 0x08 && opcode <= 0x1f) {
        displacement = dsp3[opcode & 7];
        if(opcode >= 0x10) {
            cond = (opcode >> 3) & 1;
        }
    } else if(opcode >= 0x20 && opcode <= 0x2f) {
        displacement = (int8_t)rx_next8(d);
        cond = opcode & 15;
    } else {
        displacement = (int16_t)rx_next16(d);
        if(opcode != 0x38) {
            cond = opcode - 0x3a;
        }
    }

    if(cond == 14) {
        tcg_gen_movi_tl(cpu_pc, d->pc + (uint32_t)displacement);
    } else {
        TCGv condition = tcg_temp_new();
        TCGv zero = tcg_const_tl(0);
        TCGv target = tcg_const_tl(d->pc + (uint32_t)displacement);
        TCGv fallthrough = tcg_const_tl(d->pc + d->length);

        gen_condition(condition, cond);
        tcg_gen_movcond_tl(TCG_COND_NE, cpu_pc, condition, zero, target, fallthrough);
        tcg_temp_free(fallthrough);
        tcg_temp_free(target);
        tcg_temp_free(zero);
        tcg_temp_free(condition);
    }
    d->base->is_jmp = DISAS_JUMP;
}

static void gen_push_return(RXDecode *d)
{
    TCGv stack = tcg_temp_new(), value = tcg_const_tl(d->pc + d->length);

    rx_sync_pc(d);
    gen_read_reg(stack, 0);
    tcg_gen_subi_tl(stack, stack, 4);
    gen_write_reg(0, stack);
    gen_store_mem(d->base, value, stack, 4);
    tcg_temp_free(value);
    tcg_temp_free(stack);
}

static void gen_rts(RXDecode *d)
{
    TCGv stack = tcg_temp_new(), target = tcg_temp_new();

    rx_sync_pc(d);
    gen_read_reg(stack, 0);
    gen_load_mem(target, stack, 4, false, d->base->mem_idx);
    tcg_gen_addi_tl(stack, stack, 4);
    gen_write_reg(0, stack);
    tcg_gen_mov_tl(cpu_pc, target);
    tcg_temp_free(target);
    tcg_temp_free(stack);
    d->base->is_jmp = DISAS_JUMP;
}

static void gen_bsr(RXDecode *d, int32_t displacement)
{
    gen_push_return(d);
    tcg_gen_movi_tl(cpu_pc, d->pc + (uint32_t)displacement);
    d->base->is_jmp = DISAS_JUMP;
}

static bool gen_rtsd(RXDecode *d)
{
    uint8_t b1 = rx_next8(d);
    int first = b1 >> 4;
    int last = b1 & 15;
    uint32_t adjustment;
    TCGv stack, value;

    if(first == 0) {
        return false;
    }
    adjustment = rx_next8(d) * 4 - (uint32_t)(last - first + 1) * 4;
    stack = tcg_temp_new();
    value = tcg_temp_new();
    rx_sync_pc(d);
    gen_read_reg(stack, 0);
    tcg_gen_addi_tl(stack, stack, adjustment);
    gen_write_reg(0, stack);
    for(int reg = first; reg <= last; reg++) {
        gen_load_mem(value, stack, 4, false, d->base->mem_idx);
        tcg_gen_addi_tl(stack, stack, 4);
        gen_write_reg(0, stack);
        gen_write_reg(reg, value);
    }
    gen_load_mem(value, stack, 4, false, d->base->mem_idx);
    tcg_gen_addi_tl(stack, stack, 4);
    gen_write_reg(0, stack);
    tcg_gen_mov_tl(cpu_pc, value);
    tcg_temp_free(value);
    tcg_temp_free(stack);
    d->base->is_jmp = DISAS_JUMP;
    return true;
}

static void gen_rtsd_short(RXDecode *d)
{
    uint32_t adjustment = rx_next8(d) * 4;
    TCGv stack = tcg_temp_new(), target = tcg_temp_new();

    rx_sync_pc(d);
    gen_read_reg(stack, 0);
    tcg_gen_addi_tl(stack, stack, adjustment);
    gen_write_reg(0, stack);
    gen_load_mem(target, stack, 4, false, d->base->mem_idx);
    tcg_gen_addi_tl(stack, stack, 4);
    gen_write_reg(0, stack);
    tcg_gen_mov_tl(cpu_pc, target);
    tcg_temp_free(target);
    tcg_temp_free(stack);
    d->base->is_jmp = DISAS_JUMP;
}

static bool gen_reg_branch(RXDecode *d)
{
    uint8_t b1 = rx_next8(d);
    int hi = b1 & 0xf0;
    int reg = b1 & 15;
    TCGv target;

    if(hi != 0x00 && hi != 0x10 && hi != 0x40 && hi != 0x50) {
        return false;
    }
    target = tcg_temp_new();
    if(hi == 0x10 || hi == 0x50) {
        gen_push_return(d);
    }
    gen_read_reg(target, reg);
    if(hi == 0x40 || hi == 0x50) {
        /* BSR.L Rn branches to the instruction address plus Rn.  Only the
         * return address pushed on the stack uses the next instruction. */
        tcg_gen_addi_tl(target, target, d->pc);
    }
    tcg_gen_mov_tl(cpu_pc, target);
    tcg_temp_free(target);
    d->base->is_jmp = DISAS_JUMP;
    return true;
}

static int memex_size(int mx)
{
    return mx == 2 ? 4 : mx == 0 ? 1 : 2;
}

static bool memex_signed(int mx)
{
    return mx != 3;
}

/* 0x06: MEMEX group. */
static bool gen_memex(RXDecode *d)
{
    uint8_t b1 = rx_next8(d);
    int mx = (b1 >> 6) & 3;
    int grp = (b1 >> 4) & 3;
    int op = (b1 >> 2) & 3;
    int ss = b1 & 3;
    int size = memex_size(mx);
    bool is_signed = memex_signed(mx);

    if(grp == 2) {
        uint8_t op2 = rx_next8(d);
        uint8_t b3 = rx_next8(d);
        int rsrc = b3 >> 4;
        int rdst = b3 & 15;
        bool has_address = ss != 3;
        TCGv src, address;

        switch(op2) {
            case 0x00: case 0x02: case 0x04: case 0x05:
            case 0x06: case 0x07: case 0x08: case 0x09:
            case 0x0c: case 0x0d: case 0x10: case 0x11:
            case 0x15:
                break;
            default:
                return false;
        }

        src = tcg_temp_new();
        if(ss == 3) {
            gen_read_reg(src, rsrc);
            gen_apply_size(src, size, is_signed);
        } else {
            uint32_t displacement = rx_disp(d, ss, size);

            address = tcg_temp_new();
            rx_sync_pc(d);
            gen_address_reg(address, rsrc, displacement);
            gen_load_mem(src, address, size, is_signed, d->base->mem_idx);
        }

        switch(op2) {
            case 0x00:
                gen_carry_alu(rdst, src, true);
                break;
            case 0x02:
                gen_carry_alu(rdst, src, false);
                break;
            case 0x04:
            case 0x05:
            case 0x06:
            case 0x07:
            case 0x0c:
            case 0x0d:
                gen_extended_result(op2, rdst, src);
                break;
            case 0x08:
                gen_division(rdst, src, true);
                break;
            case 0x09:
                gen_division(rdst, src, false);
                break;
            case 0x10: {
                TCGv dst = tcg_temp_new();

                if(ss == 3) {
                    gen_read_reg(dst, rdst);
                    gen_write_reg(rsrc, dst);
                    gen_write_reg(rdst, src);
                } else {
                    gen_read_reg(dst, rdst);
                    gen_store_mem(d->base, dst, address, size);
                    gen_write_reg(rdst, src);
                }
                tcg_temp_free(dst);
                break;
            }
            case 0x11:
                gen_int_float(rdst, src, false, d->pc);
                break;
            default:
                gen_int_float(rdst, src, true, d->pc);
                break;
        }
        if(has_address) {
            tcg_temp_free(address);
        }
        tcg_temp_free(src);
        return true;
    }

    if(grp == 3 || (grp == 1 && op > 1)) {
        return false;
    }

    {
        uint8_t b2 = rx_next8(d);
        int rs = b2 >> 4;
        int rd = b2 & 15;
        TCGv src = tcg_temp_new();

        gen_load_disp(d, src, ss, rs, size, is_signed);
        if(grp == 0) {
            gen_alu(op, rd, src);
        } else {
            gen_alu(op == 0 ? 4 : 5, rd, src);
        }
        tcg_temp_free(src);
    }
    return true;
}

/* 0x60-0x66: ALU/MOV #imm4. */
static void gen_imm4(RXDecode *d, uint8_t b0)
{
    uint8_t b1 = rx_next8(d);
    TCGv value = tcg_const_tl(b1 >> 4);

    if(b0 == 0x66) {
        gen_write_reg(b1 & 15, value);
    } else {
        gen_alu(b0 - 0x60, b1 & 15, value);
    }
    tcg_temp_free(value);
}

/* 0x68-0x6D: SHLR/SHAR/SHLL #imm5, Rn. */
static void gen_shift_imm5(RXDecode *d, uint8_t b0)
{
    uint8_t b1 = rx_next8(d);
    int count = ((b0 & 1) << 4) | (b1 >> 4);

    gen_shift_imm((b0 - 0x68) / 2, count, b1 & 15, b1 & 15);
}

/* 0x6E/0x6F: PUSHM/POPM. */
static bool gen_push_pop_m(RXDecode *d, uint8_t b0)
{
    uint8_t b1 = rx_next8(d);
    int first = b1 >> 4;
    int last = b1 & 15;
    TCGv stack, value;

    if(first == 0 || last == 0 || first >= last) {
        return false;
    }
    stack = tcg_temp_new();
    value = tcg_temp_new();
    rx_sync_pc(d);
    if(b0 == 0x6e) {
        for(int reg = last; reg >= first; reg--) {
            gen_read_reg(value, reg);
            gen_read_reg(stack, 0);
            tcg_gen_subi_tl(stack, stack, 4);
            gen_write_reg(0, stack);
            gen_store_mem(d->base, value, stack, 4);
        }
    } else {
        for(int reg = first; reg <= last; reg++) {
            gen_read_reg(stack, 0);
            gen_load_mem(value, stack, 4, false, d->base->mem_idx);
            tcg_gen_addi_tl(stack, stack, 4);
            gen_write_reg(0, stack);
            gen_write_reg(reg, value);
        }
    }
    tcg_temp_free(value);
    tcg_temp_free(stack);
    return true;
}

/* 0x70-0x73: ADD #imm, Rsrc, Rdst. */
static void gen_add_imm(RXDecode *d, uint8_t b0)
{
    uint8_t b1 = rx_next8(d);
    uint32_t immediate = rx_imm(d, b0 & 3);
    TCGv src = tcg_temp_new(), value = tcg_const_tl(immediate), result = tcg_temp_new();

    gen_read_reg(src, b1 >> 4);
    gen_alu_values(2, src, value, result);
    gen_write_reg(b1 & 15, result);
    tcg_temp_free(result);
    tcg_temp_free(value);
    tcg_temp_free(src);
}

/* 0x74-0x77: CMP/MUL/AND/OR #imm, Rn and the 0x75 special forms. */
static bool gen_imm_alu(RXDecode *d, uint8_t b0)
{
    uint8_t b1 = rx_next8(d);
    int opsel = b1 >> 4;
    int rd = b1 & 15;
    uint32_t immediate;
    TCGv value;
    int op;

    if(b0 == 0x75 && (opsel == 4 || opsel == 5)) {
        immediate = rx_next8(d);
        value = tcg_const_tl(immediate);
        if(opsel == 4) {
            gen_write_reg(rd, value);
        } else {
            gen_alu(1, rd, value);
        }
        tcg_temp_free(value);
        return true;
    }
    if(opsel > 3) {
        return false;
    }
    immediate = rx_imm(d, b0 & 3);
    value = tcg_const_tl(immediate);
    op = opsel == 0 ? 1 : opsel == 1 ? 3 : opsel == 2 ? 4 : 5;
    gen_alu(op, rd, value);
    tcg_temp_free(value);
    return true;
}

/* 0x78-0x7D: BSET/BCLR/BTST #bit, Rn. */
static void gen_bit_imm_group(RXDecode *d, uint8_t b0)
{
    uint8_t b1 = rx_next8(d);
    int bit = ((b0 & 1) << 4) | (b1 >> 4);

    gen_bit_imm((b0 - 0x78) / 2, bit, b1 & 15);
}

static bool gen_push_pop_control(RXDecode *d, int op, int cr)
{
    bool push = op < 14;
    TCGv stack, value;

    if(!control_valid(cr)) {
        return false;
    }
    stack = tcg_temp_new();
    value = tcg_temp_new();
    rx_sync_pc(d);
    if(push) {
        gen_read_control(value, cr, d->pc);
    }
    gen_read_reg(stack, 0);
    if(push) {
        tcg_gen_subi_tl(stack, stack, 4);
        gen_write_reg(0, stack);
        gen_store_mem(d->base, value, stack, 4);
    } else {
        gen_load_mem(value, stack, 4, false, d->base->mem_idx);
        tcg_gen_addi_tl(stack, stack, 4);
        gen_write_reg(0, stack);
        gen_write_control(d->base, cr, value);
    }
    tcg_temp_free(value);
    tcg_temp_free(stack);
    return true;
}

/* 0x7E: unary, push/pop, control register push/pop. */
static bool gen_unary_group(RXDecode *d)
{
    uint8_t b1 = rx_next8(d);
    int hi = b1 >> 4;
    int lo = b1 & 15;

    switch(hi) {
        case 0:
            gen_reg_logic(0x3b, lo, lo);
            return true;
        case 1:
            gen_unary(1, lo, lo);
            return true;
        case 2:
            gen_unary(2, lo, lo);
            return true;
        case 3:
            gen_unary(3, lo, lo);
            return true;
        case 4:
            gen_rotate_carry(lo, false);
            return true;
        case 5:
            gen_rotate_carry(lo, true);
            return true;
        case 6:
        case 7:
            return false;
        case 8:
        case 9:
        case 10: {
            int size = 1 << (hi - 8);
            TCGv stack = tcg_temp_new(), value = tcg_temp_new();

            gen_read_reg(value, lo);
            if(size == 1) {
                tcg_gen_andi_tl(value, value, 0xff);
            } else if(size == 2) {
                tcg_gen_andi_tl(value, value, 0xffff);
            }
            rx_sync_pc(d);
            gen_read_reg(stack, 0);
            tcg_gen_subi_tl(stack, stack, 4);
            gen_write_reg(0, stack);
            gen_store_mem(d->base, value, stack, 4);
            tcg_temp_free(value);
            tcg_temp_free(stack);
            return true;
        }
        case 11: {
            TCGv stack = tcg_temp_new(), value = tcg_temp_new();

            rx_sync_pc(d);
            gen_read_reg(stack, 0);
            gen_load_mem(value, stack, 4, false, d->base->mem_idx);
            tcg_gen_addi_tl(stack, stack, 4);
            gen_write_reg(0, stack);
            gen_write_reg(lo, value);
            tcg_temp_free(value);
            tcg_temp_free(stack);
            return true;
        }
        default:
            return gen_push_pop_control(d, hi, lo);
    }
}

static void gen_flag(RXDecode *d, uint8_t b1)
{
    uint32_t bit = rx_psw_flag_bit(b1);

    /* In user mode the I and U bits cannot be changed by SETPSW/CLRPSW.  The
     * processor mode is fixed for the lifetime of a TB, so this is known at
     * translation time. */
    if(d->base->mem_idx == MMU_USER_IDX && (bit & (PSW_I | PSW_U))) {
        return;
    }
    if(b1 & 0x10) {
        tcg_gen_andi_tl(cpu_psw, cpu_psw, ~bit);
    } else {
        tcg_gen_ori_tl(cpu_psw, cpu_psw, bit);
    }
    if(bit & (PSW_I | PSW_U)) {
        ((DisasContext *)d->base)->stop_after_instruction = true;
    }
}

/* 0xF0-0xF7: bit operations on memory, plus the push.b/w/l aliases. */
static bool gen_memory_bit(RXDecode *d, uint8_t b0)
{
    uint8_t b1 = rx_next8(d);
    int bit = b1 & 7;
    int rdst = b1 >> 4;
    int sd = b0 & 3;
    TCGv mask, value, address;

    if(b0 >= 0xf4 && b0 <= 0xf6 && (b1 & 15) >= 8 && (b1 & 15) <= 10) {
        int size = 1 << ((b1 & 15) - 8);
        TCGv stack = tcg_temp_new();
        uint32_t displacement = rx_disp(d, sd, size);

        value = tcg_temp_new();
        address = tcg_temp_new();
        rx_sync_pc(d);
        gen_address_reg(address, b1 >> 4, displacement);
        gen_load_mem(value, address, size, false, d->base->mem_idx);
        gen_read_reg(stack, 0);
        tcg_gen_subi_tl(stack, stack, 4);
        gen_write_reg(0, stack);
        if(size == 1) {
            tcg_gen_andi_tl(value, value, 0xff);
        } else if(size == 2) {
            tcg_gen_andi_tl(value, value, 0xffff);
        }
        gen_store_mem(d->base, value, stack, 4);
        tcg_temp_free(stack);
        tcg_temp_free(address);
        tcg_temp_free(value);
        return true;
    }
    if(sd == 3) {
        return false;
    }

    mask = tcg_const_tl(1u << bit);
    value = tcg_temp_new();
    address = tcg_temp_new();
    {
        uint32_t displacement = rx_disp(d, sd, 1);

        rx_sync_pc(d);
        gen_address_reg(address, rdst, displacement);
    }
    gen_load_mem(value, address, 1, false, d->base->mem_idx);
    if(b0 < 0xf4) {
        if(b1 & 8) {
            tcg_gen_andc_tl(value, value, mask);
        } else {
            tcg_gen_or_tl(value, value, mask);
        }
        gen_store_mem(d->base, value, address, 1);
    } else {
        tcg_gen_and_tl(value, value, mask);
        gen_bit_test(value);
    }
    tcg_temp_free(address);
    tcg_temp_free(value);
    tcg_temp_free(mask);
    return true;
}

/* 0xF8-0xFB: MOV #imm, <dst>. */
static bool gen_mov_imm(RXDecode *d, uint8_t b0)
{
    int sd = b0 & 3;
    uint8_t b1 = rx_next8(d);
    int rdst = b1 >> 4;
    int im = (b1 >> 2) & 3;
    int sz = b1 & 3;
    int size;
    uint32_t displacement = 0;
    uint32_t raw;
    bool zero_extend;
    TCGv value;

    if(sz == 3) {
        return false;
    }
    size = 1 << sz;
    if(sd != 3) {
        displacement = rx_disp(d, sd, size);
    }
    zero_extend = (im == 1 && sz == 0) || (im == 2 && sz == 1) || (im == 0 && sz == 2);
    raw = rx_imm_ext(d, im, !zero_extend);
    value = tcg_const_tl(raw);
    if(sd == 3) {
        TCGv result = tcg_temp_new();

        tcg_gen_mov_tl(result, value);
        gen_apply_size(result, size, true);
        gen_write_reg(rdst, result);
        tcg_temp_free(result);
    } else {
        TCGv address = tcg_temp_new();

        rx_sync_pc(d);
        gen_address_reg(address, rdst, displacement);
        gen_store_mem(d->base, value, address, size);
        tcg_temp_free(address);
    }
    tcg_temp_free(value);
    return true;
}

/* 0xFC: extended data-processing group. */
static bool gen_extended_0xfc(RXDecode *d)
{
    uint8_t b1 = rx_next8(d);
    uint8_t b2 = rx_next8(d);

    /* BSET/BCLR/BTST/BNOT Rsrc, <dst> (register bit index). */
    if((b1 & 0xf0) == 0x60) {
        int bit_op = (b1 >> 2) & 3;
        int sd = b1 & 3;
        int bdst = b2 >> 4;
        int bsrc = b2 & 15;
        int size = (sd == 3) ? 4 : 1;
        int bit_mask = (sd == 3) ? 0x1f : 0x07;
        TCGv bit = tcg_temp_new(), mask = tcg_const_tl(1);
        TCGv value = tcg_temp_new(), address = tcg_temp_new();

        gen_read_reg(bit, bsrc);
        tcg_gen_andi_tl(bit, bit, bit_mask);
        tcg_gen_shl_tl(mask, mask, bit);
        if(sd == 3) {
            gen_read_reg(value, bdst);
        } else {
            uint32_t displacement = rx_disp(d, sd, size);

            rx_sync_pc(d);
            gen_address_reg(address, bdst, displacement);
            gen_load_mem(value, address, size, false, d->base->mem_idx);
        }
        if(bit_op == 2) {
            tcg_gen_and_tl(value, value, mask);
            gen_bit_test(value);
        } else {
            if(bit_op == 0) {
                tcg_gen_or_tl(value, value, mask);
            } else if(bit_op == 1) {
                tcg_gen_not_tl(mask, mask);
                tcg_gen_and_tl(value, value, mask);
            } else {
                tcg_gen_xor_tl(value, value, mask);
            }
            if(sd == 3) {
                gen_write_reg(bdst, value);
            } else {
                gen_store_mem(d->base, value, address, size);
            }
        }
        tcg_temp_free(address);
        tcg_temp_free(value);
        tcg_temp_free(mask);
        tcg_temp_free(bit);
        return true;
    }

    /* XCHG Rsrc, Rdst */
    if((b1 & 0xfc) == 0x40) {
        int ss = b1 & 3;
        int xsrc = b2 >> 4;
        int xdst = b2 & 15;

        if(ss == 3) {
            TCGv src = tcg_temp_new(), dst = tcg_temp_new();

            gen_read_reg(src, xsrc);
            gen_read_reg(dst, xdst);
            gen_write_reg(xdst, src);
            gen_write_reg(xsrc, dst);
            tcg_temp_free(dst);
            tcg_temp_free(src);
        } else {
            TCGv address = tcg_temp_new(), mem = tcg_temp_new(), dst = tcg_temp_new();
            uint32_t displacement = rx_disp(d, ss, 1);

            rx_sync_pc(d);
            gen_address_reg(address, xsrc, displacement);
            gen_load_mem(mem, address, 1, false, d->base->mem_idx);
            gen_read_reg(dst, xdst);
            gen_store_mem(d->base, dst, address, 1);
            gen_write_reg(xdst, mem);
            tcg_temp_free(dst);
            tcg_temp_free(mem);
            tcg_temp_free(address);
        }
        return true;
    }

    /* SC<cond>.sz */
    if((b1 & 0xf0) == 0xd0) {
        int sz = (b1 >> 2) & 3;
        int sd = b1 & 3;
        int cond = b2 & 15;
        TCGv value;

        if(sz == 3) {
            return false;
        }
        value = tcg_temp_new();
        gen_condition(value, cond);
        gen_store_disp(d, sd, b2 >> 4, 1 << sz, value, false);
        tcg_temp_free(value);
        return true;
    }

    /* BFMOV/BFMOVZ (RXv3) */
    if(b1 == 0x5a || b1 == 0x5e) {
        uint16_t field;
        int dlsb, slsb, width;
        uint32_t mask;
        TCGv value, dst;

        if(!((DisasContext *)d->base)->is_v3) {
            return false;
        }
        field = rx_next16(d);
        dlsb = (field >> 5) & 31;
        slsb = (dlsb - (field & 31)) & 31;
        width = ((field >> 10) & 31) - dlsb;
        if(width <= 0) {
            width += 32;
        }
        mask = width == 32 ? UINT32_MAX : (1u << width) - 1;
        value = tcg_temp_new();
        dst = tcg_temp_new();
        gen_read_reg(value, b2 >> 4);
        if(width == 32) {
            tcg_gen_shri_tl(value, value, slsb);
        } else if(slsb + width <= 32) {
            tcg_gen_extract_i32(value, value, slsb, width);
        } else {
            tcg_gen_shri_tl(value, value, slsb);
            tcg_gen_andi_tl(value, value, mask);
        }
        if(b1 == 0x5a) {
            if(dlsb + width <= 32) {
                tcg_gen_deposit_z_i32(value, value, dlsb, width);
            } else {
                tcg_gen_shli_tl(value, value, dlsb);
            }
        } else {
            gen_read_reg(dst, b2 & 15);
            if(dlsb + width <= 32) {
                tcg_gen_deposit_i32(dst, dst, value, dlsb, width);
            } else {
                uint32_t destination_mask = mask << dlsb;

                tcg_gen_andi_tl(dst, dst, ~destination_mask);
                tcg_gen_shli_tl(value, value, dlsb);
                tcg_gen_or_tl(dst, dst, value);
            }
            tcg_gen_mov_tl(value, dst);
        }
        gen_write_reg(b2 & 15, value);
        tcg_temp_free(dst);
        tcg_temp_free(value);
        return true;
    }

    /* Single-precision FPU register/memory operations. */
    if(b1 >= 0x80 && b1 <= 0xa7) {
        int fop = b1 >> 2;
        int fsd = b1 & 3;
        int fsrc_reg = b2 >> 4;
        int fdst_reg = b2 & 15;
        int op = fop - 0x20;
        TCGv src, dst;

        if(op < 0 || (op > RX_FPU_DIV && op != RX_FPU_SQRT)) {
            return false;
        }
        src = tcg_temp_new();
        dst = tcg_temp_new();
        rx_sync_pc(d);
        if(fsd == 3) {
            gen_read_reg(src, fsrc_reg);
        } else {
            TCGv address = tcg_temp_new();
            uint32_t displacement = rx_disp(d, fsd, 4);

            gen_address_reg(address, fsrc_reg, displacement);
            gen_load_mem(src, address, 4, false, d->base->mem_idx);
            tcg_temp_free(address);
        }
        if(op == RX_FPU_SQRT) {
            tcg_gen_movi_tl(dst, 0);
        } else {
            gen_read_reg(dst, fdst_reg);
        }
        if(op == RX_FPU_CMP) {
            gen_helper_rx_fcmp(cpu_env, dst, src);
        } else {
            TCGv operation = tcg_const_tl(op), result = tcg_temp_new();

            gen_helper_rx_fpu(result, cpu_env, operation, dst, src);
            gen_write_reg(fdst_reg, result);
            tcg_temp_free(result);
            tcg_temp_free(operation);
        }
        tcg_temp_free(dst);
        tcg_temp_free(src);
        return true;
    }

    /* ALU operations with an unsigned-byte memory operand. */
    {
        int alu_op = b1 >> 2;
        int alu_sd = b1 & 3;

        if(alu_sd != 3 &&
           (alu_op == 12 || alu_op == 13 || alu_op == 17 || alu_op == 21)) {
            int adst = b2 & 15;
            TCGv value = tcg_temp_new(), address = tcg_temp_new();
            uint32_t displacement = rx_disp(d, alu_sd, 1);

            rx_sync_pc(d);
            gen_address_reg(address, b2 >> 4, displacement);
            gen_load_mem(value, address, 1, false, d->base->mem_idx);
            if(alu_op == 12) {
                gen_extended_result(12, adst, value);
            } else if(alu_op == 13) {
                gen_extended_result(13, adst, value);
            } else {
                gen_int_float(adst, value, alu_op == 21, d->pc);
            }
            tcg_temp_free(address);
            tcg_temp_free(value);
            return true;
        }
    }

    /* BMCnd/BNOT with immediate bit index. */
    if(b1 >= 0xe0) {
        int sd = b1 & 3;
        int bit = (b1 >> 2) & 7;
        int cond = b2 & 15;
        int dst = b2 >> 4;
        TCGv value = tcg_temp_new();

        if(sd == 3) {
            gen_read_reg(value, dst);
            gen_bit_cond_value(value, bit, cond);
            gen_write_reg(dst, value);
            tcg_temp_free(value);
        } else {
            uint32_t displacement = rx_disp(d, sd, 1);
            TCGv address = tcg_temp_new();

            rx_sync_pc(d);
            gen_address_reg(address, dst, displacement);
            gen_load_mem(value, address, 1, false, d->base->mem_idx);
            gen_bit_cond_value(value, bit, cond);
            gen_store_mem(d->base, value, address, 1);
            tcg_temp_free(address);
            tcg_temp_free(value);
        }
        return true;
    }

    {
        int rsrc = b2 >> 4;
        int rdst = b2 & 15;
        TCGv src;

        switch(b1) {
            case 0x03: case 0x07: case 0x0b: case 0x0f:
            case 0x13: case 0x17: case 0x1b: case 0x1f:
            case 0x23: case 0x27: case 0x33: case 0x37:
            case 0x3b: case 0x47: case 0x4b: case 0x4f:
            case 0x57:
                break;
            default:
                return false;
        }
        src = tcg_temp_new();
        gen_read_reg(src, rsrc);
        switch(b1) {
            case 0x03:
                gen_carry_alu(rdst, src, true);
                break;
            case 0x07:
                gen_unary(1, rsrc, rdst);
                break;
            case 0x0b:
                gen_carry_alu(rdst, src, false);
                break;
            case 0x0f:
                gen_unary(2, rsrc, rdst);
                break;
            case 0x13:
            case 0x17:
            case 0x1b:
            case 0x1f:
            case 0x33:
            case 0x37:
            case 0x4b:
            case 0x4f:
                gen_extended_result(b1 == 0x13 ? 4 : b1 == 0x17 ? 5 :
                                    b1 == 0x1b ? 6 : b1 == 0x1f ? 7 :
                                    b1 == 0x33 ? 12 : b1 == 0x37 ? 13 :
                                    b1 == 0x4b ? 14 : 15, rdst, src);
                break;
            case 0x23:
                gen_division(rdst, src, true);
                break;
            case 0x27:
                gen_division(rdst, src, false);
                break;
            case 0x3b:
                gen_reg_logic(0x3b, rsrc, rdst);
                break;
            case 0x47:
                gen_int_float(rdst, src, false, d->pc);
                break;
            default:
                gen_int_float(rdst, src, true, d->pc);
                break;
        }
        tcg_temp_free(src);
        return true;
    }
}

/* 0xFD: extended group. */
static bool gen_extended_0xfd(RXDecode *d)
{
    uint8_t b1 = rx_next8(d);

    /* DSP instructions. */
    if(b1 <= 0x0f || b1 == 0x17 || b1 == 0x18 || b1 == 0x19 ||
       b1 == 0x1e || b1 == 0x1f || (b1 >= 0x44 && b1 <= 0x4f)) {
        uint8_t b2 = rx_next8(d);
        int selector = (b2 >> 4) & 7;

        if((b1 == 0x17 && selector != 0 && selector != 1 && selector != 3) ||
           (b1 >= 0x44 && (b1 & 7) < 4)) {
            return false;
        }
        if(b1 <= 0x0f || b1 >= 0x44) {
            int mode = b1 >= 0x44 ? RX_DSP_SUB : (b1 & 4) ? RX_DSP_ADD : RX_DSP_SET;
            TCGv select = tcg_const_tl((b1 >> 3) & 1);
            TCGv mode_arg = tcg_const_tl(mode), form = tcg_const_tl(b1 & 3);
            TCGv src1 = tcg_temp_new(), src2 = tcg_temp_new();

            gen_read_reg(src1, b2 >> 4);
            gen_read_reg(src2, b2 & 15);
            gen_helper_rx_dsp_mul(cpu_env, mode_arg, form, select, src1, src2);
            tcg_temp_free(src2);
            tcg_temp_free(src1);
            tcg_temp_free(form);
            tcg_temp_free(mode_arg);
            tcg_temp_free(select);
        } else if(b1 == 0x17) {
            TCGv select = tcg_const_tl(b2 >> 7);
            TCGv field = tcg_const_tl(selector), src = tcg_temp_new();

            gen_read_reg(src, b2 & 15);
            gen_helper_rx_dsp_mvtac(cpu_env, select, field, src);
            tcg_temp_free(src);
            tcg_temp_free(field);
            tcg_temp_free(select);
        } else if(b1 == 0x18 || b1 == 0x19) {
            TCGv select = tcg_const_tl(b2 >> 7);
            TCGv shift = tcg_const_tl((b2 & 0x10) ? 2 : 1);
            TCGv wide = tcg_const_tl(b1 == 0x19), down = tcg_const_tl((b2 & 0x40) != 0);

            gen_helper_rx_dsp_rac(cpu_env, select, shift, wide, down);
            tcg_temp_free(down);
            tcg_temp_free(wide);
            tcg_temp_free(shift);
            tcg_temp_free(select);
        } else {
            TCGv select = tcg_const_tl(b2 >> 7);
            TCGv field = tcg_const_tl(selector & 3);
            TCGv shift = tcg_const_tl(b1 == 0x1e ? 2 : (b2 >> 6) & 1);
            TCGv result = tcg_temp_new();

            gen_helper_rx_dsp_mvfac(result, cpu_env, select, field, shift);
            gen_write_reg(b2 & 15, result);
            tcg_temp_free(result);
            tcg_temp_free(shift);
            tcg_temp_free(field);
            tcg_temp_free(select);
        }
        return true;
    }

    /* Post-increment / pre-decrement moves, MOVCO/MOVLI. */
    if((b1 >= 0x20 && b1 <= 0x2f) || b1 == 0x38 || b1 == 0x39 ||
       b1 == 0x3c || b1 == 0x3d) {
        int size;
        bool load, pre, is_signed = true;
        uint8_t b2;
        int base_reg, data_reg;
        TCGv address, value;

        if(b1 >= 0x20 && b1 <= 0x22) {
            size = 1 << (b1 - 0x20); load = false; pre = false;
        } else if(b1 >= 0x24 && b1 <= 0x26) {
            size = 1 << (b1 - 0x24); load = false; pre = true;
        } else if(b1 >= 0x28 && b1 <= 0x2a) {
            size = 1 << (b1 - 0x28); load = true; pre = false;
        } else if(b1 >= 0x2c && b1 <= 0x2e) {
            size = 1 << (b1 - 0x2c); load = true; pre = true;
        } else if(b1 == 0x38 || b1 == 0x39) {
            size = 1 << (b1 - 0x38); load = true; pre = false; is_signed = false;
        } else if(b1 == 0x3c || b1 == 0x3d) {
            size = 1 << (b1 - 0x3c); load = true; pre = true; is_signed = false;
        } else {
            /* MOVCO/MOVLI keep using the interpreter for the link flag,
             * and the remaining encodings are illegal. */
            return false;
        }
        b2 = rx_next8(d);
        base_reg = b2 >> 4;
        data_reg = b2 & 15;
        address = tcg_temp_new();
        value = tcg_temp_new();
        rx_sync_pc(d);
        gen_read_reg(address, base_reg);
        if(pre) {
            tcg_gen_subi_tl(address, address, size);
            gen_write_reg(base_reg, address);
        }
        if(load) {
            gen_load_mem(value, address, size, is_signed, d->base->mem_idx);
            gen_write_reg(data_reg, value);
        } else {
            gen_read_reg(value, data_reg);
            gen_store_mem(d->base, value, address, size);
        }
        if(!pre) {
            tcg_gen_addi_tl(address, address, size);
            gen_write_reg(base_reg, address);
        }
        tcg_temp_free(value);
        tcg_temp_free(address);
        return true;
    }

    /* ROTR/ROTL #imm, Rn */
    if(b1 >= 0x6c && b1 <= 0x6f) {
        uint8_t b2 = rx_next8(d);
        int count = ((b1 & 1) << 4) | (b2 >> 4);
        TCGv value = tcg_const_tl(count);

        gen_rotate(b2 & 15, value, (b1 & 2) != 0);
        tcg_temp_free(value);
        return true;
    }

    /* Floating-point operation #imm32, Rn */
    if(b1 == 0x72) {
        uint8_t b2 = rx_next8(d);
        int op = b2 >> 4;
        uint32_t immediate;
        int rd = b2 & 15;
        TCGv src, dst;

        if(op > RX_FPU_DIV) {
            return false;
        }
        immediate = rx_next32(d);
        src = tcg_const_tl(immediate);
        dst = tcg_temp_new();
        rx_sync_pc(d);
        gen_read_reg(dst, rd);
        if(op == RX_FPU_CMP) {
            gen_helper_rx_fcmp(cpu_env, dst, src);
        } else {
            TCGv operation = tcg_const_tl(op), result = tcg_temp_new();

            gen_helper_rx_fpu(result, cpu_env, operation, dst, src);
            gen_write_reg(rd, result);
            tcg_temp_free(result);
            tcg_temp_free(operation);
        }
        tcg_temp_free(dst);
        tcg_temp_free(src);
        return true;
    }

    /* SAVE/RSTR are still executed by the interpreter. */
    if(b1 == 0x76) {
        return false;
    }

    /* BN0T #imm:5, Rn */
    if(b1 >= 0xe0) {
        uint8_t b2 = rx_next8(d);

        if((b2 & 0xf0) != 0xf0) {
            return false;
        }
        gen_bit_cond(b1 & 0x1f, 15, b2 & 15);
        return true;
    }

    /* Immediate ALU / STZ / STNZ / MVTC #imm, CR */
    if((b1 & 0xf0) == 0x70) {
        int im = (b1 >> 2) & 3;
        uint8_t b2 = rx_next8(d);

        if((b1 & 3) == 3) {
            int cr = b2 & 15;
            TCGv value;

            if(!control_valid(cr)) {
                return false;
            }
            value = tcg_const_tl(rx_imm(d, im));
            gen_write_control(d->base, cr, value);
            tcg_temp_free(value);
            return true;
        }
        if((b1 & 3) != 0) {
            return false;
        }
        {
            int opn = b2 >> 4;
            int rd = b2 & 15;
            uint32_t immediate;
            TCGv value;

            switch(opn) {
                case 2: case 4: case 5: case 6: case 7: case 8: case 9:
                case 12: case 13: case 14: case 15:
                    break;
                default:
                    return false;
            }
            immediate = rx_imm(d, im);
            value = tcg_const_tl(immediate);
            switch(opn) {
                case 2:
                    gen_carry_alu(rd, value, false);
                    break;
                case 8:
                    gen_division(rd, value, true);
                    break;
                case 9:
                    gen_division(rd, value, false);
                    break;
                default:
                    gen_extended_result(opn, rd, value);
                    break;
            }
            tcg_temp_free(value);
            return true;
        }
    }

    switch(b1) {
        case 0x60:
        case 0x61:
        case 0x62: {
            uint8_t b2 = rx_next8(d);

            gen_shift_reg(b1 - 0x60, b2 >> 4, b2 & 15);
            return true;
        }
        case 0x64:
        case 0x66: {
            uint8_t b2 = rx_next8(d);
            TCGv count = tcg_temp_new();

            gen_read_reg(count, b2 >> 4);
            tcg_gen_andi_tl(count, count, 31);
            gen_rotate(b2 & 15, count, b1 == 0x66);
            tcg_temp_free(count);
            return true;
        }
        case 0x65:
        case 0x67: {
            uint8_t b2 = rx_next8(d);
            TCGv src = tcg_temp_new(), result = tcg_temp_new();

            gen_read_reg(src, b2 >> 4);
            if(b1 == 0x67) {
                tcg_gen_bswap32_tl(result, src);
            } else {
                tcg_gen_shri_tl(result, src, 8);
                tcg_gen_andi_tl(result, result, 0x00ff00ffu);
                tcg_gen_shli_tl(src, src, 8);
                tcg_gen_andi_tl(src, src, 0xff00ff00u);
                tcg_gen_or_tl(result, result, src);
            }
            gen_write_reg(b2 & 15, result);
            tcg_temp_free(result);
            tcg_temp_free(src);
            return true;
        }
        case 0x68:
        case 0x69: {
            uint8_t b2 = rx_next8(d);
            int cr = b2 & 15;
            TCGv value;

            if(!control_valid(cr)) {
                return false;
            }
            value = tcg_temp_new();
            gen_read_reg(value, b2 >> 4);
            gen_write_control(d->base, cr, value);
            tcg_temp_free(value);
            return true;
        }
        case 0x6a:
        case 0x6b: {
            uint8_t b2 = rx_next8(d);
            int cr = b2 >> 4;
            TCGv value;

            if(!control_valid(cr)) {
                return false;
            }
            value = tcg_temp_new();
            gen_read_control(value, cr, d->pc);
            gen_write_reg(b2 & 15, value);
            tcg_temp_free(value);
            return true;
        }
        default:
            if((b1 & 0xe0) == 0x80 || (b1 & 0xe0) == 0xa0 ||
               (b1 & 0xe0) == 0xc0) {
                uint8_t b2 = rx_next8(d);
                int count = ((b1 >> 4) & 1) * 16 + (b1 & 15);

                gen_shift_imm((b1 - 0x80) / 0x20, count, b2 >> 4, b2 & 15);
                return true;
            }
            return false;
    }
}

/* 0xFE: indexed register-indirect MOV/MOVU. */
static bool gen_indexed_mov(RXDecode *d)
{
    uint8_t b1 = rx_next8(d);
    uint8_t b2 = rx_next8(d);
    int mode = b1 >> 6;
    int sz = (b1 >> 4) & 3;
    int isrc = b1 & 15;
    int bsrc = b2 >> 4;
    int rdst = b2 & 15;
    int size;
    TCGv address, index, value;

    if(sz == 3 || mode == 2) {
        return false;
    }
    size = 1 << sz;
    address = tcg_temp_new();
    index = tcg_temp_new();
    value = tcg_temp_new();
    rx_sync_pc(d);
    gen_read_reg(index, isrc);
    gen_read_reg(address, bsrc);
    if(sz != 0) {
        tcg_gen_shli_tl(index, index, sz);
    }
    tcg_gen_add_tl(address, address, index);
    if(mode == 0) {
        gen_read_reg(value, rdst);
        gen_store_mem(d->base, value, address, size);
    } else {
        gen_load_mem(value, address, size, mode == 1, d->base->mem_idx);
        gen_write_reg(rdst, value);
    }
    tcg_temp_free(value);
    tcg_temp_free(index);
    tcg_temp_free(address);
    return true;
}

/* 0xFF: three-register ALU and FPU. */
static bool gen_three_reg(RXDecode *d)
{
    uint8_t b1 = rx_next8(d);
    uint8_t b2 = rx_next8(d);
    int op = b1 >> 4;
    int rdst = b1 & 15;
    int srca = b2 >> 4;
    int srcb = b2 & 15;
    TCGv a, b, result;

    switch(op) {
        case 0: case 2: case 3: case 4: case 5: case 6:
        case 8: case 10: case 11:
            break;
        default:
            return false;
    }
    a = tcg_temp_new();
    b = tcg_temp_new();
    result = tcg_temp_new();
    gen_read_reg(a, srca);
    gen_read_reg(b, srcb);
    if(op == 8 || op == 10 || op == 11) {
        TCGv operation = tcg_const_tl(op - 8);

        rx_sync_pc(d);
        gen_helper_rx_fpu(result, cpu_env, operation, b, a);
        gen_write_reg(rdst, result);
        tcg_temp_free(operation);
    } else {
        gen_alu_values(op, b, a, result);
        gen_write_reg(rdst, result);
    }
    tcg_temp_free(result);
    tcg_temp_free(b);
    tcg_temp_free(a);
    return true;
}

static int gen_rx_insn(DisasContextBase *base)
{
    RXDecode d = { base, base->pc, 0, false };
    uint8_t b0 = rx_next8(&d);

    /* 0x40-0x5F: register/memory (unsigned byte) ALU group. */
    if(b0 >= 0x40 && b0 <= 0x5f) {
        gen_alu_group(&d, b0);
        return d.length;
    }

    /* 0x80-0xBF: MOV/MOVU with a 5-bit scaled displacement. */
    if(b0 >= 0x80 && b0 <= 0xbf) {
        gen_mov_scaled(&d, b0);
        return d.length;
    }

    /* 0xC0-0xEF: MOV group. */
    if(b0 >= 0xc0 && b0 <= 0xef) {
        return gen_mov_group(&d, b0) ? (int)d.length : 0;
    }

    switch(b0) {
        case 0x02:
            gen_rts(&d);
            return d.length;
        case 0x03:
            return d.length;
        case 0x04:
        case 0x08: case 0x09: case 0x0a: case 0x0b:
        case 0x0c: case 0x0d: case 0x0e: case 0x0f:
        case 0x10: case 0x11: case 0x12: case 0x13:
        case 0x14: case 0x15: case 0x16: case 0x17:
        case 0x18: case 0x19: case 0x1a: case 0x1b:
        case 0x1c: case 0x1d: case 0x1e: case 0x1f:
        case 0x20: case 0x21: case 0x22: case 0x23:
        case 0x24: case 0x25: case 0x26: case 0x27:
        case 0x28: case 0x29: case 0x2a: case 0x2b:
        case 0x2c: case 0x2d: case 0x2e: case 0x2f:
        case 0x38: case 0x3a: case 0x3b:
            gen_branch(&d, b0);
            return d.length;
        case 0x05:
            gen_bsr(&d, (int32_t)rx_imm(&d, 3));
            return d.length;
        case 0x39:
            gen_bsr(&d, (int16_t)rx_next16(&d));
            return d.length;
        case 0x3c: case 0x3d: case 0x3e: {
            uint8_t b1 = rx_next8(&d);
            int size = 1 << (b0 - 0x3c);
            uint32_t displacement = (uint32_t)((((b1 >> 3) & 16) | (b1 & 15)) * size);
            uint32_t immediate = rx_next8(&d);
            TCGv address = tcg_temp_new(), value = tcg_const_tl(immediate);

            rx_sync_pc(&d);
            gen_address_reg(address, (b1 >> 4) & 7, displacement);
            gen_store_mem(base, value, address, size);
            tcg_temp_free(value);
            tcg_temp_free(address);
            return d.length;
        }
        case 0x3f:
            return gen_rtsd(&d) ? (int)d.length : 0;
        case 0x06:
            return gen_memex(&d) ? (int)d.length : 0;
        case 0x60: case 0x61: case 0x62: case 0x63:
        case 0x64: case 0x65: case 0x66:
            gen_imm4(&d, b0);
            return d.length;
        case 0x67:
            gen_rtsd_short(&d);
            return d.length;
        case 0x68: case 0x69: case 0x6a: case 0x6b:
        case 0x6c: case 0x6d:
            gen_shift_imm5(&d, b0);
            return d.length;
        case 0x6e:
        case 0x6f:
            return gen_push_pop_m(&d, b0) ? (int)d.length : 0;
        case 0x70: case 0x71: case 0x72: case 0x73:
            gen_add_imm(&d, b0);
            return d.length;
        case 0x74: case 0x75: case 0x76: case 0x77:
            return gen_imm_alu(&d, b0) ? (int)d.length : 0;
        case 0x78: case 0x79: case 0x7a: case 0x7b:
        case 0x7c: case 0x7d:
            gen_bit_imm_group(&d, b0);
            return d.length;
        case 0x7e:
            return gen_unary_group(&d) ? (int)d.length : 0;
        case 0x7f: {
            uint8_t b1 = ldub_code(d.pc + 1);

            if((b1 & 0xf0) == 0x00 || (b1 & 0xf0) == 0x10 ||
               (b1 & 0xf0) == 0x40 || (b1 & 0xf0) == 0x50) {
                return gen_reg_branch(&d) ? (int)d.length : 0;
            }
            if((b1 & 0xe0) == 0xa0) {
                gen_flag(&d, rx_next8(&d));
                return d.length;
            }
            return 0;
        }
        case 0xf0: case 0xf1: case 0xf2: case 0xf3:
        case 0xf4: case 0xf5: case 0xf6: case 0xf7:
            return gen_memory_bit(&d, b0) ? (int)d.length : 0;
        case 0xf8: case 0xf9: case 0xfa: case 0xfb:
            return gen_mov_imm(&d, b0) ? (int)d.length : 0;
        case 0xfc:
            return gen_extended_0xfc(&d) ? (int)d.length : 0;
        case 0xfd:
            return gen_extended_0xfd(&d) ? (int)d.length : 0;
        case 0xfe:
            return gen_indexed_mov(&d) ? (int)d.length : 0;
        case 0xff:
            return gen_three_reg(&d) ? (int)d.length : 0;
        default:
            return 0;
    }
}

void gen_sync_pc(DisasContext *dc)
{
    tcg_gen_movi_tl(cpu_pc, dc->base.pc);
}

void setup_disas_context(DisasContextBase *base, CPUState *env)
{
    base->mem_idx = cpu_mmu_index(env);
    ((DisasContext *)base)->is_v3 = env->is_v3;
}

int gen_intermediate_code(CPUState *env, DisasContextBase *base)
{
    DisasContext *dc = (DisasContext *)base;
    dc->has_store = false;
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

    if(dc->has_store) {
        int done = gen_new_label();
        int no_flush = gen_new_label();
        if(!env->singlestep_enabled && base->is_jmp == DISAS_NEXT) {
            tcg_gen_brcondi_i32(TCG_COND_EQ, dc->store_flags, 0, done);
            tcg_gen_movi_tl(cpu_pc, base->pc);
            TCGv_i32 count = tcg_const_i32(base->tb->icount);
            tcg_gen_st_i32(count, cpu_env, offsetof(CPUState, instructions_count_declaration));
            if(env->block_finished_hook_present) {
                TCGv first = tcg_const_tl(base->tb->pc);
                gen_helper_block_finished_event(first, count);
                tcg_temp_free(first);
            }
            tcg_temp_free_i32(count);
            tcg_gen_brcondi_i32(TCG_COND_LTU, dc->store_flags, 2, no_flush);
            gen_helper_rx_store_flush(cpu_env);
            gen_set_label(no_flush);
            tcg_gen_exit_tb(0);
        } else {
            tcg_gen_brcondi_i32(TCG_COND_LTU, dc->store_flags, 2, done);
            gen_helper_rx_store_flush(cpu_env);
        }
        gen_set_label(done);
        tcg_temp_free_i32(dc->store_flags);
    }

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
