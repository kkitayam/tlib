/*
 * Renesas RX instruction interpreter.
 *
 * The translator emits one call per instruction. The helper decodes the
 * instruction from guest memory and applies its architectural effects. The
 * same decoder is used in a side-effect-free mode during translation to
 * determine the instruction length and control flow.
 */

#include <math.h>
#include <stdint.h>

#include "cpu.h"
#include "helper.h"
#include "arch_callbacks.h"

#include "softmmu_exec.h"

typedef struct {
    CPUState *env;
    uint32_t pc;
    uint32_t off;
} RXReader;

static uint8_t rx_next8(RXReader *r)
{
    return ldub_code(r->pc + r->off++);
}

static uint16_t rx_next16(RXReader *r)
{
    uint16_t v = lduw_code(r->pc + r->off);
    r->off += 2;
    return v;
}

static uint32_t rx_next32(RXReader *r)
{
    uint32_t v = ldl_code(r->pc + r->off);
    r->off += 4;
    return v;
}

static uint32_t rx_get_r(CPUState *env, int n)
{
    return (n == 0) ? rx_get_active_sp(env) : env->regs[n];
}

static void rx_set_r(CPUState *env, int n, uint32_t value)
{
    if(n == 0) {
        rx_set_active_sp(env, value);
    } else {
        env->regs[n] = value;
    }
}

/* Data accesses performed inside helper code need the return address of the
 * generated block: the softmmu slow path stores it in the TLB entry so that a
 * fault can be retried. GETPC() inside a helper sub-function would return an
 * address outside the TCG buffer, so the helper entry point captures it once
 * and the accessors below use the explicit-retaddr variants. */
static __thread void *rx_helper_retaddr;

static inline uint8_t rx_ldub(CPUState *env, uint32_t addr)
{
    if(cpu_mmu_index(env) == MMU_USER_IDX) {
        return ldub_user_inner(addr, rx_helper_retaddr);
    }
    return ldub_kernel_inner(addr, rx_helper_retaddr);
}

static inline uint16_t rx_lduw(CPUState *env, uint32_t addr)
{
    if(cpu_mmu_index(env) == MMU_USER_IDX) {
        return lduw_user_inner(addr, rx_helper_retaddr);
    }
    return lduw_kernel_inner(addr, rx_helper_retaddr);
}

static inline uint32_t rx_ldl(CPUState *env, uint32_t addr)
{
    if(cpu_mmu_index(env) == MMU_USER_IDX) {
        return ldl_user_inner(addr, rx_helper_retaddr);
    }
    return ldl_kernel_inner(addr, rx_helper_retaddr);
}

static inline void rx_stb(CPUState *env, uint32_t addr, uint8_t value)
{
    if(cpu_mmu_index(env) == MMU_USER_IDX) {
        stb_user_inner(addr, value, rx_helper_retaddr);
    } else {
        stb_kernel_inner(addr, value, rx_helper_retaddr);
    }
}

static inline void rx_stw(CPUState *env, uint32_t addr, uint16_t value)
{
    if(cpu_mmu_index(env) == MMU_USER_IDX) {
        stw_user_inner(addr, value, rx_helper_retaddr);
    } else {
        stw_kernel_inner(addr, value, rx_helper_retaddr);
    }
}

static inline void rx_stl(CPUState *env, uint32_t addr, uint32_t value)
{
    if(cpu_mmu_index(env) == MMU_USER_IDX) {
        stl_user_inner(addr, value, rx_helper_retaddr);
    } else {
        stl_kernel_inner(addr, value, rx_helper_retaddr);
    }
}

static uint32_t rx_read_mem(CPUState *env, int ex, uint32_t addr, int size, int is_signed)
{
    if(!ex) {
        return 0;
    }

    switch(size) {
        case 1: {
            uint8_t v = rx_ldub(env, addr);
            return is_signed ? (uint32_t)(int32_t)(int8_t)v : v;
        }
        case 2: {
            uint16_t v = rx_lduw(env, addr);
            return is_signed ? (uint32_t)(int32_t)(int16_t)v : v;
        }
        default:
            return rx_ldl(env, addr);
    }
}

static void rx_write_mem(CPUState *env, int ex, uint32_t addr, int size, uint32_t value)
{
    if(!ex) {
        return;
    }

    switch(size) {
        case 1:
            rx_stb(env, addr, (uint8_t)value);
            break;
        case 2:
            rx_stw(env, addr, (uint16_t)value);
            break;
        default:
            rx_stl(env, addr, value);
            break;
    }
}

static uint32_t rx_apply_size(uint32_t value, int size, int is_signed)
{
    switch(size) {
        case 1:
            return is_signed ? (uint32_t)(int32_t)(int8_t)value : (value & 0xffu);
        case 2:
            return is_signed ? (uint32_t)(int32_t)(int16_t)value : (value & 0xffffu);
        default:
            return value;
    }
}

/* ------------------------------------------------------------------------- */
/* PSW helpers                                                               */
/* ------------------------------------------------------------------------- */

static const uint32_t rx_b2_mask[5] = { 0, 0xff, 0xffff, 0xffffff, 0xffffffffu };
static const uint32_t rx_b2_sign[5] = { 0, 0x80, 0x8000, 0x800000, 0x80000000u };
static const int64_t rx_b2_max[5] = { 0, 0x7f, 0x7fff, 0x7fffff, 0x7fffffff };
static const int64_t rx_b2_min[5] = { 0, -128, -32768, -8388608, INT32_MIN };

static void rx_set_sz(CPUState *env, int ex, int64_t v, int b)
{
    uint32_t f = 0;
    if(((uint64_t)v & rx_b2_mask[b]) == 0) {
        f |= PSW_ZERO;
    }
    if(((uint64_t)v & rx_b2_sign[b]) != 0) {
        f |= PSW_SIGN;
    }
    if(ex) {
        env->psw = (env->psw & ~(PSW_ZERO | PSW_SIGN)) | f;
    }
}

static void rx_set_oszc(CPUState *env, int ex, int64_t v, int b, int c)
{
    uint32_t f = 0;
    if(c) {
        f |= PSW_CARRY;
    }
    if(((uint64_t)v & rx_b2_mask[b]) == 0) {
        f |= PSW_ZERO;
    }
    if(((uint64_t)v & rx_b2_sign[b]) != 0) {
        f |= PSW_SIGN;
    }
    if(v > rx_b2_max[b] || v < rx_b2_min[b]) {
        f |= PSW_OVERFLOW;
    }
    if(ex) {
        env->psw = (env->psw & ~(PSW_CARRY | PSW_ZERO | PSW_SIGN | PSW_OVERFLOW)) | f;
    }
}

static void rx_set_szc(CPUState *env, int ex, int64_t v, int b, int c)
{
    uint32_t f = 0;
    if(c) {
        f |= PSW_CARRY;
    }
    if(((uint64_t)v & rx_b2_mask[b]) == 0) {
        f |= PSW_ZERO;
    }
    if(((uint64_t)v & rx_b2_sign[b]) != 0) {
        f |= PSW_SIGN;
    }
    if(ex) {
        env->psw = (env->psw & ~(PSW_CARRY | PSW_ZERO | PSW_SIGN)) | f;
    }
}

static void rx_set_osz(CPUState *env, int ex, int64_t v, int b)
{
    uint32_t f = 0;
    if(((uint64_t)v & rx_b2_mask[b]) == 0) {
        f |= PSW_ZERO;
    }
    if(((uint64_t)v & rx_b2_sign[b]) != 0) {
        f |= PSW_SIGN;
    }
    if(v > rx_b2_max[b] || v < rx_b2_min[b]) {
        f |= PSW_OVERFLOW;
    }
    if(ex) {
        env->psw = (env->psw & ~(PSW_ZERO | PSW_SIGN | PSW_OVERFLOW)) | f;
    }
}

static void rx_set_zc(CPUState *env, int ex, int z, int c)
{
    if(ex) {
        env->psw = (env->psw & ~(PSW_CARRY | PSW_ZERO)) | (c ? PSW_CARRY : 0) | (z ? PSW_ZERO : 0);
    }
}

static int rx_condition_true(CPUState *env, int cond)
{
    int c = (env->psw & PSW_CARRY) != 0;
    int z = (env->psw & PSW_ZERO) != 0;
    int s = (env->psw & PSW_SIGN) != 0;
    int o = (env->psw & PSW_OVERFLOW) != 0;

    switch(cond & 15) {
        case 0: return z;
        case 1: return !z;
        case 2: return c;
        case 3: return !c;
        case 4: return c && !z;
        case 5: return !(c && !z);
        case 6: return !s;
        case 7: return s;
        case 8: return !(s ^ o);
        case 9: return s ^ o;
        case 10: return !((s ^ o) || z);
        case 11: return (s ^ o) || z;
        case 12: return o;
        case 13: return !o;
        case 14: return 1;
        default: return 0;
    }
}

static uint32_t rx_do_add(CPUState *env, int ex, uint32_t dst, uint32_t src, int cin)
{
    uint64_t ll = (uint64_t)dst + src + (cin ? 1u : 0u);
    int64_t sll = (int64_t)(int32_t)dst + (int32_t)src + (cin ? 1 : 0);
    rx_set_oszc(env, ex, sll, 4, ll > 0xffffffffu);
    return (uint32_t)ll;
}

static uint32_t rx_do_sub(CPUState *env, int ex, uint32_t dst, uint32_t src, int borrow_in)
{
    uint64_t ll = (uint64_t)dst - src - (borrow_in ? 1u : 0u);
    int64_t sll = (int64_t)(int32_t)dst - (int32_t)src - (borrow_in ? 1 : 0);
    rx_set_oszc(env, ex, sll, 4, (int64_t)ll >= 0);
    return (uint32_t)ll;
}

static void rx_do_div(CPUState *env, int ex, int rd, uint32_t src, int is_signed)
{
    uint32_t dst = rx_get_r(env, rd);

    if(is_signed) {
        int32_t s = (int32_t)src;
        int32_t d = (int32_t)dst;
        if(s == 0 || (s == -1 && d == INT32_MIN)) {
            if(ex) {
                env->psw |= PSW_OVERFLOW;
            }
            return;
        }
        if(ex) {
            env->psw &= ~PSW_OVERFLOW;
            rx_set_r(env, rd, (uint32_t)(d / s));
        }
    } else {
        if(src == 0) {
            if(ex) {
                env->psw |= PSW_OVERFLOW;
            }
            return;
        }
        if(ex) {
            env->psw &= ~PSW_OVERFLOW;
            rx_set_r(env, rd, dst / src);
        }
    }
}

static void rx_do_shift(CPUState *env, int ex, int kind, int count, int src_reg, int dst_reg)
{
    uint32_t src = rx_get_r(env, src_reg);
    int64_t val = (kind == 0) ? (int64_t)(uint64_t)src : (int64_t)(int32_t)src;
    int carry = 0;

    for(int i = 0; i < count; i++) {
        carry = (kind == 2) ? ((val & 0x80000000LL) != 0) : ((val & 1LL) != 0);
        if(kind == 2) {
            val <<= 1;
        } else if(kind == 1) {
            val >>= 1;
        } else {
            val = (int64_t)((uint64_t)val >> 1);
        }
    }

    if(kind == 2) {
        rx_set_oszc(env, ex, val, 4, carry);
    } else {
        if(kind == 1 && ex) {
            env->psw &= ~PSW_OVERFLOW;
        }
        rx_set_szc(env, ex, val, 4, carry);
    }
    if(ex) {
        rx_set_r(env, dst_reg, (uint32_t)val);
    }
}

static float rx_bits_to_float(uint32_t bits)
{
    union {
        uint32_t u;
        float f;
    } cvt;
    cvt.u = bits;
    return cvt.f;
}

static uint32_t rx_float_to_bits(float value)
{
    union {
        uint32_t u;
        float f;
    } cvt;
    cvt.f = value;
    return cvt.u;
}

static void rx_set_fpsw_flags(CPUState *env, int ex, int z, int s)
{
    uint32_t fpsw;

    if(!ex) {
        return;
    }
    fpsw = env->fpsw & ~(0x02u | 0x04u);
    if(z) {
        fpsw |= 0x02u;
    }
    if(s) {
        fpsw |= 0x04u;
    }
    env->fpsw = fpsw;
}

static void rx_do_fcmp(CPUState *env, int ex, uint32_t a, uint32_t b)
{
    float fa = rx_bits_to_float(a);
    float fb = rx_bits_to_float(b);
    int unordered = (fa != fa) || (fb != fb);

    rx_set_fpsw_flags(env, ex, !unordered && fa == fb, !unordered && fa < fb);
    if(unordered && ex) {
        env->fpsw |= 0x40u;
    }
}

/* ------------------------------------------------------------------------- */
/* Stack, control registers                                                  */
/* ------------------------------------------------------------------------- */

static void rx_push(CPUState *env, int ex, uint32_t value)
{
    uint32_t sp;

    if(!ex) {
        return;
    }
    sp = rx_get_active_sp(env) - 4;
    rx_set_active_sp(env, sp);
    rx_stl(env, sp, value);
}

static void rx_push_sized(CPUState *env, int ex, uint32_t value, int size)
{
    uint32_t sp;

    if(!ex) {
        return;
    }
    sp = rx_get_active_sp(env) - (uint32_t)size;
    rx_set_active_sp(env, sp);
    rx_write_mem(env, ex, sp, size, value);
}

static uint32_t rx_pop(CPUState *env, int ex)
{
    uint32_t sp, value;

    if(!ex) {
        return 0;
    }
    sp = rx_get_active_sp(env);
    value = rx_ldl(env, sp);
    rx_set_active_sp(env, sp + 4);
    return value;
}

static int rx_get_control_register(CPUState *env, int cr, uint32_t *value)
{
    switch(cr & 0xf) {
        case 0x0: *value = env->psw; return 1;
        case 0x1: *value = env->pc; return 1;
        case 0x2: *value = env->usp; return 1;
        case 0x3: *value = env->fpsw; return 1;
        case 0x4: *value = env->cpen; return 1;
        case 0x8: *value = env->bpsw; return 1;
        case 0x9: *value = env->bpc; return 1;
        case 0xa: *value = env->isp; return 1;
        case 0xb: *value = env->fintv; return 1;
        case 0xc: *value = env->intb; return 1;
        default: *value = 0; return 0;
    }
}

static int rx_set_control_register(CPUState *env, int cr, uint32_t value)
{
    switch(cr & 0xf) {
        case 0x0: {
            uint32_t pm = env->psw & PSW_PM;
            value &= ~PSW_PM;
            value |= pm;
            if(pm != 0) {
                value &= ~(PSW_I | PSW_U | PSW_IPL_MASK);
                value |= pm;
            }
            env->psw = value;
            return 1;
        }
        case 0x1: env->pc = value; return 1;
        case 0x2: env->usp = value; return 1;
        case 0x3: env->fpsw = value; return 1;
        case 0x4: env->cpen = value; return 1;
        case 0x8: env->bpsw = value; return 1;
        case 0x9: env->bpc = value; return 1;
        case 0xa: env->isp = value; return 1;
        case 0xb: env->fintv = value; return 1;
        case 0xc: env->intb = value; return 1;
        default: return 0;
    }
}

/* ------------------------------------------------------------------------- */
/* Accumulator / RXv3 helpers                                                */
/* ------------------------------------------------------------------------- */

static inline __int128 rx_sext72(__int128 value)
{
    return (value << 56) >> 56;
}

static __int128 *rx_acc_ptr(CPUState *env, int select)
{
    return select ? &env->acc1 : &env->acc0;
}

static void rx_set_acc(CPUState *env, int ex, int select, __int128 value)
{
    if(ex) {
        *rx_acc_ptr(env, select) = rx_sext72(value);
    }
}

static __int128 rx_get_acc(CPUState *env, int select)
{
    return *rx_acc_ptr(env, select);
}

/* ------------------------------------------------------------------------- */
/* String instructions                                                       */
/* ------------------------------------------------------------------------- */

static void rx_do_scmpu(CPUState *env, int ex)
{
    uint32_t r1, r2, count;
    uint32_t uma = 0, umb = 0;

    if(!ex) {
        return;
    }
    r1 = rx_get_r(env, 1);
    r2 = rx_get_r(env, 2);
    count = rx_get_r(env, 3);

    while(count != 0) {
        uma = rx_ldub(env, r1++);
        umb = rx_ldub(env, r2++);
        count--;
        if(uma != umb || uma == 0) {
            break;
        }
    }
    rx_set_r(env, 1, r1);
    rx_set_r(env, 2, r2);
    rx_set_r(env, 3, count);
    if(uma == umb) {
        rx_set_zc(env, ex, 1, 1);
    } else {
        rx_set_zc(env, ex, 0, (int32_t)uma - (int32_t)umb >= 0);
    }
}

static void rx_do_smovu(CPUState *env, int ex)
{
    uint32_t r1, r2, count;

    if(!ex) {
        return;
    }
    r1 = rx_get_r(env, 1);
    r2 = rx_get_r(env, 2);
    count = rx_get_r(env, 3);

    while(count != 0) {
        uint8_t b = rx_ldub(env, r2++);
        rx_stb(env, r1++, b);
        count--;
        if(b == 0) {
            break;
        }
    }
    rx_set_r(env, 1, r1);
    rx_set_r(env, 2, r2);
    rx_set_r(env, 3, count);
}

static void rx_do_smovb(CPUState *env, int ex)
{
    uint32_t r1, r2, count;

    if(!ex) {
        return;
    }
    r1 = rx_get_r(env, 1);
    r2 = rx_get_r(env, 2);
    count = rx_get_r(env, 3);

    while(count != 0) {
        uint8_t b = rx_ldub(env, r2--);
        rx_stb(env, r1--, b);
        count--;
    }
    rx_set_r(env, 1, r1);
    rx_set_r(env, 2, r2);
    rx_set_r(env, 3, count);
}

static void rx_do_smovf(CPUState *env, int ex)
{
    uint32_t r1, r2, count;

    if(!ex) {
        return;
    }
    r1 = rx_get_r(env, 1);
    r2 = rx_get_r(env, 2);
    count = rx_get_r(env, 3);

    while(count != 0) {
        uint8_t b = rx_ldub(env, r2++);
        rx_stb(env, r1++, b);
        count--;
    }
    rx_set_r(env, 1, r1);
    rx_set_r(env, 2, r2);
    rx_set_r(env, 3, count);
}

static void rx_do_sstr(CPUState *env, int ex, int sz)
{
    uint32_t r1, r2, count;
    int size = 1 << sz;

    if(!ex) {
        return;
    }
    r1 = rx_get_r(env, 1);
    r2 = rx_get_r(env, 2);
    count = rx_get_r(env, 3);

    while(count != 0) {
        rx_write_mem(env, ex, r1, size, r2);
        r1 += (uint32_t)size;
        count--;
    }
    rx_set_r(env, 1, r1);
    rx_set_r(env, 3, count);
}

static void rx_do_suntil(CPUState *env, int ex, int sz, int while_not_equal)
{
    uint32_t r1, count, uma, umb = 0;
    int size = 1 << sz;

    if(!ex) {
        return;
    }
    r1 = rx_get_r(env, 1);
    count = rx_get_r(env, 3);
    uma = rx_get_r(env, 2) & ((size == 1) ? 0xffu : ((size == 2) ? 0xffffu : 0xffffffffu));

    while(count != 0) {
        count--;
        umb = rx_read_mem(env, ex, r1, size, 0);
        r1 += (uint32_t)size;
        if(while_not_equal ? (umb != uma) : (umb == uma)) {
            break;
        }
    }
    rx_set_r(env, 1, r1);
    rx_set_r(env, 3, count);
    if(uma == umb) {
        rx_set_zc(env, ex, 1, 1);
    } else {
        rx_set_zc(env, ex, 0, (int32_t)uma - (int32_t)umb >= 0);
    }
}

static void rx_do_rmpa(CPUState *env, int ex, int sz)
{
    uint32_t r1, r2, count, r4, r5, r6;
    int size = 1 << sz;

    if(!ex) {
        return;
    }
    r1 = rx_get_r(env, 1);
    r2 = rx_get_r(env, 2);
    count = rx_get_r(env, 3);
    r4 = rx_get_r(env, 4);
    r5 = rx_get_r(env, 5);
    r6 = rx_get_r(env, 6);

    while(count != 0) {
        int32_t ma = (int32_t)rx_read_mem(env, ex, r1, size, 1);
        int32_t mb = (int32_t)rx_read_mem(env, ex, r2, size, 1);
        int64_t sll;
        uint64_t tmp;

        r1 += (uint32_t)size;
        r2 += (uint32_t)size;
        sll = (int64_t)ma * (int64_t)mb;
        tmp = (uint64_t)r4 + (uint64_t)(uint32_t)sll;
        r4 = (uint32_t)tmp;
        tmp >>= 32;
        tmp += (uint64_t)r5 + (uint64_t)(uint32_t)(sll >> 32);
        r5 = (uint32_t)tmp;
        tmp >>= 32;
        tmp += (uint64_t)r6 + (uint64_t)(uint32_t)(sll >> 32);
        r6 = (uint32_t)tmp;
        count--;
    }

    rx_set_r(env, 1, r1);
    rx_set_r(env, 2, r2);
    rx_set_r(env, 3, count);
    rx_set_r(env, 4, r4);
    rx_set_r(env, 5, r5);
    if(r6 & 0x00008000u) {
        r6 |= 0xffff0000u;
    } else {
        r6 &= 0x0000ffffu;
    }
    rx_set_r(env, 6, r6);

    uint32_t s = (r6 & 0x80000000u) ? PSW_SIGN : 0;
    if(r6 != 0 && r6 != 0xffffffffu) {
        env->psw = (env->psw & ~(PSW_SIGN | PSW_OVERFLOW)) | s | PSW_OVERFLOW;
    } else {
        env->psw = (env->psw & ~(PSW_SIGN | PSW_OVERFLOW)) | s;
    }
}

static void rx_do_satr(CPUState *env, int ex)
{
    if(!ex) {
        return;
    }
    if((env->psw & PSW_OVERFLOW) && !(env->psw & PSW_SIGN)) {
        rx_set_r(env, 6, 0);
        rx_set_r(env, 5, 0x7fffffffu);
        rx_set_r(env, 4, 0xffffffffu);
    } else if((env->psw & PSW_OVERFLOW) && (env->psw & PSW_SIGN)) {
        rx_set_r(env, 6, 0xffffffffu);
        rx_set_r(env, 5, 0x80000000u);
        rx_set_r(env, 4, 0);
    }
}

static void rx_do_save(CPUState *env, int bank)
{
    int i;

    bank &= 0xf;
    for(i = 1; i <= 15; i++) {
        env->banks[bank].regs[i - 1] = rx_get_r(env, i);
    }
    env->banks[bank].usp = env->usp;
    env->banks[bank].fpsw = env->fpsw;
    env->banks[bank].acc0 = env->acc0;
    env->banks[bank].acc1 = env->acc1;
    env->banks[bank].valid = true;
}

static int rx_do_restore(CPUState *env, int bank)
{
    int i;

    bank &= 0xf;
    if(!env->banks[bank].valid) {
        return 0;
    }
    for(i = 1; i <= 15; i++) {
        rx_set_r(env, i, env->banks[bank].regs[i - 1]);
    }
    env->usp = env->banks[bank].usp;
    env->fpsw = env->banks[bank].fpsw;
    env->acc0 = env->banks[bank].acc0;
    env->acc1 = env->banks[bank].acc1;
    return 1;
}

/* ------------------------------------------------------------------------- */
/* Decoder                                                                   */
/* ------------------------------------------------------------------------- */

static int64_t rx_read_imm(RXReader *r, int im)
{
    switch(im) {
        case 0:
            return (int32_t)rx_next32(r);
        case 1:
            return (int8_t)rx_next8(r);
        case 2:
            return (int16_t)rx_next16(r);
        default: {
            uint32_t v = (uint32_t)rx_next8(r) | ((uint32_t)rx_next8(r) << 8) | ((uint32_t)rx_next8(r) << 16);
            return (int32_t)(v << 8) >> 8;
        }
    }
}

static uint32_t rx_disp_address(RXReader *r, int type, int reg, int size)
{
    switch(type) {
        case 0:
            return rx_get_r(r->env, reg);
        case 1:
            return rx_get_r(r->env, reg) + (uint32_t)((uint32_t)rx_next8(r) * (uint32_t)size);
        case 2:
            return rx_get_r(r->env, reg) + (uint32_t)((uint32_t)rx_next16(r) * (uint32_t)size);
        default:
            return 0;
    }
}

static uint32_t rx_disp_load(RXReader *r, int ex, int type, int reg, int size, int is_signed)
{
    if(type == 3) {
        return rx_apply_size(rx_get_r(r->env, reg), size, is_signed);
    }
    return rx_read_mem(r->env, ex, rx_disp_address(r, type, reg, size), size, is_signed);
}

static void rx_disp_store(RXReader *r, int ex, int type, int reg, int size, uint32_t value, int is_signed)
{
    uint32_t v = rx_apply_size(value, size, is_signed);

    if(type == 3) {
        if(ex) {
            rx_set_r(r->env, reg, v);
        }
        return;
    }
    rx_write_mem(r->env, ex, rx_disp_address(r, type, reg, size), size, v);
}

static int rx_memex_size(int mx)
{
    return (mx == 2) ? 4 : ((mx == 0) ? 1 : 2);
}

static int rx_memex_signed(int mx)
{
    return mx != 3;
}

/*
 * Execute (ex != 0) or just measure (ex == 0) one instruction located at pc.
 * Returns the instruction length, or -1 when the encoding is not supported.
 */
static int rx_execute(CPUState *env, uint32_t pc, int ex)
{
    RXReader r = { env, pc, 0 };
    uint8_t b0 = rx_next8(&r);
    uint32_t jump = 0;
    int has_jump = 0;
    int result = 0;
    static const int dsp3_map[8] = { 8, 9, 10, 3, 4, 5, 6, 7 };

#define GETR(n) rx_get_r(env, (n))
#define SETR(n, v)                 \
    do {                           \
        if(ex) {                   \
            rx_set_r(env, (n), (v)); \
        }                          \
    } while(0)
#define SET_JUMP(v) \
    do {            \
        jump = (v); \
        has_jump = 1; \
    } while(0)

    /* 0x40-0x5F: register/memory (unsigned byte) ALU group. */
    if(b0 >= 0x40 && b0 <= 0x5f) {
        uint8_t b1 = rx_next8(&r);
        int rsrc = (b1 >> 4) & 0xf;
        int rdst = b1 & 0xf;
        int ooo = (b0 >> 2) & 7;
        int ss = b0 & 3;

        if(ooo >= 6) {
            int size = (ooo == 6) ? 1 : 2;
            rx_disp_store(&r, ex, 3, rdst, size, rx_disp_load(&r, ex, ss, rsrc, size, 0), 0);
        } else {
            int size = (ss == 3) ? 4 : 1;
            uint32_t src = rx_disp_load(&r, ex, ss, rsrc, size, ss == 3);
            uint32_t dst = GETR(rdst);
            switch(ooo) {
                case 0: SETR(rdst, rx_do_sub(env, ex, dst, src, 0)); break;
                case 1: rx_do_sub(env, ex, dst, src, 0); break;
                case 2: SETR(rdst, rx_do_add(env, ex, dst, src, 0)); break;
                case 3: SETR(rdst, dst * src); break;
                case 4: {
                    uint32_t v = dst & src;
                    SETR(rdst, v);
                    rx_set_sz(env, ex, v, 4);
                    break;
                }
                default: {
                    uint32_t v = dst | src;
                    SETR(rdst, v);
                    rx_set_sz(env, ex, v, 4);
                    break;
                }
            }
        }
        result = (int)r.off;
        goto done;
    }

    /* 0x80-0xBF: MOV/MOVU with a 5-bit scaled displacement. */
    if(b0 >= 0x80 && b0 <= 0xbf) {
        uint8_t b1 = rx_next8(&r);
        int size_kind = (b0 >> 4) & 7;
        int n = ((b0 & 7) << 2) | ((b1 >> 6) & 2) | ((b1 >> 3) & 1);
        int base_reg = (b1 >> 4) & 7;
        int reg = b1 & 7;
        int movu = (size_kind == 3);
        int size = movu ? (((b0 & 8) != 0) ? 2 : 1) : (1 << size_kind);
        int load = movu || ((b0 & 8) != 0);
        uint32_t addr = GETR(base_reg) + (uint32_t)(n * size);

        if(load) {
            rx_disp_store(&r, ex, 3, reg, size, rx_read_mem(env, ex, addr, size, !movu), 0);
        } else {
            rx_write_mem(env, ex, addr, size, rx_disp_load(&r, ex, 3, reg, size, 1));
        }
        result = (int)r.off;
        goto done;
    }

    /* 0xC0-0xEF: MOV group. */
    if(b0 >= 0xc0 && b0 <= 0xef) {
        int sz = (b0 >> 4) & 3;
        int sd = (b0 >> 2) & 3;
        int ss = b0 & 3;
        uint8_t b1 = rx_next8(&r);
        int rsrc = (b1 >> 4) & 0xf;
        int rdst = b1 & 0xf;

        if(sz == 3) {
            result = -1;
            goto done;
        }

        int size = 1 << sz;
        if(ss == 3 && sd != 3) {
            rx_disp_store(&r, ex, sd, rsrc, size, GETR(rdst), 1);
        } else {
            rx_disp_store(&r, ex, sd, rdst, size, rx_disp_load(&r, ex, ss, rsrc, size, 1), 1);
        }
        result = (int)r.off;
        goto done;
    }

    switch(b0) {
        case 0x00: { /* BRK */
            uint32_t old_psw;
            if(!ex) {
                break;
            }
            if(env->intb == 0) {
                result = -1;
                goto done;
            }
            old_psw = env->psw;
            env->psw = old_psw & ~(PSW_I | PSW_U | PSW_PM);
            rx_push(env, ex, old_psw);
            rx_push(env, ex, pc + 1);
            SET_JUMP(rx_ldl(env, env->intb));
            break;
        }

        case 0x02: /* RTS */
            SET_JUMP(rx_pop(env, ex));
            break;

        case 0x03: /* NOP */
            break;

        case 0x04: /* BRA.A */
            SET_JUMP((uint32_t)((int32_t)pc + (int32_t)rx_read_imm(&r, 3)));
            break;

        case 0x05: { /* BSR.A */
            uint32_t target = (uint32_t)((int32_t)pc + (int32_t)rx_read_imm(&r, 3));
            rx_push(env, ex, pc + r.off);
            SET_JUMP(target);
            break;
        }

        case 0x08: case 0x09: case 0x0a: case 0x0b:
        case 0x0c: case 0x0d: case 0x0e: case 0x0f: /* BRA.S */
            SET_JUMP((uint32_t)((int32_t)pc + dsp3_map[b0 & 7]));
            break;

        case 0x10: case 0x11: case 0x12: case 0x13:
        case 0x14: case 0x15: case 0x16: case 0x17: /* BEQ.S */
        case 0x18: case 0x19: case 0x1a: case 0x1b:
        case 0x1c: case 0x1d: case 0x1e: case 0x1f: /* BNE.S */
            if(rx_condition_true(env, (b0 >> 3) & 1)) {
                SET_JUMP((uint32_t)((int32_t)pc + dsp3_map[b0 & 7]));
            }
            break;

        case 0x20: case 0x21: case 0x22: case 0x23:
        case 0x24: case 0x25: case 0x26: case 0x27:
        case 0x28: case 0x29: case 0x2a: case 0x2b:
        case 0x2c: case 0x2d: case 0x2e: case 0x2f: { /* B<cond>.B */
            int cond = b0 & 0xf;
            int8_t disp = (int8_t)rx_next8(&r);
            if(rx_condition_true(env, cond)) {
                SET_JUMP((uint32_t)((int32_t)pc + disp));
            }
            break;
        }

        case 0x38: /* BRA.W */
            SET_JUMP((uint32_t)((int32_t)pc + (int16_t)rx_next16(&r)));
            break;

        case 0x39: { /* BSR.W */
            int16_t disp = (int16_t)rx_next16(&r);
            rx_push(env, ex, pc + r.off);
            SET_JUMP((uint32_t)((int32_t)pc + disp));
            break;
        }

        case 0x3a: { /* BEQ.W */
            int16_t disp = (int16_t)rx_next16(&r);
            if(rx_condition_true(env, 0)) {
                SET_JUMP((uint32_t)((int32_t)pc + disp));
            }
            break;
        }

        case 0x3b: { /* BNE.W */
            int16_t disp = (int16_t)rx_next16(&r);
            if(rx_condition_true(env, 1)) {
                SET_JUMP((uint32_t)((int32_t)pc + disp));
            }
            break;
        }

        case 0x3c: case 0x3d: case 0x3e: { /* MOV #imm8, dsp5[Rn] */
            int size = 1 << (b0 - 0x3c);
            uint8_t b1 = rx_next8(&r);
            int d = (b1 >> 7) & 1;
            int dst = (b1 >> 4) & 7;
            int sppp = b1 & 0xf;
            uint32_t disp = (uint32_t)(((d << 4) | sppp) * size);
            uint8_t imm = rx_next8(&r);
            rx_write_mem(env, ex, GETR(dst) + disp, size, imm);
            break;
        }

        case 0x3f: { /* RTSD */
            uint8_t b1 = rx_next8(&r);
            int first = b1 >> 4;
            int last = b1 & 0xf;
            uint32_t disp = (uint32_t)(rx_next8(&r) * 4);
            int count;
            int i;
            if(first == 0) {
                result = -1;
                goto done;
            }
            count = last - first + 1;
            if(ex) {
                rx_set_r(env, 0, (uint32_t)((int32_t)rx_get_r(env, 0) + (int32_t)disp - count * 4));
                for(i = first; i <= last; i++) {
                    rx_set_r(env, i, rx_pop(env, ex));
                }
                SET_JUMP(rx_pop(env, ex));
            }
            break;
        }

        case 0x01: /* DBT */
        case 0x07:
            result = -1;
            goto done;

        case 0x06: { /* MEMEX group */
            uint8_t b1 = rx_next8(&r);
            int mx = (b1 >> 6) & 3;
            int grp = (b1 >> 4) & 3;
            int op = (b1 >> 2) & 3;
            int ss = b1 & 3;

            if(grp == 2) {
                uint8_t op2 = rx_next8(&r);
                uint8_t b3 = rx_next8(&r);
                int rsrc = (b3 >> 4) & 0xf;
                int rdst = b3 & 0xf;
                int msize = rx_memex_size(mx);
                int msigned = rx_memex_signed(mx);
                uint32_t maddr = 0;
                uint32_t src;
                uint32_t dst = GETR(rdst);

                if(ss == 3) {
                    src = rx_apply_size(GETR(rsrc), msize, msigned);
                } else {
                    maddr = rx_disp_address(&r, ss, rsrc, msize);
                    src = rx_read_mem(env, ex, maddr, msize, msigned);
                }

                switch(op2) {
                    case 0x00:
                        SETR(rdst, rx_do_sub(env, ex, dst, src, !(env->psw & PSW_CARRY)));
                        break;
                    case 0x02:
                        SETR(rdst, rx_do_add(env, ex, dst, src, (env->psw & PSW_CARRY) != 0));
                        break;
                    case 0x04: /* max */
                        SETR(rdst, ((int32_t)dst > (int32_t)src) ? dst : src);
                        break;
                    case 0x05: /* min */
                        SETR(rdst, ((int32_t)dst < (int32_t)src) ? dst : src);
                        break;
                    case 0x06: { /* emul */
                        int64_t p = (int64_t)(int32_t)dst * (int64_t)(int32_t)src;
                        SETR(rdst, (uint32_t)p);
                        SETR((rdst + 1) & 0xf, (uint32_t)((uint64_t)p >> 32));
                        break;
                    }
                    case 0x07: { /* emulu */
                        uint64_t p = (uint64_t)dst * (uint64_t)src;
                        SETR(rdst, (uint32_t)p);
                        SETR((rdst + 1) & 0xf, (uint32_t)(p >> 32));
                        break;
                    }
                    case 0x08:
                        rx_do_div(env, ex, rdst, src, 1);
                        break;
                    case 0x09:
                        rx_do_div(env, ex, rdst, src, 0);
                        break;
                    case 0x0c: {
                        uint32_t v = dst & src;
                        rx_set_sz(env, ex, v, 4);
                        break;
                    }
                    case 0x0d: {
                        uint32_t v = dst ^ src;
                        SETR(rdst, v);
                        rx_set_sz(env, ex, v, 4);
                        break;
                    }
                    case 0x10: { /* xchg */
                        if(ss == 3) {
                            SETR(rsrc, dst);
                            SETR(rdst, src);
                        } else {
                            rx_write_mem(env, ex, maddr, msize, dst);
                            SETR(rdst, src);
                        }
                        break;
                    }
                    case 0x11:
                        SETR(rdst, rx_float_to_bits((float)(int32_t)src));
                        break;
                    case 0x15:
                        SETR(rdst, rx_float_to_bits((float)src));
                        break;
                    default:
                        result = -1;
                        goto done;
                }
                break;
            }

            if(grp == 3) {
                result = -1;
                goto done;
            }

            {
                uint8_t b2 = rx_next8(&r);
                int rs = (b2 >> 4) & 0xf;
                int rd = b2 & 0xf;
                uint32_t mv = rx_disp_load(&r, ex, ss, rs, rx_memex_size(mx), rx_memex_signed(mx));
                uint32_t dv = GETR(rd);

                if(grp == 0) {
                    switch(op) {
                        case 0: SETR(rd, rx_do_sub(env, ex, dv, mv, 0)); break;
                        case 1: rx_do_sub(env, ex, dv, mv, 0); break;
                        case 2: SETR(rd, rx_do_add(env, ex, dv, mv, 0)); break;
                        default: SETR(rd, dv * mv); break;
                    }
                } else {
                    if(op == 0) {
                        uint32_t v = dv & mv;
                        SETR(rd, v);
                        rx_set_sz(env, ex, v, 4);
                    } else if(op == 1) {
                        uint32_t v = dv | mv;
                        SETR(rd, v);
                        rx_set_sz(env, ex, v, 4);
                    } else {
                        result = -1;
                        goto done;
                    }
                }
            }
            break;
        }

        case 0x60: case 0x61: case 0x62: case 0x63:
        case 0x64: case 0x65: case 0x66: { /* ALU/MOV #imm4 */
            uint8_t b1 = rx_next8(&r);
            uint32_t imm = b1 >> 4;
            int rd = b1 & 0xf;
            uint32_t dst = GETR(rd);

            switch(b0) {
                case 0x60: SETR(rd, rx_do_sub(env, ex, dst, imm, 0)); break;
                case 0x61: rx_do_sub(env, ex, dst, imm, 0); break;
                case 0x62: SETR(rd, rx_do_add(env, ex, dst, imm, 0)); break;
                case 0x63: SETR(rd, dst * imm); break;
                case 0x64: {
                    uint32_t v = dst & imm;
                    SETR(rd, v);
                    rx_set_sz(env, ex, v, 4);
                    break;
                }
                case 0x65: {
                    uint32_t v = dst | imm;
                    SETR(rd, v);
                    rx_set_sz(env, ex, v, 4);
                    break;
                }
                default:
                    SETR(rd, imm);
                    break;
            }
            break;
        }

        case 0x67: { /* RTSD without register list */
            uint32_t disp = (uint32_t)(rx_next8(&r) * 4);
            if(ex) {
                rx_set_r(env, 0, rx_get_r(env, 0) + disp);
                SET_JUMP(rx_pop(env, ex));
            }
            break;
        }

        case 0x68: case 0x69: case 0x6a: case 0x6b:
        case 0x6c: case 0x6d: { /* SHLR/SHAR/SHLL #imm5, Rn */
            uint8_t b1 = rx_next8(&r);
            int count = ((b0 & 1) << 4) | ((b1 >> 4) & 0xf);
            int rd = b1 & 0xf;
            rx_do_shift(env, ex, (b0 - 0x68) / 2, count, rd, rd);
            break;
        }

        case 0x6e: { /* PUSHM */
            uint8_t b1 = rx_next8(&r);
            int first = (b1 >> 4) & 0xf;
            int last = b1 & 0xf;
            if(first == 0 || last == 0 || first >= last) {
                result = -1;
                goto done;
            }
            for(int i = last; i >= first; i--) {
                rx_push(env, ex, GETR(i));
            }
            break;
        }

        case 0x6f: { /* POPM */
            uint8_t b1 = rx_next8(&r);
            int first = (b1 >> 4) & 0xf;
            int last = b1 & 0xf;
            if(first == 0 || last == 0 || first >= last) {
                result = -1;
                goto done;
            }
            for(int i = first; i <= last; i++) {
                SETR(i, rx_pop(env, ex));
            }
            break;
        }

        case 0x70: case 0x71: case 0x72: case 0x73: { /* ADD #imm, Rsrc, Rdst */
            uint8_t b1 = rx_next8(&r);
            uint32_t imm = (uint32_t)rx_read_imm(&r, b0 & 3);
            SETR(b1 & 0xf, rx_do_add(env, ex, GETR((b1 >> 4) & 0xf), imm, 0));
            break;
        }

        case 0x74: case 0x75: case 0x76: case 0x77: { /* CMP/MUL/AND/OR #imm, Rn */
            uint8_t b1 = rx_next8(&r);
            int opsel = (b1 >> 4) & 0xf;
            int rd = b1 & 0xf;

            if(b0 == 0x75 && (opsel == 0x4 || opsel == 0x5)) {
                uint32_t imm8 = rx_next8(&r);
                if(opsel == 0x4) {
                    SETR(rd, imm8);
                } else {
                    rx_do_sub(env, ex, GETR(rd), imm8, 0);
                }
                break;
            }

            if(b0 == 0x75 && opsel == 0x6) { /* INT #imm8 */
                uint32_t vector = rx_next8(&r);
                uint32_t old_psw;
                if(!ex) {
                    break;
                }
                if(env->intb == 0) {
                    result = -1;
                    goto done;
                }
                old_psw = env->psw;
                env->psw = old_psw & ~(PSW_I | PSW_U | PSW_PM);
                rx_push(env, ex, old_psw);
                rx_push(env, ex, pc + r.off);
                SET_JUMP(rx_ldl(env, env->intb + vector * 4));
                break;
            }

            if(b0 == 0x75 && opsel == 0x7) { /* MVTIPL */
                uint32_t ipl = rx_next8(&r);
                if(ex) {
                    env->psw = (env->psw & ~PSW_IPL_MASK) | ((ipl & 0xf) << PSW_IPL_SHIFT);
                }
                break;
            }

            if(opsel > 3) {
                result = -1;
                goto done;
            }

            {
                uint32_t imm = (uint32_t)rx_read_imm(&r, b0 & 3);
                uint32_t dst = GETR(rd);
                switch(opsel) {
                    case 0: rx_do_sub(env, ex, dst, imm, 0); break;
                    case 1: SETR(rd, dst * imm); break;
                    case 2: {
                        uint32_t v = dst & imm;
                        SETR(rd, v);
                        rx_set_sz(env, ex, v, 4);
                        break;
                    }
                    default: {
                        uint32_t v = dst | imm;
                        SETR(rd, v);
                        rx_set_sz(env, ex, v, 4);
                        break;
                    }
                }
            }
            break;
        }

        case 0x78: case 0x79: case 0x7a: case 0x7b:
        case 0x7c: case 0x7d: { /* BSET/BCLR/BTST #bit, Rn */
            uint8_t b1 = rx_next8(&r);
            int bit = ((b0 & 1) << 4) | ((b1 >> 4) & 0xf);
            int rd = b1 & 0xf;
            uint32_t val = GETR(rd);

            switch(b0 & 0xfe) {
                case 0x78:
                    SETR(rd, val | (1u << bit));
                    break;
                case 0x7a:
                    SETR(rd, val & ~(1u << bit));
                    break;
                default: {
                    uint32_t umb = val & (1u << bit);
                    rx_set_zc(env, ex, umb == 0, umb != 0);
                    break;
                }
            }
            break;
        }

        case 0x7e: { /* unary / push / pop / CR push/pop */
            uint8_t b1 = rx_next8(&r);
            int hi = (b1 >> 4) & 0xf;
            int lo = b1 & 0xf;

            switch(hi) {
                case 0x0: {
                    uint32_t v = ~GETR(lo);
                    SETR(lo, v);
                    rx_set_sz(env, ex, v, 4);
                    break;
                }
                case 0x1:
                    SETR(lo, rx_do_sub(env, ex, 0, GETR(lo), 0));
                    break;
                case 0x2: { /* abs */
                    int64_t sll = (int32_t)GETR(lo);
                    if(sll < 0) {
                        sll = -sll;
                    }
                    SETR(lo, (uint32_t)sll);
                    rx_set_osz(env, ex, sll, 4);
                    break;
                }
                case 0x3: /* sat */
                    if((env->psw & PSW_OVERFLOW) && (env->psw & PSW_SIGN)) {
                        SETR(lo, 0x7fffffffu);
                    } else if((env->psw & PSW_OVERFLOW) && !(env->psw & PSW_SIGN)) {
                        SETR(lo, 0x80000000u);
                    }
                    break;
                case 0x4: { /* shlr */
                    uint32_t v = GETR(lo);
                    uint32_t c = v & 1;
                    v = (v >> 1) | ((env->psw & PSW_CARRY) ? 0x80000000u : 0);
                    rx_set_szc(env, ex, v, 4, c != 0);
                    SETR(lo, v);
                    break;
                }
                case 0x5: { /* shll */
                    uint32_t v = GETR(lo);
                    uint32_t c = v & 0x80000000u;
                    v = (v << 1) | ((env->psw & PSW_CARRY) ? 1u : 0);
                    rx_set_szc(env, ex, v, 4, c != 0);
                    SETR(lo, v);
                    break;
                }
                case 0x6:
                case 0x7:
                    result = -1;
                    goto done;
                case 0x8:
                case 0x9:
                case 0xa: { /* push.b/w/l */
                    int psize = 1 << (hi - 8);
                    rx_push_sized(env, ex, rx_disp_load(&r, ex, 3, lo, psize, 1), psize);
                    break;
                }
                case 0xb: /* pop.l */
                    SETR(lo, rx_pop(env, ex));
                    break;
                case 0xc:
                case 0xd: { /* pushc */
                    int cr = ((b1 >> 4) & 1) * 16 + lo;
                    uint32_t v;
                    if(!rx_get_control_register(env, cr, &v) && !ex) {
                        v = 0;
                    }
                    rx_push(env, ex, v);
                    break;
                }
                default: { /* popc */
                    int cr = ((b1 >> 4) & 1) * 16 + lo;
                    uint32_t v = rx_pop(env, ex);
                    if(ex && !rx_set_control_register(env, cr, v)) {
                        result = -1;
                        goto done;
                    }
                    break;
                }
            }
            break;
        }

        case 0x7f: {
            uint8_t b1 = rx_next8(&r);
            int hi = b1 & 0xf0;

            if(hi == 0x00) {
                SET_JUMP(GETR(b1 & 0xf));
                break;
            }
            if(hi == 0x10) {
                rx_push(env, ex, pc + r.off);
                SET_JUMP(GETR(b1 & 0xf));
                break;
            }
            if(hi == 0x40) {
                SET_JUMP((uint32_t)((int32_t)pc + (int32_t)GETR(b1 & 0xf)));
                break;
            }
            if(hi == 0x50) {
                rx_push(env, ex, pc + r.off);
                SET_JUMP((uint32_t)((int32_t)pc + (int32_t)r.off + (int32_t)GETR(b1 & 0xf)));
                break;
            }
            if((b1 & 0xf0) == 0xa0) {
                static const int flag_map[16] = { 0, 1, 2, 3, 0, 0, 0, 0, 16, 17, 0, 0, 0, 0, 0, 0 };
                int bit = flag_map[b1 & 0xf];
                if(ex) {
                    env->psw |= (1u << bit);
                }
                break;
            }
            if((b1 & 0xf0) == 0xb0) {
                static const int flag_map[16] = { 0, 1, 2, 3, 0, 0, 0, 0, 16, 17, 0, 0, 0, 0, 0, 0 };
                int bit = flag_map[b1 & 0xf];
                if(ex) {
                    env->psw &= ~(1u << bit);
                }
                break;
            }

            switch(b1) {
                case 0x83: rx_do_scmpu(env, ex); break;
                case 0x87: rx_do_smovu(env, ex); break;
                case 0x8b: rx_do_smovb(env, ex); break;
                case 0x8f: rx_do_smovf(env, ex); break;
                case 0x93: rx_do_satr(env, ex); break;
                case 0x94: /* RTTI */
                    if(ex) {
                        env->psw = env->bpsw;
                        SET_JUMP(env->bpc);
                        env->link_flag = false;
                    }
                    break;
                case 0x95: /* RTE */
                    if(ex) {
                        uint32_t new_pc = rx_pop(env, ex);
                        uint32_t new_psw = rx_pop(env, ex);
                        env->psw = new_psw;
                        if(env->psw & PSW_PM) {
                            env->psw |= PSW_U;
                        }
                        env->link_flag = false;
                        SET_JUMP(new_pc);
                    }
                    break;
                case 0x96: /* WAIT */
                    if(ex) {
                        env->wfi = true;
                        env->exception_index = EXCP_WFI;
                        cpu_loop_exit(env);
                    }
                    break;
                default:
                    if((b1 & 0xfc) == 0x80) {
                        rx_do_suntil(env, ex, b1 & 3, 0);
                    } else if((b1 & 0xfc) == 0x84) {
                        rx_do_suntil(env, ex, b1 & 3, 1);
                    } else if((b1 & 0xfc) == 0x88) {
                        rx_do_sstr(env, ex, b1 & 3);
                    } else if((b1 & 0xfc) == 0x8c) {
                        rx_do_rmpa(env, ex, b1 & 3);
                    } else {
                        result = -1;
                        goto done;
                    }
                    break;
            }
            break;
        }

        case 0xf0: case 0xf1: case 0xf2: case 0xf3:
        case 0xf4: case 0xf5: case 0xf6: case 0xf7: { /* bit ops on memory */
            uint8_t b1 = rx_next8(&r);
            int bit = b1 & 7;
            int rdst = (b1 >> 4) & 0xf;
            int sd = b0 & 3;

            if(b0 == 0xf4 && (b1 & 0xf) >= 0x8 && (b1 & 0xf) <= 0xa) {
                int psize = 1 << ((b1 & 0xf) - 8);
                int base_reg = (b1 >> 4) & 0xf;
                rx_push_sized(env, ex, rx_read_mem(env, ex, GETR(base_reg), psize, 0), psize);
                break;
            }

            if(sd == 3) {
                result = -1;
                goto done;
            }

            {
                uint32_t address = rx_disp_address(&r, sd, rdst, 1);
                uint32_t val = rx_read_mem(env, ex, address, 1, 0);
                if(b0 < 0xf4) {
                    int set = ((b1 >> 3) & 1) == 0;
                    uint32_t v = set ? (val | (1u << bit)) : (val & ~(1u << bit));
                    rx_write_mem(env, ex, address, 1, v);
                } else {
                    uint32_t umb = val & (1u << bit);
                    rx_set_zc(env, ex, umb == 0, umb != 0);
                }
            }
            break;
        }

        case 0xf8: case 0xf9: case 0xfa: case 0xfb: { /* MOV #imm, <dst> */
            int sd = b0 & 3;
            uint8_t b1 = rx_next8(&r);
            int rdst = (b1 >> 4) & 0xf;
            int im = (b1 >> 2) & 3;
            int sz = b1 & 3;
            int size;
            uint32_t addr = 0;
            uint32_t raw;
            int imm_bits;
            int zero_extend;
            uint32_t value;

            if(sz == 3) {
                result = -1;
                goto done;
            }
            size = 1 << sz;
            if(sd != 3) {
                addr = rx_disp_address(&r, sd, rdst, size);
            }

            switch(im) {
                case 0: raw = rx_next32(&r); break;
                case 1: raw = rx_next8(&r); break;
                case 2: raw = rx_next16(&r); break;
                default: raw = (uint32_t)rx_next8(&r) | ((uint32_t)rx_next8(&r) << 8) | ((uint32_t)rx_next8(&r) << 16); break;
            }

            imm_bits = (im == 0) ? 32 : ((im == 1) ? 8 : ((im == 2) ? 16 : 24));
            zero_extend = (im == 1 && sz == 0) || (im == 2 && sz == 1) || (im == 0 && sz == 2);
            value = zero_extend ? raw : (uint32_t)((int32_t)(raw << (32 - imm_bits)) >> (32 - imm_bits));

            if(sd == 3) {
                SETR(rdst, rx_apply_size(value, size, 1));
            } else {
                rx_write_mem(env, ex, addr, size, value);
            }
            break;
        }

        case 0xfc: {
            uint8_t b1 = rx_next8(&r);
            uint8_t b2 = rx_next8(&r);

            /* BSET/BCLR/BTST/BNOT Rsrc, <dst> (register bit index). */
            if((b1 & 0xf0) == 0x60) {
                int bit_op = (b1 >> 2) & 3;
                int sd = b1 & 3;
                int bdst = (b2 >> 4) & 0xf;
                int bsrc = b2 & 0xf;
                int size = (sd == 3) ? 4 : 1;
                int bit = (int)GETR(bsrc) & ((sd == 3) ? 0x1f : 0x07);
                uint32_t address = 0;
                uint32_t val;

                if(sd == 3) {
                    val = GETR(bdst);
                } else {
                    address = rx_disp_address(&r, sd, bdst, size);
                    val = rx_read_mem(env, ex, address, size, 0);
                }

                if(bit_op == 2) {
                    uint32_t umb = val & (1u << bit);
                    rx_set_zc(env, ex, umb == 0, umb != 0);
                } else {
                    if(bit_op == 0) {
                        val |= 1u << bit;
                    } else if(bit_op == 1) {
                        val &= ~(1u << bit);
                    } else {
                        val ^= 1u << bit;
                    }
                    if(sd == 3) {
                        SETR(bdst, val);
                    } else {
                        rx_write_mem(env, ex, address, size, val);
                    }
                }
                break;
            }

            /* XCHG Rsrc, Rdst */
            if((b1 & 0xfc) == 0x40) {
                int ss = b1 & 3;
                int xsrc = (b2 >> 4) & 0xf;
                int xdst = b2 & 0xf;
                if(ss == 3) {
                    uint32_t tmp = GETR(xdst);
                    SETR(xdst, GETR(xsrc));
                    SETR(xsrc, tmp);
                } else {
                    uint32_t address = rx_disp_address(&r, ss, xsrc, 1);
                    uint32_t mem = rx_read_mem(env, ex, address, 1, 0);
                    rx_write_mem(env, ex, address, 1, GETR(xdst));
                    SETR(xdst, mem);
                }
                break;
            }

            /* SC<cond>.sz */
            if((b1 & 0xf0) == 0xd0) {
                int sz = (b1 >> 2) & 3;
                int sd = b1 & 3;
                int cond = b2 & 0xf;
                if(sz == 3) {
                    result = -1;
                    goto done;
                }
                rx_disp_store(&r, ex, sd, (b2 >> 4) & 0xf, 1 << sz, rx_condition_true(env, cond) ? 1 : 0, 0);
                break;
            }

            /* BFMOV/BFMOVZ (RXv3) */
            if(b1 == 0x5a || b1 == 0x5e) {
                uint16_t field;
                int slsb, dlsb, width;
                int bsrc, bdst;
                uint32_t mask, extracted, destination_mask, original;

                if(!env->is_v3) {
                    result = -1;
                    goto done;
                }
                field = rx_next16(&r);
                slsb = field & 31;
                dlsb = (field >> 5) & 31;
                width = ((field >> 10) & 31) - dlsb;
                if(width <= 0) {
                    width += 32;
                }
                bsrc = (b2 >> 4) & 0xf;
                bdst = b2 & 0xf;
                mask = (width >= 32) ? 0xffffffffu : ((1u << width) - 1u);
                extracted = (GETR(bsrc) >> slsb) & mask;
                destination_mask = mask << dlsb;
                original = (b1 == 0x5a) ? 0 : GETR(bdst);
                SETR(bdst, (original & ~destination_mask) | ((extracted << dlsb) & destination_mask));
                break;
            }

            /* Single-precision FPU register/memory operations. */
            if(b1 >= 0x80 && b1 <= 0xa7) {
                int fop = b1 >> 2;
                int fsd = b1 & 3;
                int fsrc_reg = (b2 >> 4) & 0xf;
                int fdst_reg = b2 & 0xf;
                uint32_t fsrc = (fsd == 3) ? GETR(fsrc_reg)
                                           : rx_read_mem(env, ex, rx_disp_address(&r, fsd, fsrc_reg, 4), 4, 0);
                uint32_t fdst = GETR(fdst_reg);

                switch(fop) {
                    case 0x20: SETR(fdst_reg, rx_float_to_bits(rx_bits_to_float(fdst) - rx_bits_to_float(fsrc))); break;
                    case 0x21: rx_do_fcmp(env, ex, fdst, fsrc); break;
                    case 0x22: SETR(fdst_reg, rx_float_to_bits(rx_bits_to_float(fdst) + rx_bits_to_float(fsrc))); break;
                    case 0x23: SETR(fdst_reg, rx_float_to_bits(rx_bits_to_float(fdst) * rx_bits_to_float(fsrc))); break;
                    case 0x24: SETR(fdst_reg, rx_float_to_bits(rx_bits_to_float(fdst) / rx_bits_to_float(fsrc))); break;
                    case 0x25: SETR(fdst_reg, (uint32_t)(int32_t)rx_bits_to_float(fsrc)); break;
                    case 0x26: SETR(fdst_reg, (uint32_t)(int32_t)nearbyintf(rx_bits_to_float(fsrc))); break;
                    case 0x28: SETR(fdst_reg, rx_float_to_bits(sqrtf(rx_bits_to_float(fsrc)))); break;
                    case 0x29: SETR(fdst_reg, (uint32_t)rx_bits_to_float(fsrc)); break;
                    default:
                        result = -1;
                        goto done;
                }
                break;
            }

            /* ALU operations with an unsigned-byte memory operand. */
            {
                int alu_op = b1 >> 2;
                int alu_sd = b1 & 3;
                if(alu_sd != 3 &&
                   (alu_op == 12 || alu_op == 13 || (alu_op >= 17 && alu_op <= 19) || alu_op == 21)) {
                    int asrc = (b2 >> 4) & 0xf;
                    int adst = b2 & 0xf;
                    uint32_t address = rx_disp_address(&r, alu_sd, asrc, 1);
                    uint32_t value = rx_read_mem(env, ex, address, 1, 0);
                    uint32_t dst = GETR(adst);

                    switch(alu_op) {
                        case 12:
                            rx_set_sz(env, ex, dst & value, 4);
                            break;
                        case 13: {
                            uint32_t v = dst ^ value;
                            SETR(adst, v);
                            rx_set_sz(env, ex, v, 4);
                            break;
                        }
                        case 17:
                            SETR(adst, rx_float_to_bits((float)(int32_t)value));
                            break;
                        case 18:
                            if(rx_condition_true(env, 0)) {
                                SETR(adst, value);
                            }
                            break;
                        case 19:
                            if(rx_condition_true(env, 1)) {
                                SETR(adst, value);
                            }
                            break;
                        default:
                            SETR(adst, rx_float_to_bits((float)value));
                            break;
                    }
                    break;
                }
            }

            /* BMCnd/BNOT with immediate bit index. */
            if(b1 >= 0xe0) {
                int bmc_sd = b1 & 3;
                int bmc_bit = (b1 >> 2) & 7;
                int bmc_cond = b2 & 0xf;
                int bmc_dst = (b2 >> 4) & 0xf;
                uint32_t bmc_address = 0;
                uint32_t bmc_value;

                if(bmc_sd == 3) {
                    bmc_value = GETR(bmc_dst);
                } else {
                    bmc_address = rx_disp_address(&r, bmc_sd, bmc_dst, 1);
                    bmc_value = rx_read_mem(env, ex, bmc_address, 1, 0);
                }

                if(bmc_cond == 15) {
                    bmc_value ^= 1u << bmc_bit;
                } else if(rx_condition_true(env, bmc_cond)) {
                    bmc_value |= 1u << bmc_bit;
                } else {
                    bmc_value &= ~(1u << bmc_bit);
                }

                if(bmc_sd == 3) {
                    SETR(bmc_dst, bmc_value);
                } else {
                    rx_write_mem(env, ex, bmc_address, 1, bmc_value);
                }
                break;
            }

            {
                int rsrc = (b2 >> 4) & 0xf;
                int rdst = b2 & 0xf;
                switch(b1) {
                    case 0x03:
                        SETR(rdst, rx_do_sub(env, ex, GETR(rdst), GETR(rsrc), !(env->psw & PSW_CARRY)));
                        break;
                    case 0x07:
                        SETR(rdst, rx_do_sub(env, ex, 0, GETR(rsrc), 0));
                        break;
                    case 0x0b:
                        SETR(rdst, rx_do_add(env, ex, GETR(rdst), GETR(rsrc), (env->psw & PSW_CARRY) != 0));
                        break;
                    case 0x0f: {
                        int64_t sll = (int32_t)GETR(rsrc);
                        if(sll < 0) {
                            sll = -sll;
                        }
                        SETR(rdst, (uint32_t)sll);
                        rx_set_osz(env, ex, sll, 4);
                        break;
                    }
                    case 0x13:
                        SETR(rdst, ((int32_t)GETR(rdst) > (int32_t)GETR(rsrc)) ? GETR(rdst) : GETR(rsrc));
                        break;
                    case 0x17:
                        SETR(rdst, ((int32_t)GETR(rdst) < (int32_t)GETR(rsrc)) ? GETR(rdst) : GETR(rsrc));
                        break;
                    case 0x1b: {
                        int64_t p = (int64_t)(int32_t)GETR(rdst) * (int64_t)(int32_t)GETR(rsrc);
                        SETR(rdst, (uint32_t)p);
                        SETR((rdst + 1) & 0xf, (uint32_t)((uint64_t)p >> 32));
                        break;
                    }
                    case 0x1f: {
                        uint64_t p = (uint64_t)GETR(rdst) * (uint64_t)GETR(rsrc);
                        SETR(rdst, (uint32_t)p);
                        SETR((rdst + 1) & 0xf, (uint32_t)(p >> 32));
                        break;
                    }
                    case 0x23:
                        rx_do_div(env, ex, rdst, GETR(rsrc), 1);
                        break;
                    case 0x27:
                        rx_do_div(env, ex, rdst, GETR(rsrc), 0);
                        break;
                    case 0x33: {
                        uint32_t v = GETR(rdst) & GETR(rsrc);
                        rx_set_sz(env, ex, v, 4);
                        break;
                    }
                    case 0x37: {
                        uint32_t v = GETR(rdst) ^ GETR(rsrc);
                        SETR(rdst, v);
                        rx_set_sz(env, ex, v, 4);
                        break;
                    }
                    case 0x3b: {
                        uint32_t v = ~GETR(rsrc);
                        SETR(rdst, v);
                        rx_set_sz(env, ex, v, 4);
                        break;
                    }
                    case 0x47:
                        SETR(rdst, rx_float_to_bits((float)(int32_t)GETR(rsrc)));
                        break;
                    case 0x4b:
                        if(rx_condition_true(env, 0)) {
                            SETR(rdst, GETR(rsrc));
                        }
                        break;
                    case 0x4f:
                        if(rx_condition_true(env, 1)) {
                            SETR(rdst, GETR(rsrc));
                        }
                        break;
                    case 0x57:
                        SETR(rdst, rx_float_to_bits((float)GETR(rsrc)));
                        break;
                    default:
                        result = -1;
                        goto done;
                }
            }
            break;
        }

        case 0xfd: {
            uint8_t b1 = rx_next8(&r);

            /* Accumulator multiply / multiply-accumulate. */
            if(b1 <= 0x0f) {
                int acc_op = b1 & 7;
                int acc_sel = (b1 >> 3) & 1;
                uint8_t b2 = rx_next8(&r);
                uint32_t src1 = GETR((b2 >> 4) & 0xf);
                uint32_t src2 = GETR(b2 & 0xf);
                int64_t lo1 = (int16_t)src1;
                int64_t lo2 = (int16_t)src2;
                int64_t hi1 = (int16_t)(src1 >> 16);
                int64_t hi2 = (int16_t)(src2 >> 16);
                __int128 product;

                switch(acc_op) {
                    case 0: product = (__int128)(hi1 * hi2) << 16; rx_set_acc(env, ex, acc_sel, product); break;
                    case 1: product = (__int128)(lo1 * lo2) << 16; rx_set_acc(env, ex, acc_sel, product); break;
                    case 2: product = (__int128)(lo1 * hi2) << 16; rx_set_acc(env, ex, acc_sel, product); break;
                    case 3: rx_set_acc(env, ex, acc_sel, (__int128)(int32_t)src1 * (int32_t)src2); break;
                    case 4: product = (__int128)(hi1 * hi2) << 16; rx_set_acc(env, ex, acc_sel, rx_get_acc(env, acc_sel) + product); break;
                    case 5: product = (__int128)(lo1 * lo2) << 16; rx_set_acc(env, ex, acc_sel, rx_get_acc(env, acc_sel) + product); break;
                    case 6: product = (__int128)(lo1 * hi2) << 16; rx_set_acc(env, ex, acc_sel, rx_get_acc(env, acc_sel) + product); break;
                    default: rx_set_acc(env, ex, acc_sel, rx_get_acc(env, acc_sel) + (__int128)(int32_t)src1 * (int32_t)src2); break;
                }
                break;
            }

            /* MVTACLO/MVTACHI/MVTACGU */
            if(b1 == 0x17) {
                uint8_t b2 = rx_next8(&r);
                int acc_sel = (b2 >> 7) & 1;
                int selector = (b2 >> 4) & 7;
                uint32_t src = GETR(b2 & 0xf);
                switch(selector) {
                    case 0: rx_set_acc(env, ex, acc_sel, (__int128)src << 32); break;
                    case 1: rx_set_acc(env, ex, acc_sel, (__int128)src); break;
                    case 3: rx_set_acc(env, ex, acc_sel, (__int128)src << 64); break;
                    default: result = -1; goto done;
                }
                break;
            }

            /* RACW/RACL/RDACW/RDACL */
            if(b1 == 0x18 || b1 == 0x19) {
                uint8_t b2 = rx_next8(&r);
                int acc_sel = (b2 >> 7) & 1;
                int round_down = (b2 & 0x40) != 0;
                int shift = (b2 & 0x10) ? 2 : 1;
                int wide = (b1 == 0x19);
                __int128 value = rx_sext72(rx_get_acc(env, acc_sel) << shift);
                __int128 mask = (__int128)0xffffffff00000000LL;
                __int128 limit = wide ? (__int128)0x007fffffff00000000LL : (__int128)0x00007fff00000000LL;

                if(!round_down) {
                    value += (__int128)1 << 31;
                }
                if(value > limit) {
                    value = limit;
                } else if(value < -limit - 1) {
                    value = -limit - 1;
                } else {
                    value &= mask;
                }
                rx_set_acc(env, ex, acc_sel, value);
                break;
            }

            /* MVFACHI/MVFACLO/MVFACMI/MVFACGU */
            if(b1 == 0x1e || b1 == 0x1f) {
                uint8_t b2 = rx_next8(&r);
                int acc_sel = (b2 >> 7) & 1;
                int selector = (b2 >> 4) & 3;
                int shift = (b1 == 0x1e) ? 2 : ((b2 >> 6) & 1);
                __int128 value = rx_sext72(rx_get_acc(env, acc_sel) << shift);
                uint32_t value32;
                switch(selector) {
                    case 0: value32 = (uint32_t)(value >> 32); break;
                    case 1: value32 = (uint32_t)value; break;
                    case 2: value32 = (uint32_t)(value >> 16); break;
                    default: value32 = (uint32_t)(value >> 64); break;
                }
                SETR(b2 & 0xf, value32);
                break;
            }

            /* Post-increment / pre-decrement moves, MOVCO/MOVLI. */
            if((b1 >= 0x20 && b1 <= 0x2f) || b1 == 0x38 || b1 == 0x39 || b1 == 0x3c || b1 == 0x3d) {
                uint8_t b2 = rx_next8(&r);
                int base_reg = (b2 >> 4) & 0xf;
                int data_reg = b2 & 0xf;

                if(b1 >= 0x28 && b1 <= 0x2a) {
                    int size = 1 << (b1 - 0x28);
                    uint32_t address = GETR(base_reg);
                    SETR(data_reg, rx_read_mem(env, ex, address, size, 1));
                    SETR(base_reg, address + (uint32_t)size);
                } else if(b1 >= 0x2c && b1 <= 0x2e) {
                    int size = 1 << (b1 - 0x2c);
                    uint32_t address = GETR(base_reg) - (uint32_t)size;
                    SETR(base_reg, address);
                    SETR(data_reg, rx_read_mem(env, ex, address, size, 1));
                } else if(b1 == 0x2f) {
                    SETR(data_reg, rx_read_mem(env, ex, GETR(base_reg), 4, 0));
                    if(ex) {
                        env->link_flag = true;
                    }
                } else if(b1 >= 0x38 && b1 <= 0x39) {
                    int size = 1 << (b1 - 0x38);
                    uint32_t address = GETR(base_reg);
                    SETR(data_reg, rx_read_mem(env, ex, address, size, 0));
                    SETR(base_reg, address + (uint32_t)size);
                } else if(b1 == 0x3c || b1 == 0x3d) {
                    int size = 1 << (b1 - 0x3c);
                    uint32_t address = GETR(base_reg) - (uint32_t)size;
                    SETR(base_reg, address);
                    SETR(data_reg, rx_read_mem(env, ex, address, size, 0));
                } else if(b1 >= 0x20 && b1 <= 0x22) {
                    int size = 1 << (b1 - 0x20);
                    uint32_t address = GETR(base_reg);
                    rx_write_mem(env, ex, address, size, GETR(data_reg));
                    SETR(base_reg, address + (uint32_t)size);
                } else if(b1 >= 0x24 && b1 <= 0x26) {
                    int size = 1 << (b1 - 0x24);
                    uint32_t address = GETR(base_reg) - (uint32_t)size;
                    SETR(base_reg, address);
                    rx_write_mem(env, ex, address, size, GETR(data_reg));
                } else if(b1 == 0x27) { /* MOVCO */
                    if(ex) {
                        if(env->link_flag) {
                            rx_stl(env, GETR(base_reg), GETR(data_reg));
                            rx_set_r(env, data_reg, 0);
                        } else {
                            rx_set_r(env, data_reg, 1);
                        }
                        env->link_flag = false;
                    }
                } else {
                    result = -1;
                    goto done;
                }
                break;
            }

            /* MSBHI/MSBLO/MSBLH/EMSBA */
            if(b1 >= 0x44 && b1 <= 0x4f) {
                int msb_op = b1 & 7;
                int acc_sel = (b1 >> 3) & 1;
                uint8_t b2 = rx_next8(&r);
                uint32_t src1 = GETR((b2 >> 4) & 0xf);
                uint32_t src2 = GETR(b2 & 0xf);
                int64_t lo1 = (int16_t)src1;
                int64_t lo2 = (int16_t)src2;
                int64_t hi1 = (int16_t)(src1 >> 16);
                int64_t hi2 = (int16_t)(src2 >> 16);
                __int128 product;

                switch(msb_op) {
                    case 4: product = (__int128)(hi1 * hi2) << 16; break;
                    case 5: product = (__int128)(lo1 * lo2) << 16; break;
                    case 6: product = (__int128)(lo1 * hi2) << 16; break;
                    case 7: product = (__int128)(int32_t)src1 * (int32_t)src2; break;
                    default: result = -1; goto done;
                }
                rx_set_acc(env, ex, acc_sel, rx_get_acc(env, acc_sel) - product);
                break;
            }

            /* ROTR/ROTL #imm, Rn */
            if(b1 >= 0x6c && b1 <= 0x6f) {
                uint8_t b2 = rx_next8(&r);
                int count = ((b1 & 1) << 4) | ((b2 >> 4) & 0xf);
                int rd = b2 & 0xf;
                uint32_t v = GETR(rd);
                int carry = 0;
                for(int i = 0; i < count; i++) {
                    if(b1 & 2) {
                        carry = (v & 0x80000000u) != 0;
                        v = (v << 1) | (v >> 31);
                    } else {
                        carry = (v & 1) != 0;
                        v = (v >> 1) | (v << 31);
                    }
                }
                rx_set_szc(env, ex, v, 4, carry);
                SETR(rd, v);
                break;
            }

            if(b1 == 0x72) { /* FCMP #imm32, Rn */
                uint8_t b2 = rx_next8(&r);
                uint32_t imm = rx_next32(&r);
                rx_do_fcmp(env, ex, GETR(b2 & 0xf), imm);
                break;
            }

            if(b1 == 0x76) { /* SAVE/RSTR */
                uint8_t b2;
                int sub;
                if(!env->is_v3) {
                    result = -1;
                    goto done;
                }
                b2 = rx_next8(&r);
                sub = (b2 >> 4) & 0xf;
                if(sub == 0xc || sub == 0xd) {
                    int bank = (int)GETR(b2 & 0xf);
                    (void)rx_next8(&r);
                    if(ex) {
                        if(sub == 0xc) {
                            rx_do_save(env, bank);
                        } else if(!rx_do_restore(env, bank)) {
                            result = -1;
                            goto done;
                        }
                    }
                } else if(sub == 0xe || sub == 0xf) {
                    int bank = (int)rx_next8(&r);
                    if(ex) {
                        if(sub == 0xe) {
                            rx_do_save(env, bank);
                        } else if(!rx_do_restore(env, bank)) {
                            result = -1;
                            goto done;
                        }
                    }
                } else {
                    result = -1;
                    goto done;
                }
                break;
            }

            /* BMCnd/BNOT #imm:5, Rn */
            if(b1 >= 0xe0) {
                uint8_t b2 = rx_next8(&r);
                int bit = b1 & 0x1f;
                int cond = (b2 >> 4) & 0xf;
                int rd = b2 & 0xf;
                uint32_t value = GETR(rd);

                if(cond == 15) {
                    value ^= 1u << bit;
                } else if(rx_condition_true(env, cond)) {
                    value |= 1u << bit;
                } else {
                    value &= ~(1u << bit);
                }
                SETR(rd, value);
                break;
            }

            /* Immediate ALU / STZ / STNZ / MVTC #imm, CR */
            if((b1 & 0xf0) == 0x70) {
                int im = (b1 >> 2) & 3;
                uint8_t b2 = rx_next8(&r);

                if((b1 & 3) == 3) { /* MVTC #imm, CR */
                    int cr = ((b2 >> 4) & 1) * 16 + (b2 & 0xf);
                    uint32_t v = (uint32_t)rx_read_imm(&r, im);
                    if(ex && !rx_set_control_register(env, cr, v)) {
                        result = -1;
                        goto done;
                    }
                    break;
                }
                if((b1 & 3) != 0) {
                    result = -1;
                    goto done;
                }

                {
                    int opn = (b2 >> 4) & 0xf;
                    int rd = b2 & 0xf;
                    uint32_t imm = (uint32_t)rx_read_imm(&r, im);
                    uint32_t dst = GETR(rd);

                    switch(opn) {
                        case 0x2:
                            SETR(rd, rx_do_add(env, ex, dst, imm, (env->psw & PSW_CARRY) != 0));
                            break;
                        case 0x4:
                            SETR(rd, ((int32_t)dst > (int32_t)imm) ? dst : imm);
                            break;
                        case 0x5:
                            SETR(rd, ((int32_t)dst < (int32_t)imm) ? dst : imm);
                            break;
                        case 0x6: {
                            int64_t p = (int64_t)(int32_t)dst * (int64_t)(int32_t)imm;
                            SETR(rd, (uint32_t)p);
                            SETR((rd + 1) & 0xf, (uint32_t)((uint64_t)p >> 32));
                            break;
                        }
                        case 0x7: {
                            uint64_t p = (uint64_t)dst * (uint64_t)imm;
                            SETR(rd, (uint32_t)p);
                            SETR((rd + 1) & 0xf, (uint32_t)(p >> 32));
                            break;
                        }
                        case 0x8:
                            rx_do_div(env, ex, rd, imm, 1);
                            break;
                        case 0x9:
                            rx_do_div(env, ex, rd, imm, 0);
                            break;
                        case 0xc: {
                            uint32_t v = dst & imm;
                            rx_set_sz(env, ex, v, 4);
                            break;
                        }
                        case 0xd: {
                            uint32_t v = dst ^ imm;
                            SETR(rd, v);
                            rx_set_sz(env, ex, v, 4);
                            break;
                        }
                        case 0xe:
                            if(rx_condition_true(env, 0)) {
                                SETR(rd, imm);
                            }
                            break;
                        case 0xf:
                            if(rx_condition_true(env, 1)) {
                                SETR(rd, imm);
                            }
                            break;
                        default:
                            result = -1;
                            goto done;
                    }
                }
                break;
            }

            switch(b1) {
                case 0x60: case 0x61: case 0x62: { /* SHLR/SHAR/SHLL Rs, Rd */
                    uint8_t b2 = rx_next8(&r);
                    rx_do_shift(env, ex, b1 - 0x60, (int)GETR((b2 >> 4) & 0xf), b2 & 0xf, b2 & 0xf);
                    break;
                }
                case 0x64: { /* ROTR Rs, Rd */
                    uint8_t b2 = rx_next8(&r);
                    int count = (int)GETR((b2 >> 4) & 0xf) & 31;
                    int rd = b2 & 0xf;
                    uint32_t v = GETR(rd);
                    int c = 0;
                    for(int i = 0; i < count; i++) {
                        c = (v & 1) != 0;
                        v = (v >> 1) | (v << 31);
                    }
                    rx_set_szc(env, ex, v, 4, c);
                    SETR(rd, v);
                    break;
                }
                case 0x65: { /* REVW */
                    uint8_t b2 = rx_next8(&r);
                    uint32_t v = GETR((b2 >> 4) & 0xf);
                    SETR(b2 & 0xf, ((v >> 8) & 0x00ff00ffu) | ((v << 8) & 0xff00ff00u));
                    break;
                }
                case 0x66: { /* ROTL Rs, Rd */
                    uint8_t b2 = rx_next8(&r);
                    int count = (int)GETR((b2 >> 4) & 0xf) & 31;
                    int rd = b2 & 0xf;
                    uint32_t v = GETR(rd);
                    int c = 0;
                    for(int i = 0; i < count; i++) {
                        c = (v & 0x80000000u) != 0;
                        v = (v << 1) | (v >> 31);
                    }
                    rx_set_szc(env, ex, v, 4, c);
                    SETR(rd, v);
                    break;
                }
                case 0x67: { /* REVL */
                    uint8_t b2 = rx_next8(&r);
                    uint32_t v = GETR((b2 >> 4) & 0xf);
                    SETR(b2 & 0xf, ((v >> 24) & 0xffu) | ((v >> 8) & 0xff00u) | ((v << 8) & 0xff0000u) |
                                        ((v << 24) & 0xff000000u));
                    break;
                }
                case 0x68: case 0x69: { /* MVTC Rsrc, CR */
                    uint8_t b2 = rx_next8(&r);
                    int rsrc = (b2 >> 4) & 0xf;
                    int cr = ((b1 & 1) * 16) + (b2 & 0xf);
                    if(ex && !rx_set_control_register(env, cr, GETR(rsrc))) {
                        result = -1;
                        goto done;
                    }
                    break;
                }
                case 0x6a: case 0x6b: { /* MVFC CR, Rdst */
                    uint8_t b2 = rx_next8(&r);
                    int rdst = b2 & 0xf;
                    int cr = ((b1 & 1) * 16) + ((b2 >> 4) & 0xf);
                    uint32_t v;
                    if(!rx_get_control_register(env, cr, &v)) {
                        result = -1;
                        goto done;
                    }
                    SETR(rdst, v);
                    break;
                }
                default:
                    if((b1 & 0xe0) == 0x80) { /* SHLR #imm, Rsrc, Rdst */
                        int count = ((b1 >> 4) & 1) * 16 + (b1 & 0xf);
                        uint8_t b2 = rx_next8(&r);
                        rx_do_shift(env, ex, 0, count, (b2 >> 4) & 0xf, b2 & 0xf);
                    } else if((b1 & 0xe0) == 0xa0) { /* SHAR #imm */
                        int count = ((b1 >> 4) & 1) * 16 + (b1 & 0xf);
                        uint8_t b2 = rx_next8(&r);
                        rx_do_shift(env, ex, 1, count, (b2 >> 4) & 0xf, b2 & 0xf);
                    } else if((b1 & 0xe0) == 0xc0) { /* SHLL #imm */
                        int count = ((b1 >> 4) & 1) * 16 + (b1 & 0xf);
                        uint8_t b2 = rx_next8(&r);
                        rx_do_shift(env, ex, 2, count, (b2 >> 4) & 0xf, b2 & 0xf);
                    } else if((b1 & 0xe0) == 0xe0) { /* BN0T #imm, Rn */
                        uint8_t b2 = rx_next8(&r);
                        if((b2 & 0xf0) != 0xf0) {
                            result = -1;
                            goto done;
                        }
                        SETR(b2 & 0xf, GETR(b2 & 0xf) ^ (1u << (b1 & 0x1f)));
                    } else {
                        result = -1;
                        goto done;
                    }
                    break;
            }
            break;
        }

        case 0xfe: { /* Indexed register-indirect MOV/MOVU */
            uint8_t b1 = rx_next8(&r);
            uint8_t b2 = rx_next8(&r);
            int mode = (b1 >> 6) & 3;
            int sz = (b1 >> 4) & 3;
            int isrc = b1 & 0xf;
            int bsrc = (b2 >> 4) & 0xf;
            int rdst = b2 & 0xf;
            int size;
            uint32_t addr;

            if(sz == 3) {
                result = -1;
                goto done;
            }
            size = 1 << sz;
            addr = GETR(isrc) * (uint32_t)size + GETR(bsrc);

            if(mode == 1) {
                SETR(rdst, rx_read_mem(env, ex, addr, size, 1));
            } else if(mode == 3) {
                SETR(rdst, rx_read_mem(env, ex, addr, size, 0));
            } else if(mode == 0) {
                rx_write_mem(env, ex, addr, size, GETR(rdst));
            } else {
                result = -1;
                goto done;
            }
            break;
        }

        case 0xff: { /* Three-register ALU */
            uint8_t b1 = rx_next8(&r);
            uint8_t b2 = rx_next8(&r);
            int op = (b1 >> 4) & 0xf;
            int rdst = b1 & 0xf;
            int srca = (b2 >> 4) & 0xf;
            int srcb = b2 & 0xf;
            uint32_t a = GETR(srca);
            uint32_t b = GETR(srcb);

            switch(op) {
                case 0x0: SETR(rdst, rx_do_sub(env, ex, b, a, 0)); break;
                case 0x2: SETR(rdst, rx_do_add(env, ex, b, a, 0)); break;
                case 0x3: SETR(rdst, b * a); break;
                case 0x4: {
                    uint32_t v = b & a;
                    SETR(rdst, v);
                    rx_set_sz(env, ex, v, 4);
                    break;
                }
                case 0x5: {
                    uint32_t v = b | a;
                    SETR(rdst, v);
                    rx_set_sz(env, ex, v, 4);
                    break;
                }
                case 0x6: {
                    uint32_t v = b ^ a;
                    SETR(rdst, v);
                    rx_set_sz(env, ex, v, 4);
                    break;
                }
                case 0x8: SETR(rdst, rx_float_to_bits(rx_bits_to_float(b) - rx_bits_to_float(a))); break;
                case 0xa: SETR(rdst, rx_float_to_bits(rx_bits_to_float(b) + rx_bits_to_float(a))); break;
                case 0xb: SETR(rdst, rx_float_to_bits(rx_bits_to_float(b) * rx_bits_to_float(a))); break;
                default:
                    result = -1;
                    goto done;
            }
            break;
        }

        default:
            result = -1;
            goto done;
    }

    result = (int)r.off;

done:
    if(ex && result >= 0) {
        env->pc = has_jump ? jump : (pc + r.off);
    }
    return result;

#undef GETR
#undef SETR
#undef SET_JUMP
}

int rx_insn_length(CPUState *env, uint32_t pc)
{
    int length = rx_execute(env, pc, 0);
    return length > 0 ? length : 1;
}

void helper_rx_exec(CPUState *env, target_ulong pc)
{
    int result;

    rx_helper_retaddr = GETPC_INTERNAL();
    result = rx_execute(env, pc, 1);
    if(result < 0) {
        env->exception_index = EXCP_RX_ILLEGAL_INSTRUCTION;
        cpu_loop_exit(env);
    }
}

void helper_rx_debug(CPUState *env)
{
    env->exception_index = EXCP_DEBUG;
    cpu_loop_exit(env);
}

/* Called by the generic softmmu layer. RX has a flat, identity mapped address
 * space, so filling a TLB entry never fails. */
int arch_tlb_fill(CPUState *env1, target_ulong addr, int access_type, int mmu_idx, void *retaddr, int no_page_fault,
                  int access_width, target_phys_addr_t *paddr)
{
    CPUState *saved_env = env;
    int ret;

    (void)access_width;
    (void)retaddr;
    env = env1;
    ret = cpu_handle_mmu_fault(env, addr, access_type, mmu_idx, no_page_fault, paddr);
    env = saved_env;
    return ret;
}

void arch_raise_mmu_fault_exception(CPUState *env, int errcode, int access_type, target_ulong address, void *retaddr)
{
    (void)errcode;
    (void)access_type;
    (void)address;
    (void)retaddr;
    /* Identity mapping: no MMU fault can be raised. */
    env->exception_index = EXCP_RX_ILLEGAL_INSTRUCTION;
    cpu_loop_exit(env);
}
