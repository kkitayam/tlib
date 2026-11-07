#include <stdint.h>

#include "cpu.h"
#include "cpu_registers.h"

uint32_t rx_get_active_sp(CPUState *env)
{
    return (env->psw & PSW_U) ? env->usp : env->isp;
}

void rx_set_active_sp(CPUState *env, uint32_t value)
{
    if(env->psw & PSW_U) {
        env->usp = value;
    } else {
        env->isp = value;
    }
}

uint64_t rx_get_acc0(CPUState *env)
{
    return (uint64_t)env->acc0;
}

uint64_t rx_get_acc1(CPUState *env)
{
    return (uint64_t)env->acc1;
}

void rx_set_acc0(CPUState *env, uint64_t value)
{
    env->acc0 = (__int128)(int64_t)value;
}

void rx_set_acc1(CPUState *env, uint64_t value)
{
    env->acc1 = (__int128)(int64_t)value;
}

static uint32_t rx_acc_high(__int128 value)
{
    return (uint32_t)(((unsigned __int128)value >> 64) & 0xffu);
}

static __int128 rx_acc_with_high(__int128 current, uint32_t high)
{
    uint32_t high8 = high & 0xffu;
    __int128 raw = (__int128)(uint64_t)current | ((__int128)high8 << 64);

    if(high8 & 0x80u) {
        raw -= (__int128)1 << 72;
    }
    return raw;
}

uint32_t rx_get_acc0_high(CPUState *env)
{
    return rx_acc_high(env->acc0);
}

uint32_t rx_get_acc1_high(CPUState *env)
{
    return rx_acc_high(env->acc1);
}

void rx_set_acc0_high(CPUState *env, uint32_t value)
{
    env->acc0 = rx_acc_with_high(env->acc0, value);
}

void rx_set_acc1_high(CPUState *env, uint32_t value)
{
    env->acc1 = rx_acc_with_high(env->acc1, value);
}

uint32_t rx_get_extb(CPUState *env)
{
    return env->extb;
}

void rx_set_extb(CPUState *env, uint32_t value)
{
    env->extb = value;
}

uint32_t *get_reg_pointer_32(int reg)
{
    switch(reg) {
        case R_0_32:
            return (cpu->psw & PSW_U) ? &cpu->usp : &cpu->isp;
        case R_1_32 ... R_15_32:
            return &cpu->regs[reg];
        case PC_32:
            return &cpu->pc;
        case PSW_32:
            return &cpu->psw;
        case USP_32:
            return &cpu->usp;
        case ISP_32:
            return &cpu->isp;
        case INTB_32:
            return &cpu->intb;
        case BPSW_32:
            return &cpu->bpsw;
        case BPC_32:
            return &cpu->bpc;
        case FINTV_32:
            return &cpu->fintv;
        case FPSW_32:
            return &cpu->fpsw;
        case EXTB_32:
            return &cpu->extb;
        default:
            return NULL;
    }
}

uint32_t tlib_get_register_value_32(int reg_number)
{
    switch(reg_number) {
        case ACC0_HIGH_32:
            return rx_get_acc0_high(cpu);
        case ACC1_HIGH_32:
            return rx_get_acc1_high(cpu);
        default:
            break;
    }

    uint32_t *ptr = get_reg_pointer_32(reg_number);
    if(ptr == NULL) {
        tlib_abortf("Read from undefined CPU register number %d detected", reg_number);
    }
    return *ptr;
}

EXC_INT_1(uint32_t, tlib_get_register_value_32, int, reg_number)

void tlib_set_register_value_32(int reg_number, uint32_t value)
{
    switch(reg_number) {
        case ACC0_HIGH_32:
            rx_set_acc0_high(cpu, value);
            return;
        case ACC1_HIGH_32:
            rx_set_acc1_high(cpu, value);
            return;
        default:
            break;
    }

    uint32_t *ptr = get_reg_pointer_32(reg_number);
    if(ptr == NULL) {
        tlib_abortf("Write to undefined CPU register number %d detected", reg_number);
    }
    *ptr = value;
}

EXC_VOID_2(tlib_set_register_value_32, int, reg_number, uint32_t, value)
