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
        default:
            return NULL;
    }
}

CPU_REGISTER_ACCESSOR(32)
