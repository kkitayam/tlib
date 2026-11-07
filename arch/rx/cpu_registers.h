#pragma once

#include "cpu-defs.h"

/*
 * Register numbering follows the RX architecture model used by Renode:
 * R0-R15 = 0-15, PC = 16, PSW = 17, USP = 18, ISP = 19, INTB = 20,
 * BPSW = 21, BPC = 22, FINTV = 23, FPSW = 24, ACC0 = 25, ACC1 = 26,
 * EXTB = 27, ACC0_HIGH = 28, ACC1_HIGH = 29.
 *
 * ACC0 and ACC1 are 72-bit. The generic 32-bit register interface exposes
 * their upper 8 bits through ACC0_HIGH/ACC1_HIGH; the low 64 bits are
 * available through tlib_rx_get_acc0/1 and tlib_rx_set_acc0/1.
 */
typedef enum {
    R_0_32 = 0,
    R_1_32 = 1,
    R_2_32 = 2,
    R_3_32 = 3,
    R_4_32 = 4,
    R_5_32 = 5,
    R_6_32 = 6,
    R_7_32 = 7,
    R_8_32 = 8,
    R_9_32 = 9,
    R_10_32 = 10,
    R_11_32 = 11,
    R_12_32 = 12,
    R_13_32 = 13,
    R_14_32 = 14,
    R_15_32 = 15,
    PC_32 = 16,
    PSW_32 = 17,
    USP_32 = 18,
    ISP_32 = 19,
    INTB_32 = 20,
    BPSW_32 = 21,
    BPC_32 = 22,
    FINTV_32 = 23,
    FPSW_32 = 24,
    ACC_32 = 25,
    ACC1_32 = 26,
    EXTB_32 = 27,
    ACC0_HIGH_32 = 28,
    ACC1_HIGH_32 = 29,
} RXRegisters;

#define RX_REGISTER_COUNT 30
