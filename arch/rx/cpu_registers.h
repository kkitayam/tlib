#pragma once

#include "cpu-defs.h"

/*
 * Register numbering follows the RX architecture model used by Renode:
 * R0-R15 = 0-15, PC = 16, PSW = 17, USP = 18, ISP = 19, INTB = 20,
 * BPSW = 21, BPC = 22, FINTV = 23, FPSW = 24, ACC0 = 25, ACC1 = 26.
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
} RXRegisters;

#define RX_REGISTER_COUNT 27
