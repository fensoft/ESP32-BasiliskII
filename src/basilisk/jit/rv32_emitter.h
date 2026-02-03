/*
 *  rv32_emitter.h - RISC-V RV32I instruction emitter for JIT
 *
 *  BasiliskII ESP32-P4 JIT Compiler
 *
 *  Provides functions to emit RISC-V machine code instructions
 *  for the ESP32-P4's RISC-V core.
 */

#ifndef RV32_EMITTER_H
#define RV32_EMITTER_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// RISC-V register names
typedef enum {
    RV_ZERO = 0,   // x0 - hardwired zero
    RV_RA   = 1,   // x1 - return address
    RV_SP   = 2,   // x2 - stack pointer
    RV_GP   = 3,   // x3 - global pointer
    RV_TP   = 4,   // x4 - thread pointer
    RV_T0   = 5,   // x5 - temporary
    RV_T1   = 6,   // x6 - temporary
    RV_T2   = 7,   // x7 - temporary
    RV_S0   = 8,   // x8 - saved register / frame pointer
    RV_S1   = 9,   // x9 - saved register
    RV_A0   = 10,  // x10 - argument / return value
    RV_A1   = 11,  // x11 - argument / return value
    RV_A2   = 12,  // x12 - argument
    RV_A3   = 13,  // x13 - argument
    RV_A4   = 14,  // x14 - argument
    RV_A5   = 15,  // x15 - argument
    RV_A6   = 16,  // x16 - argument
    RV_A7   = 17,  // x17 - argument
    RV_S2   = 18,  // x18 - saved register
    RV_S3   = 19,  // x19 - saved register
    RV_S4   = 20,  // x20 - saved register
    RV_S5   = 21,  // x21 - saved register
    RV_S6   = 22,  // x22 - saved register
    RV_S7   = 23,  // x23 - saved register
    RV_S8   = 24,  // x24 - saved register
    RV_S9   = 25,  // x25 - saved register
    RV_S10  = 26,  // x26 - saved register
    RV_S11  = 27,  // x27 - saved register
    RV_T3   = 28,  // x28 - temporary
    RV_T4   = 29,  // x29 - temporary
    RV_T5   = 30,  // x30 - temporary
    RV_T6   = 31   // x31 - temporary
} rv_reg_t;

// 68k to RISC-V register mapping
// We use saved registers (s0-s11) for 68k registers since they're callee-saved
// This means JIT code doesn't need to save/restore them on function calls
#define M68K_D0  RV_S0
#define M68K_D1  RV_S1
#define M68K_D2  RV_S2
#define M68K_D3  RV_S3
#define M68K_D4  RV_S4
#define M68K_D5  RV_S5
#define M68K_D6  RV_S6
#define M68K_D7  RV_S7
#define M68K_A0  RV_S8
#define M68K_A1  RV_S9
#define M68K_A2  RV_S10
#define M68K_A3  RV_S11
#define M68K_A4  RV_A2   // Running low on saved regs, use arg regs
#define M68K_A5  RV_A3
#define M68K_A6  RV_A4
// A7 (SP) is kept in memory since it's updated very frequently

// Temporary registers for JIT code generation
#define JIT_TMP1 RV_T0
#define JIT_TMP2 RV_T1
#define JIT_TMP3 RV_T2

// Code emitter context
typedef struct {
    uint32_t *code;        // Pointer to code buffer
    uint32_t *code_start;  // Start of allocated buffer
    uint32_t *code_end;    // End of allocated buffer
    size_t    capacity;    // Buffer size in bytes
} rv_emitter_t;

// Initialize emitter with a code buffer
void rv_emit_init(rv_emitter_t *emit, void *buffer, size_t size);

// Get current code position
uint32_t *rv_emit_get_pos(rv_emitter_t *emit);

// Get code size in bytes
size_t rv_emit_get_size(rv_emitter_t *emit);

// Check if there's room for n instructions
int rv_emit_has_room(rv_emitter_t *emit, size_t n_instructions);

// ========== R-Type Instructions ==========
// Format: funct7[31:25] | rs2[24:20] | rs1[19:15] | funct3[14:12] | rd[11:7] | opcode[6:0]

void rv_emit_add(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2);
void rv_emit_sub(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2);
void rv_emit_and(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2);
void rv_emit_or(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2);
void rv_emit_xor(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2);
void rv_emit_sll(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2);
void rv_emit_srl(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2);
void rv_emit_sra(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2);
void rv_emit_slt(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2);
void rv_emit_sltu(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2);

// ========== I-Type Instructions ==========
// Format: imm[31:20] | rs1[19:15] | funct3[14:12] | rd[11:7] | opcode[6:0]

void rv_emit_addi(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t imm);
void rv_emit_andi(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t imm);
void rv_emit_ori(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t imm);
void rv_emit_xori(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t imm);
void rv_emit_slti(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t imm);
void rv_emit_sltiu(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t imm);
void rv_emit_slli(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t shamt);
void rv_emit_srli(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t shamt);
void rv_emit_srai(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t shamt);

// Load instructions
void rv_emit_lb(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t offset);
void rv_emit_lbu(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t offset);
void rv_emit_lh(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t offset);
void rv_emit_lhu(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t offset);
void rv_emit_lw(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t offset);

// ========== S-Type Instructions (Store) ==========
// Format: imm[11:5] | rs2[24:20] | rs1[19:15] | funct3[14:12] | imm[4:0] | opcode[6:0]

void rv_emit_sb(rv_emitter_t *emit, rv_reg_t rs2, rv_reg_t rs1, int32_t offset);
void rv_emit_sh(rv_emitter_t *emit, rv_reg_t rs2, rv_reg_t rs1, int32_t offset);
void rv_emit_sw(rv_emitter_t *emit, rv_reg_t rs2, rv_reg_t rs1, int32_t offset);

// ========== B-Type Instructions (Branch) ==========
// Format: imm[12|10:5] | rs2[24:20] | rs1[19:15] | funct3[14:12] | imm[4:1|11] | opcode[6:0]

void rv_emit_beq(rv_emitter_t *emit, rv_reg_t rs1, rv_reg_t rs2, int32_t offset);
void rv_emit_bne(rv_emitter_t *emit, rv_reg_t rs1, rv_reg_t rs2, int32_t offset);
void rv_emit_blt(rv_emitter_t *emit, rv_reg_t rs1, rv_reg_t rs2, int32_t offset);
void rv_emit_bge(rv_emitter_t *emit, rv_reg_t rs1, rv_reg_t rs2, int32_t offset);
void rv_emit_bltu(rv_emitter_t *emit, rv_reg_t rs1, rv_reg_t rs2, int32_t offset);
void rv_emit_bgeu(rv_emitter_t *emit, rv_reg_t rs1, rv_reg_t rs2, int32_t offset);

// ========== U-Type Instructions ==========
// Format: imm[31:12] | rd[11:7] | opcode[6:0]

void rv_emit_lui(rv_emitter_t *emit, rv_reg_t rd, int32_t imm);
void rv_emit_auipc(rv_emitter_t *emit, rv_reg_t rd, int32_t imm);

// ========== J-Type Instructions ==========
// Format: imm[20|10:1|11|19:12] | rd[11:7] | opcode[6:0]

void rv_emit_jal(rv_emitter_t *emit, rv_reg_t rd, int32_t offset);
void rv_emit_jalr(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t offset);

// ========== Pseudo-Instructions ==========

// mv rd, rs -> addi rd, rs, 0
static inline void rv_emit_mv(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs) {
    rv_emit_addi(emit, rd, rs, 0);
}

// li rd, imm -> lui + addi sequence for any 32-bit immediate
void rv_emit_li(rv_emitter_t *emit, rv_reg_t rd, int32_t imm);

// j offset -> jal x0, offset
static inline void rv_emit_j(rv_emitter_t *emit, int32_t offset) {
    rv_emit_jal(emit, RV_ZERO, offset);
}

// jr rs -> jalr x0, rs, 0
static inline void rv_emit_jr(rv_emitter_t *emit, rv_reg_t rs) {
    rv_emit_jalr(emit, RV_ZERO, rs, 0);
}

// ret -> jalr x0, ra, 0
static inline void rv_emit_ret(rv_emitter_t *emit) {
    rv_emit_jalr(emit, RV_ZERO, RV_RA, 0);
}

// nop -> addi x0, x0, 0
static inline void rv_emit_nop(rv_emitter_t *emit) {
    rv_emit_addi(emit, RV_ZERO, RV_ZERO, 0);
}

// seqz rd, rs -> sltiu rd, rs, 1
static inline void rv_emit_seqz(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs) {
    rv_emit_sltiu(emit, rd, rs, 1);
}

// snez rd, rs -> sltu rd, x0, rs
static inline void rv_emit_snez(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs) {
    rv_emit_sltu(emit, rd, RV_ZERO, rs);
}

// neg rd, rs -> sub rd, x0, rs
static inline void rv_emit_neg(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs) {
    rv_emit_sub(emit, rd, RV_ZERO, rs);
}

// not rd, rs -> xori rd, rs, -1
static inline void rv_emit_not(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs) {
    rv_emit_xori(emit, rd, rs, -1);
}

// beqz rs, offset -> beq rs, x0, offset
static inline void rv_emit_beqz(rv_emitter_t *emit, rv_reg_t rs, int32_t offset) {
    rv_emit_beq(emit, rs, RV_ZERO, offset);
}

// bnez rs, offset -> bne rs, x0, offset
static inline void rv_emit_bnez(rv_emitter_t *emit, rv_reg_t rs, int32_t offset) {
    rv_emit_bne(emit, rs, RV_ZERO, offset);
}

#ifdef __cplusplus
}
#endif

#endif // RV32_EMITTER_H
