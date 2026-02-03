/*
 *  rv32_emitter.cpp - RISC-V RV32I instruction emitter for JIT
 *
 *  BasiliskII ESP32-P4 JIT Compiler
 */

#include "rv32_emitter.h"
#include <string.h>

// RISC-V opcode constants
#define RV_OP_LUI       0x37
#define RV_OP_AUIPC     0x17
#define RV_OP_JAL       0x6F
#define RV_OP_JALR      0x67
#define RV_OP_BRANCH    0x63
#define RV_OP_LOAD      0x03
#define RV_OP_STORE     0x23
#define RV_OP_IMM       0x13
#define RV_OP_REG       0x33

// funct3 values for branches
#define RV_BEQ          0x0
#define RV_BNE          0x1
#define RV_BLT          0x4
#define RV_BGE          0x5
#define RV_BLTU         0x6
#define RV_BGEU         0x7

// funct3 values for loads
#define RV_LB           0x0
#define RV_LH           0x1
#define RV_LW           0x2
#define RV_LBU          0x4
#define RV_LHU          0x5

// funct3 values for stores
#define RV_SB           0x0
#define RV_SH           0x1
#define RV_SW           0x2

// funct3 values for immediate ops
#define RV_ADDI         0x0
#define RV_SLTI         0x2
#define RV_SLTIU        0x3
#define RV_XORI         0x4
#define RV_ORI          0x6
#define RV_ANDI         0x7
#define RV_SLLI         0x1
#define RV_SRLI_SRAI    0x5

// funct3 values for register ops
#define RV_ADD_SUB      0x0
#define RV_SLL          0x1
#define RV_SLT          0x2
#define RV_SLTU         0x3
#define RV_XOR          0x4
#define RV_SRL_SRA      0x5
#define RV_OR           0x6
#define RV_AND          0x7

// funct7 values
#define RV_FUNCT7_NORMAL 0x00
#define RV_FUNCT7_ALT    0x20  // For SUB, SRA

// Helper to emit a 32-bit instruction
static inline void emit32(rv_emitter_t *emit, uint32_t instr) {
    if (emit->code < emit->code_end) {
        *emit->code++ = instr;
    }
}

// R-type instruction encoding
static inline uint32_t encode_r(uint32_t opcode, rv_reg_t rd, uint32_t funct3, 
                                 rv_reg_t rs1, rv_reg_t rs2, uint32_t funct7) {
    return opcode | (rd << 7) | (funct3 << 12) | (rs1 << 15) | (rs2 << 20) | (funct7 << 25);
}

// I-type instruction encoding
static inline uint32_t encode_i(uint32_t opcode, rv_reg_t rd, uint32_t funct3, 
                                 rv_reg_t rs1, int32_t imm) {
    return opcode | (rd << 7) | (funct3 << 12) | (rs1 << 15) | ((imm & 0xFFF) << 20);
}

// S-type instruction encoding
static inline uint32_t encode_s(uint32_t opcode, uint32_t funct3, 
                                 rv_reg_t rs1, rv_reg_t rs2, int32_t imm) {
    uint32_t imm_lo = imm & 0x1F;
    uint32_t imm_hi = (imm >> 5) & 0x7F;
    return opcode | (imm_lo << 7) | (funct3 << 12) | (rs1 << 15) | (rs2 << 20) | (imm_hi << 25);
}

// B-type instruction encoding
static inline uint32_t encode_b(uint32_t opcode, uint32_t funct3, 
                                 rv_reg_t rs1, rv_reg_t rs2, int32_t imm) {
    // imm[12|10:5] | rs2 | rs1 | funct3 | imm[4:1|11] | opcode
    uint32_t imm_11 = (imm >> 11) & 0x1;
    uint32_t imm_4_1 = (imm >> 1) & 0xF;
    uint32_t imm_10_5 = (imm >> 5) & 0x3F;
    uint32_t imm_12 = (imm >> 12) & 0x1;
    return opcode | (imm_11 << 7) | (imm_4_1 << 8) | (funct3 << 12) | 
           (rs1 << 15) | (rs2 << 20) | (imm_10_5 << 25) | (imm_12 << 31);
}

// U-type instruction encoding
static inline uint32_t encode_u(uint32_t opcode, rv_reg_t rd, int32_t imm) {
    return opcode | (rd << 7) | (imm & 0xFFFFF000);
}

// J-type instruction encoding
static inline uint32_t encode_j(uint32_t opcode, rv_reg_t rd, int32_t imm) {
    // imm[20|10:1|11|19:12] | rd | opcode
    uint32_t imm_19_12 = (imm >> 12) & 0xFF;
    uint32_t imm_11 = (imm >> 11) & 0x1;
    uint32_t imm_10_1 = (imm >> 1) & 0x3FF;
    uint32_t imm_20 = (imm >> 20) & 0x1;
    return opcode | (rd << 7) | (imm_19_12 << 12) | (imm_11 << 20) | 
           (imm_10_1 << 21) | (imm_20 << 31);
}

// ========== Public API ==========

void rv_emit_init(rv_emitter_t *emit, void *buffer, size_t size) {
    emit->code_start = (uint32_t *)buffer;
    emit->code = emit->code_start;
    emit->code_end = (uint32_t *)((uint8_t *)buffer + size);
    emit->capacity = size;
}

uint32_t *rv_emit_get_pos(rv_emitter_t *emit) {
    return emit->code;
}

size_t rv_emit_get_size(rv_emitter_t *emit) {
    return (emit->code - emit->code_start) * sizeof(uint32_t);
}

int rv_emit_has_room(rv_emitter_t *emit, size_t n_instructions) {
    return (emit->code + n_instructions) <= emit->code_end;
}

// ========== R-Type Instructions ==========

void rv_emit_add(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2) {
    emit32(emit, encode_r(RV_OP_REG, rd, RV_ADD_SUB, rs1, rs2, RV_FUNCT7_NORMAL));
}

void rv_emit_sub(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2) {
    emit32(emit, encode_r(RV_OP_REG, rd, RV_ADD_SUB, rs1, rs2, RV_FUNCT7_ALT));
}

void rv_emit_and(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2) {
    emit32(emit, encode_r(RV_OP_REG, rd, RV_AND, rs1, rs2, RV_FUNCT7_NORMAL));
}

void rv_emit_or(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2) {
    emit32(emit, encode_r(RV_OP_REG, rd, RV_OR, rs1, rs2, RV_FUNCT7_NORMAL));
}

void rv_emit_xor(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2) {
    emit32(emit, encode_r(RV_OP_REG, rd, RV_XOR, rs1, rs2, RV_FUNCT7_NORMAL));
}

void rv_emit_sll(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2) {
    emit32(emit, encode_r(RV_OP_REG, rd, RV_SLL, rs1, rs2, RV_FUNCT7_NORMAL));
}

void rv_emit_srl(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2) {
    emit32(emit, encode_r(RV_OP_REG, rd, RV_SRL_SRA, rs1, rs2, RV_FUNCT7_NORMAL));
}

void rv_emit_sra(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2) {
    emit32(emit, encode_r(RV_OP_REG, rd, RV_SRL_SRA, rs1, rs2, RV_FUNCT7_ALT));
}

void rv_emit_slt(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2) {
    emit32(emit, encode_r(RV_OP_REG, rd, RV_SLT, rs1, rs2, RV_FUNCT7_NORMAL));
}

void rv_emit_sltu(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, rv_reg_t rs2) {
    emit32(emit, encode_r(RV_OP_REG, rd, RV_SLTU, rs1, rs2, RV_FUNCT7_NORMAL));
}

// ========== I-Type Instructions ==========

void rv_emit_addi(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t imm) {
    emit32(emit, encode_i(RV_OP_IMM, rd, RV_ADDI, rs1, imm));
}

void rv_emit_andi(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t imm) {
    emit32(emit, encode_i(RV_OP_IMM, rd, RV_ANDI, rs1, imm));
}

void rv_emit_ori(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t imm) {
    emit32(emit, encode_i(RV_OP_IMM, rd, RV_ORI, rs1, imm));
}

void rv_emit_xori(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t imm) {
    emit32(emit, encode_i(RV_OP_IMM, rd, RV_XORI, rs1, imm));
}

void rv_emit_slti(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t imm) {
    emit32(emit, encode_i(RV_OP_IMM, rd, RV_SLTI, rs1, imm));
}

void rv_emit_sltiu(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t imm) {
    emit32(emit, encode_i(RV_OP_IMM, rd, RV_SLTIU, rs1, imm));
}

void rv_emit_slli(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t shamt) {
    emit32(emit, encode_i(RV_OP_IMM, rd, RV_SLLI, rs1, shamt & 0x1F));
}

void rv_emit_srli(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t shamt) {
    emit32(emit, encode_i(RV_OP_IMM, rd, RV_SRLI_SRAI, rs1, shamt & 0x1F));
}

void rv_emit_srai(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t shamt) {
    emit32(emit, encode_i(RV_OP_IMM, rd, RV_SRLI_SRAI, rs1, (shamt & 0x1F) | 0x400));
}

// Load instructions
void rv_emit_lb(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t offset) {
    emit32(emit, encode_i(RV_OP_LOAD, rd, RV_LB, rs1, offset));
}

void rv_emit_lbu(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t offset) {
    emit32(emit, encode_i(RV_OP_LOAD, rd, RV_LBU, rs1, offset));
}

void rv_emit_lh(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t offset) {
    emit32(emit, encode_i(RV_OP_LOAD, rd, RV_LH, rs1, offset));
}

void rv_emit_lhu(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t offset) {
    emit32(emit, encode_i(RV_OP_LOAD, rd, RV_LHU, rs1, offset));
}

void rv_emit_lw(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t offset) {
    emit32(emit, encode_i(RV_OP_LOAD, rd, RV_LW, rs1, offset));
}

// ========== S-Type Instructions (Store) ==========

void rv_emit_sb(rv_emitter_t *emit, rv_reg_t rs2, rv_reg_t rs1, int32_t offset) {
    emit32(emit, encode_s(RV_OP_STORE, RV_SB, rs1, rs2, offset));
}

void rv_emit_sh(rv_emitter_t *emit, rv_reg_t rs2, rv_reg_t rs1, int32_t offset) {
    emit32(emit, encode_s(RV_OP_STORE, RV_SH, rs1, rs2, offset));
}

void rv_emit_sw(rv_emitter_t *emit, rv_reg_t rs2, rv_reg_t rs1, int32_t offset) {
    emit32(emit, encode_s(RV_OP_STORE, RV_SW, rs1, rs2, offset));
}

// ========== B-Type Instructions (Branch) ==========

void rv_emit_beq(rv_emitter_t *emit, rv_reg_t rs1, rv_reg_t rs2, int32_t offset) {
    emit32(emit, encode_b(RV_OP_BRANCH, RV_BEQ, rs1, rs2, offset));
}

void rv_emit_bne(rv_emitter_t *emit, rv_reg_t rs1, rv_reg_t rs2, int32_t offset) {
    emit32(emit, encode_b(RV_OP_BRANCH, RV_BNE, rs1, rs2, offset));
}

void rv_emit_blt(rv_emitter_t *emit, rv_reg_t rs1, rv_reg_t rs2, int32_t offset) {
    emit32(emit, encode_b(RV_OP_BRANCH, RV_BLT, rs1, rs2, offset));
}

void rv_emit_bge(rv_emitter_t *emit, rv_reg_t rs1, rv_reg_t rs2, int32_t offset) {
    emit32(emit, encode_b(RV_OP_BRANCH, RV_BGE, rs1, rs2, offset));
}

void rv_emit_bltu(rv_emitter_t *emit, rv_reg_t rs1, rv_reg_t rs2, int32_t offset) {
    emit32(emit, encode_b(RV_OP_BRANCH, RV_BLTU, rs1, rs2, offset));
}

void rv_emit_bgeu(rv_emitter_t *emit, rv_reg_t rs1, rv_reg_t rs2, int32_t offset) {
    emit32(emit, encode_b(RV_OP_BRANCH, RV_BGEU, rs1, rs2, offset));
}

// ========== U-Type Instructions ==========

void rv_emit_lui(rv_emitter_t *emit, rv_reg_t rd, int32_t imm) {
    emit32(emit, encode_u(RV_OP_LUI, rd, imm));
}

void rv_emit_auipc(rv_emitter_t *emit, rv_reg_t rd, int32_t imm) {
    emit32(emit, encode_u(RV_OP_AUIPC, rd, imm));
}

// ========== J-Type Instructions ==========

void rv_emit_jal(rv_emitter_t *emit, rv_reg_t rd, int32_t offset) {
    emit32(emit, encode_j(RV_OP_JAL, rd, offset));
}

void rv_emit_jalr(rv_emitter_t *emit, rv_reg_t rd, rv_reg_t rs1, int32_t offset) {
    emit32(emit, encode_i(RV_OP_JALR, rd, 0, rs1, offset));
}

// ========== Pseudo-Instructions ==========

void rv_emit_li(rv_emitter_t *emit, rv_reg_t rd, int32_t imm) {
    // For small immediates, use single addi
    if (imm >= -2048 && imm < 2048) {
        rv_emit_addi(emit, rd, RV_ZERO, imm);
    } else {
        // lui + addi for larger immediates
        int32_t hi = imm + 0x800;  // Round up for negative lower 12 bits
        int32_t lo = imm - (hi & 0xFFFFF000);
        rv_emit_lui(emit, rd, hi);
        if (lo != 0) {
            rv_emit_addi(emit, rd, rd, lo);
        }
    }
}
