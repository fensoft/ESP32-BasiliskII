/*
 *  jit_compiler.cpp - 68k to RISC-V JIT compiler
 *
 *  BasiliskII ESP32-P4 JIT Compiler
 */

// Must include sysdeps.h first for type definitions
#include "sysdeps.h"

#include "jit_compiler.h"
#include "jit_cache.h"
#include "rv32_emitter.h"
#include <string.h>

#ifdef ARDUINO
#include <Arduino.h>
#include <esp_cache.h>
#endif

// Access to 68k emulator state
#include "cpu_emulation.h"
#include "m68k.h"
#include "memory.h"
#include "newcpu.h"

// Statistics
static uint32_t jit_blocks_compiled = 0;
static uint32_t jit_blocks_executed = 0;
static uint32_t jit_fallbacks = 0;

// Temporary code buffer for compilation
static uint8_t temp_code_buffer[JIT_MAX_BLOCK_SIZE] __attribute__((aligned(64)));

int jit_init(void) {
#ifdef ARDUINO
    Serial.println("[JIT] Initializing JIT compiler...");
#endif
    
    int result = jit_cache_init();
    if (result < 0) {
        return result;
    }
    
    jit_blocks_compiled = 0;
    jit_blocks_executed = 0;
    jit_fallbacks = 0;
    
#ifdef ARDUINO
    Serial.println("[JIT] JIT compiler initialized");
    Serial.println("[JIT] WARNING: JIT is experimental - may cause instability");
#endif
    
    return 0;
}

void jit_shutdown(void) {
    jit_cache_shutdown();
}

// Check if an opcode can be JIT compiled
int jit_can_compile(uint16_t opcode) {
    // MOVEQ #imm, Dn - 0x7xxx where bit 8 is 0
    if ((opcode & 0xF100) == 0x7000) {
        return 1;
    }
    
    // NOP - 0x4E71
    if (opcode == 0x4E71) {
        return 1;
    }
    
    // CLR.L Dn - 0x4280-0x4287
    if ((opcode & 0xFFF8) == 0x4280) {
        return 1;
    }
    
    // TST.L Dn - 0x4A80-0x4A87
    if ((opcode & 0xFFF8) == 0x4A80) {
        return 1;
    }
    
    // EXT.W Dn - 0x4880-0x4887
    if ((opcode & 0xFFF8) == 0x4880) {
        return 1;
    }
    
    // EXT.L Dn - 0x48C0-0x48C7
    if ((opcode & 0xFFF8) == 0x48C0) {
        return 1;
    }
    
    // SWAP Dn - 0x4840-0x4847
    if ((opcode & 0xFFF8) == 0x4840) {
        return 1;
    }
    
    // MOVE.L Dn, Dm
    if ((opcode & 0xF1F8) == 0x2000) {
        return 1;
    }
    
    // ADD.L Dn, Dm
    if ((opcode & 0xF1F8) == 0xD080) {
        return 1;
    }
    
    // SUB.L Dn, Dm
    if ((opcode & 0xF1F8) == 0x9080) {
        return 1;
    }
    
    // AND.L Dn, Dm
    if ((opcode & 0xF1F8) == 0xC080) {
        return 1;
    }
    
    // OR.L Dn, Dm
    if ((opcode & 0xF1F8) == 0x8080) {
        return 1;
    }
    
    // EOR.L Dn, Dm
    if ((opcode & 0xF1F8) == 0xB180) {
        return 1;
    }
    
    // NOT.L Dn - 0x4680-0x4687
    if ((opcode & 0xFFF8) == 0x4680) {
        return 1;
    }
    
    // NEG.L Dn - 0x4480-0x4487
    if ((opcode & 0xFFF8) == 0x4480) {
        return 1;
    }
    
    // ADDQ.L #imm, Dn - 0x5080-0x50BF (data=0-7, reg=0-7)
    if ((opcode & 0xF1C0) == 0x5080) {
        int mode = (opcode >> 3) & 7;
        if (mode == 0) {  // Dn only
            return 1;
        }
    }
    
    // SUBQ.L #imm, Dn - 0x5180-0x51BF
    if ((opcode & 0xF1C0) == 0x5180) {
        int mode = (opcode >> 3) & 7;
        if (mode == 0) {  // Dn only
            return 1;
        }
    }
    
    // LSL/LSR/ASL/ASR immediate, Dn
    if ((opcode & 0xF018) == 0xE008 || (opcode & 0xF018) == 0xE000) {
        int size = (opcode >> 6) & 3;
        if (size == 2) {  // .L only
            return 1;
        }
    }
    
    return 0;
}

// Compile a single instruction
// Returns: bytes of 68k code consumed, or negative on error
static int compile_instruction(rv_emitter_t *emit, uint16_t opcode) {
    // Register base pointer is in A0 (first argument)
    // regs.d[n] is at offset n*4 (0-28 for D0-D7)
    // Offsets relative to regstruct base
    
    // MOVEQ #imm, Dn
    if ((opcode & 0xF100) == 0x7000) {
        int dreg = (opcode >> 9) & 7;
        int8_t imm = opcode & 0xFF;
        
        rv_emit_li(emit, RV_T0, (int32_t)imm);
        rv_emit_sw(emit, RV_T0, RV_A0, dreg * 4);  // Store to D[dreg]
        return 2;
    }
    
    // NOP
    if (opcode == 0x4E71) {
        // No code needed
        return 2;
    }
    
    // CLR.L Dn
    if ((opcode & 0xFFF8) == 0x4280) {
        int dreg = opcode & 7;
        rv_emit_sw(emit, RV_ZERO, RV_A0, dreg * 4);
        return 2;
    }
    
    // MOVE.L Dn, Dm - 0x2000 | (dst << 9) | src
    if ((opcode & 0xF1F8) == 0x2000) {
        int src = opcode & 7;
        int dst = (opcode >> 9) & 7;
        
        rv_emit_lw(emit, RV_T0, RV_A0, src * 4);
        rv_emit_sw(emit, RV_T0, RV_A0, dst * 4);
        return 2;
    }
    
    // ADD.L Dn, Dm
    if ((opcode & 0xF1F8) == 0xD080) {
        int src = opcode & 7;
        int dst = (opcode >> 9) & 7;
        
        rv_emit_lw(emit, RV_T0, RV_A0, src * 4);
        rv_emit_lw(emit, RV_T1, RV_A0, dst * 4);
        rv_emit_add(emit, RV_T0, RV_T0, RV_T1);
        rv_emit_sw(emit, RV_T0, RV_A0, dst * 4);
        return 2;
    }
    
    // SUB.L Dn, Dm
    if ((opcode & 0xF1F8) == 0x9080) {
        int src = opcode & 7;
        int dst = (opcode >> 9) & 7;
        
        rv_emit_lw(emit, RV_T0, RV_A0, dst * 4);
        rv_emit_lw(emit, RV_T1, RV_A0, src * 4);
        rv_emit_sub(emit, RV_T0, RV_T0, RV_T1);
        rv_emit_sw(emit, RV_T0, RV_A0, dst * 4);
        return 2;
    }
    
    // AND.L Dn, Dm
    if ((opcode & 0xF1F8) == 0xC080) {
        int src = opcode & 7;
        int dst = (opcode >> 9) & 7;
        
        rv_emit_lw(emit, RV_T0, RV_A0, src * 4);
        rv_emit_lw(emit, RV_T1, RV_A0, dst * 4);
        rv_emit_and(emit, RV_T0, RV_T0, RV_T1);
        rv_emit_sw(emit, RV_T0, RV_A0, dst * 4);
        return 2;
    }
    
    // OR.L Dn, Dm
    if ((opcode & 0xF1F8) == 0x8080) {
        int src = opcode & 7;
        int dst = (opcode >> 9) & 7;
        
        rv_emit_lw(emit, RV_T0, RV_A0, src * 4);
        rv_emit_lw(emit, RV_T1, RV_A0, dst * 4);
        rv_emit_or(emit, RV_T0, RV_T0, RV_T1);
        rv_emit_sw(emit, RV_T0, RV_A0, dst * 4);
        return 2;
    }
    
    // EOR.L Dn, Dm
    if ((opcode & 0xF1F8) == 0xB180) {
        int src = (opcode >> 9) & 7;
        int dst = opcode & 7;
        
        rv_emit_lw(emit, RV_T0, RV_A0, src * 4);
        rv_emit_lw(emit, RV_T1, RV_A0, dst * 4);
        rv_emit_xor(emit, RV_T0, RV_T0, RV_T1);
        rv_emit_sw(emit, RV_T0, RV_A0, dst * 4);
        return 2;
    }
    
    // NOT.L Dn
    if ((opcode & 0xFFF8) == 0x4680) {
        int dreg = opcode & 7;
        
        rv_emit_lw(emit, RV_T0, RV_A0, dreg * 4);
        rv_emit_not(emit, RV_T0, RV_T0);
        rv_emit_sw(emit, RV_T0, RV_A0, dreg * 4);
        return 2;
    }
    
    // NEG.L Dn
    if ((opcode & 0xFFF8) == 0x4480) {
        int dreg = opcode & 7;
        
        rv_emit_lw(emit, RV_T0, RV_A0, dreg * 4);
        rv_emit_neg(emit, RV_T0, RV_T0);
        rv_emit_sw(emit, RV_T0, RV_A0, dreg * 4);
        return 2;
    }
    
    // ADDQ.L #imm, Dn
    if ((opcode & 0xF1C0) == 0x5080) {
        int mode = (opcode >> 3) & 7;
        if (mode == 0) {
            int dreg = opcode & 7;
            int imm = (opcode >> 9) & 7;
            if (imm == 0) imm = 8;  // 0 encodes as 8
            
            rv_emit_lw(emit, RV_T0, RV_A0, dreg * 4);
            rv_emit_addi(emit, RV_T0, RV_T0, imm);
            rv_emit_sw(emit, RV_T0, RV_A0, dreg * 4);
            return 2;
        }
    }
    
    // SUBQ.L #imm, Dn
    if ((opcode & 0xF1C0) == 0x5180) {
        int mode = (opcode >> 3) & 7;
        if (mode == 0) {
            int dreg = opcode & 7;
            int imm = (opcode >> 9) & 7;
            if (imm == 0) imm = 8;
            
            rv_emit_lw(emit, RV_T0, RV_A0, dreg * 4);
            rv_emit_addi(emit, RV_T0, RV_T0, -imm);
            rv_emit_sw(emit, RV_T0, RV_A0, dreg * 4);
            return 2;
        }
    }
    
    // SWAP Dn
    if ((opcode & 0xFFF8) == 0x4840) {
        int dreg = opcode & 7;
        
        rv_emit_lw(emit, RV_T0, RV_A0, dreg * 4);
        // Swap upper and lower 16 bits
        rv_emit_srli(emit, RV_T1, RV_T0, 16);     // t1 = upper >> 16
        rv_emit_slli(emit, RV_T0, RV_T0, 16);     // t0 = lower << 16
        rv_emit_or(emit, RV_T0, RV_T0, RV_T1);    // t0 = combined
        rv_emit_sw(emit, RV_T0, RV_A0, dreg * 4);
        return 2;
    }
    
    // EXT.W Dn (extend byte to word)
    if ((opcode & 0xFFF8) == 0x4880) {
        int dreg = opcode & 7;
        
        rv_emit_lw(emit, RV_T0, RV_A0, dreg * 4);
        rv_emit_slli(emit, RV_T0, RV_T0, 24);     // Sign extend byte
        rv_emit_srai(emit, RV_T0, RV_T0, 16);     // to word (in upper 16 bits)
        rv_emit_lw(emit, RV_T1, RV_A0, dreg * 4); // Get original
        rv_emit_lui(emit, RV_T2, 0xFFFF0);        // Mask for upper word
        rv_emit_and(emit, RV_T1, RV_T1, RV_T2);   // Keep upper
        rv_emit_srli(emit, RV_T0, RV_T0, 16);     // Shift result to lower
        rv_emit_or(emit, RV_T0, RV_T0, RV_T1);
        rv_emit_sw(emit, RV_T0, RV_A0, dreg * 4);
        return 2;
    }
    
    // EXT.L Dn (extend word to long)
    if ((opcode & 0xFFF8) == 0x48C0) {
        int dreg = opcode & 7;
        
        rv_emit_lw(emit, RV_T0, RV_A0, dreg * 4);
        rv_emit_slli(emit, RV_T0, RV_T0, 16);     // Sign extend word
        rv_emit_srai(emit, RV_T0, RV_T0, 16);     // to long
        rv_emit_sw(emit, RV_T0, RV_A0, dreg * 4);
        return 2;
    }
    
    // TST.L Dn - just need to set flags, which we're not doing yet
    if ((opcode & 0xFFF8) == 0x4A80) {
        // No operation needed for now (flags not implemented)
        return 2;
    }
    
    // Shift instructions (LSL/LSR/ASL/ASR)
    if ((opcode & 0xF010) == 0xE000) {
        int size = (opcode >> 6) & 3;
        if (size == 2) {  // .L
            int dreg = opcode & 7;
            int count_or_reg = (opcode >> 9) & 7;
            int direction = (opcode >> 8) & 1;  // 0=right, 1=left
            int ir = (opcode >> 5) & 1;         // 0=immediate, 1=register
            int type = (opcode >> 3) & 3;       // 0=ASL/R, 1=LSL/R, 2=ROXL/R, 3=ROL/R
            
            if (ir == 0) {
                // Immediate count
                int count = count_or_reg;
                if (count == 0) count = 8;
                
                rv_emit_lw(emit, RV_T0, RV_A0, dreg * 4);
                
                if (direction == 1) {
                    // Left shift
                    rv_emit_slli(emit, RV_T0, RV_T0, count);
                } else {
                    // Right shift
                    if (type == 0) {
                        // Arithmetic
                        rv_emit_srai(emit, RV_T0, RV_T0, count);
                    } else {
                        // Logical
                        rv_emit_srli(emit, RV_T0, RV_T0, count);
                    }
                }
                
                rv_emit_sw(emit, RV_T0, RV_A0, dreg * 4);
                return 2;
            }
        }
    }
    
    return JIT_ERR_UNSUPPORTED;
}

// Compile a basic block starting at the given PC
void *jit_compile_block(uint32_t m68k_pc) {
    if (!jit_cache_is_enabled()) {
        return NULL;
    }
    
    // Get pointer to 68k code
    uint8_t *code_ptr = get_real_address(m68k_pc);
    if (code_ptr == NULL) {
        return NULL;
    }
    
    // Initialize emitter with temporary buffer
    rv_emitter_t emit;
    rv_emit_init(&emit, temp_code_buffer, sizeof(temp_code_buffer));
    
    // Prologue: no stack frame needed, A0 already has regs pointer
    // The calling convention passes first arg in A0
    
    // Compile instructions until we hit one we can't compile
    // or a control flow instruction
    uint32_t pc = m68k_pc;
    int instructions = 0;
    int total_bytes = 0;
    
    while (instructions < JIT_MAX_BLOCK_INSTRUCTIONS) {
        uint16_t opcode = (code_ptr[0] << 8) | code_ptr[1];
        
        // Stop at control flow instructions
        // BRA, Bcc, BSR, JMP, JSR, RTS, RTR, RTE, TRAP, etc.
        if ((opcode & 0xF000) == 0x6000 ||  // Bcc, BRA, BSR
            opcode == 0x4E75 ||              // RTS
            opcode == 0x4E73 ||              // RTE
            opcode == 0x4E77 ||              // RTR
            (opcode & 0xFFC0) == 0x4EC0 ||   // JMP
            (opcode & 0xFFC0) == 0x4E80 ||   // JSR
            (opcode & 0xFFF0) == 0x4E40) {   // TRAP
            break;
        }
        
        // Try to compile the instruction
        int result = compile_instruction(&emit, opcode);
        if (result < 0) {
            // Can't compile this instruction
            break;
        }
        
        code_ptr += result;
        total_bytes += result;
        pc += result;
        instructions++;
    }
    
    // Need at least one instruction
    if (instructions == 0) {
        return NULL;
    }
    
    // Epilogue: return the number of instructions executed
    rv_emit_li(&emit, RV_A0, instructions);
    rv_emit_ret(&emit);
    
    // Get final code size
    size_t code_size = rv_emit_get_size(&emit);
    
    // Allocate permanent storage in the cache
    void *final_code = jit_cache_alloc(code_size);
    if (final_code == NULL) {
        return NULL;
    }
    
    // Copy code to cache
    memcpy(final_code, temp_code_buffer, code_size);
    
    // Sync caches
#ifdef ARDUINO
    esp_cache_msync(final_code, code_size, ESP_CACHE_MSYNC_FLAG_TYPE_INST);
#endif
    
    // Register the block
    jit_cache_register(m68k_pc, total_bytes, final_code, code_size);
    jit_blocks_compiled++;
    
#ifdef ARDUINO
    // Log first few compilations
    if (jit_blocks_compiled <= 5) {
        Serial.printf("[JIT] Compiled block: PC=0x%08X, %d instrs, %d bytes -> %d bytes native\n",
                      m68k_pc, instructions, total_bytes, (int)code_size);
    }
#endif
    
    return final_code;
}

// Function pointer type for JIT blocks
typedef int (*jit_func_t)(void *regs_base);

// Execute JIT compiled code for the given PC
// Returns: number of instructions executed (>0), or 0 if no JIT available
int jit_execute(uint32_t m68k_pc) {
    if (!jit_cache_is_enabled()) {
        return 0;
    }
    
    // Look up compiled code
    void *code = jit_cache_lookup(m68k_pc);
    
    if (code == NULL) {
        // Try to compile
        code = jit_compile_block(m68k_pc);
        if (code == NULL) {
            jit_fallbacks++;
            return 0;  // No JIT code available
        }
    }
    
    // Execute the compiled code
    // Pass pointer to regs.regs[0] as the register base
    // D0-D7 are at offsets 0-28, A0-A7 are at offsets 32-60
    jit_func_t func = (jit_func_t)code;
    int instructions = func(&regs.regs[0]);
    
    jit_blocks_executed++;
    return instructions;
}

void jit_print_stats(void) {
#ifdef ARDUINO
    Serial.println("========== JIT COMPILER STATS ==========");
    Serial.printf("[JIT] Blocks compiled: %u\n", jit_blocks_compiled);
    Serial.printf("[JIT] Blocks executed: %u\n", jit_blocks_executed);
    Serial.printf("[JIT] Interpreter fallbacks: %u\n", jit_fallbacks);
    
    float hit_rate = 0;
    if (jit_blocks_executed + jit_fallbacks > 0) {
        hit_rate = 100.0f * jit_blocks_executed / (jit_blocks_executed + jit_fallbacks);
    }
    Serial.printf("[JIT] JIT hit rate: %.1f%%\n", hit_rate);
#endif
    
    jit_cache_print_stats();
}
