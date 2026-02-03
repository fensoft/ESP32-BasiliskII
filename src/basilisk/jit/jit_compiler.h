/*
 *  jit_compiler.h - 68k to RISC-V JIT compiler
 *
 *  BasiliskII ESP32-P4 JIT Compiler
 *
 *  Translates 68k instruction sequences into native RISC-V code.
 */

#ifndef JIT_COMPILER_H
#define JIT_COMPILER_H

#include <stdint.h>
#include "rv32_emitter.h"
#include "jit_cache.h"

#ifdef __cplusplus
extern "C" {
#endif

// Compilation result codes
#define JIT_OK              0
#define JIT_ERR_UNSUPPORTED -1  // Instruction not supported for JIT
#define JIT_ERR_OVERFLOW    -2  // Code buffer overflow
#define JIT_ERR_INVALID     -3  // Invalid instruction

// Maximum instructions to compile in a basic block
#define JIT_MAX_BLOCK_INSTRUCTIONS 64

// Compiler context
typedef struct {
    rv_emitter_t emitter;       // RISC-V code emitter
    uint32_t     m68k_pc;       // Current 68k PC being compiled
    uint32_t     m68k_pc_start; // Start PC of current block
    uint8_t     *m68k_code;     // Pointer to 68k code in memory
    int          instr_count;   // Instructions compiled so far
    
    // Register allocation state
    // Tracks which 68k registers are currently in RISC-V registers
    uint8_t      reg_dirty[16]; // Which regs need writeback
    
    // Flags state for lazy flag evaluation
    uint8_t      flags_valid;   // Are cached flags valid?
    uint8_t      last_op_type;  // Type of last operation that set flags
    uint32_t     last_result;   // Result of last operation (for lazy N/Z)
} jit_compiler_t;

// Initialize the JIT compiler subsystem
int jit_init(void);

// Shutdown the JIT compiler
void jit_shutdown(void);

// Compile a basic block starting at the given 68k PC
// Returns pointer to compiled code, or NULL on failure
void *jit_compile_block(uint32_t m68k_pc);

// Try to execute JIT compiled code for the given PC
// Returns: 1 = JIT code executed, 0 = no JIT code available (use interpreter)
int jit_execute(uint32_t m68k_pc);

// Check if an instruction can be JIT compiled
int jit_can_compile(uint16_t opcode);

// Compile a single 68k instruction
// Returns number of bytes of 68k code consumed, or negative on error
int jit_compile_instruction(jit_compiler_t *ctx, uint16_t opcode);

// ========== Statistics ==========

void jit_print_stats(void);

#ifdef __cplusplus
}
#endif

#endif // JIT_COMPILER_H
