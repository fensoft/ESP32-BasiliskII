/*
 *  jit_cache.h - JIT translation cache for 68k to RISC-V
 *
 *  BasiliskII ESP32-P4 JIT Compiler
 *
 *  Manages the cache of translated 68k code blocks.
 *  Uses PSRAM with XIP (Execute In Place) for code storage.
 */

#ifndef JIT_CACHE_H
#define JIT_CACHE_H

#include <stdint.h>
#include <stddef.h>
#include "rv32_emitter.h"

#ifdef __cplusplus
extern "C" {
#endif

// Cache configuration
// With CONFIG_SPIRAM_XIP_FROM_PSRAM enabled, code can execute from PSRAM
#define JIT_CACHE_SIZE          (2 * 1024 * 1024)  // 2MB translation cache in PSRAM
#define JIT_LOOKUP_TABLE_SIZE   (64 * 1024)        // 64K entry lookup table
#define JIT_MAX_BLOCK_SIZE      (4 * 1024)         // Max 4KB per translated block
#define JIT_MIN_BLOCK_SIZE      (64)               // Minimum block size

// Lookup table entry - maps 68k PC to native code
typedef struct {
    uint32_t m68k_pc;          // 68k program counter (key)
    void    *native_code;      // Pointer to translated RISC-V code
    uint16_t native_size;      // Size of translated code in bytes
    uint16_t m68k_size;        // Size of original 68k block in bytes
    uint32_t exec_count;       // Execution count for profiling
} jit_block_t;

// JIT cache state
typedef struct {
    // Code cache
    uint8_t    *code_base;     // Base address of code cache
    uint8_t    *code_ptr;      // Current allocation pointer
    uint8_t    *code_end;      // End of code cache
    size_t      code_used;     // Bytes used in code cache
    
    // Block lookup table (simple direct-mapped for now)
    jit_block_t *blocks;       // Array of block entries
    uint32_t     block_count;  // Number of valid blocks
    uint32_t     block_capacity;
    
    // Statistics
    uint64_t    cache_hits;
    uint64_t    cache_misses;
    uint64_t    compilations;
    uint64_t    invalidations;
    
    // State flags
    uint8_t     initialized;
    uint8_t     enabled;
} jit_cache_t;

// Global JIT cache instance
extern jit_cache_t jit_cache;

// Initialize the JIT cache
// Returns 0 on success, -1 on failure
int jit_cache_init(void);

// Shutdown the JIT cache and free resources
void jit_cache_shutdown(void);

// Enable/disable JIT compilation
void jit_cache_enable(int enable);

// Check if JIT is enabled
int jit_cache_is_enabled(void);

// Look up a translated block by 68k PC
// Returns pointer to native code, or NULL if not found
void *jit_cache_lookup(uint32_t m68k_pc);

// Allocate space for a new translated block
// Returns pointer to code buffer, or NULL if cache is full
void *jit_cache_alloc(size_t size);

// Register a newly compiled block
// Returns 0 on success, -1 on failure
int jit_cache_register(uint32_t m68k_pc, uint16_t m68k_size, 
                       void *native_code, uint16_t native_size);

// Invalidate a block (e.g., due to self-modifying code)
void jit_cache_invalidate(uint32_t m68k_pc);

// Invalidate all blocks in a memory range
void jit_cache_invalidate_range(uint32_t start, uint32_t end);

// Flush the entire cache
void jit_cache_flush(void);

// Get cache statistics
void jit_cache_get_stats(uint64_t *hits, uint64_t *misses, 
                         uint64_t *compilations, size_t *bytes_used);

// Print cache statistics to serial
void jit_cache_print_stats(void);

// Execute a JIT-compiled block
// Returns: 0 = block executed, >0 = instructions executed before exit, <0 = error
typedef int (*jit_block_func)(void *regs_ptr);

#ifdef __cplusplus
}
#endif

#endif // JIT_CACHE_H
