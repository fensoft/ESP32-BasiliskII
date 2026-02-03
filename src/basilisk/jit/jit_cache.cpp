/*
 *  jit_cache.cpp - JIT translation cache for 68k to RISC-V
 *
 *  BasiliskII ESP32-P4 JIT Compiler
 */

#include "jit_cache.h"
#include <stdlib.h>
#include <string.h>

#ifdef ARDUINO
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_cache.h>
#endif

// Global JIT cache instance
jit_cache_t jit_cache;

// Simple hash function for block lookup
static inline uint32_t hash_pc(uint32_t pc) {
    // Mix the bits for better distribution
    pc ^= (pc >> 16);
    pc *= 0x85ebca6b;
    pc ^= (pc >> 13);
    return pc % JIT_LOOKUP_TABLE_SIZE;
}

int jit_cache_init(void) {
    memset(&jit_cache, 0, sizeof(jit_cache));
    
#ifdef ARDUINO
    // Allocate code cache in PSRAM
    // With CONFIG_SPIRAM_XIP_FROM_PSRAM=y, code can execute directly from PSRAM
    jit_cache.code_base = (uint8_t *)heap_caps_aligned_alloc(
        64,  // 64-byte alignment for cache line
        JIT_CACHE_SIZE, 
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );
    
    if (jit_cache.code_base == NULL) {
        Serial.println("[JIT] ERROR: Failed to allocate code cache in PSRAM");
        return -1;
    }
    
    Serial.printf("[JIT] Allocated %d KB code cache at 0x%08X (PSRAM with XIP)\n", 
                  JIT_CACHE_SIZE / 1024, (uint32_t)jit_cache.code_base);
    
    // Allocate lookup table in PSRAM
    jit_cache.blocks = (jit_block_t *)heap_caps_calloc(
        JIT_LOOKUP_TABLE_SIZE, 
        sizeof(jit_block_t),
        MALLOC_CAP_SPIRAM
    );
    
    if (jit_cache.blocks == NULL) {
        Serial.println("[JIT] ERROR: Failed to allocate lookup table");
        heap_caps_free(jit_cache.code_base);
        jit_cache.code_base = NULL;
        return -1;
    }
    
    Serial.printf("[JIT] Allocated %d KB lookup table (%d entries)\n",
                  (JIT_LOOKUP_TABLE_SIZE * sizeof(jit_block_t)) / 1024,
                  JIT_LOOKUP_TABLE_SIZE);
#else
    // Non-Arduino platforms (for testing)
    jit_cache.code_base = (uint8_t *)aligned_alloc(64, JIT_CACHE_SIZE);
    jit_cache.blocks = (jit_block_t *)calloc(JIT_LOOKUP_TABLE_SIZE, sizeof(jit_block_t));
    
    if (jit_cache.code_base == NULL || jit_cache.blocks == NULL) {
        free(jit_cache.code_base);
        free(jit_cache.blocks);
        return -1;
    }
#endif
    
    jit_cache.code_ptr = jit_cache.code_base;
    jit_cache.code_end = jit_cache.code_base + JIT_CACHE_SIZE;
    jit_cache.code_used = 0;
    jit_cache.block_count = 0;
    jit_cache.block_capacity = JIT_LOOKUP_TABLE_SIZE;
    
    jit_cache.cache_hits = 0;
    jit_cache.cache_misses = 0;
    jit_cache.compilations = 0;
    jit_cache.invalidations = 0;
    
    jit_cache.initialized = 1;
    jit_cache.enabled = 0;  // Start disabled, enable explicitly
    
#ifdef ARDUINO
    Serial.println("[JIT] Cache initialized successfully");
#endif
    
    return 0;
}

void jit_cache_shutdown(void) {
    if (!jit_cache.initialized) return;
    
#ifdef ARDUINO
    if (jit_cache.code_base) {
        heap_caps_free(jit_cache.code_base);
    }
    if (jit_cache.blocks) {
        heap_caps_free(jit_cache.blocks);
    }
    Serial.println("[JIT] Cache shutdown");
#else
    free(jit_cache.code_base);
    free(jit_cache.blocks);
#endif
    
    memset(&jit_cache, 0, sizeof(jit_cache));
}

void jit_cache_enable(int enable) {
    jit_cache.enabled = enable ? 1 : 0;
#ifdef ARDUINO
    Serial.printf("[JIT] JIT compilation %s\n", enable ? "ENABLED" : "DISABLED");
#endif
}

int jit_cache_is_enabled(void) {
    return jit_cache.initialized && jit_cache.enabled;
}

void *jit_cache_lookup(uint32_t m68k_pc) {
    if (!jit_cache_is_enabled()) {
        return NULL;
    }
    
    uint32_t idx = hash_pc(m68k_pc);
    jit_block_t *block = &jit_cache.blocks[idx];
    
    if (block->native_code != NULL && block->m68k_pc == m68k_pc) {
        // Cache hit
        jit_cache.cache_hits++;
        block->exec_count++;
        return block->native_code;
    }
    
    // Cache miss - try linear probe
    for (int i = 1; i < 8; i++) {
        uint32_t probe_idx = (idx + i) % JIT_LOOKUP_TABLE_SIZE;
        block = &jit_cache.blocks[probe_idx];
        
        if (block->native_code != NULL && block->m68k_pc == m68k_pc) {
            jit_cache.cache_hits++;
            block->exec_count++;
            return block->native_code;
        }
    }
    
    jit_cache.cache_misses++;
    return NULL;
}

void *jit_cache_alloc(size_t size) {
    if (!jit_cache.initialized) return NULL;
    
    // Align to 4 bytes (RISC-V instruction size)
    size = (size + 3) & ~3;
    
    if (size > JIT_MAX_BLOCK_SIZE) {
        return NULL;  // Block too large
    }
    
    if (jit_cache.code_ptr + size > jit_cache.code_end) {
        // Cache full - could implement LRU eviction here
        return NULL;
    }
    
    void *ptr = jit_cache.code_ptr;
    jit_cache.code_ptr += size;
    jit_cache.code_used += size;
    
    return ptr;
}

int jit_cache_register(uint32_t m68k_pc, uint16_t m68k_size,
                       void *native_code, uint16_t native_size) {
    if (!jit_cache.initialized) return -1;
    
    uint32_t idx = hash_pc(m68k_pc);
    jit_block_t *block = &jit_cache.blocks[idx];
    
    // Check if slot is free or needs collision handling
    if (block->native_code != NULL && block->m68k_pc != m68k_pc) {
        // Collision - try linear probing
        for (int i = 1; i < 8; i++) {
            uint32_t probe_idx = (idx + i) % JIT_LOOKUP_TABLE_SIZE;
            jit_block_t *probe = &jit_cache.blocks[probe_idx];
            
            if (probe->native_code == NULL) {
                block = probe;
                break;
            }
        }
        
        if (block->native_code != NULL && block->m68k_pc != m68k_pc) {
            // No free slot found in probe range
            // Evict the original slot (simple policy)
            jit_cache.invalidations++;
        }
    }
    
    block->m68k_pc = m68k_pc;
    block->native_code = native_code;
    block->native_size = native_size;
    block->m68k_size = m68k_size;
    block->exec_count = 0;
    
    jit_cache.block_count++;
    jit_cache.compilations++;
    
#ifdef ARDUINO
    // Synchronize instruction cache for the new code
    // This is critical for XIP - the CPU cache needs to see the new code
    esp_cache_msync(native_code, native_size, ESP_CACHE_MSYNC_FLAG_TYPE_INST);
#endif
    
    return 0;
}

void jit_cache_invalidate(uint32_t m68k_pc) {
    if (!jit_cache.initialized) return;
    
    uint32_t idx = hash_pc(m68k_pc);
    
    // Check primary slot and probes
    for (int i = 0; i < 8; i++) {
        uint32_t check_idx = (idx + i) % JIT_LOOKUP_TABLE_SIZE;
        jit_block_t *block = &jit_cache.blocks[check_idx];
        
        if (block->m68k_pc == m68k_pc && block->native_code != NULL) {
            block->native_code = NULL;
            block->m68k_pc = 0;
            jit_cache.invalidations++;
            return;
        }
    }
}

void jit_cache_invalidate_range(uint32_t start, uint32_t end) {
    if (!jit_cache.initialized) return;
    
    // Linear scan - could be optimized with sorted block list
    for (uint32_t i = 0; i < JIT_LOOKUP_TABLE_SIZE; i++) {
        jit_block_t *block = &jit_cache.blocks[i];
        if (block->native_code != NULL) {
            uint32_t block_end = block->m68k_pc + block->m68k_size;
            // Check if block overlaps with invalidation range
            if (block->m68k_pc < end && block_end > start) {
                block->native_code = NULL;
                block->m68k_pc = 0;
                jit_cache.invalidations++;
            }
        }
    }
}

void jit_cache_flush(void) {
    if (!jit_cache.initialized) return;
    
    // Clear all blocks
    memset(jit_cache.blocks, 0, JIT_LOOKUP_TABLE_SIZE * sizeof(jit_block_t));
    
    // Reset code pointer
    jit_cache.code_ptr = jit_cache.code_base;
    jit_cache.code_used = 0;
    jit_cache.block_count = 0;
    
    jit_cache.invalidations++;
    
#ifdef ARDUINO
    Serial.println("[JIT] Cache flushed");
#endif
}

void jit_cache_get_stats(uint64_t *hits, uint64_t *misses,
                         uint64_t *compilations, size_t *bytes_used) {
    if (hits) *hits = jit_cache.cache_hits;
    if (misses) *misses = jit_cache.cache_misses;
    if (compilations) *compilations = jit_cache.compilations;
    if (bytes_used) *bytes_used = jit_cache.code_used;
}

void jit_cache_print_stats(void) {
#ifdef ARDUINO
    if (!jit_cache.initialized) {
        Serial.println("[JIT] Not initialized");
        return;
    }
    
    uint64_t total = jit_cache.cache_hits + jit_cache.cache_misses;
    float hit_rate = 0.0f;
    if (total > 0) {
        hit_rate = (float)jit_cache.cache_hits * 100.0f / (float)total;
    }
    
    Serial.println("========== JIT CACHE STATS ==========");
    Serial.printf("[JIT] Status: %s\n", jit_cache.enabled ? "ENABLED" : "DISABLED");
    Serial.printf("[JIT] Blocks: %u compiled\n", (unsigned)jit_cache.compilations);
    Serial.printf("[JIT] Cache: %u KB used / %u KB total\n",
                  (unsigned)(jit_cache.code_used / 1024),
                  (unsigned)(JIT_CACHE_SIZE / 1024));
    Serial.printf("[JIT] Hit rate: %.1f%% (%llu hits, %llu misses)\n",
                  hit_rate,
                  (unsigned long long)jit_cache.cache_hits,
                  (unsigned long long)jit_cache.cache_misses);
    Serial.printf("[JIT] Invalidations: %llu\n",
                  (unsigned long long)jit_cache.invalidations);
    Serial.println("=====================================");
#endif
}
