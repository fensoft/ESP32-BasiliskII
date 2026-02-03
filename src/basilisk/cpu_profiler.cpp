/*
 *  cpu_profiler.cpp - CPU emulation performance profiler
 *
 *  BasiliskII ESP32 Port
 *
 *  Measures where time is spent during 68k emulation to identify
 *  whether the bottleneck is PSRAM bandwidth or CPU execution.
 */

#include "cpu_profiler.h"
#include <Arduino.h>

// Profiling counters
volatile uint64_t prof_cycles_fetch = 0;
volatile uint64_t prof_cycles_execute = 0;
volatile uint32_t prof_mem_reads_ram = 0;
volatile uint32_t prof_mem_reads_rom = 0;
volatile uint32_t prof_mem_writes_ram = 0;
volatile uint32_t prof_mem_writes_fb = 0;
volatile uint64_t prof_bytes_read = 0;
volatile uint64_t prof_bytes_written = 0;
volatile bool prof_enabled = false;

// Opcode frequency tracking
struct opcode_freq_entry prof_opcode_freq[PROF_OPCODE_SLOTS];
volatile uint32_t prof_opcode_total = 0;
static uint32_t opcode_sample_counter = 0;
#define OPCODE_SAMPLE_RATE 64  // Sample every 64th instruction

// Report timing
static uint32_t last_report_time = 0;
static uint64_t last_total_instructions = 0;
#define PROFILER_REPORT_INTERVAL_MS 5000

// External function to get instruction count (defined in main_esp32.cpp)
extern uint64_t getEmulatorTotalInstructions(void);

bool cpu_profiler_is_enabled(void) {
    return prof_enabled;
}

void cpu_profiler_init(bool enabled) {
    prof_enabled = enabled;
    cpu_profiler_reset();
    last_report_time = millis();
    last_total_instructions = 0;
    
    if (enabled) {
        Serial.println("[PROFILER] CPU profiler ENABLED - expect ~5-10% performance overhead");
    } else {
        Serial.println("[PROFILER] CPU profiler disabled");
    }
}

void cpu_profiler_reset(void) {
    prof_cycles_fetch = 0;
    prof_cycles_execute = 0;
    prof_mem_reads_ram = 0;
    prof_mem_reads_rom = 0;
    prof_mem_writes_ram = 0;
    prof_mem_writes_fb = 0;
    prof_bytes_read = 0;
    prof_bytes_written = 0;
    // Note: Don't reset opcode frequency - we want cumulative data
}

void cpu_profiler_reset_opcodes(void) {
    for (int i = 0; i < PROF_OPCODE_SLOTS; i++) {
        prof_opcode_freq[i].opcode = 0;
        prof_opcode_freq[i].count = 0;
    }
    prof_opcode_total = 0;
    opcode_sample_counter = 0;
}

// Record an opcode execution (sampled)
void prof_record_opcode(uint16_t opcode) {
    if (!prof_enabled) return;
    
    // Sample only every Nth instruction to reduce overhead
    opcode_sample_counter++;
    if (opcode_sample_counter < OPCODE_SAMPLE_RATE) return;
    opcode_sample_counter = 0;
    
    prof_opcode_total++;
    
    // Look for existing slot
    for (int i = 0; i < PROF_OPCODE_SLOTS; i++) {
        if (prof_opcode_freq[i].opcode == opcode) {
            prof_opcode_freq[i].count++;
            return;
        }
    }
    
    // Look for empty slot
    for (int i = 0; i < PROF_OPCODE_SLOTS; i++) {
        if (prof_opcode_freq[i].count == 0) {
            prof_opcode_freq[i].opcode = opcode;
            prof_opcode_freq[i].count = 1;
            return;
        }
    }
    
    // Replace lowest count slot if new opcode seems more common
    int min_idx = 0;
    uint32_t min_count = prof_opcode_freq[0].count;
    for (int i = 1; i < PROF_OPCODE_SLOTS; i++) {
        if (prof_opcode_freq[i].count < min_count) {
            min_count = prof_opcode_freq[i].count;
            min_idx = i;
        }
    }
    
    // Only replace if this is at least the second time we've seen this opcode
    // (heuristic to avoid thrashing)
    if (min_count < 2) {
        prof_opcode_freq[min_idx].opcode = opcode;
        prof_opcode_freq[min_idx].count = 1;
    }
}

void cpu_profiler_report(void) {
    if (!prof_enabled) {
        return;
    }
    
    uint32_t now = millis();
    if (now - last_report_time < PROFILER_REPORT_INTERVAL_MS) {
        return;
    }
    
    uint64_t total_instr = getEmulatorTotalInstructions();
    uint64_t instr_delta = total_instr - last_total_instructions;
    
    if (instr_delta == 0) {
        last_report_time = now;
        return;
    }
    
    // Calculate averages
    uint32_t avg_fetch_cycles = (uint32_t)(prof_cycles_fetch / instr_delta);
    uint32_t avg_exec_cycles = (uint32_t)(prof_cycles_execute / instr_delta);
    uint32_t total_cycles = avg_fetch_cycles + avg_exec_cycles;
    
    // Calculate percentages
    uint32_t fetch_pct = 0;
    uint32_t exec_pct = 0;
    if (total_cycles > 0) {
        fetch_pct = (avg_fetch_cycles * 100) / total_cycles;
        exec_pct = 100 - fetch_pct;
    }
    
    // Calculate bandwidth (bytes per second)
    uint32_t time_delta_ms = now - last_report_time;
    uint64_t read_bw = 0;
    uint64_t write_bw = 0;
    if (time_delta_ms > 0) {
        read_bw = (prof_bytes_read * 1000) / time_delta_ms;
        write_bw = (prof_bytes_written * 1000) / time_delta_ms;
    }
    
    // Memory accesses per instruction
    uint32_t total_mem_accesses = prof_mem_reads_ram + prof_mem_reads_rom + 
                                   prof_mem_writes_ram + prof_mem_writes_fb;
    float mem_per_instr = (float)total_mem_accesses / instr_delta;
    
    // Print report
    Serial.println();
    Serial.println("========== CPU PROFILER ==========");
    Serial.printf("[PROF] Cycles/instr: fetch=%u (%u%%) exec=%u (%u%%) total=%u\n",
                  avg_fetch_cycles, fetch_pct, avg_exec_cycles, exec_pct, total_cycles);
    Serial.printf("[PROF] Memory ops: RAM_R=%u ROM_R=%u RAM_W=%u FB_W=%u (%.1f/instr)\n",
                  prof_mem_reads_ram, prof_mem_reads_rom, 
                  prof_mem_writes_ram, prof_mem_writes_fb, mem_per_instr);
    Serial.printf("[PROF] PSRAM bandwidth: read=%lluKB/s write=%lluKB/s total=%lluKB/s\n",
                  (unsigned long long)(read_bw / 1024), 
                  (unsigned long long)(write_bw / 1024),
                  (unsigned long long)((read_bw + write_bw) / 1024));
    
    // Bottleneck analysis
    if (total_cycles > 0) {
        if (fetch_pct > 60) {
            Serial.println("[PROF] >>> BOTTLENECK: PSRAM (instruction fetch dominates)");
            Serial.println("[PROF]     Recommend: ROM caching, instruction prefetch");
        } else if (exec_pct > 70) {
            Serial.println("[PROF] >>> BOTTLENECK: CPU (instruction execution dominates)");
            Serial.println("[PROF]     Recommend: Compiler optimization, threaded interpretation");
        } else {
            Serial.println("[PROF] >>> MIXED: Both PSRAM and CPU contribute");
            Serial.println("[PROF]     Recommend: Profile specific instruction types");
        }
    }
    
    // Sort opcode frequency (bubble sort, array is small)
    for (int i = 0; i < PROF_OPCODE_SLOTS - 1; i++) {
        for (int j = 0; j < PROF_OPCODE_SLOTS - i - 1; j++) {
            if (prof_opcode_freq[j].count < prof_opcode_freq[j + 1].count) {
                struct opcode_freq_entry temp = prof_opcode_freq[j];
                prof_opcode_freq[j] = prof_opcode_freq[j + 1];
                prof_opcode_freq[j + 1] = temp;
            }
        }
    }
    
    // Report top 10 opcodes
    if (prof_opcode_total > 0) {
        Serial.println("[PROF] Top 10 opcodes (sampled):");
        for (int i = 0; i < 10 && i < PROF_OPCODE_SLOTS; i++) {
            if (prof_opcode_freq[i].count == 0) break;
            uint32_t pct = (prof_opcode_freq[i].count * 100) / prof_opcode_total;
            Serial.printf("[PROF]   %04X: %5u (%2u%%)\n", 
                         prof_opcode_freq[i].opcode, 
                         prof_opcode_freq[i].count,
                         pct);
        }
    }
    
    Serial.println("==================================");
    Serial.println();
    
    // Reset for next interval
    cpu_profiler_reset();
    last_report_time = now;
    last_total_instructions = total_instr;
}
