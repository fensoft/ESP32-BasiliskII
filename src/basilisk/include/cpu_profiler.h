/*
 *  cpu_profiler.h - CPU emulation performance profiler
 *
 *  BasiliskII ESP32 Port
 *
 *  Measures where time is spent during 68k emulation to identify
 *  whether the bottleneck is PSRAM bandwidth or CPU execution.
 */

#ifndef CPU_PROFILER_H
#define CPU_PROFILER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Check if profiler is enabled at runtime (configured in boot GUI)
bool cpu_profiler_is_enabled(void);

// Initialize the profiler (call once at startup)
void cpu_profiler_init(bool enabled);

// Call periodically to report stats (every 5 seconds)
void cpu_profiler_report(void);

// Reset all counters
void cpu_profiler_reset(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

// Only include profiler internals when compiling C++ code
#if defined(ARDUINO) || defined(ESP_PLATFORM)
#include <esp_cpu.h>
#endif

// Profiling counters (updated in hot loop)
extern volatile uint64_t prof_cycles_fetch;      // Cycles spent fetching opcodes
extern volatile uint64_t prof_cycles_execute;    // Cycles spent executing handlers
extern volatile uint32_t prof_mem_reads_ram;     // RAM read count
extern volatile uint32_t prof_mem_reads_rom;     // ROM read count
extern volatile uint32_t prof_mem_writes_ram;    // RAM write count
extern volatile uint32_t prof_mem_writes_fb;     // Frame buffer write count
extern volatile uint64_t prof_bytes_read;        // Total bytes read from PSRAM
extern volatile uint64_t prof_bytes_written;     // Total bytes written to PSRAM
extern volatile bool prof_enabled;               // Runtime enable flag

// Opcode frequency tracking for JIT optimization
#define PROF_OPCODE_SLOTS 32   // Track top 32 opcodes
struct opcode_freq_entry {
    uint16_t opcode;
    uint32_t count;
};
extern struct opcode_freq_entry prof_opcode_freq[PROF_OPCODE_SLOTS];
extern volatile uint32_t prof_opcode_total;      // Total opcodes tracked

// Record an opcode execution (sampled, not every instruction)
void prof_record_opcode(uint16_t opcode);

// Inline cycle counter (ESP32-P4 RISC-V)
static inline uint32_t prof_get_cycles(void) {
#if defined(ARDUINO) || defined(ESP_PLATFORM)
    return esp_cpu_get_cycle_count();
#else
    return 0;
#endif
}

// Macros for hot-path instrumentation
// These compile to nothing when profiler is disabled at runtime
#define PROF_FETCH_START() \
    uint32_t _pf_t0 = 0; \
    if (prof_enabled) { _pf_t0 = prof_get_cycles(); }

#define PROF_FETCH_END() \
    if (prof_enabled) { prof_cycles_fetch += (prof_get_cycles() - _pf_t0); }

#define PROF_EXEC_START() \
    uint32_t _pe_t0 = 0; \
    if (prof_enabled) { _pe_t0 = prof_get_cycles(); }

#define PROF_EXEC_END() \
    if (prof_enabled) { prof_cycles_execute += (prof_get_cycles() - _pe_t0); }

#define PROF_MEM_READ_RAM(bytes) \
    if (prof_enabled) { prof_mem_reads_ram = prof_mem_reads_ram + 1; prof_bytes_read += (bytes); }

#define PROF_MEM_READ_ROM(bytes) \
    if (prof_enabled) { prof_mem_reads_rom = prof_mem_reads_rom + 1; prof_bytes_read += (bytes); }

#define PROF_MEM_WRITE_RAM(bytes) \
    if (prof_enabled) { prof_mem_writes_ram = prof_mem_writes_ram + 1; prof_bytes_written += (bytes); }

#define PROF_MEM_WRITE_FB(bytes) \
    if (prof_enabled) { prof_mem_writes_fb = prof_mem_writes_fb + 1; prof_bytes_written += (bytes); }

#endif // __cplusplus

#endif // CPU_PROFILER_H
