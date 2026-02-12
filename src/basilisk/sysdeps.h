/*
 *  sysdeps.h - System dependent definitions for ESP32-P4
 *
 *  BasiliskII ESP32 Port
 *  Based on Basilisk II (C) 1997-2008 Christian Bauer
 */

#ifndef SYSDEPS_H
#define SYSDEPS_H

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// FreeRTOS for mutex support
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// C++ STL headers needed by BasiliskII
#include <vector>
#include <map>
using std::vector;
// Note: Don't use "using std::map" as it conflicts with Arduino's map() function
// Use std::map<> explicitly in code instead

// Include ESP32 user strings
#include "user_strings_esp32.h"

/*
 * CPU and addressing mode configuration
 */

// Using 68k emulator (not native 68k CPU)
#define EMULATED_68K 1

// Mac and host address space are distinct (virtual addressing)
#define REAL_ADDRESSING 0

// Use bank-based memory access (DIRECT_ADDRESSING requires contiguous memory layout)
#define DIRECT_ADDRESSING 0

// ROM is write protected in virtual addressing mode
#define ROM_IS_WRITE_PROTECTED 1

// No prefetch buffer needed
#define USE_PREFETCH_BUFFER 0

// ExtFS not supported on ESP32
#define SUPPORTS_EXTFS 0

// No UDP tunnel support
#define SUPPORTS_UDP_TUNNEL 0

// Use CPU emulation for periodic tasks (no threads)
#define USE_CPU_EMUL_SERVICES 1

/*
 * ESP32-P4 is little-endian RISC-V
 */
#undef WORDS_BIGENDIAN

/*
 * Data type sizes for ESP32-P4
 */
#define SIZEOF_SHORT 2
#define SIZEOF_INT 4
#define SIZEOF_LONG 4
#define SIZEOF_LONG_LONG 8
#define SIZEOF_VOID_P 4
#define SIZEOF_FLOAT 4
#define SIZEOF_DOUBLE 8

/*
 * Basic data types
 */
typedef uint8_t uint8;
typedef int8_t int8;
typedef uint16_t uint16;
typedef int16_t int16;
typedef uint32_t uint32;
typedef int32_t int32;
typedef uint64_t uint64;
typedef int64_t int64;
typedef uint32_t uintptr;
typedef int32_t intptr;

// File offset type
typedef int32_t loff_t;

// Character address type
typedef char* caddr_t;

// Time data type for timer emulation
typedef uint64_t tm_time_t;

/*
 * UAE CPU data types
 */
typedef int8 uae_s8;
typedef uint8 uae_u8;
typedef int16 uae_s16;
typedef uint16 uae_u16;
typedef int32 uae_s32;
typedef uint32 uae_u32;
typedef int64 uae_s64;
typedef uint64 uae_u64;
typedef uae_u32 uaecptr;

/*
 * ESP32-P4 RISC-V does NOT support unaligned memory access safely
 */
#undef CPU_CAN_ACCESS_UNALIGNED

/*
 * 64-bit value macros
 */
#define VAL64(a) (a ## LL)
#define UVAL64(a) (a ## ULL)

/*
 * Memory pointer type for Mac addresses
 */
#define memptr uint32

/*
 * Float format
 */
#define IEEE_FLOAT_FORMAT 1
#define HOST_FLOAT_FORMAT IEEE_FLOAT_FORMAT

/*
 * Inline hints - must be defined before use
 */
#define __inline__ inline
#define ALWAYS_INLINE inline __attribute__((always_inline))

/*
 * Byte swapping functions for little-endian ESP32 accessing big-endian Mac data
 * Using GCC built-in byte swap for optimal performance
 */

// Byte swap functions using GCC builtins (compile to single instructions)
static ALWAYS_INLINE uae_u32 do_byteswap_32(uae_u32 v) {
    return __builtin_bswap32(v);
}

static ALWAYS_INLINE uae_u16 do_byteswap_16(uae_u16 v) {
    return __builtin_bswap16(v);
}

// Get 32-bit big-endian value from memory (optimized with builtin swap)
static ALWAYS_INLINE uae_u32 do_get_mem_long(uae_u32 *a) {
    return __builtin_bswap32(*a);
}

// Get 16-bit big-endian value from memory (optimized with builtin swap)
static ALWAYS_INLINE uae_u32 do_get_mem_word(uae_u16 *a) {
    return __builtin_bswap16(*a);
}

/*
 * Fast opcode fetch path:
 * On little-endian hosts, opcode words in emulated memory are stored byte-swapped.
 * Expose the raw word so the CPU core can skip per-instruction bswap and instead
 * use swapped opcode tables/bit extraction paths.
 */
#define HAVE_GET_WORD_UNSWAPPED 1
static ALWAYS_INLINE uae_u32 do_get_mem_word_unswapped(const uae_u8 *a) {
    return *(const uae_u16 *)a;
}

// Get 8-bit value from memory
#define do_get_mem_byte(a) ((uae_u32)*((uae_u8 *)(a)))

// Put 32-bit big-endian value to memory (optimized with builtin swap)
static ALWAYS_INLINE void do_put_mem_long(uae_u32 *a, uae_u32 v) {
    *a = __builtin_bswap32(v);
}

// Put 16-bit big-endian value to memory (optimized with builtin swap)
static ALWAYS_INLINE void do_put_mem_word(uae_u16 *a, uae_u32 v) {
    *a = __builtin_bswap16((uae_u16)v);
}

// Put 8-bit value to memory
#define do_put_mem_byte(a, v) (*(uae_u8 *)(a) = (v))

/*
 * Memory bank access function call macros
 */
#define call_mem_get_func(func, addr) ((*func)(addr))
#define call_mem_put_func(func, addr, v) ((*func)(addr, v))

/*
 * CPU emulation size (0 = normal)
 */
#define CPU_EMU_SIZE 0
#undef NO_INLINE_MEMORY_ACCESS

/*
 * Enum declaration macros
 */
#define ENUMDECL typedef enum
#define ENUMNAME(name) name

/*
 * Logging function
 */
#define write_log Serial.printf

/*
 * Register parameter hints (not used on ESP32)
 */
#define REGPARAM
#define REGPARAM2

/*
 * Unused parameter macro
 */
#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

/*
 * Branch prediction hints
 * Note: ESP32 may already define these, so only define if not present
 */
#ifndef likely
#define likely(x)   __builtin_expect(!!(x), 1)
#endif
#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x), 0)
#endif

/*
 * Spinlock implementation (single-threaded, no-op)
 * Note: ESP32 already defines spinlock_t, so we use our own type
 */
typedef volatile int b2_spinlock_t;
#define spinlock_t b2_spinlock_t
#define SPIN_LOCK_UNLOCKED 0

static inline void spin_lock(b2_spinlock_t *lock) {
    UNUSED(lock);
}

static inline void spin_unlock(b2_spinlock_t *lock) {
    UNUSED(lock);
}

static inline int spin_trylock(b2_spinlock_t *lock) {
    UNUSED(lock);
    return 1;
}

/*
 * Mutex implementation using FreeRTOS semaphores for thread safety
 */
struct B2_mutex {
    SemaphoreHandle_t sem;
};

/*
 * Timing functions (implemented in timer_esp32.cpp)
 */
extern uint64 GetTicks_usec(void);
extern void Delay_usec(uint64 usec);

/*
 * Disable features not needed on ESP32
 */
#undef ENABLE_MON
#undef USE_JIT
#undef ENABLE_GTK
#undef ENABLE_XF86_DGA
#undef USE_SDL
#undef USE_SDL_VIDEO
#undef USE_SDL_AUDIO

/*
 * FPU configuration
 */
#define FPU_IEEE 1
#define FPU_X86 0
#define FPU_UAE 0

/*
 * Assembly symbol naming (not used)
 */
#define ASM_SYM(a)

/*
 * POSIX-like file I/O macros
 */
#ifndef O_RDONLY
#define O_RDONLY 0
#endif
#ifndef O_RDWR
#define O_RDWR 2
#endif

/*
 * Debug configuration
 */
#ifndef DEBUG
#define DEBUG 0
#endif

/*
 * PSRAM allocation helper - use Arduino's ps_malloc
 */
#define psram_malloc(size) ps_malloc(size)
#define psram_calloc(n, size) ps_calloc(n, size)

#endif /* SYSDEPS_H */
