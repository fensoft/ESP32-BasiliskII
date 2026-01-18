/*
 * UAE - The Un*x Amiga Emulator
 *
 * memory management
 *
 * Copyright 1995 Bernd Schmidt
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef UAE_MEMORY_H
#define UAE_MEMORY_H

#if !DIRECT_ADDRESSING && !REAL_ADDRESSING

/* Enabling this adds one additional native memory reference per 68k memory
 * access, but saves one shift (on the x86). Enabling this is probably
 * better for the cache. My favourite benchmark (PP2) doesn't show a
 * difference, so I leave this enabled. */

#if 1 || defined SAVE_MEMORY
#define SAVE_MEMORY_BANKS
#endif

typedef uae_u32 (REGPARAM2 *mem_get_func)(uaecptr) REGPARAM;
typedef void (REGPARAM2 *mem_put_func)(uaecptr, uae_u32) REGPARAM;
typedef uae_u8 *(REGPARAM2 *xlate_func)(uaecptr) REGPARAM;

#undef DIRECT_MEMFUNCS_SUCCESSFUL

#ifndef CAN_MAP_MEMORY
#undef USE_COMPILER
#endif

#if defined(USE_COMPILER) && !defined(USE_MAPPED_MEMORY)
#define USE_MAPPED_MEMORY
#endif

typedef struct {
    /* These ones should be self-explanatory... */
    mem_get_func lget, wget, bget;
    mem_put_func lput, wput, bput;
    /* Use xlateaddr to translate an Amiga address to a uae_u8 * that can
     * be used to address memory without calling the wget/wput functions.
     * This doesn't work for all memory banks, so this function may call
     * abort(). */
    xlate_func xlateaddr;
} addrbank;

extern uae_u8 filesysory[65536];

extern addrbank ram_bank;	// Mac RAM
extern addrbank rom_bank;	// Mac ROM
extern addrbank frame_bank;	// Frame buffer

/* Default memory access functions */

extern uae_u8 *REGPARAM2 default_xlate(uaecptr addr) REGPARAM;

#define bankindex(addr) (((uaecptr)(addr)) >> 16)

#ifdef SAVE_MEMORY_BANKS
// Note: mem_banks is dynamically allocated in PSRAM on ESP32
extern addrbank **mem_banks;
#define get_mem_bank(addr) (*mem_banks[bankindex(addr)])
#define put_mem_bank(addr, b) (mem_banks[bankindex(addr)] = (b))
#else
extern addrbank mem_banks[65536];
#define get_mem_bank(addr) (mem_banks[bankindex(addr)])
#define put_mem_bank(addr, b) (mem_banks[bankindex(addr)] = *(b))
#endif

extern void memory_init(void);
extern void map_banks(addrbank *bank, int first, int count);

#ifndef NO_INLINE_MEMORY_ACCESS

/*
 * FAST-PATH MEMORY ACCESS OPTIMIZATION
 * 
 * Most memory accesses in the emulator are to RAM (code/data) or ROM.
 * By adding inline checks for these common cases, we can bypass the
 * expensive memory bank lookup (pointer indirection + function call)
 * for the majority of accesses.
 * 
 * Performance impact: Significant (2-3x for memory-intensive code)
 * 
 * Memory layout:
 * - RAM: 0x00000000 to RAMSize (typically 8MB)
 * - ROM: ROMBaseMac to ROMBaseMac + ROMSize (varies by ROM type)
 * - Frame buffer: MacFrameBaseMac (0xa0000000)
 */

// Branch prediction hints (may already be defined in sysdeps.h)
#ifndef likely
#define likely(x)   __builtin_expect(!!(x), 1)
#endif
#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x), 0)
#endif

// External declarations for fast-path checks
extern uint32 RAMBaseMac;
extern uint8 *RAMBaseHost;
extern uint32 RAMSize;
extern uint32 ROMBaseMac;
extern uint8 *ROMBaseHost;
extern uint32 ROMSize;

// Fast-path long (32-bit) read
static inline uae_u32 longget_fastpath(uaecptr addr) {
    // Fast path for RAM (most common case)
    // RAM is at address 0, so just check if addr < RAMSize
    if (likely(addr < RAMSize)) {
        uae_u32 *m = (uae_u32 *)(RAMBaseHost + addr);
        return do_get_mem_long(m);
    }
    // Fast path for ROM
    if (addr >= ROMBaseMac && addr < ROMBaseMac + ROMSize) {
        uae_u32 *m = (uae_u32 *)(ROMBaseHost + (addr - ROMBaseMac));
        return do_get_mem_long(m);
    }
    // Fall back to bank lookup for other addresses (frame buffer, hardware, etc.)
    return call_mem_get_func(get_mem_bank(addr).lget, addr);
}

// Fast-path word (16-bit) read
static inline uae_u32 wordget_fastpath(uaecptr addr) {
    if (likely(addr < RAMSize)) {
        uae_u16 *m = (uae_u16 *)(RAMBaseHost + addr);
        return do_get_mem_word(m);
    }
    if (addr >= ROMBaseMac && addr < ROMBaseMac + ROMSize) {
        uae_u16 *m = (uae_u16 *)(ROMBaseHost + (addr - ROMBaseMac));
        return do_get_mem_word(m);
    }
    return call_mem_get_func(get_mem_bank(addr).wget, addr);
}

// Fast-path byte (8-bit) read
static inline uae_u32 byteget_fastpath(uaecptr addr) {
    if (likely(addr < RAMSize)) {
        return *(uae_u8 *)(RAMBaseHost + addr);
    }
    if (addr >= ROMBaseMac && addr < ROMBaseMac + ROMSize) {
        return *(uae_u8 *)(ROMBaseHost + (addr - ROMBaseMac));
    }
    return call_mem_get_func(get_mem_bank(addr).bget, addr);
}

// Fast-path long (32-bit) write
static inline void longput_fastpath(uaecptr addr, uae_u32 l) {
    // Fast path for RAM writes (most common case)
    if (likely(addr < RAMSize)) {
        uae_u32 *m = (uae_u32 *)(RAMBaseHost + addr);
        do_put_mem_long(m, l);
        return;
    }
    // ROM writes go to bank handler (which will log/ignore them)
    // Frame buffer and hardware writes also go through bank handler
    call_mem_put_func(get_mem_bank(addr).lput, addr, l);
}

// Fast-path word (16-bit) write
static inline void wordput_fastpath(uaecptr addr, uae_u32 w) {
    if (likely(addr < RAMSize)) {
        uae_u16 *m = (uae_u16 *)(RAMBaseHost + addr);
        do_put_mem_word(m, w);
        return;
    }
    call_mem_put_func(get_mem_bank(addr).wput, addr, w);
}

// Fast-path byte (8-bit) write
static inline void byteput_fastpath(uaecptr addr, uae_u32 b) {
    if (likely(addr < RAMSize)) {
        *(uae_u8 *)(RAMBaseHost + addr) = b;
        return;
    }
    call_mem_put_func(get_mem_bank(addr).bput, addr, b);
}

// Use fast-path functions for all memory access
#define longget(addr) longget_fastpath(addr)
#define wordget(addr) wordget_fastpath(addr)
#define byteget(addr) byteget_fastpath(addr)
#define longput(addr,l) longput_fastpath(addr, l)
#define wordput(addr,w) wordput_fastpath(addr, w)
#define byteput(addr,b) byteput_fastpath(addr, b)

#else

extern uae_u32 longget(uaecptr addr);
extern uae_u32 wordget(uaecptr addr);
extern uae_u32 byteget(uaecptr addr);
extern void longput(uaecptr addr, uae_u32 l);
extern void wordput(uaecptr addr, uae_u32 w);
extern void byteput(uaecptr addr, uae_u32 b);

#endif

#ifndef MD_HAVE_MEM_1_FUNCS

#define longget_1 longget
#define wordget_1 wordget
#define byteget_1 byteget
#define longput_1 longput
#define wordput_1 wordput
#define byteput_1 byteput

#endif

#endif /* !DIRECT_ADDRESSING && !REAL_ADDRESSING */

#if REAL_ADDRESSING
const uintptr MEMBaseDiff = 0;
#elif DIRECT_ADDRESSING
extern uintptr MEMBaseDiff;
#endif

#if REAL_ADDRESSING || DIRECT_ADDRESSING
static __inline__ uae_u8 *do_get_real_address(uaecptr addr)
{
	return (uae_u8 *)MEMBaseDiff + addr;
}
static __inline__ uae_u32 do_get_virtual_address(uae_u8 *addr)
{
	return (uintptr)addr - MEMBaseDiff;
}
static __inline__ uae_u32 get_long(uaecptr addr)
{
    uae_u32 * const m = (uae_u32 *)do_get_real_address(addr);
    return do_get_mem_long(m);
}
static __inline__ uae_u32 get_word(uaecptr addr)
{
    uae_u16 * const m = (uae_u16 *)do_get_real_address(addr);
    return do_get_mem_word(m);
}
static __inline__ uae_u32 get_byte(uaecptr addr)
{
    uae_u8 * const m = (uae_u8 *)do_get_real_address(addr);
    return do_get_mem_byte(m);
}
static __inline__ void put_long(uaecptr addr, uae_u32 l)
{
    uae_u32 * const m = (uae_u32 *)do_get_real_address(addr);
    do_put_mem_long(m, l);
}
static __inline__ void put_word(uaecptr addr, uae_u32 w)
{
    uae_u16 * const m = (uae_u16 *)do_get_real_address(addr);
    do_put_mem_word(m, w);
}
static __inline__ void put_byte(uaecptr addr, uae_u32 b)
{
    uae_u8 * const m = (uae_u8 *)do_get_real_address(addr);
    do_put_mem_byte(m, b);
}
static __inline__ uae_u8 *get_real_address(uaecptr addr)
{
	return do_get_real_address(addr);
}
static __inline__ uae_u32 get_virtual_address(uae_u8 *addr)
{
	return do_get_virtual_address(addr);
}
#else
static __inline__ uae_u32 get_long(uaecptr addr)
{
    return longget_1(addr);
}
static __inline__ uae_u32 get_word(uaecptr addr)
{
    return wordget_1(addr);
}
static __inline__ uae_u32 get_byte(uaecptr addr)
{
    return byteget_1(addr);
}
static __inline__ void put_long(uaecptr addr, uae_u32 l)
{
    longput_1(addr, l);
}
static __inline__ void put_word(uaecptr addr, uae_u32 w)
{
    wordput_1(addr, w);
}
static __inline__ void put_byte(uaecptr addr, uae_u32 b)
{
    byteput_1(addr, b);
}
static __inline__ uae_u8 *get_real_address(uaecptr addr)
{
    return get_mem_bank(addr).xlateaddr(addr);
}
/* gb-- deliberately not implemented since it shall not be used... */
extern uae_u32 get_virtual_address(uae_u8 *addr);
#endif /* DIRECT_ADDRESSING || REAL_ADDRESSING */

#endif /* MEMORY_H */

