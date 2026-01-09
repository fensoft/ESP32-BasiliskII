/*
 *  basilisk_glue.cpp - Glue code for connecting UAE CPU to BasiliskII (ESP32 version)
 *
 *  Basilisk II (C) Christian Bauer
 *  ESP32 port (C) 2024
 *
 *  Simplified for Mac IIci (68030, 32-bit addressing)
 */

#include "sysdeps.h"
#include "cpu_emulation.h"
#include "main.h"
#include "emul_op.h"
#include "rom_patches.h"
#include "timer.h"
#include "m68k.h"
#include "memory.h"
#include "readcpu.h"
#include "newcpu.h"

#define DEBUG 0
#include "debug.h"

// M68K_EXEC_RETURN opcode (0x7100) - causes m68k_emulop_return() to be called
#define M68K_EXEC_RETURN 0x7100

// RAM and ROM pointers
uint32 RAMBaseMac;
uint8 *RAMBaseHost;
uint32 RAMSize;
uint32 ROMBaseMac;
uint8 *ROMBaseHost;
uint32 ROMSize;

// Direct addressing base difference
#if DIRECT_ADDRESSING
uintptr MEMBaseDiff;
#endif

// CPU interrupt flag
extern bool quit_program;

// For ROM debugging
extern uint32 ROMBreakpoint;
extern bool PrintROMInfo;

/*
 *  Initialize 680x0 emulation
 */
bool Init680x0(void)
{
    Serial.println("[CPU] Initializing 68030 emulation...");
    init_m68k();
    Serial.println("[CPU] 68030 emulation initialized");
    return true;
}

/*
 *  Deinitialize 680x0 emulation
 */
void Exit680x0(void)
{
    Serial.println("[CPU] Shutting down 68k emulation");
}

/*
 *  Initialize memory map for 68k emulation
 */
void InitFrameBufferMapping(void)
{
    // With DIRECT_ADDRESSING, nothing special needed
}

/*
 *  Handle illegal instruction - for CPU detection (MOVEC etc)
 */
bool HandleIllegalInstruction(uae_u32 opcode, uaecptr pc)
{
    // MOVEC instructions (0x4E7A, 0x4E7B) are used for CPU type detection
    if (opcode == 0x4E7A || opcode == 0x4E7B) {
        static int movec_count = 0;
        movec_count++;
        if (movec_count <= 3) {
            Serial.printf("[CPU] MOVEC at 0x%08X - skipping (CPU detection)\n", pc);
        }
        m68k_incpc(4);  // Skip opcode + extension word
        return true;
    }
    
    return false;  // Not handled
}

/*
 *  Reset and start 680x0 emulation
 */
void Start680x0(void)
{
    Serial.println("\n[CPU] Starting 68030 CPU...");
    
    // Debug: verify memory setup
    Serial.printf("[CPU] RAM: %d MB at Mac 0x%08X, Host %p\n", 
                  RAMSize / (1024*1024), RAMBaseMac, RAMBaseHost);
    Serial.printf("[CPU] ROM: %d KB at Mac 0x%08X, Host %p\n", 
                  ROMSize / 1024, ROMBaseMac, ROMBaseHost);
    
    // Verify ROM looks valid
    uint32_t rom_checksum = ReadMacInt32(ROMBaseMac);
    uint32_t rom_entry = ReadMacInt32(ROMBaseMac + 4);
    Serial.printf("[CPU] ROM checksum: 0x%08X\n", rom_checksum);
    Serial.printf("[CPU] ROM entry:    0x%08X\n", rom_entry);
    
    // Reset CPU
    m68k_reset();
    
    Serial.printf("[CPU] After reset: PC = 0x%08X, A7 = 0x%08X\n", 
                  m68k_getpc(), m68k_areg(regs, 7));
    
    // Debug: Check what's at key ROM offsets
    Serial.printf("[CPU] ROM at 0x2A (entry): %04X %04X %04X %04X\n",
                  ReadMacInt16(ROMBaseMac + 0x2A),
                  ReadMacInt16(ROMBaseMac + 0x2C),
                  ReadMacInt16(ROMBaseMac + 0x2E),
                  ReadMacInt16(ROMBaseMac + 0x30));
    Serial.printf("[CPU] ROM at 0x8C (RESET patch): %04X %04X %04X %04X\n",
                  ReadMacInt16(ROMBaseMac + 0x8C),
                  ReadMacInt16(ROMBaseMac + 0x8E),
                  ReadMacInt16(ROMBaseMac + 0x90),
                  ReadMacInt16(ROMBaseMac + 0x92));
    Serial.printf("[CPU] ROM at 0x1142 (INSTALL_DRIVERS patch): %04X %04X\n",
                  ReadMacInt16(ROMBaseMac + 0x1142),
                  ReadMacInt16(ROMBaseMac + 0x1144));
    Serial.printf("[CPU] Expected EMULOP_RESET=0x7103, INSTALL_DRIVERS=0x710A\n");
    
    // Debug: Show exception vectors (should be 0 at reset, ROM will set them up)
    Serial.printf("[CPU] Exception vectors at reset:\n");
    Serial.printf("[CPU]   Vec 10 (A-line): 0x%08X\n", ReadMacInt32(0x28));
    Serial.printf("[CPU]   Vec 11 (F-line): 0x%08X\n", ReadMacInt32(0x2C));
    Serial.printf("[CPU]   Vec  2 (Bus err): 0x%08X\n", ReadMacInt32(0x08));
    Serial.printf("[CPU]   Vec  4 (Illegal): 0x%08X\n", ReadMacInt32(0x10));
    
    // Show video-related low memory globals
    Serial.printf("[CPU] Video globals at reset:\n");
    Serial.printf("[CPU]   ScrnBase (0x824): 0x%08X\n", ReadMacInt32(0x824));
    Serial.printf("[CPU]   MainDevice (0x8A4): 0x%08X\n", ReadMacInt32(0x8A4));
    Serial.printf("[CPU]   DeviceList (0x8A8): 0x%08X\n", ReadMacInt32(0x8A8));
    
    Serial.println("[CPU] Entering emulation loop...\n");
    Serial.flush();
    
    // Enter main emulation loop
    m68k_execute();
    
    Serial.println("\n[CPU] 68030 CPU stopped");
}

/*
 *  Trigger interrupt
 */
void TriggerInterrupt(void)
{
    SPCFLAGS_SET(SPCFLAG_INT);
}

void TriggerNMI(void)
{
    SPCFLAGS_SET(SPCFLAG_INT);
}

/*
 *  Execute 68k subroutine
 *  The executed routine must reside in UAE memory!
 *  r->a[7] and r->sr are unused!
 */
void Execute68k(uint32 addr, M68kRegisters *r)
{
    int i;
    
    // Save old PC
    uaecptr oldpc = m68k_getpc();
    
    // Set registers
    for (i = 0; i < 8; i++)
        m68k_dreg(regs, i) = r->d[i];
    for (i = 0; i < 7; i++)
        m68k_areg(regs, i) = r->a[i];
    
    // Push EXEC_RETURN and faked return address (points to EXEC_RETURN) on stack
    m68k_areg(regs, 7) -= 2;
    put_word(m68k_areg(regs, 7), M68K_EXEC_RETURN);
    m68k_areg(regs, 7) -= 4;
    put_long(m68k_areg(regs, 7), m68k_areg(regs, 7) + 4);
    
    // Execute routine
    m68k_setpc(addr);
    fill_prefetch_0();
    quit_program = false;
    m68k_execute();
    
    // Clean up stack
    m68k_areg(regs, 7) += 2;
    
    // Restore old PC
    m68k_setpc(oldpc);
    fill_prefetch_0();
    
    // Get registers
    for (i = 0; i < 8; i++)
        r->d[i] = m68k_dreg(regs, i);
    for (i = 0; i < 7; i++)
        r->a[i] = m68k_areg(regs, i);
    quit_program = false;
}

/*
 *  Execute MacOS 68k trap
 *  r->a[7] and r->sr are unused!
 */
void Execute68kTrap(uint16 trap, M68kRegisters *r)
{
    int i;
    
    // Save old PC
    uaecptr oldpc = m68k_getpc();
    
    // Set registers
    for (i = 0; i < 8; i++)
        m68k_dreg(regs, i) = r->d[i];
    for (i = 0; i < 7; i++)
        m68k_areg(regs, i) = r->a[i];
    
    // Push trap and EXEC_RETURN on stack
    m68k_areg(regs, 7) -= 2;
    put_word(m68k_areg(regs, 7), M68K_EXEC_RETURN);
    m68k_areg(regs, 7) -= 2;
    put_word(m68k_areg(regs, 7), trap);
    
    // Execute trap (PC points to stack where trap opcode is)
    m68k_setpc(m68k_areg(regs, 7));
    fill_prefetch_0();
    quit_program = false;
    m68k_execute();
    
    // Clean up stack
    m68k_areg(regs, 7) += 4;
    
    // Restore old PC
    m68k_setpc(oldpc);
    fill_prefetch_0();
    
    // Get registers
    for (i = 0; i < 8; i++)
        r->d[i] = m68k_dreg(regs, i);
    for (i = 0; i < 7; i++)
        r->a[i] = m68k_areg(regs, i);
    quit_program = false;
}
