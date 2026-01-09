/*
 *  hw_esp32.cpp - Hardware register emulation for Mac SE/Plus
 *
 *  This file emulates the Mac's hardware registers:
 *  - VIA (Versatile Interface Adapter) - 0xEFE1xx range
 *  - SCC (Serial Communications Controller) - 0x9FFFFx range
 *  - IWM (Integrated Woz Machine) - 0xDFE1xx range (floppy)
 *  - SCSI - not used on Mac Plus/SE
 *
 *  ESP32 port (C) 2024
 */

#include "sysdeps.h"
#include "cpu_emulation.h"
#include "main.h"

// VIA registers (at 0xEFE1xx)
// VIA is a 6522 chip that handles:
// - Keyboard/mouse via ADB (later Macs) or direct (Plus/SE)
// - Real-time clock
// - Sound enable
// - Floppy control signals
// - Various timing

// VIA register offsets (active on odd addresses, step of 0x200)
#define VIA_DATAB    0x00   // Port B data
#define VIA_DATAA    0x200  // Port A data (active on 0xEFE3FE)
#define VIA_DDRB     0x400  // Data Direction Register B
#define VIA_DDRA     0x600  // Data Direction Register A
#define VIA_T1CL     0x800  // Timer 1 Counter Low
#define VIA_T1CH     0xA00  // Timer 1 Counter High
#define VIA_T1LL     0xC00  // Timer 1 Latch Low
#define VIA_T1LH     0xE00  // Timer 1 Latch High
#define VIA_T2CL     0x1000 // Timer 2 Counter Low
#define VIA_T2CH     0x1200 // Timer 2 Counter High
#define VIA_SR       0x1400 // Shift Register
#define VIA_ACR      0x1600 // Auxiliary Control Register
#define VIA_PCR      0x1800 // Peripheral Control Register
#define VIA_IFR      0x1A00 // Interrupt Flag Register
#define VIA_IER      0x1C00 // Interrupt Enable Register
#define VIA_DATAA2   0x1E00 // Port A data (no handshake)

// VIA state
static uint8 via_datab = 0xFF;   // Port B - keyboard, RTC, etc.
static uint8 via_dataa = 0xFF;   // Port A - sound, overlay, etc.
static uint8 via_ddrb = 0x00;    // Data Direction B
static uint8 via_ddra = 0x00;    // Data Direction A
static uint8 via_t1cl = 0x00;
static uint8 via_t1ch = 0x00;
static uint8 via_t1ll = 0x00;
static uint8 via_t1lh = 0x00;
static uint8 via_t2cl = 0x00;
static uint8 via_t2ch = 0x00;
static uint8 via_sr = 0x00;      // Shift register
static uint8 via_acr = 0x00;     // Aux control
static uint8 via_pcr = 0x00;     // Peripheral control
static uint8 via_ifr = 0x00;     // Interrupt flags
static uint8 via_ier = 0x00;     // Interrupt enable

// SCC state (Zilog 8530 Serial Communications Controller)
static uint8 scc_rr0 = 0x04;     // Read Register 0 - status (Tx buffer empty)
static uint8 scc_rr1 = 0x00;     // Read Register 1
static uint8 scc_rr2 = 0x00;     // Read Register 2 - interrupt vector

// IWM state (Floppy controller)
static uint8 iwm_status = 0x00;

// Hardware access logging
static int hw_access_count = 0;

// Check if address is in hardware range (24-bit space, above 0x400000)
static inline bool is_hardware_addr(uaecptr addr)
{
    // With 24-bit addressing, mask the address
    extern bool TwentyFourBitAddressing;
    if (TwentyFourBitAddressing) {
        addr &= 0x00FFFFFF;
    }
    
    // Hardware is in the high address space (above ROM at 0x400000)
    return (addr >= 0x500000);
}

// Read VIA register
static uint8 via_read(uint32 offset)
{
    switch (offset & 0x1E00) {
        case VIA_DATAB:
            // Port B: bit 7 = sound on/off (external), bit 6 = H4 sel (RTC)
            // bit 5 = head sel, bit 4 = overlay, bits 3-0 = RTC data
            return via_datab | (~via_ddrb);  // Inputs read as 1
            
        case VIA_DATAA:
        case VIA_DATAA2:
            // Port A: bits 7-0 = sound volume / other
            return via_dataa | (~via_ddra);
            
        case VIA_DDRB:
            return via_ddrb;
            
        case VIA_DDRA:
            return via_ddra;
            
        case VIA_T1CL:
            via_ifr &= ~0x40;  // Clear T1 interrupt flag
            return via_t1cl;
            
        case VIA_T1CH:
            return via_t1ch;
            
        case VIA_T1LL:
            return via_t1ll;
            
        case VIA_T1LH:
            return via_t1lh;
            
        case VIA_T2CL:
            via_ifr &= ~0x20;  // Clear T2 interrupt flag
            return via_t2cl;
            
        case VIA_T2CH:
            return via_t2ch;
            
        case VIA_SR:
            via_ifr &= ~0x04;  // Clear shift register interrupt
            return via_sr;
            
        case VIA_ACR:
            return via_acr;
            
        case VIA_PCR:
            return via_pcr;
            
        case VIA_IFR:
            // Return interrupt flags with bit 7 = any interrupt pending & enabled
            return via_ifr | ((via_ifr & via_ier) ? 0x80 : 0);
            
        case VIA_IER:
            return via_ier | 0x80;  // Bit 7 always reads as 1
            
        default:
            return 0xFF;
    }
}

// Write VIA register
static void via_write(uint32 offset, uint8 value)
{
    switch (offset & 0x1E00) {
        case VIA_DATAB:
            via_datab = (via_datab & ~via_ddrb) | (value & via_ddrb);
            break;
            
        case VIA_DATAA:
        case VIA_DATAA2:
            via_dataa = (via_dataa & ~via_ddra) | (value & via_ddra);
            break;
            
        case VIA_DDRB:
            via_ddrb = value;
            break;
            
        case VIA_DDRA:
            via_ddra = value;
            break;
            
        case VIA_T1CL:
        case VIA_T1LL:
            via_t1ll = value;
            break;
            
        case VIA_T1CH:
            via_t1lh = value;
            via_t1ch = value;
            via_t1cl = via_t1ll;
            via_ifr &= ~0x40;  // Clear T1 interrupt
            break;
            
        case VIA_T1LH:
            via_t1lh = value;
            break;
            
        case VIA_T2CL:
            via_t2cl = value;
            break;
            
        case VIA_T2CH:
            via_t2ch = value;
            via_ifr &= ~0x20;  // Clear T2 interrupt
            break;
            
        case VIA_SR:
            via_sr = value;
            via_ifr &= ~0x04;
            break;
            
        case VIA_ACR:
            via_acr = value;
            break;
            
        case VIA_PCR:
            via_pcr = value;
            break;
            
        case VIA_IFR:
            // Writing 1 to a bit clears it
            via_ifr &= ~(value & 0x7F);
            break;
            
        case VIA_IER:
            if (value & 0x80) {
                // Set bits
                via_ier |= (value & 0x7F);
            } else {
                // Clear bits
                via_ier &= ~(value & 0x7F);
            }
            break;
    }
}

// Frame buffer area (512x342 = 21888 bytes starting at 0x500000)
// This is where the Mac screen bitmap lives
static const uint32 FRAMEBUFFER_START = 0x500000;
static const uint32 FRAMEBUFFER_SIZE = 22 * 1024;  // ~21KB

// Hardware read functions
bool hw_read_byte(uaecptr addr, uae_u8 *value)
{
    extern bool TwentyFourBitAddressing;
    uaecptr masked_addr = addr;
    if (TwentyFourBitAddressing) {
        masked_addr = addr & 0x00FFFFFF;
    }
    
    // Frame buffer access (0x500000-0x506000) - let it go to regular memory
    // This is actually in our PSRAM, so return false to use normal memory access
    if (masked_addr >= FRAMEBUFFER_START && masked_addr < FRAMEBUFFER_START + FRAMEBUFFER_SIZE) {
        return false;  // Use normal memory access for frame buffer
    }
    
    if (masked_addr < 0x500000) {
        return false;  // Not hardware, use normal memory
    }
    
    hw_access_count++;
    
    // VIA at 0xEFE1xx and 0xEFE000-0xEFFFFF range
    if ((masked_addr & 0xFF0000) == 0xEF0000) {
        if ((masked_addr & 0x00F000) == 0x00E000) {
            // VIA registers at 0xEFE1xx
            *value = via_read(masked_addr & 0x1FFF);
            if (hw_access_count <= 30) {
                Serial.printf("[HW] VIA read 0x%06X -> 0x%02X\n", masked_addr, *value);
            }
            return true;
        }
        // Other 0xEFxxxx addresses - VIA-related, return safe values
        *value = 0xFF;
        return true;
    }
    
    // SCC at 0x9FFFFx (read) and 0xBFFFFx (write)
    if ((masked_addr & 0xF00000) == 0x900000 || (masked_addr & 0xF00000) == 0xB00000) {
        // SCC read - return status (Tx buffer empty, no data)
        *value = scc_rr0;
        if (hw_access_count <= 30) {
            Serial.printf("[HW] SCC read 0x%06X -> 0x%02X\n", masked_addr, *value);
        }
        return true;
    }
    
    // IWM (floppy) at 0xDFE1xx and 0xDFF000 range
    if ((masked_addr & 0xFF0000) == 0xDF0000) {
        *value = 0x1F;  // IWM status: motor off, no disk
        return true;
    }
    
    // ROM overlay / boot area at 0xF80000
    if ((masked_addr & 0xFF0000) == 0xF80000) {
        // ROM overlay - return ROM data from actual ROM location
        extern uint8 *ROMBaseHost;
        extern uint32 ROMSize;
        uint32 rom_offset = masked_addr & 0x0FFFFF;  // Offset into ROM
        if (rom_offset < ROMSize) {
            *value = ROMBaseHost[rom_offset];
        } else {
            *value = 0xFF;
        }
        return true;
    }
    
    // SCSI at 0x580xxx
    if ((masked_addr & 0xFF0000) == 0x580000) {
        *value = 0x00;  // No SCSI device
        return true;
    }
    
    // Sound buffer area 0x5F0000-0x5FFFFF
    if ((masked_addr & 0xFF0000) == 0x5F0000) {
        *value = 0x80;  // Silence (middle value for audio)
        return true;
    }
    
    // Unknown hardware - return 0xFF (open bus)
    if (hw_access_count <= 50) {
        Serial.printf("[HW] Unknown read 0x%06X\n", masked_addr);
    }
    *value = 0xFF;
    return true;
}

bool hw_read_word(uaecptr addr, uae_u16 *value)
{
    uae_u8 hi, lo;
    if (hw_read_byte(addr, &hi) && hw_read_byte(addr + 1, &lo)) {
        *value = (hi << 8) | lo;
        return true;
    }
    return false;
}

bool hw_read_long(uaecptr addr, uae_u32 *value)
{
    uae_u16 hi, lo;
    if (hw_read_word(addr, &hi) && hw_read_word(addr + 2, &lo)) {
        *value = (hi << 16) | lo;
        return true;
    }
    return false;
}

bool hw_write_byte(uaecptr addr, uae_u8 value)
{
    extern bool TwentyFourBitAddressing;
    uaecptr masked_addr = addr;
    if (TwentyFourBitAddressing) {
        masked_addr = addr & 0x00FFFFFF;
    }
    
    // Frame buffer access - let it go to regular memory
    if (masked_addr >= FRAMEBUFFER_START && masked_addr < FRAMEBUFFER_START + FRAMEBUFFER_SIZE) {
        return false;  // Use normal memory access for frame buffer
    }
    
    if (masked_addr < 0x500000) {
        return false;  // Not hardware
    }
    
    hw_access_count++;
    
    // VIA at 0xEFE1xx and 0xEFE000-0xEFFFFF range
    if ((masked_addr & 0xFF0000) == 0xEF0000) {
        if ((masked_addr & 0x00F000) == 0x00E000) {
            // VIA registers at 0xEFE1xx
            if (hw_access_count <= 30) {
                Serial.printf("[HW] VIA write 0x%06X <- 0x%02X\n", masked_addr, value);
            }
            via_write(masked_addr & 0x1FFF, value);
        }
        // Other 0xEFxxxx writes - ignore
        return true;
    }
    
    // SCC at 0x9FFFFx (read) and 0xBFFFFx (write)
    if ((masked_addr & 0xF00000) == 0x900000 || (masked_addr & 0xF00000) == 0xB00000) {
        // SCC write - ignore for now (would configure serial port)
        return true;
    }
    
    // IWM at 0xDFE1xx and 0xDFF000 range
    if ((masked_addr & 0xFF0000) == 0xDF0000) {
        // IWM write - ignore (would control floppy)
        return true;
    }
    
    // ROM overlay at 0xF80000 - ignore writes
    if ((masked_addr & 0xFF0000) == 0xF80000) {
        return true;
    }
    
    // SCSI at 0x580xxx
    if ((masked_addr & 0xFF0000) == 0x580000) {
        return true;  // Ignore
    }
    
    // Sound buffer 0x5F0000-0x5FFFFF
    if ((masked_addr & 0xFF0000) == 0x5F0000) {
        return true;  // Ignore sound writes
    }
    
    // Unknown hardware write
    if (hw_access_count <= 50) {
        Serial.printf("[HW] Unknown write 0x%06X <- 0x%02X\n", masked_addr, value);
    }
    return true;
}

bool hw_write_word(uaecptr addr, uae_u16 value)
{
    bool hi = hw_write_byte(addr, value >> 8);
    bool lo = hw_write_byte(addr + 1, value & 0xFF);
    return hi || lo;
}

bool hw_write_long(uaecptr addr, uae_u32 value)
{
    bool hi = hw_write_word(addr, value >> 16);
    bool lo = hw_write_word(addr + 2, value & 0xFFFF);
    return hi || lo;
}

// Initialize hardware emulation
void HWInit(void)
{
    Serial.println("[HW] Initializing hardware emulation...");
    
    // Initialize VIA to sensible defaults for Mac boot
    via_datab = 0xFF;
    via_dataa = 0xFF;
    via_ddrb = 0x00;
    via_ddra = 0x00;
    via_ifr = 0x00;
    via_ier = 0x00;
    
    // Initialize SCC
    scc_rr0 = 0x04;  // Tx buffer empty
    
    hw_access_count = 0;
    
    Serial.println("[HW] Hardware emulation ready");
}
