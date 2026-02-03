/*
 *  main_esp32.cpp - Startup code for ESP32-P4
 *
 *  Basilisk II (C) Christian Bauer
 *  ESP32 port (C) 2024
 *
 *  Configuration: Mac Plus emulation (68000, 24-bit addressing, B&W)
 *  (Mac IIci code commented out for future use)
 */

#include "sysdeps.h"

#include <M5Unified.h>
#include <SD_MMC.h>

#include "cpu_emulation.h"
#include "sys.h"
#include "rom_patches.h"
#include "xpram.h"
#include "timer.h"
#include "video.h"
#include "emul_op.h"
#include "prefs.h"
#include "macos_util.h"
#include "user_strings.h"
#include "main.h"

#define DEBUG 0
#include "debug.h"

// Configuration for Mac IIci emulation (68030, 32-bit, color)
static const char *ROM_FILE = "68030.ROM";        // Mac IIci ROM (512KB)
static const char *FLOPPY_FILE = "DiskTools.img"; // Boot floppy
static const char *DISK_FILE = "Macintosh.dsk";   // Hard disk image
static const uint32_t MAC_RAM_SIZE = 8 * 1024 * 1024;  // 8MB RAM

// Other ROM options:
// static const char *ROM_FILE = "68000.rom";        // Mac Plus ROM (128KB) - NOT FULLY SUPPORTED
// static const char *ROM_FILE = "68000SE.ROM";      // Mac SE ROM (256KB) - patch issues

// File paths
static char rom_path[64];
static char floppy_path[64];
static char disk_path[64];

// CPU and FPU type, addressing mode
int CPUType;
bool CPUIs68060;
int FPUType;
bool TwentyFourBitAddressing;

// RAM and ROM pointers - defined in basilisk_glue.cpp
extern uint32 RAMBaseMac;
extern uint8 *RAMBaseHost;
extern uint32 RAMSize;
extern uint32 ROMBaseMac;
extern uint8 *ROMBaseHost;
extern uint32 ROMSize;
extern uintptr MEMBaseDiff;

// Scratch memory for Mac ROM writes (hardware base address redirection)
// This is critical for the boot sequence to work correctly
uint8 *ScratchMem = NULL;

// Interrupt flags
uint32 InterruptFlags = 0;

// Forward declarations
static bool LoadROM(void);
static bool InitMemory(void);

/*
 *  Ersatz functions
 */
char *strdup(const char *s)
{
    char *n = (char *)malloc(strlen(s) + 1);
    if (n) {
        strcpy(n, s);
    }
    return n;
}

/*
 *  Set/Clear interrupt flags (directly, no locking on ESP32)
 */
void SetInterruptFlag(uint32 flag)
{
    InterruptFlags |= flag;
}

void ClearInterruptFlag(uint32 flag)
{
    InterruptFlags &= ~flag;
}

// TriggerInterrupt and TriggerNMI are defined in basilisk_glue.cpp

/*
 *  Initialize memory for Mac IIci emulation
 *  
 *  Mac IIci: 32-bit addressing, 512KB ROM, up to 8MB RAM
 *  ROM is at high address 0x40800000
 */
static bool InitMemory(void)
{
    Serial.println("[Basilisk] Allocating memory from PSRAM (Mac IIci mode)...");
    
    // For Mac IIci (32-bit), RAM starts at 0
    RAMSize = MAC_RAM_SIZE;  // 8MB
    RAMBaseMac = 0;
    
    // Allocate RAM from PSRAM
    Serial.printf("[Basilisk] Allocating %d MB for RAM...\n", RAMSize / (1024 * 1024));
    RAMBaseHost = (uint8 *)ps_malloc(RAMSize);
    if (RAMBaseHost == NULL) {
        Serial.println("[Basilisk] ERROR: Failed to allocate RAM!");
        return false;
    }
    
    // Clear RAM
    memset(RAMBaseHost, 0, RAMSize);
    Serial.printf("[Basilisk] RAM allocated at %p\n", RAMBaseHost);
    
    // Allocate ROM space (512KB for Mac IIci, allocate 1MB for safety)
    ROMBaseHost = (uint8 *)ps_malloc(1024 * 1024);  // 1MB allocation
    if (ROMBaseHost == NULL) {
        Serial.println("[Basilisk] ERROR: Failed to allocate ROM space!");
        free(RAMBaseHost);
        return false;
    }
    memset(ROMBaseHost, 0, 1024 * 1024);
    Serial.printf("[Basilisk] ROM space allocated at %p\n", ROMBaseHost);
    
    // For Mac IIci (32-bit addressing), ROM is at high address
    ROMBaseMac = 0x40800000;  // Standard Mac IIci ROM location
    
    // MEMBaseDiff for DIRECT_ADDRESSING
    // For 32-bit Macs, this is more complex - RAM and ROM are separate
    // We'll set MEMBaseDiff to RAMBaseHost and handle ROM separately
    MEMBaseDiff = (uintptr)RAMBaseHost;
    
    Serial.printf("[Basilisk] RAM: Mac 0x%08X, Host %p, Size %d MB\n", 
                  RAMBaseMac, RAMBaseHost, RAMSize / (1024 * 1024));
    Serial.printf("[Basilisk] ROM: Mac 0x%08X, Host %p\n", 
                  ROMBaseMac, ROMBaseHost);
    Serial.printf("[Basilisk] MEMBaseDiff: 0x%08lX\n", (unsigned long)MEMBaseDiff);
    
    // Allocate scratch memory for hardware base address redirection
    // The ROM patches redirect hardware accesses to this area so they don't crash
    uint8 *scratch_raw = (uint8 *)ps_malloc(SCRATCH_MEM_SIZE);
    if (scratch_raw == NULL) {
        Serial.println("[Basilisk] ERROR: Failed to allocate scratch memory!");
        free(RAMBaseHost);
        free(ROMBaseHost);
        return false;
    }
    memset(scratch_raw, 0, SCRATCH_MEM_SIZE);
    // ScratchMem points to the middle of the block (so we have room on both sides)
    ScratchMem = scratch_raw + SCRATCH_MEM_SIZE / 2;
    Serial.printf("[Basilisk] ScratchMem allocated at %p (base %p)\n", ScratchMem, scratch_raw);
    
    return true;
}

/*
 *  Load ROM from SD card
 */
static bool LoadROM(void)
{
    Serial.println("[Basilisk] Loading ROM...");
    
    snprintf(rom_path, sizeof(rom_path), "/%s", ROM_FILE);
    File romFile = SD_MMC.open(rom_path, FILE_READ);
    
    if (!romFile) {
        Serial.printf("[Basilisk] ERROR: ROM file not found: %s\n", rom_path);
        return false;
    }
    
    ROMSize = romFile.size();
    Serial.printf("[Basilisk] ROM file: %s (%d KB)\n", ROM_FILE, ROMSize / 1024);
    
    // Mac Plus ROM should be 128KB (other ROMs: 256KB, 512KB, 1MB)
    if (ROMSize == 128 * 1024) {
        Serial.println("[Basilisk] Detected: Mac Plus/512K ROM (128KB)");
    } else if (ROMSize == 256 * 1024) {
        Serial.println("[Basilisk] Detected: Mac SE/Classic ROM (256KB)");
    } else if (ROMSize == 512 * 1024) {
        Serial.println("[Basilisk] Detected: Mac IIci/IIsi ROM (512KB)");
    } else {
        Serial.printf("[Basilisk] WARNING: Unusual ROM size: %d KB\n", ROMSize / 1024);
        // Continue anyway, might still work
    }
    
    // Print ROM header for verification
    uint8_t header[16];
    romFile.read(header, 16);
    romFile.seek(0);
    Serial.printf("[Basilisk] ROM header: %02X %02X %02X %02X  %02X %02X %02X %02X\n",
                  header[0], header[1], header[2], header[3],
                  header[4], header[5], header[6], header[7]);
    Serial.printf("[Basilisk] ROM checksum: 0x%02X%02X%02X%02X\n",
                  header[0], header[1], header[2], header[3]);
    
    // Read ROM into memory
    size_t bytesRead = romFile.read(ROMBaseHost, ROMSize);
    romFile.close();
    
    if (bytesRead != ROMSize) {
        Serial.printf("[Basilisk] ERROR: Read only %d of %d bytes!\n", bytesRead, ROMSize);
        return false;
    }
    
    Serial.println("[Basilisk] ROM loaded successfully");
    return true;
}

/*
 *  Check if a file exists on SD card
 */
static bool FileExists(const char *filename)
{
    char path[128];
    snprintf(path, sizeof(path), "/%s", filename);
    File f = SD_MMC.open(path, FILE_READ);
    if (f) {
        Serial.printf("[SD] Found: %s (%d bytes)\n", filename, f.size());
        f.close();
        return true;
    }
    return false;
}

/*
 *  List files in root directory
 */
static void ListDirectory(void)
{
    Serial.println("[SD] Root directory:");
    
    File root = SD_MMC.open("/");
    if (!root || !root.isDirectory()) {
        Serial.println("[SD] ERROR: Cannot open root directory");
        return;
    }
    
    File file = root.openNextFile();
    int count = 0;
    while (file && count < 20) {
        if (file.isDirectory()) {
            Serial.printf("[SD]   [DIR] %s/\n", file.name());
        } else {
            Serial.printf("[SD]   %s (%d bytes)\n", file.name(), file.size());
        }
        file = root.openNextFile();
        count++;
    }
    root.close();
}

/*
 *  Initialize SD card
 */
static bool InitSDCard(void)
{
    Serial.println("\n[Basilisk] ===== SD Card =====");
    
    if (!SD_MMC.begin("/sd", true, false, SDMMC_FREQ_DEFAULT, 4)) {
        Serial.println("[SD] ERROR: Mount failed!");
        return false;
    }
    
    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("[SD] Card: %llu MB\n", cardSize);
    
    ListDirectory();
    
    // Check for required files
    Serial.println("\n[Basilisk] ===== Required Files =====");
    
    bool rom_ok = FileExists(ROM_FILE);
    bool floppy_ok = FileExists(FLOPPY_FILE);
    bool disk_ok = FileExists(DISK_FILE);
    
    Serial.printf("[Check] ROM (%s): %s\n", ROM_FILE, rom_ok ? "OK" : "MISSING");
    Serial.printf("[Check] Floppy (%s): %s\n", FLOPPY_FILE, floppy_ok ? "OK" : "MISSING");  
    Serial.printf("[Check] Disk (%s): %s\n", DISK_FILE, disk_ok ? "OK" : "MISSING");
    
    if (!rom_ok) {
        Serial.println("\n[Basilisk] ERROR: ROM file missing!");
        Serial.printf("[Basilisk] Please copy %s to the SD card\n", ROM_FILE);
        Serial.println("[Basilisk] This should be a Mac IIci ROM (512KB, checksum 368CADFE)");
        return false;
    }
    
    if (!floppy_ok && !disk_ok) {
        Serial.println("\n[Basilisk] WARNING: No boot disk found!");
    }
    
    return true;
}

/*
 *  Main initialization - called from Arduino setup()
 */
bool BasiliskInit(void)
{
    Serial.println("\n========================================");
    Serial.println("   Basilisk II for ESP32-P4");
    Serial.println("   Mac IIci Emulator (68030)");
    Serial.println("========================================\n");
    
    Serial.printf("[Basilisk] Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("[Basilisk] Free PSRAM: %d bytes\n", ESP.getFreePsram());
    
    // Initialize SD card
    if (!InitSDCard()) {
        return false;
    }
    
    // Initialize memory
    if (!InitMemory()) {
        return false;
    }
    
    // Load ROM
    if (!LoadROM()) {
        return false;
    }
    
    // Initialize preferences
    Serial.println("\n[Basilisk] ===== Configuration =====");
    int dummy_argc = 0;
    char *dummy_argv[] = {NULL};
    char **dummy_argv_ptr = dummy_argv;
    PrefsInit(NULL, dummy_argc, dummy_argv_ptr);
    
    // Set CPU type for Mac Plus (68000, no FPU, 24-bit addressing)
    CPUType = 0;  // 68000
    CPUIs68060 = false;
    FPUType = 0;  // No FPU (Mac Plus didn't have one)
    TwentyFourBitAddressing = true;  // 24-bit addressing for Mac Plus
    
    Serial.println("[Config] CPU: 68000");
    Serial.println("[Config] FPU: None");
    Serial.printf("[Config] RAM: %d MB\n", MAC_RAM_SIZE / (1024 * 1024));
    Serial.println("[Config] Addressing: 24-bit");
    Serial.println("[Config] Display: 512x342 1-bit B&W");
    
    // Mac IIci configuration (commented out for future use):
    // CPUType = 3;  // 68030
    // FPUType = 1;  // 68881/68882
    // TwentyFourBitAddressing = false;  // 32-bit addressing
    
    // Initialize everything
    Serial.println("\n[Basilisk] ===== Initializing =====");
    Serial.flush();
    
    if (!InitAll(NULL)) {
        Serial.println("[Basilisk] ERROR: InitAll failed!");
        return false;
    }
    
    Serial.println("\n[Basilisk] ===== Ready =====");
    Serial.printf("[Basilisk] Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("[Basilisk] Free PSRAM: %d bytes\n", ESP.getFreePsram());
    
    return true;
}

/*
 *  Run the emulator - called from Arduino loop()
 */
void BasiliskRun(void)
{
    // Start 68k emulation
    Start680x0();
}

/*
 *  Quit emulator
 */
void QuitEmulator(void)
{
    Serial.println("[Basilisk] Shutting down...");
    ExitAll();
    
    if (RAMBaseHost) {
        free(RAMBaseHost);
        RAMBaseHost = NULL;
    }
    if (ROMBaseHost) {
        free(ROMBaseHost);
        ROMBaseHost = NULL;
    }
    
    Serial.println("[Basilisk] Shutdown complete");
    while (1) { delay(1000); }
}

/*
 *  Code was patched, flush caches
 */
void FlushCodeCache(void *start, uint32 size)
{
    // No JIT, nothing to flush
}

/*
 *  Mutex functions using FreeRTOS primitives for thread safety
 */
B2_mutex *B2_create_mutex(void)
{
    B2_mutex *m = new B2_mutex;
    if (m) {
        m->sem = xSemaphoreCreateMutex();
        if (!m->sem) {
            delete m;
            return NULL;
        }
    }
    return m;
}

void B2_lock_mutex(B2_mutex *mutex)
{
    if (mutex && mutex->sem) {
        xSemaphoreTake(mutex->sem, portMAX_DELAY);
    }
}

void B2_unlock_mutex(B2_mutex *mutex)
{
    if (mutex && mutex->sem) {
        xSemaphoreGive(mutex->sem);
    }
}

void B2_delete_mutex(B2_mutex *mutex)
{
    if (mutex) {
        if (mutex->sem) {
            vSemaphoreDelete(mutex->sem);
        }
        delete mutex;
    }
}

/*
 *  Display alerts
 */
void ErrorAlert(const char *text)
{
    Serial.printf("[ERROR] %s\n", text);
}

void WarningAlert(const char *text)
{
    Serial.printf("[WARNING] %s\n", text);
}

bool ChoiceAlert(const char *text, const char *pos, const char *neg)
{
    Serial.printf("[CHOICE] %s\n", text);
    return false;
}
