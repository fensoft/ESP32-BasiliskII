/*
 *  prefs_esp32.cpp - Preferences handling, ESP32 implementation
 *
 *  Basilisk II (C) Christian Bauer
 *  ESP32 port (C) 2024
 */

#include "sysdeps.h"
#include "prefs.h"

// Platform-specific preferences items (none for ESP32)
prefs_desc platform_prefs_items[] = {
    {NULL, TYPE_END, false, NULL}
};

/*
 *  Platform-specific preferences defaults
 */
void AddPlatformPrefsDefaults(void)
{
    // Mac SE/Classic configuration (68000, 24-bit addressing, B&W)
    // Uses ROM version 0x0276 which is properly supported by BasiliskII
    PrefsAddInt32("ramsize", 4 * 1024 * 1024);  // 4MB RAM
    PrefsAddInt32("modelid", 5);  // Mac SE (Gestalt ID)
    PrefsAddInt32("cpu", 0);  // 68000
    PrefsAddBool("fpu", false);  // No FPU
    PrefsAddBool("nosound", true);  // Disable sound
    PrefsAddBool("nogui", true);  // No GUI
    
    // Screen: 512x342, 1-bit B&W (Mac SE/Classic native)
    PrefsAddString("screen", "win/512/342/1");
    
    // ROM file - Mac SE ROM (version 0x0276)
    PrefsAddString("rom", "/68000SE.ROM");
    
    // Boot floppy disk
    PrefsAddString("floppy", "/DiskTools.img");
    
    // Hard disk
    PrefsAddString("disk", "/Macintosh.dsk");
    
    /* Other configurations:
    // Mac Plus (NOT fully supported - ROM version 0x0075)
    PrefsAddString("rom", "/68000.rom");
    PrefsAddInt32("modelid", 4);  // Mac Plus
    
    // Mac IIci (needs driver installation fix)
    PrefsAddInt32("ramsize", 8 * 1024 * 1024);  // 8MB RAM
    PrefsAddInt32("modelid", 11);  // Mac IIci (Gestalt ID)
    PrefsAddInt32("cpu", 3);  // 68030
    PrefsAddString("screen", "win/640/480/8");
    PrefsAddString("rom", "/68030.ROM");
    */
}
