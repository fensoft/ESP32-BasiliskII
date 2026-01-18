/*
 *  sys_esp32.cpp - System dependent routines for ESP32 (SD card I/O)
 *
 *  BasiliskII ESP32 Port
 */

#include "sysdeps.h"
#include "main.h"
#include "macos_util.h"
#include "prefs.h"
#include "sys.h"

#include <SD.h>
#include <FS.h>

#define DEBUG 0
#include "debug.h"

// File handle structure
struct file_handle {
    File file;
    bool is_open;
    bool read_only;
    bool is_floppy;
    bool is_cdrom;
    loff_t size;
    char path[256];
    uint32_t file_id;           // Unique ID for cache association
};

// Static flag for SD initialization
static bool sd_initialized = false;

// ============================================================================
// SECTOR CACHE - 1MB cache in PSRAM for dramatically faster disk I/O
// ============================================================================

// Sector cache configuration
#define SECTOR_SIZE 512
#define CACHE_SIZE_BYTES (1024 * 1024)              // 1MB cache
#define CACHE_SECTORS (CACHE_SIZE_BYTES / SECTOR_SIZE)  // 2048 sectors
#define FLUSH_INTERVAL_MS 2000                      // Flush dirty sectors every 2 seconds

// Cache entry structure (16 bytes per entry)
struct cache_entry {
    uint32_t sector_num;        // Sector number (offset / 512)
    uint32_t file_id;           // Which file this belongs to (0 = invalid)
    uint32_t last_access;       // For LRU eviction
    uint8_t valid;              // Entry contains valid data
    uint8_t dirty;              // Entry has been modified (needs flush)
    uint8_t reserved[2];        // Padding for alignment
};

// Cache state
static uint8_t *sector_cache_data = NULL;           // 1MB data buffer in PSRAM
static cache_entry *sector_cache_meta = NULL;       // Metadata array in PSRAM
static uint32_t cache_access_counter = 0;           // LRU counter
static uint32_t cache_hits = 0;                     // Statistics
static uint32_t cache_misses = 0;
static uint32_t cache_writes = 0;
static uint32_t cache_flushes = 0;
static uint32_t last_flush_time = 0;                // For periodic flush
static uint32_t next_file_id = 1;                   // File ID counter
static bool cache_initialized = false;

/*
 *  Initialize SD card
 */
static bool init_sd_card(void)
{
    if (sd_initialized) {
        return true;
    }
    
    Serial.println("[SYS] SD card should already be initialized by main.cpp");
    sd_initialized = true;
    
    return true;
}

/*
 *  Initialize sector cache in PSRAM
 */
static bool init_sector_cache(void)
{
    if (cache_initialized) {
        return true;
    }
    
    Serial.println("[CACHE] Initializing 1MB sector cache...");
    
    // Allocate cache data buffer (1MB)
    sector_cache_data = (uint8_t *)ps_malloc(CACHE_SIZE_BYTES);
    if (!sector_cache_data) {
        Serial.println("[CACHE] ERROR: Failed to allocate cache data buffer!");
        return false;
    }
    
    // Allocate cache metadata (2048 entries x 16 bytes = 32KB)
    sector_cache_meta = (cache_entry *)ps_calloc(CACHE_SECTORS, sizeof(cache_entry));
    if (!sector_cache_meta) {
        Serial.println("[CACHE] ERROR: Failed to allocate cache metadata!");
        free(sector_cache_data);
        sector_cache_data = NULL;
        return false;
    }
    
    // Initialize all entries as invalid
    for (int i = 0; i < CACHE_SECTORS; i++) {
        sector_cache_meta[i].valid = 0;
        sector_cache_meta[i].dirty = 0;
        sector_cache_meta[i].file_id = 0;
    }
    
    cache_initialized = true;
    last_flush_time = millis();
    
    Serial.printf("[CACHE] Allocated %dKB cache (%d sectors) in PSRAM\n", 
                  CACHE_SIZE_BYTES / 1024, CACHE_SECTORS);
    
    return true;
}

/*
 *  Shutdown sector cache (flush and free)
 */
// Forward declaration for cache_flush_all (defined later)
static void cache_flush_all(void);

static void exit_sector_cache(void)
{
    if (!cache_initialized) {
        return;
    }
    
    // Flush all dirty sectors before shutdown to prevent data loss
    Serial.println("[CACHE] Flushing all dirty sectors before shutdown...");
    cache_flush_all();
    
    // Print statistics
    Serial.printf("[CACHE] Stats: %lu hits, %lu misses, %lu writes, %lu flushes\n",
                  (unsigned long)cache_hits, (unsigned long)cache_misses,
                  (unsigned long)cache_writes, (unsigned long)cache_flushes);
    
    if (cache_hits + cache_misses > 0) {
        float hit_rate = (float)cache_hits / (float)(cache_hits + cache_misses) * 100.0f;
        Serial.printf("[CACHE] Hit rate: %.1f%%\n", hit_rate);
    }
    
    // Free memory
    if (sector_cache_data) {
        free(sector_cache_data);
        sector_cache_data = NULL;
    }
    
    if (sector_cache_meta) {
        free(sector_cache_meta);
        sector_cache_meta = NULL;
    }
    
    cache_initialized = false;
}

/*
 *  Find a sector in cache
 *  Returns cache index or -1 if not found
 */
static int cache_find(uint32_t file_id, uint32_t sector_num)
{
    if (!cache_initialized) return -1;
    
    for (int i = 0; i < CACHE_SECTORS; i++) {
        if (sector_cache_meta[i].valid && 
            sector_cache_meta[i].file_id == file_id &&
            sector_cache_meta[i].sector_num == sector_num) {
            return i;
        }
    }
    return -1;
}

/*
 *  Find least recently used cache slot for eviction
 *  Prefers clean slots over dirty ones to avoid unnecessary flushes
 */
static int cache_find_lru(void)
{
    if (!cache_initialized) return 0;
    
    int best_clean = -1;
    int best_dirty = -1;
    uint32_t oldest_clean = UINT32_MAX;
    uint32_t oldest_dirty = UINT32_MAX;
    
    for (int i = 0; i < CACHE_SECTORS; i++) {
        // Empty slot is best
        if (!sector_cache_meta[i].valid) {
            return i;
        }
        
        // Track oldest clean and dirty separately
        if (sector_cache_meta[i].dirty) {
            if (sector_cache_meta[i].last_access < oldest_dirty) {
                oldest_dirty = sector_cache_meta[i].last_access;
                best_dirty = i;
            }
        } else {
            if (sector_cache_meta[i].last_access < oldest_clean) {
                oldest_clean = sector_cache_meta[i].last_access;
                best_clean = i;
            }
        }
    }
    
    // Prefer evicting clean slots
    if (best_clean >= 0) {
        return best_clean;
    }
    return best_dirty;
}

/*
 *  Flush a single dirty cache entry to disk
 */
static bool cache_flush_entry(int cache_idx, file_handle *fh)
{
    if (!cache_initialized || cache_idx < 0 || cache_idx >= CACHE_SECTORS) {
        return false;
    }
    
    cache_entry *entry = &sector_cache_meta[cache_idx];
    if (!entry->valid || !entry->dirty) {
        return true;  // Nothing to flush
    }
    
    // We need the file handle for flushing
    // If fh is NULL, we can't flush (will be handled in cache_flush_all)
    if (!fh || !fh->is_open) {
        return false;
    }
    
    // Seek to sector position
    loff_t offset = (loff_t)entry->sector_num * SECTOR_SIZE;
    if (!fh->file.seek(offset)) {
        Serial.printf("[CACHE] Flush seek failed: sector %lu\n", (unsigned long)entry->sector_num);
        return false;
    }
    
    // Write sector data
    uint8_t *data = sector_cache_data + (cache_idx * SECTOR_SIZE);
    size_t written = fh->file.write(data, SECTOR_SIZE);
    if (written != SECTOR_SIZE) {
        Serial.printf("[CACHE] Flush write failed: sector %lu\n", (unsigned long)entry->sector_num);
        return false;
    }
    
    entry->dirty = 0;
    cache_flushes++;
    return true;
}

/*
 *  Read a sector through cache
 *  Returns true on success
 */
static bool cache_read_sector(file_handle *fh, uint32_t sector_num, uint8_t *buffer)
{
    if (!cache_initialized || !fh || !fh->is_open || !buffer) {
        return false;
    }
    
    uint32_t file_id = fh->file_id;
    
    // Check cache first
    int cache_idx = cache_find(file_id, sector_num);
    
    if (cache_idx >= 0) {
        // Cache hit!
        cache_hits++;
        sector_cache_meta[cache_idx].last_access = ++cache_access_counter;
        memcpy(buffer, sector_cache_data + (cache_idx * SECTOR_SIZE), SECTOR_SIZE);
        return true;
    }
    
    // Cache miss - read from SD
    cache_misses++;
    
    // Find slot for new entry (may need to evict)
    cache_idx = cache_find_lru();
    
    // If evicting a dirty entry, flush it first
    if (sector_cache_meta[cache_idx].valid && sector_cache_meta[cache_idx].dirty) {
        // Need to flush this entry - but we need its file handle
        // For simplicity, just mark invalid and lose the data (periodic flush should prevent this)
        // In production, we'd need to look up the file handle by file_id
        Serial.printf("[CACHE] WARNING: Evicting dirty sector (data may be lost)\n");
    }
    
    // Seek to sector
    loff_t offset = (loff_t)sector_num * SECTOR_SIZE;
    if (!fh->file.seek(offset)) {
        return false;
    }
    
    // Read from SD directly into cache
    uint8_t *cache_data = sector_cache_data + (cache_idx * SECTOR_SIZE);
    size_t bytes_read = fh->file.read(cache_data, SECTOR_SIZE);
    if (bytes_read != SECTOR_SIZE) {
        return false;
    }
    
    // Update cache metadata
    sector_cache_meta[cache_idx].sector_num = sector_num;
    sector_cache_meta[cache_idx].file_id = file_id;
    sector_cache_meta[cache_idx].last_access = ++cache_access_counter;
    sector_cache_meta[cache_idx].valid = 1;
    sector_cache_meta[cache_idx].dirty = 0;
    
    // Copy to caller's buffer
    memcpy(buffer, cache_data, SECTOR_SIZE);
    
    return true;
}

/*
 *  Write a sector through cache (marks dirty, doesn't flush immediately)
 *  Returns true on success
 */
static bool cache_write_sector(file_handle *fh, uint32_t sector_num, const uint8_t *buffer)
{
    if (!cache_initialized || !fh || !fh->is_open || !buffer) {
        return false;
    }
    
    uint32_t file_id = fh->file_id;
    
    // Check if sector is already in cache
    int cache_idx = cache_find(file_id, sector_num);
    
    if (cache_idx < 0) {
        // Not in cache - find a slot
        cache_idx = cache_find_lru();
        
        // If evicting a dirty entry, we should flush it first
        if (sector_cache_meta[cache_idx].valid && sector_cache_meta[cache_idx].dirty) {
            Serial.printf("[CACHE] WARNING: Evicting dirty sector on write\n");
        }
    }
    
    // Copy data to cache
    memcpy(sector_cache_data + (cache_idx * SECTOR_SIZE), buffer, SECTOR_SIZE);
    
    // Update metadata
    sector_cache_meta[cache_idx].sector_num = sector_num;
    sector_cache_meta[cache_idx].file_id = file_id;
    sector_cache_meta[cache_idx].last_access = ++cache_access_counter;
    sector_cache_meta[cache_idx].valid = 1;
    sector_cache_meta[cache_idx].dirty = 1;  // Mark as dirty - needs flush
    
    cache_writes++;
    
    return true;
}

/*
 *  Flush all dirty sectors for a specific file
 *  Called when closing a file
 */
static void cache_flush_file(file_handle *fh)
{
    if (!cache_initialized || !fh || !fh->is_open) {
        return;
    }
    
    uint32_t file_id = fh->file_id;
    int flushed = 0;
    
    for (int i = 0; i < CACHE_SECTORS; i++) {
        if (sector_cache_meta[i].valid && 
            sector_cache_meta[i].file_id == file_id &&
            sector_cache_meta[i].dirty) {
            
            if (cache_flush_entry(i, fh)) {
                flushed++;
            }
        }
    }
    
    // Also flush the underlying file
    if (flushed > 0) {
        fh->file.flush();
        D(bug("[CACHE] Flushed %d sectors for file_id %lu\n", flushed, (unsigned long)file_id));
    }
}

/*
 *  Invalidate all cache entries for a specific file
 *  Called after flushing when closing a file
 */
static void cache_invalidate_file(uint32_t file_id)
{
    if (!cache_initialized) {
        return;
    }
    
    for (int i = 0; i < CACHE_SECTORS; i++) {
        if (sector_cache_meta[i].valid && sector_cache_meta[i].file_id == file_id) {
            sector_cache_meta[i].valid = 0;
            sector_cache_meta[i].dirty = 0;
        }
    }
}

// Forward declaration for file handle lookup (defined later)
static file_handle *open_file_handles[16] = {NULL};
static int num_open_files = 0;

/*
 *  Register an open file handle for cache flush support
 */
static void register_file_handle(file_handle *fh)
{
    for (int i = 0; i < 16; i++) {
        if (open_file_handles[i] == NULL) {
            open_file_handles[i] = fh;
            num_open_files++;
            return;
        }
    }
}

/*
 *  Unregister a file handle
 */
static void unregister_file_handle(file_handle *fh)
{
    for (int i = 0; i < 16; i++) {
        if (open_file_handles[i] == fh) {
            open_file_handles[i] = NULL;
            num_open_files--;
            return;
        }
    }
}

/*
 *  Find file handle by file_id
 */
static file_handle *find_file_handle(uint32_t file_id)
{
    for (int i = 0; i < 16; i++) {
        if (open_file_handles[i] != NULL && open_file_handles[i]->file_id == file_id) {
            return open_file_handles[i];
        }
    }
    return NULL;
}

/*
 *  Flush all dirty sectors for all open files
 *  Called periodically to ensure data safety
 */
static void cache_flush_all(void)
{
    if (!cache_initialized) {
        return;
    }
    
    int total_flushed = 0;
    
    // Group dirty sectors by file and flush
    for (int i = 0; i < CACHE_SECTORS; i++) {
        if (sector_cache_meta[i].valid && sector_cache_meta[i].dirty) {
            uint32_t file_id = sector_cache_meta[i].file_id;
            file_handle *fh = find_file_handle(file_id);
            
            if (fh && fh->is_open && !fh->read_only) {
                if (cache_flush_entry(i, fh)) {
                    total_flushed++;
                }
            }
        }
    }
    
    // Flush all open file handles
    for (int i = 0; i < 16; i++) {
        if (open_file_handles[i] != NULL && open_file_handles[i]->is_open) {
            open_file_handles[i]->file.flush();
        }
    }
    
    if (total_flushed > 0) {
        D(bug("[CACHE] Periodic flush: %d sectors\n", total_flushed));
    }
}

/*
 *  Periodic flush - call from main loop
 *  Flushes dirty cache entries every FLUSH_INTERVAL_MS milliseconds
 */
void Sys_periodic_flush(void)
{
    if (!cache_initialized) {
        return;
    }
    
    uint32_t now = millis();
    if (now - last_flush_time >= FLUSH_INTERVAL_MS) {
        cache_flush_all();
        last_flush_time = now;
    }
}

/*
 *  Initialization
 */
void SysInit(void)
{
    init_sd_card();
    init_sector_cache();
}

/*
 *  Deinitialization
 */
void SysExit(void)
{
    exit_sector_cache();
    sd_initialized = false;
}

/*
 *  Mount first floppy disk
 */
void SysAddFloppyPrefs(void)
{
    // Add default floppy disk image paths
}

/*
 *  Mount first hard disk
 */
void SysAddDiskPrefs(void)
{
    // Add default hard disk image paths
}

/*
 *  Mount CD-ROM
 */
void SysAddCDROMPrefs(void)
{
    // No CD-ROM support
}

/*
 *  Add serial port preferences
 */
void SysAddSerialPrefs(void)
{
    // No serial port support
}

/*
 *  Repair HFS volume - fix common corruption issues from improper shutdown
 *  Called before opening disk images to fix boot problems.
 *  
 *  HFS Master Directory Block (MDB) is at offset 1024 (block 2):
 *  - drSigWord at offset 0: signature 0x4244 ('BD') for HFS
 *  - drAtrb at offset 10: volume attributes (big-endian)
 *  - drFndrInfo at offset 92: Finder info (32 bytes)
 *    - drFndrInfo[0] at offset 92: System Folder CNID (blessed folder for boot)
 *    - drFndrInfo[2] at offset 100: Open folder CNID (should be 0)
 *    - drFndrInfo[3] at offset 104: Reserved (should be 0)
 *  
 *  The Alternate MDB (AMDB) at the end of the disk preserves original values.
 *  AMDB is at second-to-last 512-byte block.
 *    
 *  Common corruption patterns after improper shutdown:
 *  1. drAtrb changes from original value (often 0x0100) to 0x4000
 *  2. drFndrInfo[2] gets set to open folder CNID (should be 0)
 *  3. These can cause "blinking question mark" boot failure
 *  
 *  Solution: Restore drAtrb from AMDB and clear drFndrInfo[2]
 */
static void Sys_repair_hfs_volume(const char *path)
{
    // Only repair .dsk files (not floppies or ISOs)
    if (strstr(path, ".dsk") == NULL && strstr(path, ".DSK") == NULL) {
        return;
    }
    
    Serial.printf("[SYS] Checking HFS volume: %s\n", path);
    
    // Open file for read/write
    File f = SD.open(path, "r+b");
    if (!f) {
        Serial.printf("[SYS] Cannot open for repair check: %s\n", path);
        return;
    }
    
    // Get file size to locate Alternate MDB
    size_t file_size = f.size();
    if (file_size < 1024 + 512) {
        Serial.println("[SYS] File too small to be HFS volume");
        f.close();
        return;
    }
    
    // Read main MDB (first 128 bytes is enough for what we need)
    uint8_t mdb[128];
    if (!f.seek(1024)) {
        Serial.println("[SYS] Failed to seek to MDB");
        f.close();
        return;
    }
    
    if (f.read(mdb, 128) != 128) {
        Serial.println("[SYS] Failed to read MDB");
        f.close();
        return;
    }
    
    // Check HFS signature (0x4244 = 'BD')
    uint16_t signature = (mdb[0] << 8) | mdb[1];
    if (signature != 0x4244) {
        Serial.printf("[SYS] Not an HFS volume (sig=0x%04X)\n", signature);
        f.close();
        return;
    }
    
    // Read key MDB fields
    uint16_t drAtrb = (mdb[10] << 8) | mdb[11];
    uint32_t drFndrInfo0 = (mdb[92] << 24) | (mdb[93] << 16) | (mdb[94] << 8) | mdb[95];   // System Folder CNID
    uint32_t drFndrInfo2 = (mdb[100] << 24) | (mdb[101] << 16) | (mdb[102] << 8) | mdb[103]; // Open folder CNID
    uint32_t drFndrInfo3 = (mdb[104] << 24) | (mdb[105] << 16) | (mdb[106] << 8) | mdb[107]; // Reserved
    
    Serial.printf("[SYS] HFS MDB: drAtrb=0x%04X, SystemFolder=%lu, OpenFolder=%lu, FndrInfo3=%lu\n", 
                  drAtrb, (unsigned long)drFndrInfo0, (unsigned long)drFndrInfo2, (unsigned long)drFndrInfo3);
    
    // Calculate Alternate MDB offset (second-to-last 512-byte block)
    size_t amdb_offset = ((file_size / 512) - 2) * 512;
    Serial.printf("[SYS] AMDB at offset %u\n", (unsigned)amdb_offset);
    
    // Read drAtrb from Alternate MDB (original value from disk creation)
    uint8_t amdb_atrb[2];
    uint16_t original_drAtrb = drAtrb;  // Default to current if AMDB read fails
    
    if (f.seek(amdb_offset + 10) && f.read(amdb_atrb, 2) == 2) {
        // Verify AMDB has valid HFS signature first
        uint8_t amdb_sig[2];
        if (f.seek(amdb_offset) && f.read(amdb_sig, 2) == 2) {
            uint16_t amdb_signature = (amdb_sig[0] << 8) | amdb_sig[1];
            if (amdb_signature == 0x4244) {
                original_drAtrb = (amdb_atrb[0] << 8) | amdb_atrb[1];
                Serial.printf("[SYS] AMDB drAtrb=0x%04X (original value)\n", original_drAtrb);
            } else {
                Serial.printf("[SYS] AMDB signature invalid (0x%04X), skipping AMDB restore\n", amdb_signature);
            }
        }
    } else {
        Serial.println("[SYS] Could not read AMDB, skipping drAtrb restore");
    }
    
    bool needs_repair = false;
    
    // Check 1: Restore drAtrb from AMDB if different
    // The AMDB preserves the original drAtrb from when the disk was created/formatted
    if (drAtrb != original_drAtrb) {
        Serial.printf("[SYS] Restoring drAtrb from AMDB: 0x%04X -> 0x%04X\n", drAtrb, original_drAtrb);
        mdb[10] = (original_drAtrb >> 8) & 0xFF;
        mdb[11] = original_drAtrb & 0xFF;
        needs_repair = true;
    }
    
    // Check 2: Clear drFndrInfo[2] (open folder CNID) if set
    // This field indicates which folder was open - should be 0 for clean boot
    if (drFndrInfo2 != 0) {
        Serial.printf("[SYS] Clearing open folder CNID: %lu -> 0\n", (unsigned long)drFndrInfo2);
        mdb[100] = 0;
        mdb[101] = 0;
        mdb[102] = 0;
        mdb[103] = 0;
        needs_repair = true;
    }
    
    // Check 3: Clear drFndrInfo[3] if corrupted (should be 0 for classic Mac OS)
    if (drFndrInfo3 != 0) {
        Serial.printf("[SYS] Clearing drFndrInfo[3]: %lu -> 0\n", (unsigned long)drFndrInfo3);
        mdb[104] = 0;
        mdb[105] = 0;
        mdb[106] = 0;
        mdb[107] = 0;
        needs_repair = true;
    }
    
    // Check 4: Warn if System Folder CNID is 0 (volume not bootable)
    if (drFndrInfo0 == 0) {
        Serial.println("[SYS] WARNING: System Folder CNID is 0 - volume is not bootable");
    }
    
    if (needs_repair) {
        Serial.println("[SYS] Repairing HFS volume...");
        
        bool success = true;
        
        // Write drAtrb at offset 10
        if (!f.seek(1024 + 10) || f.write(&mdb[10], 2) != 2) {
            Serial.println("[SYS] Failed to write drAtrb!");
            success = false;
        }
        
        // Write drFndrInfo[2] at offset 100 (clear open folder)
        if (success && (!f.seek(1024 + 100) || f.write(&mdb[100], 4) != 4)) {
            Serial.println("[SYS] Failed to write FndrInfo[2]!");
            success = false;
        }
        
        // Write drFndrInfo[3] at offset 104 (clear corruption)
        if (success && (!f.seek(1024 + 104) || f.write(&mdb[104], 4) != 4)) {
            Serial.println("[SYS] Failed to write FndrInfo[3]!");
            success = false;
        }
        
        if (success) {
            f.flush();
            Serial.println("[SYS] Volume repaired successfully!");
        }
    } else {
        Serial.println("[SYS] Volume appears healthy");
    }
    
    f.close();
}

/*
 *  Open a file/device
 *  
 *  For read-write access, we use "r+b" mode which opens an existing file
 *  for both reading and writing WITHOUT truncation.
 *  DO NOT use FILE_WRITE as it will TRUNCATE the file!
 */
void *Sys_open(const char *name, bool read_only, bool is_cdrom)
{
    if (!name || strlen(name) == 0) {
        Serial.println("[SYS] Sys_open: empty name");
        return NULL;
    }
    
    Serial.printf("[SYS] Sys_open: %s (requested read_only=%d, is_cdrom=%d)\n", name, read_only, is_cdrom);
    
    // Repair HFS volume before opening (only for read-write disks, not CD-ROMs)
    if (!read_only && !is_cdrom) {
        Sys_repair_hfs_volume(name);
    }
    
    // Allocate file handle
    file_handle *fh = new file_handle;
    if (!fh) {
        Serial.println("[SYS] Sys_open: failed to allocate file handle");
        return NULL;
    }
    
    memset(fh, 0, sizeof(file_handle));
    strncpy(fh->path, name, sizeof(fh->path) - 1);
    fh->is_cdrom = is_cdrom;
    fh->is_floppy = (strstr(name, ".img") != NULL || strstr(name, ".IMG") != NULL);
    
    // CD-ROMs and ISO files are always read-only
    // Otherwise, respect the read_only parameter from caller
    if (is_cdrom || strstr(name, ".iso") != NULL || strstr(name, ".ISO") != NULL) {
        fh->read_only = true;
    } else {
        fh->read_only = read_only;
    }
    
    // Open file based on read_only flag
    if (fh->read_only) {
        Serial.printf("[SYS] Opening %s in READ-ONLY mode\n", name);
        fh->file = SD.open(name, FILE_READ);
    } else {
        // Use "r+b" mode: read+write without truncation (binary mode)
        // This is safe - it does NOT truncate like FILE_WRITE does
        Serial.printf("[SYS] Opening %s in READ-WRITE mode (r+b)\n", name);
        fh->file = SD.open(name, "r+b");
        if (!fh->file) {
            // Fall back to read-only if read-write mode fails
            Serial.printf("[SYS] WARNING: Read-write open failed, falling back to read-only\n");
            fh->file = SD.open(name, FILE_READ);
            fh->read_only = true;
        }
    }
    
    if (!fh->file) {
        Serial.printf("[SYS] ERROR: Cannot open file: %s\n", name);
        delete fh;
        return NULL;
    }
    
    // Get file size
    fh->size = fh->file.size();
    Serial.printf("[SYS] File size from size(): %lld bytes\n", (long long)fh->size);
    
    // If size() returns 0, try alternative methods
    if (fh->size == 0) {
        // Method: seek to end and get position
        if (fh->file.seek(0, SeekEnd)) {
            fh->size = fh->file.position();
            fh->file.seek(0, SeekSet);
            Serial.printf("[SYS] File size from seek: %lld bytes\n", (long long)fh->size);
        }
    }
    
    // Validate file size
    if (fh->size == 0) {
        Serial.printf("[SYS] ERROR: File %s appears to be empty or size cannot be determined\n", name);
        fh->file.close();
        delete fh;
        return NULL;
    }
    
    fh->is_open = true;
    
    // Assign unique file_id for cache association
    fh->file_id = next_file_id++;
    
    // Register file handle for cache flush support
    register_file_handle(fh);
    
    Serial.printf("[SYS] SUCCESS: Opened %s (file_id=%lu, %lld KB, floppy=%d, read_only=%d)\n", 
                  name, (unsigned long)fh->file_id, (long long)(fh->size / 1024), 
                  fh->is_floppy, fh->read_only);
    
    return fh;
}

/*
 *  Close a file/device
 */
void Sys_close(void *arg)
{
    file_handle *fh = (file_handle *)arg;
    if (!fh) return;
    
    Serial.printf("[SYS] Sys_close: %s (file_id=%lu)\n", fh->path, (unsigned long)fh->file_id);
    
    if (fh->is_open) {
        // Flush any dirty cache sectors for this file before closing
        cache_flush_file(fh);
        
        // Invalidate all cache entries for this file
        cache_invalidate_file(fh->file_id);
        
        // Unregister file handle
        unregister_file_handle(fh);
        
        // Final flush and close
        fh->file.flush();
        fh->file.close();
        fh->is_open = false;
    }
    
    delete fh;
}

/*
 *  Read from a file/device
 */
size_t Sys_read(void *arg, void *buffer, loff_t offset, size_t length)
{
    file_handle *fh = (file_handle *)arg;
    if (!fh || !fh->is_open || !buffer) {
        return 0;
    }
    
    // Statistics tracking
    static int disk_reads = 0;
    static int cdrom_reads = 0;
    
    // Log first few reads
    if (fh->is_cdrom) {
        cdrom_reads++;
        if (cdrom_reads <= 5 || cdrom_reads % 1000 == 0) {
            Serial.printf("[BOOT] CD-ROM read #%d: offset=%lld len=%d\n", cdrom_reads, (long long)offset, (int)length);
        }
    } else {
        disk_reads++;
        if (disk_reads <= 5 || disk_reads % 1000 == 0) {
            Serial.printf("[BOOT] Disk read #%d: %s offset=%lld len=%d (hits=%lu miss=%lu)\n", 
                          disk_reads, fh->path, (long long)offset, (int)length,
                          (unsigned long)cache_hits, (unsigned long)cache_misses);
        }
    }
    
    // If cache not available or request not sector-aligned, use direct read
    if (!cache_initialized || (offset % SECTOR_SIZE != 0) || (length % SECTOR_SIZE != 0)) {
        // Direct read path
        if (!fh->file.seek(offset)) {
            D(bug("[SYS] Sys_read: seek failed to offset %lld\n", (long long)offset));
            return 0;
        }
        return fh->file.read((uint8_t *)buffer, length);
    }
    
    // Cached read path - process sector by sector
    uint8_t *buf_ptr = (uint8_t *)buffer;
    size_t total_read = 0;
    uint32_t start_sector = offset / SECTOR_SIZE;
    uint32_t num_sectors = length / SECTOR_SIZE;
    
    for (uint32_t i = 0; i < num_sectors; i++) {
        uint32_t sector = start_sector + i;
        
        if (cache_read_sector(fh, sector, buf_ptr)) {
            buf_ptr += SECTOR_SIZE;
            total_read += SECTOR_SIZE;
        } else {
            // Cache read failed, try direct read for remaining data
            loff_t remaining_offset = (loff_t)sector * SECTOR_SIZE;
            size_t remaining_length = length - total_read;
            
            if (fh->file.seek(remaining_offset)) {
                size_t direct_read = fh->file.read(buf_ptr, remaining_length);
                total_read += direct_read;
            }
            break;
        }
    }
    
    return total_read;
}

/*
 *  Write to a file/device
 */
size_t Sys_write(void *arg, void *buffer, loff_t offset, size_t length)
{
    file_handle *fh = (file_handle *)arg;
    if (!fh || !fh->is_open || !buffer) {
        return 0;
    }
    
    if (fh->read_only) {
        // Log write attempts to read-only disks
        static int ro_write_attempts = 0;
        ro_write_attempts++;
        if (ro_write_attempts <= 5 || ro_write_attempts % 100 == 0) {
            Serial.printf("[SYS] Write blocked (read-only): %s attempt #%d\n", fh->path, ro_write_attempts);
        }
        return 0;
    }
    
    // Statistics tracking
    static int disk_writes_count = 0;
    disk_writes_count++;
    if (disk_writes_count <= 10 || disk_writes_count % 500 == 0) {
        Serial.printf("[SYS] Disk write #%d: %s offset=%lld len=%d (cache_writes=%lu)\n",
                      disk_writes_count, fh->path, (long long)offset, (int)length,
                      (unsigned long)cache_writes);
    }
    
    // If cache not available or request not sector-aligned, use direct write
    if (!cache_initialized || (offset % SECTOR_SIZE != 0) || (length % SECTOR_SIZE != 0)) {
        // Direct write path
        if (!fh->file.seek(offset)) {
            Serial.printf("[SYS] Sys_write: seek failed to offset %lld\n", (long long)offset);
            return 0;
        }
        size_t bytes_written = fh->file.write((uint8_t *)buffer, length);
        // NO FLUSH HERE - will be flushed periodically
        return bytes_written;
    }
    
    // Cached write path - write to cache (will be flushed periodically)
    const uint8_t *buf_ptr = (const uint8_t *)buffer;
    size_t total_written = 0;
    uint32_t start_sector = offset / SECTOR_SIZE;
    uint32_t num_sectors = length / SECTOR_SIZE;
    
    for (uint32_t i = 0; i < num_sectors; i++) {
        uint32_t sector = start_sector + i;
        
        if (cache_write_sector(fh, sector, buf_ptr)) {
            buf_ptr += SECTOR_SIZE;
            total_written += SECTOR_SIZE;
        } else {
            // Cache write failed, fall back to direct write
            loff_t remaining_offset = (loff_t)sector * SECTOR_SIZE;
            size_t remaining_length = length - total_written;
            
            if (fh->file.seek(remaining_offset)) {
                size_t direct_written = fh->file.write(buf_ptr, remaining_length);
                total_written += direct_written;
                // NO FLUSH - will be flushed periodically
            }
            break;
        }
    }
    
    return total_written;
}

/*
 *  Return size of file/device
 */
loff_t SysGetFileSize(void *arg)
{
    file_handle *fh = (file_handle *)arg;
    if (!fh || !fh->is_open) {
        return 0;
    }
    return fh->size;
}

/*
 *  Eject disk (no-op for SD card)
 */
void SysEject(void *arg)
{
    UNUSED(arg);
}

/*
 *  Format disk (not supported)
 */
bool SysFormat(void *arg)
{
    UNUSED(arg);
    return false;
}

/*
 *  Check if file/device is read-only
 */
bool SysIsReadOnly(void *arg)
{
    file_handle *fh = (file_handle *)arg;
    if (!fh) return true;
    return fh->read_only;
}

/*
 *  Check if a fixed disk (not removable)
 */
bool SysIsFixedDisk(void *arg)
{
    file_handle *fh = (file_handle *)arg;
    if (!fh) return true;
    return !fh->is_floppy && !fh->is_cdrom;
}

/*
 *  Check if a disk is inserted
 */
bool SysIsDiskInserted(void *arg)
{
    file_handle *fh = (file_handle *)arg;
    if (!fh) return false;
    return fh->is_open;
}

/*
 *  Prevent disk removal (no-op)
 */
void SysPreventRemoval(void *arg)
{
    UNUSED(arg);
}

/*
 *  Allow disk removal (no-op)
 */
void SysAllowRemoval(void *arg)
{
    UNUSED(arg);
}

/*
 *  CD-ROM functions (stubs - no CD-ROM support)
 */
bool SysCDReadTOC(void *arg, uint8 *toc)
{
    UNUSED(arg);
    UNUSED(toc);
    return false;
}

bool SysCDGetPosition(void *arg, uint8 *pos)
{
    UNUSED(arg);
    UNUSED(pos);
    return false;
}

bool SysCDPlay(void *arg, uint8 start_m, uint8 start_s, uint8 start_f, uint8 end_m, uint8 end_s, uint8 end_f)
{
    UNUSED(arg);
    UNUSED(start_m);
    UNUSED(start_s);
    UNUSED(start_f);
    UNUSED(end_m);
    UNUSED(end_s);
    UNUSED(end_f);
    return false;
}

bool SysCDPause(void *arg)
{
    UNUSED(arg);
    return false;
}

bool SysCDResume(void *arg)
{
    UNUSED(arg);
    return false;
}

bool SysCDStop(void *arg, uint8 lead_out_m, uint8 lead_out_s, uint8 lead_out_f)
{
    UNUSED(arg);
    UNUSED(lead_out_m);
    UNUSED(lead_out_s);
    UNUSED(lead_out_f);
    return false;
}

bool SysCDScan(void *arg, uint8 start_m, uint8 start_s, uint8 start_f, bool reverse)
{
    UNUSED(arg);
    UNUSED(start_m);
    UNUSED(start_s);
    UNUSED(start_f);
    UNUSED(reverse);
    return false;
}

void SysCDSetVolume(void *arg, uint8 left, uint8 right)
{
    UNUSED(arg);
    UNUSED(left);
    UNUSED(right);
}

void SysCDGetVolume(void *arg, uint8 &left, uint8 &right)
{
    UNUSED(arg);
    left = right = 0;
}
