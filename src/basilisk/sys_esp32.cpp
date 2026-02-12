/*
 *  sys_esp32.cpp - System dependent routines for ESP32 (SD card I/O)
 *
 *  BasiliskII ESP32 Port
 *
 *  Direct I/O with lightweight sector cache.
 *
 *  SD card has internal caching, but a tiny in-memory read cache for aligned
 *  512-byte sectors helps repeated metadata reads without adding large overhead.
 */

#include "sysdeps.h"
#include "main.h"
#include "macos_util.h"
#include "prefs.h"
#include "sys.h"

#include <SD.h>
#include <FS.h>
#ifdef ARDUINO
#include <esp_heap_caps.h>
#endif

#define DEBUG 0
#include "debug.h"

#ifndef SYS_IO_PROFILE
#define SYS_IO_PROFILE 0
#endif

static constexpr size_t FILE_SECTOR_SIZE = 512;
static constexpr size_t FILE_SECTOR_CACHE_WAYS = 2;
static constexpr size_t FILE_SECTOR_CACHE_SETS = 4096;     // 2-way set-assoc
static constexpr size_t FILE_SECTOR_CACHE_ENTRIES = FILE_SECTOR_CACHE_SETS * FILE_SECTOR_CACHE_WAYS;  // 4MB total
static constexpr size_t FILE_READAHEAD_SECTORS = 8;       // 4KB read-ahead window
static constexpr bool FILE_READAHEAD_ON_ANY_512_MISS = true;

struct sector_cache_entry {
    uint8_t data[FILE_SECTOR_SIZE];
};

// File handle structure - minimal with dirty tracking
struct file_handle {
    File file;
    bool is_open;
    bool read_only;
    bool is_floppy;
    bool is_cdrom;
    bool is_dirty;      // Track if there are pending writes to flush
    bool pos_valid;     // Track cached file position to avoid redundant seek()
    loff_t pos;         // Current file position when pos_valid is true
    loff_t size;
    int32_t last_read_sector;
    uint8_t sequential_read_streak;
    sector_cache_entry *sector_cache;
    uint32_t *sector_cache_keys;
    uint32_t *sector_cache_tags;
    uint32_t sector_cache_epoch;
    uint8_t *sector_cache_victim;
    char path[256];
};

// Static flag for SD initialization
static bool sd_initialized = false;

// Open file handles for periodic flush
static file_handle *open_file_handles[16] = {NULL};

static inline sector_cache_entry *find_sector_cache_entry(file_handle *fh, uint32_t sector)
{
    if (fh == NULL || fh->sector_cache == NULL || fh->sector_cache_tags == NULL || fh->sector_cache_keys == NULL) return NULL;
    uint32_t set = sector & (FILE_SECTOR_CACHE_SETS - 1);
    sector_cache_entry *base = &fh->sector_cache[set * FILE_SECTOR_CACHE_WAYS];
    for (size_t way = 0; way < FILE_SECTOR_CACHE_WAYS; way++) {
        size_t idx = (set * FILE_SECTOR_CACHE_WAYS) + way;
        if (fh->sector_cache_tags[idx] == fh->sector_cache_epoch && fh->sector_cache_keys[idx] == sector) {
            return &base[way];
        }
    }
    return NULL;
}

static inline sector_cache_entry *select_sector_cache_slot(file_handle *fh, uint32_t sector)
{
    if (fh == NULL || fh->sector_cache == NULL || fh->sector_cache_tags == NULL ||
        fh->sector_cache_keys == NULL || fh->sector_cache_victim == NULL) return NULL;

    uint32_t set = sector & (FILE_SECTOR_CACHE_SETS - 1);
    sector_cache_entry *base = &fh->sector_cache[set * FILE_SECTOR_CACHE_WAYS];

    for (size_t way = 0; way < FILE_SECTOR_CACHE_WAYS; way++) {
        size_t idx = (set * FILE_SECTOR_CACHE_WAYS) + way;
        if (fh->sector_cache_tags[idx] != fh->sector_cache_epoch || fh->sector_cache_keys[idx] == sector) {
            return &base[way];
        }
    }

    uint8_t victim_way = fh->sector_cache_victim[set] & (FILE_SECTOR_CACHE_WAYS - 1);
    fh->sector_cache_victim[set] = (uint8_t)((victim_way + 1) & (FILE_SECTOR_CACHE_WAYS - 1));
    return &base[victim_way];
}

static inline void fill_sector_cache_entry(file_handle *fh, sector_cache_entry *entry, uint32_t sector, const uint8_t *src)
{
    if (fh == NULL || entry == NULL || src == NULL || fh->sector_cache == NULL ||
        fh->sector_cache_tags == NULL || fh->sector_cache_keys == NULL) return;
    size_t idx = (size_t)(entry - fh->sector_cache);
    if (idx >= FILE_SECTOR_CACHE_ENTRIES) return;
    fh->sector_cache_keys[idx] = sector;
    memcpy(entry->data, src, FILE_SECTOR_SIZE);
    fh->sector_cache_tags[idx] = fh->sector_cache_epoch;
}

#if SYS_IO_PROFILE
static uint64_t io_prof_read_calls = 0;
static uint64_t io_prof_read_bytes = 0;
static uint64_t io_prof_read_cache_hits = 0;
static uint64_t io_prof_read_cache_hit_bytes = 0;
static uint64_t io_prof_read_disk_calls = 0;
static uint64_t io_prof_seq_reads = 0;
static uint64_t io_prof_overlap_reads = 0;
static uint64_t io_prof_small_backtrack_reads = 0;
static uint64_t io_prof_len_512 = 0;
static uint64_t io_prof_len_1024 = 0;
static uint64_t io_prof_len_2048 = 0;
static uint64_t io_prof_len_4096 = 0;
static uint64_t io_prof_len_other = 0;
static uint64_t io_prof_off_le_4m = 0;
static uint64_t io_prof_off_le_8m = 0;
static uint64_t io_prof_off_gt_8m = 0;
static loff_t io_prof_last_read_offset = -1;
static loff_t io_prof_last_read_end = -1;
static uint32_t io_prof_last_report_ms = 0;

static inline void io_profile_report_and_reset(uint32_t now_ms)
{
    io_prof_last_report_ms = now_ms;
    Serial.printf("[SYS PERF] read_calls=%llu disk_calls=%llu read_bytes=%llu cache_hits=%llu cache_hit_bytes=%llu\n",
                  io_prof_read_calls,
                  io_prof_read_disk_calls,
                  io_prof_read_bytes,
                  io_prof_read_cache_hits,
                  io_prof_read_cache_hit_bytes);
    Serial.printf("[SYS PERF] seq=%llu overlap=%llu small_backtrack=%llu len(512=%llu 1k=%llu 2k=%llu 4k=%llu other=%llu)\n",
                  io_prof_seq_reads,
                  io_prof_overlap_reads,
                  io_prof_small_backtrack_reads,
                  io_prof_len_512,
                  io_prof_len_1024,
                  io_prof_len_2048,
                  io_prof_len_4096,
                  io_prof_len_other);
    Serial.printf("[SYS PERF] offset buckets <=4m=%llu <=8m=%llu >8m=%llu\n",
                  io_prof_off_le_4m,
                  io_prof_off_le_8m,
                  io_prof_off_gt_8m);

    io_prof_read_calls = 0;
    io_prof_read_bytes = 0;
    io_prof_read_cache_hits = 0;
    io_prof_read_cache_hit_bytes = 0;
    io_prof_read_disk_calls = 0;
    io_prof_seq_reads = 0;
    io_prof_overlap_reads = 0;
    io_prof_small_backtrack_reads = 0;
    io_prof_len_512 = 0;
    io_prof_len_1024 = 0;
    io_prof_len_2048 = 0;
    io_prof_len_4096 = 0;
    io_prof_len_other = 0;
    io_prof_off_le_4m = 0;
    io_prof_off_le_8m = 0;
    io_prof_off_gt_8m = 0;
}
#endif

static size_t read_aligned_sector_window(file_handle *fh,
                                         uint8_t *dst,
                                         loff_t offset,
                                         uint32_t start_sector,
                                         size_t sectors_requested,
                                         bool extend_to_window)
{
    if (fh == NULL || dst == NULL || sectors_requested == 0) return 0;

    if (!fh->pos_valid || fh->pos != offset) {
        if (!fh->file.seek(offset)) {
            fh->pos_valid = false;
            return 0;
        }
        fh->pos = offset;
        fh->pos_valid = true;
    }

    loff_t bytes_remaining = fh->size - offset;
    if (bytes_remaining <= 0) return 0;

    size_t max_sectors = (size_t)(bytes_remaining / (loff_t)FILE_SECTOR_SIZE);
    if (max_sectors == 0) return 0;
    if (max_sectors > FILE_READAHEAD_SECTORS) {
        max_sectors = FILE_READAHEAD_SECTORS;
    }

    size_t sectors_to_read = sectors_requested;
    if (sectors_to_read > max_sectors) {
        sectors_to_read = max_sectors;
    } else if (extend_to_window && max_sectors > sectors_to_read) {
        sectors_to_read = max_sectors;
    }

    if (sectors_to_read <= 1) {
#if SYS_IO_PROFILE
        io_prof_read_disk_calls++;
#endif
        size_t read_len = fh->file.read(dst, FILE_SECTOR_SIZE);
        if (read_len == 0) return 0;

        fh->pos += (loff_t)read_len;
        if (read_len == FILE_SECTOR_SIZE) {
            sector_cache_entry *slot = select_sector_cache_slot(fh, start_sector);
            if (slot != NULL) {
                fill_sector_cache_entry(fh, slot, start_sector, dst);
            }
        }
        return read_len;
    }

    uint8_t read_buf[FILE_SECTOR_SIZE * FILE_READAHEAD_SECTORS];
    const size_t read_bytes = sectors_to_read * FILE_SECTOR_SIZE;
#if SYS_IO_PROFILE
    io_prof_read_disk_calls++;
#endif
    size_t got = fh->file.read(read_buf, read_bytes);
    if (got == 0) return 0;

    fh->pos += (loff_t)got;

    size_t copy_len = got;
    const size_t requested_len = sectors_requested * FILE_SECTOR_SIZE;
    if (copy_len > requested_len) {
        copy_len = requested_len;
    }
    memcpy(dst, read_buf, copy_len);

    const size_t got_sectors = got / FILE_SECTOR_SIZE;
    for (size_t i = 0; i < got_sectors; i++) {
        uint32_t sector = start_sector + (uint32_t)i;
        sector_cache_entry *slot = select_sector_cache_slot(fh, sector);
        if (slot != NULL) {
            fill_sector_cache_entry(fh, slot, sector, read_buf + (i * FILE_SECTOR_SIZE));
        }
    }

    return copy_len;
}

static bool ensure_sector_cache(file_handle *fh)
{
    if (!fh) return false;
    if (fh->sector_cache != NULL && fh->sector_cache_keys != NULL &&
        fh->sector_cache_tags != NULL && fh->sector_cache_victim != NULL) return true;
#ifdef ARDUINO
    if (fh->sector_cache == NULL) {
        fh->sector_cache = (sector_cache_entry *)heap_caps_malloc(
            FILE_SECTOR_CACHE_ENTRIES * sizeof(sector_cache_entry),
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
        );
        if (fh->sector_cache == NULL) {
            fh->sector_cache = (sector_cache_entry *)heap_caps_malloc(
                FILE_SECTOR_CACHE_ENTRIES * sizeof(sector_cache_entry),
                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
            );
        }
    }
#else
    if (fh->sector_cache == NULL) {
        fh->sector_cache = (sector_cache_entry *)malloc(FILE_SECTOR_CACHE_ENTRIES * sizeof(sector_cache_entry));
    }
#endif
    if (fh->sector_cache == NULL) {
        return false;
    }

#ifdef ARDUINO
    if (fh->sector_cache_tags == NULL) {
        fh->sector_cache_tags = (uint32_t *)heap_caps_malloc(
            FILE_SECTOR_CACHE_ENTRIES * sizeof(uint32_t),
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
        );
        if (fh->sector_cache_tags == NULL) {
            fh->sector_cache_tags = (uint32_t *)heap_caps_malloc(
                FILE_SECTOR_CACHE_ENTRIES * sizeof(uint32_t),
                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
            );
        }
    }
#else
    if (fh->sector_cache_tags == NULL) {
        fh->sector_cache_tags = (uint32_t *)malloc(FILE_SECTOR_CACHE_ENTRIES * sizeof(uint32_t));
    }
#endif
    if (fh->sector_cache_tags == NULL) {
#ifdef ARDUINO
        heap_caps_free(fh->sector_cache);
#else
        free(fh->sector_cache);
#endif
        fh->sector_cache = NULL;
        return false;
    }

#ifdef ARDUINO
    if (fh->sector_cache_keys == NULL) {
        fh->sector_cache_keys = (uint32_t *)heap_caps_malloc(
            FILE_SECTOR_CACHE_ENTRIES * sizeof(uint32_t),
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
        );
        if (fh->sector_cache_keys == NULL) {
            fh->sector_cache_keys = (uint32_t *)heap_caps_malloc(
                FILE_SECTOR_CACHE_ENTRIES * sizeof(uint32_t),
                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
            );
        }
    }
#else
    if (fh->sector_cache_keys == NULL) {
        fh->sector_cache_keys = (uint32_t *)malloc(FILE_SECTOR_CACHE_ENTRIES * sizeof(uint32_t));
    }
#endif
    if (fh->sector_cache_keys == NULL) {
#ifdef ARDUINO
        heap_caps_free(fh->sector_cache);
        heap_caps_free(fh->sector_cache_tags);
#else
        free(fh->sector_cache);
        free(fh->sector_cache_tags);
#endif
        fh->sector_cache = NULL;
        fh->sector_cache_tags = NULL;
        return false;
    }

#ifdef ARDUINO
    if (fh->sector_cache_victim == NULL) {
        fh->sector_cache_victim = (uint8_t *)heap_caps_malloc(FILE_SECTOR_CACHE_SETS, MALLOC_CAP_8BIT);
    }
#else
    if (fh->sector_cache_victim == NULL) {
        fh->sector_cache_victim = (uint8_t *)malloc(FILE_SECTOR_CACHE_SETS);
    }
#endif
    if (fh->sector_cache_victim == NULL) {
#ifdef ARDUINO
        heap_caps_free(fh->sector_cache);
        heap_caps_free(fh->sector_cache_keys);
        heap_caps_free(fh->sector_cache_tags);
#else
        free(fh->sector_cache);
        free(fh->sector_cache_keys);
        free(fh->sector_cache_tags);
#endif
        fh->sector_cache = NULL;
        fh->sector_cache_keys = NULL;
        fh->sector_cache_tags = NULL;
        return false;
    }

    // Epoch-tagged cache: tags initialized invalid, keys zeroed.
    memset(fh->sector_cache_tags, 0, FILE_SECTOR_CACHE_ENTRIES * sizeof(uint32_t));
    memset(fh->sector_cache_keys, 0, FILE_SECTOR_CACHE_ENTRIES * sizeof(uint32_t));
    fh->sector_cache_epoch = 1;
    memset(fh->sector_cache_victim, 0, FILE_SECTOR_CACHE_SETS);
    return true;
}

static inline void invalidate_sector_cache_all(file_handle *fh)
{
    if (!fh || fh->sector_cache_tags == NULL) return;

    // O(1) invalidate: bump epoch. On wrap, clear tags and restart at epoch=1.
    fh->sector_cache_epoch++;
    if (fh->sector_cache_epoch == 0) {
        memset(fh->sector_cache_tags, 0, FILE_SECTOR_CACHE_ENTRIES * sizeof(uint32_t));
        fh->sector_cache_epoch = 1;
    }
    if (fh->sector_cache_victim != NULL) {
        memset(fh->sector_cache_victim, 0, FILE_SECTOR_CACHE_SETS);
    }
}

static inline void invalidate_sector_cache_range(file_handle *fh, loff_t offset, size_t length)
{
    if (!fh || length == 0) return;
    if (fh->sector_cache == NULL || fh->sector_cache_tags == NULL || fh->sector_cache_keys == NULL) return;

    const uint64_t first_sector = (uint64_t)offset >> 9;
    const uint64_t last_sector = ((uint64_t)offset + (uint64_t)length - 1ULL) >> 9;
    const uint64_t sector_count = (last_sector - first_sector) + 1ULL;

    // For large writes, epoch invalidate is cheaper than probing many sectors.
    if (sector_count > 256ULL) {
        invalidate_sector_cache_all(fh);
        return;
    }

    const uint32_t epoch = fh->sector_cache_epoch;
    for (uint64_t s = first_sector; s <= last_sector; s++) {
        uint32_t sector = (uint32_t)s;
        uint32_t set = sector & (FILE_SECTOR_CACHE_SETS - 1);
        size_t base = (size_t)set * FILE_SECTOR_CACHE_WAYS;
        for (size_t way = 0; way < FILE_SECTOR_CACHE_WAYS; way++) {
            size_t idx = base + way;
            if (fh->sector_cache_tags[idx] == epoch &&
                fh->sector_cache_keys[idx] == sector) {
                fh->sector_cache_tags[idx] = 0;
            }
        }
    }
}

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
 *  Register an open file handle
 */
static void register_file_handle(file_handle *fh)
{
    for (int i = 0; i < 16; i++) {
        if (open_file_handles[i] == NULL) {
            open_file_handles[i] = fh;
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
            return;
        }
    }
}

/*
 *  Periodic flush - ensures data is written to SD card
 *  Called every 2 seconds from main loop
 *  
 *  OPTIMIZED: Only flushes handles that have been written to since last flush.
 *  This avoids unnecessary SD card operations when files haven't changed.
 */
void Sys_periodic_flush(void)
{
#if SYS_IO_PROFILE
    uint32_t now = millis();
    if (now - io_prof_last_report_ms >= 5000) {
        io_profile_report_and_reset(now);
    }
#endif
    for (int i = 0; i < 16; i++) {
        file_handle *fh = open_file_handles[i];
        if (fh != NULL && fh->is_open && !fh->read_only && fh->is_dirty) {
            fh->file.flush();
            fh->is_dirty = false;  // Clear dirty flag after flush
        }
    }
}

/*
 *  Initialization
 */
void SysInit(void)
{
    init_sd_card();
    Serial.println("[SYS] Direct I/O mode (single-sector cache)");
}

/*
 *  Deinitialization
 */
void SysExit(void)
{
    // Flush all open files
    Sys_periodic_flush();
    sd_initialized = false;
}

/*
 *  Mount first floppy disk
 */
void SysAddFloppyPrefs(void)
{
}

/*
 *  Mount first hard disk
 */
void SysAddDiskPrefs(void)
{
}

/*
 *  Mount CD-ROM
 */
void SysAddCDROMPrefs(void)
{
}

/*
 *  Add serial port preferences
 */
void SysAddSerialPrefs(void)
{
}

/*
 *  Repair HFS volume - fix common corruption issues from improper shutdown
 */
static void Sys_repair_hfs_volume(const char *path)
{
    // Only repair .dsk files
    if (strstr(path, ".dsk") == NULL && strstr(path, ".DSK") == NULL) {
        return;
    }
    
    Serial.printf("[SYS] Checking HFS volume: %s\n", path);
    
    File f = SD.open(path, "r+b");
    if (!f) {
        return;
    }
    
    size_t file_size = f.size();
    if (file_size < 1024 + 512) {
        f.close();
        return;
    }
    
    // Read main MDB
    uint8_t mdb[128];
    if (!f.seek(1024) || f.read(mdb, 128) != 128) {
        f.close();
        return;
    }
    
    // Check HFS signature
    uint16_t signature = (mdb[0] << 8) | mdb[1];
    if (signature != 0x4244) {
        f.close();
        return;
    }
    
    // Read key fields
    uint16_t drAtrb = (mdb[10] << 8) | mdb[11];
    uint32_t drFndrInfo2 = (mdb[100] << 24) | (mdb[101] << 16) | (mdb[102] << 8) | mdb[103];
    uint32_t drFndrInfo3 = (mdb[104] << 24) | (mdb[105] << 16) | (mdb[106] << 8) | mdb[107];
    
    // Get original drAtrb from Alternate MDB
    size_t amdb_offset = ((file_size / 512) - 2) * 512;
    uint16_t original_drAtrb = drAtrb;
    
    uint8_t amdb_sig[2];
    if (f.seek(amdb_offset) && f.read(amdb_sig, 2) == 2) {
        if ((amdb_sig[0] << 8 | amdb_sig[1]) == 0x4244) {
            uint8_t amdb_atrb[2];
            if (f.seek(amdb_offset + 10) && f.read(amdb_atrb, 2) == 2) {
                original_drAtrb = (amdb_atrb[0] << 8) | amdb_atrb[1];
            }
        }
    }
    
    bool needs_repair = false;
    
    if (drAtrb != original_drAtrb) {
        mdb[10] = (original_drAtrb >> 8) & 0xFF;
        mdb[11] = original_drAtrb & 0xFF;
        needs_repair = true;
    }
    
    if (drFndrInfo2 != 0) {
        mdb[100] = mdb[101] = mdb[102] = mdb[103] = 0;
        needs_repair = true;
    }
    
    if (drFndrInfo3 != 0) {
        mdb[104] = mdb[105] = mdb[106] = mdb[107] = 0;
        needs_repair = true;
    }
    
    if (needs_repair) {
        Serial.println("[SYS] Repairing HFS volume...");
        f.seek(1024 + 10);
        f.write(&mdb[10], 2);
        f.seek(1024 + 100);
        f.write(&mdb[100], 8);
        f.flush();
        Serial.println("[SYS] Volume repaired");
    } else {
        Serial.println("[SYS] Volume OK");
    }
    
    f.close();
}

/*
 *  Open a file/device
 */
void *Sys_open(const char *name, bool read_only, bool is_cdrom)
{
    if (!name || strlen(name) == 0) {
        return NULL;
    }
    
    // Repair HFS volume before opening
    if (!read_only && !is_cdrom) {
        Sys_repair_hfs_volume(name);
    }
    
    file_handle *fh = new file_handle;
    if (!fh) {
        return NULL;
    }
    
    memset(fh, 0, sizeof(file_handle));
    strncpy(fh->path, name, sizeof(fh->path) - 1);
    fh->is_cdrom = is_cdrom;
    fh->is_floppy = (strstr(name, ".img") != NULL || strstr(name, ".IMG") != NULL);
    
    // Determine read-only status
    if (is_cdrom || strstr(name, ".iso") != NULL || strstr(name, ".ISO") != NULL) {
        fh->read_only = true;
    } else {
        fh->read_only = read_only;
    }
    
    // Open file
    if (fh->read_only) {
        fh->file = SD.open(name, FILE_READ);
    } else {
        fh->file = SD.open(name, "r+b");
        if (!fh->file) {
            fh->file = SD.open(name, FILE_READ);
            fh->read_only = true;
        }
    }
    
    if (!fh->file) {
        delete fh;
        return NULL;
    }
    
    fh->size = fh->file.size();
    if (fh->size == 0) {
        if (fh->file.seek(0, SeekEnd)) {
            fh->size = fh->file.position();
            fh->file.seek(0, SeekSet);
        }
    }
    
    if (fh->size == 0) {
        fh->file.close();
        delete fh;
        return NULL;
    }
    
    fh->is_open = true;
    fh->pos = 0;
    fh->pos_valid = true;
    fh->last_read_sector = -1;
    fh->sequential_read_streak = 0;
    fh->sector_cache_epoch = 0;
    register_file_handle(fh);
    
    Serial.printf("[SYS] Opened %s (%lld KB, ro=%d)\n", 
                  name, (long long)(fh->size / 1024), fh->read_only);
    
    return fh;
}

/*
 *  Close a file/device
 */
void Sys_close(void *arg)
{
    file_handle *fh = (file_handle *)arg;
    if (!fh) return;
    
    if (fh->is_open) {
        unregister_file_handle(fh);
        fh->file.flush();
        fh->file.close();
        fh->is_open = false;
    }

    if (fh->sector_cache != NULL) {
#ifdef ARDUINO
        heap_caps_free(fh->sector_cache);
#else
        free(fh->sector_cache);
#endif
        fh->sector_cache = NULL;
    }

    if (fh->sector_cache_tags != NULL) {
#ifdef ARDUINO
        heap_caps_free(fh->sector_cache_tags);
#else
        free(fh->sector_cache_tags);
#endif
        fh->sector_cache_tags = NULL;
    }

    if (fh->sector_cache_keys != NULL) {
#ifdef ARDUINO
        heap_caps_free(fh->sector_cache_keys);
#else
        free(fh->sector_cache_keys);
#endif
        fh->sector_cache_keys = NULL;
    }

    if (fh->sector_cache_victim != NULL) {
#ifdef ARDUINO
        heap_caps_free(fh->sector_cache_victim);
#else
        free(fh->sector_cache_victim);
#endif
        fh->sector_cache_victim = NULL;
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

#if SYS_IO_PROFILE
    io_prof_read_calls++;
    io_prof_read_bytes += length;
    if (length == 512) io_prof_len_512++;
    else if (length == 1024) io_prof_len_1024++;
    else if (length == 2048) io_prof_len_2048++;
    else if (length == 4096) io_prof_len_4096++;
    else io_prof_len_other++;
    if ((uint64_t)offset <= (4ULL * 1024ULL * 1024ULL)) io_prof_off_le_4m++;
    else if ((uint64_t)offset <= (8ULL * 1024ULL * 1024ULL)) io_prof_off_le_8m++;
    else io_prof_off_gt_8m++;

    if (io_prof_last_read_end >= 0) {
        if (offset == io_prof_last_read_end) {
            io_prof_seq_reads++;
        } else if (offset < io_prof_last_read_end && offset >= io_prof_last_read_offset) {
            io_prof_overlap_reads++;
        } else if (offset < io_prof_last_read_end && (io_prof_last_read_end - offset) <= 4096) {
            io_prof_small_backtrack_reads++;
        }
    }
    io_prof_last_read_offset = offset;
    io_prof_last_read_end = offset + (loff_t)length;

    uint32_t now = millis();
    if (now - io_prof_last_report_ms >= 5000) {
        io_profile_report_and_reset(now);
    }
#endif

    // Fast path for aligned multi-sector reads that are fully cache-resident.
    // This captures repeated 1KB/2KB/4KB metadata and catalog reads.
    if (length >= (2 * FILE_SECTOR_SIZE) &&
        length <= (FILE_SECTOR_SIZE * FILE_READAHEAD_SECTORS) &&
        (length & (FILE_SECTOR_SIZE - 1)) == 0 &&
        (offset & (FILE_SECTOR_SIZE - 1)) == 0 &&
        ensure_sector_cache(fh)) {
        const size_t sectors_needed = length >> 9;
        const uint32_t start_sector = (uint32_t)(offset >> 9);
        uint8_t *dst = (uint8_t *)buffer;
        bool all_cached = true;

        for (size_t i = 0; i < sectors_needed; i++) {
            sector_cache_entry *entry = find_sector_cache_entry(fh, start_sector + (uint32_t)i);
            if (entry == NULL) {
                all_cached = false;
                break;
            }
            memcpy(dst + (i * FILE_SECTOR_SIZE), entry->data, FILE_SECTOR_SIZE);
        }

        if (all_cached) {
            fh->last_read_sector = (int32_t)(start_sector + (uint32_t)sectors_needed - 1);
            if (sectors_needed > 1 && fh->sequential_read_streak < 0xFF) {
                fh->sequential_read_streak++;
            }
#if SYS_IO_PROFILE
            io_prof_read_cache_hits++;
            io_prof_read_cache_hit_bytes += length;
#endif
            return length;
        }

        size_t read_len = read_aligned_sector_window(
            fh,
            (uint8_t *)buffer,
            offset,
            start_sector,
            sectors_needed,
            false
        );
        if (read_len > 0) {
            const size_t full_sectors = read_len >> 9;
            if (full_sectors > 0) {
                fh->last_read_sector = (int32_t)(start_sector + (uint32_t)full_sectors - 1);
                if (full_sectors > 1) {
                    if (fh->sequential_read_streak < 0xFF) {
                        fh->sequential_read_streak++;
                    }
                } else {
                    fh->sequential_read_streak = 0;
                }
            } else {
                fh->last_read_sector = -1;
                fh->sequential_read_streak = 0;
            }
            return read_len;
        }
    }

    // Fast path for 512-byte aligned reads (dominant disk access pattern).
    if (length == FILE_SECTOR_SIZE && (offset & (FILE_SECTOR_SIZE - 1)) == 0 && ensure_sector_cache(fh)) {
        uint32_t sector = (uint32_t)(offset >> 9);
        bool is_sequential = (fh->last_read_sector >= 0 && sector == (uint32_t)(fh->last_read_sector + 1));
        if (is_sequential) {
            if (fh->sequential_read_streak < 0xFF) {
                fh->sequential_read_streak++;
            }
        } else {
            fh->sequential_read_streak = 0;
        }
        fh->last_read_sector = (int32_t)sector;

        sector_cache_entry *entry = find_sector_cache_entry(fh, sector);
        if (entry != NULL) {
            memcpy(buffer, entry->data, FILE_SECTOR_SIZE);
#if SYS_IO_PROFILE
            io_prof_read_cache_hits++;
            io_prof_read_cache_hit_bytes += FILE_SECTOR_SIZE;
#endif
            return FILE_SECTOR_SIZE;
        }

        const bool extend_to_window =
            ((is_sequential && fh->sequential_read_streak >= 1) || FILE_READAHEAD_ON_ANY_512_MISS);
        return read_aligned_sector_window(
            fh,
            (uint8_t *)buffer,
            offset,
            sector,
            1,
            extend_to_window
        );
    }

    const bool aligned_sector_read = ((offset & (FILE_SECTOR_SIZE - 1)) == 0);

    if (!fh->pos_valid || fh->pos != offset) {
        if (!fh->file.seek(offset)) {
            fh->pos_valid = false;
            return 0;
        }
        fh->pos = offset;
        fh->pos_valid = true;
    }

#if SYS_IO_PROFILE
    io_prof_read_disk_calls++;
#endif
    size_t read_len = fh->file.read((uint8_t *)buffer, length);
    if (read_len > 0) {
        fh->pos += (loff_t)read_len;
        if (aligned_sector_read && fh->sector_cache != NULL) {
            const size_t full_sectors = read_len >> 9;
            if (full_sectors > 0) {
                uint32_t sector = (uint32_t)(offset >> 9);
                const uint8_t *src = (const uint8_t *)buffer;

                // Seed cache from aligned multi-sector reads to improve follow-on
                // 512-byte metadata access hit rate.
                const size_t cache_sectors =
                    (full_sectors > FILE_READAHEAD_SECTORS) ? FILE_READAHEAD_SECTORS : full_sectors;
                for (size_t i = 0; i < cache_sectors; i++) {
                    sector_cache_entry *entry = select_sector_cache_slot(fh, sector + (uint32_t)i);
                    if (entry != NULL) {
                        fill_sector_cache_entry(
                            fh,
                            entry,
                            sector + (uint32_t)i,
                            src + (i * FILE_SECTOR_SIZE)
                        );
                    }
                }

                // Preserve sequential stream context across aligned multi-sector reads.
                fh->last_read_sector = (int32_t)(sector + (uint32_t)full_sectors - 1);
                if (full_sectors > 1) {
                    if (fh->sequential_read_streak < 0xFF) {
                        fh->sequential_read_streak++;
                    }
                } else {
                    fh->sequential_read_streak = 0;
                }
            } else {
                fh->last_read_sector = -1;
                fh->sequential_read_streak = 0;
            }
        } else {
            // Non-sector-aligned reads break 512-byte stream detection.
            fh->last_read_sector = -1;
            fh->sequential_read_streak = 0;
        }
    } else {
        fh->last_read_sector = -1;
        fh->sequential_read_streak = 0;
    }
    return read_len;
}

/*
 *  Write to a file/device - direct write, no buffering
 *  Marks handle dirty for deferred flush
 */
size_t Sys_write(void *arg, void *buffer, loff_t offset, size_t length)
{
    file_handle *fh = (file_handle *)arg;
    if (!fh || !fh->is_open || !buffer || fh->read_only) {
        return 0;
    }

    if (!fh->pos_valid || fh->pos != offset) {
        if (!fh->file.seek(offset)) {
            fh->pos_valid = false;
            return 0;
        }
        fh->pos = offset;
        fh->pos_valid = true;
    }

    size_t written = fh->file.write((uint8_t *)buffer, length);
    if (written > 0) {
        const bool full_sector_aligned_write =
            ((offset & (FILE_SECTOR_SIZE - 1)) == 0) &&
            ((written & (FILE_SECTOR_SIZE - 1)) == 0);

        if (full_sector_aligned_write && ensure_sector_cache(fh)) {
            const uint32_t start_sector = (uint32_t)(offset >> 9);
            const size_t sectors = written >> 9;
            const uint8_t *src = (const uint8_t *)buffer;
            for (size_t i = 0; i < sectors; i++) {
                const uint32_t sector = start_sector + (uint32_t)i;
                sector_cache_entry *entry = select_sector_cache_slot(fh, sector);
                if (entry != NULL) {
                    fill_sector_cache_entry(
                        fh,
                        entry,
                        sector,
                        src + (i * FILE_SECTOR_SIZE)
                    );
                }
            }
        } else {
            invalidate_sector_cache_range(fh, offset, written);
        }
        fh->is_dirty = true;  // Mark for deferred flush
        fh->pos += (loff_t)written;
    }
    return written;
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
 *  Eject disk (no-op)
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
 *  Check if a fixed disk
 */
bool SysIsFixedDisk(void *arg)
{
    file_handle *fh = (file_handle *)arg;
    if (!fh) return true;
    return !fh->is_floppy && !fh->is_cdrom;
}

/*
 *  Check if disk is inserted
 */
bool SysIsDiskInserted(void *arg)
{
    file_handle *fh = (file_handle *)arg;
    if (!fh) return false;
    return fh->is_open;
}

void SysPreventRemoval(void *arg) { UNUSED(arg); }
void SysAllowRemoval(void *arg) { UNUSED(arg); }

// CD-ROM stubs
bool SysCDReadTOC(void *arg, uint8 *toc) { UNUSED(arg); UNUSED(toc); return false; }
bool SysCDGetPosition(void *arg, uint8 *pos) { UNUSED(arg); UNUSED(pos); return false; }
bool SysCDPlay(void *arg, uint8 start_m, uint8 start_s, uint8 start_f, uint8 end_m, uint8 end_s, uint8 end_f) {
    UNUSED(arg); UNUSED(start_m); UNUSED(start_s); UNUSED(start_f);
    UNUSED(end_m); UNUSED(end_s); UNUSED(end_f); return false;
}
bool SysCDPause(void *arg) { UNUSED(arg); return false; }
bool SysCDResume(void *arg) { UNUSED(arg); return false; }
bool SysCDStop(void *arg, uint8 lead_out_m, uint8 lead_out_s, uint8 lead_out_f) {
    UNUSED(arg); UNUSED(lead_out_m); UNUSED(lead_out_s); UNUSED(lead_out_f); return false;
}
bool SysCDScan(void *arg, uint8 start_m, uint8 start_s, uint8 start_f, bool reverse) {
    UNUSED(arg); UNUSED(start_m); UNUSED(start_s); UNUSED(start_f); UNUSED(reverse); return false;
}
void SysCDSetVolume(void *arg, uint8 left, uint8 right) { UNUSED(arg); UNUSED(left); UNUSED(right); }
void SysCDGetVolume(void *arg, uint8 &left, uint8 &right) { UNUSED(arg); left = right = 0; }
