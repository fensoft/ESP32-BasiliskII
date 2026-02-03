/*
 *  video_esp32.cpp - Video/graphics emulation for ESP32-P4 with M5GFX
 *
 *  BasiliskII ESP32 Port
 *
 *  Dual-core optimized: Video rendering runs on Core 0, CPU emulation on Core 1
 *  
 *  OPTIMIZATIONS:
 *  1. 8-bit indexed frame buffer - minimizes PSRAM bandwidth
 *     - mac_frame_buffer: CPU writes here (8-bit indexed, 230KB)
 *     - Conversion to RGB565 happens at display write time
 *  2. Write-time dirty tracking - CPU marks tiles dirty as it writes
 *     - No per-frame comparison needed (eliminates ~460KB PSRAM traffic)
 *     - Dirty tiles tracked via atomic bitmap operations
 *  3. Tile-based partial updates - only updates changed screen regions
 *     - Screen divided into 16x9 grid of 40x40 pixel tiles (144 tiles total)
 *     - Only renders and pushes tiles that have changed
 *     - Falls back to full update if >80% of tiles are dirty (reduces API overhead)
 *     - Working buffers placed in internal SRAM for fast access
 *  
 *  TUNING PARAMETERS (defined below):
 *  - TILE_WIDTH/TILE_HEIGHT: Tile size in Mac pixels (40x40 default)
 *  - DIRTY_THRESHOLD_PERCENT: Threshold for switching to full update (80% default)
 *  - VIDEO_SIGNAL_INTERVAL: Frame rate target in main_esp32.cpp (~15 FPS)
 */

#include "sysdeps.h"
#include "cpu_emulation.h"
#include "main.h"
#include "adb.h"
#include "prefs.h"
#include "video.h"
#include "video_defs.h"

#include <M5Unified.h>
#include <M5GFX.h>

// FreeRTOS for dual-core support
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Watchdog timer control
#include "esp_task_wdt.h"

// ESP-IDF memory attributes (DRAM_ATTR for internal SRAM placement)
#include "esp_attr.h"
#include <esp_heap_caps.h>

// Cache control for DMA visibility
#if __has_include(<esp_cache.h>)
#include <esp_cache.h>
#define HAS_ESP_CACHE 1
#else
#define HAS_ESP_CACHE 0
#endif

// Cache line size for ESP32-P4 (64 bytes)
#define CACHE_LINE_SIZE 64

#define DEBUG 1
#include "debug.h"

// Display configuration - 640x360 with 2x pixel doubling for 1280x720 display
#define MAC_SCREEN_WIDTH  640
#define MAC_SCREEN_HEIGHT 360
#define MAC_SCREEN_DEPTH  VDEPTH_8BIT  // 8-bit indexed color
#define PIXEL_SCALE       2            // 2x scaling to fill 1280x720

// Physical display dimensions
#define DISPLAY_WIDTH     1280
#define DISPLAY_HEIGHT    720

// Tile-based dirty tracking configuration
// Tile size: 80x40 Mac pixels (160x80 display pixels after 2x scaling)
// Grid: 8 columns x 9 rows = 72 tiles total
// Coverage: 640x360 exactly (80*8=640, 40*9=360)
// OPTIMIZATION: Larger tiles reduce per-tile overhead (API calls, locks, etc.)
// while horizontal strip coalescing handles cases where only small regions change.
#define TILE_WIDTH        80
#define TILE_HEIGHT       40
#define TILES_X           8
#define TILES_Y           9
#define TOTAL_TILES       (TILES_X * TILES_Y)  // 72 tiles

// Dirty tile threshold - if more than this percentage of tiles are dirty,
// do a full update instead of partial
// NOTE: Set to 101 to ALWAYS use tile mode - tile updates are actually faster
// than full streaming even when all tiles are dirty, because tile mode uses
// double-buffered DMA while streaming mode processes rows sequentially
#define DIRTY_THRESHOLD_PERCENT  101


// Video task configuration
#define VIDEO_TASK_STACK_SIZE  8192
#define VIDEO_TASK_PRIORITY    1
#define VIDEO_TASK_CORE        0  // Run on Core 0, leaving Core 1 for CPU emulation

// Streaming full-frame path (disabled; tile mode is faster here)
// Set to 1 to enable streaming buffers and renderFrameStreaming().
#define VIDEO_USE_STREAMING 0

// ============================================================================
// Write-Through Queue Configuration
// ============================================================================
// When enabled, pixel data is captured at write time and pushed directly to
// display from the queue, eliminating PSRAM read-back for video rendering.
// This halves PSRAM video traffic (write-only, no reads for display).
// Set to 0 to use the original PSRAM read-back rendering method.
#ifndef VIDEO_USE_WRITE_THROUGH_QUEUE
#define VIDEO_USE_WRITE_THROUGH_QUEUE 0  // Disabled - queue overflow makes it slower than PSRAM read-back
#endif

#if VIDEO_USE_WRITE_THROUGH_QUEUE
// Queue size: number of write entries (8 bytes each)
// ~16KB internal SRAM = 2048 entries, reduces overflow under heavy writes
#define WRITE_QUEUE_SIZE 2048
#endif

// Frame buffer for Mac emulation (CPU writes here)
static uint8 *mac_frame_buffer = NULL;
static uint32 frame_buffer_size = 0;

// Tile rendering buffers (allocated in PSRAM to preserve internal SRAM)
static uint8 *tile_snapshot_psram = NULL;
static uint16 *tile_buffer_psram = NULL;

// Frame synchronization
static volatile bool frame_ready = false;
static portMUX_TYPE frame_spinlock = portMUX_INITIALIZER_UNLOCKED;

// Video task handle
static TaskHandle_t video_task_handle = NULL;
static volatile bool video_task_running = false;

// Palette (256 RGB565 entries) - in internal SRAM for fast access during rendering
// This is accessed for every pixel during video conversion
DRAM_ATTR static uint16 palette_rgb565[256];
// Duplicate-pixel palette (two RGB565 pixels packed into one uint32)
// Allows 32-bit stores for 2x horizontal scaling
DRAM_ATTR static uint32 palette_rgb565_dup32[256];

// Flag to track if palette has changed - avoids unnecessary copies in video task
static volatile bool palette_changed = true;

// Dirty tile bitmap - in internal SRAM for fast access during video frame processing
DRAM_ATTR static uint32 dirty_tiles[(TOTAL_TILES + 31) / 32];          // Bitmap of dirty tiles (read by video task)

// Write-time dirty tracking bitmap - marked when CPU writes to framebuffer
// This is double-buffered to avoid race conditions between CPU writes and video task reads
DRAM_ATTR static uint32 write_dirty_tiles[(TOTAL_TILES + 31) / 32];    // Tiles dirtied by CPU writes
// CPU-side dirty bitmap (no atomics on hot write path)
DRAM_ATTR static uint32 cpu_dirty_tiles[(TOTAL_TILES + 31) / 32];

// Per-tile render lock bitmap - set while video task is snapshotting a tile
// If CPU tries to write while this is set, the tile is re-marked dirty for next frame
// This prevents torn data from race conditions during snapshot
DRAM_ATTR static uint32 tile_render_active[(TOTAL_TILES + 31) / 32];   // Tiles currently being rendered

// Dirty-mark lookup tables (8-bit fast path)
// byte-in-row -> tile_x, row -> tile_y
DRAM_ATTR static uint8 tile_x_for_byte_in_row[MAC_SCREEN_WIDTH];
DRAM_ATTR static uint8 tile_y_for_row[MAC_SCREEN_HEIGHT];

// ============================================================================
// Write-Through Queue Data Structures
// ============================================================================
#if VIDEO_USE_WRITE_THROUGH_QUEUE

// Queue entry structure - 8 bytes each, aligned for efficient access
// Stores pixel data at write time to avoid PSRAM read-back during rendering
struct WriteQueueEntry {
    uint32_t offset;      // Byte offset into frame buffer (0 to frame_buffer_size-1)
    uint8_t data[4];      // Pixel data (1-4 bytes depending on write size)
    uint8_t size;         // Number of valid bytes in data[] (1, 2, or 4)
    uint8_t padding[3];   // Padding to align to 8 bytes
};

// Circular queue in internal SRAM for fast access from both CPU and video task
// Size: WRITE_QUEUE_SIZE * 8 bytes = 8KB for 1024 entries
DRAM_ATTR static WriteQueueEntry write_queue[WRITE_QUEUE_SIZE];
DRAM_ATTR static volatile uint32_t queue_head = 0;  // Write position (CPU writes here)
DRAM_ATTR static volatile uint32_t queue_tail = 0;  // Read position (video task reads from here)

// Per-tile PSRAM fallback flags - if queue overflows, we fall back to reading
// from PSRAM for affected tiles (graceful degradation)
DRAM_ATTR static uint32_t tile_needs_psram_read[(TOTAL_TILES + 31) / 32];

// ============================================================================
// Tile Content Cache - maintains shadow copies of recently-dirty tiles
// This allows rendering without reading from PSRAM
// ============================================================================

// Number of tiles we can cache in internal SRAM
// 4 tiles * 3200 bytes = 12.8KB, reasonable for internal SRAM
// (Reduced from 8 since tiles are now larger at 80x40)
#define TILE_CACHE_SIZE 4

// Tile cache entry - holds shadow copy of tile content
struct TileCacheEntry {
    int16_t tile_idx;     // Which tile this is (-1 if unused)
    uint16_t access_count; // LRU counter (higher = more recently used)
    uint8_t content[TILE_WIDTH * TILE_HEIGHT];  // 3200 bytes of pixel data (8-bit indices)
};

// Tile cache in internal SRAM
DRAM_ATTR static TileCacheEntry tile_cache[TILE_CACHE_SIZE];

// LRU counter - incremented on each cache access
static uint16_t cache_lru_counter = 0;

// Mapping from tile_idx to cache slot (-1 if not cached)
// This allows O(1) lookup of whether a tile is in cache
DRAM_ATTR static int8_t tile_to_cache_slot[TOTAL_TILES];

// Debug/profiling statistics for write-through queue
static uint32_t dbg_queue_writes = 0;          // Total writes queued
static uint32_t dbg_queue_max_depth = 0;       // Peak queue depth seen
static uint32_t dbg_queue_overflows = 0;       // Times queue was full (overflow)
static uint32_t dbg_psram_fallbacks = 0;       // Tiles that fell back to PSRAM read
static uint32_t dbg_readback_count = 0;        // Mac software read-back attempts
static uint32_t dbg_queue_entries_processed = 0;  // Queue entries processed per frame
static uint32_t dbg_cache_hits = 0;            // Tile was found in cache
static uint32_t dbg_cache_misses = 0;          // Tile needed to be loaded into cache
static uint32_t dbg_cache_evictions = 0;       // Tiles evicted from cache

// Forward declaration
static void snapshotTile(uint8 *src_buffer, int tile_x, int tile_y, uint8 *snapshot);

// Initialize tile cache at startup
static void initTileCache(void)
{
    for (int i = 0; i < TILE_CACHE_SIZE; i++) {
        tile_cache[i].tile_idx = -1;
        tile_cache[i].access_count = 0;
    }
    for (int i = 0; i < TOTAL_TILES; i++) {
        tile_to_cache_slot[i] = -1;
    }
    cache_lru_counter = 0;
}

// Find a cache slot for a tile (allocate if needed, evict LRU if full)
// Returns slot index, or -1 if allocation failed
static int allocateCacheSlot(int tile_idx)
{
    // Check if tile is already cached
    int8_t existing_slot = tile_to_cache_slot[tile_idx];
    if (existing_slot >= 0) {
        // Update LRU
        tile_cache[existing_slot].access_count = ++cache_lru_counter;
        dbg_cache_hits++;
        return existing_slot;
    }
    
    // Find an empty slot or LRU slot to evict
    int best_slot = 0;
    uint16_t oldest_access = tile_cache[0].access_count;
    
    for (int i = 0; i < TILE_CACHE_SIZE; i++) {
        if (tile_cache[i].tile_idx < 0) {
            // Empty slot - use it
            best_slot = i;
            break;
        }
        if (tile_cache[i].access_count < oldest_access) {
            oldest_access = tile_cache[i].access_count;
            best_slot = i;
        }
    }
    
    // Evict if slot was in use
    if (tile_cache[best_slot].tile_idx >= 0) {
        int evicted_tile = tile_cache[best_slot].tile_idx;
        tile_to_cache_slot[evicted_tile] = -1;
        dbg_cache_evictions++;
    }
    
    // Assign slot to new tile
    tile_cache[best_slot].tile_idx = tile_idx;
    tile_cache[best_slot].access_count = ++cache_lru_counter;
    tile_to_cache_slot[tile_idx] = best_slot;
    
    dbg_cache_misses++;
    return best_slot;
}

// Get cache slot for a tile, or -1 if not cached
static inline int getCacheSlot(int tile_idx)
{
    return tile_to_cache_slot[tile_idx];
}

// Forward declaration - implemented after variable declarations
static void applyCacheWrite(uint32_t offset, const uint8_t* data, uint32_t size);

#endif // VIDEO_USE_WRITE_THROUGH_QUEUE

// Double-buffered row buffers for streaming full-frame renders with async DMA
// (Disabled by default to free internal SRAM; tile mode is always used)
#define STREAMING_ROW_COUNT 8
#if VIDEO_USE_STREAMING
alignas(4) DRAM_ATTR static uint16 streaming_row_buffer_a[DISPLAY_WIDTH * STREAMING_ROW_COUNT];
alignas(4) DRAM_ATTR static uint16 streaming_row_buffer_b[DISPLAY_WIDTH * STREAMING_ROW_COUNT];
static uint16 *render_buffer = streaming_row_buffer_a;
static uint16 *push_buffer = streaming_row_buffer_b;
#else
static uint16 *render_buffer = NULL;
static uint16 *push_buffer = NULL;
#endif

static volatile bool force_full_update = true;               // Force full update on first frame or palette change
static int dirty_tile_count = 0;                             // Count of dirty tiles for threshold check

// Display dimensions (from M5.Display)
static int display_width = 0;
static int display_height = 0;

// Video mode info
static video_mode current_mode;

// Current video state cache - updated on mode switch for fast access during rendering
// These are used by the render loops and dirty tracking to handle different bit depths
static volatile video_depth current_depth = VDEPTH_8BIT;  // Current color depth
static volatile uint32 current_bytes_per_row = MAC_SCREEN_WIDTH;  // Bytes per row in frame buffer
static volatile int current_pixels_per_byte = 1;  // Pixels packed per byte (8=1bit, 4=2bit, 2=4bit, 1=8bit)
static volatile int current_bit_shift = 0;  // Bits to shift per pixel (7=1bit, 6=2bit, 4=4bit, 0=8bit)
static volatile uint8 current_pixel_mask = 0xFF;  // Mask for extracting pixel value

// ============================================================================
// Write-Through Queue - applyCacheWrite implementation
// (Defined here because it needs current_* variables declared above)
// ============================================================================
#if VIDEO_USE_WRITE_THROUGH_QUEUE

/*
 *  Apply a write to the tile cache
 *  Called from VideoQueueWrite path to update cached tile content directly
 *  with the write data, avoiding later PSRAM reads.
 *  
 *  @param offset  Byte offset into the Mac framebuffer
 *  @param data    Pointer to pixel data being written
 *  @param size    Number of bytes (1, 2, or 4)
 */
static void applyCacheWrite(uint32_t offset, const uint8_t* data, uint32_t size)
{
    if (offset >= frame_buffer_size) return;
    
    uint32_t bpr = current_bytes_per_row;
    video_depth depth = current_depth;
    
    // For 8-bit mode, we can update cache directly
    // For packed modes, we need to decode the write
    if (depth == VDEPTH_8BIT) {
        // Simple case: each byte offset = one pixel
        for (uint32_t i = 0; i < size; i++) {
            uint32_t byte_offset = offset + i;
            if (byte_offset >= frame_buffer_size) break;
            
            int y = byte_offset / bpr;
            int x = byte_offset % bpr;
            
            if (y >= MAC_SCREEN_HEIGHT || x >= MAC_SCREEN_WIDTH) continue;
            
            int tile_x = x / TILE_WIDTH;
            int tile_y = y / TILE_HEIGHT;
            int tile_idx = tile_y * TILES_X + tile_x;
            
            int cache_slot = getCacheSlot(tile_idx);
            if (cache_slot >= 0) {
                // Calculate position within tile
                int local_x = x % TILE_WIDTH;
                int local_y = y % TILE_HEIGHT;
                int local_offset = local_y * TILE_WIDTH + local_x;
                
                tile_cache[cache_slot].content[local_offset] = data[i];
            }
        }
    }
    // For packed modes (1/2/4-bit), we'd need more complex logic
    // For now, packed mode writes will still work via the PSRAM fallback path
}

#endif // VIDEO_USE_WRITE_THROUGH_QUEUE

// ============================================================================
// Performance profiling counters (lightweight, always enabled)
// ============================================================================
static volatile uint32_t perf_detect_us = 0;        // Time to detect dirty tiles
static volatile uint32_t perf_render_us = 0;        // Time to render and push frame
static volatile uint32_t perf_frame_count = 0;      // Frames rendered
static volatile uint32_t perf_partial_count = 0;    // Partial updates
static volatile uint32_t perf_full_count = 0;       // Full updates
static volatile uint32_t perf_skip_count = 0;       // Skipped frames (no changes)
static volatile uint32_t perf_last_report_ms = 0;   // Last time stats were printed
#define PERF_REPORT_INTERVAL_MS 5000                // Report every 5 seconds

// Monitor descriptor for ESP32
class ESP32_monitor_desc : public monitor_desc {
public:
    ESP32_monitor_desc(const vector<video_mode> &available_modes, video_depth default_depth, uint32 default_id)
        : monitor_desc(available_modes, default_depth, default_id) {}
    
    virtual void switch_to_current_mode(void);
    virtual void set_palette(uint8 *pal, int num);
    virtual void set_gamma(uint8 *gamma, int num);
};

// Pointer to our monitor
static ESP32_monitor_desc *the_monitor = NULL;

/*
 *  Convert RGB888 to swap565 format for M5GFX writePixels
 *  
 *  M5GFX uses byte-swapped RGB565 (swap565_t):
 *  - Low byte:  RRRRRGGG (R5 in bits 7-3, G high 3 bits in bits 2-0)
 *  - High byte: GGGBBBBB (G low 3 bits in bits 7-5, B5 in bits 4-0)
 */
static inline uint16 rgb888_to_rgb565(uint8 r, uint8 g, uint8 b)
{
    // swap565 format: matches M5GFX's internal swap565() function
    return ((r >> 3) << 3 | (g >> 5)) | (((g >> 2) << 5 | (b >> 3)) << 8);
}

/*
 *  Set palette for indexed color modes
 *  Thread-safe: uses spinlock since palette can be updated from CPU emulation
 *  
 *  When palette changes, we force a full screen update since all pixels
 *  may look different even though the framebuffer data hasn't changed.
 */
void ESP32_monitor_desc::set_palette(uint8 *pal, int num)
{
    D(bug("[VIDEO] set_palette: %d entries\n", num));
    
    portENTER_CRITICAL(&frame_spinlock);
    for (int i = 0; i < num && i < 256; i++) {
        uint8 r = pal[i * 3 + 0];
        uint8 g = pal[i * 3 + 1];
        uint8 b = pal[i * 3 + 2];
        uint16 c = rgb888_to_rgb565(r, g, b);
        palette_rgb565[i] = c;
        palette_rgb565_dup32[i] = ((uint32)c << 16) | c;
    }
    palette_changed = true;
    portEXIT_CRITICAL(&frame_spinlock);
    
    // Force a full screen update since palette affects all pixels
    force_full_update = true;
}

/*
 *  Set gamma table (same as palette for now)
 */
void ESP32_monitor_desc::set_gamma(uint8 *gamma, int num)
{
    // For indexed modes, gamma is applied through palette
    // For direct modes, we ignore gamma on ESP32 for simplicity
    UNUSED(gamma);
    UNUSED(num);
}

/*
 *  Helper to update the video state cache based on depth
 */
static void updateVideoStateCache(video_depth depth, uint32 bytes_per_row)
{
    current_depth = depth;
    current_bytes_per_row = bytes_per_row;
    
    switch (depth) {
        case VDEPTH_1BIT:
            current_pixels_per_byte = 8;
            current_bit_shift = 7;
            current_pixel_mask = 0x01;
            break;
        case VDEPTH_2BIT:
            current_pixels_per_byte = 4;
            current_bit_shift = 6;
            current_pixel_mask = 0x03;
            break;
        case VDEPTH_4BIT:
            current_pixels_per_byte = 2;
            current_bit_shift = 4;
            current_pixel_mask = 0x0F;
            break;
        case VDEPTH_8BIT:
        default:
            current_pixels_per_byte = 1;
            current_bit_shift = 0;
            current_pixel_mask = 0xFF;
            break;
    }
    
    Serial.printf("[VIDEO] Mode cache updated: depth=%d, bpr=%d, ppb=%d\n", 
                  (int)depth, (int)bytes_per_row, current_pixels_per_byte);
}

/*
 *  Initialize palette with default colors for the specified depth
 *  
 *  This sets up appropriate default colors:
 *  - 1-bit: Black and white (standard Mac B&W)
 *  - 2-bit: 4-color grayscale (white, light gray, dark gray, black)
 *  - 4-bit: Classic Mac 16-color palette
 *  - 8-bit: Mac 256-color palette (6x6x6 color cube + grayscale ramp)
 *  
 *  Classic Mac convention: index 0 = white, highest index = black
 */
static void initDefaultPalette(video_depth depth)
{
    portENTER_CRITICAL(&frame_spinlock);
    
    switch (depth) {
        case VDEPTH_1BIT:
            // 1-bit: Black and white
            // Index 0 = white, Index 1 = black
            palette_rgb565[0] = rgb888_to_rgb565(255, 255, 255);  // White
            palette_rgb565[1] = rgb888_to_rgb565(0, 0, 0);        // Black
            Serial.println("[VIDEO] Initialized 1-bit B&W palette");
            break;
            
        case VDEPTH_2BIT:
            // 2-bit: 4 levels of gray
            // Index 0 = white, Index 3 = black
            palette_rgb565[0] = rgb888_to_rgb565(255, 255, 255);  // White
            palette_rgb565[1] = rgb888_to_rgb565(170, 170, 170);  // Light gray
            palette_rgb565[2] = rgb888_to_rgb565(85, 85, 85);     // Dark gray
            palette_rgb565[3] = rgb888_to_rgb565(0, 0, 0);        // Black
            Serial.println("[VIDEO] Initialized 2-bit grayscale palette");
            break;
            
        case VDEPTH_4BIT:
            // 4-bit: Classic Mac 16-color palette
            // This matches the standard Mac 16-color CLUT
            {
                static const uint8 mac16[16][3] = {
                    {255, 255, 255},  // 0: White
                    {255, 255, 0},    // 1: Yellow
                    {255, 102, 0},    // 2: Orange
                    {221, 0, 0},      // 3: Red
                    {255, 0, 153},    // 4: Magenta
                    {51, 0, 153},     // 5: Purple
                    {0, 0, 204},      // 6: Blue
                    {0, 153, 255},    // 7: Cyan
                    {0, 170, 0},      // 8: Green
                    {0, 102, 0},      // 9: Dark Green
                    {102, 51, 0},     // 10: Brown
                    {153, 102, 51},   // 11: Tan
                    {187, 187, 187},  // 12: Light Gray
                    {136, 136, 136},  // 13: Medium Gray
                    {68, 68, 68},     // 14: Dark Gray
                    {0, 0, 0}         // 15: Black
                };
                for (int i = 0; i < 16; i++) {
                    palette_rgb565[i] = rgb888_to_rgb565(mac16[i][0], mac16[i][1], mac16[i][2]);
                }
            }
            Serial.println("[VIDEO] Initialized 4-bit 16-color palette");
            break;
            
        case VDEPTH_8BIT:
        default:
            // 8-bit: Mac 256-color palette
            // Uses a 6x6x6 color cube (216 colors) plus grayscale ramp
            // This provides a good default color palette for 256-color mode
            {
                int idx = 0;
                
                // First, create a 6x6x6 color cube (216 colors)
                // This gives 6 levels each of R, G, B: 0, 51, 102, 153, 204, 255
                for (int r = 0; r < 6; r++) {
                    for (int g = 0; g < 6; g++) {
                        for (int b = 0; b < 6; b++) {
                            uint8 rv = r * 51;
                            uint8 gv = g * 51;
                            uint8 bv = b * 51;
                            palette_rgb565[idx++] = rgb888_to_rgb565(rv, gv, bv);
                        }
                    }
                }
                
                // Fill remaining 40 entries with a grayscale ramp
                // This provides smooth grays for UI elements
                for (int i = 0; i < 40; i++) {
                    uint8 gray = (i * 255) / 39;
                    palette_rgb565[idx++] = rgb888_to_rgb565(gray, gray, gray);
                }
            }
            Serial.println("[VIDEO] Initialized 8-bit 256-color palette");
            break;
    }

    // Build duplicate-pixel palette for 32-bit stores
    for (int i = 0; i < 256; i++) {
        uint16 c = palette_rgb565[i];
        palette_rgb565_dup32[i] = ((uint32)c << 16) | c;
    }
    palette_changed = true;

    portEXIT_CRITICAL(&frame_spinlock);
    
    // Force a full screen update since palette changed
    force_full_update = true;
}

/*
 *  Switch to current video mode
 */
void ESP32_monitor_desc::switch_to_current_mode(void)
{
    const video_mode &mode = get_current_mode();
    D(bug("[VIDEO] switch_to_current_mode: %dx%d, depth=%d, bpr=%d\n", 
          mode.x, mode.y, mode.depth, mode.bytes_per_row));
    
    // Update the video state cache for rendering
    updateVideoStateCache(mode.depth, mode.bytes_per_row);
    
    // Initialize default palette for this depth
    // MacOS will set its own palette shortly after, but this ensures
    // the display looks reasonable immediately after the mode switch
    initDefaultPalette(mode.depth);
    
    // Update frame buffer base address
    set_mac_frame_base(MacFrameBaseMac);
    
    // Force a full screen update on mode change (already done by initDefaultPalette)
    force_full_update = true;
}

// ============================================================================
// Packed pixel decoding helpers for 1/2/4-bit modes
// ============================================================================

/*
 *  Decode a row of packed pixels to 8-bit palette indices
 *  
 *  In packed modes, multiple pixels are stored per byte:
 *  - 1-bit: 8 pixels per byte, MSB first (bit 7 = leftmost pixel)
 *  - 2-bit: 4 pixels per byte, MSB first (bits 7-6 = leftmost pixel)
 *  - 4-bit: 2 pixels per byte, MSB first (bits 7-4 = leftmost pixel)
 *  - 8-bit: 1 pixel per byte (no decoding needed)
 *  
 *  @param src       Source row in frame buffer (packed)
 *  @param dst       Destination buffer for 8-bit indices (must hold width pixels)
 *  @param width     Number of pixels to decode
 *  @param depth     Current video depth
 */
static void decodePackedRow(const uint8 *src, uint8 *dst, int width, video_depth depth)
{
    switch (depth) {
        case VDEPTH_1BIT: {
            // 8 pixels per byte, MSB first
            for (int x = 0; x < width; x++) {
                int byte_idx = x / 8;
                int bit_idx = 7 - (x % 8);  // MSB first
                dst[x] = (src[byte_idx] >> bit_idx) & 0x01;
            }
            break;
        }
        case VDEPTH_2BIT: {
            // 4 pixels per byte, MSB first
            for (int x = 0; x < width; x++) {
                int byte_idx = x / 4;
                int shift = 6 - ((x % 4) * 2);  // MSB first: 6, 4, 2, 0
                dst[x] = (src[byte_idx] >> shift) & 0x03;
            }
            break;
        }
        case VDEPTH_4BIT: {
            // 2 pixels per byte, MSB first
            for (int x = 0; x < width; x++) {
                int byte_idx = x / 2;
                int shift = (x % 2 == 0) ? 4 : 0;  // MSB first: high nibble, low nibble
                dst[x] = (src[byte_idx] >> shift) & 0x0F;
            }
            break;
        }
        case VDEPTH_8BIT:
        default:
            // Direct copy, no decoding needed
            memcpy(dst, src, width);
            break;
    }
}

/*
 *  Get pixel index from packed framebuffer at given (x, y) coordinate
 *  Used for single-pixel access when full row decode is overkill
 *  
 *  @param fb        Frame buffer pointer
 *  @param x         X coordinate (pixel)
 *  @param y         Y coordinate (row)
 *  @param bpr       Bytes per row
 *  @param depth     Current video depth
 *  @return          8-bit palette index for the pixel
 */
static inline uint8 getPackedPixel(const uint8 *fb, int x, int y, uint32 bpr, video_depth depth)
{
    const uint8 *row = fb + y * bpr;
    
    switch (depth) {
        case VDEPTH_1BIT: {
            int byte_idx = x / 8;
            int bit_idx = 7 - (x % 8);
            return (row[byte_idx] >> bit_idx) & 0x01;
        }
        case VDEPTH_2BIT: {
            int byte_idx = x / 4;
            int shift = 6 - ((x % 4) * 2);
            return (row[byte_idx] >> shift) & 0x03;
        }
        case VDEPTH_4BIT: {
            int byte_idx = x / 2;
            int shift = (x % 2 == 0) ? 4 : 0;
            return (row[byte_idx] >> shift) & 0x0F;
        }
        case VDEPTH_8BIT:
        default:
            return row[x];
    }
}

/*
 *  Check if a specific tile is marked as dirty
 */
static inline bool isTileDirty(int tile_idx)
{
    return (dirty_tiles[tile_idx / 32] & (1 << (tile_idx % 32))) != 0;
}

/*
 *  Initialize lookup tables for dirty marking (8-bit fast path)
 */
static void initDirtyLookupTables(void)
{
    for (int x = 0; x < MAC_SCREEN_WIDTH; x++) {
        tile_x_for_byte_in_row[x] = x / TILE_WIDTH;
    }
    for (int y = 0; y < MAC_SCREEN_HEIGHT; y++) {
        tile_y_for_row[y] = y / TILE_HEIGHT;
    }
}

/*
 *  Tile render lock functions - used to prevent race conditions during snapshot
 *  When a tile is being rendered (snapshotted), CPU writes to that tile will
 *  be deferred to the next frame by re-marking the tile dirty.
 */
#if VIDEO_USE_RENDER_LOCK
static inline void setTileRenderActive(int tile_idx)
{
    __atomic_or_fetch(&tile_render_active[tile_idx / 32], (1u << (tile_idx % 32)), __ATOMIC_RELEASE);
}

static inline void clearTileRenderActive(int tile_idx)
{
    __atomic_and_fetch(&tile_render_active[tile_idx / 32], ~(1u << (tile_idx % 32)), __ATOMIC_RELEASE);
}

static inline bool isTileRenderActive(int tile_idx)
{
    return (__atomic_load_n(&tile_render_active[tile_idx / 32], __ATOMIC_ACQUIRE) & (1u << (tile_idx % 32))) != 0;
}
#else
static inline void setTileRenderActive(int)
{
}

static inline void clearTileRenderActive(int)
{
}

static inline bool isTileRenderActive(int)
{
    return false;
}
#endif

/*
 *  Mark a tile as dirty at write-time (called from frame buffer put functions)
 *  This is MUCH faster than per-frame comparison as it only runs on actual writes.
 *  
 *  Handles packed pixel modes by mapping byte offset to pixel coordinates using
 *  current_bytes_per_row and current_pixels_per_byte.
 *  
 *  RACE CONDITION HANDLING:
 *  If the video task is currently rendering (snapshotting) this tile, the snapshot
 *  might contain torn data. However, we unconditionally mark the tile dirty here,
 *  which ensures it will be re-rendered cleanly in the next frame. Combined with
 *  the tile_render_active lock in renderAndPushDirtyTiles(), this provides
 *  eventual consistency - a torn frame may appear briefly but will be fixed
 *  within one frame interval (42ms).
 *  
 *  @param offset  Byte offset into the Mac framebuffer
 */
#ifdef ARDUINO
IRAM_ATTR
#endif
void VideoMarkDirtyOffset(uint32 offset)
{
    if (offset >= frame_buffer_size) return;
    
    // Get current bytes per row (volatile)
    uint32 bpr = current_bytes_per_row;
    int ppb = current_pixels_per_byte;

    // Fast path for 8-bit mode (1 byte == 1 pixel)
    if (current_depth == VDEPTH_8BIT && ppb == 1 && bpr == MAC_SCREEN_WIDTH) {
        uint32 y = offset / bpr;
        if (y >= MAC_SCREEN_HEIGHT) return;
        int byte_in_row = (int)(offset - (y * bpr));
        if (byte_in_row >= MAC_SCREEN_WIDTH) return;

        int tile_x = tile_x_for_byte_in_row[byte_in_row];
        int tile_y = tile_y_for_row[y];
        int tile_idx = tile_y * TILES_X + tile_x;
        if (tile_idx < TOTAL_TILES) {
            cpu_dirty_tiles[tile_idx / 32] |= (1u << (tile_idx % 32));
        }
        return;
    }
    
    // Calculate row from byte offset
    int y = offset / bpr;
    if (y >= MAC_SCREEN_HEIGHT) return;
    
    // Calculate byte position within row
    int byte_in_row = offset % bpr;
    
    // Calculate pixel range that this byte affects
    int pixel_start = byte_in_row * ppb;
    int pixel_end = pixel_start + ppb - 1;
    
    // Clamp to screen width
    if (pixel_start >= MAC_SCREEN_WIDTH) return;
    if (pixel_end >= MAC_SCREEN_WIDTH) pixel_end = MAC_SCREEN_WIDTH - 1;
    
    // Calculate tile range
    int tile_x_start = pixel_start / TILE_WIDTH;
    int tile_x_end = pixel_end / TILE_WIDTH;
    int tile_y = y / TILE_HEIGHT;
    
    // Mark all affected tiles dirty (unconditionally - even if being rendered)
    // This ensures tiles written during rendering are re-rendered next frame
    for (int tile_x = tile_x_start; tile_x <= tile_x_end; tile_x++) {
        int tile_idx = tile_y * TILES_X + tile_x;
        if (tile_idx < TOTAL_TILES) {
            cpu_dirty_tiles[tile_idx / 32] |= (1u << (tile_idx % 32));
        }
    }
}

/*
 *  Mark a range of tiles as dirty at write-time
 *  Used for multi-byte writes (lput, wput)
 *  
 *  For packed pixel modes, a multi-byte write can span many pixels across
 *  potentially multiple rows and tiles.
 *  
 *  See VideoMarkDirtyOffset() for race condition handling notes.
 *  
 *  @param offset  Starting byte offset into the Mac framebuffer
 *  @param size    Number of bytes being written
 */
#ifdef ARDUINO
IRAM_ATTR
#endif
void VideoMarkDirtyRange(uint32 offset, uint32 size)
{
    if (offset >= frame_buffer_size) return;
    
    // Clamp size to framebuffer bounds
    if (offset + size > frame_buffer_size) {
        size = frame_buffer_size - offset;
    }

    if (size == 0) return;
    
    // Get current bytes per row (volatile)
    uint32 bpr = current_bytes_per_row;
    int ppb = current_pixels_per_byte;

    // Fast path for 8-bit mode (1 byte == 1 pixel)
    if (current_depth == VDEPTH_8BIT && ppb == 1 && bpr == MAC_SCREEN_WIDTH) {
        int start_y = offset / bpr;
        int end_y = (offset + size - 1) / bpr;
        if (start_y >= MAC_SCREEN_HEIGHT) return;
        if (end_y >= MAC_SCREEN_HEIGHT) end_y = MAC_SCREEN_HEIGHT - 1;

        int tile_x_start = 0;
        int tile_x_end = TILES_X - 1;

        if (end_y == start_y) {
            int start_byte_in_row = (int)(offset - (start_y * bpr));
            int end_byte_in_row = (int)((offset + size - 1) - (start_y * bpr));
            tile_x_start = tile_x_for_byte_in_row[start_byte_in_row];
            tile_x_end = tile_x_for_byte_in_row[end_byte_in_row];
        }

        int tile_y_start = tile_y_for_row[start_y];
        int tile_y_end = tile_y_for_row[end_y];

        for (int tile_y = tile_y_start; tile_y <= tile_y_end; tile_y++) {
            for (int tile_x = tile_x_start; tile_x <= tile_x_end; tile_x++) {
                int tile_idx = tile_y * TILES_X + tile_x;
                cpu_dirty_tiles[tile_idx / 32] |= (1u << (tile_idx % 32));
            }
        }
        return;
    }
    
    // Calculate start and end rows
    int start_y = offset / bpr;
    int end_y = (offset + size - 1) / bpr;
    
    // For small writes or writes within a single row, just mark individual bytes
    if (end_y == start_y && size <= 4) {
        // Simple case: mark start and end bytes
        VideoMarkDirtyOffset(offset);
        if (size > 1) {
            VideoMarkDirtyOffset(offset + size - 1);
        }
        return;
    }
    
    // For larger writes spanning multiple rows, calculate affected tile columns
    // This is more efficient than marking every byte individually
    int start_byte_in_row = offset % bpr;
    int end_byte_in_row = (offset + size - 1) % bpr;
    
    // Calculate pixel columns affected
    int pixel_col_start = start_byte_in_row * ppb;
    int pixel_col_end = (end_byte_in_row + 1) * ppb - 1;
    
    // For writes spanning multiple rows, the middle rows are fully affected
    // So we need to consider columns from 0 to end for complex cases
    if (end_y > start_y) {
        // Multi-row write: could affect any column
        pixel_col_start = 0;
        pixel_col_end = MAC_SCREEN_WIDTH - 1;
    }
    
    // Calculate tile ranges
    int tile_x_start = pixel_col_start / TILE_WIDTH;
    int tile_x_end = pixel_col_end / TILE_WIDTH;
    if (tile_x_end >= TILES_X) tile_x_end = TILES_X - 1;
    
    int tile_y_start = start_y / TILE_HEIGHT;
    int tile_y_end = end_y / TILE_HEIGHT;
    if (tile_y_end >= TILES_Y) tile_y_end = TILES_Y - 1;
    
    // Mark all affected tiles dirty
    for (int tile_y = tile_y_start; tile_y <= tile_y_end; tile_y++) {
        for (int tile_x = tile_x_start; tile_x <= tile_x_end; tile_x++) {
            int tile_idx = tile_y * TILES_X + tile_x;
            cpu_dirty_tiles[tile_idx / 32] |= (1u << (tile_idx % 32));
        }
    }
}

// ============================================================================
// Write-Through Queue Functions
// ============================================================================
#if VIDEO_USE_WRITE_THROUGH_QUEUE

/*
 *  Calculate which tile(s) an offset belongs to and mark for PSRAM fallback
 *  Called when queue overflows to ensure correct rendering via fallback path
 */
static void markTileNeedsPsramRead(uint32_t offset)
{
    if (offset >= frame_buffer_size) return;
    
    uint32 bpr = current_bytes_per_row;
    int ppb = current_pixels_per_byte;
    
    int y = offset / bpr;
    if (y >= MAC_SCREEN_HEIGHT) return;
    
    int byte_in_row = offset % bpr;
    int pixel_x = byte_in_row * ppb;
    
    if (pixel_x >= MAC_SCREEN_WIDTH) return;
    
    int tile_x = pixel_x / TILE_WIDTH;
    int tile_y = y / TILE_HEIGHT;
    int tile_idx = tile_y * TILES_X + tile_x;
    
    if (tile_idx < TOTAL_TILES) {
        __atomic_or_fetch(&tile_needs_psram_read[tile_idx / 32], (1u << (tile_idx % 32)), __ATOMIC_RELAXED);
    }
}

/*
 *  Check if a tile needs PSRAM fallback (due to queue overflow)
 */
static inline bool tileNeedsPsramRead(int tile_idx)
{
    return (__atomic_load_n(&tile_needs_psram_read[tile_idx / 32], __ATOMIC_ACQUIRE) & (1u << (tile_idx % 32))) != 0;
}

/*
 *  Clear PSRAM fallback flags for all tiles
 *  Called at the start of each frame
 */
static void clearPsramFallbackFlags(void)
{
    for (int i = 0; i < (TOTAL_TILES + 31) / 32; i++) {
        tile_needs_psram_read[i] = 0;
    }
}

/*
 *  Get current queue depth (for debugging)
 */
static inline uint32_t getQueueDepth(void)
{
    uint32_t head = queue_head;
    uint32_t tail = queue_tail;
    if (head >= tail) {
        return head - tail;
    } else {
        return WRITE_QUEUE_SIZE - tail + head;
    }
}

/*
 *  Queue a pixel write for later processing by the video task
 *  This captures the pixel data at write time, eliminating the need to read
 *  from PSRAM during rendering.
 *  
 *  Also updates the tile cache directly if the affected tile is cached.
 *  
 *  Called from memory.cpp frame_direct_*put() functions on CPU core.
 *  Must be thread-safe with respect to video task reading from queue.
 *  
 *  @param offset  Byte offset into the Mac framebuffer
 *  @param data    Pointer to pixel data being written
 *  @param size    Number of bytes (1, 2, or 4)
 */
void VideoQueueWrite(uint32_t offset, const uint8_t* data, uint32_t size)
{
    if (offset >= frame_buffer_size) return;
    if (size == 0 || size > 4) return;

    // Single-producer (CPU core) / single-consumer (video task) lock-free ring
    uint32_t head = __atomic_load_n(&queue_head, __ATOMIC_RELAXED);
    uint32_t next_head = head + 1;
    if (next_head >= WRITE_QUEUE_SIZE) next_head = 0;

    uint32_t tail = __atomic_load_n(&queue_tail, __ATOMIC_ACQUIRE);
    if (next_head == tail) {
        // Queue full - mark affected tile(s) for PSRAM fallback
        markTileNeedsPsramRead(offset);
        dbg_queue_overflows++;
        return;
    }

    // Add entry to queue
    write_queue[head].offset = offset;
    write_queue[head].size = (uint8_t)size;

    // Copy pixel data (1-4 bytes)
    for (uint32_t i = 0; i < size; i++) {
        write_queue[head].data[i] = data[i];
    }

    // Publish entry
    __atomic_store_n(&queue_head, next_head, __ATOMIC_RELEASE);
    
    // Update tile cache with this write (outside spinlock for performance)
    // This directly updates the cached tile content so we don't need PSRAM reads
    applyCacheWrite(offset, data, size);
    
    // Update debug stats (outside critical section)
    dbg_queue_writes++;
    uint32_t depth = getQueueDepth();
    if (depth > dbg_queue_max_depth) {
        dbg_queue_max_depth = depth;
    }
}

/*
 *  Track read-back from video RAM (for debugging)
 *  Called from memory.cpp frame_direct_*get() functions.
 *  Helps identify if Mac software actually reads from video RAM.
 *  
 *  @param offset  Byte offset into the Mac framebuffer
 *  @param size    Number of bytes being read
 */
void VideoTrackReadBack(uint32_t offset, uint32_t size)
{
    (void)offset;  // Currently unused, but could log specific addresses
    (void)size;
    dbg_readback_count++;
}

/*
 *  Get the next entry from the write queue (for video task)
 *  Returns true if an entry was available, false if queue is empty.
 *  
 *  @param entry   Output: the queue entry
 *  @return        true if entry was retrieved, false if queue empty
 */
static bool dequeueWriteEntry(WriteQueueEntry* entry)
{
    uint32_t tail = __atomic_load_n(&queue_tail, __ATOMIC_RELAXED);
    uint32_t head = __atomic_load_n(&queue_head, __ATOMIC_ACQUIRE);

    if (tail == head) {
        // Queue empty
        return false;
    }

    // Copy entry
    *entry = write_queue[tail];

    // Advance tail
    uint32_t next_tail = tail + 1;
    if (next_tail >= WRITE_QUEUE_SIZE) next_tail = 0;
    __atomic_store_n(&queue_tail, next_tail, __ATOMIC_RELEASE);
    
    dbg_queue_entries_processed++;
    return true;
}

/*
 *  Build tile content from the tile cache or fall back to PSRAM
 *  This is the core of the write-through approach: instead of reading from
 *  PSRAM, we use the cached tile content that was updated at write time.
 *  
 *  For tiles that had queue overflow or aren't cached, falls back to PSRAM read.
 *  
 *  @param tile_x      Tile column index
 *  @param tile_y      Tile row index  
 *  @param snapshot    Output buffer (TILE_WIDTH * TILE_HEIGHT bytes)
 *  @param src_buffer  PSRAM frame buffer (for fallback)
 *  @return            true if used cache, false if fell back to PSRAM
 */
static bool buildTileFromCache(int tile_x, int tile_y, uint8* snapshot, uint8* src_buffer)
{
    int tile_idx = tile_y * TILES_X + tile_x;
    
    // Check if tile needs PSRAM fallback (queue overflow)
    if (tileNeedsPsramRead(tile_idx)) {
        snapshotTile(src_buffer, tile_x, tile_y, snapshot);
        dbg_psram_fallbacks++;
        return false;
    }
    
    // Check if tile is in cache
    int cache_slot = getCacheSlot(tile_idx);
    if (cache_slot >= 0) {
        // Copy from cache to snapshot
        memcpy(snapshot, tile_cache[cache_slot].content, TILE_WIDTH * TILE_HEIGHT);
        return true;
    }
    
    // Tile not in cache - allocate a slot and load from PSRAM
    // This happens the first time a tile is accessed, or if it was evicted
    cache_slot = allocateCacheSlot(tile_idx);
    if (cache_slot >= 0) {
        // Load tile content from PSRAM into cache
        snapshotTile(src_buffer, tile_x, tile_y, tile_cache[cache_slot].content);
        // Copy to snapshot
        memcpy(snapshot, tile_cache[cache_slot].content, TILE_WIDTH * TILE_HEIGHT);
    } else {
        // Allocation failed (shouldn't happen with LRU), fall back to PSRAM
        snapshotTile(src_buffer, tile_x, tile_y, snapshot);
        dbg_psram_fallbacks++;
    }
    
    return false;  // Had to load from PSRAM
}

/*
 *  Wrapper function for compatibility - builds tile using cache or PSRAM
 */
static void buildTileFromQueue(int tile_x, int tile_y, uint8* snapshot, uint8* src_buffer)
{
    buildTileFromCache(tile_x, tile_y, snapshot, src_buffer);
}

#else // !VIDEO_USE_WRITE_THROUGH_QUEUE

// Stub implementations when write-through queue is disabled
// These are called from memory.cpp but do nothing when queue is not used

void VideoQueueWrite(uint32_t offset, const uint8_t* data, uint32_t size)
{
    (void)offset;
    (void)data;
    (void)size;
    // No-op when write-through queue is disabled
}

void VideoTrackReadBack(uint32_t offset, uint32_t size)
{
    (void)offset;
    (void)size;
    // No-op when write-through queue is disabled
}

#endif // VIDEO_USE_WRITE_THROUGH_QUEUE

/*
 *  Collect write-dirty tiles into the render dirty bitmap and clear write bitmap
 *  Returns the number of dirty tiles
 *  Called at the start of each video frame
 */
static int collectWriteDirtyTiles(void)
{
    int count = 0;
    
    // Copy write_dirty_tiles to dirty_tiles and count
    for (int i = 0; i < (TOTAL_TILES + 31) / 32; i++) {
        // Atomically read and clear the write dirty bitmap
        uint32 bits = __atomic_exchange_n(&write_dirty_tiles[i], 0, __ATOMIC_RELAXED);
        dirty_tiles[i] = bits;
        
        // Count set bits (fast popcount)
        if (bits) {
            count += __builtin_popcount(bits);
        }
    }
    
    return count;
}

/*
 *  Flush CPU-side dirty bitmap into the shared write_dirty_tiles bitmap
 *  This reduces atomic operations on the hot write path (CPU core).
 *  Called from the CPU core periodically (see basilisk_loop()).
 */
void VideoFlushDirtyTiles(void)
{
    for (int i = 0; i < (TOTAL_TILES + 31) / 32; i++) {
        uint32 bits = cpu_dirty_tiles[i];
        if (bits) {
            __atomic_or_fetch(&write_dirty_tiles[i], bits, __ATOMIC_RELAXED);
            cpu_dirty_tiles[i] = 0;
        }
    }
}

/*
 *  Copy a single tile's source data from framebuffer to a snapshot buffer
 *  This creates a consistent snapshot of the tile to avoid race conditions
 *  when the CPU is writing to the framebuffer while we're rendering.
 *  
 *  For packed pixel modes, decodes to 8-bit indices in the snapshot buffer.
 *  
 *  OPTIMIZATION: For 8-bit mode (most common), uses word-aligned copies and
 *  prefetches next row to improve PSRAM cache utilization.
 *  
 *  @param src_buffer     Mac framebuffer (may be packed or 8-bit)
 *  @param tile_x         Tile column index (0 to TILES_X-1)
 *  @param tile_y         Tile row index (0 to TILES_Y-1)
 *  @param snapshot       Output buffer (TILE_WIDTH * TILE_HEIGHT bytes, always 8-bit indices)
 */
#ifdef ARDUINO
IRAM_ATTR
#endif
static void snapshotTile(uint8 *src_buffer, int tile_x, int tile_y, uint8 *snapshot)
{
    int src_start_x = tile_x * TILE_WIDTH;
    int src_start_y = tile_y * TILE_HEIGHT;
    
    // Get current depth and bytes per row (volatile, so copy locally)
    video_depth depth = current_depth;
    uint32 bpr = current_bytes_per_row;
    
    // Copy and decode each row of the tile to the contiguous snapshot buffer
    uint8 *dst = snapshot;
    
    if (depth == VDEPTH_8BIT) {
        // 8-bit mode: optimized copy using word-aligned transfers
        // TILE_WIDTH (80) is divisible by 4, so we can use 32-bit copies
        uint8 *base_src = src_buffer + src_start_y * bpr + src_start_x;
        
        for (int row = 0; row < TILE_HEIGHT; row++) {
            uint8 *src = base_src + row * bpr;
            
            // Prefetch next row to improve cache utilization
            // ESP32-P4 has 64-byte cache lines, so prefetch helps with PSRAM latency
            if (row + 1 < TILE_HEIGHT) {
                __builtin_prefetch(src + bpr, 0, 0);  // Prefetch next row for read
            }
            
            // Copy using 32-bit words for better PSRAM throughput
            // TILE_WIDTH must be divisible by 4 for this to work correctly
            uint32 *src32 = (uint32 *)src;
            uint32 *dst32 = (uint32 *)dst;
            for (int w = 0; w < TILE_WIDTH / 4; w++) {
                dst32[w] = src32[w];
            }
            
            dst += TILE_WIDTH;
        }
    } else {
        // Packed mode: need to decode pixels
        // For each row, extract the tile's pixel range from the packed source
        for (int row = 0; row < TILE_HEIGHT; row++) {
            uint8 *src_row = src_buffer + (src_start_y + row) * bpr;
            
            // Decode TILE_WIDTH pixels starting at src_start_x
            for (int x = 0; x < TILE_WIDTH; x++) {
                int pixel_x = src_start_x + x;
                
                switch (depth) {
                    case VDEPTH_1BIT: {
                        int byte_idx = pixel_x / 8;
                        int bit_idx = 7 - (pixel_x % 8);
                        *dst++ = (src_row[byte_idx] >> bit_idx) & 0x01;
                        break;
                    }
                    case VDEPTH_2BIT: {
                        int byte_idx = pixel_x / 4;
                        int shift = 6 - ((pixel_x % 4) * 2);
                        *dst++ = (src_row[byte_idx] >> shift) & 0x03;
                        break;
                    }
                    case VDEPTH_4BIT: {
                        int byte_idx = pixel_x / 2;
                        int shift = (pixel_x % 2 == 0) ? 4 : 0;
                        *dst++ = (src_row[byte_idx] >> shift) & 0x0F;
                        break;
                    }
                    default:
                        *dst++ = src_row[pixel_x];
                        break;
                }
            }
        }
    }
}

/*
 *  Render a tile from a contiguous snapshot buffer (not from framebuffer)
 *  This ensures we render from consistent data that won't change mid-render.
 *  
 *  @param snapshot        Tile snapshot buffer (TILE_WIDTH * TILE_HEIGHT bytes, contiguous)
 *  @param local_palette   Pre-copied palette for thread safety
 *  @param out_buffer      Output buffer for RGB565 pixels
 */
#ifdef ARDUINO
IRAM_ATTR
#endif
static void renderTileFromSnapshot(uint8 *snapshot, uint32 *local_palette32, uint16 *out_buffer)
{
    int tile_pixel_width = TILE_WIDTH * PIXEL_SCALE;  // 80 pixels
    
    uint8 *src = snapshot;
    uint16 *out = out_buffer;
    const uint32 *palette = local_palette32;
    
    // Process each row of the Mac tile
    for (int row = 0; row < TILE_HEIGHT; row++) {
        // Output row pointer (row1 is a memcpy of row0 for 2x vertical scaling)
        uint16 *dst_row0 = out;
        uint32 *dst32 = (uint32 *)dst_row0;
        
        // Process 8 pixels at a time for better memory bandwidth
        // TILE_WIDTH=80 is divisible by 8, so no remainder in 8-bit mode
        for (int x = 0; x < TILE_WIDTH; x += 8) {
            uint32 src4a = *((uint32 *)src);
            uint32 src4b = *((uint32 *)(src + 4));
            src += 8;
            
            dst32[0] = palette[src4a & 0xFF];
            dst32[1] = palette[(src4a >> 8) & 0xFF];
            dst32[2] = palette[(src4a >> 16) & 0xFF];
            dst32[3] = palette[(src4a >> 24) & 0xFF];
            dst32[4] = palette[src4b & 0xFF];
            dst32[5] = palette[(src4b >> 8) & 0xFF];
            dst32[6] = palette[(src4b >> 16) & 0xFF];
            dst32[7] = palette[(src4b >> 24) & 0xFF];
            dst32 += 8;
        }

        // Duplicate row 0 into row 1 (2x vertical scaling)
        memcpy(dst_row0 + tile_pixel_width, dst_row0, tile_pixel_width * sizeof(uint16));
        
        // Move output pointer by 2 rows (2x vertical scaling)
        out += tile_pixel_width * 2;
    }
}

/*
 *  Render a tile directly from the live framebuffer (8-bit only)
 *  Skips the snapshot copy to reduce memory traffic.
 *
 *  @param src_buffer      Mac framebuffer (8-bit indexed)
 *  @param bpr             Bytes per row (stride)
 *  @param tile_x          Tile column index
 *  @param tile_y          Tile row index
 *  @param palette         Dup32 palette for 2x horizontal scaling
 *  @param out_buffer      Output buffer for RGB565 pixels
 */
#ifdef ARDUINO
IRAM_ATTR
#endif
static void renderTileFromFramebuffer8(uint8 *src_buffer, uint32 bpr, int tile_x, int tile_y,
                                       const uint32 *palette, uint16 *out_buffer)
{
    int tile_pixel_width = TILE_WIDTH * PIXEL_SCALE;
    uint16 *out = out_buffer;
    uint8 *base_src = src_buffer + (tile_y * TILE_HEIGHT * bpr) + (tile_x * TILE_WIDTH);

    for (int row = 0; row < TILE_HEIGHT; row++) {
        uint8 *src = base_src + row * bpr;
        uint16 *dst_row0 = out;
        uint32 *dst32 = (uint32 *)dst_row0;

        // Prefetch next row to reduce PSRAM latency (safe for internal too)
        if (row + 1 < TILE_HEIGHT) {
            __builtin_prefetch(src + bpr, 0, 0);
        }

        for (int x = 0; x < TILE_WIDTH; x += 8) {
            uint32 src4a = *((uint32 *)src);
            uint32 src4b = *((uint32 *)(src + 4));
            src += 8;

            dst32[0] = palette[src4a & 0xFF];
            dst32[1] = palette[(src4a >> 8) & 0xFF];
            dst32[2] = palette[(src4a >> 16) & 0xFF];
            dst32[3] = palette[(src4a >> 24) & 0xFF];
            dst32[4] = palette[src4b & 0xFF];
            dst32[5] = palette[(src4b >> 8) & 0xFF];
            dst32[6] = palette[(src4b >> 16) & 0xFF];
            dst32[7] = palette[(src4b >> 24) & 0xFF];
            dst32 += 8;
        }

        memcpy(dst_row0 + tile_pixel_width, dst_row0, tile_pixel_width * sizeof(uint16));
        out += tile_pixel_width * 2;
    }
}

/*
 *  Render and push only dirty tiles to the display
 *  RACE-CONDITION FIX: Uses per-tile render lock.
 *  
 *  This prevents visual glitches (especially around the mouse cursor) caused by
 *  the CPU writing to the framebuffer while we're reading it:
 *  1. Set tile_render_active before snapshot
 *  2. If CPU writes during snapshot, tile is re-marked dirty for next frame
 *  3. DMA is used, but we wait for completion before reusing the buffer
 *  
 *  @param src_buffer     Mac framebuffer (8-bit indexed)
 *  @param local_palette32  Pre-copied palette for thread safety (dup32)
 */
static void renderAndPushDirtyTiles(uint8 *src_buffer, uint32 *local_palette32)
{
    // Tile buffers are allocated in PSRAM during VideoInit to preserve internal SRAM
    uint8 *tile_snapshot = tile_snapshot_psram;
    uint16 *tile_buffer = tile_buffer_psram;
    if (!tile_snapshot || !tile_buffer) {
        return;
    }
    
    int tile_pixel_width = TILE_WIDTH * PIXEL_SCALE;
    int tile_pixel_height = TILE_HEIGHT * PIXEL_SCALE;
    int tile_pixels = tile_pixel_width * tile_pixel_height;
    int tiles_rendered = 0;
    bool dma_in_flight = false;
    int buffer_index = 0;
    // Snapshot-free render path for standard 8-bit mode
    video_depth depth = current_depth;
    uint32 bpr = current_bytes_per_row;
    int ppb = current_pixels_per_byte;
    bool direct_render_8bit = (depth == VDEPTH_8BIT && ppb == 1 && bpr == MAC_SCREEN_WIDTH);

    uint16 *tile_buffers[2] = {
        tile_buffer_psram,
        tile_buffer_psram ? (tile_buffer_psram + tile_pixels) : NULL
    };

    M5.Display.startWrite();
    
    for (int ty = 0; ty < TILES_Y; ty++) {
        for (int tx = 0; tx < TILES_X; tx++) {
            int tile_idx = ty * TILES_X + tx;
            
            // Skip tiles that aren't dirty
            if (!isTileDirty(tile_idx)) {
                continue;
            }
            
            // STEP 1: Mark tile as being rendered (prevents CPU from tearing)
            setTileRenderActive(tile_idx);
            
            // STEP 2: Capture tile content (snapshot) or render directly from framebuffer
            // While render_active is set, CPU writes will re-mark tile dirty
#if VIDEO_USE_WRITE_THROUGH_QUEUE
            // Use tile cache if available - avoids PSRAM read
            buildTileFromCache(tx, ty, tile_snapshot, src_buffer);
#else
            if (!direct_render_8bit) {
                // Snapshot path for packed/palette modes
                snapshotTile(src_buffer, tx, ty, tile_snapshot);
            }
#endif

            // STEP 3: Render (direct or from snapshot)
            uint16 *tile_buffer = tile_buffers[buffer_index];
            if (!tile_buffer) {
                clearTileRenderActive(tile_idx);
                continue;
            }
#if VIDEO_USE_WRITE_THROUGH_QUEUE
            renderTileFromSnapshot(tile_snapshot, local_palette32, tile_buffer);
#else
            if (direct_render_8bit) {
                renderTileFromFramebuffer8(src_buffer, bpr, tx, ty, local_palette32, tile_buffer);
            } else {
                renderTileFromSnapshot(tile_snapshot, local_palette32, tile_buffer);
            }
#endif

            // STEP 4: Clear render lock - capture/render complete
            // Any CPU writes after this point will be visible in next frame
            clearTileRenderActive(tile_idx);
            
            // STEP 5: Push to display using async DMA (overlap render with previous DMA)
            int dst_start_x = tx * tile_pixel_width;
            int dst_start_y = ty * tile_pixel_height;
            
            if (dma_in_flight) {
                M5.Display.waitDMA();
                dma_in_flight = false;
            }
            M5.Display.setAddrWindow(dst_start_x, dst_start_y, tile_pixel_width, tile_pixel_height);
            M5.Display.writePixelsDMA(tile_buffer, tile_pixel_width * tile_pixel_height);
            dma_in_flight = true;
            buffer_index ^= 1;
            
            tiles_rendered++;
            
            // Yield every 32 tiles to let other tasks run
            // This prevents starvation during full-screen updates
            // (Reduced from every 8 tiles for better throughput)
            if ((tiles_rendered & 0x1F) == 0) {
                taskYIELD();
            }
        }
    }
    
    if (dma_in_flight) {
        M5.Display.waitDMA();
    }
    M5.Display.endWrite();
}

/*
 *  Render frame buffer directly to display using streaming (no intermediate PSRAM buffer)
 *  
 *  This optimized version eliminates the 1.8MB dsi_framebuffer by:
 *  1. Processing 2 Mac rows at a time (becomes 4 display rows with 2x scaling)
 *  2. Converting 8-bit indexed to RGB565 into internal SRAM row buffer
 *  3. Immediately pushing to display via M5GFX
 *  
 *  PSRAM traffic: ~230KB read (mac_frame_buffer only)
 *  vs old method: ~230KB read + 1.8MB write + 1.8MB read = ~3.8MB
 *  
 *  Supports all bit depths (1/2/4/8-bit) by decoding packed pixels first.
 */
#if VIDEO_USE_STREAMING
static void renderFrameStreaming(uint8 *src_buffer, uint32 *local_palette32)
{
    if (!src_buffer) return;
    
    // Get current depth and bytes per row (volatile, so copy locally)
    video_depth depth = current_depth;
    uint32 bpr = current_bytes_per_row;
    
    // Row decode buffer for packed pixel modes
    // In internal SRAM for fast access during rendering
    DRAM_ATTR static uint8 decoded_row[MAC_SCREEN_WIDTH];
    
    // Track if we have a pending DMA transfer
    bool dma_pending = false;
    int pending_display_y = 0;
    
    M5.Display.startWrite();
    
    // Process 4 Mac rows at a time (produces 8 display rows with 2x scaling)
    // Double-buffering: render to one buffer while DMA pushes the other
    for (int mac_y = 0; mac_y < MAC_SCREEN_HEIGHT; mac_y += 4) {
        uint16 *out = render_buffer;
        
        // Process 4 Mac rows into render_buffer
        for (int row_offset = 0; row_offset < 4; row_offset++) {
            int y = mac_y + row_offset;
            if (y >= MAC_SCREEN_HEIGHT) break;
            
            // Get source row pointer
            uint8 *src_row = src_buffer + y * bpr;
            
            // Decode the row if needed (converts packed pixels to 8-bit indices)
            uint8 *pixel_row;
            if (depth == VDEPTH_8BIT) {
                // 8-bit mode: direct access, no decoding needed
                pixel_row = src_row;
            } else {
                // Packed mode: decode to 8-bit indices
                decodePackedRow(src_row, decoded_row, MAC_SCREEN_WIDTH, depth);
                pixel_row = decoded_row;
            }
            
            // Output row pointers for the two scaled display rows
            uint16 *dst_row0 = out;
            uint32 *dst32 = (uint32 *)dst_row0;
            
            // Process 4 decoded pixels at a time for better memory bandwidth
            int x = 0;
            for (; x < MAC_SCREEN_WIDTH - 3; x += 4) {
                // Read 4 decoded pixels at once (32-bit read from 8-bit indices)
                uint32 src4 = *((uint32 *)(pixel_row + x));
                
                // Convert each pixel through palette and write 2x scaled (32-bit stores)
                dst32[0] = local_palette32[src4 & 0xFF];
                dst32[1] = local_palette32[(src4 >> 8) & 0xFF];
                dst32[2] = local_palette32[(src4 >> 16) & 0xFF];
                dst32[3] = local_palette32[(src4 >> 24) & 0xFF];
                dst32 += 4;
            }
            
            // Handle remaining pixels (if width not divisible by 4)
            for (; x < MAC_SCREEN_WIDTH; x++) {
                *dst32++ = local_palette32[pixel_row[x]];
            }

            // Duplicate row 0 into row 1 (2x vertical scaling)
            memcpy(dst_row0 + DISPLAY_WIDTH, dst_row0, DISPLAY_WIDTH * sizeof(uint16));
            
            // Move output pointer by 2 display rows (2x vertical scaling)
            out += DISPLAY_WIDTH * 2;
        }
        
        // Wait for any pending DMA transfer to complete before swapping buffers
        if (dma_pending) {
            M5.Display.waitDMA();
            dma_pending = false;
        }
        
        // Swap buffers - render_buffer becomes push_buffer for DMA
        uint16 *temp = render_buffer;
        render_buffer = push_buffer;
        push_buffer = temp;
        
        // Start async DMA push of the just-rendered buffer (now in push_buffer)
        // 8 display rows * 1280 pixels = 10240 pixels per chunk
        int display_y = mac_y * PIXEL_SCALE;
        M5.Display.setAddrWindow(0, display_y, DISPLAY_WIDTH, STREAMING_ROW_COUNT);
        M5.Display.writePixelsDMA(push_buffer, DISPLAY_WIDTH * STREAMING_ROW_COUNT);
        dma_pending = true;
        pending_display_y = display_y;
        
        // Yield every 32 Mac rows (8 iterations) to let IDLE task run
        // This prevents watchdog timeout during full-frame renders
        if ((mac_y & 0x1F) == 0) {
            taskYIELD();
        }
    }
    
    // Wait for final DMA transfer to complete
    if (dma_pending) {
        M5.Display.waitDMA();
    }
    
    M5.Display.endWrite();
}
#endif // VIDEO_USE_STREAMING

/*
 *  Stop the video rendering task
 */
static void stopVideoTask(void)
{
    if (video_task_running) {
        video_task_running = false;
        
        // Give task time to exit
        vTaskDelay(pdMS_TO_TICKS(100));
        
        if (video_task_handle) {
            video_task_handle = NULL;
        }
    }
}

/*
 *  Report video performance stats periodically
 */
static void reportVideoPerfStats(void)
{
    uint32_t now = millis();
    if (now - perf_last_report_ms >= PERF_REPORT_INTERVAL_MS) {
        perf_last_report_ms = now;
        
        uint32_t total_frames = perf_full_count + perf_partial_count + perf_skip_count;
        if (total_frames > 0) {
            Serial.printf("[VIDEO PERF] frames=%u (full=%u partial=%u skip=%u)\n",
                          total_frames, perf_full_count, perf_partial_count, perf_skip_count);
            Serial.printf("[VIDEO PERF] avg: detect=%uus render=%uus\n",
                          perf_detect_us / (total_frames > 0 ? total_frames : 1),
                          perf_render_us / (total_frames > 0 ? total_frames : 1));
        }
        
#if VIDEO_USE_WRITE_THROUGH_QUEUE
        // Report write-through queue statistics
        uint32_t queue_depth = getQueueDepth();
        Serial.printf("[VIDEO QUEUE] writes=%u maxdepth=%u overflows=%u fallbacks=%u\n",
                      dbg_queue_writes, dbg_queue_max_depth, dbg_queue_overflows, dbg_psram_fallbacks);
        Serial.printf("[VIDEO QUEUE] cache: hits=%u misses=%u evictions=%u depth=%u/%u\n",
                      dbg_cache_hits, dbg_cache_misses, dbg_cache_evictions, queue_depth, WRITE_QUEUE_SIZE);
        
        // Report read-back count (should be very low for good performance)
        if (dbg_readback_count > 0) {
            Serial.printf("[VIDEO QUEUE] WARNING: readbacks=%u (Mac software reading video RAM)\n",
                          dbg_readback_count);
        }
        
        // Reset queue stats for next interval
        dbg_queue_writes = 0;
        dbg_queue_max_depth = 0;
        dbg_queue_overflows = 0;
        dbg_psram_fallbacks = 0;
        dbg_cache_hits = 0;
        dbg_cache_misses = 0;
        dbg_cache_evictions = 0;
        dbg_readback_count = 0;
        dbg_queue_entries_processed = 0;
#endif
        
        // Reset counters for next interval
        perf_detect_us = 0;
        perf_render_us = 0;
        perf_frame_count = 0;
        perf_partial_count = 0;
        perf_full_count = 0;
        perf_skip_count = 0;
    }
}

/*
 *  Optimized video rendering task - uses WRITE-TIME dirty tracking
 *  
 *  Key optimizations over the old triple-buffer approach:
 *  1. 8-bit tiles render directly from mac_frame_buffer when possible
 *     (packed modes still use a small per-tile snapshot)
 *  2. NO per-frame comparison - dirty tiles are marked at write time by memory.cpp
 *  3. Event-driven with timeout - wakes on notification OR after 67ms max
 *  
 *  This eliminates ~230KB memcpy per frame and expensive tile comparisons.
 *  Dirty tracking overhead is spread across actual CPU writes instead of
 *  being a bulk operation every frame.
 */
static void videoRenderTaskOptimized(void *param)
{
    UNUSED(param);
    Serial.println("[VIDEO] Video render task started on Core 0 (write-time dirty tracking)");
    
    // Reconfigure watchdog to be more lenient for video rendering
    // Video frames can take 50-100ms, so we need a longer timeout
    // Also disable panic so it just logs a warning instead of rebooting
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 10000,      // 10 second timeout (very generous)
        .idle_core_mask = 0,      // Don't monitor IDLE tasks (they get starved by video)
        .trigger_panic = false    // Don't reboot on timeout, just warn
    };
    esp_task_wdt_reconfigure(&wdt_config);
    Serial.println("[VIDEO] Watchdog reconfigured: 10s timeout, no panic, IDLE not monitored");
    
    // Wait a moment for everything to initialize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Local palette copy for thread safety (dup32 for 32-bit stores)
    uint32 local_palette32[256];
    
    // Initialize perf reporting timer
    perf_last_report_ms = millis();
    
    // Minimum frame interval (25ms = ~40 FPS)
    // Reduced from 42ms (24 FPS) for smoother UI responsiveness
    const TickType_t min_frame_ticks = pdMS_TO_TICKS(25);
    TickType_t last_frame_ticks = xTaskGetTickCount();
    
    while (video_task_running) {
        // Note: Watchdog is configured with 10s timeout and no panic,
        // so we don't need to reset it frequently
        
        // Event-driven: wait for frame signal with timeout
        // This replaces the old polling loop - task sleeps until signaled
        // Max wait time ensures we still render periodically even if no signal
        uint32_t notification = ulTaskNotifyTake(pdTRUE, min_frame_ticks);
        
        // Also check legacy frame_ready flag for compatibility
        bool should_render = (notification > 0) || frame_ready;
        frame_ready = false;
        
        // Rate limit: ensure minimum time between frames
        TickType_t now = xTaskGetTickCount();
        TickType_t elapsed = now - last_frame_ticks;
        if (should_render && elapsed < min_frame_ticks) {
            // Too soon - skip this frame signal, we'll render on next timeout
            continue;
        }
        
        // Only render if we have something to render
        if (!should_render && !force_full_update) {
            // Timeout with nothing to do - check for write-dirty tiles anyway
            // This handles cases where writes happened but no explicit signal
        }
        
        uint32_t t0, t1;
        
        // Take a snapshot of the palette only if it changed (thread-safe)
        // This avoids 512-byte memcpy and spinlock contention on every frame
        if (palette_changed) {
            portENTER_CRITICAL(&frame_spinlock);
            memcpy(local_palette32, palette_rgb565_dup32, 256 * sizeof(uint32));
            palette_changed = false;
            portEXIT_CRITICAL(&frame_spinlock);
        }
        
        // Collect dirty tiles from write-time tracking
        t0 = micros();
        dirty_tile_count = collectWriteDirtyTiles();
        
#if VIDEO_USE_WRITE_THROUGH_QUEUE
        // Clear PSRAM fallback flags for this frame
        // Tiles that overflowed the queue last frame will be marked again if needed
        clearPsramFallbackFlags();
#endif
        
        t1 = micros();
        perf_detect_us += (t1 - t0);
        
        // If force_full_update is set (palette change, first frame), mark ALL tiles dirty
        // This ensures we always use tile mode (faster than streaming mode)
        if (force_full_update) {
            // Mark all tiles as dirty
            for (int i = 0; i < (TOTAL_TILES + 31) / 32; i++) {
                dirty_tiles[i] = 0xFFFFFFFF;
            }
            dirty_tile_count = TOTAL_TILES;
            force_full_update = false;
            perf_full_count++;
        }
        
        // RENDER - always use tile mode (faster than streaming even for full screen)
        if (dirty_tile_count > 0) {
            // Render and push only dirty tiles
            t0 = micros();
            renderAndPushDirtyTiles(mac_frame_buffer, local_palette32);
            t1 = micros();
            perf_render_us += (t1 - t0);
            
            perf_partial_count++;
        } else {
            // No tiles dirty, nothing to do!
            perf_skip_count++;
        }
        
        perf_frame_count++;
        last_frame_ticks = now;
        
        // Report performance stats periodically
        reportVideoPerfStats();
    }
    
    Serial.println("[VIDEO] Video render task exiting");
    vTaskDelete(NULL);
}

/*
 *  Initialize video driver
 */
bool VideoInit(bool classic)
{
    Serial.println("[VIDEO] VideoInit starting...");
    
    UNUSED(classic);
    
    // Get display dimensions
    display_width = M5.Display.width();
    display_height = M5.Display.height();
    Serial.printf("[VIDEO] Display size: %dx%d\n", display_width, display_height);
    
    // Verify display size matches our expectations
    if (display_width != DISPLAY_WIDTH || display_height != DISPLAY_HEIGHT) {
        Serial.printf("[VIDEO] WARNING: Expected %dx%d display, got %dx%d\n", 
                      DISPLAY_WIDTH, DISPLAY_HEIGHT, display_width, display_height);
    }
    
    // Allocate Mac frame buffer (prefer internal SRAM for speed, fallback to PSRAM)
    // For 640x360 @ 8-bit = 230,400 bytes
    frame_buffer_size = MAC_SCREEN_WIDTH * MAC_SCREEN_HEIGHT;
    
    mac_frame_buffer = (uint8 *)heap_caps_malloc(frame_buffer_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!mac_frame_buffer) {
        mac_frame_buffer = (uint8 *)ps_malloc(frame_buffer_size);
    }
    if (!mac_frame_buffer) {
        Serial.println("[VIDEO] ERROR: Failed to allocate Mac frame buffer!");
        return false;
    }
    
    Serial.printf("[VIDEO] Mac frame buffer allocated: %p (%d bytes) [%s]\n",
                  mac_frame_buffer, frame_buffer_size,
                  esp_ptr_internal(mac_frame_buffer) ? "INTERNAL" : "PSRAM");
    
    // Clear frame buffer to gray
    memset(mac_frame_buffer, 0x80, frame_buffer_size);

    // Allocate tile render buffers
    // Snapshot is small (3.2KB) so prefer internal SRAM to cut PSRAM readback traffic.
    // Tile buffer prefers internal DMA-capable SRAM, fallback to PSRAM.
    // Use double-buffering to overlap render and DMA.
    size_t snapshot_size = TILE_WIDTH * TILE_HEIGHT;
    size_t tile_buffer_size = TILE_WIDTH * PIXEL_SCALE * TILE_HEIGHT * PIXEL_SCALE * sizeof(uint16);
    tile_snapshot_psram = (uint8 *)heap_caps_malloc(snapshot_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!tile_snapshot_psram) {
        tile_snapshot_psram = (uint8 *)ps_malloc(snapshot_size);
    }
#if VIDEO_USE_STREAMING
    // Keep tile buffer in PSRAM to preserve internal SRAM for streaming row buffers
    tile_buffer_psram = (uint16 *)ps_malloc(tile_buffer_size * 2);
#else
    tile_buffer_psram = (uint16 *)heap_caps_malloc(tile_buffer_size * 2,
                                                   MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!tile_buffer_psram) {
        tile_buffer_psram = (uint16 *)ps_malloc(tile_buffer_size * 2);
    }
#endif
    if (!tile_snapshot_psram || !tile_buffer_psram) {
        Serial.println("[VIDEO] ERROR: Failed to allocate tile buffers!");
        if (tile_snapshot_psram) {
            free(tile_snapshot_psram);
            tile_snapshot_psram = NULL;
        }
        if (tile_buffer_psram) {
            free(tile_buffer_psram);
            tile_buffer_psram = NULL;
        }
        free(mac_frame_buffer);
        mac_frame_buffer = NULL;
        return false;
    }
    Serial.printf("[VIDEO] Tile buffers allocated (snapshot=%u bytes [%s], tile=%u bytes x2 [%s])\n",
                  (unsigned)snapshot_size,
                  esp_ptr_internal(tile_snapshot_psram) ? "INTERNAL" : "PSRAM",
                  (unsigned)tile_buffer_size,
                  esp_ptr_internal(tile_buffer_psram) ? "INTERNAL" : "PSRAM");
    
    // Initialize dirty tracking and render lock
    memset(dirty_tiles, 0, sizeof(dirty_tiles));
    memset(write_dirty_tiles, 0, sizeof(write_dirty_tiles));
    memset(cpu_dirty_tiles, 0, sizeof(cpu_dirty_tiles));
    memset(tile_render_active, 0, sizeof(tile_render_active));
    initDirtyLookupTables();
    force_full_update = true;  // Force full update on first frame
    
#if VIDEO_USE_WRITE_THROUGH_QUEUE
    // Initialize write-through queue and tile cache
    initTileCache();
    memset(tile_needs_psram_read, 0, sizeof(tile_needs_psram_read));
    queue_head = 0;
    queue_tail = 0;
    Serial.println("[VIDEO] Write-through queue ENABLED");
#else
    Serial.println("[VIDEO] Write-through queue DISABLED (using PSRAM read-back)");
#endif
    
    // Clear display to dark gray
    uint16 gray565 = rgb888_to_rgb565(64, 64, 64);
    M5.Display.fillScreen(gray565);
    Serial.println("[VIDEO] Initial screen cleared");
    
    // Set up Mac frame buffer pointers
    MacFrameBaseHost = mac_frame_buffer;
    MacFrameSize = frame_buffer_size;
    MacFrameLayout = FLAYOUT_DIRECT;
    
    // Initialize default palette for 8-bit mode (256 colors)
    // This sets up a proper color palette instead of grayscale,
    // so MacOS will default to "256 colors" instead of "256 grays"
    initDefaultPalette(VDEPTH_8BIT);
    
    // Create video mode vector with all supported depths
    // Per Basilisk II rules: lowest depth must be available in all resolutions,
    // and if a resolution has a depth, it must have all lower depths too.
    // We support 1/2/4/8 bit depths at 640x360.
    vector<video_mode> modes;
    video_mode mode;
    mode.x = MAC_SCREEN_WIDTH;
    mode.y = MAC_SCREEN_HEIGHT;
    mode.resolution_id = 0x80;
    mode.user_data = 0;
    
    // Add 1-bit mode (black and white)
    mode.depth = VDEPTH_1BIT;
    mode.bytes_per_row = TrivialBytesPerRow(MAC_SCREEN_WIDTH, VDEPTH_1BIT);  // 80 bytes
    modes.push_back(mode);
    Serial.printf("[VIDEO] Added mode: 1-bit, %d bytes/row\n", mode.bytes_per_row);
    
    // Add 2-bit mode (4 colors)
    mode.depth = VDEPTH_2BIT;
    mode.bytes_per_row = TrivialBytesPerRow(MAC_SCREEN_WIDTH, VDEPTH_2BIT);  // 160 bytes
    modes.push_back(mode);
    Serial.printf("[VIDEO] Added mode: 2-bit, %d bytes/row\n", mode.bytes_per_row);
    
    // Add 4-bit mode (16 colors)
    mode.depth = VDEPTH_4BIT;
    mode.bytes_per_row = TrivialBytesPerRow(MAC_SCREEN_WIDTH, VDEPTH_4BIT);  // 320 bytes
    modes.push_back(mode);
    Serial.printf("[VIDEO] Added mode: 4-bit, %d bytes/row\n", mode.bytes_per_row);
    
    // Add 8-bit mode (256 colors) - this is our default
    mode.depth = VDEPTH_8BIT;
    mode.bytes_per_row = TrivialBytesPerRow(MAC_SCREEN_WIDTH, VDEPTH_8BIT);  // 640 bytes
    modes.push_back(mode);
    Serial.printf("[VIDEO] Added mode: 8-bit, %d bytes/row\n", mode.bytes_per_row);
    
    // Store current mode info (8-bit default)
    current_mode = mode;
    
    // Initialize the video state cache for 8-bit mode
    updateVideoStateCache(VDEPTH_8BIT, mode.bytes_per_row);
    
    // Create monitor descriptor with 8-bit as default depth
    the_monitor = new ESP32_monitor_desc(modes, VDEPTH_8BIT, 0x80);
    VideoMonitors.push_back(the_monitor);
    
    // Set Mac frame buffer base address
    the_monitor->set_mac_frame_base(MacFrameBaseMac);
    
    // Start video rendering task on Core 0
    // Use the optimized version that does render + push
    video_task_running = true;
    BaseType_t result = xTaskCreatePinnedToCore(
        videoRenderTaskOptimized,
        "VideoTask",
        VIDEO_TASK_STACK_SIZE,
        NULL,
        VIDEO_TASK_PRIORITY,
        &video_task_handle,
        VIDEO_TASK_CORE
    );
    
    if (result != pdPASS) {
        Serial.println("[VIDEO] ERROR: Failed to start video task!");
        // Continue anyway - will fall back to synchronous refresh
    } else {
        Serial.printf("[VIDEO] Video task created on Core %d\n", VIDEO_TASK_CORE);
    }
    
    Serial.printf("[VIDEO] Mac frame base: 0x%08X\n", MacFrameBaseMac);
    Serial.printf("[VIDEO] Dirty tracking: %dx%d tiles (%d total), threshold %d%%\n", 
                  TILES_X, TILES_Y, TOTAL_TILES, DIRTY_THRESHOLD_PERCENT);
    Serial.println("[VIDEO] VideoInit complete (with dirty tile tracking)");
    
    return true;
}

/*
 *  Deinitialize video driver
 */
void VideoExit(void)
{
    Serial.println("[VIDEO] VideoExit");
    
    // Stop video task first
    stopVideoTask();
    
    // Clear dirty tracking and render lock (safety for potential re-init)
    memset(dirty_tiles, 0, sizeof(dirty_tiles));
    memset(write_dirty_tiles, 0, sizeof(write_dirty_tiles));
    memset(tile_render_active, 0, sizeof(tile_render_active));
    
    if (mac_frame_buffer) {
        free(mac_frame_buffer);
        mac_frame_buffer = NULL;
    }

    if (tile_snapshot_psram) {
        free(tile_snapshot_psram);
        tile_snapshot_psram = NULL;
    }
    if (tile_buffer_psram) {
        free(tile_buffer_psram);
        tile_buffer_psram = NULL;
    }
    
    // Clear monitors vector
    VideoMonitors.clear();
    
    if (the_monitor) {
        delete the_monitor;
        the_monitor = NULL;
    }
}

/*
 *  Signal that a new frame is ready for display
 *  Called from CPU emulation (Core 1) to notify video task (Core 0)
 *  This is non-blocking - CPU emulation continues immediately
 *  
 *  Uses FreeRTOS task notification for event-driven wake-up.
 *  The video task sleeps until notified, saving CPU cycles.
 */
void VideoSignalFrameReady(void)
{
    // Set legacy flag for compatibility
    frame_ready = true;
    
    // Send task notification to wake up video task immediately
    // This is more efficient than polling - video task sleeps until notified
    if (video_task_handle != NULL) {
        xTaskNotifyGive(video_task_handle);
    }
}

/*
 *  Video refresh - legacy synchronous function
 *  Now just signals the video task instead of doing the work directly
 *  This allows CPU emulation to continue while video task handles rendering
 */
void VideoRefresh(void)
{
    if (!mac_frame_buffer || !video_task_running) {
        // Fallback: if video task not running, do nothing
        return;
    }
    
    // Signal video task that a new frame is ready
    VideoSignalFrameReady();
}

/*
 *  Set fullscreen mode (no-op on ESP32)
 */
void VideoQuitFullScreen(void)
{
    // No-op
}

/*
 *  Video interrupt handler (60Hz)
 */
void VideoInterrupt(void)
{
    // Trigger ADB interrupt for mouse/keyboard updates
    SetInterruptFlag(INTFLAG_ADB);
}

/*
 *  Get pointer to frame buffer (the buffer that CPU uses)
 */
uint8 *VideoGetFrameBuffer(void)
{
    return mac_frame_buffer;
}

/*
 *  Get frame buffer size
 */
uint32 VideoGetFrameBufferSize(void)
{
    return frame_buffer_size;
}
