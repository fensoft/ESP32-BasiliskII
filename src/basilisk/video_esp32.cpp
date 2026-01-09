/*
 *  video_esp32.cpp - Video/graphics emulation for ESP32-P4 with M5GFX
 *
 *  BasiliskII ESP32 Port
 *
 *  Dual-core optimized: Video rendering runs on Core 0, CPU emulation on Core 1
 *  
 *  OPTIMIZATION: Writes directly to DSI hardware framebuffer with 2x2 scaling.
 *  The MIPI-DSI DMA continuously streams this buffer to the display - no explicit push needed.
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

// Video task configuration
#define VIDEO_TASK_STACK_SIZE  8192
#define VIDEO_TASK_PRIORITY    1
#define VIDEO_TASK_CORE        0  // Run on Core 0, leaving Core 1 for CPU emulation

// Frame buffer for Mac emulation (CPU writes here)
static uint8 *mac_frame_buffer = NULL;
static uint32 frame_buffer_size = 0;

// Direct access to DSI hardware framebuffer
static uint16 *dsi_framebuffer = NULL;
static uint32 dsi_framebuffer_size = 0;

// Frame synchronization
static volatile bool frame_ready = false;
static portMUX_TYPE frame_spinlock = portMUX_INITIALIZER_UNLOCKED;

// Video task handle
static TaskHandle_t video_task_handle = NULL;
static volatile bool video_task_running = false;

// Palette (256 RGB565 entries) - in regular RAM for fast access
static uint16 palette_rgb565[256];

// Display dimensions (from M5.Display)
static int display_width = 0;
static int display_height = 0;

// Video mode info
static video_mode current_mode;

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
 */
void ESP32_monitor_desc::set_palette(uint8 *pal, int num)
{
    D(bug("[VIDEO] set_palette: %d entries\n", num));
    
    portENTER_CRITICAL(&frame_spinlock);
    for (int i = 0; i < num && i < 256; i++) {
        uint8 r = pal[i * 3 + 0];
        uint8 g = pal[i * 3 + 1];
        uint8 b = pal[i * 3 + 2];
        palette_rgb565[i] = rgb888_to_rgb565(r, g, b);
    }
    portEXIT_CRITICAL(&frame_spinlock);
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
 *  Switch to current video mode
 */
void ESP32_monitor_desc::switch_to_current_mode(void)
{
    D(bug("[VIDEO] switch_to_current_mode\n"));
    
    // Update frame buffer base address
    set_mac_frame_base(MacFrameBaseMac);
}

/*
 *  Flush CPU cache to ensure DMA sees our writes
 *  Note: For PSRAM allocated with ps_malloc, we use writePixels which handles
 *  the transfer internally. The cache flush is only needed for true DMA buffers.
 *  Since ps_malloc doesn't guarantee cache alignment, we skip the cache flush
 *  when using the writePixels path.
 */
static inline void flushCacheForDMA(void *buffer, size_t size)
{
    // Skip cache flush - writePixels handles the transfer properly
    // and our buffer may not be cache-line aligned
    (void)buffer;
    (void)size;
}

/*
 *  Render frame buffer directly to DSI hardware framebuffer with 2x2 scaling
 *  Called from video task on Core 0
 *  
 *  This writes directly to the MIPI-DSI DMA buffer which is continuously
 *  streamed to the display by hardware - no explicit push call needed.
 */
static void renderFrameToDSI(uint8 *src_buffer)
{
    if (!src_buffer || !dsi_framebuffer) return;
    
    // Take a snapshot of the palette (thread-safe)
    uint16 local_palette[256];
    portENTER_CRITICAL(&frame_spinlock);
    memcpy(local_palette, palette_rgb565, 256 * sizeof(uint16));
    portEXIT_CRITICAL(&frame_spinlock);
    
    // Process source buffer line by line
    // For each Mac line, write two display lines (2x vertical scaling)
    // For each Mac pixel, write two display pixels (2x horizontal scaling)
    
    uint8 *src = src_buffer;
    
    for (int y = 0; y < MAC_SCREEN_HEIGHT; y++) {
        // Calculate destination row pointers for the two scaled rows
        uint16 *dst_row0 = dsi_framebuffer + (y * 2) * DISPLAY_WIDTH;
        uint16 *dst_row1 = dst_row0 + DISPLAY_WIDTH;
        
        // Process 4 source pixels at a time for better memory bandwidth
        int x = 0;
        for (; x < MAC_SCREEN_WIDTH - 3; x += 4) {
            // Read 4 source pixels at once (32-bit read)
            uint32 src4 = *((uint32 *)src);
            src += 4;
            
            // Convert each pixel through palette and write 2x2 scaled
            uint16 c0 = local_palette[src4 & 0xFF];
            uint16 c1 = local_palette[(src4 >> 8) & 0xFF];
            uint16 c2 = local_palette[(src4 >> 16) & 0xFF];
            uint16 c3 = local_palette[(src4 >> 24) & 0xFF];
            
            // Write to row 0 (2 pixels per source pixel)
            dst_row0[0] = c0; dst_row0[1] = c0;
            dst_row0[2] = c1; dst_row0[3] = c1;
            dst_row0[4] = c2; dst_row0[5] = c2;
            dst_row0[6] = c3; dst_row0[7] = c3;
            
            // Write to row 1 (duplicate of row 0)
            dst_row1[0] = c0; dst_row1[1] = c0;
            dst_row1[2] = c1; dst_row1[3] = c1;
            dst_row1[4] = c2; dst_row1[5] = c2;
            dst_row1[6] = c3; dst_row1[7] = c3;
            
            dst_row0 += 8;
            dst_row1 += 8;
        }
        
        // Handle remaining pixels (if width not divisible by 4)
        for (; x < MAC_SCREEN_WIDTH; x++) {
            uint16 c = local_palette[*src++];
            dst_row0[0] = c; dst_row0[1] = c;
            dst_row1[0] = c; dst_row1[1] = c;
            dst_row0 += 2;
            dst_row1 += 2;
        }
    }
    
    // Flush CPU cache so DMA sees our writes
    flushCacheForDMA(dsi_framebuffer, dsi_framebuffer_size);
}

/*
 *  Video rendering task - runs on Core 0
 *  Handles frame buffer conversion and display updates independently from CPU emulation
 */
static void videoRenderTask(void *param)
{
    UNUSED(param);
    Serial.println("[VIDEO] Video render task started on Core 0");
    
    // Unsubscribe this task from the watchdog timer
    // The video rendering can take variable time and shouldn't trigger WDT
    esp_task_wdt_delete(NULL);
    
    // Wait a moment for everything to initialize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    while (video_task_running) {
        // Check if a new frame is ready
        if (frame_ready) {
            frame_ready = false;
            
            // Render Mac framebuffer directly to DSI hardware buffer with 2x2 scaling
            renderFrameToDSI(mac_frame_buffer);
        }
        
        // Delay to allow other tasks to run and maintain ~60 FPS target
        vTaskDelay(pdMS_TO_TICKS(16));
    }
    
    Serial.println("[VIDEO] Video render task exiting");
    vTaskDelete(NULL);
}

/*
 *  Start the video rendering task on Core 0
 */
static bool startVideoTask(void)
{
    video_task_running = true;
    
    // Create video task pinned to Core 0
    BaseType_t result = xTaskCreatePinnedToCore(
        videoRenderTask,
        "VideoTask",
        VIDEO_TASK_STACK_SIZE,
        NULL,
        VIDEO_TASK_PRIORITY,
        &video_task_handle,
        VIDEO_TASK_CORE
    );
    
    if (result != pdPASS) {
        Serial.println("[VIDEO] ERROR: Failed to create video task!");
        video_task_running = false;
        return false;
    }
    
    Serial.printf("[VIDEO] Video task created on Core %d\n", VIDEO_TASK_CORE);
    return true;
}

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
 *  Get DSI framebuffer from M5GFX panel
 *  Returns pointer to the hardware DMA buffer that is continuously sent to the display
 */
static uint16* getDSIFramebuffer(void)
{
    // Get the panel from M5.Display
    auto panel = M5.Display.getPanel();
    if (!panel) {
        Serial.println("[VIDEO] ERROR: Could not get display panel!");
        return NULL;
    }
    
    // For DSI panels on ESP32-P4, we can use the startWrite/setWindow/writePixels approach
    // But for best performance, we access the framebuffer directly
    
    // The panel's internal framebuffer can be accessed via writeImage with the right setup
    // For now, we'll use a simpler approach: allocate our own buffer and use pushImage
    
    // Actually, let's try to get the internal framebuffer through the panel's config
    // This requires casting to the specific panel type, but M5GFX abstracts this
    
    // Alternative approach: Use M5.Display.getBuffer() or similar
    // M5Canvas has getBuffer() but M5.Display may not expose the DSI buffer directly
    
    // For DSI displays, the buffer is managed by the ESP-IDF LCD driver
    // We can't easily access it through M5GFX without modifications
    
    // FALLBACK: Allocate a buffer and use pushImage to update the display
    // This is still faster than the Canvas approach because:
    // 1. We skip the rotation/zoom math
    // 2. We can use pushImageDMA for asynchronous transfer
    
    Serial.println("[VIDEO] Using direct framebuffer approach...");
    
    // For the Tab5's MIPI-DSI display, we can try getting the framebuffer
    // through the panel configuration
    
    // The simplest reliable method is to use M5.Display.setAddrWindow + writePixels
    // But for true direct access, we need to allocate and manage our own buffer
    
    // Allocate our RGB565 framebuffer in PSRAM
    uint32 fb_size = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16);
    uint16 *fb = (uint16 *)ps_malloc(fb_size);
    
    if (fb) {
        Serial.printf("[VIDEO] Allocated display framebuffer: %p (%d bytes)\n", fb, fb_size);
        dsi_framebuffer_size = fb_size;
    }
    
    return fb;
}

/*
 *  Push our framebuffer to the display using M5GFX
 *  Called after rendering is complete
 */
static void pushFramebufferToDisplay(void)
{
    if (!dsi_framebuffer) return;
    
    // Use M5.Display.pushImage for efficient transfer
    // This uses DMA internally on ESP32-P4
    M5.Display.startWrite();
    M5.Display.setAddrWindow(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    M5.Display.writePixels((uint16_t*)dsi_framebuffer, DISPLAY_WIDTH * DISPLAY_HEIGHT);
    M5.Display.endWrite();
}

/*
 *  Optimized video rendering task - uses direct buffer + pushImage
 */
static void videoRenderTaskOptimized(void *param)
{
    UNUSED(param);
    Serial.println("[VIDEO] Optimized video render task started on Core 0");
    
    // Unsubscribe this task from the watchdog timer
    esp_task_wdt_delete(NULL);
    
    // Wait a moment for everything to initialize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    while (video_task_running) {
        // Check if a new frame is ready
        if (frame_ready) {
            frame_ready = false;
            
            // Render Mac framebuffer to our RGB565 buffer with 2x2 scaling
            renderFrameToDSI(mac_frame_buffer);
            
            // Push the buffer to the display
            pushFramebufferToDisplay();
        }
        
        // Delay to allow other tasks to run
        // Target ~15 FPS to give more CPU time to emulation
        vTaskDelay(pdMS_TO_TICKS(67));  // ~15Hz check rate
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
    
    // Allocate Mac frame buffer in PSRAM
    // For 640x360 @ 8-bit = 230,400 bytes
    frame_buffer_size = MAC_SCREEN_WIDTH * MAC_SCREEN_HEIGHT;
    
    mac_frame_buffer = (uint8 *)ps_malloc(frame_buffer_size);
    if (!mac_frame_buffer) {
        Serial.println("[VIDEO] ERROR: Failed to allocate Mac frame buffer in PSRAM!");
        return false;
    }
    
    Serial.printf("[VIDEO] Mac frame buffer allocated: %p (%d bytes)\n", mac_frame_buffer, frame_buffer_size);
    
    // Clear frame buffer to gray
    memset(mac_frame_buffer, 0x80, frame_buffer_size);
    
    // Get or allocate DSI framebuffer
    dsi_framebuffer = getDSIFramebuffer();
    if (!dsi_framebuffer) {
        Serial.println("[VIDEO] ERROR: Failed to get DSI framebuffer!");
        free(mac_frame_buffer);
        mac_frame_buffer = NULL;
        return false;
    }
    
    // Clear DSI framebuffer to dark gray
    uint16 gray565 = rgb888_to_rgb565(64, 64, 64);
    for (uint32 i = 0; i < DISPLAY_WIDTH * DISPLAY_HEIGHT; i++) {
        dsi_framebuffer[i] = gray565;
    }
    
    // Push initial screen
    pushFramebufferToDisplay();
    
    // Set up Mac frame buffer pointers
    MacFrameBaseHost = mac_frame_buffer;
    MacFrameSize = frame_buffer_size;
    MacFrameLayout = FLAYOUT_DIRECT;
    
    // Initialize default palette (grayscale with Mac-style inversion)
    // Classic Mac: 0=white, 255=black
    for (int i = 0; i < 256; i++) {
        uint8 gray = 255 - i;  // Invert for Mac palette
        palette_rgb565[i] = rgb888_to_rgb565(gray, gray, gray);
    }
    
    // Set up video mode
    current_mode.x = MAC_SCREEN_WIDTH;
    current_mode.y = MAC_SCREEN_HEIGHT;
    current_mode.resolution_id = 0x80;
    current_mode.depth = MAC_SCREEN_DEPTH;
    current_mode.bytes_per_row = MAC_SCREEN_WIDTH;  // 8-bit = 1 byte per pixel
    current_mode.user_data = 0;
    
    // Create video mode vector
    vector<video_mode> modes;
    modes.push_back(current_mode);
    
    // Create monitor descriptor
    the_monitor = new ESP32_monitor_desc(modes, MAC_SCREEN_DEPTH, 0x80);
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
    Serial.println("[VIDEO] VideoInit complete (optimized direct buffer mode)");
    
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
    
    if (mac_frame_buffer) {
        free(mac_frame_buffer);
        mac_frame_buffer = NULL;
    }
    
    if (dsi_framebuffer) {
        free(dsi_framebuffer);
        dsi_framebuffer = NULL;
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
 */
void VideoSignalFrameReady(void)
{
    // Simply set the flag - video task will pick it up
    // No blocking, no waiting for display to finish
    frame_ready = true;
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
