/*
 *  boot_gui.cpp - Pre-boot configuration GUI
 *
 *  BasiliskII ESP32 Port
 *
 *  Classic Macintosh-style boot configuration screen with:
 *  - 3-second countdown to auto-boot
 *  - Hard disk image selection
 *  - CD-ROM ISO selection
 *  - RAM size selection (4/8/12/16 MB)
 *  - WiFi network configuration
 *  - Settings persistence to SD card
 *
 *  Touch handling runs in a dedicated FreeRTOS task for responsiveness.
 */

#include <Arduino.h>
#include <M5Unified.h>
#include <M5GFX.h>
#include <SD.h>
#include <WiFi.h>
#include <vector>
#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "boot_gui.h"

// ============================================================================
// WiFi SDIO Pins for ESP32-C6 Communication
// ============================================================================

#define WIFI_SDIO_CLK  GPIO_NUM_12
#define WIFI_SDIO_CMD  GPIO_NUM_13
#define WIFI_SDIO_D0   GPIO_NUM_11
#define WIFI_SDIO_D1   GPIO_NUM_10
#define WIFI_SDIO_D2   GPIO_NUM_9
#define WIFI_SDIO_D3   GPIO_NUM_8
#define WIFI_SDIO_RST  GPIO_NUM_15

// ============================================================================
// Touch Task Infrastructure
// ============================================================================

// Touch event structure for queue-based input handling
typedef struct {
    int x;
    int y;
    bool is_pressed;
    bool was_pressed;
    bool was_released;
} TouchEvent;

// Touch task handles
static QueueHandle_t touch_queue = nullptr;
static TaskHandle_t touch_task_handle = nullptr;
static volatile bool touch_task_running = false;

#define TOUCH_TASK_STACK_SIZE 4096
#define TOUCH_TASK_PRIORITY   1
#define TOUCH_POLL_INTERVAL_MS 16  // ~60Hz polling

// Track previous touch state for edge detection
static volatile bool touch_prev_pressed = false;
static volatile bool touch_edge_pressed = false;
static volatile bool touch_edge_released = false;
static portMUX_TYPE touch_spinlock = portMUX_INITIALIZER_UNLOCKED;

// Touch task function - runs on Core 0
static void touchTask(void* param)
{
    Serial.println("[BOOT_GUI] Touch task started");
    
    TouchEvent evt;
    bool local_prev_pressed = false;
    
    while (touch_task_running) {
        // Update M5 to poll touch controller
        M5.update();
        
        auto touch = M5.Touch.getDetail();
        bool current_pressed = touch.isPressed();
        
        // Detect edges ourselves (don't rely on M5's edge detection)
        bool just_pressed = current_pressed && !local_prev_pressed;
        bool just_released = !current_pressed && local_prev_pressed;
        
        // Fill event structure
        evt.x = touch.x;
        evt.y = touch.y;
        evt.is_pressed = current_pressed;
        
        // Use spinlock to safely set edge flags
        portENTER_CRITICAL(&touch_spinlock);
        if (just_pressed) {
            touch_edge_pressed = true;
        }
        if (just_released) {
            touch_edge_released = true;
        }
        evt.was_pressed = touch_edge_pressed;
        evt.was_released = touch_edge_released;
        portEXIT_CRITICAL(&touch_spinlock);
        
        // Overwrite queue with latest touch state (non-blocking)
        xQueueOverwrite(touch_queue, &evt);
        
        local_prev_pressed = current_pressed;
        
        vTaskDelay(pdMS_TO_TICKS(TOUCH_POLL_INTERVAL_MS));
    }
    
    Serial.println("[BOOT_GUI] Touch task stopped");
    vTaskDelete(NULL);
}

// Start the touch task
static bool startTouchTask(void)
{
    if (touch_task_running) {
        return true;  // Already running
    }
    
    // Create queue for single touch event (overwrite mode)
    touch_queue = xQueueCreate(1, sizeof(TouchEvent));
    if (!touch_queue) {
        Serial.println("[BOOT_GUI] ERROR: Failed to create touch queue");
        return false;
    }
    
    // Reset edge detection state
    touch_prev_pressed = false;
    touch_edge_pressed = false;
    touch_edge_released = false;
    
    // Initialize queue with empty event
    TouchEvent empty_evt = {0, 0, false, false, false};
    xQueueOverwrite(touch_queue, &empty_evt);
    
    touch_task_running = true;
    
    // Create touch task on Core 0
    BaseType_t result = xTaskCreatePinnedToCore(
        touchTask,
        "BootGUI_Touch",
        TOUCH_TASK_STACK_SIZE,
        NULL,
        TOUCH_TASK_PRIORITY,
        &touch_task_handle,
        0  // Core 0
    );
    
    if (result != pdPASS) {
        Serial.println("[BOOT_GUI] ERROR: Failed to create touch task");
        touch_task_running = false;
        vQueueDelete(touch_queue);
        touch_queue = nullptr;
        return false;
    }
    
    Serial.println("[BOOT_GUI] Touch task created successfully");
    return true;
}

// Stop the touch task
static void stopTouchTask(void)
{
    if (!touch_task_running) {
        return;
    }
    
    Serial.println("[BOOT_GUI] Stopping touch task...");
    
    // Signal task to stop
    touch_task_running = false;
    
    // Wait for task to notice the flag and exit its loop
    // The task has a 16ms delay, so wait a bit longer
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Explicitly delete the task if it's still running
    if (touch_task_handle != nullptr) {
        // Check if task still exists before deleting
        eTaskState state = eTaskGetState(touch_task_handle);
        if (state != eDeleted && state != eInvalid) {
            vTaskDelete(touch_task_handle);
            Serial.println("[BOOT_GUI] Touch task explicitly deleted");
        }
        touch_task_handle = nullptr;
    }
    
    // Now safe to delete the queue since task is definitely gone
    if (touch_queue) {
        vQueueDelete(touch_queue);
        touch_queue = nullptr;
    }
    
    Serial.println("[BOOT_GUI] Touch task cleanup complete");
}

// Get the latest touch event from the queue
// Clears edge flags after reading to ensure edges are only reported once
static bool getTouchEvent(TouchEvent* evt)
{
    if (!touch_queue || !evt) {
        return false;
    }
    
    // Peek at the queue (non-blocking, doesn't remove item)
    if (xQueuePeek(touch_queue, evt, 0) != pdTRUE) {
        return false;
    }
    
    // Clear edge flags after reading so they're only reported once
    portENTER_CRITICAL(&touch_spinlock);
    if (evt->was_pressed) {
        touch_edge_pressed = false;
    }
    if (evt->was_released) {
        touch_edge_released = false;
    }
    portEXIT_CRITICAL(&touch_spinlock);
    
    return true;
}

// ============================================================================
// Classic Mac Color Palette
// ============================================================================

#define MAC_WHITE       0xFFFF
#define MAC_BLACK       0x0000
#define MAC_LIGHT_GRAY  0xC618  // #C0C0C0
#define MAC_DARK_GRAY   0x8410  // #808080
#define MAC_DESKTOP     0xA514  // Classic Mac desktop gray pattern base

// ============================================================================
// UI Layout Constants - Touch-friendly for 5" 1280x720 display
// ============================================================================

// Display dimensions (will be set at runtime)
static int SCREEN_WIDTH = 1280;
static int SCREEN_HEIGHT = 720;

// Full-screen layout with minimal margins
#define SCREEN_MARGIN   20
#define TITLE_BAR_HEIGHT 50
#define CONTENT_PADDING 15

// Button dimensions - large for easy touch
#define BUTTON_HEIGHT   70
#define BUTTON_PADDING  10

// List box dimensions - big items for easy touch
#define LIST_ITEM_HEIGHT 55
#define LIST_MAX_VISIBLE 6

// Radio button dimensions - large touch targets
#define RADIO_SIZE      40
#define RADIO_SPACING   140

// ============================================================================
// Happy Mac Icon (32x32 pixel art)
// ============================================================================

static const uint8_t HAPPY_MAC_ICON[] = {
    0x00, 0x0F, 0xF0, 0x00,
    0x00, 0x3F, 0xFC, 0x00,
    0x00, 0x7F, 0xFE, 0x00,
    0x00, 0xFF, 0xFF, 0x00,
    0x01, 0xFF, 0xFF, 0x80,
    0x03, 0xFF, 0xFF, 0xC0,
    0x07, 0xE0, 0x07, 0xE0,
    0x07, 0xC0, 0x03, 0xE0,
    0x0F, 0x9E, 0x79, 0xF0,
    0x0F, 0x9E, 0x79, 0xF0,
    0x0F, 0x80, 0x01, 0xF0,
    0x0F, 0x80, 0x01, 0xF0,
    0x0F, 0x8C, 0x31, 0xF0,
    0x0F, 0x87, 0xE1, 0xF0,
    0x07, 0xC0, 0x03, 0xE0,
    0x07, 0xE0, 0x07, 0xE0,
    0x03, 0xFF, 0xFF, 0xC0,
    0x01, 0xFF, 0xFF, 0x80,
    0x00, 0xFF, 0xFF, 0x00,
    0x00, 0x7F, 0xFE, 0x00,
    0x00, 0x3F, 0xFC, 0x00,
    0x00, 0x0F, 0xF0, 0x00,
    0x00, 0x07, 0xE0, 0x00,
    0x00, 0x1F, 0xF8, 0x00,
    0x00, 0x3F, 0xFC, 0x00,
    0x00, 0x7F, 0xFE, 0x00,
    0x00, 0x7F, 0xFE, 0x00,
    0x00, 0x7F, 0xFE, 0x00,
    0x00, 0x7F, 0xFE, 0x00,
    0x00, 0x3F, 0xFC, 0x00,
    0x00, 0x1F, 0xF8, 0x00,
    0x00, 0x07, 0xE0, 0x00
};

// ============================================================================
// Settings Storage
// ============================================================================

static char selected_disk_path[BOOT_GUI_MAX_PATH] = "";
static char selected_cdrom_path[BOOT_GUI_MAX_PATH] = "";
static int selected_ram_mb = 8;  // Default 8MB
static bool skip_gui = false;    // If true, skip boot GUI and go straight to emulator

// WiFi settings storage
static char wifi_ssid[64] = "";
static char wifi_password[64] = "";
static bool wifi_auto_connect = false;
static bool wifi_initialized = false;

static const char* SETTINGS_FILE = "/basilisk_settings.txt";

// ============================================================================
// File Lists
// ============================================================================

static std::vector<std::string> disk_files;
static std::vector<std::string> cdrom_files;

static int disk_selection_index = 0;
static int cdrom_selection_index = 0;  // 0 = None
static int disk_scroll_offset = 0;
static int cdrom_scroll_offset = 0;

// ============================================================================
// UI State
// ============================================================================

static bool gui_initialized = false;

// Macro to draw directly to display (no canvas buffer needed)
// This writes directly to the display framebuffer via DMA, which is much faster
// than using a PSRAM canvas and then pushing the whole thing
#define gfx M5.Display

// ============================================================================
// Forward Declarations
// ============================================================================

static void loadSettings(void);
static void saveSettings(void);
static void scanDiskFiles(void);
static void scanCDROMFiles(void);
static void drawDesktopPattern(void);
static void drawWindow(int x, int y, int w, int h, const char* title);
static void drawButton(int x, int y, int w, int h, const char* label, bool pressed);
static void drawListBox(int x, int y, int w, int h, const std::vector<std::string>& items, 
                        int selected, int scroll_offset, bool include_none);
static void drawRadioButton(int x, int y, const char* label, bool selected);
static void drawHappyMac(int x, int y, int scale);
static bool isPointInRect(int px, int py, int rx, int ry, int rw, int rh);
static void runCountdownScreen(void);
static void runSettingsScreen(void);
static void runWiFiScreen(void);
static void initWiFi(void);
static void drawKeyboard(int x, int y, int w, int h, bool shift_active, int highlight_key);
static int getKeyboardKey(int touch_x, int touch_y, int kb_x, int kb_y, int kb_w, int kb_h);
static void drawSignalBars(int x, int y, int rssi);

// ============================================================================
// Settings Load/Save
// ============================================================================

static void loadSettings(void)
{
    Serial.println("[BOOT_GUI] Loading settings...");
    
    File file = SD.open(SETTINGS_FILE, FILE_READ);
    if (!file) {
        Serial.println("[BOOT_GUI] No settings file found, using defaults");
        return;
    }
    
    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        
        int eq_pos = line.indexOf('=');
        if (eq_pos <= 0) {
            continue;
        }
        
        String key = line.substring(0, eq_pos);
        String value = line.substring(eq_pos + 1);
        key.trim();
        value.trim();
        
        if (key == "disk") {
            strncpy(selected_disk_path, value.c_str(), BOOT_GUI_MAX_PATH - 1);
            Serial.printf("[BOOT_GUI] Loaded disk: %s\n", selected_disk_path);
        } else if (key == "cdrom") {
            strncpy(selected_cdrom_path, value.c_str(), BOOT_GUI_MAX_PATH - 1);
            Serial.printf("[BOOT_GUI] Loaded cdrom: %s\n", selected_cdrom_path);
        } else if (key == "ramsize") {
            selected_ram_mb = value.toInt();
            if (selected_ram_mb != 4 && selected_ram_mb != 8 && 
                selected_ram_mb != 12 && selected_ram_mb != 16) {
                selected_ram_mb = 8;  // Default to 8MB if invalid
            }
            Serial.printf("[BOOT_GUI] Loaded RAM: %d MB\n", selected_ram_mb);
        } else if (key == "skip_gui") {
            skip_gui = (value == "yes" || value == "true" || value == "1");
            Serial.printf("[BOOT_GUI] Loaded skip_gui: %s\n", skip_gui ? "yes" : "no");
        } else if (key == "wifi_ssid") {
            strncpy(wifi_ssid, value.c_str(), sizeof(wifi_ssid) - 1);
            Serial.printf("[BOOT_GUI] Loaded WiFi SSID: %s\n", wifi_ssid);
        } else if (key == "wifi_pass") {
            strncpy(wifi_password, value.c_str(), sizeof(wifi_password) - 1);
            Serial.println("[BOOT_GUI] Loaded WiFi password");
        } else if (key == "wifi_auto") {
            wifi_auto_connect = (value == "yes" || value == "true" || value == "1");
            Serial.printf("[BOOT_GUI] Loaded wifi_auto: %s\n", wifi_auto_connect ? "yes" : "no");
        }
    }
    
    file.close();
}

static void saveSettings(void)
{
    Serial.println("[BOOT_GUI] Saving settings...");
    
    File file = SD.open(SETTINGS_FILE, FILE_WRITE);
    if (!file) {
        Serial.println("[BOOT_GUI] ERROR: Cannot open settings file for writing");
        return;
    }
    
    file.printf("disk=%s\n", selected_disk_path);
    file.printf("cdrom=%s\n", selected_cdrom_path);
    file.printf("ramsize=%d\n", selected_ram_mb);
    file.printf("skip_gui=%s\n", skip_gui ? "yes" : "no");
    
    // Save WiFi settings
    file.printf("wifi_ssid=%s\n", wifi_ssid);
    file.printf("wifi_pass=%s\n", wifi_password);
    file.printf("wifi_auto=%s\n", wifi_auto_connect ? "yes" : "no");
    
    file.close();
    Serial.println("[BOOT_GUI] Settings saved");
}

// ============================================================================
// File Scanning
// ============================================================================

static bool hasExtension(const char* filename, const char* ext)
{
    const char* dot = strrchr(filename, '.');
    if (!dot) {
        return false;
    }
    return strcasecmp(dot, ext) == 0;
}

static void scanDiskFiles(void)
{
    Serial.println("[BOOT_GUI] Scanning for disk images...");
    disk_files.clear();
    
    File root = SD.open("/");
    if (!root) {
        Serial.println("[BOOT_GUI] ERROR: Cannot open SD root");
        return;
    }
    
    while (true) {
        File entry = root.openNextFile();
        if (!entry) {
            break;
        }
        
        if (!entry.isDirectory()) {
            const char* name = entry.name();
            // Skip hidden files (starting with '.')
            if (name[0] == '.') {
                entry.close();
                continue;
            }
            if (hasExtension(name, ".dsk") || hasExtension(name, ".img")) {
                // Store with leading slash for full path
                std::string path = "/";
                path += name;
                disk_files.push_back(path);
                Serial.printf("[BOOT_GUI] Found disk: %s\n", path.c_str());
            }
        }
        entry.close();
        
        if (disk_files.size() >= BOOT_GUI_MAX_FILES) {
            break;
        }
    }
    root.close();
    
    Serial.printf("[BOOT_GUI] Found %d disk images\n", disk_files.size());
    
    // Find index of currently selected disk
    disk_selection_index = 0;
    for (size_t i = 0; i < disk_files.size(); i++) {
        if (disk_files[i] == selected_disk_path) {
            disk_selection_index = i;
            break;
        }
    }
}

static void scanCDROMFiles(void)
{
    Serial.println("[BOOT_GUI] Scanning for CD-ROM images...");
    cdrom_files.clear();
    
    File root = SD.open("/");
    if (!root) {
        Serial.println("[BOOT_GUI] ERROR: Cannot open SD root");
        return;
    }
    
    while (true) {
        File entry = root.openNextFile();
        if (!entry) {
            break;
        }
        
        if (!entry.isDirectory()) {
            const char* name = entry.name();
            // Skip hidden files (starting with '.')
            if (name[0] == '.') {
                entry.close();
                continue;
            }
            if (hasExtension(name, ".iso")) {
                std::string path = "/";
                path += name;
                cdrom_files.push_back(path);
                Serial.printf("[BOOT_GUI] Found CD-ROM: %s\n", path.c_str());
            }
        }
        entry.close();
        
        if (cdrom_files.size() >= BOOT_GUI_MAX_FILES) {
            break;
        }
    }
    root.close();
    
    Serial.printf("[BOOT_GUI] Found %d CD-ROM images\n", cdrom_files.size());
    
    // Find index of currently selected CD-ROM (0 = None)
    cdrom_selection_index = 0;
    if (strlen(selected_cdrom_path) > 0) {
        for (size_t i = 0; i < cdrom_files.size(); i++) {
            if (cdrom_files[i] == selected_cdrom_path) {
                cdrom_selection_index = i + 1;  // +1 because 0 is "None"
                break;
            }
        }
    }
}

// ============================================================================
// Drawing Functions - Desktop Pattern
// ============================================================================

static void drawDesktopPattern(void)
{
    // Classic Mac desktop gray pattern
    gfx.fillScreen(MAC_LIGHT_GRAY);
    
    // Draw subtle checkerboard pattern
    for (int y = 0; y < SCREEN_HEIGHT; y += 2) {
        for (int x = 0; x < SCREEN_WIDTH; x += 2) {
            if ((x + y) % 4 == 0) {
                gfx.drawPixel(x, y, MAC_DESKTOP);
            }
        }
    }
}

// ============================================================================
// Drawing Functions - Window
// ============================================================================

static void drawWindow(int x, int y, int w, int h, const char* title)
{
    // Drop shadow
    gfx.fillRect(x + 4, y + 4, w, h, MAC_DARK_GRAY);
    
    // Window background
    gfx.fillRect(x, y, w, h, MAC_WHITE);
    
    // Window border
    gfx.drawRect(x, y, w, h, MAC_BLACK);
    gfx.drawRect(x + 1, y + 1, w - 2, h - 2, MAC_BLACK);
    
    // Title bar background with horizontal stripes
    gfx.fillRect(x + 2, y + 2, w - 4, TITLE_BAR_HEIGHT, MAC_WHITE);
    for (int ty = y + 4; ty < y + TITLE_BAR_HEIGHT; ty += 2) {
        gfx.drawFastHLine(x + 2, ty, w - 4, MAC_BLACK);
    }
    
    // Title text background (white box in center of title bar)
    int title_width = strlen(title) * 12 + 16;
    int title_x = x + (w - title_width) / 2;
    gfx.fillRect(title_x, y + 2, title_width, TITLE_BAR_HEIGHT, MAC_WHITE);
    
    // Title text
    gfx.setTextColor(MAC_BLACK);
    gfx.setTextSize(2);
    gfx.setTextDatum(MC_DATUM);
    gfx.drawString(title, x + w / 2, y + TITLE_BAR_HEIGHT / 2 + 2);
    
    // Divider line below title bar
    gfx.drawFastHLine(x + 2, y + TITLE_BAR_HEIGHT + 2, w - 4, MAC_BLACK);
}

// ============================================================================
// Drawing Functions - Button
// ============================================================================

static void drawButton(int x, int y, int w, int h, const char* label, bool pressed)
{
    if (pressed) {
        // Pressed state - inverted
        gfx.fillRect(x, y, w, h, MAC_BLACK);
        gfx.setTextColor(MAC_WHITE);
    } else {
        // Normal state - 3D beveled
        gfx.fillRect(x, y, w, h, MAC_WHITE);
        
        // Top and left edges (light)
        gfx.drawFastHLine(x, y, w, MAC_WHITE);
        gfx.drawFastVLine(x, y, h, MAC_WHITE);
        
        // Bottom and right edges (dark)
        gfx.drawFastHLine(x, y + h - 1, w, MAC_BLACK);
        gfx.drawFastHLine(x + 1, y + h - 2, w - 2, MAC_DARK_GRAY);
        gfx.drawFastVLine(x + w - 1, y, h, MAC_BLACK);
        gfx.drawFastVLine(x + w - 2, y + 1, h - 2, MAC_DARK_GRAY);
        
        // Border
        gfx.drawRect(x, y, w, h, MAC_BLACK);
        
        gfx.setTextColor(MAC_BLACK);
    }
    
    // Button label - size based on button height
    int text_size = 2;
    if (h >= 60) {
        text_size = 3;
    }
    if (h >= 80) {
        text_size = 4;
    }
    gfx.setTextSize(text_size);
    gfx.setTextDatum(MC_DATUM);
    gfx.drawString(label, x + w / 2, y + h / 2);
}

// ============================================================================
// Drawing Functions - List Box
// ============================================================================

static void drawListBox(int x, int y, int w, int h, const std::vector<std::string>& items,
                        int selected, int scroll_offset, bool include_none)
{
    // Background
    gfx.fillRect(x, y, w, h, MAC_WHITE);
    
    // Thick border for visibility
    gfx.drawRect(x, y, w, h, MAC_BLACK);
    gfx.drawRect(x + 1, y + 1, w - 2, h - 2, MAC_BLACK);
    gfx.drawRect(x + 2, y + 2, w - 4, h - 4, MAC_BLACK);
    
    // Calculate visible items
    int visible_count = (h - 6) / LIST_ITEM_HEIGHT;
    int total_items = items.size();
    if (include_none) {
        total_items++;
    }
    
    // Draw items - larger text for touch screen
    gfx.setTextSize(2);
    gfx.setTextDatum(ML_DATUM);
    
    for (int i = 0; i < visible_count && (i + scroll_offset) < total_items; i++) {
        int item_index = i + scroll_offset;
        int item_y = y + 3 + i * LIST_ITEM_HEIGHT;
        
        const char* item_text;
        if (include_none && item_index == 0) {
            item_text = "(None)";
        } else {
            int file_index = item_index;
            if (include_none) {
                file_index--;
            }
            if (file_index >= 0 && file_index < (int)items.size()) {
                // Show just the filename, not the full path
                const char* path = items[file_index].c_str();
                item_text = path;
                if (path[0] == '/') {
                    item_text = path + 1;  // Skip leading slash
                }
            } else {
                continue;
            }
        }
        
        // Check if this item is selected
        if (item_index == selected) {
            // Selected item - inverted with padding
            gfx.fillRect(x + 3, item_y, w - 6, LIST_ITEM_HEIGHT, MAC_BLACK);
            gfx.setTextColor(MAC_WHITE);
        } else {
            gfx.setTextColor(MAC_BLACK);
        }
        
        // Draw text (truncate if too long)
        char truncated[32];
        strncpy(truncated, item_text, 28);
        truncated[28] = '\0';
        if (strlen(item_text) > 28) {
            strcat(truncated, "...");
        }
        
        gfx.drawString(truncated, x + 6, item_y + LIST_ITEM_HEIGHT / 2);
    }
    
    // Draw scroll indicators if needed
    if (scroll_offset > 0) {
        // Up arrow indicator
        gfx.fillTriangle(x + w - 12, y + 8, x + w - 8, y + 4, x + w - 4, y + 8, MAC_BLACK);
    }
    if (scroll_offset + visible_count < total_items) {
        // Down arrow indicator
        gfx.fillTriangle(x + w - 12, h + y - 8, x + w - 8, h + y - 4, x + w - 4, h + y - 8, MAC_BLACK);
    }
}

// ============================================================================
// Drawing Functions - Radio Button
// ============================================================================

static void drawRadioButton(int x, int y, const char* label, bool selected)
{
    // Large touch-friendly radio button
    int r = RADIO_SIZE / 2;
    int cx = x + r;
    int cy = y + r;
    
    // White background circle
    gfx.fillCircle(cx, cy, r, MAC_WHITE);
    
    // Outer circle border
    gfx.drawCircle(cx, cy, r, MAC_BLACK);
    gfx.drawCircle(cx, cy, r - 1, MAC_BLACK);
    
    // Fill center if selected
    if (selected) {
        gfx.fillCircle(cx, cy, r - 6, MAC_BLACK);
    }
    
    // Label - larger text
    gfx.setTextColor(MAC_BLACK);
    gfx.setTextSize(2);
    gfx.setTextDatum(ML_DATUM);
    gfx.drawString(label, x + RADIO_SIZE + 10, cy);
}

// ============================================================================
// Drawing Functions - Happy Mac Icon
// ============================================================================

static void drawHappyMac(int x, int y, int scale)
{
    for (int row = 0; row < 32; row++) {
        for (int col = 0; col < 32; col++) {
            int byte_index = row * 4 + col / 8;
            int bit_index = 7 - (col % 8);
            
            if (HAPPY_MAC_ICON[byte_index] & (1 << bit_index)) {
                if (scale == 1) {
                    gfx.drawPixel(x + col, y + row, MAC_BLACK);
                } else {
                    gfx.fillRect(x + col * scale, y + row * scale, scale, scale, MAC_BLACK);
                }
            }
        }
    }
}

// ============================================================================
// WiFi Initialization
// ============================================================================

static void initWiFi(void)
{
    if (wifi_initialized) {
        return;
    }
    
    Serial.println("[BOOT_GUI] Initializing WiFi...");
    
    // Set up SDIO pins for ESP32-C6 communication
    // This is required before any WiFi operations on Tab5
    WiFi.setPins(WIFI_SDIO_CLK, WIFI_SDIO_CMD, WIFI_SDIO_D0, 
                 WIFI_SDIO_D1, WIFI_SDIO_D2, WIFI_SDIO_D3, WIFI_SDIO_RST);
    
    // Set WiFi mode to station
    WiFi.mode(WIFI_STA);
    
    // Disconnect from any previous connection
    WiFi.disconnect();
    
    wifi_initialized = true;
    Serial.println("[BOOT_GUI] WiFi initialized");
}

// ============================================================================
// Drawing Functions - Signal Bars
// ============================================================================

static void drawSignalBars(int x, int y, int rssi)
{
    // Convert RSSI to bar count (0-4 bars)
    // Typical RSSI ranges: -30 (excellent) to -90 (unusable)
    int bars = 0;
    if (rssi >= -50) {
        bars = 4;
    } else if (rssi >= -60) {
        bars = 3;
    } else if (rssi >= -70) {
        bars = 2;
    } else if (rssi >= -80) {
        bars = 1;
    }
    
    // Draw 4 bars with increasing height
    int bar_width = 6;
    int bar_gap = 3;
    int max_height = 24;
    
    for (int i = 0; i < 4; i++) {
        int bar_height = (max_height / 4) * (i + 1);
        int bar_x = x + i * (bar_width + bar_gap);
        int bar_y = y + max_height - bar_height;
        
        if (i < bars) {
            // Filled bar
            gfx.fillRect(bar_x, bar_y, bar_width, bar_height, MAC_BLACK);
        } else {
            // Empty bar (outline only)
            gfx.drawRect(bar_x, bar_y, bar_width, bar_height, MAC_DARK_GRAY);
        }
    }
}

// ============================================================================
// On-Screen Keyboard
// ============================================================================

// Keyboard layout - QWERTY with special keys
// Key codes: -1 = none, -2 = shift, -3 = backspace, -4 = enter, -5 = cancel
#define KB_KEY_NONE      -1
#define KB_KEY_SHIFT     -2
#define KB_KEY_BACKSPACE -3
#define KB_KEY_ENTER     -4
#define KB_KEY_CANCEL    -5
#define KB_KEY_SPACE     ' '

// Keyboard rows (lowercase)
static const char* KB_ROW_1 = "1234567890";
static const char* KB_ROW_2 = "qwertyuiop";
static const char* KB_ROW_3 = "asdfghjkl";
static const char* KB_ROW_4 = "zxcvbnm";

// Keyboard rows (uppercase/shifted)
static const char* KB_ROW_1_SHIFT = "!@#$%^&*()";
static const char* KB_ROW_2_SHIFT = "QWERTYUIOP";
static const char* KB_ROW_3_SHIFT = "ASDFGHJKL";
static const char* KB_ROW_4_SHIFT = "ZXCVBNM";

#define KB_ROWS 5
#define KB_KEY_HEIGHT 55
#define KB_KEY_MARGIN 4

static void drawKeyboard(int x, int y, int w, int h, bool shift_active, int highlight_key)
{
    // Calculate key dimensions
    int row_height = KB_KEY_HEIGHT;
    int keys_per_row = 10;
    int key_width = (w - KB_KEY_MARGIN * (keys_per_row + 1)) / keys_per_row;
    
    // Background
    gfx.fillRect(x, y, w, h, MAC_DARK_GRAY);
    gfx.drawRect(x, y, w, h, MAC_BLACK);
    
    // Draw each row
    int current_y = y + KB_KEY_MARGIN;
    int key_index = 0;
    
    const char* rows[] = {
        shift_active ? KB_ROW_1_SHIFT : KB_ROW_1,
        shift_active ? KB_ROW_2_SHIFT : KB_ROW_2,
        shift_active ? KB_ROW_3_SHIFT : KB_ROW_3,
        shift_active ? KB_ROW_4_SHIFT : KB_ROW_4
    };
    
    // Draw letter/number rows
    for (int row = 0; row < 4; row++) {
        const char* row_chars = rows[row];
        int row_len = strlen(row_chars);
        int row_width = row_len * (key_width + KB_KEY_MARGIN) - KB_KEY_MARGIN;
        int start_x = x + (w - row_width) / 2;
        
        for (int col = 0; col < row_len; col++) {
            int key_x = start_x + col * (key_width + KB_KEY_MARGIN);
            bool is_highlighted = (key_index == highlight_key);
            
            // Draw key background
            if (is_highlighted) {
                gfx.fillRect(key_x, current_y, key_width, row_height, MAC_BLACK);
                gfx.setTextColor(MAC_WHITE);
            } else {
                gfx.fillRect(key_x, current_y, key_width, row_height, MAC_WHITE);
                gfx.drawRect(key_x, current_y, key_width, row_height, MAC_BLACK);
                gfx.setTextColor(MAC_BLACK);
            }
            
            // Draw key label
            char label[2] = {row_chars[col], '\0'};
            gfx.setTextSize(2);
            gfx.setTextDatum(MC_DATUM);
            gfx.drawString(label, key_x + key_width / 2, current_y + row_height / 2);
            
            key_index++;
        }
        current_y += row_height + KB_KEY_MARGIN;
    }
    
    // Draw bottom row with special keys: Shift, Space, Backspace, Enter, Cancel
    int bottom_y = current_y;
    int special_key_w = key_width * 2;
    int space_key_w = key_width * 4;
    int total_bottom_w = special_key_w * 2 + space_key_w + key_width * 2 + KB_KEY_MARGIN * 4;
    int bottom_start_x = x + (w - total_bottom_w) / 2;
    int current_x = bottom_start_x;
    
    // Shift key
    bool shift_highlighted = (highlight_key == 100);  // Special index for shift
    if (shift_highlighted || shift_active) {
        gfx.fillRect(current_x, bottom_y, special_key_w, row_height, MAC_BLACK);
        gfx.setTextColor(MAC_WHITE);
    } else {
        gfx.fillRect(current_x, bottom_y, special_key_w, row_height, MAC_WHITE);
        gfx.drawRect(current_x, bottom_y, special_key_w, row_height, MAC_BLACK);
        gfx.setTextColor(MAC_BLACK);
    }
    gfx.setTextSize(2);
    gfx.setTextDatum(MC_DATUM);
    gfx.drawString("Shift", current_x + special_key_w / 2, bottom_y + row_height / 2);
    current_x += special_key_w + KB_KEY_MARGIN;
    
    // Space key
    bool space_highlighted = (highlight_key == 101);
    if (space_highlighted) {
        gfx.fillRect(current_x, bottom_y, space_key_w, row_height, MAC_BLACK);
    } else {
        gfx.fillRect(current_x, bottom_y, space_key_w, row_height, MAC_WHITE);
        gfx.drawRect(current_x, bottom_y, space_key_w, row_height, MAC_BLACK);
    }
    current_x += space_key_w + KB_KEY_MARGIN;
    
    // Backspace key
    bool bksp_highlighted = (highlight_key == 102);
    if (bksp_highlighted) {
        gfx.fillRect(current_x, bottom_y, special_key_w, row_height, MAC_BLACK);
        gfx.setTextColor(MAC_WHITE);
    } else {
        gfx.fillRect(current_x, bottom_y, special_key_w, row_height, MAC_WHITE);
        gfx.drawRect(current_x, bottom_y, special_key_w, row_height, MAC_BLACK);
        gfx.setTextColor(MAC_BLACK);
    }
    gfx.drawString("<--", current_x + special_key_w / 2, bottom_y + row_height / 2);
    current_x += special_key_w + KB_KEY_MARGIN;
    
    // Enter key
    bool enter_highlighted = (highlight_key == 103);
    if (enter_highlighted) {
        gfx.fillRect(current_x, bottom_y, key_width, row_height, MAC_BLACK);
        gfx.setTextColor(MAC_WHITE);
    } else {
        gfx.fillRect(current_x, bottom_y, key_width, row_height, MAC_LIGHT_GRAY);
        gfx.drawRect(current_x, bottom_y, key_width, row_height, MAC_BLACK);
        gfx.setTextColor(MAC_BLACK);
    }
    gfx.drawString("OK", current_x + key_width / 2, bottom_y + row_height / 2);
    current_x += key_width + KB_KEY_MARGIN;
    
    // Cancel key
    bool cancel_highlighted = (highlight_key == 104);
    if (cancel_highlighted) {
        gfx.fillRect(current_x, bottom_y, key_width, row_height, MAC_BLACK);
        gfx.setTextColor(MAC_WHITE);
    } else {
        gfx.fillRect(current_x, bottom_y, key_width, row_height, MAC_WHITE);
        gfx.drawRect(current_x, bottom_y, key_width, row_height, MAC_BLACK);
        gfx.setTextColor(MAC_BLACK);
    }
    gfx.drawString("X", current_x + key_width / 2, bottom_y + row_height / 2);
}

// Get key code from touch position
// Returns character code, or special code (KB_KEY_*)
static int getKeyboardKey(int touch_x, int touch_y, int kb_x, int kb_y, int kb_w, int kb_h)
{
    // Check if touch is within keyboard bounds
    if (!isPointInRect(touch_x, touch_y, kb_x, kb_y, kb_w, kb_h)) {
        return KB_KEY_NONE;
    }
    
    int row_height = KB_KEY_HEIGHT;
    int keys_per_row = 10;
    int key_width = (kb_w - KB_KEY_MARGIN * (keys_per_row + 1)) / keys_per_row;
    
    // Determine which row was touched
    int rel_y = touch_y - kb_y - KB_KEY_MARGIN;
    int row = rel_y / (row_height + KB_KEY_MARGIN);
    
    if (row < 0 || row > 4) {
        return KB_KEY_NONE;
    }
    
    // Handle letter/number rows (0-3)
    if (row < 4) {
        const char* rows[] = {KB_ROW_1, KB_ROW_2, KB_ROW_3, KB_ROW_4};
        const char* row_chars = rows[row];
        int row_len = strlen(row_chars);
        int row_width = row_len * (key_width + KB_KEY_MARGIN) - KB_KEY_MARGIN;
        int start_x = kb_x + (kb_w - row_width) / 2;
        
        int rel_x = touch_x - start_x;
        if (rel_x < 0) {
            return KB_KEY_NONE;
        }
        
        int col = rel_x / (key_width + KB_KEY_MARGIN);
        if (col >= 0 && col < row_len) {
            return row_chars[col];
        }
        return KB_KEY_NONE;
    }
    
    // Handle bottom row with special keys
    int bottom_y = kb_y + KB_KEY_MARGIN + 4 * (row_height + KB_KEY_MARGIN);
    int special_key_w = key_width * 2;
    int space_key_w = key_width * 4;
    int total_bottom_w = special_key_w * 2 + space_key_w + key_width * 2 + KB_KEY_MARGIN * 4;
    int bottom_start_x = kb_x + (kb_w - total_bottom_w) / 2;
    int current_x = bottom_start_x;
    
    // Check Shift
    if (isPointInRect(touch_x, touch_y, current_x, bottom_y, special_key_w, row_height)) {
        return KB_KEY_SHIFT;
    }
    current_x += special_key_w + KB_KEY_MARGIN;
    
    // Check Space
    if (isPointInRect(touch_x, touch_y, current_x, bottom_y, space_key_w, row_height)) {
        return KB_KEY_SPACE;
    }
    current_x += space_key_w + KB_KEY_MARGIN;
    
    // Check Backspace
    if (isPointInRect(touch_x, touch_y, current_x, bottom_y, special_key_w, row_height)) {
        return KB_KEY_BACKSPACE;
    }
    current_x += special_key_w + KB_KEY_MARGIN;
    
    // Check Enter
    if (isPointInRect(touch_x, touch_y, current_x, bottom_y, key_width, row_height)) {
        return KB_KEY_ENTER;
    }
    current_x += key_width + KB_KEY_MARGIN;
    
    // Check Cancel
    if (isPointInRect(touch_x, touch_y, current_x, bottom_y, key_width, row_height)) {
        return KB_KEY_CANCEL;
    }
    
    return KB_KEY_NONE;
}

// Get highlight key index from touch position (for visual feedback)
static int getKeyboardHighlight(int touch_x, int touch_y, int kb_x, int kb_y, int kb_w, int kb_h)
{
    if (!isPointInRect(touch_x, touch_y, kb_x, kb_y, kb_w, kb_h)) {
        return -1;
    }
    
    int row_height = KB_KEY_HEIGHT;
    int keys_per_row = 10;
    int key_width = (kb_w - KB_KEY_MARGIN * (keys_per_row + 1)) / keys_per_row;
    
    int rel_y = touch_y - kb_y - KB_KEY_MARGIN;
    int row = rel_y / (row_height + KB_KEY_MARGIN);
    
    if (row < 0 || row > 4) {
        return -1;
    }
    
    // Letter/number rows
    if (row < 4) {
        int row_lengths[] = {10, 10, 9, 7};
        int row_len = row_lengths[row];
        int row_width = row_len * (key_width + KB_KEY_MARGIN) - KB_KEY_MARGIN;
        int start_x = kb_x + (kb_w - row_width) / 2;
        
        int rel_x = touch_x - start_x;
        if (rel_x < 0) {
            return -1;
        }
        
        int col = rel_x / (key_width + KB_KEY_MARGIN);
        if (col >= 0 && col < row_len) {
            int base_index = 0;
            for (int i = 0; i < row; i++) {
                base_index += row_lengths[i];
            }
            return base_index + col;
        }
        return -1;
    }
    
    // Bottom row special keys
    int bottom_y = kb_y + KB_KEY_MARGIN + 4 * (row_height + KB_KEY_MARGIN);
    int special_key_w = key_width * 2;
    int space_key_w = key_width * 4;
    int total_bottom_w = special_key_w * 2 + space_key_w + key_width * 2 + KB_KEY_MARGIN * 4;
    int bottom_start_x = kb_x + (kb_w - total_bottom_w) / 2;
    int current_x = bottom_start_x;
    
    if (isPointInRect(touch_x, touch_y, current_x, bottom_y, special_key_w, row_height)) {
        return 100;  // Shift
    }
    current_x += special_key_w + KB_KEY_MARGIN;
    
    if (isPointInRect(touch_x, touch_y, current_x, bottom_y, space_key_w, row_height)) {
        return 101;  // Space
    }
    current_x += space_key_w + KB_KEY_MARGIN;
    
    if (isPointInRect(touch_x, touch_y, current_x, bottom_y, special_key_w, row_height)) {
        return 102;  // Backspace
    }
    current_x += special_key_w + KB_KEY_MARGIN;
    
    if (isPointInRect(touch_x, touch_y, current_x, bottom_y, key_width, row_height)) {
        return 103;  // Enter
    }
    current_x += key_width + KB_KEY_MARGIN;
    
    if (isPointInRect(touch_x, touch_y, current_x, bottom_y, key_width, row_height)) {
        return 104;  // Cancel
    }
    
    return -1;
}

// ============================================================================
// Hit Testing
// ============================================================================

static bool isPointInRect(int px, int py, int rx, int ry, int rw, int rh)
{
    return (px >= rx && px < rx + rw && py >= ry && py < ry + rh);
}

// ============================================================================
// Countdown Screen
// ============================================================================

static void runCountdownScreen(void)
{
    Serial.println("[BOOT_GUI] Showing countdown screen...");
    Serial.printf("[BOOT_GUI] Screen size: %dx%d\n", SCREEN_WIDTH, SCREEN_HEIGHT);
    
    int countdown = 3;
    int prev_countdown = -1;
    uint32_t last_second = millis();
    
    // Button dimensions - HUGE and easy to tap, takes up bottom third of screen
    int btn_w = SCREEN_WIDTH - 100;
    int btn_h = 120;
    int btn_x = 50;
    int btn_y = SCREEN_HEIGHT - btn_h - 50;
    
    // Countdown text region (for partial updates)
    int countdown_region_x = SCREEN_WIDTH / 2 - 250;
    int countdown_region_y = SCREEN_HEIGHT / 2 - 110;
    int countdown_region_w = 500;
    int countdown_region_h = 60;
    
    // WiFi status region (for partial updates)
    int wifi_region_x = SCREEN_WIDTH / 2 - 300;
    int wifi_region_y = SCREEN_HEIGHT / 2 + 70;
    int wifi_region_w = 600;
    int wifi_region_h = 30;
    
    Serial.printf("[BOOT_GUI] Button rect: x=%d y=%d w=%d h=%d (bottom edge at %d)\n", 
                  btn_x, btn_y, btn_w, btn_h, btn_y + btn_h);
    
    bool button_pressed = false;
    bool prev_button_pressed = false;
    bool button_touch_started = false;  // Track if touch started in button
    bool settings_requested = false;
    bool first_frame = true;
    
    // WiFi auto-connect state
    bool wifi_connecting = false;
    bool wifi_connected = false;
    bool wifi_failed = false;
    wl_status_t prev_wifi_status = WL_IDLE_STATUS;
    uint32_t wifi_connect_start = 0;
    const uint32_t WIFI_TIMEOUT_MS = 10000;  // 10 second timeout
    
    // Start WiFi auto-connect if configured
    if (wifi_auto_connect && strlen(wifi_ssid) > 0 && strlen(wifi_password) > 0) {
        Serial.printf("[BOOT_GUI] Auto-connecting to WiFi: %s\n", wifi_ssid);
        initWiFi();
        WiFi.begin(wifi_ssid, wifi_password);
        wifi_connecting = true;
        wifi_connect_start = millis();
    }
    
    TouchEvent touch;
    
    while (countdown > 0 && !settings_requested) {
        bool button_changed = false;
        bool countdown_changed = false;
        bool wifi_status_changed = false;
        
        // Check WiFi connection status
        if (wifi_connecting) {
            wl_status_t status = WiFi.status();
            if (status != prev_wifi_status) {
                wifi_status_changed = true;
                prev_wifi_status = status;
            }
            
            if (status == WL_CONNECTED) {
                wifi_connecting = false;
                wifi_connected = true;
                wifi_status_changed = true;
                Serial.printf("[BOOT_GUI] WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
            } else if (status == WL_CONNECT_FAILED || status == WL_NO_SSID_AVAIL) {
                wifi_connecting = false;
                wifi_failed = true;
                wifi_status_changed = true;
                Serial.println("[BOOT_GUI] WiFi connection failed");
            } else if (millis() - wifi_connect_start > WIFI_TIMEOUT_MS) {
                wifi_connecting = false;
                wifi_failed = true;
                wifi_status_changed = true;
                Serial.println("[BOOT_GUI] WiFi connection timeout");
            }
        }
        
        // Get touch input from queue (non-blocking)
        if (getTouchEvent(&touch)) {
            // Check for new touch start
            if (touch.was_pressed) {
                Serial.printf("[BOOT_GUI] Touch START at (%d, %d)\n", touch.x, touch.y);
                bool in_button = isPointInRect(touch.x, touch.y, btn_x, btn_y, btn_w, btn_h);
                Serial.printf("[BOOT_GUI] In button: %s (btn_y=%d to %d)\n", 
                              in_button ? "YES" : "NO", btn_y, btn_y + btn_h);
                
                if (in_button) {
                    button_touch_started = true;
                    button_pressed = true;
                    Serial.println("[BOOT_GUI] Button touch started!");
                }
            }
            
            // Check for touch release
            if (touch.was_released) {
                Serial.println("[BOOT_GUI] Touch RELEASED");
                if (button_touch_started) {
                    // Touch started in button and was released - trigger action
                    settings_requested = true;
                    Serial.println("[BOOT_GUI] Opening settings screen!");
                }
                button_touch_started = false;
                button_pressed = false;
            }
            
            // Update button visual state while held
            if (touch.is_pressed && button_touch_started) {
                button_pressed = isPointInRect(touch.x, touch.y, btn_x, btn_y, btn_w, btn_h);
            }
        }
        
        // Check what changed
        button_changed = (button_pressed != prev_button_pressed);
        countdown_changed = (countdown != prev_countdown);
        
        // Only redraw what changed
        if (first_frame) {
            // First frame - draw everything
            gfx.fillScreen(MAC_LIGHT_GRAY);
            
            // Draw title
            gfx.setTextColor(MAC_BLACK);
            gfx.setTextSize(4);
            gfx.setTextDatum(MC_DATUM);
            gfx.drawString("BasiliskII", SCREEN_WIDTH / 2, 100);
            
            // Draw settings info (static)
            gfx.setTextSize(2);
            if (strlen(selected_disk_path) > 0) {
                const char* disk_name = selected_disk_path;
                if (disk_name[0] == '/') {
                    disk_name++;
                }
                char info[64];
                sprintf(info, "Disk: %s", disk_name);
                gfx.drawString(info, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
                
                sprintf(info, "RAM: %d MB", selected_ram_mb);
                gfx.drawString(info, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 40);
            }
            
            // Draw WiFi status
            if (wifi_connecting) {
                gfx.drawString("WiFi: Connecting...", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 80);
            } else if (wifi_connected) {
                char wifi_info[64];
                sprintf(wifi_info, "WiFi: %s", WiFi.localIP().toString().c_str());
                gfx.drawString(wifi_info, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 80);
            } else if (wifi_failed) {
                gfx.drawString("WiFi: Connection failed", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 80);
            }
            
            // Draw countdown text
            char countdown_text[32];
            sprintf(countdown_text, "Starting in %d...", countdown);
            gfx.setTextSize(4);
            gfx.drawString(countdown_text, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 - 80);
            
            // Draw button
            drawButton(btn_x, btn_y, btn_w, btn_h, "Change Settings", button_pressed);
            first_frame = false;
        } else {
            // Incremental updates - drawing directly to display
            if (countdown_changed) {
                // Clear and redraw countdown region
                gfx.fillRect(countdown_region_x, countdown_region_y, 
                                countdown_region_w, countdown_region_h, MAC_LIGHT_GRAY);
                gfx.setTextColor(MAC_BLACK);
                gfx.setTextSize(4);
                gfx.setTextDatum(MC_DATUM);
                char countdown_text[32];
                sprintf(countdown_text, "Starting in %d...", countdown);
                gfx.drawString(countdown_text, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 - 80);
            }
            
            if (wifi_status_changed) {
                // Clear and redraw WiFi status region
                gfx.fillRect(wifi_region_x, wifi_region_y, wifi_region_w, wifi_region_h, MAC_LIGHT_GRAY);
                gfx.setTextColor(MAC_BLACK);
                gfx.setTextSize(2);
                gfx.setTextDatum(MC_DATUM);
                
                if (wifi_connecting) {
                    gfx.drawString("WiFi: Connecting...", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 80);
                } else if (wifi_connected) {
                    char wifi_info[64];
                    sprintf(wifi_info, "WiFi: %s", WiFi.localIP().toString().c_str());
                    gfx.drawString(wifi_info, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 80);
                } else if (wifi_failed) {
                    gfx.drawString("WiFi: Connection failed", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 80);
                }
                
            }
            
            if (button_changed) {
                // Redraw button
                drawButton(btn_x, btn_y, btn_w, btn_h, "Change Settings", button_pressed);
            }
        }
        
        // Update state tracking
        prev_button_pressed = button_pressed;
        prev_countdown = countdown;
        
        // Update countdown - but pause while WiFi is connecting
        if (millis() - last_second >= 1000) {
            // Only decrement countdown if WiFi is not in the middle of connecting
            // This gives WiFi time to connect before we boot
            if (!wifi_connecting) {
                countdown--;
            }
            last_second = millis();
        }
        
        delay(1);  // Minimal delay - MIPI-DSI is fast, just yield to other tasks
    }
    
    if (settings_requested) {
        runSettingsScreen();
    }
}

// ============================================================================
// Settings Screen
// ============================================================================

static void runSettingsScreen(void)
{
    Serial.println("[BOOT_GUI] Showing settings screen...");
    Serial.printf("[BOOT_GUI] Found %d disk files, %d CD-ROM files\n", 
                  (int)disk_files.size(), (int)cdrom_files.size());
    
    // Full-screen layout - no window, just content areas
    int content_x = SCREEN_MARGIN;
    int content_y = SCREEN_MARGIN + TITLE_BAR_HEIGHT;
    int content_w = SCREEN_WIDTH - SCREEN_MARGIN * 2;
    
    // List box dimensions - side by side, using most of screen width
    int list_gap = 30;
    int list_w = (content_w - list_gap) / 2;
    int list_h = LIST_ITEM_HEIGHT * LIST_MAX_VISIBLE + 4;
    int disk_list_x = content_x;
    int cdrom_list_x = content_x + list_w + list_gap;
    int list_y = content_y + 50;  // Space for labels
    
    // RAM radio buttons - below lists
    int ram_y = list_y + list_h + 30;
    int ram_x = content_x;
    
    // WiFi button - next to RAM radios
    int wifi_btn_w = 150;
    int wifi_btn_h = 60;
    int wifi_btn_x = SCREEN_WIDTH - SCREEN_MARGIN - wifi_btn_w;
    int wifi_btn_y = ram_y;
    
    // Boot button - BIG and at bottom of screen
    int boot_btn_w = 400;
    int boot_btn_h = 80;
    int boot_btn_x = (SCREEN_WIDTH - boot_btn_w) / 2;
    int boot_btn_y = SCREEN_HEIGHT - boot_btn_h - SCREEN_MARGIN;
    
    // RAM radio region (for partial updates)
    int radio_start_x = ram_x + 120;
    int radio_gap = (SCREEN_WIDTH - radio_start_x - SCREEN_MARGIN - wifi_btn_w - 20) / 4;
    int radio_region_x = radio_start_x - 5;
    int radio_region_y = ram_y - 5;
    int radio_region_w = radio_gap * 4 + 20;
    int radio_region_h = RADIO_SIZE + 30;
    
    // Debug: Print layout info
    Serial.printf("[BOOT_GUI] Layout: list_y=%d, list_h=%d, item_height=%d\n", list_y, list_h, LIST_ITEM_HEIGHT);
    
    bool boot_pressed = false;
    bool prev_boot_pressed = false;
    bool boot_touch_started = false;
    bool wifi_pressed = false;
    bool prev_wifi_pressed = false;
    bool wifi_touch_started = false;
    bool should_boot = false;
    bool open_wifi = false;
    bool first_frame = true;
    
    // Track previous state for change detection
    int prev_disk_selection = disk_selection_index;
    int prev_cdrom_selection = cdrom_selection_index;
    int prev_ram_mb = selected_ram_mb;
    
    // Touch state - save position on press for use on release
    int touch_start_x = 0;
    int touch_start_y = 0;
    bool touch_in_disk_list = false;
    bool touch_in_cdrom_list = false;
    bool touch_in_boot_btn = false;
    bool touch_in_wifi_btn = false;
    
    TouchEvent touch;
    
    while (!should_boot && !open_wifi) {
        bool disk_changed = false;
        bool cdrom_changed = false;
        bool ram_changed = false;
        bool boot_btn_changed = false;
        bool wifi_btn_changed = false;
        
        // Get touch input from queue (non-blocking)
        if (getTouchEvent(&touch)) {
            // Detect new touch start - save position
            if (touch.was_pressed) {
                touch_start_x = touch.x;
                touch_start_y = touch.y;
                touch_in_disk_list = isPointInRect(touch_start_x, touch_start_y, disk_list_x, list_y, list_w, list_h);
                touch_in_cdrom_list = isPointInRect(touch_start_x, touch_start_y, cdrom_list_x, list_y, list_w, list_h);
                touch_in_boot_btn = isPointInRect(touch_start_x, touch_start_y, boot_btn_x, boot_btn_y, boot_btn_w, boot_btn_h);
                touch_in_wifi_btn = isPointInRect(touch_start_x, touch_start_y, wifi_btn_x, wifi_btn_y, wifi_btn_w, wifi_btn_h);
                
                if (touch_in_boot_btn) {
                    boot_touch_started = true;
                    boot_pressed = true;
                }
                if (touch_in_wifi_btn) {
                    wifi_touch_started = true;
                    wifi_pressed = true;
                }
                
                Serial.printf("[BOOT_GUI] Touch start at (%d, %d) disk=%d cdrom=%d boot=%d wifi=%d\n", 
                              touch_start_x, touch_start_y, touch_in_disk_list, touch_in_cdrom_list, touch_in_boot_btn, touch_in_wifi_btn);
            }
            
            // Detect touch release - use saved position for hit testing
            if (touch.was_released) {
                Serial.printf("[BOOT_GUI] Touch released, start was (%d, %d)\n", touch_start_x, touch_start_y);
                
                // Check Boot button
                if (boot_touch_started) {
                    should_boot = true;
                    Serial.println("[BOOT_GUI] Boot button pressed");
                }
                
                // Check WiFi button
                if (wifi_touch_started) {
                    open_wifi = true;
                    Serial.println("[BOOT_GUI] WiFi button pressed");
                }
                
                // Check disk list click (use saved start position)
                if (touch_in_disk_list) {
                    int clicked_item = (touch_start_y - list_y - 2) / LIST_ITEM_HEIGHT + disk_scroll_offset;
                    if (clicked_item >= 0 && clicked_item < (int)disk_files.size()) {
                        disk_selection_index = clicked_item;
                        strncpy(selected_disk_path, disk_files[clicked_item].c_str(), BOOT_GUI_MAX_PATH - 1);
                        Serial.printf("[BOOT_GUI] Selected disk [%d]: %s\n", clicked_item, selected_disk_path);
                    }
                }
                
                // Check CD-ROM list click (use saved start position)
                if (touch_in_cdrom_list) {
                    int clicked_item = (touch_start_y - list_y - 2) / LIST_ITEM_HEIGHT + cdrom_scroll_offset;
                    int total_items = cdrom_files.size() + 1;  // +1 for "None"
                    if (clicked_item >= 0 && clicked_item < total_items) {
                        cdrom_selection_index = clicked_item;
                        if (clicked_item == 0) {
                            selected_cdrom_path[0] = '\0';
                        } else {
                            strncpy(selected_cdrom_path, cdrom_files[clicked_item - 1].c_str(), BOOT_GUI_MAX_PATH - 1);
                        }
                    }
                }
                
                // Check RAM radio buttons
                int radio_y_hit = ram_y;
                int radio_hit_w = radio_gap - 10;
                int radio_hit_h = RADIO_SIZE + 20;
                
                if (isPointInRect(touch_start_x, touch_start_y, radio_start_x, radio_y_hit, radio_hit_w, radio_hit_h)) {
                    selected_ram_mb = 4;
                } else if (isPointInRect(touch_start_x, touch_start_y, radio_start_x + radio_gap, radio_y_hit, radio_hit_w, radio_hit_h)) {
                    selected_ram_mb = 8;
                } else if (isPointInRect(touch_start_x, touch_start_y, radio_start_x + radio_gap * 2, radio_y_hit, radio_hit_w, radio_hit_h)) {
                    selected_ram_mb = 12;
                } else if (isPointInRect(touch_start_x, touch_start_y, radio_start_x + radio_gap * 3, radio_y_hit, radio_hit_w, radio_hit_h)) {
                    selected_ram_mb = 16;
                }
                
                // Reset touch state
                touch_in_disk_list = false;
                touch_in_cdrom_list = false;
                touch_in_boot_btn = false;
                touch_in_wifi_btn = false;
                boot_touch_started = false;
                boot_pressed = false;
                wifi_touch_started = false;
                wifi_pressed = false;
            }
            
            // Update button visuals while held
            if (touch.is_pressed) {
                if (boot_touch_started) {
                    boot_pressed = isPointInRect(touch.x, touch.y, boot_btn_x, boot_btn_y, boot_btn_w, boot_btn_h);
                }
                if (wifi_touch_started) {
                    wifi_pressed = isPointInRect(touch.x, touch.y, wifi_btn_x, wifi_btn_y, wifi_btn_w, wifi_btn_h);
                }
            }
        }
        
        // Check what changed
        disk_changed = (disk_selection_index != prev_disk_selection);
        cdrom_changed = (cdrom_selection_index != prev_cdrom_selection);
        ram_changed = (selected_ram_mb != prev_ram_mb);
        boot_btn_changed = (boot_pressed != prev_boot_pressed);
        wifi_btn_changed = (wifi_pressed != prev_wifi_pressed);
        
        if (first_frame) {
            // First frame - draw everything
            gfx.fillScreen(MAC_LIGHT_GRAY);
            
            // Draw title
            gfx.setTextColor(MAC_BLACK);
            gfx.setTextSize(3);
            gfx.setTextDatum(TC_DATUM);
            gfx.drawString("Boot Settings", SCREEN_WIDTH / 2, SCREEN_MARGIN);
            
            // Draw labels
            gfx.setTextSize(2);
            gfx.setTextDatum(TL_DATUM);
            gfx.drawString("Hard Disk:", disk_list_x, content_y);
            gfx.drawString("CD-ROM:", cdrom_list_x, content_y);
            gfx.drawString("Memory:", ram_x, ram_y + 10);
            
            // Draw lists
            drawListBox(disk_list_x, list_y, list_w, list_h, disk_files, 
                        disk_selection_index, disk_scroll_offset, false);
            drawListBox(cdrom_list_x, list_y, list_w, list_h, cdrom_files,
                        cdrom_selection_index, cdrom_scroll_offset, true);
            
            // Draw RAM radio buttons
            drawRadioButton(radio_start_x, ram_y, "4 MB", selected_ram_mb == 4);
            drawRadioButton(radio_start_x + radio_gap, ram_y, "8 MB", selected_ram_mb == 8);
            drawRadioButton(radio_start_x + radio_gap * 2, ram_y, "12 MB", selected_ram_mb == 12);
            drawRadioButton(radio_start_x + radio_gap * 3, ram_y, "16 MB", selected_ram_mb == 16);
            
            // Draw buttons
            drawButton(wifi_btn_x, wifi_btn_y, wifi_btn_w, wifi_btn_h, "WiFi", wifi_pressed);
            drawButton(boot_btn_x, boot_btn_y, boot_btn_w, boot_btn_h, "Boot", boot_pressed);
            first_frame = false;
        } else {
            // Incremental updates - drawing directly to display
            if (disk_changed) {
                drawListBox(disk_list_x, list_y, list_w, list_h, disk_files, 
                            disk_selection_index, disk_scroll_offset, false);
            }
            
            if (cdrom_changed) {
                drawListBox(cdrom_list_x, list_y, list_w, list_h, cdrom_files,
                            cdrom_selection_index, cdrom_scroll_offset, true);
            }
            
            if (ram_changed) {
                // Clear and redraw radio region
                gfx.fillRect(radio_region_x, radio_region_y, radio_region_w, radio_region_h, MAC_LIGHT_GRAY);
                drawRadioButton(radio_start_x, ram_y, "4 MB", selected_ram_mb == 4);
                drawRadioButton(radio_start_x + radio_gap, ram_y, "8 MB", selected_ram_mb == 8);
                drawRadioButton(radio_start_x + radio_gap * 2, ram_y, "12 MB", selected_ram_mb == 12);
                drawRadioButton(radio_start_x + radio_gap * 3, ram_y, "16 MB", selected_ram_mb == 16);
            }
            
            if (boot_btn_changed) {
                drawButton(boot_btn_x, boot_btn_y, boot_btn_w, boot_btn_h, "Boot", boot_pressed);
            }
            
            if (wifi_btn_changed) {
                drawButton(wifi_btn_x, wifi_btn_y, wifi_btn_w, wifi_btn_h, "WiFi", wifi_pressed);
            }
        }
        
        // Update state tracking
        prev_disk_selection = disk_selection_index;
        prev_cdrom_selection = cdrom_selection_index;
        prev_ram_mb = selected_ram_mb;
        prev_boot_pressed = boot_pressed;
        prev_wifi_pressed = wifi_pressed;
        
        delay(1);  // Minimal delay - MIPI-DSI is fast, just yield to other tasks
    }
    
    // Handle WiFi screen navigation
    if (open_wifi) {
        runWiFiScreen();
        // After returning from WiFi screen, continue showing settings
        runSettingsScreen();
        return;
    }
    
    // Save settings before booting
    saveSettings();
}

// ============================================================================
// WiFi Configuration Screen
// ============================================================================

// WiFi network info storage
typedef struct {
    char ssid[33];
    int32_t rssi;
    uint8_t encryption;
} WiFiNetworkInfo;

static std::vector<WiFiNetworkInfo> wifi_networks;
static int wifi_selection_index = -1;
static int wifi_scroll_offset = 0;

static void runWiFiScreen(void)
{
    Serial.println("[BOOT_GUI] Showing WiFi screen...");
    
    // Initialize WiFi if not already done
    initWiFi();
    
    // Layout constants
    int content_x = SCREEN_MARGIN;
    int content_y = SCREEN_MARGIN + TITLE_BAR_HEIGHT;
    int content_w = SCREEN_WIDTH - SCREEN_MARGIN * 2;
    
    // Network list dimensions
    int list_w = content_w;
    int list_h = LIST_ITEM_HEIGHT * 6 + 4;
    int list_x = content_x;
    int list_y = content_y + 50;
    
    // Status area
    int status_y = list_y + list_h + 20;
    
    // Button dimensions
    int btn_w = 180;
    int btn_h = 60;
    int btn_gap = 20;
    
    // Scan button
    int scan_btn_x = content_x;
    int scan_btn_y = SCREEN_HEIGHT - btn_h - SCREEN_MARGIN;
    
    // Connect button
    int connect_btn_x = scan_btn_x + btn_w + btn_gap;
    int connect_btn_y = scan_btn_y;
    
    // Back button
    int back_btn_x = SCREEN_WIDTH - SCREEN_MARGIN - btn_w;
    int back_btn_y = scan_btn_y;
    
    // Password input area
    int password_y = status_y + 60;
    int password_w = 500;
    int password_x = (SCREEN_WIDTH - password_w) / 2;
    int password_h = 50;
    
    // Keyboard dimensions
    int kb_h = KB_KEY_HEIGHT * 5 + KB_KEY_MARGIN * 6;
    int kb_y = SCREEN_HEIGHT - kb_h - 10;
    int kb_x = 50;
    int kb_w = SCREEN_WIDTH - 100;
    
    // State
    bool scanning = false;
    bool connecting = false;
    bool show_keyboard = false;
    bool shift_active = false;
    char password_buffer[64] = "";
    int password_cursor = 0;
    int kb_highlight = -1;
    
    bool scan_pressed = false;
    bool scan_touch_started = false;
    bool connect_pressed = false;
    bool connect_touch_started = false;
    bool back_pressed = false;
    bool back_touch_started = false;
    bool password_touched = false;
    
    bool should_exit = false;
    
    // Touch state
    int touch_start_x = 0;
    int touch_start_y = 0;
    
    TouchEvent touch;
    
    // Copy saved password if we have a saved SSID
    if (strlen(wifi_ssid) > 0 && strlen(wifi_password) > 0) {
        strncpy(password_buffer, wifi_password, sizeof(password_buffer) - 1);
        password_cursor = strlen(password_buffer);
    }
    
    // Initial scan
    Serial.println("[BOOT_GUI] Starting initial WiFi scan...");
    scanning = true;
    int scan_result = WiFi.scanNetworks(true);  // Async scan
    Serial.printf("[BOOT_GUI] Scan initiated, result: %d\n", scan_result);
    
    // State tracking for incremental updates
    bool first_frame = true;
    bool prev_scanning = false;
    bool prev_connecting = false;
    bool prev_show_keyboard = false;
    int prev_wifi_selection = -1;
    int prev_password_len = 0;
    wl_status_t prev_wifi_status = WL_IDLE_STATUS;
    bool prev_scan_pressed = false;
    bool prev_connect_pressed = false;
    bool prev_back_pressed = false;
    int prev_kb_highlight = -1;
    bool prev_shift_active = false;
    size_t prev_network_count = 0;
    
    while (!should_exit) {
        // Check scan completion
        if (scanning) {
            int16_t result = WiFi.scanComplete();
            if (result >= 0) {
                // Scan complete
                Serial.printf("[BOOT_GUI] Scan complete, found %d networks\n", result);
                wifi_networks.clear();
                for (int i = 0; i < result; i++) {
                    WiFiNetworkInfo info;
                    strncpy(info.ssid, WiFi.SSID(i).c_str(), sizeof(info.ssid) - 1);
                    info.ssid[sizeof(info.ssid) - 1] = '\0';
                    info.rssi = WiFi.RSSI(i);
                    info.encryption = WiFi.encryptionType(i);
                    wifi_networks.push_back(info);
                    Serial.printf("[BOOT_GUI]   %s (RSSI: %d)\n", info.ssid, info.rssi);
                }
                WiFi.scanDelete();
                scanning = false;
                
                // Select saved network if found
                if (strlen(wifi_ssid) > 0) {
                    for (size_t i = 0; i < wifi_networks.size(); i++) {
                        if (strcmp(wifi_networks[i].ssid, wifi_ssid) == 0) {
                            wifi_selection_index = i;
                            break;
                        }
                    }
                }
            } else if (result == WIFI_SCAN_FAILED) {
                Serial.println("[BOOT_GUI] Scan failed");
                scanning = false;
            }
            // result == WIFI_SCAN_RUNNING means still scanning
        }
        
        // Check connection status
        if (connecting) {
            wl_status_t status = WiFi.status();
            if (status == WL_CONNECTED) {
                Serial.println("[BOOT_GUI] WiFi connected!");
                Serial.printf("[BOOT_GUI] IP: %s\n", WiFi.localIP().toString().c_str());
                connecting = false;
                
                // Save credentials on successful connection
                if (wifi_selection_index >= 0 && wifi_selection_index < (int)wifi_networks.size()) {
                    strncpy(wifi_ssid, wifi_networks[wifi_selection_index].ssid, sizeof(wifi_ssid) - 1);
                    strncpy(wifi_password, password_buffer, sizeof(wifi_password) - 1);
                    wifi_auto_connect = true;
                    saveSettings();
                }
            } else if (status == WL_CONNECT_FAILED || status == WL_NO_SSID_AVAIL) {
                Serial.println("[BOOT_GUI] Connection failed");
                connecting = false;
            }
        }
        
        // Get touch input
        if (getTouchEvent(&touch)) {
            // Handle keyboard input when visible
            if (show_keyboard) {
                if (touch.was_pressed) {
                    kb_highlight = getKeyboardHighlight(touch.x, touch.y, kb_x, kb_y, kb_w, kb_h);
                }
                
                if (touch.is_pressed) {
                    kb_highlight = getKeyboardHighlight(touch.x, touch.y, kb_x, kb_y, kb_w, kb_h);
                }
                
                if (touch.was_released) {
                    int key = getKeyboardKey(touch.x, touch.y, kb_x, kb_y, kb_w, kb_h);
                    kb_highlight = -1;
                    
                    if (key == KB_KEY_CANCEL) {
                        show_keyboard = false;
                    } else if (key == KB_KEY_ENTER) {
                        show_keyboard = false;
                    } else if (key == KB_KEY_SHIFT) {
                        shift_active = !shift_active;
                    } else if (key == KB_KEY_BACKSPACE) {
                        if (password_cursor > 0) {
                            password_cursor--;
                            password_buffer[password_cursor] = '\0';
                        }
                    } else if (key == KB_KEY_SPACE) {
                        if (password_cursor < (int)sizeof(password_buffer) - 1) {
                            password_buffer[password_cursor++] = ' ';
                            password_buffer[password_cursor] = '\0';
                        }
                    } else if (key > 0 && key < 128) {
                        // Regular character
                        char c = (char)key;
                        if (shift_active) {
                            // Convert to uppercase or shifted symbol
                            if (c >= 'a' && c <= 'z') {
                                c = c - 'a' + 'A';
                            } else {
                                // Handle number row shift symbols
                                const char* num_row = "1234567890";
                                const char* sym_row = "!@#$%^&*()";
                                for (int i = 0; i < 10; i++) {
                                    if (c == num_row[i]) {
                                        c = sym_row[i];
                                        break;
                                    }
                                }
                            }
                        }
                        if (password_cursor < (int)sizeof(password_buffer) - 1) {
                            password_buffer[password_cursor++] = c;
                            password_buffer[password_cursor] = '\0';
                        }
                    }
                }
            } else {
                // Regular screen interaction
                if (touch.was_pressed) {
                    touch_start_x = touch.x;
                    touch_start_y = touch.y;
                    
                    // Check buttons
                    if (isPointInRect(touch_start_x, touch_start_y, scan_btn_x, scan_btn_y, btn_w, btn_h)) {
                        scan_touch_started = true;
                        scan_pressed = true;
                    }
                    if (isPointInRect(touch_start_x, touch_start_y, connect_btn_x, connect_btn_y, btn_w, btn_h)) {
                        connect_touch_started = true;
                        connect_pressed = true;
                    }
                    if (isPointInRect(touch_start_x, touch_start_y, back_btn_x, back_btn_y, btn_w, btn_h)) {
                        back_touch_started = true;
                        back_pressed = true;
                    }
                    if (isPointInRect(touch_start_x, touch_start_y, password_x, password_y, password_w, password_h)) {
                        password_touched = true;
                    }
                }
                
                if (touch.was_released) {
                    // Handle button releases
                    if (scan_touch_started && !scanning) {
                        Serial.println("[BOOT_GUI] Starting WiFi scan...");
                        scanning = true;
                        WiFi.scanNetworks(true);
                    }
                    
                    if (connect_touch_started && wifi_selection_index >= 0 && !connecting) {
                        Serial.printf("[BOOT_GUI] Connecting to %s...\n", 
                                      wifi_networks[wifi_selection_index].ssid);
                        connecting = true;
                        WiFi.begin(wifi_networks[wifi_selection_index].ssid, password_buffer);
                    }
                    
                    if (back_touch_started) {
                        should_exit = true;
                    }
                    
                    if (password_touched) {
                        show_keyboard = true;
                    }
                    
                    // Check network list selection
                    if (isPointInRect(touch_start_x, touch_start_y, list_x, list_y, list_w, list_h)) {
                        int clicked_item = (touch_start_y - list_y - 2) / LIST_ITEM_HEIGHT + wifi_scroll_offset;
                        if (clicked_item >= 0 && clicked_item < (int)wifi_networks.size()) {
                            wifi_selection_index = clicked_item;
                            Serial.printf("[BOOT_GUI] Selected network: %s\n", wifi_networks[clicked_item].ssid);
                        }
                    }
                    
                    // Reset touch state
                    scan_touch_started = false;
                    scan_pressed = false;
                    connect_touch_started = false;
                    connect_pressed = false;
                    back_touch_started = false;
                    back_pressed = false;
                    password_touched = false;
                }
                
                // Update button visuals while held
                if (touch.is_pressed) {
                    if (scan_touch_started) {
                        scan_pressed = isPointInRect(touch.x, touch.y, scan_btn_x, scan_btn_y, btn_w, btn_h);
                    }
                    if (connect_touch_started) {
                        connect_pressed = isPointInRect(touch.x, touch.y, connect_btn_x, connect_btn_y, btn_w, btn_h);
                    }
                    if (back_touch_started) {
                        back_pressed = isPointInRect(touch.x, touch.y, back_btn_x, back_btn_y, btn_w, btn_h);
                    }
                }
            }
        }
        
        // Detect state changes
        wl_status_t wifi_status = WiFi.status();
        bool scanning_changed = (scanning != prev_scanning);
        bool connecting_changed = (connecting != prev_connecting);
        bool keyboard_changed = (show_keyboard != prev_show_keyboard);
        bool selection_changed = (wifi_selection_index != prev_wifi_selection);
        bool password_changed = ((int)strlen(password_buffer) != prev_password_len);
        bool status_changed = (wifi_status != prev_wifi_status);
        bool scan_btn_changed = (scan_pressed != prev_scan_pressed);
        bool connect_btn_changed = (connect_pressed != prev_connect_pressed);
        bool back_btn_changed = (back_pressed != prev_back_pressed);
        bool kb_highlight_changed = (kb_highlight != prev_kb_highlight);
        bool shift_changed = (shift_active != prev_shift_active);
        bool network_list_changed = (wifi_networks.size() != prev_network_count) || selection_changed || scanning_changed;
        
        // Save first_frame state before clearing it
        bool needs_full_draw = first_frame;
        if (first_frame) {
            // First frame - draw everything
            gfx.fillScreen(MAC_LIGHT_GRAY);
            
            // Draw title
            gfx.setTextColor(MAC_BLACK);
            gfx.setTextSize(3);
            gfx.setTextDatum(TC_DATUM);
            gfx.drawString("WiFi Settings", SCREEN_WIDTH / 2, SCREEN_MARGIN);
            
            // Draw "Networks:" label
            gfx.setTextSize(2);
            gfx.setTextDatum(TL_DATUM);
            gfx.drawString("Networks:", list_x, content_y);
            first_frame = false;
        }
        
        // Draw scanning indicator (only when state changes or first frame)
        if (scanning_changed || needs_full_draw) {
            // Clear the scanning area
            gfx.fillRect(SCREEN_WIDTH - SCREEN_MARGIN - 150, content_y, 150, 30, MAC_LIGHT_GRAY);
            if (scanning) {
                gfx.setTextColor(MAC_BLACK);
                gfx.setTextSize(2);
                gfx.setTextDatum(TR_DATUM);
                gfx.drawString("Scanning...", SCREEN_WIDTH - SCREEN_MARGIN, content_y);
            }
        }
        
        // Draw network list (only when needed)
        if (network_list_changed || needs_full_draw) {
            gfx.fillRect(list_x, list_y, list_w, list_h, MAC_WHITE);
            gfx.drawRect(list_x, list_y, list_w, list_h, MAC_BLACK);
            gfx.drawRect(list_x + 1, list_y + 1, list_w - 2, list_h - 2, MAC_BLACK);
            
            int visible_count = 6;
            gfx.setTextSize(2);
            gfx.setTextDatum(ML_DATUM);
            
            for (int i = 0; i < visible_count && (i + wifi_scroll_offset) < (int)wifi_networks.size(); i++) {
                int item_index = i + wifi_scroll_offset;
                int item_y = list_y + 3 + i * LIST_ITEM_HEIGHT;
                
                WiFiNetworkInfo& net = wifi_networks[item_index];
                
                // Highlight selected item
                if (item_index == wifi_selection_index) {
                    gfx.fillRect(list_x + 3, item_y, list_w - 6, LIST_ITEM_HEIGHT, MAC_BLACK);
                    gfx.setTextColor(MAC_WHITE);
                } else {
                    gfx.setTextColor(MAC_BLACK);
                }
                
                // Draw SSID
                char ssid_display[32];
                strncpy(ssid_display, net.ssid, 24);
                ssid_display[24] = '\0';
                if (strlen(net.ssid) > 24) {
                    strcat(ssid_display, "...");
                }
                gfx.drawString(ssid_display, list_x + 10, item_y + LIST_ITEM_HEIGHT / 2);
                
                // Draw signal bars
                int bars_x = list_x + list_w - 60;
                int bars_y = item_y + (LIST_ITEM_HEIGHT - 24) / 2;
                
                if (item_index == wifi_selection_index) {
                    // Invert signal bar colors for selected item
                    int bars = 0;
                    if (net.rssi >= -50) {
                        bars = 4;
                    } else if (net.rssi >= -60) {
                        bars = 3;
                    } else if (net.rssi >= -70) {
                        bars = 2;
                    } else if (net.rssi >= -80) {
                        bars = 1;
                    }
                    
                    int bar_width = 6;
                    int bar_gap = 3;
                    int max_height = 24;
                    
                    for (int b = 0; b < 4; b++) {
                        int bar_height = (max_height / 4) * (b + 1);
                        int bar_x = bars_x + b * (bar_width + bar_gap);
                        int bar_y_pos = bars_y + max_height - bar_height;
                        
                        if (b < bars) {
                            gfx.fillRect(bar_x, bar_y_pos, bar_width, bar_height, MAC_WHITE);
                        } else {
                            gfx.drawRect(bar_x, bar_y_pos, bar_width, bar_height, MAC_DARK_GRAY);
                        }
                    }
                } else {
                    drawSignalBars(bars_x, bars_y, net.rssi);
                }
                
                // Draw lock icon if encrypted
                if (net.encryption != WIFI_AUTH_OPEN) {
                    gfx.setTextColor(item_index == wifi_selection_index ? MAC_WHITE : MAC_BLACK);
                    gfx.drawString("*", list_x + list_w - 90, item_y + LIST_ITEM_HEIGHT / 2);
                }
            }
        }
        
        // Draw status area (only when status changes or first frame)
        if (status_changed || connecting_changed || needs_full_draw) {
            // Clear status area
            gfx.fillRect(content_x, status_y, content_w, 30, MAC_LIGHT_GRAY);
            
            gfx.setTextColor(MAC_BLACK);
            gfx.setTextSize(2);
            gfx.setTextDatum(TL_DATUM);
            
            const char* status_text = "Not connected";
            if (wifi_status == WL_CONNECTED) {
                status_text = "Connected";
            } else if (connecting) {
                status_text = "Connecting...";
            } else if (wifi_status == WL_CONNECT_FAILED) {
                status_text = "Connection failed";
            } else if (wifi_status == WL_NO_SSID_AVAIL) {
                status_text = "Network not found";
            }
            
            char status_line[128];
            sprintf(status_line, "Status: %s", status_text);
            gfx.drawString(status_line, content_x, status_y);
            
            // Show IP if connected
            if (wifi_status == WL_CONNECTED) {
                sprintf(status_line, "IP: %s", WiFi.localIP().toString().c_str());
                gfx.drawString(status_line, content_x + 300, status_y);
            }
        }
        
        // Draw password field (only when password changes, keyboard visibility changes, or first frame)
        if (password_changed || keyboard_changed || needs_full_draw) {
            // Clear and redraw password area
            gfx.fillRect(password_x - 130, password_y, password_w + 140, password_h, MAC_LIGHT_GRAY);
            
            if (!show_keyboard) {
                gfx.setTextColor(MAC_BLACK);
                gfx.setTextSize(2);
                gfx.setTextDatum(TL_DATUM);
                gfx.drawString("Password:", password_x - 120, password_y + 15);
            }
            
            // Password field
            gfx.fillRect(password_x, password_y, password_w, password_h, MAC_WHITE);
            gfx.drawRect(password_x, password_y, password_w, password_h, MAC_BLACK);
            gfx.drawRect(password_x + 1, password_y + 1, password_w - 2, password_h - 2, MAC_BLACK);
            
            // Draw password as dots
            gfx.setTextDatum(ML_DATUM);
            int pw_len = strlen(password_buffer);
            if (pw_len > 0) {
                char password_display[65];
                for (int i = 0; i < pw_len && i < 64; i++) {
                    password_display[i] = '*';
                }
                password_display[pw_len] = '\0';
                gfx.setTextColor(MAC_BLACK);
                gfx.drawString(password_display, password_x + 10, password_y + password_h / 2);
            } else {
                gfx.setTextColor(MAC_DARK_GRAY);
                gfx.drawString("Tap to enter password", password_x + 10, password_y + password_h / 2);
            }
        }
        
        // Draw keyboard or buttons depending on mode
        if (keyboard_changed || needs_full_draw) {
            if (show_keyboard) {
                // Draw keyboard overlay - covers from kb_y-60 to bottom of screen
                gfx.fillRect(0, kb_y - 60, SCREEN_WIDTH, SCREEN_HEIGHT - kb_y + 60, MAC_LIGHT_GRAY);
                
                // Draw current input
                gfx.setTextColor(MAC_BLACK);
                gfx.setTextSize(2);
                gfx.setTextDatum(MC_DATUM);
                gfx.drawString(password_buffer, SCREEN_WIDTH / 2, kb_y - 30);
                
                // Draw keyboard
                drawKeyboard(kb_x, kb_y, kb_w, kb_h, shift_active, kb_highlight);
            } else {
                // Clear entire keyboard area (same region that was covered when showing)
                gfx.fillRect(0, kb_y - 60, SCREEN_WIDTH, SCREEN_HEIGHT - kb_y + 60, MAC_LIGHT_GRAY);
                
                // Also redraw the password label since it may have been covered
                gfx.setTextColor(MAC_BLACK);
                gfx.setTextSize(2);
                gfx.setTextDatum(TL_DATUM);
                gfx.drawString("Password:", password_x - 120, password_y + 15);
                
                // Redraw password field
                gfx.fillRect(password_x, password_y, password_w, password_h, MAC_WHITE);
                gfx.drawRect(password_x, password_y, password_w, password_h, MAC_BLACK);
                gfx.drawRect(password_x + 1, password_y + 1, password_w - 2, password_h - 2, MAC_BLACK);
                
                gfx.setTextDatum(ML_DATUM);
                int pw_len = strlen(password_buffer);
                if (pw_len > 0) {
                    char password_display[65];
                    for (int i = 0; i < pw_len && i < 64; i++) {
                        password_display[i] = '*';
                    }
                    password_display[pw_len] = '\0';
                    gfx.setTextColor(MAC_BLACK);
                    gfx.drawString(password_display, password_x + 10, password_y + password_h / 2);
                } else {
                    gfx.setTextColor(MAC_DARK_GRAY);
                    gfx.drawString("Tap to enter password", password_x + 10, password_y + password_h / 2);
                }
                
                // Draw buttons
                drawButton(scan_btn_x, scan_btn_y, btn_w, btn_h, "Scan", scan_pressed);
                drawButton(connect_btn_x, connect_btn_y, btn_w, btn_h, "Connect", connect_pressed);
                drawButton(back_btn_x, back_btn_y, btn_w, btn_h, "Back", back_pressed);
            }
        } else if (show_keyboard) {
            // Keyboard visible - update only if something changed
            if (kb_highlight_changed || shift_changed || password_changed) {
                // Clear and redraw input area
                gfx.fillRect(0, kb_y - 60, SCREEN_WIDTH, 50, MAC_LIGHT_GRAY);
                gfx.setTextColor(MAC_BLACK);
                gfx.setTextSize(2);
                gfx.setTextDatum(MC_DATUM);
                gfx.drawString(password_buffer, SCREEN_WIDTH / 2, kb_y - 30);
                
                // Redraw keyboard
                drawKeyboard(kb_x, kb_y, kb_w, kb_h, shift_active, kb_highlight);
            }
        } else {
            // Buttons visible - update only if pressed state changed
            if (scan_btn_changed) {
                drawButton(scan_btn_x, scan_btn_y, btn_w, btn_h, "Scan", scan_pressed);
            }
            if (connect_btn_changed) {
                drawButton(connect_btn_x, connect_btn_y, btn_w, btn_h, "Connect", connect_pressed);
            }
            if (back_btn_changed) {
                drawButton(back_btn_x, back_btn_y, btn_w, btn_h, "Back", back_pressed);
            }
        }
        
        // Update previous state for next iteration
        prev_scanning = scanning;
        prev_connecting = connecting;
        prev_show_keyboard = show_keyboard;
        prev_wifi_selection = wifi_selection_index;
        prev_password_len = strlen(password_buffer);
        prev_wifi_status = wifi_status;
        prev_scan_pressed = scan_pressed;
        prev_connect_pressed = connect_pressed;
        prev_back_pressed = back_pressed;
        prev_kb_highlight = kb_highlight;
        prev_shift_active = shift_active;
        prev_network_count = wifi_networks.size();
        
        delay(1);  // Minimal delay - just yield to other tasks
    }
    
    Serial.println("[BOOT_GUI] Exiting WiFi screen");
}

// ============================================================================
// Public API
// ============================================================================

bool BootGUI_Init(void)
{
    Serial.println("[BOOT_GUI] Initializing...");
    
    // IMPORTANT: Warm up the touch panel
    // The touch controller needs several update cycles to become responsive
    Serial.println("[BOOT_GUI] Warming up touch panel...");
    for (int i = 0; i < 20; i++) {
        M5.update();
        delay(50);
    }
    Serial.println("[BOOT_GUI] Touch panel ready");
    
    // Start the touch polling task for responsive input
    if (!startTouchTask()) {
        Serial.println("[BOOT_GUI] WARNING: Failed to start touch task, falling back to sync mode");
    }
    
    // Get display dimensions
    SCREEN_WIDTH = M5.Display.width();
    SCREEN_HEIGHT = M5.Display.height();
    Serial.printf("[BOOT_GUI] Display size: %dx%d\n", SCREEN_WIDTH, SCREEN_HEIGHT);
    
    // Drawing directly to display framebuffer - no canvas needed
    // This is much faster than using a PSRAM canvas + pushSprite
    gfx.setColorDepth(16);
    
    // Load saved settings
    loadSettings();
    
    // Scan for disk files
    scanDiskFiles();
    scanCDROMFiles();
    
    // If no disk is selected but we found some, select the first one
    if (strlen(selected_disk_path) == 0 && disk_files.size() > 0) {
        strncpy(selected_disk_path, disk_files[0].c_str(), BOOT_GUI_MAX_PATH - 1);
        disk_selection_index = 0;
    }
    
    gui_initialized = true;
    Serial.println("[BOOT_GUI] Initialization complete");
    
    return true;
}

void BootGUI_Run(void)
{
    if (!gui_initialized) {
        Serial.println("[BOOT_GUI] ERROR: GUI not initialized");
        return;
    }
    
    // Check if we should skip the GUI
    if (skip_gui) {
        Serial.println("[BOOT_GUI] skip_gui=yes, skipping boot GUI");
        Serial.printf("[BOOT_GUI] Using saved settings: disk=%s, ram=%dMB\n", 
                      selected_disk_path, selected_ram_mb);
        
        // Stop the touch task before returning to emulator
        stopTouchTask();
        
        // Clean up WiFi if it was initialized but not connected
        if (wifi_initialized) {
            WiFi.scanDelete();
            wl_status_t status = WiFi.status();
            if (status != WL_CONNECTED) {
                Serial.println("[BOOT_GUI] Disconnecting WiFi (not connected)...");
                WiFi.disconnect(true);
                WiFi.mode(WIFI_OFF);
                delay(100);
                wifi_initialized = false;
            }
        }
        return;
    }
    
    Serial.println("[BOOT_GUI] Running boot GUI...");
    
    // Run countdown screen (may transition to settings screen)
    runCountdownScreen();
    
    // Stop the touch task before returning to emulator (emulator has its own input task)
    stopTouchTask();
    
    // Clean up WiFi if it was initialized but not connected
    // This prevents any lingering WiFi state from interfering with the emulator
    if (wifi_initialized) {
        // Cancel any in-progress scan
        WiFi.scanDelete();
        
        wl_status_t status = WiFi.status();
        if (status != WL_CONNECTED) {
            Serial.println("[BOOT_GUI] Disconnecting WiFi (not connected)...");
            WiFi.disconnect(true);  // Disconnect and turn off WiFi
            WiFi.mode(WIFI_OFF);
            delay(100);  // Give WiFi time to clean up
            wifi_initialized = false;  // Mark as uninitialized
            Serial.println("[BOOT_GUI] WiFi cleanup complete");
        } else {
            Serial.printf("[BOOT_GUI] WiFi connected, keeping connection (IP: %s)\n", 
                         WiFi.localIP().toString().c_str());
        }
    }
    
    Serial.println("[BOOT_GUI] Boot GUI complete, proceeding to emulator");
}

const char* BootGUI_GetDiskPath(void)
{
    return selected_disk_path;
}

const char* BootGUI_GetCDROMPath(void)
{
    return selected_cdrom_path;
}

uint32_t BootGUI_GetRAMSize(void)
{
    return (uint32_t)selected_ram_mb * 1024 * 1024;
}

int BootGUI_GetRAMSizeMB(void)
{
    return selected_ram_mb;
}

const char* BootGUI_GetWiFiSSID(void)
{
    return wifi_ssid;
}

const char* BootGUI_GetWiFiPassword(void)
{
    return wifi_password;
}

bool BootGUI_GetWiFiAutoConnect(void)
{
    return wifi_auto_connect;
}

bool BootGUI_IsWiFiConnected(void)
{
    return WiFi.status() == WL_CONNECTED;
}

uint32_t BootGUI_GetWiFiIP(void)
{
    if (WiFi.status() != WL_CONNECTED) {
        return 0;
    }
    // Return IP in host byte order (ESP32 gives it in network byte order)
    IPAddress ip = WiFi.localIP();
    return (uint32_t(ip[0]) << 24) | (uint32_t(ip[1]) << 16) | 
           (uint32_t(ip[2]) << 8) | uint32_t(ip[3]);
}
