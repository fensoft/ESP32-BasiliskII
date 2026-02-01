/*
 *  audio_esp32.cpp - Audio support for ESP32-P4 with M5Unified Speaker
 *
 *  BasiliskII ESP32 Port
 *
 *  Uses M5Unified Speaker class to interface with ES8388 codec on Tab5.
 *  Audio data is retrieved from the Mac OS Apple Mixer via 68k code execution,
 *  then converted from big-endian to little-endian and sent to the speaker.
 *
 *  Audio task runs on Core 0 alongside video/input tasks.
 */

#include "sysdeps.h"
#include "cpu_emulation.h"
#include "main.h"
#include "prefs.h"
#include "audio.h"
#include "audio_defs.h"

#include <M5Unified.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_attr.h"

#define DEBUG 0
#include "debug.h"

// ============================================================================
// Audio Configuration
// ============================================================================

// Audio task configuration
#define AUDIO_TASK_STACK_SIZE  4096
#define AUDIO_TASK_PRIORITY    2  // Slightly higher priority than video/input
#define AUDIO_TASK_CORE        0  // Run on Core 0, leaving Core 1 for CPU emulation

// Audio buffer configuration
// Using 22050 Hz for better ESP32 performance (lower CPU load)
// Mac OS will resample if needed
#define AUDIO_SAMPLE_RATE      22050
#define AUDIO_BUFFER_FRAMES    1024   // Number of frames per buffer
#define AUDIO_CHANNELS         2      // Stereo
#define AUDIO_SAMPLE_SIZE      16     // 16-bit samples

// Calculate buffer sizes
#define AUDIO_BYTES_PER_FRAME  (AUDIO_CHANNELS * (AUDIO_SAMPLE_SIZE / 8))
#define AUDIO_BUFFER_SIZE      (AUDIO_BUFFER_FRAMES * AUDIO_BYTES_PER_FRAME)

// Mac volume is 8.8 fixed point, max is 0x0100
#define MAC_MAX_VOLUME 0x0100

// ============================================================================
// Audio State
// ============================================================================

// The currently selected audio parameters (indices in audio_sample_rates[] etc. vectors)
static int audio_sample_rate_index = 0;
static int audio_sample_size_index = 0;
static int audio_channel_count_index = 0;

// Audio task handle and state
static TaskHandle_t audio_task_handle = NULL;
static volatile bool audio_task_running = false;

// Semaphore for synchronization between audio task and AudioInterrupt
static SemaphoreHandle_t audio_irq_done_sem = NULL;

// Audio mixing buffer - in PSRAM for larger size
static int16_t *audio_mix_buf = NULL;

// Volume and mute state
// Start at 50% to avoid distortion - Mac OS can adjust via Sound control panel
static int main_volume = MAC_MAX_VOLUME / 2;
static int speaker_volume = MAC_MAX_VOLUME / 2;
static bool main_mute = false;
static bool speaker_mute = false;

// Speaker initialization state
static bool speaker_initialized = false;

// ============================================================================
// Internal Functions
// ============================================================================

/*
 *  Set AudioStatus to reflect current audio stream format
 */
static void set_audio_status_format(void)
{
    AudioStatus.sample_rate = audio_sample_rates[audio_sample_rate_index];
    AudioStatus.sample_size = audio_sample_sizes[audio_sample_size_index];
    AudioStatus.channels = audio_channel_counts[audio_channel_count_index];
}

/*
 *  Calculate the effective volume (0-255 for M5Unified Speaker)
 */
static uint8_t get_effective_volume(void)
{
    if (main_mute || speaker_mute) {
        return 0;
    }
    
    // Combine main and speaker volume
    // Both are 8.8 fixed point, max 0x0100
    // Result should be 0-255 for M5Unified
    uint32_t combined = (main_volume * speaker_volume) / MAC_MAX_VOLUME;
    
    // Scale to 0-255 range
    uint32_t volume = (combined * 255) / MAC_MAX_VOLUME;
    if (volume > 255) {
        volume = 255;
    }
    
    return (uint8_t)volume;
}

/*
 *  Initialize M5Unified Speaker
 */
static bool init_speaker(void)
{
    Serial.println("[AUDIO] Initializing M5Unified Speaker...");
    
    // Get current speaker config
    auto spk_cfg = M5.Speaker.config();
    
    // Configure for our audio format
    spk_cfg.sample_rate = AUDIO_SAMPLE_RATE;
    spk_cfg.stereo = (AUDIO_CHANNELS == 2);
    spk_cfg.buzzer = false;  // Not using buzzer mode
    spk_cfg.use_dac = false; // Use I2S, not DAC
    
    // Apply configuration
    M5.Speaker.config(spk_cfg);
    
    // Start the speaker
    if (!M5.Speaker.begin()) {
        Serial.println("[AUDIO] ERROR: Failed to start M5Unified Speaker");
        return false;
    }
    
    // Set initial volume
    M5.Speaker.setVolume(get_effective_volume());
    
    Serial.printf("[AUDIO] Speaker initialized: %d Hz, %s\n",
                  AUDIO_SAMPLE_RATE,
                  AUDIO_CHANNELS == 2 ? "stereo" : "mono");
    
    speaker_initialized = true;
    return true;
}

/*
 *  Stop M5Unified Speaker
 */
static void stop_speaker(void)
{
    if (speaker_initialized) {
        M5.Speaker.stop();
        M5.Speaker.end();
        speaker_initialized = false;
        Serial.println("[AUDIO] Speaker stopped");
    }
}

/*
 *  Convert big-endian Mac audio data to little-endian ESP32 format
 *  Volume is handled by M5Unified Speaker, not here
 */
static void convert_audio_data(const uint8_t *src, int16_t *dst, int sample_count)
{
    // Mac audio is big-endian, ESP32 is little-endian
    // Just byte-swap, let M5Unified handle volume
    
    for (int i = 0; i < sample_count; i++) {
        // Read big-endian 16-bit sample and convert to little-endian
        dst[i] = (int16_t)((src[i * 2] << 8) | src[i * 2 + 1]);
    }
}

/*
 *  Audio streaming task - runs on Core 0
 *  Periodically requests audio data from Mac OS and sends to speaker
 */
static void audioTask(void *param)
{
    (void)param;
    Serial.println("[AUDIO] Audio task started on Core 0");
    
    // Wait interval based on buffer size and sample rate
    // buffer_frames / sample_rate * 1000 = milliseconds per buffer
    const uint32_t buffer_ms = (AUDIO_BUFFER_FRAMES * 1000) / AUDIO_SAMPLE_RATE;
    const TickType_t task_interval = pdMS_TO_TICKS(buffer_ms > 0 ? buffer_ms : 20);
    
    Serial.printf("[AUDIO] Buffer interval: %u ms\n", buffer_ms);
    
    while (audio_task_running) {
        // Check if there are active sound sources
        if (AudioStatus.num_sources > 0 && audio_open) {
            // Trigger audio interrupt to get new buffer from Mac OS
            D(bug("[AUDIO] Triggering audio interrupt\n"));
            SetInterruptFlag(INTFLAG_AUDIO);
            TriggerInterrupt();
            
            // Wait for AudioInterrupt to complete (with timeout)
            if (xSemaphoreTake(audio_irq_done_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Get stream info from Apple Mixer
                uint32_t apple_stream_info = ReadMacInt32(audio_data + adatStreamInfo);
                
                if (apple_stream_info && !main_mute && !speaker_mute && speaker_initialized) {
                    // Calculate work size
                    uint32_t sample_count = ReadMacInt32(apple_stream_info + scd_sampleCount);
                    uint32_t src_channels = ReadMacInt16(apple_stream_info + scd_numChannels);
                    uint32_t src_sample_size = ReadMacInt16(apple_stream_info + scd_sampleSize);
                    
                    int work_size = sample_count * (src_sample_size >> 3) * src_channels;
                    
                    D(bug("[AUDIO] Got %d samples, %d channels, %d bits\n",
                          sample_count, src_channels, src_sample_size));
                    
                    if (work_size > 0 && work_size <= AUDIO_BUFFER_SIZE) {
                        // Get source buffer pointer
                        uint8_t *src = Mac2HostAddr(ReadMacInt32(apple_stream_info + scd_buffer));
                        
                        if (src != NULL && audio_mix_buf != NULL) {
                            // Handle mono to stereo expansion if needed
                            bool need_expand = (AUDIO_CHANNELS == 2 && src_channels == 1);
                            
                            if (need_expand && src_sample_size == 8) {
                                // Expand 8-bit mono to 16-bit stereo
                                // Volume is handled by M5Unified Speaker
                                for (int i = 0; i < (int)sample_count; i++) {
                                    // Convert unsigned 8-bit to signed 16-bit
                                    int16_t sample = ((int16_t)src[i] - 128) << 8;
                                    audio_mix_buf[i * 2] = sample;      // Left
                                    audio_mix_buf[i * 2 + 1] = sample;  // Right
                                }
                                
                                // Send to speaker
                                M5.Speaker.playRaw(audio_mix_buf, sample_count * 2, AUDIO_SAMPLE_RATE, true, 1, 0, false);
                            } else if (src_sample_size == 16) {
                                // Convert 16-bit big-endian to little-endian
                                // Volume is handled by M5Unified Speaker
                                int total_samples = sample_count * src_channels;
                                convert_audio_data(src, audio_mix_buf, total_samples);
                                
                                // Expand mono to stereo if needed
                                if (need_expand) {
                                    // In-place expansion (work backwards to avoid overwrite)
                                    for (int i = sample_count - 1; i >= 0; i--) {
                                        audio_mix_buf[i * 2] = audio_mix_buf[i];
                                        audio_mix_buf[i * 2 + 1] = audio_mix_buf[i];
                                    }
                                    total_samples = sample_count * 2;
                                }
                                
                                // Send to speaker
                                M5.Speaker.playRaw(audio_mix_buf, total_samples, AUDIO_SAMPLE_RATE, true, 1, 0, false);
                            }
                        }
                    }
                }
            } else {
                D(bug("[AUDIO] Timeout waiting for AudioInterrupt\n"));
            }
        }
        
        // Wait until next buffer interval
        vTaskDelay(task_interval);
    }
    
    Serial.println("[AUDIO] Audio task exiting");
    vTaskDelete(NULL);
}

/*
 *  Open audio device
 */
static bool open_audio(void)
{
    // Initialize supported formats if not already done
    if (audio_sample_sizes.empty()) {
        // Support 22050 Hz (recommended for ESP32) and 44100 Hz
        audio_sample_rates.push_back(22050 << 16);
        audio_sample_rates.push_back(44100 << 16);
        
        // Support 8-bit and 16-bit
        audio_sample_sizes.push_back(8);
        audio_sample_sizes.push_back(16);
        
        // Support mono and stereo
        audio_channel_counts.push_back(1);
        audio_channel_counts.push_back(2);
        
        // Default to 22050 Hz, 16-bit, stereo
        audio_sample_rate_index = 0;  // 22050 Hz
        audio_sample_size_index = 1;  // 16-bit
        audio_channel_count_index = 1; // Stereo
    }
    
    // Set audio frames per block
    audio_frames_per_block = AUDIO_BUFFER_FRAMES;
    
    // Allocate audio buffer in PSRAM
    if (audio_mix_buf == NULL) {
        audio_mix_buf = (int16_t *)heap_caps_malloc(AUDIO_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
        if (audio_mix_buf == NULL) {
            Serial.println("[AUDIO] ERROR: Failed to allocate audio buffer");
            return false;
        }
        memset(audio_mix_buf, 0, AUDIO_BUFFER_SIZE);
        Serial.printf("[AUDIO] Allocated %d byte audio buffer in PSRAM\n", AUDIO_BUFFER_SIZE);
    }
    
    // Initialize speaker
    if (!init_speaker()) {
        return false;
    }
    
    // Set AudioStatus to reflect format
    set_audio_status_format();
    
    audio_open = true;
    return true;
}

/*
 *  Close audio device
 */
static void close_audio(void)
{
    stop_speaker();
    
    if (audio_mix_buf != NULL) {
        heap_caps_free(audio_mix_buf);
        audio_mix_buf = NULL;
    }
    
    audio_open = false;
}

// ============================================================================
// Public API (called from audio.cpp and emulator core)
// ============================================================================

/*
 *  Initialization
 */
void AudioInit(void)
{
    Serial.println("[AUDIO] Initializing audio subsystem...");
    
    // Init audio status and feature flags
    AudioStatus.sample_rate = AUDIO_SAMPLE_RATE << 16;  // 16.16 fixed point
    AudioStatus.sample_size = AUDIO_SAMPLE_SIZE;
    AudioStatus.channels = AUDIO_CHANNELS;
    AudioStatus.mixer = 0;
    AudioStatus.num_sources = 0;
    audio_component_flags = cmpWantsRegisterMessage | kStereoOut | k16BitOut;
    
    // Sound disabled in prefs? Then do nothing
    if (PrefsFindBool("nosound")) {
        Serial.println("[AUDIO] Sound disabled in preferences");
        return;
    }
    
    // Create semaphore for audio interrupt synchronization
    audio_irq_done_sem = xSemaphoreCreateBinary();
    if (audio_irq_done_sem == NULL) {
        Serial.println("[AUDIO] ERROR: Failed to create audio semaphore");
        return;
    }
    
    // Open audio device
    if (!open_audio()) {
        Serial.println("[AUDIO] Failed to open audio device");
        return;
    }
    
    // Start audio task on Core 0
    audio_task_running = true;
    BaseType_t result = xTaskCreatePinnedToCore(
        audioTask,
        "AudioTask",
        AUDIO_TASK_STACK_SIZE,
        NULL,
        AUDIO_TASK_PRIORITY,
        &audio_task_handle,
        AUDIO_TASK_CORE
    );
    
    if (result != pdPASS) {
        Serial.println("[AUDIO] ERROR: Failed to create audio task");
        audio_task_running = false;
        close_audio();
        return;
    }
    
    Serial.printf("[AUDIO] Audio task created on Core %d\n", AUDIO_TASK_CORE);
    Serial.println("[AUDIO] Audio subsystem initialized successfully");
}

/*
 *  Deinitialization
 */
void AudioExit(void)
{
    Serial.println("[AUDIO] Shutting down audio subsystem...");
    
    // Stop audio task
    if (audio_task_running) {
        audio_task_running = false;
        // Give task time to exit
        vTaskDelay(pdMS_TO_TICKS(100));
        audio_task_handle = NULL;
    }
    
    // Close audio device
    close_audio();
    
    // Delete semaphore
    if (audio_irq_done_sem != NULL) {
        vSemaphoreDelete(audio_irq_done_sem);
        audio_irq_done_sem = NULL;
    }
    
    Serial.println("[AUDIO] Audio subsystem shut down");
}

/*
 *  First source added, start audio stream
 */
void audio_enter_stream(void)
{
    D(bug("[AUDIO] audio_enter_stream\n"));
    // Audio task handles this automatically via AudioStatus.num_sources
}

/*
 *  Last source removed, stop audio stream
 */
void audio_exit_stream(void)
{
    D(bug("[AUDIO] audio_exit_stream\n"));
    // Audio task handles this automatically via AudioStatus.num_sources
    
    // Stop any current playback
    if (speaker_initialized) {
        M5.Speaker.stop();
    }
}

/*
 *  MacOS audio interrupt, read next data block
 *  Called from emul_op.cpp when INTFLAG_AUDIO is set
 */
void AudioInterrupt(void)
{
    D(bug("[AUDIO] AudioInterrupt\n"));
    
    // Get data from Apple Mixer
    if (AudioStatus.mixer) {
        M68kRegisters r;
        r.a[0] = audio_data + adatStreamInfo;
        r.a[1] = AudioStatus.mixer;
        Execute68k(audio_data + adatGetSourceData, &r);
        D(bug("[AUDIO] GetSourceData() returns %08lx\n", r.d[0]));
    } else {
        WriteMacInt32(audio_data + adatStreamInfo, 0);
    }
    
    // Signal audio task that data is ready
    if (audio_irq_done_sem != NULL) {
        xSemaphoreGive(audio_irq_done_sem);
    }
    
    D(bug("[AUDIO] AudioInterrupt done\n"));
}

/*
 *  Set sampling parameters
 *  "index" is an index into the audio_sample_rates[] etc. vectors
 *  It is guaranteed that AudioStatus.num_sources == 0
 */
bool audio_set_sample_rate(int index)
{
    if (index < 0 || index >= (int)audio_sample_rates.size()) {
        return false;
    }
    
    audio_sample_rate_index = index;
    set_audio_status_format();
    
    Serial.printf("[AUDIO] Sample rate set to %d Hz\n",
                  audio_sample_rates[index] >> 16);
    return true;
}

bool audio_set_sample_size(int index)
{
    if (index < 0 || index >= (int)audio_sample_sizes.size()) {
        return false;
    }
    
    audio_sample_size_index = index;
    set_audio_status_format();
    
    Serial.printf("[AUDIO] Sample size set to %d bits\n",
                  audio_sample_sizes[index]);
    return true;
}

bool audio_set_channels(int index)
{
    if (index < 0 || index >= (int)audio_channel_counts.size()) {
        return false;
    }
    
    audio_channel_count_index = index;
    set_audio_status_format();
    
    Serial.printf("[AUDIO] Channels set to %d\n",
                  audio_channel_counts[index]);
    return true;
}

/*
 *  Get/set volume controls
 *  Volume values use 8.8 fixed point with 0x0100 meaning "maximum volume"
 *  Left channel in upper 16 bits, right channel in lower 16 bits
 */

bool audio_get_main_mute(void)
{
    return main_mute;
}

uint32 audio_get_main_volume(void)
{
    uint32 chan = main_volume;
    return (chan << 16) | chan;
}

bool audio_get_speaker_mute(void)
{
    return speaker_mute;
}

uint32 audio_get_speaker_volume(void)
{
    uint32 chan = speaker_volume;
    return (chan << 16) | chan;
}

void audio_set_main_mute(bool mute)
{
    main_mute = mute;
    
    // Update M5Unified Speaker volume
    if (speaker_initialized) {
        M5.Speaker.setVolume(get_effective_volume());
    }
    
    D(bug("[AUDIO] Main mute set to %d\n", mute));
}

void audio_set_main_volume(uint32 vol)
{
    // Average left and right channels
    main_volume = ((vol >> 16) + (vol & 0xffff)) / 2;
    if (main_volume > MAC_MAX_VOLUME) {
        main_volume = MAC_MAX_VOLUME;
    }
    
    // Update M5Unified Speaker volume
    if (speaker_initialized) {
        M5.Speaker.setVolume(get_effective_volume());
    }
    
    D(bug("[AUDIO] Main volume set to %d\n", main_volume));
}

void audio_set_speaker_mute(bool mute)
{
    speaker_mute = mute;
    
    // Update M5Unified Speaker volume
    if (speaker_initialized) {
        M5.Speaker.setVolume(get_effective_volume());
    }
    
    D(bug("[AUDIO] Speaker mute set to %d\n", mute));
}

void audio_set_speaker_volume(uint32 vol)
{
    // Average left and right channels
    speaker_volume = ((vol >> 16) + (vol & 0xffff)) / 2;
    if (speaker_volume > MAC_MAX_VOLUME) {
        speaker_volume = MAC_MAX_VOLUME;
    }
    
    // Update M5Unified Speaker volume
    if (speaker_initialized) {
        M5.Speaker.setVolume(get_effective_volume());
    }
    
    D(bug("[AUDIO] Speaker volume set to %d\n", speaker_volume));
}
