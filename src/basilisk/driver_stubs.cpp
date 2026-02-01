/*
 *  driver_stubs.cpp - Stub implementations for disabled drivers
 *
 *  BasiliskII ESP32 Port
 *  
 *  These stubs provide minimal implementations for drivers that are
 *  disabled on ESP32 but are referenced by the main emulation code.
 */

#include "sysdeps.h"
#include "cpu_emulation.h"
#include "main.h"
#include "macos_util.h"
#include "scsi.h"
#include "serial.h"
// Note: ether.h not needed - real ethernet implementation in ether.cpp/ether_esp32.cpp
// Note: audio.h not needed - real audio implementation in audio.cpp/audio_esp32.cpp
#include "user_strings.h"

/*
 * Global tick inhibit flag (referenced by emul_op.cpp)
 */
bool tick_inhibit = false;

/*
 * SCSI driver stubs
 */

int16 SCSIReset(void) { return noErr; }
int16 SCSIGet(void) { return noErr; }
int16 SCSISelect(int id) { (void)id; return noErr; }
int16 SCSICmd(int len, uint8 *cmd) { (void)len; (void)cmd; return noErr; }
int16 SCSIRead(uint32 tib) { (void)tib; return noErr; }
int16 SCSIWrite(uint32 tib) { (void)tib; return noErr; }
int16 SCSIComplete(uint32 stat, uint32 msg, uint32 ticks) { (void)stat; (void)msg; (void)ticks; return noErr; }
uint16 SCSIStat(void) { return 0; }  // Return 0 = bus free
int16 SCSIMsgIn(void) { return 0; }
int16 SCSIMsgOut(void) { return noErr; }
int16 SCSIMgrBusy(void) { return 0; }  // Return 0 = not busy

/*
 * Serial driver stubs
 */

// Dummy serial port object
class DummySERDPort : public SERDPort {
public:
    DummySERDPort() : SERDPort() {}
    virtual ~DummySERDPort() {}
    
    virtual int16 open(uint16 config) { (void)config; return noErr; }
    virtual int16 prime_in(uint32 pb, uint32 dce) { (void)pb; (void)dce; return noErr; }
    virtual int16 prime_out(uint32 pb, uint32 dce) { (void)pb; (void)dce; return noErr; }
    virtual int16 control(uint32 pb, uint32 dce, uint16 code) { (void)pb; (void)dce; (void)code; return noErr; }
    virtual int16 status(uint32 pb, uint32 dce, uint16 code) { (void)pb; (void)dce; (void)code; return noErr; }
    virtual int16 close(void) { return noErr; }
};

// Create dummy port instances
static DummySERDPort dummy_port_a;
static DummySERDPort dummy_port_b;

// The serial port instance (referenced by serial_dummy.cpp)
SERDPort *the_serd_port[2] = { &dummy_port_a, &dummy_port_b };

void SerialInit(void) {}
void SerialExit(void) {}
int16 SerialOpen(uint32 pb, uint32 dce, int port) { (void)pb; (void)dce; (void)port; return noErr; }
int16 SerialPrime(uint32 pb, uint32 dce, int port) { (void)pb; (void)dce; (void)port; return noErr; }
int16 SerialControl(uint32 pb, uint32 dce, int port) { (void)pb; (void)dce; (void)port; return noErr; }
int16 SerialStatus(uint32 pb, uint32 dce, int port) { (void)pb; (void)dce; (void)port; return noErr; }
int16 SerialClose(uint32 pb, uint32 dce, int port) { (void)pb; (void)dce; (void)port; return noErr; }
void SerialInterrupt(void) {}

/*
 * Ethernet driver - real implementation in ether.cpp and ether_esp32.cpp
 * (stubs removed to avoid duplicate symbols)
 */

/*
 * Audio driver - real implementation in audio.cpp and audio_esp32.cpp
 * (stubs removed to avoid duplicate symbols)
 */

/*
 * Timer functions - ESP32 implementation
 */

// Get current time in microseconds
void timer_current_time(uint64 &time) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time = (uint64)tv.tv_sec * 1000000 + tv.tv_usec;
}

// Build date/time as base for Mac clock
// ESP32 doesn't have RTC, so we use build time as the starting point
static time_t get_build_timestamp(void) {
    // Parse __DATE__ (format: "Jan 17 2026") and __TIME__ (format: "14:30:00")
    static time_t build_time = 0;
    static bool initialized = false;
    
    if (!initialized) {
        const char *date_str = __DATE__;  // "Mmm DD YYYY"
        const char *time_str = __TIME__;  // "HH:MM:SS"
        
        // Month lookup
        const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
        int month = 0;
        for (int i = 0; i < 12; i++) {
            if (strncmp(date_str, months[i], 3) == 0) {
                month = i;
                break;
            }
        }
        
        int day = atoi(date_str + 4);
        int year = atoi(date_str + 7);
        int hour = atoi(time_str);
        int minute = atoi(time_str + 3);
        int second = atoi(time_str + 6);
        
        struct tm tm_build;
        memset(&tm_build, 0, sizeof(tm_build));
        tm_build.tm_year = year - 1900;
        tm_build.tm_mon = month;
        tm_build.tm_mday = day;
        tm_build.tm_hour = hour;
        tm_build.tm_min = minute;
        tm_build.tm_sec = second;
        
        build_time = mktime(&tm_build);
        initialized = true;
        
        Serial.printf("[TIME] Build timestamp: %s %s -> %ld\n", date_str, time_str, (long)build_time);
    }
    
    return build_time;
}

// Return current date/time as Mac seconds since 1904
uint32 TimerDateTime(void) {
    // Mac epoch is Jan 1, 1904
    // Unix epoch is Jan 1, 1970
    // Difference is 2082844800 seconds
    
    // Get time from ESP32 - if not set via NTP, use build time as base
    time_t t = time(NULL);
    
    // If time appears to be near Unix epoch (before year 2020), use build time instead
    // This handles ESP32 without RTC or NTP
    if (t < 1577836800) {  // Jan 1, 2020
        // Use build time + seconds since boot
        static time_t boot_time = 0;
        static uint32_t boot_millis = 0;
        
        if (boot_time == 0) {
            boot_time = get_build_timestamp();
            boot_millis = millis();
        }
        
        // Add elapsed time since boot
        t = boot_time + ((millis() - boot_millis) / 1000);
    }
    
    return (uint32)(t + 2082844800UL);
}

// Return microsecond counter (split into hi/lo 32-bit parts)
void Microseconds(uint32 &hi, uint32 &lo) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64 us = (uint64)tv.tv_sec * 1000000 + tv.tv_usec;
    hi = (uint32)(us >> 32);
    lo = (uint32)(us & 0xFFFFFFFF);
}

// Add two times
void timer_add_time(uint64 &res, uint64 a, uint64 b) {
    res = a + b;
}

// Subtract two times
void timer_sub_time(uint64 &res, uint64 a, uint64 b) {
    res = a - b;
}

// Compare two times
int timer_cmp_time(uint64 a, uint64 b) {
    if (a < b) return -1;
    if (a > b) return 1;
    return 0;
}

// Convert Mac time to host time (microseconds)
void timer_mac2host_time(uint64 &res, int32 mactime) {
    if (mactime > 0) {
        // Positive: already in microseconds
        res = (uint64)mactime;
    } else {
        // Negative: milliseconds (negate to get positive)
        res = (uint64)(-mactime) * 1000;
    }
}

// Convert host time to Mac time
int32 timer_host2mac_time(uint64 hosttime) {
    if (hosttime < 0x7fffffff) {
        return (int32)hosttime;  // Microseconds
    } else {
        return (int32)(-(hosttime / 1000));  // Milliseconds (negative)
    }
}

/*
 * Clipboard driver stubs
 */

void ClipInit(void) {}
void ClipExit(void) {}
void GetScrap(void **handle, uint32 type, int32 offset) { 
    (void)handle; (void)type; (void)offset;
}
void PutScrap(uint32 type, void *data, int32 length) {
    (void)type; (void)data; (void)length;
}

/*
 * SCSI Init/Exit stubs
 */

void SCSIInit(void) {}
void SCSIExit(void) {}

/*
 * User string lookup
 */

const char *GetString(int num) {
    // Return empty string for unknown string IDs
    static const char *empty = "";
    (void)num;
    return empty;
}
