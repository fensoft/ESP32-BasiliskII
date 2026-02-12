/*
 *  ether_esp32.cpp - Ethernet device driver for ESP32
 *
 *  BasiliskII ESP32 Port
 *  Based on Basilisk II (C) 1997-2008 Christian Bauer
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */

#include "sysdeps.h"

#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <map>

#include "cpu_emulation.h"
#include "main.h"
#include "macos_util.h"
#include "prefs.h"
#include "ether.h"
#include "ether_defs.h"
#include "net_router.h"

#define DEBUG 0
#include "debug.h"

// ============================================================================
// Global Variables
// ============================================================================

// Protocol handlers - maps Ethernet protocol type to MacOS handler address
static std::map<uint16, uint32> protocol_handlers;

// Network receive task
static TaskHandle_t net_rx_task_handle = nullptr;
static volatile bool net_rx_task_running = false;

// Mutex for thread safety
static SemaphoreHandle_t net_mutex = nullptr;

// Flag indicating network is initialized
static bool net_initialized = false;

// Statistics
static uint32 packets_sent = 0;
static uint32 packets_received = 0;

// ============================================================================
// Forward Declarations
// ============================================================================

static void net_rx_task(void *param);

// ============================================================================
// Platform-Specific Ethernet Functions
// ============================================================================

/*
 *  Initialize network driver
 */
bool ether_init(void)
{
    Serial.println("[ETHER] Initializing ESP32 network driver...");
    
    // Check if WiFi is connected
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[ETHER] WiFi not connected, networking disabled");
        return false;
    }
    
    // Create mutex for thread safety
    net_mutex = xSemaphoreCreateMutex();
    if (net_mutex == nullptr) {
        Serial.println("[ETHER] Failed to create mutex");
        return false;
    }
    
    // Initialize the router
    if (!router_init()) {
        Serial.println("[ETHER] Failed to initialize router");
        vSemaphoreDelete(net_mutex);
        net_mutex = nullptr;
        return false;
    }
    
    // Generate MAC address from ESP32 chip ID
    // Use the WiFi MAC as base and modify it slightly
    uint8 wifi_mac[6];
    WiFi.macAddress(wifi_mac);
    
    // Create a unique MAC for the emulated Ethernet
    // Use locally administered address (bit 1 of first byte set)
    ether_addr[0] = 0x02;  // Locally administered, unicast
    ether_addr[1] = 'B';   // Basilisk identifier
    ether_addr[2] = wifi_mac[2];
    ether_addr[3] = wifi_mac[3];
    ether_addr[4] = wifi_mac[4];
    ether_addr[5] = wifi_mac[5];
    
    Serial.printf("[ETHER] MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  ether_addr[0], ether_addr[1], ether_addr[2],
                  ether_addr[3], ether_addr[4], ether_addr[5]);
    
    // Start network receive task on Core 0
    net_rx_task_running = true;
    BaseType_t result = xTaskCreatePinnedToCore(
        net_rx_task,
        "net_rx",
        4096,           // Stack size
        nullptr,        // Parameters
        1,              // Priority
        &net_rx_task_handle,
        0               // Core 0
    );
    
    if (result != pdPASS) {
        Serial.println("[ETHER] Failed to create network RX task");
        router_exit();
        vSemaphoreDelete(net_mutex);
        net_mutex = nullptr;
        return false;
    }
    
    net_initialized = true;
    Serial.println("[ETHER] Network driver initialized");
    return true;
}

/*
 *  Deinitialize network driver
 */
void ether_exit(void)
{
    Serial.println("[ETHER] Shutting down network driver...");
    
    // Stop receive task
    if (net_rx_task_handle != nullptr) {
        net_rx_task_running = false;
        vTaskDelay(pdMS_TO_TICKS(100));  // Give task time to exit
        vTaskDelete(net_rx_task_handle);
        net_rx_task_handle = nullptr;
    }
    
    // Shutdown router
    router_exit();
    
    // Delete mutex
    if (net_mutex != nullptr) {
        vSemaphoreDelete(net_mutex);
        net_mutex = nullptr;
    }
    
    // Clear protocol handlers
    protocol_handlers.clear();
    
    net_initialized = false;
    
    Serial.printf("[ETHER] Stats: sent=%lu, received=%lu\n", packets_sent, packets_received);
    Serial.println("[ETHER] Network driver shut down");
}

/*
 *  Reset network driver
 */
void ether_reset(void)
{
    D(bug("[ETHER] Reset\n"));

    if (!net_initialized || net_mutex == nullptr) {
        return;
    }

    if (xSemaphoreTake(net_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        protocol_handlers.clear();
        xSemaphoreGive(net_mutex);
    }
}

/*
 *  Add multicast address
 *  In NAT mode, we don't need real multicast support
 */
int16 ether_add_multicast(uint32 pb)
{
    D(bug("[ETHER] Add multicast\n"));
    return noErr;  // Silently succeed
}

/*
 *  Delete multicast address
 */
int16 ether_del_multicast(uint32 pb)
{
    D(bug("[ETHER] Delete multicast\n"));
    return noErr;  // Silently succeed
}

/*
 *  Attach protocol handler
 */
int16 ether_attach_ph(uint16 type, uint32 handler)
{
    D(bug("[ETHER] Attach protocol handler type=%04x handler=%08x\n", type, handler));
    
    if (!net_initialized || net_mutex == nullptr) {
        return lapProtErr;
    }
    
    if (xSemaphoreTake(net_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return lapProtErr;
    }
    
    // Check if already attached
    if (protocol_handlers.find(type) != protocol_handlers.end()) {
        xSemaphoreGive(net_mutex);
        return lapProtErr;
    }
    
    protocol_handlers[type] = handler;
    xSemaphoreGive(net_mutex);
    
    return noErr;
}

/*
 *  Detach protocol handler
 */
int16 ether_detach_ph(uint16 type)
{
    D(bug("[ETHER] Detach protocol handler type=%04x\n", type));
    
    if (!net_initialized || net_mutex == nullptr) {
        return lapProtErr;
    }
    
    if (xSemaphoreTake(net_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return lapProtErr;
    }
    
    if (protocol_handlers.erase(type) == 0) {
        xSemaphoreGive(net_mutex);
        return lapProtErr;
    }
    
    xSemaphoreGive(net_mutex);
    return noErr;
}

/*
 *  Transmit raw ethernet packet
 */
int16 ether_write(uint32 wds)
{
    D(bug("[ETHER] Write packet, wds=%08x\n", wds));
    
    if (!net_initialized) {
        return excessCollsns;
    }
    
    // Gracefully handle WiFi having dropped mid-session
    if (WiFi.status() != WL_CONNECTED) {
        return excessCollsns;
    }
    
    // Convert WDS to linear buffer
    uint8 packet[1514];
    int len = ether_wds_to_buffer(wds, packet);
    
    if (len < 14) {
        D(bug("[ETHER] Packet too short: %d bytes\n", len));
        return eLenErr;
    }
    
    D(bug("[ETHER] Sending %d byte packet\n", len));
    
    // Pass to router for NAT processing
    if (router_write_packet(packet, len)) {
        packets_sent++;
        return noErr;
    }
    
    return excessCollsns;
}

/*
 *  Start UDP packet reception thread
 *  Not used in NAT mode, but required by interface
 */
bool ether_start_udp_thread(int socket_fd)
{
    return false;  // UDP tunnel not supported
}

/*
 *  Stop UDP packet reception thread
 */
void ether_stop_udp_thread(void)
{
    // Nothing to do
}

/*
 *  Ethernet interrupt - called from emulator main loop
 *  Delivers received packets to MacOS
 */
void EtherInterrupt(void)
{
    if (!net_initialized) {
        return;
    }
    
    // Check for pending packets
    if (!router_has_pending_packets()) {
        return;
    }
    
    D(bug("[ETHER] EtherInterrupt\n"));
    
    // Allocate packet buffer in MacOS memory
    EthernetPacket ether_packet;
    uint32 packet = ether_packet.addr();
    
    // Dequeue packets and deliver to MacOS
    uint8 buffer[1514];
    int len;
    
    while ((len = router_dequeue_packet(buffer, sizeof(buffer))) > 0) {
        if (len < 14) {
            continue;  // Too short
        }
        
        D(bug("[ETHER] Received %d byte packet\n", len));
        
        // Copy packet to MacOS memory
        Host2Mac_memcpy(packet, buffer, len);
        
        // Get protocol type from Ethernet header (bytes 12-13)
        uint16 type = (buffer[12] << 8) | buffer[13];
        
        // Look up protocol handler
        // For 802.3 frames (length <= 1500), use type 0
        uint16 search_type = (type <= 1500) ? 0 : type;
        
        if (xSemaphoreTake(net_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            continue;
        }
        
        auto it = protocol_handlers.find(search_type);
        if (it == protocol_handlers.end()) {
            xSemaphoreGive(net_mutex);
            D(bug("[ETHER] No handler for protocol %04x\n", search_type));
            continue;
        }
        
        uint32 handler = it->second;
        xSemaphoreGive(net_mutex);
        
        if (handler == 0) {
            continue;
        }
        
        packets_received++;
        
        // Copy header to RHA (Read Header Area)
        Mac2Mac_memcpy(ether_data + ed_RHA, packet, 14);
        
        // Call protocol handler with appropriate registers
        M68kRegisters r;
        r.d[0] = type;                          // Packet type
        r.d[1] = len - 14;                      // Remaining length (without header)
        r.a[0] = packet + 14;                   // Pointer to packet data (after header)
        r.a[3] = ether_data + ed_RHA + 14;      // Pointer behind header in RHA
        r.a[4] = ether_data + ed_ReadPacket;    // Pointer to ReadPacket/ReadRest routines
        
        D(bug("[ETHER] Calling handler %08x, type=%04x, len=%d\n", handler, type, len - 14));
        Execute68k(handler, &r);
    }
}

/*
 *  Network receive task - runs on Core 0
 *  Polls the router for incoming data and signals the emulator
 */
static void net_rx_task(void *param)
{
    Serial.println("[ETHER] Network RX task started");
    
    while (net_rx_task_running) {
        // If WiFi has dropped, sleep longer to avoid busy-looping on dead sockets
        if (WiFi.status() != WL_CONNECTED) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        
        // Poll the router for incoming packets
        router_poll();
        
        // If packets are available, signal the emulator
        if (router_has_pending_packets()) {
            SetInterruptFlag(INTFLAG_ETHER);
            TriggerInterrupt();
        }
        
        // Small delay to avoid hogging CPU
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    Serial.println("[ETHER] Network RX task stopped");
    vTaskDelete(nullptr);
}
