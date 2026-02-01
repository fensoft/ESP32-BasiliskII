/*
 *  net_router.cpp - NAT Router for ESP32 networking
 *
 *  BasiliskII ESP32 Port
 *  Based on Basilisk II (C) 1997-2008 Christian Bauer
 *  Windows router code (C) Lauri Pesonen
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */

#include "sysdeps.h"

#include <Arduino.h>
#include <WiFi.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <errno.h>

#include "cpu_emulation.h"
#include "main.h"
#include "ether.h"
#include "net_router.h"

#define DEBUG 0
#include "debug.h"

// ============================================================================
// Configuration
// ============================================================================

// Packet queue size
#define PACKET_QUEUE_SIZE    16
#define MAX_PACKET_SIZE      1514

// ============================================================================
// Global State
// ============================================================================

// Router's virtual MAC address
static const uint8 router_mac[6] = { 0x52, 0x54, 0x00, 0x12, 0x34, 0x56 };

// MacOS IP address (host byte order), learned from ARP
static uint32 macos_ip = MACOS_IP_ADDR;

// IP identification counter
static uint16 ip_ident = 1;

// Connection tracking table
static net_conn_t connections[MAX_NET_CONNECTIONS];

// Packet queue for delivering packets to MacOS
static QueueHandle_t rx_packet_queue = nullptr;

// Mutex for connection table
static SemaphoreHandle_t conn_mutex = nullptr;

// Packet buffer structure for queue
typedef struct {
    uint8 data[MAX_PACKET_SIZE];
    int len;
} packet_buffer_t;

// Pre-allocated packet buffers
static packet_buffer_t packet_buffers[PACKET_QUEUE_SIZE];
static int next_buffer = 0;

// Router initialized flag
static bool router_initialized = false;

// ============================================================================
// Helper Functions - Byte Order
// ============================================================================

static inline uint16 net_htons(uint16 h) { return __builtin_bswap16(h); }
static inline uint16 net_ntohs(uint16 n) { return __builtin_bswap16(n); }
static inline uint32 net_htonl(uint32 h) { return __builtin_bswap32(h); }
static inline uint32 net_ntohl(uint32 n) { return __builtin_bswap32(n); }

// ============================================================================
// Checksum Functions
// ============================================================================

static uint16 compute_checksum(uint16 *data, int len)
{
    uint32 sum = 0;
    
    while (len > 1) {
        sum += net_ntohs(*data++);
        len -= 2;
    }
    
    // Add left-over byte if any
    if (len == 1) {
        sum += (*(uint8 *)data) << 8;
    }
    
    // Fold 32-bit sum into 16-bit
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    
    return (uint16)(~sum);
}

void make_ip_checksum(ip_hdr_t *ip)
{
    ip->checksum = 0;
    int hdr_len = (ip->ver_ihl & 0x0F) * 4;
    uint16 *data = (uint16 *)((uint8 *)ip + sizeof(mac_hdr_t));
    ip->checksum = net_htons(compute_checksum(data, hdr_len));
}

void make_icmp_checksum(icmp_pkt_t *icmp, int total_len)
{
    icmp->checksum = 0;
    int icmp_len = total_len - sizeof(ip_hdr_t);
    uint16 *data = (uint16 *)((uint8 *)icmp + sizeof(ip_hdr_t));
    icmp->checksum = net_htons(compute_checksum(data, icmp_len));
}

static uint32 tcp_pseudo_checksum(tcp_pkt_t *tcp, int tcp_len)
{
    uint32 sum = 0;
    
    // Source address
    sum += (net_ntohl(tcp->ip.src) >> 16) & 0xFFFF;
    sum += net_ntohl(tcp->ip.src) & 0xFFFF;
    
    // Destination address
    sum += (net_ntohl(tcp->ip.dest) >> 16) & 0xFFFF;
    sum += net_ntohl(tcp->ip.dest) & 0xFFFF;
    
    // Protocol
    sum += IP_PROTO_TCP;
    
    // TCP length
    sum += tcp_len;
    
    return sum;
}

void make_tcp_checksum(tcp_pkt_t *tcp, int total_len)
{
    tcp->checksum = 0;
    int tcp_len = total_len - sizeof(ip_hdr_t);
    
    // Start with pseudo-header
    uint32 sum = tcp_pseudo_checksum(tcp, tcp_len);
    
    // Add TCP header and data
    uint16 *data = (uint16 *)((uint8 *)tcp + sizeof(ip_hdr_t));
    int len = tcp_len;
    
    while (len > 1) {
        sum += net_ntohs(*data++);
        len -= 2;
    }
    
    if (len == 1) {
        sum += (*(uint8 *)data) << 8;
    }
    
    // Fold
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    
    tcp->checksum = net_htons((uint16)(~sum));
}

void make_udp_checksum(udp_pkt_t *udp, int total_len)
{
    // UDP checksum is optional, set to 0
    udp->checksum = 0;
}

// ============================================================================
// Router MAC/IP Access
// ============================================================================

const uint8 *router_get_mac_addr(void)
{
    return router_mac;
}

uint32 router_get_macos_ip(void)
{
    return macos_ip;
}

void router_set_macos_ip(uint32 ip)
{
    macos_ip = ip;
}

bool router_is_connected(void)
{
    return router_initialized && (WiFi.status() == WL_CONNECTED);
}

// ============================================================================
// Connection Management
// ============================================================================

static net_conn_t *find_connection(int proto, uint16 local_port, uint16 remote_port)
{
    for (int i = 0; i < MAX_NET_CONNECTIONS; i++) {
        if (connections[i].in_use &&
            connections[i].protocol == proto &&
            connections[i].local_port == local_port &&
            connections[i].remote_port == remote_port) {
            return &connections[i];
        }
    }
    return nullptr;
}

static net_conn_t *alloc_connection(void)
{
    for (int i = 0; i < MAX_NET_CONNECTIONS; i++) {
        if (!connections[i].in_use) {
            memset(&connections[i], 0, sizeof(net_conn_t));
            connections[i].in_use = true;
            connections[i].socket_fd = -1;
            connections[i].tcp_state = TCP_STATE_CLOSED;
            connections[i].last_activity = xTaskGetTickCount() * portTICK_PERIOD_MS;
            connections[i].timeout_ms = SOCKET_TIMEOUT_MS;
            return &connections[i];
        }
    }
    return nullptr;
}

static void free_connection(net_conn_t *conn)
{
    if (conn == nullptr) return;
    
    if (conn->socket_fd >= 0) {
        close(conn->socket_fd);
        conn->socket_fd = -1;
    }
    
    if (conn->rx_buffer != nullptr) {
        free(conn->rx_buffer);
        conn->rx_buffer = nullptr;
    }
    
    conn->in_use = false;
}

static void close_expired_connections(void)
{
    uint32 now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    for (int i = 0; i < MAX_NET_CONNECTIONS; i++) {
        if (connections[i].in_use) {
            if ((now - connections[i].last_activity) > connections[i].timeout_ms) {
                D(bug("[ROUTER] Connection %d expired\n", i));
                free_connection(&connections[i]);
            }
        }
    }
}

// ============================================================================
// Packet Queue
// ============================================================================

void router_enqueue_packet(uint8 *packet, int len)
{
    if (rx_packet_queue == nullptr || len > MAX_PACKET_SIZE || len <= 0) {
        return;
    }
    
    // Get a buffer
    packet_buffer_t *buf = &packet_buffers[next_buffer];
    next_buffer = (next_buffer + 1) % PACKET_QUEUE_SIZE;
    
    // Copy data
    memcpy(buf->data, packet, len);
    buf->len = len;
    
    // Queue the buffer (non-blocking)
    if (xQueueSend(rx_packet_queue, &buf, 0) != pdTRUE) {
        D(bug("[ROUTER] Packet queue full, dropping packet\n"));
    }
}

int router_dequeue_packet(uint8 *buffer, int max_len)
{
    if (rx_packet_queue == nullptr) {
        return 0;
    }
    
    packet_buffer_t *buf = nullptr;
    if (xQueueReceive(rx_packet_queue, &buf, 0) != pdTRUE) {
        return 0;
    }
    
    if (buf == nullptr) {
        return 0;
    }
    
    int len = buf->len;
    if (len > max_len) {
        len = max_len;
    }
    
    memcpy(buffer, buf->data, len);
    return len;
}

bool router_has_pending_packets(void)
{
    if (rx_packet_queue == nullptr) {
        return false;
    }
    return uxQueueMessagesWaiting(rx_packet_queue) > 0;
}

// ============================================================================
// ARP Processing
// ============================================================================

static void handle_arp(arp_pkt_t *arp, int len)
{
    D(bug("[ROUTER] ARP packet: opcode=%d\n", net_ntohs(arp->opcode)));
    
    if (len < sizeof(arp_pkt_t)) {
        return;
    }
    
    // Only handle Ethernet/IP ARP
    if (net_ntohs(arp->htype) != 1 || net_ntohs(arp->ptype) != ETH_TYPE_IP4) {
        return;
    }
    
    if (arp->halen != 6 || arp->palen != 4) {
        return;
    }
    
    // Get source and destination IPs
    uint32 src_ip = (arp->src_ip[0] << 24) | (arp->src_ip[1] << 16) |
                    (arp->src_ip[2] << 8) | arp->src_ip[3];
    uint32 dst_ip = (arp->dst_ip[0] << 24) | (arp->dst_ip[1] << 16) |
                    (arp->dst_ip[2] << 8) | arp->dst_ip[3];
    
    D(bug("[ROUTER] ARP: src=%08X dst=%08X\n", src_ip, dst_ip));
    
    // Learn MacOS IP address from ARP requests
    if (src_ip != 0) {
        macos_ip = src_ip;
    }
    
    // Handle ARP request
    if (net_ntohs(arp->opcode) == ARP_REQUEST) {
        // If asking about the router/gateway, reply with our MAC
        if (dst_ip == ROUTER_IP_ADDR || dst_ip == ROUTER_DNS_ADDR ||
            (dst_ip & ROUTER_NET_MASK) != (ROUTER_NET_ADDR & ROUTER_NET_MASK)) {
            
            // Build ARP reply
            arp_pkt_t reply;
            
            // MAC header
            memcpy(reply.mac.dest, ether_addr, 6);
            memcpy(reply.mac.src, router_mac, 6);
            reply.mac.type = net_htons(ETH_TYPE_ARP);
            
            // ARP
            reply.htype = net_htons(1);
            reply.ptype = net_htons(ETH_TYPE_IP4);
            reply.halen = 6;
            reply.palen = 4;
            reply.opcode = net_htons(ARP_REPLY);
            
            // Sender = router
            memcpy(reply.src_hw, router_mac, 6);
            reply.src_ip[0] = (dst_ip >> 24) & 0xFF;
            reply.src_ip[1] = (dst_ip >> 16) & 0xFF;
            reply.src_ip[2] = (dst_ip >> 8) & 0xFF;
            reply.src_ip[3] = dst_ip & 0xFF;
            
            // Target = MacOS
            memcpy(reply.dst_hw, ether_addr, 6);
            memcpy(reply.dst_ip, arp->src_ip, 4);
            
            D(bug("[ROUTER] Sending ARP reply\n"));
            router_enqueue_packet((uint8 *)&reply, sizeof(reply));
        }
    }
}

// ============================================================================
// ICMP Processing
// ============================================================================

static void send_icmp_reply(icmp_pkt_t *request, int len)
{
    // Allocate reply packet
    uint8 *reply_buf = (uint8 *)malloc(len);
    if (reply_buf == nullptr) return;
    
    icmp_pkt_t *reply = (icmp_pkt_t *)reply_buf;
    memcpy(reply, request, len);
    
    // Swap MAC addresses
    memcpy(reply->ip.mac.dest, ether_addr, 6);
    memcpy(reply->ip.mac.src, router_mac, 6);
    
    // Swap IP addresses
    reply->ip.src = request->ip.dest;
    reply->ip.dest = request->ip.src;
    
    // Set ICMP type to echo reply
    reply->type = ICMP_ECHO_REPLY;
    reply->code = 0;
    
    // Recalculate checksums
    make_icmp_checksum(reply, len);
    make_ip_checksum(&reply->ip);
    
    router_enqueue_packet(reply_buf, len);
    free(reply_buf);
}

static void handle_icmp(icmp_pkt_t *icmp, int len)
{
    D(bug("[ROUTER] ICMP packet: type=%d code=%d\n", icmp->type, icmp->code));
    
    if (len < sizeof(icmp_pkt_t)) {
        return;
    }
    
    uint32 dest_ip = net_ntohl(icmp->ip.dest);
    
    // Check if pinging the gateway
    if (dest_ip == ROUTER_IP_ADDR || dest_ip == ROUTER_DNS_ADDR) {
        if (icmp->type == ICMP_ECHO_REQUEST) {
            D(bug("[ROUTER] Responding to ping to gateway\n"));
            send_icmp_reply(icmp, len);
        }
        return;
    }
    
    // Forward ping to external host
    if (icmp->type == ICMP_ECHO_REQUEST) {
        D(bug("[ROUTER] Forwarding ping to %08X\n", dest_ip));
        
        // Create raw ICMP socket
        int sock = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
        if (sock < 0) {
            D(bug("[ROUTER] Failed to create raw socket: %d\n", errno));
            // Can't do raw sockets, send simulated reply instead
            send_icmp_reply(icmp, len);
            return;
        }
        
        // Set destination
        struct sockaddr_in dest;
        memset(&dest, 0, sizeof(dest));
        dest.sin_family = AF_INET;
        dest.sin_addr.s_addr = net_htonl(dest_ip);
        
        // Send ICMP portion (without MAC and IP headers, but lwIP will add IP)
        uint8 *icmp_data = (uint8 *)icmp + sizeof(ip_hdr_t);
        int icmp_len = len - sizeof(ip_hdr_t);
        
        // Set TTL
        int ttl = icmp->ip.ttl - 1;
        if (ttl <= 0) {
            close(sock);
            return;
        }
        setsockopt(sock, IPPROTO_IP, IP_TTL, &ttl, sizeof(ttl));
        
        // Send
        sendto(sock, icmp_data, icmp_len, 0, (struct sockaddr *)&dest, sizeof(dest));
        
        // Set socket non-blocking for receive
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);
        
        // Wait for reply with timeout
        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        
        // Receive reply
        uint8 recv_buf[1500];
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int recv_len = recvfrom(sock, recv_buf, sizeof(recv_buf), 0,
                                (struct sockaddr *)&from, &from_len);
        
        close(sock);
        
        if (recv_len > 0) {
            D(bug("[ROUTER] Received ICMP reply, %d bytes\n", recv_len));
            
            // Build response packet for MacOS
            int total_len = sizeof(ip_hdr_t) + recv_len;
            uint8 *reply_buf = (uint8 *)malloc(total_len);
            if (reply_buf != nullptr) {
                icmp_pkt_t *reply = (icmp_pkt_t *)reply_buf;
                
                // MAC header
                memcpy(reply->ip.mac.dest, ether_addr, 6);
                memcpy(reply->ip.mac.src, router_mac, 6);
                reply->ip.mac.type = net_htons(ETH_TYPE_IP4);
                
                // IP header
                reply->ip.ver_ihl = 0x45;
                reply->ip.tos = 0;
                reply->ip.total_len = net_htons(total_len - sizeof(mac_hdr_t));
                reply->ip.ident = net_htons(ip_ident++);
                reply->ip.flags_frag = 0;
                reply->ip.ttl = 64;
                reply->ip.proto = IP_PROTO_ICMP;
                reply->ip.src = net_htonl(dest_ip);
                reply->ip.dest = net_htonl(macos_ip);
                
                // Copy ICMP data
                memcpy(reply_buf + sizeof(ip_hdr_t), recv_buf, recv_len);
                
                make_ip_checksum(&reply->ip);
                
                router_enqueue_packet(reply_buf, total_len);
                free(reply_buf);
            }
        }
    }
}

// ============================================================================
// DHCP Server
// ============================================================================

static uint8 get_dhcp_message_type(uint8 *options, int options_len)
{
    int i = 0;
    while (i < options_len) {
        uint8 opt = options[i];
        if (opt == DHCP_OPT_END) break;
        if (opt == DHCP_OPT_PAD) {
            i++;
            continue;
        }
        if (i + 1 >= options_len) break;
        uint8 opt_len = options[i + 1];
        if (opt == DHCP_OPT_MSG_TYPE && opt_len >= 1) {
            return options[i + 2];
        }
        i += 2 + opt_len;
    }
    return 0;
}

static void handle_dhcp(dhcp_pkt_t *dhcp, int len)
{
    // Verify it's a DHCP request (BOOTREQUEST)
    if (dhcp->op != 1) return;  // Not a request
    
    // Verify magic cookie
    if (net_ntohl(dhcp->magic) != DHCP_MAGIC_COOKIE) return;
    
    // Get options area
    int options_offset = sizeof(dhcp_pkt_t) - sizeof(udp_pkt_t);
    int options_len = len - sizeof(dhcp_pkt_t);
    if (options_len <= 0) return;
    
    uint8 *options = (uint8 *)dhcp + sizeof(dhcp_pkt_t);
    uint8 msg_type = get_dhcp_message_type(options, options_len);
    
    D(bug("[DHCP] Received DHCP message type %d\n", msg_type));
    
    if (msg_type != DHCP_DISCOVER && msg_type != DHCP_REQUEST) {
        return;  // Only handle DISCOVER and REQUEST
    }
    
    // Learn MAC address from DHCP request
    // (We'll use this to set macos_ip once they accept the lease)
    
    // Build DHCP response
    int reply_options_len = 64;  // Enough for our options
    int reply_len = sizeof(dhcp_pkt_t) + reply_options_len;
    uint8 *reply_buf = (uint8 *)malloc(reply_len);
    if (reply_buf == nullptr) return;
    
    memset(reply_buf, 0, reply_len);
    dhcp_pkt_t *reply = (dhcp_pkt_t *)reply_buf;
    
    // MAC header - broadcast for OFFER, unicast for ACK
    memcpy(reply->udp.ip.mac.dest, ether_addr, 6);  // Send to requesting client
    memcpy(reply->udp.ip.mac.src, router_mac, 6);
    reply->udp.ip.mac.type = net_htons(ETH_TYPE_IP4);
    
    // IP header
    reply->udp.ip.ver_ihl = 0x45;
    reply->udp.ip.tos = 0;
    reply->udp.ip.total_len = net_htons(reply_len - sizeof(mac_hdr_t));
    reply->udp.ip.ident = net_htons(ip_ident++);
    reply->udp.ip.flags_frag = 0;
    reply->udp.ip.ttl = 64;
    reply->udp.ip.proto = IP_PROTO_UDP;
    reply->udp.ip.src = net_htonl(ROUTER_IP_ADDR);
    reply->udp.ip.dest = net_htonl(MACOS_IP_ADDR);  // Offer this IP
    
    // UDP header
    reply->udp.src_port = net_htons(DHCP_SERVER_PORT);
    reply->udp.dest_port = net_htons(DHCP_CLIENT_PORT);
    reply->udp.len = net_htons(reply_len - sizeof(ip_hdr_t));
    reply->udp.checksum = 0;  // Optional for UDP
    
    // DHCP fields
    reply->op = 2;  // BOOTREPLY
    reply->htype = 1;  // Ethernet
    reply->hlen = 6;
    reply->hops = 0;
    reply->xid = dhcp->xid;  // Same transaction ID
    reply->secs = 0;
    reply->flags = 0;
    reply->ciaddr = 0;
    reply->yiaddr = net_htonl(MACOS_IP_ADDR);  // Offered IP
    reply->siaddr = net_htonl(ROUTER_IP_ADDR);  // Server IP
    reply->giaddr = 0;
    memcpy(reply->chaddr, dhcp->chaddr, 16);  // Client MAC
    reply->magic = net_htonl(DHCP_MAGIC_COOKIE);
    
    // Build options
    uint8 *opts = reply_buf + sizeof(dhcp_pkt_t);
    int opt_idx = 0;
    
    // Message type (OFFER or ACK)
    opts[opt_idx++] = DHCP_OPT_MSG_TYPE;
    opts[opt_idx++] = 1;
    opts[opt_idx++] = (msg_type == DHCP_DISCOVER) ? DHCP_OFFER : DHCP_ACK;
    
    // Server identifier
    opts[opt_idx++] = DHCP_OPT_SERVER_ID;
    opts[opt_idx++] = 4;
    opts[opt_idx++] = (ROUTER_IP_ADDR >> 24) & 0xFF;
    opts[opt_idx++] = (ROUTER_IP_ADDR >> 16) & 0xFF;
    opts[opt_idx++] = (ROUTER_IP_ADDR >> 8) & 0xFF;
    opts[opt_idx++] = ROUTER_IP_ADDR & 0xFF;
    
    // Lease time (1 day = 86400 seconds)
    opts[opt_idx++] = DHCP_OPT_LEASE_TIME;
    opts[opt_idx++] = 4;
    opts[opt_idx++] = 0x00;
    opts[opt_idx++] = 0x01;
    opts[opt_idx++] = 0x51;
    opts[opt_idx++] = 0x80;  // 86400
    
    // Subnet mask
    opts[opt_idx++] = DHCP_OPT_SUBNET_MASK;
    opts[opt_idx++] = 4;
    opts[opt_idx++] = 255;
    opts[opt_idx++] = 255;
    opts[opt_idx++] = 255;
    opts[opt_idx++] = 0;
    
    // Router/Gateway
    opts[opt_idx++] = DHCP_OPT_ROUTER;
    opts[opt_idx++] = 4;
    opts[opt_idx++] = (ROUTER_IP_ADDR >> 24) & 0xFF;
    opts[opt_idx++] = (ROUTER_IP_ADDR >> 16) & 0xFF;
    opts[opt_idx++] = (ROUTER_IP_ADDR >> 8) & 0xFF;
    opts[opt_idx++] = ROUTER_IP_ADDR & 0xFF;
    
    // DNS server
    opts[opt_idx++] = DHCP_OPT_DNS;
    opts[opt_idx++] = 4;
    opts[opt_idx++] = (ROUTER_DNS_ADDR >> 24) & 0xFF;
    opts[opt_idx++] = (ROUTER_DNS_ADDR >> 16) & 0xFF;
    opts[opt_idx++] = (ROUTER_DNS_ADDR >> 8) & 0xFF;
    opts[opt_idx++] = ROUTER_DNS_ADDR & 0xFF;
    
    // End option
    opts[opt_idx++] = DHCP_OPT_END;
    
    // Calculate IP checksum
    make_ip_checksum(&reply->udp.ip);
    
    D(bug("[DHCP] Sending DHCP %s for IP %d.%d.%d.%d\n",
          (msg_type == DHCP_DISCOVER) ? "OFFER" : "ACK",
          (MACOS_IP_ADDR >> 24) & 0xFF,
          (MACOS_IP_ADDR >> 16) & 0xFF,
          (MACOS_IP_ADDR >> 8) & 0xFF,
          MACOS_IP_ADDR & 0xFF));
    
    // Update macos_ip when we send an ACK
    if (msg_type == DHCP_REQUEST) {
        macos_ip = MACOS_IP_ADDR;
    }
    
    router_enqueue_packet(reply_buf, reply_len);
    free(reply_buf);
}

// ============================================================================
// UDP Processing
// ============================================================================

static void handle_udp(udp_pkt_t *udp, int len)
{
    if (len < sizeof(udp_pkt_t)) {
        return;
    }
    
    // Check for DHCP requests (client port 68, server port 67)
    uint16 src_port = net_ntohs(udp->src_port);
    uint16 dest_port = net_ntohs(udp->dest_port);
    
    if (src_port == DHCP_CLIENT_PORT && dest_port == DHCP_SERVER_PORT) {
        if (len >= sizeof(dhcp_pkt_t)) {
            handle_dhcp((dhcp_pkt_t *)udp, len);
        }
        return;
    }
    
    uint32 dest_ip = net_ntohl(udp->ip.dest);
    
    D(bug("[ROUTER] UDP: %d -> %08X:%d\n", src_port, dest_ip, dest_port));
    
    // Get data
    uint8 *data = (uint8 *)udp + sizeof(udp_pkt_t);
    int data_len = net_ntohs(udp->len) - 8;  // UDP header is 8 bytes
    
    if (data_len <= 0 || data_len > len - sizeof(udp_pkt_t)) {
        return;
    }
    
    // Handle DNS queries to our virtual DNS server
    if (dest_ip == ROUTER_DNS_ADDR && dest_port == 53) {
        // Forward to real DNS server
        dest_ip = WiFi.dnsIP().operator uint32_t();
        if (dest_ip == 0) {
            dest_ip = 0x08080808;  // Google DNS fallback
        }
        dest_ip = net_ntohl(dest_ip);  // Convert to host byte order
    }
    
    // Find or create connection
    if (xSemaphoreTake(conn_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    net_conn_t *conn = find_connection(IPPROTO_UDP, src_port, dest_port);
    if (conn == nullptr) {
        conn = alloc_connection();
        if (conn == nullptr) {
            xSemaphoreGive(conn_mutex);
            D(bug("[ROUTER] No free connections\n"));
            return;
        }
        
        // Create UDP socket
        conn->socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (conn->socket_fd < 0) {
            free_connection(conn);
            xSemaphoreGive(conn_mutex);
            D(bug("[ROUTER] Failed to create UDP socket\n"));
            return;
        }
        
        // Set non-blocking
        int flags = fcntl(conn->socket_fd, F_GETFL, 0);
        fcntl(conn->socket_fd, F_SETFL, flags | O_NONBLOCK);
        
        conn->protocol = IPPROTO_UDP;
        conn->local_ip = macos_ip;
        conn->remote_ip = dest_ip;
        conn->local_port = src_port;
        conn->remote_port = dest_port;
    }
    
    conn->last_activity = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xSemaphoreGive(conn_mutex);
    
    // Send data
    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = net_htons(dest_port);
    dest_addr.sin_addr.s_addr = net_htonl(dest_ip);
    
    int sent = sendto(conn->socket_fd, data, data_len, 0,
                      (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    
    D(bug("[ROUTER] UDP sent %d bytes\n", sent));
}

static void poll_udp_connection(net_conn_t *conn)
{
    if (conn->socket_fd < 0) return;
    
    uint8 recv_buf[1500];
    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);
    
    int recv_len = recvfrom(conn->socket_fd, recv_buf, sizeof(recv_buf), 0,
                            (struct sockaddr *)&from, &from_len);
    
    if (recv_len <= 0) {
        return;
    }
    
    D(bug("[ROUTER] UDP received %d bytes\n", recv_len));
    
    conn->last_activity = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Build UDP packet for MacOS
    int total_len = sizeof(udp_pkt_t) + recv_len;
    uint8 *pkt_buf = (uint8 *)malloc(total_len);
    if (pkt_buf == nullptr) return;
    
    udp_pkt_t *reply = (udp_pkt_t *)pkt_buf;
    
    // MAC header
    memcpy(reply->ip.mac.dest, ether_addr, 6);
    memcpy(reply->ip.mac.src, router_mac, 6);
    reply->ip.mac.type = net_htons(ETH_TYPE_IP4);
    
    // IP header
    reply->ip.ver_ihl = 0x45;
    reply->ip.tos = 0;
    reply->ip.total_len = net_htons(total_len - sizeof(mac_hdr_t));
    reply->ip.ident = net_htons(ip_ident++);
    reply->ip.flags_frag = 0;
    reply->ip.ttl = 64;
    reply->ip.proto = IP_PROTO_UDP;
    reply->ip.src = net_htonl(conn->remote_ip);
    reply->ip.dest = net_htonl(conn->local_ip);
    
    // UDP header
    reply->src_port = net_htons(conn->remote_port);
    reply->dest_port = net_htons(conn->local_port);
    reply->len = net_htons(recv_len + 8);
    reply->checksum = 0;
    
    // Copy data
    memcpy(pkt_buf + sizeof(udp_pkt_t), recv_buf, recv_len);
    
    make_ip_checksum(&reply->ip);
    
    router_enqueue_packet(pkt_buf, total_len);
    free(pkt_buf);
}

// ============================================================================
// TCP Processing
// ============================================================================

static void send_tcp_packet(net_conn_t *conn, uint8 flags, uint8 *data, int data_len)
{
    int total_len = sizeof(tcp_pkt_t) + data_len;
    uint8 *pkt_buf = (uint8 *)malloc(total_len);
    if (pkt_buf == nullptr) return;
    
    tcp_pkt_t *tcp = (tcp_pkt_t *)pkt_buf;
    
    // MAC header
    memcpy(tcp->ip.mac.dest, ether_addr, 6);
    memcpy(tcp->ip.mac.src, router_mac, 6);
    tcp->ip.mac.type = net_htons(ETH_TYPE_IP4);
    
    // IP header
    tcp->ip.ver_ihl = 0x45;
    tcp->ip.tos = 0;
    tcp->ip.total_len = net_htons(total_len - sizeof(mac_hdr_t));
    tcp->ip.ident = net_htons(ip_ident++);
    tcp->ip.flags_frag = 0;
    tcp->ip.ttl = 64;
    tcp->ip.proto = IP_PROTO_TCP;
    tcp->ip.src = net_htonl(conn->remote_ip);
    tcp->ip.dest = net_htonl(conn->local_ip);
    
    // TCP header
    tcp->src_port = net_htons(conn->remote_port);
    tcp->dest_port = net_htons(conn->local_port);
    tcp->seq = net_htonl(conn->seq_out);
    tcp->ack = net_htonl(conn->seq_in);
    tcp->data_off = 0x50;  // 5 * 4 = 20 bytes header, no options
    tcp->flags = flags;
    tcp->window = net_htons(MAX_SEGMENT_SIZE);
    tcp->urgent = 0;
    
    // Copy data if any
    if (data != nullptr && data_len > 0) {
        memcpy(pkt_buf + sizeof(tcp_pkt_t), data, data_len);
    }
    
    make_tcp_checksum(tcp, total_len);
    make_ip_checksum(&tcp->ip);
    
    router_enqueue_packet(pkt_buf, total_len);
    free(pkt_buf);
}

static void handle_tcp(tcp_pkt_t *tcp, int len)
{
    if (len < sizeof(tcp_pkt_t)) {
        return;
    }
    
    uint16 src_port = net_ntohs(tcp->src_port);
    uint16 dest_port = net_ntohs(tcp->dest_port);
    uint32 dest_ip = net_ntohl(tcp->ip.dest);
    uint32 seq = net_ntohl(tcp->seq);
    uint32 ack = net_ntohl(tcp->ack);
    uint8 flags = tcp->flags;
    
    int header_len = (tcp->data_off >> 4) * 4;
    uint8 *data = (uint8 *)tcp + sizeof(ip_hdr_t) + header_len;
    int data_len = net_ntohs(tcp->ip.total_len) - ((tcp->ip.ver_ihl & 0x0F) * 4) - header_len;
    
    if (data_len < 0) data_len = 0;
    
    D(bug("[ROUTER] TCP: %d -> %08X:%d, flags=%02X, seq=%u, ack=%u, data=%d\n",
          src_port, dest_ip, dest_port, flags, seq, ack, data_len));
    
    // Handle RST
    if (flags & TCP_FLAG_RST) {
        if (xSemaphoreTake(conn_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            net_conn_t *conn = find_connection(IPPROTO_TCP, src_port, dest_port);
            if (conn != nullptr) {
                D(bug("[ROUTER] TCP RST, closing connection\n"));
                free_connection(conn);
            }
            xSemaphoreGive(conn_mutex);
        }
        return;
    }
    
    if (xSemaphoreTake(conn_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    net_conn_t *conn = find_connection(IPPROTO_TCP, src_port, dest_port);
    
    // Handle SYN (new connection)
    if ((flags & TCP_FLAG_SYN) && conn == nullptr) {
        conn = alloc_connection();
        if (conn == nullptr) {
            xSemaphoreGive(conn_mutex);
            D(bug("[ROUTER] No free connections for TCP\n"));
            return;
        }
        
        // Create TCP socket
        conn->socket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (conn->socket_fd < 0) {
            free_connection(conn);
            xSemaphoreGive(conn_mutex);
            D(bug("[ROUTER] Failed to create TCP socket\n"));
            return;
        }
        
        // Set non-blocking
        int opt_flags = fcntl(conn->socket_fd, F_GETFL, 0);
        fcntl(conn->socket_fd, F_SETFL, opt_flags | O_NONBLOCK);
        
        conn->protocol = IPPROTO_TCP;
        conn->local_ip = macos_ip;
        conn->remote_ip = dest_ip;
        conn->local_port = src_port;
        conn->remote_port = dest_port;
        conn->tcp_state = TCP_STATE_LISTEN;
        conn->seq_in = seq + 1;
        conn->seq_out = 1;  // Our initial sequence number
        
        // Connect to remote host
        struct sockaddr_in dest_addr;
        memset(&dest_addr, 0, sizeof(dest_addr));
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = net_htons(dest_port);
        dest_addr.sin_addr.s_addr = net_htonl(dest_ip);
        
        int ret = connect(conn->socket_fd, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (ret < 0 && errno != EINPROGRESS) {
            D(bug("[ROUTER] TCP connect failed: %d\n", errno));
            free_connection(conn);
            xSemaphoreGive(conn_mutex);
            return;
        }
        
        conn->tcp_state = TCP_STATE_SYN_SENT;
        
        // Send SYN+ACK to MacOS immediately (optimistic)
        send_tcp_packet(conn, TCP_FLAG_SYN | TCP_FLAG_ACK, nullptr, 0);
        conn->seq_out++;  // SYN consumes one sequence number
        conn->tcp_state = TCP_STATE_SYN_RCVD;
        
        xSemaphoreGive(conn_mutex);
        return;
    }
    
    if (conn == nullptr) {
        xSemaphoreGive(conn_mutex);
        return;
    }
    
    conn->last_activity = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // State machine
    switch (conn->tcp_state) {
        case TCP_STATE_SYN_RCVD:
            if (flags & TCP_FLAG_ACK) {
                conn->tcp_state = TCP_STATE_ESTABLISHED;
                D(bug("[ROUTER] TCP ESTABLISHED\n"));
            }
            break;
            
        case TCP_STATE_ESTABLISHED:
            // Handle data
            if (data_len > 0) {
                // Send to remote host
                int sent = send(conn->socket_fd, data, data_len, 0);
                if (sent > 0) {
                    conn->seq_in += sent;
                    D(bug("[ROUTER] TCP sent %d bytes to remote\n", sent));
                }
                // ACK the data
                send_tcp_packet(conn, TCP_FLAG_ACK, nullptr, 0);
            }
            
            // Handle FIN
            if (flags & TCP_FLAG_FIN) {
                conn->seq_in++;
                send_tcp_packet(conn, TCP_FLAG_ACK, nullptr, 0);
                shutdown(conn->socket_fd, SHUT_WR);
                conn->tcp_state = TCP_STATE_CLOSE_WAIT;
                
                // Send our FIN
                send_tcp_packet(conn, TCP_FLAG_FIN | TCP_FLAG_ACK, nullptr, 0);
                conn->seq_out++;
                conn->tcp_state = TCP_STATE_LAST_ACK;
            }
            break;
            
        case TCP_STATE_CLOSE_WAIT:
            // Waiting for MacOS to close
            break;
            
        case TCP_STATE_LAST_ACK:
            if (flags & TCP_FLAG_ACK) {
                D(bug("[ROUTER] TCP closed\n"));
                free_connection(conn);
            }
            break;
            
        case TCP_STATE_FIN_WAIT_1:
            if ((flags & TCP_FLAG_FIN) && (flags & TCP_FLAG_ACK)) {
                conn->seq_in++;
                send_tcp_packet(conn, TCP_FLAG_ACK, nullptr, 0);
                conn->tcp_state = TCP_STATE_TIME_WAIT;
            } else if (flags & TCP_FLAG_FIN) {
                conn->seq_in++;
                send_tcp_packet(conn, TCP_FLAG_ACK, nullptr, 0);
                conn->tcp_state = TCP_STATE_CLOSING;
            } else if (flags & TCP_FLAG_ACK) {
                conn->tcp_state = TCP_STATE_FIN_WAIT_2;
            }
            break;
            
        case TCP_STATE_FIN_WAIT_2:
            if (flags & TCP_FLAG_FIN) {
                conn->seq_in++;
                send_tcp_packet(conn, TCP_FLAG_ACK, nullptr, 0);
                conn->tcp_state = TCP_STATE_TIME_WAIT;
            }
            break;
            
        case TCP_STATE_CLOSING:
            if (flags & TCP_FLAG_ACK) {
                conn->tcp_state = TCP_STATE_TIME_WAIT;
            }
            break;
            
        case TCP_STATE_TIME_WAIT:
            // Will be cleaned up by timeout
            break;
            
        default:
            break;
    }
    
    xSemaphoreGive(conn_mutex);
}

static void poll_tcp_connection(net_conn_t *conn)
{
    if (conn->socket_fd < 0) return;
    if (conn->tcp_state != TCP_STATE_ESTABLISHED &&
        conn->tcp_state != TCP_STATE_CLOSE_WAIT) return;
    
    uint8 recv_buf[MAX_SEGMENT_SIZE];
    int recv_len = recv(conn->socket_fd, recv_buf, sizeof(recv_buf), 0);
    
    if (recv_len > 0) {
        D(bug("[ROUTER] TCP received %d bytes from remote\n", recv_len));
        
        conn->last_activity = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Send data to MacOS
        send_tcp_packet(conn, TCP_FLAG_ACK | TCP_FLAG_PSH, recv_buf, recv_len);
        conn->seq_out += recv_len;
    } else if (recv_len == 0) {
        // Remote closed
        D(bug("[ROUTER] TCP remote closed\n"));
        
        send_tcp_packet(conn, TCP_FLAG_FIN | TCP_FLAG_ACK, nullptr, 0);
        conn->seq_out++;
        conn->tcp_state = TCP_STATE_FIN_WAIT_1;
    }
}

// ============================================================================
// Main Router Functions
// ============================================================================

bool router_write_packet(uint8 *packet, int len)
{
    if (!router_initialized || len < sizeof(mac_hdr_t)) {
        return false;
    }
    
    mac_hdr_t *mac = (mac_hdr_t *)packet;
    uint16 eth_type = net_ntohs(mac->type);
    
    D(bug("[ROUTER] Write packet: type=%04X, len=%d\n", eth_type, len));
    
    switch (eth_type) {
        case ETH_TYPE_ARP:
            handle_arp((arp_pkt_t *)packet, len);
            return true;
            
        case ETH_TYPE_IP4: {
            ip_hdr_t *ip = (ip_hdr_t *)packet;
            uint8 proto = ip->proto;
            
            switch (proto) {
                case IP_PROTO_ICMP:
                    handle_icmp((icmp_pkt_t *)packet, len);
                    return true;
                    
                case IP_PROTO_TCP:
                    handle_tcp((tcp_pkt_t *)packet, len);
                    return true;
                    
                case IP_PROTO_UDP:
                    handle_udp((udp_pkt_t *)packet, len);
                    return true;
                    
                default:
                    D(bug("[ROUTER] Unknown IP protocol: %d\n", proto));
                    return false;
            }
        }
        
        default:
            D(bug("[ROUTER] Unknown Ethernet type: %04X\n", eth_type));
            return false;
    }
}

void router_poll(void)
{
    if (!router_initialized) return;
    
    if (xSemaphoreTake(conn_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    
    // Poll all active connections
    for (int i = 0; i < MAX_NET_CONNECTIONS; i++) {
        if (connections[i].in_use) {
            switch (connections[i].protocol) {
                case IPPROTO_UDP:
                    poll_udp_connection(&connections[i]);
                    break;
                    
                case IPPROTO_TCP:
                    poll_tcp_connection(&connections[i]);
                    break;
            }
        }
    }
    
    // Close expired connections periodically
    static uint32 last_cleanup = 0;
    uint32 now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - last_cleanup > 5000) {
        close_expired_connections();
        last_cleanup = now;
    }
    
    xSemaphoreGive(conn_mutex);
}

bool router_init(void)
{
    Serial.println("[ROUTER] Initializing NAT router...");
    
    // Create packet queue
    rx_packet_queue = xQueueCreate(PACKET_QUEUE_SIZE, sizeof(packet_buffer_t *));
    if (rx_packet_queue == nullptr) {
        Serial.println("[ROUTER] Failed to create packet queue");
        return false;
    }
    
    // Create connection mutex
    conn_mutex = xSemaphoreCreateMutex();
    if (conn_mutex == nullptr) {
        Serial.println("[ROUTER] Failed to create connection mutex");
        vQueueDelete(rx_packet_queue);
        rx_packet_queue = nullptr;
        return false;
    }
    
    // Initialize connection table
    memset(connections, 0, sizeof(connections));
    for (int i = 0; i < MAX_NET_CONNECTIONS; i++) {
        connections[i].socket_fd = -1;
    }
    
    router_initialized = true;
    
    Serial.printf("[ROUTER] Virtual network: %d.%d.%d.%d/24\n",
                  (ROUTER_NET_ADDR >> 24) & 0xFF,
                  (ROUTER_NET_ADDR >> 16) & 0xFF,
                  (ROUTER_NET_ADDR >> 8) & 0xFF,
                  ROUTER_NET_ADDR & 0xFF);
    Serial.printf("[ROUTER] Gateway: %d.%d.%d.%d\n",
                  (ROUTER_IP_ADDR >> 24) & 0xFF,
                  (ROUTER_IP_ADDR >> 16) & 0xFF,
                  (ROUTER_IP_ADDR >> 8) & 0xFF,
                  ROUTER_IP_ADDR & 0xFF);
    Serial.printf("[ROUTER] MacOS IP: %d.%d.%d.%d\n",
                  (MACOS_IP_ADDR >> 24) & 0xFF,
                  (MACOS_IP_ADDR >> 16) & 0xFF,
                  (MACOS_IP_ADDR >> 8) & 0xFF,
                  MACOS_IP_ADDR & 0xFF);
    
    Serial.println("[ROUTER] NAT router initialized");
    return true;
}

void router_exit(void)
{
    Serial.println("[ROUTER] Shutting down NAT router...");
    
    router_initialized = false;
    
    // Close all connections
    if (conn_mutex != nullptr && xSemaphoreTake(conn_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        for (int i = 0; i < MAX_NET_CONNECTIONS; i++) {
            if (connections[i].in_use) {
                free_connection(&connections[i]);
            }
        }
        xSemaphoreGive(conn_mutex);
    }
    
    // Delete mutex
    if (conn_mutex != nullptr) {
        vSemaphoreDelete(conn_mutex);
        conn_mutex = nullptr;
    }
    
    // Delete queue
    if (rx_packet_queue != nullptr) {
        vQueueDelete(rx_packet_queue);
        rx_packet_queue = nullptr;
    }
    
    Serial.println("[ROUTER] NAT router shut down");
}
