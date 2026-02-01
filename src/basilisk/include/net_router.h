/*
 *  net_router.h - NAT Router for ESP32 networking
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

#ifndef NET_ROUTER_H
#define NET_ROUTER_H

#include "sysdeps.h"

// ============================================================================
// Network Configuration
// ============================================================================

// Virtual network addresses (host byte order)
#define ROUTER_NET_ADDR      0x0A000200  // 10.0.2.0/24
#define ROUTER_NET_MASK      0xFFFFFF00  // 255.255.255.0
#define ROUTER_IP_ADDR       0x0A000202  // 10.0.2.2 (gateway)
#define ROUTER_DNS_ADDR      0x0A000203  // 10.0.2.3 (DNS)
#define MACOS_IP_ADDR        0x0A00020F  // 10.0.2.15 (default MacOS IP)

// Maximum number of concurrent connections
#define MAX_NET_CONNECTIONS  16

// Maximum segment size for TCP
#define MAX_SEGMENT_SIZE     1460

// Socket timeout in milliseconds
#define SOCKET_TIMEOUT_MS    60000

// ============================================================================
// Packet Type Definitions
// ============================================================================

// Ethernet type codes (network byte order values)
#define ETH_TYPE_IP4   0x0800
#define ETH_TYPE_ARP   0x0806
#define ETH_TYPE_IP6   0x86DD

// IP protocol numbers
#define IP_PROTO_ICMP  1
#define IP_PROTO_TCP   6
#define IP_PROTO_UDP   17

// ICMP types
#define ICMP_ECHO_REPLY         0
#define ICMP_DEST_UNREACHABLE   3
#define ICMP_ECHO_REQUEST       8
#define ICMP_TIME_EXCEEDED      11

// ARP opcodes
#define ARP_REQUEST    1
#define ARP_REPLY      2

// TCP flags
#define TCP_FLAG_FIN   0x01
#define TCP_FLAG_SYN   0x02
#define TCP_FLAG_RST   0x04
#define TCP_FLAG_PSH   0x08
#define TCP_FLAG_ACK   0x10
#define TCP_FLAG_URG   0x20

// DHCP ports and message types
#define DHCP_SERVER_PORT   67
#define DHCP_CLIENT_PORT   68
#define DHCP_MAGIC_COOKIE  0x63825363

#define DHCP_DISCOVER      1
#define DHCP_OFFER         2
#define DHCP_REQUEST       3
#define DHCP_DECLINE       4
#define DHCP_ACK           5
#define DHCP_NAK           6
#define DHCP_RELEASE       7
#define DHCP_INFORM        8

// DHCP options
#define DHCP_OPT_PAD             0
#define DHCP_OPT_SUBNET_MASK     1
#define DHCP_OPT_ROUTER          3
#define DHCP_OPT_DNS             6
#define DHCP_OPT_HOSTNAME        12
#define DHCP_OPT_REQUESTED_IP    50
#define DHCP_OPT_LEASE_TIME      51
#define DHCP_OPT_MSG_TYPE        53
#define DHCP_OPT_SERVER_ID       54
#define DHCP_OPT_PARAM_REQUEST   55
#define DHCP_OPT_END             255

// TCP states
enum tcp_state_t {
    TCP_STATE_CLOSED,
    TCP_STATE_LISTEN,
    TCP_STATE_SYN_SENT,
    TCP_STATE_SYN_RCVD,
    TCP_STATE_ESTABLISHED,
    TCP_STATE_CLOSE_WAIT,
    TCP_STATE_LAST_ACK,
    TCP_STATE_FIN_WAIT_1,
    TCP_STATE_FIN_WAIT_2,
    TCP_STATE_CLOSING,
    TCP_STATE_TIME_WAIT
};

// ============================================================================
// Packet Header Structures (packed for network use)
// ============================================================================

#pragma pack(push, 1)

// Ethernet MAC header
typedef struct {
    uint8 dest[6];
    uint8 src[6];
    uint16 type;  // Network byte order
} mac_hdr_t;

// ARP packet
typedef struct {
    mac_hdr_t mac;
    uint16 htype;     // Hardware type (1 = Ethernet)
    uint16 ptype;     // Protocol type (0x0800 = IP)
    uint8 halen;      // Hardware address length (6)
    uint8 palen;      // Protocol address length (4)
    uint16 opcode;    // Operation (1=request, 2=reply)
    uint8 src_hw[6];  // Sender hardware address
    uint8 src_ip[4];  // Sender protocol address
    uint8 dst_hw[6];  // Target hardware address
    uint8 dst_ip[4];  // Target protocol address
} arp_pkt_t;

// IPv4 header
typedef struct {
    mac_hdr_t mac;
    uint8 ver_ihl;    // Version (4 bits) + IHL (4 bits)
    uint8 tos;        // Type of service
    uint16 total_len; // Total length
    uint16 ident;     // Identification
    uint16 flags_frag;// Flags (3 bits) + Fragment offset (13 bits)
    uint8 ttl;        // Time to live
    uint8 proto;      // Protocol
    uint16 checksum;  // Header checksum
    uint32 src;       // Source address
    uint32 dest;      // Destination address
} ip_hdr_t;

// ICMP header
typedef struct {
    ip_hdr_t ip;
    uint8 type;
    uint8 code;
    uint16 checksum;
    uint16 ident;
    uint16 seq;
    // Data follows
} icmp_pkt_t;

// UDP header
typedef struct {
    ip_hdr_t ip;
    uint16 src_port;
    uint16 dest_port;
    uint16 len;
    uint16 checksum;
    // Data follows
} udp_pkt_t;

// TCP header
typedef struct {
    ip_hdr_t ip;
    uint16 src_port;
    uint16 dest_port;
    uint32 seq;
    uint32 ack;
    uint8 data_off;   // Data offset (4 bits) + Reserved (4 bits)
    uint8 flags;
    uint16 window;
    uint16 checksum;
    uint16 urgent;
    // Options + Data follow
} tcp_pkt_t;

// DHCP packet structure
typedef struct {
    udp_pkt_t udp;
    uint8 op;           // 1 = BOOTREQUEST, 2 = BOOTREPLY
    uint8 htype;        // Hardware type (1 = Ethernet)
    uint8 hlen;         // Hardware address length (6)
    uint8 hops;         // Hops
    uint32 xid;         // Transaction ID
    uint16 secs;        // Seconds elapsed
    uint16 flags;       // Flags
    uint32 ciaddr;      // Client IP address
    uint32 yiaddr;      // 'Your' (client) IP address
    uint32 siaddr;      // Server IP address
    uint32 giaddr;      // Gateway IP address
    uint8 chaddr[16];   // Client hardware address
    uint8 sname[64];    // Server host name
    uint8 file[128];    // Boot file name
    uint32 magic;       // Magic cookie (0x63825363)
    // Options follow (variable length)
} dhcp_pkt_t;

#pragma pack(pop)

// ============================================================================
// Connection Tracking Structure
// ============================================================================

typedef struct {
    bool in_use;
    int socket_fd;
    int protocol;           // IPPROTO_TCP, IPPROTO_UDP, or IPPROTO_ICMP
    
    // Connection addresses (host byte order)
    uint32 local_ip;        // MacOS IP
    uint32 remote_ip;       // Internet destination
    uint16 local_port;      // MacOS port
    uint16 remote_port;     // Remote port
    
    // TCP state machine
    tcp_state_t tcp_state;
    uint32 seq_in;          // Next expected sequence from MacOS
    uint32 seq_out;         // Next sequence to send to MacOS
    uint32 ack_out;         // Last ACK sent to MacOS
    uint16 remote_window;   // Remote host's receive window
    
    // Timeouts
    uint32 last_activity;   // Tick count of last activity
    uint32 timeout_ms;      // Connection timeout
    
    // Receive buffer for reassembly
    uint8 *rx_buffer;
    int rx_buffer_len;
    int rx_data_len;
} net_conn_t;

// ============================================================================
// Router Interface Functions
// ============================================================================

// Initialize the network router
bool router_init(void);

// Shutdown the network router
void router_exit(void);

// Process an outgoing packet from MacOS
// Returns true if the packet was handled (should not be passed through)
bool router_write_packet(uint8 *packet, int len);

// Poll for incoming packets and deliver to MacOS
// This should be called periodically from the network task
void router_poll(void);

// Enqueue a packet to be delivered to MacOS
// Thread-safe, can be called from network task
void router_enqueue_packet(uint8 *packet, int len);

// Dequeue a packet for delivery to MacOS
// Returns packet length, or 0 if no packet available
int router_dequeue_packet(uint8 *buffer, int max_len);

// Check if there are packets pending for MacOS
bool router_has_pending_packets(void);

// ============================================================================
// Helper Functions
// ============================================================================

// Compute IP header checksum
void make_ip_checksum(ip_hdr_t *ip);

// Compute ICMP checksum
void make_icmp_checksum(icmp_pkt_t *icmp, int len);

// Compute TCP checksum
void make_tcp_checksum(tcp_pkt_t *tcp, int len);

// Compute UDP checksum (optional for UDP)
void make_udp_checksum(udp_pkt_t *udp, int len);

// Get the router's MAC address
const uint8 *router_get_mac_addr(void);

// Get the MacOS IP address (host byte order)
uint32 router_get_macos_ip(void);

// Set the MacOS IP address (host byte order)
void router_set_macos_ip(uint32 ip);

// Check if network is connected
bool router_is_connected(void);

#endif // NET_ROUTER_H
