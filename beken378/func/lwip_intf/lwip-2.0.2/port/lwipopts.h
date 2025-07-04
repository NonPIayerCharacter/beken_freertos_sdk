/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#include "sys_config.h"

/**
 * Loopback demo related options.
 */
#define LWIP_NETIF_LOOPBACK             1
#define LWIP_HAVE_LOOPIF                1
#define LWIP_NETIF_LOOPBACK_MULTITHREADING       1
#define LWIP_LOOPBACK_MAX_PBUFS         8

#define TCPIP_THREAD_NAME               "tcp/ip"
#if (CFG_SUPPORT_MATTER)
#define TCPIP_THREAD_STACKSIZE          1024
#else
#define TCPIP_THREAD_STACKSIZE          512
#endif

#if CFG_OS_FREERTOS
#define TCPIP_THREAD_PRIO               2
#else
#define TCPIP_THREAD_PRIO               7
#endif

#define DEFAULT_THREAD_STACKSIZE        200
#if CFG_OS_FREERTOS
#define DEFAULT_THREAD_PRIO             8
#else
#define DEFAULT_THREAD_PRIO             1
#endif

/* Disable lwIP asserts */
//#define LWIP_NOASSERT			        1

#define LWIP_DEBUG                      0
#define LWIP_DEBUG_TRACE                0
#define SOCKETS_DEBUG                   LWIP_DBG_OFF
#define IP_DEBUG                        LWIP_DBG_OFF
#define ETHARP_DEBUG                    LWIP_DBG_OFF
#define NETIF_DEBUG                     LWIP_DBG_OFF
#define PBUF_DEBUG                      LWIP_DBG_OFF
#define MEMP_DEBUG                      LWIP_DBG_OFF
#define API_LIB_DEBUG                   LWIP_DBG_OFF
#define API_MSG_DEBUG                   LWIP_DBG_OFF
#define ICMP_DEBUG                      LWIP_DBG_OFF
#define IGMP_DEBUG                      LWIP_DBG_OFF
#define INET_DEBUG                      LWIP_DBG_OFF
#define IP_REASS_DEBUG                  LWIP_DBG_OFF
#define RAW_DEBUG                       LWIP_DBG_OFF
#define MEM_DEBUG                       LWIP_DBG_OFF
#define SYS_DEBUG                       LWIP_DBG_OFF
#define TCP_DEBUG                       LWIP_DBG_OFF
#define TCP_INPUT_DEBUG                 LWIP_DBG_OFF
#define TCP_FR_DEBUG                    LWIP_DBG_OFF
#define TCP_RTO_DEBUG                   LWIP_DBG_OFF
#define TCP_CWND_DEBUG                  LWIP_DBG_OFF
#define TCP_WND_DEBUG                   LWIP_DBG_OFF
#define TCP_OUTPUT_DEBUG                LWIP_DBG_OFF
#define TCP_RST_DEBUG                   LWIP_DBG_OFF
#define TCP_QLEN_DEBUG                  LWIP_DBG_OFF
#define UDP_DEBUG                       LWIP_DBG_OFF
#define TCPIP_DEBUG                     LWIP_DBG_OFF
#define PPP_DEBUG                       LWIP_DBG_OFF
#define SLIP_DEBUG                      LWIP_DBG_OFF
#define DHCP_DEBUG                      LWIP_DBG_OFF
#define AUTOIP_DEBUG                    LWIP_DBG_OFF
#define SNMP_MSG_DEBUG                  LWIP_DBG_OFF
#define SNMP_MIB_DEBUG                  LWIP_DBG_OFF
#define DNS_DEBUG                       LWIP_DBG_OFF
#define IP6_DEBUG                       LWIP_DBG_OFF

//#define LWIP_COMPAT_MUTEX      		    1
/**
 * SYS_LIGHTWEIGHT_PROT==1: if you want inter-task protection for certain
 * critical regions during buffer allocation, deallocation and memory
 * allocation and deallocation.
 */
#define SYS_LIGHTWEIGHT_PROT            1

/*
   ------------------------------------
   ---------- Memory options ----------
   ------------------------------------
*/

/**
 * MEM_ALIGNMENT: should be set to the alignment of the CPU
 *    4 byte alignment -> #define MEM_ALIGNMENT 4
 *    2 byte alignment -> #define MEM_ALIGNMENT 2
 */
#define MEM_ALIGNMENT                   4

#define MAX_SOCKETS_TCP 8
#define MAX_LISTENING_SOCKETS_TCP 6
#define MAX_SOCKETS_UDP 10
#define TCP_SND_BUF_COUNT 5

/* Value of TCP_SND_BUF_COUNT denotes the number of buffers and is set by
 * CONFIG option available in the SDK
 */
/* Buffer size needed for TCP: Max. number of TCP sockets * Size of pbuf *
 * Max. number of TCP sender buffers per socket
 *
 * Listening sockets for TCP servers do not require the same amount buffer
 * space. Hence do not consider these sockets for memory computation
 */
#define TCP_MEM_SIZE     (MAX_SOCKETS_TCP * \
							PBUF_POOL_BUFSIZE * (TCP_SND_BUF/TCP_MSS))

/* Buffer size needed for UDP: Max. number of UDP sockets * Size of pbuf
 */
#define UDP_MEM_SIZE (MAX_SOCKETS_UDP * PBUF_POOL_BUFSIZE)

/**
 * MEM_SIZE: the size of the heap memory. If the application will send
 * a lot of data that needs to be copied, this should be set high.
 */
#if ((defined(CFG_LWIP_MEM_POLICY))&&(CFG_LWIP_MEM_POLICY == LWIP_REDUCE_THE_PLAN))
#define MEM_SIZE (16*1024)
#else
#define MEM_SIZE (32*1024)
#endif


/*
   ------------------------------------------------
   ---------- Internal Memory Pool Sizes ----------
   ------------------------------------------------
*/
/**
 * MEMP_NUM_PBUF: the number of memp struct pbufs (used for PBUF_ROM and PBUF_REF).
 * If the application sends a lot of data out of ROM (or other static memory),
 * this should be set high.
 */
#if (CFG_SUPPORT_MATTER)
#define MEMP_NUM_PBUF                   16
#else
#define MEMP_NUM_PBUF                   10
#endif

/**
 * MEMP_NUM_TCP_PCB: the number of simulatenously active TCP connections.
 * (requires the LWIP_TCP option)
 */
#define MEMP_NUM_TCP_PCB                MAX_SOCKETS_TCP

#define MEMP_NUM_TCP_PCB_LISTEN         MAX_LISTENING_SOCKETS_TCP

/**
 * MEMP_NUM_TCP_SEG: the number of simultaneously queued TCP segments.
 * (requires the LWIP_TCP option)
 */
//#define MEMP_NUM_TCP_SEG                12

/**
 * MEMP_NUM_TCPIP_MSG_INPKT: the number of struct tcpip_msg, which are used
 * for incoming packets. 
 * (only needed if you use tcpip.c)
 */

#define MEMP_NUM_TCPIP_MSG_INPKT        16

/**
 * MEMP_NUM_SYS_TIMEOUT: the number of simulateously active timeouts.
 * (requires NO_SYS==0)
 */
#define MEMP_NUM_SYS_TIMEOUT            12

/**
 * MEMP_NUM_NETBUF: the number of struct netbufs.
 * (only needed if you use the sequential API, like api_lib.c)
 */

#define MEMP_NUM_NETBUF                 16

/**
 * MEMP_NUM_NETCONN: the number of struct netconns.
 * (only needed if you use the sequential API, like api_lib.c)
 *
 * This number corresponds to the maximum number of active sockets at any
 * given point in time. This number must be sum of max. TCP sockets, max. TCP
 * sockets used for listening, and max. number of UDP sockets
 */
#define MEMP_NUM_NETCONN	(MAX_SOCKETS_TCP + \
	MAX_LISTENING_SOCKETS_TCP + MAX_SOCKETS_UDP)



/**
 * PBUF_POOL_SIZE: the number of buffers in the pbuf pool.
 */
#if ((defined(CFG_LWIP_MEM_POLICY))&&(CFG_LWIP_MEM_POLICY == LWIP_REDUCE_THE_PLAN))
#if (CFG_SUPPORT_MATTER)
#define PBUF_POOL_SIZE                  12
#else
#define PBUF_POOL_SIZE                  3
#endif
#else
#if CFG_IPERF_TEST_ACCEL
#define PBUF_POOL_SIZE                  16
#else
#define PBUF_POOL_SIZE                  10
#endif
#endif

/*
   ----------------------------------
   ---------- Pbuf options ----------
   ----------------------------------
*/

/**
 * PBUF_POOL_BUFSIZE: the size of each pbuf in the pbuf pool. The default is
 * designed to accomodate single full size TCP frame in one pbuf, including
 * TCP_MSS, IP header, and link header.
 */
#define PBUF_POOL_BUFSIZE               1580

/*
   ---------------------------------
   ---------- RAW options ----------
   ---------------------------------
*/
/**
 * LWIP_RAW==1: Enable application layer to hook into the IP layer itself.
 */
#define LWIP_RAW                        1
#if (CFG_SUPPORT_MATTER)
#define LWIP_IPV6                        1
#endif

/* Enable IPv4 Auto IP	*/
#ifdef CONFIG_AUTOIP
#define LWIP_AUTOIP                     1
#define LWIP_DHCP_AUTOIP_COOP           1
#define LWIP_DHCP_AUTOIP_COOP_TRIES		5
#endif

/*
   ------------------------------------
   ---------- Socket options ----------
   ------------------------------------
*/
/**
 * LWIP_SOCKET==1: Enable Socket API (require to use sockets.c)
 */
#define LWIP_SOCKET                     1
#define LWIP_NETIF_API			1

/**
 * LWIP_RECV_CB==1: Enable callback when a socket receives data.
 */
#define LWIP_RECV_CB                1
/**
 * SO_REUSE==1: Enable SO_REUSEADDR option.
 */
#define SO_REUSE                        1
#define SO_REUSE_RXTOALL 				1

/**
 * Enable TCP_KEEPALIVE
 */
#define LWIP_TCP_KEEPALIVE              1

/*
   ----------------------------------------
   ---------- Statistics options ----------
   ----------------------------------------
*/
/**
 * LWIP_STATS==1: Enable statistics collection in lwip_stats.
 */
#define LWIP_STATS                      1

/**
 * LWIP_STATS_DISPLAY==1: Compile in the statistics output functions.
 */
#define LWIP_STATS_DISPLAY              0

/* Disable lwIP asserts */
#define LWIP_NOASSERT

/* Disable lwIP error log */
#define LWIP_ERROR(message, expression, handler)

/*
   ----------------------------------
   ---------- DHCP options ----------
   ----------------------------------
*/
/**
 * LWIP_DHCP==1: Enable DHCP module.
 */
#define LWIP_DHCP                       1
#define LWIP_NETIF_STATUS_CALLBACK      1

/**
 * DNS related options, revisit later to fine tune.
 */
#define LWIP_DNS                        1
#define DNS_TABLE_SIZE                  2  // number of table entries, default 4
//#define DNS_MAX_NAME_LENGTH           64  // max. name length, default 256
#define DNS_MAX_SERVERS                 2  // number of DNS servers, default 2
#define DNS_DOES_NAME_CHECK             1  // compare received name with given,def 0 
#define DNS_MSG_SIZE                    512
#define MDNS_MSG_SIZE                   512
#define MDNS_TABLE_SIZE                 1  // number of mDNS table entries
#define MDNS_MAX_SERVERS                1  // number of mDNS multicast addresses
/* TODO: Number of active UDP PCBs is equal to number of active UDP sockets plus
 * two. Need to find the users of these 2 PCBs
 */
#define MEMP_NUM_UDP_PCB		(MAX_SOCKETS_UDP + 2)
/* NOTE: some times the socket() call for SOCK_DGRAM might fail if you dont
 * have enough MEMP_NUM_UDP_PCB */

/*
   ----------------------------------
   ---------- IGMP options ----------
   ----------------------------------
*/
/**
 * LWIP_IGMP==1: Turn on IGMP module.
 */
#define LWIP_IGMP                       1

/**
 * LWIP_SO_SNDTIMEO==1: Enable send timeout for sockets/netconns and
 * SO_SNDTIMEO processing.
 */
#define LWIP_SO_SNDTIMEO                1

/**
 * LWIP_SO_RCVTIMEO==1: Enable receive timeout for sockets/netconns and
 * SO_RCVTIMEO processing.
 */
#define LWIP_SO_RCVTIMEO                1
#define LWIP_SO_SNDTIMEO                1
/**
 * TCP_LISTEN_BACKLOG==1: Handle backlog connections.
 */
#define TCP_LISTEN_BACKLOG		        1
#define LWIP_PROVIDE_ERRNO		        1 

#include <errno.h>
#define ERRNO				            1

//#define LWIP_SNMP 1


/*
   ------------------------------------------------
   ---------- Network Interfaces options ----------
   ------------------------------------------------
*/
/**
 * LWIP_NETIF_HOSTNAME==1: use DHCP_OPTION_HOSTNAME with netif's hostname
 * field.
 */
#define LWIP_NETIF_HOSTNAME             1


/*
The STM32F107 allows computing and verifying the IP, UDP, TCP and ICMP checksums by hardware:
 - To use this feature let the following define uncommented.
 - To disable it and process by CPU comment the  the checksum.
*/
//#define CHECKSUM_BY_HARDWARE


#ifdef CHECKSUM_BY_HARDWARE
  /* CHECKSUM_GEN_IP==0: Generate checksums by hardware for outgoing IP packets.*/
  #define CHECKSUM_GEN_IP                 0
  /* CHECKSUM_GEN_UDP==0: Generate checksums by hardware for outgoing UDP packets.*/
  #define CHECKSUM_GEN_UDP                0
  /* CHECKSUM_GEN_TCP==0: Generate checksums by hardware for outgoing TCP packets.*/
  #define CHECKSUM_GEN_TCP                0
  /* CHECKSUM_CHECK_IP==0: Check checksums by hardware for incoming IP packets.*/
  #define CHECKSUM_CHECK_IP               0
  /* CHECKSUM_CHECK_UDP==0: Check checksums by hardware for incoming UDP packets.*/
  #define CHECKSUM_CHECK_UDP              0
  /* CHECKSUM_CHECK_TCP==0: Check checksums by hardware for incoming TCP packets.*/
  #define CHECKSUM_CHECK_TCP              0
#else
  /* CHECKSUM_GEN_IP==1: Generate checksums in software for outgoing IP packets.*/
  #define CHECKSUM_GEN_IP                 1
  /* CHECKSUM_GEN_UDP==1: Generate checksums in software for outgoing UDP packets.*/
  #define CHECKSUM_GEN_UDP                1
  /* CHECKSUM_GEN_TCP==1: Generate checksums in software for outgoing TCP packets.*/
  #define CHECKSUM_GEN_TCP                1
  /* CHECKSUM_CHECK_IP==1: Check checksums in software for incoming IP packets.*/
  #define CHECKSUM_CHECK_IP               1
  /* CHECKSUM_CHECK_UDP==1: Check checksums in software for incoming UDP packets.*/
  #define CHECKSUM_CHECK_UDP              1
  /* CHECKSUM_CHECK_TCP==1: Check checksums in software for incoming TCP packets.*/
  #define CHECKSUM_CHECK_TCP              1
#endif

/**
 * TCP_RESOURCE_FAIL_RETRY_LIMIT: limit for retrying sending of tcp segment
 * on resource failure error returned by driver.
 */
#define TCP_RESOURCE_FAIL_RETRY_LIMIT     50

//#ifdef CONFIG_ENABLE_MXCHIP
/* save memory */
#if ((defined(CFG_LWIP_MEM_POLICY))&&(CFG_LWIP_MEM_POLICY == LWIP_REDUCE_THE_PLAN))
#define TCP_MSS                 (1500 - 40)

/* TCP receive window. */
#define TCP_WND                 (3 * TCP_MSS)
/* TCP sender buffer space (bytes). */
#define TCP_SND_BUF             (10*TCP_MSS)

#define TCP_SND_QUEUELEN        (20)
#else
#define TCP_MSS                 (1500 - 40)

#if CFG_IPERF_TEST_ACCEL
/* TCP receive window. */
#define TCP_WND                 (16*TCP_MSS)
/* TCP sender buffer space (bytes). */
#define TCP_SND_BUF             (16*TCP_MSS)

#define TCP_SND_QUEUELEN        (32)

#else

/* TCP receive window. */
#define TCP_WND                 (10*TCP_MSS)
/* TCP sender buffer space (bytes). */
#define TCP_SND_BUF             (10*TCP_MSS)

#define TCP_SND_QUEUELEN        (20)

#endif /*end CFG_IPERF_TEST_ACCEL*/

#endif

/* ARP before DHCP causes multi-second delay  - turn it off */
#define DHCP_DOES_ARP_CHECK            (0)

#define TCP_MAX_ACCEPT_CONN 5
#define MEMP_NUM_TCP_SEG               (TCP_SND_QUEUELEN*2)

#define IP_REASS_MAX_PBUFS              0
#define IP_REASSEMBLY                   0
#define IP_REASS_MAX_PBUFS              0
#define IP_REASSEMBLY                   0
#define MEMP_NUM_REASSDATA              0
#define IP_FRAG                         0

#define MEM_LIBC_MALLOC                (1)

#define MEMP_MEM_MALLOC (0)
#define TCP_MSL (TCP_TMR_INTERVAL)

#define LWIP_COMPAT_MUTEX_ALLOWED       (1)

#if (LWIP_STATS)
#define MEMP_STATS                       1
#define MEM_STATS                        1
#endif
#define TCPIP_MBOX_SIZE                 16
#define DEFAULT_ACCEPTMBOX_SIZE         8
#define DEFAULT_RAW_RECVMBOX_SIZE       4
#define DEFAULT_UDP_RECVMBOX_SIZE       8
#define DEFAULT_TCP_RECVMBOX_SIZE       8

#define LWIP_DONT_PROVIDE_BYTEORDER_FUNCTIONS

#define ETHARP_SUPPORT_STATIC_ENTRIES   1

#define LWIP_RIPPLE20                   1

/* Beken specific LWIP options */
#define BK_DHCP                         1

#endif /* __LWIPOPTS_H__ */

