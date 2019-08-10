/**
 * @file
 * SNTP client module
 */

/*
 * Copyright (c) 2007-2009 Frédéric Bernon, Simon Goldschmidt
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
 * Author: Frédéric Bernon, Simon Goldschmidt
 */


/**
 * @defgroup sntp SNTP
 * @ingroup apps
 *
 * This is simple "SNTP" client for the lwIP raw API.
 * It is a minimal implementation of SNTPv4 as specified in RFC 4330.
 *
 * For a list of some public NTP servers, see this link :
 * http://support.ntp.org/bin/view/Servers/NTPPoolServers
 *
 * @todo:
 * - set/change servers at runtime
 * - complete SNTP_CHECK_RESPONSE checks 3 and 4
 */

#include "lwip/apps/sntp.h"

#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "lwip/udp.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/dhcp.h"

#include <string.h>
#include <time.h>

#if LWIP_UDP

/* Handle support for more than one server via SNTP_MAX_SERVERS */
#if SNTP_MAX_SERVERS > 1
#define SNTP_SUPPORT_MULTIPLE_SERVERS 1
#else /* NTP_MAX_SERVERS > 1 */
#define SNTP_SUPPORT_MULTIPLE_SERVERS 0
#endif /* NTP_MAX_SERVERS > 1 */

#if (SNTP_UPDATE_DELAY < 15000) && !defined(SNTP_SUPPRESS_DELAY_CHECK)
#error "SNTPv4 RFC 4330 enforces a minimum update time of 15 seconds (define SNTP_SUPPRESS_DELAY_CHECK to disable this error)!"
#endif

/* Configure behaviour depending on microsecond or second precision */
#ifdef SNTP_SET_SYSTEM_TIME_US
#define SNTP_CALC_TIME_US           1
#define SNTP_RECEIVE_TIME_SIZE      2
#else
#define SNTP_SET_SYSTEM_TIME_US(sec, us)
#define SNTP_CALC_TIME_US           0
#define SNTP_RECEIVE_TIME_SIZE      1
#endif


/* the various debug levels for this file */
#define SNTP_DEBUG_TRACE        (SNTP_DEBUG | LWIP_DBG_TRACE)
#define SNTP_DEBUG_STATE        (SNTP_DEBUG | LWIP_DBG_STATE)
#define SNTP_DEBUG_WARN         (SNTP_DEBUG | LWIP_DBG_LEVEL_WARNING)
#define SNTP_DEBUG_WARN_STATE   (SNTP_DEBUG | LWIP_DBG_LEVEL_WARNING | LWIP_DBG_STATE)
#define SNTP_DEBUG_SERIOUS      (SNTP_DEBUG | LWIP_DBG_LEVEL_SERIOUS)

#define SNTP_ERR_KOD                1

/* SNTP protocol defines */
#define SNTP_MSG_LEN                48

#define SNTP_OFFSET_LI_VN_MODE      0
#define SNTP_LI_MASK                0xC0
#define SNTP_LI_NO_WARNING          0x00
#define SNTP_LI_LAST_MINUTE_61_SEC  0x01
#define SNTP_LI_LAST_MINUTE_59_SEC  0x02
#define SNTP_LI_ALARM_CONDITION     0x03 /* (clock not synchronized) */

#define SNTP_VERSION_MASK           0x38
#define SNTP_VERSION                (4/* NTP Version 4*/<<3)

#define SNTP_MODE_MASK              0x07
#define SNTP_MODE_CLIENT            0x03
#define SNTP_MODE_SERVER            0x04
#define SNTP_MODE_BROADCAST         0x05

#define SNTP_OFFSET_STRATUM         1
#define SNTP_STRATUM_KOD            0x00

#define SNTP_OFFSET_ORIGINATE_TIME  24
#define SNTP_OFFSET_RECEIVE_TIME    32
#define SNTP_OFFSET_TRANSMIT_TIME   40

/* number of seconds between 1900 and 1970 (MSB=1)*/
#define DIFF_SEC_1900_1970         (2208988800UL)
/* number of seconds between 1970 and Feb 7, 2036 (6:28:16 UTC) (MSB=0) */
#define DIFF_SEC_1970_2036         (2085978496UL)

/**
 * SNTP packet format (without optional fields)
 * Timestamps are coded as 64 bits:
 * - 32 bits seconds since Jan 01, 1970, 00:00
 * - 32 bits seconds fraction (0-padded)
 * For future use, if the MSB in the seconds part is set, seconds are based
 * on Feb 07, 2036, 06:28:16.
 */
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct sntp_msg {
  PACK_STRUCT_FLD_8(u8_t  li_vn_mode);
  PACK_STRUCT_FLD_8(u8_t  stratum);
  PACK_STRUCT_FLD_8(u8_t  poll);
  PACK_STRUCT_FLD_8(u8_t  precision);
  PACK_STRUCT_FIELD(u32_t root_delay);
  PACK_STRUCT_FIELD(u32_t root_dispersion);
  PACK_STRUCT_FIELD(u32_t reference_identifier);
  PACK_STRUCT_FIELD(u32_t reference_timestamp[2]);
  PACK_STRUCT_FIELD(u32_t originate_timestamp[2]);
  PACK_STRUCT_FIELD(u32_t receive_timestamp[2]);
  PACK_STRUCT_FIELD(u32_t transmit_timestamp[2]);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

/* function prototypes */
static void sntp_request(void *arg);

/** The operating mode */
static u8_t sntp_opmode;

/** The UDP pcb used by the SNTP client */
static struct udp_pcb* sntp_pcb;
/** Names/Addresses of servers */
struct sntp_server {
#if SNTP_SERVER_DNS
  char* name;
#endif /* SNTP_SERVER_DNS */
  ip_addr_t addr;
};
static struct sntp_server sntp_servers[SNTP_MAX_SERVERS];

#if SNTP_GET_SERVERS_FROM_DHCP
static u8_t sntp_set_servers_from_dhcp;
#endif /* SNTP_GET_SERVERS_FROM_DHCP */
#if SNTP_SUPPORT_MULTIPLE_SERVERS
/** The currently used server (initialized to 0) */
static u8_t sntp_current_server;
#else /* SNTP_SUPPORT_MULTIPLE_SERVERS */
#define sntp_current_server 0
#endif /* SNTP_SUPPORT_MULTIPLE_SERVERS */

#if SNTP_RETRY_TIMEOUT_EXP
#define SNTP_RESET_RETRY_TIMEOUT() sntp_retry_timeout = SNTP_RETRY_TIMEOUT
/** Retry time, initialized with SNTP_RETRY_TIMEOUT and doubled with each retry. */
static u32_t sntp_retry_timeout;
#else /* SNTP_RETRY_TIMEOUT_EXP */
#define SNTP_RESET_RETRY_TIMEOUT()
#define sntp_retry_timeout SNTP_RETRY_TIMEOUT
#endif /* SNTP_RETRY_TIMEOUT_EXP */

#if SNTP_CHECK_RESPONSE >= 1
/** Saves the last server address to compare with response */
static ip_addr_t sntp_last_server_address;
#endif /* SNTP_CHECK_RESPONSE >= 1 */

#if SNTP_CHECK_RESPONSE >= 2
/** Saves the last timestamp sent (which is sent back by the server)
 * to compare against in response */
static u32_t sntp_last_timestamp_sent[2];
#endif /* SNTP_CHECK_RESPONSE >= 2 */

/**
 * SNTP processing of received timestamp
 */
static void
sntp_process(u32_t *receive_timestamp)
{
  /* convert SNTP time (1900-based) to unix GMT time (1970-based)
   * if MSB is 0, SNTP time is 2036-based!
   */
  u32_t rx_secs = lwip_ntohl(receive_timestamp[0]);
  int is_1900_based = ((rx_secs & 0x80000000) != 0);
  u32_t t = is_1900_based ? (rx_secs - DIFF_SEC_1900_1970) : (rx_secs + DIFF_SEC_1970_2036);
  time_t tim = t;

#if SNTP_CALC_TIME_US
  u32_t us = lwip_ntohl(receive_timestamp[1]) / 4295;
  SNTP_SET_SYSTEM_TIME_US(t, us);
  /* display local time from GMT time */
  LWIP_DEBUGF(SNTP_DEBUG_TRACE, ("sntp_process: %s, %"U32_F" us", ctime(&tim), us));

#else /* SNTP_CALC_TIME_US */

  /* change system time and/or the update the RTC clock */
  SNTP_SET_SYSTEM_TIME(t);
  /* display local time from GMT time */
  LWIP_DEBUGF(SNTP_DEBUG_TRACE, ("sntp_process: %s", ctime(&tim)));
#endif /* SNTP_CALC_TIME_US */
  LWIP_UNUSED_ARG(tim);
}

/**
 * Initialize request struct to be sent to server.
 */
static void
sntp_initialize_request(struct sntp_msg *req)
{
  memset(req, 0, SNTP_MSG_LEN);
  req->li_vn_mode = SNTP_LI_NO_WARNING | SNTP_VERSION | SNTP_MODE_CLIENT;

#if SNTP_CHECK_RESPONSE >= 2
  {
    u32_t sntp_time_sec, sntp_time_us;
    /* fill in transmit timestamp and save it in 'sntp_last_timestamp_sent' */
    SNTP_GET_SYSTEM_TIME(sntp_time_sec, sntp_time_us);
    sntp_last_timestamp_sent[0] = lwip_htonl(sntp_time_sec + DIFF_SEC_1900_1970);
    req->transmit_timestamp[0] = sntp_last_timestamp_sent[0];
    /* we send/save us instead of fraction to be faster... */
    sntp_last_timestamp_sent[1] = lwip_htonl(sntp_time_us);
    req->transmit_timestamp[1] = sntp_last_timestamp_sent[1];
  }
#endif /* SNTP_CHECK_RESPONSE >= 2 */
}

/**
 * Retry: send a new request (and increase retry timeout).
 *
 * @param arg is unused (only necessary to conform to sys_timeout)
 */
static void
sntp_retry(void* arg)
{
  LWIP_UNUSED_ARG(arg);

  LWIP_DEBUGF(SNTP_DEBUG_STATE, ("sntp_retry: Next request will be sent in %"U32_F" ms\n",
    sntp_retry_timeout));

  /* set up a timer to send a retry and increase the retry delay */
  sys_timeout(sntp_retry_timeout, sntp_request, NULL);

#if SNTP_RETRY_TIMEOUT_EXP
  {
    u32_t new_retry_timeout;
    /* increase the timeout for next retry */
    new_retry_timeout = sntp_retry_timeout << 1;
    /* limit to maximum timeout and prevent overflow */
    if ((new_retry_timeout <= SNTP_RETRY_TIMEOUT_MAX) &&
        (new_retry_timeout > sntp_retry_timeout)) {
      sntp_retry_timeout = new_retry_timeout;
    }
  }
#endif /* SNTP_RETRY_TIMEOUT_EXP */
}

#if SNTP_SUPPORT_MULTIPLE_SERVERS
/**
 * If Kiss-of-Death is received (or another packet parsing error),
 * try the next server or retry the current server and increase the retry
 * timeout if only one server is available.
 * (implicitly, SNTP_MAX_SERVERS > 1)
 *
 * @param arg is unused (only necessary to conform to sys_timeout)
 */
static void
sntp_try_next_server(void* arg)
{
  u8_t old_server, i;
  LWIP_UNUSED_ARG(arg);

  old_server = sntp_current_server;
  for (i = 0; i < SNTP_MAX_SERVERS - 1; i++) {
    sntp_current_server++;
    if (sntp_current_server >= SNTP_MAX_SERVERS) {
      sntp_current_server = 0;
    }
    if (!ip_addr_isany(&sntp_servers[sntp_current_server].addr)
#if SNTP_SERVER_DNS
        || (sntp_servers[sntp_current_server].name != NULL)
#endif
        ) {
      LWIP_DEBUGF(SNTP_DEBUG_STATE, ("sntp_try_next_server: Sending request to server %"U16_F"\n",
        (u16_t)sntp_current_server));
      /* new server: reset retry timeout */
      SNTP_RESET_RETRY_TIMEOUT();
      /* instantly send a request to the next server */
      sntp_request(NULL);
      return;
    }
  }
  /* no other valid server found */
  sntp_current_server = old_server;
  sntp_retry(NULL);
}
#else /* SNTP_SUPPORT_MULTIPLE_SERVERS */
/* Always retry on error if only one server is supported */
#define sntp_try_next_server    sntp_retry
#endif /* SNTP_SUPPORT_MULTIPLE_SERVERS */

/** UDP recv callback for the sntp pcb */
static void
sntp_recv(void *arg, struct udp_pcb* pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  u8_t mode;
  u8_t stratum;
  u32_t receive_timestamp[SNTP_RECEIVE_TIME_SIZE];
  err_t err;

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(pcb);

  /* packet received: stop retry timeout  */
  sys_untimeout(sntp_try_next_server, NULL);
  sys_untimeout(sntp_request, NULL);

  err = ERR_ARG;
#if SNTP_CHECK_RESPONSE >= 1
  /* check server address and port */
  if (((sntp_opmode != SNTP_OPMODE_POLL) || ip_addr_cmp(addr, &sntp_last_server_address)) &&
    (port == SNTP_PORT))
#else /* SNTP_CHECK_RESPONSE >= 1 */
  LWIP_UNUSED_ARG(addr);
  LWIP_UNUSED_ARG(port);
#endif /* SNTP_CHECK_RESPONSE >= 1 */
  {
    /* process the response */
    if (p->tot_len == SNTP_MSG_LEN) {
      pbuf_copy_partial(p, &mode, 1, SNTP_OFFSET_LI_VN_MODE);
      mode &= SNTP_MODE_MASK;
      /* if this is a SNTP response... */
      if (((sntp_opmode == SNTP_OPMODE_POLL) && (mode == SNTP_MODE_SERVER)) ||
          ((sntp_opmode == SNTP_OPMODE_LISTENONLY) && (mode == SNTP_MODE_BROADCAST))) {
        pbuf_copy_partial(p, &stratum, 1, SNTP_OFFSET_STRATUM);
        if (stratum == SNTP_STRATUM_KOD) {
          /* Kiss-of-death packet. Use another server or increase UPDATE_DELAY. */
          err = SNTP_ERR_KOD;
          LWIP_DEBUGF(SNTP_DEBUG_STATE, ("sntp_recv: Received Kiss-of-Death\n"));
        } else {
#if SNTP_CHECK_RESPONSE >= 2
          /* check originate_timetamp against sntp_last_timestamp_sent */
          u32_t originate_timestamp[2];
          pbuf_copy_partial(p, &originate_timestamp, 8, SNTP_OFFSET_ORIGINATE_TIME);
          if ((originate_timestamp[0] != sntp_last_timestamp_sent[0]) ||
              (originate_timestamp[1] != sntp_last_timestamp_sent[1]))
          {
            LWIP_DEBUGF(SNTP_DEBUG_WARN, ("sntp_recv: Invalid originate timestamp in response\n"));
          } else
#endif /* SNTP_CHECK_RESPONSE >= 2 */
          /* @todo: add code for SNTP_CHECK_RESPONSE >= 3 and >= 4 here */
          {
            /* correct answer */
            err = ERR_OK;
            pbuf_copy_partial(p, &receive_timestamp, SNTP_RECEIVE_TIME_SIZE * 4, SNTP_OFFSET_TRANSMIT_TIME);
          }
        }
      } else {
        LWIP_DEBUGF(SNTP_DEBUG_WARN, ("sntp_recv: Invalid mode in response: %"U16_F"\n", (u16_t)mode));
        /* wait for correct response */
        err = ERR_TIMEOUT;
      }
    } else {
      LWIP_DEBUGF(SNTP_DEBUG_WARN, ("sntp_recv: Invalid packet length: %"U16_F"\n", p->tot_len));
    }
  }
#if SNTP_CHECK_RESPONSE >= 1
  else {
    /* packet from wrong remote address or port, wait for correct response */
    err = ERR_TIMEOUT;
  }
#endif /* SNTP_CHECK_RESPONSE >= 1 */
  pbuf_free(p);
  if (err == ERR_OK) {
    sntp_process(receive_timestamp);

    /* Set up timeout for next request (only if poll response was received)*/
    if (sntp_opmode == SNTP_OPMODE_POLL) {
      /* Correct response, reset retry timeout */
      SNTP_RESET_RETRY_TIMEOUT();

      sys_timeout((u32_t)SNTP_UPDATE_DELAY, sntp_request, NULL);
      LWIP_DEBUGF(SNTP_DEBUG_STATE, ("sntp_recv: Scheduled next time request: %"U32_F" ms\n",
        (u32_t)SNTP_UPDATE_DELAY));
    }
  } else if (err != ERR_TIMEOUT) {
    /* Errors are only processed in case of an explicit poll response */
    if (sntp_opmode == SNTP_OPMODE_POLL) {
      if (err == SNTP_ERR_KOD) {
        /* Kiss-of-death packet. Use another server or increase UPDATE_DELAY. */
        sntp_try_next_server(NULL);
      } else {
        /* another error, try the same server again */
        sntp_retry(NULL);
      }
    }
  }
}

/** Actually send an sntp request to a server.
 *
 * @param server_addr resolved IP address of the SNTP server
 */
static void
sntp_send_request(const ip_addr_t *server_addr)
{
  struct pbuf* p;
  p = pbuf_alloc(PBUF_TRANSPORT, SNTP_MSG_LEN, PBUF_RAM);
  if (p != NULL) {
    struct sntp_msg *sntpmsg = (struct sntp_msg *)p->payload;
    LWIP_DEBUGF(SNTP_DEBUG_STATE, ("sntp_send_request: Sending request to server\n"));
    /* initialize request message */
    sntp_initialize_request(sntpmsg);
    /* send request */
    udp_sendto(sntp_pcb, p, server_addr, SNTP_PORT);
    /* free the pbuf after sending it */
    pbuf_free(p);
    /* set up receive timeout: try next server or retry on timeout */
    sys_timeout((u32_t)SNTP_RECV_TIMEOUT, sntp_try_next_server, NULL);
#if SNTP_CHECK_RESPONSE >= 1
    /* save server address to verify it in sntp_recv */
    ip_addr_set(&sntp_last_server_address, server_addr);
#endif /* SNTP_CHECK_RESPONSE >= 1 */
  } else {
    LWIP_DEBUGF(SNTP_DEBUG_SERIOUS, ("sntp_send_request: Out of memory, trying again in %"U32_F" ms\n",
      (u32_t)SNTP_RETRY_TIMEOUT));
    /* out of memory: set up a timer to send a retry */
    sys_timeout((u32_t)SNTP_RETRY_TIMEOUT, sntp_request, NULL);
  }
}

#if SNTP_SERVER_DNS
/**
 * DNS found callback when using DNS names as server address.
 */
static void
sntp_dns_found(const char* hostname, const ip_addr_t *ipaddr, void *arg)
{
  LWIP_UNUSED_ARG(hostname);
  LWIP_UNUSED_ARG(arg);

  if (ipaddr != NULL) {
    /* Address resolved, send request */
    LWIP_DEBUGF(SNTP_DEBUG_STATE, ("sntp_dns_found: Server address resolved, sending request\n"));
    sntp_send_request(ipaddr);
  } else {
    /* DNS resolving failed -> try another server */
    LWIP_DEBUGF(SNTP_DEBUG_WARN_STATE, ("sntp_dns_found: Failed to resolve server address resolved, trying next server\n"));
    sntp_try_next_server(NULL);
  }
}
#endif /* SNTP_SERVER_DNS */

/**
 * Send out an sntp request.
 *
 * @param arg is unused (only necessary to conform to sys_timeout)
 */
static void
sntp_request(void *arg)
{
  ip_addr_t sntp_server_address;
  err_t err;

  LWIP_UNUSED_ARG(arg);

  /* initialize SNTP server address */
#if SNTP_SERVER_DNS
  if (sntp_servers[sntp_current_server].name) {
    /* always resolve the name and rely on dns-internal caching & timeout */
    ip_addr_set_zero(&sntp_servers[sntp_current_server].addr);
    err = dns_gethostbyname(sntp_servers[sntp_current_server].name, &sntp_server_address,
      sntp_dns_found, NULL);
    if (err == ERR_INPROGRESS) {
      /* DNS request sent, wait for sntp_dns_found being called */
      LWIP_DEBUGF(SNTP_DEBUG_STATE, ("sntp_request: Waiting for server address to be resolved.\n"));
      return;
    } else if (err == ERR_OK) {
      sntp_servers[sntp_current_server].addr = sntp_server_address;
    }
  } else
#endif /* SNTP_SERVER_DNS */
  {
    sntp_server_address = sntp_servers[sntp_current_server].addr;
    err = (ip_addr_isany_val(sntp_server_address)) ? ERR_ARG : ERR_OK;
  }

  if (err == ERR_OK) {
    LWIP_DEBUGF(SNTP_DEBUG_TRACE, ("sntp_request: current server address is %s\n",
      ipaddr_ntoa(&sntp_server_address)));
    sntp_send_request(&sntp_server_address);
  } else {
    /* address conversion failed, try another server */
    LWIP_DEBUGF(SNTP_DEBUG_WARN_STATE, ("sntp_request: Invalid server address, trying next server.\n"));
    sys_timeout((u32_t)SNTP_RETRY_TIMEOUT, sntp_try_next_server, NULL);
  }
}

/**
 * @ingroup sntp
 * Initialize this module.
 * Send out request instantly or after SNTP_STARTUP_DELAY(_FUNC).
 */
void
sntp_init(void)
{
#ifdef SNTP_SERVER_ADDRESS
#if SNTP_SERVER_DNS
  sntp_setservername(0, SNTP_SERVER_ADDRESS);
#else
#error SNTP_SERVER_ADDRESS string not supported SNTP_SERVER_DNS==0
#endif
#endif /* SNTP_SERVER_ADDRESS */

  if (sntp_pcb == NULL) {
    sntp_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    LWIP_ASSERT("Failed to allocate udp pcb for sntp client", sntp_pcb != NULL);
    if (sntp_pcb != NULL) {
      udp_recv(sntp_pcb, sntp_recv, NULL);

      if (sntp_opmode == SNTP_OPMODE_POLL) {
        SNTP_RESET_RETRY_TIMEOUT();
#if SNTP_STARTUP_DELAY
        sys_timeout((u32_t)SNTP_STARTUP_DELAY_FUNC, sntp_request, NULL);
#else
        sntp_request(NULL);
#endif
      } else if (sntp_opmode == SNTP_OPMODE_LISTENONLY) {
        ip_set_option(sntp_pcb, SOF_BROADCAST);
        udp_bind(sntp_pcb, IP_ANY_TYPE, SNTP_PORT);
      }
    }
  }
}

/**
 * @ingroup sntp
 * Stop this module.
 */
void
sntp_stop(void)
{
  if (sntp_pcb != NULL) {
    sys_untimeout(sntp_request, NULL);
    sys_untimeout(sntp_try_next_server, NULL);
    udp_remove(sntp_pcb);
    sntp_pcb = NULL;
  }
}

/**
 * @ingroup sntp
 * Get enabled state.
 */
u8_t sntp_enabled(void)
{
  return (sntp_pcb != NULL)? 1 : 0;
}

/**
 * @ingroup sntp
 * Sets the operating mode.
 * @param operating_mode one of the available operating modes
 */
void
sntp_setoperatingmode(u8_t operating_mode)
{
  LWIP_ASSERT("Invalid operating mode", operating_mode <= SNTP_OPMODE_LISTENONLY);
  LWIP_ASSERT("Operating mode must not be set while SNTP client is running", sntp_pcb == NULL);
  sntp_opmode = operating_mode;
}

/**
 * @ingroup sntp
 * Gets the operating mode.
 */
u8_t
sntp_getoperatingmode(void)
{
  return sntp_opmode;
}

#if SNTP_GET_SERVERS_FROM_DHCP
/**
 * Config SNTP server handling by IP address, name, or DHCP; clear table
 * @param set_servers_from_dhcp enable or disable getting server addresses from dhcp
 */
void
sntp_servermode_dhcp(int set_servers_from_dhcp)
{
  u8_t new_mode = set_servers_from_dhcp ? 1 : 0;
  if (sntp_set_servers_from_dhcp != new_mode) {
    sntp_set_servers_from_dhcp = new_mode;
  }
}
#endif /* SNTP_GET_SERVERS_FROM_DHCP */

/**
 * @ingroup sntp
 * Initialize one of the NTP servers by IP address
 *
 * @param idx the index of the NTP server to set must be < SNTP_MAX_SERVERS
 * @param server IP address of the NTP server to set
 */
void
sntp_setserver(u8_t idx, const ip_addr_t *server)
{
  if (idx < SNTP_MAX_SERVERS) {
    if (server != NULL) {
      sntp_servers[idx].addr = (*server);
    } else {
      ip_addr_set_zero(&sntp_servers[idx].addr);
    }
#if SNTP_SERVER_DNS
    sntp_servers[idx].name = NULL;
#endif
  }
}

#if LWIP_DHCP && SNTP_GET_SERVERS_FROM_DHCP
/**
 * Initialize one of the NTP servers by IP address, required by DHCP
 *
 * @param numdns the index of the NTP server to set must be < SNTP_MAX_SERVERS
 * @param dnsserver IP address of the NTP server to set
 */
void
dhcp_set_ntp_servers(u8_t num, const ip4_addr_t *server)
{
  LWIP_DEBUGF(SNTP_DEBUG_TRACE, ("sntp: %s %u.%u.%u.%u as NTP server #%u via DHCP\n",
    (sntp_set_servers_from_dhcp ? "Got" : "Rejected"),
    ip4_addr1(server), ip4_addr2(server), ip4_addr3(server), ip4_addr4(server), num));
  if (sntp_set_servers_from_dhcp && num) {
    u8_t i;
    for (i = 0; (i < num) && (i < SNTP_MAX_SERVERS); i++) {
      ip_addr_t addr;
      ip_addr_copy_from_ip4(addr, server[i]);
      sntp_setserver(i, &addr);
    }
    for (i = num; i < SNTP_MAX_SERVERS; i++) {
      sntp_setserver(i, NULL);
    }
  }
}
#endif /* LWIP_DHCP && SNTP_GET_SERVERS_FROM_DHCP */

/**
 * @ingroup sntp
 * Obtain one of the currently configured by IP address (or DHCP) NTP servers
 *
 * @param idx the index of the NTP server
 * @return IP address of the indexed NTP server or "ip_addr_any" if the NTP
 *         server has not been configured by address (or at all).
 */
const ip_addr_t*
sntp_getserver(u8_t idx)
{
  if (idx < SNTP_MAX_SERVERS) {
    return &sntp_servers[idx].addr;
  }
  return IP_ADDR_ANY;
}

#if SNTP_SERVER_DNS
/**
 * Initialize one of the NTP servers by name
 *
 * @param numdns the index of the NTP server to set must be < SNTP_MAX_SERVERS
 * @param dnsserver DNS name of the NTP server to set, to be resolved at contact time
 */
void
sntp_setservername(u8_t idx, char *server)
{
  if (idx < SNTP_MAX_SERVERS) {
    sntp_servers[idx].name = server;
  }
}

/**
 * Obtain one of the currently configured by name NTP servers.
 *
 * @param numdns the index of the NTP server
 * @return IP address of the indexed NTP server or NULL if the NTP
 *         server has not been configured by name (or at all)
 */
char *
sntp_getservername(u8_t idx)
{
  if (idx < SNTP_MAX_SERVERS) {
    return sntp_servers[idx].name;
  }
  return NULL;
}
#endif /* SNTP_SERVER_DNS */

#endif /* LWIP_UDP */
