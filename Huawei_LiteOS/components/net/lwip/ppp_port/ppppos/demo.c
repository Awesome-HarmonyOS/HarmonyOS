/*----------------------------------------------------------------------------
 * Copyright (c) <2016-2018>, <Huawei Technologies Co., Ltd>
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or other materials
 * provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific prior written
 * permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *---------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------
 * Notice of Export Control Law
 * ===============================================
 * Huawei LiteOS may be subject to applicable export control laws and regulations, which might
 * include those applicable to Huawei LiteOS of U.S. and the country in which you are located.
 * Import, export and usage of Huawei LiteOS in any manner by you shall be in compliance with such
 * applicable export control laws and regulations.
 *---------------------------------------------------------------------------*/
/*
PPP interface for lwIP

Author: Sylvain Rochet

Table of Contents:
- Supported PPP protocols and features
- Raw API PPP example for all protocols
- PPPoS input path (raw API, IRQ safe API, TCPIP API)
- Thread safe PPP API (PPPAPI)
- Notify phase callback (PPP_NOTIFY_PHASE)
- Upgrading from lwIP <= 1.4.x to lwIP >= 2.0.x
Supported PPP protocols and features
======================================

Supported Low level protocols:
* PPP over serial using HDLC-like framing, such as wired dialup modems
  or mobile telecommunications GPRS/EDGE/UMTS/HSPA+/LTE modems
* PPP over Ethernet, such as xDSL modems
* PPP over L2TP (Layer 2 Tunneling Protocol) LAC (L2TP Access Concentrator),
  IP tunnel over UDP, such as VPN access

Supported auth protocols:
* PAP, Password Authentication Protocol
* CHAP, Challenge-Handshake Authentication Protocol, also known as CHAP-MD5
* MSCHAPv1, Microsoft version of CHAP, version 1
* MSCHAPv2, Microsoft version of CHAP, version 2
* EAP, Extensible Authentication Protocol

Supported address protocols:
* IPCP, IP Control Protocol, IPv4 addresses negotiation
* IP6CP, IPv6 Control Protocol, IPv6 link-local addresses negotiation

Supported encryption protocols:
* MPPE, Microsoft Point-to-Point Encryption

Supported compression or miscellaneous protocols, for serial links only:
* PFC, Protocol Field Compression
* ACFC, Address-and-Control-Field-Compression
* ACCM, Asynchronous-Control-Character-Map
* VJ, Van Jacobson TCP/IP Header Compression

*/

#if USE_PPPOS

#include "netif/ppp/ppp.h"
#include "netif/ppp/pppapi.h"
#include "netif/ppp/pppos.h"


int gPppRcvMode = 0;
int gConnect = 0;

u32_t sys_jiffies(void)
{
    UINT32 ret;
    extern UINT64      g_ullTickCount;

    ret = (UINT32)g_ullTickCount;
    return ret;
}
#include "osport.h"


/* The PPP control block */
ppp_pcb *ppp;

/* The PPP IP interface */
struct netif ppp_netif;

/* //PPP status callback  ??????
 * PPP status callback  ??????
 * ===================
 *
 * PPP status callback is called on PPP status change (up, down, ? from lwIP
 * core thread
 */
extern const ip_addr_t *dns_getserver(u8_t numdns);
/* PPP status callback example */
static void status_cb(ppp_pcb *pcb, int err_code, void *ctx)
{
    struct netif *pppif = ppp_netif(pcb);
    LWIP_UNUSED_ARG(ctx);

    switch(err_code)
    {
    case PPPERR_NONE:
    {
#if LWIP_DNS
        const ip_addr_t *ns;
#endif /* LWIP_DNS */
        printf("status_cb: Connected\n");
#if PPP_IPV4_SUPPORT
        printf("   our_ipaddr  = %s\n", ipaddr_ntoa(&pppif->ip_addr));
        printf("   his_ipaddr  = %s\n", ipaddr_ntoa(&pppif->gw));
        printf("   netmask     = %s\n", ipaddr_ntoa(&pppif->netmask));
#if LWIP_DNS
        ns = dns_getserver(0);
        printf("   dns1        = %s\n", ipaddr_ntoa(ns));
        ns = dns_getserver(1);
        printf("   dns2        = %s\n", ipaddr_ntoa(ns));
        gConnect = 1;
#endif /* LWIP_DNS */
#endif /* PPP_IPV4_SUPPORT */
#if PPP_IPV6_SUPPORT
        printf("   our6_ipaddr = %s\n", ip6addr_ntoa(netif_ip6_addr(
                    pppif, 0)));
#endif /* PPP_IPV6_SUPPORT */
        break;
    }
    case PPPERR_PARAM:
    {
        printf("status_cb: Invalid parameter\n");
        break;
    }
    case PPPERR_OPEN:
    {
        printf("status_cb: Unable to open PPP session\n");
        break;
    }
    case PPPERR_DEVICE:
    {
        printf("status_cb: Invalid I/O device for PPP\n");
        break;
    }
    case PPPERR_ALLOC:
    {
        printf("status_cb: Unable to allocate resources\n");
        break;
    }
    case PPPERR_USER:
    {
        printf("status_cb: User interrupt\n");
        break;
    }
    case PPPERR_CONNECT:
    {
        printf("status_cb: Connection lost\n");
        break;
    }
    case PPPERR_AUTHFAIL:
    {
        printf("status_cb: Failed authentication challenge\n");
        break;
    }
    case PPPERR_PROTOCOL:
    {
        printf("status_cb: Failed to meet protocol\n");
        break;
    }
    case PPPERR_PEERDEAD:
    {
        printf("status_cb: Connection timeout\n");
        break;
    }
    case PPPERR_IDLETIMEOUT:
    {
        printf("status_cb: Idle Timeout\n");
        break;
    }
    case PPPERR_CONNECTTIME:
    {
        printf("status_cb: Max connect time reached\n");
        break;
    }
    case PPPERR_LOOPBACK:
    {
        printf("status_cb: Loopback detected\n");
        break;
    }
    default:
    {
        printf("status_cb: Unknown error code %d\n", err_code);
        break;
    }
    }

    /*
     * This should be in the switch case, this is put outside of the switch
     * case for example readability.
     */

    if (err_code == PPPERR_NONE)
    {
        return;
    }

    /* ppp_close() was previously called, don't reconnect */
    if (err_code == PPPERR_USER)
    {
        /* ppp_free(); -- can be called here */
        return;
    }

    /*
     * Try to reconnect in 30 seconds, if you need a modem chatscript you have
     * to do a much better signaling here ;-)
     */
    ppp_connect(pcb, 30);
    /* OR ppp_listen(pcb); */
}
static u32_t output_cb(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx)
{
    return iodev_write(0, data, len, 100);
}


void *main_pppinput(unsigned int args)
{
    int ret;
    unsigned char buf[256];
    while(1)
    {
        if(gPppRcvMode)
        {
            iodev_debugmode(2, 2);
            ret = iodev_read(0, buf, 256, 10);
            if(ret > 0)
            {
                pppos_input(ppp, (unsigned char *)buf, ret);
            }
        }
    }
}
VOID *main_ppp(UINT32  args)
{
    /* Initilialize the LwIP stack without RTOS */
    tcpip_init(NULL, NULL);

    //here we make the modem to data mode
    iodev_debugmode(2, 1);
    extern int AtDial(char *devname, char *apn);
    while(0 != AtDial("uart3", NULL))
    {
    }
    ppp = pppos_create(&ppp_netif, (pppos_output_cb_fn)output_cb, status_cb, NULL);
    if(NULL != ppp)
    {
        extern void *main_pppinput(unsigned int args);
        task_create("main_pppinput", main_pppinput, 0x800, NULL, NULL, 0);

        /* Set this interface as default route */
        ppp_set_default(ppp);
        /* Ask the peer for up to 2 DNS server addresses. */
        ppp_set_usepeerdns(ppp, 1);

        /* Auth configuration, this is pretty self-explanatory */
        ppp_set_auth(ppp, PPPAUTHTYPE_ANY, "login", "password");
        gPppRcvMode = 1;
        u16_t holdoff = 0;
        ppp_connect(ppp, holdoff);
        while(gConnect == 0) //wait to do the connect
        {
            LOS_TaskDelay(10);
        }
        extern void agent_tiny_entry(void);
        agent_tiny_entry();
        ppp_free(ppp);
    }


    return NULL;
}
#endif





