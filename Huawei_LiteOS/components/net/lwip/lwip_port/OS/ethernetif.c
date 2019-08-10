/**
* @file
* Ethernet Interface Skeleton
*
*/

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
 *--------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------
 * Notice of Export Control Law
 * ===============================================
 * Huawei LiteOS may be subject to applicable export control laws and regulations, which might
 * include those applicable to Huawei LiteOS of U.S. and the country in which you are located.
 * Import, export and usage of Huawei LiteOS in any manner by you shall be in compliance with such
 * applicable export control laws and regulations.
 *---------------------------------------------------------------------------*/

/*
* Copyright (c) 2001-2004 Swedish Institute of Computer Science.
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

/*
* This file is a skeleton for developing Ethernet network interface
* drivers for lwIP. Add code to the low_level functions and do a
* search-and-replace for the word "ethernetif" to replace it with
* something that better describes your network interface.
*/


#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/timeouts.h"
#if LWIP_IPV4 && LWIP_IPV6
#elif LWIP_IPV6
#include "lwip/ethip6.h"
#else
#include "netif/etharp.h"
#endif
#include "ethernetif.h"
#include <string.h>
#include "lwip/tcpip.h"


/* Define those to better describe your network interface. */
#define IFNAME0 's'
#define IFNAME1 't'


static struct netif *s_pxNetIf = NULL;
static struct ethernet_api s_eth_api;

//void ethernetif_input( void * pvParameters );
#if LWIP_IPV4 && LWIP_IPV6
#elif LWIP_IPV6
#else
static void arp_timer(void *arg);
#endif
int8_t ethernetif_api_register(struct ethernet_api *api)
{
    if(api == NULL)
    {
        return ERR_ARG;
    }
    memcpy(&s_eth_api, api, sizeof(struct ethernet_api));

    return ERR_OK;
}

/**
* In this function, the hardware should be initialized.
* Called from ethernetif_init().
*
* @param netif the already initialized lwip network interface structure
*        for this ethernetif
*/
static void low_level_init(struct netif *netif)
{
    if(s_eth_api.init)
    {
        (void)s_eth_api.init(netif);
        s_pxNetIf = netif;
    }
}

/**
* This function should do the actual transmission of the packet. The packet is
* contained in the pbuf that is passed to the function. This pbuf
* might be chained.
*
* @param netif the lwip network interface structure for this ethernetif
* @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
* @return ERR_OK if the packet could be sent
*         an err_t value if the packet couldn't be sent
*
* @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
*       strange results. You might consider waiting for space in the DMA queue
*       to become availale since the stack doesn't retry to send a packet
*       dropped because of memory failure (except for the TCP timers).
*/
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
    err_t err = ERR_IF;

    if(s_eth_api.output)
    {
        err = s_eth_api.output(netif, p);
    }

    return err;
}

/**
* Should allocate a pbuf and transfer the bytes of the incoming
* packet from the interface into the pbuf.
*
* @param netif the lwip network interface structure for this ethernetif
* @return a pbuf filled with the received packet (including MAC header)
*         NULL on memory error
*/
static struct pbuf *low_level_input(struct netif *netif)
{
    struct pbuf *p = NULL;

    if(s_eth_api.input)
    {
        p = s_eth_api.input(netif);
    }
    return p;
}

/**
* This function is the ethernetif_input task, it is processed when a packet
* is ready to be read from the interface. It uses the function low_level_input()
* that should handle the actual reception of bytes from the network
* interface. Then the type of the received packet is determined and
* the appropriate input function is called.
*
* @param netif the lwip network interface structure for this ethernetif
*/
void ethernetif_input( void *pvParameters )
{
    struct pbuf *p;
    err_t err;

    //LWIP_ASSERT("s_pxNetIf != NULL", (s_pxNetIf != NULL));
    /* move received packet into a new pbuf */
    do
    {
        SYS_ARCH_DECL_PROTECT(sr);

        SYS_ARCH_PROTECT(sr);
        p = low_level_input(s_pxNetIf);
        SYS_ARCH_UNPROTECT(sr);

        if (p == NULL)
        {
            return;
        }
        /*netif_add func now use tcpip_input, sys_init.c
        netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, ethernetif_init, tcpip_input);
        */
        /*err = s_pxNetIf->input(p, s_pxNetIf);*/
        err = tcpip_input(p, s_pxNetIf);
        if (err != ERR_OK)
        {
            LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
            pbuf_free(p);
            p = NULL;
        }
    }
    while(p != NULL);
}

/**
* Should be called at the beginning of the program to set up the
* network interface. It calls the function low_level_init() to do the
* actual setup of the hardware.
*
* This function should be passed as a parameter to netif_add().
*
* @param netif the lwip network interface structure for this ethernetif
* @return ERR_OK if the loopif is initialized
*         ERR_MEM if private data couldn't be allocated
*         any other err_t on error
*/
err_t ethernetif_init(struct netif *netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
#if LWIP_IPV4 && LWIP_IPV6
#elif LWIP_IPV6
    netif->output_ip6 = ethip6_output;
#else
    netif->output = etharp_output;
#endif

    netif->linkoutput = low_level_output;


    /* initialize the hardware */
    low_level_init(netif);
#if LWIP_IPV4 && LWIP_IPV6
#elif LWIP_IPV6
#else
    etharp_init();
    sys_timeout(ARP_TMR_INTERVAL, arp_timer, NULL);
#endif

    return ERR_OK;
}

#if LWIP_IPV4 && LWIP_IPV6
#elif LWIP_IPV6
#else
static void arp_timer(void *arg)
{
    etharp_tmr();
    sys_timeout(ARP_TMR_INTERVAL, arp_timer, NULL);
}
#endif

#if LWIP_IPV6
static ip6_addr_t g_lwip_ipv6_gw;
ip6_addr_t *get_lwip_ipv6_default_gw(const struct netif *netif, const ip6_addr_t *ip6addr)
{
    (void)ip6addr;
    (void)netif;
    return &g_lwip_ipv6_gw;
}

void set_lwip_ipv6_default_gw(struct netif *netif, const ip6_addr_t *gw)
{
    (void)netif;
    if (gw == NULL)
    {
        return;
    }
    memcpy( &g_lwip_ipv6_gw, gw, sizeof(g_lwip_ipv6_gw));
}
#endif

