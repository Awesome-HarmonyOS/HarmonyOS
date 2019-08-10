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

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "lwip/err.h"
//#include "ethernetif.h"
#include <string.h>

#include "los_mux.h"
#include "los_sem.h"
#include "ENC28J60.H"


/* Define those to better describe your network interface. */
#define IFNAME0 's'
#define IFNAME1 't'

/* The time to block waiting for input. */
#define TIME_WAITING_FOR_INPUT                 ( 5 )

//void ethernetif_input( void * pvParameters );
static void arp_timer(void* arg);
#define CN_MIN_FRAMLEN    40
#define CN_CACHED_BUFLEN  1518
#define CN_PEND_FOREVER   (1000000)
typedef struct
{
    UINT8 rbuf[CN_CACHED_BUFLEN];
    UINT8 wbuf[CN_CACHED_BUFLEN];
    UINT32 wlen;
    UINT32 rlen;
    UINT32 rcvsync;
    UINT32 mutex;
    struct netif  *netif;//used for the lwip
}tagIfCB;
static tagIfCB  *pIfCB;  //use this structure to receive or send the data
static tagIfCB  gIfCBMem;
/**
* In this function, the hardware should be initialized.
* Called from ethernetif_init().
*
* @param netif the already initialized lwip network interface structure
*        for this ethernetif
*/

uint8_t MACAddr[6] = {0x00,0x80,0xE1,0X00,0X00,0X00} ;


void ethernetif_input( void * pvParameters );


static void low_level_init(struct netif* netif)
{
    /* Init ETH */

    //do the initialize 
	enc28j60Init(MACAddr);
    /*
    pIfCB = LOS_MemAlloc(m_aucSysMem0,sizeof(tagIfCB));
    if(NULL == pIfCB)
    {
        while(1); //some err here
    }
    */
    pIfCB = &gIfCBMem;
    memset(pIfCB,0,sizeof(tagIfCB));
    LOS_MuxCreate(&pIfCB->mutex);
    LOS_SemCreate(0,&pIfCB->rcvsync);
#if LWIP_ARP || LWIP_ETHERNET

    /* set MAC hardware address length */
    netif->hwaddr_len = ETH_HWADDR_LEN;

    /* set MAC hardware address */
    netif->hwaddr[0] =  MACAddr[0];
    netif->hwaddr[1] =  MACAddr[1];
    netif->hwaddr[2] =  MACAddr[2];
    netif->hwaddr[3] =  MACAddr[3];
    netif->hwaddr[4] =  MACAddr[4];
    netif->hwaddr[5] =  MACAddr[5];
    /* maximum transfer unit */
    netif->mtu = 1500;
    netif->flags |= NETIF_FLAG_LINK_UP;
	pIfCB->netif = netif;
    /* Accept broadcast address and ARP traffic */
    /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
#if LWIP_ARP
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
#else
    netif->flags |= NETIF_FLAG_BROADCAST;
#endif /* LWIP_ARP */
    /* create the task that handles the ETH_MAC */
    sys_thread_new((char*)"ethernetif_input", ethernetif_input, netif, 0x400, 5);
#endif /* LWIP_ARP || LWIP_ETHERNET */
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

static err_t low_level_output(struct netif* netif, struct pbuf* p)
{
    err_t errval  = ERR_USE;
    struct pbuf* q;
    uint8_t* dst;
    uint8_t* src;
    int      cpylen = 0;
    int      sndlen= 0;
    //check the parameters
	
    if ((NULL == p) || (pIfCB->netif != netif)) {
        goto EXIT_PARAERR;
    }
    if ((p->tot_len > CN_CACHED_BUFLEN) || (p->tot_len < CN_MIN_FRAMLEN)) {
        goto EXIT_PARAERR;
    }
    //ok,now copy the data from pbuffer to the cached buffer
    if (LOS_OK == LOS_MuxPend(pIfCB->mutex,CN_PEND_FOREVER)) {

        sndlen = p->tot_len;
        dst = pIfCB->wbuf;
        // copy frame from pbufs to driver buffers 
        for (q = p; q != NULL; q = q->next)
        {
            src = (uint8_t*)q->payload;
            cpylen = q->len;
            memcpy(dst, src, cpylen);
            dst += cpylen;
        }
        pIfCB->wlen = sndlen;
        enc28j60PacketSend(pIfCB->wlen, pIfCB->wbuf);
        pIfCB->wlen = 0;
        LOS_MuxPost(pIfCB->mutex);
        errval = ERR_OK;
    }
EXIT_PARAERR:
    return errval;
}
/**
* Should allocate a pbuf and transfer the bytes of the incoming
* packet from the interface into the pbuf.
*
* @param netif the lwip network interface structure for this ethernetif
* @return a pbuf filled with the received packet (including MAC header)
*         NULL on memory error
*/

static struct pbuf* low_level_input(struct netif* netif)
{
    struct pbuf* p = NULL;
    struct pbuf* q = NULL;
    int rcvlen = 0;
    int cpylen = 0;
    uint8_t *dst;
    uint8_t *src;
    //ok,now copy the data from pbuffer to the cached buffer
    if (LOS_OK == LOS_MuxPend(pIfCB->mutex,CN_PEND_FOREVER))
    {
        pIfCB->rlen = enc28j60PacketReceive(CN_CACHED_BUFLEN, pIfCB->rbuf);
        if (pIfCB->rlen > 0) 
        {
			rcvlen = pIfCB->rlen ;
            p = pbuf_alloc(PBUF_RAW, rcvlen, PBUF_POOL);
            if (NULL != p) 
            {
                 //now do the copy here
                src = pIfCB->rbuf;
                for (q = p; q != NULL; q = q->next)
                {
                    if (rcvlen > 0)
                    {
                        cpylen = q->len;
                        dst = (uint8_t*)q->payload;
                        memcpy(dst,src,cpylen);
                        rcvlen -= cpylen;
                        src += cpylen;
                    }
                }
            }
            pIfCB->rlen = 0;
        }
        LOS_MuxPost(pIfCB->mutex);
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
void ethernetif_input( void* pvParameters )
{
    struct pbuf* p;
    err_t err = ERR_TIMEOUT ;
    /* move received packet into a new pbuf */
    while (1)
    {
        LOS_TaskDelay(10);
        p = low_level_input(pIfCB->netif);
        if (NULL != p) 
        {
            err = pIfCB->netif->input(p, pIfCB->netif);
            if (err != ERR_OK) //if not ok,we should free the buf here
            {
                LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
                pbuf_free(p);
                p = NULL;
            }
        }   
    }
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
err_t ethernetif_init(struct netif* netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;

    netif->output = etharp_output;
    netif->linkoutput = low_level_output;

    /* initialize the hardware */
    low_level_init(netif);
    etharp_init();
    sys_timeout(ARP_TMR_INTERVAL, arp_timer, NULL);
    return ERR_OK;
}
static void arp_timer(void* arg)
{
    etharp_tmr();
    sys_timeout(ARP_TMR_INTERVAL, arp_timer, NULL);
}


