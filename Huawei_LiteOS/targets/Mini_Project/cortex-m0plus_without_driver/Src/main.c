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
#include "main.h"

#include "lwip/netif.h"

#include "lwip/opt.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "netif/etharp.h"
#include "lwip/sockets.h"
#include "lwip/tcpip.h"
#include "lwip/init.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "ethernetif.h"

#include "mbedtls/net.h"
#include "mbedtls/ssl.h"



#include "los_base.h"
#include "los_task.ph"
#include "los_typedef.h"
#include "los_sys.h"




UINT32 g_TskHandle;

struct netif gnetif;
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;
uint8_t IP_ADDRESS[4];
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];


struct ethernet_api g_eth_api = {
    .init     = NULL,
    .output   = NULL,
    .input    = NULL,
};



void net_init(void)
{
    /* IP addresses initialization */
    IP_ADDRESS[0] = 192;
    IP_ADDRESS[1] = 168;
    IP_ADDRESS[2] = 0;
    IP_ADDRESS[3] = 115;
    NETMASK_ADDRESS[0] = 255;
    NETMASK_ADDRESS[1] = 255;
    NETMASK_ADDRESS[2] = 255;
    NETMASK_ADDRESS[3] = 0;
    GATEWAY_ADDRESS[0] = 192;
    GATEWAY_ADDRESS[1] = 168;
    GATEWAY_ADDRESS[2] = 0;
    GATEWAY_ADDRESS[3] = 1;

    
    /* Initilialize the LwIP stack without RTOS */
    tcpip_init(NULL, NULL);
    printf("lwip test init ok.\n");
    /* IP addresses initialization without DHCP (IPv4) */
    IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
    IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
    IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);

    (void)ethernetif_api_register(&g_eth_api);/*注册特定网卡的API*/
    /* add the network interface (IPv4/IPv6) without RTOS */
    (void)netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);//lint !e546

    /* Registers the default network interface */
    netif_set_default(&gnetif);
    if (netif_is_link_up(&gnetif))
    {
        /* When the netif is fully configured this function must be called */
        gnetif.flags |= NETIF_FLAG_LINK_UP;
        netif_set_up(&gnetif);
    }
    else
    {
        /* When the netif link is down this function must be called */
        netif_set_down(&gnetif);
    }
}




int main(void)
{
    UINT32 uwRet = LOS_OK;
	extern VOID HardWare_Init(VOID);
    HardWare_Init();

    uwRet = LOS_KernelInit();
    if (uwRet != LOS_OK)
    {
        return LOS_NOK;
    }


    extern UINT32 create_work_tasks(VOID);
    uwRet =create_work_tasks();

    if (uwRet != LOS_OK)
    {
        return LOS_NOK;
    }


    (void)LOS_Start();
    return 0;
}

/*
 * atiny_adapter user interface
 */
void atiny_usleep(unsigned long usec)
{
    extern void delayus(uint32_t usec);
    delayus((uint32_t)usec);
}

int atiny_random(void *output, size_t len)
{
    extern int hal_rng_generate_buffer(void* buf, size_t len);
    return hal_rng_generate_buffer(output, len);
}

void atiny_reboot(void)
{
    extern void HAL_NVIC_SystemReset(void);
    HAL_NVIC_SystemReset();
}


void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}


