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

#include "sys_init.h"

#ifdef WITH_LWIP
struct     netif gnetif;
uint8_t    IP_ADDRESS[4];
uint8_t    NETMASK_ADDRESS[4];
uint8_t    GATEWAY_ADDRESS[4];

#if LWIP_IPV4 && LWIP_IPV6
ip_addr_t  ipaddr;
ip_addr_t  netmask;
ip_addr_t  gw;
#elif LWIP_IPV6
#else
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;
#endif

static void lwip_impl_register(void)
{
    STlwIPFuncSsp stlwIPSspCbk = {0};
    stlwIPSspCbk.pfRand = hal_rng_generate_number;
    lwIPRegSspCbk(&stlwIPSspCbk);
}

void net_init(void)
{
    /* IP addresses initialization */
    IP_ADDRESS[0] = 192;
    IP_ADDRESS[1] = 168;
    IP_ADDRESS[2] = 1;
    IP_ADDRESS[3] = 115;
    NETMASK_ADDRESS[0] = 255;
    NETMASK_ADDRESS[1] = 255;
    NETMASK_ADDRESS[2] = 255;
    NETMASK_ADDRESS[3] = 0;
    GATEWAY_ADDRESS[0] = 192;
    GATEWAY_ADDRESS[1] = 168;
    GATEWAY_ADDRESS[2] = 1;
    GATEWAY_ADDRESS[3] = 1;

#if LWIP_IPV4 && LWIP_IPV6
    IP_ADDR4(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
    IP_ADDR4(&netmask,NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
    IP_ADDR4(&gw,     GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);
#elif LWIP_IPV6
    //IP_ADDR6(&ipaddr,);
#else 
    /* IP addresses initialization without DHCP (IPv4) */
    IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
    IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
    IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);
#endif

    lwip_impl_register();

    /* Initilialize the LwIP stack without RTOS */
    tcpip_init(NULL, NULL);
    printf("lwip test init ok.\n");
    

    (void)ethernetif_api_register(&g_eth_api);/*×¢²áÌØ¶¨Íø¿¨µÄAPI*/
    /* add the network interface (IPv4/IPv6) without RTOS */
#if LWIP_IPV4 && LWIP_IPV6
    (void)netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, ethernetif_init, tcpip_input);//lint !e546
#elif LWIP_IPV6

    (void)netif_add(&gnetif, NULL, ethernetif_init, tcpip_input);
    netif_create_ip6_linklocal_address(&gnetif, 1);
    {
        ip6_addr_t ip6;
        err_t ret;
        s8_t idx;
        ip6_addr_t ipv6_gw;

        if (inet_pton(AF_INET6, "2000::2", &ip6) <= 0)
        {
            printf("set source ip6 failed \n");
            return;
        }
        ret = netif_add_ip6_address(&gnetif, &ip6, &idx);
        if (ret != 0)
        {
            printf("netif_add_ip6_address failed,ret %d\n", ret);
            return;
        }

        if (inet_pton(AF_INET6, "2000::1", &ipv6_gw) <= 0)
        {
            printf("inet_pton failed\n");
            return;
        }
        set_lwip_ipv6_default_gw(&gnetif, &ipv6_gw);
    }
#else
    (void)netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, ethernetif_init, tcpip_input);//lint !e546
#endif

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
#endif
uint32_t HAL_GetTick(void)
{
    return (uint32_t)LOS_TickCountGet();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Activate the Over-Drive mode
    */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    SystemCoreClockUpdate();
}
#ifdef WITH_LWIP
void hieth_hw_init(void)
{
    extern void ETH_IRQHandler(void);
    (void)LOS_HwiCreate(ETH_IRQn, 1, 0, ETH_IRQHandler, 0);
}
#endif

/*
 * atiny_adapter user interface
 */
void atiny_usleep(unsigned long usec)
{
    delayus((uint32_t)usec);
}

int atiny_random(void *output, size_t len)
{
    return hal_rng_generate_buffer(output, len);
}

void atiny_reboot(void)
{
    HAL_NVIC_SystemReset();
}

