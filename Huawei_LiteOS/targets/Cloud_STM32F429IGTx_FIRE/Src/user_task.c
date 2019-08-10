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
#ifdef CONFIG_FEATURE_FOTA
#include "ota_port.h"
#endif
#include "nb_iot/los_nb_api.h"
#include "at_frame/at_api.h"
#include "at_device/bc95.h"
#ifdef WITH_MQTT
#include "flash_adaptor.h"
#include "agenttiny_mqtt/agent_tiny_demo.h"
#else
//#include "agenttiny_lwm2m/agent_tiny_demo.h"
#endif



static UINT32 g_atiny_tskHandle;
static UINT32 g_fs_tskHandle;





void atiny_task_entry(void)
{
    extern void agent_tiny_entry();
#if defined(WITH_LINUX) || defined(WITH_LWIP)
    hieth_hw_init();
    net_init();
#elif defined(WITH_AT_FRAMEWORK)


    #if defined(USE_ESP8266)
    extern at_adaptor_api esp8266_interface;
    printf("\r\n=============agent_tiny_entry  USE_ESP8266============================\n");
    at_api_register(&esp8266_interface);

    #elif defined(USE_EMTC_BG36)
    extern at_adaptor_api emtc_bg36_interface;
    printf("\r\n=============agent_tiny_entry  USE_EMTC_BG36============================\n");
    at_api_register(&emtc_bg36_interface);

    #elif defined(USE_SIM900A)
    extern at_adaptor_api sim900a_interface;
    printf("\r\n=============agent_tiny_entry  USE_SIM900A============================\n");
    at_api_register(&sim900a_interface);

    #elif defined(USE_NB_NEUL95)
    extern at_adaptor_api bc95_interface;
    printf("\r\n=============agent_tiny_entry  USE_NB_NEUL95============================\n");
    los_nb_init((const int8_t *)"172.25.233.98",(const int8_t *)"5600",NULL);
    los_nb_notify("\r\n+NSONMI:",strlen("\r\n+NSONMI:"),NULL,nb_cmd_match);
    at_api_register(&bc95_interface);

    #elif defined(USE_NB_NEUL95_NO_ATINY)
    demo_nbiot_only();
    #else

    #endif
#else
#endif

#ifdef WITH_MQTT
    flash_adaptor_init();
    {

        demo_param_s demo_param = {.init = NULL,
                                   .write_flash_info = flash_adaptor_write_mqtt_info,
                                   .read_flash_info = flash_adaptor_read_mqtt_info};
        agent_tiny_demo_init(&demo_param);
    }
#endif


#if !defined(USE_NB_NEUL95_NO_ATINY)
#ifdef CONFIG_FEATURE_FOTA
    hal_init_ota();
#endif
    agent_tiny_entry();
#endif
}


UINT32 creat_agenttiny_task(VOID)
{
    UINT32 uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    task_init_param.usTaskPrio = 2;
    task_init_param.pcName = "agenttiny_task";
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)atiny_task_entry;

#if defined(CONFIG_FEATURE_FOTA) || defined(WITH_MQTT)
    task_init_param.uwStackSize = 0x2000; /* fota use mbedtls bignum to verify signature  consuming more stack  */
#else
    task_init_param.uwStackSize = 0x1000;
#endif

    uwRet = LOS_TaskCreate(&g_atiny_tskHandle, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;
}


UINT32 creat_fs_task(void)
{
    UINT32 uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    task_init_param.usTaskPrio = 2;
    task_init_param.pcName = "main_task";
    extern void fs_demo(void);
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)fs_demo;


    task_init_param.uwStackSize = 0x1000;

    uwRet = LOS_TaskCreate(&g_fs_tskHandle, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;
}



#if defined(WITH_DTLS) && defined(SUPPORT_DTLS_SRV)
static UINT32 g_dtls_server_tskHandle;
uint32_t create_dtls_server_task()
{
    uint32_t uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    task_init_param.usTaskPrio = 3;
    task_init_param.pcName = "dtls_server_task";
    extern void dtls_server_task(void);
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)dtls_server_task;

    task_init_param.uwStackSize = 0x1000;

    uwRet = LOS_TaskCreate(&g_dtls_server_tskHandle, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;
}
#endif


UINT32 create_work_tasks(VOID)
{
    UINT32 uwRet = LOS_OK;

    uwRet = creat_agenttiny_task();
    if (uwRet != LOS_OK)
    {
    	return LOS_NOK;
    }

#if defined(FS_SPIFFS) || defined(FS_FATFS)
    uwRet = creat_fs_task();
    if (uwRet != LOS_OK)
    {
    	return LOS_NOK;
    }
#endif

#if defined(USE_PPPOS)
    #include "osport.h"
    extern void uart_init(void);  //this uart used for the pppos interface
    uart_init();
    extern VOID *main_ppp(UINT32  args);
    task_create("main_ppp", main_ppp, 0x1500, NULL, NULL, 2);
#endif


#if defined(WITH_DTLS) && defined(SUPPORT_DTLS_SRV)
    uwRet = create_dtls_server_task()
    if (uwRet != LOS_OK)
    {
    	return LOS_NOK;
    }
#endif

    return uwRet;

}



