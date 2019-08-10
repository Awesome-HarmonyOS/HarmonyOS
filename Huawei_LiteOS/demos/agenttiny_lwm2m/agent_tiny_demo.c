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

#include "agent_tiny_demo.h"

#if defined WITH_AT_FRAMEWORK
#include "at_frame/at_api.h"
#endif

//#define DEFAULT_SERVER_IP "180.101.147.115" /*dianxin*/
//#define DEFAULT_SERVER_IP "2000::1"
#define DEFAULT_SERVER_IP "192.168.1.104" /*local ipv4*/


#define LWM2M_LIFE_TIME     50000

char *g_endpoint_name = "44440003";
#ifdef WITH_DTLS

char *g_endpoint_name_s = "20181214";
char *g_endpoint_name_iots = "20181214";
char *g_endpoint_name_bs = "20181214";
unsigned char g_psk_iot_value[] = {0x68,0xda,0x7a,0xea,0xf6,0x12,0xfd,0x95,0xbb,0xe0,0x91,0x5a,0x67,0xca,0x56,0xb3}; //0x33 -> 0x32
unsigned char g_psk_bs_value[] = {0x68,0xda,0x7a,0xea,0xf6,0x12,0xfd,0x95,0xbb,0xe0,0x91,0x5a,0x67,0xca,0x56,0xb3};
//unsigned char g_psk_value[16] = {0x58,0xea,0xfd,0xab,0x2f,0x38,0x4d,0x39,0x80,0x69,0x4d,0x1c,0xda,0x69,0xb0,0x43};


#endif

static void *g_phandle = NULL;
static atiny_device_info_t g_device_info;
static atiny_param_t g_atiny_params;


void ack_callback(atiny_report_type_e type, int cookie, data_send_status_e status)
{
    ATINY_LOG(LOG_DEBUG, "type:%d cookie:%d status:%d\n", type, cookie, status);
}
/*lint -e550*/
void app_data_report(void)
{
    uint8_t buf[5] = {0, 1, 6, 5, 9};
    data_report_t report_data;
    int ret = 0;
    int cnt = 0;
    report_data.buf = buf;
    report_data.callback = ack_callback;
    report_data.cookie = 0;
    report_data.len = sizeof(buf);
    report_data.type = APP_DATA;
    (void)ret;
    while(1)
    {
        report_data.cookie = cnt;
        cnt++;
        ret = atiny_data_report(g_phandle, &report_data);
        ATINY_LOG(LOG_DEBUG, "data report ret: %d\n", ret);
        ret = atiny_data_change(g_phandle, DEVICE_MEMORY_FREE);
        ATINY_LOG(LOG_DEBUG, "data change ret: %d\n", ret);
        (void)LOS_TaskDelay(250 * 8);
    }
}
/*lint +e550*/

UINT32 creat_report_task()
{
    UINT32 uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;
    UINT32 TskHandle;

    task_init_param.usTaskPrio = 1;
    task_init_param.pcName = "app_data_report";
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)app_data_report;
    task_init_param.uwStackSize = 0x1000;

    uwRet = LOS_TaskCreate(&TskHandle, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;

}


void agent_tiny_entry(void)
{
    UINT32 uwRet = LOS_OK;
    atiny_param_t *atiny_params;
    atiny_security_param_t  *iot_security_param = NULL;
    atiny_security_param_t  *bs_security_param = NULL;

    atiny_device_info_t *device_info = &g_device_info;


#ifdef WITH_DTLS
    device_info->endpoint_name = g_endpoint_name_s;
#else
    device_info->endpoint_name = g_endpoint_name;
#endif
#ifdef CONFIG_FEATURE_FOTA
    device_info->manufacturer = "Lwm2mFota";
    device_info->dev_type = "Lwm2mFota";
#else
    device_info->manufacturer = "Agent_Tiny";
#endif
    atiny_params = &g_atiny_params;
    atiny_params->server_params.binding = "UQ";
    //atiny_params->server_params.life_time = LWM2M_LIFE_TIME;
    atiny_params->server_params.life_time = 20;
    atiny_params->server_params.storing_cnt = 0;

    atiny_params->server_params.bootstrap_mode = BOOTSTRAP_FACTORY;
    atiny_params->server_params.hold_off_time = 10;

    //pay attention: index 0 for iot server, index 1 for bootstrap server.
    iot_security_param = &(atiny_params->security_params[0]);
    bs_security_param = &(atiny_params->security_params[1]);


    iot_security_param->server_ip = DEFAULT_SERVER_IP;
    bs_security_param->server_ip  = DEFAULT_SERVER_IP;


#ifdef WITH_DTLS
    iot_security_param->server_port = "5684";
    bs_security_param->server_port = "5684";

    iot_security_param->psk_Id = g_endpoint_name_iots;
    iot_security_param->psk = (char *)g_psk_iot_value;
    iot_security_param->psk_len = sizeof(g_psk_iot_value);

    bs_security_param->psk_Id = g_endpoint_name_bs;
    bs_security_param->psk = (char *)g_psk_bs_value;
    bs_security_param->psk_len = sizeof(g_psk_bs_value);
#else
    iot_security_param->server_port = "5683";
    bs_security_param->server_port = "5683";

    iot_security_param->psk_Id = NULL;
    iot_security_param->psk = NULL;
    iot_security_param->psk_len = 0;

    bs_security_param->psk_Id = NULL;
    bs_security_param->psk = NULL;
    bs_security_param->psk_len = 0;
#endif

    if(ATINY_OK != atiny_init(atiny_params, &g_phandle))
    {
        return;
    }

    uwRet = creat_report_task();
    if(LOS_OK != uwRet)
    {
        return;
    }

    (void)atiny_bind(device_info, g_phandle);
}





