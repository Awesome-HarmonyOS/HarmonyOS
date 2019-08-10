/*
 * Copyright 2008 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include "cmockery.h"
#include "atiny_lwm2m/agenttiny.h"
#include "osdepends/liteos/cmsis_os2.h"
#include "regresstest.h"



#ifdef CONFIG_FEATURE_FOTA
#include "fota_port.h"
#endif

#define DEFAULT_SERVER_IPV4 "192.168.1.102"

#define LWM2M_LIFE_TIME     50000

//static char * g_endpoint_name = "44440003";
#ifdef WITH_DTLS

static char* g_endpoint_name_s = "18602560533";
static char* g_endpoint_name_iots = "18602560533";
static char* g_endpoint_name_bs = "18602560533";
static unsigned char g_psk_iot_value[] = {0x33,0x44,0x55};
static unsigned char g_psk_bs_value[] = {0x33,0x44,0x55};
#endif

static void* g_phandle = NULL;
static atiny_device_info_t g_device_info;
static atiny_param_t g_atiny_params;

static uint32_t g_TskHandle;

extern UINT32 creat_report_task();

void test_init_task(UINT32 uwArg){

    uint32_t uwRet = ATINY_OK;
    atiny_param_t* atiny_params;
    atiny_security_param_t  *iot_security_param = NULL;
    atiny_security_param_t  *bs_security_param = NULL;

    atiny_device_info_t *device_info = &g_device_info;

    printf("now call test_init_task!!!\n");

#ifdef CONFIG_FEATURE_FOTA
    extern void agent_tiny_fota_init(void);
    agent_tiny_fota_init();
#endif

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

    //pay attention: index 0 for iot server, index 1 for bootstrap server.
    iot_security_param = &(atiny_params->security_params[0]);
    bs_security_param = &(atiny_params->security_params[1]);


    iot_security_param->server_ip = DEFAULT_SERVER_IPV4;
    bs_security_param->server_ip = DEFAULT_SERVER_IPV4;

#ifdef WITH_DTLS
    iot_security_param->server_port = "5684";
    bs_security_param->server_port = "5684";

    iot_security_param->psk_Id = g_endpoint_name_iots;
    iot_security_param->psk = (char*)g_psk_iot_value;
    iot_security_param->psk_len = sizeof(g_psk_iot_value);

    bs_security_param->psk_Id = g_endpoint_name_bs;
    bs_security_param->psk = (char*)g_psk_bs_value;
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

    uwRet = atiny_init(atiny_params, &g_phandle);
    assert_int_equal(uwRet, ATINY_OK);

    uwRet = creat_report_task();
    if(LOS_OK != uwRet)
    {
        printf("creat_report_task failed !!!\n");
        return;
    }
    printf("now call atiny_bind!!!\n");
    (void)atiny_bind(device_info, g_phandle);
}


UINT32 creat_init_task()
{
    uint32_t uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    memset(&task_init_param,0,sizeof(TSK_INIT_PARAM_S));
    task_init_param.usTaskPrio = 0;
    task_init_param.pcName = "test_deinit_task";
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)test_init_task;

#ifdef CONFIG_FEATURE_FOTA
    task_init_param.uwStackSize = 0x2000; /* fota use mbedtls bignum to verify signature  consuming more stack  */
#else
    task_init_param.uwStackSize = 0x1000;
#endif

    uwRet = LOS_TaskCreate((UINT32 *)&g_TskHandle, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;
}


void *test_deinit_task(UINT32 uwArg)
{
    osDelay(10*1000);
    atiny_deinit(g_phandle);
    return (void *)uwArg;
}


UINT32 creat_deinit_task()
{
    uint32_t uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    memset(&task_init_param,0,sizeof(TSK_INIT_PARAM_S));
    task_init_param.usTaskPrio = 0;
    task_init_param.pcName = "test_deinit_task";
    task_init_param.pfnTaskEntry = test_deinit_task;

#ifdef CONFIG_FEATURE_FOTA
    task_init_param.uwStackSize = 0x2000; /* fota use mbedtls bignum to verify signature  consuming more stack  */
#else
    task_init_param.uwStackSize = 0x1000;
#endif

    uwRet = LOS_TaskCreate((UINT32 *)&g_TskHandle, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;
}


// Test case that fails as leak_memory() leaks a dynamically allocated block.
void agenttiny_init_test(void **state) {
	printf("now in agenttiny_init_test\n");
    creat_init_task();

}

void agenttiny_deinit_test(void **state){
    creat_deinit_task();

}


int agenttiny_test_main(void) {
    const UnitTest tests[] = {
        unit_test(agenttiny_init_test),
        unit_test(agenttiny_deinit_test),
    };

    return run_tests(tests);
}
