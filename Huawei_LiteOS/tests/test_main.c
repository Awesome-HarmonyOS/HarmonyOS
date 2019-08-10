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
#if defined(WITH_LWIP)
#include "../../../test_agenttiny/test_agenttiny.h"
#endif

static UINT32 g_atiny_tskHandle;
static UINT32 g_fs_tskHandle;
static UINT32 g_sota_tskHandle;

UINT32 creat_fs_test_task(void)
{
    UINT32 uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    task_init_param.usTaskPrio = 2;
    task_init_param.pcName = "fs_test_main";
    extern int fs_test_main(void);
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)fs_test_main;


    task_init_param.uwStackSize = 0x1000;

    uwRet = LOS_TaskCreate(&g_fs_tskHandle, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;
}

UINT32 creat_sota_test_task(void)
{
    UINT32 uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    task_init_param.usTaskPrio = 2;
    task_init_param.pcName = "sota_test_main";
    extern int sota_test_main(void);
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)sota_test_main;


    task_init_param.uwStackSize = 0x3000;

    uwRet = LOS_TaskCreate(&g_sota_tskHandle, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;
}

UINT32 creat_agenttiny_test_task(void)
{
    UINT32 uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    task_init_param.usTaskPrio = 2;
    task_init_param.pcName = "agenttiny_test_main";
    extern void test_agenttiny(void);
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)test_agenttiny;

    task_init_param.uwStackSize = 0x3000;

    uwRet = LOS_TaskCreate(&g_atiny_tskHandle, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;
}

int demo_cmockery_test(void)
{
    UINT32 uwRet = LOS_OK;
#if (defined(FS_SPIFFS) || defined(FS_FATFS))

    uwRet = creat_fs_test_task();
    if (uwRet != LOS_OK)
    {
    	return LOS_NOK;
    }
#endif

#if defined(WITH_AT_FRAMEWORK) && defined(USE_NB_NEUL95_NO_ATINY) && defined(WITH_SOTA)

    uwRet = creat_sota_test_task();
    if (uwRet != LOS_OK)
    {
    	return LOS_NOK;
    }
#endif

#if defined(WITH_LWIP) && (!defined(USE_NB_NEUL95_NO_ATINY))

    uwRet = creat_agenttiny_test_task();
    if (uwRet != LOS_OK)
    {
    	return LOS_NOK;
    }
#endif

    return uwRet;
}
