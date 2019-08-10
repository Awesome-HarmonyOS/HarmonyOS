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

#include "los_mux.h"
#include "los_task.h"
#include "los_api_mutex.h"
#include "los_inspect_entry.h"

#ifdef LOSCFG_LIB_LIBC
#include "string.h"
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

/* mux handle id */
static UINT32 g_Testmux01;

/* task pid */
static UINT32 g_TestTaskID01;
static UINT32 g_TestTaskID02;

static VOID Example_MutexTask1(VOID)
{
    UINT32 uwRet;

    dprintf("task1 try to get mutex, wait 10 Tick. \r\n");
    /* get mux */
    uwRet = LOS_MuxPend(g_Testmux01, 10);

    if (uwRet == LOS_OK)
    {
        dprintf("task1 get mutex g_Testmux01. \r\n");
        /* release mux */
        LOS_MuxPost(g_Testmux01);
        return;
    }
    else if (uwRet == LOS_ERRNO_MUX_TIMEOUT)
    {
        dprintf("task1 timeout and try to get  mutex, wait forever. \r\n");
        /* LOS_WAIT_FOREVER type get mux, LOS_MuxPend return until has been get mux */
        uwRet = LOS_MuxPend(g_Testmux01, LOS_WAIT_FOREVER);
        if (uwRet == LOS_OK)
        {
            dprintf("task1 wait forever,got mutex g_Testmux01 success. \r\n");
            /* release mux */
            LOS_MuxPost(g_Testmux01);
            uwRet = LOS_InspectStatusSetByID(LOS_INSPECT_MUTEX, LOS_INSPECT_STU_SUCCESS);
            if (LOS_OK != uwRet)
            {
                dprintf("Set Inspect Status Err \r\n");
            }
            return;
        }
    }
    return;
}

static VOID Example_MutexTask2(VOID)
{
    UINT32 uwRet;

    dprintf("task2 try to get mutex, wait forever. \r\n");
    /* get mux */
    uwRet = LOS_MuxPend(g_Testmux01, LOS_WAIT_FOREVER);
    if(uwRet != LOS_OK)
    {
        dprintf("task2 LOS_MuxPend failed . \r\n");
        return;
    }

    dprintf("task2 get mutex g_Testmux01 and suspend 100 Tick. \r\n");

    /* task delay 100 ticks */
    LOS_TaskDelay(100);

    dprintf("task2 resumed and post the g_Testmux01 \r\n");
    /* release mux */
    LOS_MuxPost(g_Testmux01);
    return;
}

UINT32 Example_MutexLock(VOID)
{
    UINT32 uwRet;
    TSK_INIT_PARAM_S stTask1;
    TSK_INIT_PARAM_S stTask2;

    /* create mux */
    LOS_MuxCreate(&g_Testmux01);

    /* lock task schedue */
    LOS_TaskLock();

    /* create task1 */
    memset(&stTask1, 0, sizeof(TSK_INIT_PARAM_S));
    stTask1.pfnTaskEntry = (TSK_ENTRY_FUNC)Example_MutexTask1;
    stTask1.pcName       = "MutexTsk1";
    stTask1.uwStackSize  = LOSCFG_BASE_CORE_TSK_DEFAULT_STACK_SIZE;
    stTask1.usTaskPrio   = 5;
    uwRet = LOS_TaskCreate(&g_TestTaskID01, &stTask1);
    if (uwRet != LOS_OK)
    {
        dprintf("task1 create failed . \r\n");
        return LOS_NOK;
    }

    /* create task2 */
    memset(&stTask2, 0, sizeof(TSK_INIT_PARAM_S));
    stTask2.pfnTaskEntry = (TSK_ENTRY_FUNC)Example_MutexTask2;
    stTask2.pcName       = "MutexTsk2";
    stTask2.uwStackSize  = LOSCFG_BASE_CORE_TSK_DEFAULT_STACK_SIZE;
    stTask2.usTaskPrio   = 4;
    uwRet = LOS_TaskCreate(&g_TestTaskID02, &stTask2);
    if (uwRet != LOS_OK)
    {
        dprintf("task2 create failed . \r\n");
        return LOS_NOK;
    }

    /* unlock task schedue */
    LOS_TaskUnlock();
    /* task delay 300 ticks */
    LOS_TaskDelay(300);

    /* delete mux */
    LOS_MuxDelete(g_Testmux01);

    return LOS_OK;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */
