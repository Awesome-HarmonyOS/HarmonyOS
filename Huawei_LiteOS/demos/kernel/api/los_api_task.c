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

#include "math.h"
#include "time.h"
#include "los_task.h"
#include "los_api_task.h"
#include "los_inspect_entry.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

static UINT32 g_uwTskHiID;
static UINT32 g_uwTskLoID;

#define TSK_PRIOR_HI 4
#define TSK_PRIOR_LO 5

static UINT32 Example_TaskHi(VOID)
{
    UINT32 uwRet = LOS_OK;

    dprintf("Enter TaskHi Handler.\r\n");

    /* task delay 5 ticks, task will be suspend */
    uwRet = LOS_TaskDelay(5);
    if (uwRet != LOS_OK)
    {
        dprintf("Delay Task Failed.\r\n");
        return LOS_NOK;
    }

    /* task resumed */
    dprintf("TaskHi LOS_TaskDelay Done.\r\n");

    /* suspend self */
    uwRet = LOS_TaskSuspend(g_uwTskHiID);
    if (uwRet != LOS_OK)
    {
        dprintf("Suspend TaskHi Failed.\r\n");
        uwRet = LOS_InspectStatusSetByID(LOS_INSPECT_TASK, LOS_INSPECT_STU_ERROR);
        if (LOS_OK != uwRet)
        {
            dprintf("Set Inspect Status Err.\r\n");
        }
        return LOS_NOK;
    }

    dprintf("TaskHi LOS_TaskResume Success.\r\n");

    uwRet = LOS_InspectStatusSetByID(LOS_INSPECT_TASK, LOS_INSPECT_STU_SUCCESS);
    if (LOS_OK != uwRet)
    {
        dprintf("Set Inspect Status Err.\r\n");
    }

    /* delete self */
    if(LOS_OK != LOS_TaskDelete(g_uwTskHiID))
    {
        dprintf("TaskHi delete failed .\r\n");
        return LOS_NOK;
    }

    return LOS_OK;
}

static UINT32 Example_TaskLo(VOID)
{
    UINT32 uwRet;

    dprintf("Enter TaskLo Handler.\r\n");

    /* task delay 10 ticks, task will be suspend */
    uwRet = LOS_TaskDelay(10);
    if (uwRet != LOS_OK)
    {
        dprintf("Delay TaskLo Failed.\r\n");
        return LOS_NOK;
    }

    dprintf("TaskHi LOS_TaskSuspend Success.\r\n");

    /* resumed task g_uwTskHiID */
    uwRet = LOS_TaskResume(g_uwTskHiID);
    if (uwRet != LOS_OK)
    {
        dprintf("Resume TaskHi Failed.\r\n");
        uwRet = LOS_InspectStatusSetByID(LOS_INSPECT_TASK, LOS_INSPECT_STU_ERROR);
        if (LOS_OK != uwRet)
        {
            dprintf("Set Inspect Status Err.\r\n");
        }
        return LOS_NOK;
    }

    /* delete self */
    if(LOS_OK != LOS_TaskDelete(g_uwTskLoID))
    {
        dprintf("TaskLo delete failed .\r\n");
        return LOS_NOK;
    }

    return LOS_OK;
}

UINT32 Example_TskCaseEntry(VOID)
{
    UINT32 uwRet;
    TSK_INIT_PARAM_S stInitParam;

    /* lock task shcedue */
    LOS_TaskLock();

    dprintf("LOS_TaskLock() Success!\r\n");

    stInitParam.pfnTaskEntry = (TSK_ENTRY_FUNC)Example_TaskHi;
    stInitParam.usTaskPrio = TSK_PRIOR_HI;
    stInitParam.pcName = "HIGH_NAME";
    stInitParam.uwStackSize = LOSCFG_BASE_CORE_TSK_DEFAULT_STACK_SIZE;
    /* create high prio task */
    uwRet = LOS_TaskCreate(&g_uwTskHiID, &stInitParam);
    if (uwRet != LOS_OK)
    {
        LOS_TaskUnlock();

        dprintf("Example_TaskHi create Failed!\r\n");
        return LOS_NOK;
    }

    dprintf("Example_TaskHi create Success!\r\n");

    stInitParam.pfnTaskEntry = (TSK_ENTRY_FUNC)Example_TaskLo;
    stInitParam.usTaskPrio = TSK_PRIOR_LO;
    stInitParam.pcName = "LOW_NAME";
    stInitParam.uwStackSize = LOSCFG_BASE_CORE_TSK_DEFAULT_STACK_SIZE;
    /* create low prio task */
    uwRet = LOS_TaskCreate(&g_uwTskLoID, &stInitParam);
    if (uwRet != LOS_OK)
    {
        /* delete high prio task */
        if (LOS_OK != LOS_TaskDelete(g_uwTskHiID))
        {
            dprintf("TaskHi delete failed .\r\n");
        }

        LOS_TaskUnlock();

        dprintf("Example_TaskLo create Failed!\r\n");

        return LOS_NOK;
    }

    dprintf("Example_TaskLo create Success!\r\n");

    /* unlock task schedue */
    LOS_TaskUnlock();

    return uwRet;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */
