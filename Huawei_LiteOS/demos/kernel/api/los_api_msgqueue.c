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

#include "los_base.h"
#include "los_task.h"
#include "los_swtmr.h"
#include "los_hwi.h"
#include "los_queue.h"
#include "los_event.h"
#include "los_typedef.h"
#include "los_api_msgqueue.h"
#include "los_inspect_entry.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

static UINT32 g_uwQueue;

static CHAR abuf[] = "test is message x";

/* task1 send data */
static VOID *send_Entry(UINT32 uwParam)
{
    UINT32 i = 0,uwRet = 0;
    UINT32 uwlen = sizeof(abuf);

    while (i < API_MSG_NUM)
    {
        abuf[uwlen -2] = '0' + i;
        i++;

        /* write abuf data to queue */
        uwRet = LOS_QueueWrite(g_uwQueue, abuf, uwlen, 0);
        if(uwRet != LOS_OK)
        {
            dprintf("send message failure,error:%x \r\n",uwRet);
        }

        LOS_TaskDelay(5);
    }

    return NULL;
}

/* task2 recv data */
static VOID *recv_Entry(UINT32 uwParam)
{
    UINT32 uwReadbuf;
    UINT32 uwRet = LOS_OK;
    UINT32 uwMsgCount = 0;

    while (1)
    {
        /* read data from queue to uwReadbuf */
        uwRet = LOS_QueueRead(g_uwQueue, &uwReadbuf, 24, 0);
        if (uwRet != LOS_OK)
        {
            dprintf("recv message failure,error:%x \r\n",uwRet);
            break;
        }
        else
        {
            dprintf("recv message:%s \r\n", (CHAR *)uwReadbuf);
            uwMsgCount++;
        }

        (VOID)LOS_TaskDelay(5);
    }

    /* delete queue */
    while (LOS_OK != LOS_QueueDelete(g_uwQueue))
    {
        (VOID)LOS_TaskDelay(1);
    }

    dprintf("delete the queue success! \r\n");

    if (API_MSG_NUM == uwMsgCount)
    {
        uwRet = LOS_InspectStatusSetByID(LOS_INSPECT_MSG, LOS_INSPECT_STU_SUCCESS);
        if (LOS_OK != uwRet)
        {
            dprintf("Set Inspect Status Err \r\n");
        }
    }
    else
    {
        uwRet = LOS_InspectStatusSetByID(LOS_INSPECT_MSG, LOS_INSPECT_STU_ERROR);
        if (LOS_OK != uwRet)
        {
            dprintf("Set Inspect Status Err \r\n");
        }
    }

    return NULL;
}

UINT32 Example_MsgQueue(VOID)
{
    UINT32 uwRet = 0;
    UINT32 uwTask1, uwTask2;
    TSK_INIT_PARAM_S stInitParam1;

    /* create task1 */
    stInitParam1.pfnTaskEntry = send_Entry;
    stInitParam1.usTaskPrio = 9;
    stInitParam1.uwStackSize = LOSCFG_BASE_CORE_TSK_DEFAULT_STACK_SIZE;
    stInitParam1.pcName = "sendQueue";
    LOS_TaskLock();//lock task schedue
    uwRet = LOS_TaskCreate(&uwTask1, &stInitParam1);
    if (uwRet != LOS_OK)
    {
        dprintf("create task1 failed!,error:%x \r\n",uwRet);
        return uwRet;
    }

    /*create task2*/
    stInitParam1.pfnTaskEntry = recv_Entry;
    uwRet = LOS_TaskCreate(&uwTask2, &stInitParam1);
    if (uwRet != LOS_OK)
    {
        dprintf("create task2 failed!,error:%x \r\n",uwRet);
        return uwRet;
    }

    /* create queue */
    uwRet = LOS_QueueCreate("queue", 5, &g_uwQueue, 0, 24);
    if (uwRet != LOS_OK)
    {
        dprintf("create queue failure!,error:%x \r\n",uwRet);
    }

    dprintf("create the queue success! \r\n");
    LOS_TaskUnlock();//unlock task schedue

    return LOS_OK;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */
