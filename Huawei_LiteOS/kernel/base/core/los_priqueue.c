/*----------------------------------------------------------------------------
 * Copyright (c) <2013-2015>, <Huawei Technologies Co., Ltd>
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

#include "los_priqueue.inc"
#include "los_base.ph"
#include "los_task.ph"
#include "los_memory.h"
#include "los_compiler.h"


LITE_OS_SEC_BSS LOS_DL_LIST *g_pstLosPriorityQueueList;
static LITE_OS_SEC_BSS UINT32 g_uwPriQueueBitmap = 0;

#define PRIQUEUE_PRIOR0_BIT                     (UINT32)0x80000000

LITE_OS_SEC_TEXT VOID osPriqueueInit(VOID)
{
    UINT32 uwPri = 0;
    UINT32 uwSize = 0;

    uwSize = OS_PRIORITY_QUEUE_PRIORITYNUM * sizeof(LOS_DL_LIST);
    g_pstLosPriorityQueueList = (LOS_DL_LIST *)LOS_MemAlloc(m_aucSysMem0, uwSize);
    if (NULL == g_pstLosPriorityQueueList)
    {
        return;
    }

    for (uwPri = 0; uwPri < OS_PRIORITY_QUEUE_PRIORITYNUM; ++uwPri)
    {
        LOS_ListInit(&g_pstLosPriorityQueueList[uwPri]);
    }
}

LITE_OS_SEC_TEXT VOID osPriqueueEnqueue(LOS_DL_LIST *ptrPQItem, UINT32 uwPri)
{
    if (LOS_ListEmpty(&g_pstLosPriorityQueueList[uwPri]))
    {
        g_uwPriQueueBitmap |= (PRIQUEUE_PRIOR0_BIT >> uwPri);
    }

    LOS_ListTailInsert(&g_pstLosPriorityQueueList[uwPri], ptrPQItem);
}

LITE_OS_SEC_TEXT VOID osPriqueueDequeue(LOS_DL_LIST *ptrPQItem)
{
    LOS_TASK_CB *pstRunTsk;
    LOS_ListDelete(ptrPQItem);

    pstRunTsk = LOS_DL_LIST_ENTRY(ptrPQItem, LOS_TASK_CB, stPendList);  /*lint !e413*/
    if (LOS_ListEmpty(&g_pstLosPriorityQueueList[pstRunTsk->usPriority]))
    {
        g_uwPriQueueBitmap &= (~(PRIQUEUE_PRIOR0_BIT >> pstRunTsk->usPriority));
    }
}

LITE_OS_SEC_TEXT LOS_DL_LIST *osPriqueueTop(VOID)
{
    UINT32 uwPri = 0;

    if (0 != g_uwPriQueueBitmap)
    {
        uwPri = CLZ(g_uwPriQueueBitmap);
        return LOS_DL_LIST_FIRST(&g_pstLosPriorityQueueList[uwPri]);
    }

    return (LOS_DL_LIST *)NULL;
}

LITE_OS_SEC_TEXT UINT32 osPriqueueSize(UINT32 uwPri)
{
    UINT32      uwItemCnt = 0;
    LOS_DL_LIST *pstCurPQNode = (LOS_DL_LIST *)NULL;

    LOS_DL_LIST_FOR_EACH(pstCurPQNode, &g_pstLosPriorityQueueList[uwPri])
    {
        ++uwItemCnt;
    }

    return uwItemCnt;
}

LITE_OS_SEC_TEXT UINT32 osPriqueueTotalSize(VOID)
{
    UINT32 uwPri = 0;
    UINT32 uwTotalSize = 0;

    for (uwPri = 0; uwPri < OS_PRIORITY_QUEUE_PRIORITYNUM; ++uwPri)
    {
        uwTotalSize += osPriqueueSize(uwPri);
    }

    return uwTotalSize;
}
