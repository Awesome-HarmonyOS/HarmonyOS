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

#include "los_task.inc"
#include "los_base.ph"
#include "los_memory.ph"
#include "los_priqueue.ph"
#include "los_sem.ph"
#include "los_mux.ph"

#include "los_hw.h"
#ifdef LOSCFG_LIB_LIBC
#include "string.h"
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */
extern int sys_suspend(void);
LITE_OS_SEC_BSS  LOS_TASK_CB                         *g_pstTaskCBArray;
LITE_OS_SEC_BSS  ST_LOS_TASK                         g_stLosTask;
LITE_OS_SEC_BSS  UINT16                                  g_usLosTaskLock;
LITE_OS_SEC_BSS  UINT32                                  g_uwTskMaxNum;
LITE_OS_SEC_BSS  UINT32                                  g_uwIdleTaskID;
LITE_OS_SEC_BSS  UINT32                                  g_uwSwtmrTaskID;
LITE_OS_SEC_DATA LOS_DL_LIST                         g_stTaskTimerList;
LITE_OS_SEC_DATA_INIT LOS_DL_LIST                    g_stLosFreeTask;
LITE_OS_SEC_DATA_INIT LOS_DL_LIST                    g_stTskRecyleList;
LITE_OS_SEC_BSS  TSK_SORTLINK_ATTRIBUTE_S            g_stTskSortLink;
LITE_OS_SEC_BSS  BOOL                                g_bTaskScheduled;
/*lint -e64 -e552*/
#if (LOSCFG_BASE_CORE_TSK_MONITOR == YES)
TSKSWITCHHOOK g_pfnTskSwitchHook = NULL;
#endif /* LOSCFG_BASE_CORE_TSK_MONITOR == YES */

/*****************************************************************************
 Function : osTskIdleBGD
 Description : Idle background.
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT VOID osIdleTask(VOID)
{

    while (1)
    {

    }
}

/*****************************************************************************
 Function : osTaskPriModify
 Description : Change task priority.
 Input       : pstTaskCB    --- task control block
                 usPriority      --- priority
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR VOID osTaskPriModify(LOS_TASK_CB *pstTaskCB, UINT16 usPriority)
{
    if (pstTaskCB->usTaskStatus & OS_TASK_STATUS_READY)
    {
        LOS_PriqueueDequeue(&pstTaskCB->stPendList);
        pstTaskCB->usTaskStatus &= (~OS_TASK_STATUS_READY);
        pstTaskCB->usPriority = usPriority;
        pstTaskCB->usTaskStatus |= OS_TASK_STATUS_READY;
        LOS_PriqueueEnqueue(&pstTaskCB->stPendList, pstTaskCB->usPriority);
    }
    else
    {
        pstTaskCB->usPriority = usPriority;
    }
}

/*****************************************************************************
 Function : osTaskAdd2TimerList
 Description : Add task to sorted delay list.
 Input       : pstTaskCB    --- task control block
               uwTimeout    --- wait time, ticks
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT VOID osTaskAdd2TimerList(LOS_TASK_CB *pstTaskCB, UINT32 uwTimeout)
{
    LOS_TASK_CB *pstTskDelay;
    LOS_DL_LIST *pstListObject;
    UINT32 uwSortIndex;
    UINT32 uwRollNum;

    uwSortIndex = uwTimeout & OS_TSK_SORTLINK_MASK;
    uwRollNum = (uwTimeout >> OS_TSK_SORTLINK_LOGLEN);
    (uwSortIndex > 0) ? 0 : (uwRollNum--);
    EVALUATE_L(pstTaskCB->uwIdxRollNum, uwRollNum);
    uwSortIndex = (uwSortIndex + g_stTskSortLink.usCursor);
    uwSortIndex = uwSortIndex & OS_TSK_SORTLINK_MASK;
    EVALUATE_H(pstTaskCB->uwIdxRollNum, uwSortIndex);
    pstListObject = g_stTskSortLink.pstSortLink + uwSortIndex;
    if (pstListObject->pstNext == pstListObject)
    {
        LOS_ListTailInsert(pstListObject, &pstTaskCB->stTimerList);
    }
    else
    {
        pstTskDelay = LOS_DL_LIST_ENTRY((pstListObject)->pstNext, LOS_TASK_CB, stTimerList); /*lint !e413*/
        do
        {
            if (UWROLLNUM(pstTskDelay->uwIdxRollNum) <= UWROLLNUM(pstTaskCB->uwIdxRollNum))
            {
                UWROLLNUMSUB(pstTaskCB->uwIdxRollNum, pstTskDelay->uwIdxRollNum);
            }
            else
            {
                UWROLLNUMSUB(pstTskDelay->uwIdxRollNum, pstTaskCB->uwIdxRollNum);
                break;
            }

            pstTskDelay = LOS_DL_LIST_ENTRY(pstTskDelay->stTimerList.pstNext, LOS_TASK_CB, stTimerList); /*lint !e413*/
        } while (&pstTskDelay->stTimerList != (pstListObject));

        LOS_ListTailInsert(&pstTskDelay->stTimerList, &pstTaskCB->stTimerList);
    }
}


LITE_OS_SEC_TEXT VOID osTimerListDelete(LOS_TASK_CB *pstTaskCB)
{
    LOS_DL_LIST  *pstListObject;
    LOS_TASK_CB  *pstNextTask;
    UINT32 uwSortIndex;

    uwSortIndex = UWSORTINDEX(pstTaskCB->uwIdxRollNum);
    pstListObject = g_stTskSortLink.pstSortLink + uwSortIndex;

    if (pstListObject != pstTaskCB->stTimerList.pstNext)
    {
        pstNextTask = LOS_DL_LIST_ENTRY(pstTaskCB->stTimerList.pstNext, LOS_TASK_CB, stTimerList); /*lint !e413*/
        UWROLLNUMADD(pstNextTask->uwIdxRollNum, pstTaskCB->uwIdxRollNum);
    }

    LOS_ListDelete(&pstTaskCB->stTimerList);
}

LITE_OS_SEC_TEXT VOID osTaskScan(VOID)
{
    LOS_TASK_CB *pstTaskCB;
    BOOL bNeedSchedule = FALSE;
    LOS_DL_LIST *pstListObject;
    UINT16 usTempStatus;

    g_stTskSortLink.usCursor = (g_stTskSortLink.usCursor + 1) % OS_TSK_SORTLINK_LEN;
    pstListObject = g_stTskSortLink.pstSortLink + g_stTskSortLink.usCursor;
    if (pstListObject->pstNext == pstListObject)
    {
        return;
    }

    for (pstTaskCB = LOS_DL_LIST_ENTRY((pstListObject)->pstNext, LOS_TASK_CB, stTimerList);&pstTaskCB->stTimerList != (pstListObject);) /*lint !e413*/
    {
        usTempStatus = pstTaskCB->usTaskStatus;
        if (UWROLLNUM(pstTaskCB->uwIdxRollNum) > 0)
        {
            UWROLLNUMDEC(pstTaskCB->uwIdxRollNum);
            break;
        }

        LOS_ListDelete(&pstTaskCB->stTimerList);
        if (OS_TASK_STATUS_PEND & usTempStatus)
        {
            pstTaskCB->usTaskStatus &= ~(OS_TASK_STATUS_PEND);
            LOS_ListDelete(&pstTaskCB->stPendList);
            pstTaskCB->pTaskSem = NULL;
            pstTaskCB->pTaskMux = NULL;
        }
        else if (OS_TASK_STATUS_EVENT & usTempStatus)
        {
            pstTaskCB->usTaskStatus &= ~(OS_TASK_STATUS_EVENT);
        }
        else if (OS_TASK_STATUS_PEND_QUEUE & usTempStatus)
        {
            LOS_ListDelete(&pstTaskCB->stPendList);
            pstTaskCB->usTaskStatus &= ~(OS_TASK_STATUS_PEND_QUEUE);
        }
        else
        {
            pstTaskCB->usTaskStatus &= ~(OS_TASK_STATUS_DELAY);
        }

        if (!((OS_TASK_STATUS_SUSPEND) & usTempStatus))
        {
            pstTaskCB->usTaskStatus |= OS_TASK_STATUS_READY;
            LOS_PriqueueEnqueue(&pstTaskCB->stPendList, pstTaskCB->usPriority);
            bNeedSchedule = TRUE;
        }

        pstTaskCB = LOS_DL_LIST_ENTRY(pstListObject->pstNext, LOS_TASK_CB, stTimerList); /*lint !e413*/
    }

    if (bNeedSchedule)
    {
        LOS_Schedule();
    }
}

/*****************************************************************************
 Function : osTaskInit
 Description : Task init function.
 Input       : None
 Output      : None
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 osTaskInit(VOID)
{
    UINT32 uwSize;
    UINT32 uwIndex;
    LOS_DL_LIST *pstListObject;

    uwSize = (g_uwTskMaxNum + 1) * sizeof(LOS_TASK_CB);
    g_pstTaskCBArray = (LOS_TASK_CB *)LOS_MemAlloc(m_aucSysMem0, uwSize);
    if (NULL == g_pstTaskCBArray)
    {
        return LOS_ERRNO_TSK_NO_MEMORY;
    }

    (VOID)memset(g_pstTaskCBArray, 0, uwSize);
    LOS_ListInit(&g_stTaskTimerList);
    LOS_ListInit(&g_stLosFreeTask);
    LOS_ListInit(&g_stTskRecyleList);
    for (uwIndex = 0; uwIndex <= LOSCFG_BASE_CORE_TSK_LIMIT; uwIndex++)
    {
        g_pstTaskCBArray[uwIndex].usTaskStatus = OS_TASK_STATUS_UNUSED;
        g_pstTaskCBArray[uwIndex].uwTaskID = uwIndex;
        LOS_ListTailInsert(&g_stLosFreeTask, &g_pstTaskCBArray[uwIndex].stPendList);
    }

    (VOID)memset((void *)(&g_stLosTask), 0, sizeof(g_stLosTask));
    g_stLosTask.pstRunTask = &g_pstTaskCBArray[g_uwTskMaxNum];
    g_stLosTask.pstRunTask->uwTaskID = uwIndex;
    g_stLosTask.pstRunTask->usTaskStatus = (OS_TASK_STATUS_UNUSED | OS_TASK_STATUS_RUNNING);
    g_stLosTask.pstRunTask->usPriority = OS_TASK_PRIORITY_LOWEST + 1;
    osPriqueueInit();
    uwSize = sizeof(LOS_DL_LIST) * OS_TSK_SORTLINK_LEN;
    pstListObject = (LOS_DL_LIST *)LOS_MemAlloc(m_aucSysMem0, uwSize);
    if (NULL == pstListObject)
    {
        return LOS_ERRNO_TSK_NO_MEMORY;
    }

    (VOID)memset((void *)pstListObject, 0, uwSize);
    g_stTskSortLink.pstSortLink = pstListObject;
    g_stTskSortLink.usCursor = 0;
    for (uwIndex = 0; uwIndex < OS_TSK_SORTLINK_LEN; uwIndex++, pstListObject++)
    {
        LOS_ListInit(pstListObject);
    }

    return LOS_OK;
}


/*****************************************************************************
 Function : osIdleTaskCreate
 Description : Create idle task.
 Input       : None
 Output      : None
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 osIdleTaskCreate(VOID)
{
    UINT32 uwRet;
    TSK_INIT_PARAM_S stTaskInitParam;

    (VOID)memset((void *)(&stTaskInitParam), 0, sizeof(TSK_INIT_PARAM_S));
    stTaskInitParam.pfnTaskEntry = (TSK_ENTRY_FUNC)osIdleTask;
    stTaskInitParam.uwStackSize = LOSCFG_BASE_CORE_TSK_IDLE_STACK_SIZE;
    stTaskInitParam.pcName = "IdleCore000";
    stTaskInitParam.usTaskPrio = OS_TASK_PRIORITY_LOWEST;
    uwRet = LOS_TaskCreate(&g_uwIdleTaskID, &stTaskInitParam);

    if (uwRet != LOS_OK)
    {
        return uwRet;
    }

    return LOS_OK;
}

/*****************************************************************************
 Function : LOS_CurTaskIDGet
 Description : get id of current running task.
 Input       : None
 Output      : None
 Return      : task id
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_CurTaskIDGet(VOID)
{
    if (NULL == g_stLosTask.pstRunTask)
    {
        return LOS_ERRNO_TSK_ID_INVALID;
    }
    return g_stLosTask.pstRunTask->uwTaskID;
}

/*****************************************************************************
 Function : osTaskSelfDelete
 Description : Delete self task
 Input       : uwTaskID --- Task ID, uwIntSave
                 uvIntSave   --- interrupt flag
 Output      : None
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 osTaskSelfDelete(UINT32 uwTaskID, UINTPTR uvIntSave)
{
    LOS_TASK_CB *pstTaskCB;
    UINT16 usTempStatus;

    pstTaskCB = OS_TCB_FROM_TID(uwTaskID);
    usTempStatus = pstTaskCB->usTaskStatus;
    if (OS_TASK_STATUS_READY & usTempStatus)
    {
        LOS_PriqueueDequeue(&pstTaskCB->stPendList);
        pstTaskCB->usTaskStatus &= (~OS_TASK_STATUS_READY);
    }
    else if ((OS_TASK_STATUS_PEND | OS_TASK_STATUS_PEND_QUEUE) & usTempStatus)
    {
        LOS_ListDelete(&pstTaskCB->stPendList);
        if ((OS_TASK_STATUS_DELAY | OS_TASK_STATUS_TIMEOUT) & usTempStatus)
        {
            osTimerListDelete(pstTaskCB);
        }
    }

    pstTaskCB->usTaskStatus &= (~(OS_TASK_STATUS_SUSPEND));
    pstTaskCB->usTaskStatus |= OS_TASK_STATUS_UNUSED;
    pstTaskCB->uwEvent.uwEventID = 0xFFFFFFFF;
    pstTaskCB->uwEventMask = 0;

    g_stLosTask.pstNewTask = LOS_DL_LIST_ENTRY(LOS_PriqueueTop(), LOS_TASK_CB, stPendList); /*lint !e413*/
    if (OS_TASK_STATUS_RUNNING & pstTaskCB->usTaskStatus)
    {
        LOS_ListTailInsert(&g_stTskRecyleList, &pstTaskCB->stPendList);
        g_stLosTask.pstRunTask = &g_pstTaskCBArray[g_uwTskMaxNum];
        g_stLosTask.pstRunTask->uwTaskID = uwTaskID;
        g_stLosTask.pstRunTask->usTaskStatus = pstTaskCB->usTaskStatus;
        g_stLosTask.pstRunTask->uwTopOfStack = pstTaskCB->uwTopOfStack;
        g_stLosTask.pstRunTask->pcTaskName = pstTaskCB->pcTaskName;
        pstTaskCB->usTaskStatus = OS_TASK_STATUS_UNUSED;
        (VOID)LOS_IntRestore(uvIntSave);
        osSchedule();
        return LOS_OK;
    }
    else if (OS_TASK_STATUS_UNUSED & pstTaskCB->usTaskStatus)
    {
        (VOID)LOS_IntRestore(uvIntSave);
        osSchedule();
        return LOS_OK;
    }

    (VOID)LOS_IntRestore(uvIntSave);
    return LOS_OK;
}

/*****************************************************************************
 Function : osTaskEntry
 Description : All task entry
 Input       : uwTaskID     --- The ID of the task to be run
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT VOID osTaskEntry(UINT32 uwTaskID)
{
    LOS_TASK_CB *pstTaskCB;
    UINT32 uwIntSave;

    OS_TASK_ID_CHECK(uwTaskID);

    pstTaskCB = OS_TCB_FROM_TID(uwTaskID);
    if (pstTaskCB->pThreadJoin)
    {
        pstTaskCB->pThreadJoinRetval =  pstTaskCB->pfnTaskEntry( pstTaskCB->auwArgs[0],
                                        pstTaskCB->auwArgs[1],
                                        pstTaskCB->auwArgs[2],
                                        pstTaskCB->auwArgs[3]);
    }
    else
    {
        pstTaskCB->pfnTaskEntry( pstTaskCB->auwArgs[0],
                                 pstTaskCB->auwArgs[1],
                                 pstTaskCB->auwArgs[2],
                                 pstTaskCB->auwArgs[3]);    /*lint !e534*/
    }

    if (pstTaskCB->usTaskStatus & LOS_TASK_STATUS_DETACHED)
    {
        uwIntSave = LOS_IntLock();
        g_usLosTaskLock = 0;
        (VOID)osTaskSelfDelete(pstTaskCB->uwTaskID, uwIntSave);
    }
    /* join mode: waiting for child task done */
    else
    {
        uwIntSave = LOS_IntLock();
        g_usLosTaskLock = 0;
        (VOID)LOS_IntRestore(uwIntSave);

        LOS_TaskLock();
        if (pstTaskCB->pThreadJoin)
        {
            if (LOS_SemPost((UINT32)(((SEM_CB_S *)pstTaskCB->pThreadJoin)->usSemID)) != LOS_OK)
            {
                PRINT_ERR("osTaskEntry LOS_SemPost fail!\n");
            }
            pstTaskCB->pThreadJoin = NULL;
        }

        if (OS_TASK_STATUS_READY & pstTaskCB->usTaskStatus)
        {
            LOS_PriqueueDequeue(&pstTaskCB->stPendList);
            pstTaskCB->usTaskStatus &= (~OS_TASK_STATUS_READY);
        }
        LOS_TaskUnlock();
        LOS_Schedule();
    }

}

/*****************************************************************************
 Function : LOS_TaskCreateOnly
 Description : Create a task and suspend
 Input       : pstInitParam --- Task init parameters
 Output      : puwTaskID    --- Save task ID
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_TaskCreateOnly(UINT32 *puwTaskID, TSK_INIT_PARAM_S *pstInitParam)
{
    UINT32 uwTaskID = 0;
    UINTPTR uvIntSave;
    VOID  *pTopStack;
    VOID  *pStackPtr;
    LOS_TASK_CB *pstTaskCB;
    UINT32 uwErrRet = OS_ERROR;

    if (NULL == puwTaskID)
    {
        return LOS_ERRNO_TSK_ID_INVALID;
    }

    if (NULL == pstInitParam)
    {
        return LOS_ERRNO_TSK_PTR_NULL;
    }

    if (NULL == pstInitParam->pcName )
    {
        return LOS_ERRNO_TSK_NAME_EMPTY;
    }

    if (NULL == pstInitParam->pfnTaskEntry)
    {
        return LOS_ERRNO_TSK_ENTRY_NULL;
    }

    if ((pstInitParam->usTaskPrio) > OS_TASK_PRIORITY_LOWEST)
    {
        return LOS_ERRNO_TSK_PRIOR_ERROR;
    }

    if (pstInitParam->uwStackSize > OS_SYS_MEM_SIZE)
    {
        return LOS_ERRNO_TSK_STKSZ_TOO_LARGE;
    }

    if (0 == pstInitParam->uwStackSize)
    {
        pstInitParam->uwStackSize = LOSCFG_BASE_CORE_TSK_DEFAULT_STACK_SIZE;
    }
    pstInitParam->uwStackSize = ALIGN(pstInitParam->uwStackSize , 8);

    if (pstInitParam->uwStackSize < LOS_TASK_MIN_STACK_SIZE)
    {
        return LOS_ERRNO_TSK_STKSZ_TOO_SMALL;
    }

    uvIntSave = LOS_IntLock();
    while (!LOS_ListEmpty(&g_stTskRecyleList))
    {
        pstTaskCB = OS_TCB_FROM_PENDLIST(LOS_DL_LIST_FIRST(&g_stTskRecyleList)); /*lint !e413*/
        LOS_ListDelete(LOS_DL_LIST_FIRST(&g_stTskRecyleList));
        LOS_ListAdd(&g_stLosFreeTask, &pstTaskCB->stPendList);
        (VOID)LOS_MemFree(m_aucSysMem0, (VOID *)pstTaskCB->uwTopOfStack);
    }

    if (LOS_ListEmpty(&g_stLosFreeTask))
    {
        uwErrRet = LOS_ERRNO_TSK_TCB_UNAVAILABLE;
        OS_GOTO_ERREND();
    }

    pstTaskCB = OS_TCB_FROM_PENDLIST(LOS_DL_LIST_FIRST(&g_stLosFreeTask)); /*lint !e413*/
    LOS_ListDelete(LOS_DL_LIST_FIRST(&g_stLosFreeTask));
    (VOID)LOS_IntRestore(uvIntSave);
    uwTaskID = pstTaskCB->uwTaskID;

    pTopStack = (void *)LOS_MemAllocAlign(m_aucSysMem0, pstInitParam->uwStackSize, 8);

    if (NULL == pTopStack)
    {
        uvIntSave = LOS_IntLock();
        LOS_ListAdd(&g_stLosFreeTask, &pstTaskCB->stPendList);
        uwErrRet = LOS_ERRNO_TSK_NO_MEMORY;
        OS_GOTO_ERREND();
    }

    pStackPtr = osTskStackInit(uwTaskID, pstInitParam->uwStackSize, pTopStack);
    pstTaskCB->pStackPointer = pStackPtr;
    pstTaskCB->auwArgs[0] = pstInitParam->auwArgs[0];
    pstTaskCB->auwArgs[1] = pstInitParam->auwArgs[1];
    pstTaskCB->auwArgs[2] = pstInitParam->auwArgs[2];
    pstTaskCB->auwArgs[3] = pstInitParam->auwArgs[3];
    pstTaskCB->uwTopOfStack = (UINT32)pTopStack;
    pstTaskCB->uwStackSize = pstInitParam->uwStackSize;
    pstTaskCB->pTaskSem = NULL;
    pstTaskCB->pThreadJoin = NULL;
    pstTaskCB->pTaskMux = NULL;
    pstTaskCB->usTaskStatus = OS_TASK_STATUS_SUSPEND;
    pstTaskCB->usTaskStatus |= (pstInitParam->uwResved ? LOS_TASK_STATUS_DETACHED : 0);/*set the task is detached or joinable*/
    pstTaskCB->usPriority = pstInitParam->usTaskPrio;
    pstTaskCB->pfnTaskEntry = pstInitParam->pfnTaskEntry;
    pstTaskCB->uwEvent.uwEventID = 0xFFFFFFFF;
    pstTaskCB->uwEventMask = 0;
    pstTaskCB->pcTaskName   = pstInitParam->pcName;
    pstTaskCB->puwMsg = NULL;

    *puwTaskID = uwTaskID;
    return LOS_OK;

LOS_ERREND:
    (VOID)LOS_IntRestore(uvIntSave);
    return uwErrRet;
}


/*****************************************************************************
 Function : LOS_TaskCreate
 Description : Create a task
 Input       : pstInitParam --- Task init parameters
 Output      : puwTaskID    --- Save task ID
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_TaskCreate(UINT32 *puwTaskID, TSK_INIT_PARAM_S *pstInitParam)
{
    UINT32 uwRet = LOS_OK;
    UINTPTR uvIntSave;
    LOS_TASK_CB *pstTaskCB;

    uwRet = LOS_TaskCreateOnly(puwTaskID, pstInitParam);
    if (LOS_OK != uwRet)
    {
        return uwRet;
    }
    pstTaskCB = OS_TCB_FROM_TID(*puwTaskID);

    uvIntSave = LOS_IntLock();
    pstTaskCB->usTaskStatus &= (~OS_TASK_STATUS_SUSPEND);
    pstTaskCB->usTaskStatus |= OS_TASK_STATUS_READY;

    LOS_PriqueueEnqueue(&pstTaskCB->stPendList, pstTaskCB->usPriority);
    g_stLosTask.pstNewTask = LOS_DL_LIST_ENTRY(LOS_PriqueueTop(), LOS_TASK_CB, stPendList); /*lint !e413*/

    if ((g_bTaskScheduled) && (g_usLosTaskLock == 0))
    {
        if (g_stLosTask.pstRunTask != g_stLosTask.pstNewTask)
        {
            if (LOS_CHECK_SCHEDULE)
            {
                (VOID)LOS_IntRestore(uvIntSave);
                osSchedule();
                return LOS_OK;
            }
        }
    }

    (VOID)LOS_IntRestore(uvIntSave);
    return LOS_OK;
}

/*****************************************************************************
 Function : LOS_TaskResume
 Description : Resume suspend task
 Input       : uwTaskID --- Task ID
 Output      : None
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_TaskResume(UINT32 uwTaskID)
{
    UINTPTR uvIntSave;
    LOS_TASK_CB *pstTaskCB;
    UINT16 usTempStatus;
    UINT32 uwErrRet = OS_ERROR;

    if (uwTaskID > LOSCFG_BASE_CORE_TSK_LIMIT)
    {
        return LOS_ERRNO_TSK_ID_INVALID;
    }

    pstTaskCB = OS_TCB_FROM_TID(uwTaskID);
    uvIntSave = LOS_IntLock();
    usTempStatus = pstTaskCB->usTaskStatus;

    if (OS_TASK_STATUS_UNUSED & usTempStatus)
    {
        uwErrRet = LOS_ERRNO_TSK_NOT_CREATED;
        OS_GOTO_ERREND();
    }
    else if (!(OS_TASK_STATUS_SUSPEND & usTempStatus))
    {
        uwErrRet = LOS_ERRNO_TSK_NOT_SUSPENDED;
        OS_GOTO_ERREND();
    }

    pstTaskCB->usTaskStatus &= (~OS_TASK_STATUS_SUSPEND);
    if (!(OS_CHECK_TASK_BLOCK & pstTaskCB->usTaskStatus) )
    {
        pstTaskCB->usTaskStatus |= OS_TASK_STATUS_READY;
        LOS_PriqueueEnqueue(&pstTaskCB->stPendList, pstTaskCB->usPriority);
        if (g_bTaskScheduled)
        {
            (VOID)LOS_IntRestore(uvIntSave);
            LOS_Schedule();
            return LOS_OK;
        }
        g_stLosTask.pstNewTask = LOS_DL_LIST_ENTRY(LOS_PriqueueTop(), LOS_TASK_CB, stPendList); /*lint !e413*/
    }

    (VOID)LOS_IntRestore(uvIntSave);
    return LOS_OK;

LOS_ERREND:
    (VOID)LOS_IntRestore(uvIntSave);
    return uwErrRet;
}

/*****************************************************************************
 Function : LOS_TaskSuspend
 Description : Suspend task
 Input       : uwTaskID --- Task ID
 Output      : None
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_TaskSuspend(UINT32 uwTaskID)
{
    UINTPTR uvIntSave;
    LOS_TASK_CB *pstTaskCB;
    UINT16 usTempStatus;
    UINT32 uwErrRet = OS_ERROR;

    if (uwTaskID == g_uwIdleTaskID)
    {
        return LOS_ERRNO_TSK_OPERATE_IDLE;
    }

    if (uwTaskID == g_uwSwtmrTaskID)
    {
        return LOS_ERRNO_TSK_SUSPEND_SWTMR_NOT_ALLOWED;
    }

    if (OS_TSK_GET_INDEX(uwTaskID) >= g_uwTskMaxNum)
    {
        return LOS_ERRNO_TSK_ID_INVALID;
    }

    pstTaskCB = OS_TCB_FROM_TID(uwTaskID);
    uvIntSave = LOS_IntLock();
    usTempStatus = pstTaskCB->usTaskStatus;
    if (OS_TASK_STATUS_UNUSED & usTempStatus)
    {
        uwErrRet = LOS_ERRNO_TSK_NOT_CREATED;
        OS_GOTO_ERREND();
    }

    if (OS_TASK_STATUS_SUSPEND & usTempStatus)
    {
        uwErrRet = LOS_ERRNO_TSK_ALREADY_SUSPENDED;
        OS_GOTO_ERREND();
    }

    if ((OS_TASK_STATUS_RUNNING & usTempStatus) && (g_usLosTaskLock != 0))
    {
        uwErrRet = LOS_ERRNO_TSK_SUSPEND_LOCKED;
        OS_GOTO_ERREND();
    }

    if (OS_TASK_STATUS_READY & usTempStatus)
    {
        LOS_PriqueueDequeue(&pstTaskCB->stPendList);
        pstTaskCB->usTaskStatus &= (~OS_TASK_STATUS_READY);
    }

    pstTaskCB->usTaskStatus |= OS_TASK_STATUS_SUSPEND;
    if (uwTaskID == g_stLosTask.pstRunTask->uwTaskID)
    {
        (VOID)LOS_IntRestore(uvIntSave);
        LOS_Schedule();
        return LOS_OK;
    }

    (VOID)LOS_IntRestore(uvIntSave);
    return LOS_OK;

LOS_ERREND:
    (VOID)LOS_IntRestore(uvIntSave);
    return uwErrRet;
}

/*****************************************************************************
 Function : LOS_TaskDelete
 Description : Delete a task
 Input       : uwTaskID --- Task ID
 Output      : None
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_TaskDelete(UINT32 uwTaskID)
{
    UINTPTR uvIntSave;
    LOS_TASK_CB *pstTaskCB;
    UINT16 usTempStatus;
    UINT32 uwErrRet = OS_ERROR;

    if (uwTaskID == g_uwIdleTaskID)
    {
        return LOS_ERRNO_TSK_OPERATE_IDLE;
    }

    if (OS_TSK_GET_INDEX(uwTaskID) >= g_uwTskMaxNum)
    {
        return LOS_ERRNO_TSK_ID_INVALID;
    }

    uvIntSave = LOS_IntLock();

    pstTaskCB = OS_TCB_FROM_TID(uwTaskID);

    usTempStatus = pstTaskCB->usTaskStatus;

    if (OS_TASK_STATUS_UNUSED & pstTaskCB->usTaskStatus)
    {
        uwErrRet = LOS_ERRNO_TSK_NOT_CREATED;
        OS_GOTO_ERREND();
    }

    /* If the task is running and scheduler is locked then you can not delete it */
    if ((OS_TASK_STATUS_RUNNING & usTempStatus) && (g_usLosTaskLock != 0))
    {
        uwErrRet = LOS_ERRNO_TSK_DELETE_LOCKED;
        OS_GOTO_ERREND();
    }

    if (OS_TASK_STATUS_READY & usTempStatus)
    {
        LOS_PriqueueDequeue(&pstTaskCB->stPendList);
        pstTaskCB->usTaskStatus &= (~OS_TASK_STATUS_READY);
    }
    else if ((OS_TASK_STATUS_PEND & usTempStatus) || (OS_TASK_STATUS_PEND_QUEUE & usTempStatus))
    {
        LOS_ListDelete(&pstTaskCB->stPendList);
    }

    if ((OS_TASK_STATUS_DELAY | OS_TASK_STATUS_TIMEOUT) & usTempStatus)
    {
        osTimerListDelete(pstTaskCB);
    }

    pstTaskCB->usTaskStatus &= (~(OS_TASK_STATUS_SUSPEND));
    pstTaskCB->usTaskStatus |= OS_TASK_STATUS_UNUSED;
    pstTaskCB->uwEvent.uwEventID = 0xFFFFFFFF;
    pstTaskCB->uwEventMask = 0;

    g_stLosTask.pstNewTask = LOS_DL_LIST_ENTRY(LOS_PriqueueTop(), LOS_TASK_CB, stPendList); /*lint !e413*/
    if (OS_TASK_STATUS_RUNNING & pstTaskCB->usTaskStatus)
    {
        LOS_ListTailInsert(&g_stTskRecyleList, &pstTaskCB->stPendList);
        g_stLosTask.pstRunTask = &g_pstTaskCBArray[g_uwTskMaxNum];
        g_stLosTask.pstRunTask->uwTaskID = uwTaskID;
        g_stLosTask.pstRunTask->usTaskStatus = pstTaskCB->usTaskStatus;
        g_stLosTask.pstRunTask->uwTopOfStack = pstTaskCB->uwTopOfStack;
        g_stLosTask.pstRunTask->pcTaskName = pstTaskCB->pcTaskName;
        pstTaskCB->usTaskStatus = OS_TASK_STATUS_UNUSED;
        (VOID)LOS_IntRestore(uvIntSave);
        osSchedule();
        return LOS_OK;
    }
    else
    {
        pstTaskCB->usTaskStatus = OS_TASK_STATUS_UNUSED;
        LOS_ListAdd(&g_stLosFreeTask, &pstTaskCB->stPendList);
        (VOID)LOS_MemFree(m_aucSysMem0, (VOID *)pstTaskCB->uwTopOfStack);
    }

    (VOID)LOS_IntRestore(uvIntSave);
    return LOS_OK;

LOS_ERREND:
    (VOID)LOS_IntRestore(uvIntSave);
    return uwErrRet;
}

/*****************************************************************************
 Function : LOS_TaskDelay
 Description : delay the current task
 Input       : uwTick    --- time
 Output      :None
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_TaskDelay(UINT32 uwTick)
{
    UINTPTR uvIntSave;

    if (OS_INT_ACTIVE)
    {
        return LOS_ERRNO_TSK_DELAY_IN_INT;
    }

    if (g_usLosTaskLock != 0)
    {
        return LOS_ERRNO_TSK_DELAY_IN_LOCK;
    }

    if (uwTick == 0)
    {
        return LOS_TaskYield();
    }
    else
    {
        uvIntSave = LOS_IntLock();
        LOS_PriqueueDequeue(&(g_stLosTask.pstRunTask->stPendList));
        g_stLosTask.pstRunTask->usTaskStatus &= (~OS_TASK_STATUS_READY);
        osTaskAdd2TimerList((LOS_TASK_CB *)g_stLosTask.pstRunTask, uwTick);
        g_stLosTask.pstRunTask->usTaskStatus |= OS_TASK_STATUS_DELAY;
        (VOID)LOS_IntRestore(uvIntSave);
        LOS_Schedule();
    }

    return LOS_OK;
}

/*****************************************************************************
 Function : LOS_TaskPriGet
 Description : Get the priority of the task
 Input       : uwTaskID
 Output      : None
 Return      : TSK_PRIOR_T on success or OS_INVALID on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT16 LOS_TaskPriGet(UINT32 uwTaskID)
{
    UINTPTR uvIntSave;
    LOS_TASK_CB *pstTaskCB;
    UINT16 usPriority;

    if (OS_CHECK_TSK_PID_NOIDLE(uwTaskID))
    {
       return (UINT16)OS_INVALID;
    }

    pstTaskCB = OS_TCB_FROM_TID(uwTaskID);

    uvIntSave = LOS_IntLock();

    if (OS_TASK_STATUS_UNUSED & pstTaskCB->usTaskStatus)
    {
        (VOID)LOS_IntRestore(uvIntSave);
        return (UINT16)OS_INVALID;
    }

    usPriority = pstTaskCB->usPriority;
    (VOID)LOS_IntRestore(uvIntSave);
    return usPriority;
}

/*****************************************************************************
 Function : LOS_TaskPriSet
 Description : Set the priority of the task
 Input       : uwTaskID
               usTaskPrio
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_TaskPriSet(UINT32 uwTaskID, UINT16 usTaskPrio)
{
    BOOL   bIsReady;
    UINTPTR uvIntSave;
    LOS_TASK_CB *pstTaskCB;
    UINT16 usTempStatus;

    if (usTaskPrio > OS_TASK_PRIORITY_LOWEST)
    {
        return LOS_ERRNO_TSK_PRIOR_ERROR;
    }

    if (uwTaskID == g_uwIdleTaskID)
    {
        return LOS_ERRNO_TSK_OPERATE_IDLE;
    }

    if (OS_CHECK_TSK_PID_NOIDLE(uwTaskID))
    {
        return LOS_ERRNO_TSK_ID_INVALID;
    }

    pstTaskCB = OS_TCB_FROM_TID(uwTaskID);
    uvIntSave = LOS_IntLock();
    usTempStatus = pstTaskCB->usTaskStatus;
    if (OS_TASK_STATUS_UNUSED & usTempStatus)
    {
        (VOID)LOS_IntRestore(uvIntSave);
        return LOS_ERRNO_TSK_NOT_CREATED;
    }
    /* delete the task and insert with right priority into ready queue */
    bIsReady = (OS_TASK_STATUS_READY & usTempStatus);
    if (bIsReady)
    {
        LOS_PriqueueDequeue(&pstTaskCB->stPendList);
        pstTaskCB->usTaskStatus &= (~OS_TASK_STATUS_READY);
        pstTaskCB->usPriority = usTaskPrio;
        pstTaskCB->usTaskStatus |= OS_TASK_STATUS_READY;
        LOS_PriqueueEnqueue(&pstTaskCB->stPendList, pstTaskCB->usPriority);
    }
    else
    {
        pstTaskCB->usPriority = usTaskPrio;
    }

    (VOID)LOS_IntRestore(uvIntSave);

    /* delete the task and insert with right priority into ready queue */
    if (bIsReady)
    {
        LOS_Schedule();
    }
    
    return LOS_OK;
}

/*****************************************************************************
 Function : LOS_CurTaskPriSet
 Description : Set the priority of the current task
 Input       : usTaskPrio
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_CurTaskPriSet(UINT16 usTaskPrio)
{
    UINT32 uwRet;
    uwRet = LOS_TaskPriSet(g_stLosTask.pstRunTask->uwTaskID, usTaskPrio);
    return uwRet;
}

/*****************************************************************************
 Function : LOS_TaskYield
 Description : Adjust the procedure order of specified task
 Input       : usTaskPrio
               uwNextTask
 Output      : None
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_TaskYield(VOID)
{
    UINT32 uwTskCount = 0;
    UINTPTR uvIntSave;

    if(g_stLosTask.pstRunTask->uwTaskID >= g_uwTskMaxNum)
    {
        return LOS_ERRNO_TSK_ID_INVALID;
    }
    uvIntSave = LOS_IntLock();
    uwTskCount = LOS_PriqueueSize(g_stLosTask.pstRunTask->usPriority);
    if (uwTskCount > 1)
    {
        LOS_ListDelete(&(g_stLosTask.pstRunTask->stPendList));
        g_stLosTask.pstRunTask->usTaskStatus |= OS_TASK_STATUS_READY;
        LOS_PriqueueEnqueue(&(g_stLosTask.pstRunTask->stPendList), g_stLosTask.pstRunTask->usPriority);
    }
    else
    {
        (VOID)LOS_IntRestore(uvIntSave);
        return LOS_ERRNO_TSK_YIELD_NOT_ENOUGH_TASK;
    }

    (VOID)LOS_IntRestore(uvIntSave);
    LOS_Schedule();
    return LOS_OK;
}

/*****************************************************************************
 Function : LOS_TaskLock
 Description : Task lock
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR VOID LOS_TaskLock(VOID)
{
    UINTPTR uvIntSave;

    uvIntSave = LOS_IntLock();
    g_usLosTaskLock++;
    (VOID)LOS_IntRestore(uvIntSave);
}

/*****************************************************************************
 Function : LOS_TaskUnlock
 Description : Task unlock
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR VOID LOS_TaskUnlock(VOID)
{
    UINTPTR uvIntSave;

    uvIntSave = LOS_IntLock();
    if (g_usLosTaskLock > 0)
    {
        g_usLosTaskLock--;
        if (0 == g_usLosTaskLock)
        {
            (VOID)LOS_IntRestore(uvIntSave);
            LOS_Schedule();
            return;
        }
    }

    (VOID)LOS_IntRestore(uvIntSave);
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */
