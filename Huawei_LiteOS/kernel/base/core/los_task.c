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
#include "string.h"
#include "los_task.inc"
#include "los_base.ph"
#include "los_memory.ph"
#include "los_memstat.ph"
#include "los_priqueue.ph"
#include "los_sem.ph"
#include "los_mux.ph"
#if (LOSCFG_PLATFORM_EXC == YES)
#include "los_exc.ph"
#endif
#if (LOSCFG_KERNEL_TICKLESS == YES)
#include "los_tickless.ph"
#endif
#if (LOSCFG_BASE_CORE_CPUP == YES)
#include "los_cpup.ph"
#endif
#include "los_hw.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

LITE_OS_SEC_BSS  LOS_TASK_CB                         *g_pstTaskCBArray;
LITE_OS_SEC_BSS  ST_LOS_TASK                         g_stLosTask;
LITE_OS_SEC_BSS  UINT16                              g_usLosTaskLock;
LITE_OS_SEC_BSS  UINT32                              g_uwTskMaxNum;
LITE_OS_SEC_BSS  UINT32                              g_uwIdleTaskID;
LITE_OS_SEC_BSS  UINT32                              g_uwSwtmrTaskID;
LITE_OS_SEC_BSS  LOS_DL_LIST                         g_stTaskTimerList;
LITE_OS_SEC_BSS  LOS_DL_LIST                         g_stLosFreeTask;
LITE_OS_SEC_BSS  LOS_DL_LIST                         g_stTskRecyleList;
LITE_OS_SEC_BSS  TSK_SORTLINK_ATTRIBUTE_S            g_stTskSortLink;
LITE_OS_SEC_BSS  BOOL                                g_bTaskScheduled;

LITE_OS_SEC_DATA_INIT TSKSWITCHHOOK g_pfnTskSwitchHook = (TSKSWITCHHOOK)NULL; /*lint !e611*/
#if (LOSCFG_LIB_LIBC_NEWLIB_REENT == YES)
LITE_OS_SEC_DATA_INIT TSKSWITCHHOOK g_pfnTskSwitchImpurePtrHook = (TSKSWITCHHOOK)NULL; /*lint !e611*/
#endif
#if (LOSCFG_BASE_CORE_TSK_MONITOR == YES)
LITE_OS_SEC_DATA_INIT TSKSWITCHHOOK g_pfnUsrTskSwitchHook = (TSKSWITCHHOOK)NULL; /*lint !e611*/
#endif /* LOSCFG_BASE_CORE_TSK_MONITOR == YES */

#if (LOSCFG_BASE_CORE_EXC_TSK_SWITCH == YES)
LITE_OS_SEC_BSS OS_TASK_SWITCH_INFO g_astTskSwitchInfo;
#endif

#define CHECK_TASKID(uwTaskID)\
{\
   if (uwTaskID == g_uwIdleTaskID)\
   {\
       return LOS_ERRNO_TSK_OPERATE_IDLE;\
   }\
   else if (uwTaskID == g_uwSwtmrTaskID)\
   {\
       return LOS_ERRNO_TSK_SUSPEND_SWTMR_NOT_ALLOWED;\
   }\
   else if (OS_TSK_GET_INDEX(uwTaskID) >= g_uwTskMaxNum)\
   {\
       return LOS_ERRNO_TSK_ID_INVALID;\
   }\
}

#if (LOSCFG_KERNEL_TICKLESS == YES)
LITE_OS_SEC_TEXT_MINOR UINT32 osTaskNextSwitchTimeGet(VOID)
{
    LOS_TASK_CB *pstTaskCB;
    UINT32 uwTaskSortLinkTick = 0;
    LOS_DL_LIST *pstListObject;
    UINT32 uwTempTicks = 0;
    UINT32 uwIndex =0;

    for (uwIndex = 0; uwIndex < OS_TSK_SORTLINK_LEN; uwIndex++)
    {
        pstListObject = g_stTskSortLink.pstSortLink + (g_stTskSortLink.usCursor + uwIndex)%OS_TSK_SORTLINK_LEN;
        if (pstListObject->pstNext != pstListObject)
        {
            pstTaskCB = LOS_DL_LIST_ENTRY((pstListObject)->pstNext, LOS_TASK_CB, stTimerList);
            uwTempTicks = (uwIndex == 0) ? OS_TSK_SORTLINK_LEN : uwIndex;
            uwTempTicks += (UINT32)(UWROLLNUM(pstTaskCB->uwIdxRollNum) * OS_TSK_SORTLINK_LEN);
            if(uwTaskSortLinkTick == 0 || uwTaskSortLinkTick > uwTempTicks)
            {
               uwTaskSortLinkTick = uwTempTicks;
            }
        }
    }

    return uwTaskSortLinkTick;
}
#endif

/*****************************************************************************
 Function : osTskIdleBGD
 Description : Idle background.
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT WEAK VOID osIdleTask(VOID)
{
    while (1)
    {
#if (LOSCFG_KERNEL_TICKLESS == YES)
        osTicklessHandler();
#else
    #if (LOSCFG_KERNEL_RUNSTOP == YES)
        osEnterSleep();
    #endif
#endif
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
        osPriqueueDequeue(&pstTaskCB->stPendList);
        pstTaskCB->usTaskStatus &= (~OS_TASK_STATUS_READY);
        pstTaskCB->usPriority = usPriority;
        pstTaskCB->usTaskStatus |= OS_TASK_STATUS_READY;
        osPriqueueEnqueue(&pstTaskCB->stPendList, pstTaskCB->usPriority);
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
            osPriqueueEnqueue(&pstTaskCB->stPendList, pstTaskCB->usPriority);
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
 Function : osConvertTskStatus
 Description : Convert task status to string.
 Input       : usTaskStatus    --- task status
 Output      : None
 Return      : string
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT8 *osConvertTskStatus(UINT16 usTaskStatus)
{
    if (OS_TASK_STATUS_RUNNING & usTaskStatus)
    {
        return (UINT8 *)"Running";
    }
    else if (OS_TASK_STATUS_READY &  usTaskStatus)
    {
        return (UINT8 *)"Ready";
    }
    else if (OS_TASK_STATUS_DELAY &  usTaskStatus)
    {
        return (UINT8 *)"Delay";
    }
    else if (OS_TASK_STATUS_PEND & usTaskStatus)
    {
        if (OS_TASK_STATUS_TIMEOUT & usTaskStatus)
        {
            return (UINT8 *)"PendTimeOut";
        }

        return (UINT8 *)"Pend";
    }
    else if (OS_TASK_STATUS_SUSPEND & usTaskStatus)
    {
        return (UINT8 *)"Suspend";
    }
    else if (OS_TASK_STATUS_PEND_QUEUE& usTaskStatus)
    {
        if (OS_TASK_STATUS_TIMEOUT & usTaskStatus)
        {
            return (UINT8 *)"QueuePendTimeOut";
        }

        return (UINT8 *)"QueuePend";
    }

    return (UINT8 *)"Impossible";
}

LITE_OS_SEC_TEXT_MINOR UINT32 osGetTaskWaterLine(UINT32 uwTaskID)
{
    UINT32 *puwStack;
    UINT32 uwPeakUsed;

    if (OS_TASK_MAGIC_WORD == *(UINT32 *)(((LOS_TASK_CB *)g_pstTaskCBArray) + uwTaskID)->uwTopOfStack)
    {
        puwStack = (UINT32 *)((((LOS_TASK_CB *)g_pstTaskCBArray) + uwTaskID)->uwTopOfStack + 4);
        while ((puwStack < (UINT32 *)(((LOS_TASK_CB *)g_pstTaskCBArray) + uwTaskID)->pStackPointer) && (*puwStack == 0xCACACACA))
        {
            puwStack += 1;
        }
        uwPeakUsed = ((((LOS_TASK_CB *)g_pstTaskCBArray) + uwTaskID)->uwStackSize - ((UINT32)puwStack - (((LOS_TASK_CB *)g_pstTaskCBArray) + uwTaskID)->uwTopOfStack));
    }
    else
    {
        PRINT_ERR("CURRENT task %s stack overflow!\n", (((LOS_TASK_CB *)g_pstTaskCBArray) + uwTaskID)->pcTaskName);
        uwPeakUsed = 0xFFFFFFFF;
    }
    return uwPeakUsed;
}

/*****************************************************************************
 Function : osGetAllTskInfo
 Description : Get all task info.
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 osGetAllTskInfo(VOID)
{
    LOS_TASK_CB *pstTaskCB;
    UINT32       uwLoop;
#if (LOSCFG_BASE_CORE_CPUP == YES)
    CPUP_INFO_S *pstCpu = (CPUP_INFO_S *)NULL;
    CPUP_INFO_S *pstCpu10s = (CPUP_INFO_S *)NULL;
    CPUP_INFO_S *pstCpu1s = (CPUP_INFO_S *)NULL;
#endif

#if (LOSCFG_BASE_CORE_CPUP == YES)
    pstCpu = (CPUP_INFO_S *)LOS_MemAlloc((VOID *)OS_SYS_MEM_ADDR, sizeof(CPUP_INFO_S) * g_uwTskMaxNum);
    if (pstCpu == NULL)
    {
        PRINT_ERR("%s[%d] malloc failure!\n", __FUNCTION__, __LINE__);/*lint !e515*/
        return OS_ERROR;
    }
    (VOID)memset((VOID *)pstCpu, (int)0, sizeof(CPUP_INFO_S) * g_uwTskMaxNum);

    pstCpu10s = (CPUP_INFO_S *)LOS_MemAlloc((VOID *)OS_SYS_MEM_ADDR, sizeof(CPUP_INFO_S) * g_uwTskMaxNum);
    if (pstCpu10s == NULL)
    {
        PRINT_ERR("%s[%d] malloc failure!\n", __FUNCTION__, __LINE__);/*lint !e515*/
        (VOID)LOS_MemFree((VOID *)OS_SYS_MEM_ADDR, pstCpu);
        return OS_ERROR;
    }
    (VOID)memset((VOID *)pstCpu10s, (int)0, sizeof(CPUP_INFO_S) * g_uwTskMaxNum);

    pstCpu1s = (CPUP_INFO_S *)LOS_MemAlloc((VOID *)OS_SYS_MEM_ADDR, sizeof(CPUP_INFO_S) * g_uwTskMaxNum);
    if (pstCpu1s == NULL)
    {
        PRINT_ERR("%s[%d] malloc failure!\n", __FUNCTION__, __LINE__);/*lint !e515*/
        (VOID)LOS_MemFree((VOID *)OS_SYS_MEM_ADDR, pstCpu);
        (VOID)LOS_MemFree((VOID *)OS_SYS_MEM_ADDR, pstCpu10s);
        return OS_ERROR;
    }
    (VOID)memset((VOID *)pstCpu1s, (int)0, sizeof(CPUP_INFO_S) * g_uwTskMaxNum);

    LOS_TaskLock();
    (VOID)LOS_AllTaskCpuUsage(g_uwTskMaxNum, pstCpu, 0xffff);
    (VOID)LOS_AllTaskCpuUsage(g_uwTskMaxNum, pstCpu10s, 0);
    (VOID)LOS_AllTaskCpuUsage(g_uwTskMaxNum, pstCpu1s, 1);
    LOS_TaskUnlock();
#endif

    PRINT_ERR("\r\nName                          TID    Priority   Status       StackSize    WaterLine    StackPoint  TopOfStack   EventMask  SemID");/*lint !e515*/
#if (LOSCFG_BASE_CORE_CPUP == YES)
    PRINT_ERR(" CPUUSE   CPUUSE10s  CPUUSE1s  ");/*lint !e515*/
#endif /* LOSCFG_BASE_CORE_CPUP */
    PRINT_ERR("\n");/*lint !e515*/
    PRINT_ERR("----                          ---    --------   --------     ---------    ----------   ----------  ----------   ---------  -----");/*lint !e515*/
#if (LOSCFG_BASE_CORE_CPUP == YES)
    PRINT_ERR("  ------- ---------  ---------");/*lint !e515*/
#endif /* LOSCFG_BASE_CORE_CPUP */
    PRINT_ERR("\n");/*lint !e515*/

    for (uwLoop = 0; uwLoop < g_uwTskMaxNum; uwLoop++)
    {
        //uvIntSave = LOS_IntLock();
        pstTaskCB = (((LOS_TASK_CB *)g_pstTaskCBArray) + uwLoop);
        if (pstTaskCB->usTaskStatus & OS_TASK_STATUS_UNUSED)
        {
            //LOS_IntRestore(uvIntSave);
            continue;
        }

        PRINT_ERR("%-30s, 0x%-5x, %-11d, %-13s, 0x%-11x, 0x%-11x, 0x%-10x, 0x%-11x, 0x%-9x",
                          pstTaskCB->pcTaskName,
                          pstTaskCB->uwTaskID,
                          pstTaskCB->usPriority,
                          osConvertTskStatus(pstTaskCB->usTaskStatus),
                          pstTaskCB->uwStackSize,
                          osGetTaskWaterLine(pstTaskCB->uwTaskID),
                          (UINT32)pstTaskCB->pStackPointer,
                          pstTaskCB->uwTopOfStack,
                          pstTaskCB->uwEventMask);/*lint !e515*/

        if (pstTaskCB->pTaskSem != NULL)
        {
            PRINT_ERR("0x%-7x", ((SEM_CB_S *)pstTaskCB->pTaskSem)->usSemID);/*lint !e516*/
        }
        else
        {
            PRINT_ERR("0x%-7x", 0xFFFF);
        }

#if (LOSCFG_BASE_CORE_CPUP == YES)
        PRINT_ERR("%2d.%-7d"
                          "%2d.%-9d"
                          "%2d.%-6d",
                          pstCpu[pstTaskCB->uwTaskID].uwUsage / LOS_CPUP_PRECISION_MULT,
                          pstCpu[pstTaskCB->uwTaskID].uwUsage % LOS_CPUP_PRECISION_MULT,
                          pstCpu10s[pstTaskCB->uwTaskID].uwUsage / LOS_CPUP_PRECISION_MULT,
                          pstCpu10s[pstTaskCB->uwTaskID].uwUsage % LOS_CPUP_PRECISION_MULT,
                          pstCpu1s[pstTaskCB->uwTaskID].uwUsage / LOS_CPUP_PRECISION_MULT,
                          pstCpu1s[pstTaskCB->uwTaskID].uwUsage % LOS_CPUP_PRECISION_MULT);/*lint !e515 !e516*/
#endif /* LOSCFG_BASE_CORE_CPUP */
        PRINT_ERR("\n");/*lint !e515*/
    }

#if (LOSCFG_BASE_CORE_CPUP == YES)
    (VOID)LOS_MemFree((VOID *)OS_SYS_MEM_ADDR, pstCpu);
    (VOID)LOS_MemFree((VOID *)OS_SYS_MEM_ADDR, pstCpu10s);
    (VOID)LOS_MemFree((VOID *)OS_SYS_MEM_ADDR, pstCpu1s);
#endif

    return LOS_OK;
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

    (VOID)memset((VOID *)(&g_stLosTask), 0, sizeof(g_stLosTask));
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

    (VOID)memset((VOID *)pstListObject, 0, uwSize);
    g_stTskSortLink.pstSortLink = pstListObject;
    g_stTskSortLink.usCursor = 0;
    for (uwIndex = 0; uwIndex < OS_TSK_SORTLINK_LEN; uwIndex++, pstListObject++)
    {
        LOS_ListInit(pstListObject);
    }

#if ((LOSCFG_PLATFORM_EXC == YES) && (LOSCFG_SAVE_EXC_INFO == YES))
    osExcRegister((EXC_INFO_TYPE)OS_EXC_TYPE_TSK, (EXC_INFO_SAVE_CALLBACK)LOS_TaskInfoGet, &g_uwTskMaxNum);
#endif

#if (LOSCFG_LIB_LIBC_NEWLIB_REENT == YES)
    extern LITE_OS_SEC_TEXT VOID osTaskSwitchImpurePtr(VOID);
    g_pfnTskSwitchImpurePtrHook = osTaskSwitchImpurePtr;
#endif
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

    (VOID)memset((VOID *)(&stTaskInitParam), 0, sizeof(TSK_INIT_PARAM_S));
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
 Function : LOS_NextTaskIDGet
 Description : get id of next running task.
 Input       : None
 Output      : None
 Return      : task id
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_NextTaskIDGet(VOID)
{
    if (NULL == g_stLosTask.pstNewTask)
    {
        return LOS_ERRNO_TSK_ID_INVALID;
    }
    return g_stLosTask.pstNewTask->uwTaskID;
}

/*****************************************************************************
 Function : LOS_CurTaskNameGet
 Description : get name of current running task.
 Input       : None
 Output      : None
 Return      : task name
 *****************************************************************************/
LITE_OS_SEC_TEXT CHAR *LOS_CurTaskNameGet(VOID)
{
    CHAR *pcTaskName = NULL;

    if (NULL != g_stLosTask.pstRunTask)
    {
        pcTaskName = g_stLosTask.pstRunTask->pcTaskName;
    }

    return pcTaskName;
}

/*****************************************************************************
 Function : osTaskSwitchCheck
 Description : Check task switch
 Input       : Node
 Output      : None
 Return      : None
 *****************************************************************************/
#if (LOSCFG_BASE_CORE_TSK_MONITOR == YES)
LITE_OS_SEC_TEXT VOID osTaskSwitchCheck(VOID)
{
    if ((*(UINT32 *)(g_stLosTask.pstRunTask->uwTopOfStack)) != OS_TASK_MAGIC_WORD)
    {
        PRINT_ERR("CURRENT task ID: %s:%d stack overflow!\n", g_stLosTask.pstRunTask->pcTaskName, g_stLosTask.pstRunTask->uwTaskID);
    }
    if (((UINT32)(g_stLosTask.pstNewTask->pStackPointer) <= g_stLosTask.pstNewTask->uwTopOfStack) ||
        ((UINT32)(g_stLosTask.pstNewTask->pStackPointer) > g_stLosTask.pstNewTask->uwTopOfStack + g_stLosTask.pstNewTask->uwStackSize))
    {
        PRINT_ERR("HIGHEST task ID: %s:%d SP error!\n", g_stLosTask.pstNewTask->pcTaskName, g_stLosTask.pstNewTask->uwTaskID);
        PRINT_ERR("HIGHEST task StackPointer: 0x%x TopOfStack: 0x%x\n", (UINT32)(g_stLosTask.pstNewTask->pStackPointer), g_stLosTask.pstNewTask->uwTopOfStack);
    }

#if (LOSCFG_BASE_CORE_EXC_TSK_SWITCH == YES)
    /* record task switch info */
    g_astTskSwitchInfo.auwPID[g_astTskSwitchInfo.ucIdx] = (UINT16)(g_stLosTask.pstNewTask->uwTaskID);
    memcpy(g_astTskSwitchInfo.acName[g_astTskSwitchInfo.ucIdx], g_stLosTask.pstNewTask->pcTaskName, LOS_TASK_NAMELEN);
    g_astTskSwitchInfo.acName[g_astTskSwitchInfo.ucIdx][LOS_TASK_NAMELEN -1] = '\0';

    if (++g_astTskSwitchInfo.ucIdx == OS_TASK_SWITCH_INFO_COUNT)
    {
        g_astTskSwitchInfo.ucIdx = 0;
        g_astTskSwitchInfo.ucIsFull |= 0x80;
    }
#endif

    if (g_pfnUsrTskSwitchHook != NULL)
    {
        g_pfnUsrTskSwitchHook();
    }

#if (LOSCFG_BASE_CORE_CPUP == YES)
    osTskCycleEndStart();
#endif /* LOSCFG_BASE_CORE_CPUP */
}

LITE_OS_SEC_TEXT_MINOR VOID osTaskMonInit(VOID)
{
#if (LOSCFG_BASE_CORE_EXC_TSK_SWITCH == YES)
    (VOID)memset(&g_astTskSwitchInfo, 0, sizeof(OS_TASK_SWITCH_INFO));
    g_astTskSwitchInfo.ucIsFull = 0x7F & OS_TASK_SWITCH_INFO_COUNT;
#if ((LOSCFG_PLATFORM_EXC == YES) && (LOSCFG_SAVE_EXC_INFO == YES))
    osExcRegister((EXC_INFO_TYPE)OS_EXC_TYPE_TSK_SWITCH, (EXC_INFO_SAVE_CALLBACK)LOS_TaskSwitchInfoGet, &g_astTskSwitchInfo);
#endif
#endif
    g_pfnTskSwitchHook = osTaskSwitchCheck;
    g_pfnUsrTskSwitchHook = (TSKSWITCHHOOK)NULL; /*lint !e611*/
    return ;
}
#endif

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

    OS_TASK_ID_CHECK(uwTaskID);

    pstTaskCB = OS_TCB_FROM_TID(uwTaskID);

    (VOID)pstTaskCB->pfnTaskEntry(pstTaskCB->uwArg);

    g_usLosTaskLock = 0;
    (VOID)LOS_TaskDelete(pstTaskCB->uwTaskID);
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

    if (NULL == pstInitParam->pcName)
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

    if (((pstInitParam->usTaskPrio) == OS_TASK_PRIORITY_LOWEST)
        && (pstInitParam->pfnTaskEntry != OS_IDLE_TASK_ENTRY))
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
    pstInitParam->uwStackSize = ALIGN(pstInitParam->uwStackSize , LOSCFG_STACK_POINT_ALIGN_SIZE);

    if (pstInitParam->uwStackSize < LOSCFG_BASE_CORE_TSK_MIN_STACK_SIZE)
    {
        return LOS_ERRNO_TSK_STKSZ_TOO_SMALL;
    }

    uvIntSave = LOS_IntLock();
    while (!LOS_ListEmpty(&g_stTskRecyleList))
    {
        pstTaskCB = OS_TCB_FROM_PENDLIST(LOS_DL_LIST_FIRST(&g_stTskRecyleList)); /*lint !e413*/
        LOS_ListDelete(LOS_DL_LIST_FIRST(&g_stTskRecyleList));
        LOS_ListAdd(&g_stLosFreeTask, &pstTaskCB->stPendList);
        (VOID)LOS_MemFree(OS_TASK_STACK_ADDR, (VOID *)pstTaskCB->uwTopOfStack);
        pstTaskCB->uwTopOfStack = (UINT32)NULL;
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

    pTopStack = (VOID *)LOS_MemAllocAlign(OS_TASK_STACK_ADDR, pstInitParam->uwStackSize, LOSCFG_STACK_POINT_ALIGN_SIZE);

    if (NULL == pTopStack)
    {
        uvIntSave = LOS_IntLock();
        LOS_ListAdd(&g_stLosFreeTask, &pstTaskCB->stPendList);
        uwErrRet = LOS_ERRNO_TSK_NO_MEMORY;
        OS_GOTO_ERREND();
    }

    pStackPtr = osTskStackInit(uwTaskID, pstInitParam->uwStackSize, pTopStack);
    pstTaskCB->pStackPointer     = pStackPtr;
    pstTaskCB->uwArg             = pstInitParam->uwArg;
    pstTaskCB->uwTopOfStack      = (UINT32)pTopStack;
    pstTaskCB->uwStackSize       = pstInitParam->uwStackSize;
    pstTaskCB->pTaskSem          = NULL;
    pstTaskCB->pTaskMux          = NULL;
    pstTaskCB->usTaskStatus      = OS_TASK_STATUS_SUSPEND;
    pstTaskCB->usPriority        = pstInitParam->usTaskPrio;
    pstTaskCB->pfnTaskEntry      = pstInitParam->pfnTaskEntry;
    pstTaskCB->uwEvent.uwEventID = 0xFFFFFFFF;
    pstTaskCB->uwEventMask       = 0;
    pstTaskCB->pcTaskName        = pstInitParam->pcName;
    pstTaskCB->puwMsg = NULL;
#if (LOSCFG_LIB_LIBC_NEWLIB_REENT == YES)
    /* Initialise this task's Newlib reent structure. */
    _REENT_INIT_PTR(&(pstTaskCB->stNewLibReent));
#endif

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

#if (LOSCFG_BASE_CORE_CPUP == YES)
    g_pstCpup[pstTaskCB->uwTaskID].uwID = pstTaskCB->uwTaskID;
    g_pstCpup[pstTaskCB->uwTaskID].usStatus = pstTaskCB->usTaskStatus;
#endif

    osPriqueueEnqueue(&pstTaskCB->stPendList, pstTaskCB->usPriority);
    g_stLosTask.pstNewTask = LOS_DL_LIST_ENTRY(osPriqueueTop(), LOS_TASK_CB, stPendList); /*lint !e413*/

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
        osPriqueueEnqueue(&pstTaskCB->stPendList, pstTaskCB->usPriority);
        if (g_bTaskScheduled)
        {
            (VOID)LOS_IntRestore(uvIntSave);
            LOS_Schedule();
            return LOS_OK;
        }
        g_stLosTask.pstNewTask = LOS_DL_LIST_ENTRY(osPriqueueTop(), LOS_TASK_CB, stPendList); /*lint !e413*/
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

    CHECK_TASKID(uwTaskID);
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
        osPriqueueDequeue(&pstTaskCB->stPendList);
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

    CHECK_TASKID(uwTaskID);
    uvIntSave = LOS_IntLock();

    pstTaskCB = OS_TCB_FROM_TID(uwTaskID);

    usTempStatus = pstTaskCB->usTaskStatus;

    if (OS_TASK_STATUS_UNUSED & usTempStatus)
    {
        uwErrRet = LOS_ERRNO_TSK_NOT_CREATED;
        OS_GOTO_ERREND();
    }

    /* If the task is running and scheduler is locked then you can not delete it */
    if ((OS_TASK_STATUS_RUNNING & usTempStatus) && (g_usLosTaskLock != 0))
    {
        PRINT_INFO("In case of task lock, task deletion is not recommended\n");
        g_usLosTaskLock = 0;
    }

    if (OS_TASK_STATUS_READY & usTempStatus)
    {
        osPriqueueDequeue(&pstTaskCB->stPendList);
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
#if (LOSCFG_BASE_CORE_CPUP == YES)
    (VOID)memset((VOID *)&g_pstCpup[pstTaskCB->uwTaskID], 0, sizeof(OS_CPUP_S));
#endif
    g_stLosTask.pstNewTask = LOS_DL_LIST_ENTRY(osPriqueueTop(), LOS_TASK_CB, stPendList); /*lint !e413*/
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
        (VOID)LOS_MemFree(OS_TASK_STACK_ADDR, (VOID *)pstTaskCB->uwTopOfStack);
        pstTaskCB->uwTopOfStack = (UINT32)NULL;
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
        osPriqueueDequeue(&(g_stLosTask.pstRunTask->stPendList));
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

    if (uwTaskID == g_uwSwtmrTaskID)
    {
        return LOS_ERRNO_TSK_OPERATE_SWTMR;
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
        osPriqueueDequeue(&pstTaskCB->stPendList);
        pstTaskCB->usTaskStatus &= (~OS_TASK_STATUS_READY);
        pstTaskCB->usPriority = usTaskPrio;
        pstTaskCB->usTaskStatus |= OS_TASK_STATUS_READY;
        osPriqueueEnqueue(&pstTaskCB->stPendList, pstTaskCB->usPriority);
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

/**************************************************************************
 Function    : osTaskWait
 Description : pend a task in pstList
 Input       : pstList
               uwTimeOut -- Expiry time
 Output      : none
 Return      : LOS_OK on success or LOS_NOK on failure
**************************************************************************/
LITE_OS_SEC_TEXT VOID osTaskWait(LOS_DL_LIST *pstList, UINT32 uwTaskStatus, UINT32 uwTimeOut)
{
    LOS_TASK_CB *pstRunTsk;
    LOS_DL_LIST *pstPendObj;

    pstRunTsk = g_stLosTask.pstRunTask;
    osPriqueueDequeue(&pstRunTsk->stPendList);
    pstRunTsk->usTaskStatus &= (~OS_TASK_STATUS_READY);
    pstPendObj = &pstRunTsk->stPendList;
    pstRunTsk->usTaskStatus |= uwTaskStatus;
    LOS_ListTailInsert(pstList,pstPendObj);
    if (uwTimeOut != LOS_WAIT_FOREVER)
    {
        pstRunTsk->usTaskStatus |= OS_TASK_STATUS_TIMEOUT;
        osTaskAdd2TimerList((LOS_TASK_CB *)pstRunTsk, uwTimeOut);
    }
}

/**************************************************************************
 Function    : osTaskWake
 Description : delete the task from pendlist and also add to the priqueue
 Input       : pstResumedTask --> resumed task
 Output      : pstResumedTask
 Return      : none
**************************************************************************/
LITE_OS_SEC_TEXT VOID osTaskWake(LOS_TASK_CB *pstResumedTask, UINT32 uwTaskStatus)
{
    LOS_ListDelete(&pstResumedTask->stPendList);
    pstResumedTask->usTaskStatus &= (~uwTaskStatus);
    if (pstResumedTask->usTaskStatus & OS_TASK_STATUS_TIMEOUT)
    {
        osTimerListDelete(pstResumedTask);
        pstResumedTask->usTaskStatus &= (~OS_TASK_STATUS_TIMEOUT);
    }
    if (!(pstResumedTask->usTaskStatus & OS_TASK_STATUS_SUSPEND))
    {
        pstResumedTask->usTaskStatus |= OS_TASK_STATUS_READY;
        osPriqueueEnqueue(&pstResumedTask->stPendList, pstResumedTask->usPriority);
    }
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
    if(!(g_stLosTask.pstRunTask->usTaskStatus & OS_TASK_STATUS_READY))
    {
        return LOS_OK;
    }
    uvIntSave = LOS_IntLock();
    uwTskCount = osPriqueueSize(g_stLosTask.pstRunTask->usPriority);
    if (uwTskCount > 1)
    {
        LOS_ListDelete(&(g_stLosTask.pstRunTask->stPendList));
        g_stLosTask.pstRunTask->usTaskStatus |= OS_TASK_STATUS_READY;
        osPriqueueEnqueue(&(g_stLosTask.pstRunTask->stPendList), g_stLosTask.pstRunTask->usPriority);
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

/*****************************************************************************
 Function : LOS_TaskInfoGet
 Description : Get the information of the task
 Input       : uwTaskID
 Output      : pstTaskInfo
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_TaskInfoGet(UINT32 uwTaskID, TSK_INFO_S *pstTaskInfo)
{
    UINT32    uwIntSave;
    LOS_TASK_CB *pstTaskCB;
    UINT32 * puwStack;

    if (NULL == pstTaskInfo)
    {
        return LOS_ERRNO_TSK_PTR_NULL;
    }

    if (OS_CHECK_TSK_PID_NOIDLE(uwTaskID))
    {
        return LOS_ERRNO_TSK_ID_INVALID;
    }

    pstTaskCB = OS_TCB_FROM_TID(uwTaskID);
    uwIntSave = LOS_IntLock();

    if (OS_TASK_STATUS_UNUSED & pstTaskCB->usTaskStatus)
    {
        (VOID)LOS_IntRestore(uwIntSave);
        return LOS_ERRNO_TSK_NOT_CREATED;
    }

    pstTaskInfo->uwSP = (UINT32)pstTaskCB->pStackPointer;
    pstTaskInfo->usTaskStatus = pstTaskCB->usTaskStatus;
    pstTaskInfo->usTaskPrio = pstTaskCB->usPriority;
    pstTaskInfo->uwStackSize  = pstTaskCB->uwStackSize;
    pstTaskInfo->uwTopOfStack = pstTaskCB->uwTopOfStack;
    pstTaskInfo->uwEvent = pstTaskCB->uwEvent;
    pstTaskInfo->uwEventMask = pstTaskCB->uwEventMask;
    pstTaskInfo->uwSemID = pstTaskCB->pTaskSem != NULL ? ((SEM_CB_S *)(pstTaskCB->pTaskSem))->usSemID : LOSCFG_BASE_IPC_SEM_LIMIT;
    pstTaskInfo->uwMuxID = pstTaskCB->pTaskMux != NULL ? ((MUX_CB_S *)(pstTaskCB->pTaskMux))->ucMuxID : LOSCFG_BASE_IPC_MUX_LIMIT;
    pstTaskInfo->pTaskSem = pstTaskCB->pTaskSem;
    pstTaskInfo->pTaskMux = pstTaskCB->pTaskMux;
    pstTaskInfo->uwTaskID = uwTaskID;

    (VOID)strncpy(pstTaskInfo->acName, pstTaskCB->pcTaskName, LOS_TASK_NAMELEN - 1);
    pstTaskInfo->acName[LOS_TASK_NAMELEN - 1] = '\0';

    pstTaskInfo->uwBottomOfStack = TRUNCATE(((UINT32)(pstTaskCB->uwTopOfStack) + (pstTaskCB->uwStackSize)), OS_TASK_STACK_ADDR_ALIGN);
    pstTaskInfo->uwCurrUsed = pstTaskInfo->uwBottomOfStack - pstTaskInfo->uwSP;

    if (OS_TASK_MAGIC_WORD == *(UINT32 *)pstTaskInfo->uwTopOfStack)
    {
        puwStack = (UINT32 *)(pstTaskInfo->uwTopOfStack + 4);
        while ((puwStack < (UINT32 *)pstTaskInfo->uwSP) && (*puwStack == 0xCACACACA))
        {
            puwStack += 1;
        }

        pstTaskInfo->uwPeakUsed = pstTaskCB->uwStackSize - ((UINT32)puwStack - pstTaskInfo->uwTopOfStack);
        pstTaskInfo->bOvf = FALSE;
    }
    else
    {
        pstTaskInfo->uwPeakUsed = 0xFFFFFFFF;
        pstTaskInfo->bOvf = TRUE;
    }

    (VOID)LOS_IntRestore(uwIntSave);

    return LOS_OK;
}

/*****************************************************************************
 Function : LOS_TaskStatusGet
 Description : Get status of the task
 Input       : uwTaskID
 Output      : puwTaskStatus
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_TaskStatusGet(UINT32 uwTaskID, UINT32 *puwTaskStatus)
{
    UINT32    uwIntSave;
    LOS_TASK_CB *pstTaskCB;

    if (NULL == puwTaskStatus)
    {
        return LOS_ERRNO_TSK_PTR_NULL;
    }

    if (OS_CHECK_TSK_PID_NOIDLE(uwTaskID))
    {
        return LOS_ERRNO_TSK_ID_INVALID;
    }

    pstTaskCB = OS_TCB_FROM_TID(uwTaskID);
    uwIntSave = LOS_IntLock();

    if (OS_TASK_STATUS_UNUSED & pstTaskCB->usTaskStatus)
    {
        (VOID)LOS_IntRestore(uwIntSave);
        return LOS_ERRNO_TSK_NOT_CREATED;
    }

    *puwTaskStatus = pstTaskCB->usTaskStatus;

    (VOID)LOS_IntRestore(uwIntSave);

    return LOS_OK;
}

#if (LOSCFG_BASE_CORE_EXC_TSK_SWITCH == YES)
/*****************************************************************************
 Function : LOS_TaskSwitchInfoGet
 Description : save the information of the task switch
 Input       : uwIdx
 Output      : pTaskSwitchInfo
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_TaskSwitchInfoGet(UINT32 uwIdx, UINT32 *puwTaskSwitchInfo)
{
    UINTPTR uvIntSave;

    if (uwIdx >= OS_TASK_SWITCH_INFO_COUNT)
    {
        uwIdx -= OS_TASK_SWITCH_INFO_COUNT;
    }

    if (NULL == puwTaskSwitchInfo)
    {
        return LOS_ERRNO_TSK_PTR_NULL;
    }

    uvIntSave = LOS_IntLock();

    (*puwTaskSwitchInfo) = g_astTskSwitchInfo.auwPID[uwIdx];
    memcpy((VOID *)(puwTaskSwitchInfo + 1), g_astTskSwitchInfo.acName[uwIdx], LOS_TASK_NAMELEN);

    (VOID)LOS_IntRestore(uvIntSave);
    return LOS_OK;
}
#endif

/*****************************************************************************
Function   : LOS_TaskInfoMonitor
Description: Get all task info
Input      : None
Return     : LOS_OK on success ,or OS_ERROR on failure
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_TaskInfoMonitor(VOID)
{
    UINT32 uwRet;

    uwRet = osGetAllTskInfo();

    return uwRet;
}

/*****************************************************************************
 Function : LOS_TaskIsRunning
 Description : Check if LiteOS has been started.
 Input       : VOID
 Output      : VOID
 Return      : TRUE means LiteOS was started, FALSE means not.
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR BOOL LOS_TaskIsRunning(VOID)
{
    return g_bTaskScheduled;
}

/*****************************************************************************
 Function : LOS_NewTaskIDGet
 Description : get id of current new task.
 Input       : None
 Output      : None
 Return      : task id
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_NewTaskIDGet(VOID)
{
    if (NULL == g_stLosTask.pstNewTask)
    {
        return LOS_ERRNO_TSK_ID_INVALID;
    }
    return g_stLosTask.pstNewTask->uwTaskID;
}

/*****************************************************************************
 Function : LOS_TaskNameGet
 Description : get Name of current new task.
 Input       : uwTaskID -----task id
 Output      : None
 Return      : task name
 *****************************************************************************/
LITE_OS_SEC_TEXT CHAR* LOS_TaskNameGet(UINT32 uwTaskID)
{
    UINT32    uwIntSave;
    LOS_TASK_CB *pstTaskCB;

    if (OS_CHECK_TSK_PID_NOIDLE(uwTaskID))
    {
        return NULL;
    }

    pstTaskCB = OS_TCB_FROM_TID(uwTaskID);

    uwIntSave = LOS_IntLock();
    if (OS_TASK_STATUS_UNUSED & pstTaskCB->usTaskStatus)
    {
        (VOID)LOS_IntRestore(uwIntSave);
        return NULL;
    }
    (VOID)LOS_IntRestore(uwIntSave);

    return pstTaskCB->pcTaskName;
}

#if (LOSCFG_LIB_LIBC_NEWLIB_REENT == YES)
/*****************************************************************************
 Function : osTaskSwitchImpurePtr
 Description : Switch Newlib's _impure_ptr to point to the next run task.
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT VOID osTaskSwitchImpurePtr(VOID)
{
    /* Switch Newlib's _impure_ptr variable to point to the _reent
       structure specific to next run task. */
    _impure_ptr = &(g_stLosTask.pstNewTask->stNewLibReent);
}
#endif

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

