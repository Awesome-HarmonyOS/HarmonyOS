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
#include "los_swtmr.inc"
#include "los_base.ph"
#include "los_membox.ph"
#include "los_memory.ph"
#include "los_queue.ph"
#include "los_task.ph"
#include "los_hwi.h"
#if (LOSCFG_PLATFORM_EXC == YES)
#include "los_exc.h"
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

#if (LOSCFG_BASE_CORE_SWTMR == YES)

LITE_OS_SEC_BSS UINT32            m_uwSwTmrHandlerQueue;       /*Software Timer timeout queue ID*/
LITE_OS_SEC_BSS SWTMR_CTRL_S     *m_pstSwtmrCBArray;           /*first address in Timer memory space  */
LITE_OS_SEC_BSS SWTMR_CTRL_S     *m_pstSwtmrFreeList;          /*Free list of Softwaer Timer*/
LITE_OS_SEC_BSS SWTMR_CTRL_S     *m_pstSwtmrSortList;          /*The software timer count list*/

#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
LITE_OS_SEC_BSS UINT32            m_uwSwTmrAlignID[LOSCFG_BASE_CORE_SWTMR_LIMIT] = {0};    /* store swtmr align */
LITE_OS_SEC_DATA_INIT static UINT32                     m_uwSwtimerRousesTime = 0;          // suspend time
LITE_OS_SEC_DATA_INIT static SWTMR_CTRL_S               *m_pstRouses = NULL;              // first swtmr that can wake up
LITE_OS_SEC_DATA_INIT static SWTMR_CTRL_S               *m_pstRousesPrev = NULL;
#endif

#define CHECK_SWTMRID(usSwTmrID, uvIntSave, usSwTmrCBID, pstSwtmr)\
{\
    if (usSwTmrID >= OS_SWTMR_MAX_TIMERID)\
   {\
       return LOS_ERRNO_SWTMR_ID_INVALID;\
   }\
   uvIntSave = LOS_IntLock();\
   usSwTmrCBID = usSwTmrID % LOSCFG_BASE_CORE_SWTMR_LIMIT;\
   pstSwtmr = m_pstSwtmrCBArray + usSwTmrCBID;\
   if (pstSwtmr->usTimerID != usSwTmrID)\
   {\
       LOS_IntRestore(uvIntSave);\
       return LOS_ERRNO_SWTMR_ID_INVALID;\
   }\
}



/*****************************************************************************
Function   : osSwTmrTask
Description: Swtmr task main loop, handle time-out timer.
Input      : None
Output     : None
Return     : None
*****************************************************************************/
LITE_OS_SEC_TEXT VOID osSwTmrTask(VOID)
{
    SWTMR_HANDLER_ITEM_S stSwtmrHandle;
    UINT32 uwReadSzie;
    UINT32 uwRet;
    UINT64 ullTick;
    uwReadSzie = sizeof(SWTMR_HANDLER_ITEM_S);
    for ( ; ; )
    {
        uwRet = LOS_QueueReadCopy(m_uwSwTmrHandlerQueue, &stSwtmrHandle, &uwReadSzie, LOS_WAIT_FOREVER);
        if (uwRet == LOS_OK && uwReadSzie == sizeof(SWTMR_HANDLER_ITEM_S))
        {
            if (stSwtmrHandle.pfnHandler != NULL)
            {
                ullTick = LOS_TickCountGet();
                stSwtmrHandle.pfnHandler(stSwtmrHandle.uwArg);
                ullTick = LOS_TickCountGet()- ullTick;

                if (ullTick >= 2)
                {
                    PRINT_WARN("timer_handler(%p) cost too many ms(%d)\n", stSwtmrHandle.pfnHandler, (UINT32)(ullTick * 1000 /LOSCFG_BASE_CORE_TICK_PER_SECOND));
                }

            }
        }
    }//end of for
}

/*****************************************************************************
Function   : osSwTmrTaskCreate
Description: Create Software Timer
Input      : None
Output     : None
Return     : LOS_OK on success or error code on failure
*****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 osSwTmrTaskCreate(VOID)
{
    UINT32 uwRet;
    TSK_INIT_PARAM_S stSwTmrTask;

    (VOID)memset(&stSwTmrTask, 0, sizeof(TSK_INIT_PARAM_S));
    stSwTmrTask.pfnTaskEntry    = (TSK_ENTRY_FUNC)osSwTmrTask;
    stSwTmrTask.uwStackSize     = 0x1000;
    stSwTmrTask.pcName          = "Swt_Task";
    stSwTmrTask.usTaskPrio      = 0;
    uwRet = LOS_TaskCreate(&g_uwSwtmrTaskID, &stSwTmrTask);
    return uwRet;
}

/*****************************************************************************
Function   : osSwTmrInit
Description: Initializes Software Timer
Input      : None
Output     : None
Return     : LOS_OK on success or error code on failure
*****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 osSwTmrInit(VOID)
{
    UINT32 uwSize;
    UINT16 usIndex;
    UINT32 uwRet;
    SWTMR_CTRL_S *pstSwtmr;
    SWTMR_CTRL_S *pstTemp;

    if (0 == LOSCFG_BASE_CORE_SWTMR_LIMIT)  /*lint !e506*/
    {
        return LOS_ERRNO_SWTMR_MAXSIZE_INVALID;
    }

#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
    (VOID)memset((VOID *)m_uwSwTmrAlignID, 0, LOSCFG_BASE_CORE_SWTMR_LIMIT * sizeof(UINT32));
 #endif

    m_pstSwtmrSortList = (SWTMR_CTRL_S *)NULL;
    uwSize = sizeof(SWTMR_CTRL_S) * LOSCFG_BASE_CORE_SWTMR_LIMIT;
    pstSwtmr = (SWTMR_CTRL_S *)LOS_MemAlloc(m_aucSysMem0, uwSize);
    if (NULL == pstSwtmr)
    {
        return LOS_ERRNO_SWTMR_NO_MEMORY;
    }

    (VOID)memset((VOID *)pstSwtmr, 0, uwSize);
    m_pstSwtmrCBArray = pstSwtmr;
    m_pstSwtmrFreeList = pstSwtmr;
    pstSwtmr->usTimerID = 0;
    pstTemp = pstSwtmr;
    pstSwtmr++;
    for (usIndex = 1; usIndex < LOSCFG_BASE_CORE_SWTMR_LIMIT; usIndex++, pstSwtmr++)
    {
        pstSwtmr->usTimerID = usIndex;
        pstTemp->pstNext = pstSwtmr;
        pstTemp = pstSwtmr;
    }

    uwRet = LOS_QueueCreate((CHAR *)NULL, OS_SWTMR_HANDLE_QUEUE_SIZE, &m_uwSwTmrHandlerQueue, 0, sizeof(SWTMR_HANDLER_ITEM_S));
    if (uwRet != LOS_OK)
    {
        return LOS_ERRNO_SWTMR_QUEUE_CREATE_FAILED;
    }

    uwRet = osSwTmrTaskCreate();
    if (LOS_OK != uwRet)
    {
        return LOS_ERRNO_SWTMR_TASK_CREATE_FAILED;
    }

    return LOS_OK;
}

/*****************************************************************************
Function   : osSwTmrStart
Description: Start Software Timer
Input      : pstSwtmr ---------- Need to start Software Timer
Output     : None
Return     : None
*****************************************************************************/
LITE_OS_SEC_TEXT VOID osSwTmrStart(SWTMR_CTRL_S *pstSwtmr)
{
    SWTMR_CTRL_S *pstPrev = (SWTMR_CTRL_S *)NULL;
    SWTMR_CTRL_S *pstCur = (SWTMR_CTRL_S *)NULL;

#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
    UINT32 uwCurrSwtmrTimes, uwSwtmrTimes;
    UINT32 uwMinInLarge = 0xFFFFFFFF, uwMaxInLitte = 0xFFFFFFFF;
    UINT32 uwMinInLargeID = LOSCFG_BASE_CORE_SWTMR_LIMIT;
    UINT32 uwMaxInLitteID = LOSCFG_BASE_CORE_SWTMR_LIMIT;
    UINT16 usSwTmrCBID;
    UINT16 usSwtmrIdIndex;
    UINT32 uwCount = 0;
#endif

    pstSwtmr->uwCount = pstSwtmr->uwInterval;

#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
    usSwTmrCBID = pstSwtmr->usTimerID % LOSCFG_BASE_CORE_SWTMR_LIMIT;
    if(CHECK_ALIGN_SWTMR_CAN_MULTI_ALIGN(m_uwSwTmrAlignID[usSwTmrCBID]))
    {
        SET_ALIGN_SWTMR_ALREADY_ALIGNED(m_uwSwTmrAlignID[usSwTmrCBID]);
        uwCurrSwtmrTimes = GET_ALIGN_SWTMR_DIVISOR_TIMERS(m_uwSwTmrAlignID[usSwTmrCBID]);
        for(usSwtmrIdIndex = 0; usSwtmrIdIndex < LOSCFG_BASE_CORE_SWTMR_LIMIT; usSwtmrIdIndex++)
        {
            uwSwtmrTimes = GET_ALIGN_SWTMR_DIVISOR_TIMERS(m_uwSwTmrAlignID[usSwtmrIdIndex]);
            if(uwSwtmrTimes == 0 //swtmr not creat
                || usSwtmrIdIndex == usSwTmrCBID //swtmr is pstSwtmr
                || !CHECK_ALIGN_SWTMR_ALREADY_ALIGN(m_uwSwTmrAlignID[usSwtmrIdIndex])) //swtmr not start
            {
                continue;
            }
            if(uwSwtmrTimes >= uwCurrSwtmrTimes && uwSwtmrTimes % uwCurrSwtmrTimes == 0)
            {
                if(uwMinInLarge > uwSwtmrTimes / uwCurrSwtmrTimes)
                {
                    uwMinInLarge = uwSwtmrTimes / uwCurrSwtmrTimes;
                    uwMinInLargeID = usSwtmrIdIndex;
                }
            }
            else if(uwSwtmrTimes < uwCurrSwtmrTimes && uwCurrSwtmrTimes % uwSwtmrTimes == 0)
            {
                if(uwMaxInLitte > uwCurrSwtmrTimes / uwSwtmrTimes)
                {
                    uwMaxInLitte = uwCurrSwtmrTimes / uwSwtmrTimes;
                    uwMaxInLitteID = usSwtmrIdIndex;
                }
            }
        }
        if(uwMinInLargeID != LOSCFG_BASE_CORE_SWTMR_LIMIT)
        {
            pstCur = m_pstSwtmrSortList;
            while (pstCur != NULL)
            {
                uwCount += pstCur->uwCount;
                if (pstCur->usTimerID == ((SWTMR_CTRL_S  *)(m_pstSwtmrCBArray + uwMinInLargeID))->usTimerID)
                {
                    break;
                }
                pstCur = pstCur->pstNext;
            }
            if(pstCur != NULL)
            {
                pstSwtmr->uwCount = pstSwtmr->uwInterval - (pstCur->uwInterval - uwCount)% pstSwtmr->uwInterval;
            }
        }
        else if(uwMaxInLitteID != LOSCFG_BASE_CORE_SWTMR_LIMIT)
        {
            pstSwtmr->uwCount = 0;
            pstPrev = m_pstSwtmrCBArray + uwMaxInLitteID;
            pstCur = pstPrev->pstNext;
            goto Inset_list;
        }
    }
    else if(CHECK_ALIGN_SWTMR_CAN_PERIODIC_ALIGN(m_uwSwTmrAlignID[usSwTmrCBID]))
    {
        SET_ALIGN_SWTMR_ALREADY_ALIGNED(m_uwSwTmrAlignID[usSwTmrCBID]);
        pstCur = m_pstSwtmrSortList;
        while (pstCur != NULL)
        {
            if (pstCur->uwInterval ==  pstSwtmr->uwInterval
                && CHECK_ALIGN_SWTMR_ALREADY_ALIGN(m_uwSwTmrAlignID[pstCur->usTimerID % LOSCFG_BASE_CORE_SWTMR_LIMIT]))
            {
                break;
            }
            pstCur = pstCur->pstNext;
        }
        if(pstCur != NULL)
        {
            pstSwtmr->uwCount = 0;
            pstPrev = pstCur;
            pstCur = pstCur->pstNext;
            goto Inset_list;
        }
    }
#endif

    pstCur = m_pstSwtmrSortList;
    while (pstCur != NULL)
    {
        if (pstCur->uwCount > pstSwtmr->uwCount)
        {
            break;
        }

        pstSwtmr->uwCount -= pstCur->uwCount;
        pstPrev = pstCur;
        pstCur = pstCur->pstNext;
    }

#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
Inset_list:
#endif

    pstSwtmr->pstNext = pstCur;

    if (pstCur != NULL)
    {
        pstCur->uwCount -= pstSwtmr->uwCount;
    }

    if (pstPrev == NULL)
    {
        m_pstSwtmrSortList = pstSwtmr;
    }
    else
    {
        pstPrev->pstNext = pstSwtmr;
    }

    pstSwtmr->ucState = OS_SWTMR_STATUS_TICKING;

    return;
}

/*****************************************************************************
Function   : osSwTmrDelete
Description: Delete Software Timer
Input      : pstSwtmr --- Need to delete Software Timer, When using, Ensure that it can't be NULL.
Output     : None
Return     : None
*****************************************************************************/
LITE_OS_SEC_TEXT STATIC_INLINE VOID osSwtmrDelete(SWTMR_CTRL_S *pstSwtmr)
{
    /**insert to free list **/
    pstSwtmr->pstNext = m_pstSwtmrFreeList;
    m_pstSwtmrFreeList = pstSwtmr;
    pstSwtmr->ucState = OS_SWTMR_STATUS_UNUSED;

#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
    m_uwSwTmrAlignID[pstSwtmr->usTimerID % LOSCFG_BASE_CORE_SWTMR_LIMIT] = 0;
#endif
}

/*****************************************************************************
Function   : osSwtmrStop
Description: Stop of Software Timer interface
Input      : pstSwtmr
Output     : None
Return     : None
*****************************************************************************/
LITE_OS_SEC_TEXT VOID osSwtmrStop(SWTMR_CTRL_S *pstSwtmr)
{
    SWTMR_CTRL_S *pstPrev = (SWTMR_CTRL_S *)NULL;
    SWTMR_CTRL_S *pstCur = (SWTMR_CTRL_S *)NULL;

    if(!m_pstSwtmrSortList)
        return;

    pstCur = m_pstSwtmrSortList;

    while (pstCur != pstSwtmr)
    {
        pstPrev = pstCur;
        pstCur = pstCur->pstNext;
    }

    if (pstCur->pstNext != NULL)
    {
        pstCur->pstNext->uwCount += pstCur->uwCount;
    }

    if (pstPrev == NULL)
    {
        m_pstSwtmrSortList = pstCur->pstNext;
    }
    else
    {
        pstPrev->pstNext = pstCur->pstNext;
    }

    pstCur->pstNext = (SWTMR_CTRL_S *)NULL;
    pstCur->ucState = OS_SWTMR_STATUS_CREATED;

#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
    SET_ALIGN_SWTMR_ALREADY_NOT_ALIGNED(m_uwSwTmrAlignID[pstSwtmr->usTimerID % LOSCFG_BASE_CORE_SWTMR_LIMIT]);
#endif
}

/*****************************************************************************
Function   : osSwTmrTimeoutHandle
Description: Software Timer time out handler
Input      : None
Output     : None
Return     : None
*****************************************************************************/
LITE_OS_SEC_TEXT static VOID osSwTmrTimeoutHandle(VOID)
{
    SWTMR_CTRL_S *pstSwtmr = m_pstSwtmrSortList;
    SWTMR_HANDLER_ITEM_S stSwtmrHandler;

    while (pstSwtmr != NULL && pstSwtmr->uwCount == 0)
    {
        m_pstSwtmrSortList = pstSwtmr->pstNext;
        stSwtmrHandler.pfnHandler = pstSwtmr->pfnHandler;
        stSwtmrHandler.uwArg = pstSwtmr->uwArg;
        (VOID)LOS_QueueWriteCopy(m_uwSwTmrHandlerQueue, &stSwtmrHandler, sizeof(SWTMR_HANDLER_ITEM_S), LOS_NO_WAIT);
        if (pstSwtmr->ucMode == LOS_SWTMR_MODE_ONCE)
        {
            osSwtmrDelete(pstSwtmr);
            if (pstSwtmr->usTimerID < OS_SWTMR_MAX_TIMERID - LOSCFG_BASE_CORE_SWTMR_LIMIT)
                pstSwtmr->usTimerID += LOSCFG_BASE_CORE_SWTMR_LIMIT;
            else
                pstSwtmr->usTimerID %= LOSCFG_BASE_CORE_SWTMR_LIMIT;
        }
        else if ( pstSwtmr->ucMode == LOS_SWTMR_MODE_PERIOD)
        {
            osSwTmrStart(pstSwtmr);
        }
        else if (pstSwtmr->ucMode == LOS_SWTMR_MODE_NO_SELFDELETE)
        {
            pstSwtmr->ucState = OS_SWTMR_STATUS_CREATED;
        }

        pstSwtmr = m_pstSwtmrSortList;
    }
}

/*****************************************************************************
Function   : osSwtmrScan
Description: Tick interrupt interface module of Software Timer
Input      : None
Output     : None
Return     : LOS_OK on success or error code on failure
*****************************************************************************/
LITE_OS_SEC_TEXT UINT32 osSwtmrScan(VOID)
{
    if (m_pstSwtmrSortList != NULL)
    {
        if (--(m_pstSwtmrSortList->uwCount) == 0)
        {
            osSwTmrTimeoutHandle();
        }
    }
    return LOS_OK;
}

/*****************************************************************************
Function   : osSwTmrGetNextTimeout
Description: Get next timeout
Input      : None
Output     : None
Return     : Count of the Timer list
*****************************************************************************/
#if (LOSCFG_KERNEL_TICKLESS == YES)
#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
LITE_OS_SEC_TEXT UINT32 osSwTmrGetNextTimeout(VOID)
{
    SWTMR_CTRL_S *pstCur = NULL;
    UINT32 uwTmpTime = 0, uwSleepTime = 0;

    pstCur = m_pstSwtmrSortList;

    //find first timer can wakeup the system
    while (pstCur != NULL)
    {
        if(OS_SWTMR_ROUSES_ALLOW == pstCur->ucRouses)
        {
            m_pstRouses = pstCur;
            break;
        }

        uwTmpTime += pstCur->uwCount;
        m_pstRousesPrev = pstCur;
        pstCur = pstCur->pstNext;
    }

    if(pstCur != NULL)
    {
        uwSleepTime = pstCur->uwCount + uwTmpTime;
        m_uwSwtimerRousesTime  = uwSleepTime;
    }
    else
    {
        return	0xFFFFFFFF;
    }

    return uwSleepTime;
}
#else
LITE_OS_SEC_TEXT UINT32 osSwTmrGetNextTimeout(VOID)
{
    if (m_pstSwtmrSortList == NULL)
    {
        return 0xFFFFFFFF;
    }
    return m_pstSwtmrSortList->uwCount;
}
#endif
#endif

/*****************************************************************************
Function   : osSwtimerInsert
Description: Insert a list of swtmr
Input      : None
Output     : None
Return     : None
*****************************************************************************/
#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
LITE_OS_SEC_TEXT VOID osSwtimerInsert(SWTMR_CTRL_S **pstHead, SWTMR_CTRL_S *pstSwtmr)
{
    SWTMR_CTRL_S *pstPrev = NULL;
    SWTMR_CTRL_S *pstNextTmp = pstSwtmr->pstNext;
    SWTMR_CTRL_S *pstCur = *pstHead;

    while(pstSwtmr!=NULL)
    {
        while (pstCur != NULL)
        {
            if (pstCur->uwCount > pstSwtmr->uwCount)
            {
                break;
            }

            pstSwtmr->uwCount -= pstCur->uwCount;
            pstPrev = pstCur;
            pstCur = pstCur->pstNext;
        }
        pstSwtmr->pstNext = pstCur;

        if (pstCur != NULL)
        {
            pstCur->uwCount -= pstSwtmr->uwCount;
        }
        if (pstPrev == NULL)
        {
            *pstHead = pstSwtmr;
        }
        else
        {
            pstPrev->pstNext = pstSwtmr;
        }

        pstPrev = pstSwtmr;
        pstSwtmr = pstNextTmp;
        pstNextTmp = pstNextTmp->pstNext;
    }

    return;
}
#endif

/*****************************************************************************
Function   : osSwTmrAdjust
Description: Adjust Software Timer list
Input      : sleep_time
Output     : None
Return     : None
*****************************************************************************/
#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
LITE_OS_SEC_TEXT VOID osSwTmrAdjust(UINT32 uwSleepTime)
{
    SWTMR_CTRL_S  *pstCur = NULL;

    if(m_pstRouses == NULL)
        return;

    if (uwSleepTime > m_uwSwtimerRousesTime)
    {
        uwSleepTime = m_uwSwtimerRousesTime;
    }

    if(uwSleepTime <= m_pstRouses->uwCount)
    {
        m_pstRouses->uwCount -= uwSleepTime;
    }
    else
    {
        m_pstRouses->uwCount = m_uwSwtimerRousesTime - uwSleepTime;

        if(m_pstRousesPrev!=NULL)
        {
            m_pstRousesPrev->pstNext = NULL;
            pstCur = m_pstSwtmrSortList;
            osSwtimerInsert(&m_pstRouses,pstCur);
            m_pstSwtmrSortList = m_pstRouses;
        }
    }
    if (m_pstSwtmrSortList->uwCount == 0)
    {
        osSwTmrTimeoutHandle();
    }

    m_pstRouses = NULL;
    m_pstRousesPrev = NULL;
}
#else
LITE_OS_SEC_TEXT VOID osSwTmrAdjust(UINT32 uwSleepTime)
{
    if (m_pstSwtmrSortList == NULL)
    {
        return ;
    }

    if (uwSleepTime > m_pstSwtmrSortList->uwCount)
    {
        uwSleepTime = m_pstSwtmrSortList->uwCount;
    }

    m_pstSwtmrSortList->uwCount -= uwSleepTime;

    if (m_pstSwtmrSortList->uwCount == 0)
    {
        osSwTmrTimeoutHandle();
    }
}
#endif

/*****************************************************************************
Function   : osSwtmrTimeGet
Description:Obtain a software timer ticks.
Input       : pstSwtmr
Output     : None
Return     : None
Other      : None
*****************************************************************************/
LITE_OS_SEC_TEXT UINT32 osSwtmrTimeGet(SWTMR_CTRL_S *pstSwtmr)
{
    SWTMR_CTRL_S *pstCur = (SWTMR_CTRL_S *)NULL;
    UINT32 uwTick = 0;

    pstCur = m_pstSwtmrSortList;
    while (1)
    {
        uwTick += pstCur->uwCount;
        if (pstCur == pstSwtmr)
        {
            break;
        }

        pstCur = pstCur->pstNext;
    }

    return uwTick;
}

/*****************************************************************************
Function   : LOS_SwtmrCreate
Description: Create software timer
Input      : uwInterval
             usMode
             pfnHandler
             uwArg
Output     : pusSwTmrID
Return     : LOS_OK on success or error code on failure
*****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_SwtmrCreate(UINT32 uwInterval, UINT8 ucMode, SWTMR_PROC_FUNC pfnHandler, UINT16 *pusSwTmrID, UINT32 uwArg
#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
                    , UINT8 ucRouses, UINT8 ucSensitive
#endif
                    )
{
    SWTMR_CTRL_S  *pstSwtmr;
    UINTPTR  uvIntSave;

    if (0 == uwInterval)
    {
        return LOS_ERRNO_SWTMR_INTERVAL_NOT_SUITED;
    }

    if ((LOS_SWTMR_MODE_ONCE != ucMode)
        && (LOS_SWTMR_MODE_PERIOD != ucMode)
        && (LOS_SWTMR_MODE_NO_SELFDELETE != ucMode))
    {
        return LOS_ERRNO_SWTMR_MODE_INVALID;
    }

    if (NULL == pfnHandler)
    {
        return LOS_ERRNO_SWTMR_PTR_NULL;
    }

    if (NULL == pusSwTmrID)
    {
        return LOS_ERRNO_SWTMR_RET_PTR_NULL;
    }

#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
    if((OS_SWTMR_ROUSES_IGNORE != ucRouses)&& (OS_SWTMR_ROUSES_ALLOW != ucRouses))
    {
        return OS_ERRNO_SWTMR_ROUSES_INVALID;
    }

    if((OS_SWTMR_ALIGN_INSENSITIVE != ucSensitive)&& (OS_SWTMR_ALIGN_SENSITIVE != ucSensitive))
    {
        return OS_ERRNO_SWTMR_ALIGN_INVALID;
    }
#endif

    uvIntSave = LOS_IntLock();
    if (NULL == m_pstSwtmrFreeList)
    {
        LOS_IntRestore(uvIntSave);
        return LOS_ERRNO_SWTMR_MAXSIZE;
    }

    pstSwtmr = m_pstSwtmrFreeList;
    m_pstSwtmrFreeList = pstSwtmr->pstNext;
    LOS_IntRestore(uvIntSave);
    pstSwtmr->pfnHandler    = pfnHandler;
    pstSwtmr->ucMode        = ucMode;
    pstSwtmr->uwInterval    = uwInterval;
    pstSwtmr->pstNext       = (SWTMR_CTRL_S *)NULL;
    pstSwtmr->uwCount       = 0;
    pstSwtmr->uwArg         = uwArg;
#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
    pstSwtmr->ucRouses      = ucRouses;
    pstSwtmr->ucSensitive   = ucSensitive;
#endif
    pstSwtmr->ucState       = OS_SWTMR_STATUS_CREATED;
    *pusSwTmrID = pstSwtmr->usTimerID;

    return LOS_OK;
}

/*****************************************************************************
Function   : LOS_SwtmrStart
Description: Start software timer
Input      : usSwTmrID ------- Software timer ID
Output     : None
Return     : LOS_OK on success or error code on failure
*****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_SwtmrStart(UINT16 usSwTmrID)
{
    SWTMR_CTRL_S  *pstSwtmr;
    UINTPTR  uvIntSave;
#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
    UINT32 uwTimes;
#endif
    UINT32 uwRet = LOS_OK;
    UINT16 usSwTmrCBID;

    CHECK_SWTMRID(usSwTmrID, uvIntSave, usSwTmrCBID, pstSwtmr);
#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
    if( OS_SWTMR_ALIGN_INSENSITIVE == pstSwtmr->ucSensitive && LOS_SWTMR_MODE_PERIOD == pstSwtmr->ucMode )
    {
        SET_ALIGN_SWTMR_CAN_ALIGNED(m_uwSwTmrAlignID[pstSwtmr->usTimerID % LOSCFG_BASE_CORE_SWTMR_LIMIT]);
        if(pstSwtmr->uwInterval % LOS_COMMON_DIVISOR == 0)
        {
            SET_ALIGN_SWTMR_CAN_MULTIPLE(m_uwSwTmrAlignID[pstSwtmr->usTimerID % LOSCFG_BASE_CORE_SWTMR_LIMIT]);
            uwTimes = pstSwtmr->uwInterval / (LOS_COMMON_DIVISOR);
            SET_ALIGN_SWTMR_DIVISOR_TIMERS(m_uwSwTmrAlignID[pstSwtmr->usTimerID % LOSCFG_BASE_CORE_SWTMR_LIMIT], uwTimes);
        }
    }
 #endif

    switch (pstSwtmr->ucState)
    {
    case OS_SWTMR_STATUS_UNUSED:
        uwRet = LOS_ERRNO_SWTMR_NOT_CREATED;
        break;
    case OS_SWTMR_STATUS_TICKING:
        osSwtmrStop(pstSwtmr);
    case OS_SWTMR_STATUS_CREATED: /*lint !e616*/
        osSwTmrStart(pstSwtmr);
        break;
    default:
        uwRet = LOS_ERRNO_SWTMR_STATUS_INVALID;
        break;
    }

    LOS_IntRestore(uvIntSave);
    return uwRet;
}

/*****************************************************************************
Function   : LOS_SwtmrStop
Description: Stop software timer
Input      : usSwTmrID ------- Software timer ID
Output     : None
Return     : LOS_OK on success or error code on failure
*****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_SwtmrStop(UINT16 usSwTmrID)
{
    SWTMR_CTRL_S *pstSwtmr;
    UINTPTR uvIntSave;
    UINT16 usSwTmrCBID;
    UINT32 uwRet = LOS_OK;

    CHECK_SWTMRID(usSwTmrID, uvIntSave, usSwTmrCBID, pstSwtmr);
    switch (pstSwtmr->ucState)
    {
    case OS_SWTMR_STATUS_UNUSED:
        uwRet = LOS_ERRNO_SWTMR_NOT_CREATED;
        break;
    case OS_SWTMR_STATUS_CREATED:
        uwRet = LOS_ERRNO_SWTMR_NOT_STARTED;
        break;
    case OS_SWTMR_STATUS_TICKING:
        osSwtmrStop(pstSwtmr);
        break;
    default:
        uwRet = LOS_ERRNO_SWTMR_STATUS_INVALID;
        break;
    }

    LOS_IntRestore(uvIntSave);
    return uwRet;
}

LITE_OS_SEC_TEXT UINT32 LOS_SwtmrTimeGet(UINT16 usSwTmrID, UINT32 *uwTick)
{
    SWTMR_CTRL_S  *pstSwtmr;
    UINTPTR  uvIntSave;
    UINT32 uwRet = LOS_OK;
    UINT16 usSwTmrCBID;

    if (usSwTmrID >= OS_SWTMR_MAX_TIMERID)
    {
        return LOS_ERRNO_SWTMR_ID_INVALID;
    }

    if (uwTick == NULL)
    {
        return LOS_ERRNO_SWTMR_TICK_PTR_NULL;
    }

    uvIntSave = LOS_IntLock();
    usSwTmrCBID = usSwTmrID % LOSCFG_BASE_CORE_SWTMR_LIMIT;
    pstSwtmr = m_pstSwtmrCBArray + usSwTmrCBID;

    if (pstSwtmr->usTimerID != usSwTmrID)
    {
        LOS_IntRestore(uvIntSave);
        return LOS_ERRNO_SWTMR_ID_INVALID;
    }
    switch (pstSwtmr->ucState)
    {
    case OS_SWTMR_STATUS_UNUSED:
        uwRet = LOS_ERRNO_SWTMR_NOT_CREATED;
        break;
    case OS_SWTMR_STATUS_CREATED:
        uwRet = LOS_ERRNO_SWTMR_NOT_STARTED;
        break;
    case OS_SWTMR_STATUS_TICKING:
        *uwTick = osSwtmrTimeGet(pstSwtmr);
        break;
    default:
        uwRet = LOS_ERRNO_SWTMR_STATUS_INVALID;
        break;
    }
    LOS_IntRestore(uvIntSave);
    return uwRet;
}

/*****************************************************************************
Function   : LOS_SwtmrDelete
Description: Delete software timer
Input      : usSwTmrID ------- Software timer ID
Output     : None
Return     : LOS_OK on success or error code on failure
*****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_SwtmrDelete(UINT16 usSwTmrID)
{
    SWTMR_CTRL_S  *pstSwtmr;
    UINTPTR  uvIntSave;
    UINT32 uwRet = LOS_OK;
    UINT16 usSwTmrCBID;

    CHECK_SWTMRID(usSwTmrID, uvIntSave, usSwTmrCBID, pstSwtmr);
    switch (pstSwtmr->ucState)
    {
    case OS_SWTMR_STATUS_UNUSED:
        uwRet = LOS_ERRNO_SWTMR_NOT_CREATED;
        break;
    case OS_SWTMR_STATUS_TICKING:
        osSwtmrStop(pstSwtmr);
    case OS_SWTMR_STATUS_CREATED:  /*lint !e616*/
        osSwtmrDelete(pstSwtmr);
        break;
    default:
        uwRet = LOS_ERRNO_SWTMR_STATUS_INVALID;
        break;
    }

    LOS_IntRestore(uvIntSave);
    return uwRet;
}

#endif /*(LOSCFG_BASE_CORE_SWTMR == YES)*/


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif
