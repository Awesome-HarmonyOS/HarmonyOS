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

#include "los_sem.inc"
#include "los_base.ph"
#include "los_err.ph"
#include "los_memory.ph"
#include "los_priqueue.ph"
#include "los_sys.ph"
#include "los_task.ph"
#if (LOSCFG_PLATFORM_EXC == YES)
#include "los_exc.h"
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */


#if (LOSCFG_BASE_IPC_SEM == YES)

LITE_OS_SEC_BSS LOS_DL_LIST  g_stUnusedSemList;
LITE_OS_SEC_BSS SEM_CB_S    *g_pstAllSem;

/*****************************************************************************
 Function     : osSemInit
 Description  : Initialize the  Semaphore doubly linked list,
 Input        : None,
 Output       : None,
 Return       : LOS_OK on success ,or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 osSemInit(VOID)
{
    SEM_CB_S    *pstSemNode;
    UINT16      uwIndex;

    LOS_ListInit(&g_stUnusedSemList);

    if (LOSCFG_BASE_IPC_SEM_LIMIT == 0)   /*lint !e506*/
    {
        return LOS_ERRNO_SEM_MAXNUM_ZERO;
    }

    g_pstAllSem = (SEM_CB_S *)LOS_MemAlloc(m_aucSysMem0, (LOSCFG_BASE_IPC_SEM_LIMIT * sizeof(SEM_CB_S)));
    if (NULL == g_pstAllSem)
    {
        return LOS_ERRNO_SEM_NO_MEMORY;
    }

    /* Connect all the ECBs in a doubly linked list. */
    for (uwIndex = 0; uwIndex < LOSCFG_BASE_IPC_SEM_LIMIT; uwIndex++)
    {
        pstSemNode = ((SEM_CB_S *)g_pstAllSem) + uwIndex;
        pstSemNode->usSemID = uwIndex;
        pstSemNode->usSemStat = OS_SEM_UNUSED;
        LOS_ListTailInsert(&g_stUnusedSemList, &pstSemNode->stSemList);
    }
    return LOS_OK;
}


/*****************************************************************************
 Function     : osSemCreate
 Description  : create the  Semaphore
 Input        : None,
 Output       : None,
 Return       : LOS_OK on success ,or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 osSemCreate (UINT16 usCount, UINT16 usMaxCount, UINT32 *puwSemHandle)
{
    UINT32      uwIntSave;
    SEM_CB_S    *pstSemCreated;
    LOS_DL_LIST *pstUnusedSem;
    UINT32      uwErrNo;
    UINT32      uwErrLine;

    if (NULL == puwSemHandle)
    {
        return LOS_ERRNO_SEM_PTR_NULL;
    }

    if (usCount > usMaxCount)
    {
        OS_GOTO_ERR_HANDLER(LOS_ERRNO_SEM_OVERFLOW);
    }

    uwIntSave = LOS_IntLock();

    if (LOS_ListEmpty(&g_stUnusedSemList))
    {
        LOS_IntRestore(uwIntSave);
        OS_GOTO_ERR_HANDLER(LOS_ERRNO_SEM_ALL_BUSY);
    }

    pstUnusedSem = LOS_DL_LIST_FIRST(&(g_stUnusedSemList));
    LOS_ListDelete(pstUnusedSem);
    pstSemCreated = (GET_SEM_LIST(pstUnusedSem)); /*lint !e413*/
    pstSemCreated->usSemCount = usCount;
    pstSemCreated->usSemStat = OS_SEM_USED;
    pstSemCreated->usMaxSemCount = usMaxCount;
    LOS_ListInit(&pstSemCreated->stSemList);
    *puwSemHandle = (UINT32)pstSemCreated->usSemID;
    LOS_IntRestore(uwIntSave);
    return LOS_OK;

ErrHandler:
    OS_RETURN_ERROR_P2(uwErrLine, uwErrNo);
}

/*****************************************************************************
 Function     : LOS_SemCreate
 Description  : Create a semaphore,
 Input        : uwCount--------- semaphore count,
 Output       : puwSemHandle-----Index of semaphore,
 Return       : LOS_OK on success ,or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_SemCreate (UINT16 usCount, UINT32 *puwSemHandle)
{
    return osSemCreate(usCount, OS_SEM_COUNTING_MAX_COUNT, puwSemHandle);
}

/*****************************************************************************
 Function     : LOS_BinarySemCreate
 Description  : Create a binary semaphore,
 Input        : uwCount--------- semaphore count,
 Output       : puwSemHandle-----Index of semaphore,
 Return       : LOS_OK on success ,or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_BinarySemCreate (UINT16 usCount, UINT32 *puwSemHandle)
{
    return osSemCreate(usCount, OS_SEM_BINARY_MAX_COUNT, puwSemHandle);
}

/*****************************************************************************
 Function     : LOS_SemDelete
 Description : Delete a semaphore,
 Input          : uwSemHandle--------- semaphore operation handle,
 Output       : None
 Return       : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_SemDelete(UINT32 uwSemHandle)
{
    UINT32      uwIntSave;
    SEM_CB_S    *pstSemDeleted;
    UINT32      uwErrNo;
    UINT32      uwErrLine;

    if (uwSemHandle >= (UINT32)LOSCFG_BASE_IPC_SEM_LIMIT)
    {
        OS_GOTO_ERR_HANDLER(LOS_ERRNO_SEM_INVALID);
    }

    pstSemDeleted = GET_SEM(uwSemHandle);
    uwIntSave = LOS_IntLock();
    if (OS_SEM_UNUSED == pstSemDeleted->usSemStat)
    {
        LOS_IntRestore(uwIntSave);
        OS_GOTO_ERR_HANDLER(LOS_ERRNO_SEM_INVALID);
    }

    if (!LOS_ListEmpty(&pstSemDeleted->stSemList))
    {
        LOS_IntRestore(uwIntSave);
        OS_GOTO_ERR_HANDLER(LOS_ERRNO_SEM_PENDED);
    }

    LOS_ListAdd(&g_stUnusedSemList, &pstSemDeleted->stSemList);
    pstSemDeleted->usSemStat = OS_SEM_UNUSED;
    LOS_IntRestore(uwIntSave);
    return LOS_OK;
ErrHandler:
    OS_RETURN_ERROR_P2(uwErrLine, uwErrNo);
}

/*****************************************************************************
 Function     : LOS_SemPend
 Description : Specified semaphore P operation,
 Input          : uwSemHandle--------- semaphore operation handle,
 		     uwTimeout   ---------- waitting time
 Output       : None
 Return       : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_SemPend(UINT32 uwSemHandle, UINT32 uwTimeout)
{
    UINT32      uwIntSave;
    SEM_CB_S    *pstSemPended;
    UINT32      uwRetErr;
    LOS_TASK_CB *pstRunTsk;

    if (uwSemHandle >= (UINT32)LOSCFG_BASE_IPC_SEM_LIMIT)
    {
        OS_RETURN_ERROR(LOS_ERRNO_SEM_INVALID);
    }

    pstSemPended = GET_SEM(uwSemHandle);
    uwIntSave = LOS_IntLock();
    if (OS_SEM_UNUSED == pstSemPended->usSemStat)
    {
        LOS_IntRestore(uwIntSave);
        OS_RETURN_ERROR(LOS_ERRNO_SEM_INVALID);
    }

    if (pstSemPended->usSemCount > 0)
    {
        pstSemPended->usSemCount--;
        LOS_IntRestore(uwIntSave);
        return LOS_OK;
    }

    if (!uwTimeout)
    {
        uwRetErr = LOS_ERRNO_SEM_UNAVAILABLE;
        goto errre_uniSemPend;
    }

    if (OS_INT_ACTIVE)
    {
        uwRetErr = LOS_ERRNO_SEM_PEND_INTERR;
        PRINT_ERR("!!!LOS_ERRNO_SEM_PEND_INTERR!!!\n");
#if (LOSCFG_PLATFORM_EXC == YES)
        osBackTrace();
#endif
        goto errre_uniSemPend;
    }

    if (g_usLosTaskLock)
    {
        uwRetErr = LOS_ERRNO_SEM_PEND_IN_LOCK;
        PRINT_ERR("!!!LOS_ERRNO_SEM_PEND_IN_LOCK!!!\n");
#if (LOSCFG_PLATFORM_EXC == YES)
        osBackTrace();
#endif
        goto errre_uniSemPend;
    }

    pstRunTsk = (LOS_TASK_CB *)g_stLosTask.pstRunTask;
    pstRunTsk->pTaskSem = (VOID *)pstSemPended;
    osTaskWait(&pstSemPended->stSemList, OS_TASK_STATUS_PEND, uwTimeout);
    (VOID)LOS_IntRestore(uwIntSave);
    LOS_Schedule();

    if (pstRunTsk->usTaskStatus & OS_TASK_STATUS_TIMEOUT)
    {
        uwIntSave = LOS_IntLock();
        pstRunTsk->usTaskStatus &= (~OS_TASK_STATUS_TIMEOUT);
        uwRetErr = LOS_ERRNO_SEM_TIMEOUT;
        goto errre_uniSemPend;
    }

    return LOS_OK;

errre_uniSemPend:
    (VOID)LOS_IntRestore(uwIntSave);
    OS_RETURN_ERROR(uwRetErr);
}

/*****************************************************************************
 Function     : LOS_SemPend
 Description  : Specified semaphore V operation,
 Input        : uwSemHandle--------- semaphore operation handle,
 Output       : None
 Return       : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_SemPost(UINT32 uwSemHandle)
{
    UINT32      uwIntSave;
    SEM_CB_S    *pstSemPosted = GET_SEM(uwSemHandle);
    LOS_TASK_CB *pstResumedTask;

    if (uwSemHandle >= LOSCFG_BASE_IPC_SEM_LIMIT)
    {
        return LOS_ERRNO_SEM_INVALID;
    }

    uwIntSave = LOS_IntLock();

    if (OS_SEM_UNUSED == pstSemPosted->usSemStat)
    {
        LOS_IntRestore(uwIntSave);
        OS_RETURN_ERROR(LOS_ERRNO_SEM_INVALID);
    }

    if (pstSemPosted->usMaxSemCount == pstSemPosted->usSemCount)
    {
        (VOID)LOS_IntRestore(uwIntSave);
        OS_RETURN_ERROR(LOS_ERRNO_SEM_OVERFLOW);
    }
    if (!LOS_ListEmpty(&pstSemPosted->stSemList))
    {
        pstResumedTask = OS_TCB_FROM_PENDLIST(LOS_DL_LIST_FIRST(&(pstSemPosted->stSemList))); /*lint !e413*/
        pstResumedTask->pTaskSem = NULL;
        osTaskWake(pstResumedTask, OS_TASK_STATUS_PEND);

        (VOID)LOS_IntRestore(uwIntSave);
        LOS_Schedule();
    }
    else
    {
        pstSemPosted->usSemCount++;
        (VOID)LOS_IntRestore(uwIntSave);
    }

    return LOS_OK;
}

#endif /*(LOSCFG_BASE_IPC_SEM == YES)*/

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */
