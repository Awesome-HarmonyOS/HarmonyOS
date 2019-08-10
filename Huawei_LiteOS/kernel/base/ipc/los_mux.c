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

#include "los_mux.inc"
#include "los_err.ph"
#include "los_memory.ph"
#include "los_priqueue.ph"
#include "los_task.ph"
#if (LOSCFG_PLATFORM_EXC == YES)
#include "los_exc.h"
#endif
#include "los_hw.h"

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */


#if (LOSCFG_BASE_IPC_MUX == YES)

LITE_OS_SEC_BSS MUX_CB_S             *g_pstAllMux;
LITE_OS_SEC_BSS LOS_DL_LIST          g_stUnusedMuxList;


/*****************************************************************************
 Funtion	     : osMuxInit,
 Description  : Initializes the mutex,
 Input        : None
 Output       : None
 Return       : LOS_OK on success ,or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 osMuxInit(VOID)
{
    MUX_CB_S *pstMuxNode;
    UINT32   uwIndex;

    LOS_ListInit(&g_stUnusedMuxList);

    if (LOSCFG_BASE_IPC_MUX_LIMIT == 0)   /*lint !e506*/
    {
        return LOS_ERRNO_MUX_MAXNUM_ZERO;
    }

    g_pstAllMux = (MUX_CB_S *)LOS_MemAlloc(m_aucSysMem0, (LOSCFG_BASE_IPC_MUX_LIMIT * sizeof(MUX_CB_S)));
    if (NULL == g_pstAllMux)
    {
        return LOS_ERRNO_MUX_NO_MEMORY;
    }

    for (uwIndex = 0; uwIndex < LOSCFG_BASE_IPC_MUX_LIMIT; uwIndex++)
    {
        pstMuxNode              = ((MUX_CB_S *)g_pstAllMux) + uwIndex;
        pstMuxNode->ucMuxID     = uwIndex;
        pstMuxNode->ucMuxStat   = OS_MUX_UNUSED;
        LOS_ListTailInsert(&g_stUnusedMuxList, &pstMuxNode->stMuxList);
    }
    return LOS_OK;
}

/*****************************************************************************
 Function     : LOS_MuxCreate
 Description  : Create a mutex,
 Input        : None
 Output       : puwMuxHandle ------ Mutex operation handle
 Return       : LOS_OK on success ,or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT  UINT32  LOS_MuxCreate (UINT32 *puwMuxHandle)
{
    UINT32      uwIntSave;
    MUX_CB_S    *pstMuxCreated;
    LOS_DL_LIST *pstUnusedMux;
    UINT32      uwErrNo;
    UINT32      uwErrLine;

    if (NULL == puwMuxHandle)
    {
        return LOS_ERRNO_MUX_PTR_NULL;
    }

    uwIntSave = LOS_IntLock();
    if (LOS_ListEmpty(&g_stUnusedMuxList))
    {
        LOS_IntRestore(uwIntSave);
        OS_GOTO_ERR_HANDLER(LOS_ERRNO_MUX_ALL_BUSY);
    }

    pstUnusedMux                = LOS_DL_LIST_FIRST(&(g_stUnusedMuxList));
    LOS_ListDelete(pstUnusedMux);
    pstMuxCreated               = (GET_MUX_LIST(pstUnusedMux)); /*lint !e413*/
    pstMuxCreated->usMuxCount   = 0;
    pstMuxCreated->ucMuxStat    = OS_MUX_USED;
    pstMuxCreated->usPriority   = 0;
    pstMuxCreated->pstOwner     = (LOS_TASK_CB *)NULL;
    LOS_ListInit(&pstMuxCreated->stMuxList);
    *puwMuxHandle               = (UINT32)pstMuxCreated->ucMuxID;
    LOS_IntRestore(uwIntSave);
    return LOS_OK;
ErrHandler:
    OS_RETURN_ERROR_P2(uwErrLine, uwErrNo);
}

/*****************************************************************************
 Function     : LOS_MuxDelete
 Description  : Delete a mutex,
 Input        : uwMuxHandle------Mutex operation handle
 Output       : None
 Return       : LOS_OK on success ,or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_MuxDelete(UINT32 uwMuxHandle)
{
    UINT32    uwIntSave;
    MUX_CB_S *pstMuxDeleted;
    UINT32   uwErrNo;
    UINT32   uwErrLine;

    if (uwMuxHandle >= (UINT32)LOSCFG_BASE_IPC_MUX_LIMIT)
    {
        OS_GOTO_ERR_HANDLER(LOS_ERRNO_MUX_INVALID);
    }

    pstMuxDeleted = GET_MUX(uwMuxHandle);
    uwIntSave = LOS_IntLock();
    if (OS_MUX_UNUSED == pstMuxDeleted->ucMuxStat)
    {
        LOS_IntRestore(uwIntSave);
        OS_GOTO_ERR_HANDLER(LOS_ERRNO_MUX_INVALID);
    }

    if (!LOS_ListEmpty(&pstMuxDeleted->stMuxList) || pstMuxDeleted->usMuxCount)
    {
        LOS_IntRestore(uwIntSave);
        OS_GOTO_ERR_HANDLER(LOS_ERRNO_MUX_PENDED);
    }

    LOS_ListAdd(&g_stUnusedMuxList, &pstMuxDeleted->stMuxList);
    pstMuxDeleted->ucMuxStat = OS_MUX_UNUSED;

    LOS_IntRestore(uwIntSave);

    return LOS_OK;
ErrHandler:
    OS_RETURN_ERROR_P2(uwErrLine, uwErrNo);
}

/*****************************************************************************
 Function     : LOS_MuxPend
 Description  : Specify the mutex P operation,
 Input        : uwMuxHandle ------ Mutex operation handleone,
 		       uwTimeOut  ------- waiting time,
 Output       : None
 Return       : LOS_OK on success ,or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_MuxPend(UINT32 uwMuxHandle, UINT32 uwTimeout)
{
    UINT32     uwIntSave;
    MUX_CB_S  *pstMuxPended;
    UINT32     uwRetErr;
    LOS_TASK_CB  *pstRunTsk;

    if (uwMuxHandle >= (UINT32)LOSCFG_BASE_IPC_MUX_LIMIT)
    {
        OS_RETURN_ERROR(LOS_ERRNO_MUX_INVALID);
    }

    pstMuxPended = GET_MUX(uwMuxHandle);
    uwIntSave = LOS_IntLock();
    if (OS_MUX_UNUSED == pstMuxPended->ucMuxStat)
    {
        LOS_IntRestore(uwIntSave);
        OS_RETURN_ERROR(LOS_ERRNO_MUX_INVALID);
    }

    if (OS_INT_ACTIVE)
    {
        LOS_IntRestore(uwIntSave);
        return LOS_ERRNO_MUX_PEND_INTERR;
    }

    pstRunTsk = (LOS_TASK_CB *)g_stLosTask.pstRunTask;
    if (pstMuxPended->usMuxCount == 0)
    {
        pstMuxPended->usMuxCount++;
        pstMuxPended->pstOwner = pstRunTsk;
        pstMuxPended->usPriority = pstRunTsk->usPriority;
        LOS_IntRestore(uwIntSave);
        return LOS_OK;
    }

    if (pstMuxPended->pstOwner == pstRunTsk)
    {
        pstMuxPended->usMuxCount++;
        LOS_IntRestore(uwIntSave);
        return LOS_OK;
    }

    if (!uwTimeout)
    {
        LOS_IntRestore(uwIntSave);
        return LOS_ERRNO_MUX_UNAVAILABLE;
    }

    if (g_usLosTaskLock)
    {
        uwRetErr = LOS_ERRNO_MUX_PEND_IN_LOCK;
        PRINT_ERR("!!!LOS_ERRNO_MUX_PEND_IN_LOCK!!!\n");
#if (LOSCFG_PLATFORM_EXC == YES)
        osBackTrace();
#endif
        goto errre_uniMuxPend;
    }

    pstRunTsk->pTaskMux = (VOID *)pstMuxPended;

    if (pstMuxPended->pstOwner->usPriority > pstRunTsk->usPriority)
    {
        osTaskPriModify(pstMuxPended->pstOwner, pstRunTsk->usPriority);
    }

    osTaskWait(&pstMuxPended->stMuxList, OS_TASK_STATUS_PEND, uwTimeout);

    (VOID)LOS_IntRestore(uwIntSave);
    LOS_Schedule();

    if (pstRunTsk->usTaskStatus & OS_TASK_STATUS_TIMEOUT)
    {
        uwIntSave = LOS_IntLock();
        pstRunTsk->usTaskStatus &= (~OS_TASK_STATUS_TIMEOUT);
        (VOID)LOS_IntRestore(uwIntSave);
        uwRetErr = LOS_ERRNO_MUX_TIMEOUT;
        goto error_uniMuxPend;
    }

    return LOS_OK;

errre_uniMuxPend:
    (VOID)LOS_IntRestore(uwIntSave);
error_uniMuxPend:
    OS_RETURN_ERROR(uwRetErr);
}

/*****************************************************************************
 Function     : LOS_MuxPost
 Description  : Specify the mutex V operation,
 Input        : uwMuxHandle ------ Mutex operation handle,
 Output       : None
 Return       : LOS_OK on success ,or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_MuxPost(UINT32 uwMuxHandle)
{
    UINT32      uwIntSave;
    MUX_CB_S    *pstMuxPosted = GET_MUX(uwMuxHandle);
    LOS_TASK_CB *pstResumedTask;
    LOS_TASK_CB *pstRunTsk;

    uwIntSave = LOS_IntLock();

    if ((uwMuxHandle >= (UINT32)LOSCFG_BASE_IPC_MUX_LIMIT) ||
        (OS_MUX_UNUSED == pstMuxPosted->ucMuxStat))
    {
        LOS_IntRestore(uwIntSave);
        OS_RETURN_ERROR(LOS_ERRNO_MUX_INVALID);
    }

    pstRunTsk = (LOS_TASK_CB *)g_stLosTask.pstRunTask;
    if ((pstMuxPosted->usMuxCount == 0) || (pstMuxPosted->pstOwner != pstRunTsk))
    {
        LOS_IntRestore(uwIntSave);
        OS_RETURN_ERROR(LOS_ERRNO_MUX_INVALID);
    }

    if (--(pstMuxPosted->usMuxCount) != 0)
    {
        LOS_IntRestore(uwIntSave);
        return LOS_OK;
    }

    if ((pstMuxPosted->pstOwner->usPriority) != pstMuxPosted->usPriority)
    {
        osTaskPriModify(pstMuxPosted->pstOwner, pstMuxPosted->usPriority);
    }

    if (!LOS_ListEmpty(&pstMuxPosted->stMuxList))
    {
        pstResumedTask = OS_TCB_FROM_PENDLIST(LOS_DL_LIST_FIRST(&(pstMuxPosted->stMuxList))); /*lint !e413*/

        pstMuxPosted->usMuxCount    = 1;
        pstMuxPosted->pstOwner      = pstResumedTask;
        pstMuxPosted->usPriority    = pstResumedTask->usPriority;
        pstResumedTask->pTaskMux    = NULL;

        osTaskWake(pstResumedTask, OS_TASK_STATUS_PEND);

        (VOID)LOS_IntRestore(uwIntSave);
        LOS_Schedule();
    }
    else
    {
        (VOID)LOS_IntRestore(uwIntSave);
    }

    return LOS_OK;
}
#endif /*(LOSCFG_BASE_IPC_MUX == YES)*/


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */
