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

#include "los_queue.inc"
#include "los_membox.ph"
#include "los_memory.ph"
#include "los_priqueue.ph"
#include "los_task.ph"
#include "los_hwi.h"
#include "los_hw.h"
#ifdef LOSCFG_LIB_LIBC
#include "string.h"
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

#if (LOSCFG_BASE_IPC_QUEUE == YES)

/*lint -save -e64*/
LITE_OS_SEC_BSS      QUEUE_CB_S       *g_pstAllQueue;

/**************************************************************************
 Function    : osQueueInit
 Description : queue initial
 Input       : usMaxQueue  --- Maximum queue count
 Output      : None
 Return      : LOS_OK on success or error code on failure
**************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 osQueueInit()
{

    if (0 == LOSCFG_BASE_IPC_QUEUE_LIMIT)   /*lint !e506*/
    {
        return LOS_ERRNO_QUEUE_MAXNUM_ZERO;
    }

    g_pstAllQueue = (QUEUE_CB_S *)LOS_MemAlloc(m_aucSysMem0, LOSCFG_BASE_IPC_QUEUE_LIMIT * sizeof(QUEUE_CB_S));
    if (NULL == g_pstAllQueue)
    {
        return LOS_ERRNO_QUEUE_NO_MEMORY;
    }

    memset(g_pstAllQueue, 0, LOSCFG_BASE_IPC_QUEUE_LIMIT * sizeof(QUEUE_CB_S));

    return LOS_OK;
}

/**************************************************************************
 Function    : osQueueCreate
 Description :  Create a queue
 Input       : usLen           --- Queue length
               puwQueueID      --- Queue ID
               usMaxMsgSize    --- Maximum message size in byte
 Output      : ppstQueueCBOut
 Return      : LOS_OK on success or error code on failure
**************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 osQueueCreate(UINT16 usLen,
                                      UINT32 *puwQueueID,
                                      UINT16 usMaxMsgSize,
                                      QUEUE_CB_S **ppstQueueCBOut)
{
    UINT32      uwIndex;
    QUEUE_CB_S    *pstQueueCB;

    /* get a unused queue */
    pstQueueCB = g_pstAllQueue;
    for (uwIndex = 0; uwIndex < LOSCFG_BASE_IPC_QUEUE_LIMIT; uwIndex++, pstQueueCB++)
    {
        if (OS_QUEUE_UNUSED == pstQueueCB->usQueueState)
        {
            *puwQueueID = uwIndex + 1;
            break;
        }
    }

    if (uwIndex == LOSCFG_BASE_IPC_QUEUE_LIMIT)
    {
        return LOS_ERRNO_QUEUE_CB_UNAVAILABLE;
    }

    (VOID)memset(pstQueueCB, 0, sizeof(QUEUE_CB_S));
    pstQueueCB->pucQueue = (UINT8 *)LOS_MemAlloc(m_aucSysMem0, usLen * usMaxMsgSize);
    if (NULL == pstQueueCB->pucQueue)
    {
        return LOS_ERRNO_QUEUE_CREATE_NO_MEMORY;
    }
    pstQueueCB->usQueueLen = usLen;
    pstQueueCB->usQueueSize = usMaxMsgSize;
    pstQueueCB->usQueueState = OS_QUEUE_INUSED;
    *ppstQueueCBOut = pstQueueCB;

    return LOS_OK;
}

/**************************************************************************
 Function    : osQueuePend
 Description : pend a task
 Input       : pstRunTsk
               pstPendList
               uwTimeOut
 Output      : pstRunTsk
 Return      : none
**************************************************************************/
LITE_OS_SEC_TEXT static VOID osQueuePend(LOS_TASK_CB *pstRunTsk, LOS_DL_LIST *pstPendList, UINT32  uwTimeOut)
{
    LOS_DL_LIST *pstPendObj = (LOS_DL_LIST *)NULL;
    LOS_TASK_CB *pstTskCB = (LOS_TASK_CB *)NULL;

    LOS_PriqueueDequeue(&pstRunTsk->stPendList);
    pstRunTsk->usTaskStatus &= (~OS_TASK_STATUS_READY);
    pstPendObj = &(pstRunTsk->stPendList);
    pstRunTsk->usTaskStatus |= OS_TASK_STATUS_PEND_QUEUE;
    if (LOS_ListEmpty(pstPendList))
    {
        LOS_ListTailInsert(pstPendList, pstPendObj);
    }
    else
    {
        LOS_DL_LIST_FOR_EACH_ENTRY(pstTskCB, pstPendList, LOS_TASK_CB, stPendList) /*lint !e413*/
        {
            if (pstRunTsk->usPriority < pstTskCB->usPriority)
            {
                break;
            }
        }
        LOS_ListAdd(pstTskCB->stPendList.pstPrev, pstPendObj);
    }

    if (uwTimeOut != LOS_WAIT_FOREVER)
    {
        pstRunTsk->usTaskStatus |= OS_TASK_STATUS_TIMEOUT;
        osTaskAdd2TimerList(pstRunTsk, uwTimeOut);
    }

    return;
}

/**************************************************************************
 Function    : osQueueWakeUp
 Description : wake up the first task in the pending queue
 Input       : pstPendList
 Output      : pstPendList
 Return      : none
**************************************************************************/
LITE_OS_SEC_TEXT static VOID osQueueWakeUp(LOS_DL_LIST *pstPendList)
{
    LOS_TASK_CB    *pstResumedTask;

    pstResumedTask = OS_TCB_FROM_PENDLIST(LOS_DL_LIST_FIRST(pstPendList)); /*lint !e413*/
    LOS_ListDelete(LOS_DL_LIST_FIRST(pstPendList));
    pstResumedTask->usTaskStatus &= (~OS_TASK_STATUS_PEND_QUEUE);
    if (pstResumedTask->usTaskStatus & OS_TASK_STATUS_TIMEOUT)
    {
        osTimerListDelete(pstResumedTask);
        pstResumedTask->usTaskStatus &= (~OS_TASK_STATUS_TIMEOUT);
    }

    if (!(pstResumedTask->usTaskStatus & OS_TASK_STATUS_SUSPEND))
    {
        pstResumedTask->usTaskStatus |= OS_TASK_STATUS_READY;
        LOS_PriqueueEnqueue(&pstResumedTask->stPendList, pstResumedTask->usPriority);
    }

    return;
}

/*****************************************************************************
 Function    : LOS_QueueCreate
 Description : Create a queue
 Input       : pcQueueName  --- Queue name, less than 4 characters
               usLen        --- Queue lenth
               uwFlags      --- Queue type, FIFO or PRIO
               usMaxMsgSize --- Maximum message size in byte
 Output      : puwQueueID   --- Queue ID
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_QueueCreate(CHAR *pcQueueName,
                                          UINT16 usLen,
                                          UINT32 *puwQueueID,
                                          UINT32 uwFlags,
                                          UINT16 usMaxMsgSize )
{
    QUEUE_CB_S    *pstQueueCB;
    UINTPTR     uvIntSave;
    UINT32      uwRet;

    (VOID)pcQueueName;
    (VOID)uwFlags;

    if (NULL == puwQueueID)
    {
        return LOS_ERRNO_QUEUE_CREAT_PTR_NULL;
    }

    if(usMaxMsgSize > OS_NULL_SHORT -4)
    {
        return LOS_ERRNO_QUEUE_SIZE_TOO_BIG;
    }

    if ((0 == usLen) || (0 == usMaxMsgSize))
    {
        return LOS_ERRNO_QUEUE_PARA_ISZERO;
    }

    uvIntSave = LOS_IntLock();
    uwRet = osQueueCreate(usLen, puwQueueID, (usMaxMsgSize + sizeof(UINT32)), &pstQueueCB);
    if(LOS_OK != uwRet)
    {
        LOS_IntRestore(uvIntSave);
        return uwRet;
    }
    LOS_ListInit(&pstQueueCB->stWriteList);
    LOS_ListInit(&pstQueueCB->stReadList);
    LOS_ListInit(&pstQueueCB->stMemList);
    pstQueueCB->usWritableCnt = usLen;

    LOS_IntRestore(uvIntSave);

    return LOS_OK;
}

/*****************************************************************************
 Function    : LOS_QueueRead
 Description : read queue
 Input       : uwQueueID
               uwBufferSize
               uwTimeOut
 Output      : pBufferAddr
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_QueueRead(UINT32  uwQueueID,
                    VOID *  pBufferAddr,
                    UINT32  uwBufferSize,
                    UINT32  uwTimeOut)
{
    QUEUE_CB_S    *pstQueueCB;
    UINT8       *pucQueueNode;
    LOS_TASK_CB  *pstRunTsk;
    UINTPTR     uvIntSave;
    UINT32      uwRet = LOS_OK;
    UINT32 uwInnerID = uwQueueID - 1;

    if ( uwInnerID >= LOSCFG_BASE_IPC_QUEUE_LIMIT)
    {
        return LOS_ERRNO_QUEUE_READ_INVALID;
    }

    if (NULL == pBufferAddr)
    {
        return LOS_ERRNO_QUEUE_READ_PTR_NULL;
    }

    if (0 == uwBufferSize)
    {
        return LOS_ERRNO_QUEUE_READSIZE_ISZERO;
    }

    if (LOS_NO_WAIT != uwTimeOut)
    {
        if (OS_INT_ACTIVE)
        {
            return LOS_ERRNO_QUEUE_READ_IN_INTERRUPT;
        }
    }

    uvIntSave = LOS_IntLock();
    pstQueueCB = (QUEUE_CB_S *)GET_QUEUE_HANDLE(uwInnerID);

    if (OS_QUEUE_UNUSED == pstQueueCB->usQueueState)
    {
        GOTO_QUEUE_END(LOS_ERRNO_QUEUE_READ_NOT_CREATE);
    }

    if (0 == pstQueueCB->usReadableCnt)
    {
        if (LOS_NO_WAIT == uwTimeOut)
        {
            GOTO_QUEUE_END(LOS_ERRNO_QUEUE_ISEMPTY);
        }

        if (g_usLosTaskLock)
        {
            GOTO_QUEUE_END(LOS_ERRNO_QUEUE_PEND_IN_LOCK);
        }

        pstRunTsk = (LOS_TASK_CB *)g_stLosTask.pstRunTask;
        osQueuePend(pstRunTsk, &pstQueueCB->stReadList, uwTimeOut);
        LOS_IntRestore(uvIntSave);
        LOS_Schedule();

        uvIntSave = LOS_IntLock();

        if (pstRunTsk->usTaskStatus & OS_TASK_STATUS_TIMEOUT)
        {
            pstRunTsk->usTaskStatus &= (~OS_TASK_STATUS_TIMEOUT);
            GOTO_QUEUE_END(LOS_ERRNO_QUEUE_TIMEOUT);
        }
    }
    else
    {
        pstQueueCB->usReadableCnt--;
    }

    pucQueueNode = &(pstQueueCB->pucQueue[((pstQueueCB->usQueueHead) * (pstQueueCB->usQueueSize))]);
   *(UINT32*)pBufferAddr = *(UINT32*)(pucQueueNode);
    if (++pstQueueCB->usQueueHead == pstQueueCB->usQueueLen)
    {
        pstQueueCB->usQueueHead = 0;
    }

    if (!LOS_ListEmpty(&pstQueueCB->stWriteList))
    {
        osQueueWakeUp(&pstQueueCB->stWriteList);
        LOS_IntRestore(uvIntSave);
        LOS_Schedule();

        return LOS_OK;
    }
    else
    {
        pstQueueCB->usWritableCnt++;
    }

QUEUE_END:
    LOS_IntRestore(uvIntSave);
    return uwRet;
}

/*****************************************************************************
 Function    : LOS_QueueWrite
 Description : Write queue
 Input       : uwQueueID
               pBufferAddr
               uwBufferSize
               uwTimeOut
 Output      : None
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_QueueWrite( UINT32 uwQueueID,
                                     VOID * pBufferAddr,
                                     UINT32 uwBufferSize,
                                     UINT32 uwTimeOut )
{
    QUEUE_CB_S *pstQueueCB;
    UINT8    *pucQueueNode;
    LOS_TASK_CB *pstRunTsk;
    UINTPTR  uvIntSave;
    UINT32  uwRet = LOS_OK;
    UINT32 uwInnerID = uwQueueID - 1;

    if(uwInnerID >= LOSCFG_BASE_IPC_QUEUE_LIMIT)
    {
        return LOS_ERRNO_QUEUE_WRITE_INVALID;
    }

    if (NULL == pBufferAddr)
    {
        return LOS_ERRNO_QUEUE_WRITE_PTR_NULL;
    }

    if(0 == uwBufferSize)
    {
        return LOS_ERRNO_QUEUE_WRITESIZE_ISZERO;
    }

    if (LOS_NO_WAIT != uwTimeOut)
    {
        if (OS_INT_ACTIVE)
        {
            return LOS_ERRNO_QUEUE_WRITE_IN_INTERRUPT;
        }
    }

    uvIntSave = LOS_IntLock();

    pstQueueCB = (QUEUE_CB_S *)GET_QUEUE_HANDLE(uwInnerID);

    if (OS_QUEUE_UNUSED == pstQueueCB->usQueueState)
    {
        GOTO_QUEUE_END(LOS_ERRNO_QUEUE_WRITE_NOT_CREATE);
    }

    if (uwBufferSize > pstQueueCB->usQueueSize)
    {
        GOTO_QUEUE_END(LOS_ERRNO_QUEUE_WRITE_SIZE_TOO_BIG);
    }

    if (0 == pstQueueCB->usWritableCnt)
    {
        if (LOS_NO_WAIT == uwTimeOut)
        {
            GOTO_QUEUE_END(LOS_ERRNO_QUEUE_ISFULL);
        }

        if (g_usLosTaskLock)
        {
            GOTO_QUEUE_END(LOS_ERRNO_QUEUE_PEND_IN_LOCK);
        }

        pstRunTsk = (LOS_TASK_CB *)g_stLosTask.pstRunTask;
        osQueuePend(pstRunTsk, &pstQueueCB->stWriteList, uwTimeOut);
        LOS_IntRestore(uvIntSave);
        LOS_Schedule();

        uvIntSave = LOS_IntLock();
        if (pstRunTsk->usTaskStatus & OS_TASK_STATUS_TIMEOUT)
        {
            pstRunTsk->usTaskStatus &= (~OS_TASK_STATUS_TIMEOUT);
            GOTO_QUEUE_END(LOS_ERRNO_QUEUE_TIMEOUT);
        }
    }
    else
    {
         pstQueueCB->usWritableCnt--;
    }

    pucQueueNode = &(pstQueueCB->pucQueue[((pstQueueCB->usQueueTail) * (pstQueueCB->usQueueSize))]);
    *((UINT32 *)pucQueueNode) = (UINT32)pBufferAddr;

    if(++pstQueueCB->usQueueTail == pstQueueCB->usQueueLen)
    {
        pstQueueCB->usQueueTail = 0;
    }

    if (!LOS_ListEmpty(&pstQueueCB->stReadList))
    {
        osQueueWakeUp(&pstQueueCB->stReadList);
        LOS_IntRestore(uvIntSave);
        LOS_Schedule();
        return LOS_OK;
    }
    else
    {
        pstQueueCB->usReadableCnt++;
    }
QUEUE_END:
    LOS_IntRestore(uvIntSave);
    return uwRet;
}

/*****************************************************************************
 Function    : osQueueMailAlloc
 Description : Mail allocate memory
 Input       : uwQueueID   --- QueueID
             : pMailPool   --- MailPool
             : uwTimeOut   --- TimeOut
 Output      :
 Return      : pointer if success otherwise NULL
 *****************************************************************************/
LITE_OS_SEC_TEXT VOID *osQueueMailAlloc(UINT32  uwQueueID, VOID* pMailPool, UINT32 uwTimeOut)
{
    VOID *pMem = (VOID *)NULL;
    UINTPTR uvIntSave;
    QUEUE_CB_S *pstQueueCB = (QUEUE_CB_S *)NULL;
    LOS_TASK_CB *pstRunTsk = (LOS_TASK_CB *)NULL;
    UINT32 uwInnerID = uwQueueID - 1;

    if (uwInnerID >= LOSCFG_BASE_IPC_QUEUE_LIMIT)
    {
        return NULL;
    }

    if (pMailPool == NULL)
    {
        return NULL;
    }

    if (LOS_NO_WAIT != uwTimeOut)
    {
        if (OS_INT_ACTIVE)
        {
            return NULL;
        }
    }

    uvIntSave = LOS_IntLock();
    pstQueueCB = GET_QUEUE_HANDLE(uwInnerID);
    pMem = LOS_MemboxAlloc(pMailPool);
    if (NULL == pMem)
    {
        if (uwTimeOut == LOS_NO_WAIT)
        {
            goto END;
        }

        pstRunTsk = (LOS_TASK_CB *)g_stLosTask.pstRunTask;
        osQueuePend(pstRunTsk, &pstQueueCB->stMemList, uwTimeOut);
        LOS_IntRestore(uvIntSave);
        LOS_Schedule();

        uvIntSave = LOS_IntLock();
        if (pstRunTsk->usTaskStatus & OS_TASK_STATUS_TIMEOUT)
        {
            pstRunTsk->usTaskStatus &= (~OS_TASK_STATUS_TIMEOUT);
            goto END;
        }
        else
        {
            if (NULL == pstRunTsk->puwMsg)
            {
                //TODO:fault handle
            }
            pMem = pstRunTsk->puwMsg;
            pstRunTsk->puwMsg = NULL;
        }
    }

END:
    LOS_IntRestore(uvIntSave);
    return pMem;
}

/*****************************************************************************
 Function    : osQueueMailFree
 Description : Mail free memory
 Input       : uwQueueID   --- QueueID
             : pMailPool   --- MailPool
 Output      :
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 osQueueMailFree(UINT32  uwQueueID, VOID* pMailPool, VOID* pMailMem)
{
    VOID *pMem = (VOID *)NULL;
    UINTPTR uvIntSave;
    QUEUE_CB_S *pstQueueCB = (QUEUE_CB_S *)NULL;
    LOS_TASK_CB *pstResumedTask = (LOS_TASK_CB *)NULL;
    UINT32 uwInnerID = uwQueueID - 1;

    if (uwInnerID >= LOSCFG_BASE_IPC_QUEUE_LIMIT)
    {
        return LOS_ERRNO_QUEUE_MAIL_HANDLE_INVALID;
    }

    if (pMailPool == NULL)
    {
        return LOS_ERRNO_QUEUE_MAIL_PTR_INVALID;
    }

    uvIntSave = LOS_IntLock();

    if (LOS_MemboxFree(pMailPool, pMailMem))
    {
        LOS_IntRestore(uvIntSave);
        return LOS_ERRNO_QUEUE_MAIL_FREE_ERROR;
    }

    pstQueueCB = GET_QUEUE_HANDLE(uwInnerID);
    if (!LOS_ListEmpty(&pstQueueCB->stMemList))
    {
        pstResumedTask = OS_TCB_FROM_PENDLIST(LOS_DL_LIST_FIRST(&pstQueueCB->stMemList)); /*lint !e413*/
        osQueueWakeUp(&pstQueueCB->stMemList);
        pMem = LOS_MemboxAlloc(pMailPool);
        if (NULL == pMem)
        {
            //TODO: fault handle
        }

        pstResumedTask->puwMsg = pMem;
        LOS_IntRestore(uvIntSave);
        LOS_Schedule();
    }
    else
    {
        LOS_IntRestore(uvIntSave);
    }
    return LOS_OK;
}

/*****************************************************************************
 Function    : LOS_QueueDelete
 Description : Delete a queue
 Input       : puwQueueID   --- QueueID
 Output      :
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_QueueDelete(UINT32 uwQueueID)
{
    QUEUE_CB_S *pstQueueCB;
    UINTPTR  uvIntSave;
    UINT32 uwRet;
    UINT32 uwInnerID = uwQueueID - 1;

    if (uwInnerID >= LOSCFG_BASE_IPC_QUEUE_LIMIT)
    {
        return LOS_ERRNO_QUEUE_NOT_FOUND;
    }

    uvIntSave = LOS_IntLock();
    pstQueueCB = (QUEUE_CB_S *)GET_QUEUE_HANDLE(uwInnerID);
    if (OS_QUEUE_UNUSED == pstQueueCB->usQueueState)
    {
        GOTO_QUEUE_END(LOS_ERRNO_QUEUE_NOT_CREATE);
    }

    if (!LOS_ListEmpty(&pstQueueCB->stReadList))
    {
        GOTO_QUEUE_END(LOS_ERRNO_QUEUE_IN_TSKUSE);
    }

    if (!LOS_ListEmpty(&pstQueueCB->stWriteList))
    {
        GOTO_QUEUE_END(LOS_ERRNO_QUEUE_IN_TSKUSE);
    }

    if (!LOS_ListEmpty(&pstQueueCB->stMemList))
    {
        GOTO_QUEUE_END(LOS_ERRNO_QUEUE_IN_TSKUSE);
    }

    if ((pstQueueCB->usWritableCnt + pstQueueCB->usReadableCnt) != pstQueueCB->usQueueLen)
    {
        GOTO_QUEUE_END(LOS_ERRNO_QUEUE_IN_TSKWRITE);
    }

    uwRet = LOS_MemFree(m_aucSysMem0, (VOID *)(pstQueueCB->pucQueue));
    if (LOS_OK != uwRet)
    {
        GOTO_QUEUE_END(uwRet);
    }

    pstQueueCB->usQueueState = OS_QUEUE_UNUSED;

QUEUE_END:
    LOS_IntRestore(uvIntSave);
    return uwRet;
}

#endif /*(LOSCFG_BASE_IPC_QUEUE == YES)*/

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */
