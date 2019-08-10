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
#include "los_queue.ph"
#include "los_membox.ph"
#include "los_memory.ph"
#include "los_priqueue.ph"
#include "los_task.ph"
#if (LOSCFG_PLATFORM_EXC == YES)
#include "los_exc.ph"
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

#if (LOSCFG_BASE_IPC_QUEUE == YES)

/*lint -save -e64*/
LITE_OS_SEC_BSS      QUEUE_CB_S       *g_pstAllQueue;
LITE_OS_SEC_BSS      LOS_DL_LIST      g_stFreeQueueList;
#if ((LOSCFG_PLATFORM_EXC == YES) && (LOSCFG_SAVE_EXC_INFO == YES))
LITE_OS_SEC_BSS      UINT32           g_uwExcQueueMaxNum;
#endif

/**************************************************************************
 Function    : osQueueInit
 Description : queue initial
 Input       : None
 Output      : None
 Return      : LOS_OK on success or error code on failure
**************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 osQueueInit(VOID)
{
    QUEUE_CB_S *pstQueueNode;
    UINT16   usIndex;

    if (0 == LOSCFG_BASE_IPC_QUEUE_LIMIT)   /*lint !e506*/
    {
        return LOS_ERRNO_QUEUE_MAXNUM_ZERO;
    }

    g_pstAllQueue = (QUEUE_CB_S *)LOS_MemAlloc(m_aucSysMem0, LOSCFG_BASE_IPC_QUEUE_LIMIT * sizeof(QUEUE_CB_S));
    if (NULL == g_pstAllQueue)
    {
        return LOS_ERRNO_QUEUE_NO_MEMORY;
    }

    (VOID)memset(g_pstAllQueue, 0, LOSCFG_BASE_IPC_QUEUE_LIMIT * sizeof(QUEUE_CB_S));

    LOS_ListInit(&g_stFreeQueueList);
    for (usIndex = 0; usIndex < LOSCFG_BASE_IPC_QUEUE_LIMIT; usIndex++)
    {
        pstQueueNode = ((QUEUE_CB_S *)g_pstAllQueue) + usIndex;
        pstQueueNode->usQueueID = usIndex;
        LOS_ListTailInsert(&g_stFreeQueueList, &pstQueueNode->stReadWriteList[OS_QUEUE_WRITE]);
    }

#if ((LOSCFG_PLATFORM_EXC == YES) && (LOSCFG_SAVE_EXC_INFO == YES))
    g_uwExcQueueMaxNum = LOSCFG_BASE_IPC_QUEUE_LIMIT;
    osExcRegister(OS_EXC_TYPE_QUE, (EXC_INFO_SAVE_CALLBACK)LOS_QueueInfoGet, &g_uwExcQueueMaxNum);
#endif

    return LOS_OK;
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
    QUEUE_CB_S      *pstQueueCB;
    UINTPTR         uvIntSave;
    LOS_DL_LIST     *pstUnusedQueue;
    UINT8           *pucQueue;
    UINT16          usMsgSize = usMaxMsgSize + sizeof(UINT32);

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

    /* Memory allocation is time-consuming, to shorten the time of disable interrupt,
       move the memory allocation to here. */
    pucQueue = (UINT8 *)LOS_MemAlloc(m_aucSysMem0, usLen * usMsgSize);
    if (NULL == pucQueue)
    {
        return LOS_ERRNO_QUEUE_CREATE_NO_MEMORY;
    }

    uvIntSave = LOS_IntLock();
    if (LOS_ListEmpty(&g_stFreeQueueList))
    {
        LOS_IntRestore(uvIntSave);
        (VOID)LOS_MemFree(m_aucSysMem0, pucQueue);
        return LOS_ERRNO_QUEUE_CB_UNAVAILABLE;
    }

    pstUnusedQueue = LOS_DL_LIST_FIRST(&(g_stFreeQueueList));
    LOS_ListDelete(pstUnusedQueue);
    pstQueueCB = (GET_QUEUE_LIST(pstUnusedQueue));
    pstQueueCB->usQueueLen = usLen;
    pstQueueCB->usQueueSize = usMsgSize;
    pstQueueCB->pucQueue = pucQueue;
    pstQueueCB->usQueueState = OS_QUEUE_INUSED;
    pstQueueCB->usReadWriteableCnt[OS_QUEUE_READ]  = 0;
    pstQueueCB->usReadWriteableCnt[OS_QUEUE_WRITE] = usLen;
    pstQueueCB->usQueueHead = 0;
    pstQueueCB->usQueueTail = 0;
    LOS_ListInit(&pstQueueCB->stReadWriteList[OS_QUEUE_READ]);
    LOS_ListInit(&pstQueueCB->stReadWriteList[OS_QUEUE_WRITE]);
    LOS_ListInit(&pstQueueCB->stMemList);
    LOS_IntRestore(uvIntSave);

    *puwQueueID = pstQueueCB->usQueueID;

    return LOS_OK;
}

LITE_OS_SEC_TEXT static INLINE UINT32 osQueueReadParameterCheck(UINT32 uwQueueID, VOID *pBufferAddr, UINT32 *puwBufferSize, UINT32 uwTimeOut)
{
    if (uwQueueID >= LOSCFG_BASE_IPC_QUEUE_LIMIT)
    {
        return LOS_ERRNO_QUEUE_INVALID;
    }
    if ((NULL == pBufferAddr) || (NULL == puwBufferSize))
    {
        return LOS_ERRNO_QUEUE_READ_PTR_NULL;
    }

    if (0 == *puwBufferSize)
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
    return LOS_OK;
}


LITE_OS_SEC_TEXT static INLINE UINT32 osQueueWriteParameterCheck(UINT32 uwQueueID, VOID *pBufferAddr, UINT32 *puwBufferSize, UINT32 uwTimeOut)
{
    if (uwQueueID >= LOSCFG_BASE_IPC_QUEUE_LIMIT)
    {
        return LOS_ERRNO_QUEUE_INVALID;
    }

    if (NULL == pBufferAddr)
    {
        return LOS_ERRNO_QUEUE_WRITE_PTR_NULL;
    }

    if (0 == *puwBufferSize)
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
    return LOS_OK;
}

LITE_OS_SEC_TEXT static INLINE VOID osQueueBufferOperate(QUEUE_CB_S *pstQueueCB, UINT32 uwOperateType, VOID *pBufferAddr, UINT32 *puwBufferSize)
{
    UINT8        *pucQueueNode;
    UINT32       uwMsgDataSize = 0;
    UINT16      usQueuePosion = 0;

    /* get the queue position */
    switch (OS_QUEUE_OPERATE_GET(uwOperateType))
    {
        case OS_QUEUE_READ_HEAD:
            usQueuePosion = pstQueueCB->usQueueHead;
            (pstQueueCB->usQueueHead + 1 == pstQueueCB->usQueueLen) ? (pstQueueCB->usQueueHead = 0) : (pstQueueCB->usQueueHead++);
            break;

        case OS_QUEUE_WRITE_HEAD:
            (0 == pstQueueCB->usQueueHead) ? (pstQueueCB->usQueueHead = pstQueueCB->usQueueLen - 1) : (--pstQueueCB->usQueueHead);
            usQueuePosion = pstQueueCB->usQueueHead;
            break;

        case OS_QUEUE_WRITE_TAIL :
            usQueuePosion = pstQueueCB->usQueueTail;
            (pstQueueCB->usQueueTail + 1 == pstQueueCB->usQueueLen) ? (pstQueueCB->usQueueTail = 0) : (pstQueueCB->usQueueTail++);
            break;

        default: //read tail , reserved.
            PRINT_ERR("invalid queue operate type!\n");
            return;
    }

    pucQueueNode = &(pstQueueCB->pucQueue[(usQueuePosion * (pstQueueCB->usQueueSize))]);

    if(OS_QUEUE_IS_POINT(uwOperateType))
    {
        if(OS_QUEUE_IS_READ(uwOperateType))
        {
            *(UINT32 *)pBufferAddr = *(UINT32 *)pucQueueNode;
        }
        else
        {
            *(UINT32 *)pucQueueNode = *(UINT32 *)pBufferAddr;//change to pp when calling osQueueOperate
        }
    }
    else
    {
        if(OS_QUEUE_IS_READ(uwOperateType))
        {
            memcpy((VOID *)&uwMsgDataSize, (VOID *)(pucQueueNode + pstQueueCB->usQueueSize - sizeof(UINT32)), sizeof(UINT32));
            memcpy((VOID *)pBufferAddr, (VOID *)pucQueueNode, uwMsgDataSize);
            *puwBufferSize = uwMsgDataSize;
        }
        else
        {
            memcpy((VOID *)pucQueueNode, (VOID *)pBufferAddr, *puwBufferSize);
            memcpy((VOID *)(pucQueueNode + pstQueueCB->usQueueSize - sizeof(UINT32)), puwBufferSize, sizeof(UINT32));
        }
    }
}


LITE_OS_SEC_TEXT UINT32 osQueueOperate(UINT32 uwQueueID, UINT32 uwOperateType, VOID *pBufferAddr, UINT32 *puwBufferSize, UINT32 uwTimeOut)
{
    QUEUE_CB_S *pstQueueCB;
    LOS_TASK_CB  *pstRunTsk;
    UINTPTR      uvIntSave;
    LOS_TASK_CB  *pstResumedTask;
    UINT32       uwRet = LOS_OK;
    UINT32       uwReadWrite = OS_QUEUE_READ_WRITE_GET(uwOperateType);

    uvIntSave = LOS_IntLock();

    pstQueueCB = (QUEUE_CB_S *)GET_QUEUE_HANDLE(uwQueueID);
    if (OS_QUEUE_UNUSED == pstQueueCB->usQueueState)
    {
        uwRet = LOS_ERRNO_QUEUE_NOT_CREATE;
        goto QUEUE_END;

    }

    if(OS_QUEUE_IS_READ(uwOperateType) && (*puwBufferSize < pstQueueCB->usQueueSize - sizeof(UINT32)))
    {
        uwRet = LOS_ERRNO_QUEUE_READ_SIZE_TOO_SMALL;
        goto QUEUE_END;
    }
    else if(OS_QUEUE_IS_WRITE(uwOperateType) && (*puwBufferSize > pstQueueCB->usQueueSize - sizeof(UINT32)))
    {
        uwRet = LOS_ERRNO_QUEUE_WRITE_SIZE_TOO_BIG;
        goto QUEUE_END;
    }

    if (0 == pstQueueCB->usReadWriteableCnt[uwReadWrite])
    {
        if (LOS_NO_WAIT == uwTimeOut)
        {
            uwRet = OS_QUEUE_IS_READ(uwOperateType) ? LOS_ERRNO_QUEUE_ISEMPTY : LOS_ERRNO_QUEUE_ISFULL;
            goto QUEUE_END;
        }

        if (g_usLosTaskLock)
        {
            uwRet = LOS_ERRNO_QUEUE_PEND_IN_LOCK;
            goto QUEUE_END;
        }

        pstRunTsk = (LOS_TASK_CB *)g_stLosTask.pstRunTask;
        osTaskWait(&pstQueueCB->stReadWriteList[uwReadWrite], OS_TASK_STATUS_PEND_QUEUE, uwTimeOut);
        LOS_IntRestore(uvIntSave);
        LOS_Schedule();

        uvIntSave = LOS_IntLock();

        if (pstRunTsk->usTaskStatus & OS_TASK_STATUS_TIMEOUT)
        {
            pstRunTsk->usTaskStatus &= (~OS_TASK_STATUS_TIMEOUT);
            uwRet = LOS_ERRNO_QUEUE_TIMEOUT;
            goto QUEUE_END;
        }
    }
    else
    {
        pstQueueCB->usReadWriteableCnt[uwReadWrite]--;
    }

    osQueueBufferOperate(pstQueueCB, uwOperateType, pBufferAddr, puwBufferSize);

    if (!LOS_ListEmpty(&pstQueueCB->stReadWriteList[!uwReadWrite])) /*lint !e514*/
    {
        pstResumedTask = OS_TCB_FROM_PENDLIST(LOS_DL_LIST_FIRST(&pstQueueCB->stReadWriteList[!uwReadWrite])); /*lint !e413 !e514*/
        osTaskWake(pstResumedTask, OS_TASK_STATUS_PEND_QUEUE);
        LOS_IntRestore(uvIntSave);
        LOS_Schedule();
        return LOS_OK;
    }
    else
    {
        pstQueueCB->usReadWriteableCnt[!uwReadWrite]++; /*lint !e514*/
    }

QUEUE_END:
    LOS_IntRestore(uvIntSave);
    return uwRet;
}

/*****************************************************************************
 Function    : LOS_QueueReadCopy
 Description : Read queue
 Input       : uwQueueID
               puwBufferSize
               uwTimeOut
 Output      : pBufferAddr
               puwBufferSize
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_QueueReadCopy(UINT32  uwQueueID,
                    VOID *  pBufferAddr,
                    UINT32 * puwBufferSize,
                    UINT32  uwTimeOut)
{
    UINT32 uwRet;
    UINT32 uwOperateType;

    uwRet = osQueueReadParameterCheck(uwQueueID, pBufferAddr, puwBufferSize, uwTimeOut);
    if(uwRet != LOS_OK)
    {
        return uwRet;
    }

    uwOperateType = OS_QUEUE_OPERATE_TYPE(OS_QUEUE_READ, OS_QUEUE_HEAD,OS_QUEUE_NOT_POINT);
    return osQueueOperate(uwQueueID, uwOperateType, pBufferAddr, puwBufferSize, uwTimeOut);
}

/*****************************************************************************
 Function    : LOS_QueueWriteHeadCopy
 Description : Write queue head
 Input       : uwQueueID
               pBufferAddr
               uwBufferSize
               uwTimeOut
 Output      : None
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_QueueWriteHeadCopy(UINT32 uwQueueID,
                                     VOID * pBufferAddr,
                                     UINT32 uwBufferSize,
                                     UINT32 uwTimeOut )
{
    UINT32 uwRet;
    UINT32 uwOperateType;

    uwRet = osQueueWriteParameterCheck(uwQueueID, pBufferAddr, &uwBufferSize, uwTimeOut);
    if(uwRet != LOS_OK)
    {
        return uwRet;
    }

    uwOperateType = OS_QUEUE_OPERATE_TYPE(OS_QUEUE_WRITE, OS_QUEUE_HEAD,OS_QUEUE_NOT_POINT);
    return osQueueOperate(uwQueueID, uwOperateType, pBufferAddr, &uwBufferSize, uwTimeOut);
}

/*****************************************************************************
 Function    : LOS_QueueWriteCopy
 Description : Write queue tail
 Input       : uwQueueID
               pBufferAddr
               uwBufferSize
               uwTimeOut
 Output      : None
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_QueueWriteCopy( UINT32 uwQueueID,
                                     VOID * pBufferAddr,
                                     UINT32 uwBufferSize,
                                     UINT32 uwTimeOut )
{
    UINT32 uwRet;
    UINT32 uwOperateType;

    uwRet = osQueueWriteParameterCheck(uwQueueID, pBufferAddr, &uwBufferSize, uwTimeOut);
    if(uwRet != LOS_OK)
    {
        return uwRet;
    }

    uwOperateType = OS_QUEUE_OPERATE_TYPE(OS_QUEUE_WRITE, OS_QUEUE_TAIL, OS_QUEUE_NOT_POINT);
    return osQueueOperate(uwQueueID, uwOperateType, pBufferAddr, &uwBufferSize, uwTimeOut);
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
LITE_OS_SEC_TEXT UINT32 LOS_QueueRead(UINT32  uwQueueID, VOID *pBufferAddr, UINT32 uwBufferSize, UINT32 uwTimeOut)
{
    UINT32 uwRet;
    UINT32 uwOperateType;

    uwRet = osQueueReadParameterCheck(uwQueueID, pBufferAddr, &uwBufferSize, uwTimeOut);
    if(uwRet != LOS_OK)
    {
        return uwRet;
    }

    uwOperateType = OS_QUEUE_OPERATE_TYPE(OS_QUEUE_READ, OS_QUEUE_HEAD, OS_QUEUE_POINT);
    return osQueueOperate(uwQueueID, uwOperateType, pBufferAddr, &uwBufferSize, uwTimeOut);
}

/*****************************************************************************
 Function    : LOS_QueueWrite
 Description : Write queue tail
 Input       : uwQueueID
               pBufferAddr
               uwBufferSize
               uwTimeOut
 Output      : None
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_QueueWrite(UINT32 uwQueueID, VOID *pBufferAddr, UINT32 uwBufferSize, UINT32 uwTimeOut)
{
    UINT32 uwRet;
    UINT32 uwOperateType;

    uwBufferSize = sizeof(UINT32*);

    uwRet = osQueueWriteParameterCheck(uwQueueID, pBufferAddr, &uwBufferSize, uwTimeOut);
    if(uwRet != LOS_OK)
    {
        return uwRet;
    }

    uwOperateType = OS_QUEUE_OPERATE_TYPE(OS_QUEUE_WRITE, OS_QUEUE_TAIL, OS_QUEUE_POINT);
    return osQueueOperate(uwQueueID, uwOperateType, &pBufferAddr, &uwBufferSize, uwTimeOut);
}

/*****************************************************************************
 Function    : LOS_QueueWriteHead
 Description : write queue head
 Input       : uwQueueID
               pBufferAddr
               uwBufferSize
               uwTimeOut
 Output      : None
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_QueueWriteHead( UINT32 uwQueueID,
                                     VOID * pBufferAddr,
                                     UINT32 uwBufferSize,
                                     UINT32 uwTimeOut )
{
    if(pBufferAddr == NULL)
    {
        return LOS_ERRNO_QUEUE_WRITE_PTR_NULL;
    }
    uwBufferSize = sizeof(UINT32*);
    return LOS_QueueWriteHeadCopy(uwQueueID, &pBufferAddr, uwBufferSize, uwTimeOut);
}

/*****************************************************************************
 Function    : osQueueMailAlloc
 Description : Mail allocate memory
 Input       : uwQueueID   --- QueueID
             : pMailPool   --- MailPool
             : uwTimeOut   --- TimeOut
 Output      : None
 Return      : pointer if success otherwise NULL
 *****************************************************************************/
LITE_OS_SEC_TEXT VOID *osQueueMailAlloc(UINT32  uwQueueID, VOID* pMailPool, UINT32 uwTimeOut)
{
    VOID *pMem = (VOID *)NULL;
    UINTPTR uvIntSave;
    QUEUE_CB_S *pstQueueCB = (QUEUE_CB_S *)NULL;
    LOS_TASK_CB *pstRunTsk = (LOS_TASK_CB *)NULL;

    if (uwQueueID >= LOSCFG_BASE_IPC_QUEUE_LIMIT)
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
    pstQueueCB = GET_QUEUE_HANDLE(uwQueueID);
    if (OS_QUEUE_UNUSED == pstQueueCB->usQueueState)
    {
        goto END;
    }

    pMem = LOS_MemboxAlloc(pMailPool);
    if (NULL == pMem)
    {
        if (uwTimeOut == LOS_NO_WAIT)
        {
            goto END;
        }

        pstRunTsk = (LOS_TASK_CB *)g_stLosTask.pstRunTask;
        osTaskWait(&pstQueueCB->stMemList, OS_TASK_STATUS_PEND_QUEUE, uwTimeOut);
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
            /* When enters the current branch, means the current task already got a available membox,
             * so the pstRunTsk->puwMsg can not be NULL.
             */
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
 Output      : None
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT UINT32 osQueueMailFree(UINT32  uwQueueID, VOID* pMailPool, VOID* pMailMem)
{
    VOID *pMem = (VOID *)NULL;
    UINTPTR uvIntSave;
    QUEUE_CB_S *pstQueueCB = (QUEUE_CB_S *)NULL;
    LOS_TASK_CB *pstResumedTask = (LOS_TASK_CB *)NULL;

    if (uwQueueID >= LOSCFG_BASE_IPC_QUEUE_LIMIT)
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

    pstQueueCB = GET_QUEUE_HANDLE(uwQueueID);
    if (OS_QUEUE_UNUSED == pstQueueCB->usQueueState)
    {
        LOS_IntRestore(uvIntSave);
        return LOS_ERRNO_QUEUE_NOT_CREATE;
    }

    if (!LOS_ListEmpty(&pstQueueCB->stMemList))
    {
        pstResumedTask = OS_TCB_FROM_PENDLIST(LOS_DL_LIST_FIRST(&pstQueueCB->stMemList)); /*lint !e413*/
        osTaskWake(pstResumedTask, OS_TASK_STATUS_PEND_QUEUE);
        pMem = LOS_MemboxAlloc(pMailPool);
        /* At the state of LOS_IntLock, the allocation can not be failed after releasing succefully. */

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
 Output      : None
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_QueueDelete(UINT32 uwQueueID)
{
    QUEUE_CB_S *pstQueueCB;
    UINT8 *pucQueue = NULL;
    UINTPTR  uvIntSave;
    UINT32 uwRet;

    if (uwQueueID >= LOSCFG_BASE_IPC_QUEUE_LIMIT)
    {
        return LOS_ERRNO_QUEUE_NOT_FOUND;
    }

    uvIntSave = LOS_IntLock();
    pstQueueCB = (QUEUE_CB_S *)GET_QUEUE_HANDLE(uwQueueID);
    if (OS_QUEUE_UNUSED == pstQueueCB->usQueueState)
    {
        uwRet = LOS_ERRNO_QUEUE_NOT_CREATE;
        goto QUEUE_END;
    }

    if (!LOS_ListEmpty(&pstQueueCB->stReadWriteList[OS_QUEUE_READ]))
    {
        uwRet = LOS_ERRNO_QUEUE_IN_TSKUSE;
        goto QUEUE_END;
    }

    if (!LOS_ListEmpty(&pstQueueCB->stReadWriteList[OS_QUEUE_WRITE]))
    {
        uwRet = LOS_ERRNO_QUEUE_IN_TSKUSE;
        goto QUEUE_END;
    }

    if (!LOS_ListEmpty(&pstQueueCB->stMemList))
    {
        uwRet = LOS_ERRNO_QUEUE_IN_TSKUSE;
        goto QUEUE_END;
    }

    if ((pstQueueCB->usReadWriteableCnt[OS_QUEUE_WRITE] + pstQueueCB->usReadWriteableCnt[OS_QUEUE_READ]) != pstQueueCB->usQueueLen)
    {
        uwRet = LOS_ERRNO_QUEUE_IN_TSKWRITE;
        goto QUEUE_END;
    }

    pucQueue = pstQueueCB->pucQueue;
    pstQueueCB->pucQueue = (UINT8 *)NULL;
    pstQueueCB->usQueueState = OS_QUEUE_UNUSED;
    LOS_ListAdd(&g_stFreeQueueList, &pstQueueCB->stReadWriteList[OS_QUEUE_WRITE]);
    LOS_IntRestore(uvIntSave);

    uwRet = LOS_MemFree(m_aucSysMem0, (VOID *)pucQueue);
    return uwRet;

QUEUE_END:
    LOS_IntRestore(uvIntSave);
    return uwRet;
}

/*****************************************************************************
 Function    : LOS_QueueInfoGet
 Description : Get queue infomation
 Input       : puwQueueID   --- QueueID
 Output      : pstQueueInfo
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_QueueInfoGet(UINT32 uwQueueID, QUEUE_INFO_S *pstQueueInfo)
{
    UINTPTR uvIntSave;
    UINT32 uwRet = LOS_OK;
    QUEUE_CB_S *pstQueueCB;
    LOS_TASK_CB *pstTskCB;

    if (NULL == pstQueueInfo)
    {
        return LOS_ERRNO_QUEUE_PTR_NULL;
    }

    if (uwQueueID >= LOSCFG_BASE_IPC_QUEUE_LIMIT)
    {
        return LOS_ERRNO_QUEUE_INVALID;
    }

    (VOID)memset((VOID *)pstQueueInfo, 0, sizeof(QUEUE_INFO_S));
    uvIntSave = LOS_IntLock();

    pstQueueCB = (QUEUE_CB_S *)GET_QUEUE_HANDLE(uwQueueID);

    if (OS_QUEUE_UNUSED == pstQueueCB->usQueueState)
    {
        uwRet = LOS_ERRNO_QUEUE_NOT_CREATE;
        goto QUEUE_END;
    }

    pstQueueInfo->uwQueueID = uwQueueID;
    pstQueueInfo->usQueueLen = pstQueueCB->usQueueLen;
    pstQueueInfo->usQueueSize = pstQueueCB->usQueueSize;
    pstQueueInfo->usQueueHead = pstQueueCB->usQueueHead;
    pstQueueInfo->usQueueTail = pstQueueCB->usQueueTail;
    pstQueueInfo->usReadableCnt = pstQueueCB->usReadWriteableCnt[OS_QUEUE_READ];
    pstQueueInfo->usWritableCnt = pstQueueCB->usReadWriteableCnt[OS_QUEUE_WRITE];

    LOS_DL_LIST_FOR_EACH_ENTRY(pstTskCB, &pstQueueCB->stReadWriteList[OS_QUEUE_READ], LOS_TASK_CB, stPendList) /*lint !e413*/
    {
        pstQueueInfo->uwWaitReadTask |= (1 << pstTskCB->uwTaskID);
    }

    LOS_DL_LIST_FOR_EACH_ENTRY(pstTskCB, &pstQueueCB->stReadWriteList[OS_QUEUE_WRITE], LOS_TASK_CB, stPendList) /*lint !e413*/
    {
        pstQueueInfo->uwWaitWriteTask |= (1 << pstTskCB->uwTaskID);
    }

    LOS_DL_LIST_FOR_EACH_ENTRY(pstTskCB, &pstQueueCB->stMemList, LOS_TASK_CB, stPendList) /*lint !e413*/
    {
        pstQueueInfo->uwWaitMemTask |= (1 << pstTskCB->uwTaskID);
    }

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
