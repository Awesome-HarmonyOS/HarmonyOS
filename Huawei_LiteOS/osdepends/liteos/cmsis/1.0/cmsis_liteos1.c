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
#include "cmsis_os.h"
#include "los_event.h"
#include "los_membox.h"
#include "los_hwi.h"

#include "los_mux.ph"
#include "los_queue.ph"
#include "los_sem.ph"
#include "los_swtmr.ph"
#include "los_sys.ph"
#include "los_task.ph"
#include "los_tick.ph"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */
#if (CMSIS_OS_VER == 1)

#define PRIORITY_WIN 4u

INT32 osKernelRunning(void)
{
    extern BOOL g_bTaskScheduled;
    return (g_bTaskScheduled);
}

/// Start the RTOS Kernel with executing the specified thread
osStatus osKernelStart(void)
{
    (VOID)LOS_Start();
    return osOK;
}

UINT32 osKernelSysTick (void)
{
    //todo: need to use external clock source
    return (UINT32)g_ullTickCount;
}

osStatus osKernelInitialize (void)
{
    INT32 ret;

    ret = LOS_KernelInit();
    if (ret != LOS_OK)
    {
        return osErrorOS;
    }

    return osOK;
}

// Thread Public API

/// Create a thread and add it to Active Threads and set it to state READY
osThreadId osThreadCreate(const osThreadDef_t *thread_def, void *argument)
{
    osThreadId tskcb;
    TSK_INIT_PARAM_S stTskInitParam;
    UINT32 uwTskHandle;
    UINT32 uwRet;
    if ((thread_def == NULL) ||
            (thread_def->pthread == NULL) ||
            (thread_def->tpriority < osPriorityIdle) ||
            (thread_def->tpriority > osPriorityRealtime))
    {
        return (osThreadId)NULL;
    }

    (VOID)memset(&stTskInitParam, 0, sizeof(TSK_INIT_PARAM_S));
    stTskInitParam.pfnTaskEntry = (TSK_ENTRY_FUNC)thread_def->pthread;
    stTskInitParam.uwStackSize  = thread_def->stacksize;
    stTskInitParam.pcName       = thread_def->name;
    stTskInitParam.uwArg        = (UINT32)argument;
    stTskInitParam.usTaskPrio   = (UINT16)(PRIORITY_WIN - thread_def->tpriority);  /*1~7*/

    uwRet = LOS_TaskCreate(&uwTskHandle, &stTskInitParam);

    if(LOS_OK != uwRet )
    {
        return (osThreadId)NULL;
    }

    tskcb = (osThreadId)&g_pstTaskCBArray[uwTskHandle];

    return tskcb;
}

/// Return the thread ID of the current running thread
osThreadId osThreadGetId(void)
{
    return (osThreadId)g_stLosTask.pstRunTask;
}

UINT32 osThreadGetPId(osThreadId thread_id)
{
    return ((LOS_TASK_CB *)thread_id)->uwTaskID;
}
/// Terminate execution of a thread and remove it from ActiveThreads
osStatus osThreadTerminate(osThreadId thread_id)
{
    UINT32  uwRet;

    if (thread_id == NULL)
        return osErrorParameter;

    if (OS_INT_ACTIVE)
        return osErrorISR;

    uwRet = LOS_TaskDelete(((LOS_TASK_CB *)thread_id)->uwTaskID);

    if (uwRet == LOS_OK)
        return osOK;
    else
        return osErrorOS;
}

/// Pass control to next thread that is in state READY
osStatus osThreadYield(void)
{
    UINT32  uwRet;

    if (OS_INT_ACTIVE)
        return osErrorISR;

    uwRet = LOS_TaskYield();

    if (uwRet == LOS_OK)
        return osOK;
    else
        return osErrorOS;
}

/// Change prority of an active thread
osStatus osThreadSetPriority(osThreadId thread_id, osPriority priority)
{
    UINT32  uwRet;
    UINT16    usPriorityTemp;

    if (thread_id == NULL)
        return osErrorParameter;

    if (OS_INT_ACTIVE)
        return osErrorISR;

    if (priority < osPriorityIdle || priority > osPriorityRealtime)
        return osErrorPriority;

    usPriorityTemp = PRIORITY_WIN - priority;

    uwRet = LOS_TaskPriSet(((LOS_TASK_CB *)thread_id)->uwTaskID, usPriorityTemp);

    if (uwRet == LOS_OK)
        return osOK;
    else
        return osErrorOS;
}

/// Get current prority of an active thread
osPriority osThreadGetPriority(osThreadId thread_id)
{
    UINT16 usPriorityTemp;
    INT16 osPriorityRet;

    if (thread_id == NULL)
        return osPriorityError;

    usPriorityTemp = LOS_TaskPriGet(((LOS_TASK_CB *)thread_id)->uwTaskID);

    osPriorityRet = PRIORITY_WIN - usPriorityTemp;

    if (osPriorityRet < osPriorityIdle || osPriorityRet > osPriorityRealtime)
        return osPriorityError;

    return (osPriority)osPriorityRet;
}

osSemaphoreId osBinarySemaphoreCreate(const osSemaphoreDef_t *semaphore_def, INT32 count)
{
#if (LOSCFG_BASE_IPC_SEM == YES)
    UINT32   uwRet;
    UINT32   *SemHandle;
    SEM_CB_S *pstSemCreated;

    if (semaphore_def == NULL)
    {
        return (osSemaphoreId)NULL;
    }

    SemHandle = (UINT32 *)(semaphore_def->puwSemHandle);
    uwRet =  LOS_BinarySemCreate (count,  SemHandle);

    if (uwRet == LOS_OK)
    {
        pstSemCreated = GET_SEM(*SemHandle);
        return (osSemaphoreId)pstSemCreated;
    }
    else
    {
        return (osSemaphoreId)NULL;
    }
#endif
}

osSemaphoreId osSemaphoreCreate(const osSemaphoreDef_t *semaphore_def, INT32 count)
{
#if (LOSCFG_BASE_IPC_SEM == YES)
    UINT32 uwRet;
    UINT32 *SemHandle;

    if (semaphore_def == NULL)
    {
        return (osSemaphoreId)NULL;
    }

    SemHandle = (UINT32 *)(semaphore_def->puwSemHandle);
    uwRet =  LOS_SemCreate (count,  SemHandle);

    if (uwRet == LOS_OK)
    {
        return (osSemaphoreId)GET_SEM(*SemHandle);
    }
    else
    {
        return (osSemaphoreId)NULL;
    }
#endif
}

/// Wait until a Semaphore becomes available
/*
number of available tokens, or -1 in case of incorrect parameters.
*/
INT32 osSemaphoreWait(osSemaphoreId semaphore_id, UINT32 millisec)
{
#if (LOSCFG_BASE_IPC_SEM == YES)
    UINT32 uwRet;
    UINT32 SemID;

    if (semaphore_id == NULL)
    {
        return -1;
    }

    if (OS_INT_ACTIVE)
    {
        return -1;
    }

    SemID = ((SEM_CB_S *)semaphore_id)->usSemID;

    uwRet = LOS_SemPend(SemID, LOS_MS2Tick(millisec));

    if (uwRet == LOS_OK)
    {
        return ((SEM_CB_S *)semaphore_id)->usSemCount;
    }
    else
    {
        return -1;
    }
#endif
}

/// Release a Semaphore
/*
osOK: the semaphore has been released.
osErrorResource: all tokens have already been released.
osErrorParameter: the parameter semaphore_id is incorrect.
*/
osStatus osSemaphoreRelease(osSemaphoreId semaphore_id)
{
#if (LOSCFG_BASE_IPC_SEM == YES)
    UINT32  uwRet;
    UINT32  SemID;

    if (semaphore_id == NULL)
    {
        return osErrorParameter;
    }

    SemID = ((SEM_CB_S *)semaphore_id)->usSemID;
    uwRet = LOS_SemPost(SemID);

    if (uwRet == LOS_OK)
    {
        return osOK;
    }
    else if (uwRet == LOS_ERRNO_SEM_INVALID)
    {
        return osErrorParameter;
    }
    else
    {
        return osErrorResource;
    }
#endif
}

/*
osOK: the semaphore object has been deleted.
osErrorISR: osSemaphoreDelete cannot be called from interrupt service routines.
osErrorResource: the semaphore object could not be deleted.
osErrorParameter: the parameter semaphore_id is incorrect.
*/
osStatus osSemaphoreDelete (osSemaphoreId semaphore_id)
{
#if (LOSCFG_BASE_IPC_SEM == YES)
    UINT32  uwRet;
    UINT32  SemID;

    if (semaphore_id == NULL)
    {
        return osErrorParameter;
    }

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    SemID = ((SEM_CB_S *)semaphore_id)->usSemID;
    uwRet = LOS_SemDelete(SemID);

    if (uwRet == LOS_OK)
    {
        return osOK;
    }
    else if (uwRet == LOS_ERRNO_SEM_INVALID)
    {
        return osErrorParameter;
    }
    else
    {
        return osErrorResource;
    }
#endif
}

//Mutex Public API

/// Create and Initialize a Mutex object.
/// \param[in]     mutex_def     mutex definition referenced with \ref osMutex.
/// \return mutex ID for reference by other functions or NULL in case of error.
/// \note MUST REMAIN UNCHANGED: \b osMutexCreate shall be consistent in every CMSIS-RTOS.
osMutexId osMutexCreate (const osMutexDef_t *mutex_def)
{
#if (LOSCFG_BASE_IPC_MUX == YES)
    UINT32  uwRet;
    UINT32 *MuxHandle;

    if(mutex_def == NULL)
    {
        return (osMutexId)NULL;
    }

    MuxHandle = (UINT32 *)(mutex_def->puwMuxHandle);
    uwRet =  LOS_MuxCreate (MuxHandle);

    if(uwRet == LOS_OK)
    {
        return (osMutexId)GET_MUX(*MuxHandle);
    }
    else
    {
        return (osMutexId)NULL;
    }
#endif
}

/// Wait until a Mutex becomes available.
/// \param[in]     mutex_id      mutex ID obtained by \ref osMutexCreate.
/// \param[in]     millisec      timeout value or 0 in case of no time-out.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osMutexWait shall be consistent in every CMSIS-RTOS.
/*
osOK: the mutex has been obtain.
osErrorTimeoutResource: the mutex could not be obtained in the given time.
osErrorResource: the mutex could not be obtained when no timeout was specified.
osErrorParameter: the parameter mutex_id is incorrect.
osErrorISR: osMutexWait cannot be called from interrupt service routines.
*/
osStatus osMutexWait (osMutexId mutex_id, UINT32 millisec)
{
#if (LOSCFG_BASE_IPC_MUX == YES)
    UINT32  uwRet;
    UINT32  MutID;

    if (mutex_id == NULL)
    {
        return osErrorParameter;
    }

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    MutID = ((MUX_CB_S *)mutex_id)->ucMuxID;

    uwRet = LOS_MuxPend(MutID, LOS_MS2Tick(millisec));

    if(uwRet == LOS_OK)
    {
        return osOK;
    }
    else if(uwRet == LOS_ERRNO_MUX_TIMEOUT)
    {
        return osErrorTimeoutResource;
    }
    else if(uwRet == LOS_ERRNO_MUX_UNAVAILABLE)
    {
        return osErrorResource;
    }
    else if(uwRet == LOS_ERRNO_MUX_PEND_INTERR)
    {
        return osErrorISR;
    }
    else
    {
        return osErrorParameter;
    }
#endif
}

/// Release a Mutex that was obtained by \ref osMutexWait.
/// \param[in]     mutex_id      mutex ID obtained by \ref osMutexCreate.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osMutexRelease shall be consistent in every CMSIS-RTOS.
/*
osOK: the mutex has been correctly released.
osErrorResource: the mutex was not obtained before.
osErrorParameter: the parameter mutex_id is incorrect.
osErrorISR: osMutexRelease cannot be called from interrupt service routines.        //
*/
osStatus osMutexRelease (osMutexId mutex_id)
{
#if (LOSCFG_BASE_IPC_MUX == YES)
    UINT32  uwRet;
    UINT32  MutID;


    if (mutex_id == NULL)
    {
        return osErrorParameter;
    }

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    MutID = ((MUX_CB_S *)mutex_id)->ucMuxID;
    uwRet = LOS_MuxPost(MutID);

    if(uwRet == LOS_OK)
    {
        return osOK;
    }
    else
    {
        return osErrorResource;
    }
#endif
}

/// Delete a Mutex that was created by \ref osMutexCreate.
/// \param[in]     mutex_id      mutex ID obtained by \ref osMutexCreate.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osMutexDelete shall be consistent in every CMSIS-RTOS.
/*
osOK: the mutex object has been deleted.
osErrorISR: osMutexDelete cannot be called from interrupt service routines.    //osErrorISR
osErrorResource: all tokens have already been released.
osErrorParameter: the parameter mutex_id is incorrect.
*/
osStatus osMutexDelete (osMutexId mutex_id)
{
#if (LOSCFG_BASE_IPC_MUX == YES)
    UINT32  uwRet;
    UINT32  MutID;

    if (mutex_id == NULL)
    {
        return osErrorParameter;
    }

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    MutID = ((MUX_CB_S *)mutex_id)->ucMuxID;
    uwRet = LOS_MuxDelete(MutID);

    if(uwRet == LOS_OK)
    {
        return osOK;
    }
    else if(uwRet == LOS_ERRNO_MUX_INVALID)
    {
        return osErrorResource;
    }
    else
    {
        return osErrorParameter;
    }
#endif
}

osPoolId osPoolCreate (const osPoolDef_t *pool_def)
{
    UINT32 uwBlkSize, uwBoxSize;
    UINT32 uwRet;

    if ((pool_def == NULL) ||
            (pool_def->pool_sz == 0) ||
            (pool_def->item_sz == 0) ||
            (pool_def->pool == NULL))
    {
        return (osPoolId)NULL;
    }

    uwBlkSize = (pool_def->item_sz + 3) & ~3;
    uwBoxSize = /*sizeof(OS_MEMBOX_S) + */pool_def->pool_sz * uwBlkSize; /* delete sizeof(OS_MEMBOX_S) after membox management change */

    uwRet = LOS_MemboxInit(pool_def->pool, uwBoxSize, uwBlkSize);
    if(uwRet != LOS_OK)
    {
        return (osPoolId)NULL;
    }

    return (osPoolId)(pool_def->pool);
}

void *osPoolAlloc (osPoolId pool_id)
{
    void *ptr;

    if (pool_id == NULL)
        return NULL;

    ptr = LOS_MemboxAlloc(pool_id);

    return ptr;
}


void *osPoolCAlloc (osPoolId pool_id)
{
    void *ptr;

    if (pool_id == NULL)
        return NULL;

    ptr = LOS_MemboxAlloc(pool_id);

    if (ptr)
        LOS_MemboxClr(pool_id, ptr);

    return ptr;
}


osStatus osPoolFree (osPoolId pool_id, void *block)
{
    INT32 res;

    if (pool_id == NULL)
        return osErrorParameter;

    res = LOS_MemboxFree(pool_id, block);

    if (res != 0)
        return osErrorValue;

    return osOK;
}

// Message Queue Management Public API

static inline UINT32 osMessageCheckRet(UINT32 uwRet)
{
    if (uwRet == LOS_OK)
    {
        uwRet = osOK;
    }
    else if (uwRet == LOS_ERRNO_QUEUE_INVALID || uwRet == LOS_ERRNO_QUEUE_WRITE_IN_INTERRUPT)
    {
        uwRet = osErrorParameter;
    }
    else if (uwRet == LOS_ERRNO_QUEUE_TIMEOUT || uwRet == LOS_ERRNO_QUEUE_ISFULL)
    {
        uwRet = osEventTimeout;
    }
    else
    {
        uwRet = osErrorOS;
    }
    return uwRet;
}


/// Create and Initialize Message Queue
osMessageQId osMessageCreate(osMessageQDef_t *queue_def, osThreadId thread_id)
{
#if (LOSCFG_BASE_IPC_QUEUE == YES)
    UINT32 uwQueueID;
    UINT32 uwRet;

    (void)(thread_id);
    if (NULL == queue_def)
    {
        return (osMessageQId)NULL;
    }
    uwRet = LOS_QueueCreate((char *)NULL, (UINT16)(queue_def->queue_sz), &uwQueueID, 0, (UINT16)( queue_def->item_sz));
    if (uwRet == LOS_OK)
    {
        return (osMessageQId)GET_QUEUE_HANDLE(uwQueueID);
    }
    else
    {
        return (osMessageQId)NULL;
    }
#endif
}

/// Put a Message to a Queue header
osStatus osMessagePutHead(const osMessageQId queue_id, UINT32 info, UINT32 millisec)
{
#if (LOSCFG_BASE_IPC_QUEUE == YES)
    UINT32 uwRet;
    uwRet = LOS_QueueWriteHead(MESSAGEQID_TO_QUEUEID(queue_id), (void *)info, sizeof(UINT32), LOS_MS2Tick(millisec));
    uwRet = osMessageCheckRet(uwRet);
    return (osStatus)uwRet;
#endif
}

/// Put a Message to a Queue
osStatus osMessagePut(const osMessageQId queue_id, UINT32 info, UINT32 millisec)
{
#if (LOSCFG_BASE_IPC_QUEUE == YES)
    UINT32 uwRet;
    if(NULL == queue_id)
    {
        return osErrorParameter;
    }

    uwRet = LOS_QueueWrite(MESSAGEQID_TO_QUEUEID(queue_id), (void *)info, sizeof(UINT32), LOS_MS2Tick(millisec));
    uwRet = osMessageCheckRet(uwRet);
    return (osStatus)uwRet;
#endif
}

/// Get a Message or Wait for a Message from a Queue
osEvent osMessageGet(osMessageQId queue_id, UINT32 millisec)
{
#if (LOSCFG_BASE_IPC_QUEUE == YES)
    osEvent ret;
    UINT32 uwRet;
    if(NULL == queue_id)
    {
        ret.status = osErrorParameter;
        return ret;
    }
    (VOID)memset((void *)(&ret), 0, sizeof(osEvent));
    uwRet = LOS_QueueRead(MESSAGEQID_TO_QUEUEID(queue_id), &(ret.value.v), sizeof(UINT32), LOS_MS2Tick(millisec));
    if (uwRet == LOS_OK)
    {
        ret.status = osEventMessage;
    }
    else if (uwRet == LOS_ERRNO_QUEUE_INVALID || uwRet == LOS_ERRNO_QUEUE_READ_IN_INTERRUPT)
    {
        ret.status = osErrorParameter;
    }
    else if (uwRet == LOS_ERRNO_QUEUE_ISEMPTY || uwRet == LOS_ERRNO_QUEUE_TIMEOUT)
    {
        ret.status = osEventTimeout;
    }
    else
    {
        ret.status = osErrorOS;
    }

    return ret;
#endif
}


// Mail Queue Management Public API

/// Create and Initialize mail queue
osMailQId osMailCreate(osMailQDef_t *queue_def, osThreadId thread_id)
{
#if (LOSCFG_BASE_IPC_QUEUE == YES)
    UINT32 uwRet;
    UINT32 uwQueueID;
    UINT32 uwBlkSize, uwBoxSize;

    (void)(thread_id);
    if (NULL == queue_def)
    {
        return (osMailQId)NULL;
    }
    uwRet = LOS_QueueCreate((char *)NULL, (UINT16)(queue_def->queue_sz), &uwQueueID, 0, sizeof(UINT32));
    if (uwRet == LOS_OK)
    {
        *(UINT32 *)(((void **)queue_def->pool) + 0) = (UINT32)(GET_QUEUE_HANDLE(uwQueueID));
        uwBlkSize = (queue_def->item_sz + 3) & (~3);
        uwBoxSize = /* sizeof(OS_MEMBOX_S) + */queue_def->queue_sz * uwBlkSize; /* delete sizeof(OS_MEMBOX_S) after membox management change */

        (void)LOS_MemboxInit(*(((void **)queue_def->pool) + 1), uwBoxSize, uwBlkSize);
        return (osMailQId)queue_def->pool;
    }
    return (osMailQId)NULL;
#endif
}

/// Allocate a memory block from a mail
void *osMailAlloc(osMailQId queue_id, UINT32 millisec)
{
#if (LOSCFG_BASE_IPC_QUEUE == YES)
    void *pool = NULL;
    UINT32 uwQueueID;

    if (queue_id == NULL)
    {
        return NULL;
    }

    uwQueueID = *((UINT32 *)(((void **)queue_id) + 0));
    pool = *((((void **)queue_id) + 1));

    return (void *)osQueueMailAlloc(MESSAGEQID_TO_QUEUEID(uwQueueID), pool, LOS_MS2Tick(millisec));
#endif
}

/// Allocate a memory block from a mail and set memory block to zero
void *osMailCAlloc(osMailQId queue_id, UINT32 millisec)
{
#if (LOSCFG_BASE_IPC_QUEUE == YES)
    void *mem = NULL;
    LOS_MEMBOX_INFO *pool = (LOS_MEMBOX_INFO *)NULL;
    mem = osMailAlloc(queue_id, millisec);

    if (mem != NULL)
    {
        pool = (LOS_MEMBOX_INFO *)(*(((void **)queue_id) + 1));
        (VOID)memset(mem, 0, pool->uwBlkSize);
    }

    return mem;
#endif
}

/// Free a memory block from a mail
osStatus osMailFree(osMailQId queue_id, void *mail)
{
#if (LOSCFG_BASE_IPC_QUEUE == YES)
    void *pool = NULL;
    UINT32 uwQueueID;
    UINT32 uwRet;

    if (queue_id == NULL)
    {
        return osErrorParameter;
    }

    uwQueueID = *((UINT32 *)(((void **)queue_id) + 0));
    pool = *((((void **)queue_id) + 1));

    uwRet = osQueueMailFree(MESSAGEQID_TO_QUEUEID(uwQueueID), pool, mail);
    if (uwRet == LOS_ERRNO_QUEUE_MAIL_HANDLE_INVALID || uwRet == LOS_ERRNO_QUEUE_MAIL_PTR_INVALID)
    {
        return osErrorParameter;
    }
    else if (uwRet == LOS_ERRNO_QUEUE_MAIL_FREE_ERROR)
    {
        return osErrorOS;
    }
    return osOK;
#endif
}

/// Put a mail to a queue Header
osStatus osMailPutHead(osMailQId queue_id, void *mail)
{
#if (LOSCFG_BASE_IPC_QUEUE == YES)
    UINT32 uwQueueID;

    if (queue_id == NULL)
    {
        return osErrorParameter;
    }

    if (mail == NULL)
    {
        return osErrorValue;
    }

    uwQueueID = *((UINT32 *)(((void **)queue_id) + 0));

    return osMessagePutHead((osMessageQId)uwQueueID, (UINT32)mail, 0);
#endif
}

/// Put a mail to a queue
osStatus osMailPut(osMailQId queue_id, void *mail)
{
#if (LOSCFG_BASE_IPC_QUEUE == YES)
    UINT32 uwQueueID;

    if (queue_id == NULL)
    {
        return osErrorParameter;
    }

    if (mail == NULL)
    {
        return osErrorValue;
    }

    uwQueueID = *((UINT32 *)(((void **)queue_id) + 0));

    return osMessagePut((osMessageQId)uwQueueID, (UINT32)mail, 0);
#endif
}

/// Get a mail from a queue
osEvent osMailGet(osMailQId queue_id, UINT32 millisec)
{
#if (LOSCFG_BASE_IPC_QUEUE == YES)
    UINT32 uwQueueID;
    osEvent ret;

    if (queue_id == NULL)
    {
        ret.status = osErrorParameter;
        return ret;
    }

    uwQueueID = *((UINT32 *)(((void **)queue_id) + 0));
    ret = osMessageGet((osMessageQId)uwQueueID, millisec);

    if (ret.status == osEventMessage)
    {
        ret.status = osEventMail;
    }
    return ret;
#endif
}

/*lint -e1055 -e534*/
osStatus osMailClear(osMailQId queue_id)
{
#if (LOSCFG_BASE_IPC_QUEUE == YES)
    osEvent evt;
    UINTPTR uvIntSave;
    uvIntSave = LOS_IntLock();
    while(1)
    {
        evt = osMailGet(queue_id, 0);
        if(evt.status == osEventMail)
        {
            (VOID)osMailFree(queue_id, evt.value.p);
        }
        else if(evt.status == osEventTimeout)
        {
            (VOID)LOS_IntRestore(uvIntSave);
            return osOK;
        }
        else
        {
            (VOID)LOS_IntRestore(uvIntSave);
            return evt.status;
        }
    }
#endif
}

INT32 osSignalSet (osThreadId thread_id, INT32 signals)
{
    EVENT_CB_S sig;
    UINT32 old_sig;
    UINT32 uwRet;

    if (((LOS_TASK_CB *)thread_id) == NULL)
    {
        return 0x80000000;/*lint !e569*/
    }

    if (signals & (~((0x1 << osFeature_Signals) - 1)))
    {
        return osErrorValue;
    }

    sig = ((LOS_TASK_CB *)thread_id)->uwEvent;
    old_sig = sig.uwEventID;
    if (sig.uwEventID == 0xFFFFFFFF)
    {
        uwRet = LOS_EventInit(&(((LOS_TASK_CB *)thread_id)->uwEvent));
        if (uwRet != LOS_OK)
        {
            return osErrorOS;
        }
    }
    uwRet = LOS_EventWrite(&(((LOS_TASK_CB *)thread_id)->uwEvent), signals);
    if (uwRet != LOS_OK)
    {
        return osErrorOS;
    }

    return old_sig;
}

INT32 osSignalClear (osThreadId thread_id, INT32 signals)
{
    EVENT_CB_S sig;
    UINT32 old_sig;
    UINT32 uwRet;

    if (((LOS_TASK_CB *)thread_id) == NULL)
    {
        return 0x80000000; /*lint !e569*/
    }

    if (signals & (~((0x1 << osFeature_Signals) - 1)))
    {
        return osErrorValue;
    }

    sig = ((LOS_TASK_CB *)thread_id)->uwEvent;
    old_sig = sig.uwEventID;
    uwRet = LOS_EventClear(&(((LOS_TASK_CB *)thread_id)->uwEvent), ~(UINT32)signals);
    if (uwRet != LOS_OK)
    {
        return osErrorValue;
    }

    return old_sig;
}

osEvent osSignalWait (INT32 signals, UINT32 millisec)
{
    UINT32 uwRet = 0;
    osEvent ret;
    UINT32 uwFlags = 0;
    UINT32 uwTimeOut = osWaitForever;
    EVENT_CB_S sig;
    LOS_TASK_CB  *pstRunTsk;

    if (OS_INT_ACTIVE)
    {
        /* Not allowed in ISR */
        ret.status = osErrorISR;
        return ret;
    }

    if (signals & (~((0x1 << osFeature_Signals) - 1)))
    {
        ret.status = osErrorValue;
        return ret;
    }

    if (signals != 0)
    {
        uwFlags |= LOS_WAITMODE_AND;
    }
    else
    {
        signals = 0xFFFFFFFF & ((0x1 << osFeature_Signals) - 1);
        uwFlags |= LOS_WAITMODE_OR;
    }

    uwTimeOut = LOS_MS2Tick(millisec);

    pstRunTsk = g_stLosTask.pstRunTask;
    sig = ((LOS_TASK_CB *)pstRunTsk)->uwEvent;
    if (sig.uwEventID == 0xFFFFFFFF)
    {
        uwRet = LOS_EventInit(&(((LOS_TASK_CB *)(g_stLosTask.pstRunTask))->uwEvent));
        if (uwRet != LOS_OK)
        {
            ret.status = osErrorOS;
            return ret;
        }
    }
    uwRet = LOS_EventRead(&(((LOS_TASK_CB *)(g_stLosTask.pstRunTask))->uwEvent), signals, uwFlags | LOS_WAITMODE_CLR, uwTimeOut);
    if (uwRet == LOS_ERRNO_EVENT_READ_TIMEOUT)
    {
        ret.status = osEventTimeout;
        ret.value.signals = 0;
    }
    else if (uwRet == 0)
    {
        ret.status = osOK;
        ret.value.signals = 0;
    }
    else if(uwRet == LOS_ERRNO_EVENT_PTR_NULL ||
            uwRet == LOS_ERRNO_EVENT_EVENTMASK_INVALID ||
            uwRet == LOS_ERRNO_EVENT_READ_IN_LOCK ||
            uwRet == LOS_ERRNO_EVENT_READ_IN_INTERRUPT)
    {
        ret.status = osErrorOS;
        ret.value.signals = 0;
    }
    else
    {
        ret.status = osEventSignal;
        ret.value.signals = uwRet;
    }

    return ret;
}

#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
osTimerId osTimerExtCreate (const osTimerDef_t *timer_def, os_timer_type type, void *argument, os_timer_rouses_type ucRouses, os_timer_align_type ucSensitive)
{
    SWTMR_CTRL_S *pstSwtmr = NULL;
#if (LOSCFG_BASE_CORE_SWTMR == YES)
    UINT32 uwRet;
    UINT16 usSwTmrID;

    if ((timer_def == NULL)
            || (timer_def->ptimer == NULL)
            || (timer_def->default_interval == 0)
            || ((type != osTimerOnce) && (type != osTimerPeriodic)))
    {
        return NULL;
    }

    uwRet = LOS_SwtmrCreate(timer_def->default_interval, type,
                            (SWTMR_PROC_FUNC)(timer_def->ptimer),
                            &usSwTmrID, (UINT32)argument, ucRouses, ucSensitive);

    if (uwRet != LOS_OK)
    {
        return NULL;
    }

    pstSwtmr = OS_SWT_FROM_SID(usSwTmrID);
#endif
    return pstSwtmr;
}
#endif

osTimerId osTimerCreate (const osTimerDef_t *timer_def, os_timer_type type, void *argument)
{
    SWTMR_CTRL_S *pstSwtmr = (SWTMR_CTRL_S *)NULL;
#if (LOSCFG_BASE_CORE_SWTMR == YES)
    UINT32 uwRet;
    UINT16 usSwTmrID;

    if ((timer_def == NULL)
            || (timer_def->ptimer == NULL)
            || (timer_def->default_interval == 0)
            || ((type != osTimerOnce) && (type != osTimerPeriodic) && (type != osTimerDelay)))
    {
        return (osTimerId)NULL;
    }

    uwRet = LOS_SwtmrCreate(timer_def->default_interval, type,
                            (SWTMR_PROC_FUNC)(timer_def->ptimer),
                            &usSwTmrID, (UINT32)argument
#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
                            , osTimerRousesAllow, osTimerAlignIgnore
#endif
                           );

    if (uwRet != LOS_OK)
    {
        return (osTimerId)NULL;
    }

    pstSwtmr = OS_SWT_FROM_SID(usSwTmrID);
#endif
    return pstSwtmr;
}

osStatus osTimerStart (osTimerId timer_id, UINT32 millisec)
{
#if (LOSCFG_BASE_CORE_SWTMR == YES)
    SWTMR_CTRL_S  *pstSwtmr;
    UINT32   uwInterval;

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    pstSwtmr = (SWTMR_CTRL_S *)timer_id;
    if (pstSwtmr == NULL)
    {
        return osErrorParameter;
    }

    uwInterval = LOS_MS2Tick(millisec);
    if (uwInterval == 0)
    {
        return osErrorValue;
    }

    pstSwtmr->uwInterval = uwInterval;
    if (LOS_SwtmrStart(pstSwtmr->usTimerID))
    {
        return osErrorResource;
    }
#endif
    return osOK;
}

osStatus osTimerStop (osTimerId timer_id)
{
#if (LOSCFG_BASE_CORE_SWTMR == YES)
    SWTMR_CTRL_S  *pstSwtmr;

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    pstSwtmr = (SWTMR_CTRL_S *)timer_id;
    if (pstSwtmr == NULL)
    {
        return osErrorParameter;
    }

    if (LOS_SwtmrStop(pstSwtmr->usTimerID))
    {
        return osErrorResource;
    }
#endif
    return osOK;
}

osStatus osTimerRestart (osTimerId timer_id, UINT32 millisec, UINT8 strict)
{
#if (LOSCFG_BASE_CORE_SWTMR == YES)
    osStatus status = osOK;

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    status = osTimerStop(timer_id);
    if (strict && (status != osOK))
    {
        return status;
    }

    status = osTimerStart(timer_id, millisec);
    if (status != osOK)
    {
        return status;
    }
#endif
    return osOK;
}

osStatus osTimerDelete (osTimerId timer_id)
{
#if (LOSCFG_BASE_CORE_SWTMR == YES)
    SWTMR_CTRL_S  *pstSwtmr;

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    pstSwtmr = (SWTMR_CTRL_S *)timer_id;
    if (pstSwtmr == NULL)
    {
        return osErrorParameter;
    }

    if (LOS_SwtmrDelete(pstSwtmr->usTimerID))
    {
        return osErrorResource;
    }
#endif
    return osOK;

}

osStatus osDelay (UINT32 millisec)
{
    UINT32   uwInterval;
    UINT32   uwRet = 0;

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    if (millisec == 0)
    {
        return osOK;
    }

    //check if in idle we use udelay instead
    if(g_uwIdleTaskID == LOS_CurTaskIDGet())
    {
        PRINT_ERR("Idle Task Do Not Support Delay!\n");
        return osOK;
    }

    uwInterval = LOS_MS2Tick(millisec);

    uwRet = LOS_TaskDelay(uwInterval);

    if (uwRet == LOS_OK)
    {
        return osEventTimeout;
    }
    else
    {
        return osErrorResource;
    }
}

#if (defined (osFeature_Wait)  &&  (osFeature_Wait != 0))
osEvent osWait (UINT32 millisec)
{
    osEvent evt;
    UINT32   uwInterval;
    UINT32   uwRet = 0;

    if (OS_INT_ACTIVE)
    {
        evt.status = osErrorISR;
        return evt;
    }

    if (millisec == 0)
    {
        evt.status = osOK;
        return evt;
    }

    /* TODO: osEventSignal, osEventMessage, osEventMail */
    uwInterval = LOS_MS2Tick(millisec);

    uwRet = LOS_TaskDelay(uwInterval);

    if (uwRet == LOS_OK)
    {
        evt.status = osEventTimeout;
    }
    else
    {
        evt.status = osErrorResource;
    }

    return evt;
}
#endif

#if (LOSCFG_COMPAT_CMSIS_FW == YES)

fwMailQId g_fwMailQList = (fwMailQId)NULL;
UINT32 g_maxEventTime = 0x400;
#endif

fwMailQId fwMailCreate (fwMailQDef_t *queue_def, osThreadId thread_id)
{
#if (LOSCFG_COMPAT_CMSIS_FW == YES)
    // add mailQ to list
    (void)osMailCreate(queue_def->queue_id, thread_id);
    queue_def->next = (struct fw_MailQ_def *)g_fwMailQList;
    g_fwMailQList = (fwMailQId)queue_def;
    return (fwMailQId)queue_def;
#else
    return osMailCreate(queue_def, thread_id);
#endif
}

void *fwMailAlloc (fwMailQId queue_id, UINT32 millisec, UINT8 tag, UINT8 cmd)
{
    void *mem = NULL;

    if (queue_id == NULL)
        return NULL;
#if (LOSCFG_COMPAT_CMSIS_FW == YES)
    mem = osMailAlloc((osMailQId)((((fwMailQDef_t *)queue_id)->queue_id)->pool), millisec);
#else
    mem = osMailAlloc(queue_id, millisec);
#endif

    if (mem != NULL)
    {
        ((fw_event_t *)mem)->cmd = cmd;
        ((fw_event_t *)mem)->tag = tag;
    }

    return mem;
}

void *fwMailCAlloc (fwMailQId queue_id, UINT32 millisec, UINT8 tag, UINT8 cmd)
{
    void *mem = NULL;

    if (queue_id == NULL)
        return NULL;
#if (LOSCFG_COMPAT_CMSIS_FW == YES)
    mem = osMailCAlloc((osMailQId)((((fwMailQDef_t *)queue_id)->queue_id)->pool), millisec);
#else
    mem = osMailCAlloc(queue_id, millisec);
#endif

    if (mem != NULL)
    {
        ((fw_event_t *)mem)->cmd = cmd;
        ((fw_event_t *)mem)->tag = tag;
    }

    return mem;
}

osStatus fwMailFree (fwMailQId queue_id, void *mail)
{
    if (queue_id == NULL)
        return osErrorParameter;

#if (LOSCFG_COMPAT_CMSIS_FW == YES)
    return osMailFree((osMailQId)((((fwMailQDef_t *)queue_id)->queue_id)->pool), mail);
#else
    return osMailFree(queue_id, mail);
#endif
}

osStatus fwMailPut (fwMailQId queue_id, void *mail)
{
    if (queue_id == NULL)
        return osErrorParameter;
#if (LOSCFG_COMPAT_CMSIS_FW == YES)
    return osMailPut((osMailQId)((((fwMailQDef_t *)queue_id)->queue_id)->pool), mail);
#else
    return osMailPut(queue_id, mail);
#endif
}
/*lint  -e438 -e550 -e529*/
osEvent fwMailGet (fwMailQId queue_id, UINT32 millisec)
{
    osEvent evt;
    UINT32 max_time;
    void *pool;
    UINT32 uwQueueID;

    if (queue_id == NULL)
    {
        evt.status = osErrorParameter;
        return evt;
    }
#if (LOSCFG_COMPAT_CMSIS_FW == YES)

    pool = ((((fwMailQDef_t *)queue_id)->queue_id)->pool);
    uwQueueID = *((UINT32 *)(((void **)(pool)) + 0));
    max_time = GET_EVENT_MAXTIME(queue_id) != 0 ? GET_EVENT_MAXTIME(queue_id) : g_maxEventTime;

    if (((fwMailQDef_t *)queue_id)->event_begin_time != 0 &&
            (osKernelSysTick() - ((fwMailQDef_t *)queue_id)->event_begin_time ) > max_time)
    {
        ((fwMailQDef_t *)queue_id)->timeout_cnt++;
        PRINT_ERR("[ERR] Get QID %d TIMEOUTCNT %d\n", uwQueueID, ((fwMailQDef_t *)queue_id)->timeout_cnt);
    }

    ((fwMailQDef_t *)queue_id)->event_begin_time = 0;
    evt = osMailGet((osMailQId)((((fwMailQDef_t *)queue_id)->queue_id)->pool), millisec);
    if (evt.status == osEventMail)
    {
        ((fwMailQDef_t *)queue_id)->last_event = *(fw_event_t *)(evt.value.p);
        ((fwMailQDef_t *)queue_id)->event_begin_time = osKernelSysTick();
    }
#else
    evt = osMailGet(queue_id, millisec);
#endif
    return evt;
}

/*lint  -e438 -e550*/
UINT32 fwMailQGetStatus(void)
{
#if (LOSCFG_COMPAT_CMSIS_FW == YES)
    fwMailQDef_t *ptr = (fwMailQDef_t *)NULL;
    void *pool = NULL;
    UINT32 uwQueueID;
    UINT32 curr_tick = osKernelSysTick();
    UINT32 max_time = 0;
    UINT8 ret = 0;

    ptr = (fwMailQDef_t *)g_fwMailQList;
    while (ptr != NULL)
    {
        max_time = GET_EVENT_MAXTIME(ptr) != 0 ? GET_EVENT_MAXTIME(ptr) : g_maxEventTime;
        pool = ((ptr->queue_id)->pool);
        uwQueueID = *((UINT32 *)(((void **)(pool)) + 0));
        if ( ptr->event_begin_time != 0 && (curr_tick - ptr->event_begin_time) > max_time)
        {
            ptr->timeout_cnt++;
            PRINT_ERR("[ERR] QID %d OUTQUE %d Phase %d\n", uwQueueID, ptr->event_begin_time, GET_EVENT_PHASE(ptr));
            PRINT_ERR("TAG %d CMD %d\n", ptr->last_event.tag, ptr->last_event.cmd);
            ret++;
        }
        if (ptr->timeout_cnt != 0)
        {
            PRINT_ERR("QID %d TIMEOUTCNT %d\n", uwQueueID, ptr->timeout_cnt);
            ret += ptr->timeout_cnt;
            ptr->timeout_cnt = 0;
        }
        ptr = ptr->next;
    }

    if (ret)
        return 1;

    return 0;
#else
    return 0;
#endif
}
#endif // (CMSIS_OS_VER == 1)
#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */
