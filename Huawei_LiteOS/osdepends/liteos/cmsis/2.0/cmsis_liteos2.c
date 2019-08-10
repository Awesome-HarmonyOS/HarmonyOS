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
#include "los_typedef.h"
#include "los_printf.h"

#include "los_event.h"
#include "los_membox.h"
#include "los_hwi.h"

#include "los_mux.ph"
#include "los_queue.ph"
#include "los_sem.ph"
#include "los_memory.ph"
#include "los_swtmr.ph"
#include "los_sys.ph"
#include "los_task.ph"
#include "los_tick.ph"
//#include "los_sleep.h"

//#include "hal_clocks.h"
#include "string.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */
#if (CMSIS_OS_VER == 2)

/* Kernel initialization state */
static osKernelState_t KernelState;

extern BOOL g_bTaskScheduled;

#ifdef LOS_RUNSTOP
cb_uart_is_need_awake_fn uart_is_need_awake_callback = NULL;
#endif
#define LOS_PRIORITY_WIN 8

const osVersion_t g_stLosVersion = {001, 001};

#define LITEOS_VERSION_MAJOR    1
#define LITEOS_VERSION_MINOR    0
#define LITEOS_VERSION_BUILD    0

/* Kernel version and identification string definition */
#define KERNEL_VERSION            (((UINT32)LITEOS_VERSION_MAJOR * 10000000UL) | \
                                   ((UINT32)LITEOS_VERSION_MINOR *    10000UL) | \
                                   ((UINT32)LITEOS_VERSION_BUILD *        1UL))

#define KERNEL_ID   "HUAWEI-LiteOS"

#ifndef UNUSED
#define UNUSED(var) do { (void)var; } while(0)
#endif
//  ==== Kernel Management Functions ====
uint32_t osTaskStackWaterMarkGet(UINT32 uwTaskID);

extern VOID LOS_GetSystickCycle(UINT32 *puwCntHi, UINT32 *puwCntLo);

osStatus_t osKernelInitialize (void)
{
    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    if (KernelState != osKernelInactive)
    {
        return osError;
    }

    if(LOS_OK == LOS_KernelInit())
    {
        KernelState = osKernelReady;
        return osOK;
    }
    else
    {
        return osError;
    }
}


osStatus_t osKernelGetInfo (osVersion_t *version, char *id_buf, uint32_t id_size)
{
    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    if (version != NULL)
    {
        version->api = g_stLosVersion.api;
        version->kernel = g_stLosVersion.kernel;
    }

    if ((id_buf != NULL) && (id_size != 0U))
    {
        if (id_size > sizeof(KERNEL_ID))
        {
            id_size = sizeof(KERNEL_ID);
        }
        memcpy(id_buf, KERNEL_ID, id_size);
    }

    return osOK;
}


osKernelState_t osKernelGetState (void)
{
    if (OS_INT_ACTIVE)
    {
        return osKernelError;
    }

    if(!g_bTaskScheduled)
    {
        if (KernelState == osKernelReady)
        {
            return osKernelReady;
        }
        else
        {
            return osKernelInactive;
        }
    }
    else if(g_usLosTaskLock > 0)
    {
        return osKernelLocked;
    }
    else
    {
        return osKernelRunning;
    }
}


osStatus_t osKernelStart (void)
{
    if(OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    if(KernelState == osKernelReady)
    {
        if(LOS_OK == LOS_Start())
        {
            KernelState = osKernelRunning;
            return osOK;
        }
        else
        {
            return osError;
        }
    }
    else
    {
        return osError;
    }
}


int32_t osKernelLock (void)
{
    int32_t lock;

    if(OS_INT_ACTIVE)
    {
        return (int32_t)osErrorISR;
    }

    if(!g_bTaskScheduled)
    {
        return (int32_t)osError;
    }

    if(g_usLosTaskLock > 0)
    {
        lock = 1;
    }
    else
    {
        LOS_TaskLock();
        lock = 0;
    }

    return lock;
}


int32_t osKernelUnlock (void)
{
    int32_t lock;

    if(OS_INT_ACTIVE)
    {
        return (int32_t)osErrorISR;
    }

    if(!g_bTaskScheduled)
    {
        return (int32_t)osError;
    }

    if(g_usLosTaskLock > 0)
    {
        LOS_TaskUnlock();
        if (g_usLosTaskLock != 0)
        {
            return (int32_t)osError;
        }
        lock = 1;
    }
    else
    {
        lock = 0;
    }

    return lock;
}


int32_t osKernelRestoreLock (int32_t lock)
{
    if(OS_INT_ACTIVE)
    {
        return (int32_t)osErrorISR;
    }

    if(!g_bTaskScheduled)
    {
        return (int32_t)osError;
    }

    switch (lock)
    {
    case 0:
        LOS_TaskUnlock();
        if (g_usLosTaskLock != 0)
        {
            break;
        }
        return 0;
    case 1:
        LOS_TaskLock();
        return 1;
    default:
        break;
    }

    return (int32_t)osError;

}


uint64_t osKernelGetTickCount (void)
{
    uint64_t ticks;
    UINTPTR uvIntSave;

    if(OS_INT_ACTIVE)
    {
        ticks = 0U;
    }
    else
    {
        uvIntSave = LOS_IntLock();
        ticks = g_ullTickCount;
        LOS_IntRestore(uvIntSave);
    }

    return ticks;
}


uint64_t osKernelGetTick2ms(void)
{
    return osKernelGetTickCount() * (OS_SYS_MS_PER_SECOND / LOSCFG_BASE_CORE_TICK_PER_SECOND);
}


uint64_t osMs2Tick(uint64_t ms)
{

    return ms * (LOSCFG_BASE_CORE_TICK_PER_SECOND / OS_SYS_MS_PER_SECOND);
}


uint32_t osKernelGetTickFreq (void)
{
    uint32_t freq;

    if (OS_INT_ACTIVE)
    {
        freq = 0U;
    }
    else
    {
        freq = LOSCFG_BASE_CORE_TICK_PER_SECOND;
    }

    return (freq);
}


uint32_t osKernelGetSysTimerCount (void)
{
    uint32_t count_high = 0;
    uint32_t count_low = 0;
    if (OS_INT_ACTIVE)
    {
        count_low = 0U;
    }
    else
    {
        LOS_GetSystickCycle((UINT32 *)&count_high, (UINT32 *)&count_low);
    }
    return count_low;
}


uint32_t osKernelGetSysTimerFreq (void)
{
    return OS_SYS_CLOCK;
}


//  ==== Thread Management Functions ====

osThreadId_t osThreadNew (osThreadFunc_t func, void *argument, const osThreadAttr_t *attr)
{
    UNUSED(argument);

    UINT32 uwTid;
    UINT32 uwRet;
    LOS_TASK_CB *pstTaskCB = NULL;
    TSK_INIT_PARAM_S stTskInitParam;

    if (OS_INT_ACTIVE)
    {
        return NULL;
    }

    if ((attr == NULL) ||
            (func == NULL) ||
            (attr->priority < osPriorityLow1) ||
            (attr->priority > osPriorityAboveNormal6))
    {
        return (osThreadId_t)NULL;
    }

    memset(&stTskInitParam, 0, sizeof(TSK_INIT_PARAM_S));
    stTskInitParam.pfnTaskEntry = (TSK_ENTRY_FUNC)func;
    stTskInitParam.uwStackSize  = attr->stack_size * 4;
    stTskInitParam.pcName       = (CHAR *)attr->name;
    stTskInitParam.usTaskPrio   = OS_TASK_PRIORITY_LOWEST - ((UINT16)(attr->priority) - LOS_PRIORITY_WIN); /*0~31*/

    uwRet = LOS_TaskCreate(&uwTid, &stTskInitParam);

    if (LOS_OK != uwRet)
    {
        return (osThreadId_t)NULL;
    }

    pstTaskCB = OS_TCB_FROM_TID(uwTid);

    return (osThreadId_t)pstTaskCB;
}


const char *osThreadGetName (osThreadId_t thread_id)
{
    LOS_TASK_CB *pstTaskCB = NULL;

    if (OS_INT_ACTIVE || thread_id == NULL)
    {
        return NULL;
    }

    pstTaskCB = (LOS_TASK_CB *)thread_id;

    return pstTaskCB->pcTaskName;
}


osThreadId_t osThreadGetId (void)
{
    if (OS_INT_ACTIVE)
    {
        return NULL;
    }

    return (osThreadId_t)(g_stLosTask.pstRunTask);
}


osThreadState_t osThreadGetState (osThreadId_t thread_id)
{
    UINT16 usTaskStatus;
    osThreadState_t stState;
    LOS_TASK_CB *pstTaskCB = NULL;

    if (OS_INT_ACTIVE || thread_id == NULL)
    {
        return osThreadError;
    }

    pstTaskCB = (LOS_TASK_CB *)thread_id;
    usTaskStatus = pstTaskCB->usTaskStatus;

    if (usTaskStatus & OS_TASK_STATUS_RUNNING)
    {
        stState = osThreadRunning;
    }
    else if (usTaskStatus & OS_TASK_STATUS_READY)
    {
        stState = osThreadReady;
    }
    else if (usTaskStatus &
             (OS_TASK_STATUS_DELAY | OS_TASK_STATUS_PEND |
              OS_TASK_STATUS_SUSPEND | OS_TASK_STATUS_PEND_QUEUE))
    {
        stState = osThreadBlocked;
    }
    else if (usTaskStatus & OS_TASK_STATUS_UNUSED)
    {
        stState = osThreadInactive;
    }
    else
    {
        stState = osThreadError;
    }

    return stState;
}


uint32_t osThreadGetStackSize (osThreadId_t thread_id)
{
    LOS_TASK_CB *pstTaskCB = NULL;

    if (OS_INT_ACTIVE || thread_id == NULL)
    {
        return 0U;
    }

    pstTaskCB = (LOS_TASK_CB *)thread_id;

    return pstTaskCB->uwStackSize;
}


uint32_t osTaskStackWaterMarkGet(UINT32 uwTaskID)
{
    UINT32 uwCount = 0;
    UINT32 *puwTopOfStack;
    UINTPTR uvIntSave;
    LOS_TASK_CB *pstTaskCB = NULL;

    if (uwTaskID > LOSCFG_BASE_CORE_TSK_LIMIT)
    {
        return 0;
    }

    uvIntSave = LOS_IntLock();

    pstTaskCB = OS_TCB_FROM_TID(uwTaskID);
    if (OS_TASK_STATUS_UNUSED & (pstTaskCB->usTaskStatus))
    {
        (VOID)LOS_IntRestore(uvIntSave);
        return 0;
    }

    // first 4 bytes is OS_TASK_MAGIC_WORD, skip
    puwTopOfStack = (UINT32 *)pstTaskCB->uwTopOfStack + 1;

    while (*puwTopOfStack == (UINT32)OS_TASK_STACK_INIT)
    {
        ++puwTopOfStack;
        ++uwCount;
    }

    uwCount *= sizeof(UINT32);

    (VOID)LOS_IntRestore(uvIntSave);
    return uwCount;
}


uint32_t osThreadGetStackSpace (osThreadId_t thread_id)
{
    LOS_TASK_CB *pstTaskCB = NULL;

    if (OS_INT_ACTIVE || thread_id == NULL)
    {
        return 0U;
    }

    pstTaskCB = (LOS_TASK_CB *)thread_id;

    return osTaskStackWaterMarkGet(pstTaskCB->uwTaskID);
}


osStatus_t osThreadSetPriority (osThreadId_t thread_id, osPriority_t priority)
{
    UINT32  uwRet;
    UINT16  usPriority;
    LOS_TASK_CB *pstTaskCB = NULL;

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    if (thread_id == NULL)
    {
        return osErrorParameter;
    }

    if (priority < osPriorityLow1 || priority > osPriorityAboveNormal6)
    {
        return osErrorParameter;
    }

    pstTaskCB   = (LOS_TASK_CB *)thread_id;
    usPriority  = OS_TASK_PRIORITY_LOWEST - ((UINT16)priority - LOS_PRIORITY_WIN);
    uwRet       = LOS_TaskPriSet(pstTaskCB->uwTaskID, usPriority);
    switch (uwRet)
    {
    case LOS_ERRNO_TSK_PRIOR_ERROR:
    case LOS_ERRNO_TSK_OPERATE_IDLE:
    case LOS_ERRNO_TSK_ID_INVALID:
        return osErrorParameter;

    case LOS_ERRNO_TSK_NOT_CREATED:
        return osErrorResource;

    default:
        return osOK;
    }
}


osPriority_t osThreadGetPriority (osThreadId_t thread_id)
{
    UINT16 usRet;
    LOS_TASK_CB *pstTaskCB = NULL;

    if (OS_INT_ACTIVE || thread_id == NULL)
    {
        return osPriorityError;
    }

    pstTaskCB = (LOS_TASK_CB *)thread_id;
    usRet = LOS_TaskPriGet(pstTaskCB->uwTaskID);

    if (usRet == (UINT16)OS_INVALID)
    {
        return osPriorityError;
    }

    return (osPriority_t)(OS_TASK_PRIORITY_LOWEST - (usRet - LOS_PRIORITY_WIN));
}


osStatus_t osThreadYield (void)
{
    UINT32 uwRet;

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    uwRet = LOS_TaskYield();

    if (uwRet == LOS_OK)
    {
        return osOK;
    }

    return osError;
}


osStatus_t osThreadSuspend (osThreadId_t thread_id)
{
    UINT32 uwRet;
    LOS_TASK_CB *pstTaskCB = NULL;

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    if (thread_id == NULL)
    {
        return osErrorParameter;
    }

    pstTaskCB = (LOS_TASK_CB *)thread_id;
    uwRet = LOS_TaskSuspend(pstTaskCB->uwTaskID);
    switch (uwRet)
    {
    case LOS_ERRNO_TSK_OPERATE_IDLE:
    case LOS_ERRNO_TSK_SUSPEND_SWTMR_NOT_ALLOWED:
    case LOS_ERRNO_TSK_ID_INVALID:
        return osErrorParameter;

    case LOS_ERRNO_TSK_NOT_CREATED:
    case LOS_ERRNO_TSK_ALREADY_SUSPENDED:
    case LOS_ERRNO_TSK_SUSPEND_LOCKED:
        return osErrorResource;

    default:
        return osOK;
    }
}


osStatus_t osThreadResume (osThreadId_t thread_id)
{
    UINT32 uwRet;
    LOS_TASK_CB *pstTaskCB = NULL;

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    if (thread_id == NULL)
    {
        return osErrorParameter;
    }

    pstTaskCB = (LOS_TASK_CB *)thread_id;
    uwRet = LOS_TaskResume(pstTaskCB->uwTaskID);

    switch (uwRet)
    {
    case LOS_ERRNO_TSK_ID_INVALID:
        return osErrorParameter;

    case LOS_ERRNO_TSK_NOT_CREATED:
    case LOS_ERRNO_TSK_NOT_SUSPENDED:
        return osErrorResource;

    default:
        return osOK;
    }

}


osStatus_t osThreadTerminate (osThreadId_t thread_id)
{
    UINT32 uwRet;
    LOS_TASK_CB *pstTaskCB = NULL;

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    if (thread_id == NULL)
    {
        return osErrorParameter;
    }

    pstTaskCB = (LOS_TASK_CB *)thread_id;
    uwRet = LOS_TaskDelete(pstTaskCB->uwTaskID);

    switch (uwRet)
    {
    case LOS_ERRNO_TSK_OPERATE_IDLE:
    case LOS_ERRNO_TSK_SUSPEND_SWTMR_NOT_ALLOWED:
    case LOS_ERRNO_TSK_ID_INVALID:
        return osErrorParameter;

    case LOS_ERRNO_TSK_NOT_CREATED:
        return osErrorResource;

    default:
        return osOK;
    }
}


uint32_t osThreadGetCount (void)
{
    uint32_t uwCount = 0;
    int index = 0;

    if (OS_INT_ACTIVE)
    {
        return 0U;
    }

    for(; index <= LOSCFG_BASE_CORE_TSK_LIMIT; index++)
    {
        if (!((g_pstTaskCBArray + index)->usTaskStatus & OS_TASK_STATUS_UNUSED))
        {
            uwCount++;
        }
    }

    return uwCount;
}


//  ==== Generic Wait Functions ====

osStatus_t osDelay (uint32_t ticks)
{
    UINT32 uwRet = 0;

    uwRet = LOS_TaskDelay(ticks);
    if(uwRet == LOS_OK)
    {
        return osOK;
    }
    else
    {
        return osError;
    }
}


osStatus_t osDelayUntil (uint64_t ticks)
{
    UINT32 uwRet;
    UINT32 uwTicks;
    UINT64 tickCount = osKernelGetTickCount();

    if(ticks < tickCount)
    {
        return osError;
    }

    uwTicks = (UINT32)(ticks - tickCount);

    uwRet = LOS_TaskDelay(uwTicks);
    if(uwRet == LOS_OK)
    {
        return osOK;
    }
    else
    {
        return osError;
    }
}

//  ==== Timer Management Functions ====
#if (LOSCFG_BASE_CORE_SWTMR == YES)
osTimerId_t osTimerNew (osTimerFunc_t func, osTimerType_t type, void *argument, const osTimerAttr_t *attr)
{
    UNUSED(attr);
    UINT16 usSwTmrID;
    UINT8 mode;

    if ((OS_INT_ACTIVE) || (NULL == func) ||
            ((osTimerOnce != type) && (osTimerPeriodic != type)))
    {
        return (osTimerId_t)NULL;
    }

    if(osTimerOnce == type)
    {
        mode = LOS_SWTMR_MODE_NO_SELFDELETE;
    }
    else
    {
        mode = LOS_SWTMR_MODE_PERIOD;
    }
#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
    if (LOS_OK != LOS_SwtmrCreate(1, mode, (SWTMR_PROC_FUNC)func, &usSwTmrID, (UINT32)argument, OS_SWTMR_ROUSES_ALLOW, OS_SWTMR_ALIGN_SENSITIVE))
    {
        return (osTimerId_t)NULL;
    }
#else
    if (LOS_OK != LOS_SwtmrCreate(1, mode, (SWTMR_PROC_FUNC)func, &usSwTmrID, (UINT32)argument))
    {
        return (osTimerId_t)NULL;
    }
#endif

    return (osTimerId_t)OS_SWT_FROM_SID(usSwTmrID);
}


osStatus_t osTimerStart (osTimerId_t timer_id, uint32_t ticks)
{
    UINT32 uwRet;
    SWTMR_CTRL_S *pstSwtmr;

    if ((0 == ticks) || (NULL == timer_id))
    {
        return osErrorParameter;
    }

    pstSwtmr = (SWTMR_CTRL_S *)timer_id;
    pstSwtmr->uwInterval = LOS_Tick2MS(ticks);
    //pstSwtmr->uwExpiry = LOS_Tick2MS(ticks);
    uwRet = LOS_SwtmrStart(pstSwtmr->usTimerID);
    if (LOS_OK == uwRet)
    {
        return osOK;
    }
    else if (LOS_ERRNO_SWTMR_ID_INVALID == uwRet)
    {
        return osErrorParameter;
    }
    else
    {
        return osErrorResource;
    }
}


const char *osTimerGetName(osTimerId_t timer_id)
{
    UNUSED(timer_id);
    return (const char *)NULL;
}


osStatus_t osTimerStop (osTimerId_t timer_id)
{
    UINT32 uwRet;
    SWTMR_CTRL_S *pstSwtmr = (SWTMR_CTRL_S *)timer_id;

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    if (NULL == pstSwtmr)
    {
        return osErrorParameter;
    }

    uwRet = LOS_SwtmrStop(pstSwtmr->usTimerID);
    if (LOS_OK == uwRet)
    {
        return osOK;
    }
    else if (LOS_ERRNO_SWTMR_ID_INVALID == uwRet)
    {
        return osErrorParameter;
    }
    else
    {
        return osErrorResource;
    }
}


uint32_t osTimerIsRunning (osTimerId_t timer_id)
{
    if ((OS_INT_ACTIVE) || (NULL == timer_id))
    {
        return 0;
    }

    return (OS_SWTMR_STATUS_TICKING == ((SWTMR_CTRL_S *)timer_id)->ucState);
}


osStatus_t osTimerDelete (osTimerId_t timer_id)
{
    UINT32 uwRet;
    SWTMR_CTRL_S  *pstSwtmr = (SWTMR_CTRL_S *)timer_id;

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    if (NULL == pstSwtmr)
    {
        return osErrorParameter;
    }

    uwRet = LOS_SwtmrDelete(pstSwtmr->usTimerID);
    if (LOS_OK == uwRet)
    {
        return osOK;
    }
    else if (LOS_ERRNO_SWTMR_ID_INVALID == uwRet)
    {
        return osErrorParameter;
    }
    else
    {
        return osErrorResource;
    }
}
#endif

osEventFlagsId_t osEventFlagsNew (const osEventFlagsAttr_t *attr)
{
    PEVENT_CB_S pstEventCB = NULL;
    UINT32 uwRet;

    UNUSED(attr);

    if (OS_INT_ACTIVE)
    {
        return (osEventFlagsId_t)NULL;
    }

    pstEventCB = (PEVENT_CB_S)LOS_MemAlloc(m_aucSysMem0, sizeof(EVENT_CB_S));
    if(pstEventCB == NULL )
    {
        return (osEventFlagsId_t)NULL;
    }

    uwRet = LOS_EventInit(pstEventCB);
    if (uwRet == LOS_ERRNO_EVENT_PTR_NULL)
    {
        return (osEventFlagsId_t)NULL;
    }
    else
    {
        return (osEventFlagsId_t)pstEventCB;
    }
}


const char *osEventFlagsGetName (osEventFlagsId_t ef_id)
{
    UNUSED(ef_id);

    if (OS_INT_ACTIVE)
    {
        return (const char *)NULL;
    }

    return (const char *)NULL;
}


uint32_t osEventFlagsSet (osEventFlagsId_t ef_id, uint32_t flags)
{
    PEVENT_CB_S pstEventCB = (PEVENT_CB_S)ef_id;
    UINT32 uwRet;
    uint32_t rflags;

    uwRet = LOS_EventWrite(pstEventCB, (UINT32)flags);
    if (uwRet != LOS_OK)
    {
        return (uint32_t)osFlagsErrorParameter;
    }
    else
    {
        rflags = pstEventCB->uwEventID;
        return rflags;
    }
}


uint32_t osEventFlagsClear (osEventFlagsId_t ef_id, uint32_t flags)
{
    PEVENT_CB_S pstEventCB = (PEVENT_CB_S)ef_id;
    UINTPTR uwIntSave;
    uint32_t rflags;
    UINT32 uwRet;

    if (pstEventCB == NULL)
    {
        return (uint32_t)osFlagsErrorParameter;
    }

    uwIntSave = LOS_IntLock();
    rflags = pstEventCB->uwEventID;

    uwRet = LOS_EventClear(pstEventCB, ~flags);
    LOS_IntRestore(uwIntSave);
    if (uwRet !=  LOS_OK)
    {
        return (uint32_t)osFlagsErrorParameter;
    }
    else
    {
        return rflags;
    }
}


uint32_t osEventFlagsGet (osEventFlagsId_t ef_id)
{
    PEVENT_CB_S pstEventCB = (PEVENT_CB_S)ef_id;
    UINTPTR uwIntSave;
    uint32_t rflags;

    if (pstEventCB == NULL)
    {
        return (uint32_t)osFlagsErrorParameter;
    }

    uwIntSave = LOS_IntLock();
    rflags = pstEventCB->uwEventID;
    LOS_IntRestore(uwIntSave);

    return rflags;
}

uint32_t osEventFlagsWait (osEventFlagsId_t ef_id, uint32_t flags, uint32_t options, uint32_t timeout)
{
    PEVENT_CB_S pstEventCB = (PEVENT_CB_S)ef_id;
    UINT32 uwMode = 0;
    UINT32 uwRet;
    uint32_t rflags = 0;

    if (options > (osFlagsWaitAny | osFlagsWaitAll | osFlagsNoClear))
    {
        return (uint32_t)osFlagsErrorParameter;
    }

    if ((options & osFlagsWaitAll) == osFlagsWaitAll)
    {
        uwMode |= LOS_WAITMODE_AND;
    }
    else
    {
        uwMode |= LOS_WAITMODE_OR;
    }

    if ((options & osFlagsNoClear)  == osFlagsNoClear)
    {
        uwMode &= ~LOS_WAITMODE_CLR;
    }
    else
    {
        uwMode |= LOS_WAITMODE_CLR;
    }

    uwRet = LOS_EventRead(pstEventCB, (UINT32)flags, uwMode, (UINT32)timeout);
    switch(uwRet)
    {
    case LOS_ERRNO_EVENT_PTR_NULL:
    case LOS_ERRNO_EVENT_EVENTMASK_INVALID:
    case LOS_ERRNO_EVENT_SETBIT_INVALID:
        return (uint32_t)osFlagsErrorParameter;

    case LOS_ERRNO_EVENT_READ_IN_INTERRUPT:
    case LOS_ERRNO_EVENT_FLAGS_INVALID:
    case LOS_ERRNO_EVENT_READ_IN_LOCK:
        return (uint32_t)osFlagsErrorResource;

    case LOS_ERRNO_EVENT_READ_TIMEOUT:
        return (uint32_t)osFlagsErrorTimeout;

    default :
        rflags = (uint32_t)uwRet;
        return rflags;
    }
}


osStatus_t osEventFlagsDelete (osEventFlagsId_t ef_id)
{
    PEVENT_CB_S pstEventCB = (PEVENT_CB_S)ef_id;
    UINTPTR uwIntSave;
    osStatus_t uwRet;

    uwIntSave = LOS_IntLock();
    if (LOS_EventDestory(pstEventCB) == LOS_OK)
    {
        uwRet = osOK;
    }
    else
    {
        uwRet = osErrorParameter;
    }
    LOS_IntRestore(uwIntSave);

    if (LOS_MemFree(m_aucSysMem0, (void *)pstEventCB) == LOS_OK)
    {
        uwRet = osOK;
    }
    else
    {
        uwRet = osErrorParameter;
    }

    return uwRet;
}


//  ==== Mutex Management Functions ====
#if (LOSCFG_BASE_IPC_MUX == YES)
osMutexId_t osMutexNew (const osMutexAttr_t *attr)
{
    UINT32 uwRet;
    UINT32 uwMuxId;

    UNUSED(attr);

    if (OS_INT_ACTIVE)
    {
        return NULL;
    }

    uwRet = LOS_MuxCreate(&uwMuxId);

    if(uwRet == LOS_OK)
    {
        return (osMutexId_t)(GET_MUX(uwMuxId));
    }
    else
    {
        return (osMutexId_t)NULL;
    }
}


osStatus_t osMutexAcquire (osMutexId_t mutex_id, uint32_t timeout)
{
    UINT32  uwRet;

    if (mutex_id == NULL)
    {
        return osErrorParameter;
    }

    if (OS_INT_ACTIVE && (timeout != LOS_NO_WAIT))
    {
        timeout = 0;
    }

    uwRet = LOS_MuxPend(((MUX_CB_S *)mutex_id)->ucMuxID, timeout);

    if(uwRet == LOS_OK)
    {
        return osOK;
    }
    else if (uwRet == LOS_ERRNO_MUX_TIMEOUT)
    {
        return osErrorTimeout;
    }
    else if (uwRet == LOS_ERRNO_MUX_INVALID)
    {
        return osErrorParameter;
    }
    else
    {
        return osErrorResource;
    }
}


osStatus_t osMutexRelease (osMutexId_t mutex_id)
{
    UINT32  uwRet;

    if (mutex_id == NULL)
    {
        return osErrorParameter;
    }

    uwRet = LOS_MuxPost(((MUX_CB_S *)mutex_id)->ucMuxID);

    if (uwRet == LOS_OK)
    {
        return osOK;
    }
    else
    {
        return osErrorResource;
    }
}


osThreadId_t osMutexGetOwner (osMutexId_t mutex_id)
{
    UINT32 uwIntSave;
    LOS_TASK_CB *pstTaskCB;

    if (OS_INT_ACTIVE)
    {
        return NULL;
    }

    if (mutex_id == NULL)
    {
        return NULL;
    }

    uwIntSave = LOS_IntLock();
    pstTaskCB = ((MUX_CB_S *)mutex_id)->pstOwner;
    (VOID)LOS_IntRestore(uwIntSave);

    return (osThreadId_t)pstTaskCB;
}


osStatus_t osMutexDelete (osMutexId_t mutex_id)
{
    UINT32  uwRet;

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    if (mutex_id == NULL)
    {
        return osErrorParameter;
    }

    uwRet = LOS_MuxDelete(((MUX_CB_S *)mutex_id)->ucMuxID);

    if(uwRet == LOS_OK)
    {
        return osOK;
    }
    else if (uwRet == LOS_ERRNO_MUX_INVALID)
    {
        return osErrorParameter;
    }
    else
    {
        return osErrorResource;
    }
}
#endif

//  ==== Semaphore Management Functions ====
#if (LOSCFG_BASE_IPC_SEM == YES)

osSemaphoreId_t osSemaphoreNew (uint32_t max_count, uint32_t initial_count, const osSemaphoreAttr_t *attr)
{
    UINT32 uwRet;
    UINT32 uwSemId;

    UNUSED(attr);

    if (OS_INT_ACTIVE)
    {
        return (osSemaphoreId_t)NULL;
    }

    if(1 == max_count)
    {
        uwRet = LOS_BinarySemCreate((UINT16)initial_count, &uwSemId);
    }
    else
    {
        uwRet = LOS_SemCreate((UINT16)initial_count, &uwSemId);
    }

    if (uwRet == LOS_OK)
    {
        return (osSemaphoreId_t)(GET_SEM(uwSemId));
    }
    else
    {
        return (osSemaphoreId_t)NULL;
    }
}


osStatus_t osSemaphoreAcquire (osSemaphoreId_t semaphore_id, uint32_t timeout)
{
    UINT32 uwRet;

    if (semaphore_id == NULL)
    {
        return osErrorParameter;
    }

    if (OS_INT_ACTIVE && (timeout != LOS_NO_WAIT))
    {
        return osErrorISR;
    }

    uwRet = LOS_SemPend(((SEM_CB_S *)semaphore_id)->usSemID, timeout);

    if (uwRet == LOS_OK)
    {
        return osOK;
    }
    else if (uwRet == LOS_ERRNO_SEM_TIMEOUT)
    {
        return osErrorTimeout;
    }
    else if (uwRet == LOS_ERRNO_SEM_INVALID)
    {
        return osErrorParameter;
    }
    else if (uwRet == LOS_ERRNO_SEM_PEND_INTERR)
    {
        return osErrorISR;
    }
    else
    {
        return osErrorResource;
    }
}


osStatus_t osSemaphoreRelease (osSemaphoreId_t semaphore_id)
{
    UINT32  uwRet;

    if (semaphore_id == NULL)
    {
        return osErrorParameter;
    }

    uwRet = LOS_SemPost(((SEM_CB_S *)semaphore_id)->usSemID);

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
}


uint32_t osSemaphoreGetCount (osSemaphoreId_t semaphore_id)
{
    UINT32 uwIntSave;
    UINT32 uwCount;

    if (OS_INT_ACTIVE)
    {
        return 0;
    }

    if (semaphore_id == NULL)
    {
        return 0;
    }

    uwIntSave = LOS_IntLock();
    uwCount = ((SEM_CB_S *)semaphore_id)->usSemCount;
    (VOID)LOS_IntRestore(uwIntSave);

    return uwCount;
}


osStatus_t osSemaphoreDelete (osSemaphoreId_t semaphore_id)
{
    UINT32  uwRet;

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    if (semaphore_id == NULL)
    {
        return osErrorParameter;
    }

    uwRet = LOS_SemDelete(((SEM_CB_S *)semaphore_id)->usSemID);

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
}
#endif


//  ==== Message Queue Management Functions ====
#if (LOSCFG_BASE_IPC_QUEUE == YES)
osMessageQueueId_t osMessageQueueNew (uint32_t msg_count, uint32_t msg_size, const osMessageQueueAttr_t *attr)
{
    UINT32 uwQueueID;
    UINT32 uwRet;
    UNUSED(attr);
    osMessageQueueId_t handle;

    if(0 == msg_count || 0 == msg_size || OS_INT_ACTIVE)
    {
        return (osMessageQueueId_t)NULL;
    }

    uwRet = LOS_QueueCreate((char *)NULL, (UINT16)msg_count, &uwQueueID, 0, (UINT16)msg_size);
    if (uwRet == LOS_OK)
    {
        handle = (osMessageQueueId_t)(GET_QUEUE_HANDLE(uwQueueID));
    }
    else
    {
        handle = (osMessageQueueId_t)NULL;
    }

    return handle;
}


osStatus_t osMessageQueuePut (osMessageQueueId_t mq_id, const void *msg_ptr, uint8_t msg_prio, uint32_t timeout)
{
    UNUSED(msg_prio);
    UINT32 uwRet;
    UINT32 uwBufferSize;
    QUEUE_CB_S *pstQueue = (QUEUE_CB_S *)mq_id;

    if (pstQueue == NULL || msg_ptr == NULL || ((OS_INT_ACTIVE) && (0 != timeout)))
    {
        return osErrorParameter;
    }

    uwBufferSize = (UINT32)(pstQueue->usQueueSize - sizeof(UINT32));
    uwRet = LOS_QueueWriteCopy((UINT32)pstQueue->usQueueID, (void *)msg_ptr, uwBufferSize, timeout);
    if (uwRet == LOS_OK)
    {
        return osOK;
    }
    else if(uwRet == LOS_ERRNO_QUEUE_INVALID || uwRet == LOS_ERRNO_QUEUE_NOT_CREATE)
    {
        return osErrorParameter;
    }
    else if (uwRet == LOS_ERRNO_QUEUE_TIMEOUT)
    {
        return osErrorTimeout;
    }
    else
    {
        return osErrorResource;
    }
}


osStatus_t osMessageQueueGet (osMessageQueueId_t mq_id, void *msg_ptr, uint8_t *msg_prio, uint32_t timeout)
{
    UNUSED(msg_prio);
    UINT32 uwRet;
    UINT32 uwBufferSize;
    QUEUE_CB_S *pstQueue = (QUEUE_CB_S *)mq_id;

    if (pstQueue == NULL || msg_ptr == NULL || ((OS_INT_ACTIVE) && (0 != timeout)))
    {
        return osErrorParameter;
    }

    uwBufferSize = (UINT32)(pstQueue->usQueueSize - sizeof(UINT32));
    uwRet = LOS_QueueReadCopy((UINT32)pstQueue->usQueueID, msg_ptr, &uwBufferSize, timeout);
    if (uwRet == LOS_OK)
    {
        return osOK;
    }
    else if (uwRet == LOS_ERRNO_QUEUE_INVALID || uwRet == LOS_ERRNO_QUEUE_NOT_CREATE)
    {
        return osErrorParameter;
    }
    else if (uwRet == LOS_ERRNO_QUEUE_TIMEOUT)
    {
        return osErrorTimeout;
    }
    else
    {
        return osErrorResource;
    }
}

uint32_t osMessageQueueGetCapacity(osMessageQueueId_t mq_id)
{
    uint32_t capacity;
    QUEUE_CB_S *pstQueue = (QUEUE_CB_S *)mq_id;

    if (pstQueue == NULL)
    {
        capacity = 0U;
    }
    else
    {
        capacity = pstQueue->usQueueLen;
    }

    return (capacity);
}

uint32_t osMessageQueueGetMsgSize(osMessageQueueId_t mq_id)
{
    uint32_t size;
    QUEUE_CB_S *pstQueue = (QUEUE_CB_S *)mq_id;

    if (pstQueue == NULL)
    {
        size = 0U;
    }
    else
    {
        size = pstQueue->usQueueSize - sizeof(UINT32);
    }

    return (size);
}


uint32_t osMessageQueueGetCount (osMessageQueueId_t mq_id)
{
    uint32_t count;
    UINTPTR uwIntSave;
    QUEUE_CB_S *pstQueue = (QUEUE_CB_S *)mq_id;

    if (pstQueue == NULL)
    {
        count = 0U;
    }
    else
    {
        uwIntSave = LOS_IntLock();
        count = (uint32_t)(pstQueue->usReadWriteableCnt[OS_QUEUE_READ]);
        LOS_IntRestore(uwIntSave);
    }
    return count;
}


uint32_t osMessageQueueGetSpace (osMessageQueueId_t mq_id)
{
    uint32_t space;
    UINTPTR uwIntSave;
    QUEUE_CB_S *pstQueue = (QUEUE_CB_S *)mq_id;

    if (pstQueue == NULL)
    {
        space = 0U;
    }
    else
    {
        uwIntSave = LOS_IntLock();
        space = (uint32_t)pstQueue->usReadWriteableCnt[OS_QUEUE_WRITE];
        LOS_IntRestore(uwIntSave);
    }
    return space;
}


osStatus_t osMessageQueueDelete (osMessageQueueId_t mq_id)
{
    QUEUE_CB_S *pstQueue = (QUEUE_CB_S *)mq_id;
    UINT32 uwRet;

    if (pstQueue == NULL)
    {
        return osErrorParameter;
    }

    if (OS_INT_ACTIVE)
    {
        return osErrorISR;
    }

    uwRet = LOS_QueueDelete((UINT32)pstQueue->usQueueID);
    if (uwRet == LOS_OK)
    {
        return osOK;
    }
    else if(uwRet == LOS_ERRNO_QUEUE_NOT_FOUND || uwRet == LOS_ERRNO_QUEUE_NOT_CREATE)
    {
        return osErrorParameter;
    }
    else
    {
        return osErrorResource;
    }
}
#endif

#ifdef LOS_RUNSTOP
void osUartVetoCallbackRegister(cb_uart_is_need_awake_fn cb)
{
    if (cb != NULL)
    {
        uart_is_need_awake_callback = (cb_uart_is_need_awake_fn)cb;
    }
    return;
}

void osAddStopClocksVeto(void)
{
    tickless_add_stop_clocks_veto();
}

void osRemoveStopClocksVeto(void)
{
    tickless_remove_stop_clocks_veto();
}
#endif

#endif // (CMSIS_OS_VER == 2)
#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */
