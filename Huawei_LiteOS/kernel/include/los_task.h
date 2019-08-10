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

/**@defgroup los_task Task
 * @ingroup kernel
 */

#ifndef _LOS_TASK_H
#define _LOS_TASK_H

#include "los_base.h"
#include "los_list.h"
#include "los_sys.h"
#include "los_hw.h"
#include "los_tick.h"
#include "los_event.h"
#include "los_err.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/**
 * @ingroup los_task
 * Task error code: Insufficient memory for task creation.
 *
 * Value: 0x03000200
 *
 * Solution: Allocate bigger memory partition to task creation.
 */
#define LOS_ERRNO_TSK_NO_MEMORY                     LOS_ERRNO_OS_FATAL(LOS_MOD_TSK, 0x00)

/**
 * @ingroup los_task
 * Task error code: Null parameter.
 *
 * Value: 0x02000201
 *
 * Solution: Check the parameter.
 */
#define LOS_ERRNO_TSK_PTR_NULL                      LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x01)

/**
 * @ingroup los_task
 * Task error code: The task stack is not aligned.
 *
 * Value: 0x02000202
 *
 * Solution: Align the task stack.
 */
#define LOS_ERRNO_TSK_STKSZ_NOT_ALIGN               LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x02)

/**
 * @ingroup los_task
 * Task error code: Incorrect task priority.
 *
 * Value: 0x02000203
 *
 * Solution: Re-configure the task priority by referring to the priority range.
 */
#define LOS_ERRNO_TSK_PRIOR_ERROR                   LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x03)

/**
 * @ingroup los_task
 * Task error code: The task entrance is NULL.
 *
 * Value: 0x02000204
 *
 * Solution: Define the task entrance function.
 */
#define LOS_ERRNO_TSK_ENTRY_NULL                    LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x04)

/**
 * @ingroup los_task
 * Task error code: The task name is NULL.
 *
 * Value: 0x02000205
 *
 * Solution: Set the task name.
 */
#define LOS_ERRNO_TSK_NAME_EMPTY                    LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x05)

/**
 * @ingroup los_task
 * Task error code: The task stack size is too small.
 *
 * Value: 0x02000206
 *
 * Solution: Expand the task stack.
 */
#define LOS_ERRNO_TSK_STKSZ_TOO_SMALL               LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x06)

/**
 * @ingroup los_task
 * Task error code: Invalid task ID.
 *
 * Value: 0x02000207
 *
 * Solution: Check the task ID.
 */
#define LOS_ERRNO_TSK_ID_INVALID                    LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x07)

/**
 * @ingroup los_task
 * Task error code: The task is already suspended.
 *
 * Value: 0x02000208
 *
 * Solution: Suspend the task after it is resumed.
 */
#define LOS_ERRNO_TSK_ALREADY_SUSPENDED             LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x08)

/**
 * @ingroup los_task
 * Task error code: The task is not suspended.
 *
 * Value: 0x02000209
 *
 * Solution: Suspend the task.
 */
#define LOS_ERRNO_TSK_NOT_SUSPENDED                 LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x09)

/**
 * @ingroup los_task
 * Task error code: The task is not created.
 *
 * Value: 0x0200020a
 *
 * Solution: Create the task.
 */
#define LOS_ERRNO_TSK_NOT_CREATED                   LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x0a)

/**
 * @ingroup los_task
 * Task error code: The task message is nonzero.
 *
 * Value: 0x0200020c
 *
 * Solution: This error code is not in use temporarily.
 */
#define LOS_ERRNO_TSK_MSG_NONZERO                   LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x0c)

/**
 * @ingroup los_task
 * Task error code: The task delay occurs during an interrupt.
 *
 * Value: 0x0300020d
 *
 * Solution: Perform this operation after exiting from the interrupt.
 */
#define LOS_ERRNO_TSK_DELAY_IN_INT                  LOS_ERRNO_OS_FATAL(LOS_MOD_TSK, 0x0d)

/**
 * @ingroup los_task
 * Task error code: The task delay occurs when the task is locked.
 *
 * Value: 0x0200020e
 *
 * Solution: Perform this operation after unlocking the task.
 */
#define LOS_ERRNO_TSK_DELAY_IN_LOCK                 LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x0e)

/**
 * @ingroup los_task
 * Task error code: The task that is being scheduled is invalid.
 *
 * Value: 0x0200020f
 *
 * Solution: Check the task.
 */
#define LOS_ERRNO_TSK_YIELD_INVALID_TASK            LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x0f)

/**
 * @ingroup los_task
 * Task error code: Only one task or no task is available for scheduling.
 *
 * Value: 0x02000210
 *
 * Solution: Increase the number of tasks.
 */
#define LOS_ERRNO_TSK_YIELD_NOT_ENOUGH_TASK         LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x10)

/**
 * @ingroup los_task
 * Task error code: No free task control block is available.
 *
 * Value: 0x02000211
 *
 * Solution: Increase the number of task control blocks.
 */
#define LOS_ERRNO_TSK_TCB_UNAVAILABLE               LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x11)

/**
 * @ingroup los_task
 * Task error code: The task hook function is not matchable.
 *
 * Value: 0x02000212
 *
 * Solution: This error code is not in use temporarily.
 */
#define LOS_ERRNO_TSK_HOOK_NOT_MATCH                LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x12)

/**
 * @ingroup los_task
 * Task error code: The number of task hook functions exceeds the permitted upper limit.
 *
 * Value: 0x02000213
 *
 * Solution: This error code is not in use temporarily.
 */
#define LOS_ERRNO_TSK_HOOK_IS_FULL                  LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x13)

/**
 * @ingroup los_task
 * Task error code: The operation is performed on the idle task.
 *
 * Value: 0x02000214
 *
 * Solution: Check the task ID and do not operate on the idle task.
 */
#define LOS_ERRNO_TSK_OPERATE_IDLE                  LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x14)

/**
 * @ingroup los_task
 * Task error code: The task that is being suspended is locked.
 *
 * Value: 0x03000215
 *
 * Solution: Suspend the task after unlocking the task.
 */
#define LOS_ERRNO_TSK_SUSPEND_LOCKED                LOS_ERRNO_OS_FATAL(LOS_MOD_TSK, 0x15)

/**
 * @ingroup los_task
 * Task error code: The task stack fails to be freed.
 *
 * Value: 0x02000217
 *
 * Solution: This error code is not in use temporarily.
 */
#define LOS_ERRNO_TSK_FREE_STACK_FAILED             LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x17)

/**
 * @ingroup los_task
 * Task error code: The task stack area is too small.
 *
 * Value: 0x02000218
 *
 * Solution: This error code is not in use temporarily.
 */
#define LOS_ERRNO_TSK_STKAREA_TOO_SMALL             LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x18)

/**
 * @ingroup los_task
 * Task error code: The task fails to be activated.
 *
 * Value: 0x03000219
 *
 * Solution: Perform task switching after creating an idle task.
 */
#define LOS_ERRNO_TSK_ACTIVE_FAILED                 LOS_ERRNO_OS_FATAL(LOS_MOD_TSK, 0x19)

/**
 * @ingroup los_task
 * Task error code: Too many task configuration items.
 *
 * Value: 0x0200021a
 *
 * Solution: This error code is not in use temporarily.
 */
#define LOS_ERRNO_TSK_CONFIG_TOO_MANY               LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x1a)

/**
 * @ingroup los_task
 * Task error code:
 *
 * Value: 0x0200021b
 *
 * Solution: This error code is not in use temporarily.
 */
#define LOS_ERRNO_TSK_CP_SAVE_AREA_NOT_ALIGN        LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x1b)

/**
 * @ingroup los_task
 * Task error code:
 *
 * Value: 0x0200021d
 *
 * Solution: This error code is not in use temporarily.
 */
#define LOS_ERRNO_TSK_MSG_Q_TOO_MANY                LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x1d)

/**
 * @ingroup los_task
 * Task error code:
 *
 * Value: 0x0200021e
 *
 * Solution: This error code is not in use temporarily.
 */
#define LOS_ERRNO_TSK_CP_SAVE_AREA_NULL             LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x1e)

/**
 * @ingroup los_task
 * Task error code:
 *
 * Value: 0x0200021f
 *
 * Solution: This error code is not in use temporarily.
 */
#define LOS_ERRNO_TSK_SELF_DELETE_ERR               LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x1f)

/**
 * @ingroup los_task
 * Task error code: The task stack size is too large.
 *
 * Value: 0x02000220
 *
 * Solution: shrink the task stack size parameter.
 */
#define LOS_ERRNO_TSK_STKSZ_TOO_LARGE               LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x20)

/**
 * @ingroup los_task
 * Task error code: Suspending software timer task is not allowed.
 *
 * Value: 0x02000221
 *
 * Solution: Check the task ID and do not suspend software timer task.
 */
#define LOS_ERRNO_TSK_SUSPEND_SWTMR_NOT_ALLOWED                LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x21)

/**
 * @ingroup los_task
 * Task error code: The operation is performed on the software timer task.
 *
 * Value: 0x02000222
 *
 * Solution: Check the task ID and do not operate on the software timer task.
 */
#define LOS_ERRNO_TSK_OPERATE_SWTMR                 LOS_ERRNO_OS_ERROR(LOS_MOD_TSK, 0x22)

/**
 * @ingroup los_task
 * Define the type of the task switching hook function.
 *
 */
typedef VOID (*TSKSWITCHHOOK)(VOID);

/**
 * @ingroup los_task
 * User task switching hook function.
 *
 */
extern TSKSWITCHHOOK g_pfnUsrTskSwitchHook;

/**
* @ingroup  los_task
* @brief Define the type of a task entrance function.
*
* @par Description:
* This API is used to define the type of a task entrance function and call it after a task is created and triggered.
* @attention None.
*
* @param  uwParam1 [IN] Type #UINT32 The first parameter passed to the task handling function.
* @param  uwParam2 [IN] Type #UINT32 The second parameter passed to the task handling function.
* @param  uwParam3 [IN] Type #UINT32 The third parameter passed to the task handling function.
* @param  uwParam4 [IN] Type #UINT32 The fourth parameter passed to the task handling function.
*
* @retval None.
* @par Dependency:
* <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
* @see
* @since Huawei LiteOS V100R001C00
*/
typedef VOID *(*TSK_ENTRY_FUNC)(UINT32 uwArg);

/**
 * @ingroup los_task
 * Define the structure of the parameters used for task creation.
 *
 * Information of specified parameters passed in during task creation.
 */
typedef struct tagTskInitParam
{
   TSK_ENTRY_FUNC       pfnTaskEntry;               /**< Task entrance function                 */
   UINT16               usTaskPrio;                 /**< Task priority                          */
   UINT32               uwArg;                      /**< Task parameters                        */
   UINT32               uwStackSize;                /**< Task stack size                        */
   CHAR                 *pcName;                    /**< Task name                              */
   UINT32               uwResved;                   /**< Reserved                               */
} TSK_INIT_PARAM_S;

/**
 * @ingroup los_task
 * Task name length
 *
 */
#define LOS_TASK_NAMELEN                            32

/**
 * @ingroup los_task
 * Task information structure.
 *
 */
typedef struct tagTskInfo
{
    CHAR                acName[LOS_TASK_NAMELEN];   /**< Task entrance function         */
    UINT32              uwTaskID;                   /**< Task ID                        */
    UINT16              usTaskStatus;               /**< Task status                    */
    UINT16              usTaskPrio;                 /**< Task priority                  */
    VOID                *pTaskSem;                  /**< Semaphore pointer              */
    VOID                *pTaskMux;                  /**< Mutex pointer                  */
    UINT32              uwSemID;                    /**< Sem ID                         */
    UINT32              uwMuxID;                    /**< Mux ID                         */
    EVENT_CB_S          uwEvent;                    /**< Event                          */
    UINT32              uwEventMask;                /**< Event mask                     */
    UINT32              uwStackSize;                /**< Task stack size                */
    UINT32              uwTopOfStack;               /**< Task stack top                 */
    UINT32              uwBottomOfStack;            /**< Task stack bottom              */
    UINT32              uwSP;                       /**< Task SP pointer                */
    UINT32              uwCurrUsed;                 /**< Current task stack usage       */
    UINT32              uwPeakUsed;                 /**< Task stack usage peak          */
    BOOL                bOvf;                       /**< Flag that indicates whether a task stack overflow occurs         */
} TSK_INFO_S;

/**
 * @ingroup los_task
 * Task switch information structure.
 *
 */
#define OS_TASK_SWITCH_INFO_COUNT                           (0xA)
typedef struct tagTaskSwitchInfo
{
    UINT8 ucIdx;
    UINT8 ucIsFull; /* bit [7] store isfull status, bits [6:0] store count of task switch info */
    UINT16 auwPID[OS_TASK_SWITCH_INFO_COUNT];
    CHAR   acName[OS_TASK_SWITCH_INFO_COUNT][LOS_TASK_NAMELEN];
}OS_TASK_SWITCH_INFO;


/**
 * @ingroup  los_task
 * @brief Create a task and suspend.
 *
 * @par Description:
 * This API is used to create a task and suspend it. This task will not be added to the queue of ready tasks before resume it.
 *
 * @attention
 * <ul>
 * <li>During task creation, the task control block and task stack of the task that is previously automatically deleted are deallocated.</li>
 * <li>The task name is a pointer and is not allocated memory.</li>
 * <li>If the size of the task stack of the task to be created is 0, configure #LOSCFG_BASE_CORE_TSK_DEFAULT_STACK_SIZE to specify the default task stack size. The stack size should be a reasonable value, if the size is too large, may cause memory exhaustion.</li>
 * <li>The task stack size must be aligned on the boundary of 8 bytes. The size is determined by whether it is big enough to avoid task stack overflow.</li>
 * <li>Less parameter value indicates higher task priority.</li>
 * <li>The task name cannot be null.</li>
 * <li>The pointer to the task executing function cannot be null.</li>
 * <li>The two parameters of this interface is pointer, it should be a correct value, otherwise, the system may be abnormal.</li>
 * </ul>
 *
 * @param  puwTaskID    [OUT] Type  #UINT32 * Task ID.
 * @param  pstInitParam [IN]  Type  #TSK_INIT_PARAM_S * Parameter for task creation.
 *
 * @retval #LOS_ERRNO_TSK_ID_INVALID        Invalid Task ID, param puwTaskID is NULL.
 * @retval #LOS_ERRNO_TSK_PTR_NULL          Param pstInitParam is NULL.
 * @retval #LOS_ERRNO_TSK_NAME_EMPTY        The task name is NULL.
 * @retval #LOS_ERRNO_TSK_ENTRY_NULL        The task entrance is NULL.
 * @retval #LOS_ERRNO_TSK_PRIOR_ERROR       Incorrect task priority.
 * @retval #LOS_ERRNO_TSK_STKSZ_TOO_LARGE   The task stack size is too large.
 * @retval #LOS_ERRNO_TSK_STKSZ_TOO_SMALL   The task stack size is too small.
 * @retval #LOS_ERRNO_TSK_TCB_UNAVAILABLE   No free task control block is available.
 * @retval #LOS_ERRNO_TSK_NO_MEMORY         Insufficient memory for task creation.
 * @retval #LOS_OK                          The task is successfully created.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * <ul><li>los_config.h: the header file that contains system configuration items.</li></ul>
 * @see LOS_TaskDelete
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_TaskCreateOnly(UINT32 *puwTaskID, TSK_INIT_PARAM_S *pstInitParam);

/**
 * @ingroup  los_task
 * @brief Create a task.
 *
 * @par Description:
 * This API is used to create a task. If the priority of the task created after system initialized is higher than the current task and task scheduling is not locked, it is scheduled for running.
 * If not, the created task is added to the queue of ready tasks.
 *
 * @attention
 * <ul>
 * <li>During task creation, the task control block and task stack of the task that is previously automatically deleted are deallocated.</li>
 * <li>The task name is a pointer and is not allocated memory.</li>
 * <li>If the size of the task stack of the task to be created is 0, configure #LOSCFG_BASE_CORE_TSK_DEFAULT_STACK_SIZE to specify the default task stack size.</li>
 * <li>The task stack size must be aligned on the boundary of 8 bytes. The size is determined by whether it is big enough to avoid task stack overflow.</li>
 * <li>Less parameter value indicates higher task priority.</li>
 * <li>The task name cannot be null.</li>
 * <li>The pointer to the task executing function cannot be null.</li>
 * <li>The two parameters of this interface is pointer, it should be a correct value, otherwise, the system may be abnormal.</li>
 * </ul>
 *
 * @param  puwTaskID    [OUT] Type  #UINT32 * Task ID.
 * @param  pstInitParam [IN]  Type  #TSK_INIT_PARAM_S * Parameter for task creation.
 *
 * @retval #LOS_ERRNO_TSK_ID_INVALID        Invalid Task ID, param puwTaskID is NULL.
 * @retval #LOS_ERRNO_TSK_PTR_NULL          Param pstInitParam is NULL.
 * @retval #LOS_ERRNO_TSK_NAME_EMPTY        The task name is NULL.
 * @retval #LOS_ERRNO_TSK_ENTRY_NULL        The task entrance is NULL.
 * @retval #LOS_ERRNO_TSK_PRIOR_ERROR       Incorrect task priority.
 * @retval #LOS_ERRNO_TSK_STKSZ_TOO_LARGE   The task stack size is too large.
 * @retval #LOS_ERRNO_TSK_STKSZ_TOO_SMALL   The task stack size is too small.
 * @retval #LOS_ERRNO_TSK_TCB_UNAVAILABLE   No free task control block is available.
 * @retval #LOS_ERRNO_TSK_NO_MEMORY         Insufficient memory for task creation.
 * @retval #LOS_OK                          The task is successfully created.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * <ul><li>los_config.h: the header file that contains system configuration items.</li></ul>
 * @see LOS_TaskDelete
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_TaskCreate(UINT32 *puwTaskID, TSK_INIT_PARAM_S *pstInitParam);

/**
 * @ingroup  los_task
 * @brief Resume a task.
 *
 * @par Description:
 * This API is used to resume a suspended task.
 *
 * @attention
 * <ul>
 * <li>If the task is delayed or blocked, resume the task without adding it to the queue of ready tasks.</li>
 * <li>If the priority of the task resumed after system initialized is higher than the current task and task scheduling is not locked, it is scheduled for running.</li>
 * </ul>
 *
 * @param  uwTaskID [IN] Type #UINT32 Task ID. The task id value is obtained from task creation.
 *
 * @retval #LOS_ERRNO_TSK_ID_INVALID        Invalid Task ID
 * @retval #LOS_ERRNO_TSK_NOT_CREATED       The task is not created.
 * @retval #LOS_ERRNO_TSK_NOT_SUSPENDED     The task is not suspended.
 * @retval #LOS_OK                          The task is successfully resumed.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see LOS_TaskSuspend
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_TaskResume(UINT32 uwTaskID);

/**
 * @ingroup  los_task
 * @brief Suspend a task.
 *
 * @par Description:
 * This API is used to suspend a specified task, and the task will be removed from the queue of ready tasks.
 *
 * @attention
 * <ul>
 * <li>The task that is running and locked cannot be suspended.</li>
 * <li>The idle task and swtmr task cannot be suspended.</li>
 * </ul>
 *
 * @param  uwTaskID [IN] Type #UINT32 Task ID. The task id value is obtained from task creation.
 *
 * @retval #LOS_ERRNO_TSK_OPERATE_IDLE                  Check the task ID and do not operate on the idle task.
 * @retval #LOS_ERRNO_TSK_SUSPEND_SWTMR_NOT_ALLOWED     Check the task ID and do not operate on the swtmr task.
 * @retval #LOS_ERRNO_TSK_ID_INVALID                    Invalid Task ID
 * @retval #LOS_ERRNO_TSK_NOT_CREATED                   The task is not created.
 * @retval #LOS_ERRNO_TSK_ALREADY_SUSPENDED             The task is already suspended.
 * @retval #LOS_ERRNO_TSK_SUSPEND_LOCKED                The task being suspended is current task and task scheduling is locked.
 * @retval #LOS_OK                                      The task is successfully suspended.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see LOS_TaskResume
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_TaskSuspend(UINT32 uwTaskID);

/**
 * @ingroup  los_task
 * @brief Delete a task.
 *
 * @par Description:
 * This API is used to delete a specified task and release the resources for its task stack and task control block.
 *
 * @attention
 * <ul>
 * <li>The idle task and swtmr task cannot be deleted.</li>
 * <li>If delete current task maybe cause unexpected error.</li>
 * <li>If a task get a mutex is deleted or automatically deleted before release this mutex, other tasks pended this mutex maybe never be shchduled.</li>
 * </ul>
 *
 * @param  uwTaskID [IN] Type #UINT32 Task ID. The task id value is obtained from task creation.
 *
 * @retval #LOS_ERRNO_TSK_OPERATE_IDLE                  Check the task ID and do not operate on the idle task.
 * @retval #LOS_ERRNO_TSK_SUSPEND_SWTMR_NOT_ALLOWED     Check the task ID and do not operate on the swtmr task.
 * @retval #LOS_ERRNO_TSK_ID_INVALID                    Invalid Task ID
 * @retval #LOS_ERRNO_TSK_NOT_CREATED                   The task is not created.
 * @retval #LOS_OK                                      The task is successfully deleted.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see LOS_TaskCreate | LOS_TaskCreateOnly
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_TaskDelete(UINT32 uwTaskID);

/**
 * @ingroup  los_task
 * @brief Delay a task.
 *
 * @par Description:
 * This API is used to delay the execution of the current task. The task is able to be scheduled after it is delayed for a specified number of Ticks.
 *
 * @attention
 * <ul>
 * <li>The task fails to be delayed if it is being delayed during interrupt processing or it is locked.</li>
 * <li>If 0 is passed in and the task scheduling is not locked, execute the next task in the queue of tasks with the same priority of the current task.
 * If no ready task with the priority of the current task is available, the task scheduling will not occur, and the current task continues to be executed.</li>
 * <li>Using the interface before system initialized is not allowed.</li>
 * </ul>
 *
 * @param  uwTick [IN] Type #UINT32 Number of Ticks for which the task is delayed.
 *
 * @retval #LOS_ERRNO_TSK_DELAY_IN_INT              The task delay occurs during an interrupt.
 * @retval #LOS_ERRNO_TSK_DELAY_IN_LOCK             The task delay occurs when the task scheduling is locked.
 * @retval #LOS_ERRNO_TSK_ID_INVALID                Invalid Task ID
 * @retval #LOS_ERRNO_TSK_YIELD_NOT_ENOUGH_TASK     No tasks with the same priority is available for scheduling.
 * @retval #LOS_OK                                  The task is successfully delayed.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_TaskDelay(UINT32 uwTick);

/**
 * @ingroup  los_task
 * @brief Lock the task scheduling.
 *
 * @par Description:
 * This API is used to lock the task scheduling. Task switching will not occur if the task scheduling is locked.
 *
 * @attention
 * <ul>
 * <li>If the task scheduling is locked, but interrupts are not disabled, tasks are still able to be interrupted.</li>
 * <li>One is added to the number of task scheduling locks if this API is called. The number of locks is decreased by one if the task scheduling is unlocked. Therefore, this API should be used together with LOS_TaskUnlock.</li>
 * </ul>
 *
 * @param  None.
 *
 * @retval None.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see LOS_TaskUnlock
 * @since Huawei LiteOS V100R001C00
 */
extern VOID LOS_TaskLock(VOID);

/**
 * @ingroup  los_task
 * @brief Unlock the task scheduling.
 *
 * @par Description:
 * This API is used to unlock the task scheduling. Calling this API will decrease the number of task locks by one. If a task is locked more than once, the task scheduling will be unlocked only when the number of locks becomes zero.
 *
 * @attention
 * <ul>
 * <li>The number of locks is decreased by one if this API is called. One is added to the number of task scheduling locks if the task scheduling is locked. Therefore, this API should be used together with LOS_TaskLock.</li>
 * </ul>
 *
 * @param  None.
 *
 * @retval None.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see LOS_TaskLock
 * @since Huawei LiteOS V100R001C00
 */
extern VOID LOS_TaskUnlock(VOID);

/**
 * @ingroup  los_task
 * @brief Set a task priority.
 *
 * @par Description:
 * This API is used to set the priority of a specified task.
 *
 * @attention
 * <ul>
 * <li>If the set priority is higher than the priority of the current running task, task scheduling probably occurs.</li>
 * <li>Changing the priority of the current running task also probably causes task scheduling.</li>
 * <li>Using the interface to change the priority of software timer task and idle task is not allowed.</li>
 * <li>Using the interface in the interrupt is not allowed.</li>
 * </ul>
 *
 * @param  uwTaskID   [IN] Type #UINT32 Task ID. The task id value is obtained from task creation.
 * @param  usTaskPrio [IN] Type #UINT16 Task priority.
 *
 * @retval #LOS_ERRNO_TSK_PRIOR_ERROR    Incorrect task priority.Re-configure the task priority
 * @retval #LOS_ERRNO_TSK_OPERATE_IDLE   Check the task ID and do not operate on the idle task.
 * @retval #LOS_ERRNO_TSK_ID_INVALID     Invalid Task ID
 * @retval #LOS_ERRNO_TSK_NOT_CREATED    The task is not created.
 * @retval #LOS_OK                       The task priority is successfully set to a specified priority.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see LOS_TaskPriSet
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_TaskPriSet(UINT32 uwTaskID, UINT16 usTaskPrio);

/**
 * @ingroup  los_task
 * @brief Set the priority of the current running task to a specified priority.
 *
 * @par Description:
 * This API is used to set the priority of the current running task to a specified priority.
 *
 * @attention
 * <ul>
 * <li>Changing the priority of the current running task probably causes task scheduling.</li>
 * <li>Using the interface to change the priority of software timer task and idle task is not allowed.</li>
 * <li>Using the interface in the interrupt is not allowed.</li>
 * </ul>
 *
 * @param  usTaskPrio [IN] Type #UINT16 Task priority.
 *
 * @retval #LOS_ERRNO_TSK_PRIOR_ERROR     Incorrect task priority.Re-configure the task priority
 * @retval #LOS_ERRNO_TSK_OPERATE_IDLE    Check the task ID and do not operate on the idle task.
 * @retval #LOS_ERRNO_TSK_ID_INVALID      Invalid Task ID
 * @retval #LOS_ERRNO_TSK_NOT_CREATED     The task is not created.
 * @retval #LOS_OK                        The priority of the current running task is successfully set to a specified priority.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see LOS_TaskPriGet
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_CurTaskPriSet(UINT16 usTaskPrio);

/**
 * @ingroup  los_task
 * @brief Change the scheduling sequence of tasks with the same priority.
 *
 * @par Description:
 * This API is used to move current task in a queue of tasks with the same priority to the tail of the queue of ready tasks.
 *
 * @attention
 * <ul>
 * <li>At least two ready tasks need to be included in the queue of ready tasks with the same priority. If the less than two ready tasks are included in the queue, an error is reported.</li>
 * </ul>
 *
 * @param  None.
 *
 * @retval #LOS_ERRNO_TSK_ID_INVALID                    Invalid Task ID
 * @retval #LOS_ERRNO_TSK_YIELD_NOT_ENOUGH_TASK         No tasks with the same priority is available for scheduling.
 * @retval #LOS_OK                                      The scheduling sequence of tasks with same priority is successfully changed.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_TaskYield(VOID);

/**
 * @ingroup  los_task
 * @brief Obtain a task priority.
 *
 * @par Description:
 * This API is used to obtain the priority of a specified task.
 *
 * @attention None.
 *
 * @param  uwTaskID [IN] Type #UINT32 Task ID. The task id value is obtained from task creation.
 *
 * @retval #OS_INVALID      The task priority fails to be obtained.
 * @retval #UINT16          The task priority.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see LOS_TaskPriSet
 * @since Huawei LiteOS V100R001C00
 */
extern UINT16 LOS_TaskPriGet(UINT32 uwTaskID);

/**
 * @ingroup  los_task
 * @brief Obtain current running task ID.
 *
 * @par Description:
 * This API is used to obtain the ID of current running task.
 *
 * @attention
 * <ul>
 * <li> This interface should not be called before system initialized.</li>
 * </ul>
 *
 * @retval #LOS_ERRNO_TSK_ID_INVALID    Invalid Task ID.
 * @retval #UINT32                      Task ID.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_CurTaskIDGet(VOID);

/**
 * @ingroup  los_task
 * @brief Obtain next running task ID.
 *
 * @par Description:
 * This API is used to obtain the ID of next running task.
 *
 * @attention None.
 *
 *
 * @retval #LOS_ERRNO_TSK_ID_INVALID    invalid Task ID.
 * @retval #UINT32                      task id.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_NextTaskIDGet(VOID);

/**
 * @ingroup  los_task
 * @brief Obtain next running task ID.
 *
 * @par Description:
 * This API is used to obtain the ID of next running task.
 *
 * @attention None.
 *
 *
 * @retval #NULL            invalid Task name.
 * @retval #CHAR*           task name.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
extern CHAR *LOS_CurTaskNameGet(VOID);

/**
 * @ingroup  los_task
 * @brief Obtain a task information structure.
 *
 * @par Description:
 * This API is used to obtain a task information structure.
 *
 * @attention
 * <ul>
 * <li>One parameter of this interface is a pointer, it should be a correct value, otherwise, the system may be abnormal.</li>
 * </ul>
 *
 * @param  uwTaskID    [IN]  Type  #UINT32 Task ID. The task id value is obtained from task creation.
 * @param  pstTaskInfo [OUT] Type  #TSK_INFO_S* Pointer to the task information structure to be obtained.
 *
 * @retval #LOS_ERRNO_TSK_PTR_NULL        Null parameter.
 * @retval #LOS_ERRNO_TSK_ID_INVALID      Invalid task ID.
 * @retval #LOS_ERRNO_TSK_NOT_CREATED     The task is not created.
 * @retval #LOS_OK                        The task information structure is successfully obtained.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_TaskInfoGet(UINT32 uwTaskID, TSK_INFO_S *pstTaskInfo);

/**
 * @ingroup  los_task
 * @brief Obtain the task status.
 *
 * @par Description:
 * This API is used to obtain the task status.
 *
 * @attention None.
 *
 * @param  uwTaskID   [IN] Type  #TSK_HANDLE_T Task ID.
 * @param  puwTaskStatus [OUT] Type  #UINT32 Pointer to the task status to be obtained.
 *
 * @retval #LOS_ERRNO_TSK_PTR_NULL                    0x02000201: Null parameter.
 * @retval #LOS_ERRNO_TSK_ID_INVALID                  0x02000207: Invalid task ID.
 * @retval #LOS_ERRNO_TSK_NOT_CREATED                 0x0200020a: The task is not created.
 * @retval #LOS_OK                                   0: The task information structure is successfully obtained.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_TaskStatusGet(UINT32 uwTaskID, UINT32* puwTaskStatus);

/**
 *@ingroup los_monitor
 *@brief Obtain all tasks info.
 *
 *@par Description:
 *This API is used to obtain all tasks info.
 *@attention
 *<ul>
 *<li>This API can be called only after the CPU usage is initialized. Otherwise, -1 will be returned.</li>
 *</ul>
 *
 *@param None.
 *
 *@retval #OS_ERROR           -1:all tasks info obtain failed.
 *@retval #LOS_OK              0:all tasks info is successfully obtained.
 *@par Dependency:
 *<ul><li>los_monitor.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_TaskInfoMonitor
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_TaskInfoMonitor(VOID);

/**
 * @ingroup  los_task
 * @brief Obtain tasks switch info.
 *
 * @par Description:
 * This API is used to obtain tasks switch info.
 *
 * @attention None.
 *
 * @param  uwIdx            [IN]  Type  #UINT32  Switch info array index.
 * @param  pTaskSwitchInfo  [OUT] Type  #UINT32* First 4 bytes is task id, and then is task name, name len is OS_TSK_NAME_LEN.
 *
 * @retval #LOS_ERRNO_TSK_PTR_NULL           0x02000201: Null parameter.
 * @retval #LOS_OK                           0: The task switch information is successfully obtained.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_TaskSwitchInfoGet(UINT32 uwIdx, UINT32 *pTaskSwitchInfo);

/**
 * @ingroup  los_task
 * @brief Obtain tasks schduling info.
 *
 * @par Description:
 * This API is used to obtain task is scheduled.
 *
 * @attention None.
 *
 * @param None.
 *
 * @retval                          1: Tasks is scheduled.
 * @retval                          0: Tasks not scheduling yet.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
extern BOOL LOS_TaskIsRunning(VOID);

/**
 * @ingroup  los_task
 * @brief Obtain current new task ID.
 *
 * @par Description:
 * This API is used to obtain the ID of new  task.
 *
 * @attention None.
 *
 *
 * @retval #LOS_ERRNO_TSK_ID_INVALID    0x02000207: invalid Task ID.
 * @retval # Task ID.
 * @par Dependency:
 * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
 extern  UINT32 LOS_NewTaskIDGet(VOID);
 /**
  * @ingroup  los_task
  * @brief Obtain current new task name.
  *
  * @par Description:
  * This API is used to obtain the name of new  task.
  *
  * @attention None.
  *
  *
  * @retval #NULL: invalid Task name.
  * @retval # Task name.
  * @par Dependency:
  * <ul><li>los_task.h: the header file that contains the API declaration.</li></ul>
  * @see
  * @since Huawei LiteOS V100R001C00
  */
 extern CHAR* LOS_TaskNameGet(UINT32 uwTaskID);

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_TASK_H */
