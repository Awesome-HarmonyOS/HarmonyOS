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

#ifndef _LOS_TASK_PH
#define _LOS_TASK_PH

#include "los_task.h"

#if (LOSCFG_LIB_LIBC_NEWLIB_REENT == YES)
#include <reent.h>
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


/**
 * @ingroup los_task
 * Null task ID
 *
 */
#define OS_TASK_ERRORID                             0xFFFFFFFF

/**
 * @ingroup los_task
 * Define a usable task priority.
 *
 * Highest task priority.
 */
#define OS_TASK_PRIORITY_HIGHEST                    0

/**
 * @ingroup los_task
 * Define a usable task priority.
 *
 * Lowest task priority.
 */
#define OS_TASK_PRIORITY_LOWEST                     31

/**
 * @ingroup los_task
 * Flag that indicates the task or task control block status.
 *
 * The task control block is unused.
 */
#define OS_TASK_STATUS_UNUSED                       0x0001

/**
 * @ingroup los_task
 * Flag that indicates the task or task control block status.
 *
 * The task is suspended.
 */
#define OS_TASK_STATUS_SUSPEND                      0x0002

/**
 * @ingroup los_task
 * Flag that indicates the task or task control block status.
 *
 * The task is ready.
 */
#define OS_TASK_STATUS_READY                        0x0004

/**
 * @ingroup los_task
 * Flag that indicates the task or task control block status.
 *
 * The task is blocked.
 */
#define OS_TASK_STATUS_PEND                         0x0008

/**
 * @ingroup los_task
 * Flag that indicates the task or task control block status.
 *
 * The task is running.
 */
#define OS_TASK_STATUS_RUNNING                      0x0010

/**
 * @ingroup los_task
 * Flag that indicates the task or task control block status.
 *
 * The task is delayed.
 */
#define OS_TASK_STATUS_DELAY                        0x0020

/**
 * @ingroup los_task
 * Flag that indicates the task or task control block status.
 *
 * The time for waiting for an event to occur expires.
 */
#define OS_TASK_STATUS_TIMEOUT                      0x0040

/**
 * @ingroup los_task
 * Flag that indicates the task or task control block status.
 *
 * The task is waiting for an event to occur.
 */
#define OS_TASK_STATUS_EVENT                        0x0400

/**
 * @ingroup los_task
 * Flag that indicates the task or task control block status.
 *
 * The task is reading an event.
 */
#define OS_TASK_STATUS_EVENT_READ                   0x0800

/**
 * @ingroup los_task
 * Flag that indicates the task or task control block status.
 *
 * A software timer is waiting for an event to occur.
 */
#define OS_TASK_STATUS_SWTMR_WAIT                   0x1000

/**
 * @ingroup los_task
 * Flag that indicates the task or task control block status.
 *
 * The task is blocked on a queue.
 */
#define OS_TASK_STATUS_PEND_QUEUE                   0x2000

/**
 * @ingroup los_task
 * Flag that indicates the task or task control block status.
 *
 * The task is blocked on a mutex.
 */
#define OS_TASK_STATUS_PEND_MUT                     0x4000

/**
 * @ingroup los_task
 * Flag that indicates the task or task control block status.
 *
 * The task is blocked on a semaphore.
 */
#define OS_TASK_STATUS_PEND_SEM                     0x8000

/**
 * @ingroup los_task
 * Boundary on which the stack size is aligned.
 *
 */
#define OS_TASK_STACK_SIZE_ALIGN                    16

/**
 * @ingroup los_task
 * Boundary on which the stack address is aligned.
 *
 */
#define OS_TASK_STACK_ADDR_ALIGN                    8

/**
 * @ingroup los_task
 * Task stack top magic number.
 *
 */
#define OS_TASK_MAGIC_WORD                          0xCCCCCCCC

/**
 * @ingroup los_task
 * Initial task stack value.
 *
 */
#define OS_TASK_STACK_INIT                          0xCACACACA

/**
 * @ingroup los_task
 * Number of usable task priorities.
 */
#define OS_TSK_PRINUM                               (OS_TASK_PRIORITY_LOWEST - OS_TASK_PRIORITY_HIGHEST + 1)

/**
* @ingroup  los_task
* @brief Check whether a task ID is valid.
*
* @par Description:
* This API is used to check whether a task ID, excluding the idle task ID, is valid.
* @attention None.
*
* @param  uwTaskID [IN] Task ID.
*
* @retval 0 or 1. One indicates that the task ID is invalid, whereas zero indicates that the task ID is valid.
* @par Dependency:
* <ul><li>los_task.ph: the header file that contains the API declaration.</li></ul>
* @see
* @since Huawei LiteOS V100R001C00
*/
#define OS_TSK_GET_INDEX(uwTaskID)                      (uwTaskID)

/**
* @ingroup  los_task
* @brief Obtain the pointer to a task control block.
*
* @par Description:
* This API is used to obtain the pointer to a task control block using a corresponding parameter.
* @attention None.
*
* @param  ptr [IN] Parameter used for obtaining the task control block.
*
* @retval Pointer to the task control block.
* @par Dependency:
* <ul><li>los_task.ph: the header file that contains the API declaration.</li></ul>
* @see
* @since Huawei LiteOS V100R001C00
*/
#define OS_TCB_FROM_PENDLIST(ptr)                       LOS_DL_LIST_ENTRY(ptr, LOS_TASK_CB, stPendList)

/**
* @ingroup  los_task
* @brief Obtain the pointer to a task control block.
*
* @par Description:
* This API is used to obtain the pointer to a task control block that has a specified task ID.
* @attention None.
*
* @param  TaskID [IN] Task ID.
*
* @retval Pointer to the task control block.
* @par Dependency:
* <ul><li>los_task.ph: the header file that contains the API declaration.</li></ul>
* @see
* @since Huawei LiteOS V100R001C00
*/
#define OS_TCB_FROM_TID(TaskID)                       (((LOS_TASK_CB *)g_pstTaskCBArray) + (TaskID))
#define OS_IDLE_TASK_ENTRY                            ((TSK_ENTRY_FUNC)osIdleTask)

/**
 * @ingroup los_task
 * Define the task control block structure.
 */
typedef struct tagTaskCB
{
    VOID                        *pStackPointer;             /**< Task stack pointer          */
    UINT16                      usTaskStatus;
    UINT16                      usPriority;
    UINT32                      uwStackSize;                /**< Task stack size             */
    UINT32                      uwTopOfStack;               /**< Task stack top              */
    UINT32                      uwTaskID;                   /**< Task ID                     */
    TSK_ENTRY_FUNC              pfnTaskEntry;               /**< Task entrance function      */
    VOID                        *pTaskSem;                  /**< Task-held semaphore         */
    VOID                        *pTaskMux;                  /**< Task-held mutex             */
    UINT32                      uwArg;                      /**< Parameter                   */
    CHAR                        *pcTaskName;                /**< Task name                   */
    LOS_DL_LIST                 stPendList;
    LOS_DL_LIST                 stTimerList;
    UINT32                      uwIdxRollNum;
    EVENT_CB_S                  uwEvent;
    UINT32                      uwEventMask;                /**< Event mask                  */
    UINT32                      uwEventMode;                /**< Event mode                  */
    VOID                        *puwMsg;                    /**< Memory allocated to queues  */
#if (LOSCFG_LIB_LIBC_NEWLIB_REENT == YES)
    struct _reent stNewLibReent;                            /**< NewLib _reent struct        */
#endif
} LOS_TASK_CB;

typedef struct stLosTask
{
    LOS_TASK_CB   *pstRunTask;
    LOS_TASK_CB   *pstNewTask;
} ST_LOS_TASK;

extern ST_LOS_TASK          g_stLosTask;

/**
 * @ingroup los_task
 * Task lock flag.
 *
 */
extern UINT16               g_usLosTaskLock;

/**
 * @ingroup los_task
 * Maximum number of tasks.
 *
 */
extern UINT32               g_uwTskMaxNum;

/**
 * @ingroup los_task
 * Idle task ID.
 *
 */
extern UINT32               g_uwIdleTaskID;

/**
 * @ingroup los_task
 * Software timer task ID.
 *
 */
extern UINT32               g_uwSwtmrTaskID;

/**
 * @ingroup los_task
 * Starting address of a task.
 *
 */
extern LOS_TASK_CB          *g_pstTaskCBArray;

/**
 * @ingroup los_task
 * Delayed task linked list.
 *
 */
extern LOS_DL_LIST          g_stTaskTimerList;

/**
 * @ingroup los_task
 * Free task linked list.
 *
 */
extern LOS_DL_LIST          g_stLosFreeTask;

/**
 * @ingroup los_task
 * Circular linked list that stores tasks that are deleted automatically.
 *
 */
extern LOS_DL_LIST          g_stTskRecyleList;

/**
 * @ingroup los_task
 * Time slice structure.
 */
typedef struct tagTaskTimeSlice
{
    LOS_TASK_CB             *pstTask;                       /**< Current running task   */
    UINT16                  usTime;                         /**< Expiration time point           */
    UINT16                  usTout;                         /**< Expiration duration             */
} OS_TASK_ROBIN_S;

extern VOID osTaskSchedule(VOID);


/**
 * @ingroup  los_task
 * @brief Modify the priority of task.
 *
 * @par Description:
 * This API is used to modify the priority of task.
 *
 * @attention
 * <ul>
 * <li>The pstTaskCB should be a correct pointer to task control block structure.</li>
 * <li>the usPriority should be in [0, OS_TASK_PRIORITY_LOWEST].</li>
 * </ul>
 *
 * @param  pstTaskCB [IN] Type #LOS_TASK_CB * pointer to task control block structure.
 * @param  usPriority  [IN] Type #UINT16 the priority of task.
 *
 * @retval  None.
 * @par Dependency:
 * <ul><li>los_task.ph: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
extern VOID osTaskPriModify(LOS_TASK_CB *pstTaskCB, UINT16 usPriority);

/**
 * @ingroup  los_task
 * @brief Scan a task.
 *
 * @par Description:
 * This API is used to scan a task.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  None.
 *
 * @retval  None.
 * @par Dependency:
 * <ul><li>los_task.ph: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
extern VOID osTaskScan(VOID);

/**
 * @ingroup  los_task
 * @brief Initialization a task.
 *
 * @par Description:
 * This API is used to initialization a task.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  None.
 *
 * @retval  UINT32    Initialization result.
 * @par Dependency:
 * <ul><li>los_task.ph: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 osTaskInit(VOID);

/**
 * @ingroup  los_task
 * @brief Create idle task.
 *
 * @par Description:
 * This API is used to create idle task.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  None.
 *
 * @retval  UINT32   Create result.
 * @par Dependency:
 * <ul><li>los_task.ph: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 osIdleTaskCreate(VOID);

/**
 * @ingroup  los_task
 * @brief Check task switch.
 *
 * @par Description:
 * This API is used to check task switch.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  None.
 *
 * @retval  None.
 * @par Dependency:
 * <ul><li>los_task.ph: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
extern VOID osTaskSwitchCheck(VOID);

/**
 * @ingroup  los_task
 * @brief TaskMonInit.
 *
 * @par Description:
 * This API is used to taskMonInit.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  None.
 *
 * @retval  None.
 * @par Dependency:
 * <ul><li>los_task.ph: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
extern VOID osTaskMonInit(VOID);

/**
 * @ingroup  los_task
 * @brief Task entry.
 *
 * @par Description:
 * This API is used to task entry.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  uwTaskID  [IN] Type #UINT32   task id.
 *
 * @retval  None.
 * @par Dependency:
 * <ul><li>los_task.ph: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R001C00
 */
extern VOID osTaskEntry(UINT32 uwTaskID);

/**
 * @ingroup  los_task
 * @brief pend running task to pendlist
 *
 * @par Description:
 * This API is used to pend task to  pendlist and add to sorted delay list.
 *
 * @attention
 * <ul>
 * <li>The pstList should be a vaild pointer to pendlist.</li>
 * </ul>
 *
 * @param  pstList      [IN] Type #LOS_DL_LIST * pointer to list which running task will be pended.
 * @param  uwTaskStatus [IN] Type #UINT32  Task Status.
 * @param  uwTimeOut    [IN] Type #UINT32  Expiry time. The value range is [0,LOS_WAIT_FOREVER].
 *
 * @retval  LOS_OK       wait success
 * @retval  LOS_NOK      pend out
 * @par Dependency:
 * <ul><li>los_task.ph: the header file that contains the API declaration.</li></ul>
 * @see osTaskWake
 * @since Huawei LiteOS V100R001C00
 */
extern VOID osTaskWait(LOS_DL_LIST *pstList, UINT32 uwTaskStatus, UINT32 uwTimeOut);

/**
 * @ingroup  los_task
 * @brief delete task from pendlist.
 *
 * @par Description:
 * This API is used to delete task from pendlist and also add to the priqueue.
 *
 * @attention
 * <ul>
 * <li>The pstList should be a vaild pointer to pend list.</li>
 * </ul>
 *
 * @param  pstResumedTask [IN] Type #LOS_TASK_CB * pointer to the task which will be add to priqueue.
 * @param  uwTaskStatus [IN] Type #UINT32  Task Status.
 *
 * @retval  None.
 * @par Dependency:
 * <ul><li>los_task.ph: the header file that contains the API declaration.</li></ul>
 * @see osTaskWait
 * @since Huawei LiteOS V100R001C00
 */
extern VOID osTaskWake(LOS_TASK_CB *pstResumedTask, UINT32 uwTaskStatus);

/**
 * @ingroup  los_task
 * @brief Get the task water line.
 *
 * @par Description:
 * This API is used to get the task water line.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  uwTaskID [IN] Type #UINT32 task id.
 *
 * @retval  UINT32  Task water line.
 * @par Dependency:
 * <ul><li>los_task.ph: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 osGetTaskWaterLine(UINT32 uwTaskID);

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_TASK_PH */
