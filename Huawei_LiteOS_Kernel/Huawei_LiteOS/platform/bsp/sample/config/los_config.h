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

/**@defgroup los_config System configuration items
 * @ingroup kernel
 */

#ifndef _LOS_CONFIG_H
#define _LOS_CONFIG_H

#include "los_typedef.h"
#include "los_printf.h"
#include "stdio.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/****************************** System clock module configuration ****************************/
/**
 * @ingroup los_config
 * System clock (unit: HZ)
 */
#define OS_SYS_CLOCK                                    16000000

/**
* @ingroup los_config
* limit addr range when search for  'func local(frame pointer)' or 'func name'
*/
extern char __data_end;
extern char __bss_start;
#define OS_SYS_FUNC_ADDR_START                          &__bss_start
#define OS_SYS_FUNC_ADDR_END                            &__data_end

/**
 * @ingroup los_config
 * Number of Ticks in one second
 */
#define LOSCFG_BASE_CORE_TICK_PER_SECOND                1000

/**
 * @ingroup los_config
 * External configuration item for timer tailoring
 */
#define LOSCFG_BASE_CORE_TICK_HW_TIME                  NO

/****************************** Hardware interrupt module configuration ******************************/
/**
 * @ingroup los_config
 * Configuration item for hardware interrupt tailoring
 */
#define LOSCFG_PLATFORM_HWI                             YES

/**
 * @ingroup los_config
 * Maximum number of used hardware interrupts, including Tick timer interrupts.
 */
#define LOSCFG_PLATFORM_HWI_LIMIT                       96

/****************************** Task module configuration ********************************/
/**
 * @ingroup los_config
 * Default task priority
 */
#define LOSCFG_BASE_CORE_TSK_DEFAULT_PRIO               10

/**
 * @ingroup los_config
 * Maximum supported number of tasks except the idle task rather than the number of usable tasks
 */
#define LOSCFG_BASE_CORE_TSK_LIMIT                      15              // max num task

/**
 * @ingroup los_config
 * Size of the idle task stack
 */
#define LOSCFG_BASE_CORE_TSK_IDLE_STACK_SIZE            SIZE(0x500)     // IDLE task stack

/**
 * @ingroup los_config
 * Default task stack size
 */
#define LOSCFG_BASE_CORE_TSK_DEFAULT_STACK_SIZE         SIZE(0x2D0)     // default stack

/**
 * @ingroup los_config
 * Minimum stack size.
 */
#define LOS_TASK_MIN_STACK_SIZE                         (ALIGN(0x130, 16))

/**
 * @ingroup los_config
 * Configuration item for task Robin tailoring
 */
#define LOSCFG_BASE_CORE_TIMESLICE                      YES             // task-ROBIN moduel cutting switch

/**
 * @ingroup los_config
 * Longest execution time of tasks with the same priorities
 */
#define LOSCFG_BASE_CORE_TIMESLICE_TIMEOUT              10

/**
 * @ingroup los_config
 * Configuration item for task (stack) monitoring module tailoring
 */
#define LOSCFG_BASE_CORE_TSK_MONITOR                    YES

/**
 * @ingroup los_config
 * Configuration item for performance moniter unit
 */
#define OS_INCLUDE_PERF                                 YES

/**
 * @ingroup los_config
 * Define a usable task priority.Highest task priority.
 */
#define LOS_TASK_PRIORITY_HIGHEST                       0

/**
 * @ingroup los_config
 * Define a usable task priority.Lowest task priority.
 */
#define LOS_TASK_PRIORITY_LOWEST                        31

/****************************** Semaphore module configuration ******************************/
/**
 * @ingroup los_config
 * Configuration item for semaphore module tailoring
 */
#define LOSCFG_BASE_IPC_SEM                             YES

/**
 * @ingroup los_config
 * Maximum supported number of semaphores
 */
#define LOSCFG_BASE_IPC_SEM_LIMIT                       10              // the max sem-numb

/****************************** mutex module configuration ******************************/
/**
 * @ingroup los_config
 * Configuration item for mutex module tailoring
 */
#define LOSCFG_BASE_IPC_MUX                             YES

/**
 * @ingroup los_config
 * Maximum supported number of mutexes
 */
#define LOSCFG_BASE_IPC_MUX_LIMIT                       10              // the max mutex-num

/****************************** Queue module configuration ********************************/
/**
 * @ingroup los_config
 * Configuration item for queue module tailoring
 */
#define LOSCFG_BASE_IPC_QUEUE                           YES

/**
 * @ingroup los_config
 * Maximum supported number of queues rather than the number of usable queues
 */
#define LOSCFG_BASE_IPC_QUEUE_LIMIT                     10              //the max queue-numb

/****************************** Software timer module configuration **************************/
#if (LOSCFG_BASE_IPC_QUEUE == YES)
/**
 * @ingroup los_config
 * Configuration item for software timer module tailoring
 */
#define LOSCFG_BASE_CORE_SWTMR                          YES

/**
 * @ingroup los_config
 * Maximum supported number of software timers rather than the number of usable software timers
 */
#define LOSCFG_BASE_CORE_SWTMR_LIMIT                    16					// the max SWTMR numb

/**
 * @ingroup los_config
 * Max number of software timers ID
 */
#define OS_SWTMR_MAX_TIMERID                            ((65535/LOSCFG_BASE_CORE_SWTMR_LIMIT) * LOSCFG_BASE_CORE_SWTMR_LIMIT)

/**
 * @ingroup los_config
 * Maximum size of a software timer queue
 */
#define OS_SWTMR_HANDLE_QUEUE_SIZE                      (LOSCFG_BASE_CORE_SWTMR_LIMIT + 0)

/**
 * @ingroup los_config
 * Minimum divisor of software timer multiple alignment
 */
 #define LOS_COMMON_DIVISOR                             10
#endif

/****************************** Memory module configuration **************************/
/**
 * @ingroup los_config
 * Starting address of the memory
 */
#define OS_SYS_MEM_ADDR                                 &m_aucSysMem0[0]

/**
 * @ingroup los_config
 * Ending address of the memory
 */
extern UINT32 g_sys_mem_addr_end;
extern char _PT0_ADDR;
extern char _PT0_END;

/**
 * @ingroup los_config
 * Memory size
 */
#define OS_SYS_MEM_SIZE                                     0x00008000          // size

/**
 * @ingroup los_config
 * Configuration module tailoring of mem node integrity checking
 */
#define LOSCFG_BASE_MEM_NODE_INTEGRITY_CHECK                YES

/**
 * @ingroup los_config
 * Configuration module tailoring of mem node size checking
 */
#define LOSCFG_BASE_MEM_NODE_SIZE_CHECK                     YES

/**
 * @ingroup los_config
 * Number of memory checking blocks
 */
#define OS_SYS_MEM_NUM                                      20

/****************************** fw Interface configuration **************************/
/**
 * @ingroup los_config
 * Configuration item for the monitoring of task communication
 */
#define LOSCFG_COMPAT_CMSIS_FW                              YES

/****************************** proc module configuration **************************/
/**
 * @ingroup los_config
 * Version number
 */
#define VER                                                 "Huawei LiteOS KernelV100R001c00B021"

/****************************** others **************************/
/**
 * @ingroup los_config
 * Configuration system wake-up info to open
 */
#define OS_SR_WAKEUP_INFO                                   YES

/**
 * @ingroup los_config
 * Configuration library function is included
 */
#ifndef LOSCFG_LIB_LIBC
#define LOSCFG_LIB_LIBC
#endif

/* Declaration of Huawei LiteOS module initialization functions*/

/**
 * @ingroup  los_config
 * @brief: Task init function.
 *
 * @par Description:
 * This API is used to initialize task module.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param: None.
 *
 * @retval #LOS_ERRNO_TSK_NO_MEMORY            0x03000200:Insufficient memory for task creation.
 * @retval #LOS_OK                             0:Task initialization success.
 *
 * @par Dependency:
 * <ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 osTaskInit(VOID);



/**
 * @ingroup  los_config
 * @brief: hardware interrupt init function.
 *
 * @par Description:
 * This API is used to initialize hardware interrupt module.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param: None.
 *
 * @retval #LOS_OK                      0:Hardware interrupt initialization success.
 *
 * @par Dependency:
 * <ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern VOID osHwiInit(void);



/**
 * @ingroup  los_config
 * @brief: Semaphore init function.
 *
 * @par Description:
 * This API is used to initialize Semaphore module.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param: None.
 *
 * @retval #LOS_ERRNO_SEM_NO_MEMORY     0x02000700:The memory is insufficient.
 * @retval #LOS_OK                      0:Semaphore initialization success.
 *
 * @par Dependency:
 * <ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 osSemInit(void);



/**
 * @ingroup  los_config
 * @brief: Mutex init function.
 *
 * @par Description:
 * This API is used to initialize mutex module.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param: None.
 *
 * @retval #LOS_ERRNO_MUX_NO_MEMORY     0x02001d00:The memory request fails.
 * @retval #LOS_OK                      0:Mutex initialization success.
 *
 * @par Dependency:
 * <ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 osMuxInit(void);



/**
 * @ingroup  los_config
 * @brief: Queue init function.
 *
 * @par Description:
 * This API is used to initialize Queue module.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param: None.
 *
 * @retval #LOS_ERRNO_QUEUE_MAXNUM_ZERO 0x02000600:The maximum number of queue resources is configured to 0.
 * @retval #LOS_ERRNO_QUEUE_NO_MEMORY   0x02000601:The queue block memory fails to be initialized.
 * @retval #LOS_OK                      0:Queue initialization success.
 *
 * @par Dependency:
 * <ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 osQueueInit(void);



/**
 * @ingroup  los_config
 * @brief: Software Timers init function.
 *
 * @par Description:
 * This API is used to initialize Software Timers module.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param: None.
 *
 * @retval #LOS_ERRNO_SWTMR_MAXSIZE_INVALID         0x02000308:Invalid configured number of software timers.
 * @retval #LOS_ERRNO_SWTMR_NO_MEMORY               0x02000307:Insufficient memory for software timer linked list creation.
 * @retval #LOS_ERRNO_SWTMR_HANDLER_POOL_NO_MEM     0x0200030a:Insufficient memory allocated by membox.
 * @retval #LOS_ERRNO_SWTMR_QUEUE_CREATE_FAILED     0x0200030b:The software timer queue fails to be created.
 * @retval #LOS_ERRNO_SWTMR_TASK_CREATE_FAILED      0x0200030c:The software timer task fails to be created.
 * @retval #LOS_OK                                  0:Software Timers initialization success.
 *
 * @par Dependency:
 * <ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 osSwTmrInit(void);



/**
 * @ingroup  los_config
 * @brief: Task start running function.
 *
 * @par Description:
 * This API is used to start a task.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param: None.
 *
 * @retval None.
 *
 * @par Dependency:
 * <ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern VOID LOS_StartToRun(VOID);



/**
 * @ingroup  los_config
 * @brief: Test Task init function.
 *
 * @par Description:
 * This API is used to initialize Test Task.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param: None.
 *
 * @retval #LOS_OK                                  0:App_Task initialization success.
 *
 * @par Dependency:
 * <ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 los_TestInit(VOID);


/**
 * @ingroup  los_config
 * @brief: Task start function.
 *
 * @par Description:
 * This API is used to start all tasks.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param: None.
 *
 * @retval None.
 *
 * @par Dependency:
 * <ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern VOID   osStart(void);



/**
 * @ingroup  los_config
 * @brief: Hardware init function.
 *
 * @par Description:
 * This API is used to initialize Hardware module.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param: None.
 *
 * @retval None.
 *
 * @par Dependency:
 * <ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern VOID   osHwInit(VOID);



/**
 *@ingroup los_config
 *@brief Configuring the maximum number of tasks.
 *
 *@par Description:
 *This API is used to configuring the maximum number of tasks.
 *
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param: None.
 *
 *@retval None.
 *
 *@par Dependency:
 *<ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
extern VOID osRegister(VOID);



/**
 *@ingroup los_config
 *@brief System kernel initialization function.
 *
 *@par Description:
 *This API is used to Initialize kernel ,configure all system modules.
 *
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param: None.
 *
 *@retval #LOS_OK                                  0:System kernel initialization success.
 *
 *@par Dependency:
 *<ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
extern int osMain(void);



/**
 *@ingroup los_config
 *@brief Configure Tick Interrupt Start.
 *
 *@par Description:
 *This API is used to configure Tick Interrupt Start.
 *
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param: None.
 *
 *@retval #LOS_OK                               0:configure Tick Interrupt success.
 *@retval #LOS_ERRNO_TICK_CFG_INVALID           0x02000400:configure Tick Interrupt failed.
 *
 *@par Dependency:
 *<ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 osTickStart(VOID);


/**
 *@ingroup los_config
 *@brief Scheduling initialization.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to initialize scheduling that is used for later task scheduling.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param: None.
 *
 *@retval: None.
 *@par Dependency:
 *<ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 *@see
 *@since Huawei LiteOS V100R001C00
 */
extern VOID osTimesliceInit(VOID);



/**
 * @ingroup  los_config
 * @brief: System memory init function.
 *
 * @par Description:
 * This API is used to initialize system memory module.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param: None.
 *
 * @retval #LOS_OK                                  0:System memory initialization success.
 * @retval #OS_ERROR                                (UINT32)(-1):System memory initialization failed.
 *
 * @par Dependency:
 * <ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 osMemSystemInit(VOID);



/**
 * @ingroup  los_config
 * @brief: Task Monitor init function.
 *
 * @par Description:
 * This API is used to initialize Task Monitor module.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param: None.
 *
 * @retval #LOS_OK                                  0:Task Monitor initialization success.
 *
 * @par Dependency:
 * <ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern VOID osTaskMonInit(VOID);



/**
 * @ingroup  los_config
 * @brief: CPUP init function.
 *
 * @par Description:
 * This API is used to initialize CPUP module.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param: None.
 *
 * @retval #LOS_ERRNO_CPUP_NO_MEMORY                0x02001e00:The request for memory fails.
 * @retval #LOS_OK                                  0:CPUP initialization success.
 *
 * @par Dependency:
 * <ul><li>los_config.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 osCpupInit(VOID);

extern void osBackTrace();

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */


#endif /* _LOS_CONFIG_H */
