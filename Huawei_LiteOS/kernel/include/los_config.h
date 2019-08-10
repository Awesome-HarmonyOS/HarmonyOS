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
#include "target_config.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


/*=============================================================================
                                        System clock module configuration
=============================================================================*/

/**
 * @ingroup los_config
 * System clock (unit: HZ)
 */
#ifndef OS_SYS_CLOCK
#define OS_SYS_CLOCK                                        (100000000UL)
#endif

/**
 * @ingroup los_config
 * timer1 clock (unit: HZ)
 */
#ifndef OS_TIME_TIMER_CLOCK
#define OS_TIME_TIMER_CLOCK                                 OS_SYS_CLOCK
#endif

/**
 * @ingroup los_config
 * Number of Ticks in one second
 */
#ifndef LOSCFG_BASE_CORE_TICK_PER_SECOND
#define LOSCFG_BASE_CORE_TICK_PER_SECOND                    (1000UL)
#endif

#if defined(LOSCFG_BASE_CORE_TICK_PER_SECOND) \
    && ((LOSCFG_BASE_CORE_TICK_PER_SECOND < 1UL) || (LOSCFG_BASE_CORE_TICK_PER_SECOND > 1000000000UL))
    #error "LOSCFG_BASE_CORE_TICK_PER_SECOND SHOULD big than 0, and less than 1000000000UL"
#endif


#if (LOSCFG_BASE_CORE_TICK_PER_SECOND <= 1000UL)
/**
 * @ingroup los_config
 * How much time one tick spent (unit:ms)
 */
#ifndef LOSCFG_BASE_CORE_TICK_PERIOD_MS
#define LOSCFG_BASE_CORE_TICK_PERIOD_MS                     (1000UL / LOSCFG_BASE_CORE_TICK_PER_SECOND)
#endif

#elif (LOSCFG_BASE_CORE_TICK_PER_SECOND <= 1000000UL)
/**
 * @ingroup los_config
 * How much time one tick spent (unit:us)
 */
#ifndef LOSCFG_BASE_CORE_TICK_PERIOD_US
#define LOSCFG_BASE_CORE_TICK_PERIOD_US                     (1000000UL / LOSCFG_BASE_CORE_TICK_PER_SECOND)
#endif

#else
/**
 * @ingroup los_config
 * How much time one tick spent (unit:ns)
 */
#ifndef LOSCFG_BASE_CORE_TICK_PERIOD_NS
#define LOSCFG_BASE_CORE_TICK_PERIOD_NS                     (1000000000UL / LOSCFG_BASE_CORE_TICK_PER_SECOND)
#endif
#endif

/**
 * @ingroup los_config
 * External configuration item for timer tailoring
 */
#ifndef LOSCFG_BASE_CORE_TICK_HW_TIME1
#define LOSCFG_BASE_CORE_TICK_HW_TIME1                      YES
#endif

#ifndef LOSCFG_BASE_CORE_TICK_HW_TIME
#define LOSCFG_BASE_CORE_TICK_HW_TIME                       NO
#endif

/**
 * @ingroup los_config
 * Configuration liteos kernel tickless
 */
#ifndef LOSCFG_KERNEL_TICKLESS
#define LOSCFG_KERNEL_TICKLESS                              NO
#endif

/*=============================================================================
                                        Hardware interrupt module configuration
=============================================================================*/

/**
 * @ingroup los_config
 * Configuration item for hardware interrupt tailoring
 */
#ifndef LOSCFG_PLATFORM_HWI
#define LOSCFG_PLATFORM_HWI                                 YES
#endif

/**
 * @ingroup los_config
 * Maximum number of used hardware interrupts, including Tick timer interrupts.
 */
#ifndef LOSCFG_PLATFORM_HWI_LIMIT
#define LOSCFG_PLATFORM_HWI_LIMIT                           32
#endif

/*=============================================================================
                                       Task module configuration
=============================================================================*/

/**
 * @ingroup los_config
 * Minimum stack size.
 *
 * 0x80 bytes, aligned on a boundary of 8.
 */
#ifndef LOSCFG_BASE_CORE_TSK_MIN_STACK_SIZE
#define LOSCFG_BASE_CORE_TSK_MIN_STACK_SIZE                 (ALIGN(0x80, 4))
#endif

/**
 * @ingroup los_config
 * Default task priority
 */
#ifndef LOSCFG_BASE_CORE_TSK_DEFAULT_PRIO
#define LOSCFG_BASE_CORE_TSK_DEFAULT_PRIO                   10
#endif

/**
 * @ingroup los_config
 * Maximum supported number of tasks except the idle task rather than the number of usable tasks
 */
#ifndef LOSCFG_BASE_CORE_TSK_LIMIT
#define LOSCFG_BASE_CORE_TSK_LIMIT                          5
#endif

/**
 * @ingroup los_config
 * Size of the idle task stack
 */
#ifndef LOSCFG_BASE_CORE_TSK_IDLE_STACK_SIZE
#define LOSCFG_BASE_CORE_TSK_IDLE_STACK_SIZE                (0x180UL)
#endif

/**
 * @ingroup los_config
 * Default task stack size
 */
#ifndef LOSCFG_BASE_CORE_TSK_DEFAULT_STACK_SIZE
#define LOSCFG_BASE_CORE_TSK_DEFAULT_STACK_SIZE             (0x400UL)
#endif

/**
 * @ingroup los_config
 * Configuration item for task Robin tailoring
 */
#ifndef LOSCFG_BASE_CORE_TIMESLICE
#define LOSCFG_BASE_CORE_TIMESLICE                          YES
#endif

/**
 * @ingroup los_config
 * Longest execution time of tasks with the same priorities
 */
#ifndef LOSCFG_BASE_CORE_TIMESLICE_TIMEOUT
#define LOSCFG_BASE_CORE_TIMESLICE_TIMEOUT                  10
#endif

/**
 * @ingroup los_config
 * Configuration item for task (stack) monitoring module tailoring
 */
#ifndef LOSCFG_BASE_CORE_TSK_MONITOR
#define LOSCFG_BASE_CORE_TSK_MONITOR                        NO
#endif

/**
 * @ingroup los_config
 * Configuration item for task perf task filter hook
 */
#ifndef LOSCFG_BASE_CORE_EXC_TSK_SWITCH
#define LOSCFG_BASE_CORE_EXC_TSK_SWITCH                     NO
#endif

/**
 * @ingroup los_config
 * Define a usable task priority.Highest task priority.
 */
#ifndef LOS_TASK_PRIORITY_HIGHEST
#define LOS_TASK_PRIORITY_HIGHEST                           0
#endif

/**
 * @ingroup los_config
 * Define a usable task priority.Lowest task priority.
 */
#ifndef LOS_TASK_PRIORITY_LOWEST
#define LOS_TASK_PRIORITY_LOWEST                            31
#endif

/**
 * @ingroup los_config
 * SP align size.
 */
#ifndef LOSCFG_STACK_POINT_ALIGN_SIZE
#define LOSCFG_STACK_POINT_ALIGN_SIZE                       8
#endif


/*=============================================================================
                                       Semaphore module configuration
=============================================================================*/

/**
 * @ingroup los_config
 * Configuration item for semaphore module tailoring
 */
#ifndef LOSCFG_BASE_IPC_SEM
#define LOSCFG_BASE_IPC_SEM                                 YES
#endif

/**
 * @ingroup los_config
 * Maximum supported number of semaphores
 */
#ifndef LOSCFG_BASE_IPC_SEM_LIMIT
#define LOSCFG_BASE_IPC_SEM_LIMIT                           6
#endif

/*=============================================================================
                                       Mutex module configuration
=============================================================================*/

/**
 * @ingroup los_config
 * Configuration item for mutex module tailoring
 */
#ifndef LOSCFG_BASE_IPC_MUX
#define LOSCFG_BASE_IPC_MUX                                YES
#endif

/**
 * @ingroup los_config
 * Maximum supported number of mutexes
 */
#ifndef LOSCFG_BASE_IPC_MUX_LIMIT
#define LOSCFG_BASE_IPC_MUX_LIMIT                           6
#endif

/*=============================================================================
                                       Queue module configuration
=============================================================================*/

/**
 * @ingroup los_config
 * Configuration item for queue module tailoring
 */
#ifndef LOSCFG_BASE_IPC_QUEUE
#define LOSCFG_BASE_IPC_QUEUE                               YES
#endif

/**
 * @ingroup los_config
 * Maximum supported number of queues rather than the number of usable queues
 */
#ifndef LOSCFG_BASE_IPC_QUEUE_LIMIT
#define LOSCFG_BASE_IPC_QUEUE_LIMIT                         6
#endif


/*=============================================================================
                                       Software timer module configuration
=============================================================================*/

/**
 * @ingroup los_config
 * Configuration item for software timer module tailoring
 */
#ifndef LOSCFG_BASE_CORE_SWTMR
#define LOSCFG_BASE_CORE_SWTMR                              YES
#endif

/**
 * @ingroup los_config
 * Maximum supported number of software timers rather than the number of usable software timers
 */
#ifndef LOSCFG_BASE_CORE_SWTMR_LIMIT
#define LOSCFG_BASE_CORE_SWTMR_LIMIT                        5
#endif

/**
 * @ingroup los_config
 * Software timer task stack size
 */
#ifndef LOSCFG_BASE_CORE_TSK_SWTMR_STACK_SIZE
#define LOSCFG_BASE_CORE_TSK_SWTMR_STACK_SIZE               LOSCFG_BASE_CORE_TSK_DEFAULT_STACK_SIZE
#endif

/**
 * @ingroup los_config
 * Configurate item for handling software timer interrupt in task tailoring
 */
#ifndef LOSCFG_BASE_CORE_SWTMR_TASK
#define LOSCFG_BASE_CORE_SWTMR_TASK                         YES
#endif

/**
 * @ingroup los_config
 * Configurate item for software timer align tailoring
 */
#ifndef LOSCFG_BASE_CORE_SWTMR_ALIGN
#define LOSCFG_BASE_CORE_SWTMR_ALIGN                        NO
#endif

#if(LOSCFG_BASE_CORE_SWTMR == NO && LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
    #error "swtmr align first need support swmtr, should make LOSCFG_BASE_CORE_SWTMR = YES"
#endif

/**
 * @ingroup los_config
 * Max number of software timers ID
 */
#ifndef OS_SWTMR_MAX_TIMERID
#define OS_SWTMR_MAX_TIMERID                                ((65535 / LOSCFG_BASE_CORE_SWTMR_LIMIT) * LOSCFG_BASE_CORE_SWTMR_LIMIT)
#endif

/**
 * @ingroup los_config
 * Maximum size of a software timer queue
 */
#ifndef OS_SWTMR_HANDLE_QUEUE_SIZE
#define OS_SWTMR_HANDLE_QUEUE_SIZE                          (LOSCFG_BASE_CORE_SWTMR_LIMIT + 0)
#endif

/**
 * @ingroup los_config
 * Minimum divisor of software timer multiple alignment
 */
#ifndef LOS_COMMON_DIVISOR
#define LOS_COMMON_DIVISOR                                 10
#endif

/*=============================================================================
                                       Memory module configuration
=============================================================================*/

extern UINT8 *m_aucSysMem0;

/**
 * @ingroup los_config
 * Starting address of the memory
 */
#ifndef OS_SYS_MEM_ADDR
#define OS_SYS_MEM_ADDR                                     (&m_aucSysMem0[0])
#endif

/**
 * @ingroup los_config
 * Starting address of the task stack
 */
#ifndef OS_TASK_STACK_ADDR
#define OS_TASK_STACK_ADDR                                   OS_SYS_MEM_ADDR
#endif

/**
 * @ingroup los_config
 * Ending address of the memory
 */
extern UINT32 g_sys_mem_addr_end;


/**
 * @ingroup los_config
 * Memory size
 */
#ifndef OS_SYS_MEM_SIZE
#define OS_SYS_MEM_SIZE                                     (0x10000UL)
#endif

#ifndef LOSCFG_MEMORY_BESTFIT
#define LOSCFG_MEMORY_BESTFIT                               YES
#endif

/**
 * @ingroup los_config
 * Configuration module tailoring of more mempry pool checking
 */
#ifndef LOSCFG_MEM_MUL_POOL
#define LOSCFG_MEM_MUL_POOL                                 NO
#endif

/**
 * @ingroup los_config
 * Configuration module tailoring of slab memory
 */
#ifndef LOSCFG_KERNEL_MEM_SLAB
#define LOSCFG_KERNEL_MEM_SLAB                              YES
#endif

/**
 * @ingroup los_config
 * Configuration module tailoring of mem node integrity checking
 */
#ifndef LOSCFG_BASE_MEM_NODE_INTEGRITY_CHECK
#define LOSCFG_BASE_MEM_NODE_INTEGRITY_CHECK                NO
#endif

/**
 * @ingroup los_config
 * Configuration module tailoring of mem node size checking
 */
#ifndef LOSCFG_BASE_MEM_NODE_SIZE_CHECK
#define LOSCFG_BASE_MEM_NODE_SIZE_CHECK                     YES
#endif

/**
 * @ingroup los_config
 * Number of memory checking blocks
 */
#ifndef OS_SYS_MEM_NUM
#define OS_SYS_MEM_NUM                                      20
#endif

/**
 * @ingroup los_config
 * Configuration heap memory peak statistics
 */
#ifndef LOSCFG_HEAP_MEMORY_PEAK_STATISTICS
#define LOSCFG_HEAP_MEMORY_PEAK_STATISTICS                  YES
#endif

/**
 * @ingroup los_config
 * Size of unaligned memory
 */
#ifndef OS_SYS_NOCACHEMEM_SIZE
#define OS_SYS_NOCACHEMEM_SIZE                              0x0UL
#endif

/**
 * @ingroup los_config
 * Starting address of the unaligned memory
 */
#if (OS_SYS_NOCACHEMEM_SIZE > 0)
#define OS_SYS_NOCACHEMEM_ADDR                              &m_aucSysNoCacheMem0[0]
#endif

/**
 * @ingroup los_config
 * Configuration module tailoring of the total amount of memory used for tasks
 */
#ifndef LOSCFG_MEM_TASK_USED_STATISTICS
#define LOSCFG_MEM_TASK_USED_STATISTICS                     NO
#endif


/*=============================================================================
                                        Exception module configuration
=============================================================================*/

/**
 * @ingroup los_config
 * Configuration item for exception tailoring
 */
#ifndef LOSCFG_PLATFORM_EXC
#define LOSCFG_PLATFORM_EXC                                 NO
#endif

/**
 * @ingroup los_config
 * Configuration item for saveing exception info tailoring
 */
#ifndef LOSCFG_SAVE_EXC_INFO
#define LOSCFG_SAVE_EXC_INFO                                NO
#endif

#if (LOSCFG_PLATFORM_EXC == YES)

/**
 * @ingroup los_config
 * Configuration exception call stack analysis max depth
 */
#ifndef LOSCFG_EXC_CALL_STACK_ANALYSIS_MAX_DEPTH
#define LOSCFG_EXC_CALL_STACK_ANALYSIS_MAX_DEPTH            16
#endif

/**
 * @ingroup los_config
 * Configuration code start address and code size, msp start address and size
 *
 * NOTE: Users must reconfigure these macros, otherwise, the invocation relationship
 *       can not be correctly analyzed.
 */
#ifndef LOSCFG_EXC_CODE_START_ADDR
#define LOSCFG_EXC_CODE_START_ADDR                          (0x08000000)  /* invalid, Please reconfigure it */
#endif
#ifndef LOSCFG_EXC_CODE_SIZE
#define LOSCFG_EXC_CODE_SIZE                                (0x00100000)  /* invalid, Please reconfigure it */
#endif
#ifndef LOSCFG_EXC_MSP_START_ADDR
#define LOSCFG_EXC_MSP_START_ADDR                           (0x20000000)  /* invalid, Please reconfigure it */
#endif
#ifndef LOSCFG_EXC_MSP_SIZE
#define LOSCFG_EXC_MSP_SIZE                                 (0x00080000)  /* invalid, Please reconfigure it */
#endif

#endif  /* LOSCFG_PLATFORM_EXC == YES */

#if(LOSCFG_PLATFORM_EXC == NO && LOSCFG_SAVE_EXC_INFO == YES)
    #error "save exception info need support platform exception, should make LOSCFG_PLATFORM_EXC = YES"
#endif

/*=============================================================================
                                       MPU module configuration
=============================================================================*/
/**
 * @ingroup los_config
 * Configuration item for MPU
 */
#ifndef LOSCFG_BASE_CORE_MPU
#define LOSCFG_BASE_CORE_MPU                                NO
#endif

/**
 * @ingroup los_config
   * MPU support number : MPU maximum number of region support(According to the cotex-m4 authority Guide)
 */
#ifndef LOSCFG_MPU_MAX_SUPPORT
#define LOSCFG_MPU_MAX_SUPPORT                              8
#endif

/**
 * @ingroup los_config
   * MPU support address range : from LOSCFG_MPU_MIN_ADDRESS to LOSCFG_MPU_MAX_ADDRESS
 */
#ifndef LOSCFG_MPU_MIN_ADDRESS
#define LOSCFG_MPU_MIN_ADDRESS                              0x0UL           // Minimum protected address
#endif

#ifndef LOSCFG_MPU_MAX_ADDRESS
#define LOSCFG_MPU_MAX_ADDRESS                              0xFFFFFFFFUL    // Maximum protected address
#endif


/*=============================================================================
                                       Runstop module configuration
=============================================================================*/

/**
 * @ingroup los_config
 * Configuration item for runstop module tailoring
 */
#ifndef LOSCFG_KERNEL_RUNSTOP
#define LOSCFG_KERNEL_RUNSTOP                               NO
#endif

/*=============================================================================
                                            Perf module configuration
=============================================================================*/

/**
 * @ingroup los_config
 * Configuration item for performance moniter unit
 */
#ifndef OS_INCLUDE_PERF
#define OS_INCLUDE_PERF                                     NO
#endif


/*=============================================================================
                                        CPUP configuration
=============================================================================*/

/**
 * @ingroup los_config
 * Configuration item for CPU usage tailoring
 */
#ifndef LOSCFG_BASE_CORE_CPUP
#define LOSCFG_BASE_CORE_CPUP                               NO
#endif


/*=============================================================================
                                       fw Interface configuration
=============================================================================*/

/**
 * @ingroup los_config
 * Configuration item for the monitoring of task communication
 */
#ifndef LOSCFG_COMPAT_CMSIS_FW
#define LOSCFG_COMPAT_CMSIS_FW                              NO
#endif


/*=============================================================================
                                       Shell module configuration
=============================================================================*/

/**
 * @ingroup los_config
 * Configuration item for shell module tailoring
 */
#ifndef OS_INCLUDE_SHELL
#define OS_INCLUDE_SHELL                                    NO
#endif


/*=============================================================================
                                       Test module configuration
=============================================================================*/

/**
 * @ingroup los_config
 * Configuration test case to open
 */
#ifndef LOSCFG_TEST
#define LOSCFG_TEST                                         NO
#endif


/*=============================================================================
                                       LiteOS kernel version configuration
=============================================================================*/

/**
 * @ingroup los_config
 * Version number
 */
#ifndef LITEOS_VER
#define LITEOS_VER                                          "Huawei LiteOS Kernel V200R001c50"
#endif

/**
 * @ingroup los_config
 * Configuration CMSIS_OS_VER
 */
#ifndef CMSIS_OS_VER
#define CMSIS_OS_VER                                        1
#endif


/*=============================================================================
                                       LIB module configuration
=============================================================================*/

/**
 * @ingroup los_config
 * newlib struct _reent
 */
#ifndef LOSCFG_LIB_LIBC_NEWLIB_REENT
#define LOSCFG_LIB_LIBC_NEWLIB_REENT                        NO
#endif


/*=============================================================================
                                       VFS module configuration
=============================================================================*/

/**
 * @ingroup los_config
 * Configuration item for enabling LiteOS VFS
 */
#ifndef LOSCFG_ENABLE_VFS
#define LOSCFG_ENABLE_VFS                                   NO
#endif


/**
 * @ingroup los_config
 * Configuration item for enabling LiteOS KIFS (kernel info fs)
 */
#ifndef LOSCFG_ENABLE_KIFS
#define LOSCFG_ENABLE_KIFS                                  NO
#endif


/*=============================================================================
                                       DEVFS module configuration
=============================================================================*/

/**
 * @ingroup los_config
 * Configuration item for enabling LiteOS DEVFS
 */
#ifndef LOSCFG_ENABLE_DEVFS
#define LOSCFG_ENABLE_DEVFS                                 NO
#else
#if (LOSCFG_ENABLE_DEVFS == YES)
#undef  LOSCFG_ENABLE_VFS
#define LOSCFG_ENABLE_VFS                                   YES
#undef  LOSCFG_ENABLE_KIFS
#define LOSCFG_ENABLE_KIFS                                  YES
#endif
#endif


/*=============================================================================
                                       Declaration of Huawei LiteOS module initialization functions
=============================================================================*/


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
 * @brief: User application Task init function.
 *
 * @par Description:
 * This API is used to initialize User application Task module.
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
//extern UINT32 osAppInit(VOID);


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
extern LITE_OS_SEC_TEXT_INIT UINT32 osMemSystemInit(VOID);


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


extern LITE_OS_SEC_TEXT_INIT UINT32 LOS_Start(void);


extern LITE_OS_SEC_TEXT_INIT int main(void);


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */


#endif /* _LOS_CONFIG_H */
