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

#ifndef _LOS_SEM_PH
#define _LOS_SEM_PH

#include "los_sem.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


/**
 * @ingroup los_sem
 * Semaphore control structure.
 */
typedef struct
{
    UINT8           usSemStat;          /**< Semaphore state*/
    UINT16          uwSemCount;         /**< Number of available semaphores*/
    UINT32          usSemID;            /**< Semaphore control structure ID*/
    LOS_DL_LIST     stSemList;          /**< Queue of tasks that are waiting on a semaphore*/
} SEM_CB_S;

/**
 * @ingroup los_sem
 *The semaphore is not in use.
 *
 */
#define OS_SEM_UNUSED                   0
/**
 * @ingroup los_sem
 *The semaphore is used.
 *
 */
#define OS_SEM_USED                     1
/**
 * @ingroup los_sem
 * Obtain the head node in a semaphore doubly linked list.
 *
 */
#define GET_SEM_LIST(ptr)               LOS_DL_LIST_ENTRY(ptr, SEM_CB_S, stSemList)
extern SEM_CB_S    *g_pstAllSem;
/**
 * @ingroup los_sem
 * Obtain a semaphore ID.
 *
 */
#define GET_SEM(semid)                  (((SEM_CB_S *)g_pstAllSem) + (semid))
/**
 * @ingroup los_sem
 * Maximum value of task information.
 *
 */
#define OS_MAX_PENDTASK_INFO            4


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_SEM_PH */
