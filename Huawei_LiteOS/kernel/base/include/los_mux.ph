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

#ifndef _LOS_MUX_PH
#define _LOS_MUX_PH

#include "los_task.ph"

#include "los_mux.h"

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif /* __cplusplus */
#endif /* __cplusplus */


/**
 * @ingroup los_mux
 * Mutex object.
 */
typedef struct
{
    UINT8           ucMuxStat;       /**< State OS_MUX_UNUSED,OS_MUX_USED  */
    UINT16          usMuxCount;      /**< Times of locking a mutex */
    UINT32          ucMuxID;         /**< Handle ID*/
    LOS_DL_LIST     stMuxList;       /**< Mutex linked list*/
    LOS_TASK_CB     *pstOwner;       /**< The current thread that is locking a mutex*/
    UINT16          usPriority;      /**< Priority of the thread that is locking a mutex */
} MUX_CB_S;

/**
 * @ingroup los_mux
 * Mutex state: not in use.
 */
#define OS_MUX_UNUSED                   0

/**
 * @ingroup los_mux
 * Mutex state: in use.
 */
#define OS_MUX_USED                     1

extern MUX_CB_S             *g_pstAllMux;

/**
 * @ingroup los_mux
 * Obtain the pointer to a mutex object of the mutex that has a specified handle.
 */
#define GET_MUX(muxid)                  (((MUX_CB_S *)g_pstAllMux) + (muxid))

/**
 *@ingroup los_mux
 *@brief Initializes the mutex.
 *
 *@par Description:
 *This API is used to initializes the mutex.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param None.
 *
 *@retval UINT32     Initialization result.
 *@par Dependency:
 *<ul><li>los_mux.ph: the header file that contains the API declaration.</li></ul>
 *@see LOS_MuxDelete
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 osMuxInit(VOID);

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_MUX_PH */
