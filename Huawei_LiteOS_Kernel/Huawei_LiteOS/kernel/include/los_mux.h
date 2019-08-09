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

/** @defgroup los_mux Mutex
 * @ingroup kernel
 */

#ifndef _LOS_MUX_H
#define _LOS_MUX_H

#include "los_base.h"
#include "los_sys.h"
#include "los_list.h"
#include "los_task.h"

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif /* __cplusplus */
#endif /* __cplusplus */


/**
 * @ingroup los_mux
 * Mutex error code: The memory request fails.
 *
 * Value: 0x02001d00
 *
 * Solution: Decrease the number of mutexes defined by LOSCFG_BASE_IPC_MUX_LIMIT.
 */
#define LOS_ERRNO_MUX_NO_MEMORY         LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x00)

/**
 * @ingroup los_mux
 * Mutex error code: The mutex is not usable.
 *
 * Value: 0x02001d01
 *
 * Solution: Check whether the mutex ID and the mutex state are applicable for the current operation.
 */
#define LOS_ERRNO_MUX_INVALID            LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x01)

/**
* @ingroup los_mux
* Mutex error code: Null pointer.
*
* Value: 0x02001d02
*
* Solution: Check whether the input parameter is usable.
*/
#define LOS_ERRNO_MUX_PTR_NULL           LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x02)

/**
* @ingroup los_mux
* Mutex error code: No mutex is available and the mutex request fails.
*
* Value: 0x02001d03
*
* Solution: Increase the number of mutexes defined by LOSCFG_BASE_IPC_MUX_LIMIT.
*/
#define LOS_ERRNO_MUX_ALL_BUSY           LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x03)

/**
* @ingroup los_mux
* Mutex error code: The mutex fails to be locked in non-blocking mode because it is locked by another thread.
*
* Value: 0x02001d04
*
* Solution: Lock the mutex after it is unlocked by the thread that owns it, or set a waiting time.
*/
#define LOS_ERRNO_MUX_UNAVAILABLE       LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x04)

/**
* @ingroup los_mux
* Mutex error code: The mutex is being locked during an interrupt.
*
* Value: 0x02001d05
*
* Solution: Check whether the mutex is being locked during an interrupt.
*/
#define LOS_ERRNO_MUX_PEND_INTERR       LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x05)

/**
* @ingroup los_mux
* Mutex error code: A thread locks a mutex after waiting for the mutex to be unlocked by another thread when the task scheduling is disabled.
*
* Value: 0x02001d06
*
* Solution: Check whether the task scheduling is disabled, or set uwtimeout to 0, which means that the thread will not wait for the mutex to become available.
*/
#define LOS_ERRNO_MUX_PEND_IN_LOCK      LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x06)

/**
* @ingroup los_mux
* Mutex error code: The mutex locking times out.
*
* Value: 0x02001d07
*
* Solution: Increase the waiting time or set the waiting time to LOS_WAIT_FOREVER (forever-blocking mode).
*/
#define LOS_ERRNO_MUX_TIMEOUT           LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x07)

/**
 * @ingroup los_mux
 *
 * Value: 0x02001d08
 * Not in use temporarily.
 */
#define LOS_ERRNO_MUX_OVERFLOW          LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x08)

/**
* @ingroup los_mux
* Mutex error code: The mutex to be deleted is being locked.
*
* Value: 0x02001d09
*
* Solution: Delete the mutex after it is unlocked.
*/
#define LOS_ERRNO_MUX_PENDED            LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x09)

/**
 * @ingroup los_mux
 *
 * Value: 0x02001d0A
 * Not in use temporarily.
 */
#define LOS_ERRNO_MUX_GET_COUNT_ERR     LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x0A)

/**
 * @ingroup los_mux
 *
 * Value: 0x02001d0B
 * Not in use temporarily.
 */
#define LOS_ERRNO_MUX_REG_ERROR         LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x0B)


/**
 *@ingroup los_mux
 *@brief Create a mutex.
 *
 *@par Description:
 *This API is used to create a mutex. If there are available mutexes, the mutex is successfully created with a handle to the mutex returned.
 *@attention
 *<ul>
 *<li>The total number of mutexes is pre-configured. If there are no available mutexes, the mutex creation fails.</li>
 *</ul>
 *
 *@param puwMuxHandle   [OUT] ID of the handle to the successfully created mutex.
 *
 *@retval #LOS_ERRNO_MUX_PTR_NULL           0x02001d02: The puwMuxHandle pointer is NULL.
 *@retval #LOS_ERRNO_MUX_ALL_BUSY           0x02001d03: The mutex fails to be created because all mutexes in the OS are being in use.
 *@retval #LOS_OK                          0x00000000: The mutex is successfully created.
 *@par Dependency:
 *<ul><li>los_mux.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_MuxDelete
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MuxCreate(UINT32 *puwMuxHandle);

/**
 *@ingroup los_mux
 *@brief Delete a mutex.
 *
 *@par Description:
 *This API is used to delete a specified mutex and add it to the list of available mutexes.
 *@attention
 *<ul>
 *<li>The mutex that is not owned or waited on by any thread is able to be successfully deleted.</li>
 *</ul>
 *
 *@param puwMuxHandle   [IN] ID of the handle to the mutex to be deleted.
 *
 *@retval #LOS_ERRNO_MUX_INVALID            0x02001d01: The mutex fails to be deleted because it is not usable or it is being in use.
 *@retval #LOS_ERRNO_MUX_PENDED             0x02001d09: The mutex fails to be deleted because it is locked.
 *@retval #LOS_OK                          0x00000000: The mutex is successfully deleted.
 *@par Dependency:
 *<ul><li>los_mux.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_MuxDelete
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MuxDelete(UINT32 puwMuxHandle);

/**
 *@ingroup los_mux
 *@brief Wait to lock a mutex.
 *
 *@par Description:
 *This API is used to wait for a specified period of time to lock a mutex.
 *@attention
 *<ul>
 *<li>The function fails if the mutex that is waited on is already locked by another thread when the task scheduling is disabled.</li>
 *<li>Do not wait on a mutex during an interrupt.</li>
 *<li>The priority inheritance protocol is supported. If a higher-priority thread is waiting on a mutex, it changes the priority of the thread that owns the mutex to avoid priority inversion.</li>
 *<li>A recursive mutex can be locked more than once by the same thread.</li>
 *</ul>
 *
 *@param uwMuxHandle    [IN] ID of the handle to the mutex to be waited on.
 *@param uwTimeout      [IN] Waiting time. The value range is [0,LOS_WAIT_FOREVER].
 *
 *@retval #LOS_ERRNO_MUX_INVALID            0x02001d01: The mutex state (for example, the mutex does not exist or is not in use) is not applicable for the current operation.
 *@retval #LOS_ERRNO_MUX_UNAVAILABLE        0x02001d04: The mutex fails to be locked because it is locked by another thread and a period of time is not set for waiting for the mutex to become available.
 *@retval #LOS_ERRNO_MUX_PEND_INTERR        0x02001d05: The mutex is being locked during an interrupt.
 *@retval #LOS_ERRNO_MUX_PEND_IN_LOCK       0x02001d06: The mutex is waited on when the task scheduling is disabled.
 *@retval #LOS_ERRNO_MUX_TIMEOUT            0x02001d07: The mutex waiting times out.
 *@retval #LOS_OK                          0x00000000: The mutex is successfully locked.
 *@par Dependency:
 *<ul><li>los_mux.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_MuxPost
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MuxPend(UINT32 uwMuxHandle, UINT32 uwTimeout);

/**
 *@ingroup los_mux
 *@brief Release a mutex.
 *
 *@par Description:
 *This API is used to release a specified mutex.
 *@attention
 *<ul>
 *<li>Do not release a mutex during an interrupt.</li>
 *<li>If a recursive mutex is locked for many times, it must be unlocked for the same times to be released.</li>
 *</ul>
 *
 *@param uwMuxHandle    [IN] ID of the handle to the mutex to be released.
 *
 *@retval #LOS_ERRNO_MUX_INVALID            0x02001d01: The mutex state (for example, the mutex does not exist or is not in use) is not applicable for the current operation.
 *@retval #LOS_ERRNO_MUX_PEND_INTERR        0x02001d05: The mutex is being released during an interrupt.
 *@retval #LOS_ERRNO_MUX_INVALID            0x02001d01: The mutex state (for example, the mutex to be released is owned by another thread) is not applicable for the current operation.
 *@retval #LOS_OK                          0x00000000: The mutex is successfully released.
 *@par Dependency:
 *<ul><li>los_mux.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_MuxPend
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MuxPost(UINT32 uwMuxHandle);


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* _LOS_MUX_H */
