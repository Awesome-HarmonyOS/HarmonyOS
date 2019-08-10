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

/**@defgroup los_sem Semaphore
 * @ingroup kernel
 */

#ifndef _LOS_SEM_H
#define _LOS_SEM_H

#include "los_base.h"
#include "los_err.h"
#include "los_list.h"
#include "los_task.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


/**
 * @ingroup los_sem
 * Semaphore error code: The memory is insufficient.
 *
 * Value: 0x02000700
 *
 * Solution: Allocate more memory.
 */
#define LOS_ERRNO_SEM_NO_MEMORY                 LOS_ERRNO_OS_ERROR(LOS_MOD_SEM, 0x00)

/**
 * @ingroup los_sem
 * Semaphore error code: Invalid parameter.
 *
 * Value: 0x02000701
 *
 * Solution: Change the passed-in invalid parameter value to a valid value.
 */
#define LOS_ERRNO_SEM_INVALID                   LOS_ERRNO_OS_ERROR(LOS_MOD_SEM, 0x01)

/**
 * @ingroup los_sem
 * Semaphore error code: Null pointer.
 *
 * Value: 0x02000702
 *
 * Solution: Change the passed-in null pointer to a valid non-null pointer.
 */
#define LOS_ERRNO_SEM_PTR_NULL                  LOS_ERRNO_OS_ERROR(LOS_MOD_SEM, 0x02)

/**
 * @ingroup los_sem
 * Semaphore error code: No semaphore control structure is available.
 *
 * Value: 0x02000703
 *
 * Solution: Perform corresponding operations based on the requirements in the code context.
 */
#define LOS_ERRNO_SEM_ALL_BUSY                  LOS_ERRNO_OS_ERROR(LOS_MOD_SEM, 0x03)

/**
* @ingroup los_sem
* Semaphore error code: Invalid parameter that specifies the timeout interval.
*
* Value: 0x02000704
*
*
* Solution: Change the passed-in parameter value to a valid nonzero value.
*/
#define LOS_ERRNO_SEM_UNAVAILABLE               LOS_ERRNO_OS_ERROR(LOS_MOD_SEM, 0x04)

/**
* @ingroup los_sem
* Semaphore error code: The API is called during an interrupt, which is forbidden.
*
* Value: 0x02000705
*
* Solution: Do not call the API during an interrupt.
*/
#define LOS_ERRNO_SEM_PEND_INTERR               LOS_ERRNO_OS_ERROR(LOS_MOD_SEM, 0x05)

/**
* @ingroup los_sem
* Semaphore error code: The task is unable to request a semaphore because task scheduling is locked.
*
* Value: 0x02000706
*
*Solution: Do not call LOS_SemPend when task scheduling is locked.
*/
#define LOS_ERRNO_SEM_PEND_IN_LOCK              LOS_ERRNO_OS_ERROR(LOS_MOD_SEM, 0x06)

/**
 * @ingroup los_sem
 * Semaphore error code: The request for a semaphore times out.
 *
 * Value: 0x02000707
 *
 * Solution: Change the passed-in parameter value to the value within the valid range.
 */
#define LOS_ERRNO_SEM_TIMEOUT                   LOS_ERRNO_OS_ERROR(LOS_MOD_SEM, 0x07)

/**
 * @ingroup los_sem
 * Semaphore error code: The times of semaphore release exceed the maximum times permitted.
 *
 * Value: 0x02000708
 *
 * Solution: Perform corresponding operations based on the requirements in the code context.
 */
#define LOS_ERRNO_SEM_OVERFLOW                  LOS_ERRNO_OS_ERROR(LOS_MOD_SEM, 0x08)

/**
 * @ingroup los_sem
 * Semaphore error code: The queue of the tasks that are waiting on the semaphore control structure is not null.
 *
 * Value: 0x02000709
 *
 * Solution: Delete the semaphore after awaking all tasks that are waiting on the semaphore.
 */
#define LOS_ERRNO_SEM_PENDED                    LOS_ERRNO_OS_ERROR(LOS_MOD_SEM, 0x09)

/**
 * @ingroup los_sem
 * Semaphore error code: LOS_ERRNO_SEM_MAXNUM_ZERO is error.
 *
 * Value: 0x0200070A
 *
 * Solution: LOS_ERRNO_SEM_MAXNUM_ZERO should not be error.
 */
#define LOS_ERRNO_SEM_MAXNUM_ZERO                    LOS_ERRNO_OS_ERROR(LOS_MOD_SEM, 0x0A)

/**
 *@ingroup los_sem
 *@brief Create a Counting semaphore.
 *
 *@par Description:
 *This API is used to create a semaphore control structure according to the initial number of available semaphores specified by usCount and return the ID of this semaphore control structure.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param usCount        [IN] Initial number of available semaphores. The value range is [0, OS_SEM_COUNTING_MAX_COUNT).
 *@param puwSemHandle   [OUT] ID of the semaphore control structure that is initialized.
 *
 *@retval #LOS_ERRNO_SEM_PTR_NULL     The passed-in puwSemHandle value is NULL.
 *@retval #LOS_ERRNO_SEM_OVERFLOW     The passed-in usCount value is greater than the maximum number of available semaphores.
 *@retval #LOS_ERRNO_SEM_ALL_BUSY     No semaphore control structure is available.
 *@retval #LOS_OK   The semaphore is successfully created.
 *@par Dependency:
 *<ul><li>los_sem.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_SemDelete
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_SemCreate(UINT16 usCount, UINT32 *puwSemHandle);

/**
 *@ingroup los_sem
 *@brief Create a binary semaphore.
 *
 *@par Description:
 *This API is used to create a binary semaphore control structure according to the initial number of available semaphores specified by usCount and return the ID of this semaphore control structure.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param usCount        [IN] Initial number of available semaphores. The value range is [0, 1].
 *@param puwSemHandle   [OUT] ID of the semaphore control structure that is initialized.
 *
 *@retval #LOS_ERRNO_SEM_PTR_NULL     The passed-in puwSemHandle value is NULL.
 *@retval #LOS_ERRNO_SEM_OVERFLOW     The passed-in usCount value is greater than the maximum number of available semaphores.
 *@retval #LOS_ERRNO_SEM_ALL_BUSY     No semaphore control structure is available.
 *@retval #LOS_OK   The semaphore is successfully created.
 *@par Dependency:
 *<ul><li>los_sem.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_SemDelete
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_BinarySemCreate (UINT16 usCount, UINT32 *puwSemHandle);

/**
 *@ingroup los_sem
 *@brief Delete a semaphore.
 *
 *@par Description:
 *This API is used to delete a semaphore control structure that has an ID specified by uwSemHandle.
 *@attention
 *<ul>
 *<li>The specified sem id must be created first. </li>
 *</ul>
 *
 *@param uwSemHandle   [IN] ID of the semaphore control structure to be deleted. The ID of the semaphore control structure is obtained from semaphore creation.
 *
 *@retval #LOS_ERRNO_SEM_INVALID  The passed-in uwSemHandle value is invalid.
 *@retval #LOS_ERRNO_SEM_PENDED   The queue of the tasks that are waiting on the semaphore control structure is not null.
 *@retval #LOS_OK   The semaphore control structure is successfully deleted.
 *@par Dependency:
 *<ul><li>los_sem.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_SemCreate
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_SemDelete(UINT32 uwSemHandle);

/**
 *@ingroup los_sem
 *@brief Request a semaphore.
 *
 *@par Description:
 *This API is used to request a semaphore based on the semaphore control structure ID specified by uwSemHandle and the parameter that specifies the timeout period.
 *@attention
 *<ul>
 *<li>The specified sem id must be created first. </li>
 *</ul>
 *
 *@param uwSemHandle   [IN] ID of the semaphore control structure to be requested. The ID of the semaphore control structure is obtained from semaphore creation.
 *@param uwTimeout     [IN] Timeout interval for waiting on the semaphore. The value range is [0, 0xFFFFFFFF]. If the value is set to 0, the semaphore is not waited on. If the value is set to 0xFFFFFFFF, the semaphore is waited on forever(unit: Tick).
 *
 *@retval #LOS_ERRNO_SEM_INVALID          The passed-in uwSemHandle value is invalid.
 *@retval #LOS_ERRNO_SEM_UNAVAILABLE      There is no available semaphore resource.
 *@retval #LOS_ERRNO_SEM_PEND_INTERR      The API is called during an interrupt, which is forbidden.
 *@retval #LOS_ERRNO_SEM_PEND_IN_LOCK     The task is unable to request a semaphore because task scheduling is locked.
 *@retval #LOS_ERRNO_SEM_TIMEOUT	 The request for the semaphore times out.
 *@retval #LOS_OK   The semaphore request succeeds.
 *@par Dependency:
 *<ul><li>los_sem.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_SemPost | LOS_SemCreate
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_SemPend(UINT32 uwSemHandle, UINT32 uwTimeout);

/**
 *@ingroup los_sem
 *@brief Release a semaphore.
 *
 *@par Description:
 *This API is used to release a semaphore that has a semaphore control structure ID specified by uwSemHandle.
 *@attention
 *<ul>
 *<li>The specified sem id must be created first. </li>
 *</ul>
 *
 *@param uwSemHandle   [IN] ID of the semaphore control structure to be released.The ID of the semaphore control structure is obtained from semaphore creation.
 *
 *@retval #LOS_ERRNO_SEM_INVALID      The passed-in uwSemHandle value is invalid.
 *@retval #LOS_ERRNO_SEM_OVERFLOW     The times of semaphore release exceed the maximum times permitted.
 *@retval #LOS_OK                     The semaphore is successfully released.
 *@par Dependency:
 *<ul><li>los_sem.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_SemPend | LOS_SemCreate
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_SemPost(UINT32 uwSemHandle);


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_SEM_H */
