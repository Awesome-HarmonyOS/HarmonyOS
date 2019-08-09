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

#ifndef _LOS_QUEUE_PH
#define _LOS_QUEUE_PH

#include "los_queue.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


/**
  * @ingroup los_queue
  * Queue information block structure
  */
typedef struct tagQueueCB
{
    UINT8       *pucQueue;              /**< Pointer to a queue handle */
    UINT16      usQueueState;           /**< Queue state */
    UINT16      usQueueLen;             /**< Queue length */
    UINT16      usQueueSize;            /**< Node size     */
    UINT16      usQueueHead;            /**< Node head       */
    UINT16      usQueueTail;            /**< Node tail       */
    UINT16      usWritableCnt;          /**< Count of writable resources   */
    UINT16      usReadableCnt;          /**< Count of readable resources   */
    UINT16      usReserved;             /**< Reserved         */
    LOS_DL_LIST stWriteList;            /**< Pointer to the linked list to be written   */
    LOS_DL_LIST stReadList;             /**< Pointer to the linked list to be read   */
    LOS_DL_LIST stMemList;              /**< Pointer to the memory linked list */
} QUEUE_CB_S;

/* queue state */
/**
  *  @ingroup los_queue
  *  Message queue state: not in use.
  */
#define OS_QUEUE_UNUSED                 0

/**
  *  @ingroup los_queue
  *  Message queue state: used.
  */
#define OS_QUEUE_INUSED     1

/**
  *  @ingroup los_queue
  *  Not in use.
  */
#define OS_QUEUE_WAIT_FOR_POOL          1

/**
  *  @ingroup los_queue
  *  Normal message queue.
  */
#define OS_QUEUE_NORMAL                 0

/**
  *  @ingroup los_queue
  *  Queue information control block
  */
extern QUEUE_CB_S *g_pstAllQueue;

/**
  *  @ingroup los_queue
  *  Obtain a handle of the queue that has a specified ID.
  */
#define GET_QUEUE_HANDLE(QueueID)       (((QUEUE_CB_S *)g_pstAllQueue) + (QueueID))

/**
  *  @ingroup los_queue
  *  The function exits and returns error code.
  */
#define GOTO_QUEUE_END(uwErrorNum) do { uwRet = uwErrorNum; \
                                    goto QUEUE_END; } while (0)

/**
 *@ingroup los_queue
 *@brief Alloc a stationary memory for a mail.
 *
 *@par Description:
 *This API is used to alloc a stationary memory for a mail according to uwQueueID.
 *@attention
 *<ul>
 *<li>Do not alloc memory in unblocking modes such as interrupt.</li>
 *<li>This API cannot be called before the Huawei LiteOS is initialized.</li>
 *<li>The argument uwTimeOut is a relative time.</li>
 *</ul>
 *
 *@param uwQueueID        [IN]        Queue ID. The value range is [1,LOSCFG_BASE_IPC_QUEUE_LIMIT].
 *@param pMailPool        [IN]        The memory poll that stores the mail.
 *@param uwTimeOut        [IN]        Expiry time. The value range is [0,LOS_WAIT_FOREVER].
 *
 *@retval   #NULL                     The memory allocation is failed.
 *@retval   #pMem                     The address of alloc memory.
 *@par Dependency:
 *<ul><li>los_queue.h: the header file that contains the API declaration.</li></ul>
 *@see osQueueMailFree
 *@since Huawei LiteOS V100R001C00
 */
extern VOID *osQueueMailAlloc(UINT32 uwQueueID, VOID *pMailPool, UINT32 uwTimeOut);

/**
 *@ingroup los_queue
 *@brief Free a stationary memory of a mail.
 *
 *@par Description:
 *This API is used to free a stationary memory for a mail according to uwQueueID.
 *@attention
 *<ul>
 *<li>This API cannot be called before the Huawei LiteOS is initialized.</li>
 *</ul>
 *
 *@param uwQueueID        [IN]        Queue ID. The value range is [1,LOSCFG_BASE_IPC_QUEUE_LIMIT].
 *@param pMailPool        [IN]        The mail memory poll address.
 *@param pMailMem         [IN]        The mail memory block address.
 *
 *@retval   #LOS_OK                                 0x00000000: The memory free successfully.
 *@retval   #OS_ERRNO_QUEUE_MAIL_HANDLE_INVALID     0x02000619: The handle of the queue passed-in when the memory for the queue is being freed is invalid.
 *@retval   #OS_ERRNO_QUEUE_MAIL_PTR_INVALID        0x0200061a: The pointer to the memory to be freed is null.
 *@retval   #OS_ERRNO_QUEUE_MAIL_FREE_ERROR         0x0200061b: The memory for the queue fails to be freed.
 *@par Dependency:
 *<ul><li>los_queue.h: the header file that contains the API declaration.</li></ul>
 *@see osQueueMailAlloc
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 osQueueMailFree(UINT32 uwQueueID, VOID *pMailPool, VOID *pMailMem);


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_QUEUE_PH */
