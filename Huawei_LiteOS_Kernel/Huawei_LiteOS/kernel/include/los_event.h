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

/**@defgroup los_event Event
 * @ingroup kernel
 */

#ifndef _LOS_EVENT_H
#define _LOS_EVENT_H

#include "los_base.h"
#include "los_list.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


/**
 * @ingroup los_event
 * Event reading mode: The task waits for all its expected events to occur.
 */
#define LOS_WAITMODE_AND         (4) /* all bits must be set */

/**
 * @ingroup los_event
 * Event reading mode: The task waits for any of its expected events to occur.
 */
#define LOS_WAITMODE_OR          (2) /* any bit must be set  */

/**
 * @ingroup los_event
 * Event reading mode: The event flag is immediately cleared after the event is read.
 */
#define LOS_WAITMODE_CLR         (1) /* clear when satisfied */

/**
 * @ingroup los_event
 * Bit 25 of the event mask cannot be set to an event because it is set to an error code.
 *
 * Value: 0x02001c00
 *
 * Solution: Set bits excluding bit 25 of the event mask to events.
 */
#define LOS_ERRNO_EVENT_SETBIT_INVALID                      LOS_ERRNO_OS_ERROR(LOS_MOD_EVENT, 0x00)
/**
 * @ingroup los_event
 * Event reading error code: Event reading times out.
 *
 * Value: 0x02001c01
 *
 * Solution: Increase the waiting time for event reading, or make another task write a mask for the event.
 */
#define LOS_ERRNO_EVENT_READ_TIMEOUT                        LOS_ERRNO_OS_ERROR(LOS_MOD_EVENT, 0x01)

/**
 * @ingroup los_event
 * Event reading error code: The EVENTMASK input parameter value is valid. The input parameter value must not be 0.
 *
 * Value: 0x02001c02
 *
 * Solution: Pass in a valid EVENTMASK value.
 */
#define LOS_ERRNO_EVENT_EVENTMASK_INVALID                   LOS_ERRNO_OS_ERROR(LOS_MOD_EVENT, 0x02)

/**
 * @ingroup los_event
 * Event reading error code: The event is being read during an interrupt.
 *
 * Value: 0x02001c03
 *
 * Solution: Read the event in a task.
 */
#define LOS_ERRNO_EVENT_READ_IN_INTERRUPT                   LOS_ERRNO_OS_ERROR(LOS_MOD_EVENT, 0x03)

/**
 * @ingroup los_event
 * Event reading error code: The uwFlags input parameter value used in the event reading API is invalid. This input parameter value is obtained by performing an OR operation on corresponding bits of either OS_EVENT_ANY or OS_EVENT_ANY and corresponding bits of either OS_EVENT_WAIT or OS_EVENT_NOWAIT. The waiting time must be set to a nonzero value when an event is read in the mode of OS_EVENT_WAIT.
 *
 * Value: 0x02001c04
 *
 * Solution: Pass in a valid uwFlags value.
 */
#define LOS_ERRNO_EVENT_FLAGS_INVALID                       LOS_ERRNO_OS_ERROR(LOS_MOD_EVENT, 0x04)

/**
 * @ingroup los_event
 * Event reading error code: The task is locked and is unable to read the event.
 *
 * Value: 0x02001c05
 *
 * Solution: Unlock the task and read the event.
 */
#define LOS_ERRNO_EVENT_READ_IN_LOCK                        LOS_ERRNO_OS_ERROR(LOS_MOD_EVENT, 0x05)

/**
 * @ingroup los_event
 * Event reading error code: Null pointer.
 *
 * Value: 0x02001c06
 *
 * Solution: Check whether the input parameter is null.
 */
#define LOS_ERRNO_EVENT_PTR_NULL                            LOS_ERRNO_OS_ERROR(LOS_MOD_EVENT, 0x06)

/**
 * @ingroup los_event
 * Event control structure
 */
typedef struct tagEvent
{
    UINT32      uwEventID;      /**< Event mask in the event control block, indicating the event that has been logically processed.*/
    LOS_DL_LIST stEventList;    /**< Event control block linked list*/
} EVENT_CB_S, *PEVENT_CB_S;

/**
 *@ingroup los_event
 *@brief Initialize an event control block.
 *
 *@par Description:
 *This API is used to initialize the event control block pointed to by pstEventCB.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pstEventCB [OUT] Pointer to the event control block to be initialized.
 *
 *@retval #LOS_ERRNO_EVENT_PTR_NULL 0x02001c06: Null pointer.
 *@retval #LOS_OK  0: The event control block is successfully initialized.
 *@par Dependency:
 *<ul><li>los_event.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_EventClear
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_EventInit(PEVENT_CB_S pstEventCB);

/**
 *@ingroup los_event
 *@brief Obtain an event specified by the event ID.
 *
 *@par Description:
 *This API is used to check whether an event expected by the user occurs according to the event ID, event mask, and event reading mode, and process the event based on the event reading mode. The event ID must point to valid memory.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param uwEventID      [IN/OUT] Pointer to the ID of the event to be checked.
 *@param uwEventMask    [IN] Mask of the event expected to occur by the user, indicating the event obtained after it is logically processed that matches the ID pointed to by uwEventID.
 *@param uwMode         [IN] Event reading mode. The modes include LOS_WAIT_FOREVER, LOS_WAITMODE_AND, LOS_WAITMODE_OR, LOS_WAITMODE_CLR, OS_WAITMODE_NOWAIT.
 *
 *@retval 0     The event expected by the user does not occur.
 *@retval Nonzero   The event expected by the user occurs.
 *@par Dependency:
 *<ul><li>los_event.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_EventRead | LOS_EventWrite
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_EventPoll(UINT32 *uwEventID, UINT32 uwEventMask, UINT32 uwMode);

/**
 *@ingroup los_event
 *@brief Read an event.
 *
 *@par Description:
 *This API is used to block or schedule a task that reads an event of which the event control block, event mask, reading mode, and timeout information are specified.
 *</ul>
 *@attention
 *<ul>
 *<li>An error code and an event return value can be same. To differentiate the error code and return value, bit 25 of the event mask is forbidden to be used.</li>
 *</ul>
 *
 *@param pstEventCB     [IN/OUT] Pointer to the event control block to be checked. This parameter must point to valid memory.
 *@param uwEventMask    [IN] Mask of the event expected to occur by the user, indicating the event obtained after it is logically processed that matches the ID pointed to by uwEventID.
 *@param uwMode         [IN] Event reading mode.
 *@param uwTimeOut      [IN] Timeout interval of event reading (unit: Tick).
 *
 *@retval #LOS_ERRNO_EVENT_SETBIT_INVALID    0x02001c00: Bit 25 of the event mask cannot be set because it is set to an error number.
 *@retval #LOS_ERRNO_EVENT_EVENTMASK_INVALID 0x02001c02: The passed-in event reading mode is incorrect.
 *@retval #LOS_ERRNO_EVENT_READ_IN_INTERRUPT 0x02001c03: The event is being read during an interrupt.
 *@retval #LOS_ERRNO_EVENT_FLAGS_INVALID     0x02001c04: The event mode is invalid.
 *@retval #LOS_ERRNO_EVENT_READ_IN_LOCK      0x02001c05: The event reading task is locked.
 *@retval #LOS_ERRNO_EVENT_PTR_NULL          0x02001c06: The passed-in pointer is null.
 *@retval 0     The event expected by the user does not occur.
 *@retval Nonzero   The event expected by the user occurs.
 *@par Dependency:
 *<ul><li>los_event.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_EventPoll | LOS_EventWrite
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_EventRead(PEVENT_CB_S pstEventCB, UINT32 uwEventMask, UINT32 uwMode, UINT32 uwTimeOut);

/**
 *@ingroup los_event
 *@brief Write an event.
 *
 *@par Description:
 *This API is used to write an event specified by the passed-in event mask into an event control block pointed to by pstEventCB.
 *@attention
 *<ul>
 *<li>To determine whether the LOS_EventRead API returns an event or an error code, bit 25 of the event mask is forbidden to be used.</li>
 *</ul>
 *
 *@param pstEventCB [IN/OUT] Pointer to the event control block into which an event is to be written. This parameter must point to valid memory.
 *@param uwEvents   [IN] Event mask to be written.
 *
 *@retval #LOS_ERRNO_EVENT_SETBIT_INVALID 0x02001c00: Bit 25 of the event mask cannot be set to an event because it is set to an error code.
 *@retval #LOS_ERRNO_EVENT_PTR_NULL 0x02001c06: Null pointer.
 *@retval #LOS_OK  0: The event is successfully written.
 *@par Dependency:
 *<ul><li>los_event.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_EventPoll | LOS_EventRead
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_EventWrite(PEVENT_CB_S pstEventCB, UINT32 uwEvents);

/**
 *@ingroup los_event
 *@brief Clear the event occurring in a specified task.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to set the ID of an event that has a specified mask and of which the information is stored in an event control block pointed to by pstEventCB to 0. pstEventCB must point to valid memory.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>uwEvents The value of uwEvents needs to be reversed when it is passed-in.</li>
 *</ul>
 *
 *@param pstEventCB     [IN/OUT] Pointer to the event control block to be cleared.
 *@param uwEvents       [IN] Mask of the event to be cleared.
 *
 *@retval #LOS_ERRNO_EVENT_PTR_NULL 0x02001c06: Null pointer.
 *@retval #LOS_OK 0: The event is successfully cleared.
 *@par Dependency:
 *<ul><li>los_event.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_EventPoll | LOS_EventRead，LOS_EventWrite
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_EventClear(PEVENT_CB_S pstEventCB, UINT32 uwEvents);

/**
 *@ingroup los_event
 *@brief Destory a event.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to Destory a event.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None</li>
 *</ul>
 *
 *@param pstEventCB     [IN/OUT] Pointer to the event control block to be Destoryed.
 *
 *@retval #LOS_ERRNO_EVENT_PTR_NULL 0x02001c06: Null pointer.
 *@retval #LOS_OK 0: The event is successfully cleared.
 *@par Dependency:
 *<ul><li>los_event.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_EventPoll | LOS_EventRead，LOS_EventWrite
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_EventDestory(PEVENT_CB_S pstEventCB);


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_EVENT_H */
