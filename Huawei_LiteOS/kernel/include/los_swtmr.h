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

/** @defgroup los_swtmr Software timer
 * @ingroup kernel
 */

#ifndef _LOS_SWTMR_H
#define _LOS_SWTMR_H

#include "los_base.h"
#include "los_task.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


/**
 * @ingroup los_swtmr
 * Software timer error code: The timeout handling function is NULL.
 *
 * Value: 0x02000300
 *
 * Solution: Define the timeout handling function.
 */
#define LOS_ERRNO_SWTMR_PTR_NULL                     LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x00)

/**
 * @ingroup los_swtmr
 * Software timer error code: The expiration time is 0.
 *
 * Value: 0x02000301
 *
 * Solution: Re-define the expiration time.
 */
#define LOS_ERRNO_SWTMR_INTERVAL_NOT_SUITED          LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x01)

/**
 * @ingroup los_swtmr
 * Software timer error code: Invalid software timer mode.
 *
 * Value: 0x02000302
 *
 * Solution: Check the mode value. The value range is [0,3].
 */
#define LOS_ERRNO_SWTMR_MODE_INVALID                 LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x02)

/**
 * @ingroup los_swtmr
 * Software timer error code: The passed-in software timer ID is NULL.
 *
 * Value: 0x02000303
 *
 * Solution: Define the software timer ID before passing it in.
 */
#define LOS_ERRNO_SWTMR_RET_PTR_NULL                 LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x03)

/**
 * @ingroup los_swtmr
 * Software timer error code: The number of software timers exceeds the configured permitted maximum number.
 *
 * Value: 0x02000304
 *
 * Solution: Re-configure the permitted maximum number of software timers, or wait for a software timer to become available.
 */
#define LOS_ERRNO_SWTMR_MAXSIZE                      LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x04)

/**
 * @ingroup los_swtmr
 * Software timer error code: Invalid software timer ID.
 *
 * Value: 0x02000305
 *
 * Solution: Pass in a valid software timer ID.
 */
#define LOS_ERRNO_SWTMR_ID_INVALID                   LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x05)

/**
 * @ingroup los_swtmr
 * Software timer error code: The software timer is not created.
 *
 * Value: 0x02000306
 *
 * Solution: Create a software timer.
 */
#define LOS_ERRNO_SWTMR_NOT_CREATED                  LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x06)

/**
 * @ingroup los_swtmr
 * Software timer error code: Insufficient memory for software timer linked list creation.
 *
 * Value: 0x02000307
 *
 * Solution: Allocate bigger memory partition to software timer linked list creation.
 */
#define LOS_ERRNO_SWTMR_NO_MEMORY                    LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x07)

/**
 * @ingroup los_swtmr
 * Software timer error code: Invalid configured number of software timers.
 *
 * Value: 0x02000308
 *
 * Solution: Re-configure the number of software timers.
 */
#define LOS_ERRNO_SWTMR_MAXSIZE_INVALID              LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x08)

/**
 * @ingroup los_swtmr
 * Software timer error code: The software timer is being used during an interrupt.
 *
 * Value: 0x02000309
 *
 * Solution: Change the source code and do not use the software timer during an interrupt.
 */
#define LOS_ERRNO_SWTMR_HWI_ACTIVE                   LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x09)

/**
 * @ingroup los_swtmr
 * Software timer error code: Insufficient memory allocated by membox.
 *
 * Value: 0x0200030a
 *
 * Solution: Expand the memory allocated by membox.
 */
#define LOS_ERRNO_SWTMR_HANDLER_POOL_NO_MEM          LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x0a)

/**
 * @ingroup los_swtmr
 * Software timer error code: The software timer queue fails to be created.
 *
 * Value: 0x0200030b
 *
 * Solution: Check whether more memory can be allocated to the queue to be created.
 */
#define LOS_ERRNO_SWTMR_QUEUE_CREATE_FAILED          LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x0b)

/**
 * @ingroup los_swtmr
 * Software timer error code: The software timer task fails to be created.
 *
 * Value: 0x0200030c
 *
 * Solution: Check whether the memory is sufficient and re-create the task.
 */
#define LOS_ERRNO_SWTMR_TASK_CREATE_FAILED           LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x0c)

/**
 * @ingroup los_swtmr
 * Software timer error code: The software timer is not started.
 *
 * Value: 0x0200030d
 *
 * Solution: Start the software timer.
 */
#define LOS_ERRNO_SWTMR_NOT_STARTED                  LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x0d)

/**
 * @ingroup los_swtmr
 * Software timer error code: Invalid software timer state.
 *
 * Value: 0x0200030e
 *
 * Solution: Check the software timer state.
 */
#define LOS_ERRNO_SWTMR_STATUS_INVALID               LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x0e)

/**
 * @ingroup los_swtmr
 * This error code is not in use temporarily.
 */
#define LOS_ERRNO_SWTMR_SORTLIST_NULL                LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x0f)

/**
 * @ingroup los_swtmr
 * Software timer error code: The passed-in number of remaining Ticks configured on the software timer is NULL.
 *
 * Value: 0x02000310
 *
 * Solution: Define a variable of the number of remaining Ticks before passing in the number of remaining Ticks.
 */
#define LOS_ERRNO_SWTMR_TICK_PTR_NULL                LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x10)

#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
#define OS_ERRNO_SWTMR_ROUSES_INVALID               LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x11)
#define OS_ERRNO_SWTMR_ALIGN_INVALID                LOS_ERRNO_OS_ERROR(LOS_MOD_SWTMR, 0x12)

#define OS_SWTMR_BIT_ALREADY_ALIGNED                (1 << 31)
#define OS_SWTMR_BIT_CAN_ALIGNED                    (1 << 30)
#define OS_SWTMR_BIT_MULTIPLE                       (1 << 29)

#define SET_ALIGN_SWTMR_ALREADY_ALIGNED(num)                ((num) = 0x80000000 | ((num) & 0x7FFFFFFF))
#define SET_ALIGN_SWTMR_ALREADY_NOT_ALIGNED(num)            ((num) = ((num) & 0x7FFFFFFF))
#define SET_ALIGN_SWTMR_CAN_ALIGNED(num)                    ((num) = 0x40000000| (num & 0xBFFFFFFF))
#define SET_ALIGN_SWTMR_CAN_NOT_ALIGNED(num)                ((num) = (num & 0xBFFFFFFF))
#define SET_ALIGN_SWTMR_CAN_MULTIPLE(num)                   ((num)= 0x20000000|(num & 0xDFFFFFFF) )
#define SET_ALIGN_SWTMR_CAN_NOT_MULTIPLE(num)               ((num)= (num & 0xDFFFFFFF) )
#define SET_ALIGN_SWTMR_DIVISOR_TIMERS(num, value)          (num = (value & 0x00FFFFFF) | (num & 0xFF000000))//bit23:0
#define GET_ALIGN_SWTMR_DIVISOR_TIMERS(num)                 (num & 0x00FFFFFF)

#define CHECK_ALIGN_SWTMR_CAN_MULTI_ALIGN(num)              ((num & 0xE0000000) == 0x60000000)
#define CHECK_ALIGN_SWTMR_CAN_PERIODIC_ALIGN(num)           ((num & 0xE0000000) == 0x40000000)
#define CHECK_ALIGN_SWTMR_ALREADY_ALIGN(num)                ((num & 0xC0000000) == 0xC0000000)

enum enSwTmrRousesType
{
    OS_SWTMR_ROUSES_IGNORE,    /* timer don't need to wake up system */
    OS_SWTMR_ROUSES_ALLOW,     /* timer can wake up system */
};

enum enSwTmrAlignSensitive
{
    OS_SWTMR_ALIGN_SENSITIVE,         /* timer don't need to align  */
    OS_SWTMR_ALIGN_INSENSITIVE,     /* timer need to align */
};
#endif

/**
 * @ingroup los_swtmr
 * Software timer mode
 */
enum enSwTmrType
{
    LOS_SWTMR_MODE_ONCE,                 /**< One-off software timer */
    LOS_SWTMR_MODE_PERIOD,               /**< Periodic software timer */
    LOS_SWTMR_MODE_NO_SELFDELETE,        /**< One-off software timer, but not self-delete */
    LOS_SWTMR_MODE_OPP,                  /**< After the one-off timer finishes timing, the periodic software timer is enabled. This mode is not supported temporarily.*/
};

/**
* @ingroup  los_swtmr
* @brief Define the type of a callback function that handles software timer timeout.
*
* @par Description:
* This API is used to define the type of a callback function that handles software timer timeout, so that it can be called when software timer timeout.
*
* @attention
* <ul>
* <li>None.</li>
* </ul>
*
* @param  uwPar     [IN] the parameter of the callback function that handles software timer timeout.
*
* @retval None.
* @par Dependency:
* <ul><li>los_swtmr.h: the header file that contains the API declaration.</li></ul>
* @see None.
* @since Huawei LiteOS V100R001C00
*/
typedef VOID (*SWTMR_PROC_FUNC)(UINT32 uwPar);

/**
 * @ingroup los_swtmr
 * Software timer control structure
 */
typedef struct tagSwTmrCtrl
{
    struct tagSwTmrCtrl *pstNext;       /**< Pointer to the next software timer                      */
    UINT8               ucState;        /**< Software timer state                                    */
    UINT8               ucMode;         /**< Software timer mode                                     */
#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
    UINT8               ucRouses;       /*wake up enable                                             */
    UINT8               ucSensitive;    /*align enable                                               */
#endif
    UINT16              usTimerID;      /**< Software timer ID                                       */
    UINT32              uwCount;        /**< Times that a software timer works                       */
    UINT32              uwInterval;     /**< Timeout interval of a periodic software timer           */
    UINT32              uwArg;          /**< Parameter passed in when the callback function that handles software timer timeout is called */
    SWTMR_PROC_FUNC     pfnHandler;     /**< Callback function that handles software timer timeout   */
} SWTMR_CTRL_S;


/**
 *@ingroup los_swtmr
 *@brief Start a software timer.
 *
 *@par Description:
 *This API is used to start a software timer that has a specified ID.
 *@attention
 *<ul>
 *<li>The specific timer must be created first</li>
 *</ul>
 *
 *@param  usSwTmrID  [IN] Software timer ID created by LOS_SwtmrCreate. The value of ID should be in [0, LOSCFG_BASE_CORE_SWTMR_LIMIT - 1].
 *
 *@retval #LOS_ERRNO_SWTMR_ID_INVALID       Invalid software timer ID.
 *@retval #LOS_ERRNO_SWTMR_NOT_CREATED      The software timer is not created.
 *@retval #LOS_ERRNO_SWTMR_STATUS_INVALID   Invalid software timer state.
 *@retval #LOS_OK                           The software timer is successfully started.
 *@par Dependency:
 *<ul><li>los_swtmr.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_SwtmrStop | LOS_SwtmrCreate
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_SwtmrStart(UINT16 usSwTmrID);

/**
 *@ingroup los_swtmr
 *@brief Stop a software timer.
 *
 *@par Description:
 *This API is used to stop a software timer that has a specified ID.
 *@attention
 *<ul>
 *<li>The specific timer should be created and started firstly.</li>
 *</ul>
 *
 *@param  usSwTmrID  [IN] Software timer ID created by LOS_SwtmrCreate. The value of ID should be in [0, LOSCFG_BASE_CORE_SWTMR_LIMIT - 1].
 *
 *@retval #LOS_ERRNO_SWTMR_ID_INVALID       Invalid software timer ID.
 *@retval #LOS_ERRNO_SWTMR_NOT_CREATED      The software timer is not created.
 *@retval #LOS_ERRNO_SWTMR_NOT_STARTED      The software timer is not started.
 *@retval #LOS_ERRNO_SWTMR_STATUS_INVALID   Invalid software timer state.
 *@retval #LOS_OK                           The software timer is successfully stopped.
 *@par Dependency:
 *<ul><li>los_swtmr.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_SwtmrStart | LOS_SwtmrCreate
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_SwtmrStop(UINT16 usSwTmrID);

/**
 *@ingroup los_swtmr
 *@brief Obtain the number of remaining Ticks configured on a software timer.
 *
 *@par Description:
 *This API is used to obtain the number of remaining Ticks configured on the software timer of which the ID is specified by usSwTmrID.
 *@attention
 *<ul>
 *<li>The specific timer should be created and started successfully, error happends otherwise.</li>
 *</ul>
 *
 *@param  usSwTmrID  [IN]  Software timer ID created by LOS_SwtmrCreate. The value of ID should be in [0, LOSCFG_BASE_CORE_SWTMR_LIMIT - 1].
 *@param  uwTick     [OUT] Number of remaining Ticks configured on the software timer.
 *
 *@retval #LOS_ERRNO_SWTMR_ID_INVALID      Invalid software timer ID.
 *@retval #LOS_ERRNO_SWTMR_NOT_CREATED     The software timer is not created.
 *@retval #LOS_ERRNO_SWTMR_NOT_STARTED     The software timer is not started.
 *@retval #LOS_ERRNO_SWTMR_STATUS_INVALID  Invalid software timer state.
 *@retval #LOS_OK                          The number of remaining Ticks is successfully obtained.
 *@par Dependency:
 *<ul><li>los_swtmr.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_SwtmrCreate
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_SwtmrTimeGet(UINT16 usSwTmrID, UINT32 *uwTick);

/**
 *@ingroup los_swtmr
 *@brief Create a software timer.
 *
 *@par Description:
 *This API is used to create a software timer that has specified timing duration, timeout handling function, and trigger mode, and to return a handle by which the software timer can be referenced.
 *@attention
 *<ul>
 *<li>Do not use the delay interface in the callback function that handles software timer timeout.</li>
 *<li>Threre are LOSCFG_BASE_CORE_SWTMR_LIMIT timers available, change it's value when necessory.</li>
 *</ul>
 *
 *@param  uwInterval     [IN] Timing duration of the software timer to be created (unit: ms).
 *@param  ucMode         [IN] Software timer mode. Pass in one of the modes specified by enSwTmrType. There are three types of modes, one-off, periodic, and continuously periodic after one-off, of which the third mode is not supported temporarily.
 *@param  pfnHandler     [IN] Callback function that handles software timer timeout.
 *@param  pusSwTmrID     [OUT] Software timer ID created by LOS_SwtmrCreate.
 *@param  uwArg          [IN] Parameter passed in when the callback function that handles software timer timeout is called.
 *
 *@retval #LOS_ERRNO_SWTMR_INTERVAL_NOT_SUITED   The software timer timeout interval is 0.
 *@retval #LOS_ERRNO_SWTMR_MODE_INVALID          Invalid software timer mode.
 *@retval #LOS_ERRNO_SWTMR_PTR_NULL              The callback function that handles software timer timeout is NULL.
 *@retval #LOS_ERRNO_SWTMR_RET_PTR_NULL          The passed-in software timer ID is NULL.
 *@retval #LOS_ERRNO_SWTMR_MAXSIZE               The number of software timers exceeds the configured permitted maximum number.
 *@retval #LOS_OK                                The software timer is successfully created.
 *@par Dependency:
 *<ul><li>los_swtmr.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_SwtmrDelete
 *@since Huawei LiteOS V100R001C00
 */
 extern UINT32 LOS_SwtmrCreate(UINT32 uwInterval, UINT8 ucMode, SWTMR_PROC_FUNC pfnHandler, UINT16 *pusSwTmrID, UINT32 uwArg
                                          #if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
                                          , UINT8 ucRouses, UINT8 ucSensitive
                                          #endif
                                          );

/**
 *@ingroup los_swtmr
 *@brief Delete a software timer.
 *
 *@par Description:
 *This API is used to delete a software timer.
 *@attention
 *<ul>
 *<li>The specific timer should be created and then stopped firstly.</li>
 *</ul>
 *
 *@param  usSwTmrID     [IN] Software timer ID created by LOS_SwtmrCreate. The value of ID should be in [0, LOSCFG_BASE_CORE_SWTMR_LIMIT - 1].
 *
 *@retval #LOS_ERRNO_SWTMR_ID_INVALID        Invalid software timer ID.
 *@retval #LOS_ERRNO_SWTMR_NOT_CREATED       The software timer is not created.
 *@retval #LOS_ERRNO_SWTMR_STATUS_INVALID    Invalid software timer state.
 *@retval #LOS_OK                            The software timer is successfully deleted.
 *@par Dependency:
 *<ul><li>los_swtmr.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_SwtmrCreate
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_SwtmrDelete(UINT16 usSwTmrID);


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_SWTMR_H */
