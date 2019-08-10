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

#ifndef _LOS_SWTMR_PH
#define _LOS_SWTMR_PH

#include "los_swtmr.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


/**
 * @ingroup los_swtmr
 * Software timer state
 */
enum enSwTmrState
{
    OS_SWTMR_STATUS_UNUSED,             /**< The software timer is not used.*/
    OS_SWTMR_STATUS_CREATED,            /**< The software timer is created.*/
    OS_SWTMR_STATUS_TICKING,            /**< The software timer is timing.*/
};

/**
 * @ingroup los_swtmr
 * Structure of the callback function that handles software timer timeout
 */
typedef struct tagSwTmrHandlerItem
{
    SWTMR_PROC_FUNC     pfnHandler;     /**< Callback function that handles software timer timeout          */
    UINT32              uwArg;          /**< Parameter passed in when the callback function that handles software timer timeout is called     */
} SWTMR_HANDLER_ITEM_S;

/**
 * @ingroup los_swtmr
 * Type of the pointer to the structure of the callback function that handles software timer timeout
 */
typedef SWTMR_HANDLER_ITEM_S    *SWTMR_HANDLER_ITEM_P;

extern SWTMR_CTRL_S             *m_pstSwtmrCBArray;

#define OS_SWT_FROM_SID(SwTmrID)    ((SWTMR_CTRL_S *)m_pstSwtmrCBArray + (SwTmrID % LOSCFG_BASE_CORE_SWTMR_LIMIT))

/**
 *@ingroup los_swtmr
 *@brief Scan a software timer.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to scan a software timer when a Tick interrupt occurs and determine whether the software timer expires.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param  None.
 *
 *@retval None.
 *@par Dependency:
 *<ul><li>los_swtmr.ph: the header file that contains the API declaration.</li></ul>
 *@see LOS_SwtmrStop
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 osSwtmrScan(VOID);

/**
 *@ingroup los_swtmr
 *@brief Initialization software timer.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to initialization software.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param  None.
 *
 *@retval None.
 *@par Dependency:
 *<ul><li>los_swtmr.ph: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 osSwTmrInit(VOID);

/**
 *@ingroup los_swtmr
 *@brief Get next timeout.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to get next timeout.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param  None.
 *
 *@retval None.
 *@par Dependency:
 *<ul><li>los_swtmr.ph: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 osSwTmrGetNextTimeout(VOID);

/**
 *@ingroup los_swtmr
 *@brief Adjust software timer list.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to adjust software timer list.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param  UINT32   Sleep time.
 *
 *@retval UINT32    Sleep time.
 *@par Dependency:
 *<ul><li>los_swtmr.ph: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern VOID osSwTmrAdjust(UINT32 uwSleepTime);

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_SWTMR_PH */
