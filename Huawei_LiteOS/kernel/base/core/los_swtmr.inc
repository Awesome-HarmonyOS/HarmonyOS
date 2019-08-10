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

#ifndef _LOS_SWTMR_INC
#define _LOS_SWTMR_INC

#include "los_swtmr.ph"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


extern SWTMR_CTRL_S *m_pstSwtmrFreeList;

/**
 * @ingroup los_swtmr
 * Handle function of software timer task .
 *
 * @par Description:
 * <ul>
 * <li>This API is used to handle the overtime software timer.</li>
 * </ul>
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  None.
 *
 * @retval None.
 * @par Dependency:
 * <ul><li>los_swtmr.inc: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern VOID osSwTmrTask(VOID);

/**
 * @ingroup los_swtmr
 * Create the task of software timer.
 *
 * @par Description:
 * <ul>
 * <li>This API is used to create the task of software timer.</li>
 * </ul>
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  None.
 *
 * @retval UINT32 Create result.
 * @par Dependency:
 * <ul><li>los_swtmr.inc: the header file that contains the API declaration.</li></ul>
 * @see osSwtmrDelete
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 osSwTmrTaskCreate(VOID);

/**
 * @ingroup los_swtmr
 * Start a software timer.
 *
 * @par Description:
 * <ul>
 * <li>This API is used to start a software timer.</li>
 * </ul>
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstSwtmr [IN] Type #SWTMR_CTRL_S * pointer to swtmr control block structure.
 *
 * @retval None.
 * @par Dependency:
 * <ul><li>los_swtmr.inc: the header file that contains the API declaration.</li></ul>
 * @see osSwtmrStop
 * @since Huawei LiteOS V100R001C00
 */
extern VOID osSwTmrStart(SWTMR_CTRL_S *pstSwtmr);

/**
 * @ingroup los_swtmr
 * Stop a software timer.
 *
 * @par Description:
 * <ul>
 * <li>This API is used to stop a software timer.</li>
 * </ul>
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstSwtmr [IN] Type #SWTMR_CTRL_S * pointer to swtmr control block structure.
 *
 * @retval None.
 * @par Dependency:
 * <ul><li>los_swtmr.inc: the header file that contains the API declaration.</li></ul>
 * @see osSwTmrStart
 * @since Huawei LiteOS V100R001C00
 */
extern VOID osSwtmrStop(SWTMR_CTRL_S *pstSwtmr);

/**
 * @ingroup los_swtmr
 * Insert a software timer to timer list.
 *
 * @par Description:
 * <ul>
 * <li>This API is used to insert a software timer to timer list.</li>
 * </ul>
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstHead  [OUT] Type #SWTMR_CTRL_S * pointer to swtmr control block structure head.
 * @param  pstSwtmr [IN] Type #SWTMR_CTRL_S * pointer to swtmr control block structure.
 *
 * @retval None.
 * @par Dependency:
 * <ul><li>los_swtmr.inc: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern VOID osSwtimerInsert(SWTMR_CTRL_S **pstHead, SWTMR_CTRL_S *pstSwtmr);

/**
 * @ingroup los_swtmr
 * Obtain the number of remaining Ticks configured on a software timer.
 *
 * @par Description:
 * <ul>
 * <li>This API is used to obtain the number of remaining Ticks configured on the software timer.</li>
 * </ul>
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstSwtmr [IN] Type #SWTMR_CTRL_S * pointer to swtmr control block structure.
 *
 * @retval UINT32  Ticks.
 * @par Dependency:
 * <ul><li>los_swtmr.inc: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 osSwtmrTimeGet(SWTMR_CTRL_S *pstSwtmr);

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_SWTMR_INC */
