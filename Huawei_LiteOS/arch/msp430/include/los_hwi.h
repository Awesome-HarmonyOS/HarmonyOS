/*----------------------------------------------------------------------------
 * Copyright (c) <2013-2018>, <Huawei Technologies Co., Ltd>
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

 /**@defgroup los_hwi Hardware interrupt
   *@ingroup kernel
 */
#ifndef _LOS_HWI_H
#define _LOS_HWI_H

#include "los_errno.h"
#include "stdint.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/**
 * @ingroup  los_hwi
 * Define the type of a hardware interrupt handling function.
 */

struct hwi_int_handle
{
    void   (* pfn) (uintptr_t);
    uintptr_t arg;
};


/**
 * @ingroup los_hwi
 * Count of interrupts.
 */
extern unsigned char g_int_cnt;

/**
 * @ingroup los_hwi
 * An interrupt is active.
 */
#define OS_INT_ACTIVE               (g_int_cnt > 0)


/**
 * @ingroup los_hwi
 * An interrupt is inactive.
 */
#define OS_INT_INACTIVE             (!(OS_INT_ACTIVE))


/**
 * @ingroup los_hwi
 * Hardware interrupt error code: Invalid interrupt number.
 *
 * Value: 0x02000900
 *
 * Solution: Ensure that the interrupt number is valid. The value range of the interrupt vector is [0, RESET_VECTOR], the defination for vector numbers can be found at the SOC header file, for example: msp430f5438a.h.
 */
#define OS_ERRNO_HWI_NUM_INVALID                            LOS_ERRNO_OS_ERROR(LOS_MOD_HWI, 0x00)

/**
 * @ingroup los_hwi
 * Hardware interrupt error code: Null hardware interrupt handling function.
 *
 * Value: 0x02000901
 *
 * Solution: Pass in a valid non-null hardware interrupt handling function.
 */
#define OS_ERRNO_HWI_PROC_FUNC_NULL                         LOS_ERRNO_OS_ERROR(LOS_MOD_HWI, 0x01)

/**
 * @ingroup los_hwi
 * Hardware interrupt error code: The interrupt has already been created.
 *
 * Value: 0x02000904
 *
 * Solution: Check whether the interrupt specified by the passed-in interrupt number has already been created.
 */
#define OS_ERRNO_HWI_ALREADY_CREATED                        LOS_ERRNO_OS_ERROR(LOS_MOD_HWI, 0x04)

/**
 * @ingroup  los_hwi
 * @brief Create a hardware interrupt.
 *
 * @par Description:
 * This API is used to configure a hardware interrupt and register a hardware interrupt handling function.
 *
 * @attention
 * <ul>
 * <li>Hardware interrupt vector number value range: [0, RESET_VECTOR]. The usable vector numbers can be found at the SOC header files, for example: msp430f5438a.h.</li>
 * <li>Before executing an interrupt on a platform, refer to the chip manual of the platform.</li>
 * </ul>
 *
 * @param  vector [IN] Type#int: hardware interrupt vector number. For example: WDT_VECTOR, RESET_VECTOR.
 * @param  prio   [IN] Type#int: hardware interrupt priority. Not used.
 * @param  mode   [IN] Type#int: hardware interrupt mode. Not used.
 * @param  pfn    [IN] Type#void (*) (uintptr_t): interrupt handler used when a hardware interrupt is triggered.
 * @param  arg    [IN] Type#uintptr_t: input parameter of the interrupt handler used when a hardware interrupt is triggered.
 *
 * @retval #OS_ERRNO_HWI_NUM_INVALID                  0x02000900: Invalid interrupt number.
 * @retval #OS_ERRNO_HWI_PROC_FUNC_NULL               0x02000901: Null hardware interrupt handling function.
 * @retval #OS_ERRNO_HWI_ALREADY_CREATED              0x02000904: The interrupt handler being created has already been created.
 * @retval #LOS_OK                                    0x00000000: The interrupt is successfully created.
 * @par Dependency:
 * <ul><li>los_hwi.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_HwiCreate (int irq,int prio, int mode,
                             void (*pfn) (uintptr_t),
                             uintptr_t arg);

/**
 *@ingroup los_hwi
 *@brief Enable all interrupts.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to enable all interrupts.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param None.
 *
 *@retval original interrupt mask status.
 *@par Dependency:
 *<ul><li>los_hwi.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_IntRestore
 *@since Huawei LiteOS V100R001C00
 */
extern uintptr_t LOS_IntUnLock (void);


/**
 *@ingroup los_hwi
 *@brief Disable all interrupts.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to disable all interrupts.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param None.
 *
 *@retval original interrupt mask status.
 *@par Dependency:
 *<ul><li>los_hwi.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_IntRestore
 *@since Huawei LiteOS V100R001C00
 */
extern uintptr_t LOS_IntLock (void);


/**
 *@ingroup los_hwi
 *@brief Restore interrupts.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to restore the interrupt mask status.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>This API can be called only after all interrupts are disabled, and the input parameter value should be the value returned by calling the all interrupt disabling API.</li>
 *</ul>
 *
 *@param mask [IN] new interrupt mask.
 *
 *@retval None.
 *@par Dependency:
 *<ul><li>los_hwi.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_IntLock
 *@since Huawei LiteOS V100R001C00
 */
extern void LOS_IntRestore (uintptr_t mask);


/**
 * @ingroup  los_hwi
 * @brief Delete hardware interrupt.
 *
 * @par Description:
 * This API is used to delete hardware interrupt vector handler.
 *
 * @attention
 * <ul>
 * <li>Hardware interrupt vector value range: [0, RESET_VECTOR].</li>
 * <li>RESET_VECTOR specifies the maximum number of interrupts that can be created.</li>
 * <li>Before executing an interrupt on a platform, refer to the chip manual of the platform.</li>
 * </ul>
 *
 * @param  vector [IN] Type#int: hardware interrupt vector number.
 *
 * @retval #OS_ERRNO_HWI_NUM_INVALID              0x02000900: Invalid interrupt number.
 * @retval #LOS_OK                                0x00000000: The interrupt is successfully delete.
 * @par Dependency:
 * <ul><li>los_hwi.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_HwiDelete (int vector);

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_HWI_H */

