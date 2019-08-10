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

/**@defgroup los_tick Tick
 * @ingroup kernel
 */

#ifndef _LOS_TICK_H
#define _LOS_TICK_H

#include "los_errno.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


/**
 * @ingroup los_tick
 * Tick error code: The Tick configuration is incorrect.
 *
 * Value: 0x02000400
 *
 * Solution: Change values of the OS_SYS_CLOCK and LOSCFG_BASE_CORE_TICK_PER_SECOND system time configuration modules in Los_config.h.
 */
#define LOS_ERRNO_TICK_CFG_INVALID                  LOS_ERRNO_OS_ERROR(LOS_MOD_TICK, 0x00)

/**
 * @ingroup los_tick
 * Tick error code: This error code is not in use temporarily.
 *
 * Value: 0x02000401
 *
 * Solution: None.
 */
#define LOS_ERRNO_TICK_NO_HWTIMER                   LOS_ERRNO_OS_ERROR(LOS_MOD_TICK, 0x01)

/**
 * @ingroup los_tick
 * Tick error code: The number of Ticks is too small.
 *
 * Value: 0x02000402
 *
 * Solution: Change values of the OS_SYS_CLOCK and LOSCFG_BASE_CORE_TICK_PER_SECOND system time configuration modules according to the SysTick_Config function.
 */
#define LOS_ERRNO_TICK_PER_SEC_TOO_SMALL            LOS_ERRNO_OS_ERROR(LOS_MOD_TICK, 0x02)

/**
 *  @ingroup  los_tick
 *  @brief: System clock get function.
 *
 *  @par Description:
 *  This API is used to get system clock.
 *
 * @attention:
 * <ul><li>None.</li></ul>
 *
 * @param: None.
 *
 * @retval: system clock.
 *
 * @par Dependency:
 * <ul><li>los_tick.h: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R001C00
 *
 * */
extern UINT32 LOS_SysClockGet(void);
extern BOOL g_bSysTickStart;

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_TICK_H */
