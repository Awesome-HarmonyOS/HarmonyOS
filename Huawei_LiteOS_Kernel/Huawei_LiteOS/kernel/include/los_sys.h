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

/**@defgroup los_sys System time
 * @ingroup kernel
 */

#ifndef _LOS_SYS_H
#define _LOS_SYS_H

#include "los_base.h"
#include "los_hwi.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


/**
 *@ingroup los_sys
 *System time basic function error code: Null pointer.
 *
 *Value: 0x02000010
 *
 *Solution: Check whether the input parameter is null.
 */
#define LOS_ERRNO_SYS_PTR_NULL                                  LOS_ERRNO_OS_ERROR(LOS_MOD_SYS, 0x10)

/**
 *@ingroup los_sys
 *System time basic function error code: Invalid system clock configuration.
 *
 *Value: 0x02000011
 *
 *Solution: Configure a valid system clock in los_config.h.
 */
#define LOS_ERRNO_SYS_CLOCK_INVALID                             LOS_ERRNO_OS_ERROR(LOS_MOD_SYS, 0x11)

/**
 *@ingroup los_sys
 *System time basic function error code: This error code is not in use temporarily.
 *
 *Value: 0x02000012
 *
 *Solution: None.
 */
#define LOS_ERRNO_SYS_MAXNUMOFCORES_IS_INVALID                  LOS_ERRNO_OS_ERROR(LOS_MOD_SYS, 0x12)

/**
 *@ingroup los_sys
 *System time error code: This error code is not in use temporarily.
 *
 *Value: 0x02000013
 *
 *Solution: None.
 */
#define LOS_ERRNO_SYS_PERIERRCOREID_IS_INVALID                  LOS_ERRNO_OS_ERROR(LOS_MOD_SYS, 0x13)

/**
 *@ingroup los_sys
 *System time error code: This error code is not in use temporarily.
 *
 *Value: 0x02000014
 *
 *Solution: None.
 */
#define LOS_ERRNO_SYS_HOOK_IS_FULL                              LOS_ERRNO_OS_ERROR(LOS_MOD_SYS, 0x14)

/**
 * @ingroup los_typedef
 * system time structure.
 */
typedef struct tagSysTime
{
    UINT16  uwYear;    /**< value 1970 ~ 2038 or 1970 ~ 2100 */
    UINT8   ucMonth;   /**< value 1 - 12 */
    UINT8   ucDay;     /**< value 1 - 31 */
    UINT8   ucHour;    /**< value 0 - 23 */
    UINT8   ucMinute;  /**< value 0 - 59 */
    UINT8   ucSecond;  /**< value 0 - 59 */
    UINT8   ucWeek;    /**< value 0 - 6  */
} SYS_TIME_S;

/**
 *@ingroup los_sys
 *@brief Obtain the number of Ticks.
 *
 *@par Description:
 *This API is used to obtain the number of Ticks.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param  None.
 *
 *@retval The number of Ticks is successfully obtained.
 *@par Dependency:
 *<ul><li>los_sys.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT64 LOS_TickCountGet (VOID);

/**
 *@ingroup los_sys
 *@brief Obtain the number of cycles in one second.
 *
 *@par Description:
 *This API is used to obtain the number of cycles in one second.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param  None.
 *
 *@retval Number of cycles obtained through the conversion. The number of cycles in one second is successfully obtained.
 *@par Dependency:
 *<ul><li>los_sys.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_CyclePerTickGet(VOID);

/**
 *@ingroup los_sys
 *@brief Convert Ticks to milliseconds.
 *
 *@par Description:
 *This API is used to convert Ticks to milliseconds.
 *@attention
 *<ul>
 *<li>The number of milliseconds obtained through the conversion is 32-bit.</li>
 *</ul>
 *
 *@param  tick  [IN] Number of Ticks. The value range is (0,OS_SYS_CLOCK).
 *
 *@retval Number of milliseconds obtained through the conversion. Ticks are successfully converted to milliseconds.
 *@par  Dependency:
 *<ul><li>los_sys.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_Tick2MS(UINT32 uwTick);

/**
 *@ingroup los_sys
 *@brief Convert milliseconds to Ticks.
 *
 *@par Description:
 *This API is used to convert milliseconds to Ticks.
 *@attention
 *<ul>
 *<li>Pay attention to the value to be converted because data possibly overflows.</li>
 *</ul>
 *
 *@param  millisec  [IN] Number of milliseconds.
 *
 *@retval Number of Ticks obtained through the conversion. Milliseconds are successfully converted to Ticks.
 *@par Dependency:
 *<ul><li>los_sys.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MS2Tick(UINT32 uwMillisec);

/**
 *@ingroup los_sys
 *@brief System reboot.
 *
 *@par Description:
 *This API is used to restart system.
 *@attention
 *<ul>
 *<li></li>
 *</ul>
 *
 *@param None.
 *
 *@retval None.
 *@par Dependency:
 *<ul><li>los_sys.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern VOID LOS_Reboot(VOID);
#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_SYS_H */
