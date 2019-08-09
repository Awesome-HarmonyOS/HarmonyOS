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

#ifndef _LOS_SYS_PH
#define _LOS_SYS_PH

#include "los_base.ph"

#include "los_sys.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


/**
 * @ingroup los_sys
 *Number of milliseconds in one second.
 */
#define OS_SYS_MS_PER_SECOND                1000

/**
 * @ingroup los_sys
 *Number of microseconds in one second.
 */
#define OS_SYS_US_PER_SECOND                1000000

/**
 *@ingroup los_sys
 *@brief Convert cycles to milliseconds.
 *
 *@par Description:
 *This API is used to convert cycles to milliseconds.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param  udwCycle     [IN] Number of cycles.
 *
 *@retval Number of milliseconds obtained through the conversion.    Cycles are successfully converted to milliseconds.
 *@par Dependency:
 *<ul><li>los_sys.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
INLINE UINT64 osCycle2MS(UINT64 udwCycle)
{
    return (UINT64)((udwCycle / (OS_SYS_CLOCK / OS_SYS_MS_PER_SECOND)));
}

/**
 *@ingroup los_sys
 *@brief Convert cycles to microseconds.
 *
 *@par Description:
 *This API is used to convert cycles to microseconds.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param  udwCycle     [IN] Number of cycles.
 *
 *@retval Number of microseconds obtained through the conversion. Cycles are successfully converted to microseconds.
 *@par Dependency:
 *<ul><li>los_sys.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
INLINE UINT64 osCycle2US(UINT64 udwCycle)
{
    UINT64 udwTmp = OS_SYS_CLOCK / OS_SYS_US_PER_SECOND;
    return (UINT64)(udwCycle / udwTmp);
}

/**
 *@ingroup los_sys
 *@brief Convert cycles to milliseconds.
 *
 *@par Description:
 *This API is used to convert cycles to milliseconds.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param  pstCpuTick  [IN] Number of CPU cycles.
 *@param  puwMsHi     [OUT] Upper 32 bits of the number of milliseconds.
 *@param  puwMsLo     [OUT] Lower 32 bits of the number of milliseconds.
 *
 *@retval #LOS_ERRNO_SYS_PTR_NULL    0x02000011: Invalid parameter.
 *@retval #LOS_OK                   0:  Cycles are successfully converted to microseconds.
 *@par Dependency:
 *<ul><li>los_sys.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 osCpuTick2MS(CPU_TICK *pstCpuTick, UINT32 *puwMsHi, UINT32 *puwMsLo);

/**
 *@ingroup los_sys
 *@brief Convert cycles to microseconds.
 *
 *@par Description:
 *This API is used to convert cycles to microseconds.
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param  pstCpuTick  [IN] Number of CPU cycles.
 *@param  puwUsHi     [OUT] Upper 32 bits of the number of microseconds.
 *@param  puwUsLo     [OUT] Lower 32 bits of the number of microseconds.
 *
 *@retval #LOS_ERRNO_SYS_PTR_NULL    0x02000011: Invalid parameter.
 *@retval #LOS_OK                   0: Cycles are successfully converted to microseconds.
 *@par Dependency:
 *<ul><li>los_sys.h: the header file that contains the API declaration.</li></ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 osCpuTick2US(CPU_TICK *pstCpuTick, UINT32 *puwUsHi, UINT32 *puwUsLo);


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_SYS_PH */
