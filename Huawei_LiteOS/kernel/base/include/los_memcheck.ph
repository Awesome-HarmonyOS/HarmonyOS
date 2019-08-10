/*----------------------------------------------------------------------------
 *      Huawei - LiteOS
 *----------------------------------------------------------------------------
 *      Name:    LOS_MEMCHECK.H
 *      Purpose: Memory check header file
 *      Rev.:    V1.0.0
 *----------------------------------------------------------------------------
 *

 * Copyright (c) 2014, Huawei Technologies Co., Ltd.
 * All rights reserved.
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.

 *THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *----------------------------------------------------------------------------*/
#ifndef _LOS_MEMCHECK_PH
#define _LOS_MEMCHECK_PH

#include "los_memcheck.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

/**
 *@ingroup los_memboxcheck
 *@brief Update the information of the memory.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to update the information of the memory.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pPool       [IN/OUT]  Type #VOID  *  Memory pool number.
 *@param uwSize    [IN] Type #UINT32   Memory size.
 *@param type        [IN] Type #UINT32   Memory mang type.
 *
 *@retval UINT32 Updateinformation result.
 *@par Dependency:
 *<ul>
 *<li>los_memboxcheck.ph: the header file that contains the API declaration.</li>
 *</ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
UINT32 osMemInfoUpdate(VOID *pPool, UINT32 uwSize, UINT32 uwType);

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#endif /* _LOS_MEMCHECK_PH */
