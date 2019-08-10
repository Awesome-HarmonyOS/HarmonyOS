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
#ifndef _LOS_MEMCHECK_H
#define _LOS_MEMCHECK_H

#include "los_base.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#define MEM_INFO_SIZE                   (sizeof(MEM_INFO) * OS_SYS_MEM_NUM + 4)
extern UINT8 g_aucMemMang[];

enum _MEM_MANG_TYPE
{
    MEM_MANG_MEMBOX,
    MEM_MANG_MEMORY,
    MEM_MANG_EMPTY,
};

enum _MEM_MANG_SOUCE
{
    MEM_MANG_UNUSED,
    MEM_MANG_INIT,
    MEM_MANG_INT,
    MEM_MANG_TASK,
};

typedef struct _MEM_INFO
{
    UINT32 uwType;
    UINT32 uwStartAddr;
    UINT32 uwSize;
}MEM_INFO;

typedef struct _SLAB_INFO
{
    UINT32 item_sz;
    UINT32 item_cnt;
    UINT32 cur_usage;
}SLAB_INFO;

#define SLAB_CLASS_NUM                  (4U)
typedef struct _MEM_INFO_S
{
    UINT32 uwType;
    UINT32 uwStartAddr;
    UINT32 uwSize;
    UINT32 uwFree;
    UINT32 uwBlockSize;
    UINT32 uwErrorAddr;
    UINT32 uwErrorLen;
    UINT32 uwErrorOwner;
    SLAB_INFO stSlabInfo[SLAB_CLASS_NUM];
}MEM_INFO_S;

/**
 *@ingroup los_memboxcheck
 *@brief Get the information of the exc memory.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to get the information of the exc memory.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param uwMemNum       [IN] Type #UINT32  Memory pool number.
 *@param pstMemExcInfo   [IN/OUT] Type #MEM_INFO_S *  information of the exc memory.
 *
 *@retval UINT32 Get information result.
 *@par Dependency:
 *<ul>
 *<li>los_memboxcheck.h: the header file that contains the API declaration.</li>
 *</ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
UINT32 LOS_MemExcInfoGet(UINT32 uwMemNum, MEM_INFO_S *pstMemExcInfo);

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#endif /* _LOS_MEMCHECK_H */
