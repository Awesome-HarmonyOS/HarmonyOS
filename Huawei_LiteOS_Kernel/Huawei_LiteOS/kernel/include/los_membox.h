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

/** @defgroup los_membox Static memory
 * @ingroup mem
 */

#ifndef _LOS_MEMBOX_H
#define _LOS_MEMBOX_H

#include "los_config.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/**
 * @ingroup los_membox
 * Define whether to check the address validity
 */
#if (LOSCFG_BASE_MEM_NODE_INTEGRITY_CHECK == YES)
#define LOS_MEMBOX_CHECK
#endif

/**
 * @ingroup los_membox
 * Structure of a free node in a memory pool
 */
typedef struct tagMEMBOX_NODE
{
    struct tagMEMBOX_NODE *pstNext;            /**<Free node's pointer to the next node in a memory pool*/
} LOS_MEMBOX_NODE;

/**
 * @ingroup los_membox
 * Memory pool information structure
 */
typedef struct
{
   UINT32           uwBlkSize;                            /**<Block size*/
   UINT32           uwBlkNum;                             /**<Block number*/
   LOS_MEMBOX_NODE  stFreeList;                 /**<Free list*/
} LOS_MEMBOX_INFO;

typedef LOS_MEMBOX_INFO OS_MEMBOX_S;

#ifdef LOS_MEMBOX_CHECK
#define LOS_MEMBOX_MAGIC_SIZE    4
#else
#define LOS_MEMBOX_MAGIC_SIZE    0
#endif

/**
 * @ingroup los_membox
 * Memory pool alignment
 */
#define LOS_MEMBOX_ALLIGNED(memAddr)           (((UINT32)(memAddr) + 3) & 0xfffffffc)

/**
 * @ingroup los_membox
 * Memory pool size
 */
#define LOS_MEMBOX_SIZE(uwBlkSize, uwBlkNum)   (sizeof(LOS_MEMBOX_INFO) + LOS_MEMBOX_ALLIGNED(uwBlkSize + LOS_MEMBOX_MAGIC_SIZE) * (uwBlkNum))

/**
 *@ingroup los_membox
 *@brief Initialize a memory pool.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to initialize a memory pool.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pPool    [IN] Memory pool address.
 *@param uwBoxSize   [IN] Memory pool size.
 *@param uwBlkSize   [IN] Memory block size.
 *
 *@retval #LOS_NOK  1: The memory pool is successfully initialized.
 *@retval #LOS_OK   0: The memory pool fails to be initialized.
 *@par Dependency:
 *<ul>
 *<li>los_membox.h: the header file that contains the API declaration.</li>
 *</ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemboxInit(VOID *pPool, UINT32 uwBoxSize, UINT32 uwBlkSize);

/**
 *@ingroup los_membox
 *@brief Request a memory block.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to request a memory block.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pPool    [IN] Memory pool address.
 *
 *@retval Memory block address. The request is accepted.
 *@retval NULL. The request fails.
 *@par Dependency:
 *<ul>
 *<li>los_membox.h: the header file that contains the API declaration.</li>
 *</ul>
 *@see LOS_MemboxFree
 *@since Huawei LiteOS V100R001C00
 */
extern VOID *LOS_MemboxAlloc(VOID *pPool);

/**
 *@ingroup los_membox
 *@brief Free a memory block.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to free a memory block.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pPool     [IN] Memory pool address.
 *@param pBox        [IN] Memory block address.
 *
 *@retval #LOS_NOK  1: This memory block fails to be freed.
 *@retval #LOS_OK   0: This memory block is successfully freed.
 *@par Dependency:
 *<ul>
 *<li>los_membox.h: the header file that contains the API declaration.</li>
 *</ul>
 *@see LOS_MemboxAlloc
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemboxFree(VOID *pPool, VOID *pBox);

/**
 *@ingroup los_membox
 *@brief Clear a memory block.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to set the memory block value to be 0.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>None.</li>
 *</ul>
 *
 *@param pPool    [IN] Memory pool address.
 *@param pBox        [IN] Memory block address.
 *
 *@retval None.
 *@par Dependency:
 *<ul>
 *<li>los_membox.h: the header file that contains the API declaration.</li>
 *</ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern VOID LOS_MemboxClr(VOID *pPool, VOID *pBox);


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_MEMBOX_H */
