/*----------------------------------------------------------------------------
 * Copyright (c) <2013-2017>, <Huawei Technologies Co., Ltd>
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
#ifndef _LOS_MEMBOX_H
#define _LOS_MEMBOX_H

#include "los_config.h"
#if (LOSCFG_PLATFORM_EXC == YES)
#include "los_memcheck.h"
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/**
 * @ingroup los_membox
 * Define whether to check the address validity
 */
#if (LOSCFG_PLATFORM_EXC == YES)
#define LOS_MEMBOX_CHECK
extern UINT8 g_aucMemMang[];
#endif

/**
 * @ingroup los_membox
 * Structure of a free node in a memory pool
 */
typedef struct tagMEMBOX_NODE
{
    struct tagMEMBOX_NODE *pstNext;            /* Free node's pointer to the next node in a memory pool */
} LOS_MEMBOX_NODE;

/**
 * @ingroup los_membox
 * Memory pool information structure
 */
typedef struct
{
   UINT32           uwBlkSize;                  /* Block size */
   UINT32           uwBlkNum;                   /* Total number of blocks */
   UINT32           uwBlkCnt;                   /* The number of allocated blocks */
   LOS_MEMBOX_NODE  stFreeList;                 /* Free list */
} LOS_MEMBOX_INFO;

/**
 * @ingroup los_membox
 * Default enabled membox's magic word detection function, this makes each block of membox
 * need an extra 4 bytes of space. If it is not necessary, please do not change it.
 * If the magic word of membox disabled, a bug will be generated, that is, when free a block
 * that has been freed, the membox will be destroyed.
 */
#define LOS_MEMBOX_MAGIC_CHECK
#ifdef LOS_MEMBOX_MAGIC_CHECK
#define LOS_MEMBOX_MAGIC_SIZE    4
#else
#define LOS_MEMBOX_MAGIC_SIZE    0
#endif

/**
 * @ingroup los_membox
 * The memory box is aligned to 4 (memory pool addr or memory box node size)
 */
#define LOS_MEMBOX_ALIGNED(align)           (((UINT32)(align) + 3) & 0xfffffffc)

/**
 * @ingroup los_membox
 * Memory pool size
 * Users can use this macro to calculate the total size of membox based on block size and block number
 */
#define LOS_MEMBOX_SIZE(uwBlkSize, uwBlkNum)   (sizeof(LOS_MEMBOX_INFO) + LOS_MEMBOX_ALIGNED(uwBlkSize + LOS_MEMBOX_MAGIC_SIZE) * (uwBlkNum))

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
 *<li>The uwBoxSize parameter value should match the following two conditions : 1) Be less than or equal to the Memory pool size; 2) Be greater than the size of LOS_MEMBOX_INFO.</li>
 *</ul>
 *
 *@param pBoxMem     [IN] Memory pool address.
 *@param uwBoxSize   [IN] Memory pool size.
 *@param uwBlkSize   [IN] Memory block size.
 *
 *@retval #LOS_NOK   The memory pool fails to be initialized.
 *@retval #LOS_OK    The memory pool is successfully initialized.
 *@par Dependency:
 *<ul>
 *<li>los_membox.h: the header file that contains the API declaration.</li>
 *</ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemboxInit(VOID *pBoxMem, UINT32 uwBoxSize, UINT32 uwBlkSize);

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
 *<li>The input pPool parameter must be initialized via func LOS_MemboxInit.</li>
 *</ul>
 *
 *@param pBoxMem     [IN] Memory pool address.
 *
 *@retval #VOID*      The request is accepted, and return a memory block address.
 *@retval #NULL       The request fails.
 *@par Dependency:
 *<ul>
 *<li>los_membox.h: the header file that contains the API declaration.</li>
 *</ul>
 *@see LOS_MemboxFree
 *@since Huawei LiteOS V100R001C00
 */
extern VOID *LOS_MemboxAlloc(VOID *pBoxMem);

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
 *<li>The input pPool parameter must be initialized via func LOS_MemboxInit.</li>
 *<li>The input pBox parameter must be allocated by LOS_MemboxAlloc.</li>
 *</ul>
 *
 *@param pBoxMem     [IN] Memory pool address.
 *@param pBox        [IN] Memory block address.
 *
 *@retval #LOS_NOK   This memory block fails to be freed.
 *@retval #LOS_OK    This memory block is successfully freed.
 *@par Dependency:
 *<ul>
 *<li>los_membox.h: the header file that contains the API declaration.</li>
 *</ul>
 *@see LOS_MemboxAlloc
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemboxFree(VOID *pBoxMem, VOID *pBox);

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
 *<li>The input pPool parameter must be initialized via func LOS_MemboxInit.</li>
 *<li>The input pBox parameter must be allocated by LOS_MemboxAlloc.</li>
 *</ul>
 *
 *@param pBoxMem     [IN] Memory pool address.
 *@param pBox        [IN] Memory block address.
 *
 *@retval VOID
 *@par Dependency:
 *<ul>
 *<li>los_membox.h: the header file that contains the API declaration.</li>
 *</ul>
 *@see None.
 *@since Huawei LiteOS V100R001C00
 */
extern VOID LOS_MemboxClr(VOID *pBoxMem, VOID *pBox);


/**
 *@ingroup los_membox
 *@brief calculate membox information.
 *
 *@par Description:
 *<ul>
 *<li>This API is used to calculate membox information.</li>
 *</ul>
 *@attention
 *<ul>
 *<li>One parameter of this interface is a pointer, it should be a correct value, otherwise, the system may be abnormal.</li>
 *</ul>
 *
 *@param  pBoxMem        [IN]  Type  #VOID*   Pointer to the calculate membox.
 *@param  uwMaxBlk       [OUT] Type  #UINT32* Record membox max block.
 *@param  uwBlkCnt       [OUT] Type  #UINT32* Record membox block count alreay allocated.
 *@param  uwBlkSize      [OUT] Type  #UINT32* Record membox block size.
 *
 *@retval #LOS_OK        The heap status calculate success.
 *@retval #LOS_NOK       The membox  status calculate with some error.
 *@par Dependency:
 *<ul><li>los_memory.h: the header file that contains the API declaration.</li></ul>
 *@see LOS_MemAlloc | LOS_MemRealloc | LOS_MemFree
 *@since Huawei LiteOS V100R001C00
 */
extern UINT32 LOS_MemboxStatisticsGet(VOID *pBoxMem, UINT32 *puwMaxBlk, UINT32 *puwBlkCnt, UINT32 *puwBlkSize);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif
