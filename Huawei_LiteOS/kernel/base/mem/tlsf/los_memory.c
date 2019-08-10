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

#include "string.h"
#include "los_typedef.h"
#include "los_memory.ph"
#include "los_compiler.h"

#if (LOSCFG_KERNEL_MEM_SLAB == YES)
#include "los_slab.ph"
#endif

#include "los_hwi.h"
#if (LOSCFG_PLATFORM_EXC == YES)
#include "los_exc.ph"
#endif

#include "los_task.ph"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

LITE_OS_SEC_DATA_INIT static UINT32 g_uwTlsf_AllocCount = 0;
LITE_OS_SEC_DATA_INIT static UINT32 g_uwTlsf_FreeCount = 0;


extern ST_LOS_TASK g_stLosTask;

#define AARCHPTR UINT32

LITE_OS_SEC_ALW_INLINE static inline INT32 __log2(UINT32 uwWord)
{
    return uwWord ? (sizeof(uwWord) * 8 - CLZ(uwWord) - 1) : 0;
}

LITE_OS_SEC_TEXT static inline INT32 tlsf_ffs(UINT32 uwWord)
{
    return __log2(uwWord & (UINT32)(-(UINT32)uwWord)); /*lint !e501*/
}

LITE_OS_SEC_TEXT static inline INT32 tlsf_fls(UINT32 uwWord)
{
    const INT32 swBit = uwWord ? 32 - CLZ(uwWord) : 0;
    return swBit - 1;
}

#define tlsf_fls_sizet tlsf_fls

/*
 * Block header structure.
 *
 * There are several implementation subtleties involved:
 * - The prev_phys_block field is only valid if the previous pNode is free.
 * - The prev_phys_block field is actually stored at the end of the
 *   previous pNode. It appears at the beginning of this structure only to
 *   simplify the implementation.
 * - The pNext_free / pPrev_free fields are only valid if the pNode is free.
 */
typedef struct LOS_MEM_DYN_NODE {
    /* Points to the previous physical pNode. */
    struct LOS_MEM_DYN_NODE *pstPreNode;
    /* The size of this pNode, excluding the pNode header. */
    UINT32 uwSize;
    /* Next and previous free blocks. */
    struct LOS_MEM_DYN_NODE *pNext_free;
    struct LOS_MEM_DYN_NODE *pPrev_free;
} LOS_MEM_DYN_NODE;

enum tlsf_public {
    SL_INDEX_COUNT_LOG2 = 2,
};

enum tlsf_private {
    /* All allocation sizes and addresses are aligned to 4 bytes. */
    ALIGN_SIZE_LOG2 = 2,
    ALIGN_SIZE = (1 << ALIGN_SIZE_LOG2),
    FL_INDEX_MAX = 30,
    SL_INDEX_COUNT = (1 << SL_INDEX_COUNT_LOG2),
    FL_INDEX_SHIFT = (SL_INDEX_COUNT_LOG2 + ALIGN_SIZE_LOG2),
    FL_INDEX_COUNT = (FL_INDEX_MAX - FL_INDEX_SHIFT + 1),
    SMALL_BLOCK_SIZE = (1 << FL_INDEX_SHIFT),
};

#if (LOSCFG_MEM_MUL_POOL == YES)
VOID *g_pPoolHead = NULL;
#endif

typedef struct LOS_MEM_POOL_INFO {
    LOS_MEM_DYN_NODE stBlock_null;
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    struct LOS_SLAB_CONTROL_HEADER stSlabCtrlHdr;
#endif
    UINT32 uwPoolSize;
    UINT32 fl_bitmap;
    UINT32 sl_bitmap[FL_INDEX_COUNT];
    LOS_MEM_DYN_NODE *pstBlocks[FL_INDEX_COUNT][SL_INDEX_COUNT];

#if (LOSCFG_MEM_MUL_POOL == YES)
    VOID *pNextPool;
#endif
} LOS_MEM_POOL_INFO;

/* pool_t: a pNode of memory that TLSF can manage. */
typedef VOID *pool_t;

/* Add/remove memory pools. */
pool_t osMemPoolAdd(VOID *tlsf, VOID *pPool, UINT32 uwBytes);

/* Overheads/limits of internal structures. */
UINT32 osMemHeadSize(VOID);

/* Debugging. */
#define TSLF_CONFIG_DEBUG

#ifdef TLSF_CONFIG_ASSERT
#include <assert.h>
#define tlsf_assert(expr) assert(expr)
#define   alignCheck(align)  tlsf_assert(0 == ((align) & ((align) - 1)) && "must align to a power of two");
#else
#define tlsf_assert(expr) (VOID)(0)
#define   alignCheck(align)
#endif

/*
 * Cast and min/max macros.
 */
#define tlsf_cast(t, exp) ((t)(exp))
#define tlsf_min(a, b) ((a) < (b) ? (a) : (b))
#define tlsf_max(a, b) ((a) > (b) ? (a) : (b))

#define tlsf_offset(a,b) LOS_OFF_SET_OF(a, b)

LITE_OS_SEC_TEXT VOID *osSlabCtrlHdrGet(VOID *pPool)
{
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    return (&(tlsf_cast(LOS_MEM_POOL_INFO *, pPool)->stSlabCtrlHdr));
#else
    return NULL;
#endif
}


/*
 * Since pNode sizes are always at least a multiple of 4, the two least
 * significant bits of the size field are used to store the pNode status:
 * - bit 0: whether pNode is busy or free
 * - bit 1: whether previous pNode is busy or free
 */
static const UINT32 block_header_free_bit = 1 << 0;
static const UINT32 block_header_prev_free_bit = 1 << 1;

#define DEBUG_SPACE   4

/*
 * The size of the pNode header exposed to used blocks is the size field.
 * The pstPreNode field is stored *inside* the previous free pNode.
 */
static const UINT32 block_header_overhead = tlsf_offset(LOS_MEM_DYN_NODE, uwSize);

/* User data starts directly after the size field in a used pNode. */
static const UINT32 block_start_offset =
    tlsf_offset(LOS_MEM_DYN_NODE, uwSize) + sizeof(UINT32) + DEBUG_SPACE; /*lint !e413*/

/*
 * A free pNode must be large enough to store its header minus the size of
 * the pstPreNode field, and no larger than the number of addressable
 * bits for FL_INDEX.
 */
static const UINT32 block_size_min =
    sizeof(LOS_MEM_DYN_NODE) - sizeof(LOS_MEM_DYN_NODE *);
static const UINT32 block_size_max = tlsf_cast(UINT32, 1) << FL_INDEX_MAX;

#define control_t LOS_MEM_POOL_INFO

#define OS_MEM_TASKID_SET(node, ID)  \
            do \
            { \
                AARCHPTR uwTmp = (AARCHPTR)(((LOS_MEM_DYN_NODE *)node)->pNext_free); \
                uwTmp &= 0xffff0000; \
                uwTmp |= ID; \
                ((LOS_MEM_DYN_NODE *)node)->pNext_free = (LOS_MEM_DYN_NODE *)uwTmp; \
            }while(0)

#define OS_MEM_TASKID_GET(node)  ((AARCHPTR)(((LOS_MEM_DYN_NODE *)node)->pNext_free) & 0xffff)

VOID osMemInfoPrint(pool_t pPool);
#define IS_ALIGNED(value, alignSize)  (0 == (((AARCHPTR)(value)) & ((AARCHPTR)(alignSize - 1))))
/*
 * LOS_MEM_DYN_NODE member functions.
 */
LITE_OS_SEC_TEXT static VOID osMemNodeSizeSet(LOS_MEM_DYN_NODE *pNode, UINT32 uwSize)
{
    pNode->uwSize = uwSize | (pNode->uwSize & (block_header_free_bit | block_header_prev_free_bit));
}

LITE_OS_SEC_TEXT static UINT32 osMemNodeSizeGet(const LOS_MEM_DYN_NODE *pNode)
{
    return pNode->uwSize & ~(block_header_free_bit | block_header_prev_free_bit);
}

LITE_OS_SEC_TEXT static INT32 osMemEndCheck(const LOS_MEM_DYN_NODE *pNode)
{
    return osMemNodeSizeGet(pNode) == 0;
}

LITE_OS_SEC_TEXT static INT32 osMemFreeCheck(const LOS_MEM_DYN_NODE *pNode)
{
    return tlsf_cast(INT32, pNode->uwSize & block_header_free_bit);
}

LITE_OS_SEC_TEXT static VOID osMemFreeMark(LOS_MEM_DYN_NODE *pNode)
{
    pNode->uwSize |= block_header_free_bit;
}

LITE_OS_SEC_TEXT static VOID osMemUsedMark(LOS_MEM_DYN_NODE *pNode)
{
    pNode->uwSize &= ~block_header_free_bit;
}

LITE_OS_SEC_TEXT static INT32 osMemPreFreeCheck(const LOS_MEM_DYN_NODE *pNode)
{
    return tlsf_cast(INT32, pNode->uwSize & block_header_prev_free_bit);
}

LITE_OS_SEC_TEXT static VOID osMemPreFreeMark(LOS_MEM_DYN_NODE *pNode)
{
    pNode->uwSize |= block_header_prev_free_bit;
}

LITE_OS_SEC_TEXT static VOID osMemPreUsedMark(LOS_MEM_DYN_NODE *pNode)
{
    pNode->uwSize &= ~block_header_prev_free_bit;
}

LITE_OS_SEC_TEXT LOS_MEM_DYN_NODE *osMemPtrToNode(const VOID *pPtr)
{
    return tlsf_cast(LOS_MEM_DYN_NODE *,
                     tlsf_cast(UINT8*, pPtr) - block_start_offset);
}

LITE_OS_SEC_TEXT static VOID *osMemNodeToPtr(const LOS_MEM_DYN_NODE *pNode)
{
    return tlsf_cast(VOID *,
                     tlsf_cast(UINT8 *, pNode) + block_start_offset);
}

/* Return location of next pNode after pNode of given size. */
LITE_OS_SEC_TEXT static LOS_MEM_DYN_NODE *osMemPtrOffset(const VOID *pPtr, INT32 swSize)
{
    return tlsf_cast(LOS_MEM_DYN_NODE *, tlsf_cast(AARCHPTR, pPtr) + swSize);
}

LITE_OS_SEC_TEXT static LOS_MEM_DYN_NODE * osMemFirstNode(VOID *pPool)
{
    return osMemPtrOffset((const VOID *)((AARCHPTR)pPool + osMemHeadSize()), -(INT32)block_header_overhead); /*lint !e570*/
}

LITE_OS_SEC_TEXT static LOS_MEM_DYN_NODE *osMemEndNode(pool_t pPool)
{
    return (LOS_MEM_DYN_NODE *)((AARCHPTR)pPool + ((control_t *)pPool)->uwPoolSize - block_start_offset);
}

LITE_OS_SEC_TEXT static AARCHPTR osMemEndPtr(pool_t pPool)
{
    return (AARCHPTR)pPool + ((control_t *)pPool)->uwPoolSize;
}

/* Return location of previous pNode. */
LITE_OS_SEC_TEXT static LOS_MEM_DYN_NODE *osMemNodePre(const LOS_MEM_DYN_NODE *pNode)
{
    tlsf_assert(osMemPreFreeCheck(pNode) && "previous pNode must be free");
    return pNode->pstPreNode;
}

/* Return location of next existing pNode. */
LITE_OS_SEC_TEXT static LOS_MEM_DYN_NODE *osMemNodeNext(const LOS_MEM_DYN_NODE *pNode)
{
    LOS_MEM_DYN_NODE *pNext = osMemPtrOffset(
        tlsf_cast(VOID *, pNode), (INT32)osMemNodeSizeGet(pNode));
    tlsf_assert(!osMemEndCheck(pNode));
    return pNext;
}

LITE_OS_SEC_TEXT static LOS_MEM_DYN_NODE *osMemNextLink(LOS_MEM_DYN_NODE *pNode)
{
    LOS_MEM_DYN_NODE *pNext = osMemNodeNext(pNode);
    pNext->pstPreNode = pNode;
    return pNext;
}

LITE_OS_SEC_TEXT static VOID osMemFreeSet(LOS_MEM_DYN_NODE *pNode)
{
    /* Link the pNode to the next pNode, first. */
    LOS_MEM_DYN_NODE *pNext = osMemNextLink(pNode);
    osMemPreFreeMark(pNext);
    osMemFreeMark(pNode);
}

LITE_OS_SEC_TEXT static VOID osMemUsedSet(LOS_MEM_DYN_NODE *pNode)
{
    LOS_MEM_DYN_NODE *pNext = osMemNodeNext(pNode);
    osMemPreUsedMark(pNext);
    osMemUsedMark(pNode);
}

LITE_OS_SEC_TEXT static AARCHPTR osMemAlignDown(AARCHPTR uwData, UINT32 uwAlign)
{
    alignCheck(uwAlign);
    return uwData - (uwData & (uwAlign - 1));
}

LITE_OS_SEC_TEXT static AARCHPTR osMemAlignUp(AARCHPTR uwData, UINT32 uwAlign)
{
    alignCheck(uwAlign);
    return (uwData + (AARCHPTR)(uwAlign - 1)) & ~((AARCHPTR)(uwAlign - 1));
}

LITE_OS_SEC_TEXT static VOID *osMemAlignPtr(const VOID *pPtr, UINT32 uwAlign)
{
    const AARCHPTR uwAligned = (tlsf_cast(AARCHPTR, pPtr) + (AARCHPTR)(uwAlign - 1)) & ~((AARCHPTR)(uwAlign - 1));
    alignCheck(uwAlign);
    return tlsf_cast(VOID *, uwAligned);
}

LITE_OS_SEC_TEXT static UINT32 osMemReqSizeAdjust(UINT32 uwSize, UINT32 uwAlign)
{
    UINT32 uwAdjust = 0;
    if (uwSize && uwSize < block_size_max)
    {
        const UINT32 uwAligned = osMemAlignUp(uwSize, uwAlign) + block_start_offset;
        uwAdjust = tlsf_max(uwAligned, block_size_min);
    }
    return uwAdjust;
}

LITE_OS_SEC_TEXT static VOID osMemMapInsert(UINT32 uwSize, INT32 *pFli, INT32 *pSli)
{
    INT32 swFl, swSl;
    if (uwSize < SMALL_BLOCK_SIZE)
    {
        /* Store small blocks in first list. */
        swFl = 0;
        swSl = tlsf_cast(INT32, uwSize) / (SMALL_BLOCK_SIZE / SL_INDEX_COUNT);
    }
    else
    {
        swFl = tlsf_fls_sizet(uwSize);
        swSl = tlsf_cast(INT32, uwSize >> (swFl - SL_INDEX_COUNT_LOG2)) ^
             (1 << SL_INDEX_COUNT_LOG2);
        swFl -= (FL_INDEX_SHIFT - 1);
    }

    *pFli = swFl;
    *pSli = swSl;
    //PRINT_ERR("FL = %d, SL = %d\n", swFl,swSl);
}

LITE_OS_SEC_TEXT static VOID osMemMapSearch(UINT32 uwSize, INT32 *pFli, INT32 *pSli)
{
    if (uwSize >= SMALL_BLOCK_SIZE) {
        const UINT32 uwRound = (1 << (tlsf_fls_sizet(uwSize) - SL_INDEX_COUNT_LOG2)) - 1;
        uwSize += uwRound;
    }
    osMemMapInsert(uwSize, pFli, pSli);
}

LITE_OS_SEC_TEXT static LOS_MEM_DYN_NODE *osMemSuitableNodeSearch(control_t *control,
                                             INT32 *pFli, INT32 *pSli)
{
    INT32 swFl = *pFli;
    INT32 swSl = *pSli;
    UINT32 swSl_map = control->sl_bitmap[swFl] & (((UINT32)~0) << swSl);
    if (!swSl_map) {
        /* No pNode exists. Search in the next largest first-level list. */
        const UINT32 swFl_map = control->fl_bitmap & (((UINT32)~0) << (swFl + 1));
        if (!swFl_map) {
            /* No free blocks available, memory has been exhausted. */
            return NULL;
        }

        swFl = tlsf_ffs(swFl_map);
        *pFli = swFl;
        swSl_map = control->sl_bitmap[swFl];
    }
    tlsf_assert(swSl_map && "internal error - second level bitmap is null");
    swSl = tlsf_ffs(swSl_map);
    *pSli = swSl;

    /* Return the first pNode in the free list. */
    return control->pstBlocks[swFl][swSl];
}

LITE_OS_SEC_TEXT static VOID osMemFreeNodeRemove(control_t *control,
                              LOS_MEM_DYN_NODE *pNode,
                              INT32 swFl, INT32 swSl)
{
    LOS_MEM_DYN_NODE *pPrev = pNode->pPrev_free;
    LOS_MEM_DYN_NODE *pNext = pNode->pNext_free;
    tlsf_assert(pPrev && "pPrev_free field can not be null");
    tlsf_assert(pNext && "pNext_free field can not be null");
    pNext->pPrev_free = pPrev;
    pPrev->pNext_free = pNext;
    /* If this pNode is the head of the free list, set new head. */
    if (control->pstBlocks[swFl][swSl] == pNode) {
        control->pstBlocks[swFl][swSl] = pNext;
        /* If the new head is null, clear the bitmap. */
        if (pNext == &control->stBlock_null) {
            control->sl_bitmap[swFl] &= ~(1 << swSl); /*lint !e502*/

            /* If the second bitmap is now empty, clear the fl bitmap. */
            if (!control->sl_bitmap[swFl]) {
                control->fl_bitmap &= ~(1 << swFl); /*lint !e502*/
            }
        }
    }
}

/* Insert a free pNode into the free pNode list. */
LITE_OS_SEC_TEXT static VOID osMemFreeNodeInsert(control_t *control,
                              LOS_MEM_DYN_NODE *pNode,
                              INT32 swFl, INT32 swSl)
{
    LOS_MEM_DYN_NODE *pstCurrent = control->pstBlocks[swFl][swSl];
    //tlsf_assert(pstCurrent && "free list cannot have a null entry");
    //tlsf_assert(pNode && "cannot insert a null entry into the free list");

    pNode->pNext_free = pstCurrent;
    pNode->pPrev_free = &control->stBlock_null;
    pstCurrent->pPrev_free = pNode;

    tlsf_assert(osMemNodeToPtr(pNode) ==
                    osMemAlignPtr(osMemNodeToPtr(pNode), ALIGN_SIZE) &&
                "pNode not aligned properly");
    /*
     * Insert the new pNode at the head of the list, and mark the first-
     * and second-level bitmaps appropriately.
     */

    control->pstBlocks[swFl][swSl] = pNode;
    control->fl_bitmap |= (1 << swFl);
    control->sl_bitmap[swFl] |= (1 << swSl);
}

/* Remove a given pNode from the free list. */
LITE_OS_SEC_TEXT static VOID osMemNodeRemove(control_t *control, LOS_MEM_DYN_NODE *pNode)
{
    INT32 swFl, swSl;
    osMemMapInsert(osMemNodeSizeGet(pNode), &swFl, &swSl);
    osMemFreeNodeRemove(control, pNode, swFl, swSl);
}

/* Insert a given pNode into the free list. */
LITE_OS_SEC_TEXT static VOID osMemNodeInsert(control_t *control, LOS_MEM_DYN_NODE *pNode)
{
    INT32 swFl, swSl;
    osMemMapInsert(osMemNodeSizeGet(pNode), &swFl, &swSl);
    osMemFreeNodeInsert(control, pNode, swFl, swSl);
}

LITE_OS_SEC_TEXT static INT32 osMemNodeSpitCheck(LOS_MEM_DYN_NODE *pNode, UINT32 uwSize)
{
    return osMemNodeSizeGet(pNode) >= sizeof(LOS_MEM_DYN_NODE) + uwSize;
}

/* Split a pNode into two, the second of which is free. */
LITE_OS_SEC_TEXT static LOS_MEM_DYN_NODE *osMemNodeSpit(LOS_MEM_DYN_NODE *pNode, UINT32 uwSize)
{
    /* Calculate the amount of space left in the remaining pNode. */
    LOS_MEM_DYN_NODE *remaining = osMemPtrOffset((VOID *)pNode, (INT32)uwSize);

    const UINT32 remain_size = osMemNodeSizeGet(pNode) - (uwSize);

    tlsf_assert(osMemNodeToPtr(remaining) == osMemAlignPtr(osMemNodeToPtr(remaining),
                                                     ALIGN_SIZE) &&
                "remaining pNode not aligned properly");

    tlsf_assert(osMemNodeSizeGet(pNode) ==
                remain_size + uwSize);
    osMemNodeSizeSet(remaining, remain_size);
    tlsf_assert(osMemNodeSizeGet(remaining) >= block_size_min &&
                "pNode split with invalid size");

    osMemNodeSizeSet(pNode, uwSize);
    osMemFreeSet(remaining);

    return remaining;
}

/* Absorb a free pNode's storage into an adjacent previous free pNode. */
LITE_OS_SEC_TEXT static LOS_MEM_DYN_NODE *osMemNodeAbsorb(LOS_MEM_DYN_NODE *pPrev, LOS_MEM_DYN_NODE *pNode)
{
    tlsf_assert(!osMemEndCheck(pPrev) && "previous pNode can't be last");
    /* Note: Leaves flags untouched. */
    pPrev->uwSize += osMemNodeSizeGet(pNode);
    (VOID)osMemNextLink(pPrev);
    return pPrev;
}

/* Merge a just-freed pNode with an adjacent previous free pNode. */
LITE_OS_SEC_TEXT static LOS_MEM_DYN_NODE *osMemPreNodeMerge(control_t *control,
                                        LOS_MEM_DYN_NODE *pNode)
{
    if (osMemPreFreeCheck(pNode)) {
        LOS_MEM_DYN_NODE *pPrev = osMemNodePre(pNode);
        tlsf_assert(pPrev && "prev physical pNode can't be null");
        tlsf_assert(osMemFreeCheck(pPrev) &&
                    "prev pNode is not free though marked as such");
        osMemNodeRemove(control, pPrev);
        pNode = osMemNodeAbsorb(pPrev, pNode);
    }

    return pNode;
}

/* Merge a just-freed pNode with an adjacent free pNode. */
LITE_OS_SEC_TEXT static LOS_MEM_DYN_NODE *osMemNextNodeMerge(control_t *control,
                                        LOS_MEM_DYN_NODE *pNode)
{
    LOS_MEM_DYN_NODE *pNext = osMemNodeNext(pNode);
    tlsf_assert(pNext && "next physical pNode can't be null");

    if (osMemFreeCheck(pNext)) {
        tlsf_assert(!osMemEndCheck(pNode) && "previous pNode can't be last");
        osMemNodeRemove(control, pNext);
        pNode = osMemNodeAbsorb(pNode, pNext);
    }

    return pNode;
}

/* Trim any trailing pNode space off the end of a pNode, return to pool. */
LITE_OS_SEC_TEXT static VOID osMemFreeNodeTrim(control_t *control,
                            LOS_MEM_DYN_NODE *pNode,
                            UINT32 uwSize)
{
    tlsf_assert(osMemFreeCheck(pNode) && "pNode must be free");
    if (osMemNodeSpitCheck(pNode, uwSize))
    {
        LOS_MEM_DYN_NODE *remaining_block = osMemNodeSpit(pNode, uwSize);
        (VOID)osMemNextLink(pNode);
        osMemPreFreeMark(remaining_block);
        osMemNodeInsert(control, remaining_block);
    }
}

/* Trim any trailing pNode space off the end of a used pNode, return to pool. */
LITE_OS_SEC_TEXT_MINOR static VOID osMemUsedNodeTrim(control_t *control,
                            LOS_MEM_DYN_NODE *pNode,
                            UINT32 uwSize)
{
    tlsf_assert(!osMemFreeCheck(pNode) && "pNode must be used");
    if (osMemNodeSpitCheck(pNode, uwSize)) {
        /* If the next pNode is free, we must coalesce. */
        LOS_MEM_DYN_NODE *remaining_block = osMemNodeSpit(pNode, uwSize);
        osMemPreUsedMark(remaining_block);
        remaining_block->pstPreNode = pNode;

        remaining_block = osMemNextNodeMerge(control, remaining_block);
        osMemNodeInsert(control, remaining_block);
    }
}

LITE_OS_SEC_TEXT static LOS_MEM_DYN_NODE *osMemFreeNodeTrimLeading(control_t *control,
                                               LOS_MEM_DYN_NODE *pNode,
                                               UINT32 uwSize)
{
    LOS_MEM_DYN_NODE *remaining_block = pNode;
    if (osMemNodeSpitCheck(pNode, uwSize)) {
        /* We want the 2nd pNode. */
        remaining_block = osMemNodeSpit(pNode, uwSize);
        osMemPreFreeMark(remaining_block);

        (VOID)osMemNextLink(pNode);
        osMemNodeInsert(control, pNode);
    }

    return remaining_block;
}

LITE_OS_SEC_TEXT static LOS_MEM_DYN_NODE *osMemFreeNodeLocate(control_t *control, UINT32 uwSize)
{
    INT32 swFl = 0, swSl = 0;
    LOS_MEM_DYN_NODE *pNode = NULL;

    if (uwSize) {
        osMemMapInsert(uwSize, &swFl, &swSl);
        pNode = osMemSuitableNodeSearch(control, &swFl, &swSl);
    }

    if (pNode) {
        if (osMemNodeSizeGet(pNode) >= uwSize)
            goto EXIT;
        while (pNode->pNext_free != &control->stBlock_null)
        {
            pNode = pNode->pNext_free;
            if (osMemNodeSizeGet(pNode) >= uwSize)
                goto EXIT;
        }
        osMemMapSearch(uwSize, &swFl, &swSl);
        pNode = osMemSuitableNodeSearch(control, &swFl, &swSl);
        if (pNode == NULL || osMemNodeSizeGet(pNode) < uwSize)
            return NULL;
EXIT:
        osMemFreeNodeRemove(control, pNode, swFl, swSl);
    }

    if (pNode && !pNode->uwSize)
        pNode = NULL;

    return pNode;
}

LITE_OS_SEC_TEXT static VOID *osMemUsedNodePrepare(control_t *control, LOS_MEM_DYN_NODE *pNode, UINT32 uwSize)
{
    VOID *pPtr = NULL;
    if (pNode)
    {
        tlsf_assert(uwSize && "size must be non-zero");
        osMemFreeNodeTrim(control, pNode, uwSize);
        osMemUsedSet(pNode);

        /* If the operation occured before task initialization(g_stLosTask.pstRunTask was not assigned)
           or in interrupt,make the value of taskid of pNode to oxffffffff*/
        if (g_stLosTask.pstRunTask != NULL && OS_INT_INACTIVE)
        {
            OS_MEM_TASKID_SET(pNode, g_stLosTask.pstRunTask->uwTaskID);
        }
        else
        {
            /* If the task mode does not initialize, the field is the 0xffffffff */
            OS_MEM_TASKID_SET(pNode, OS_NULL_INT);
            /* TODO: the commend task-MEMUSE is not include system initialization malloc */
        }
        pPtr = osMemNodeToPtr(pNode);
    }
    else
    {
        PRINT_INFO("-----------------------------------------------------------------------------------------------------------\n");
        osMemInfoPrint((pool_t)control);
        PRINT_ERR("No suitable free block, require free node size: 0x%x\n", uwSize);
        PRINT_INFO("-----------------------------------------------------------------------------------------------------------\n");
        return NULL;
    }
    return pPtr;
}

/* Clear structure and point all empty lists at the null pNode. */
LITE_OS_SEC_TEXT_INIT static VOID osMemControlClear(control_t *control, UINT32 uwBytes)
{
    UINT32 uwFlIndex, uwSlIndex;

    control->stBlock_null.pNext_free = &control->stBlock_null;
    control->stBlock_null.pPrev_free = &control->stBlock_null;

    control->uwPoolSize = uwBytes;

    control->fl_bitmap = 0;
    for (uwFlIndex = 0; uwFlIndex < FL_INDEX_COUNT; ++uwFlIndex) {
        control->sl_bitmap[uwFlIndex] = 0;
        for (uwSlIndex = 0; uwSlIndex < SL_INDEX_COUNT; ++uwSlIndex) {
            control->pstBlocks[uwFlIndex][uwSlIndex] = &control->stBlock_null;
        }
    }
}

/*
 * Debugging utilities.
 */
#ifdef TSLF_CONFIG_DEBUG
LITE_OS_SEC_TEXT static UINT32 osMemNodeInsideCheck(pool_t pPool, VOID *pPtr)
{
    if (((AARCHPTR)pPtr < (AARCHPTR)osMemFirstNode(pPool))  ||
        ((AARCHPTR)pPtr > (AARCHPTR)osMemEndNode(pPool)))
        return LOS_NOK;
    else
        return LOS_OK;
}

LITE_OS_SEC_TEXT INT32 osMemMagicCheck(pool_t pPool, VOID *pPtr)
{
    const LOS_MEM_DYN_NODE *pNode = NULL;

    if(pPtr == NULL || !IS_ALIGNED(pPtr, sizeof(VOID *)))
    {
        return LOS_NOK;
    }
    else
    {
        pNode = osMemPtrToNode(pPtr);
    }

    if (pNode == osMemFirstNode(pPool))
        return LOS_OK;
    else if ((AARCHPTR)pNode->pstPreNode & (ALIGN_SIZE - 1))
        return LOS_NOK;
    else if(osMemNodeInsideCheck(pPool, pNode->pstPreNode) == LOS_NOK)
        return LOS_NOK;
    else if ((AARCHPTR)pNode == (AARCHPTR)osMemNodeNext(pNode->pstPreNode))
        return LOS_OK;
    else
        return LOS_NOK;
}

LITE_OS_SEC_TEXT UINT32 osMemInfoGet(pool_t pPool, LOS_MEM_STATUS *pstStatus)
{
    LOS_MEM_DYN_NODE *pNode = osMemFirstNode(pPool);
    UINT32 uwTotalUsedSize = 0, uwTotalFreeSize = 0, uwMaxFreeNodeSize = 0;
    UINT32 uwTmpSize = 0;
    UINTPTR uvIntSave;

    if (pstStatus == NULL)
    {
        return LOS_NOK;
    }

    if (pPool == NULL)
    {
        PRINT_ERR("wrong mem pool addr: 0x%x\n", (UINT32)pPool);
        return LOS_NOK;
    }

    if (osMemMagicCheck(pPool, (VOID *)osMemEndPtr(pPool)) == LOS_NOK)
    {
        PRINT_ERR("wrong mem pool addr: 0x%x\n", (UINT32)pPool);
        return LOS_NOK;
    }

    uvIntSave = LOS_IntLock();

    while (pNode && !osMemEndCheck(pNode)) {
        uwTmpSize = osMemNodeSizeGet(pNode);
        if (osMemFreeCheck(pNode))
        {
            uwTotalFreeSize += uwTmpSize;
            if (uwTmpSize > uwMaxFreeNodeSize)
                uwMaxFreeNodeSize = uwTmpSize;
        }
        else
        {
            uwTotalUsedSize += uwTmpSize;
        }
        pNode = osMemNodeNext(pNode);
    }

    pstStatus->usedSize = uwTotalUsedSize + block_start_offset;
    pstStatus->freeSize = uwTotalFreeSize;
    pstStatus->totalSize = uwMaxFreeNodeSize;
    pstStatus->allocCount = g_uwTlsf_AllocCount;
    pstStatus->freeCount = g_uwTlsf_FreeCount;

    LOS_IntRestore(uvIntSave);

    return LOS_OK;

}

LITE_OS_SEC_TEXT VOID osMemInfoPrint(pool_t pPool)
{
    LOS_MEM_POOL_INFO *pstPoolInfo = (LOS_MEM_POOL_INFO *)pPool;
    LOS_MEM_STATUS stStatus = {0};
    if (LOS_NOK == osMemInfoGet(pPool, &stStatus))
        return;

    PRINT_ERR("pool addr    pool size    total size     used size    free size   alloc Count    free Count\n    0x%-8x   0x%-8x   0x%-8x    0x%-8x   0x%-16x   0x%-13x    0x%-13x\n",
                (UINT32)pPool, pstPoolInfo->uwPoolSize, stStatus.totalSize, stStatus.usedSize, stStatus.freeSize, stStatus.allocCount, stStatus.freeCount); /*lint !e515*/
    return;
}

LITE_OS_SEC_TEXT_INIT UINT32 osMemPoolSizeGet(VOID * tlsf)
{
    control_t *control = tlsf_cast(control_t *, tlsf);
    if (tlsf == NULL)
        return LOS_NOK;

    return control->uwPoolSize;
}
#endif /* TLSF_CONFIG_DEBUG */

LITE_OS_SEC_TEXT_INIT UINT32 osMemHeadSize(VOID)
{
    return sizeof(control_t);
}

LITE_OS_SEC_TEXT_INIT pool_t osMemPoolAdd(VOID *tlsf, VOID *pPool, UINT32 uwBytes)
{
    LOS_MEM_DYN_NODE *pNode;
    LOS_MEM_DYN_NODE *pNext;

    const UINT32 uwPoolBytes = osMemAlignDown(uwBytes - block_start_offset, ALIGN_SIZE) + block_header_overhead;

    if (((AARCHPTR)pPool % ALIGN_SIZE) != 0) {
        PRINT_ERR("Memory must be aligned by %u bytes.\n", (INT32)ALIGN_SIZE);
        return NULL;
    }

    if (uwPoolBytes < block_size_min || uwPoolBytes > block_size_max)
    {
        PRINT_ERR("Memory size must be between 0x%x and 0x%x bytes.\n",
               (INT32)(block_start_offset + block_size_min),
               (INT32)(block_start_offset + block_size_max));/*lint !e515*/

        return NULL;
    }

    /*
     * Create the main free pNode. Offset the start of the pNode slightly
     * so that the pstPreNode field falls outside of the pool -
     * it will never be used.
    */
    pNode = osMemPtrOffset(pPool, -(INT32)block_header_overhead); /*lint !e570*/
    osMemNodeSizeSet(pNode, uwPoolBytes);
    osMemFreeMark(pNode);
    osMemPreUsedMark(pNode);
    osMemNodeInsert(tlsf_cast(control_t *, tlsf), pNode);
    /* Split the pNode to create a zero-size sentinel pNode. */
    pNext = osMemNextLink(pNode);
    osMemNodeSizeSet(pNext, 0);
    osMemUsedMark(pNext);
    osMemPreFreeMark(pNext);

    return pPool;
}

/*
 * TLSF main interface.
 */

LITE_OS_SEC_TEXT_INIT VOID * osMemPoolCreat(VOID *pPool, UINT32 uwBytes)
{
    if (((AARCHPTR)pPool % ALIGN_SIZE) != 0)
    {
        PRINT_ERR("Memory must be aligned to %u bytes.\n", (UINT32)ALIGN_SIZE);
        return NULL;
    }

    osMemControlClear(tlsf_cast(control_t *, pPool), uwBytes);

    return tlsf_cast(VOID *, pPool);
}

/*****************************************************************************
 Function : LOS_MemInit
 Description : Initialize Dynamic Memory pool
 Input       : pPool    --- Pointer to memory pool
               uwBytes  --- Size of memory in bytes to allocate
 Output      : None
 Return      : LOS_OK - Ok, LOS_NOK - Error
*****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_MemInit(VOID *pPool, UINT32 uwBytes)
{
    VOID *tlsf;
    UINTPTR uvIntSave;

#if (LOSCFG_MEM_MUL_POOL == YES)
    VOID *pNext = g_pPoolHead;
    VOID * pCur;
    UINT32 uwPoolEnd;
#endif

    if (!pPool || uwBytes <= osMemHeadSize())
    {
        return LOS_NOK;
    }

    uvIntSave = LOS_IntLock();

    if(!IS_ALIGNED(uwBytes, ALIGN_SIZE))
    {
        PRINT_ERR("pool [0x%x, 0x%x) size 0x%x sholud be aligned with ALIGN_SIZE\n",
                  (UINT32)pPool, (AARCHPTR)pPool + uwBytes, uwBytes);/*lint !e515*/
        uwBytes = osMemAlignDown(uwBytes, ALIGN_SIZE);
    }

#if (LOSCFG_MEM_MUL_POOL == YES)
    while (pNext != NULL)
    {
        uwPoolEnd = (AARCHPTR)pNext + osMemPoolSizeGet(pNext);
        if ((pPool <= pNext && ((AARCHPTR)pPool + uwBytes) > (AARCHPTR)pNext) ||
            ((AARCHPTR)pPool < uwPoolEnd && ((AARCHPTR)pPool + uwBytes) >= uwPoolEnd))
        {
            PRINT_ERR("pool [0x%x, 0x%x) conflict with pool 0x%x, 0x%x)\n",
                          (UINT32)pPool, (AARCHPTR)pPool + uwBytes,
                          (UINT32)pNext, (AARCHPTR)pNext + osMemPoolSizeGet(pNext));/*lint !e515*/
            LOS_IntRestore(uvIntSave);
            return LOS_NOK;
        }
        pCur = pNext;
        pNext = ((LOS_MEM_POOL_INFO *)pNext)->pNextPool;
    }
#endif

    tlsf = osMemPoolCreat(pPool, uwBytes);
    if (osMemPoolAdd(tlsf, (UINT8 *)pPool + osMemHeadSize(), uwBytes - osMemHeadSize()) == 0)
    {
        LOS_IntRestore(uvIntSave);
        return LOS_NOK;
    }

#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    if (uwBytes >= SLAB_BASIC_NEED_SIZE)//if size of pool is small than size of slab need, don`t init slab
    {
        UINT32 uwIdx = 0;
        struct LOS_SLAB_CONTROL_HEADER *pstSlabMemHead = osSlabCtrlHdrGet(pPool);

        for (uwIdx = 0; uwIdx < SLAB_MEM_COUNT; uwIdx++)
        {
            pstSlabMemHead->stSlabClass[uwIdx].alloc = NULL;
        }

        if (osSlabMemInit(pPool) == FALSE)
        {
            LOS_IntRestore(uvIntSave);
            return LOS_NOK;
        }
    }
#endif

#if (LOSCFG_MEM_MUL_POOL == YES)
    if (g_pPoolHead == NULL)
    {
        g_pPoolHead = pPool;
    }
    else
    {
        ((LOS_MEM_POOL_INFO *)pCur)->pNextPool = pPool;
    }

    ((LOS_MEM_POOL_INFO *)pPool)->pNextPool = NULL;
#endif

    LOS_IntRestore(uvIntSave);
    return LOS_OK;
}

#if (LOSCFG_MEM_MUL_POOL == YES)
LITE_OS_SEC_TEXT_INIT UINT32 LOS_MemDeInit(VOID *pPool)
{
    UINTPTR uvIntSave, uwRet = LOS_NOK;
    VOID *pNext, *pCur;

    uvIntSave = LOS_IntLock();
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    osSlabMemDeinit(pPool);
#endif
    do
    {
        if (pPool == NULL)
            break;

        if (pPool == g_pPoolHead)
        {
            g_pPoolHead = ((LOS_MEM_POOL_INFO *)g_pPoolHead)->pNextPool;
            uwRet = LOS_OK;
            break;
        }

        pCur = g_pPoolHead;
        pNext = g_pPoolHead;

        while (pNext != NULL)
        {
            if (pPool == pNext)
            {
                ((LOS_MEM_POOL_INFO *)pCur)->pNextPool = ((LOS_MEM_POOL_INFO *)pNext)->pNextPool;
                uwRet = LOS_OK;
                break;
            }
            pCur = pNext;
            pNext = ((LOS_MEM_POOL_INFO *)pNext)->pNextPool;
        }
    }while(0);

    LOS_IntRestore(uvIntSave);
    return uwRet;
}

LITE_OS_SEC_TEXT_INIT UINT32 LOS_MemPoolList(VOID)
{
    VOID *pNext = g_pPoolHead;
    UINT32 uwIndex = 0;
    while (pNext != NULL)
    {
        PRINT_ERR("pool%d :\n", uwIndex++);
        osMemInfoPrint(pNext);
        pNext = ((LOS_MEM_POOL_INFO *)pNext)->pNextPool;
    }
    return uwIndex;
}
#endif

LITE_OS_SEC_TEXT VOID *osHeapAlloc(VOID *tlsf, UINT32 uwSize)
{
    UINT32 adjust;
    LOS_MEM_DYN_NODE *pNode;
    VOID *pPtr = NULL;
    control_t *control = tlsf_cast(control_t *, tlsf);

    adjust = osMemReqSizeAdjust(uwSize, ALIGN_SIZE);
    if(0 == adjust)
    {
        PRINT_ERR("require node size 0x%x is too large\n",uwSize);
        return NULL;
    }
    pNode = osMemFreeNodeLocate(control, adjust);
    pPtr = osMemUsedNodePrepare(control, pNode, adjust);

    if (NULL != pPtr)
    {
        g_uwTlsf_AllocCount++;
    }

    return pPtr;
}

/*****************************************************************************
 Function : LOS_MemAlloc
 Description : Allocate Memory from Memory pool
 Input       : tlsf    --- Pointer to memory pool
               uwSize  --- Size of memory in bytes to allocate
 Output      : None
 Return      : Pointer to allocated memory
*****************************************************************************/
LITE_OS_SEC_TEXT VOID *LOS_MemAlloc(VOID * tlsf, UINT32 uwSize)
{
    VOID *pPtr = NULL;
    UINTPTR uvIntSave;

    if ((0 == uwSize) || (NULL == tlsf) || (uwSize > block_size_max))
    {
        return NULL;
    }
    uvIntSave = LOS_IntLock();

#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    if (uwSize <= (SLAB_MEM_ALLOCATOR_SIZE/SLAB_MEM_COUNT))
    {
        pPtr = osSlabMemAlloc(tlsf, uwSize);
    }

    if (pPtr == NULL)
#endif
    {
        pPtr = osHeapAlloc(tlsf, uwSize);
    }
    LOS_IntRestore(uvIntSave);
    return pPtr;
}

/*****************************************************************************
 Function : LOS_MemAllocAlign
 Description : align size then allocate node from Memory pool
 Input       : tlsf         --- Pointer to memory pool
               uwSize       --- Size of memory in bytes to allocate
               uwAlign      --- align form
 Output      : None
 Return      : Pointer to allocated memory node
*****************************************************************************/
LITE_OS_SEC_TEXT VOID *LOS_MemAllocAlign(VOID * tlsf, UINT32 uwSize, UINT32 uwAlign)
{
    control_t *control = tlsf_cast(control_t *, tlsf);
    UINT32 adjust;
    UINT32 gap_minimum;
    UINT32 size_with_gap;
    UINT32 aligned_size;
    LOS_MEM_DYN_NODE *pNode;
    VOID *pPtr;
    UINTPTR uvIntSave;

    if (uwSize == 0 || !tlsf || uwSize > block_size_max || uwAlign == 0 || !IS_ALIGNED(uwAlign, sizeof(VOID *)))
    {
        return NULL;
    }

    uvIntSave = LOS_IntLock();

    uwSize += 4;
    adjust = osMemReqSizeAdjust(uwSize, ALIGN_SIZE);
    if(0 == adjust)
    {
        PRINT_ERR("require node size 0x%x is too large\n",uwSize);
        LOS_IntRestore(uvIntSave);
        return NULL;
    }
    /*
     * We must allocate an additional minimum pNode size bytes so that if
     * our free pNode will leave an alignment gap which is smaller, we can
     * trim a leading free pNode and release it back to the pool. We must
     * do this because the previous physical pNode is in use, therefore
     * the pstPreNode field is not valid, and we can't simply adjust
     * the size of that pNode.
     */
    gap_minimum = sizeof(LOS_MEM_DYN_NODE);
    if (((adjust + gap_minimum) > (((UINT32)-1) - uwAlign)) || ((uwAlign + gap_minimum) > (((UINT32)-1) - adjust)))
    {
        LOS_IntRestore(uvIntSave);
        return NULL;
    }
    size_with_gap = osMemReqSizeAdjust(adjust + uwAlign + gap_minimum, ALIGN_SIZE);
    if(0 == size_with_gap)
    {
        PRINT_ERR("require node size 0x%x is too large\n",uwSize);
        LOS_IntRestore(uvIntSave);
        return NULL;
    }
    /*
     * If alignment is less than or equals base alignment, we're done.
     * If we requested 0 bytes, return null, as tlsf_malloc(0) does.
     */
    aligned_size = (adjust && uwAlign > ALIGN_SIZE) ? size_with_gap : adjust;

    pNode = osMemFreeNodeLocate(control, aligned_size);

    if (pNode) {
        VOID *aligned;
        UINT32 gap;
        pPtr = osMemNodeToPtr(pNode);
        aligned = osMemAlignPtr(pPtr, uwAlign);
        gap = tlsf_cast(
            UINT32, tlsf_cast(AARCHPTR, aligned) - tlsf_cast(AARCHPTR, pPtr));

        /* If gap size is too small, offset to next aligned boundary. */
        if (gap && gap < gap_minimum)
        {
            const UINT32 gap_remain = gap_minimum - gap;
            const UINT32 offset = tlsf_max(gap_remain, uwAlign);
            const VOID *next_aligned =
                tlsf_cast(VOID *, tlsf_cast(AARCHPTR, aligned) + offset);

            aligned = osMemAlignPtr(next_aligned, uwAlign);
            gap = tlsf_cast(UINT32, tlsf_cast(AARCHPTR, aligned) -
                                        tlsf_cast(AARCHPTR, pPtr));
        }

        if (gap)
        {
            tlsf_assert(gap >= gap_minimum && "gap size too small");
            pNode = osMemFreeNodeTrimLeading(control, pNode, gap);
        }
    }

    pPtr = osMemUsedNodePrepare(control, pNode, adjust);

    LOS_IntRestore(uvIntSave);
    return pPtr;
}

LITE_OS_SEC_TEXT BOOL osHeapFree(VOID * tlsf, VOID *pPtr)
{
    control_t *control;

    LOS_MEM_DYN_NODE *pNode;

    control = (LOS_MEM_POOL_INFO *)tlsf;
    pNode = osMemPtrToNode(pPtr);

    if (osMemFreeCheck(pNode))
    {
        return FALSE;
    }

    if (osMemMagicCheck(tlsf, pPtr) == LOS_NOK)
    {
        return FALSE;
    }

    osMemFreeSet(pNode);
    pNode = osMemPreNodeMerge(control, pNode);
    pNode = osMemNextNodeMerge(control, pNode);
    osMemNodeInsert(control, pNode);

    g_uwTlsf_FreeCount++;
    return TRUE;
}

/*****************************************************************************
 Function : LOS_MemFree
 Description : Free Memory and return it to Memory pool
 Input       : tlsf     --- Pointer to memory pool
               pPtr     --- Pointer to memory to free
 Output      : None
 Return      : LOS_OK - OK, LOS_NOK - Error
*****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_MemFree(VOID * tlsf, VOID *pPtr)
{
    UINTPTR uvIntSave;
    BOOL uwRet;


    if ((NULL == tlsf) || (NULL == pPtr))
    {
        return LOS_NOK;
    }
    uvIntSave = LOS_IntLock();
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    uwRet = osSlabMemFree(tlsf, pPtr);
    if (uwRet == FALSE)
#endif
    {
        {
            uwRet = osHeapFree(tlsf,pPtr);
        }
    }
    LOS_IntRestore(uvIntSave);
    return (uwRet == TRUE ? LOS_OK : LOS_NOK);
}

/*
 * The TLSF pNode information provides us with enough information to
 * provide a reasonably intelligent implementation of realloc, growing or
 * shrinking the currently allocated pNode as required.
 *
 * This routine handles the somewhat esoteric edge cases of realloc:
 * - a non-zero size with a null pointer will behave like malloc
 * - a zero size with a non-null pointer will behave like free
 * - a request that cannot be satisfied will leave the original buffer
 *   untouched
 * - an extended buffer size will leave the newly-allocated area with
 *   contents undefined
 */
LITE_OS_SEC_TEXT_MINOR VOID *osHeapRealloc(VOID * tlsf, VOID *pPtr, UINT32 uwSize)
{
    VOID *pNewPtr = NULL;
    control_t *control = tlsf_cast(control_t *, tlsf);
    LOS_MEM_DYN_NODE *pNode = osMemPtrToNode(pPtr);
    LOS_MEM_DYN_NODE *pNext = osMemNodeNext(pNode);

    const size_t cursize = osMemNodeSizeGet(pNode);
    const size_t combined = cursize + osMemNodeSizeGet(pNext);
    const size_t adjust = osMemReqSizeAdjust(uwSize, ALIGN_SIZE);
    if(0 == adjust)
    {
        PRINT_ERR("require node size 0x%x is too large\n",uwSize);
        return NULL;
    }
    if (osMemFreeCheck(pNode))
    {
        PRINTK("block already marked as free\n");
        return NULL;
    }

    /*
     * If the next block is used, or when combined with the current
     * block, does not offer enough space, we must reallocate and copy.
    */
    if (adjust > cursize && (!osMemFreeCheck(pNext) || adjust > combined))
    {
        pNewPtr = LOS_MemAlloc(tlsf, uwSize);
        if (pNewPtr)
        {
            const size_t minsize = tlsf_min((cursize - block_start_offset), uwSize);
            memcpy(pNewPtr, pPtr, minsize);
            (VOID)LOS_MemFree(tlsf, pPtr);
        }
    } else
    {
        /* Do we need to expand to the next block? */
        if (adjust > cursize)
        {
            (VOID)osMemNextNodeMerge(control, pNode);
            osMemUsedSet(pNode);
        }

        /* Trim the resulting block and return the original pointer. */
        osMemUsedNodeTrim(control, pNode, adjust);

        pNewPtr = pPtr;
    }

    return pNewPtr;
}

/*****************************************************************************
 Function : LOS_MemRealloc
 Description : realloc memory from Memory pool
 Input       : tlsf     --- Pointer to memory pool
               pPtr     --- Pointer to memory
               uwSize   --- new size
 Output      : None
 Return      : Pointer to allocated memory node
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR VOID *LOS_MemRealloc(VOID * tlsf, VOID *pPtr, UINT32 uwSize)
{
    VOID *pRetPtr = NULL;
    UINTPTR uvIntSave;

#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    UINT32 uwMinSize = 0;
    UINT32 uwOldSize = 0;
#endif

    if ((INT32)uwSize < 0)
    {
        return NULL;
    }

    uvIntSave = LOS_IntLock();

    /* Zero-size requests are treated as free. */
    if (pPtr && uwSize == 0)
    {
        (VOID)LOS_MemFree(tlsf, pPtr);
    }
    /* Requests with NULL pointers are treated as malloc. */
    else if (pPtr == NULL)
    {
        pRetPtr = LOS_MemAlloc(tlsf, uwSize);
    }
    else
    {
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
        uwOldSize = osSlabMemCheck(tlsf, pPtr);

        if (uwOldSize != (UINT32)-1)
        {
            uwMinSize = uwSize > uwOldSize ? uwOldSize : uwSize;
        }
        pRetPtr = LOS_MemAlloc(tlsf, uwSize);

        if (pRetPtr != NULL)
        {
            (VOID)memcpy(pRetPtr, pPtr, uwMinSize);
            (VOID)LOS_MemFree(tlsf, pPtr);
        }
        else
#endif
        {
            pRetPtr = osHeapRealloc(tlsf, pPtr, uwSize);
        }
    }
    LOS_IntRestore(uvIntSave);
    return pRetPtr;
}

LITE_OS_SEC_TEXT_INIT UINT32 osMemSystemInit(VOID)
{
    UINT32 uwRet = LOS_OK;

    uwRet = LOS_MemInit((VOID *)OS_SYS_MEM_ADDR, OS_SYS_MEM_SIZE);

#if ((LOSCFG_PLATFORM_EXC == YES) && (LOSCFG_SAVE_EXC_INFO == YES))
    osExcRegister(OS_EXC_TYPE_MEM, (EXC_INFO_SAVE_CALLBACK)LOS_MemExcInfoGet, g_aucMemMang);
#endif
    osMemInfoPrint((VOID *)OS_SYS_MEM_ADDR);

    return uwRet;
}

LITE_OS_SEC_TEXT UINT32 LOS_MemStatisticsGet(VOID *pPool, LOS_MEM_STATUS *pstStatus)
{
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    LOS_SLAB_STATUS stSlabStatus;
    UINT32 uwErr;
#endif
    LOS_MEM_POOL_INFO *pstPoolInfo = (LOS_MEM_POOL_INFO *)pPool;
    LOS_MEM_STATUS stStatus;
    if (LOS_NOK == osMemInfoGet(pPool, &stStatus))
    {
        return LOS_NOK;
    }

    pstStatus->totalSize  = stStatus.totalSize;
    pstStatus->usedSize   = stStatus.usedSize;
    pstStatus->freeSize   = stStatus.freeSize;
    pstStatus->allocCount = stStatus.allocCount;
    pstStatus->freeCount  = stStatus.freeCount;

#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    uwErr = osSlabStatisticsGet(pPool, &stSlabStatus);
    if (uwErr != LOS_OK)
    {
        return LOS_NOK;
    }

    pstStatus->totalSize  = stStatus.totalSize;
    pstStatus->usedSize   = stStatus.usedSize - stSlabStatus.freeSize;  //all slab region inside of heap used region
    pstStatus->freeSize   = stStatus.freeSize + stSlabStatus.freeSize;
    pstStatus->allocCount = stStatus.allocCount + stSlabStatus.allocCount;
    pstStatus->freeCount  = stStatus.freeCount + stSlabStatus.freeCount;
#endif
    return LOS_OK;
}


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

