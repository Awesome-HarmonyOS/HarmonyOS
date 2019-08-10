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

/**@defgroup los_slab Slab
 * @ingroup kernel
 */

#ifndef _LOS_SLAB_PH
#define _LOS_SLAB_PH

#include "los_base.h"
#include "los_slab.h"
#include <los_typedef.h>

typedef struct __s_LOS_SLAB_STATUS {
    UINT32 totalSize;
    UINT32 usedSize;
    UINT32 freeSize;
    UINT32 allocCount;
    UINT32 freeCount;
}LOS_SLAB_STATUS;

typedef struct tagOS_SLAB_BLOCK_NODE{
    UINT16 usMagic;
    UINT8  ucBlkSz;
    UINT8  ucRecordId;
}OS_SLAB_BLOCK_NODE;

struct AtomicBitset {
    UINT32 numBits;
    UINT32 words[];/*lint !e43*/
};

typedef struct __s_OS_SLAB_ALLOCATOR {
    UINT32 uwItemSz;
    UINT8 *ucDataChunks;
    struct AtomicBitset *bitset;/*lint !e43*/
}OS_SLAB_ALLOCATOR;

typedef struct __s_OS_SLAB_MEM {
    UINT32 blkSz;
    UINT32 blkCnt;
    UINT32 blkUsedCnt;
    OS_SLAB_ALLOCATOR *alloc;
}OS_SLAB_MEM;

struct LOS_SLAB_CONTROL_HEADER{
    OS_SLAB_MEM stSlabClass[SLAB_MEM_COUNT];
};

#define OS_SLAB_MAGIC (0xdede)
#define OS_SLAB_BLOCK_HEAD_GET(pPtr)                                   ((OS_SLAB_BLOCK_NODE *)((UINT8 *)pPtr - sizeof(OS_SLAB_BLOCK_NODE)))
#define OS_SLAB_BLOCK_MAGIC_SET(pstSlabNode)                           (((OS_SLAB_BLOCK_NODE *)pstSlabNode)->usMagic = (UINT16)OS_SLAB_MAGIC)
#define OS_SLAB_BLOCK_MAGIC_GET(pstSlabNode)                           (((OS_SLAB_BLOCK_NODE *)pstSlabNode)->usMagic)
#define OS_SLAB_BLOCK_SIZE_SET(pstSlabNode, uwSize)                    (((OS_SLAB_BLOCK_NODE *)pstSlabNode)->ucBlkSz = uwSize)
#define OS_SLAB_BLOCK_SIZE_GET(pstSlabNode)                            (((OS_SLAB_BLOCK_NODE *)pstSlabNode)->ucBlkSz)
#define OS_SLAB_BLOCK_ID_SET(pstSlabNode, uwId)                        (((OS_SLAB_BLOCK_NODE *)pstSlabNode)->ucRecordId = uwId)
#define OS_SLAB_BLOCK_ID_GET(pstSlabNode)                              (((OS_SLAB_BLOCK_NODE *)pstSlabNode)->ucRecordId)
#define OS_ALLOC_FROM_SLAB_CHECK(pstSlabNode)                          (((OS_SLAB_BLOCK_NODE *)pstSlabNode)->usMagic == (UINT16)OS_SLAB_MAGIC)

#define ATOMIC_BITSET_SZ(numbits)   (sizeof(struct AtomicBitset) + ((numbits) + 31) / 8)
#define ATOMIC_BITSET_DECL(nam, numbits, extra_keyword)         extra_keyword UINT8 _##nam##_store [ATOMIC_BITSET_SZ(numbits)] __attribute__((aligned(4))); extra_keyword struct AtomicBitset *nam = (struct AtomicBitset*)_##nam##_store

/**
 * @ingroup  los_slab
 * @brief Initialization atomic bitset.
 *
 * @par Description:
 * This API is used to initialization atomic bitset.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstSet            [IN/OUT] Type #AtomicBitset *   Atomic bitset.
 * @param  numBits     [IN] Type #UINT32   Bits number.
 *
 * @retval None.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see osSlabAllocatorDestroy
 * @since Huawei LiteOS V100R002C00
 */
VOID osAtomicBitsetInit(struct AtomicBitset *pstSet, UINT32 uwNumBits);

/**
 * @ingroup  los_slab
 * @brief Get atomic bitset number.
 *
 * @par Description:
 * This API is used to get atomic bitset number.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstSet            [IN] Type #AtomicBitset *   Atomic bitset.
 *
 * @retval UINT32   Atomic bitset number.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see osSlabAllocatorDestroy
 * @since Huawei LiteOS V100R002C00
 */
UINT32 osAtomicBitsetGetNumBits(const struct AtomicBitset *pstSet);

/**
 * @ingroup  los_slab
 * @brief Get atomic bitset bit.
 *
 * @par Description:
 * This API is used to get atomic bitset bit.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstSet              [IN] Type #AtomicBitset *   Atomic bitset.
 * @param  uwNum            [IN] Type #UINT32   Bit number.
 *
 * @retval BOOL  success or failed
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see osSlabAllocatorDestroy
 * @since Huawei LiteOS V100R002C00
 */
BOOL osAtomicBitsetGetBit(const struct AtomicBitset *pstSet, UINT32 uwNum);

/**
 * @ingroup  los_slab
 * @brief Clear the atomic bitset bit.
 *
 * @par Description:
 * This API is used to clear the atomic bitset bit.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstSet              [IN] Type #AtomicBitset *   Atomic bitset.
 * @param  uwNum            [IN] Type #UINT32   Bit number.
 *
 * @retval None.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see osSlabAllocatorDestroy
 * @since Huawei LiteOS V100R002C00
 */
VOID osAtomicBitsetClearBit(struct AtomicBitset *pstSet, UINT32 uwNum);

/**
 * @ingroup  los_slab
 * @brief Set the atomic bitset bit.
 *
 * @par Description:
 * This API is used to set the atomic bitset bit.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstSet              [IN] Type #AtomicBitset *   Atomic bitset.
 * @param  uwNum            [IN] Type #UINT32   Bit number.
 *
 * @retval None.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see osSlabAllocatorDestroy
 * @since Huawei LiteOS V100R002C00
 */
VOID osAtomicBitsetSetBit(struct AtomicBitset *pstSet, UINT32 uwNum);

/**
 * @ingroup  los_slab
 * @brief Clear and set the atomic bitset bit.
 *
 * @par Description:
 * This API is used to clear and set the atomic bitset bit.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstSet         [IN] Type #AtomicBitset *   Atomic bitset.
 *
 * @retval INT32   The address of the first available bit.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see osSlabAllocatorDestroy
 * @since Huawei LiteOS V100R002C00
 */
INT32 osAtomicBitsetFindClearAndSet(struct AtomicBitset *pstSet);

/**
 * @ingroup  los_slab
 * @brief Change the order of the output idx of osAtomicBitsetFindClearAndSet to order of natural numbers.
 *
 * @par Description:
 * This API is used to change the order of the output idx of osAtomicBitsetFindClearAndSet to order of natural numbers.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstSet         [IN] Type #AtomicBitset *   Atomic bitset.
 * @param  wIdx           [IN] Type #INT32 *   index.
 *
 * @retval INT32   Index to natural numbers.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see osSlabAllocatorDestroy
 * @since Huawei LiteOS V100R002C00
 */
INT32 osAtomicBitsetIdxChgToNatural(struct AtomicBitset *pstBitset, INT32 swIdx);

/**
 * @ingroup  los_slab
 * @brief Judgment the atomic bitset is empty.
 *
 * @par Description:
 * This API is used to judgment the atomic bitset is empty.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstSet         [IN] Type #AtomicBitset *   Atomic bitset.
 *
 * @retval BOOL   Judgment result.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see osSlabAllocatorDestroy
 * @since Huawei LiteOS V100R002C00
 */
BOOL osAtomicBitsetEmpty(struct AtomicBitset *pstBitset);

//thread/interrupt safe. allocations will not fail if space exists. even in interrupts.
//itemAlign over 4 will not be guaranteed since the heap does not hand out chunks with that kind of alignment

/**
 * @ingroup  los_slab
 * @brief create a new slab allocator.
 *
 * @par Description:
 * This API is used to create a new slab allocator.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pPool    [IN] Pointer to the memory pool that contains the memory block to be allocated.
 * @param  itemSz     [IN] The size of one slab page.
 * @param  itemAlign  [IN] Type alignment, 4 byte-aligned, 8 byte-aligned, etc.
 * @param  numItems   [IN] The number of slab page.
 *
 * @retval #OS_SLAB_ALLOCATOR*        The address of slab allocator.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see osSlabAllocatorDestroy
 * @since Huawei LiteOS V100R002C00
 */
OS_SLAB_ALLOCATOR *osSlabAllocatorNew(VOID *pPool, UINT32 uwItemSz, UINT32 uwItemAlign, UINT32 uwNumItems);

/**
 * @ingroup  los_slab
 * @brief Destroy a slab allocator.
 *
 * @par Description:
 * This API is used to Destroy a slab allocator.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pPool    [IN] Pointer to the memory pool that contains the memory block to be allocated.
 * @param  pstAllocator     [IN] a slab allocator.
 *
 * @retval #VOID
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see osSlabAllocatorNew
 * @since Huawei LiteOS V100R002C00
 */
VOID osSlabAllocatorDestroy(VOID *pPool, OS_SLAB_ALLOCATOR *pstAllocator);

/**
 * @ingroup  los_slab
 * @brief Allocate a slab page form allocator.
 *
 * @par Description:
 * This API is used to allocate a slab page form allocator.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstAllocator     [IN] a slab allocator.
 *
 * @retval #VOID*        return a slab page.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see osSlabAllocatorFree
 * @since Huawei LiteOS V100R002C00
 */
VOID* osSlabAllocatorAlloc(OS_SLAB_ALLOCATOR *pstAllocator);

/**
 * @ingroup  los_slab
 * @brief Free a slab page.
 *
 * @par Description:
 * This API is used to free a slab page.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstAllocator     [IN] a slab allocator.
 * @param  ptrP          [IN] a slab page.
 *
 * @retval #FALSE        Failed to free a slab page.
 * @retval #TRUE         Succees to free a slab page.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see LOS_SlabAllocatorAlloc
 * @since Huawei LiteOS V100R002C00
 */
BOOL osSlabAllocatorFree(OS_SLAB_ALLOCATOR *pstAllocator, VOID* ptrP);

/**
 * @ingroup  los_slab
 * @brief Get a slab page index with judgment.
 *
 * @par Description:
 * This API is used to get a slab page index with judgment.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstAllocator     [IN] a slab allocator.
 * @param  uwIdx             [IN] index
 *
 * @retval  VOID *
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R002C00
 */
VOID* osSlabAllocatorGetNth(OS_SLAB_ALLOCATOR *pstAllocator, UINT32 uwIdx); // -> pointer or NULL if that slot is empty   may be not int-safe. YMMV

/**
 * @ingroup  los_slab
 * @brief Get a slab page index without judgment.
 *
 * @par Description:
 * This API is used to get a slab page index without judgment.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstAllocator     [IN] a slab allocator.
 * @param  uwIdx             [IN] index
 *
 * @retval  VOID *
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R002C00
 */
VOID* osSlabAllocatorGetIdxP(OS_SLAB_ALLOCATOR *pstAllocator, UINT32 uwIdx);

/**
 * @ingroup  los_slab
 * @brief Get the index of slab page in slab allocator.
 *
 * @par Description:
 * This API is used to get the index of slab page in slab allocator.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  allocator     [IN] a slab allocator.
 * @param  ptrP          [IN] a slab page.
 *
 * @retval #-1           Illegal index .
 * @retval #UINT32       Succees to get index.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R002C00
 */
UINT32 osSlabAllocatorGetIndex(OS_SLAB_ALLOCATOR *allocator, VOID *ptr);

/**
 * @ingroup  los_slab
 * @brief Get the number of slab page.
 *
 * @par Description:
 * This API is used to get the number of slab page.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  allocator     [IN] a slab allocator.
 *
 * @retval #UINT32       Succees to get the number of slab page.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R002C00
 */
UINT32 osSlabAllocatorGetNumItems(OS_SLAB_ALLOCATOR *allocator);

/**
 * @ingroup  los_slab
 * @brief Check the slab allocator.
 *
 * @par Description:
 * This API is used to check the slab allocator.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  allocator     [IN] a slab allocator.
 *
 * @retval #FALSE        The slab allocator is already used.
 * @retval #TRUE         The slab allocator is not used.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R002C00
 */
BOOL osSlabAllocatorEmpty(OS_SLAB_ALLOCATOR *allocator);

/**
 * @ingroup  los_slab
 * @brief Get the used number of slab page in slab allocator.
 *
 * @par Description:
 * This API is used to get the used number of slab page in slab allocator.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  allocator     [IN] a slab allocator.
 *
 * @retval #UINT32        The used number of slab page in slab allocator.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R002C00
 */
UINT32 osSlabAllocatorGetUsedItemCnt(OS_SLAB_ALLOCATOR *allocator);

/**
 * @ingroup  los_slab
 * @brief Get the info of slab allocator.
 *
 * @par Description:
 * This API is used to get the info of slab allocator.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  allocator     [IN] a slab allocator.
 * @param  item_sz       [OUT] a slab page size.
 * @param  item_cnt      [OUT] a slab page num.
 * @param  cur_usage     [OUT] a used number of slab page.
 *
 * @retval #VOID
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R002C00
 */
VOID osSlabAllocatorGetSlabInfo(OS_SLAB_ALLOCATOR *allocator, UINT32 *item_sz, UINT32 *item_cnt, UINT32 *cur_usage);

/**
 * @ingroup  los_slab
 * @brief init slab allocator.
 *
 * @par Description:
 * This API is used to init slab allocator.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pPool    [IN] Pointer to the memory pool that contains the memory block to be allocated.
 *
 * @retval BOOL
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R002C00
 */
extern BOOL osSlabMemInit(VOID *pPool);

/**
 * @ingroup  los_slab
 * @brief alloc mem by slab allocator.
 *
 * @par Description:
 * This API is used to alloc mem by slab allocator.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pPool    [IN] Pointer to the memory pool that contains the memory block to be allocated.
 * @param  sz       [IN] Size of mem to alloc
 *
 * @retval VOID *  The address of alloced mem or NULL.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R002C00
 */
extern VOID *osSlabMemAlloc(VOID *pPool, UINT32 sz);

/**
 * @ingroup  los_slab
 * @brief free mem by slab allocator.
 *
 * @par Description:
 * This API is used to free mem by slab allocator.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pPool    [IN] Pointer to the memory pool that contains the memory block to be allocated.
 * @param  ptr      [IN] Pointer to the memory that to be free
 *
 * @retval BOOL   success or failed
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R002C00
 */
extern BOOL osSlabMemFree(VOID *pPool, VOID* ptr);

/**
 * @ingroup  los_slab
 * @brief deinit slab allocator.
 *
 * @par Description:
 * This API is used to deinit slab allocator.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pPool    [IN] Pointer to the memory pool that contains the memory block to be allocated.
 *
 * @retval VOID
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see
 * @since Huawei LiteOS V100R002C00
 */
extern VOID osSlabMemDeinit(VOID *pPool);

/**
 * @ingroup  los_slab
 * @brief Check slab allocator.
 *
 * @par Description:
 * This API is used to check slab allocator.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pstAllocator    [IN] a slab allocator.
 * @param  ptrP    [IN] Slab node.
 *
 * @retval VOID
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R002C00
 */
extern BOOL osSlabAllocatorCheck(OS_SLAB_ALLOCATOR *pstAllocator, VOID* ptrP);

/**
 * @ingroup  los_slab
 * @brief Get SlabCtrlHdr.
 *
 * @par Description:
 * This API is used to get SlabCtrlHdr.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pPool    [IN] Pointer to the memory pool that contains the memory block to be allocated.
 *
 * @retval VOID
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R002C00
 */
extern VOID *osSlabCtrlHdrGet(VOID *pPool);

/**
 * @ingroup  los_slab
 * @brief Check the slab memory.
 *
 * @par Description:
 * This API is used to check the slab memory.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pPool    [IN] Pointer to the memory pool that contains the memory block to be allocated.
 * @param  pPtr    [IN] Slab block head.
 *
 * @retval UINT32   block size.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R002C00
 */
extern UINT32 osSlabMemCheck(VOID *pPool, VOID* pPtr);

/**
 * @ingroup  los_slab
 * @brief Get the slab status.
 *
 * @par Description:
 * This API is used to get the slab status.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pPool         [IN] Pointer to the memory pool that contains the memory block to be allocated.
 * @param  pstStatus    [IN/OUT] Slab block status.
 *
 * @retval UINT32   Get status result.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R002C00
 */
extern UINT32 osSlabStatisticsGet(VOID *pPool, LOS_SLAB_STATUS *pstStatus);

/**
 * @ingroup  los_slab
 * @brief Get the max free block size.
 *
 * @par Description:
 * This API is used to get the max free block size.
 *
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param  pPool         [IN] Pointer to the memory pool that contains the memory block to be allocated.
 *
 * @retval UINT32   Max free block size.
 * @par Dependency:
 * <ul><li>los_slab.ph: the header file that contains the API declaration.</li></ul>
 * @see None.
 * @since Huawei LiteOS V100R002C00
 */
extern UINT32 osSlabGetMaxFreeBlkSize(VOID *pPool);

#endif

