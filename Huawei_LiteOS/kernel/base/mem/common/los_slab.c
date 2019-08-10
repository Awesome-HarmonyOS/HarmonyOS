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
#include <string.h>
#include <stdint.h>
#include <los_slab.ph>

extern VOID* osHeapAlloc(VOID *pPool, UINT32 uwSz);
extern BOOL osHeapFree(VOID *pPool, VOID* pPtr);

VOID osAtomicBitsetInit(struct AtomicBitset *pstSet, UINT32 uwNumBits)
{
    pstSet->numBits = uwNumBits;
    memset(pstSet->words, 0, (uwNumBits + 31) / 8);
    if (uwNumBits & 31) //mark all high bits so that osAtomicBitsetFindClearAndSet() is simpler
    {
        pstSet->words[uwNumBits / 32] = ((UINT32)((INT32)-1LL)) << (uwNumBits & 31);
    }
}

inline UINT32 osAtomicBitsetGetNumBits(const struct AtomicBitset *pstSet)
{
    return pstSet->numBits;
}

BOOL osAtomicBitsetGetBit(const struct AtomicBitset *pstSet, UINT32 uwNum)
{
    if (uwNum >= pstSet->numBits) /* any value is as good as the next */
    {
        return FALSE;
    }
    return !!((pstSet->words[uwNum / 32]) & (1UL << (uwNum & 31)));
}

VOID osAtomicBitsetClearBit(struct AtomicBitset *pstSet, UINT32 uwNum)
{
    UINT32 *puwWordPtr = pstSet->words + uwNum / 32;

    if (uwNum >= pstSet->numBits)
    {
        return;
    }
    (*puwWordPtr) &= ~(1UL << (uwNum & 31));
}

/* find from the high bit to high bit£¬return the address of the first available bit */
INT32 osAtomicBitsetFindClearAndSet(struct AtomicBitset *pstSet)
{
    UINT32 uwIdx, uwNumWords = (pstSet->numBits + 31) / 32;
    UINT32 *puwWordPtr = pstSet->words;
    UINT32 uwTmpWord;
    INT32 swCnt = 0;

    for (uwIdx = 0; uwIdx < uwNumWords; uwIdx++, puwWordPtr++)
    {
        if (*puwWordPtr == 0xFFFFFFFF)
        {
            continue;
        }

        uwTmpWord = ~(*puwWordPtr);

        while(uwTmpWord)
        {
            uwTmpWord = uwTmpWord >> 1UL;
            swCnt++;
        }

        *puwWordPtr |= (1UL << (swCnt - 1));

        return (INT32)(uwIdx * 32 + swCnt - 1);
    }

    return -1;
}

/* change the order of the output idx of osAtomicBitsetFindClearAndSet to order of natural numbers */
INT32 osAtomicBitsetIdxChgToNatural(struct AtomicBitset *pstBitset, INT32 swIdx)
{
    UINT32 uwRet, uwB;
    if (swIdx < 0)
    {
        return swIdx;
    }
    uwB = 31 + (swIdx & ~31);
    if (uwB > pstBitset->numBits - 1)
    {
        uwB = pstBitset->numBits - 1;
    }
    uwRet = uwB - (swIdx & 31);
    return uwRet;
}

BOOL osAtomicBitsetEmpty(struct AtomicBitset *pstBitset)
{
    UINT32 uwIdx = 0;
    for (uwIdx = 0; uwIdx < pstBitset->numBits / 32;)
    {
        if (pstBitset->words[uwIdx] != 0)
        {
            return FALSE;
        }
        uwIdx++;
    }
    if (pstBitset->numBits & 31)
    {
        if (pstBitset->words[uwIdx] & ~(0xFFFFFFFF << (pstBitset->numBits & 31)))
        {
            return FALSE;
        }
    }
    return TRUE;
}

OS_SLAB_ALLOCATOR* osSlabAllocatorNew(VOID *pPool, UINT32 uwItemSz, UINT32 uwItemAlign, UINT32 uwNumItems)
{
    OS_SLAB_ALLOCATOR *pstAllocator;
    UINT32 uwBitsetSz, uwDataSz;

    /* calcualte size */
    uwBitsetSz = ATOMIC_BITSET_SZ(uwNumItems);

    uwBitsetSz = (uwBitsetSz + uwItemAlign - 1) & ~(uwItemAlign - 1);
    uwItemSz = (uwItemSz + uwItemAlign - 1) & ~(uwItemAlign - 1);
    uwDataSz = uwItemSz * uwNumItems;

    pstAllocator = (OS_SLAB_ALLOCATOR*)osHeapAlloc(pPool, sizeof(OS_SLAB_ALLOCATOR) + uwBitsetSz + uwDataSz);

    if (pstAllocator)
    {
        pstAllocator->uwItemSz = uwItemSz;

        pstAllocator->bitset = (struct AtomicBitset *)((UINT8*)pstAllocator + sizeof(OS_SLAB_ALLOCATOR));
        pstAllocator->ucDataChunks = ((UINT8*)pstAllocator->bitset) + uwBitsetSz;
        osAtomicBitsetInit(pstAllocator->bitset, uwNumItems);
    }

    return pstAllocator;
}


VOID osSlabAllocatorDestroy(VOID *pPool, OS_SLAB_ALLOCATOR *pstAllocator)
{
    (VOID)osHeapFree(pPool, pstAllocator);
}

VOID* osSlabAllocatorAlloc(OS_SLAB_ALLOCATOR *pstAllocator)
{
    INT32 swItemIdx = osAtomicBitsetFindClearAndSet(pstAllocator->bitset);
    if (swItemIdx < 0)
        return NULL;

    return pstAllocator->ucDataChunks + pstAllocator->uwItemSz * swItemIdx;
}

BOOL osSlabAllocatorFree(OS_SLAB_ALLOCATOR *pstAllocator, VOID* ptrP)
{
    UINT8 *ptr = (UINT8*)ptrP;
    UINT32 uwItemOffset = ptr - pstAllocator->ucDataChunks;
    UINT32 uwItemIdx = uwItemOffset / pstAllocator->uwItemSz;

    //check for invalid inputs
    if ((uwItemOffset % pstAllocator->uwItemSz) || (uwItemIdx >= osAtomicBitsetGetNumBits(pstAllocator->bitset)) || !osAtomicBitsetGetBit(pstAllocator->bitset, uwItemIdx))
        return FALSE;

    osAtomicBitsetClearBit(pstAllocator->bitset, uwItemIdx);
    return TRUE;
}

VOID* osSlabAllocatorGetNth(OS_SLAB_ALLOCATOR *pstAllocator, UINT32 uwIdx)
{
    if (!osAtomicBitsetGetBit(pstAllocator->bitset, uwIdx))
        return NULL;

    return pstAllocator->ucDataChunks + pstAllocator->uwItemSz * uwIdx;
}

VOID* osSlabAllocatorGetIdxP(OS_SLAB_ALLOCATOR *pstAllocator, UINT32 uwIdx)
{
    return pstAllocator->ucDataChunks + pstAllocator->uwItemSz * uwIdx;
}

UINT32 osSlabAllocatorGetIndex(OS_SLAB_ALLOCATOR *pstAllocator, VOID* ptrP)
{
    UINT8 *ptr = (UINT8*)ptrP;
    UINT32 uwItemOffset = ptr - pstAllocator->ucDataChunks;
    UINT32 uwItemIdx = uwItemOffset / pstAllocator->uwItemSz;

    if ((uwItemOffset % pstAllocator->uwItemSz) || (uwItemIdx >= osAtomicBitsetGetNumBits(pstAllocator->bitset)) || !osAtomicBitsetGetBit(pstAllocator->bitset, uwItemIdx))
        return (UINT32)(-1);

    return uwItemIdx;
}

UINT32 osSlabAllocatorGetNumItems(OS_SLAB_ALLOCATOR *pstAllocator)
{
    return osAtomicBitsetGetNumBits(pstAllocator->bitset);
}

BOOL osSlabAllocatorEmpty(OS_SLAB_ALLOCATOR *pstAllocator)
{
    return osAtomicBitsetEmpty(pstAllocator->bitset);
}

UINT32 osSlabAllocatorGetUsedItemCnt(OS_SLAB_ALLOCATOR *pstAllocator)
{
    UINT32 uwUsed, uwIdx;
    struct AtomicBitset *p_bitset = pstAllocator->bitset;
    for (uwUsed = 0, uwIdx = 0; uwIdx < p_bitset->numBits; uwIdx++)
    {
        if (osAtomicBitsetGetBit(p_bitset, uwIdx))
        {
            uwUsed ++;
        }
    }
    return uwUsed;
}

VOID osSlabAllocatorGetSlabInfo(OS_SLAB_ALLOCATOR *pstAllocator, UINT32 *puwItemSz, UINT32 *puwItemCnt, UINT32 *puwCurUsage)
{
    *puwItemSz = pstAllocator->uwItemSz;
    *puwItemCnt = osAtomicBitsetGetNumBits(pstAllocator->bitset);
    *puwCurUsage = osSlabAllocatorGetUsedItemCnt(pstAllocator);
}

BOOL osSlabAllocatorCheck(OS_SLAB_ALLOCATOR *pstAllocator, VOID* ptrP)
{
    UINT8 *ptr = (UINT8*)ptrP;
    UINT32 uwItemOffset = ptr - pstAllocator->ucDataChunks;
    UINT32 uwItemIdx = uwItemOffset / pstAllocator->uwItemSz;

    //check for invalid inputs

    if ((uwItemOffset % pstAllocator->uwItemSz) || (uwItemIdx >= osAtomicBitsetGetNumBits(pstAllocator->bitset))
        || !osAtomicBitsetGetBit(pstAllocator->bitset, uwItemIdx))
        return FALSE;

    return TRUE;
}

