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
#define _LOS_SLAB_MEM_C_

#include <los_printf.h>
#include <los_slab.ph>
#include <los_hwi.h>

#if (LOSCFG_MEM_TASK_USED_STATISTICS == YES)
#include "los_memstat.inc"
#endif

VOID *osSlabBlockHeadFill(OS_SLAB_BLOCK_NODE *pstSlabNode, UINT32 uwBlkSz)
{
    OS_SLAB_BLOCK_MAGIC_SET(pstSlabNode);
    OS_SLAB_BLOCK_SIZE_SET(pstSlabNode, uwBlkSz);
    OS_SLAB_BLOCK_ID_SET(pstSlabNode, 0);//now undefine how to use ID
    return (VOID *)(pstSlabNode + 1);
}

/*****************************************************************************
 Function : osSlabMemInit
 Description : To initialize the slab memory management
 Input       : None
 Output      : None
 Return      : None
*****************************************************************************/
BOOL osSlabMemInit(VOID *pPool)
{
    struct LOS_SLAB_CONTROL_HEADER *pstSlabMemHead = osSlabCtrlHdrGet(pPool);
    UINT32 uwIdx = 0;
    UINT32 uwTmp = 0;
    UINT32 uwBlkSz = 0;
    UINT32 uwBlkCnt = 0;

    for (uwIdx = 0; uwIdx < SLAB_MEM_COUNT; uwIdx++)
    {
        uwBlkSz = (SLAB_MEM_CALSS_STEP_SIZE << uwIdx);
        uwBlkCnt = SLAB_MEM_ALLOCATOR_SIZE / uwBlkSz;
        pstSlabMemHead->stSlabClass[uwIdx].blkSz = uwBlkSz;
        pstSlabMemHead->stSlabClass[uwIdx].blkCnt = uwBlkCnt;
        pstSlabMemHead->stSlabClass[uwIdx].blkUsedCnt = 0;
        if (NULL != pstSlabMemHead->stSlabClass[uwIdx].alloc)
        {
            PRINT_WARN("SlabMemAllocator[%d] inited before\n", uwIdx);
            uwTmp++;
        }
        else
        {
            pstSlabMemHead->stSlabClass[uwIdx].alloc = osSlabAllocatorNew(pPool, uwBlkSz + sizeof(OS_SLAB_BLOCK_NODE), (UINT32)sizeof(VOID *), uwBlkCnt);
        }
    }

    return ((0 == uwTmp) ? TRUE : FALSE);
}

/*****************************************************************************
 Function : osSlabMemAlloc
 Description : To alloc memory block
 Input       :  UITN32 sz --- size of the  memory we want to alloc
 Output      : None
 Return      : pointer :the address of the memory we alloced
*****************************************************************************/
VOID *osSlabMemAlloc(VOID *pPool, UINT32 uwSz)
{
    VOID *pRet = NULL;
    UINTPTR uvIntSave;
    struct LOS_SLAB_CONTROL_HEADER *pstSlabMem = osSlabCtrlHdrGet(pPool);
    OS_SLAB_MEM *pstSlabAlloc = NULL;
    UINT32 uwIdx = 0;

    if (uwSz > (SLAB_MEM_CALSS_STEP_SIZE << (SLAB_MEM_COUNT - 1)))
    {
        return NULL;
    }

    for (uwIdx = 0; uwIdx < SLAB_MEM_COUNT; uwIdx++)
    {
        if (uwSz<= pstSlabMem->stSlabClass[uwIdx].blkSz)
        {
            uvIntSave = LOS_IntLock();

            if (pstSlabMem->stSlabClass[uwIdx].blkUsedCnt >= pstSlabMem->stSlabClass[uwIdx].blkCnt)
            {
                (VOID)LOS_IntRestore(uvIntSave);
                return NULL;
            }

            if (NULL == pstSlabMem->stSlabClass[uwIdx].alloc)
            {
                (VOID)LOS_IntRestore(uvIntSave);
                return NULL;
            }

            pstSlabAlloc = &(pstSlabMem->stSlabClass[uwIdx]);
            pRet = osSlabAllocatorAlloc(pstSlabAlloc->alloc);
            if (NULL != pRet)
            {
                /* alloc success */
                pRet = osSlabBlockHeadFill((OS_SLAB_BLOCK_NODE *)pRet, pstSlabMem->stSlabClass[uwIdx].blkSz);
                pstSlabMem->stSlabClass[uwIdx].blkUsedCnt++;
#if (LOSCFG_MEM_TASK_USED_STATISTICS == YES)
                OS_MEM_ADD_USED(pstSlabMem->stSlabClass[uwIdx].blkSz);
#endif
            }

            (VOID)LOS_IntRestore(uvIntSave);
            return pRet;
        }
    }

    return NULL;
}

/*****************************************************************************
 Function : osSlabMemFree
 Description : To free the  memory block
 Input       : VOID* pPtr: the pointer of heap memory we want to free
 Output      : None
 Return      : TRUE:success FALSE:error
*****************************************************************************/
BOOL osSlabMemFree(VOID *pPool, VOID* pPtr)
{
    UINTPTR uvIntSave;
    struct LOS_SLAB_CONTROL_HEADER *pstSlabMem = osSlabCtrlHdrGet(pPool);
    BOOL bRet = FALSE;
    OS_SLAB_MEM *pstSlabAlloc;
    UINT32 uwIdx = 0;
    OS_SLAB_BLOCK_NODE *pstSlabNode = OS_SLAB_BLOCK_HEAD_GET(pPtr);

    if (!OS_ALLOC_FROM_SLAB_CHECK(pstSlabNode))
    {
        return FALSE;
    }
    for (uwIdx = 0; uwIdx < SLAB_MEM_COUNT; uwIdx++)
    {
        if (OS_SLAB_BLOCK_SIZE_GET(pstSlabNode) <= pstSlabMem->stSlabClass[uwIdx].blkSz)
        {
            uvIntSave = LOS_IntLock();

            pstSlabAlloc = &(pstSlabMem->stSlabClass[uwIdx]);
            if (TRUE == osSlabAllocatorFree(pstSlabAlloc->alloc, pstSlabNode))
            {
                bRet = TRUE;
                pstSlabMem->stSlabClass[uwIdx].blkUsedCnt--;
#if (LOSCFG_MEM_TASK_USED_STATISTICS == YES)
                OS_MEM_REDUCE_USED(pstSlabMem->stSlabClass[uwIdx].blkSz);
#endif
            }

            (VOID)LOS_IntRestore(uvIntSave);
            return bRet;
        }
    }
    return FALSE;
}

/*****************************************************************************
 Function : osSlabMemDeinit
 Description :  deinitialize the slab memory ,set back to the original status
 Input       : None
 Output      : None
 Return      : None
*****************************************************************************/
VOID osSlabMemDeinit(VOID *pPool)
{
    UINT32 uwIdx;
    struct LOS_SLAB_CONTROL_HEADER *pstSlabMem = NULL;
    OS_SLAB_MEM *pstSlabAlloc;
    UINT32 uwBlkSz;
    UINT32 uwBlkCnt;

    if (NULL == pPool)
    {
        return ;
    }
    pstSlabMem = osSlabCtrlHdrGet(pPool);

    for (uwIdx = 0; uwIdx < SLAB_MEM_COUNT; uwIdx++)
    {
        uwBlkSz = (0x10 << uwIdx);
        uwBlkCnt = SLAB_MEM_ALLOCATOR_SIZE / uwBlkSz;
        pstSlabMem->stSlabClass[uwIdx].blkSz = uwBlkSz;
        pstSlabMem->stSlabClass[uwIdx].blkCnt = uwBlkCnt;
        if (NULL != pstSlabMem->stSlabClass[uwIdx].alloc)
        {
            pstSlabAlloc = &(pstSlabMem->stSlabClass[uwIdx]);
            osSlabAllocatorDestroy(pPool, pstSlabAlloc->alloc);
            pstSlabMem->stSlabClass[uwIdx].alloc = NULL;
        }
    }
    return ;
}

UINT32 osSlabMemCheck(VOID *pPool, VOID* pPtr)
{
    UINTPTR uvIntSave;
    struct LOS_SLAB_CONTROL_HEADER *pstSlabMem = osSlabCtrlHdrGet(pPool);
    UINT32 uwRetBlkSz = (UINT32)-1;
    OS_SLAB_MEM *pstSlabAlloc;
    UINT32 uwIdx = 0;
    OS_SLAB_BLOCK_NODE *pstSlabNode = OS_SLAB_BLOCK_HEAD_GET(pPtr);

    if ((!OS_ALLOC_FROM_SLAB_CHECK(pstSlabNode))
        || (OS_SLAB_BLOCK_SIZE_GET(pstSlabNode) > pstSlabMem->stSlabClass[SLAB_MEM_COUNT - 1].blkSz))
    {
        return uwRetBlkSz;
    }

    uvIntSave = LOS_IntLock();
    for (uwIdx = 0; uwIdx < SLAB_MEM_COUNT; uwIdx++)
    {
        pstSlabAlloc = &(pstSlabMem->stSlabClass[uwIdx]);
        if (osSlabAllocatorCheck(pstSlabAlloc->alloc, pstSlabNode) == TRUE)
        {
            uwRetBlkSz = pstSlabMem->stSlabClass[uwIdx].blkSz;
        }
    }
    (VOID)LOS_IntRestore(uvIntSave);

    return uwRetBlkSz;
}

UINT32 osSlabStatisticsGet(VOID *pPool, LOS_SLAB_STATUS *pstStatus)
{
    struct LOS_SLAB_CONTROL_HEADER *pstSlabMem = NULL;
    OS_SLAB_MEM *pstSlabAlloc;
    UINT32 uwItemSz = 0;
    UINT32 uwItemCnt = 0;
    UINT32 uwCurUsage = 0;
    UINT32 uwTotalUsage = 0;
    UINT32 uwTotalMem = 0;
    UINT32 uwTotalallocCount = 0;
    UINT32 uwTotalfreeCount = 0;
    UINT32 uwIdx = 0;

    if ((NULL == pstStatus) || (NULL == pPool))
    {
        return LOS_NOK;
    }
    pstSlabMem = osSlabCtrlHdrGet(pPool);

    for (uwIdx = 0; uwIdx < SLAB_MEM_COUNT; uwIdx++)
    {
        pstSlabAlloc = &(pstSlabMem->stSlabClass[uwIdx]);

        osSlabAllocatorGetSlabInfo(pstSlabAlloc->alloc, &uwItemSz, &uwItemCnt, &uwCurUsage);
        uwTotalUsage += (uwCurUsage * uwItemSz);
        uwTotalMem += (uwItemCnt * uwItemSz);
        uwTotalallocCount += pstSlabMem->stSlabClass[uwIdx].blkUsedCnt;
        uwTotalfreeCount  += pstSlabMem->stSlabClass[uwIdx].blkCnt - pstSlabMem->stSlabClass[uwIdx].blkUsedCnt;
    }

    if (uwTotalMem < uwTotalUsage)
    {
        return LOS_NOK;
    }

    pstStatus->totalSize  = uwTotalMem;
    pstStatus->usedSize   = uwTotalUsage;
    pstStatus->freeSize   = pstStatus->totalSize - pstStatus->usedSize;
    pstStatus->allocCount = uwTotalallocCount;
    pstStatus->freeCount  = uwTotalfreeCount;
    return LOS_OK;
}

UINT32 osSlabGetMaxFreeBlkSize(VOID *pPool)
{
    struct LOS_SLAB_CONTROL_HEADER *pstSlabMem = osSlabCtrlHdrGet(pPool);
    OS_SLAB_MEM *pstSlabAlloc;
    UINT32 uwItemSz = 0;
    UINT32 uwItemCnt = 0;
    UINT32 uwCurUsage = 0;
    int uwIdx = 0;

    for (uwIdx = SLAB_MEM_COUNT - 1; uwIdx >= 0; uwIdx--)
    {
        pstSlabAlloc = &(pstSlabMem->stSlabClass[uwIdx]);
        if (pstSlabAlloc->alloc)
        {
            osSlabAllocatorGetSlabInfo(pstSlabAlloc->alloc, &uwItemSz, &uwItemCnt, &uwCurUsage);
            if (uwCurUsage != uwItemCnt)
            {
                return uwItemSz;
            }
        }
    }

    return 0;
}
