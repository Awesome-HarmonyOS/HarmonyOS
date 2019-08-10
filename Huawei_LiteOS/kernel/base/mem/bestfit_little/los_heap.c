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

/**@defgroup los_heap Heap
 * @ingroup kernel
 */
#include <string.h>
#include <los_hwi.h>
#include <los_config.h>
#include <los_heap.ph>
#include <los_typedef.h>

#if (LOSCFG_MEM_TASK_USED_STATISTICS == YES)
#include "los_memstat.inc"
#endif

LITE_OS_SEC_DATA_INIT static UINT32 g_uwAllocCount = 0;
LITE_OS_SEC_DATA_INIT static UINT32 g_uwFreeCount = 0;

#if (LOSCFG_HEAP_MEMORY_PEAK_STATISTICS == YES)
LITE_OS_SEC_DATA_INIT static UINT32 g_uwCurHeapUsed = 0;
LITE_OS_SEC_DATA_INIT static UINT32 g_uwMaxHeapUsed = 0;
#endif

#define HEAP_CAST(t, exp) ((t)(exp))
#define HEAP_ALIGN 4
#define ALIGNE(sz) (sz + HEAP_ALIGN - 1) & ~(HEAP_ALIGN - 1)

/*****************************************************************************
 Function : osHeapPrvGetNext
 Description : look up the next memory node according to one memory node in the memory block list.
 Input       : struct LOS_HEAP_MANAGER *pHeapMan    --- Pointer to the manager,to distinguish heap
               struct LOS_HEAP_NODE* node  --- Size of memory in bytes to allocate
 Output      : None
 Return      : Pointer to next memory node
*****************************************************************************/
LITE_OS_SEC_TEXT struct LOS_HEAP_NODE* osHeapPrvGetNext(struct LOS_HEAP_MANAGER *pstHeapMan, struct LOS_HEAP_NODE* pstNode)
{
    return (pstHeapMan->pstTail == pstNode) ? NULL : (struct LOS_HEAP_NODE*)(pstNode->ucData + pstNode->uwSize);
}

/*****************************************************************************
 Function : osHeapInit
 Description : To initialize the heap memory and get the begin address and size of heap memory,then initialize LOS_HEAP_MANAGER .
 Input       : struct LOS_HEAP_MANAGER *pHeapMan    --- Pointer to the manager,to distinguish heap
               VOID *pPool  --- begin address of the heap memory pool
               UITN32 uwSz  --- size of the heap memory pool
 Output      : None
 Return      : 1:success 0:error
*****************************************************************************/
LITE_OS_SEC_TEXT_INIT BOOL osHeapInit(VOID *pPool, UINT32 uwSz)
{
    struct LOS_HEAP_NODE* pstNode;
    struct LOS_HEAP_MANAGER *pstHeapMan = HEAP_CAST(struct LOS_HEAP_MANAGER *, pPool);

    if (!pstHeapMan || (uwSz <= (sizeof(struct LOS_HEAP_NODE) +  sizeof(struct LOS_HEAP_MANAGER))))
        return FALSE;

    memset(pPool, 0, uwSz);

    pstHeapMan->uwSize = uwSz;

    pstNode = pstHeapMan->pstHead = (struct LOS_HEAP_NODE*)((UINT8*)pPool + sizeof(struct LOS_HEAP_MANAGER));

    pstHeapMan->pstTail = pstNode;

    pstNode->uwUsed = 0;
    pstNode->pstPrev = NULL;
    pstNode->uwSize = uwSz - sizeof(struct LOS_HEAP_NODE) - sizeof(struct LOS_HEAP_MANAGER);

    return TRUE;
}

/*****************************************************************************
 Function : osHeapAlloc
 Description : To alloc memory block from the heap memory poll
 Input       : VOID *pPool   --- Pointer to the manager,to distinguish heap
               UINT32 uwSz   --- size of the heap memory pool
 Output      : None
 Return      : NULL:error    other value:the address of the memory we alloced
*****************************************************************************/
LITE_OS_SEC_TEXT VOID* osHeapAlloc(VOID *pPool, UINT32 uwSz)
{
    struct LOS_HEAP_NODE *pstNode, *pstT, *pstBest = NULL;
    VOID* pRet = NULL;
    UINT32 uvIntSave;

    struct LOS_HEAP_MANAGER *pstHeapMan = HEAP_CAST(struct LOS_HEAP_MANAGER *, pPool);
    if (!pstHeapMan)
    {
        return NULL;
    }

    uvIntSave = LOS_IntLock();

    uwSz = ALIGNE(uwSz);
    pstNode = pstHeapMan->pstTail;

    while (pstNode)
    {
        if (!pstNode->uwUsed && pstNode->uwSize >= uwSz && (!pstBest || pstBest->uwSize > pstNode->uwSize))
        {
            pstBest = pstNode;
            if (pstBest->uwSize == uwSz)
            {
                goto SIZE_MATCH;
            }
        }
        pstNode = pstNode->pstPrev;
    }

    if (!pstBest) /*alloc failed*/
    {
        PRINT_ERR("there's not enough whole to alloc %x Bytes!\n",uwSz);
        goto out;
    }

    if (pstBest->uwSize - uwSz > sizeof(struct LOS_HEAP_NODE))
    {
        /* hole divide into 2 */
        pstNode = (struct LOS_HEAP_NODE*)(pstBest->ucData + uwSz);

        pstNode->uwUsed = 0;
        pstNode->uwSize = pstBest->uwSize - uwSz- sizeof(struct LOS_HEAP_NODE);
        pstNode->pstPrev = pstBest;

        pstT = osHeapPrvGetNext(pstHeapMan, pstBest);

        if (pstT == NULL)
        {
            pstHeapMan->pstTail = pstNode;      /* pstBest is tail */
        }
        else
        {
            pstT->pstPrev = pstNode;
        }

        pstBest->uwSize = uwSz;
    }

SIZE_MATCH:
    pstBest->uwAlign = 0;
    pstBest->uwUsed = 1;
    pRet = pstBest->ucData;
#if (LOSCFG_MEM_TASK_USED_STATISTICS == YES)
    OS_MEM_ADD_USED(pstBest->uwSize);
#endif

#if (LOSCFG_HEAP_MEMORY_PEAK_STATISTICS == YES)
    g_uwCurHeapUsed += (uwSz + sizeof(struct LOS_HEAP_NODE));
    if(g_uwCurHeapUsed > g_uwMaxHeapUsed)
    {
        g_uwMaxHeapUsed = g_uwCurHeapUsed;
    }
#endif

    g_uwAllocCount++;

out:
    if (pstHeapMan->pstTail->uwSize < 1024)
        osAlarmHeapInfo(pstHeapMan);

    LOS_IntRestore(uvIntSave);

    return pRet;
}

/*****************************************************************************
 Function : osHeapAllocAlign
 Description : To alloc memory block from the heap memory poll with
 Input       : VOID *pPool   --- Pointer to the manager,to distinguish heap
               UINT32 uwSz   --- size of the heap memory pool
               UINT32 uwBoundary --- boundary the heap needs align
 Output      : None
 Return      : NULL:error    other value:the address of the memory we alloced
*****************************************************************************/
LITE_OS_SEC_TEXT VOID* osHeapAllocAlign(VOID *pPool, UINT32 uwSz, UINT32 uwBoundary)
{
    VOID *pRet = NULL;
    UINT32 uwUseSize;
    UINT32 uwGapSize;
    VOID *pAlignedPtr;

    if ((NULL == pPool) || (0 == uwSz) || (uwBoundary < sizeof(VOID *)) || !IS_ALIGNED(uwBoundary))
    {
        return NULL;
    }

    /* worst case is that the node happen to be 4 bytes ahead of the boundary */
    uwUseSize = uwSz + uwBoundary - sizeof(void*);
    pRet = osHeapAlloc(pPool, uwUseSize);

    if (pRet)
    {
        pAlignedPtr = (VOID *)OS_MEM_ALIGN(pRet, uwBoundary);
        if (pRet == pAlignedPtr)
        {
            goto out;
        }

        uwGapSize = (UINT32)pAlignedPtr - (UINT32)pRet;
        OS_MEM_SET_ALIGN_FLAG(uwGapSize);
        *((UINT32 *)((UINT32)pAlignedPtr - 4)) = uwGapSize;

        pRet = pAlignedPtr;
    }
out:
    return pRet;
}

/*****************************************************************************
 Function : osHeapFree
 Description : To free the  memory block from  heap memory poll
 Input       : VOID* pPool      --- Pointer to the manager,to distinguish heap
               VOID* pPtr:      --- the pointer of heap memory we want to free
 Output      : None
 Return      : 1:success 0:error
*****************************************************************************/
LITE_OS_SEC_TEXT BOOL osHeapFree(VOID *pPool, VOID* pPtr)
{
    struct LOS_HEAP_NODE *pstNode, *pstT;
    UINT32 uvIntSave, uwGapSize;
    BOOL bRet = TRUE;

    struct LOS_HEAP_MANAGER *pstHeapMan = HEAP_CAST(struct LOS_HEAP_MANAGER *, pPool);

    if (!pstHeapMan || !pPtr)
    {
        return LOS_NOK;
    }

    /* find the real ptr through gap size */
    uwGapSize = *((UINT32 *)((UINT32)pPtr - 4));
    if (OS_MEM_GET_ALIGN_FLAG(uwGapSize))
    {
        uwGapSize = OS_MEM_GET_ALIGN_GAPSIZE(uwGapSize);
        pPtr = (VOID *)((UINT32)pPtr - uwGapSize);
    }

    if ((UINT32)pPtr < (UINT32)pstHeapMan->pstHead
        || (UINT32)pPtr > ((UINT32)pstHeapMan->pstTail + sizeof(struct LOS_HEAP_NODE)))
    {
        PRINT_ERR("0x%x out of range!\n", (UINT32)pPtr);
        return FALSE;
    }

    uvIntSave = LOS_IntLock();

    pstNode = ((struct LOS_HEAP_NODE*)pPtr) - 1;

    /* check if the address is a node of the heap memory list*/
    if ((pstNode->uwUsed == 0) || (!((UINT32)pstNode == (UINT32)pstHeapMan->pstHead)
        && ((UINT32)pstNode->pstPrev < (UINT32)pstHeapMan->pstHead
            || (UINT32)pstNode->pstPrev > ((UINT32)pstHeapMan->pstTail + sizeof(struct LOS_HEAP_NODE))
            || ((UINT32)osHeapPrvGetNext(pstHeapMan, pstNode->pstPrev) != (UINT32)pstNode)
        )))
    {
        bRet = FALSE;
        goto out;
    }

    /* set to unused status */
    pstNode->uwUsed = 0;
#if (LOSCFG_MEM_TASK_USED_STATISTICS == YES)
    OS_MEM_REDUCE_USED(pstNode->uwSize);
#endif

#if (LOSCFG_HEAP_MEMORY_PEAK_STATISTICS == YES)
    if (g_uwCurHeapUsed >= (pstNode->uwSize + sizeof(struct LOS_HEAP_NODE)))
    {
        g_uwCurHeapUsed -= (pstNode->uwSize + sizeof(struct LOS_HEAP_NODE));
    }
#endif

    /* unused region before and after combination */
    while (pstNode->pstPrev && !pstNode->pstPrev->uwUsed)
        pstNode = pstNode->pstPrev;

    while (((pstT = osHeapPrvGetNext(pstHeapMan, pstNode)) != NULL) && !pstT->uwUsed)
    {
        pstNode->uwSize += sizeof(struct LOS_HEAP_NODE) + pstT->uwSize;
        if (pstHeapMan->pstTail == pstT)
            pstHeapMan->pstTail = pstNode;
    }

    if ((pstT = osHeapPrvGetNext(pstHeapMan, pstNode)) != NULL)
        pstT->pstPrev = pstNode;

    g_uwFreeCount++;

out:
    LOS_IntRestore(uvIntSave);

    return bRet;
}

LITE_OS_SEC_TEXT_MINOR VOID osAlarmHeapInfo(VOID *pPool)
{
    struct LOS_HEAP_MANAGER *pstHeapMan = HEAP_CAST(struct LOS_HEAP_MANAGER *, pPool);
    LOS_HEAP_STATUS stStatus = {0};
    if (LOS_NOK == osHeapStatisticsGet(pPool, &stStatus))
        return;

    PRINT_INFO("pool addr    pool size    total size     used size    free size   alloc Count    free Count\n0x%-8x   0x%-8x   0x%-8x    0x%-8x   0x%-16x   0x%-13x    0x%-13x\n",
                        pPool, pstHeapMan->uwSize, stStatus.totalSize, stStatus.usedSize, stStatus.freeSize, stStatus.allocCount, stStatus.freeCount);
    (void)pstHeapMan;
}

LITE_OS_SEC_TEXT_MINOR UINT32 osHeapStatisticsGet(VOID *pPool, LOS_HEAP_STATUS *pstStatus)
{
    UINT32 uwHeapUsed = 0;
    struct LOS_HEAP_NODE *pstNode = NULL;
    struct LOS_HEAP_MANAGER *pstRamHeap = HEAP_CAST(struct LOS_HEAP_MANAGER *, pPool);

    if (!pstRamHeap)
    {
        return LOS_NOK;
    }

    if (NULL == pstStatus)
    {
        return LOS_NOK;
    }

    pstNode = pstRamHeap->pstTail;
    while (pstNode)
    {
        if (pstNode->uwUsed)
        {
            uwHeapUsed += (pstNode->uwSize + sizeof(struct LOS_HEAP_NODE));
        }
        pstNode = pstNode->pstPrev;
    }

    if (pstRamHeap->uwSize < uwHeapUsed)
    {
        return LOS_NOK;
    }

    pstStatus->usedSize    = uwHeapUsed;
    pstStatus->totalSize   = pstRamHeap->uwSize;
    pstStatus->freeSize    = pstStatus->totalSize - pstStatus->usedSize;
    pstStatus->allocCount  = g_uwAllocCount;
    pstStatus->freeCount   = g_uwFreeCount;

    return LOS_OK;
}

#if (LOSCFG_HEAP_MEMORY_PEAK_STATISTICS == YES)
LITE_OS_SEC_TEXT_MINOR UINT32 osHeapGetHeapMemoryPeak(VOID)
{
    return g_uwMaxHeapUsed;
}
#endif

LITE_OS_SEC_TEXT_MINOR UINT32 osHeapGetMaxFreeBlkSize(VOID *pPool)
{
    UINT32 uwSize = 0;
    UINT32 uwTemp = 0;
    struct LOS_HEAP_NODE *pstNode = NULL;

    struct LOS_HEAP_MANAGER *pstRamHeap = HEAP_CAST(struct LOS_HEAP_MANAGER *, pPool);

    if (!pstRamHeap)
    {
        return LOS_NOK;
    }

    pstNode = pstRamHeap->pstTail;

    while (pstNode)
    {
        if (!(pstNode->uwUsed))
        {
            uwTemp = pstNode->uwSize;
            if (uwTemp > uwSize)
            {
                uwSize = uwTemp;
            }
        }
        pstNode = pstNode->pstPrev;
    }
    return uwSize;
}


