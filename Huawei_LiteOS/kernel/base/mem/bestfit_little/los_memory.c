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
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
#include "los_slab.ph"
#endif
#include "los_heap.ph"
#include "los_hwi.h"
#if (LOSCFG_PLATFORM_EXC == YES)
#include "los_exc.ph"
#endif
#if (LOSCFG_PLATFORM_EXC == YES)
#include "los_memcheck.ph"
#endif

#if (LOSCFG_MEM_MUL_POOL == YES)
VOID *g_pPoolHead = NULL;
#endif

#define OS_SLAB_CAST(_t, _exp) ((_t)(_exp))
#define OS_MEM_POOL_BASE_ALIGN 4
#define IS_POOL_ALIGNED(value, alignSize)  (0 == (((UINT32)(value)) & ((UINT32)(alignSize - 1))))

LITE_OS_SEC_TEXT_MINOR VOID *osSlabCtrlHdrGet(VOID *pPool)
{
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    return (&(OS_SLAB_CAST(struct LOS_HEAP_MANAGER *, pPool)->stSlabCtrlHdr));
#else
    return NULL;
#endif
}

/*****************************************************************************
 Function : LOS_MemInit
 Description : Initialize Dynamic Memory pool
 Input       : pPool    --- Pointer to memory pool
               uwSize  --- Size of memory in bytes to allocate
 Output      : None
 Return      : LOS_OK - Ok, LOS_NOK - Error
*****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_MemInit(VOID *pPool, UINT32 uwSize)
{
    BOOL bRet = TRUE;
    UINTPTR uvIntSave;
#if (LOSCFG_MEM_MUL_POOL == YES)
    VOID *pNext = g_pPoolHead;
    VOID * pCur = g_pPoolHead;
    UINT32 uwPoolEnd;
#endif

    if (!pPool || uwSize <= sizeof(struct LOS_HEAP_MANAGER))
        return LOS_NOK;

    if (!IS_POOL_ALIGNED(pPool, OS_MEM_POOL_BASE_ALIGN))
        return LOS_NOK;

    uvIntSave = LOS_IntLock();

#if (LOSCFG_MEM_MUL_POOL == YES)
    while (pNext != NULL)
    {
        uwPoolEnd = (UINT32)pNext + ((struct LOS_HEAP_MANAGER *)pNext)->uwSize;
        if ((pPool <= pNext && ((UINT32)pPool + uwSize) > (UINT32)pNext) ||
            ((UINT32)pPool < uwPoolEnd && ((UINT32)pPool + uwSize) >= uwPoolEnd))
        {
            PRINT_ERR("pool [%p, 0x%x) conflict with pool [%p, 0x%x)\n",
                          pPool, (UINT32)pPool + uwSize,
                          pNext, (UINT32)pNext + ((struct LOS_HEAP_MANAGER *)pNext)->uwSize);

            LOS_IntRestore(uvIntSave);
            return LOS_NOK;
        }
        pCur = pNext;
        pNext = ((struct LOS_HEAP_MANAGER *)pNext)->pNextPool;
    }
#endif

    bRet = osHeapInit(pPool, uwSize);
    if(!bRet)
    {
        LOS_IntRestore(uvIntSave);
        return LOS_NOK;
    }
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    if (uwSize >= SLAB_BASIC_NEED_SIZE)//if size of pool is small than size of slab need, don`t init slab
    {
        bRet = osSlabMemInit(pPool);
        if(!bRet)
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
        ((struct LOS_HEAP_MANAGER *)pCur)->pNextPool = pPool;
    }

    ((struct LOS_HEAP_MANAGER *)pPool)->pNextPool = NULL;
#endif

#if ((LOSCFG_PLATFORM_EXC == YES) && (LOSCFG_SAVE_EXC_INFO == YES))
    osMemInfoUpdate(pPool, uwSize, MEM_MANG_MEMORY);
#endif

    LOS_IntRestore(uvIntSave);
    return LOS_OK;
}

LITE_OS_SEC_TEXT_INIT UINT32 osMemSystemInit(VOID)
{
    UINT32 uwRet = LOS_OK;

    uwRet = LOS_MemInit((VOID *)OS_SYS_MEM_ADDR, OS_SYS_MEM_SIZE);

#if ((LOSCFG_PLATFORM_EXC == YES) && (LOSCFG_SAVE_EXC_INFO == YES))
    osExcRegister(OS_EXC_TYPE_MEM, (EXC_INFO_SAVE_CALLBACK)LOS_MemExcInfoGet, g_aucMemMang);
#endif
    return uwRet;
}

#if (LOSCFG_MEM_MUL_POOL == YES)
LITE_OS_SEC_TEXT_INIT UINT32 LOS_MemDeInit(VOID *pPool)
{
    UINTPTR uvIntSave, uvRet = LOS_NOK;
    VOID *pNext, *pCur;

    if (NULL == pPool)
    {
        return uvRet;
    }
    uvIntSave = LOS_IntLock();
    do
    {
        if (pPool == g_pPoolHead)
        {
            g_pPoolHead = ((struct LOS_HEAP_MANAGER *)g_pPoolHead)->pNextPool;
            uvRet = LOS_OK;
            break;
        }

        pCur = g_pPoolHead;
        pNext = g_pPoolHead;

        while (pNext != NULL)
        {
            if (pPool == pNext)
            {
                ((struct LOS_HEAP_MANAGER *)pCur)->pNextPool = ((struct LOS_HEAP_MANAGER *)pNext)->pNextPool;
                uvRet = LOS_OK;
                break;
            }
            pCur = pNext;
            pNext = ((struct LOS_HEAP_MANAGER *)pNext)->pNextPool;
        }
    }while(0);

#if ((LOSCFG_PLATFORM_EXC == YES) && (LOSCFG_SAVE_EXC_INFO == YES))
    if (uvRet == LOS_OK)
        osMemInfoUpdate(pPool, 0, MEM_MANG_EMPTY);
#endif
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    osSlabMemDeinit(pPool);
#endif
    LOS_IntRestore(uvIntSave);
    return uvRet;
}

LITE_OS_SEC_TEXT_INIT UINT32 LOS_MemPoolList(VOID)
{
    VOID *pNext = g_pPoolHead;
    UINT32 uwIndex = 0;

    while (pNext != NULL)
    {
        uwIndex++;
        osAlarmHeapInfo(pNext);
        pNext = ((struct LOS_HEAP_MANAGER *)pNext)->pNextPool;
    }
    return uwIndex;
}
#endif

/*****************************************************************************
 Function : LOS_MemAlloc
 Description : Allocate Memory from Memory pool
 Input       : pPool    --- Pointer to memory pool
               uwSize   --- Size of memory in bytes to allocate
 Output      : None
 Return      : Pointer to allocated memory
*****************************************************************************/
LITE_OS_SEC_TEXT VOID *LOS_MemAlloc (VOID *pPool, UINT32 uwSize)
{
    VOID *pRet = NULL;

    if ((NULL == pPool) || (0 == uwSize))
    {
        return pRet;
    }

#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    pRet = osSlabMemAlloc(pPool, uwSize);
    if(pRet == NULL)
#endif
        pRet = osHeapAlloc(pPool, uwSize);

    return pRet;
}
/*****************************************************************************
 Function : LOS_MemAllocAlign
 Description : align size then allocate node from Memory pool
 Input       : pPool        --- Pointer to memory pool
               uwSize       --- Size of memory in bytes to allocate
               uwBoundary   --- align form
 Output      : None
 Return      : Pointer to allocated memory node
*****************************************************************************/
LITE_OS_SEC_TEXT VOID *LOS_MemAllocAlign(VOID *pPool, UINT32 uwSize, UINT32 uwBoundary)
{
    return osHeapAllocAlign(pPool, uwSize, uwBoundary);
}

/*****************************************************************************
 Function : LOS_MemRealloc
 Description : realloc memory from Memory pool
 Input       : pPool    --- Pointer to memory pool
               pPtr     --- Pointer to memory
               uwSize   --- new size
 Output      : None
 Return      : Pointer to allocated memory node
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR VOID *LOS_MemRealloc(VOID *pPool, VOID *pPtr, UINT32 uwSize)
{
    VOID *p = NULL;
    UINTPTR uvIntSave;
    struct LOS_HEAP_NODE *pstNode;
    UINT32 uwCpySize = 0;
#if (LOSCFG_KERNEL_MEM_SLAB == YES)	
    UINT32 uwOldSize = (UINT32)-1;
#endif
    UINT32 uwGapSize = 0;

    if ((int)uwSize < 0)
    {
        return NULL;
    }
    uvIntSave = LOS_IntLock();

    /* Zero-size requests are treated as free. */
    if ((NULL != pPtr) && (0 == uwSize))
    {
        (VOID)LOS_MemFree(pPool, pPtr);
    }
    /* Requests with NULL pointers are treated as malloc. */
    else if (NULL == pPtr)
    {
        p = LOS_MemAlloc(pPool, uwSize);
    }
    else
    {
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
        uwOldSize = osSlabMemCheck(pPool, pPtr);
        if (uwOldSize != (UINT32)-1)
        {
            uwCpySize = uwSize > uwOldSize ? uwOldSize : uwSize;
        }
        else
#endif
        {
            /* find the real ptr through gap size */
            uwGapSize = *((UINT32 *)((UINT32)pPtr - 4));
            if (OS_MEM_GET_ALIGN_FLAG(uwGapSize))
            {
                return NULL;
            }

            pstNode = ((struct LOS_HEAP_NODE *)pPtr) - 1;
            uwCpySize = uwSize > pstNode->uwSize ? pstNode->uwSize : uwSize;
        }
        p = LOS_MemAlloc(pPool, uwSize);

        if (p != NULL)
        {
            (VOID)memcpy(p, pPtr, uwCpySize);
            (VOID)LOS_MemFree(pPool, pPtr);
        }
    }

    LOS_IntRestore(uvIntSave);
    return p;
}

/*****************************************************************************
 Function : LOS_MemFree
 Description : Free Memory and return it to Memory pool
 Input       : pPool    --- Pointer to memory pool
               pMem     --- Pointer to memory to free
 Output      : None
 Return      : LOS_OK - OK, LOS_NOK - Error
*****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_MemFree (VOID *pPool, VOID *pMem)
{
    BOOL bRet = FALSE;

    if ((NULL == pPool) || (NULL == pMem))
    {
        return LOS_NOK;
    }

#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    bRet = osSlabMemFree(pPool, pMem);
    if(bRet != TRUE)
#endif
        bRet = osHeapFree(pPool, pMem);

    return (bRet == TRUE ? LOS_OK : LOS_NOK);
}

LITE_OS_SEC_TEXT UINT32 LOS_MemStatisticsGet(VOID *pPool, LOS_MEM_STATUS *pstStatus)
{
    LOS_HEAP_STATUS stHeapStatus;
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    LOS_SLAB_STATUS stSlabStatus;
#endif
    UINT32 uwErr;

    uwErr = osHeapStatisticsGet(pPool, &stHeapStatus);
    if (uwErr != LOS_OK)
    {
        return LOS_NOK;
    }

    pstStatus->totalSize  = stHeapStatus.totalSize;
    pstStatus->usedSize   = stHeapStatus.usedSize;
    pstStatus->freeSize   = stHeapStatus.freeSize;
    pstStatus->allocCount = stHeapStatus.allocCount;
    pstStatus->freeCount  = stHeapStatus.freeCount;

#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    uwErr = osSlabStatisticsGet(pPool, &stSlabStatus);
    if (uwErr != LOS_OK)
    {
        return LOS_NOK;
    }

    pstStatus->totalSize  = stHeapStatus.totalSize;
    pstStatus->usedSize   = stHeapStatus.usedSize - stSlabStatus.freeSize;  //all slab region inside of heap used region
    pstStatus->freeSize   = stHeapStatus.freeSize + stSlabStatus.freeSize;
    pstStatus->allocCount = stHeapStatus.allocCount + stSlabStatus.allocCount;
    pstStatus->freeCount  = stHeapStatus.freeCount + stSlabStatus.freeCount;
#endif
    return LOS_OK;
}

LITE_OS_SEC_TEXT_MINOR UINT32 LOS_MemGetMaxFreeBlkSize(VOID *pPool)
{
    UINT32 uwMaxFreeSize = osHeapGetMaxFreeBlkSize(pPool);
    UINT32 uwMaxSlabFreeSize = 0;
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    uwMaxSlabFreeSize = osSlabGetMaxFreeBlkSize(pPool);
#endif

#ifndef MAX
#define MAX(x,y) (x)>(y)?(x):(y)
#endif
    return MAX(uwMaxFreeSize, uwMaxSlabFreeSize);
}
