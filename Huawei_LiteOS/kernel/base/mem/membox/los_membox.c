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

#include "los_typedef.h"
#include "los_hwi.h"
#include "los_membox.ph"

#if (LOSCFG_PLATFORM_EXC == YES)
#include "los_memcheck.ph"
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/**
 * The address of the static memory pool must be aligned to the boundary of 4.
 */
#define OS_BOXMEM_BASE_ALIGN  4
#define IS_BOXMEM_ALIGNED(value, alignSize)  (0 == (((UINT32)(value)) & ((UINT32)(alignSize - 1))))

/**
 * Get the address of the next memory node in the static memory pool.
 */
#define OS_MEMBOX_NODE_NEXT(addr, uwBlkSize)  (LOS_MEMBOX_NODE *)((UINT8 *)(addr) + (uwBlkSize))

/**
 * The magic word of the memory box.
 */
#ifdef LOS_MEMBOX_MAGIC_CHECK
#define OS_MEMBOX_MAGIC              0xa55a5aa5
#define OS_MEMBOX_SET_MAGIC(addr)    *((UINT32 *)(addr)) = OS_MEMBOX_MAGIC
#define OS_MEMBOX_CHECK_MAGIC(addr)  ((*((UINT32 *)(addr)) == OS_MEMBOX_MAGIC) ? LOS_OK: LOS_NOK)
#else
#define OS_MEMBOX_SET_MAGIC(addr)
#define OS_MEMBOX_CHECK_MAGIC(addr)  LOS_OK
#endif

/**
 * Get the address of memory block according to the magic word information.
 */
#define OS_MEMBOX_USER_ADDR(addr)  ((VOID *)((UINT8 *)(addr) + LOS_MEMBOX_MAGIC_SIZE))
#define OS_MEMBOX_NODE_ADDR(addr)  ((LOS_MEMBOX_NODE *)((UINT8 *)(addr) - LOS_MEMBOX_MAGIC_SIZE))


/*****************************************************************************
 Function : osCheckBoxMem
 Description : Check whether the memory block is valid
 Input       : pstBoxInfo  --- Pointer to the memory pool
               pstNode     --- Pointer to the memory block that will be checked
 Output      : None
 Return      : LOS_OK - OK, LOS_NOK - Error
*****************************************************************************/
LITE_OS_SEC_TEXT static INLINE UINT32 osCheckBoxMem(const LOS_MEMBOX_INFO *pstBoxInfo, const VOID *pstNode)
{
    UINT32 uwOffSet;

    if (pstBoxInfo->uwBlkSize == 0)
    {
        return LOS_NOK;
    }

    uwOffSet = (UINT32)pstNode - (UINT32)(pstBoxInfo + 1);
    if ((uwOffSet % pstBoxInfo->uwBlkSize) != 0)
    {
        return LOS_NOK;
    }

    if ((uwOffSet / pstBoxInfo->uwBlkSize) >= pstBoxInfo->uwBlkNum)
    {
        return LOS_NOK;
    }

    return OS_MEMBOX_CHECK_MAGIC(pstNode);
}

/*****************************************************************************
 Function : LOS_MemboxInit
 Description : Initialize Static Memory pool
 Input       : pBoxMem    --- Pointer to the memory pool
               uwBoxSize  --- Size of the memory pool
               uwBlkSize  --- Size of the memory block
 Output      : None
 Return      : LOS_OK - OK, LOS_NOK - Error
*****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_MemboxInit(VOID *pBoxMem, UINT32 uwBoxSize, UINT32 uwBlkSize)
{
    LOS_MEMBOX_INFO *pstBoxInfo = (LOS_MEMBOX_INFO *)pBoxMem;
    LOS_MEMBOX_NODE *pstNode = NULL;
    UINT32 i;
    UINTPTR uvIntSave;

    if (pBoxMem == NULL || uwBlkSize == 0 || uwBoxSize < sizeof(LOS_MEMBOX_INFO))
    {
        return LOS_NOK;
    }

    if (!IS_BOXMEM_ALIGNED(pBoxMem, OS_BOXMEM_BASE_ALIGN))
    {
        return LOS_NOK;
    }

    uvIntSave = LOS_IntLock();

    /*
     * The node size is aligned to the next 4 boundary.
     * Memory that is not enough for one node size in the memory pool will be ignored.
     */
    pstBoxInfo->uwBlkSize = LOS_MEMBOX_ALIGNED(uwBlkSize + LOS_MEMBOX_MAGIC_SIZE);
    pstBoxInfo->uwBlkNum = (uwBoxSize - sizeof(LOS_MEMBOX_INFO)) / pstBoxInfo->uwBlkSize;
    pstBoxInfo->uwBlkCnt = 0;

    if (pstBoxInfo->uwBlkNum == 0)
    {
        LOS_IntRestore(uvIntSave);
        return LOS_NOK;
    }

    pstNode = (LOS_MEMBOX_NODE *)(pstBoxInfo + 1);
    pstBoxInfo->stFreeList.pstNext = pstNode;

    for (i = 0; i < pstBoxInfo->uwBlkNum - 1; ++i)
    {
        pstNode->pstNext = OS_MEMBOX_NODE_NEXT(pstNode, pstBoxInfo->uwBlkSize);
        pstNode = pstNode->pstNext;
    }
    pstNode->pstNext = (LOS_MEMBOX_NODE *)NULL;  /* The last node */

#if ((LOSCFG_PLATFORM_EXC == YES) && (LOSCFG_SAVE_EXC_INFO == YES))
    osMemInfoUpdate(pBoxMem, uwBoxSize, MEM_MANG_MEMBOX);
#endif

    (VOID)LOS_IntRestore(uvIntSave);

    return LOS_OK;
}

/*****************************************************************************
 Function : LOS_MemboxAlloc
 Description : Allocate Memory block from Static Memory pool
 Input       : pBoxMem  --- Pointer to memory pool
 Output      : None
 Return      : Pointer to allocated memory block
*****************************************************************************/
LITE_OS_SEC_TEXT VOID *LOS_MemboxAlloc(VOID *pBoxMem)
{
    LOS_MEMBOX_INFO *pstBoxInfo = (LOS_MEMBOX_INFO *)pBoxMem;
    LOS_MEMBOX_NODE *pstNode = NULL;
    LOS_MEMBOX_NODE *pRet = NULL;
    UINTPTR uvIntSave;

    if (pBoxMem == NULL)
    {
        return NULL;
    }

    uvIntSave = LOS_IntLock();

    pstNode = &pstBoxInfo->stFreeList;
    if (pstNode->pstNext != NULL)
    {
        pRet = pstNode->pstNext;
        pstNode->pstNext = pRet->pstNext;
        OS_MEMBOX_SET_MAGIC(pRet);
        pstBoxInfo->uwBlkCnt++;
    }

    (VOID)LOS_IntRestore(uvIntSave);

    return pRet == NULL ? NULL : OS_MEMBOX_USER_ADDR(pRet);
}

/*****************************************************************************
 Function : LOS_MemboxFree
 Description : Free Memory block and return it to Static Memory pool
 Input       : pBoxMem  --- Pointer to memory pool
               pBox     --- Pointer to memory block to free
 Output      : None
 Return      : LOS_OK - OK, LOS_NOK - Error
*****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_MemboxFree(VOID *pBoxMem, VOID *pBox)
{
    LOS_MEMBOX_INFO *pstBoxInfo = (LOS_MEMBOX_INFO *)pBoxMem;
    UINT32 uwRet = LOS_NOK;
    UINTPTR uvIntSave;

    if (pBoxMem == NULL || pBox == NULL)
    {
        return LOS_NOK;
    }

    uvIntSave = LOS_IntLock();

    do
    {
        LOS_MEMBOX_NODE *pstNode = OS_MEMBOX_NODE_ADDR(pBox);

        if (osCheckBoxMem(pstBoxInfo, pstNode) != LOS_OK)
        {
            break;
        }

        pstNode->pstNext = pstBoxInfo->stFreeList.pstNext;
        pstBoxInfo->stFreeList.pstNext = pstNode;
        pstBoxInfo->uwBlkCnt--;
        uwRet = LOS_OK;
    } while (0);

    (VOID)LOS_IntRestore(uvIntSave);

    return uwRet;
}

/*****************************************************************************
 Function : LOS_MemboxClr
 Description : Clear the memory block
 Input       : pBoxMem  --- Pointer to memory pool
               pBox     --- Pointer to memory block to clear
 Output      : None
 Return      : None
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR VOID LOS_MemboxClr(VOID *pBoxMem, VOID *pBox)
{
    LOS_MEMBOX_INFO *pstBoxInfo = (LOS_MEMBOX_INFO *)pBoxMem;

    if (pBoxMem == NULL || pBox == NULL)
    {
        return;
    }

    memset(pBox, 0, pstBoxInfo->uwBlkSize - LOS_MEMBOX_MAGIC_SIZE);
}

/*****************************************************************************
 Function : LOS_MemboxStatisticsGet
 Description : Get information about membox
 Input       : pBoxMem     --- Pointer to the calculate membox
 Output      : puwMaxBlk   --- Record the total number of membox
               puwBlkCnt   --- Record the number of the allocated blocks of membox
               puwBlkSize  --- Record the block size of membox
 Return      : LOS_OK - OK, LOS_NOK - Error
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_MemboxStatisticsGet(VOID *pBoxMem, UINT32 *puwMaxBlk, UINT32 *puwBlkCnt, UINT32 *puwBlkSize)
{
    if ((NULL == pBoxMem) || (NULL == puwMaxBlk) || (NULL == puwBlkCnt) || (NULL == puwBlkSize))
    {
        return LOS_NOK;
    }

    *puwMaxBlk = ((LOS_MEMBOX_INFO *)pBoxMem)->uwBlkNum;    /* Total number of blocks */
    *puwBlkCnt = ((LOS_MEMBOX_INFO *)pBoxMem)->uwBlkCnt;    /* The number of allocated blocks */
    *puwBlkSize = ((LOS_MEMBOX_INFO *)pBoxMem)->uwBlkSize;  /* Block size */

    return LOS_OK;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */
