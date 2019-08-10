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

#include "los_memcheck.ph"
#include "los_memory.ph"
#include "los_membox.ph"
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
#include "los_slab.ph"
#endif
#include "los_heap.ph"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */
UINT32 memexc_count = 0;

#if ((LOSCFG_PLATFORM_EXC == YES) && (LOSCFG_SAVE_EXC_INFO == YES))
LITE_OS_SEC_BSS UINT8 g_aucMemMang[MEM_INFO_SIZE];
/*****************************************************************************
 Function	 : LOS_MemExcInfoGet
 Description : Get the information of the exc memory
 Input       : uwMemNum
 Output      : pstMemExcInfo
 Return      : return 0
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_MemExcInfoGet(UINT32 uwMemNum, MEM_INFO_S *pstMemExcInfo)
{
    UINT32 uwItemSz;
    UINT32 uwItemCnt;
    UINT32 uwCurUsage;
    UINT32 uwIdx;
    UINT32 uwMaxBlk = 0;
    UINT32 uwBlkCnt = 0;
    UINT32 uwBlkSize = 0;
    LOS_MEM_STATUS stStatus;
    MEM_INFO *pstMemInfo = NULL;
#if (LOSCFG_KERNEL_MEM_SLAB == YES)
    OS_SLAB_MEM *pstSlabAlloc;
#endif

    if(uwMemNum >= *(UINT32 *)g_aucMemMang || pstMemExcInfo == NULL)
    {
        return LOS_NOK;
    }
    pstMemInfo = (MEM_INFO *)(g_aucMemMang + sizeof(UINT32)) + uwMemNum;
    pstMemExcInfo->uwType = pstMemInfo->uwType;
    pstMemExcInfo->uwStartAddr = pstMemInfo->uwStartAddr;
    pstMemExcInfo->uwSize = pstMemInfo->uwSize;
    pstMemExcInfo->uwFree = 0;
    pstMemExcInfo->uwBlockSize = 0;
    pstMemExcInfo->uwErrorAddr = 0;
    pstMemExcInfo->uwErrorLen = 0;
    pstMemExcInfo->uwErrorOwner = 0;

    if (pstMemInfo->uwType == MEM_MANG_MEMBOX)
    {
        (VOID)LOS_MemboxStatisticsGet((VOID *)(pstMemInfo->uwStartAddr), &uwMaxBlk, &uwBlkCnt, &uwBlkSize);
        pstMemExcInfo->uwBlockSize = uwBlkSize;
        pstMemExcInfo->uwSize = uwMaxBlk;//Block num
        pstMemExcInfo->uwFree = uwMaxBlk - uwBlkCnt;
    }
    else if(pstMemInfo->uwType == MEM_MANG_MEMORY)
    {

#if (LOSCFG_KERNEL_MEM_SLAB == YES)
        struct LOS_SLAB_CONTROL_HEADER *pstSlabMem = osSlabCtrlHdrGet((VOID *)pstMemExcInfo->uwStartAddr);
        for (uwIdx = 0; uwIdx < SLAB_MEM_COUNT; uwIdx++)
        {
            pstSlabAlloc = &(pstSlabMem->stSlabClass[uwIdx]);
            if(pstSlabAlloc->alloc)
            {
                osSlabAllocatorGetSlabInfo(pstSlabAlloc->alloc, &uwItemSz, &uwItemCnt, &uwCurUsage);

                pstMemExcInfo->stSlabInfo[uwIdx].cur_usage = uwCurUsage;
                pstMemExcInfo->stSlabInfo[uwIdx].item_cnt = uwItemCnt;
                pstMemExcInfo->stSlabInfo[uwIdx].item_sz = uwItemSz;
            }
            else
            {
                pstMemExcInfo->stSlabInfo[uwIdx].cur_usage = 0;
                pstMemExcInfo->stSlabInfo[uwIdx].item_cnt = 0;
                pstMemExcInfo->stSlabInfo[uwIdx].item_sz = 0;
            }
        }
#endif
        (VOID)LOS_MemStatisticsGet((VOID *)(pstMemInfo->uwStartAddr), &stStatus);

        pstMemExcInfo->uwSize = stStatus.totalSize;
        pstMemExcInfo->uwFree = stStatus.freeSize;
    }
    else {
        PRINT_ERR("%s:the type of %x  is MEM_MANG_EMPTY !\n", __func__, pstMemInfo->uwStartAddr);
    }

    return LOS_OK;
}

LITE_OS_SEC_TEXT UINT32 osMemInfoUpdate(VOID *pPool, UINT32 uwSize, UINT32 uwType)
{
    UINT32 *puwMemCount = (UINT32 *)g_aucMemMang;
    MEM_INFO *pstMemInfo = (MEM_INFO *)(g_aucMemMang + sizeof(UINT32));
    UINTPTR uvIntSave;
    UINT8 ucLoop;
    UINT32 uwRet = LOS_OK;

    uvIntSave = LOS_IntLock();
    for (ucLoop = 0; ucLoop < *puwMemCount; ucLoop++)
    {
        if (uwType == MEM_MANG_EMPTY)
        {
            if (pstMemInfo->uwStartAddr == (UINT32)pPool)
            {
                pstMemInfo->uwType = MEM_MANG_EMPTY;
                LOS_IntRestore(uvIntSave);
                return  LOS_OK;
            }
        }
        else if (pstMemInfo->uwStartAddr == (UINT32)pPool )
        {
            (*puwMemCount)--;
            uwRet = LOS_NOK;
            break;
        }
        else if (pstMemInfo->uwType == MEM_MANG_EMPTY)
        {
            (*puwMemCount)--;
            break;
        }
        pstMemInfo++;
    }
    if(*puwMemCount < OS_SYS_MEM_NUM && uwType != MEM_MANG_EMPTY)
    {
        pstMemInfo->uwType = uwType;
        pstMemInfo->uwStartAddr = (UINT32)pPool;
        pstMemInfo->uwSize = uwSize;
        (*puwMemCount)++;
    }
    LOS_IntRestore(uvIntSave);
    return uwRet;
}
#endif

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */
