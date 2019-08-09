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

#include "los_memory.inc"
#include "los_task.ph"

#ifdef LOSCFG_LIB_LIBC
#include "string.h"
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

extern UINT8 *m_aucSysMem0;
UINT8 g_ucMemStart[OS_SYS_MEM_SIZE];

LITE_OS_SEC_TEXT_INIT UINT32 osMemSystemInit()
{
    UINT32 uwRet;

    m_aucSysMem0 = g_ucMemStart;
    uwRet = LOS_MemInit(m_aucSysMem0, OS_SYS_MEM_SIZE);
    return uwRet;
}

/*****************************************************************************
 Function : osMemFindSuitableFreeBlock
 Description : find suitable free block use "best fit" algorithm
 Input       : pPool    --- Pointer to memory pool
                 uwAllocSize  --- Size of memory in bytes which note need allocate
 Output      : None
 Return      :NULL--no suitable block found
                 pstTem--pointer a suitable free block
*****************************************************************************/
INLINE LOS_MEM_DYN_NODE *osMemFindSuitableFreeBlock(VOID *pPool, UINT32 uwAllocSize)
{
    LOS_DL_LIST *pstListHead = (LOS_DL_LIST *)NULL;

    for (pstListHead = OS_MEM_HEAD(pPool, uwAllocSize); pstListHead != NULL; pstListHead = LOS_DLnkNextMultiHead(OS_MEM_HEAD_ADDR(pPool), pstListHead))
    {
        LOS_MEM_DYN_NODE *pstTmp = (LOS_MEM_DYN_NODE *)NULL;
        LOS_DL_LIST_FOR_EACH_ENTRY(pstTmp, pstListHead, LOS_MEM_DYN_NODE, stFreeNodeInfo) /*lint !e413*/
        {
            if (pstTmp->uwSizeAndFlag >= uwAllocSize)
            {
                return pstTmp;
            }
        }
    }

    return (LOS_MEM_DYN_NODE *)NULL;
}

/*****************************************************************************
 Function : osMemClearNode
 Description : clear a mem Node , set every member to NULL
 Input       : pstNode    --- Pointer to the mem node which will be cleared up
 Output      : None
 Return      : None
*****************************************************************************/
INLINE VOID osMemClearNode(LOS_MEM_DYN_NODE *pstNode)
{
    pstNode->stFreeNodeInfo.pstPrev = (LOS_DL_LIST *)NULL;
    pstNode->stFreeNodeInfo.pstNext = (LOS_DL_LIST *)NULL;
    pstNode->pstPreNode = (LOS_MEM_DYN_NODE *)NULL;
}

/*****************************************************************************
 Function : osMemMergeNode
 Description : merge this node and pre node ,then clear this node info
 Input       : pstNode    --- Pointer to node which will be merged
 Output      : None
 Return      : None
*****************************************************************************/
INLINE VOID osMemMergeNode(LOS_MEM_DYN_NODE *pstNode)
{
    LOS_MEM_DYN_NODE *pstNextNode = (LOS_MEM_DYN_NODE *)NULL;

    pstNode->pstPreNode->uwSizeAndFlag += pstNode->uwSizeAndFlag;
    pstNextNode = (LOS_MEM_DYN_NODE *)((UINT32)pstNode + pstNode->uwSizeAndFlag);
    pstNextNode->pstPreNode = pstNode->pstPreNode;
    osMemClearNode(pstNode);
}

/*****************************************************************************
 Function : osMemSpitNode
 Description : spit new node from pstAllocNode, and merge remainder mem if necessary
 Input       : pPool --Pointer to memory pool
                  pstAllocNode --the source node which new node be spit from to.
                                        After pick up it's node info, change to point the new node
                  uwAllocSize -- the size of new node
 Output      : pstAllocNode -- save new node addr
 Return      : None
*****************************************************************************/
INLINE VOID osMemSpitNode(VOID *pPool,
                            LOS_MEM_DYN_NODE *pstAllocNode, UINT32 uwAllocSize)
{
    LOS_MEM_DYN_NODE *pstNewFreeNode = (LOS_MEM_DYN_NODE *)NULL;
    LOS_MEM_DYN_NODE *pstNextNode = (LOS_MEM_DYN_NODE *)NULL;
    LOS_DL_LIST *pstListHead = (LOS_DL_LIST *)NULL;

    pstNewFreeNode = (LOS_MEM_DYN_NODE *)((UINT8 *)pstAllocNode + uwAllocSize);
    pstNewFreeNode->pstPreNode = pstAllocNode;
    pstNewFreeNode->uwSizeAndFlag = pstAllocNode->uwSizeAndFlag - uwAllocSize;
    pstAllocNode->uwSizeAndFlag = uwAllocSize;
    pstNextNode = OS_MEM_NEXT_NODE(pstNewFreeNode);
    pstNextNode->pstPreNode = pstNewFreeNode;
    if (!OS_MEM_NODE_GET_USED_FLAG(pstNextNode->uwSizeAndFlag))
    {
        LOS_ListDelete(&(pstNextNode->stFreeNodeInfo));
        osMemMergeNode(pstNextNode);
    }

    pstListHead = OS_MEM_HEAD(pPool, pstNewFreeNode->uwSizeAndFlag);
    if (NULL == pstListHead)
    {
        PRINT_ERR("%s %d\n", __FUNCTION__, __LINE__);
        return;
    }

    LOS_ListAdd(pstListHead,&(pstNewFreeNode->stFreeNodeInfo));
}

/*****************************************************************************
 Function : osMemFreeNode
 Description : free the node from memory & if there are free node beside, merger them.
                    at last update "pstListHead' which saved all free node control head
 Input       : pstNode -- the node which need be freed
                  pPool --Pointer to memory pool
 Output      : None
 Return      : None
*****************************************************************************/
INLINE VOID osMemFreeNode(LOS_MEM_DYN_NODE *pstNode, VOID *pPool)
{
    LOS_MEM_DYN_NODE *pstNextNode = (LOS_MEM_DYN_NODE *)NULL;
    LOS_DL_LIST *pstListHead = (LOS_DL_LIST *)NULL;

    OS_MEM_REDUCE_USED(OS_MEM_NODE_GET_SIZE(pstNode->uwSizeAndFlag));
    pstNode->uwSizeAndFlag = OS_MEM_NODE_GET_SIZE(pstNode->uwSizeAndFlag);
    if ((pstNode->pstPreNode != NULL) &&
        (!OS_MEM_NODE_GET_USED_FLAG(pstNode->pstPreNode->uwSizeAndFlag)))
    {
        LOS_MEM_DYN_NODE *pstPreNode = pstNode->pstPreNode;
        osMemMergeNode(pstNode);
        pstNextNode = OS_MEM_NEXT_NODE(pstPreNode);
        if (!OS_MEM_NODE_GET_USED_FLAG(pstNextNode->uwSizeAndFlag))
        {
            LOS_ListDelete(&(pstNextNode->stFreeNodeInfo));
            osMemMergeNode(pstNextNode);
        }

        LOS_ListDelete(&(pstPreNode->stFreeNodeInfo));
        pstListHead = OS_MEM_HEAD(pPool, pstPreNode->uwSizeAndFlag);
        if (NULL == pstListHead)
        {
            PRINT_ERR("%s %d\n", __FUNCTION__, __LINE__);
            return;
        }

        LOS_ListAdd(pstListHead,&(pstPreNode->stFreeNodeInfo));
    }
    else
    {
        pstNextNode = OS_MEM_NEXT_NODE(pstNode);
        if (!OS_MEM_NODE_GET_USED_FLAG(pstNextNode->uwSizeAndFlag))
        {
            LOS_ListDelete(&(pstNextNode->stFreeNodeInfo));
            osMemMergeNode(pstNextNode);
        }

        pstListHead = OS_MEM_HEAD(pPool, pstNode->uwSizeAndFlag);
        if (NULL == pstListHead)
        {
            PRINT_ERR("%s %d\n", __FUNCTION__, __LINE__);
            return;
        }

        LOS_ListAdd(pstListHead,&(pstNode->stFreeNodeInfo));
    }
}

/*****************************************************************************
 Function : osMemCheckUsedNode
 Description : check the result if pointer memory node belongs to pointer memory pool
 Input       : pPool --Pointer to memory pool
                  pstNode -- the node which need be checked
 Output      : None
 Return      : LOS_OK or LOS_NOK
*****************************************************************************/
#ifdef LOS_DLNK_SAFE_CHECK
INLINE UINT32 osMemCheckUsedNode(VOID *pPool, LOS_MEM_DYN_NODE *pstNode)
{
    LOS_MEM_DYN_NODE *pstTmp = NULL;
    LOS_MEM_POOL_INFO *pstPoolInfo = (LOS_MEM_POOL_INFO *)pPool;
    LOS_MEM_DYN_NODE *pstEnd = OS_MEM_END_NODE(pPool, pstPoolInfo->uwPoolSize);

    for (pstTmp = OS_MEM_FIRST_NODE(pPool); pstTmp < pstEnd; pstTmp = OS_MEM_NEXT_NODE(pstTmp ))
    {
        if ((pstTmp == pstNode) &&
            OS_MEM_NODE_GET_USED_FLAG(pstTmp->uwSizeAndFlag))
        {
            return LOS_OK;
        }
        else if (pstTmp > pstNode)
        {
            return LOS_NOK;
        }
    }

    return LOS_NOK;
}

#elif defined(LOS_DLNK_SIMPLE_CHECK)
INLINE UINT32 osMemCheckUsedNode(VOID *pPool, LOS_MEM_DYN_NODE *pstNode)
{
    LOS_MEM_POOL_INFO *pstPoolInfo = (LOS_MEM_POOL_INFO *)pPool;
    LOS_MEM_DYN_NODE *pstStartNode = OS_MEM_FIRST_NODE(pPool);
    LOS_MEM_DYN_NODE *pstEndNode = OS_MEM_END_NODE(pPool, pstPoolInfo->uwPoolSize);
    if (!OS_MEM_MIDDLE_ADDR_OPEN_END(pstStartNode, pstNode, pstEndNode))
    {
        return LOS_NOK;
    }

    if (!OS_MEM_NODE_GET_USED_FLAG(pstNode->uwSizeAndFlag))
    {
        return LOS_NOK;
    }

    if ((!OS_MEM_MAGIC_VALID(pstNode->stFreeNodeInfo.pstPrev))
        //|| (!OS_MEM_MAGIC_VALID(pstNode->stFreeNodeInfo.pstNext))
        )
    {
        return LOS_NOK;
    }

    return LOS_OK;
}

#else
INLINE BOOL osMemIsNodeValid(const LOS_MEM_DYN_NODE *pstNode, const LOS_MEM_DYN_NODE *pstStartNode, const LOS_MEM_DYN_NODE *pstEndNode,
       const UINT8 *pucStartPool, const UINT8 *pucEndPool)
{
    if (!OS_MEM_MIDDLE_ADDR(pstStartNode, pstNode, pstEndNode))
    {
        return FALSE;
    }

    if (OS_MEM_NODE_GET_USED_FLAG(pstNode->uwSizeAndFlag))
    {
        if ((!OS_MEM_MAGIC_VALID(pstNode->stFreeNodeInfo.pstPrev))
             // || (!OS_MEM_MAGIC_VALID(pstNode->stFreeNodeInfo.pstNext))
             )
        {
            return FALSE;
        }
        return TRUE;
    }

    if ((!OS_MEM_MIDDLE_ADDR_OPEN_END(pucStartPool, pstNode->stFreeNodeInfo.pstPrev, pucEndPool))
       // || (!OS_MEM_MIDDLE_ADDR_OPEN_END(pucStartPool, pstNode->stFreeNodeInfo.pstNext, pucEndPool))
       )
    {
        return FALSE;
    }

    return TRUE;
}

INLINE UINT32 osMemCheckUsedNode(VOID *pPool, LOS_MEM_DYN_NODE *pstNode)
{
    LOS_MEM_POOL_INFO *pstPoolInfo = (LOS_MEM_POOL_INFO *)pPool;
    LOS_MEM_DYN_NODE *pstStartNode = OS_MEM_FIRST_NODE(pPool);
    LOS_MEM_DYN_NODE *pstEndNode = OS_MEM_END_NODE(pPool, pstPoolInfo->uwPoolSize);
    UINT8 *pucEndPool = (UINT8 *)pPool + pstPoolInfo->uwPoolSize;
    const LOS_MEM_DYN_NODE *pstNextNode = (const LOS_MEM_DYN_NODE *)NULL;
    if (!osMemIsNodeValid(pstNode, pstStartNode, pstEndNode, (UINT8 *)pPool, pucEndPool))
    {
        return LOS_NOK;
    }

    if (!OS_MEM_NODE_GET_USED_FLAG(pstNode->uwSizeAndFlag))

    {
        return LOS_NOK;
    }

    pstNextNode = OS_MEM_NEXT_NODE(pstNode);
    if (!osMemIsNodeValid(pstNextNode, pstStartNode, pstEndNode, (UINT8 *)pPool, pucEndPool))
    {
        return LOS_NOK;
    }

    if (pstNextNode->pstPreNode != pstNode)
    {
        return LOS_NOK;
    }

    if (pstNode != pstStartNode)
    {
        if (!osMemIsNodeValid(pstNode->pstPreNode, pstStartNode, pstEndNode, (UINT8 *)pPool, pucEndPool))
        {
            return LOS_NOK;
        }

        if (OS_MEM_NEXT_NODE(pstNode->pstPreNode) != pstNode)
        {
            return LOS_NOK;
        }
    }

    return LOS_OK;
}

#endif

/*****************************************************************************
 Function : osMemSetMagicNumAndTaskid
 Description : set magic & taskid
 Input       : pstNode -- the node which will be set magic &  taskid
 Output      : None
 Return      : None
*****************************************************************************/
INLINE VOID osMemSetMagicNumAndTaskid(LOS_MEM_DYN_NODE *pstNode)
{
    OS_MEM_SET_MAGIC(pstNode->stFreeNodeInfo.pstPrev);

    /* In the process of dynamic memory initialization,direct use of uninitialized global variablewhich initialized in task initialization.
        Need to exclude this scene, make the value of pstNode->stFreeNodeInfo.pstNext to 0xffffffff */
    if (g_stLosTask.pstRunTask != NULL)
    {
        pstNode->stFreeNodeInfo.pstNext = (LOS_DL_LIST *)(g_stLosTask.pstRunTask->uwTaskID);
    }
    else
    {
        /* If the task mode does not initialize, the field is the 0xffffffff */
        pstNode->stFreeNodeInfo.pstNext = (LOS_DL_LIST *)0xffffffff;
        /* TODO: the commend task-MEMUSE is not include system initialization malloc */
    }
}

/*****************************************************************************
 Function : osMemAllocWithCheck
 Description : Allocate node from Memory pool
 Input       : pPool    --- Pointer to memory pool
                 uwSize  --- Size of memory in bytes to allocate
 Output      : None
 Return      : Pointer to allocated memory
*****************************************************************************/
INLINE VOID *osMemAllocWithCheck(VOID *pPool, UINT32  uwSize)
{
    LOS_MEM_DYN_NODE *pstAllocNode = (LOS_MEM_DYN_NODE *)NULL;
    UINT32 uwAllocSize;

    uwAllocSize = OS_MEM_ALIGN(uwSize + OS_MEM_NODE_HEAD_SIZE, OS_MEM_ALIGN_SIZE);
    pstAllocNode = osMemFindSuitableFreeBlock(pPool, uwAllocSize);
    if (pstAllocNode == NULL)
    {
        PRINT_ERR("[%s] No suitable free block, require free node size: 0x%x\n", __FUNCTION__, uwAllocSize);
        return NULL;
    }
    if ((uwAllocSize + OS_MEM_NODE_HEAD_SIZE + OS_MEM_ALIGN_SIZE) <= pstAllocNode->uwSizeAndFlag)
    {
        osMemSpitNode(pPool, pstAllocNode, uwAllocSize);
    }
    LOS_ListDelete(&(pstAllocNode->stFreeNodeInfo));
    osMemSetMagicNumAndTaskid(pstAllocNode);
    OS_MEM_NODE_SET_USED_FLAG(pstAllocNode->uwSizeAndFlag);
    OS_MEM_ADD_USED(OS_MEM_NODE_GET_SIZE(pstAllocNode->uwSizeAndFlag));
    return (pstAllocNode + 1);

}

/*****************************************************************************
 Function : osMemReAllocSmaller
 Description : reAlloc a smaller memory node
 Input       : pPool    --- Pointer to memory pool
                 uwAllocSize  --- the size of new node which will be alloced
                 pstNode --the node which wille be realloced
                 uwNodeSize -- the size of old node
 Output      : pstNode -- pointer to the new node after realloc
 Return      : None
*****************************************************************************/
INLINE VOID osMemReAllocSmaller(VOID *pPool, UINT32 uwAllocSize, LOS_MEM_DYN_NODE *pstNode, UINT32 uwNodeSize)
{
     if ((uwAllocSize + OS_MEM_NODE_HEAD_SIZE + OS_MEM_ALIGN_SIZE) <= uwNodeSize)
     {
         pstNode->uwSizeAndFlag = uwNodeSize;
         osMemSpitNode(pPool, pstNode, uwAllocSize);
         OS_MEM_NODE_SET_USED_FLAG(pstNode->uwSizeAndFlag);
         OS_MEM_REDUCE_USED(uwNodeSize - uwAllocSize);
     }
}

/*****************************************************************************
 Function : osMemMergeNodeForReAllocBigger
 Description : reAlloc a Bigger memory node after merge pstNode and nextNode
 Input       : pPool    --- Pointer to memory pool
                 uwAllocSize  --- the size of new node which will be alloced
                 pstNode --the node which wille be realloced
                 uwNodeSize -- the size of old node
                 pstNextNode -- pointer next node which will be merged
 Output      : pstNode -- pointer to the new node after realloc
 Return      : None
*****************************************************************************/
INLINE VOID osMemMergeNodeForReAllocBigger(VOID *pPool, UINT32 uwAllocSize, LOS_MEM_DYN_NODE *pstNode, UINT32 uwNodeSize, LOS_MEM_DYN_NODE *pstNextNode)
{
    pstNode->uwSizeAndFlag = uwNodeSize;
    LOS_ListDelete(&(pstNextNode->stFreeNodeInfo));
    osMemMergeNode(pstNextNode);
    OS_MEM_ADD_USED(pstNode->uwSizeAndFlag - uwNodeSize);
    if ((uwAllocSize + OS_MEM_NODE_HEAD_SIZE + OS_MEM_ALIGN_SIZE) <= pstNode->uwSizeAndFlag)
    {
        OS_MEM_REDUCE_USED(pstNode->uwSizeAndFlag - uwAllocSize);
        osMemSpitNode(pPool, pstNode, uwAllocSize);
    }
    OS_MEM_NODE_SET_USED_FLAG(pstNode->uwSizeAndFlag);
}

/*****************************************************************************
 Function : LOS_MemInit
 Description : Initialize Dynamic Memory pool
 Input       : pPool    --- Pointer to memory pool
                 uwSize  --- Size of memory in bytes to allocate
 Output      : None
 Return      : LOS_OK - Ok, OS_ERROR - Error
*****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_MemInit(VOID *pPool, UINT32  uwSize)
{
    LOS_MEM_DYN_NODE *pstNewNode = (LOS_MEM_DYN_NODE *)NULL;
    LOS_MEM_DYN_NODE *pstEndNode = (LOS_MEM_DYN_NODE *)NULL;
    LOS_MEM_POOL_INFO *pstPoolInfo = (LOS_MEM_POOL_INFO *)NULL;
    UINTPTR uvIntSave;
    LOS_DL_LIST *pstListHead = (LOS_DL_LIST *)NULL;

    if ((pPool == NULL) || (uwSize < (OS_MEM_MIN_POOL_SIZE)))
    {
        return OS_ERROR;
    }

    uvIntSave = LOS_IntLock();

    pstPoolInfo = (LOS_MEM_POOL_INFO *)pPool;
    pstPoolInfo->pPoolAddr = pPool;
    pstPoolInfo->uwPoolSize = uwSize;
    LOS_DLnkInitMultiHead(OS_MEM_HEAD_ADDR(pPool));
    pstNewNode = OS_MEM_FIRST_NODE(pPool);
    pstNewNode->uwSizeAndFlag = ((uwSize - ((UINT32)pstNewNode - (UINT32)pPool)) - OS_MEM_NODE_HEAD_SIZE);
    pstNewNode->pstPreNode = (LOS_MEM_DYN_NODE *)NULL;
    pstListHead = OS_MEM_HEAD(pPool, pstNewNode->uwSizeAndFlag);
    if (NULL == pstListHead)
    {
        PRINT_ERR("%s %d\n", __FUNCTION__, __LINE__);
        LOS_IntRestore(uvIntSave);
        return OS_ERROR;
    }

    LOS_ListTailInsert(pstListHead,&(pstNewNode->stFreeNodeInfo));
    pstEndNode = (LOS_MEM_DYN_NODE *)OS_MEM_END_NODE(pPool, uwSize);
    (VOID)memset(pstEndNode, 0 ,sizeof(*pstEndNode));
    pstEndNode->pstPreNode = pstNewNode;
    pstEndNode->uwSizeAndFlag = OS_MEM_NODE_HEAD_SIZE;
    OS_MEM_NODE_SET_USED_FLAG(pstEndNode->uwSizeAndFlag);
    osMemSetMagicNumAndTaskid(pstEndNode);
    LOS_IntRestore(uvIntSave);

    return LOS_OK;
}

/*****************************************************************************
 Function : LOS_MemAlloc
 Description : Allocate node from Memory pool
 Input       : pPool    --- Pointer to memory pool
                 uwSize  --- Size of memory in bytes to allocate
 Output      : None
 Return      : Pointer to allocated memory node
*****************************************************************************/
LITE_OS_SEC_TEXT VOID *LOS_MemAlloc (VOID *pPool, UINT32  uwSize)
{
    VOID *pPtr = NULL;
    UINTPTR uvIntSave = LOS_IntLock();

    do
    {
        if ((pPool == NULL) || (uwSize == 0))
        {
            break;
        }

        if (OS_MEM_NODE_GET_USED_FLAG(uwSize))
        {
            break;
        }

        pPtr = osMemAllocWithCheck(pPool, uwSize);
    } while (0);

    LOS_IntRestore(uvIntSave);
    return pPtr;
}

/*****************************************************************************
 Function : LOS_MemAllocAlign
 Description : align size then allocate node from Memory pool
 Input       : pPool    --- Pointer to memory pool
                 uwSize  --- Size of memory in bytes to allocate
                 uwBoundary -- align form
 Output      : None
 Return      : Pointer to allocated memory node
*****************************************************************************/
LITE_OS_SEC_TEXT VOID *LOS_MemAllocAlign(VOID *pPool, UINT32 uwSize, UINT32 uwBoundary)
{
    UINT32 uwUseSize = 0;
    UINT32 uwGapSize = 0;
    VOID *pPtr = NULL;
    VOID *pAlignedPtr = NULL;
    UINTPTR uvIntSave = LOS_IntLock();

    do
    {
        if ((pPool == NULL) || (uwSize == 0))
        {
            break;
        }

        uwUseSize = uwSize + uwBoundary + 4; /* 4bytes stores offset between alignedPtr and ptr */

        if (OS_MEM_NODE_GET_USED_FLAG(uwUseSize))
        {
            break;
        }

        pPtr = osMemAllocWithCheck(pPool, uwUseSize);

        pAlignedPtr = (VOID *)OS_MEM_ALIGN(pPtr, uwBoundary);

        if (pPtr == pAlignedPtr)
        {
            break;
        }

        /* store gapSize in address (ptr -4), it will be checked while free */
        uwGapSize = (UINT32)pAlignedPtr - (UINT32)pPtr;
        OS_MEM_NODE_SET_ALIGNED_FLAG(uwGapSize);
        *((UINT32 *)((UINT32)pAlignedPtr - 4)) = uwGapSize;

        pPtr = pAlignedPtr;

    } while (0);

     LOS_IntRestore(uvIntSave);

    return pPtr;
}

/*****************************************************************************
 Function : LOS_MemFree
 Description : free the node from memory & if there are free node beside, merger them.
                    at last update "pstListHead' which saved all free node control head
 Input       : pPool --Pointer to memory pool
                  pMem -- the node which need be freed
 Output      : None
 Return      : LOS_OK -Ok,  LOS_NOK -failed
*****************************************************************************/
LITE_OS_SEC_TEXT UINT32 LOS_MemFree(VOID *pPool, VOID *pMem)
{
    UINT32 uwRet = LOS_NOK;
    UINT32 uwGapSize = 0;
    UINTPTR uvIntSave = LOS_IntLock();

    do
    {
        LOS_MEM_DYN_NODE *pstNode = (LOS_MEM_DYN_NODE *)NULL;

        if ((pPool == NULL) || (pMem == NULL))
        {
            break;
        }

        uwGapSize = *((UINT32 *)((UINT32)pMem - 4));
        if (OS_MEM_NODE_GET_ALIGNED_FLAG(uwGapSize))
        {
            uwGapSize = OS_MEM_NODE_GET_ALIGNED_GAPSIZE(uwGapSize);
            pMem = (VOID *)((UINT32)pMem - uwGapSize);
        }

        pstNode = (LOS_MEM_DYN_NODE *)((UINT32)pMem - OS_MEM_NODE_HEAD_SIZE);
        uwRet = osMemCheckUsedNode(pPool, pstNode);
        if (uwRet == LOS_OK)
        {
            osMemFreeNode(pstNode, pPool);
        }

    } while(0);

    LOS_IntRestore(uvIntSave);
    return uwRet;
}

/*****************************************************************************
 Function : LOS_MemRealloc
 Description : reAlloc memory node
 Input       : pPool    --- Pointer to memory pool
                  pPtr  --- pointer to memory node which will be realloced
                 uwSize  --- the size of new node
 Output      : None
 Return      : Pointer to allocated memory
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR VOID *LOS_MemRealloc (VOID *pPool,  VOID *pPtr, UINT32 uwSize)
{
    UINTPTR uvIntSave;
    UINT32 uwGapSize = 0;
    VOID *pNewPtr = NULL;

    uvIntSave = LOS_IntLock();

    do
    {
        LOS_MEM_DYN_NODE *pstNode = (LOS_MEM_DYN_NODE *)NULL;
        UINT32 uwRet;
        UINT32 uwAllocSize;
        UINT32 uwNodeSize;
        LOS_MEM_DYN_NODE *pstNextNode = (LOS_MEM_DYN_NODE *)NULL;

        if (pPtr == NULL)
        {
            pNewPtr = LOS_MemAlloc((VOID *)pPool, (UINT32)uwSize);
            break;
        }

        if (uwSize == 0)
        {
            if (LOS_MemFree((VOID *)pPool, (VOID *)pPtr) != LOS_OK)
                 PRINT_ERR("%s, %d\n", __FUNCTION__, __LINE__);
            break;
        }

        uwGapSize = *((UINT32 *)((UINT32)pPtr - 4));
        if (OS_MEM_NODE_GET_ALIGNED_FLAG(uwGapSize))
        {
            uwGapSize = OS_MEM_NODE_GET_ALIGNED_GAPSIZE(uwGapSize);
            pPtr = (VOID *)((UINT32)pPtr - uwGapSize);
        }
        pstNode = (LOS_MEM_DYN_NODE *)((UINT32)pPtr - OS_MEM_NODE_HEAD_SIZE);
        uwRet = osMemCheckUsedNode(pPool, pstNode);
        if (uwRet != LOS_OK)
        {
            break;
        }

        uwAllocSize = OS_MEM_ALIGN(uwSize + OS_MEM_NODE_HEAD_SIZE, OS_MEM_ALIGN_SIZE);
        uwNodeSize = OS_MEM_NODE_GET_SIZE(pstNode->uwSizeAndFlag);
        if (uwNodeSize >= uwAllocSize)
        {
            osMemReAllocSmaller(pPool, uwAllocSize, pstNode, uwNodeSize);
            pNewPtr = pPtr;
            break;
        }

        pstNextNode = OS_MEM_NEXT_NODE(pstNode);
        if ((!OS_MEM_NODE_GET_USED_FLAG(pstNextNode->uwSizeAndFlag)) &&
            ((pstNextNode->uwSizeAndFlag + uwNodeSize) >= uwAllocSize))
        {
            osMemMergeNodeForReAllocBigger(pPool, uwAllocSize, pstNode, uwNodeSize, pstNextNode);
            pNewPtr = pPtr;
            break;
        }

        pNewPtr = osMemAllocWithCheck(pPool, uwSize);
        if (pNewPtr != NULL)
        {
            (VOID)memcpy(pNewPtr, pPtr, uwNodeSize - OS_MEM_NODE_HEAD_SIZE);
            osMemFreeNode(pstNode, pPool);
        }

    } while (0);

    LOS_IntRestore(uvIntSave);
    return pNewPtr;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */
