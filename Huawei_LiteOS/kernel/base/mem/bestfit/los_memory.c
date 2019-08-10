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

/*ISP_LINT*/
/*lint -e7
 -esym(7,*)*/
/*lint -e826 -e834 -e835 -e845 -e838
 -esym(826,*) -esym(834,*) -esym(835,*) -esym(845,*) -esym(838,*)*/
#include "string.h"
#include "los_memory.h"
#include "los_memory.inc"
#include "los_task.ph"
#include "los_exc.h"

#if (LOSCFG_MEM_TASK_USED_STATISTICS == YES)
#include "los_memstat.inc"
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/**
 * @ingroup los_memory
 * Memory linked list node structure
 */
typedef struct tagLOS_MEM_DYN_NODE
{
    LOS_DL_LIST stFreeNodeInfo;             /**<Free memory node  */
    struct tagLOS_MEM_DYN_NODE *pstPreNode;    /**<Pointer to the previous memory node*/
    UINT32 uwSizeAndFlag;                   /**<Size and flag of the current node (the highest bit represents a flag, and the rest bits specify the size)*/
}LOS_MEM_DYN_NODE;

/**
 * @ingroup los_memory
 * Memory pool information structure
 */
typedef struct tagLOS_MEM_POOL_INFO
{
    VOID *pPoolAddr;                        /**<Starting address of a memory pool  */
    UINT32 uwPoolSize;                      /**<Memory pool size    */
} LOS_MEM_POOL_INFO;

extern VOID print_hex(unsigned int *ptr, unsigned int num);

extern UINT8 *m_aucSysMem0;
LITE_OS_SEC_TEXT_INIT UINT32 osMemSystemInit()
{
    UINT32 uwRet;
    UINT32 uwMemSize;

    m_aucSysMem0 = (UINT8 *)(((UINT32)m_aucSysMem0 + (64 - 1)) & ~(64 - 1));
    uwMemSize = g_sys_mem_addr_end  - OS_SYS_NOCACHEMEM_SIZE - (UINT32)m_aucSysMem0;
    PRINT_INFO("LiteOS heap memory address:0x%x,size:0x%x\n",m_aucSysMem0,uwMemSize);
    uwRet = LOS_MemInit(m_aucSysMem0, uwMemSize);
    return uwRet;
}

#if OS_SYS_NOCACHEMEM_SIZE
extern UINT8 *m_aucSysNoCacheMem0;
LITE_OS_SEC_TEXT_INIT UINT32 osNocacheMemSystemInit(VOID)
{
    UINT32 uwRet;
    m_aucSysNoCacheMem0 = (g_sys_mem_addr_end - OS_SYS_NOCACHEMEM_SIZE);
    uwRet = LOS_MemInit(m_aucSysNoCacheMem0, OS_SYS_NOCACHEMEM_SIZE);
    return uwRet;
}
#endif

LITE_OS_SEC_DATA_INIT MALLOC_HOOK g_MALLOC_HOOK = (MALLOC_HOOK)NULL; /*lint !e611*/

VOID *osMemFindNodeCtrl(VOID *pPtr);
#ifdef OS_MEM_CHECK_DEBUG
LITE_OS_SEC_DATA static UINT8 ucCheckMemLevel = (UINT8)LOS_MEM_CHECK_LEVEL_DEFAULT;
#endif

LITE_OS_SEC_TEXT_MINOR VOID LOS_MemFreeNodeCheck(VOID *pPool, LOS_MEM_DYN_NODE *pstNode)
{
    LOS_DL_LIST *pstListHead = (LOS_DL_LIST *)NULL;
    LOS_MULTIPLE_DLNK_HEAD *pstHeadAddr = (LOS_MULTIPLE_DLNK_HEAD *)((UINT32)pPool + sizeof(LOS_MEM_POOL_INFO));
    UINT32 uwSized = pstNode->uwSizeAndFlag & 0x3fffffff;
    UINT32 uwIdx = (UINT32)(31 - __builtin_clz(uwSized) - OS_MIN_MULTI_DLNK_LOG2);
    LOS_TASK_CB *pstTaskCB;
    UINT32 uwTaskID;
    UINT32 uwflag = 0;

    if (OS_MEM_NODE_GET_USED_FLAG(pstNode->uwSizeAndFlag))
        return;
    pstListHead = pstHeadAddr->stListHead[uwIdx].pstNext;
    if (pstListHead == pstNode->stFreeNodeInfo.pstPrev)
        uwflag = 1;
    while (pstListHead != &(pstHeadAddr->stListHead[uwIdx]) && (uwflag == 0))
    {
        pstListHead = pstListHead->pstNext;
        if (pstListHead == pstNode->stFreeNodeInfo.pstPrev)
        {
            uwflag = 1;
        }
    }

    if (uwflag == 0)
    {
        uwTaskID = (UINT32)(pstNode->pstPreNode->stFreeNodeInfo.pstNext);
        if (uwTaskID >= g_uwTskMaxNum)
        {
        LOS_Panic("///////[LOS_MemFreeNodeCheck] Task ID %d in pre node is invalid!////////\n", uwTaskID);
        }
        pstTaskCB = OS_TCB_FROM_TID(uwTaskID);

        if ((pstTaskCB->usTaskStatus & OS_TASK_STATUS_UNUSED) ||
            (pstTaskCB->pfnTaskEntry == NULL) ||
            (pstTaskCB->pcTaskName == NULL))
        {
            LOS_Panic("\r\n[LOS_MemFreeNodeCheck] Task ID %d in pre node  is not created!\n", uwTaskID);
        }
        LOS_Panic("[LOS_MemFreeNodeCheck] cur node: 0x%x\n"
                   "pre node: 0x%x\n"
                   "pre node was allocated by task:%s\n",
                   pstNode, pstNode->pstPreNode,  pstTaskCB->pcTaskName);/*lint !e515 !e516*/
    }
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
LITE_OS_SEC_ALW_INLINE STATIC_INLINE LOS_MEM_DYN_NODE *osMemFindSuitableFreeBlock(VOID *pPool, UINT32 uwAllocSize)
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
LITE_OS_SEC_ALW_INLINE STATIC_INLINE VOID osMemClearNode(LOS_MEM_DYN_NODE *pstNode)
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
LITE_OS_SEC_ALW_INLINE STATIC_INLINE VOID osMemMergeNode(LOS_MEM_DYN_NODE *pstNode)
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
LITE_OS_SEC_ALW_INLINE STATIC_INLINE VOID osMemSpitNode(VOID *pPool,
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
LITE_OS_SEC_TEXT STATIC_INLINE VOID osMemFreeNode(LOS_MEM_DYN_NODE *pstNode, VOID *pPool)
{
    LOS_MEM_DYN_NODE *pstNextNode = (LOS_MEM_DYN_NODE *)NULL;
    LOS_DL_LIST *pstListHead = (LOS_DL_LIST *)NULL;

#if (LOSCFG_MEM_TASK_USED_STATISTICS == YES)
    OS_MEM_REDUCE_USED(OS_MEM_NODE_GET_SIZE(pstNode->uwSizeAndFlag));
#endif
    pstNode->uwSizeAndFlag = OS_MEM_NODE_GET_SIZE(pstNode->uwSizeAndFlag);
    if ((pstNode->pstPreNode != NULL) &&
        (!OS_MEM_NODE_GET_USED_FLAG(pstNode->pstPreNode->uwSizeAndFlag)))
    {
        LOS_MEM_DYN_NODE *pstPreNode = pstNode->pstPreNode;
        osMemMergeNode(pstNode);
        pstNextNode = OS_MEM_NEXT_NODE(pstPreNode);
        if (!OS_MEM_NODE_GET_USED_FLAG(pstNextNode->uwSizeAndFlag))
        {
#ifdef OS_MEM_ENABLE_ALLOC_CHECK
            LOS_MemFreeNodeCheck(pPool, pstNextNode);//fjg
#endif
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
#ifdef OS_MEM_ENABLE_ALLOC_CHECK
            LOS_MemFreeNodeCheck(pPool, pstNextNode);//fjg
#endif
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
LITE_OS_SEC_TEXT_MINOR STATIC_INLINE UINT32 osMemCheckUsedNode(VOID *pPool, LOS_MEM_DYN_NODE *pstNode)
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
LITE_OS_SEC_TEXT_MINOR STATIC_INLINE UINT32 osMemCheckUsedNode(VOID *pPool, LOS_MEM_DYN_NODE *pstNode)
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
LITE_OS_SEC_ALW_INLINE STATIC_INLINE BOOL osMemIsNodeValid(const LOS_MEM_DYN_NODE *pstNode, const LOS_MEM_DYN_NODE *pstStartNode, const LOS_MEM_DYN_NODE *pstEndNode,
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

LITE_OS_SEC_TEXT_MINOR STATIC_INLINE UINT32 osMemCheckUsedNode(VOID *pPool, LOS_MEM_DYN_NODE *pstNode)
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
LITE_OS_SEC_TEXT_MINOR STATIC_INLINE VOID osMemSetMagicNumAndTaskid(LOS_MEM_DYN_NODE *pstNode)
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
 Function : LOS_MemIntegrityCheck
 Description : memory pool integrity checking
 Input       : pPool --Pointer to memory pool
 Output      : None
 Return      : LOS_OK --memory pool integrate  or LOS_NOK--memory pool impaired
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_MemIntegrityCheck(VOID *pPool)
{
    LOS_MEM_DYN_NODE *pstTmpNode = (LOS_MEM_DYN_NODE *)NULL;
    LOS_MEM_DYN_NODE *pstPreNode = (LOS_MEM_DYN_NODE *)NULL;
    LOS_MEM_POOL_INFO *pstPoolInfo = (LOS_MEM_POOL_INFO *)pPool;
    UINT8 *pucEndPool;
    LOS_TASK_CB *pstTaskCB;
    UINT32 uwTaskID;
    UINTPTR uvIntSave;

    if (pPool == NULL)
    {
        return LOS_NOK;
    }

    pucEndPool = (UINT8 *)pPool + pstPoolInfo->uwPoolSize;

    uvIntSave = LOS_IntLock();
    pstPreNode = OS_MEM_FIRST_NODE(pPool);
    for (pstTmpNode = OS_MEM_FIRST_NODE(pPool); pstTmpNode < OS_MEM_END_NODE(pPool, pstPoolInfo->uwPoolSize);
        pstTmpNode = OS_MEM_NEXT_NODE(pstTmpNode))
    {

        if (OS_MEM_NODE_GET_USED_FLAG(pstTmpNode->uwSizeAndFlag))
        {
            if (!OS_MEM_MAGIC_VALID(pstTmpNode->stFreeNodeInfo.pstPrev))
            {
                PRINT_ERR("[%s], %d, memory check error!\n"
                    "memory used but magic num wrong, stFreeNodeInfo.pstPrev(magic num):0x%x \n",
                    __FUNCTION__, __LINE__, pstTmpNode->stFreeNodeInfo.pstPrev); /*lint !e626 !e515*/
                goto errout;
            }
        }
        else //is free node, check free node range
        {
            if (!OS_MEM_MIDDLE_ADDR_OPEN_END(pPool, pstTmpNode->stFreeNodeInfo.pstPrev, pucEndPool))
            {
                PRINT_ERR("[%s], %d, memory check error!\n"
                    "stFreeNodeInfo.pstPrev:0x%x is out of legal mem range[0x%x, 0x%x]\n",
                    __FUNCTION__, __LINE__, pstTmpNode->stFreeNodeInfo.pstPrev, pPool, pucEndPool); /*lint !e626 !e515*/
                goto errout;
            }
            if (!OS_MEM_MIDDLE_ADDR_OPEN_END(pPool, pstTmpNode->stFreeNodeInfo.pstNext, pucEndPool))
            {
                PRINT_ERR("[%s], %d, memory check error!\n"
                    "stFreeNodeInfo.pstNext:0x%x is out of legal mem range[0x%x, 0x%x]\n",
                    __FUNCTION__, __LINE__, pstTmpNode->stFreeNodeInfo.pstNext, pPool, pucEndPool); /*lint !e626 !e515*/
                goto errout;
            }

        LOS_MemFreeNodeCheck(pPool, pstTmpNode);//fjg
        }

        pstPreNode = pstTmpNode;
    }
    LOS_IntRestore(uvIntSave);
    return LOS_OK;

errout:
    LOS_IntRestore(uvIntSave);
    PRINT_ERR("<------- ERR Node Pre 0x%x byte\n", OS_EXC_ERR_NODE_RANGE);/*lint !e515 !e516*/
    print_hex((unsigned int *)(pstTmpNode - OS_EXC_ERR_NODE_RANGE), OS_EXC_ERR_NODE_RANGE / 4);
    PRINT_ERR("ERR Node Next 0x%x byte ------->\n", OS_EXC_ERR_NODE_RANGE);/*lint !e515 !e516*/
    print_hex((unsigned int *)pstTmpNode, OS_EXC_ERR_NODE_RANGE / 4);
    uwTaskID = (UINT32)(pstPreNode->stFreeNodeInfo.pstNext);
    if (uwTaskID >= g_uwTskMaxNum)
    {
        LOS_Panic("Task ID %d in pre node is invalid!\n", uwTaskID);
    }
    pstTaskCB = OS_TCB_FROM_TID(uwTaskID);

    if ((pstTaskCB->usTaskStatus & OS_TASK_STATUS_UNUSED) ||
        (pstTaskCB->pfnTaskEntry == NULL) ||
        (pstTaskCB->pcTaskName == NULL))
    {
        LOS_Panic("\r\nTask ID %d in pre node  is not created!\n", uwTaskID);
    }
    LOS_Panic("cur node: 0x%x\n"
               "pre node: 0x%x\n"
               "pre node was allocated by task:%s\n",
               pstTmpNode, pstPreNode,  pstTaskCB->pcTaskName);/*lint !e515 !e516*/
    return LOS_NOK;
}

/*****************************************************************************
 Function : osMemAllocWithCheck
 Description : Allocate node from Memory pool
 Input       : pPool    --- Pointer to memory pool
                 uwSize  --- Size of memory in bytes to allocate
 Output      : None
 Return      : Pointer to allocated memory
*****************************************************************************/
LITE_OS_SEC_TEXT STATIC_INLINE VOID *osMemAllocWithCheck(VOID *pPool, UINT32  uwSize)
{
    LOS_MEM_DYN_NODE *pstAllocNode = (LOS_MEM_DYN_NODE *)NULL;
    UINT32 uwAllocSize;

    if (g_MALLOC_HOOK != NULL)
        g_MALLOC_HOOK();

#ifdef OS_MEM_ENABLE_ALLOC_CHECK
    (VOID)LOS_MemIntegrityCheck(pPool);
#endif

    uwAllocSize = OS_MEM_ALIGN(uwSize + OS_MEM_NODE_HEAD_SIZE, OS_MEM_ALIGN_SIZE);
    pstAllocNode = osMemFindSuitableFreeBlock(pPool, uwAllocSize);
    if (pstAllocNode == NULL)
    {
        PRINT_ERR("[%s] No suitable free block\n", __FUNCTION__);/*lint !e515*/
        return NULL;
    }
    if ((uwAllocSize + OS_MEM_NODE_HEAD_SIZE + OS_MEM_ALIGN_SIZE) <= pstAllocNode->uwSizeAndFlag)
    {
        osMemSpitNode(pPool, pstAllocNode, uwAllocSize);
    }
    LOS_ListDelete(&(pstAllocNode->stFreeNodeInfo));
    osMemSetMagicNumAndTaskid(pstAllocNode);
    OS_MEM_NODE_SET_USED_FLAG(pstAllocNode->uwSizeAndFlag);
#if (LOSCFG_MEM_TASK_USED_STATISTICS == YES)
    OS_MEM_ADD_USED(OS_MEM_NODE_GET_SIZE(pstAllocNode->uwSizeAndFlag));
#endif
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
LITE_OS_SEC_TEXT_MINOR STATIC_INLINE VOID osMemReAllocSmaller(VOID *pPool, UINT32 uwAllocSize, LOS_MEM_DYN_NODE *pstNode, UINT32 uwNodeSize)
{
     if ((uwAllocSize + OS_MEM_NODE_HEAD_SIZE + OS_MEM_ALIGN_SIZE) <= uwNodeSize)
     {
         pstNode->uwSizeAndFlag = uwNodeSize;
         osMemSpitNode(pPool, pstNode, uwAllocSize);
         OS_MEM_NODE_SET_USED_FLAG(pstNode->uwSizeAndFlag);
#if (LOSCFG_MEM_TASK_USED_STATISTICS == YES)
         OS_MEM_REDUCE_USED(uwNodeSize - uwAllocSize);
#endif
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
LITE_OS_SEC_TEXT_MINOR STATIC_INLINE VOID osMemMergeNodeForReAllocBigger(VOID *pPool, UINT32 uwAllocSize, LOS_MEM_DYN_NODE *pstNode, UINT32 uwNodeSize, LOS_MEM_DYN_NODE *pstNextNode)
{
    pstNode->uwSizeAndFlag = uwNodeSize;
    LOS_ListDelete(&(pstNextNode->stFreeNodeInfo));
    osMemMergeNode(pstNextNode);
#if (LOSCFG_MEM_TASK_USED_STATISTICS == YES)
    OS_MEM_ADD_USED(pstNode->uwSizeAndFlag - uwNodeSize);
#endif
    if ((uwAllocSize + OS_MEM_NODE_HEAD_SIZE + OS_MEM_ALIGN_SIZE) <= pstNode->uwSizeAndFlag)
    {
#if (LOSCFG_MEM_TASK_USED_STATISTICS == YES)
        OS_MEM_REDUCE_USED(pstNode->uwSizeAndFlag - uwAllocSize);
#endif
        osMemSpitNode(pPool, pstNode, uwAllocSize);
    }
    OS_MEM_NODE_SET_USED_FLAG(pstNode->uwSizeAndFlag);
}

/*****************************************************************************
 Function : LOS_MemInit
 Description : Initialize Dynamic Memory pool
 Input       : pPool    --- Pointer to memory pool
               uwSize   --- Size of memory in bytes to allocate
 Output      : None
 Return      : LOS_OK - Ok, LOS_NOK - Error
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
        return LOS_NOK;
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
        return LOS_NOK;
    }

    LOS_ListTailInsert(pstListHead,&(pstNewNode->stFreeNodeInfo));
    pstEndNode = (LOS_MEM_DYN_NODE *)OS_MEM_END_NODE(pPool, uwSize);
    memset(pstEndNode, 0 ,sizeof(*pstEndNode));
    pstEndNode->pstPreNode = pstNewNode;
    pstEndNode->uwSizeAndFlag = OS_MEM_NODE_HEAD_SIZE;
    OS_MEM_NODE_SET_USED_FLAG(pstEndNode->uwSizeAndFlag);
    osMemSetMagicNumAndTaskid(pstEndNode);
    LOS_IntRestore(uvIntSave);

    return LOS_OK;
}

/*****************************************************************************
 Function : LOS_MemAlloc
 Description : Allocate Memory from Memory pool
 Input       : pPool    --- Pointer to memory pool
               uwSize   --- Size of memory in bytes to allocate
 Output      : None
 Return      : Pointer to allocated memory
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
 Input       : pPool        --- Pointer to memory pool
               uwSize       --- Size of memory in bytes to allocate
               uwBoundary   --- align form
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
 Description : Free Memory and return it to Memory pool
 Input       : pPool    --- Pointer to memory pool
               pMem     --- Pointer to memory to free
 Output      : None
 Return      : LOS_OK - OK, LOS_NOK - Error
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
 Description : realloc memory from Memory pool
 Input       : pPool    --- Pointer to memory pool
               pPtr     --- Pointer to memory
               uwSize   --- new size
 Output      : None
 Return      : Pointer to allocated memory node
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR VOID *LOS_MemRealloc (VOID *pPool,  VOID *pPtr, UINT32 uwSize)
{
    UINTPTR uvIntSave;
    UINT32 uwGapSize = 0;
    VOID *pNewPtr = NULL;

    if((int)uwSize < 0)
        return NULL;

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
            memcpy(pNewPtr, pPtr, uwNodeSize - OS_MEM_NODE_HEAD_SIZE);
            osMemFreeNode(pstNode, pPool);
        }

    } while (0);

    LOS_IntRestore(uvIntSave);
    return pNewPtr;
}

/*****************************************************************************
 Function : LOS_MemTotalUsedGet
 Description : figure the pointer memory pool for it's total mem used
 Input       : pPool    --- Pointer to memory pool
 Output      : None
 Return      : the size of the pool has been used
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_MemTotalUsedGet(VOID *pPool)
{
    LOS_MEM_DYN_NODE *pstTmpNode = (LOS_MEM_DYN_NODE *)NULL;
    LOS_MEM_POOL_INFO *pstPoolInfo = (LOS_MEM_POOL_INFO *)pPool;
    UINT32 uwMemUsed = 0;
    UINTPTR uvIntSave;

    if (pPool == NULL)
    {
        return LOS_NOK;
    }

    uvIntSave = LOS_IntLock();

    for (pstTmpNode = OS_MEM_FIRST_NODE(pPool); pstTmpNode <= OS_MEM_END_NODE(pPool, pstPoolInfo->uwPoolSize);
        pstTmpNode = OS_MEM_NEXT_NODE(pstTmpNode))
    {
        if (OS_MEM_NODE_GET_USED_FLAG(pstTmpNode->uwSizeAndFlag))
        {
            uwMemUsed += OS_MEM_NODE_GET_SIZE(pstTmpNode->uwSizeAndFlag);
        }
    }

    LOS_IntRestore(uvIntSave);

    return uwMemUsed;
}

/*****************************************************************************
 Function : LOS_MemLastUsedGet
 Description : get the size of last node(except end node which size is zero)
 Input       : pPool    --- Pointer to memory pool
 Output      : None
 Return      : the size of the last node
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_MemLastUsedGet(VOID *pPool)
{
    LOS_MEM_POOL_INFO *pstPoolInfo = (LOS_MEM_POOL_INFO *)pPool;

    if (pPool == NULL)
    {
        return LOS_NOK;
    }

    return ((UINT32)(OS_MEM_END_NODE(pPool, pstPoolInfo->uwPoolSize)->pstPreNode) + sizeof(LOS_MEM_DYN_NODE));
}

/*****************************************************************************
 Function : LOS_MemUsedBlksGet
 Description : get the number of used node
 Input       : pPool    --- Pointer to memory pool
 Output      : None
 Return      : the number of used node
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_MemUsedBlksGet(VOID *pPool)
{
    LOS_MEM_DYN_NODE *pstTmpNode = (LOS_MEM_DYN_NODE *)NULL;
    LOS_MEM_POOL_INFO *pstPoolInfo = (LOS_MEM_POOL_INFO *)pPool;
    UINT32 uwBlkNums = 0;
    UINTPTR uvIntSave;

    if (pPool == NULL)
    {
        return LOS_NOK;
    }

    uvIntSave = LOS_IntLock();

    for (pstTmpNode = OS_MEM_FIRST_NODE(pPool); pstTmpNode <= OS_MEM_END_NODE(pPool, pstPoolInfo->uwPoolSize);
        pstTmpNode= OS_MEM_NEXT_NODE(pstTmpNode))
    {
        if (OS_MEM_NODE_GET_USED_FLAG(pstTmpNode->uwSizeAndFlag))
        {
            uwBlkNums++;
        }
    }

    LOS_IntRestore(uvIntSave);

    return uwBlkNums;
}

/*****************************************************************************
 Function :LOS_MemTaskIdGet
 Description : get a memory node's taskID if pointer node is "used node"
 Input       : pPtr   --- pointer to aim node
 Output      : None
 Return      : taskID --Ok or OS_INVALID --pointer node is illegal or free node
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_MemTaskIdGet(VOID *pPtr)
{
    LOS_MEM_DYN_NODE *pstTmpNode = (LOS_MEM_DYN_NODE *)NULL;
    LOS_MEM_POOL_INFO *pstPoolInfo = (LOS_MEM_POOL_INFO *)OS_SYS_MEM_ADDR;
    UINTPTR uvIntSave;

    if (pPtr == NULL ||
        pPtr < (VOID *)OS_MEM_FIRST_NODE(OS_SYS_MEM_ADDR) ||
        pPtr > (VOID *)OS_MEM_END_NODE(OS_SYS_MEM_ADDR, pstPoolInfo->uwPoolSize))
    {
        PRINT_ERR("input ptr 0x%x is out of system memory range[0x%x, 0x%x]\n", (UINT32)pPtr, OS_MEM_FIRST_NODE(OS_SYS_MEM_ADDR),
            OS_MEM_END_NODE(OS_SYS_MEM_ADDR, pstPoolInfo->uwPoolSize));/*lint !e515 !e516*/
        return OS_INVALID;
    }

    uvIntSave= LOS_IntLock();

    for (pstTmpNode = OS_MEM_FIRST_NODE(OS_SYS_MEM_ADDR); pstTmpNode <= OS_MEM_END_NODE(OS_SYS_MEM_ADDR, pstPoolInfo->uwPoolSize);
        pstTmpNode = OS_MEM_NEXT_NODE(pstTmpNode))
    {

        if ((UINT32)pPtr < (UINT32)pstTmpNode)
        {
            if (OS_MEM_NODE_GET_USED_FLAG(pstTmpNode->pstPreNode->uwSizeAndFlag))
            {
                LOS_IntRestore(uvIntSave);
                return (UINT32)(pstTmpNode->pstPreNode->stFreeNodeInfo.pstNext);
            }
            else
            {
                LOS_IntRestore(uvIntSave);
                PRINT_ERR("input ptr 0x%x is belong to a free mem node\n", pPtr); /*lint !e626 !e515*/
                return OS_INVALID;
            }
        }
    }

    LOS_IntRestore(uvIntSave);
    return OS_INVALID;
}

/*****************************************************************************
 Function : LOS_MemFreeBlksGet
 Description : get the number of free node
 Input       : pool    --- Pointer to memory pool
 Output      : None
 Return      : the number of free node
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_MemFreeBlksGet(VOID *pPool)
{
    LOS_MEM_DYN_NODE *pstTmpNode = (LOS_MEM_DYN_NODE *)NULL;
    LOS_MEM_POOL_INFO *pstPoolInfo = (LOS_MEM_POOL_INFO *)pPool;
    UINT32 uwBlkNums = 0;
    UINTPTR uvIntSave;

    if (pPool == NULL)
    {
        return LOS_NOK;
    }

    uvIntSave = LOS_IntLock();

    for (pstTmpNode = OS_MEM_FIRST_NODE(pPool); pstTmpNode <= OS_MEM_END_NODE(pPool, pstPoolInfo->uwPoolSize);
        pstTmpNode = OS_MEM_NEXT_NODE(pstTmpNode))
    {
        if (!OS_MEM_NODE_GET_USED_FLAG(pstTmpNode->uwSizeAndFlag))
        {
            uwBlkNums++;
        }
    }

    LOS_IntRestore(uvIntSave);

    return uwBlkNums;
}

/*****************************************************************************
 Function : osMemResetEndNode
 Description : reset "end node"
 Input       : None
 Output      : endNode -- pointer to "end node"
 Return      : the number of free node
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR LOS_MEM_DYN_NODE *osMemResetEndNode(VOID)
{
    LOS_MEM_DYN_NODE *pstEndNode = (LOS_MEM_DYN_NODE *)OS_MEM_END_NODE(OS_SYS_MEM_ADDR, OS_SYS_MEM_SIZE);
    memset(pstEndNode, 0, sizeof(*pstEndNode));
    pstEndNode->pstPreNode = (LOS_MEM_DYN_NODE *)NULL;
    pstEndNode->uwSizeAndFlag = OS_MEM_NODE_HEAD_SIZE;
    OS_MEM_NODE_SET_USED_FLAG(pstEndNode->uwSizeAndFlag);

    return pstEndNode;
}

/*****************************************************************************
 Function : LOS_MemPoolSizeGet
 Description : get the memory pool's size
 Input       : pPool    --- Pointer to memory pool
 Output      : LOS_NOK & Other value -- The size of the memory pool.
 Return      : the size of the memory pool
*****************************************************************************/
 LITE_OS_SEC_TEXT_MINOR UINT32 LOS_MemPoolSizeGet(VOID *pPool)
{
    if (pPool == NULL)
    {
        return LOS_NOK;
    }
    return ((LOS_MEM_POOL_INFO *)pPool)->uwPoolSize;
}

/*****************************************************************************
 Function : LOS_MemGetUsed
 Description : figure the system memory pool for it's total mem used
 Input       : None
 Output      : None
 Return      : the size of the system memory pool has been used
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_MemGetUsed(VOID)
{
    return LOS_MemTotalUsedGet(OS_SYS_MEM_ADDR);
}

/*****************************************************************************
 Function : LOS_MemGetTotal
 Description : get the system memory pool's size
 Input       : None
 Output      : None
 Return      : the size of the system memory pool
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_MemGetTotal(VOID)
{
    return ((LOS_MEM_POOL_INFO *)OS_SYS_MEM_ADDR)->uwPoolSize;
}

#ifdef OS_MEM_CHECK_DEBUG

/*****************************************************************************
  Function: LOS_MemNodeSizeCheck
  Description: get a pNode's(pPtr) size ,include total size and available size
  Input        :pPool --which pPool doesn't your pPtr belong to
                   pPtr --point to source node
  Output      :puwTotalSize -- save total size
                   puwAvailSize -- save availabe size
  Return : errorID or LOS_OK
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_MemNodeSizeCheck(VOID *pPool, VOID *pPtr, UINT32 *puwTotalSize, UINT32 *puwAvailSize)
{
    VOID *pHead = NULL;
    LOS_MEM_POOL_INFO *pstPoolInfo = (LOS_MEM_POOL_INFO *)pPool;
    UINT8 *pucEndPool;

    if ( ucCheckMemLevel == LOS_MEM_CHECK_LEVEL_DISABLE)
    {
        return OS_ERRNO_MEMCHECK_DISABLED;
    }

    if ( NULL == pPool)
    {
        return OS_ERRNO_MEMCHECK_NOT_INIT;
    }
    if (pPtr == NULL)
    {
        return OS_ERRNO_MEMCHECK_PARA_NULL;
    }

    pucEndPool = (UINT8 *)pPool + pstPoolInfo->uwPoolSize;
    if ( !( OS_MEM_MIDDLE_ADDR_OPEN_END(pPool, (UINT8 *)pPtr, pucEndPool)))
    {
        return OS_ERRNO_MEMCHECK_OUTSIDE;
    }

    if ( ucCheckMemLevel == LOS_MEM_CHECK_LEVEL_HIGH)
    {
        pHead = osMemFindNodeCtrl(pPtr);
        if ((pHead == NULL) || ((((LOS_MEM_DYN_NODE *)pHead)->uwSizeAndFlag & (~OS_MEM_NODE_USED_FLAG)) < ((UINT32)pPtr - (UINT32)pHead)))
        {
            return OS_ERRNO_MEMCHECK_NO_HEAD;
        }
        *puwTotalSize = (((LOS_MEM_DYN_NODE *)pHead)->uwSizeAndFlag - sizeof(LOS_MEM_DYN_NODE)) & (~OS_MEM_NODE_USED_FLAG);
        *puwAvailSize = (((LOS_MEM_DYN_NODE *)pHead)->uwSizeAndFlag - ((UINT32)pPtr - (UINT32)pHead)) & (~OS_MEM_NODE_USED_FLAG);
        return LOS_OK;
    }
    if ( ucCheckMemLevel == LOS_MEM_CHECK_LEVEL_LOW)
    {
        if (pPtr != (VOID *)OS_MEM_ALIGN(pPtr, OS_MEM_ALIGN_SIZE))
        {
            return OS_ERRNO_MEMCHECK_NO_HEAD;
        }
        pHead = (VOID *)((UINT32)pPtr - sizeof(LOS_MEM_DYN_NODE));
        if (OS_MEM_MAGIC_VALID(((LOS_MEM_DYN_NODE *)pHead)->stFreeNodeInfo.pstPrev))
        {
            *puwTotalSize = (((LOS_MEM_DYN_NODE *)pHead)->uwSizeAndFlag - sizeof(LOS_MEM_DYN_NODE)) & (~OS_MEM_NODE_USED_FLAG);
            *puwAvailSize = (((LOS_MEM_DYN_NODE *)pHead)->uwSizeAndFlag - sizeof(LOS_MEM_DYN_NODE)) & (~OS_MEM_NODE_USED_FLAG);
            return LOS_OK;
        }
        else
        {
            return OS_ERRNO_MEMCHECK_NO_HEAD;
        }
    }

    return OS_ERRNO_MEMCHECK_WRONG_LEVEL;
}

/*****************************************************************************
Function     : osMemFindNodeCtrl
Description : get a pool's memCtrl
Input         :pPtr -- point to source pPtr
Output       :None
Return        : search forward for pPtr's memCtrl or "NULL"
@attention : this func couldn't ensure the return memCtrl belongs to pPtr
it just find forward the most nearly one
*******************************************************************************/
LITE_OS_SEC_TEXT_MINOR VOID *osMemFindNodeCtrl(VOID *pPtr)
{
    UINT8 *pucHead = (UINT8 *)pPtr;

    if( pPtr == NULL )
    {
        return NULL;
    }

    pucHead = (UINT8 *)OS_MEM_ALIGN((VOID *)pucHead, OS_MEM_ALIGN_SIZE);
    while (!OS_MEM_MAGIC_VALID(((LOS_MEM_DYN_NODE *)pucHead)->stFreeNodeInfo.pstPrev))
    {
        pucHead -= 4;
    }
    return pucHead;
}

/*****************************************************************************
 Function : LOS_MemCheckLevelSet
 Description : setting ucCheckMemLevel which decide the manner of memcheck
 Input       : ucLevel -- waht level want to set
 Output      : None
 Return      : LOS_OK -- setting succeed
                  OS_ERRNO_MEMCHECK_WRONG_LEVEL -- setting failed due to illegal parameter
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_MemCheckLevelSet(UINT8 ucLevel)
{
    if ( ucLevel == LOS_MEM_CHECK_LEVEL_LOW)
    {
        PRINTK("%s: LOS_MEM_CHECK_LEVEL_LOW \n", __FUNCTION__);
    }
    else if(ucLevel == LOS_MEM_CHECK_LEVEL_HIGH)
    {
        PRINTK("%s: LOS_MEM_CHECK_LEVEL_HIGH \n", __FUNCTION__);
    }
    else if (ucLevel == LOS_MEM_CHECK_LEVEL_DISABLE)
    {
        PRINTK("%s: LOS_MEM_CHECK_LEVEL_DISABLE \n", __FUNCTION__);
    }
    else
    {
        PRINTK("%s: wrong para, setting failed !! \n", __FUNCTION__);
        return OS_ERRNO_MEMCHECK_WRONG_LEVEL;
    }
    ucCheckMemLevel = ucLevel;
    return LOS_OK;
}

LITE_OS_SEC_TEXT_MINOR UINT8 LOS_MemCheckLevelGet(VOID)
{
    return ucCheckMemLevel;
}

#endif /* OS_MEM_CHECK_DEBUG */

LITE_OS_SEC_TEXT_MINOR UINT32 osMemSysNodeCheck(VOID *pDst, VOID *pSrc, UINT32 uwLength, UINT8 ucPos)
{
#ifdef OS_MEM_CHECK_DEBUG
    UINT32 uwRet = 0;
    UINT32 uwTotalSize=0;
    UINT32 uwAvailSize=0;

    if (ucPos == 0) /* if this func was called by memset */
    {
        uwRet = LOS_MemNodeSizeCheck(m_aucSysMem0, pDst, &uwTotalSize, &uwAvailSize);
        if (uwRet == LOS_OK && uwLength > uwAvailSize)
        {
            PRINT_ERR("---------------------------------------------\n");/*lint !e515*/
            PRINT_ERR("memset: dst inode uwAvailSize is not enough"
                                " uwAvailSize = 0x%x uwLength = 0x%x\n", uwAvailSize, uwLength);/*lint !e515 !e516*/
            osBackTrace();
            PRINT_ERR("---------------------------------------------\n");/*lint !e515*/
            return LOS_NOK;
        }
    }
    else if (ucPos == 1) /* if this func was called by memcpy */
    {
        uwRet = LOS_MemNodeSizeCheck(m_aucSysMem0, pDst, &uwTotalSize, &uwAvailSize);
        if (uwRet == LOS_OK && uwLength > uwAvailSize)
        {
            PRINT_ERR("---------------------------------------------\n");/*lint !e515*/
            PRINT_ERR("memcpy: dst inode uwAvailSize is not enough"
                               " uwAvailSize = 0x%x uwLength = 0x%x\n", uwAvailSize, uwLength);/*lint !e515 !e516*/
            osBackTrace();
            PRINT_ERR("---------------------------------------------\n");/*lint !e515*/
            return LOS_NOK;
        }
        uwRet = LOS_MemNodeSizeCheck(m_aucSysMem0, pSrc, &uwTotalSize, &uwAvailSize);
        if (uwRet == LOS_OK && uwLength > uwAvailSize)
        {
            PRINT_ERR("---------------------------------------------\n");/*lint !e515*/
            PRINT_ERR("memcpy: src inode uwAvailSize is not enough"
                               " uwAvailSize = 0x%x uwLength = 0x%x\n", uwAvailSize, uwLength);/*lint !e515 !e516*/
            osBackTrace();
            PRINT_ERR("---------------------------------------------\n");/*lint !e515*/
            return LOS_NOK;
        }
    }
#endif
    return LOS_OK;
}


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */
