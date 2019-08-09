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

#ifndef _LOS_MEMORY_INC
#define _LOS_MEMORY_INC

#include "los_memory.ph"
#include "los_multipledlinkhead.ph"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


#define OS_MEM_ENABLE_MEM_STATISTICS
#ifdef OS_MEM_ENABLE_MEM_STATISTICS
#include "los_memstat.ph"
#define OS_MEM_ADD_USED(arg)         osTaskMemUsedInc(arg)
#define OS_MEM_REDUCE_USED(arg)      osTaskMemUsedDec(arg)
#else
#define OS_MEM_ADD_USED(arg)
#define OS_MEM_REDUCE_USED(arg)
#endif

#define OS_MEM_ALIGN(p, alignSize)    (((unsigned int)(p) + alignSize -1) &(~ (alignSize -1)))
#define OS_MEM_NODE_HEAD_SIZE sizeof(LOS_MEM_DYN_NODE)
#define OS_MEM_MIN_POOL_SIZE (OS_DLNK_HEAD_SIZE + 2 * OS_MEM_NODE_HEAD_SIZE + sizeof(LOS_MEM_POOL_INFO))
#define OS_MEM_ALIGN_SIZE 4
#define OS_MEM_NODE_USED_FLAG 0x80000000
#define OS_MEM_NODE_ALIGNED_FLAG 0x40000000

#define OS_MEM_NODE_GET_ALIGNED_FLAG(uwSizeAndFlag) ((uwSizeAndFlag) & OS_MEM_NODE_ALIGNED_FLAG)
#define OS_MEM_NODE_SET_ALIGNED_FLAG(uwSizeAndFlag)  (uwSizeAndFlag = ((uwSizeAndFlag) | OS_MEM_NODE_ALIGNED_FLAG))
#define OS_MEM_NODE_GET_ALIGNED_GAPSIZE(uwSizeAndFlag) ((uwSizeAndFlag) & (~OS_MEM_NODE_ALIGNED_FLAG))
#define OS_MEM_NODE_GET_USED_FLAG(uwSizeAndFlag) ((uwSizeAndFlag) & OS_MEM_NODE_USED_FLAG)
#define OS_MEM_NODE_SET_USED_FLAG(uwSizeAndFlag) (uwSizeAndFlag = ((uwSizeAndFlag) | OS_MEM_NODE_USED_FLAG))
#define OS_MEM_NODE_GET_SIZE(uwSizeAndFlag) ((uwSizeAndFlag) & (~OS_MEM_NODE_USED_FLAG))
#define OS_MEM_IS_NODE_NEXT_EXIST(pstNode, pstPoolInfo) (((UINT32)(pstNode) + (pstNode)->uwSizeAndFlag) < ((UINT32)(pstPoolInfo) + (pstPoolInfo)->uwPoolSize))
#define OS_MEM_HEAD(pPool, uwSize) OS_DLnkHead(OS_MEM_HEAD_ADDR(pPool), uwSize)
#define OS_MEM_HEAD_ADDR(pPool) ((VOID *)((UINT32)(pPool) + sizeof(LOS_MEM_POOL_INFO)))
#define OS_MEM_NEXT_NODE(pstNode) ((LOS_MEM_DYN_NODE *)((UINT8 *)(pstNode) + OS_MEM_NODE_GET_SIZE((pstNode)->uwSizeAndFlag)))
#define OS_MEM_FIRST_NODE(pPool) ((LOS_MEM_DYN_NODE *) ((UINT8 *)OS_MEM_HEAD_ADDR(pPool) + OS_DLNK_HEAD_SIZE))
#define OS_MEM_END_NODE(pPool, uwSize) ((LOS_MEM_DYN_NODE *)(((UINT8 *)(pPool) + (uwSize)) - OS_MEM_NODE_HEAD_SIZE))
#define OS_MEM_MIDDLE_ADDR_OPEN_END(startAddr, middleAddr, endAddr) (((UINT8 *)(startAddr) <= ((UINT8 *)(middleAddr))) && (((UINT8 *)(middleAddr)) < ((UINT8 *)(endAddr))))
#define OS_MEM_MIDDLE_ADDR(startAddr, middleAddr, endAddr) (((UINT8 *)(startAddr) <= ((UINT8 *)(middleAddr))) && (((UINT8 *)(middleAddr)) <= ((UINT8 *)(endAddr))))
#define OS_MEM_SET_MAGIC(value) (value) = (LOS_DL_LIST *)((UINT32)(&(value)) ^ 0xffffffff)
#define OS_MEM_MAGIC_VALID(value) ((((UINT32)(value)) ^ ((UINT32)(&(value)))) == 0xffffffff)


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_MEMORY_INC */
