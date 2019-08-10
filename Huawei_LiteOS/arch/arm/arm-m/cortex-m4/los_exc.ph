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

#ifndef _LOS_EXC_PH
#define _LOS_EXC_PH

#include "los_exc.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

#if (LOSCFG_PLATFORM_EXC == YES)

typedef enum
{
    OS_EXC_TYPE_CONTEXT     = 0,
    OS_EXC_TYPE_TSK         = 1,
    OS_EXC_TYPE_QUE         = 2,
    OS_EXC_TYPE_NVIC        = 3,
    OS_EXC_TYPE_TSK_SWITCH  = 4,
    OS_EXC_TYPE_MEM         = 5,
    OS_EXC_TYPE_MAX         = 6
} EXC_INFO_TYPE;

typedef struct tagExcInfoCallBackArray
{
    EXC_INFO_TYPE           uwType;
    UINT32                  uwValid;
    EXC_INFO_SAVE_CALLBACK  pFnExcInfoCb;
    VOID*                   pArg;
}EXC_INFO_ARRAY_S;


#define OS_NVIC_SHCSR                       0xE000ED24
#define OS_NVIC_CCR                         0xE000ED14

#define OS_NVIC_INT_ENABLE_SIZE             0x20
#define OS_NVIC_INT_PRI_SIZE                0xF0

#define OS_NVIC_INT_PEND_SIZE               OS_NVIC_INT_ACT_SIZE
#define OS_NVIC_INT_ACT_SIZE                OS_NVIC_INT_ENABLE_SIZE


#define MAX_SCENE_INFO_SIZE     (8 + sizeof(EXC_INFO_S) - 4 + sizeof(EXC_CONTEXT_S))
#define MAX_TSK_INFO_SIZE       (8 + sizeof(TSK_INFO_S) * (LOSCFG_BASE_CORE_TSK_LIMIT + 1))
#define MAX_INT_INFO_SIZE       (8 + 0x164)

#if (LOSCFG_BASE_IPC_QUEUE == YES)
#define MAX_QUEUE_INFO_SIZE     (8 + sizeof(QUEUE_INFO_S) * LOSCFG_BASE_IPC_QUEUE_LIMIT)
#else
#define MAX_QUEUE_INFO_SIZE     (0)
#endif

#if (LOSCFG_BASE_CORE_EXC_TSK_SWITCH == YES)
#define MAX_SWITCH_INFO_SIZE    (8 + (sizeof(UINT32) + sizeof(CHAR) * LOS_TASK_NAMELEN) * OS_TASK_SWITCH_INFO_COUNT)
#else
#define MAX_SWITCH_INFO_SIZE    (0)
#endif

#if (LOSCFG_BASE_MEM_NODE_INTEGRITY_CHECK == YES)
#define MAX_MEM_INFO_SIZE       (8 + sizeof(MEM_INFO_S) * OS_SYS_MEM_NUM)
#else
#define MAX_MEM_INFO_SIZE       (0)
#endif

#if (LOSCFG_SAVE_EXC_INFO == YES)
#define MAX_EXC_MEM_SIZE ( 4 + MAX_SCENE_INFO_SIZE + MAX_TSK_INFO_SIZE + MAX_QUEUE_INFO_SIZE + MAX_INT_INFO_SIZE + MAX_SWITCH_INFO_SIZE + MAX_MEM_INFO_SIZE + 4)
#else
#define MAX_EXC_MEM_SIZE ( 0 )
#endif

extern VOID osExcRegister(EXC_INFO_TYPE uwType, EXC_INFO_SAVE_CALLBACK pFunc, VOID *pArg);

#endif

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif
