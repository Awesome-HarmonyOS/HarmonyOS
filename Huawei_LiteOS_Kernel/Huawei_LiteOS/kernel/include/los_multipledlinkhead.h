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

#ifndef _LOS_MULTIPLE_DLINK_HEAD_H
#define _LOS_MULTIPLE_DLINK_HEAD_H

#include "los_base.h"
#include "los_list.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


#define OS_MAX_MULTI_DLNK_LOG2              30
#define OS_MIN_MULTI_DLNK_LOG2              4
#define OS_MULTI_DLNK_NUM                   ((OS_MAX_MULTI_DLNK_LOG2 - OS_MIN_MULTI_DLNK_LOG2) + 1)
#define OS_DLNK_HEAD_SIZE                   OS_MULTI_DLNK_HEAD_SIZE
#define OS_DLnkInitHead                     LOS_DLnkInitMultiHead
#define OS_DLnkHead                         LOS_DLnkMultiHead
#define OS_DLnkNextHead                     LOS_DLnkNextMultiHead
#define OS_DLnkFirstHead                    LOS_DLnkFirstMultiHead
#define OS_MULTI_DLNK_HEAD_SIZE             sizeof(LOS_MULTIPLE_DLNK_HEAD)

typedef struct
{
    LOS_DL_LIST stListHead[OS_MULTI_DLNK_NUM];
} LOS_MULTIPLE_DLNK_HEAD;

INLINE LOS_DL_LIST *LOS_DLnkNextMultiHead(VOID *pHeadAddr, LOS_DL_LIST *pstListHead)
{
    LOS_MULTIPLE_DLNK_HEAD *head = (LOS_MULTIPLE_DLNK_HEAD *)pHeadAddr;

    return (&(head->stListHead[OS_MULTI_DLNK_NUM - 1]) == pstListHead) ? NULL : (pstListHead + 1);
}

INLINE LOS_DL_LIST *LOS_DLnkFirstMultiHead(VOID *pHeadAddr)
{
    return (LOS_DL_LIST *)pHeadAddr;
}

extern VOID LOS_DLnkInitMultiHead(VOID *pHeadAddr);
extern LOS_DL_LIST *LOS_DLnkMultiHead(VOID *pHeadAddr, UINT32 uwSize);


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* _LOS_MULTIPLE_DLINK_HEAD_H */
