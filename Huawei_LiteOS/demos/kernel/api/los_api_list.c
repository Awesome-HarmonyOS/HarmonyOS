/*----------------------------------------------------------------------------
 * Copyright (c) <2016-2018>, <Huawei Technologies Co., Ltd>
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

#include "los_list.h"
#include <stdio.h>
#include <stdlib.h>
#include "los_config.h"
#include "los_memory.h"
#include "los_api_list.h"
#include "los_inspect_entry.h"

#ifdef LOSCFG_LIB_LIBC
#include "string.h"
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

UINT32 Example_list(VOID)
{
    UINT32 uwRet = LOS_OK;

    /* init */
    dprintf("initial...... \r\n");
    LOS_DL_LIST *head;
    head = (LOS_DL_LIST *)LOS_MemAlloc(m_aucSysMem0, sizeof(LOS_DL_LIST));
    if (head == NULL)
    {
        dprintf("malloc failed \r\n");
        return LOS_NOK;
    }

    LOS_ListInit(head);
    if (!LOS_ListEmpty(head))
    {
        dprintf("initial failed \r\n");
        return LOS_NOK;
    }

    /* tail insert node*/
    dprintf("node add and tail add......\r\n");

    LOS_DL_LIST *node1 = (LOS_DL_LIST *)LOS_MemAlloc(m_aucSysMem0, sizeof(LOS_DL_LIST));
    if (node1 == NULL)
    {
        dprintf("malloc failed\n");
        return LOS_NOK;
    }

    LOS_DL_LIST *node2 = (LOS_DL_LIST *)LOS_MemAlloc(m_aucSysMem0, sizeof(LOS_DL_LIST));
    if (node2 == NULL)
    {
        dprintf("malloc failed \r\n");
        LOS_MemFree(m_aucSysMem0, node1);
        return LOS_NOK;
    }

    LOS_DL_LIST *tail = (LOS_DL_LIST *)LOS_MemAlloc(m_aucSysMem0, sizeof(LOS_DL_LIST));
    if (tail == NULL)
    {
        dprintf("malloc failed \r\n");
        LOS_MemFree(m_aucSysMem0, node1);
        LOS_MemFree(m_aucSysMem0, node2);
        return LOS_NOK;
    }


    LOS_ListAdd(head,node1);
    LOS_ListAdd(node1,node2);
    if ((node1->pstPrev == head) || (node2->pstPrev == node1))
    {
        dprintf("add node success \r\n");
    }

    LOS_ListTailInsert(head, tail);
    if (tail->pstPrev == node2)
    {
        dprintf("add tail success \r\n");
    }

    /* delete node */
    dprintf("delete node......\r\n");
    LOS_ListDelete(node1);
    LOS_MemFree(m_aucSysMem0, node1);
    if (head->pstNext == node2)
    {
        dprintf("delete node success\r\n");
        uwRet = LOS_InspectStatusSetByID(LOS_INSPECT_LIST, LOS_INSPECT_STU_SUCCESS);
        if (LOS_OK != uwRet)
        {
            dprintf("Set Inspect Status Err\r\n");
        }
    }
    else
    {
        dprintf("delete node error\r\n");
        uwRet = LOS_InspectStatusSetByID(LOS_INSPECT_LIST, LOS_INSPECT_STU_ERROR);
        if (LOS_OK != uwRet)
        {
            dprintf("Set Inspect Status Err\r\n");
        }
    }

    return LOS_OK;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */
