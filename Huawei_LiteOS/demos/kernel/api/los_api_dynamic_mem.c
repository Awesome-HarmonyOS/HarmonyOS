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

#include <stdlib.h>
#include "los_config.h"
#include "los_memory.h"
#include "los_api_dynamic_mem.h"
#include "los_inspect_entry.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#define MEM_DYN_SIZE  256
static UINT32 pDynMem[MEM_DYN_SIZE/4];
extern UINT32 LOS_MemInit(VOID *pPool, UINT32 uwSize);

UINT32 Example_Dyn_Mem(VOID)
{
    UINT32 *p_num = NULL;
    UINT32 uwRet;
    uwRet = LOS_MemInit(pDynMem, MEM_DYN_SIZE);
    if (LOS_OK == uwRet)
    {
        dprintf("mempool init ok! \r\n");
    }
    else
    {
        dprintf("mempool init failed! \r\n");
        return LOS_NOK;
    }

    /* mem alloc */
    p_num = (UINT32 *)LOS_MemAlloc(pDynMem, 4);
    if (NULL == p_num)
    {
        dprintf("mem alloc failed! \r\n");
        return LOS_NOK;
    }
    dprintf("mem alloc ok \r\n");

    /* assignment */
    *p_num = 828;
    dprintf("*p_num = %d \r\n", *p_num);

    /* mem free */
    uwRet = LOS_MemFree(pDynMem, p_num);
    if (LOS_OK == uwRet)
    {
        dprintf("mem free ok!\r\n");
        uwRet = LOS_InspectStatusSetByID(LOS_INSPECT_DMEM, LOS_INSPECT_STU_SUCCESS);
        if (LOS_OK != uwRet)
        {
            dprintf("Set Inspect Status Err \r\n");
        }
    }
    else
    {
        dprintf("mem free failed! \r\n");
        uwRet = LOS_InspectStatusSetByID(LOS_INSPECT_DMEM, LOS_INSPECT_STU_ERROR);
        if (LOS_OK != uwRet)
        {
            dprintf("Set Inspect Status Err \r\n");
        }
        return LOS_NOK;
    }
    return LOS_OK;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */
