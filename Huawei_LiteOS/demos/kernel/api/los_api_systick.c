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

#include "los_sys.h"
#include "los_task.h"
#include "los_api_systick.h"
#include "los_inspect_entry.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

VOID Example_TransformTime(VOID)
{
    UINT32 uwMs;
    UINT32 uwTick;
    uwTick = LOS_MS2Tick(10000);
    dprintf("uwTick = %d .\r\n",uwTick);
    uwMs = LOS_Tick2MS(100);
    dprintf("uwMs = %d .\r\n",uwMs);
}

UINT32 Example_GetTick(VOID)
{
    UINT32 uwRet = LOS_OK;
    UINT32 uwcyclePerTick;
    UINT64 uwTickCount1,uwTickCount2;

    uwcyclePerTick  = LOS_CyclePerTickGet();
    if (0 != uwcyclePerTick)
    {
        dprintf("LOS_CyclePerTickGet = %d .\r\n", uwcyclePerTick);
    }

    uwTickCount1 = LOS_TickCountGet();
    if (0 != uwTickCount1)
    {
        dprintf("LOS_TickCountGet = %d .\r\n", (UINT32)uwTickCount1);
    }
    LOS_TaskDelay(200);
    uwTickCount2 = LOS_TickCountGet();
    if (0 != uwTickCount2)
    {
        dprintf("LOS_TickCountGet after delay = %d .\r\n", (UINT32)uwTickCount2);
    }

    if ((uwTickCount2 - uwTickCount1) >= 200)
    {
        uwRet = LOS_InspectStatusSetByID(LOS_INSPECT_SYSTIC, LOS_INSPECT_STU_SUCCESS);
        if (LOS_OK != uwRet)
        {
            dprintf("Set Inspect Status Err .\r\n");
        }
        return LOS_OK;
    }
    else
    {
        uwRet = LOS_InspectStatusSetByID(LOS_INSPECT_SYSTIC, LOS_INSPECT_STU_ERROR);
        if (LOS_OK != uwRet)
        {
            dprintf("Set Inspect Status Err .\r\n");
        }
        return LOS_NOK;
    }
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */
