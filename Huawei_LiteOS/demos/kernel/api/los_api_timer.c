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

#include <stdio.h>
#include "los_swtmr.h"
#include "time.h"
#include "los_sys.h"
#include "los_api_timer.h"
#include "los_inspect_entry.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

static UINT32 g_timercount1 = 0;
static UINT32 g_timercount2 = 0;

static VOID Timer1_Callback(UINT32 arg)
{
    UINT32 tick_last1;

    g_timercount1++;
    tick_last1 = (UINT32)LOS_TickCountGet();
    dprintf("g_timercount1=%d .\r\n", g_timercount1);
    dprintf("tick_last1=%d .\r\n", tick_last1);
}

static VOID Timer2_Callback(UINT32 arg)
{
    UINT32 uwRet = LOS_OK;
    UINT32 tick_last2;

    tick_last2 = (UINT32)LOS_TickCountGet();
    g_timercount2++;
    dprintf("g_timercount2=%d .\r\n", g_timercount2);
    dprintf("tick_last2=%d .\r\n", tick_last2);
    uwRet = LOS_InspectStatusSetByID(LOS_INSPECT_TIMER, LOS_INSPECT_STU_SUCCESS);
    if (LOS_OK != uwRet)
    {
        dprintf("Set Inspect Status Err .\r\n");
    }
}

UINT32 Example_swTimer(VOID)
{
    UINT16 id1;
    UINT16 id2;// timer id
    UINT32 uwRet = LOS_OK;

#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
    uwRet = LOS_SwtmrCreate(1000, LOS_SWTMR_MODE_ONCE, Timer1_Callback, &id1, 1, OS_SWTMR_ROUSES_ALLOW, OS_SWTMR_ALIGN_SENSITIVE);
#else
    uwRet = LOS_SwtmrCreate(1000, LOS_SWTMR_MODE_ONCE, Timer1_Callback, &id1, 1);
#endif
    if (LOS_OK != uwRet)
    {
        dprintf("create Timer1 failed .\r\n");
    }
    else
    {
        dprintf("create Timer1 success .\r\n");
    }

#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == YES)
    uwRet = LOS_SwtmrCreate(100, LOS_SWTMR_MODE_PERIOD, Timer2_Callback, &id2, 1, OS_SWTMR_ROUSES_ALLOW, OS_SWTMR_ALIGN_SENSITIVE);
#else
    uwRet = LOS_SwtmrCreate(100, LOS_SWTMR_MODE_PERIOD, Timer2_Callback, &id2, 1);
#endif
    if(LOS_OK != uwRet)
    {
        dprintf("create Timer2 failed .\r\n");
    }
    else
    {
        dprintf("create Timer2 success .\r\n");
    }

    uwRet = LOS_SwtmrStart(id1);
    if (LOS_OK != uwRet)
    {
        dprintf("start Timer1 failed .\r\n");
    }
    else
    {
        dprintf("start Timer1 sucess .\r\n");
    }

    (VOID)LOS_TaskDelay(200);

    uwRet = LOS_SwtmrStop(id1);
    if (LOS_OK != uwRet)
    {
        dprintf("stop Timer1 failed .\r\n");
    }
    else
    {
        dprintf("stop Timer1 sucess .\r\n");
    }

    uwRet = LOS_SwtmrStart(id1);
    if (LOS_OK != uwRet)
    {
        dprintf("start Timer1 failed .\r\n");
    }

    (VOID)LOS_TaskDelay(1000);

    /*the timer that mode is once, kernel will delete it automatically when timer is timeout */
    uwRet = LOS_SwtmrDelete(id1);
    if (LOS_OK != uwRet)
    {
        dprintf("delete Timer1 failed .\r\n");
    }
    else
    {
        dprintf("delete Timer1 sucess .\r\n");
    }

    uwRet = LOS_SwtmrStart(id2);
    if (LOS_OK != uwRet)
    {
        dprintf("start Timer2 failed .\r\n");
    }
    else
    {
        dprintf("start Timer2 success .\r\n");
    }

    (VOID)LOS_TaskDelay(1000);

    uwRet = LOS_SwtmrStop(id2);
    if (LOS_OK != uwRet)
    {
        dprintf("stop Timer2 failed .\r\n");
    }

    uwRet = LOS_SwtmrDelete(id2);
    if (LOS_OK != uwRet)
    {
        dprintf("delete Timer2 failed .\r\n");
    }

    return LOS_OK;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */
