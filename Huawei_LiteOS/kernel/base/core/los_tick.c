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

#include "los_tick.inc"
#include "los_base.ph"
#include "los_swtmr.ph"
#include "los_task.ph"
#include "los_timeslice.ph"
#if (LOSCFG_KERNEL_TICKLESS == YES)
#include "los_tickless.ph"
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */


LITE_OS_SEC_BSS UINT64      g_ullTickCount;
LITE_OS_SEC_BSS UINT32      g_uwTicksPerSec;
LITE_OS_SEC_BSS UINT32      g_uwCyclePerSec;
LITE_OS_SEC_BSS UINT32      g_uwCyclesPerTick;
LITE_OS_SEC_BSS UINT32      g_uwSysClock;
LITE_OS_SEC_DATA_INIT BOOL  g_bSysTickStart = FALSE;

#if (LOSCFG_KERNEL_TICKLESS == YES)
/*****************************************************************************
 Description : Tick interruption handler
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT VOID osTickHandlerLoop(UINT32 uwElapseTicks)
{
    UINT32 uwIndex;

    for (uwIndex = 0; uwIndex < uwElapseTicks; uwIndex++)
    {
#if (LOSCFG_BASE_CORE_TICK_HW_TIME == YES)
        platform_tick_handler();
#endif

        g_ullTickCount ++;

#if(LOSCFG_BASE_CORE_TIMESLICE == YES)
        osTimesliceCheck();
#endif
        osTaskScan();   //task timeout scan
#if (LOSCFG_BASE_CORE_SWTMR == YES)
        (VOID)osSwtmrScan();
#endif
    }
}

#endif

/*****************************************************************************
 Description : Tick interruption handler
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT VOID osTickHandler(VOID)
{
#if (LOSCFG_KERNEL_TICKLESS == YES)
    if (g_bReloadSysTickFlag)
    {
        LOS_SysTickReload(OS_SYS_CLOCK / LOSCFG_BASE_CORE_TICK_PER_SECOND);
        g_bReloadSysTickFlag = 0;
    }
    g_bTickIrqFlag = g_bTicklessFlag;

    #if (LOSCFG_PLATFORM_HWI == NO)
    if (g_uwSysTickIntFlag == TICKLESS_OS_TICK_INT_WAIT)
    {
        g_uwSysTickIntFlag = TICKLESS_OS_TICK_INT_SET;
    }
    #endif
#endif

#if (LOSCFG_BASE_CORE_TICK_HW_TIME == YES)
    platform_tick_handler();
#endif

    g_ullTickCount ++;

#if(LOSCFG_BASE_CORE_TIMESLICE == YES)
    osTimesliceCheck();
#endif

    osTaskScan();   //task timeout scan

#if (LOSCFG_BASE_CORE_SWTMR == YES)
    (VOID)osSwtmrScan();
#endif
}

LITE_OS_SEC_TEXT UINT32 LOS_SysClockGet(void)
{
    return g_uwSysClock;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */
