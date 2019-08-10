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


#include "los_hwi.h"
#include "los_tick.ph"
#include "los_tickless.inc"
#include "los_hw.h"
#include "los_task.h"

#if (LOSCFG_KERNEL_TICKLESS == YES)

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

BOOL g_bTicklessFlag = 1;
BOOL g_bTickIrqFlag = 0;
BOOL g_bReloadSysTickFlag = 0;
#if (LOSCFG_PLATFORM_HWI == NO)
enum TICKLESS_OS_TICK_INT_FLAG g_uwSysTickIntFlag = TICKLESS_OS_TICK_INT_INIT;
#endif

volatile UINT32 g_uwSleepTicks = 0;

extern UINT32 osTaskNextSwitchTimeGet(VOID);
extern UINT32 osSwTmrGetNextTimeout(VOID);


LITE_OS_SEC_TEXT VOID LOS_TicklessEnable(VOID)
{
    g_bTicklessFlag = 1;
}

LITE_OS_SEC_TEXT VOID LOS_TicklessDisable(VOID)
{
    g_bTicklessFlag = 0;
}

static inline UINT32 osSleepTicksGet(VOID)
{
    UINT32 uwTskSortLinkTicks = 0;
    UINT32 uwSwtmrSortLinkTicks = 0;
    UINT32 uwSleepTicks = 0;

    /** Context guarantees that the interrupt has been closed */
    uwTskSortLinkTicks  = osTaskNextSwitchTimeGet();
    uwSwtmrSortLinkTicks = osSwTmrGetNextTimeout();

    uwSleepTicks = (uwTskSortLinkTicks < uwSwtmrSortLinkTicks) ? uwTskSortLinkTicks : uwSwtmrSortLinkTicks;
    return uwSleepTicks;
}

inline VOID osUpdateKernelTickCount(UINT32 uwHwiIndex)
{
    /** this function must be called in interrupt context */
    if (g_uwSleepTicks > 1)
    {
        UINT32 uwCyclesPerTick = OS_SYS_CLOCK / LOSCFG_BASE_CORE_TICK_PER_SECOND;
        UINT32 uwCurrSysCycles, uwElapseSysCycles, uwElapseTicks, uwRemainSysCycles;

        g_bReloadSysTickFlag = 0;
    #if (LOSCFG_PLATFORM_HWI == YES)
        if (uwHwiIndex == (SysTick_IRQn + OS_SYS_VECTOR_CNT))
    #else
        if (g_uwSysTickIntFlag == TICKLESS_OS_TICK_INT_SET) /* OS tick interrupt */
    #endif
        {
            uwElapseTicks = (g_uwSleepTicks - 1);
            LOS_SysTickReload(OS_SYS_CLOCK / LOSCFG_BASE_CORE_TICK_PER_SECOND);
        }
        else
        {
            uwCurrSysCycles = LOS_SysTickCurrCycleGet();
        #if (LOSCFG_SYSTICK_CNT_DIR_DECREASE == YES)
            uwElapseSysCycles = ((g_uwSleepTicks * uwCyclesPerTick) - uwCurrSysCycles);
        #else
            uwElapseSysCycles = uwCurrSysCycles;
        #endif
            uwElapseTicks = uwElapseSysCycles / uwCyclesPerTick;
            uwRemainSysCycles = uwElapseSysCycles % uwCyclesPerTick;
            if (uwRemainSysCycles > 0)
            {
                LOS_SysTickReload(uwRemainSysCycles);
                g_bReloadSysTickFlag = 1;
            }
            else
            {
                LOS_SysTickReload(uwCyclesPerTick);
            }
        }
        osTickHandlerLoop(uwElapseTicks);
        g_uwSleepTicks = 0;
     #if (LOSCFG_PLATFORM_HWI == NO)
        g_uwSysTickIntFlag = TICKLESS_OS_TICK_INT_INIT;
     #endif
    }
}

VOID osTicklessStart(VOID)
{
    UINT32 uwCyclesPerTick = OS_SYS_CLOCK / LOSCFG_BASE_CORE_TICK_PER_SECOND;
    UINT32 uwMaxTicks = LOSCFG_SYSTICK_LOAD_RELOAD_MAX / uwCyclesPerTick;
    UINTPTR uvIntSave = 0;
    UINT32 uwSleepTicks = 0;

    uvIntSave = LOS_IntLock();
    LOS_SysTickStop();
    if (LOS_SysTickGetIntStatus()) /* SysTick interrupt pend */
    {
        goto out;
    }

    uwSleepTicks = osSleepTicksGet();
    if (uwSleepTicks > 1)
    {
        UINT32 uwSleepCycles, uwCurrSysCycles;
        if (uwSleepTicks >= uwMaxTicks)
        {
            uwSleepTicks = uwMaxTicks;
        }

        uwSleepCycles = uwSleepTicks * uwCyclesPerTick;
        uwCurrSysCycles = LOS_SysTickCurrCycleGet();
    #if (LOSCFG_SYSTICK_CNT_DIR_DECREASE == YES)
        LOS_SysTickReload(uwSleepCycles - uwCyclesPerTick + uwCurrSysCycles);
    #else
        LOS_SysTickReload(uwSleepCycles - uwCurrSysCycles);
    #endif
        g_uwSleepTicks = uwSleepTicks;
    #if (LOSCFG_PLATFORM_HWI == NO)
        if (g_uwSysTickIntFlag == TICKLESS_OS_TICK_INT_INIT)
        {
            g_uwSysTickIntFlag = TICKLESS_OS_TICK_INT_WAIT;
        }
    #endif
    }
out:
    LOS_SysTickStart();
    LOS_IntRestore(uvIntSave);

    return;
}

VOID osTicklessHandler(VOID)
{
#if (LOSCFG_PLATFORM_HWI == YES)

    if (g_bTickIrqFlag)
    {
        g_bTickIrqFlag = 0;
        osTicklessStart();
    }

    osEnterSleep();

#else

    if (g_bTickIrqFlag)
    {
        UINTPTR uvIntSave;
        uvIntSave = LOS_IntLock();
        LOS_TaskLock();

        g_bTickIrqFlag = 0;
        osTicklessStart();

        osEnterSleep();
        LOS_IntRestore(uvIntSave);

        /*
         * Here: Handling interrupts. However, because the task scheduler is locked,
         * there will be no task switching, after the interrupt exit, the CPU returns
         * here to continue excuting the following code.
         */

        uvIntSave = LOS_IntLock();
        osUpdateKernelTickCount(0);  /* param: 0 - invalid */
        LOS_TaskUnlock();
        LOS_IntRestore(uvIntSave);
    }
    else
    {
        /* Waiting for g_bTickIrqFlag setup, at most one tick time, sleep directly */
        osEnterSleep();
    }

#endif
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif  /* end of #if (LOSCFG_KERNEL_TICKLESS == YES) */

