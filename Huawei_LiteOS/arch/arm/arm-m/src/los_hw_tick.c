/*----------------------------------------------------------------------------
 * Copyright (c) <2013-2018>, <Huawei Technologies Co., Ltd>
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

#include "los_tick.ph"
#include "los_base.h"
#include "los_task.ph"
#include "los_swtmr.h"
#include "los_hwi.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

/*****************************************************************************
Function   : osTickStart
Description: Configure Tick Interrupt Start
Input   : none
output  : none
return  : LOS_OK - Success , or LOS_ERRNO_TICK_CFG_INVALID - failed
*****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 osTickStart(VOID)
{
    UINT32 uwRet;

    if ((0 == OS_SYS_CLOCK)
        || (0 == LOSCFG_BASE_CORE_TICK_PER_SECOND)
        || (LOSCFG_BASE_CORE_TICK_PER_SECOND > OS_SYS_CLOCK))/*lint !e506*/
    {
        return LOS_ERRNO_TICK_CFG_INVALID;
    }

#if (LOSCFG_PLATFORM_HWI == YES)
#if (OS_HWI_WITH_ARG == YES)
    osSetVector(SysTick_IRQn, (HWI_PROC_FUNC)osTickHandler, NULL);
#else
    osSetVector(SysTick_IRQn, osTickHandler);
#endif
#endif

    g_uwCyclesPerTick = OS_SYS_CLOCK / LOSCFG_BASE_CORE_TICK_PER_SECOND;
    g_ullTickCount = 0;

    uwRet = SysTick_Config(OS_SYS_CLOCK/LOSCFG_BASE_CORE_TICK_PER_SECOND);
    if (uwRet == 1)
    {
        return LOS_ERRNO_TICK_PER_SEC_TOO_SMALL;
    }

    g_bSysTickStart = TRUE;

    return LOS_OK;
}

#if (LOSCFG_KERNEL_TICKLESS == YES)
/*****************************************************************************
Function   : LOS_SysTickReload
Description: reconfig systick
Input   : none
output  : none
return  : none
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR VOID LOS_SysTickReload(UINT32 uwCyclesPerTick)
{
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->LOAD  = (uint32_t)(uwCyclesPerTick - 1UL);               /* set reload register */
    SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}
#endif

/*****************************************************************************
Function   : LOS_SysTickCurrCycleGet
Description: Get System cycle count
Input   : none
output  : SysTick->VAL
return  : none
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_SysTickCurrCycleGet(VOID)
{
    UINT32 uwHwCycle;
    UINTPTR uvIntSave;

    uvIntSave = LOS_IntLock();
    uwHwCycle = SysTick->VAL;

    /*tick has come, but may interrupt environment, not counting the Tick interrupt response, to do +1 */
    if (((SCB->ICSR & 0x4000000) != 0))
    {
        uwHwCycle = SysTick->VAL;
        uwHwCycle += g_uwCyclesPerTick;
    }

    LOS_IntRestore(uvIntSave);

    return uwHwCycle;
}

/*****************************************************************************
Function   : LOS_GetCpuCycle
Description: Get System cycle count
Input   : none
output  : puwCntHi  --- CpuTick High 4 byte
          puwCntLo  --- CpuTick Low 4 byte
return  : none
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR VOID LOS_GetCpuCycle(UINT32 *puwCntHi, UINT32 *puwCntLo)
{
    UINT64 ullSwTick;
    UINT64 ullCycle;
    UINT32 uwHwCycle;
    UINTPTR uvIntSave;

    uvIntSave = LOS_IntLock();

    ullSwTick = g_ullTickCount;
    uwHwCycle = SysTick->VAL;

    /*tick has come, but may interrupt environment, not counting the Tick interrupt response, to do +1 */
    if (((SCB->ICSR & 0x4000000) != 0))
    {
        uwHwCycle = SysTick->VAL;
        ullSwTick++;
    }

    ullCycle = (((ullSwTick) * g_uwCyclesPerTick) + (g_uwCyclesPerTick - uwHwCycle));

    *puwCntHi = ullCycle >> 32;
    *puwCntLo = ullCycle & 0xFFFFFFFFU;

    LOS_IntRestore(uvIntSave);

    return;
}

/*****************************************************************************
Function   : LOS_GetSystickCycle
Description: Get Sys tick cycle count
Input   : none
output  : puwCntHi  --- SysTick count High 4 byte
          puwCntLo  --- SysTick count Low 4 byte
return  : none
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR VOID LOS_GetSystickCycle(UINT32 *puwCntHi, UINT32 *puwCntLo)
{
    UINT64 ullSwTick;
    UINT64 ullCycle;
    UINT32 uwHwCycle;
    UINTPTR uvIntSave;
    UINT32 uwSystickLoad;
    UINT32 uwSystickCur;

    uvIntSave = LOS_IntLock();

    ullSwTick = g_ullTickCount;

    uwSystickLoad = SysTick->LOAD;
    uwSystickCur = SysTick->VAL;
    uwHwCycle = uwSystickLoad - uwSystickCur;

    /*tick has come, but may interrupt environment, not counting the Tick interrupt response, to do +1 */
    if (((SCB->ICSR & 0x4000000) != 0))
    {
        uwHwCycle = uwSystickLoad - uwSystickCur;
        ullSwTick++;
    }

    ullCycle = uwHwCycle + ullSwTick * uwSystickLoad;
    *puwCntHi = ullCycle >> 32;
    *puwCntLo = ullCycle & 0xFFFFFFFFU;

    LOS_IntRestore(uvIntSave);

    return;
}
#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */
