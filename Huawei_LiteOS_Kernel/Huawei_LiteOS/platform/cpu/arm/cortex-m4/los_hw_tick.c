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

LITE_OS_SEC_BSS UINT32  g_uwCyclesPerTick;

/*lint -save -e40 -e10 -e26 -e1013*/
/*****************************************************************************
Function   : osTickStart
Description: Configure Tick Interrupt Start
Input   : none
output  : none
return  : LOS_OK - Success , or LOS_ERRNO_TICK_CFG_INVALID - failed
*****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 osTickStart(VOID)
{
    if ((0 == OS_SYS_CLOCK)
        || (0 == LOSCFG_BASE_CORE_TICK_PER_SECOND)
        || (LOSCFG_BASE_CORE_TICK_PER_SECOND > OS_SYS_CLOCK))
    {
        return LOS_ERRNO_TICK_CFG_INVALID;
    }

    m_pstHwiForm[OS_EXC_SYS_TICK] = (HWI_PROC_FUNC)osInterrupt;
    m_pstHwiSlaveForm[OS_EXC_SYS_TICK] = osTickHandler;

    g_uwCyclesPerTick = OS_SYS_CLOCK / LOSCFG_BASE_CORE_TICK_PER_SECOND;
    g_ullTickCount = 0;

    *(volatile UINT32 *)OS_SYSTICK_RELOAD_REG = OS_SYS_CLOCK/LOSCFG_BASE_CORE_TICK_PER_SECOND - 1;
    *((volatile UINT8 *)OS_NVIC_EXCPRI_BASE + (((UINT32)(-1) & 0xF) - 4)) = ((7 << 4) & 0xff);
    *(volatile UINT32 *)OS_SYSTICK_CURRENT_REG = 0;
    *(volatile UINT32 *)OS_SYSTICK_CONTROL_REG = (1 << 2) | (1 << 1) | (1 << 0);

    return LOS_OK;
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
    UINT32 uwIntSta;
    UINT32 uwHwCycle;
    UINTPTR uwIntSave;

    uwIntSave = LOS_IntLock();

    ullSwTick = g_ullTickCount;

    uwHwCycle = *(volatile UINT32*)OS_SYSTICK_CURRENT_REG;
    uwIntSta  = *(volatile UINT32*)OS_NVIC_INT_CTRL;

    /*tick has come, but may interrupt environment, not counting the Tick interrupt response, to do +1 */
    if (((uwIntSta & 0x4000000) != 0))
    {
        uwHwCycle = *(volatile UINT32*)OS_SYSTICK_CURRENT_REG;
        ullSwTick++;
    }

    ullCycle = (((ullSwTick) * g_uwCyclesPerTick) + (g_uwCyclesPerTick - uwHwCycle));
    *puwCntHi = ullCycle >> 32;
    *puwCntLo = ullCycle & 0xFFFFFFFFU;

    LOS_IntRestore(uwIntSave);

    return;
}
#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */