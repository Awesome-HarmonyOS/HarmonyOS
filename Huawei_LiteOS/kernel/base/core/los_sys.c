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

#include "los_sys.inc"
#include "los_tick.ph"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/*****************************************************************************
Function   : LOS_TickCountGet
Description: get current tick
Input      : None
Output     : None
Return     : current tick
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT64 LOS_TickCountGet (VOID)
{
    return g_ullTickCount;
}

/*****************************************************************************
Function   : LOS_CyclePerTickGet
Description: Get System cycle number corresponding to each tick
Input      : None
Output     : None
Return     : cycle number corresponding to each tick
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_CyclePerTickGet(VOID)
{
    /*lint -e40*/
    return OS_SYS_CLOCK / LOSCFG_BASE_CORE_TICK_PER_SECOND;/*lint !e160*/
    /*lint +e40*/
}

/*****************************************************************************
Function   : LOS_MS2Tick
Description: milliseconds convert to Tick
Input      : milliseconds
Output     : None
Return     : Tick
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_MS2Tick(UINT32 uwMillisec)
{
    if (0xFFFFFFFF == uwMillisec)
    {
        return 0xFFFFFFFF;
    }

    return ((UINT64)uwMillisec * LOSCFG_BASE_CORE_TICK_PER_SECOND) / OS_SYS_MS_PER_SECOND;
}

/*****************************************************************************
Function   : LOS_Tick2MS
Description: Tick convert to milliseconds
Input      : TICK
Output     : None
Return     : milliseconds
*****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 LOS_Tick2MS(UINT32 uwTick)
{
    return ((UINT64)uwTick * OS_SYS_MS_PER_SECOND) / LOSCFG_BASE_CORE_TICK_PER_SECOND;
}

/*****************************************************************************
Function   : osCpuTick2MS
Description: cycle convert to milliseconds
Input      : uwInterval ---------- cycle
Output     : puwUsHi    ---------- High 32 milliseconds
             puwUsLo    ---------- Low 32 milliseconds
Return     : LOS_OK on success ,or error code on failure
*****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 osCpuTick2MS(CPU_TICK *pstCpuTick, UINT32 *puwMsHi, UINT32 *puwMsLo)
{
    UINT64 ullCpuTick;
    double dTemp;

    if ( (NULL == pstCpuTick) || (NULL == puwMsHi) || (NULL == puwMsLo) )
    {
        return LOS_ERRNO_SYS_PTR_NULL;
    }

    ullCpuTick = ((UINT64)pstCpuTick->uwCntHi << OS_SYS_MV_32_BIT) | pstCpuTick->uwCntLo;
    dTemp = ullCpuTick / (((double)OS_SYS_CLOCK) / OS_SYS_MS_PER_SECOND); /*lint !e160 !e653 !e40*/
    ullCpuTick = (UINT64)dTemp;

    *puwMsLo = (UINT32)ullCpuTick;
    *puwMsHi = (UINT32)(ullCpuTick >> OS_SYS_MV_32_BIT);

    return LOS_OK;
}

/*****************************************************************************
Function   : osCpuTick2US
Description: cycle convert to Microsecond
Input      : uwInterval ---------- cycle
Output     : puwUsHi    ---------- High 32 Microsecond
             puwUsLo    ---------- Low 32 Microsecond
Return     : LOS_OK on success ,or error code on failure
*****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 osCpuTick2US(CPU_TICK *pstCpuTick, UINT32 *puwUsHi, UINT32 *puwUsLo)
{
    UINT64 ullCpuTick;
    double dTemp;

    if ( (NULL == pstCpuTick) || (NULL == puwUsHi) || (NULL == puwUsLo) )
    {
        return LOS_ERRNO_SYS_PTR_NULL;
    }

    ullCpuTick = ((UINT64)pstCpuTick->uwCntHi << OS_SYS_MV_32_BIT) | pstCpuTick->uwCntLo;
    dTemp = ullCpuTick / (((double)OS_SYS_CLOCK) / OS_SYS_US_PER_SECOND); /*lint !e160 !e653 !e40*/
    ullCpuTick = (UINT64)dTemp;

    *puwUsLo = (UINT32)ullCpuTick;
    *puwUsHi = (UINT32)(ullCpuTick >> OS_SYS_MV_32_BIT);

    return LOS_OK;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */
