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

#include "los_sys.ph"
#include "los_task.ph"
#include "los_tick.ph"
#include "los_typedef.ph"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

#if(LOSCFG_BASE_CORE_TIMESLICE == YES)
LITE_OS_SEC_BSS OS_TASK_ROBIN_S        g_stTaskTimeSlice;

/*****************************************************************************
 Function     : osTimesliceInit
 Description  : Initialztion Timeslice
 Input        : None
 Output       : None
 Return       : None
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT VOID osTimesliceInit(VOID)
{
    g_stTaskTimeSlice.pstTask = (LOS_TASK_CB *)NULL;
    g_stTaskTimeSlice.usTout = LOSCFG_BASE_CORE_TIMESLICE_TIMEOUT;
}

/*****************************************************************************
 Function     : osTimesliceCheck
 Description  : check Timeslice
 Input        : None
 Output       : None
 Return       : None
 *****************************************************************************/
LITE_OS_SEC_TEXT VOID osTimesliceCheck(VOID)
{
    if (g_stTaskTimeSlice.pstTask != g_stLosTask.pstRunTask)
    {
        g_stTaskTimeSlice.pstTask = g_stLosTask.pstRunTask;
        g_stTaskTimeSlice.usTime = (UINT16)g_ullTickCount + g_stTaskTimeSlice.usTout - 1;
    }

    if (g_stTaskTimeSlice.usTime == (UINT16)g_ullTickCount)
    {
        g_stTaskTimeSlice.pstTask = (LOS_TASK_CB *)NULL;
        if (LOS_TaskYield() != LOS_OK)
        {
            PRINT_INFO("%s, %d\n", __FUNCTION__, __LINE__);
        }
    } /*lint !e548*/
}

#endif

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

