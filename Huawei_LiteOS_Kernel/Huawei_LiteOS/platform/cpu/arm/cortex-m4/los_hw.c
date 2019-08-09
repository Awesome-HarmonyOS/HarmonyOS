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

#include "los_base.h"
#include "los_task.ph"
#include "los_hw.h"
#include "los_sys.ph"
#include "los_priqueue.ph"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/*****************************************************************************
 Function    : osSchedule
 Description : task scheduling
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
VOID osSchedule(VOID)
{
    osTaskSchedule();
}

/*****************************************************************************
 Function    : LOS_Schedule
 Description : Function to determine whether task scheduling is required
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
VOID LOS_Schedule(VOID)
{
    UINTPTR uvIntSave;
    uvIntSave = LOS_IntLock();

    /* Find the highest task */
    g_stLosTask.pstNewTask = LOS_DL_LIST_ENTRY(LOS_PriqueueTop(), LOS_TASK_CB, stPendList);

    /* In case that running is not highest then reschedule */
    if (g_stLosTask.pstRunTask != g_stLosTask.pstNewTask)
    {
        if ((!g_usLosTaskLock))
        {
            (VOID)LOS_IntRestore(uvIntSave);

            osTaskSchedule();

            return;
        }
    }

    (VOID)LOS_IntRestore(uvIntSave);
}

/*****************************************************************************
 Function    : osTaskExit
 Description : Task exit function
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR VOID osTaskExit(VOID)
{
    __asm ("cpsid i" : : : "memory");
    while(1);
}

/*****************************************************************************
 Function    : osTskStackInit
 Description : Task stack initialization function
 Input       : uwTaskID     --- TaskID
               uwStackSize  --- Total size of the stack
               pTopStack    --- Top of task's stack
 Output      : None
 Return      : Context pointer
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT VOID *osTskStackInit(UINT32 uwTaskID, UINT32 uwStackSize, VOID *pTopStack)
{
    UINT32 uwIdx;
    TSK_CONTEXT_S  *pstContext;

    /*initialize the task stack, write magic num to stack top*/
    for (uwIdx = 1; uwIdx < (uwStackSize/sizeof(UINT32)); uwIdx++)
    {
        *((UINT32 *)pTopStack + uwIdx) = OS_TASK_STACK_INIT;
    }
    *((UINT32 *)(pTopStack)) = OS_TASK_MAGIC_WORD;

    pstContext    = (TSK_CONTEXT_S *)(((UINT32)pTopStack + uwStackSize) - sizeof(TSK_CONTEXT_S));
#if 1
    pstContext->S16 = 0xAA000010;
    pstContext->S17 = 0xAA000011;
    pstContext->S18 = 0xAA000012;
    pstContext->S19 = 0xAA000013;
    pstContext->S20 = 0xAA000014;
    pstContext->S21 = 0xAA000015;
    pstContext->S22 = 0xAA000016;
    pstContext->S23 = 0xAA000017;
    pstContext->S24 = 0xAA000018;
    pstContext->S25 = 0xAA000019;
    pstContext->S26 = 0xAA00001A;
    pstContext->S27 = 0xAA00001B;
    pstContext->S28 = 0xAA00001C;
    pstContext->S29 = 0xAA00001D;
    pstContext->S30 = 0xAA00001E;
    pstContext->S31 = 0xAA00001F;
#endif
    pstContext->uwR4  = 0x04040404L;
    pstContext->uwR5  = 0x05050505L;
    pstContext->uwR6  = 0x06060606L;
    pstContext->uwR7  = 0x07070707L;
    pstContext->uwR8  = 0x08080808L;
    pstContext->uwR9  = 0x09090909L;
    pstContext->uwR10 = 0x10101010L;
    pstContext->uwR11 = 0x11111111L;
    pstContext->uwPriMask = 0;
    pstContext->uwR0  = uwTaskID;
    pstContext->uwR1  = 0x01010101L;
    pstContext->uwR2  = 0x02020202L;
    pstContext->uwR3  = 0x03030303L;
    pstContext->uwR12 = 0x12121212L;
    pstContext->uwLR  = (UINT32)osTaskExit;
    pstContext->uwPC  = (UINT32)osTaskEntry;
    pstContext->uwxPSR = 0x01000000L;
#if 1
    pstContext->S0 = 0xAA000000;
    pstContext->S1 = 0xAA000001;
    pstContext->S2 = 0xAA000002;
    pstContext->S3 = 0xAA000003;
    pstContext->S4 = 0xAA000004;
    pstContext->S5 = 0xAA000005;
    pstContext->S6 = 0xAA000006;
    pstContext->S7 = 0xAA000007;
    pstContext->S8 = 0xAA000008;
    pstContext->S9 = 0xAA000009;
    pstContext->S10 = 0xAA00000A;
    pstContext->S11 = 0xAA00000B;
    pstContext->S12 = 0xAA00000C;
    pstContext->S13 = 0xAA00000D;
    pstContext->S14 = 0xAA00000E;
    pstContext->S15 = 0xAA00000F;
    pstContext->FPSCR = 0x00000000;
    pstContext->NO_NAME = 0xAA000011;
#endif
    return (VOID *)pstContext;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */


