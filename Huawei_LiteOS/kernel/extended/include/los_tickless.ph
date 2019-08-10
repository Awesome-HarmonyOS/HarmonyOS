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

#ifndef _LOS_TICKLESS_PH
#define _LOS_TICKLESS_PH

#include "los_tickless.h"
#include "los_hw_tick.h"
#include "los_config.h"

#if (LOSCFG_PLATFORM_HWI == NO)
enum TICKLESS_OS_TICK_INT_FLAG
{
    TICKLESS_OS_TICK_INT_INIT = 0,
    TICKLESS_OS_TICK_INT_WAIT = 1,  /* tickless start, waiting for the next tick interrupt to happen */
    TICKLESS_OS_TICK_INT_SET  = 2,  /* tick interrupt happened */
};
extern enum TICKLESS_OS_TICK_INT_FLAG g_uwSysTickIntFlag;
#endif

extern BOOL g_bTickIrqFlag;
extern BOOL g_bReloadSysTickFlag;
extern volatile UINT32 g_uwSleepTicks;
extern BOOL g_bTicklessFlag;
extern VOID tick_timer_reload(UINT32 period);
extern VOID osSysTimeUpdate(UINT32 uwSleepTicks);
extern VOID osTicklessStart(VOID);
#if (LOSCFG_KERNEL_TICKLESS == YES)
extern inline VOID osUpdateKernelTickCount(UINT32 uwHwiIndex);
#endif
extern VOID osTicklessHandler(VOID);
#endif /* _LOS_TICKLESS_PH */
