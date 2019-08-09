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

#include "los_sys.h"
#include "los_tick.h"
#include "los_task.ph"
#include "los_config.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#pragma data_alignment=8
UINT8 *m_aucSysMem0;
UINT32 g_sys_mem_addr_end = 0;
extern UINT8 g_ucMemStart[];
extern UINT32 osTickInit(UINT32 uwSystemClock, UINT32 uwTickPerSecond);
extern UINT32   g_uwTskMaxNum;

void osEnableFPU(void)
{
    *(volatile UINT32 *)0xE000ED88 |= ((3UL << 10*2)|(3UL << 11*2));
    //SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
}
/*****************************************************************************
 Function    : osRegister
 Description : Configuring the maximum number of tasks
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT VOID osRegister(VOID)
{
    g_uwTskMaxNum = LOSCFG_BASE_CORE_TSK_LIMIT + 1; /* Reserved 1 for IDLE */
    g_sys_mem_addr_end = (UINT32)g_ucMemStart + OS_SYS_MEM_SIZE;
    return;
}

/*****************************************************************************
 Function    : LOS_Start
 Description : Task start function
 Input       : None
 Output      : None
 Return      : LOS_OK
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_Start()
{
    UINT32 uwRet;
#if (LOSCFG_BASE_CORE_TICK_HW_TIME == NO)
    uwRet = osTickStart();

    if (uwRet != LOS_OK)
    {
        PRINT_ERR("osTickStart error\n");
        return uwRet;
    }
#else
    os_timer_init();
#endif
    LOS_StartToRun();

    return uwRet;
}

/*****************************************************************************
 Function    : osMain
 Description : System kernel initialization function, configure all system modules
 Input       : None
 Output      : None
 Return      : LOS_OK
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT int osMain(void)
{
    UINT32 uwRet;

    osRegister();

    uwRet = osMemSystemInit();
    if (uwRet != LOS_OK)
    {
        PRINT_ERR("osMemSystemInit error %d\n", uwRet);
        return uwRet;
    }

#if (LOSCFG_PLATFORM_HWI == YES)
    {
        osHwiInit();
    }
#endif

    uwRet =osTaskInit();
    if (uwRet != LOS_OK)
    {
        PRINT_ERR("osTaskInit error\n");
        return uwRet;
    }

#if (LOSCFG_BASE_IPC_SEM == YES)
    {
        uwRet = osSemInit();
        if (uwRet != LOS_OK)
        {
            return uwRet;
        }
    }
#endif

#if (LOSCFG_BASE_IPC_MUX == YES)
    {
        uwRet = osMuxInit();
        if (uwRet != LOS_OK)
        {
            return uwRet;
        }
    }
#endif

#if (LOSCFG_BASE_IPC_QUEUE == YES)
    {
        uwRet = osQueueInit();
        if (uwRet != LOS_OK)
        {
            PRINT_ERR("osQueueInit error\n");
            return uwRet;
        }
    }
#endif

#if (LOSCFG_BASE_CORE_SWTMR == YES)
    {
        uwRet = osSwTmrInit();
        if (uwRet != LOS_OK)
        {
            PRINT_ERR("osSwTmrInit error\n");
            return uwRet;
        }
    }
#endif

    #if(LOSCFG_BASE_CORE_TIMESLICE == YES)
    osTimesliceInit();
    #endif

    uwRet = osIdleTaskCreate();
    if (uwRet != LOS_OK) {
        return uwRet;
    }

    return LOS_OK;
}


/*****************************************************************************
 Function    : main
 Description : Main function entry
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT
int main(void)
{
    UINT32 uwRet;
    uwRet = osMain();
    if (uwRet != LOS_OK) {
        return LOS_NOK;
    }

    LOS_Start();

    for (;;);
    /* Replace the dots (...) with your own code.  */
}

void osBackTrace(){}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */
