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
#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#include "los_demo_entry.h"
#include "los_task.h"
#include <string.h>

static UINT32 g_uwDemoTaskID;

#ifdef LOS_KERNEL_DEBUG_OUT
/* print via SWO */
#ifdef LOS_KERNEL_TEST_KEIL_SWSIMU

#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

struct __FILE
{
    INT32 handle; /* Add whatever needed */
};

FILE __stdout;
FILE __stdin;

#if defined ( __ARMCC_VERSION ) || defined ( __ICCARM__ )  /* KEIL and IAR: printf will call fputc to print */
INT32 fputc(INT32 ch, FILE *f)
{
    if (DEMCR & TRCENA)
    {
        while (ITM_Port32(0) == 0);
        ITM_Port8(0) = ch;
    }
    return(ch);
}
#elif defined ( __GNUC__ )  /* GCC: printf will call _write to print */
__attribute__((used)) INT32 _write(INT32 fd, CHAR *ptr, INT32 len)
{
    INT32 i;
    for (i = 0; i < len; i++)
    {
        if (DEMCR & TRCENA)
        {
            while (ITM_Port32(0) == 0);
            ITM_Port8(0) = ptr[i];
        }
    }
    return len;
}
#endif
#endif

#else

INT32 dprintf_none(const CHAR *format,...)
{
    return 0;
}
#endif /* #ifdef LOS_KERNEL_DEBUG_OUT */

static LITE_OS_SEC_TEXT VOID LOS_Demo_Tskfunc(VOID)
{
#ifdef LOS_KERNEL_TEST_ALL
#else /* LOS_KERNEL_TEST_ALL */

/* only test some function */
#ifdef LOS_KERNEL_TEST_TASK
    Example_TskCaseEntry();
#endif
#ifdef LOS_KERNEL_TEST_MEM_DYNAMIC
     Example_Dyn_Mem();
#endif
#ifdef LOS_KERNEL_TEST_MEM_STATIC
    Example_StaticMem();
#endif
#ifdef LOS_KERNEL_TEST_INTRRUPT
    Example_Interrupt();
#endif
#ifdef LOS_KERNEL_TEST_QUEUE
    Example_MsgQueue();
#endif
#ifdef LOS_KERNEL_TEST_EVENT
    Example_SndRcvEvent();
#endif
#ifdef LOS_KERNEL_TEST_MUTEX
    Example_MutexLock();
#endif
#ifdef LOS_KERNEL_TEST_SEMPHORE
    Example_Semphore();
#endif
#ifdef LOS_KERNEL_TEST_SYSTICK
    Example_GetTick();
#endif
#ifdef LOS_KERNEL_TEST_SWTIMER
    Example_swTimer();
#endif
#ifdef LOS_KERNEL_TEST_LIST
    Example_list();
#endif
#endif/* LOS_KERNEL_TEST_ALL */

    while (1)
    {
        (VOID)LOS_TaskDelay(100);
    }
}

VOID LOS_Demo_Entry(VOID)
{
    UINT32 uwRet;
    TSK_INIT_PARAM_S stTaskInitParam;

    (VOID)memset((VOID *)(&stTaskInitParam), 0, sizeof(TSK_INIT_PARAM_S));
    stTaskInitParam.pfnTaskEntry = (TSK_ENTRY_FUNC)LOS_Demo_Tskfunc;
    stTaskInitParam.uwStackSize = LOSCFG_BASE_CORE_TSK_DEFAULT_STACK_SIZE;
    stTaskInitParam.pcName = "ApiDemo";
    stTaskInitParam.usTaskPrio = 30;
    uwRet = LOS_TaskCreate(&g_uwDemoTaskID, &stTaskInitParam);

    if (uwRet != LOS_OK)
    {
        dprintf("Api demo test task create failed \r\n");
        return;
    }
    return;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */
