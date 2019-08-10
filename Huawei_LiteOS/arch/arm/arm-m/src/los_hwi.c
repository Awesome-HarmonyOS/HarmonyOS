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

#include "los_hwi.h"
#if (LOSCFG_KERNEL_TICKLESS == YES)
#include "los_tickless.ph"
#endif
#if (LOSCFG_PLATFORM_HWI == NO)
#include "los_tick.ph"
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

#if (LOSCFG_PLATFORM_HWI == YES)

/*lint -save -e40 -e522 -e533*/

LITE_OS_SEC_DATA_INIT UINT32  g_vuwIntCount = 0;
/*lint -restore*/
#ifdef __ICCARM__
#pragma  location = ".data.vector"
#elif defined (__CC_ARM) || defined (__GNUC__)
LITE_OS_SEC_VEC
#endif

HWI_PROC_FUNC m_pstHwiForm[OS_VECTOR_CNT] =
{
    (HWI_PROC_FUNC)0,                    // [0] Top of Stack
    (HWI_PROC_FUNC)Reset_Handler,        // [1] reset
    (HWI_PROC_FUNC)osHwiDefaultHandler,  // [2] NMI Handler
    (HWI_PROC_FUNC)osHwiDefaultHandler,  // [3] Hard Fault Handler
    (HWI_PROC_FUNC)osHwiDefaultHandler,  // [4] MPU Fault Handler
    (HWI_PROC_FUNC)osHwiDefaultHandler,  // [5] Bus Fault Handler
    (HWI_PROC_FUNC)osHwiDefaultHandler,  // [6] Usage Fault Handler
    (HWI_PROC_FUNC)0,                    // [7] Reserved
    (HWI_PROC_FUNC)0,                    // [8] Reserved
    (HWI_PROC_FUNC)0,                    // [9] Reserved
    (HWI_PROC_FUNC)0,                    // [10] Reserved
    (HWI_PROC_FUNC)osHwiDefaultHandler,  // [11] SVCall Handler
    (HWI_PROC_FUNC)osHwiDefaultHandler,  // [12] Debug Monitor Handler
    (HWI_PROC_FUNC)0,                    // [13] Reserved
    (HWI_PROC_FUNC)PendSV_Handler,       // [14] PendSV Handler
    (HWI_PROC_FUNC)osHwiDefaultHandler,  // [15] SysTick Handler
};
#if (OS_HWI_WITH_ARG == YES)
LITE_OS_SEC_DATA_INIT HWI_SLAVE_FUNC m_pstHwiSlaveForm[OS_VECTOR_CNT] = {{(HWI_PROC_FUNC)0,(HWI_ARG_T)0}};
#else
LITE_OS_SEC_DATA_INIT HWI_PROC_FUNC m_pstHwiSlaveForm[OS_VECTOR_CNT] = {0};
#endif

#endif /*(LOSCFG_PLATFORM_HWI == YES)*/

/*****************************************************************************
 Function    : osIntNumGet
 Description : Get a interrupt number
 Input       : None
 Output      : None
 Return      : Interrupt Indexes number
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 osIntNumGet(VOID)
{
    return __get_IPSR();
}

#if (LOSCFG_PLATFORM_HWI == YES)
/*****************************************************************************
 Function    : osHwiDefaultHandler
 Description : default handler of the hardware interrupt
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
/*lint -e529*/
LITE_OS_SEC_TEXT_MINOR VOID  osHwiDefaultHandler(VOID)
{
    UINT32 uwIrqNum = osIntNumGet();
    PRINT_ERR("%s irqnum:%d\n", __FUNCTION__, uwIrqNum);
    while(1);
}

/*****************************************************************************
 Function    : osInterrupt
 Description : Hardware interrupt entry function
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT VOID  osInterrupt(VOID)
{
    UINT32 uwHwiIndex;
    UINT32 uwIntSave;

#if(LOSCFG_KERNEL_RUNSTOP == YES)
    SCB->SCR &= (UINT32)~((UINT32)SCB_SCR_SLEEPDEEP_Msk);
#endif

    uwIntSave = LOS_IntLock();

    g_vuwIntCount++;

    LOS_IntRestore(uwIntSave);

    uwHwiIndex = osIntNumGet();
#if (LOSCFG_KERNEL_TICKLESS == YES)
    osUpdateKernelTickCount(uwHwiIndex);
#endif

#if (OS_HWI_WITH_ARG == YES)
    if (m_pstHwiSlaveForm[uwHwiIndex].pfnHandler!=0)
    {
        m_pstHwiSlaveForm[uwHwiIndex].pfnHandler((VOID*)m_pstHwiSlaveForm[uwHwiIndex].pParm);
    }
#else
    if (m_pstHwiSlaveForm[uwHwiIndex] !=0)
    {
        m_pstHwiSlaveForm[uwHwiIndex]();
    }
#endif
    uwIntSave = LOS_IntLock();

    g_vuwIntCount--;

    LOS_IntRestore(uwIntSave);

}
/*****************************************************************************
 Function    : osHwiInit
 Description : initialization of the hardware interrupt
 Input       : None
 Output      : None
 Return      : OS_SUCCESS
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT VOID osHwiInit()
{
    UINT32 uwIndex;

    for(uwIndex = OS_SYS_VECTOR_CNT; uwIndex < OS_VECTOR_CNT; uwIndex++)
    {
        m_pstHwiForm[uwIndex] = (HWI_PROC_FUNC)osHwiDefaultHandler;
    }

    /* Interrupt vector table location */
    SCB->VTOR = (UINT32)m_pstHwiForm;
#if (__CORTEX_M >= 0x03U)  /* only for Cortex-M3 and above */
    NVIC_SetPriorityGrouping(OS_NVIC_AIRCR_PRIGROUP);
#endif

    return;
}
/*****************************************************************************
 Function    : LOS_HwiCreate
 Description : create hardware interrupt
 Input       : uwHwiNum   --- hwi num to create
               usHwiPrio  --- priority of the hwi
               usMode     --- unused
               pfnHandler --- hwi handler
               uwArg      --- param of the hwi handler
 Output      : None
 Return      : OS_SUCCESS on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_HwiCreate( HWI_HANDLE_T  uwHwiNum,
                                      HWI_PRIOR_T   usHwiPrio,
                                      HWI_MODE_T    usMode,
                                      HWI_PROC_FUNC pfnHandler,
                                      HWI_ARG_T     uwArg )
{
    UINTPTR uvIntSave;

    if (NULL == pfnHandler)
    {
        return OS_ERRNO_HWI_PROC_FUNC_NULL;
    }

    if (uwHwiNum >= OS_HWI_MAX_NUM)
    {
        return OS_ERRNO_HWI_NUM_INVALID;
    }

    if (m_pstHwiForm[uwHwiNum + OS_SYS_VECTOR_CNT] != (HWI_PROC_FUNC)osHwiDefaultHandler)
    {
        return OS_ERRNO_HWI_ALREADY_CREATED;
    }

    if ((usHwiPrio > OS_HWI_PRIO_LOWEST) || (usHwiPrio < OS_HWI_PRIO_HIGHEST))
    {
        return OS_ERRNO_HWI_PRIO_INVALID;
    }

    uvIntSave = LOS_IntLock();
#if (OS_HWI_WITH_ARG == YES)
    osSetVector(uwHwiNum, pfnHandler, uwArg);
#else
    osSetVector(uwHwiNum, pfnHandler);
#endif
    NVIC_EnableIRQ((IRQn_Type)uwHwiNum);
    NVIC_SetPriority((IRQn_Type)uwHwiNum, usHwiPrio);

    LOS_IntRestore(uvIntSave);

    return LOS_OK;

}

/*****************************************************************************
 Function    : LOS_HwiDelete
 Description : Delete hardware interrupt
 Input       : uwHwiNum   --- hwi num to delete
 Output      : None
 Return      : LOS_OK on success or error code on failure
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT UINT32 LOS_HwiDelete(HWI_HANDLE_T uwHwiNum)
{
    UINT32 uwIntSave;

    if (uwHwiNum >= OS_HWI_MAX_NUM)
    {
        return OS_ERRNO_HWI_NUM_INVALID;
    }

    NVIC_DisableIRQ((IRQn_Type)uwHwiNum);

    uwIntSave = LOS_IntLock();

    m_pstHwiForm[uwHwiNum + OS_SYS_VECTOR_CNT] = (HWI_PROC_FUNC)osHwiDefaultHandler;

    LOS_IntRestore(uwIntSave);

    return LOS_OK;
}

#else

/*****************************************************************************
 Function    : SysTick_Handler
 Description : This function handles SysTick exception, Call LiteOS interface
               osTickHandler.
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
void SysTick_Handler(void)
{
    if (g_bSysTickStart)
    {
        osTickHandler();
    }
    else
    {
        g_ullTickCount++;
    }
}

#endif /*(LOSCFG_PLATFORM_HWI == YES)*/

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */


