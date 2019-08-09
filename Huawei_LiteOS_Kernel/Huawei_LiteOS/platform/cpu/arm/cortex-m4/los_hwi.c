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

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/*lint -save -e40 -e522 -e533*/

/*lint -restore*/
UINT32  g_vuwIntCount = 0;

#pragma  location = ".vector"
LITE_OS_SEC_VEC HWI_PROC_FUNC m_pstHwiForm[OS_M4_VECTOR_CNT] =
{
  0,                    // [0] Top of Stack
  Reset_Handler,        // [1] reset
  osHwiDefaultHandler,  // [2] NMI Handler
  osHwiDefaultHandler,  // [3] Hard Fault Handler
  osHwiDefaultHandler,  // [4] MPU Fault Handler
  osHwiDefaultHandler,  // [5] Bus Fault Handler
  osHwiDefaultHandler,  // [6] Usage Fault Handler
  0,                    // [7] Reserved
  0,                    // [8] Reserved
  0,                    // [9] Reserved
  0,                    // [10] Reserved
  osHwiDefaultHandler,  // [11] SVCall Handler
  osHwiDefaultHandler,  // [12] Debug Monitor Handler
  0,                    // [13] Reserved
  osPendSV,             // [14] PendSV Handler
  osHwiDefaultHandler,  // [15] SysTick Handler
};
HWI_PROC_FUNC m_pstHwiSlaveForm[OS_M4_VECTOR_CNT] = {0};

/*****************************************************************************
 Function    : osIntNumGet
 Description : Get a interrupt number
 Input       : None
 Output      : None
 Return      : Interrupt Indexes number
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR UINT32 osIntNumGet(VOID)
{
   UINT32 uwIntNum;

  __asm ("mrs %0, ipsr" : "=r" (uwIntNum));
  return uwIntNum;
}

/*****************************************************************************
 Function    : osHwiDefaultHandler
 Description : default handler of the hardware interrupt
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT_MINOR VOID  osHwiDefaultHandler(VOID)
{
    UINT32 irq_num = osIntNumGet();
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

    uwIntSave = LOS_IntLock();
    g_vuwIntCount++;
    LOS_IntRestore(uwIntSave);

    uwHwiIndex = osIntNumGet();

    if (m_pstHwiSlaveForm[uwHwiIndex] !=0)
    {
        m_pstHwiSlaveForm[uwHwiIndex]();
    }

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
    for(uwIndex = OS_M4_SYS_VECTOR_CNT; uwIndex < OS_M4_VECTOR_CNT; uwIndex++)
    {
        m_pstHwiForm[uwIndex] = osHwiDefaultHandler;
    }

    /* Interrupt vector table location */
     *(volatile UINT32 *)OS_NVIC_VTOR =  (UINT32)m_pstHwiForm;

     *(volatile UINT32 *)OS_NVIC_AIRCR = (0x05FA0000 | OS_NVIC_AIRCR_PRIGROUP << 8);
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
    if (uwHwiNum >= OS_M4_IRQ_VECTOR_CNT)
    {
        return OS_ERRNO_HWI_NUM_INVALID;
    }
    if (m_pstHwiForm[uwHwiNum + OS_M4_SYS_VECTOR_CNT] != osHwiDefaultHandler)
    {
        return OS_ERRNO_HWI_ALREADY_CREATED;
    }
    if (usHwiPrio > OS_HWI_PRIO_LOWEST)
    {
        return OS_ERRNO_HWI_PRIO_INVALID;
    }

    uvIntSave = LOS_IntLock();

    osSetVector(uwHwiNum, pfnHandler);

    nvicSetIRQ(uwHwiNum);

    nvicSetIrqPRI(uwHwiNum, usHwiPrio << 4);

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

    if (uwHwiNum >= OS_M4_IRQ_VECTOR_CNT)
    {
        return OS_ERRNO_HWI_NUM_INVALID;
    }

    nvicClrIRQ(uwHwiNum);

    uwIntSave = LOS_IntLock();

    m_pstHwiForm[uwHwiNum + OS_M4_SYS_VECTOR_CNT] = (HWI_PROC_FUNC)osHwiDefaultHandler;

    LOS_IntRestore(uwIntSave);

    return LOS_OK;
}


#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */


