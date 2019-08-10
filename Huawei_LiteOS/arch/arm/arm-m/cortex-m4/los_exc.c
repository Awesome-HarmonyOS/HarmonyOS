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


#include "los_exc.inc"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#if (LOSCFG_PLATFORM_EXC == YES)

UINT32 g_uwCurNestCount = 0;
EXC_INFO_S m_stExcInfo;
/*
 * CFSR register, include UFSR,BFSR and MFSR.
 *
 * +----------------------------------------------------------------------------------+
 * |                                      UFSR                                        |
 * |-----------+-----------+-----------+-------+------+-------+----------+------------|
 * |  31 - 26  |     25    |     24    | 23-20 |  19  |   18  |    17    |     16     |
 * |-----------+-----------+-----------+-------+------+-------+----------+------------|
 * |    ---    | DIVBYZERO | UNALIGNED |  ---  | NOCP | INVPC | INVSTATE | UNDEFINSTR |
 * +----------------------------------------------------------------------------------+
 *
 * +----------------------------------------------------------------------------------+
 * |                                     BFSR                                         |
 * |-----------+-----+--------+--------+----------+-------------+-----------+---------|
 * |     15    |  14 |   13   |   12   |    11    |      10     |     9     |    8    |
 * |-----------+-----+--------+--------+----------+-------------+-----------+---------|
 * | BFARVALID | --- | LSPERR | STKERR | UNSTKERR | IMPRECISERR | PRECISERR | IBUSERR |
 * +----------------------------------------------------------------------------------+
 *
 * +----------------------------------------------------------------------------------+
 * |                                     MFSR                                         |
 * |-----------+-------+---------+---------+-----------+-------+----------+-----------|
 * |     7     |   6   |    5    |    4    |     3     |   2   |    1     |     0     |
 * |-----------+-------+---------+---------+-----------+-------+----------+-----------|
 * | MMARVALID |  ---  | MLSPERR | MSTKERR | MUNSTKERR |  ---  | DACCVIOL | ICACCVIOL |
 * +----------------------------------------------------------------------------------+
 */
UINT8 g_uwExcTbl[32] =
{
    0, 0, 0, 0, 0, 0, OS_EXC_UF_DIVBYZERO, OS_EXC_UF_UNALIGNED,
    0, 0, 0, 0, OS_EXC_UF_NOCP, OS_EXC_UF_INVPC, OS_EXC_UF_INVSTATE, OS_EXC_UF_UNDEFINSTR,
    0, 0, OS_EXC_BF_LSPERR, OS_EXC_BF_STKERR, OS_EXC_BF_UNSTKERR, OS_EXC_BF_IMPRECISERR, OS_EXC_BF_PRECISERR, OS_EXC_BF_IBUSERR,
    0, 0, OS_EXC_MF_MLSPERR, OS_EXC_MF_MSTKERR, OS_EXC_MF_MUNSTKERR, 0, OS_EXC_MF_DACCVIOL, OS_EXC_MF_IACCVIOL
};

#if (LOSCFG_SAVE_EXC_INFO == YES)
VOID *m_puwExcContent;
UINT32 g_uwArraySize = 0;
EXC_INFO_ARRAY_S m_stExcArray[OS_EXC_TYPE_MAX - 1];
static VOID osExcSave2DDR(VOID);
#endif

extern VOID LOS_Reboot(VOID);

/*****************************************************************************
 Function    : osExcInfoDisplay
 Description : EXC info display
 Input       : pstExc --- Pointer to the EXC data
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT VOID osExcInfoDisplay(EXC_INFO_S *pstExc)
{
    PRINT_ERR("Phase      = 0x%x\n", pstExc->usPhase);
    PRINT_ERR("Type       = 0x%x\n", pstExc->usType);
    PRINT_ERR("FaultAddr  = 0x%x\n", pstExc->uwFaultAddr);
    PRINT_ERR("ThrdPid    = 0x%x\n", pstExc->uwThrdPid);
    PRINT_ERR("R0         = 0x%x\n", pstExc->pstContext->uwR0);
    PRINT_ERR("R1         = 0x%x\n", pstExc->pstContext->uwR1);
    PRINT_ERR("R2         = 0x%x\n", pstExc->pstContext->uwR2);
    PRINT_ERR("R3         = 0x%x\n", pstExc->pstContext->uwR3);
    PRINT_ERR("R4         = 0x%x\n", pstExc->pstContext->uwR4);
    PRINT_ERR("R5         = 0x%x\n", pstExc->pstContext->uwR5);
    PRINT_ERR("R6         = 0x%x\n", pstExc->pstContext->uwR6);
    PRINT_ERR("R7         = 0x%x\n", pstExc->pstContext->uwR7);
    PRINT_ERR("R8         = 0x%x\n", pstExc->pstContext->uwR8);
    PRINT_ERR("R9         = 0x%x\n", pstExc->pstContext->uwR9);
    PRINT_ERR("R10        = 0x%x\n", pstExc->pstContext->uwR10);
    PRINT_ERR("R11        = 0x%x\n", pstExc->pstContext->uwR11);
    PRINT_ERR("R12        = 0x%x\n", pstExc->pstContext->uwR12);
    PRINT_ERR("PriMask    = 0x%x\n", pstExc->pstContext->uwPriMask);
    PRINT_ERR("SP         = 0x%x\n", pstExc->pstContext->uwSP);
    PRINT_ERR("LR         = 0x%x\n", pstExc->pstContext->uwLR);
    PRINT_ERR("PC         = 0x%x\n", pstExc->pstContext->uwPC);
    PRINT_ERR("xPSR       = 0x%x\n", pstExc->pstContext->uwxPSR);

    PRINT_ERR("\nplease use the addr2line tool to analyze the call stack on PC:\n");
    PRINT_ERR("addr2line -e (xxx.axf/xxx.elf/xxx.out) -a -f ");
    for (UINT32 i = 0; i < pstExc->uwCallStackDepth; i++)
    {
        PRINT_ERR("%#x ", pstExc->uwCallStack[i]);
    }

    return;
}

/*****************************************************************************
 Function    : osExcCallStackAnalysis
 Description : Call stack analysis
 Input       : pstExc  ---  point to exception info
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT VOID osExcCallStackAnalysis(EXC_INFO_S *pstExc)
{
    UINT32 uwSP;
    UINT32 uwLR;
    UINT32 uwPC;
    UINT32 uwStackStartAddr;
    UINT32 uwStackSize;
    UINT32 uwDepth = 0;
    BOOL   bFirstLrValid = FALSE;

    uwSP = pstExc->pstContext->uwSP;  /* sp pointer before entering exception */

    /*
     * save first and second depth
     * first: PC before entering exception
     * second: (LR - 4) before entering exception
     * NOTE: If an exception occurs in the interrupt, LR may be EXC_RETURN, so we must make sure
     *       that LR is valid, exclude EXC_RETURN.
     */
    pstExc->uwCallStack[uwDepth++] = pstExc->pstContext->uwPC;
    if ((pstExc->pstContext->uwLR >= LOSCFG_EXC_CODE_START_ADDR) && \
        (pstExc->pstContext->uwLR <= LOSCFG_EXC_CODE_START_ADDR + LOSCFG_EXC_CODE_SIZE))
    {
        pstExc->uwCallStack[uwDepth++] = pstExc->pstContext->uwLR - sizeof(VOID *);  /* lr = pc + 4 */
        bFirstLrValid = TRUE;
    }

    /*
     * get the start address and size of the stack before entering the exception
     */
    if (pstExc->usPhase == OS_EXC_IN_TASK)  /* task use PSP */
    {
        uwStackStartAddr = g_stLosTask.pstRunTask->uwTopOfStack;
        uwStackSize = g_stLosTask.pstRunTask->uwStackSize;
    }
    else  /* init and interrupt use MSP */
    {
        uwStackStartAddr = LOSCFG_EXC_MSP_START_ADDR;
        uwStackSize = LOSCFG_EXC_MSP_SIZE;
    }

    /*
     * check stack overflow, if so, compensate for overflow, readjust stack start address and size.
     */
    if (uwSP < uwStackStartAddr)  /* stack top overflow */
    {
        if (pstExc->usPhase == OS_EXC_IN_TASK)
        {
            PRINT_ERR("task %s stack top overflow\n", g_stLosTask.pstRunTask->pcTaskName);
        }
        else
        {
            PRINT_ERR("MSP top overflow\n");
        }
        uwStackSize += (uwStackStartAddr - uwSP);  /* compensate overflow size */
        uwStackStartAddr = uwSP;  /* readjust stack start address */
    }

    /*
     * Traverses the entire stack from the top of the stack to the bottom of the stack,
     * find all LR.
     */
    for (; uwSP < uwStackStartAddr + uwStackSize; uwSP += sizeof(UINT32))
    {
        uwLR = *(UINT32 *)uwSP;
        if (uwLR % 2 == 0)  /* Thumb instruction, LR bit0 == 1 */
        {
            continue;
        }

        /*
         * It may be LR, which must be further determined based on the start address
         * and end address of the code segment.
         */
        if ((uwLR >= LOSCFG_EXC_CODE_START_ADDR) && (uwLR <= LOSCFG_EXC_CODE_START_ADDR + LOSCFG_EXC_CODE_SIZE) \
            && (uwDepth < LOSCFG_EXC_CALL_STACK_ANALYSIS_MAX_DEPTH))
        {
            uwPC = uwLR - sizeof(VOID *);  /* lr = pc + 4 */
            /* the second depth(first LR) has been saved */
            if ((uwDepth == 2) && (uwPC == pstExc->uwCallStack[1]) && (bFirstLrValid == TRUE))
            {
                continue;
            }
            pstExc->uwCallStack[uwDepth++] = uwPC;
        }
    }
    pstExc->uwCallStackDepth = uwDepth;  /* save call stack depth */

    return;
}

/*****************************************************************************
 Function    : osExcHandleEntry
 Description : EXC handler entry
 Input       : uwExcType     --- EXC type
             : uwFaultAddr   --- The address of an accurate address access error
             : uwPid         --- Interrupt number
             : puwExcBufAddr --- Point to the stack frame without FPU
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT VOID osExcHandleEntry(UINT32 uwExcType, UINT32 uwFaultAddr, UINT32 uwPid, EXC_CONTEXT_S *pstExcBufAddr)
{
    /* uwExcType high 16 bits: 0x01 - uwFaultAddr valid, 0x02 - in hwi */
    UINT16 usTmpFlag = (uwExcType >> 16) & 0xFFFF;

    g_uwCurNestCount++;
#if (LOSCFG_PLATFORM_HWI == YES)
    extern UINT32 g_vuwIntCount;
    g_vuwIntCount++;
#endif

    /* Save interrupt nesting times */
    m_stExcInfo.usNestCnt = (UINT16)g_uwCurNestCount;
    /* uwExcType low 16 bits: exc type */
    m_stExcInfo.usType = (UINT16)uwExcType & 0xFFFF;
#if (LOSCFG_SAVE_EXC_INFO == YES)
    /* Initializing a pointer to save an exception context */
    m_puwExcContent = (UINT32 *)m_aucTaskArray;
#endif

    /* Save the fault address when an exception occurs if it is valid */
    if (usTmpFlag & OS_EXC_FLAG_FAULTADDR_VALID)
    {
        m_stExcInfo.uwFaultAddr = uwFaultAddr;
    }
    else
    {
        m_stExcInfo.uwFaultAddr = OS_EXC_IMPRECISE_ACCESS_ADDR;
    }

    /* Save the phase of the exception */
    extern BOOL g_bTaskScheduled;
    if (g_bTaskScheduled && (NULL !=  g_stLosTask.pstRunTask))
    {
        if (usTmpFlag & OS_EXC_FLAG_IN_HWI)
        {
            m_stExcInfo.usPhase = OS_EXC_IN_HWI;
            m_stExcInfo.uwThrdPid = uwPid;  /* interrupt number */
        }
        else
        {
            m_stExcInfo.usPhase = OS_EXC_IN_TASK;
            m_stExcInfo.uwThrdPid = g_stLosTask.pstRunTask->uwTaskID;  /* task id */
        }
    }
    else
    {
        m_stExcInfo.usPhase = OS_EXC_IN_INIT;
        m_stExcInfo.uwThrdPid = 0xffffffff;
    }

    /* uwExcType BIT_28: Wether or not to use FPU */
    if(uwExcType & OS_EXC_FLAG_NO_FLOAT)
    {
        m_stExcInfo.usFpuContext = 0;
    #if FPU_EXIST
        /* NOTE: S16-S31, S0-S15,FPSCR,NO_NAME invalid */
        m_stExcInfo.pstContext = (EXC_CONTEXT_S *)((UINT8 *)pstExcBufAddr - 64); /* point to S16 */
    #else
        m_stExcInfo.pstContext = pstExcBufAddr; /* point to uwR4 */
    #endif
    }
    else
    {
        m_stExcInfo.usFpuContext = 1;
        m_stExcInfo.pstContext = pstExcBufAddr;  /* point to S16 */
    }

    osExcCallStackAnalysis(&m_stExcInfo);

#if (LOSCFG_SAVE_EXC_INFO == YES)
    osExcSave2DDR();
#endif

    osExcInfoDisplay(&m_stExcInfo);

    LOS_Reboot();
}

#if (LOSCFG_SAVE_EXC_INFO == YES)
/*****************************************************************************
 Function    : osExcSaveIntStatus
 Description : Save NVIC register group
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
static VOID osExcSaveIntStatus(VOID)
{
    /* Save exc type */
    *((UINT32 *)m_puwExcContent) = OS_EXC_TYPE_NVIC;
    m_puwExcContent = (UINT8 *)m_puwExcContent + 4;

    /* Save total size of NVIC */
    *((UINT32 *)m_puwExcContent) = 0x164; // = OS_NVIC_INT_ENABLE_SIZE + OS_NVIC_INT_PEND_SIZE + OS_NVIC_INT_ACT_SIZE + OS_NVIC_INT_PRI_SIZE + 12 + 4 + 4
    m_puwExcContent = (UINT8 *)m_puwExcContent + 4;

    /* Save interrupt enable register group (start addr: 0xE000E100, size: 32bytes, CMSIS-Core: NVIC->ISER[0 - 7]) */
    memcpy(m_puwExcContent, (const void *)OS_NVIC_SETENA_BASE, OS_NVIC_INT_ENABLE_SIZE);
    m_puwExcContent =(UINT8 *)m_puwExcContent + OS_NVIC_INT_ENABLE_SIZE;

    /* Save interrupt pend register group (start addr: 0xE000E200, size: 32bytes, CMSIS-Core: NVIC->ISPR[0 - 7]) */
    memcpy(m_puwExcContent, (const void *)OS_NVIC_SETPEND_BASE, OS_NVIC_INT_PEND_SIZE);
    m_puwExcContent = (UINT8 *)m_puwExcContent + OS_NVIC_INT_PEND_SIZE;

    /* Save interrupt active status register group (start addr: 0xE000E300, size: 32bytes, CMSIS-Core: NVIC->IABR[0 - 7]) */
    memcpy(m_puwExcContent, (const void *)OS_NVIC_INT_ACT_BASE, OS_NVIC_INT_ACT_SIZE);
    m_puwExcContent = (UINT8 *)m_puwExcContent + OS_NVIC_INT_ACT_SIZE;

    /* Save interrupt priority register group (start addr: 0xE000E400, size: 240bytes), CMSIS-Core: NVIC->IP[0 - 239] */
    memcpy(m_puwExcContent, (const void *)OS_NVIC_PRI_BASE, OS_NVIC_INT_PRI_SIZE);
    m_puwExcContent = (UINT8 *)m_puwExcContent + OS_NVIC_INT_PRI_SIZE;

    /* Save system exception priority register group (start addr: 0xE000ED18, size: 12bytes, CMSIS-Core: SCB->SHP[0 - 11]) */
    memcpy(m_puwExcContent, (const void *)OS_NVIC_EXCPRI_BASE, 12);
    m_puwExcContent = (UINT8 *)m_puwExcContent + 12;

    /* Save system processing control and state register group (start addr: 0xE000ED24, size: 4bytes, CMSIS-Core: SCB->SHCSR) */
    memcpy(m_puwExcContent, (const void *)OS_NVIC_SHCSR, 4);
    m_puwExcContent = (UINT8 *)m_puwExcContent + 4;

    /* Save interrupt control and status register group (start addr: 0xE000ED04, size: 4bytes, CMSIS-Core: SCB->ICSR) */
    memcpy(m_puwExcContent, (const void *)OS_NVIC_INT_CTRL, 4);
    m_puwExcContent = (UINT8 *)m_puwExcContent + 4;

    return;
}

/*****************************************************************************
 Function    : osExcRegister
 Description : Register exception
 Input       : uwType  --- exception type
             : pFunc   --- exception callback function
             : pArg    --- exception save info arg, not pFunc
 Output      : None
 Return      : None
 *****************************************************************************/
VOID osExcRegister(EXC_INFO_TYPE uwType, EXC_INFO_SAVE_CALLBACK pFunc, VOID *pArg)
{
    EXC_INFO_ARRAY_S *pstExcInfo;

    if (uwType == 0 || uwType >= OS_EXC_TYPE_MAX || pFunc == NULL)
    {
        PRINT_ERR("osExcRegister ERROR!\n");
        return;
    }

    pstExcInfo = &(m_stExcArray[uwType - 1]);
    pstExcInfo->uwType       = uwType;
    pstExcInfo->pFnExcInfoCb = pFunc;
    pstExcInfo->pArg         = pArg;
    pstExcInfo->uwValid      = TRUE;
}

/*****************************************************************************
 Function    : osExcSaveSysInfo
 Description : Saving exception information by calling callback function recursively
 Input       : uwType  --- exception type(OS_EXC_TYPE_CONTEXT+1 ---> OS_EXC_TYPE_MAX-1)
             : pFunc   --- callback function
             : uwLoop  --- loop total count
             : uwLen   --- The size of the data saved each time
             : uwIdx   --- loop index
 Output      : None
 Return      : None
 *****************************************************************************/
VOID osExcSaveSysInfo(EXC_INFO_TYPE uwType, EXC_INFO_SAVE_CALLBACK pFunc, UINT32 uwLoop, UINT32 uwLen, UINT32 uwIdx)
{
    UINT32 uwRet;
    UINT32 uwBuffer[OS_EXC_MAX_BUF_LEN];

    /* Save exception type and size */
    *((UINT32 *)m_puwExcContent) = uwType;
    m_puwExcContent = (UINT8 *)m_puwExcContent + 4;
    *((UINT32 *)m_puwExcContent)  = uwLen * (uwLoop - uwIdx);
    m_puwExcContent = (UINT8 *)m_puwExcContent + 4;

    for (; uwIdx < uwLoop; uwIdx++)
    {
        memset((VOID *)uwBuffer, 0, sizeof(UINT32) * OS_EXC_MAX_BUF_LEN);
        uwRet = pFunc(uwIdx, (VOID *)uwBuffer);
        if (LOS_OK == uwRet)
        {
            memcpy(m_puwExcContent, (VOID *)uwBuffer, uwLen);
        }
        m_puwExcContent =(UINT8 *)m_puwExcContent + uwLen;
    }
}

/*****************************************************************************
 Function    : osExcSaveInfo
 Description : save exception info
 Input       : uwType  --- exception type(OS_EXC_TYPE_CONTEXT+1 ---> OS_EXC_TYPE_MAX-1)
             : pFunc   --- exception callback
             : pArg    --- register arg, used to generate the arg passed to the callback
 Output      : None
 Return      : None
 *****************************************************************************/
static VOID osExcSaveInfo(EXC_INFO_TYPE uwType, EXC_INFO_SAVE_CALLBACK pFunc, VOID *pArg)
{
    UINT32 uwLen;
    UINT32 uwIdx;
    UINT32 uwLoop;
    UINT32 uwTaskSwitchCount = 0;
    OS_TASK_SWITCH_INFO *pstTaskSwitchInfo;

    switch(uwType)
    {
        case OS_EXC_TYPE_TSK:  /* save task info */
            uwLen = sizeof(TSK_INFO_S);
            uwLoop = *(UINT32 *)pArg;
            uwIdx = 0;
            break;

        case OS_EXC_TYPE_QUE:  /* save queue info */
            uwLen = sizeof(QUEUE_INFO_S);
            uwLoop = *(UINT32 *)pArg;
            uwIdx = 0;
            break;

        case OS_EXC_TYPE_NVIC:  /* save NVIC info */
            (VOID)pFunc(0, 0);
            goto END;

        case OS_EXC_TYPE_TSK_SWITCH:  /* save task switch info */
            pstTaskSwitchInfo = pArg;
            uwTaskSwitchCount = pstTaskSwitchInfo->ucIsFull & 0x7F;
            uwLen = sizeof(UINT32) + sizeof(CHAR) * LOS_TASK_NAMELEN; /* auwPID + acName */
            if (pstTaskSwitchInfo->ucIsFull & 0x80)
            {
                uwIdx = pstTaskSwitchInfo->ucIdx;
                uwLoop = uwIdx + uwTaskSwitchCount;
            }
            else
            {
                uwIdx = 0;
                uwLoop = pstTaskSwitchInfo->ucIdx;
            }
            break;

        case OS_EXC_TYPE_MEM:  /* save mem info */
            uwLen = sizeof(MEM_INFO_S);
            uwLoop = *(UINT32 *)pArg;
            uwIdx = 0;
            break;

        default:
            goto END;
    }
    osExcSaveSysInfo(uwType, (EXC_INFO_SAVE_CALLBACK)pFunc, uwLoop, uwLen, uwIdx);
END:
    return;
}

/*****************************************************************************
 Function    : osExcSave2DDR
 Description : Save exception info to RAM
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
static VOID osExcSave2DDR(VOID)
{
    UINT32 uwIdx = 0;
    UINT32 uwExcContextSize;

    if (m_stExcInfo.usFpuContext == 1)
    {
        uwExcContextSize = sizeof(EXC_CONTEXT_S);
    }
    else
    {
        uwExcContextSize = sizeof(EXC_CONTEXT_S) - 136; /* except FPU register */
    }

    memset(m_puwExcContent, 0xff, g_uwArraySize);

    /* Cortex-M type */
    *((UINT32 *)m_puwExcContent) = 4;
    m_puwExcContent = (UINT8 *)m_puwExcContent + 4;

    /*
     * Save exception type: OS_EXC_TYPE_CONTEXT
     */
    *((UINT32 *)m_puwExcContent) = OS_EXC_TYPE_CONTEXT;
    m_puwExcContent = (UINT8 *)m_puwExcContent + 4;

    /* The size of struct EXC_INFO_S(except member EXC_CONTEXT_S*) and exception context size */
    *((UINT32 *)m_puwExcContent) = sizeof(EXC_INFO_S) - sizeof(EXC_CONTEXT_S *) + uwExcContextSize;
    m_puwExcContent = (UINT8 *)m_puwExcContent + 4;

    /* Save struct m_stExcInfo except m_stExcInfo.pstContext */
    memcpy((VOID *)m_puwExcContent, (VOID *)&m_stExcInfo, sizeof(EXC_INFO_S) - sizeof(EXC_CONTEXT_S *));
    m_puwExcContent = (UINT8 *)m_puwExcContent + sizeof(EXC_INFO_S) - sizeof(EXC_CONTEXT_S *);

    /* Save struct EXC_CONTEXT_S */
    if (m_stExcInfo.usFpuContext == 0)
    {
        #if FPU_EXIST
        /* m_stExcInfo.pstContext: init --- point to S16, S16->S31 invalid
         *                         + 64 --- point to uwR4
         *                         copy uwR4 -> uwxPSR */
        memcpy((VOID *)m_puwExcContent, (UINT8 *)m_stExcInfo.pstContext + 64, uwExcContextSize);
        #else
        memcpy((VOID *)m_puwExcContent, m_stExcInfo.pstContext, uwExcContextSize);
        #endif
    }
    else
    {
        memcpy((VOID *)m_puwExcContent, m_stExcInfo.pstContext, uwExcContextSize);
    }
    m_puwExcContent = (UINT8 *)m_puwExcContent + uwExcContextSize;

    /*
     * Save exception type: OS_EXC_TYPE_CONTEXT+1 ---> OS_EXC_TYPE_MAX-1
     */
    for (uwIdx = 0; uwIdx < OS_EXC_TYPE_MAX - 1; uwIdx++)
    {
        if (m_stExcArray[uwIdx].uwValid == FALSE)
        {
            continue;
        }
        osExcSaveInfo(m_stExcArray[uwIdx].uwType, m_stExcArray[uwIdx].pFnExcInfoCb, m_stExcArray[uwIdx].pArg);
    }

    /*
     * Save exception type: OS_EXC_TYPE_MAX
     */
    *((UINT32 *)m_puwExcContent) = OS_EXC_TYPE_MAX;
    m_puwExcContent = (UINT8*)m_puwExcContent + 4;

    return;
}
#endif /*(LOSCFG_SAVE_EXC_INFO == YES)*/

/*****************************************************************************
 Function    : osExcInit
 Description : Initializes the EXC
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT VOID osExcInit(UINT32 uwArraySize)
{
#if (LOSCFG_PLATFORM_HWI == YES)
    /* Register exception handler to interrupt vector table in RAM */
    m_pstHwiForm[-14 + OS_SYS_VECTOR_CNT] = NMI_Handler;
    m_pstHwiForm[-13 + OS_SYS_VECTOR_CNT] = HardFault_Handler;
    m_pstHwiForm[-12 + OS_SYS_VECTOR_CNT] = MemManage_Handler;
    m_pstHwiForm[-11 + OS_SYS_VECTOR_CNT] = BusFault_Handler;
    m_pstHwiForm[-10 + OS_SYS_VECTOR_CNT] = UsageFault_Handler;
    m_pstHwiForm[-5  + OS_SYS_VECTOR_CNT] = SVC_Handler;
#endif

    /* Enable USGFAULT(BIT_18), BUSFAULT(BIT_17), MEMFAULT(BIT_16) */
    *(volatile UINT32 *)OS_NVIC_SHCSR |= 0x70000;

    /* Enable DIV 0(BIT_4) exception, unaligned(BIT_3) disable */
    *(volatile UINT32 *)OS_NVIC_CCR |= 0x10;

#if (LOSCFG_SAVE_EXC_INFO == YES)
    g_uwArraySize = uwArraySize;
    osExcRegister((EXC_INFO_TYPE)OS_EXC_TYPE_NVIC, (EXC_INFO_SAVE_CALLBACK)osExcSaveIntStatus, NULL);
#endif
}

VOID osBackTrace(VOID)
{
    return;
}

#endif /*(LOSCFG_PLATFORM_EXC == YES)*/

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */

