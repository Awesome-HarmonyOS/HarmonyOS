/** ----------------------------------------------------------------------------
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
/** ----------------------------------------------------------------------------
 * Notice of Export Control Law
 * ===============================================
 * Huawei LiteOS may be subject to applicable export control laws and regulations, which might
 * include those applicable to Huawei LiteOS of U.S. and the country in which you are located.
 * Import, export and usage of Huawei LiteOS in any manner by you shall be in compliance with such
 * applicable export control laws and regulations.
 *---------------------------------------------------------------------------*/

/****************************************************************************************
*                                  EXPORT FUNCTIONS
****************************************************************************************/

    .global  LOS_IntLock
    .global  LOS_IntUnLock
    .global  LOS_IntRestore
    .global  LOS_StartToRun
    .global  osTaskSchedule
    .global  PendSV_Handler

/****************************************************************************************
*                                  EXTERN PARAMETERS
****************************************************************************************/

    .extern  g_stLosTask
    .extern  g_pfnTskSwitchHook
    .extern  g_bTaskScheduled

/****************************************************************************************
*                                  EQU
****************************************************************************************/

.equ    OS_NVIC_INT_CTRL,              0xE000ED04  /* Interrupt Control and State Register. */
.equ    OS_NVIC_PENDSVSET,             0x10000000  /* Value to trigger PendSV exception. */

.equ    OS_NVIC_SYSPRI2,               0xE000ED20  /* System Handler Priority Register 2. */
.equ    OS_NVIC_PENDSV_SYSTICK_PRI,    0xFFFF0000  /* SysTick + PendSV priority level (lowest). */

.equ    OS_TASK_STATUS_RUNNING,        0x0010      /* Task Status Flag (RUNNING). */

/****************************************************************************************
*                                  CODE GENERATION DIRECTIVES
****************************************************************************************/

    .section .text
    .thumb
    .syntax unified
    .arch armv7e-m

/****************************************************************************************
* Function:
*        VOID LOS_StartToRun(VOID);
* Description:
*        Start the first task, which is the highest priority task in the priority queue.
*        Other tasks are started by task scheduling.
****************************************************************************************/
    .type LOS_StartToRun, %function
LOS_StartToRun:
    CPSID   I

    /**
     * Set PendSV and SysTick prority to the lowest.
     * read ---> modify ---> write-back.
     */
    LDR     R0, =OS_NVIC_SYSPRI2
    LDR     R1, =OS_NVIC_PENDSV_SYSTICK_PRI
    LDR     R2, [R0]
    ORR     R1, R1, R2
    STR     R1, [R0]

    /**
     * Set g_bTaskScheduled = 1.
     */
    LDR     R0, =g_bTaskScheduled
    MOV     R1, #1
    STR     R1, [R0]

    /**
     * Set g_stLosTask.pstRunTask = g_stLosTask.pstNewTask.
     */
    LDR     R0, =g_stLosTask
    LDR     R1, [R0, #4]
    STR     R1, [R0]

    /**
     * Set g_stLosTask.pstRunTask->usTaskStatus |= OS_TASK_STATUS_RUNNING.
     */
    LDR     R1, [R0]
    LDRH    R2, [R1, #4]
    MOV     R3, #OS_TASK_STATUS_RUNNING
    ORR     R2, R2, R3
    STRH    R2, [R1, #4]

    /**
     * Restore the default stack frame(R0-R3,R12,LR,PC,xPSR) of g_stLosTask.pstRunTask to R0-R7.
     * [Initial EXC_RETURN ignore,] return by setting the CONTROL register.
     *
     * The initial stack of the current running task is as follows:
     *
     *                     POP: Restore the context of the current running task ===>|
     *                                                                 High addr--->|
     *                                                       Bottom of the stack--->|
     * ----------+---------------------------------+--------------------------------+
     *           |  R4-R11,  PRIMASK,  EXC_RETURN  |  R0-R3,  R12,  LR,  PC,  xPSR  |
     * ----------+---------------------------------+--------------------------------+
     *           |<---Top of the stack, restored from g_stLosTask.pstRunTask->pStackPointer
     *           |<---           skip          --->|<---     copy to R0-R7      --->|
     *                                                                R12 to PSP--->|
     *                                        Stack pointer after LOS_StartToRun--->|
     */
    LDR     R12, [R1]
    ADD     R12, R12, #36          /* skip R4-R11, PRIMASK. */
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    ADD     R12, R12, #4           /* if FPU exist, skip EXC_RETURN. */
#endif
    LDMFD   R12!, {R0-R7}

    /**
     * Set the stack pointer of g_stLosTask.pstRunTask to PSP.
     */
    MSR     PSP, R12

    /**
     * Set the CONTROL register, after schedule start, privilege level and stack = PSP.
     */
    MOV     R12, #2
    MSR     CONTROL, R12

    /**
     * Enable interrupt. (The default PRIMASK value is 0, so enable directly)
     */
    MOV     LR, R5
    CPSIE   I

    /**
     * Jump directly to the default PC of g_stLosTask.pstRunTask, the field information
     * of the main function will be destroyed and will never be returned.
     */
    BX      R6

/****************************************************************************************
* Function:
*        UINTPTR LOS_IntLock(VOID);
* Description:
*        Disable all interrupts except Reset,NMI and HardFault.
*        The value of currnet interruption state will be returned to the caller to save.
*
* Function:
*        VOID LOS_IntRestore(UINTPTR uvIntSave);
* Description:
*        Restore the locked interruption of LOS_IntLock.
*        The caller must pass in the value of interruption state previously saved.
****************************************************************************************/
    .type LOS_IntLock, %function
LOS_IntLock:
    MRS     R0, PRIMASK
    CPSID   I
    BX      LR

    .type LOS_IntUnLock, %function
LOS_IntUnLock:
    MRS     R0, PRIMASK
    CPSIE   I
    BX      LR

    .type LOS_IntRestore, %function
LOS_IntRestore:
    MSR     PRIMASK, R0
    BX      LR

/****************************************************************************************
* Function:
*        VOID osTaskSchedule(VOID);
* Description:
*        Start the task swtich process by software trigger PendSV interrupt.
****************************************************************************************/
    .type osTaskSchedule, %function
osTaskSchedule:
    LDR     R0, =OS_NVIC_INT_CTRL
    LDR     R1, =OS_NVIC_PENDSVSET
    STR     R1, [R0]
    BX      LR

/****************************************************************************************
* Function:
*        VOID PendSV_Handler(VOID);
* Description:
*        PendSV interrupt handler, switch the context of the task.
*        First: Save the context of the current running task(g_stLosTask.pstRunTask)
*               to its own stack.
*        Second: Restore the context of the next running task(g_stLosTask.pstNewTask)
*                from its own stack.
****************************************************************************************/
    .type PendSV_Handler, %function
PendSV_Handler:
    /**
     * R12: Save the interruption state of the current running task.
     * Disable all interrupts except Reset,NMI and HardFault
     */
    MRS     R12, PRIMASK
    CPSID   I

    /**
     * Call task switch hook.
     */
    LDR     R2, =g_pfnTskSwitchHook
    LDR     R2, [R2]
    CBZ     R2, TaskSwitch
    PUSH    {R12, LR}
    BLX     R2
    POP     {R12, LR}

TaskSwitch:
    /**
     * R0 = now stack pointer of the current running task.
     */
    MRS     R0, PSP

    /**
     * Save the stack frame([S16-S31],R4-R11) of the current running task.
     * R12 save the PRIMASK value of the current running task.
     * NOTE: 1. Before entering the exception handler function, these registers
     *       ([NO_NAME,FPSCR,S15-S0],xPSR,PC,LR,R12,R3-R0) have been automatically
     *       saved by the CPU in the stack of the current running task.
     *       2. If lazy stacking is enabled, space is reserved on the stack for
     *       the floating-point context(FPSCR,S15-S0), but the floating-point state
     *       is not saved. when the floating-point instruction(VSTMDBEQ  R0!, {D8-D15})
     *       is executed, the floating-point context(FPSCR,S15-S0) is first saved into
     *       the space reserved on the stack. In other words, the instruction
     *       'VSTMDBEQ  R0!, {D8-D15}' will trigger the CPU to save 'FPSCR,S15-S0' first.
     *
     * The stack of the current running task is as follows:
     *
     *   |<=== PUSH: Save the context of the current running task
     *   |                                                                     High addr--->|
     * --+-----------------------------------+-------------------------------------------+---
     *   | R4-R11,PRIMASK,EXC_RETURN,S16-S31 | R0-R3,R12,LR,PC,xPSR,S0-S15,FPSCR,NO_NAME |
     *   |                                                         [   lazy stacking    ]|
     * --+-----------------------------------+-------------------------------------------+---
     *                                        Stack pointer before entering exception--->|
     *                                       |<---           cpu auto saved          --->|
     *                                       |<---PSP to R0
     *   |<---Top of the stack, save to g_stLosTask.pstRunTask->pStackPointer
     */
#if defined (__VFP_FP__) && !defined(__SOFTFP__)       /* if FPU exist. */
    TST     R14, #0x10             /* if the task using the FPU context, push s16-s31. */
    IT      EQ
    VSTMDBEQ  R0!, {D8-D15}
    STMFD   R0!, {R14}             /* save EXC_RETURN. */
#endif
    STMFD   R0!, {R4-R12}          /* save the core registers and PRIMASK. */

    /**
     * R5,R8.
     */
    LDR     R5, =g_stLosTask
    MOV     R8, #OS_TASK_STATUS_RUNNING

    /**
     * Save the stack pointer of the current running task to TCB.
     * (g_stLosTask.pstRunTask->pStackPointer = R0)
     */
    LDR     R6, [R5]
    STR     R0, [R6]

    /**
     * Clear the RUNNING state of the current running task.
     * (g_stLosTask.pstRunTask->usTaskStatus &= ~OS_TASK_STATUS_RUNNING)
     */
    LDRH    R7, [R6, #4]
    BIC     R7, R7, R8
    STRH    R7, [R6, #4]

    /**
     * Switch the current running task to the next running task.
     * (g_stLosTask.pstRunTask = g_stLosTask.pstNewTask)
     */
    LDR     R0, [R5, #4]
    STR     R0, [R5]

    /**
     * Set the RUNNING state of the next running task.
     * (g_stLosTask.pstNewTask->usTaskStatus |= OS_TASK_STATUS_RUNNING)
     */
    LDRH    R7, [R0, #4]
    ORR     R7, R7, R8
    STRH    R7, [R0, #4]

    /**
     * Restore the stack pointer of the next running task from TCB.
     * (R1 = g_stLosTask.pstNewTask->pStackPointer)
     */
    LDR     R1, [R0]

    /**
     * Restore the stack frame(R4-R11,[S16-S31]) of the next running task.
     * R12 restore the PRIMASK value of the next running task.
     * NOTE: After exiting the exception handler function, these registers
     *       (PC,xPSR,R0-R3,R12,LR,[S0-S15,FPSCR,NO_NAME]) will be automatically
     *       restored by the CPU from the stack of the next running task.
     *
     * 1. The stack of the next running task is as follows:
     *
     *                             POP: Restore the context of the next running task ===>|
     *                                                                         High addr--->|
     * --+-----------------------------------+-------------------------------------------+---
     *   | R4-R11,PRIMASK,EXC_RETURN,S16-S31 | R0-R3,R12,LR,PC,xPSR,S0-S15,FPSCR,NO_NAME |
     * --+-----------------------------------+-------------------------------------------+---
     *   |<---Top of the stack, restored from g_stLosTask.pstNewTask->pStackPointer
     *                          R1 to PSP--->|
     *                                       |<---        cpu auto restoring         --->|
     *                                          Stack pointer after exiting exception--->|
     *
     * 2. If the next running task is run for the first time, the stack is as follows:
     *
     *                        POP: Restore the context of the next running task ===>|
     *                                                                 High addr--->|
     *                                                       Bottom of the stack--->|
     * ----------+---------------------------------+--------------------------------+
     *           |  R4-R11,  PRIMASK,  EXC_RETURN  |  R0-R3,  R12,  LR,  PC,  xPSR  |
     * ----------+---------------------------------+--------------------------------+
     *           |<---Top of the stack, restored from g_stLosTask.pstNewTask->pStackPointer
     *                                R1 to PSP--->|
     *                                             |<---   cpu auto restoring   --->|
     *                                     Stack pointer after exiting exception--->|
     */
    LDMFD   R1!, {R4-R12}          /* restore the core registers and PRIMASK. */
#if defined (__VFP_FP__) && !defined(__SOFTFP__)       /* if FPU exist. */
    LDMFD   R1!, {R14}             /* restore EXC_RETURN. */
    TST     R14, #0x10             /* if the task using the FPU context, pop s16-s31. */
    IT      EQ
    VLDMIAEQ  R1!, {D8-D15}
#endif

    /**
     * Set the stack pointer of the next running task to PSP.
     */
    MSR     PSP, R1

    /**
     * Restore the interruption state of the next running task.
     */
    MSR     PRIMASK, R12
    BX      LR

