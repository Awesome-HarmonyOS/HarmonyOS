;----------------------------------------------------------------------------
 ; Copyright (c) <2016-2018>, <Huawei Technologies Co., Ltd>
 ; All rights reserved.
 ; Redistribution and use in source and binary forms, with or without modification,
 ; are permitted provided that the following conditions are met:
 ; 1. Redistributions of source code must retain the above copyright notice, this list of
 ; conditions and the following disclaimer.
 ; 2. Redistributions in binary form must reproduce the above copyright notice, this list
 ; of conditions and the following disclaimer in the documentation and/or other materials
 ; provided with the distribution.
 ; 3. Neither the name of the copyright holder nor the names of its contributors may be used
 ; to endorse or promote products derived from this software without specific prior written
 ; permission.
 ; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 ; "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 ; THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 ; PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 ; CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 ; EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 ; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 ; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 ; WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 ; OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ; ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ;---------------------------------------------------------------------------*/
;----------------------------------------------------------------------------
 ; Notice of Export Control Law
 ; ===============================================
 ; Huawei LiteOS may be subject to applicable export control laws and regulations, which might
 ; include those applicable to Huawei LiteOS of U.S. and the country in which you are located.
 ; Import, export and usage of Huawei LiteOS in any manner by you shall be in compliance with such
 ; applicable export control laws and regulations.
 ;---------------------------------------------------------------------------*/

;****************************************************************************************
;                                  EXPORT FUNCTIONS
;****************************************************************************************

    EXPORT  LOS_IntLock
    EXPORT  LOS_IntUnLock
    EXPORT  LOS_IntRestore
    EXPORT  LOS_StartToRun
    EXPORT  osTaskSchedule
    EXPORT  PendSV_Handler

;****************************************************************************************
;                                  EXTERN PARAMETERS
;****************************************************************************************

    IMPORT  g_stLosTask
    IMPORT  g_pfnTskSwitchHook
    IMPORT  g_bTaskScheduled

;****************************************************************************************
;                                  EQU
;****************************************************************************************

OS_NVIC_INT_CTRL              EQU    0xE000ED04  ; Interrupt Control and State Register.
OS_NVIC_PENDSVSET             EQU    0x10000000  ; Value to trigger PendSV exception.

OS_NVIC_SYSPRI2               EQU    0xE000ED20  ; System Handler Priority Register 2.
OS_NVIC_PENDSV_SYSTICK_PRI    EQU    0xC0C00000  ; SysTick + PendSV priority level (lowest).

OS_TASK_STATUS_RUNNING        EQU    0x0010      ; Task Status Flag (RUNNING).

;****************************************************************************************
;                                  CODE GENERATION DIRECTIVES
;****************************************************************************************

    AREA    |.text|, CODE, READONLY
    THUMB
    REQUIRE8
    PRESERVE8

;****************************************************************************************
; Function:
;        VOID LOS_StartToRun(VOID);
; Description:
;        Start the first task, which is the highest priority task in the priority queue.
;        Other tasks are started by task scheduling.
;****************************************************************************************
LOS_StartToRun
    CPSID   I

    ;/**
    ; * Set PendSV and SysTick prority to the lowest.
    ; * read ---> modify ---> write-back.
    ; */
    LDR     R0, =OS_NVIC_SYSPRI2
    LDR     R1, =OS_NVIC_PENDSV_SYSTICK_PRI
    LDR     R2, [R0]
    ORRS    R1, R1, R2
    STR     R1, [R0]

    ;/**
    ; * Set g_bTaskScheduled = 1.
    ; */
    LDR     R0, =g_bTaskScheduled
    MOVS    R1, #1
    STR     R1, [R0]

    ;/**
    ; * Now PSP is an invalid value, R13 uses MSP as the stack pointer.
    ; * After setting the control register, R13 will switch to PSP.
    ; * To ensure that R13 is valid, set PSP = MSP.
    ; */
    MRS     R0, MSP
    MSR     PSP, R0

    ;/**
    ; * Set the CONTROL register, after schedule start, privilege level and stack = PSP.
    ; */
    MOVS    R0, #2
    MSR     CONTROL, R0

    ;/**
    ; * Set g_stLosTask.pstRunTask = g_stLosTask.pstNewTask.
    ; */
    LDR     R0, =g_stLosTask
    LDR     R1, [R0, #4]
    STR     R1, [R0]

    ;/**
    ; * Set g_stLosTask.pstRunTask->usTaskStatus |= OS_TASK_STATUS_RUNNING.
    ; */
    LDR     R1, [R0]
    LDRH    R2, [R1, #4]
    MOVS    R3, #OS_TASK_STATUS_RUNNING
    ORRS    R2, R2, R3
    STRH    R2, [R1, #4]

    ;/**
    ; * Restore the default stack frame(R0-R3,R12,LR,PC,xPSR) of g_stLosTask.pstRunTask to R0-R7.
    ; * Return by setting the CONTROL register.
    ; *
    ; * The initial stack of the current running task is as follows:
    ; *
    ; *                 POP: Restore the context of the current running task ===>|
    ; *                                                             High addr--->|
    ; *                                                   Bottom of the stack--->|
    ; * ----------+-----------------------+--------------------------------------+
    ; *           |   R4-R11,   PRIMASK   |   R0-R3,   R12,   LR,   PC,   xPSR   |
    ; * ----------+-----------------------+--------------------------------------+
    ; *           |<---Top of the stack, restored from g_stLosTask.pstRunTask->pStackPointer
    ; *           |<---      skip     --->|<---        copy to R0-R7         --->|
    ; *                                                             R3 to PSP--->|
    ; *                                    Stack pointer after LOS_StartToRun--->|
    ; */
    LDR     R3, [R1]
    ADDS    R3, R3, #36            ; skip R4-R11, PRIMASK.

    LDMFD   R3!, {R0-R2}           ; restore R0-R2.
    ADDS    R3, R3, #4             ; skip R3.
    LDMFD   R3!, {R4-R7}           ; restore R12,LR,PC,xPSR.

    ;/**
    ; * Set the stack pointer of g_stLosTask.pstRunTask to PSP.
    ; */
    MSR     PSP, R3

    SUBS    R3, R3, #20
    LDR     R3, [R3]               ; restore R3.

    ;/**
    ; * Enable interrupt. (The default PRIMASK value is 0, so enable directly)
    ; */
    MOV     LR, R5
    CPSIE   I

    ;/**
    ; * Jump directly to the default PC of g_stLosTask.pstRunTask, the field information
    ; * of the main function will be destroyed and will never be returned.
    ; */
    BX      R6

;****************************************************************************************
; Function:
;        UINTPTR LOS_IntLock(VOID);
; Description:
;        Disable all interrupts except Reset,NMI and HardFault.
;        The value of currnet interruption state will be returned to the caller to save.
;
; Function:
;        VOID LOS_IntRestore(UINTPTR uvIntSave);
; Description:
;        Restore the locked interruption of LOS_IntLock.
;        The caller must pass in the value of interruption state previously saved.
;****************************************************************************************
LOS_IntLock
    MRS     R0, PRIMASK
    CPSID   I
    BX      LR

LOS_IntUnLock
    MRS     R0, PRIMASK
    CPSIE   I
    BX      LR

LOS_IntRestore
    MSR     PRIMASK, R0
    BX      LR

;****************************************************************************************
; Function:
;        VOID osTaskSchedule(VOID);
; Description:
;        Start the task swtich process by software trigger PendSV interrupt.
;****************************************************************************************
osTaskSchedule
    LDR     R0, =OS_NVIC_INT_CTRL
    LDR     R1, =OS_NVIC_PENDSVSET
    STR     R1, [R0]
    BX      LR

;****************************************************************************************
; Function:
;        VOID PendSV_Handler(VOID);
; Description:
;        PendSV interrupt handler, switch the context of the task.
;        First: Save the context of the current running task(g_stLosTask.pstRunTask)
;               to its own stack.
;        Second: Restore the context of the next running task(g_stLosTask.pstNewTask)
;                from its own stack.
;****************************************************************************************
PendSV_Handler
    ;/**
    ; * R12: Save the interruption state of the current running task.
    ; * Disable all interrupts except Reset,NMI and HardFault
    ; */
    MRS     R12, PRIMASK
    CPSID   I

    ;/**
    ; * Call task switch hook.
    ; */
    LDR     R2, =g_pfnTskSwitchHook
    LDR     R2, [R2]
    CMP     R2, #0
    BEQ     TaskSwitch
    MOV     R1, LR
    PUSH    {R0, R1}
    BLX     R2
    POP     {R0, R1}
    MOV     LR, R1

TaskSwitch
    ;/**
    ; * R0 = now stack pointer of the current running task.
    ; */
    MRS     R0, PSP

    ;/**
    ; * Save the stack frame(R4-R11) of the current running task.
    ; * R12 save the PRIMASK value of the current running task.
    ; * NOTE: Before entering the exception handler function, these registers
    ; *       (xPSR,PC,LR,R12,R3-R0) have been automatically
    ; *       saved by the CPU in the stack of the current running task.
    ; *
    ; * The stack of the current running task is as follows:
    ; *
    ; *           |<=== PUSH: Save the context of the current running task
    ; *           |                                                      High addr--->|
    ; * ----------+-----------------------+--------------------------------------+-----
    ; *           |   R4-R11,   PRIMASK   |   R0-R3,   R12,   LR,   PC,   xPSR   |
    ; * ----------+-----------------------+--------------------------------------+-----
    ; *                               Stack pointer before entering exception--->|
    ; *                                   |<---        cpu auto saved        --->|
    ; *                                   |<---PSP to R0
    ; *           |<---Top of the stack, save to g_stLosTask.pstRunTask->pStackPointer
    ; */
    SUBS    R0, #36
    STMIA   R0!, {R4-R7}           ; save R4-R7.
    MOV     R3, R8                 ; copy R8-r12 to R3-r7.
    MOV     R4, R9
    MOV     R5, R10
    MOV     R6, R11
    MOV     R7, R12
    STMIA   R0!, {R3-R7}           ; save R8-R12.
    SUBS    R0, #36

    ;/**
    ; * R5,R3.
    ; */
    LDR     R5, =g_stLosTask
    MOVS    R3, #OS_TASK_STATUS_RUNNING

    ;/**
    ; * Save the stack pointer of the current running task to TCB.
    ; * (g_stLosTask.pstRunTask->pStackPointer = R0)
    ; */
    LDR     R6, [R5]
    STR     R0, [R6]

    ;/**
    ; * Clear the RUNNING state of the current running task.
    ; * (g_stLosTask.pstRunTask->usTaskStatus &= ~OS_TASK_STATUS_RUNNING)
    ; */
    LDRH    R7, [R6, #4]
    BICS    R7, R7, R3
    STRH    R7, [R6, #4]

    ;/**
    ; * Switch the current running task to the next running task.
    ; * (g_stLosTask.pstRunTask = g_stLosTask.pstNewTask)
    ; */
    LDR     R0, [R5, #4]
    STR     R0, [R5]

    ;/**
    ; * Set the RUNNING state of the next running task.
    ; * (g_stLosTask.pstNewTask->usTaskStatus |= OS_TASK_STATUS_RUNNING)
    ; */
    LDRH    R7, [R0, #4]
    ORRS    R7, R7, R3
    STRH    R7, [R0, #4]

    ;/**
    ; * Restore the stack pointer of the next running task from TCB.
    ; * (R1 = g_stLosTask.pstNewTask->pStackPointer)
    ; */
    LDR     R1, [R0]

    ;/**
    ; * Restore the stack frame(R4-R11) of the next running task.
    ; * R12 restore the PRIMASK value of the next running task.
    ; * NOTE: After exiting the exception handler function, these registers
    ; *       (xPSR,R0-R3,R12,LR,PC) will be automatically
    ; *       restored by the CPU from the stack of the next running task.
    ; *
    ; * 1. The stack of the next running task is as follows:
    ; *
    ; *                    POP: Restore the context of the next running task ===>|
    ; *                                                                  High addr--->|
    ; * ----------+-----------------------+--------------------------------------+-----
    ; *           |   R4-R11,   PRIMASK   |   R0-R3,   R12,   LR,   PC,   xPSR   |
    ; * ----------+-----------------------+--------------------------------------+-----
    ; *           |<---Top of the stack, restored from g_stLosTask.pstNewTask->pStackPointer
    ; *                      R1 to PSP--->|
    ; *                                   |<---      cpu auto restoring      --->|
    ; *                                 Stack pointer after exiting exception--->|
    ; *
    ; * 2. If the next running task is run for the first time, the stack is as follows:
    ; *
    ; *                    POP: Restore the context of the next running task ===>|
    ; *                                                             High addr--->|
    ; *                                                   Bottom of the stack--->|
    ; * ----------+-----------------------+--------------------------------------+
    ; *           |   R4-R11,   PRIMASK   |   R0-R3,   R12,   LR,   PC,   xPSR   |
    ; * ----------+-----------------------+--------------------------------------+
    ; *           |<---Top of the stack, restored from g_stLosTask.pstNewTask->pStackPointer
    ; *                      R1 to PSP--->|
    ; *                                   |<---      cpu auto restoring      --->|
    ; *                                 Stack pointer after exiting exception--->|
    ; */
    ADDS    R1, #16
    LDMFD   R1!, {R3-R7}           ; restore R8-R12 to R3-R7.
    MOV     R8, R3                 ; copy R3-R7 to R8-R12.
    MOV     R9, R4
    MOV     R10, R5
    MOV     R11, R6
    MOV     R12, R7
    ;/**
    ; * Set the stack pointer of the next running task to PSP.
    ; */
    MSR     PSP, R1
    SUBS    R1, #36
    LDMFD   R1!, {R4-R7}           ; restore R4-R7.

    ;/**
    ; * Restore the interruption state of the next running task.
    ; */
    MSR     PRIMASK, R12
    BX      LR

    ALIGN
    END

