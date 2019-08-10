;----------------------------------------------------------------------------
 ; Copyright (c) <2013-2015>, <Huawei Technologies Co., Ltd>
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
;                                  CODE GENERATION DIRECTIVES
;****************************************************************************************

    PRESERVE8
    AREA    |.text|, CODE, READONLY
    THUMB

;****************************************************************************************
;                                  EXPORT FUNCTIONS
;****************************************************************************************

    EXPORT  NMI_Handler
    EXPORT  HardFault_Handler
    EXPORT  MemManage_Handler
    EXPORT  BusFault_Handler
    EXPORT  UsageFault_Handler
;    EXPORT  SVC_Handler

;****************************************************************************************
;                                  EXTERN PARAMETERS
;****************************************************************************************

    IMPORT osExcHandleEntry
    IMPORT g_uwExcTbl
    IMPORT g_bTaskScheduled
    IMPORT it_is_memManangeFault
    IMPORT it_is_busFault
    IMPORT it_is_usageFault

;****************************************************************************************
;                                  EQU
;****************************************************************************************

OS_EXC_CAUSE_NMI            EQU   18
OS_EXC_CAUSE_HARDFAULT      EQU   19
OS_EXC_CAUSE_MEMFAULT       EQU   20
OS_EXC_CAUSE_BUSFAULT       EQU   21
OS_EXC_CAUSE_USAGEFAULT     EQU   22
OS_EXC_CAUSE_SVC            EQU   23

HF_DEBUGEVT                 EQU   24
HF_VECTBL                   EQU   25

FLAG_ADDR_VALID             EQU   0x10000       ; bit 16
FLAG_HWI_ACTIVE             EQU   0x20000       ; bit 17
FLAG_NO_FLOAT               EQU   0x10000000    ; bit 28

OS_NVIC_CFSR                EQU   0xE000ED28    ; include BusFault/MemFault/UsageFault State Regeister
OS_NVIC_HFSR                EQU   0xE000ED2C    ; HardFault State Regeister
OS_NVIC_BFAR                EQU   0xE000ED38
OS_NVIC_MMFAR               EQU   0xE000ED34
OS_NVIC_ACT_BASE            EQU   0xE000E300
OS_NVIC_SHCSRS              EQU   0xE000ED24
OS_NVIC_SHCSR_MASK          EQU   0xC00         ; SYSTICKACT and PENDSVACT

ADDR_SETPEND        EQU        0XE000E200        ;the addr of setpend register
ADDR_CLRPEND        EQU        0XE000E280        ;the addr of clrpend register
;****************************************************************************************
; Function:
;        VOID NMI_Handler(VOID);
; Description:
;        NMI Handler.
;****************************************************************************************
NMI_Handler
    ;/**
    ; * Before executing instruction 'B osExcDispatch', the value of R0 is as follows.
    ; * < R0 >:
    ; * +------------------------------------------------------+------------------------+
    ; * |                          31-8                        |          7-0           |
    ; * +------------------------------------------------------+------------------------+
    ; * |                          ---                         |    OS_EXC_CAUSE_NMI    |
    ; * +------------------------------------------------------+------------------------+
    ; * < R1 >: invalid
    ; */
    MOVS  R0, #OS_EXC_CAUSE_NMI
    MOVS  R1, #0
    B  osExcDispatch

;****************************************************************************************
; Function:
;        VOID HardFault_Handler(VOID);
; Description:
;        HardFault Handler.
;****************************************************************************************
HardFault_Handler
    ;/**
    ; * save fault type to R0
    ; */
    MOVS  R0, #OS_EXC_CAUSE_HARDFAULT    
    B  osExcDispatch      

;****************************************************************************************
; Function:
;        VOID BusFault_Handler(VOID);
; Description:
;        BusFault Handler.
;****************************************************************************************
BusFault_Handler
    BL it_is_busFault
;****************************************************************************************
; Function:
;        VOID MemManage_Handler(VOID);
; Description:
;        MemManage Handler.
;****************************************************************************************
MemManage_Handler
    BL it_is_memManangeFault
;****************************************************************************************
; Function:
;        VOID UsageFault_Handler(VOID);
; Description:
;        UsageFault Handler.
;****************************************************************************************
UsageFault_Handler
    BL it_is_usageFault

    ;/**
    ; * When executing osExcDispatch,  R2 will be used.
    ; */
osExcDispatch
    LDR   R2, =ADDR_SETPEND
    LDR   R2, [R2]
        
    CMP R2,#0
    BEQ _whether_in_initialization                ;NO external interrupt occurred

    ;/**
    ; * Interrupts and initialization phase always use MSP.
    ; */
_ExcInMSP
    ;/**
    ; * Before executing instruction 'B _handleEntry', MSP is as follows.
    ; * MSP:
    ; *                                                                              High addr--->|
    ; * +--------------------------------------------------------------------------------+---------
    ; *                                 | R4-R11,PRIMASK,SAVED_SP | R0-R3,R12,LR,PC,xPSR |
    ; * +--------------------------------------------------------------------------------+---------
    ; *                          R13--->|          Initial R13--->|<---      #32     --->|<---SAVED_SP
    ; *                                                           |   (CPU auto saved)   |
    ; *
    ; */
_NoFloatInMsp
    CMP R2,#0        ;check r2 ,find the phase interrupts occurred
    BEQ __11
    MOVS R1,#0x2f    ;in interrupt phase ,r1=47
    LSLS R0,R0,#8
    ORRS R0,R0,R1    ;make r1 available
    B __12
__11
    MOVS R1,#0        ;in initialization phase r1=0
    LSLS R0,R0,#8
    ORRS R0,R0,R1    ;make r1 available
__12
    ADD   R3, R13, #32                 ; #32: skip [R0-R3,R12,LR,PC,xPSR]
    PUSH  {R3}                         ; push [SAVED_SP]: MSP+32 = Stack pointer in MSP before entering the exception
    MRS   R12, PRIMASK                   
    MOV R3,R12
    PUSH {R3}
    MOV R3,R11
    PUSH {R3}
    MOV R3,R10
    PUSH {R3}
    MOV R3,R9
    PUSH {R3}
    MOV R3,R8
    PUSH {R3}
    PUSH {R7}
    PUSH {R6}
    PUSH {R5}
    PUSH {R4}                     ; push R4-R11,PRIMASK to MSP in certain order

    B     _handleEntry

    ;/**
    ; * Check whether during the initialization phase.
    ; * If g_bTaskScheduled == 0, it is in the initialization phase.
    ; */
_whether_in_initialization
    LDR  R1, =g_bTaskScheduled
    LDR  R1, [R1]
    MOVS R3,#1
    TST  R1,R3
    BEQ  _ExcInMSP                     ; initialization phase use MSP

    ;/**
    ; * Before executing _handleEntry, MSP is as follows.
    ; * MSP:
    ; *                                                                              High addr--->|
    ; * +--------------------------------------------------------------------------------+---------
    ; *                                  | R4-R11,PRIMASK,TASK_SP | R0-R3,R12,LR,PC,xPSR |
    ; * +--------------------------------------------------------------------------------+---------
    ; *                           R13--->|                        |<---      #32     --->|<---Initial R13
    ; *                                                           |  (copied from PSP)   |
    ; *                                                           |<---R2(no use)
    ; *
    ; * NOTE: stack frame: R0-R3,R12,LR,PC,xPSR.
    ; */
_NoFloatInPsp
    MOVS R1,#1 ;in task ,r1 = 1
    LSLS R0,R0,#8
    ORRS R0,R0,R1    ;make r1 available

    MOV   R1, R13
    SUB   R13, #32                     ; #32: MSP reserved, used to store stack frame in PSP

    MRS   R3, PSP

    ADDS   R3, R3, #32                 ; PSP+32 = Stack pointer of the task before entering the exception

    PUSH  {R3}                        ; push task SP to MSP
    MRS R3,PRIMASK
    PUSH {R3}
    MOV R3,R11
    PUSH {R3}
    MOV R3,R10
    PUSH {R3}
    MOV R3,R9
    PUSH {R3}
    MOV R3,R8
    PUSH {R3}
    PUSH {R7}
    PUSH {R6}
    PUSH {R5}
    PUSH {R4}
    
    MRS R3,PSP
    ;/* Copy stack frame from the stack of the current running task to MSP */
    LDMIA R3!, {R4-R7}
    SUBS R1,#0X20
    STMIA R1!, {R4-R7}
    LDMIA R3!, {R4-R7}
    STMIA R1!, {R4-R7}

    ;/**
    ; * _handleEntry: Call osExcHandleEntry
    ; * param1: R0 --- type num and phase
    ; * param2: R1 --- the sp being used(in Cortex-M0,it is only for debug )
    ; * param3: R2 --- external interrupt ID
    ; * param4: R3 --- Point to the top of the stack(R4 or S16) that the exception stack frame in MSP.
    ; */
_handleEntry
    MOV R3, R13
    MOV R1,R13
    CPSID I
    LDR R4,=osExcHandleEntry
    MOV PC,R4
    NOP
    ALIGN
    END