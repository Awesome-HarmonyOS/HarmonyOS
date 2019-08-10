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
    EXPORT  SVC_Handler

;****************************************************************************************
;                                  EXTERN PARAMETERS
;****************************************************************************************

    IMPORT osExcHandleEntry
    IMPORT g_uwExcTbl
    IMPORT g_bTaskScheduled

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
    MOV  R0, #OS_EXC_CAUSE_NMI
    MOV  R1, #0
    B  osExcDispatch

;****************************************************************************************
; Function:
;        VOID HardFault_Handler(VOID);
; Description:
;        HardFault Handler.
;****************************************************************************************
HardFault_Handler
    ;/**
    ; * Check HardFault state register.
    ; *
    ; * HFSR:
    ; * +----------+--------+--------+--------+-------+
    ; * |    31    |   30   | 29 - 2 |    1   |   0   |
    ; * +----------+--------+--------+--------+-------+
    ; * | DEBUGEVT | FORCED |   --   | VECTBL |   --  |
    ; * +----------+--------+--------+--------+-------+
    ; */
    MOV  R0, #OS_EXC_CAUSE_HARDFAULT
    LDR  R2, =OS_NVIC_HFSR
    LDR  R2, [R2]

    ;/**
    ; * Check whether HardFault are triggered by debugging events.
    ; * Before executing instruction 'BNE osExcDispatch', the value of R0 is as follows.
    ; * < R0 >:
    ; * +----------------------------------------+-------------+------------------------+
    ; * |                 31-16                  |    15-8     |          7-0           |
    ; * +----------------------------------------+-------------+------------------------+
    ; * |                  ---                   | HF_DEBUGEVT | OS_EXC_CAUSE_HARDFAULT |
    ; * +----------------------------------------+-------------+------------------------+
    ; * < R1 >: invalid
    ; */
    MOV  R1, #HF_DEBUGEVT
    ORR  R0, R0, R1, LSL #0x8
    TST  R2, #0x80000000
    BNE  osExcDispatch                 ; DEBUGEVT

    ;/**
    ; * Check whether HardFault is caused by the failure of the fetch vector.
    ; * Before executing instruction 'BNE osExcDispatch', the value of R0 is as follows.
    ; * < R0 >:
    ; * +----------------------------------------+-------------+------------------------+
    ; * |                 31-16                  |    15-8     |          7-0           |
    ; * +----------------------------------------+-------------+------------------------+
    ; * |                  ---                   |  HF_VECTBL  | OS_EXC_CAUSE_HARDFAULT |
    ; * +----------------------------------------+-------------+------------------------+
    ; * < R1 >: invalid
    ; */
    AND  R0, R0, #0x000000FF
    MOV  R1, #HF_VECTBL
    ORR  R0, R0, R1, LSL #0x8
    TST  R2, #0x00000002
    BNE  osExcDispatch                  ; VECTBL

    ;/**
    ; * If it`s not DEBUGEVT and VECTBL, that is FORCED, then read the CFSR register to
    ; * check BusFault, MemFault and UsageFault.
    ; * R0: OS_EXC_CAUSE_HARDFAULT
    ; *
    ; * CFSR:
    ; * +----------------+--------+--------+
    ; * |      31-16     |  15-8  |  7-0   |
    ; * +----------------+--------+--------+
    ; * |      UFSR      |  BFSR  |  MFSR  |
    ; * +----------------+--------+--------+
    ; */
    AND  R0, R0, #0x000000FF

    LDR  R2, =OS_NVIC_CFSR
    LDR  R2, [R2]

    TST  R2, #0x8000                   ; BFSR->BFARVALID
    BNE  _HFBusFault                   ; BusFault

    TST  R2, #0x80                     ; MFSR->MMARVALID
    BNE  _HFMemFault                   ; MemFault

    ;/**
    ; * BFARVALID and MMARVALID flag both invalid.
    ; * R12: 0 --- The error address is invalid.
    ; */
    MOV  R12, #0
    B    osHFExcCommonBMU

    ;/**
    ; * BFARVALID flag valid, read BFAR register.
    ; * R1 : BFAR value --- The address value of a bus error.
    ; * R12: The error address is valid.
    ; */
_HFBusFault
    LDR  R1, =OS_NVIC_BFAR
    LDR  R1, [R1]
    MOV  R12, #FLAG_ADDR_VALID
    B    osHFExcCommonBMU

    ;/**
    ; * MMARVALID flag valid, read MMFAR register.
    ; * R1 : MMFAR value --- The address value of memory management error.
    ; * R12: The error address is valid.
    ; */
_HFMemFault
    LDR  R1, =OS_NVIC_MMFAR
    LDR  R1, [R1]
    MOV  R12, #FLAG_ADDR_VALID

    ;/**
    ; * osHFExcCommonBMU: --- Get specific error status from table g_uwExcTbl, stored in R0.
    ; * Before executing instruction 'B osExcDispatch', the value of R0 is as follows.
    ; * < R0 >:
    ; * +-------------------+-----------------+------------------+------------------------+
    ; * |       31-17       |        16       |        15-8      |          7-0           |
    ; * +-------------------+-----------------+------------------+------------------------+
    ; * |        ---        | FLAG_ADDR_VALID | Error state code | OS_EXC_CAUSE_HARDFAULT |
    ; * |                   |        or       |       in         |                        |
    ; * |                   |   0(invalid)    | table g_uwExcTbl |                        |
    ; * +-------------------+-----------------+------------------+------------------------+
    ; * < R1 >: The value of BFAR or MMFAR if the bit16(FLAG_ADDR_VALID) of R0 is set to 1,
    ; *         else invalid.
    ; */
osHFExcCommonBMU
    CLZ  R2, R2
    LDR  R3, =g_uwExcTbl
    ADD  R3, R3, R2
    LDRB R2, [R3]
    ORR  R0, R0, R2, LSL #0x8
    ORR  R0, R0, R12
    B    osExcDispatch

;****************************************************************************************
; Function:
;        VOID SVC_Handler(VOID);
; Description:
;        SVC Handler.
;****************************************************************************************
SVC_Handler
    TST   LR, #0x4                     ; EXC_RETURN[b2] --- PSP or MSP
    ITE   EQ
    MRSEQ R0, MSP
    MRSNE R0, PSP
    LDR   R1, [R0, #24]                ; The PC value in the stack frame
    LDRB  R0, [R1, #-2]                ; R0: The number of SVC (0 - 255)
    MOV   R1, #0
    ;B     osExcDispatch
_SvcLoop
    B     _SvcLoop

;****************************************************************************************
; Function:
;        VOID BusFault_Handler(VOID);
; Description:
;        BusFault Handler.
;****************************************************************************************
BusFault_Handler
    LDR  R0, =OS_NVIC_CFSR
    LDR  R0, [R0]
    LDR  R2, =OS_EXC_CAUSE_BUSFAULT

    TST  R0, #0x8000                   ; BFSR->BFARVALID
    BEQ  _ExcBusNoADDR

    LDR  R1, =OS_NVIC_BFAR
    LDR  R1, [R1]                      ; R1:  The value of BFAR
    MOV  R12, #FLAG_ADDR_VALID         ; R12: BusFault addr valid
    AND  R0, R0, #0x3F00               ; R0:  Reserved the b13-b8 of the BFSR
    B    osExcCommonBMU

_ExcBusNoADDR
    MOV  R12, #0                       ; R12: BusFault addr invalid
    AND  R0, R0, #0x3F00               ; R0:  Reserved the b13-b8 of the BFSR
    B    osExcCommonBMU

;****************************************************************************************
; Function:
;        VOID MemManage_Handler(VOID);
; Description:
;        MemManage Handler.
;****************************************************************************************
MemManage_Handler
    LDR  R0, =OS_NVIC_CFSR
    LDR  R0, [R0]
    LDR  R2, =OS_EXC_CAUSE_MEMFAULT

    TST  R0, #0x80                     ; MFSR->MMARVALID
    BEQ  _ExcMemNoADDR

    LDR  R1, =OS_NVIC_MMFAR
    LDR  R1, [R1]                      ; R1:  The value of MMFAR
    MOV  R12, #FLAG_ADDR_VALID         ; R12: MemFault addr valid
    AND  R0, R0, #0x3B                 ; R0:  Reserved the b5-b0 of the MFSR
    B    osExcCommonBMU

_ExcMemNoADDR
    MOV  R12, #0                       ; R12: MemFault addr invalid
    AND  R0, R0, #0x3B                 ; R0:  Reserved the b5-b0 of the MFSR
    B    osExcCommonBMU

;****************************************************************************************
; Function:
;        VOID UsageFault_Handler(VOID);
; Description:
;        UsageFault Handler.
;****************************************************************************************
UsageFault_Handler
	
    LDR  R0, =OS_NVIC_CFSR
    LDR  R0, [R0]
    LDR  R2, =OS_EXC_CAUSE_USAGEFAULT

    MOV  R1, #0x030F
    LSL  R1, R1, #16
    AND  R0, R0, R1                    ; R0:  reserved UFSR
    MOV  R12, #0                       ; R12: Fault addr invalid
	


    ;/**
    ; * osExcCommonBMU: BusFault_Handler,MemManage_Handler and UsageFault_Handler share.
    ; * Get specific error status from table g_uwExcTbl, stored in R0.
    ; * Before executing osExcDispatch, the value of R0 is as follows.
    ; * < R0 >:
    ; * +-------------------+-----------------+------------------+------------------------+
    ; * |       31-17       |        16       |        15-8      |          7-0           |
    ; * +-------------------+-----------------+------------------+------------------------+
    ; * |        ---        | FLAG_ADDR_VALID | Error state code |OS_EXC_CAUSE_BUSFAULT or|
    ; * |                   |        or       |       in         |OS_EXC_CAUSE_MEMFAULT or|
    ; * |                   |   0(invalid)    | table g_uwExcTbl |OS_EXC_CAUSE_USAGEFAULT |
    ; * +-------------------+-----------------+------------------+------------------------+
    ; * < R1 >: The value of BFAR or MMFAR if the bit16(FLAG_ADDR_VALID) of R0 is set to 1,
    ; *         else invalid.
    ; */
osExcCommonBMU
    CLZ  R0, R0
    LDR  R3, =g_uwExcTbl
    ADD  R3, R3, R0
    LDRB R0, [R3]
    LSL  R0, R0, #0x8
    ORR  R0, R0, R2
    ORR  R0, R0, R12
	
	;MRS R2,PSP		;PSP-->>37D8

    ;****************************************************************************************
    ; osExcDispatch: NMI_Handler, HardFault_Handler, SVC_Handler, BusFault_Handler, MemManage_Handler,
    ;                  UsageFault_Handler sharing.
    ;****************************************************************************************

    ;/**
    ; * When executing osExcDispatch, R0, R1 will be used.
    ; * The possible values of R0 and R1 are as follows.
    ; *
    ; * < R0 >:
    ; * +----------------+-----------------+---------------------+------------------------+
    ; * |     31-17      |        16       |         15-8        |          7-0           |
    ; * +----------------+-----------------+---------------------+------------------------+
    ; * |                | FLAG_ADDR_VALID | Error state code in | OS_EXC_CAUSE_HARDFAULT |
    ; * |      ---       |       or        | table g_uwExcTbl    |or OS_EXC_CAUSE_MEMFAULT|
    ; * |                |   0(invalid)    | or      HF_DEBUGEVT |or OS_EXC_CAUSE_BUSFAULT|
    ; * |                |                 | or      HF_VECTBL   |or OS_EXC_CAUSE_NMI  or |
    ; * |                |                 |                     | OS_EXC_CAUSE_USAGEFAULT|
    ; * +----------------+-----------------+---------------------+------------------------+
    ; * b17: FLAG_HWI_ACTIVE
    ; * b28: FLAG_NO_FLOAT
    ; * NOTE: b17 and b28 will be set later.
    ; *
    ; * < R1 >:
    ; * If the bit16 of R0 is 1, then R1 is the value of BFAR or MMFAR, otherwise the
    ; * value in R1 is invalid.
    ; *
    ; */
osExcDispatch
    LDR   R2, =OS_NVIC_ACT_BASE
    MOV   R12, #8                      ; #8: externel interrupt active check loop counter(#0 - #239)
	


_hwiActiveCheck
    LDR   R3, [R2]                     ; R3 store the value of externel interrupt active status
    CMP   R3, #0
    BEQ   _hwiActiveCheckNext

    ;/**
    ; * Exception occured in external interrupt.
    ; */
    ORR   R0, R0, #FLAG_HWI_ACTIVE     ; R0[b17] = 1, externel interrupt active valid    &&&&&&&&&&
    RBIT  R2, R3                       ; bit reversal
    CLZ   R2, R2
    RSB   R12, R12, #8                 ; R12 = 8 - R12
    ADD   R2, R2, R12, LSL #5          ; R2: external interrupt number as uwPid

    ;/**
    ; * Interrupts and initialization phase always use MSP.
    ; */
_ExcInMSP
    ;TST   LR, #0x10                    ; EXC_RETURN[b4] --- FPU(0) or without FPU(1)
    ;BNE   _NoFloatInMsp

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
    ADD   R3, R13, #32                 ; #32: skip [R0-R3,R12,LR,PC,xPSR]
    PUSH  {R3}                         ; push [SAVED_SP]: MSP+32 = Stack pointer in MSP before entering the exception
    MRS   R12, PRIMASK
    PUSH  {R4-R12}                     ; push R4-R11,PRIMASK to MSP

	ORR   R0, R0, #FLAG_NO_FLOAT       ; R0[b28] = 1, no FPU    &&&&&&&&&&
    B     _handleEntry

_hwiActiveCheckNext
    ADD   R2, R2, #4                   ; next NVIC ACT ADDR
    SUBS  R12, R12, #1
    BNE   _hwiActiveCheck

    ;/**
    ; * Not in externel interrupt, check whether it is SysTick or PendSV.
    ; */

    LDR   R2, =OS_NVIC_SHCSRS
    LDRH  R2,[R2]
    LDR   R3,=OS_NVIC_SHCSR_MASK
    AND   R2, R2, R3
    CMP   R2, #0
    BNE   _ExcInMSP                    ; SysTick or PendSV active

    ;/**
    ; * Check whether an exception occurs during the initialization phase.
    ; * If g_bTaskScheduled == 0, it is in the initialization phase.
    ; */
    LDR  R2, =g_bTaskScheduled
    LDR  R2, [R2]
    TST  R2, #1
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
    MOV   R2, R13
    SUB   R13, #32                     ; #32: MSP reserved, used to store stack frame in PSP

    MRS   R3, PSP
    ADD   R12, R3, #32                 ; PSP+32 = Stack pointer of the task before entering the exception
	
	PUSH  {R12}                        ; push task SP to MSP
    MRS   R12, PRIMASK
    PUSH  {R4-R12}                     ; push R4-R11,PRIMASK of the current running task to MSP

    ;/* Copy stack frame from the stack of the current running task to MSP */
    LDMFD R3, {R4-R11}                 ; restore stack frame of PSP to R4-R11
    STMFD R2!, {R4-R11}                ; save stack frame to MSP

    ORR   R0, R0, #FLAG_NO_FLOAT       ; R0[b28] = 1, no FPU    &&&&&&&&&&

    ;/**
    ; * _handleEntry: Call osExcHandleEntry
    ; * param1: R0 --- b28:    FLAG_NO_FLOAT.
    ; *                b17:    FLAG_HWI_ACTIVE.
    ; *                b16:    FLAG_ADDR_VALID.
    ; *                b15-b8: Error state code in table g_uwExcTbl or HF_DEBUGEVT or HF_VECTBL.
    ; *                b7-b0:  OS_EXC_CAUSE_HARDFAULT or OS_EXC_CAUSE_NMI or OS_EXC_CAUSE_MEMFAULT
    ; *                        or OS_EXC_CAUSE_BUSFAULT or OS_EXC_CAUSE_USAGEFAULT.
    ; * param2: R1 --- The value of BFAR or MMFAR if R0[b16] = 1, otherwise invalid.
    ; * param3: R2 --- external interrupt number(0-239) if R0[b17] = 1, otherwise invalid.
    ; * param4: R3 --- Point to the top of the stack(R4 or S16) that the exception stack frame in MSP.
    ; */
_handleEntry
    MOV R3, R13
    CPSID I
    CPSID F
    B  osExcHandleEntry

    NOP
    ALIGN
    END