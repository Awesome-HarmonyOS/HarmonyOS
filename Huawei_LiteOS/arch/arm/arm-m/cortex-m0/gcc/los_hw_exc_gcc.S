/*----------------------------------------------------------------------------
  Copyright (c) <2013-2015>, <Huawei Technologies Co., Ltd>
  All rights reserved.
  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice, this list of
  conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice, this list
  of conditions and the following disclaimer in the documentation and/or other materials
  provided with the distribution.
  3. Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific prior written
  permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS
  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
----------------------------------------------------------------------------
  Notice of Export Control Law
  ===============================================
  Huawei LiteOS may be subject to applicable export control laws and regulations, which might
  include those applicable to Huawei LiteOS of U.S. and the country in which you are located.
  Import, export and usage of Huawei LiteOS in any manner by you shall be in compliance with such
  applicable export control laws and regulations.
 ---------------------------------------------------------------------------*/

/****************************************************************************************
                                  CODE GENERATION DIRECTIVES
****************************************************************************************/

    .section .text
    .thumb
    .syntax unified
    .arch armv6-m

/****************************************************************************************
                                  EXPORT FUNCTIONS
****************************************************************************************/

    .global NMI_Handler
    .global HardFault_Handler
    .global MemManage_Handler
    .global BusFault_Handler
    .global UsageFault_Handler



/****************************************************************************************
                                  EXTERN PARAMETERS
****************************************************************************************/

    .extern osExcHandleEntry
    .extern g_uwExcTbl
    .extern g_bTaskScheduled
    .extern it_is_memManangeFault
    .extern it_is_busFault
    .extern it_is_usageFault

/****************************************************************************************
                                  EQU
****************************************************************************************/

.equ OS_EXC_CAUSE_NMI,               18
.equ OS_EXC_CAUSE_HARDFAULT,         19
.equ OS_EXC_CAUSE_MEMFAULT,          20
.equ OS_EXC_CAUSE_BUSFAULT,          21
.equ OS_EXC_CAUSE_USAGEFAULT,        22
.equ OS_EXC_CAUSE_SVC,               23

.equ HF_DEBUGEVT,                    24
.equ HF_VECTBL,                      25

.equ FLAG_ADDR_VALID,                0x10000        /*bit 16*/
.equ FLAG_HWI_ACTIVE,                0x20000        /*bit 17*/
.equ FLAG_NO_FLOAT,                  0x10000000     /*bit 28*/

.equ OS_NVIC_CFSR,                   0xE000ED28     /*include BusFault/MemFault/UsageFault State Regeister*/
.equ OS_NVIC_HFSR,                   0xE000ED2C     /*HardFault State Regeister*/
.equ OS_NVIC_BFAR,                   0xE000ED38
.equ OS_NVIC_MMFAR,                  0xE000ED34
.equ OS_NVIC_ACT_BASE,               0xE000E300
.equ OS_NVIC_SHCSRS,                 0xE000ED24
.equ OS_NVIC_SHCSR_MASK,             0xC00          /*SYSTICKACT and PENDSVACT*/

.equ ADDR_SETPEND,                    0XE000E200        /*the addr of setpend register*/
.equ ADDR_CLRPEND,                    0XE000E280        /*the addr of clrpend register*/
.equ ADDR_SETENA,                   0XE000E100
.equ EXC_RETURN_NEST,                0XFFFFFFF1      /*return to some irq_handler*/
/****************************************************************************************
 Function:
        VOID NMI_Handler(VOID)
 Description:
        NMI Handler.
****************************************************************************************/
    .type NMI_Handler, %function
NMI_Handler:
    /**
     * Before executing instruction 'B osExcDispatch', the value of R0 is as follows.
     * < R0 >:
     * +------------------------------------------------------+------------------------+
     * |                          31-8                        |          7-0           |
     * +------------------------------------------------------+------------------------+
     * |                          ---                         |    OS_EXC_CAUSE_NMI    |
     * +------------------------------------------------------+------------------------+
     * < R1 >: invalid
     */
    MOVS  R0, #OS_EXC_CAUSE_NMI
    MOVS  R1, #0
    B  osExcDispatch

/****************************************************************************************
 Function:
        VOID HardFault_Handler(VOID)
 Description:
        HardFault Handler.
****************************************************************************************/
    .type HardFault_Handler, %function
HardFault_Handler:
    /**
     * save fault type to R0
     */
    MOVS  R0, #OS_EXC_CAUSE_HARDFAULT
    B  osExcDispatch


    /**
     * When executing osExcDispatch,  R2 will be used.
     */
osExcDispatch:

    LDR  R2, =ADDR_SETPEND
    LDR  R2, [R2]
    LDR  R3,=0X40
    LSLS R3,R3,#0X18
    SUBS R3,R3,#1  /*R3 =0011 1111 1111 1111 1111 1111 1111 1111 filter the highest active 2 bits error*/
    ANDS  R2,R2,R3
    LDR  R3,=ADDR_SETENA
    LDR  R3,[R3]
    ANDS R2,R2,R3  /*if an interrupt is enabled and active ,it is a nomal exception ,otherwise,it should be ignored*/

    MOV  R1,LR
    LDR  R3, =EXC_RETURN_NEST
    CMP  R1,R3
    BEQ  __in_interrupt  /*jump when they are equal,if LR equals it,they are in some interrupt*/
    B    __second_judge
__in_interrupt:
    CMP R2,#0
    BEQ __set_R2_33
    B   _NoFloatInMsp

__set_R2_33:
    MOVS R2,#0x21  /*if in interrupt,but the setpendAddr is 0,sth wrong happened, set id to 33*/
    B _NoFloatInMsp

__second_judge:
    CMP R2,#0
    BEQ _whether_in_initialization                /*jump when no external interrupt occurred*/

    /**
     * Interrupts and initialization phase always use MSP.
     */
_ExcInMSP:
    /**
     * Before executing instruction 'B _handleEntry', MSP is as follows.
     * MSP:
     *                                                                              High addr--->|
     * +--------------------------------------------------------------------------------+---------
     *                                 | R4-R11,PRIMASK,SAVED_SP | R0-R3,R12,LR,PC,xPSR |
     * +--------------------------------------------------------------------------------+---------
     *                          R13--->|          Initial R13--->|<---      #32     --->|<---SAVED_SP
     *                                                           |   (CPU auto saved)   |
     *
     */
_NoFloatInMsp:
    CMP R2,#0        /*check r2 ,find the phase interrupts occurred*/
    BEQ __11
    MOVS R1,#0x2f    /*in interrupt phase ,r1=47*/
    LSLS R0,R0,#8
    ORRS R0,R0,R1    /*make r1 available*/
    B __12
__11:
    MOVS R1,#0        /*in initialization phase r1=0*/
    LSLS R0,R0,#8
    ORRS R0,R0,R1    /* make r1 available */
__12:
    ADD   R3, R13, #32                  /*#32: skip [R0-R3,R12,LR,PC,xPSR]*/
    PUSH  {R3}                          /*push [SAVED_SP]: MSP+32 = Stack pointer in MSP before entering the exception*/
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
    PUSH {R4}                      /*push R4-R11,PRIMASK to MSP in certain order*/

    B     _handleEntry

    /**
     * Check whether during the initialization phase.
     * If g_bTaskScheduled == 0, it is in the initialization phase.
     */
_whether_in_initialization:
    LDR  R1, =g_bTaskScheduled
    LDR  R1, [R1]
    MOVS R3,#1
    TST  R1,R3
    BEQ  _ExcInMSP                      /*initialization phase use MSP*/

    /**
     * Before executing _handleEntry, MSP is as follows.
     * MSP:
     *                                                                              High addr--->|
     * +--------------------------------------------------------------------------------+---------
     *                                  | R4-R11,PRIMASK,TASK_SP | R0-R3,R12,LR,PC,xPSR |
     * +--------------------------------------------------------------------------------+---------
     *                           R13--->|                        |<---      #32     --->|<---Initial R13
     *                                                           |  (copied from PSP)   |
     *                                                           |<---R2(no use)
     *
     * NOTE: stack frame: R0-R3,R12,LR,PC,xPSR.
     */
_NoFloatInPsp:
    MOVS R1,#1         /*in task ,r1 = 1 */
    LSLS R0,R0,#8
    ORRS R0,R0,R1       /*make r1 available*/

    MOV   R1, R13
    SUB   R13, #32                      /*#32: MSP reserved, used to store stack frame in PSP*/

    MRS   R3, PSP

    ADDS   R3, R3, #32                  /*PSP+32 = Stack pointer of the task before entering the exception*/

    PUSH  {R3}                          /*push task SP to MSP*/
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
    /* Copy stack frame from the stack of the current running task to MSP */
    LDMIA R3!, {R4-R7}
    SUBS R1,#0X20
    STMIA R1!, {R4-R7}
    LDMIA R3!, {R4-R7}
    STMIA R1!, {R4-R7}

    /**
     * _handleEntry: Call osExcHandleEntry
     * param1: R0 --- type num and phase
     * param2: R1 --- the sp being used(in Cortex-M0,it is only for debug )
     * param3: R2 --- external interrupt ID
     * param4: R3 --- Point to the top of the stack(R4 or S16) that the exception stack frame in MSP.
     */
_handleEntry:
    MOV R3, R13
    MOV R1,R13
    CPSID I
    LDR R4,=osExcHandleEntry
    MOV PC,R4
    NOP
