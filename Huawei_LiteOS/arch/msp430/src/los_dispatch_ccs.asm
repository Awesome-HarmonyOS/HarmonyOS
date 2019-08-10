;/*----------------------------------------------------------------------------
; * Copyright (c) <2013-2018>, <Huawei Technologies Co., Ltd>
; * All rights reserved.
; * Redistribution and use in source and binary forms, with or without modification,
; * are permitted provided that the following conditions are met:
; * 1. Redistributions of source code must retain the above copyright notice, this list of
; * conditions and the following disclaimer.
; * 2. Redistributions in binary form must reproduce the above copyright notice, this list
; * of conditions and the following disclaimer in the documentation and/or other materials
; * provided with the distribution.
; * 3. Neither the name of the copyright holder nor the names of its contributors may be used
; * to endorse or promote products derived from this software without specific prior written
; * permission.
; * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
; * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
; * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
; * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
; * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
; * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
; * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
; * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
; * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
; * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
; * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
; *---------------------------------------------------------------------------*/
;/*----------------------------------------------------------------------------
; * Notice of Export Control Law
; * ===============================================
; * Huawei LiteOS may be subject to applicable export control laws and regulations, which might
; * include those applicable to Huawei LiteOS of U.S. and the country in which you are located.
; * Import, export and usage of Huawei LiteOS in any manner by you shall be in compliance with such
; * applicable export control laws and regulations.
; *---------------------------------------------------------------------------*/

        .cdecls C, LIST, "msp430.h"         ; Include device header file
        .cdecls C, LIST, "ccsmacros.h"      ; Include macros used ccs header file

        .data
        .global _los_heap_start
        .global _los_heap_end

        .asg    256,    LOSCFG_MSP430_IRQ_SIZE

        .align  4
_los_heap_start:
        .ulong  __HEAP_START
_los_heap_end:
        .ulong  __STACK_END - LOSCFG_MSP430_IRQ_SIZE

        .text
        .global LOS_IntLock
        .global LOS_IntUnLock
        .global LOS_IntRestore
        .global LOS_StartToRun
        .global osSchedule
        .ref    g_bTaskScheduled
        .ref    g_stLosTask
        .ref    g_int_cnt
        .ref    osTaskSwitchCheck
        .ref    g_usLosTaskLock
        .ref    osHwiDispatch
        .retain
        .retainrefs

        .asg    0x10, TASK_STATUS_RUNNING

;-------------------------------------------------------------------------------
;       irq vectors
;-------------------------------------------------------------------------------

        .asg    0, irq

        .loop

        .if irq < 10
        .sect   ".int0:irq:"
        .else
        .sect   ".int:irq:"
        .endif

        .short  irq_stub:irq:

        .break ($symcmp (""".int:irq:""", SYSNMI_VECTOR) = 0)

        .eval   irq + 1, irq
        .endloop

;-------------------------------------------------------------------------------
;       irq entry stubs
;-------------------------------------------------------------------------------

        .sect   ".text:_isr"

        .asg    irq + 1, irqs
        .asg    0, irq
irq_stubs:
        .loop   irqs
irq_stub:irq:
        FCALL   #irq_handler

        .eval   irq + 1, irq
        .endloop

        .unasg  irq
        .unasg  irqs

irq_handler:

        ;
        ; now the stack:
        ;
        ; +-------------------------+----+----+
        ; | irq_stubs + irq * 4 + 4 | SR | PC |
        ; +-------------------------+----+----+
        ;

        XPUSH   r14
        XPUSH   r13
        XPUSH   r12
        XMOV    (3 * REG_SIZE)(sp), r12
        XMOV    r15, (3 * REG_SIZE)(sp)

        ;
        ; now the stack:
        ;
        ; +-----+-----+-----+-----+----+----+
        ; | R12 | R13 | R14 | R15 | SR | PC |
        ; +-----+-----+-----+-----+----+----+
        ;
        ; now r12 = irq_stubs + irq * 4 + 4,
        ;
        ; ((r12 - (irq_stubs + 4)) / 4) is the irq number
        ; ((r12 - (irq_stubs + 4)) / 2) is the vector number
        ;

        INC.B   &g_int_cnt
        CMP.B   #1, &g_int_cnt
        JNE     already_on_irq_stack

        XMOV    sp, &__STACK_END - REG_SIZE
        XMOV    #__STACK_END - REG_SIZE, sp
already_on_irq_stack:

        SUB.W   #irq_stubs + 4, r12             ; irq_stubs at CODE16
        RRA.w   r12
        RRA.w   r12

        FCALL   #osHwiDispatch                  ; interrupt can be enabld in osHwiDispatch

        DINT
        NOP                                     ; required by architecture

        DEC.B   &g_int_cnt
        CMP.B   #0, &g_int_cnt
        JNE     _rfi

        XMOV    0(sp), sp

        CMP.W   #0, &g_usLosTaskLock
        JNE     _rfi

        PCMP    &g_stLosTask + PTR_SIZE, &g_stLosTask
        JEQ     _rfi            ; need schedule, save context

        XPUSHM  #8, r11         ; save r4-r11

_switch_new:

        PMOV    &g_stLosTask, r12
        PMOV    sp, 0(r12)
        BIC.W   #TASK_STATUS_RUNNING, PTR_SIZE(r12)

        FCALL   #osTaskSwitchCheck

_load_new:
        PMOV    &g_stLosTask + PTR_SIZE, r12
        PMOV    r12, &g_stLosTask
        PMOV    0(r12), sp
        BIS.W   #TASK_STATUS_RUNNING, PTR_SIZE(r12)
        XPOPM   #8, r11

_rfi:
        XPOPM   #4, r15
        RETI

LOS_StartToRun:
        DINT
        NOP                     ; required by architecture
        MOV.W   #1, &g_bTaskScheduled
        JMP     _load_new

osSchedule:
        MOV.W   sr, r12
        DINT
        NOP                     ; required by architecture
        PCMP    &g_stLosTask + PTR_SIZE, &g_stLosTask
        JEQ     _ret

        .if __LARGE_CODE_MODEL__ = 1

        ;
        ; fake ra for calla as the same with interrupt:
        ;
        ; +---------+---------+    +----+----+
        ; | ra00:15 | ra16:19 | -> | sr | ra |
        ; +---------+---------+    +----+----+
        ;
        ; ra16:19 in sr16:19
        ;

        MOV.W   2(sp), r13
        RLA     r13
        RLA     r13
        RLA     r13
        RLA     r13
        SWPB    r13
        BIS.W   r13, r12

        MOV.W   0(sp), 2(sp)
        MOV.W   r12, 0(sp)

        .else

        PUSH.W  r12

        .endif

        PSUB    #4 * REG_SIZE, sp   ; reserve space for r12-r15, needless save

        XPUSHM  #8, r11             ; save r4-r11

        JMP     _switch_new

_ret:
        NOP                         ; required by architecture
        MOV.W   r12, sr
        NOP                         ; required by architecture
        FRET

LOS_IntLock:
        MOV.W   sr, r12
        AND.W   #GIE, r12
        DINT
        NOP                         ; required by architecture
        FRET

LOS_IntUnLock:
        MOV.W   sr, r12
        AND.W   #GIE, r12
        NOP                         ; required by architecture
        EINT
        NOP
        FRET

LOS_IntRestore:
        AND.W   #GIE, r12
        NOP                         ; required by architecture
        BIS.W   r12, sr
        NOP                         ; required by architecture
        FRET

;-------------------------------------------------------------------------------
;       System HEAP
;-------------------------------------------------------------------------------

        .sect   .sysmem
        .align  8
__HEAP_START:

;-------------------------------------------------------------------------------
;       Stack Pointer definition
;-------------------------------------------------------------------------------

        .global __STACK_END
        .sect   .stack

        .end

