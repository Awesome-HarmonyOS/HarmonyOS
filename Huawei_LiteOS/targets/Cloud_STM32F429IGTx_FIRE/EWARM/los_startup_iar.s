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

    MODULE  ?cstartup

    ;; Forward declaration of sections.
    SECTION CSTACK:DATA:NOROOT(3)
    SECTION LOS_HEAP:DATA:NOROOT(3)

    SECTION .text:CODE:NOROOT(2)
    PUBLIC  __LOS_HEAP_ADDR_START__
    PUBLIC  __LOS_HEAP_ADDR_END__
    EXTERN  __ICFEDIT_region_RAM_end__
    DATA
__LOS_HEAP_ADDR_START__
    DCD     sfb(LOS_HEAP)
__LOS_HEAP_ADDR_END__
    DCD     __ICFEDIT_region_RAM_end__

    SECTION .intvec:CODE:NOROOT(2)
    
    EXTERN  __iar_program_start
    EXTERN  SystemInit
    EXPORT  Reset_Handler
    PUBLIC  __vector_table

    DATA
__vector_table
    DCD     sfe(CSTACK)
    DCD     Reset_Handler   ; Reset Handler
    DCD     0               ; NMI Handler
    DCD     0               ; Hard Fault Handler
    DCD     0               ; MPU Fault Handler
    DCD     0               ; Bus Fault Handler
    DCD     0               ; Usage Fault Handler
    DCD     0               ; Reserved
    DCD     0               ; Reserved
    DCD     0               ; Reserved
    DCD     0               ; Reserved
    DCD     0               ; SVCall Handler
    DCD     0               ; Debug Monitor Handler
    DCD     0               ; Reserved
    DCD     0               ; PendSV Handler
    DCD     0               ; SysTick_Handler

    THUMB
    PUBWEAK Reset_Handler
    SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler
    LDR     R0, =SystemInit
    BLX     R0
    LDR     R0, =__iar_program_start
    BX      R0
    END

