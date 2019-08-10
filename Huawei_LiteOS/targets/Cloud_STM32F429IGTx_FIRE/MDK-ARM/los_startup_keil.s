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

LOS_Heap_Min_Size   EQU     0x400

                AREA    LOS_HEAP, NOINIT, READWRITE, ALIGN=3
__los_heap_base
LOS_Heap_Mem    SPACE   LOS_Heap_Min_Size


                AREA    LOS_HEAP_INFO, DATA, READONLY, ALIGN=2
                IMPORT  |Image$$ARM_LIB_STACKHEAP$$ZI$$Base|
                EXPORT  __LOS_HEAP_ADDR_START__
                EXPORT  __LOS_HEAP_ADDR_END__
__LOS_HEAP_ADDR_START__
                DCD     __los_heap_base
__LOS_HEAP_ADDR_END__
                DCD     |Image$$ARM_LIB_STACKHEAP$$ZI$$Base| - 1


                PRESERVE8

                AREA    RESET, CODE, READONLY
                THUMB

                IMPORT  ||Image$$ARM_LIB_STACKHEAP$$ZI$$Limit||

                EXPORT  _BootVectors
                EXPORT  Reset_Handler

_BootVectors
                DCD     ||Image$$ARM_LIB_STACKHEAP$$ZI$$Limit||
                DCD     Reset_Handler


Reset_Handler
                LDR.W   R0, =0xE000ED88
                LDR     R1, [R0]
                ORR     R1, R1, #(0xF << 20)
                STR     R1, [R0]
                CPSID   I

                IMPORT  SystemInit
                IMPORT  __main
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0


                ALIGN
                END

