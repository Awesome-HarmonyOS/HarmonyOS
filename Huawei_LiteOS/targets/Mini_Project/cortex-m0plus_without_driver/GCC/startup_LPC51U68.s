/* --------------------------------------------------------------------------*/
/* @file:    startup_LPC51U68.S                                                */
/* @purpose: CMSIS Cortex-M0+ Core Device Startup File                       */
/*           LPC51U68                                                          */
/* @version: 1.0                                                      */
/* @date:    2017-12-15                                                         */
/* --------------------------------------------------------------------------*/
/*                                                                           */
/* The Clear BSD License                                                     */
/* Copyright 1997-2016 Freescale Semiconductor, Inc.                         */
/* Copyright 2016-2018 NXP                                                   */
/* All rights reserved.                                                      */
/*                                                                           */
/* Redistribution and use in source and binary forms, with or without        */
/* modification, are permitted (subject to the limitations in the            */
/* disclaimer below) provided that the following conditions are met:         */
/*                                                                           */
/* * Redistributions of source code must retain the above copyright          */
/*   notice, this list of conditions and the following disclaimer.           */
/*                                                                           */
/* * Redistributions in binary form must reproduce the above copyright       */
/*   notice, this list of conditions and the following disclaimer in the     */
/*   documentation and/or other materials provided with the distribution.    */
/*                                                                           */
/* * Neither the name of the copyright holder nor the names of its           */
/*   contributors may be used to endorse or promote products derived from    */
/*   this software without specific prior written permission.                */
/*                                                                           */
/* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE           */
/* GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT       */
/* HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED               */
/* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF      */
/* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                  */
/* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE     */
/* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR       */
/* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF      */
/* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR           */
/* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,     */
/* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE      */
/* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN    */
/* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                             */
/*****************************************************************************/
/* Version: GCC for ARM Embedded Processors                                  */
/*****************************************************************************/


    .syntax unified
    .arch armv6-m

    .section .isr_vector, "a"
    .align 2
    .globl __Vectors
__Vectors:
    .long   __StackTop                                      /* Top of Stack       */
    .long   Reset_Handler                                   /* Reset Handler      */
    .long   NMI_Handler                                     /* NMI Handler        */
    .long   HardFault_Handler                               /* Hard Fault Handler */
    .long   0                                               /* Reserved           */
    .long   0                                               /* Reserved           */
    .long   0                                               /* Reserved           */
    .long   0                                               /* Reserved           */
    .long   0                                               /* Reserved           */
    .long   0                                               /* Reserved           */
    .long   0                                               /* Reserved           */
    .long   SVC_Handler                                     /* SVCall Handler     */
    .long   0                                               /* Reserved           */
    .long   0                                               /* Reserved           */
    .long   PendSV_Handler                                  /* PendSV Handler     */
    .long   SysTick_Handler                                 /* SysTick Handler    */

    /* External Interrupts */
    .long   WDT_BOD_IRQHandler                     /* Windowed watchdog timer, Brownout detect */
    .long   DMA0_IRQHandler                     /* DMA controller */
    .long   GINT0_IRQHandler                     /* GPIO group 0 */
    .long   GINT1_IRQHandler                     /* GPIO group 1 */
    .long   PIN_INT0_IRQHandler                     /* Pin interrupt 0 or pattern match engine slice 0 */
    .long   PIN_INT1_IRQHandler                     /* Pin interrupt 1or pattern match engine slice 1 */
    .long   PIN_INT2_IRQHandler                     /* Pin interrupt 2 or pattern match engine slice 2 */
    .long   PIN_INT3_IRQHandler                     /* Pin interrupt 3 or pattern match engine slice 3 */
    .long   UTICK0_IRQHandler                     /* Micro-tick Timer */
    .long   MRT0_IRQHandler                     /* Multi-rate timer */
    .long   CTIMER0_IRQHandler                     /* Standard counter/timer CTIMER0 */
    .long   CTIMER1_IRQHandler                     /* Standard counter/timer CTIMER1 */
    .long   SCT0_IRQHandler                     /* SCTimer/PWM */
    .long   CTIMER3_IRQHandler                     /* Standard counter/timer CTIMER3 */
    .long   FLEXCOMM0_IRQHandler                     /* Flexcomm Interface 0 (USART, SPI, I2C) */
    .long   FLEXCOMM1_IRQHandler                     /* Flexcomm Interface 1 (USART, SPI, I2C) */
    .long   FLEXCOMM2_IRQHandler                     /* Flexcomm Interface 2 (USART, SPI, I2C) */
    .long   FLEXCOMM3_IRQHandler                     /* Flexcomm Interface 3 (USART, SPI, I2C) */
    .long   FLEXCOMM4_IRQHandler                     /* Flexcomm Interface 4 (USART, SPI, I2C) */
    .long   FLEXCOMM5_IRQHandler                     /* Flexcomm Interface 5 (USART, SPI, I2C) */
    .long   FLEXCOMM6_IRQHandler                     /* Flexcomm Interface 6 (USART, SPI, I2C, I2S) */
    .long   FLEXCOMM7_IRQHandler                     /* Flexcomm Interface 7 (USART, SPI, I2C, I2S) */
    .long   ADC0_SEQA_IRQHandler                     /* ADC0 sequence A completion. */
    .long   ADC0_SEQB_IRQHandler                     /* ADC0 sequence B completion. */
    .long   ADC0_THCMP_IRQHandler                     /* ADC0 threshold compare and error. */
    .long   Reserved41_IRQHandler                     /* Reserved interrupt */
    .long   Reserved42_IRQHandler                     /* Reserved interrupt */
    .long   USB0_NEEDCLK_IRQHandler                     /* USB Activity Wake-up Interrupt */
    .long   USB0_IRQHandler                     /* USB device */
    .long   RTC_IRQHandler                     /* RTC alarm and wake-up interrupts */
    .long   Reserved46_IRQHandler                     /* Reserved interrupt */
    .long   Reserved47_IRQHandler                     /* Reserved interrupt */

    .size    __Vectors, . - __Vectors

   .text
    .thumb

/* Reset Handler */
    .thumb_func
    .align 2
    .globl   Reset_Handler
    .weak    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    cpsid   i               /* Mask interrupts */

#ifndef __NO_SYSTEM_INIT
    ldr   r0,=SystemInit
    blx   r0
#endif
/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

    ldr    r1, =__etext
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

    subs    r3, r2
    ble     .LC0

.LC1:
    subs    r3, 4
    ldr    r0, [r1,r3]
    str    r0, [r2,r3]
    bgt    .LC1
.LC0:

#ifdef __STARTUP_CLEAR_BSS
/*     This part of work usually is done in C library startup code. Otherwise,
 *     define this macro to enable it in this startup.
 *
 *     Loop to zero out BSS section, which uses following symbols
 *     in linker script:
 *      __bss_start__: start of BSS section. Must align to 4
 *      __bss_end__: end of BSS section. Must align to 4
 */
    ldr r1, =__bss_start__
    ldr r2, =__bss_end__

    subs    r2, r1
    ble .LC3

    movs    r0, 0
.LC2:
    str r0, [r1, r2]
    subs    r2, 4
    bge .LC2
.LC3:
#endif
    cpsie   i               /* Unmask interrupts */

#ifndef __START
#define __START _start
#endif
#ifndef __ATOLLIC__
    ldr   r0,=__START
    blx   r0
#else
    ldr   r0,=__libc_init_array
    blx   r0
    ldr   r0,=main
    bx    r0
#endif
    .pool
    .size Reset_Handler, . - Reset_Handler

    .align  1
    .thumb_func
    .weak DefaultISR
    .type DefaultISR, %function
DefaultISR:
    ldr r0, =DefaultISR
    bx r0
    .size DefaultISR, . - DefaultISR

    .align 1
    .thumb_func
    .weak NMI_Handler
    .type NMI_Handler, %function
NMI_Handler:
    ldr   r0,=NMI_Handler
    bx    r0
    .size NMI_Handler, . - NMI_Handler

    .align 1
    .thumb_func
    .weak HardFault_Handler
    .type HardFault_Handler, %function
HardFault_Handler:
    ldr   r0,=HardFault_Handler
    bx    r0
    .size HardFault_Handler, . - HardFault_Handler

    .align 1
    .thumb_func
    .weak SVC_Handler
    .type SVC_Handler, %function
SVC_Handler:
    ldr   r0,=SVC_Handler
    bx    r0
    .size SVC_Handler, . - SVC_Handler

    .align 1
    .thumb_func
    .weak PendSV_Handler
    .type PendSV_Handler, %function
PendSV_Handler:
    ldr   r0,=PendSV_Handler
    bx    r0
    .size PendSV_Handler, . - PendSV_Handler

    .align 1
    .thumb_func
    .weak SysTick_Handler
    .type SysTick_Handler, %function
SysTick_Handler:
    ldr   r0,=SysTick_Handler
    bx    r0
    .size SysTick_Handler, . - SysTick_Handler

    .align 1
    .thumb_func
    .weak WDT_BOD_IRQHandler
    .type WDT_BOD_IRQHandler, %function
WDT_BOD_IRQHandler:
    ldr   r0,=WDT_BOD_DriverIRQHandler
    bx    r0
    .size WDT_BOD_IRQHandler, . - WDT_BOD_IRQHandler

    .align 1
    .thumb_func
    .weak DMA0_IRQHandler
    .type DMA0_IRQHandler, %function
DMA0_IRQHandler:
    ldr   r0,=DMA0_DriverIRQHandler
    bx    r0
    .size DMA0_IRQHandler, . - DMA0_IRQHandler

    .align 1
    .thumb_func
    .weak GINT0_IRQHandler
    .type GINT0_IRQHandler, %function
GINT0_IRQHandler:
    ldr   r0,=GINT0_DriverIRQHandler
    bx    r0
    .size GINT0_IRQHandler, . - GINT0_IRQHandler

    .align 1
    .thumb_func
    .weak GINT1_IRQHandler
    .type GINT1_IRQHandler, %function
GINT1_IRQHandler:
    ldr   r0,=GINT1_DriverIRQHandler
    bx    r0
    .size GINT1_IRQHandler, . - GINT1_IRQHandler

    .align 1
    .thumb_func
    .weak PIN_INT0_IRQHandler
    .type PIN_INT0_IRQHandler, %function
PIN_INT0_IRQHandler:
    ldr   r0,=PIN_INT0_DriverIRQHandler
    bx    r0
    .size PIN_INT0_IRQHandler, . - PIN_INT0_IRQHandler

    .align 1
    .thumb_func
    .weak PIN_INT1_IRQHandler
    .type PIN_INT1_IRQHandler, %function
PIN_INT1_IRQHandler:
    ldr   r0,=PIN_INT1_DriverIRQHandler
    bx    r0
    .size PIN_INT1_IRQHandler, . - PIN_INT1_IRQHandler

    .align 1
    .thumb_func
    .weak PIN_INT2_IRQHandler
    .type PIN_INT2_IRQHandler, %function
PIN_INT2_IRQHandler:
    ldr   r0,=PIN_INT2_DriverIRQHandler
    bx    r0
    .size PIN_INT2_IRQHandler, . - PIN_INT2_IRQHandler

    .align 1
    .thumb_func
    .weak PIN_INT3_IRQHandler
    .type PIN_INT3_IRQHandler, %function
PIN_INT3_IRQHandler:
    ldr   r0,=PIN_INT3_DriverIRQHandler
    bx    r0
    .size PIN_INT3_IRQHandler, . - PIN_INT3_IRQHandler

    .align 1
    .thumb_func
    .weak UTICK0_IRQHandler
    .type UTICK0_IRQHandler, %function
UTICK0_IRQHandler:
    ldr   r0,=UTICK0_DriverIRQHandler
    bx    r0
    .size UTICK0_IRQHandler, . - UTICK0_IRQHandler

    .align 1
    .thumb_func
    .weak MRT0_IRQHandler
    .type MRT0_IRQHandler, %function
MRT0_IRQHandler:
    ldr   r0,=MRT0_DriverIRQHandler
    bx    r0
    .size MRT0_IRQHandler, . - MRT0_IRQHandler

    .align 1
    .thumb_func
    .weak CTIMER0_IRQHandler
    .type CTIMER0_IRQHandler, %function
CTIMER0_IRQHandler:
    ldr   r0,=CTIMER0_DriverIRQHandler
    bx    r0
    .size CTIMER0_IRQHandler, . - CTIMER0_IRQHandler

    .align 1
    .thumb_func
    .weak CTIMER1_IRQHandler
    .type CTIMER1_IRQHandler, %function
CTIMER1_IRQHandler:
    ldr   r0,=CTIMER1_DriverIRQHandler
    bx    r0
    .size CTIMER1_IRQHandler, . - CTIMER1_IRQHandler

    .align 1
    .thumb_func
    .weak SCT0_IRQHandler
    .type SCT0_IRQHandler, %function
SCT0_IRQHandler:
    ldr   r0,=SCT0_DriverIRQHandler
    bx    r0
    .size SCT0_IRQHandler, . - SCT0_IRQHandler

    .align 1
    .thumb_func
    .weak CTIMER3_IRQHandler
    .type CTIMER3_IRQHandler, %function
CTIMER3_IRQHandler:
    ldr   r0,=CTIMER3_DriverIRQHandler
    bx    r0
    .size CTIMER3_IRQHandler, . - CTIMER3_IRQHandler

    .align 1
    .thumb_func
    .weak FLEXCOMM0_IRQHandler
    .type FLEXCOMM0_IRQHandler, %function
FLEXCOMM0_IRQHandler:
    ldr   r0,=FLEXCOMM0_DriverIRQHandler
    bx    r0
    .size FLEXCOMM0_IRQHandler, . - FLEXCOMM0_IRQHandler

    .align 1
    .thumb_func
    .weak FLEXCOMM1_IRQHandler
    .type FLEXCOMM1_IRQHandler, %function
FLEXCOMM1_IRQHandler:
    ldr   r0,=FLEXCOMM1_DriverIRQHandler
    bx    r0
    .size FLEXCOMM1_IRQHandler, . - FLEXCOMM1_IRQHandler

    .align 1
    .thumb_func
    .weak FLEXCOMM2_IRQHandler
    .type FLEXCOMM2_IRQHandler, %function
FLEXCOMM2_IRQHandler:
    ldr   r0,=FLEXCOMM2_DriverIRQHandler
    bx    r0
    .size FLEXCOMM2_IRQHandler, . - FLEXCOMM2_IRQHandler

    .align 1
    .thumb_func
    .weak FLEXCOMM3_IRQHandler
    .type FLEXCOMM3_IRQHandler, %function
FLEXCOMM3_IRQHandler:
    ldr   r0,=FLEXCOMM3_DriverIRQHandler
    bx    r0
    .size FLEXCOMM3_IRQHandler, . - FLEXCOMM3_IRQHandler

    .align 1
    .thumb_func
    .weak FLEXCOMM4_IRQHandler
    .type FLEXCOMM4_IRQHandler, %function
FLEXCOMM4_IRQHandler:
    ldr   r0,=FLEXCOMM4_DriverIRQHandler
    bx    r0
    .size FLEXCOMM4_IRQHandler, . - FLEXCOMM4_IRQHandler

    .align 1
    .thumb_func
    .weak FLEXCOMM5_IRQHandler
    .type FLEXCOMM5_IRQHandler, %function
FLEXCOMM5_IRQHandler:
    ldr   r0,=FLEXCOMM5_DriverIRQHandler
    bx    r0
    .size FLEXCOMM5_IRQHandler, . - FLEXCOMM5_IRQHandler

    .align 1
    .thumb_func
    .weak FLEXCOMM6_IRQHandler
    .type FLEXCOMM6_IRQHandler, %function
FLEXCOMM6_IRQHandler:
    ldr   r0,=FLEXCOMM6_DriverIRQHandler
    bx    r0
    .size FLEXCOMM6_IRQHandler, . - FLEXCOMM6_IRQHandler

    .align 1
    .thumb_func
    .weak FLEXCOMM7_IRQHandler
    .type FLEXCOMM7_IRQHandler, %function
FLEXCOMM7_IRQHandler:
    ldr   r0,=FLEXCOMM7_DriverIRQHandler
    bx    r0
    .size FLEXCOMM7_IRQHandler, . - FLEXCOMM7_IRQHandler

    .align 1
    .thumb_func
    .weak ADC0_SEQA_IRQHandler
    .type ADC0_SEQA_IRQHandler, %function
ADC0_SEQA_IRQHandler:
    ldr   r0,=ADC0_SEQA_DriverIRQHandler
    bx    r0
    .size ADC0_SEQA_IRQHandler, . - ADC0_SEQA_IRQHandler

    .align 1
    .thumb_func
    .weak ADC0_SEQB_IRQHandler
    .type ADC0_SEQB_IRQHandler, %function
ADC0_SEQB_IRQHandler:
    ldr   r0,=ADC0_SEQB_DriverIRQHandler
    bx    r0
    .size ADC0_SEQB_IRQHandler, . - ADC0_SEQB_IRQHandler

    .align 1
    .thumb_func
    .weak ADC0_THCMP_IRQHandler
    .type ADC0_THCMP_IRQHandler, %function
ADC0_THCMP_IRQHandler:
    ldr   r0,=ADC0_THCMP_DriverIRQHandler
    bx    r0
    .size ADC0_THCMP_IRQHandler, . - ADC0_THCMP_IRQHandler

    .align 1
    .thumb_func
    .weak Reserved41_IRQHandler
    .type Reserved41_IRQHandler, %function
Reserved41_IRQHandler:
    ldr   r0,=Reserved41_DriverIRQHandler
    bx    r0
    .size Reserved41_IRQHandler, . - Reserved41_IRQHandler

    .align 1
    .thumb_func
    .weak Reserved42_IRQHandler
    .type Reserved42_IRQHandler, %function
Reserved42_IRQHandler:
    ldr   r0,=Reserved42_DriverIRQHandler
    bx    r0
    .size Reserved42_IRQHandler, . - Reserved42_IRQHandler

    .align 1
    .thumb_func
    .weak USB0_NEEDCLK_IRQHandler
    .type USB0_NEEDCLK_IRQHandler, %function
USB0_NEEDCLK_IRQHandler:
    ldr   r0,=USB0_NEEDCLK_DriverIRQHandler
    bx    r0
    .size USB0_NEEDCLK_IRQHandler, . - USB0_NEEDCLK_IRQHandler

    .align 1
    .thumb_func
    .weak USB0_IRQHandler
    .type USB0_IRQHandler, %function
USB0_IRQHandler:
    ldr   r0,=USB0_DriverIRQHandler
    bx    r0
    .size USB0_IRQHandler, . - USB0_IRQHandler

    .align 1
    .thumb_func
    .weak RTC_IRQHandler
    .type RTC_IRQHandler, %function
RTC_IRQHandler:
    ldr   r0,=RTC_DriverIRQHandler
    bx    r0
    .size RTC_IRQHandler, . - RTC_IRQHandler

    .align 1
    .thumb_func
    .weak Reserved46_IRQHandler
    .type Reserved46_IRQHandler, %function
Reserved46_IRQHandler:
    ldr   r0,=Reserved46_DriverIRQHandler
    bx    r0
    .size Reserved46_IRQHandler, . - Reserved46_IRQHandler

    .align 1
    .thumb_func
    .weak Reserved47_IRQHandler
    .type Reserved47_IRQHandler, %function
Reserved47_IRQHandler:
    ldr   r0,=Reserved47_DriverIRQHandler
    bx    r0
    .size Reserved47_IRQHandler, . - Reserved47_IRQHandler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro def_irq_handler  handler_name
    .weak \handler_name
    .set  \handler_name, DefaultISR
    .endm
    def_irq_handler    WDT_BOD_DriverIRQHandler
    def_irq_handler    DMA0_DriverIRQHandler
    def_irq_handler    GINT0_DriverIRQHandler
    def_irq_handler    GINT1_DriverIRQHandler
    def_irq_handler    PIN_INT0_DriverIRQHandler
    def_irq_handler    PIN_INT1_DriverIRQHandler
    def_irq_handler    PIN_INT2_DriverIRQHandler
    def_irq_handler    PIN_INT3_DriverIRQHandler
    def_irq_handler    UTICK0_DriverIRQHandler
    def_irq_handler    MRT0_DriverIRQHandler
    def_irq_handler    CTIMER0_DriverIRQHandler
    def_irq_handler    CTIMER1_DriverIRQHandler
    def_irq_handler    SCT0_DriverIRQHandler
    def_irq_handler    CTIMER3_DriverIRQHandler
    def_irq_handler    FLEXCOMM0_DriverIRQHandler
    def_irq_handler    FLEXCOMM1_DriverIRQHandler
    def_irq_handler    FLEXCOMM2_DriverIRQHandler
    def_irq_handler    FLEXCOMM3_DriverIRQHandler
    def_irq_handler    FLEXCOMM4_DriverIRQHandler
    def_irq_handler    FLEXCOMM5_DriverIRQHandler
    def_irq_handler    FLEXCOMM6_DriverIRQHandler
    def_irq_handler    FLEXCOMM7_DriverIRQHandler
    def_irq_handler    ADC0_SEQA_DriverIRQHandler
    def_irq_handler    ADC0_SEQB_DriverIRQHandler
    def_irq_handler    ADC0_THCMP_DriverIRQHandler
    def_irq_handler    Reserved41_DriverIRQHandler
    def_irq_handler    Reserved42_DriverIRQHandler
    def_irq_handler    USB0_NEEDCLK_DriverIRQHandler
    def_irq_handler    USB0_DriverIRQHandler
    def_irq_handler    RTC_DriverIRQHandler
    def_irq_handler    Reserved46_DriverIRQHandler
    def_irq_handler    Reserved47_DriverIRQHandler

    .end
