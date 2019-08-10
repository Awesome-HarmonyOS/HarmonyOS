/*
** ###################################################################
**     Processors:          LPC51U68JBD48
**                          LPC51U68JBD64
**
**     Compilers:           Keil ARM C/C++ Compiler
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**                          MCUXpresso Compiler
**
**     Reference manual:    LPC51U68 User manual User manual Rev. 1.0 13 Dec 2017
**     Version:             rev. 1.0, 2017-12-15
**     Build:               b180116
**
**     Abstract:
**         CMSIS Peripheral Access Layer for LPC51U68
**
**     The Clear BSD License
**     Copyright 1997-2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2018 NXP
**     All rights reserved.
**
**     Redistribution and use in source and binary forms, with or without
**     modification, are permitted (subject to the limitations in the
**     disclaimer below) provided that the following conditions are met:
**
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**
**     * Neither the name of the copyright holder nor the names of its
**       contributors may be used to endorse or promote products derived from
**       this software without specific prior written permission.
**
**     NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
**     GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
**     HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
**     WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
**     MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
**     LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
**     CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
**     SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
**     BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
**     WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
**     OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
**     IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 1.0 (2017-12-15)
**         Initial version.
**
** ###################################################################
*/

/*!
 * @file LPC51U68.h
 * @version 1.0
 * @date 2017-12-15
 * @brief CMSIS Peripheral Access Layer for LPC51U68
 *
 * CMSIS Peripheral Access Layer for LPC51U68
 */

#ifndef _LPC51U68_H_
#define _LPC51U68_H_                             /**< Symbol preventing repeated inclusion */

/** Memory map major version (memory maps with equal major version number are
 * compatible) */
#define MCU_MEM_MAP_VERSION 0x0100U
/** Memory map minor version */
#define MCU_MEM_MAP_VERSION_MINOR 0x0000U


/* ----------------------------------------------------------------------------
   -- Interrupt vector numbers
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Interrupt_vector_numbers Interrupt vector numbers
 * @{
 */

/** Interrupt Number Definitions */
#define NUMBER_OF_INT_VECTORS 48                 /**< Number of interrupts in the Vector table */

typedef enum IRQn {
  /* Auxiliary constants */
  NotAvail_IRQn                = -128,             /**< Not available device specific interrupt */

  /* Core interrupts */
  NonMaskableInt_IRQn          = -14,              /**< Non Maskable Interrupt */
  HardFault_IRQn               = -13,              /**< Cortex-M0 SV Hard Fault Interrupt */
  SVCall_IRQn                  = -5,               /**< Cortex-M0 SV Call Interrupt */
  PendSV_IRQn                  = -2,               /**< Cortex-M0 Pend SV Interrupt */
  SysTick_IRQn                 = -1,               /**< Cortex-M0 System Tick Interrupt */

  /* Device specific interrupts */
  WDT_BOD_IRQn                 = 0,                /**< Windowed watchdog timer, Brownout detect */
  DMA0_IRQn                    = 1,                /**< DMA controller */
  GINT0_IRQn                   = 2,                /**< GPIO group 0 */
  GINT1_IRQn                   = 3,                /**< GPIO group 1 */
  PIN_INT0_IRQn                = 4,                /**< Pin interrupt 0 or pattern match engine slice 0 */
  PIN_INT1_IRQn                = 5,                /**< Pin interrupt 1or pattern match engine slice 1 */
  PIN_INT2_IRQn                = 6,                /**< Pin interrupt 2 or pattern match engine slice 2 */
  PIN_INT3_IRQn                = 7,                /**< Pin interrupt 3 or pattern match engine slice 3 */
  UTICK0_IRQn                  = 8,                /**< Micro-tick Timer */
  MRT0_IRQn                    = 9,                /**< Multi-rate timer */
  CTIMER0_IRQn                 = 10,               /**< Standard counter/timer CTIMER0 */
  CTIMER1_IRQn                 = 11,               /**< Standard counter/timer CTIMER1 */
  SCT0_IRQn                    = 12,               /**< SCTimer/PWM */
  CTIMER3_IRQn                 = 13,               /**< Standard counter/timer CTIMER3 */
  FLEXCOMM0_IRQn               = 14,               /**< Flexcomm Interface 0 (USART, SPI, I2C) */
  FLEXCOMM1_IRQn               = 15,               /**< Flexcomm Interface 1 (USART, SPI, I2C) */
  FLEXCOMM2_IRQn               = 16,               /**< Flexcomm Interface 2 (USART, SPI, I2C) */
  FLEXCOMM3_IRQn               = 17,               /**< Flexcomm Interface 3 (USART, SPI, I2C) */
  FLEXCOMM4_IRQn               = 18,               /**< Flexcomm Interface 4 (USART, SPI, I2C) */
  FLEXCOMM5_IRQn               = 19,               /**< Flexcomm Interface 5 (USART, SPI, I2C) */
  FLEXCOMM6_IRQn               = 20,               /**< Flexcomm Interface 6 (USART, SPI, I2C, I2S) */
  FLEXCOMM7_IRQn               = 21,               /**< Flexcomm Interface 7 (USART, SPI, I2C, I2S) */
  ADC0_SEQA_IRQn               = 22,               /**< ADC0 sequence A completion. */
  ADC0_SEQB_IRQn               = 23,               /**< ADC0 sequence B completion. */
  ADC0_THCMP_IRQn              = 24,               /**< ADC0 threshold compare and error. */
  Reserved41_IRQn              = 25,               /**< Reserved interrupt */
  Reserved42_IRQn              = 26,               /**< Reserved interrupt */
  USB0_NEEDCLK_IRQn            = 27,               /**< USB Activity Wake-up Interrupt */
  USB0_IRQn                    = 28,               /**< USB device */
  RTC_IRQn                     = 29,               /**< RTC alarm and wake-up interrupts */
  Reserved46_IRQn              = 30,               /**< Reserved interrupt */
  Reserved47_IRQn              = 31                /**< Reserved interrupt */
} IRQn_Type;

/*!
 * @}
 */ /* end of group Interrupt_vector_numbers */


/* ----------------------------------------------------------------------------
   -- Cortex M0 Core Configuration
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Cortex_Core_Configuration Cortex M0 Core Configuration
 * @{
 */

#define __CM0PLUS_REV                  0x0000    /**< Core revision r0p0 */
#define __MPU_PRESENT                  0         /**< Defines if an MPU is present or not */
#define __VTOR_PRESENT                 1         /**< Defines if VTOR is present or not */
#define __NVIC_PRIO_BITS               2         /**< Number of priority bits implemented in the NVIC */
#define __Vendor_SysTickConfig         0         /**< Vendor specific implementation of SysTickConfig is defined */

#include "core_cm0plus.h"              /* Core Peripheral Access Layer */
#include "system_LPC51U68.h"           /* Device specific configuration file */

/*!
 * @}
 */ /* end of group Cortex_Core_Configuration */


/* ----------------------------------------------------------------------------
   -- Device Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Peripheral_access_layer Device Peripheral Access Layer
 * @{
 */


/*
** Start of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #if (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
  #else
    #pragma push
    #pragma anon_unions
  #endif
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=extended
#else
  #error Not supported compiler type
#endif

/* ----------------------------------------------------------------------------
   -- ADC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Peripheral_Access_Layer ADC Peripheral Access Layer
 * @{
 */

/** ADC - Register Layout Typedef */
typedef struct {
  __IO uint32_t CTRL;                              /**< ADC Control register. Contains the clock divide value, resolution selection, sampling time selection, and mode controls., offset: 0x0 */
  __IO uint32_t INSEL;                             /**< Input Select. Allows selection of the temperature sensor as an alternate input to ADC channel 0., offset: 0x4 */
  __IO uint32_t SEQ_CTRL[2];                       /**< ADC Conversion Sequence-n control register: Controls triggering and channel selection for conversion sequence-n. Also specifies interrupt mode for sequence-n., array offset: 0x8, array step: 0x4 */
  __I  uint32_t SEQ_GDAT[2];                       /**< ADC Sequence-n Global Data register. This register contains the result of the most recent ADC conversion performed under sequence-n., array offset: 0x10, array step: 0x4 */
       uint8_t RESERVED_0[8];
  __I  uint32_t DAT[12];                           /**< ADC Channel 0 Data register. This register contains the result of the most recent conversion completed on channel 0., array offset: 0x20, array step: 0x4 */
  __IO uint32_t THR0_LOW;                          /**< ADC Low Compare Threshold register 0: Contains the lower threshold level for automatic threshold comparison for any channels linked to threshold pair 0., offset: 0x50 */
  __IO uint32_t THR1_LOW;                          /**< ADC Low Compare Threshold register 1: Contains the lower threshold level for automatic threshold comparison for any channels linked to threshold pair 1., offset: 0x54 */
  __IO uint32_t THR0_HIGH;                         /**< ADC High Compare Threshold register 0: Contains the upper threshold level for automatic threshold comparison for any channels linked to threshold pair 0., offset: 0x58 */
  __IO uint32_t THR1_HIGH;                         /**< ADC High Compare Threshold register 1: Contains the upper threshold level for automatic threshold comparison for any channels linked to threshold pair 1., offset: 0x5C */
  __IO uint32_t CHAN_THRSEL;                       /**< ADC Channel-Threshold Select register. Specifies which set of threshold compare registers are to be used for each channel, offset: 0x60 */
  __IO uint32_t INTEN;                             /**< ADC Interrupt Enable register. This register contains enable bits that enable the sequence-A, sequence-B, threshold compare and data overrun interrupts to be generated., offset: 0x64 */
  __IO uint32_t FLAGS;                             /**< ADC Flags register. Contains the four interrupt/DMA trigger flags and the individual component overrun and threshold-compare flags. (The overrun bits replicate information stored in the result registers)., offset: 0x68 */
  __IO uint32_t STARTUP;                           /**< ADC Startup register., offset: 0x6C */
  __IO uint32_t CALIB;                             /**< ADC Calibration register., offset: 0x70 */
} ADC_Type;

/* ----------------------------------------------------------------------------
   -- ADC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Masks ADC Register Masks
 * @{
 */

/*! @name CTRL - ADC Control register. Contains the clock divide value, resolution selection, sampling time selection, and mode controls. */
#define ADC_CTRL_CLKDIV_MASK                     (0xFFU)
#define ADC_CTRL_CLKDIV_SHIFT                    (0U)
#define ADC_CTRL_CLKDIV(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_CTRL_CLKDIV_SHIFT)) & ADC_CTRL_CLKDIV_MASK)
#define ADC_CTRL_ASYNMODE_MASK                   (0x100U)
#define ADC_CTRL_ASYNMODE_SHIFT                  (8U)
#define ADC_CTRL_ASYNMODE(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_CTRL_ASYNMODE_SHIFT)) & ADC_CTRL_ASYNMODE_MASK)
#define ADC_CTRL_RESOL_MASK                      (0x600U)
#define ADC_CTRL_RESOL_SHIFT                     (9U)
#define ADC_CTRL_RESOL(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_CTRL_RESOL_SHIFT)) & ADC_CTRL_RESOL_MASK)
#define ADC_CTRL_BYPASSCAL_MASK                  (0x800U)
#define ADC_CTRL_BYPASSCAL_SHIFT                 (11U)
#define ADC_CTRL_BYPASSCAL(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_CTRL_BYPASSCAL_SHIFT)) & ADC_CTRL_BYPASSCAL_MASK)
#define ADC_CTRL_TSAMP_MASK                      (0x7000U)
#define ADC_CTRL_TSAMP_SHIFT                     (12U)
#define ADC_CTRL_TSAMP(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_CTRL_TSAMP_SHIFT)) & ADC_CTRL_TSAMP_MASK)

/*! @name INSEL - Input Select. Allows selection of the temperature sensor as an alternate input to ADC channel 0. */
#define ADC_INSEL_SEL_MASK                       (0x3U)
#define ADC_INSEL_SEL_SHIFT                      (0U)
#define ADC_INSEL_SEL(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_INSEL_SEL_SHIFT)) & ADC_INSEL_SEL_MASK)

/*! @name SEQ_CTRL - ADC Conversion Sequence-n control register: Controls triggering and channel selection for conversion sequence-n. Also specifies interrupt mode for sequence-n. */
#define ADC_SEQ_CTRL_CHANNELS_MASK               (0xFFFU)
#define ADC_SEQ_CTRL_CHANNELS_SHIFT              (0U)
#define ADC_SEQ_CTRL_CHANNELS(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_SEQ_CTRL_CHANNELS_SHIFT)) & ADC_SEQ_CTRL_CHANNELS_MASK)
#define ADC_SEQ_CTRL_TRIGGER_MASK                (0x3F000U)
#define ADC_SEQ_CTRL_TRIGGER_SHIFT               (12U)
#define ADC_SEQ_CTRL_TRIGGER(x)                  (((uint32_t)(((uint32_t)(x)) << ADC_SEQ_CTRL_TRIGGER_SHIFT)) & ADC_SEQ_CTRL_TRIGGER_MASK)
#define ADC_SEQ_CTRL_TRIGPOL_MASK                (0x40000U)
#define ADC_SEQ_CTRL_TRIGPOL_SHIFT               (18U)
#define ADC_SEQ_CTRL_TRIGPOL(x)                  (((uint32_t)(((uint32_t)(x)) << ADC_SEQ_CTRL_TRIGPOL_SHIFT)) & ADC_SEQ_CTRL_TRIGPOL_MASK)
#define ADC_SEQ_CTRL_SYNCBYPASS_MASK             (0x80000U)
#define ADC_SEQ_CTRL_SYNCBYPASS_SHIFT            (19U)
#define ADC_SEQ_CTRL_SYNCBYPASS(x)               (((uint32_t)(((uint32_t)(x)) << ADC_SEQ_CTRL_SYNCBYPASS_SHIFT)) & ADC_SEQ_CTRL_SYNCBYPASS_MASK)
#define ADC_SEQ_CTRL_START_MASK                  (0x4000000U)
#define ADC_SEQ_CTRL_START_SHIFT                 (26U)
#define ADC_SEQ_CTRL_START(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_SEQ_CTRL_START_SHIFT)) & ADC_SEQ_CTRL_START_MASK)
#define ADC_SEQ_CTRL_BURST_MASK                  (0x8000000U)
#define ADC_SEQ_CTRL_BURST_SHIFT                 (27U)
#define ADC_SEQ_CTRL_BURST(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_SEQ_CTRL_BURST_SHIFT)) & ADC_SEQ_CTRL_BURST_MASK)
#define ADC_SEQ_CTRL_SINGLESTEP_MASK             (0x10000000U)
#define ADC_SEQ_CTRL_SINGLESTEP_SHIFT            (28U)
#define ADC_SEQ_CTRL_SINGLESTEP(x)               (((uint32_t)(((uint32_t)(x)) << ADC_SEQ_CTRL_SINGLESTEP_SHIFT)) & ADC_SEQ_CTRL_SINGLESTEP_MASK)
#define ADC_SEQ_CTRL_LOWPRIO_MASK                (0x20000000U)
#define ADC_SEQ_CTRL_LOWPRIO_SHIFT               (29U)
#define ADC_SEQ_CTRL_LOWPRIO(x)                  (((uint32_t)(((uint32_t)(x)) << ADC_SEQ_CTRL_LOWPRIO_SHIFT)) & ADC_SEQ_CTRL_LOWPRIO_MASK)
#define ADC_SEQ_CTRL_MODE_MASK                   (0x40000000U)
#define ADC_SEQ_CTRL_MODE_SHIFT                  (30U)
#define ADC_SEQ_CTRL_MODE(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_SEQ_CTRL_MODE_SHIFT)) & ADC_SEQ_CTRL_MODE_MASK)
#define ADC_SEQ_CTRL_SEQ_ENA_MASK                (0x80000000U)
#define ADC_SEQ_CTRL_SEQ_ENA_SHIFT               (31U)
#define ADC_SEQ_CTRL_SEQ_ENA(x)                  (((uint32_t)(((uint32_t)(x)) << ADC_SEQ_CTRL_SEQ_ENA_SHIFT)) & ADC_SEQ_CTRL_SEQ_ENA_MASK)

/* The count of ADC_SEQ_CTRL */
#define ADC_SEQ_CTRL_COUNT                       (2U)

/*! @name SEQ_GDAT - ADC Sequence-n Global Data register. This register contains the result of the most recent ADC conversion performed under sequence-n. */
#define ADC_SEQ_GDAT_RESULT_MASK                 (0xFFF0U)
#define ADC_SEQ_GDAT_RESULT_SHIFT                (4U)
#define ADC_SEQ_GDAT_RESULT(x)                   (((uint32_t)(((uint32_t)(x)) << ADC_SEQ_GDAT_RESULT_SHIFT)) & ADC_SEQ_GDAT_RESULT_MASK)
#define ADC_SEQ_GDAT_THCMPRANGE_MASK             (0x30000U)
#define ADC_SEQ_GDAT_THCMPRANGE_SHIFT            (16U)
#define ADC_SEQ_GDAT_THCMPRANGE(x)               (((uint32_t)(((uint32_t)(x)) << ADC_SEQ_GDAT_THCMPRANGE_SHIFT)) & ADC_SEQ_GDAT_THCMPRANGE_MASK)
#define ADC_SEQ_GDAT_THCMPCROSS_MASK             (0xC0000U)
#define ADC_SEQ_GDAT_THCMPCROSS_SHIFT            (18U)
#define ADC_SEQ_GDAT_THCMPCROSS(x)               (((uint32_t)(((uint32_t)(x)) << ADC_SEQ_GDAT_THCMPCROSS_SHIFT)) & ADC_SEQ_GDAT_THCMPCROSS_MASK)
#define ADC_SEQ_GDAT_CHN_MASK                    (0x3C000000U)
#define ADC_SEQ_GDAT_CHN_SHIFT                   (26U)
#define ADC_SEQ_GDAT_CHN(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_SEQ_GDAT_CHN_SHIFT)) & ADC_SEQ_GDAT_CHN_MASK)
#define ADC_SEQ_GDAT_OVERRUN_MASK                (0x40000000U)
#define ADC_SEQ_GDAT_OVERRUN_SHIFT               (30U)
#define ADC_SEQ_GDAT_OVERRUN(x)                  (((uint32_t)(((uint32_t)(x)) << ADC_SEQ_GDAT_OVERRUN_SHIFT)) & ADC_SEQ_GDAT_OVERRUN_MASK)
#define ADC_SEQ_GDAT_DATAVALID_MASK              (0x80000000U)
#define ADC_SEQ_GDAT_DATAVALID_SHIFT             (31U)
#define ADC_SEQ_GDAT_DATAVALID(x)                (((uint32_t)(((uint32_t)(x)) << ADC_SEQ_GDAT_DATAVALID_SHIFT)) & ADC_SEQ_GDAT_DATAVALID_MASK)

/* The count of ADC_SEQ_GDAT */
#define ADC_SEQ_GDAT_COUNT                       (2U)

/*! @name DAT - ADC Channel 0 Data register. This register contains the result of the most recent conversion completed on channel 0. */
#define ADC_DAT_RESULT_MASK                      (0xFFF0U)
#define ADC_DAT_RESULT_SHIFT                     (4U)
#define ADC_DAT_RESULT(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_DAT_RESULT_SHIFT)) & ADC_DAT_RESULT_MASK)
#define ADC_DAT_THCMPRANGE_MASK                  (0x30000U)
#define ADC_DAT_THCMPRANGE_SHIFT                 (16U)
#define ADC_DAT_THCMPRANGE(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_DAT_THCMPRANGE_SHIFT)) & ADC_DAT_THCMPRANGE_MASK)
#define ADC_DAT_THCMPCROSS_MASK                  (0xC0000U)
#define ADC_DAT_THCMPCROSS_SHIFT                 (18U)
#define ADC_DAT_THCMPCROSS(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_DAT_THCMPCROSS_SHIFT)) & ADC_DAT_THCMPCROSS_MASK)
#define ADC_DAT_CHANNEL_MASK                     (0x3C000000U)
#define ADC_DAT_CHANNEL_SHIFT                    (26U)
#define ADC_DAT_CHANNEL(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_DAT_CHANNEL_SHIFT)) & ADC_DAT_CHANNEL_MASK)
#define ADC_DAT_OVERRUN_MASK                     (0x40000000U)
#define ADC_DAT_OVERRUN_SHIFT                    (30U)
#define ADC_DAT_OVERRUN(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_DAT_OVERRUN_SHIFT)) & ADC_DAT_OVERRUN_MASK)
#define ADC_DAT_DATAVALID_MASK                   (0x80000000U)
#define ADC_DAT_DATAVALID_SHIFT                  (31U)
#define ADC_DAT_DATAVALID(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_DAT_DATAVALID_SHIFT)) & ADC_DAT_DATAVALID_MASK)

/* The count of ADC_DAT */
#define ADC_DAT_COUNT                            (12U)

/*! @name THR0_LOW - ADC Low Compare Threshold register 0: Contains the lower threshold level for automatic threshold comparison for any channels linked to threshold pair 0. */
#define ADC_THR0_LOW_THRLOW_MASK                 (0xFFF0U)
#define ADC_THR0_LOW_THRLOW_SHIFT                (4U)
#define ADC_THR0_LOW_THRLOW(x)                   (((uint32_t)(((uint32_t)(x)) << ADC_THR0_LOW_THRLOW_SHIFT)) & ADC_THR0_LOW_THRLOW_MASK)

/*! @name THR1_LOW - ADC Low Compare Threshold register 1: Contains the lower threshold level for automatic threshold comparison for any channels linked to threshold pair 1. */
#define ADC_THR1_LOW_THRLOW_MASK                 (0xFFF0U)
#define ADC_THR1_LOW_THRLOW_SHIFT                (4U)
#define ADC_THR1_LOW_THRLOW(x)                   (((uint32_t)(((uint32_t)(x)) << ADC_THR1_LOW_THRLOW_SHIFT)) & ADC_THR1_LOW_THRLOW_MASK)

/*! @name THR0_HIGH - ADC High Compare Threshold register 0: Contains the upper threshold level for automatic threshold comparison for any channels linked to threshold pair 0. */
#define ADC_THR0_HIGH_THRHIGH_MASK               (0xFFF0U)
#define ADC_THR0_HIGH_THRHIGH_SHIFT              (4U)
#define ADC_THR0_HIGH_THRHIGH(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_THR0_HIGH_THRHIGH_SHIFT)) & ADC_THR0_HIGH_THRHIGH_MASK)

/*! @name THR1_HIGH - ADC High Compare Threshold register 1: Contains the upper threshold level for automatic threshold comparison for any channels linked to threshold pair 1. */
#define ADC_THR1_HIGH_THRHIGH_MASK               (0xFFF0U)
#define ADC_THR1_HIGH_THRHIGH_SHIFT              (4U)
#define ADC_THR1_HIGH_THRHIGH(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_THR1_HIGH_THRHIGH_SHIFT)) & ADC_THR1_HIGH_THRHIGH_MASK)

/*! @name CHAN_THRSEL - ADC Channel-Threshold Select register. Specifies which set of threshold compare registers are to be used for each channel */
#define ADC_CHAN_THRSEL_CH0_THRSEL_MASK          (0x1U)
#define ADC_CHAN_THRSEL_CH0_THRSEL_SHIFT         (0U)
#define ADC_CHAN_THRSEL_CH0_THRSEL(x)            (((uint32_t)(((uint32_t)(x)) << ADC_CHAN_THRSEL_CH0_THRSEL_SHIFT)) & ADC_CHAN_THRSEL_CH0_THRSEL_MASK)
#define ADC_CHAN_THRSEL_CH1_THRSEL_MASK          (0x2U)
#define ADC_CHAN_THRSEL_CH1_THRSEL_SHIFT         (1U)
#define ADC_CHAN_THRSEL_CH1_THRSEL(x)            (((uint32_t)(((uint32_t)(x)) << ADC_CHAN_THRSEL_CH1_THRSEL_SHIFT)) & ADC_CHAN_THRSEL_CH1_THRSEL_MASK)
#define ADC_CHAN_THRSEL_CH2_THRSEL_MASK          (0x4U)
#define ADC_CHAN_THRSEL_CH2_THRSEL_SHIFT         (2U)
#define ADC_CHAN_THRSEL_CH2_THRSEL(x)            (((uint32_t)(((uint32_t)(x)) << ADC_CHAN_THRSEL_CH2_THRSEL_SHIFT)) & ADC_CHAN_THRSEL_CH2_THRSEL_MASK)
#define ADC_CHAN_THRSEL_CH3_THRSEL_MASK          (0x8U)
#define ADC_CHAN_THRSEL_CH3_THRSEL_SHIFT         (3U)
#define ADC_CHAN_THRSEL_CH3_THRSEL(x)            (((uint32_t)(((uint32_t)(x)) << ADC_CHAN_THRSEL_CH3_THRSEL_SHIFT)) & ADC_CHAN_THRSEL_CH3_THRSEL_MASK)
#define ADC_CHAN_THRSEL_CH4_THRSEL_MASK          (0x10U)
#define ADC_CHAN_THRSEL_CH4_THRSEL_SHIFT         (4U)
#define ADC_CHAN_THRSEL_CH4_THRSEL(x)            (((uint32_t)(((uint32_t)(x)) << ADC_CHAN_THRSEL_CH4_THRSEL_SHIFT)) & ADC_CHAN_THRSEL_CH4_THRSEL_MASK)
#define ADC_CHAN_THRSEL_CH5_THRSEL_MASK          (0x20U)
#define ADC_CHAN_THRSEL_CH5_THRSEL_SHIFT         (5U)
#define ADC_CHAN_THRSEL_CH5_THRSEL(x)            (((uint32_t)(((uint32_t)(x)) << ADC_CHAN_THRSEL_CH5_THRSEL_SHIFT)) & ADC_CHAN_THRSEL_CH5_THRSEL_MASK)
#define ADC_CHAN_THRSEL_CH6_THRSEL_MASK          (0x40U)
#define ADC_CHAN_THRSEL_CH6_THRSEL_SHIFT         (6U)
#define ADC_CHAN_THRSEL_CH6_THRSEL(x)            (((uint32_t)(((uint32_t)(x)) << ADC_CHAN_THRSEL_CH6_THRSEL_SHIFT)) & ADC_CHAN_THRSEL_CH6_THRSEL_MASK)
#define ADC_CHAN_THRSEL_CH7_THRSEL_MASK          (0x80U)
#define ADC_CHAN_THRSEL_CH7_THRSEL_SHIFT         (7U)
#define ADC_CHAN_THRSEL_CH7_THRSEL(x)            (((uint32_t)(((uint32_t)(x)) << ADC_CHAN_THRSEL_CH7_THRSEL_SHIFT)) & ADC_CHAN_THRSEL_CH7_THRSEL_MASK)
#define ADC_CHAN_THRSEL_CH8_THRSEL_MASK          (0x100U)
#define ADC_CHAN_THRSEL_CH8_THRSEL_SHIFT         (8U)
#define ADC_CHAN_THRSEL_CH8_THRSEL(x)            (((uint32_t)(((uint32_t)(x)) << ADC_CHAN_THRSEL_CH8_THRSEL_SHIFT)) & ADC_CHAN_THRSEL_CH8_THRSEL_MASK)
#define ADC_CHAN_THRSEL_CH9_THRSEL_MASK          (0x200U)
#define ADC_CHAN_THRSEL_CH9_THRSEL_SHIFT         (9U)
#define ADC_CHAN_THRSEL_CH9_THRSEL(x)            (((uint32_t)(((uint32_t)(x)) << ADC_CHAN_THRSEL_CH9_THRSEL_SHIFT)) & ADC_CHAN_THRSEL_CH9_THRSEL_MASK)
#define ADC_CHAN_THRSEL_CH10_THRSEL_MASK         (0x400U)
#define ADC_CHAN_THRSEL_CH10_THRSEL_SHIFT        (10U)
#define ADC_CHAN_THRSEL_CH10_THRSEL(x)           (((uint32_t)(((uint32_t)(x)) << ADC_CHAN_THRSEL_CH10_THRSEL_SHIFT)) & ADC_CHAN_THRSEL_CH10_THRSEL_MASK)
#define ADC_CHAN_THRSEL_CH11_THRSEL_MASK         (0x800U)
#define ADC_CHAN_THRSEL_CH11_THRSEL_SHIFT        (11U)
#define ADC_CHAN_THRSEL_CH11_THRSEL(x)           (((uint32_t)(((uint32_t)(x)) << ADC_CHAN_THRSEL_CH11_THRSEL_SHIFT)) & ADC_CHAN_THRSEL_CH11_THRSEL_MASK)

/*! @name INTEN - ADC Interrupt Enable register. This register contains enable bits that enable the sequence-A, sequence-B, threshold compare and data overrun interrupts to be generated. */
#define ADC_INTEN_SEQA_INTEN_MASK                (0x1U)
#define ADC_INTEN_SEQA_INTEN_SHIFT               (0U)
#define ADC_INTEN_SEQA_INTEN(x)                  (((uint32_t)(((uint32_t)(x)) << ADC_INTEN_SEQA_INTEN_SHIFT)) & ADC_INTEN_SEQA_INTEN_MASK)
#define ADC_INTEN_SEQB_INTEN_MASK                (0x2U)
#define ADC_INTEN_SEQB_INTEN_SHIFT               (1U)
#define ADC_INTEN_SEQB_INTEN(x)                  (((uint32_t)(((uint32_t)(x)) << ADC_INTEN_SEQB_INTEN_SHIFT)) & ADC_INTEN_SEQB_INTEN_MASK)
#define ADC_INTEN_OVR_INTEN_MASK                 (0x4U)
#define ADC_INTEN_OVR_INTEN_SHIFT                (2U)
#define ADC_INTEN_OVR_INTEN(x)                   (((uint32_t)(((uint32_t)(x)) << ADC_INTEN_OVR_INTEN_SHIFT)) & ADC_INTEN_OVR_INTEN_MASK)
#define ADC_INTEN_ADCMPINTEN0_MASK               (0x18U)
#define ADC_INTEN_ADCMPINTEN0_SHIFT              (3U)
#define ADC_INTEN_ADCMPINTEN0(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_INTEN_ADCMPINTEN0_SHIFT)) & ADC_INTEN_ADCMPINTEN0_MASK)
#define ADC_INTEN_ADCMPINTEN1_MASK               (0x60U)
#define ADC_INTEN_ADCMPINTEN1_SHIFT              (5U)
#define ADC_INTEN_ADCMPINTEN1(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_INTEN_ADCMPINTEN1_SHIFT)) & ADC_INTEN_ADCMPINTEN1_MASK)
#define ADC_INTEN_ADCMPINTEN2_MASK               (0x180U)
#define ADC_INTEN_ADCMPINTEN2_SHIFT              (7U)
#define ADC_INTEN_ADCMPINTEN2(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_INTEN_ADCMPINTEN2_SHIFT)) & ADC_INTEN_ADCMPINTEN2_MASK)
#define ADC_INTEN_ADCMPINTEN3_MASK               (0x600U)
#define ADC_INTEN_ADCMPINTEN3_SHIFT              (9U)
#define ADC_INTEN_ADCMPINTEN3(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_INTEN_ADCMPINTEN3_SHIFT)) & ADC_INTEN_ADCMPINTEN3_MASK)
#define ADC_INTEN_ADCMPINTEN4_MASK               (0x1800U)
#define ADC_INTEN_ADCMPINTEN4_SHIFT              (11U)
#define ADC_INTEN_ADCMPINTEN4(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_INTEN_ADCMPINTEN4_SHIFT)) & ADC_INTEN_ADCMPINTEN4_MASK)
#define ADC_INTEN_ADCMPINTEN5_MASK               (0x6000U)
#define ADC_INTEN_ADCMPINTEN5_SHIFT              (13U)
#define ADC_INTEN_ADCMPINTEN5(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_INTEN_ADCMPINTEN5_SHIFT)) & ADC_INTEN_ADCMPINTEN5_MASK)
#define ADC_INTEN_ADCMPINTEN6_MASK               (0x18000U)
#define ADC_INTEN_ADCMPINTEN6_SHIFT              (15U)
#define ADC_INTEN_ADCMPINTEN6(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_INTEN_ADCMPINTEN6_SHIFT)) & ADC_INTEN_ADCMPINTEN6_MASK)
#define ADC_INTEN_ADCMPINTEN7_MASK               (0x60000U)
#define ADC_INTEN_ADCMPINTEN7_SHIFT              (17U)
#define ADC_INTEN_ADCMPINTEN7(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_INTEN_ADCMPINTEN7_SHIFT)) & ADC_INTEN_ADCMPINTEN7_MASK)
#define ADC_INTEN_ADCMPINTEN8_MASK               (0x180000U)
#define ADC_INTEN_ADCMPINTEN8_SHIFT              (19U)
#define ADC_INTEN_ADCMPINTEN8(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_INTEN_ADCMPINTEN8_SHIFT)) & ADC_INTEN_ADCMPINTEN8_MASK)
#define ADC_INTEN_ADCMPINTEN9_MASK               (0x600000U)
#define ADC_INTEN_ADCMPINTEN9_SHIFT              (21U)
#define ADC_INTEN_ADCMPINTEN9(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_INTEN_ADCMPINTEN9_SHIFT)) & ADC_INTEN_ADCMPINTEN9_MASK)
#define ADC_INTEN_ADCMPINTEN10_MASK              (0x1800000U)
#define ADC_INTEN_ADCMPINTEN10_SHIFT             (23U)
#define ADC_INTEN_ADCMPINTEN10(x)                (((uint32_t)(((uint32_t)(x)) << ADC_INTEN_ADCMPINTEN10_SHIFT)) & ADC_INTEN_ADCMPINTEN10_MASK)
#define ADC_INTEN_ADCMPINTEN11_MASK              (0x6000000U)
#define ADC_INTEN_ADCMPINTEN11_SHIFT             (25U)
#define ADC_INTEN_ADCMPINTEN11(x)                (((uint32_t)(((uint32_t)(x)) << ADC_INTEN_ADCMPINTEN11_SHIFT)) & ADC_INTEN_ADCMPINTEN11_MASK)

/*! @name FLAGS - ADC Flags register. Contains the four interrupt/DMA trigger flags and the individual component overrun and threshold-compare flags. (The overrun bits replicate information stored in the result registers). */
#define ADC_FLAGS_THCMP0_MASK                    (0x1U)
#define ADC_FLAGS_THCMP0_SHIFT                   (0U)
#define ADC_FLAGS_THCMP0(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_THCMP0_SHIFT)) & ADC_FLAGS_THCMP0_MASK)
#define ADC_FLAGS_THCMP1_MASK                    (0x2U)
#define ADC_FLAGS_THCMP1_SHIFT                   (1U)
#define ADC_FLAGS_THCMP1(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_THCMP1_SHIFT)) & ADC_FLAGS_THCMP1_MASK)
#define ADC_FLAGS_THCMP2_MASK                    (0x4U)
#define ADC_FLAGS_THCMP2_SHIFT                   (2U)
#define ADC_FLAGS_THCMP2(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_THCMP2_SHIFT)) & ADC_FLAGS_THCMP2_MASK)
#define ADC_FLAGS_THCMP3_MASK                    (0x8U)
#define ADC_FLAGS_THCMP3_SHIFT                   (3U)
#define ADC_FLAGS_THCMP3(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_THCMP3_SHIFT)) & ADC_FLAGS_THCMP3_MASK)
#define ADC_FLAGS_THCMP4_MASK                    (0x10U)
#define ADC_FLAGS_THCMP4_SHIFT                   (4U)
#define ADC_FLAGS_THCMP4(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_THCMP4_SHIFT)) & ADC_FLAGS_THCMP4_MASK)
#define ADC_FLAGS_THCMP5_MASK                    (0x20U)
#define ADC_FLAGS_THCMP5_SHIFT                   (5U)
#define ADC_FLAGS_THCMP5(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_THCMP5_SHIFT)) & ADC_FLAGS_THCMP5_MASK)
#define ADC_FLAGS_THCMP6_MASK                    (0x40U)
#define ADC_FLAGS_THCMP6_SHIFT                   (6U)
#define ADC_FLAGS_THCMP6(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_THCMP6_SHIFT)) & ADC_FLAGS_THCMP6_MASK)
#define ADC_FLAGS_THCMP7_MASK                    (0x80U)
#define ADC_FLAGS_THCMP7_SHIFT                   (7U)
#define ADC_FLAGS_THCMP7(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_THCMP7_SHIFT)) & ADC_FLAGS_THCMP7_MASK)
#define ADC_FLAGS_THCMP8_MASK                    (0x100U)
#define ADC_FLAGS_THCMP8_SHIFT                   (8U)
#define ADC_FLAGS_THCMP8(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_THCMP8_SHIFT)) & ADC_FLAGS_THCMP8_MASK)
#define ADC_FLAGS_THCMP9_MASK                    (0x200U)
#define ADC_FLAGS_THCMP9_SHIFT                   (9U)
#define ADC_FLAGS_THCMP9(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_THCMP9_SHIFT)) & ADC_FLAGS_THCMP9_MASK)
#define ADC_FLAGS_THCMP10_MASK                   (0x400U)
#define ADC_FLAGS_THCMP10_SHIFT                  (10U)
#define ADC_FLAGS_THCMP10(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_THCMP10_SHIFT)) & ADC_FLAGS_THCMP10_MASK)
#define ADC_FLAGS_THCMP11_MASK                   (0x800U)
#define ADC_FLAGS_THCMP11_SHIFT                  (11U)
#define ADC_FLAGS_THCMP11(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_THCMP11_SHIFT)) & ADC_FLAGS_THCMP11_MASK)
#define ADC_FLAGS_OVERRUN0_MASK                  (0x1000U)
#define ADC_FLAGS_OVERRUN0_SHIFT                 (12U)
#define ADC_FLAGS_OVERRUN0(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_OVERRUN0_SHIFT)) & ADC_FLAGS_OVERRUN0_MASK)
#define ADC_FLAGS_OVERRUN1_MASK                  (0x2000U)
#define ADC_FLAGS_OVERRUN1_SHIFT                 (13U)
#define ADC_FLAGS_OVERRUN1(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_OVERRUN1_SHIFT)) & ADC_FLAGS_OVERRUN1_MASK)
#define ADC_FLAGS_OVERRUN2_MASK                  (0x4000U)
#define ADC_FLAGS_OVERRUN2_SHIFT                 (14U)
#define ADC_FLAGS_OVERRUN2(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_OVERRUN2_SHIFT)) & ADC_FLAGS_OVERRUN2_MASK)
#define ADC_FLAGS_OVERRUN3_MASK                  (0x8000U)
#define ADC_FLAGS_OVERRUN3_SHIFT                 (15U)
#define ADC_FLAGS_OVERRUN3(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_OVERRUN3_SHIFT)) & ADC_FLAGS_OVERRUN3_MASK)
#define ADC_FLAGS_OVERRUN4_MASK                  (0x10000U)
#define ADC_FLAGS_OVERRUN4_SHIFT                 (16U)
#define ADC_FLAGS_OVERRUN4(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_OVERRUN4_SHIFT)) & ADC_FLAGS_OVERRUN4_MASK)
#define ADC_FLAGS_OVERRUN5_MASK                  (0x20000U)
#define ADC_FLAGS_OVERRUN5_SHIFT                 (17U)
#define ADC_FLAGS_OVERRUN5(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_OVERRUN5_SHIFT)) & ADC_FLAGS_OVERRUN5_MASK)
#define ADC_FLAGS_OVERRUN6_MASK                  (0x40000U)
#define ADC_FLAGS_OVERRUN6_SHIFT                 (18U)
#define ADC_FLAGS_OVERRUN6(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_OVERRUN6_SHIFT)) & ADC_FLAGS_OVERRUN6_MASK)
#define ADC_FLAGS_OVERRUN7_MASK                  (0x80000U)
#define ADC_FLAGS_OVERRUN7_SHIFT                 (19U)
#define ADC_FLAGS_OVERRUN7(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_OVERRUN7_SHIFT)) & ADC_FLAGS_OVERRUN7_MASK)
#define ADC_FLAGS_OVERRUN8_MASK                  (0x100000U)
#define ADC_FLAGS_OVERRUN8_SHIFT                 (20U)
#define ADC_FLAGS_OVERRUN8(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_OVERRUN8_SHIFT)) & ADC_FLAGS_OVERRUN8_MASK)
#define ADC_FLAGS_OVERRUN9_MASK                  (0x200000U)
#define ADC_FLAGS_OVERRUN9_SHIFT                 (21U)
#define ADC_FLAGS_OVERRUN9(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_OVERRUN9_SHIFT)) & ADC_FLAGS_OVERRUN9_MASK)
#define ADC_FLAGS_OVERRUN10_MASK                 (0x400000U)
#define ADC_FLAGS_OVERRUN10_SHIFT                (22U)
#define ADC_FLAGS_OVERRUN10(x)                   (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_OVERRUN10_SHIFT)) & ADC_FLAGS_OVERRUN10_MASK)
#define ADC_FLAGS_OVERRUN11_MASK                 (0x800000U)
#define ADC_FLAGS_OVERRUN11_SHIFT                (23U)
#define ADC_FLAGS_OVERRUN11(x)                   (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_OVERRUN11_SHIFT)) & ADC_FLAGS_OVERRUN11_MASK)
#define ADC_FLAGS_SEQA_OVR_MASK                  (0x1000000U)
#define ADC_FLAGS_SEQA_OVR_SHIFT                 (24U)
#define ADC_FLAGS_SEQA_OVR(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_SEQA_OVR_SHIFT)) & ADC_FLAGS_SEQA_OVR_MASK)
#define ADC_FLAGS_SEQB_OVR_MASK                  (0x2000000U)
#define ADC_FLAGS_SEQB_OVR_SHIFT                 (25U)
#define ADC_FLAGS_SEQB_OVR(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_SEQB_OVR_SHIFT)) & ADC_FLAGS_SEQB_OVR_MASK)
#define ADC_FLAGS_SEQA_INT_MASK                  (0x10000000U)
#define ADC_FLAGS_SEQA_INT_SHIFT                 (28U)
#define ADC_FLAGS_SEQA_INT(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_SEQA_INT_SHIFT)) & ADC_FLAGS_SEQA_INT_MASK)
#define ADC_FLAGS_SEQB_INT_MASK                  (0x20000000U)
#define ADC_FLAGS_SEQB_INT_SHIFT                 (29U)
#define ADC_FLAGS_SEQB_INT(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_SEQB_INT_SHIFT)) & ADC_FLAGS_SEQB_INT_MASK)
#define ADC_FLAGS_THCMP_INT_MASK                 (0x40000000U)
#define ADC_FLAGS_THCMP_INT_SHIFT                (30U)
#define ADC_FLAGS_THCMP_INT(x)                   (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_THCMP_INT_SHIFT)) & ADC_FLAGS_THCMP_INT_MASK)
#define ADC_FLAGS_OVR_INT_MASK                   (0x80000000U)
#define ADC_FLAGS_OVR_INT_SHIFT                  (31U)
#define ADC_FLAGS_OVR_INT(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_FLAGS_OVR_INT_SHIFT)) & ADC_FLAGS_OVR_INT_MASK)

/*! @name STARTUP - ADC Startup register. */
#define ADC_STARTUP_ADC_ENA_MASK                 (0x1U)
#define ADC_STARTUP_ADC_ENA_SHIFT                (0U)
#define ADC_STARTUP_ADC_ENA(x)                   (((uint32_t)(((uint32_t)(x)) << ADC_STARTUP_ADC_ENA_SHIFT)) & ADC_STARTUP_ADC_ENA_MASK)
#define ADC_STARTUP_ADC_INIT_MASK                (0x2U)
#define ADC_STARTUP_ADC_INIT_SHIFT               (1U)
#define ADC_STARTUP_ADC_INIT(x)                  (((uint32_t)(((uint32_t)(x)) << ADC_STARTUP_ADC_INIT_SHIFT)) & ADC_STARTUP_ADC_INIT_MASK)

/*! @name CALIB - ADC Calibration register. */
#define ADC_CALIB_CALIB_MASK                     (0x1U)
#define ADC_CALIB_CALIB_SHIFT                    (0U)
#define ADC_CALIB_CALIB(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_CALIB_CALIB_SHIFT)) & ADC_CALIB_CALIB_MASK)
#define ADC_CALIB_CALREQD_MASK                   (0x2U)
#define ADC_CALIB_CALREQD_SHIFT                  (1U)
#define ADC_CALIB_CALREQD(x)                     (((uint32_t)(((uint32_t)(x)) << ADC_CALIB_CALREQD_SHIFT)) & ADC_CALIB_CALREQD_MASK)
#define ADC_CALIB_CALVALUE_MASK                  (0x1FCU)
#define ADC_CALIB_CALVALUE_SHIFT                 (2U)
#define ADC_CALIB_CALVALUE(x)                    (((uint32_t)(((uint32_t)(x)) << ADC_CALIB_CALVALUE_SHIFT)) & ADC_CALIB_CALVALUE_MASK)


/*!
 * @}
 */ /* end of group ADC_Register_Masks */


/* ADC - Peripheral instance base addresses */
/** Peripheral ADC0 base address */
#define ADC0_BASE                                (0x400A0000u)
/** Peripheral ADC0 base pointer */
#define ADC0                                     ((ADC_Type *)ADC0_BASE)
/** Array initializer of ADC peripheral base addresses */
#define ADC_BASE_ADDRS                           { ADC0_BASE }
/** Array initializer of ADC peripheral base pointers */
#define ADC_BASE_PTRS                            { ADC0 }
/** Interrupt vectors for the ADC peripheral type */
#define ADC_SEQ_IRQS                             { ADC0_SEQA_IRQn, ADC0_SEQB_IRQn }
#define ADC_THCMP_IRQS                           { ADC0_THCMP_IRQn }

/*!
 * @}
 */ /* end of group ADC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- ASYNC_SYSCON Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ASYNC_SYSCON_Peripheral_Access_Layer ASYNC_SYSCON Peripheral Access Layer
 * @{
 */

/** ASYNC_SYSCON - Register Layout Typedef */
typedef struct {
  __IO uint32_t ASYNCPRESETCTRL;                   /**< Async peripheral reset control, offset: 0x0 */
  __O  uint32_t ASYNCPRESETCTRLSET;                /**< Set bits in ASYNCPRESETCTRL, offset: 0x4 */
  __O  uint32_t ASYNCPRESETCTRLCLR;                /**< Clear bits in ASYNCPRESETCTRL, offset: 0x8 */
       uint8_t RESERVED_0[4];
  __IO uint32_t ASYNCAPBCLKCTRL;                   /**< Async peripheral clock control, offset: 0x10 */
  __O  uint32_t ASYNCAPBCLKCTRLSET;                /**< Set bits in ASYNCAPBCLKCTRL, offset: 0x14 */
  __O  uint32_t ASYNCAPBCLKCTRLCLR;                /**< Clear bits in ASYNCAPBCLKCTRL, offset: 0x18 */
       uint8_t RESERVED_1[4];
  __IO uint32_t ASYNCAPBCLKSELA;                   /**< Async APB clock source select A, offset: 0x20 */
} ASYNC_SYSCON_Type;

/* ----------------------------------------------------------------------------
   -- ASYNC_SYSCON Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ASYNC_SYSCON_Register_Masks ASYNC_SYSCON Register Masks
 * @{
 */

/*! @name ASYNCPRESETCTRL - Async peripheral reset control */
#define ASYNC_SYSCON_ASYNCPRESETCTRL_CTIMER3_MASK (0x2000U)
#define ASYNC_SYSCON_ASYNCPRESETCTRL_CTIMER3_SHIFT (13U)
#define ASYNC_SYSCON_ASYNCPRESETCTRL_CTIMER3(x)  (((uint32_t)(((uint32_t)(x)) << ASYNC_SYSCON_ASYNCPRESETCTRL_CTIMER3_SHIFT)) & ASYNC_SYSCON_ASYNCPRESETCTRL_CTIMER3_MASK)

/*! @name ASYNCPRESETCTRLSET - Set bits in ASYNCPRESETCTRL */
#define ASYNC_SYSCON_ASYNCPRESETCTRLSET_ARST_SET_MASK (0xFFFFFFFFU)
#define ASYNC_SYSCON_ASYNCPRESETCTRLSET_ARST_SET_SHIFT (0U)
#define ASYNC_SYSCON_ASYNCPRESETCTRLSET_ARST_SET(x) (((uint32_t)(((uint32_t)(x)) << ASYNC_SYSCON_ASYNCPRESETCTRLSET_ARST_SET_SHIFT)) & ASYNC_SYSCON_ASYNCPRESETCTRLSET_ARST_SET_MASK)

/*! @name ASYNCPRESETCTRLCLR - Clear bits in ASYNCPRESETCTRL */
#define ASYNC_SYSCON_ASYNCPRESETCTRLCLR_ARST_CLR_MASK (0xFFFFFFFFU)
#define ASYNC_SYSCON_ASYNCPRESETCTRLCLR_ARST_CLR_SHIFT (0U)
#define ASYNC_SYSCON_ASYNCPRESETCTRLCLR_ARST_CLR(x) (((uint32_t)(((uint32_t)(x)) << ASYNC_SYSCON_ASYNCPRESETCTRLCLR_ARST_CLR_SHIFT)) & ASYNC_SYSCON_ASYNCPRESETCTRLCLR_ARST_CLR_MASK)

/*! @name ASYNCAPBCLKCTRL - Async peripheral clock control */
#define ASYNC_SYSCON_ASYNCAPBCLKCTRL_CTIMER3_MASK (0x2000U)
#define ASYNC_SYSCON_ASYNCAPBCLKCTRL_CTIMER3_SHIFT (13U)
#define ASYNC_SYSCON_ASYNCAPBCLKCTRL_CTIMER3(x)  (((uint32_t)(((uint32_t)(x)) << ASYNC_SYSCON_ASYNCAPBCLKCTRL_CTIMER3_SHIFT)) & ASYNC_SYSCON_ASYNCAPBCLKCTRL_CTIMER3_MASK)

/*! @name ASYNCAPBCLKCTRLSET - Set bits in ASYNCAPBCLKCTRL */
#define ASYNC_SYSCON_ASYNCAPBCLKCTRLSET_ACLK_SET_MASK (0xFFFFFFFFU)
#define ASYNC_SYSCON_ASYNCAPBCLKCTRLSET_ACLK_SET_SHIFT (0U)
#define ASYNC_SYSCON_ASYNCAPBCLKCTRLSET_ACLK_SET(x) (((uint32_t)(((uint32_t)(x)) << ASYNC_SYSCON_ASYNCAPBCLKCTRLSET_ACLK_SET_SHIFT)) & ASYNC_SYSCON_ASYNCAPBCLKCTRLSET_ACLK_SET_MASK)

/*! @name ASYNCAPBCLKCTRLCLR - Clear bits in ASYNCAPBCLKCTRL */
#define ASYNC_SYSCON_ASYNCAPBCLKCTRLCLR_ACLK_CLR_MASK (0xFFFFFFFFU)
#define ASYNC_SYSCON_ASYNCAPBCLKCTRLCLR_ACLK_CLR_SHIFT (0U)
#define ASYNC_SYSCON_ASYNCAPBCLKCTRLCLR_ACLK_CLR(x) (((uint32_t)(((uint32_t)(x)) << ASYNC_SYSCON_ASYNCAPBCLKCTRLCLR_ACLK_CLR_SHIFT)) & ASYNC_SYSCON_ASYNCAPBCLKCTRLCLR_ACLK_CLR_MASK)

/*! @name ASYNCAPBCLKSELA - Async APB clock source select A */
#define ASYNC_SYSCON_ASYNCAPBCLKSELA_SEL_MASK    (0x3U)
#define ASYNC_SYSCON_ASYNCAPBCLKSELA_SEL_SHIFT   (0U)
#define ASYNC_SYSCON_ASYNCAPBCLKSELA_SEL(x)      (((uint32_t)(((uint32_t)(x)) << ASYNC_SYSCON_ASYNCAPBCLKSELA_SEL_SHIFT)) & ASYNC_SYSCON_ASYNCAPBCLKSELA_SEL_MASK)


/*!
 * @}
 */ /* end of group ASYNC_SYSCON_Register_Masks */


/* ASYNC_SYSCON - Peripheral instance base addresses */
/** Peripheral ASYNC_SYSCON base address */
#define ASYNC_SYSCON_BASE                        (0x40040000u)
/** Peripheral ASYNC_SYSCON base pointer */
#define ASYNC_SYSCON                             ((ASYNC_SYSCON_Type *)ASYNC_SYSCON_BASE)
/** Array initializer of ASYNC_SYSCON peripheral base addresses */
#define ASYNC_SYSCON_BASE_ADDRS                  { ASYNC_SYSCON_BASE }
/** Array initializer of ASYNC_SYSCON peripheral base pointers */
#define ASYNC_SYSCON_BASE_PTRS                   { ASYNC_SYSCON }

/*!
 * @}
 */ /* end of group ASYNC_SYSCON_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CRC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Peripheral_Access_Layer CRC Peripheral Access Layer
 * @{
 */

/** CRC - Register Layout Typedef */
typedef struct {
  __IO uint32_t MODE;                              /**< CRC mode register, offset: 0x0 */
  __IO uint32_t SEED;                              /**< CRC seed register, offset: 0x4 */
  union {                                          /* offset: 0x8 */
    __I  uint32_t SUM;                               /**< CRC checksum register, offset: 0x8 */
    __O  uint32_t WR_DATA;                           /**< CRC data register, offset: 0x8 */
  };
} CRC_Type;

/* ----------------------------------------------------------------------------
   -- CRC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Register_Masks CRC Register Masks
 * @{
 */

/*! @name MODE - CRC mode register */
#define CRC_MODE_CRC_POLY_MASK                   (0x3U)
#define CRC_MODE_CRC_POLY_SHIFT                  (0U)
#define CRC_MODE_CRC_POLY(x)                     (((uint32_t)(((uint32_t)(x)) << CRC_MODE_CRC_POLY_SHIFT)) & CRC_MODE_CRC_POLY_MASK)
#define CRC_MODE_BIT_RVS_WR_MASK                 (0x4U)
#define CRC_MODE_BIT_RVS_WR_SHIFT                (2U)
#define CRC_MODE_BIT_RVS_WR(x)                   (((uint32_t)(((uint32_t)(x)) << CRC_MODE_BIT_RVS_WR_SHIFT)) & CRC_MODE_BIT_RVS_WR_MASK)
#define CRC_MODE_CMPL_WR_MASK                    (0x8U)
#define CRC_MODE_CMPL_WR_SHIFT                   (3U)
#define CRC_MODE_CMPL_WR(x)                      (((uint32_t)(((uint32_t)(x)) << CRC_MODE_CMPL_WR_SHIFT)) & CRC_MODE_CMPL_WR_MASK)
#define CRC_MODE_BIT_RVS_SUM_MASK                (0x10U)
#define CRC_MODE_BIT_RVS_SUM_SHIFT               (4U)
#define CRC_MODE_BIT_RVS_SUM(x)                  (((uint32_t)(((uint32_t)(x)) << CRC_MODE_BIT_RVS_SUM_SHIFT)) & CRC_MODE_BIT_RVS_SUM_MASK)
#define CRC_MODE_CMPL_SUM_MASK                   (0x20U)
#define CRC_MODE_CMPL_SUM_SHIFT                  (5U)
#define CRC_MODE_CMPL_SUM(x)                     (((uint32_t)(((uint32_t)(x)) << CRC_MODE_CMPL_SUM_SHIFT)) & CRC_MODE_CMPL_SUM_MASK)

/*! @name SEED - CRC seed register */
#define CRC_SEED_CRC_SEED_MASK                   (0xFFFFFFFFU)
#define CRC_SEED_CRC_SEED_SHIFT                  (0U)
#define CRC_SEED_CRC_SEED(x)                     (((uint32_t)(((uint32_t)(x)) << CRC_SEED_CRC_SEED_SHIFT)) & CRC_SEED_CRC_SEED_MASK)

/*! @name SUM - CRC checksum register */
#define CRC_SUM_CRC_SUM_MASK                     (0xFFFFFFFFU)
#define CRC_SUM_CRC_SUM_SHIFT                    (0U)
#define CRC_SUM_CRC_SUM(x)                       (((uint32_t)(((uint32_t)(x)) << CRC_SUM_CRC_SUM_SHIFT)) & CRC_SUM_CRC_SUM_MASK)

/*! @name WR_DATA - CRC data register */
#define CRC_WR_DATA_CRC_WR_DATA_MASK             (0xFFFFFFFFU)
#define CRC_WR_DATA_CRC_WR_DATA_SHIFT            (0U)
#define CRC_WR_DATA_CRC_WR_DATA(x)               (((uint32_t)(((uint32_t)(x)) << CRC_WR_DATA_CRC_WR_DATA_SHIFT)) & CRC_WR_DATA_CRC_WR_DATA_MASK)


/*!
 * @}
 */ /* end of group CRC_Register_Masks */


/* CRC - Peripheral instance base addresses */
/** Peripheral CRC_ENGINE base address */
#define CRC_ENGINE_BASE                          (0x40095000u)
/** Peripheral CRC_ENGINE base pointer */
#define CRC_ENGINE                               ((CRC_Type *)CRC_ENGINE_BASE)
/** Array initializer of CRC peripheral base addresses */
#define CRC_BASE_ADDRS                           { CRC_ENGINE_BASE }
/** Array initializer of CRC peripheral base pointers */
#define CRC_BASE_PTRS                            { CRC_ENGINE }

/*!
 * @}
 */ /* end of group CRC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CTIMER Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CTIMER_Peripheral_Access_Layer CTIMER Peripheral Access Layer
 * @{
 */

/** CTIMER - Register Layout Typedef */
typedef struct {
  __IO uint32_t IR;                                /**< Interrupt Register. The IR can be written to clear interrupts. The IR can be read to identify which of eight possible interrupt sources are pending., offset: 0x0 */
  __IO uint32_t TCR;                               /**< Timer Control Register. The TCR is used to control the Timer Counter functions. The Timer Counter can be disabled or reset through the TCR., offset: 0x4 */
  __IO uint32_t TC;                                /**< Timer Counter. The 32 bit TC is incremented every PR+1 cycles of the APB bus clock. The TC is controlled through the TCR., offset: 0x8 */
  __IO uint32_t PR;                                /**< Prescale Register. When the Prescale Counter (PC) is equal to this value, the next clock increments the TC and clears the PC., offset: 0xC */
  __IO uint32_t PC;                                /**< Prescale Counter. The 32 bit PC is a counter which is incremented to the value stored in PR. When the value in PR is reached, the TC is incremented and the PC is cleared. The PC is observable and controllable through the bus interface., offset: 0x10 */
  __IO uint32_t MCR;                               /**< Match Control Register. The MCR is used to control if an interrupt is generated and if the TC is reset when a Match occurs., offset: 0x14 */
  __IO uint32_t MR[4];                             /**< Match Register . MR can be enabled through the MCR to reset the TC, stop both the TC and PC, and/or generate an interrupt every time MR matches the TC., array offset: 0x18, array step: 0x4 */
  __IO uint32_t CCR;                               /**< Capture Control Register. The CCR controls which edges of the capture inputs are used to load the Capture Registers and whether or not an interrupt is generated when a capture takes place., offset: 0x28 */
  __I  uint32_t CR[4];                             /**< Capture Register . CR is loaded with the value of TC when there is an event on the CAPn. input., array offset: 0x2C, array step: 0x4 */
  __IO uint32_t EMR;                               /**< External Match Register. The EMR controls the match function and the external match pins., offset: 0x3C */
       uint8_t RESERVED_0[48];
  __IO uint32_t CTCR;                              /**< Count Control Register. The CTCR selects between Timer and Counter mode, and in Counter mode selects the signal and edge(s) for counting., offset: 0x70 */
  __IO uint32_t PWMC;                              /**< PWM Control Register. The PWMCON enables PWM mode for the external match pins., offset: 0x74 */
} CTIMER_Type;

/* ----------------------------------------------------------------------------
   -- CTIMER Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CTIMER_Register_Masks CTIMER Register Masks
 * @{
 */

/*! @name IR - Interrupt Register. The IR can be written to clear interrupts. The IR can be read to identify which of eight possible interrupt sources are pending. */
#define CTIMER_IR_MR0INT_MASK                    (0x1U)
#define CTIMER_IR_MR0INT_SHIFT                   (0U)
#define CTIMER_IR_MR0INT(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_IR_MR0INT_SHIFT)) & CTIMER_IR_MR0INT_MASK)
#define CTIMER_IR_MR1INT_MASK                    (0x2U)
#define CTIMER_IR_MR1INT_SHIFT                   (1U)
#define CTIMER_IR_MR1INT(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_IR_MR1INT_SHIFT)) & CTIMER_IR_MR1INT_MASK)
#define CTIMER_IR_MR2INT_MASK                    (0x4U)
#define CTIMER_IR_MR2INT_SHIFT                   (2U)
#define CTIMER_IR_MR2INT(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_IR_MR2INT_SHIFT)) & CTIMER_IR_MR2INT_MASK)
#define CTIMER_IR_MR3INT_MASK                    (0x8U)
#define CTIMER_IR_MR3INT_SHIFT                   (3U)
#define CTIMER_IR_MR3INT(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_IR_MR3INT_SHIFT)) & CTIMER_IR_MR3INT_MASK)
#define CTIMER_IR_CR0INT_MASK                    (0x10U)
#define CTIMER_IR_CR0INT_SHIFT                   (4U)
#define CTIMER_IR_CR0INT(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_IR_CR0INT_SHIFT)) & CTIMER_IR_CR0INT_MASK)
#define CTIMER_IR_CR1INT_MASK                    (0x20U)
#define CTIMER_IR_CR1INT_SHIFT                   (5U)
#define CTIMER_IR_CR1INT(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_IR_CR1INT_SHIFT)) & CTIMER_IR_CR1INT_MASK)
#define CTIMER_IR_CR2INT_MASK                    (0x40U)
#define CTIMER_IR_CR2INT_SHIFT                   (6U)
#define CTIMER_IR_CR2INT(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_IR_CR2INT_SHIFT)) & CTIMER_IR_CR2INT_MASK)
#define CTIMER_IR_CR3INT_MASK                    (0x80U)
#define CTIMER_IR_CR3INT_SHIFT                   (7U)
#define CTIMER_IR_CR3INT(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_IR_CR3INT_SHIFT)) & CTIMER_IR_CR3INT_MASK)

/*! @name TCR - Timer Control Register. The TCR is used to control the Timer Counter functions. The Timer Counter can be disabled or reset through the TCR. */
#define CTIMER_TCR_CEN_MASK                      (0x1U)
#define CTIMER_TCR_CEN_SHIFT                     (0U)
#define CTIMER_TCR_CEN(x)                        (((uint32_t)(((uint32_t)(x)) << CTIMER_TCR_CEN_SHIFT)) & CTIMER_TCR_CEN_MASK)
#define CTIMER_TCR_CRST_MASK                     (0x2U)
#define CTIMER_TCR_CRST_SHIFT                    (1U)
#define CTIMER_TCR_CRST(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_TCR_CRST_SHIFT)) & CTIMER_TCR_CRST_MASK)

/*! @name TC - Timer Counter. The 32 bit TC is incremented every PR+1 cycles of the APB bus clock. The TC is controlled through the TCR. */
#define CTIMER_TC_TCVAL_MASK                     (0xFFFFFFFFU)
#define CTIMER_TC_TCVAL_SHIFT                    (0U)
#define CTIMER_TC_TCVAL(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_TC_TCVAL_SHIFT)) & CTIMER_TC_TCVAL_MASK)

/*! @name PR - Prescale Register. When the Prescale Counter (PC) is equal to this value, the next clock increments the TC and clears the PC. */
#define CTIMER_PR_PRVAL_MASK                     (0xFFFFFFFFU)
#define CTIMER_PR_PRVAL_SHIFT                    (0U)
#define CTIMER_PR_PRVAL(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_PR_PRVAL_SHIFT)) & CTIMER_PR_PRVAL_MASK)

/*! @name PC - Prescale Counter. The 32 bit PC is a counter which is incremented to the value stored in PR. When the value in PR is reached, the TC is incremented and the PC is cleared. The PC is observable and controllable through the bus interface. */
#define CTIMER_PC_PCVAL_MASK                     (0xFFFFFFFFU)
#define CTIMER_PC_PCVAL_SHIFT                    (0U)
#define CTIMER_PC_PCVAL(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_PC_PCVAL_SHIFT)) & CTIMER_PC_PCVAL_MASK)

/*! @name MCR - Match Control Register. The MCR is used to control if an interrupt is generated and if the TC is reset when a Match occurs. */
#define CTIMER_MCR_MR0I_MASK                     (0x1U)
#define CTIMER_MCR_MR0I_SHIFT                    (0U)
#define CTIMER_MCR_MR0I(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR0I_SHIFT)) & CTIMER_MCR_MR0I_MASK)
#define CTIMER_MCR_MR0R_MASK                     (0x2U)
#define CTIMER_MCR_MR0R_SHIFT                    (1U)
#define CTIMER_MCR_MR0R(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR0R_SHIFT)) & CTIMER_MCR_MR0R_MASK)
#define CTIMER_MCR_MR0S_MASK                     (0x4U)
#define CTIMER_MCR_MR0S_SHIFT                    (2U)
#define CTIMER_MCR_MR0S(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR0S_SHIFT)) & CTIMER_MCR_MR0S_MASK)
#define CTIMER_MCR_MR1I_MASK                     (0x8U)
#define CTIMER_MCR_MR1I_SHIFT                    (3U)
#define CTIMER_MCR_MR1I(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR1I_SHIFT)) & CTIMER_MCR_MR1I_MASK)
#define CTIMER_MCR_MR1R_MASK                     (0x10U)
#define CTIMER_MCR_MR1R_SHIFT                    (4U)
#define CTIMER_MCR_MR1R(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR1R_SHIFT)) & CTIMER_MCR_MR1R_MASK)
#define CTIMER_MCR_MR1S_MASK                     (0x20U)
#define CTIMER_MCR_MR1S_SHIFT                    (5U)
#define CTIMER_MCR_MR1S(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR1S_SHIFT)) & CTIMER_MCR_MR1S_MASK)
#define CTIMER_MCR_MR2I_MASK                     (0x40U)
#define CTIMER_MCR_MR2I_SHIFT                    (6U)
#define CTIMER_MCR_MR2I(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR2I_SHIFT)) & CTIMER_MCR_MR2I_MASK)
#define CTIMER_MCR_MR2R_MASK                     (0x80U)
#define CTIMER_MCR_MR2R_SHIFT                    (7U)
#define CTIMER_MCR_MR2R(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR2R_SHIFT)) & CTIMER_MCR_MR2R_MASK)
#define CTIMER_MCR_MR2S_MASK                     (0x100U)
#define CTIMER_MCR_MR2S_SHIFT                    (8U)
#define CTIMER_MCR_MR2S(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR2S_SHIFT)) & CTIMER_MCR_MR2S_MASK)
#define CTIMER_MCR_MR3I_MASK                     (0x200U)
#define CTIMER_MCR_MR3I_SHIFT                    (9U)
#define CTIMER_MCR_MR3I(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR3I_SHIFT)) & CTIMER_MCR_MR3I_MASK)
#define CTIMER_MCR_MR3R_MASK                     (0x400U)
#define CTIMER_MCR_MR3R_SHIFT                    (10U)
#define CTIMER_MCR_MR3R(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR3R_SHIFT)) & CTIMER_MCR_MR3R_MASK)
#define CTIMER_MCR_MR3S_MASK                     (0x800U)
#define CTIMER_MCR_MR3S_SHIFT                    (11U)
#define CTIMER_MCR_MR3S(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MCR_MR3S_SHIFT)) & CTIMER_MCR_MR3S_MASK)

/*! @name MR - Match Register . MR can be enabled through the MCR to reset the TC, stop both the TC and PC, and/or generate an interrupt every time MR matches the TC. */
#define CTIMER_MR_MATCH_MASK                     (0xFFFFFFFFU)
#define CTIMER_MR_MATCH_SHIFT                    (0U)
#define CTIMER_MR_MATCH(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_MR_MATCH_SHIFT)) & CTIMER_MR_MATCH_MASK)

/* The count of CTIMER_MR */
#define CTIMER_MR_COUNT                          (4U)

/*! @name CCR - Capture Control Register. The CCR controls which edges of the capture inputs are used to load the Capture Registers and whether or not an interrupt is generated when a capture takes place. */
#define CTIMER_CCR_CAP0RE_MASK                   (0x1U)
#define CTIMER_CCR_CAP0RE_SHIFT                  (0U)
#define CTIMER_CCR_CAP0RE(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP0RE_SHIFT)) & CTIMER_CCR_CAP0RE_MASK)
#define CTIMER_CCR_CAP0FE_MASK                   (0x2U)
#define CTIMER_CCR_CAP0FE_SHIFT                  (1U)
#define CTIMER_CCR_CAP0FE(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP0FE_SHIFT)) & CTIMER_CCR_CAP0FE_MASK)
#define CTIMER_CCR_CAP0I_MASK                    (0x4U)
#define CTIMER_CCR_CAP0I_SHIFT                   (2U)
#define CTIMER_CCR_CAP0I(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP0I_SHIFT)) & CTIMER_CCR_CAP0I_MASK)
#define CTIMER_CCR_CAP1RE_MASK                   (0x8U)
#define CTIMER_CCR_CAP1RE_SHIFT                  (3U)
#define CTIMER_CCR_CAP1RE(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP1RE_SHIFT)) & CTIMER_CCR_CAP1RE_MASK)
#define CTIMER_CCR_CAP1FE_MASK                   (0x10U)
#define CTIMER_CCR_CAP1FE_SHIFT                  (4U)
#define CTIMER_CCR_CAP1FE(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP1FE_SHIFT)) & CTIMER_CCR_CAP1FE_MASK)
#define CTIMER_CCR_CAP1I_MASK                    (0x20U)
#define CTIMER_CCR_CAP1I_SHIFT                   (5U)
#define CTIMER_CCR_CAP1I(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP1I_SHIFT)) & CTIMER_CCR_CAP1I_MASK)
#define CTIMER_CCR_CAP2RE_MASK                   (0x40U)
#define CTIMER_CCR_CAP2RE_SHIFT                  (6U)
#define CTIMER_CCR_CAP2RE(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP2RE_SHIFT)) & CTIMER_CCR_CAP2RE_MASK)
#define CTIMER_CCR_CAP2FE_MASK                   (0x80U)
#define CTIMER_CCR_CAP2FE_SHIFT                  (7U)
#define CTIMER_CCR_CAP2FE(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP2FE_SHIFT)) & CTIMER_CCR_CAP2FE_MASK)
#define CTIMER_CCR_CAP2I_MASK                    (0x100U)
#define CTIMER_CCR_CAP2I_SHIFT                   (8U)
#define CTIMER_CCR_CAP2I(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP2I_SHIFT)) & CTIMER_CCR_CAP2I_MASK)
#define CTIMER_CCR_CAP3RE_MASK                   (0x200U)
#define CTIMER_CCR_CAP3RE_SHIFT                  (9U)
#define CTIMER_CCR_CAP3RE(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP3RE_SHIFT)) & CTIMER_CCR_CAP3RE_MASK)
#define CTIMER_CCR_CAP3FE_MASK                   (0x400U)
#define CTIMER_CCR_CAP3FE_SHIFT                  (10U)
#define CTIMER_CCR_CAP3FE(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP3FE_SHIFT)) & CTIMER_CCR_CAP3FE_MASK)
#define CTIMER_CCR_CAP3I_MASK                    (0x800U)
#define CTIMER_CCR_CAP3I_SHIFT                   (11U)
#define CTIMER_CCR_CAP3I(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_CCR_CAP3I_SHIFT)) & CTIMER_CCR_CAP3I_MASK)

/*! @name CR - Capture Register . CR is loaded with the value of TC when there is an event on the CAPn. input. */
#define CTIMER_CR_CAP_MASK                       (0xFFFFFFFFU)
#define CTIMER_CR_CAP_SHIFT                      (0U)
#define CTIMER_CR_CAP(x)                         (((uint32_t)(((uint32_t)(x)) << CTIMER_CR_CAP_SHIFT)) & CTIMER_CR_CAP_MASK)

/* The count of CTIMER_CR */
#define CTIMER_CR_COUNT                          (4U)

/*! @name EMR - External Match Register. The EMR controls the match function and the external match pins. */
#define CTIMER_EMR_EM0_MASK                      (0x1U)
#define CTIMER_EMR_EM0_SHIFT                     (0U)
#define CTIMER_EMR_EM0(x)                        (((uint32_t)(((uint32_t)(x)) << CTIMER_EMR_EM0_SHIFT)) & CTIMER_EMR_EM0_MASK)
#define CTIMER_EMR_EM1_MASK                      (0x2U)
#define CTIMER_EMR_EM1_SHIFT                     (1U)
#define CTIMER_EMR_EM1(x)                        (((uint32_t)(((uint32_t)(x)) << CTIMER_EMR_EM1_SHIFT)) & CTIMER_EMR_EM1_MASK)
#define CTIMER_EMR_EM2_MASK                      (0x4U)
#define CTIMER_EMR_EM2_SHIFT                     (2U)
#define CTIMER_EMR_EM2(x)                        (((uint32_t)(((uint32_t)(x)) << CTIMER_EMR_EM2_SHIFT)) & CTIMER_EMR_EM2_MASK)
#define CTIMER_EMR_EM3_MASK                      (0x8U)
#define CTIMER_EMR_EM3_SHIFT                     (3U)
#define CTIMER_EMR_EM3(x)                        (((uint32_t)(((uint32_t)(x)) << CTIMER_EMR_EM3_SHIFT)) & CTIMER_EMR_EM3_MASK)
#define CTIMER_EMR_EMC0_MASK                     (0x30U)
#define CTIMER_EMR_EMC0_SHIFT                    (4U)
#define CTIMER_EMR_EMC0(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_EMR_EMC0_SHIFT)) & CTIMER_EMR_EMC0_MASK)
#define CTIMER_EMR_EMC1_MASK                     (0xC0U)
#define CTIMER_EMR_EMC1_SHIFT                    (6U)
#define CTIMER_EMR_EMC1(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_EMR_EMC1_SHIFT)) & CTIMER_EMR_EMC1_MASK)
#define CTIMER_EMR_EMC2_MASK                     (0x300U)
#define CTIMER_EMR_EMC2_SHIFT                    (8U)
#define CTIMER_EMR_EMC2(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_EMR_EMC2_SHIFT)) & CTIMER_EMR_EMC2_MASK)
#define CTIMER_EMR_EMC3_MASK                     (0xC00U)
#define CTIMER_EMR_EMC3_SHIFT                    (10U)
#define CTIMER_EMR_EMC3(x)                       (((uint32_t)(((uint32_t)(x)) << CTIMER_EMR_EMC3_SHIFT)) & CTIMER_EMR_EMC3_MASK)

/*! @name CTCR - Count Control Register. The CTCR selects between Timer and Counter mode, and in Counter mode selects the signal and edge(s) for counting. */
#define CTIMER_CTCR_CTMODE_MASK                  (0x3U)
#define CTIMER_CTCR_CTMODE_SHIFT                 (0U)
#define CTIMER_CTCR_CTMODE(x)                    (((uint32_t)(((uint32_t)(x)) << CTIMER_CTCR_CTMODE_SHIFT)) & CTIMER_CTCR_CTMODE_MASK)
#define CTIMER_CTCR_CINSEL_MASK                  (0xCU)
#define CTIMER_CTCR_CINSEL_SHIFT                 (2U)
#define CTIMER_CTCR_CINSEL(x)                    (((uint32_t)(((uint32_t)(x)) << CTIMER_CTCR_CINSEL_SHIFT)) & CTIMER_CTCR_CINSEL_MASK)
#define CTIMER_CTCR_ENCC_MASK                    (0x10U)
#define CTIMER_CTCR_ENCC_SHIFT                   (4U)
#define CTIMER_CTCR_ENCC(x)                      (((uint32_t)(((uint32_t)(x)) << CTIMER_CTCR_ENCC_SHIFT)) & CTIMER_CTCR_ENCC_MASK)
#define CTIMER_CTCR_SELCC_MASK                   (0xE0U)
#define CTIMER_CTCR_SELCC_SHIFT                  (5U)
#define CTIMER_CTCR_SELCC(x)                     (((uint32_t)(((uint32_t)(x)) << CTIMER_CTCR_SELCC_SHIFT)) & CTIMER_CTCR_SELCC_MASK)

/*! @name PWMC - PWM Control Register. The PWMCON enables PWM mode for the external match pins. */
#define CTIMER_PWMC_PWMEN0_MASK                  (0x1U)
#define CTIMER_PWMC_PWMEN0_SHIFT                 (0U)
#define CTIMER_PWMC_PWMEN0(x)                    (((uint32_t)(((uint32_t)(x)) << CTIMER_PWMC_PWMEN0_SHIFT)) & CTIMER_PWMC_PWMEN0_MASK)
#define CTIMER_PWMC_PWMEN1_MASK                  (0x2U)
#define CTIMER_PWMC_PWMEN1_SHIFT                 (1U)
#define CTIMER_PWMC_PWMEN1(x)                    (((uint32_t)(((uint32_t)(x)) << CTIMER_PWMC_PWMEN1_SHIFT)) & CTIMER_PWMC_PWMEN1_MASK)
#define CTIMER_PWMC_PWMEN2_MASK                  (0x4U)
#define CTIMER_PWMC_PWMEN2_SHIFT                 (2U)
#define CTIMER_PWMC_PWMEN2(x)                    (((uint32_t)(((uint32_t)(x)) << CTIMER_PWMC_PWMEN2_SHIFT)) & CTIMER_PWMC_PWMEN2_MASK)
#define CTIMER_PWMC_PWMEN3_MASK                  (0x8U)
#define CTIMER_PWMC_PWMEN3_SHIFT                 (3U)
#define CTIMER_PWMC_PWMEN3(x)                    (((uint32_t)(((uint32_t)(x)) << CTIMER_PWMC_PWMEN3_SHIFT)) & CTIMER_PWMC_PWMEN3_MASK)


/*!
 * @}
 */ /* end of group CTIMER_Register_Masks */


/* CTIMER - Peripheral instance base addresses */
/** Peripheral CTIMER0 base address */
#define CTIMER0_BASE                             (0x40008000u)
/** Peripheral CTIMER0 base pointer */
#define CTIMER0                                  ((CTIMER_Type *)CTIMER0_BASE)
/** Peripheral CTIMER1 base address */
#define CTIMER1_BASE                             (0x40009000u)
/** Peripheral CTIMER1 base pointer */
#define CTIMER1                                  ((CTIMER_Type *)CTIMER1_BASE)
/** Peripheral CTIMER3 base address */
#define CTIMER3_BASE                             (0x40048000u)
/** Peripheral CTIMER3 base pointer */
#define CTIMER3                                  ((CTIMER_Type *)CTIMER3_BASE)
/** Array initializer of CTIMER peripheral base addresses */
#define CTIMER_BASE_ADDRS                        { CTIMER0_BASE, CTIMER1_BASE, CTIMER3_BASE }
/** Array initializer of CTIMER peripheral base pointers */
#define CTIMER_BASE_PTRS                         { CTIMER0, CTIMER1, CTIMER3 }
/** Interrupt vectors for the CTIMER peripheral type */
#define CTIMER_IRQS                              { CTIMER0_IRQn, CTIMER1_IRQn, CTIMER3_IRQn }

/*!
 * @}
 */ /* end of group CTIMER_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- DMA Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Peripheral_Access_Layer DMA Peripheral Access Layer
 * @{
 */

/** DMA - Register Layout Typedef */
typedef struct {
  __IO uint32_t CTRL;                              /**< DMA control., offset: 0x0 */
  __I  uint32_t INTSTAT;                           /**< Interrupt status., offset: 0x4 */
  __IO uint32_t SRAMBASE;                          /**< SRAM address of the channel configuration table., offset: 0x8 */
       uint8_t RESERVED_0[20];
  struct {                                         /* offset: 0x20, array step: 0x5C */
    __IO uint32_t ENABLESET;                         /**< Channel Enable read and Set for all DMA channels., array offset: 0x20, array step: 0x5C */
         uint8_t RESERVED_0[4];
    __O  uint32_t ENABLECLR;                         /**< Channel Enable Clear for all DMA channels., array offset: 0x28, array step: 0x5C */
         uint8_t RESERVED_1[4];
    __I  uint32_t ACTIVE;                            /**< Channel Active status for all DMA channels., array offset: 0x30, array step: 0x5C */
         uint8_t RESERVED_2[4];
    __I  uint32_t BUSY;                              /**< Channel Busy status for all DMA channels., array offset: 0x38, array step: 0x5C */
         uint8_t RESERVED_3[4];
    __IO uint32_t ERRINT;                            /**< Error Interrupt status for all DMA channels., array offset: 0x40, array step: 0x5C */
         uint8_t RESERVED_4[4];
    __IO uint32_t INTENSET;                          /**< Interrupt Enable read and Set for all DMA channels., array offset: 0x48, array step: 0x5C */
         uint8_t RESERVED_5[4];
    __O  uint32_t INTENCLR;                          /**< Interrupt Enable Clear for all DMA channels., array offset: 0x50, array step: 0x5C */
         uint8_t RESERVED_6[4];
    __IO uint32_t INTA;                              /**< Interrupt A status for all DMA channels., array offset: 0x58, array step: 0x5C */
         uint8_t RESERVED_7[4];
    __IO uint32_t INTB;                              /**< Interrupt B status for all DMA channels., array offset: 0x60, array step: 0x5C */
         uint8_t RESERVED_8[4];
    __O  uint32_t SETVALID;                          /**< Set ValidPending control bits for all DMA channels., array offset: 0x68, array step: 0x5C */
         uint8_t RESERVED_9[4];
    __O  uint32_t SETTRIG;                           /**< Set Trigger control bits for all DMA channels., array offset: 0x70, array step: 0x5C */
         uint8_t RESERVED_10[4];
    __O  uint32_t ABORT;                             /**< Channel Abort control for all DMA channels., array offset: 0x78, array step: 0x5C */
  } COMMON[1];
       uint8_t RESERVED_1[900];
  struct {                                         /* offset: 0x400, array step: 0x10 */
    __IO uint32_t CFG;                               /**< Configuration register for DMA channel ., array offset: 0x400, array step: 0x10 */
    __I  uint32_t CTLSTAT;                           /**< Control and status register for DMA channel ., array offset: 0x404, array step: 0x10 */
    __IO uint32_t XFERCFG;                           /**< Transfer configuration register for DMA channel ., array offset: 0x408, array step: 0x10 */
         uint8_t RESERVED_0[4];
  } CHANNEL[20];
} DMA_Type;

/* ----------------------------------------------------------------------------
   -- DMA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Masks DMA Register Masks
 * @{
 */

/*! @name CTRL - DMA control. */
#define DMA_CTRL_ENABLE_MASK                     (0x1U)
#define DMA_CTRL_ENABLE_SHIFT                    (0U)
#define DMA_CTRL_ENABLE(x)                       (((uint32_t)(((uint32_t)(x)) << DMA_CTRL_ENABLE_SHIFT)) & DMA_CTRL_ENABLE_MASK)

/*! @name INTSTAT - Interrupt status. */
#define DMA_INTSTAT_ACTIVEINT_MASK               (0x2U)
#define DMA_INTSTAT_ACTIVEINT_SHIFT              (1U)
#define DMA_INTSTAT_ACTIVEINT(x)                 (((uint32_t)(((uint32_t)(x)) << DMA_INTSTAT_ACTIVEINT_SHIFT)) & DMA_INTSTAT_ACTIVEINT_MASK)
#define DMA_INTSTAT_ACTIVEERRINT_MASK            (0x4U)
#define DMA_INTSTAT_ACTIVEERRINT_SHIFT           (2U)
#define DMA_INTSTAT_ACTIVEERRINT(x)              (((uint32_t)(((uint32_t)(x)) << DMA_INTSTAT_ACTIVEERRINT_SHIFT)) & DMA_INTSTAT_ACTIVEERRINT_MASK)

/*! @name SRAMBASE - SRAM address of the channel configuration table. */
#define DMA_SRAMBASE_OFFSET_MASK                 (0xFFFFFE00U)
#define DMA_SRAMBASE_OFFSET_SHIFT                (9U)
#define DMA_SRAMBASE_OFFSET(x)                   (((uint32_t)(((uint32_t)(x)) << DMA_SRAMBASE_OFFSET_SHIFT)) & DMA_SRAMBASE_OFFSET_MASK)

/*! @name COMMON_ENABLESET - Channel Enable read and Set for all DMA channels. */
#define DMA_COMMON_ENABLESET_ENA_MASK            (0xFFFFFFFFU)
#define DMA_COMMON_ENABLESET_ENA_SHIFT           (0U)
#define DMA_COMMON_ENABLESET_ENA(x)              (((uint32_t)(((uint32_t)(x)) << DMA_COMMON_ENABLESET_ENA_SHIFT)) & DMA_COMMON_ENABLESET_ENA_MASK)

/* The count of DMA_COMMON_ENABLESET */
#define DMA_COMMON_ENABLESET_COUNT               (1U)

/*! @name COMMON_ENABLECLR - Channel Enable Clear for all DMA channels. */
#define DMA_COMMON_ENABLECLR_CLR_MASK            (0xFFFFFFFFU)
#define DMA_COMMON_ENABLECLR_CLR_SHIFT           (0U)
#define DMA_COMMON_ENABLECLR_CLR(x)              (((uint32_t)(((uint32_t)(x)) << DMA_COMMON_ENABLECLR_CLR_SHIFT)) & DMA_COMMON_ENABLECLR_CLR_MASK)

/* The count of DMA_COMMON_ENABLECLR */
#define DMA_COMMON_ENABLECLR_COUNT               (1U)

/*! @name COMMON_ACTIVE - Channel Active status for all DMA channels. */
#define DMA_COMMON_ACTIVE_ACT_MASK               (0xFFFFFFFFU)
#define DMA_COMMON_ACTIVE_ACT_SHIFT              (0U)
#define DMA_COMMON_ACTIVE_ACT(x)                 (((uint32_t)(((uint32_t)(x)) << DMA_COMMON_ACTIVE_ACT_SHIFT)) & DMA_COMMON_ACTIVE_ACT_MASK)

/* The count of DMA_COMMON_ACTIVE */
#define DMA_COMMON_ACTIVE_COUNT                  (1U)

/*! @name COMMON_BUSY - Channel Busy status for all DMA channels. */
#define DMA_COMMON_BUSY_BSY_MASK                 (0xFFFFFFFFU)
#define DMA_COMMON_BUSY_BSY_SHIFT                (0U)
#define DMA_COMMON_BUSY_BSY(x)                   (((uint32_t)(((uint32_t)(x)) << DMA_COMMON_BUSY_BSY_SHIFT)) & DMA_COMMON_BUSY_BSY_MASK)

/* The count of DMA_COMMON_BUSY */
#define DMA_COMMON_BUSY_COUNT                    (1U)

/*! @name COMMON_ERRINT - Error Interrupt status for all DMA channels. */
#define DMA_COMMON_ERRINT_ERR_MASK               (0xFFFFFFFFU)
#define DMA_COMMON_ERRINT_ERR_SHIFT              (0U)
#define DMA_COMMON_ERRINT_ERR(x)                 (((uint32_t)(((uint32_t)(x)) << DMA_COMMON_ERRINT_ERR_SHIFT)) & DMA_COMMON_ERRINT_ERR_MASK)

/* The count of DMA_COMMON_ERRINT */
#define DMA_COMMON_ERRINT_COUNT                  (1U)

/*! @name COMMON_INTENSET - Interrupt Enable read and Set for all DMA channels. */
#define DMA_COMMON_INTENSET_INTEN_MASK           (0xFFFFFFFFU)
#define DMA_COMMON_INTENSET_INTEN_SHIFT          (0U)
#define DMA_COMMON_INTENSET_INTEN(x)             (((uint32_t)(((uint32_t)(x)) << DMA_COMMON_INTENSET_INTEN_SHIFT)) & DMA_COMMON_INTENSET_INTEN_MASK)

/* The count of DMA_COMMON_INTENSET */
#define DMA_COMMON_INTENSET_COUNT                (1U)

/*! @name COMMON_INTENCLR - Interrupt Enable Clear for all DMA channels. */
#define DMA_COMMON_INTENCLR_CLR_MASK             (0xFFFFFFFFU)
#define DMA_COMMON_INTENCLR_CLR_SHIFT            (0U)
#define DMA_COMMON_INTENCLR_CLR(x)               (((uint32_t)(((uint32_t)(x)) << DMA_COMMON_INTENCLR_CLR_SHIFT)) & DMA_COMMON_INTENCLR_CLR_MASK)

/* The count of DMA_COMMON_INTENCLR */
#define DMA_COMMON_INTENCLR_COUNT                (1U)

/*! @name COMMON_INTA - Interrupt A status for all DMA channels. */
#define DMA_COMMON_INTA_IA_MASK                  (0xFFFFFFFFU)
#define DMA_COMMON_INTA_IA_SHIFT                 (0U)
#define DMA_COMMON_INTA_IA(x)                    (((uint32_t)(((uint32_t)(x)) << DMA_COMMON_INTA_IA_SHIFT)) & DMA_COMMON_INTA_IA_MASK)

/* The count of DMA_COMMON_INTA */
#define DMA_COMMON_INTA_COUNT                    (1U)

/*! @name COMMON_INTB - Interrupt B status for all DMA channels. */
#define DMA_COMMON_INTB_IB_MASK                  (0xFFFFFFFFU)
#define DMA_COMMON_INTB_IB_SHIFT                 (0U)
#define DMA_COMMON_INTB_IB(x)                    (((uint32_t)(((uint32_t)(x)) << DMA_COMMON_INTB_IB_SHIFT)) & DMA_COMMON_INTB_IB_MASK)

/* The count of DMA_COMMON_INTB */
#define DMA_COMMON_INTB_COUNT                    (1U)

/*! @name COMMON_SETVALID - Set ValidPending control bits for all DMA channels. */
#define DMA_COMMON_SETVALID_SV_MASK              (0xFFFFFFFFU)
#define DMA_COMMON_SETVALID_SV_SHIFT             (0U)
#define DMA_COMMON_SETVALID_SV(x)                (((uint32_t)(((uint32_t)(x)) << DMA_COMMON_SETVALID_SV_SHIFT)) & DMA_COMMON_SETVALID_SV_MASK)

/* The count of DMA_COMMON_SETVALID */
#define DMA_COMMON_SETVALID_COUNT                (1U)

/*! @name COMMON_SETTRIG - Set Trigger control bits for all DMA channels. */
#define DMA_COMMON_SETTRIG_TRIG_MASK             (0xFFFFFFFFU)
#define DMA_COMMON_SETTRIG_TRIG_SHIFT            (0U)
#define DMA_COMMON_SETTRIG_TRIG(x)               (((uint32_t)(((uint32_t)(x)) << DMA_COMMON_SETTRIG_TRIG_SHIFT)) & DMA_COMMON_SETTRIG_TRIG_MASK)

/* The count of DMA_COMMON_SETTRIG */
#define DMA_COMMON_SETTRIG_COUNT                 (1U)

/*! @name COMMON_ABORT - Channel Abort control for all DMA channels. */
#define DMA_COMMON_ABORT_ABORTCTRL_MASK          (0xFFFFFFFFU)
#define DMA_COMMON_ABORT_ABORTCTRL_SHIFT         (0U)
#define DMA_COMMON_ABORT_ABORTCTRL(x)            (((uint32_t)(((uint32_t)(x)) << DMA_COMMON_ABORT_ABORTCTRL_SHIFT)) & DMA_COMMON_ABORT_ABORTCTRL_MASK)

/* The count of DMA_COMMON_ABORT */
#define DMA_COMMON_ABORT_COUNT                   (1U)

/*! @name CHANNEL_CFG - Configuration register for DMA channel . */
#define DMA_CHANNEL_CFG_PERIPHREQEN_MASK         (0x1U)
#define DMA_CHANNEL_CFG_PERIPHREQEN_SHIFT        (0U)
#define DMA_CHANNEL_CFG_PERIPHREQEN(x)           (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_CFG_PERIPHREQEN_SHIFT)) & DMA_CHANNEL_CFG_PERIPHREQEN_MASK)
#define DMA_CHANNEL_CFG_HWTRIGEN_MASK            (0x2U)
#define DMA_CHANNEL_CFG_HWTRIGEN_SHIFT           (1U)
#define DMA_CHANNEL_CFG_HWTRIGEN(x)              (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_CFG_HWTRIGEN_SHIFT)) & DMA_CHANNEL_CFG_HWTRIGEN_MASK)
#define DMA_CHANNEL_CFG_TRIGPOL_MASK             (0x10U)
#define DMA_CHANNEL_CFG_TRIGPOL_SHIFT            (4U)
#define DMA_CHANNEL_CFG_TRIGPOL(x)               (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_CFG_TRIGPOL_SHIFT)) & DMA_CHANNEL_CFG_TRIGPOL_MASK)
#define DMA_CHANNEL_CFG_TRIGTYPE_MASK            (0x20U)
#define DMA_CHANNEL_CFG_TRIGTYPE_SHIFT           (5U)
#define DMA_CHANNEL_CFG_TRIGTYPE(x)              (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_CFG_TRIGTYPE_SHIFT)) & DMA_CHANNEL_CFG_TRIGTYPE_MASK)
#define DMA_CHANNEL_CFG_TRIGBURST_MASK           (0x40U)
#define DMA_CHANNEL_CFG_TRIGBURST_SHIFT          (6U)
#define DMA_CHANNEL_CFG_TRIGBURST(x)             (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_CFG_TRIGBURST_SHIFT)) & DMA_CHANNEL_CFG_TRIGBURST_MASK)
#define DMA_CHANNEL_CFG_BURSTPOWER_MASK          (0xF00U)
#define DMA_CHANNEL_CFG_BURSTPOWER_SHIFT         (8U)
#define DMA_CHANNEL_CFG_BURSTPOWER(x)            (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_CFG_BURSTPOWER_SHIFT)) & DMA_CHANNEL_CFG_BURSTPOWER_MASK)
#define DMA_CHANNEL_CFG_SRCBURSTWRAP_MASK        (0x4000U)
#define DMA_CHANNEL_CFG_SRCBURSTWRAP_SHIFT       (14U)
#define DMA_CHANNEL_CFG_SRCBURSTWRAP(x)          (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_CFG_SRCBURSTWRAP_SHIFT)) & DMA_CHANNEL_CFG_SRCBURSTWRAP_MASK)
#define DMA_CHANNEL_CFG_DSTBURSTWRAP_MASK        (0x8000U)
#define DMA_CHANNEL_CFG_DSTBURSTWRAP_SHIFT       (15U)
#define DMA_CHANNEL_CFG_DSTBURSTWRAP(x)          (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_CFG_DSTBURSTWRAP_SHIFT)) & DMA_CHANNEL_CFG_DSTBURSTWRAP_MASK)
#define DMA_CHANNEL_CFG_CHPRIORITY_MASK          (0x70000U)
#define DMA_CHANNEL_CFG_CHPRIORITY_SHIFT         (16U)
#define DMA_CHANNEL_CFG_CHPRIORITY(x)            (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_CFG_CHPRIORITY_SHIFT)) & DMA_CHANNEL_CFG_CHPRIORITY_MASK)

/* The count of DMA_CHANNEL_CFG */
#define DMA_CHANNEL_CFG_COUNT                    (20U)

/*! @name CHANNEL_CTLSTAT - Control and status register for DMA channel . */
#define DMA_CHANNEL_CTLSTAT_VALIDPENDING_MASK    (0x1U)
#define DMA_CHANNEL_CTLSTAT_VALIDPENDING_SHIFT   (0U)
#define DMA_CHANNEL_CTLSTAT_VALIDPENDING(x)      (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_CTLSTAT_VALIDPENDING_SHIFT)) & DMA_CHANNEL_CTLSTAT_VALIDPENDING_MASK)
#define DMA_CHANNEL_CTLSTAT_TRIG_MASK            (0x4U)
#define DMA_CHANNEL_CTLSTAT_TRIG_SHIFT           (2U)
#define DMA_CHANNEL_CTLSTAT_TRIG(x)              (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_CTLSTAT_TRIG_SHIFT)) & DMA_CHANNEL_CTLSTAT_TRIG_MASK)

/* The count of DMA_CHANNEL_CTLSTAT */
#define DMA_CHANNEL_CTLSTAT_COUNT                (20U)

/*! @name CHANNEL_XFERCFG - Transfer configuration register for DMA channel . */
#define DMA_CHANNEL_XFERCFG_CFGVALID_MASK        (0x1U)
#define DMA_CHANNEL_XFERCFG_CFGVALID_SHIFT       (0U)
#define DMA_CHANNEL_XFERCFG_CFGVALID(x)          (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_XFERCFG_CFGVALID_SHIFT)) & DMA_CHANNEL_XFERCFG_CFGVALID_MASK)
#define DMA_CHANNEL_XFERCFG_RELOAD_MASK          (0x2U)
#define DMA_CHANNEL_XFERCFG_RELOAD_SHIFT         (1U)
#define DMA_CHANNEL_XFERCFG_RELOAD(x)            (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_XFERCFG_RELOAD_SHIFT)) & DMA_CHANNEL_XFERCFG_RELOAD_MASK)
#define DMA_CHANNEL_XFERCFG_SWTRIG_MASK          (0x4U)
#define DMA_CHANNEL_XFERCFG_SWTRIG_SHIFT         (2U)
#define DMA_CHANNEL_XFERCFG_SWTRIG(x)            (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_XFERCFG_SWTRIG_SHIFT)) & DMA_CHANNEL_XFERCFG_SWTRIG_MASK)
#define DMA_CHANNEL_XFERCFG_CLRTRIG_MASK         (0x8U)
#define DMA_CHANNEL_XFERCFG_CLRTRIG_SHIFT        (3U)
#define DMA_CHANNEL_XFERCFG_CLRTRIG(x)           (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_XFERCFG_CLRTRIG_SHIFT)) & DMA_CHANNEL_XFERCFG_CLRTRIG_MASK)
#define DMA_CHANNEL_XFERCFG_SETINTA_MASK         (0x10U)
#define DMA_CHANNEL_XFERCFG_SETINTA_SHIFT        (4U)
#define DMA_CHANNEL_XFERCFG_SETINTA(x)           (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_XFERCFG_SETINTA_SHIFT)) & DMA_CHANNEL_XFERCFG_SETINTA_MASK)
#define DMA_CHANNEL_XFERCFG_SETINTB_MASK         (0x20U)
#define DMA_CHANNEL_XFERCFG_SETINTB_SHIFT        (5U)
#define DMA_CHANNEL_XFERCFG_SETINTB(x)           (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_XFERCFG_SETINTB_SHIFT)) & DMA_CHANNEL_XFERCFG_SETINTB_MASK)
#define DMA_CHANNEL_XFERCFG_WIDTH_MASK           (0x300U)
#define DMA_CHANNEL_XFERCFG_WIDTH_SHIFT          (8U)
#define DMA_CHANNEL_XFERCFG_WIDTH(x)             (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_XFERCFG_WIDTH_SHIFT)) & DMA_CHANNEL_XFERCFG_WIDTH_MASK)
#define DMA_CHANNEL_XFERCFG_SRCINC_MASK          (0x3000U)
#define DMA_CHANNEL_XFERCFG_SRCINC_SHIFT         (12U)
#define DMA_CHANNEL_XFERCFG_SRCINC(x)            (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_XFERCFG_SRCINC_SHIFT)) & DMA_CHANNEL_XFERCFG_SRCINC_MASK)
#define DMA_CHANNEL_XFERCFG_DSTINC_MASK          (0xC000U)
#define DMA_CHANNEL_XFERCFG_DSTINC_SHIFT         (14U)
#define DMA_CHANNEL_XFERCFG_DSTINC(x)            (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_XFERCFG_DSTINC_SHIFT)) & DMA_CHANNEL_XFERCFG_DSTINC_MASK)
#define DMA_CHANNEL_XFERCFG_XFERCOUNT_MASK       (0x3FF0000U)
#define DMA_CHANNEL_XFERCFG_XFERCOUNT_SHIFT      (16U)
#define DMA_CHANNEL_XFERCFG_XFERCOUNT(x)         (((uint32_t)(((uint32_t)(x)) << DMA_CHANNEL_XFERCFG_XFERCOUNT_SHIFT)) & DMA_CHANNEL_XFERCFG_XFERCOUNT_MASK)

/* The count of DMA_CHANNEL_XFERCFG */
#define DMA_CHANNEL_XFERCFG_COUNT                (20U)


/*!
 * @}
 */ /* end of group DMA_Register_Masks */


/* DMA - Peripheral instance base addresses */
/** Peripheral DMA0 base address */
#define DMA0_BASE                                (0x40082000u)
/** Peripheral DMA0 base pointer */
#define DMA0                                     ((DMA_Type *)DMA0_BASE)
/** Array initializer of DMA peripheral base addresses */
#define DMA_BASE_ADDRS                           { DMA0_BASE }
/** Array initializer of DMA peripheral base pointers */
#define DMA_BASE_PTRS                            { DMA0 }
/** Interrupt vectors for the DMA peripheral type */
#define DMA_IRQS                                 { DMA0_IRQn }

/*!
 * @}
 */ /* end of group DMA_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FLEXCOMM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FLEXCOMM_Peripheral_Access_Layer FLEXCOMM Peripheral Access Layer
 * @{
 */

/** FLEXCOMM - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[4088];
  __IO uint32_t PSELID;                            /**< Peripheral Select and Flexcomm ID register., offset: 0xFF8 */
  __I  uint32_t PID;                               /**< Peripheral identification register., offset: 0xFFC */
} FLEXCOMM_Type;

/* ----------------------------------------------------------------------------
   -- FLEXCOMM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FLEXCOMM_Register_Masks FLEXCOMM Register Masks
 * @{
 */

/*! @name PSELID - Peripheral Select and Flexcomm ID register. */
#define FLEXCOMM_PSELID_PERSEL_MASK              (0x7U)
#define FLEXCOMM_PSELID_PERSEL_SHIFT             (0U)
#define FLEXCOMM_PSELID_PERSEL(x)                (((uint32_t)(((uint32_t)(x)) << FLEXCOMM_PSELID_PERSEL_SHIFT)) & FLEXCOMM_PSELID_PERSEL_MASK)
#define FLEXCOMM_PSELID_LOCK_MASK                (0x8U)
#define FLEXCOMM_PSELID_LOCK_SHIFT               (3U)
#define FLEXCOMM_PSELID_LOCK(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXCOMM_PSELID_LOCK_SHIFT)) & FLEXCOMM_PSELID_LOCK_MASK)
#define FLEXCOMM_PSELID_USARTPRESENT_MASK        (0x10U)
#define FLEXCOMM_PSELID_USARTPRESENT_SHIFT       (4U)
#define FLEXCOMM_PSELID_USARTPRESENT(x)          (((uint32_t)(((uint32_t)(x)) << FLEXCOMM_PSELID_USARTPRESENT_SHIFT)) & FLEXCOMM_PSELID_USARTPRESENT_MASK)
#define FLEXCOMM_PSELID_SPIPRESENT_MASK          (0x20U)
#define FLEXCOMM_PSELID_SPIPRESENT_SHIFT         (5U)
#define FLEXCOMM_PSELID_SPIPRESENT(x)            (((uint32_t)(((uint32_t)(x)) << FLEXCOMM_PSELID_SPIPRESENT_SHIFT)) & FLEXCOMM_PSELID_SPIPRESENT_MASK)
#define FLEXCOMM_PSELID_I2CPRESENT_MASK          (0x40U)
#define FLEXCOMM_PSELID_I2CPRESENT_SHIFT         (6U)
#define FLEXCOMM_PSELID_I2CPRESENT(x)            (((uint32_t)(((uint32_t)(x)) << FLEXCOMM_PSELID_I2CPRESENT_SHIFT)) & FLEXCOMM_PSELID_I2CPRESENT_MASK)
#define FLEXCOMM_PSELID_I2SPRESENT_MASK          (0x80U)
#define FLEXCOMM_PSELID_I2SPRESENT_SHIFT         (7U)
#define FLEXCOMM_PSELID_I2SPRESENT(x)            (((uint32_t)(((uint32_t)(x)) << FLEXCOMM_PSELID_I2SPRESENT_SHIFT)) & FLEXCOMM_PSELID_I2SPRESENT_MASK)
#define FLEXCOMM_PSELID_ID_MASK                  (0xFFFFF000U)
#define FLEXCOMM_PSELID_ID_SHIFT                 (12U)
#define FLEXCOMM_PSELID_ID(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXCOMM_PSELID_ID_SHIFT)) & FLEXCOMM_PSELID_ID_MASK)

/*! @name PID - Peripheral identification register. */
#define FLEXCOMM_PID_Minor_Rev_MASK              (0xF00U)
#define FLEXCOMM_PID_Minor_Rev_SHIFT             (8U)
#define FLEXCOMM_PID_Minor_Rev(x)                (((uint32_t)(((uint32_t)(x)) << FLEXCOMM_PID_Minor_Rev_SHIFT)) & FLEXCOMM_PID_Minor_Rev_MASK)
#define FLEXCOMM_PID_Major_Rev_MASK              (0xF000U)
#define FLEXCOMM_PID_Major_Rev_SHIFT             (12U)
#define FLEXCOMM_PID_Major_Rev(x)                (((uint32_t)(((uint32_t)(x)) << FLEXCOMM_PID_Major_Rev_SHIFT)) & FLEXCOMM_PID_Major_Rev_MASK)
#define FLEXCOMM_PID_ID_MASK                     (0xFFFF0000U)
#define FLEXCOMM_PID_ID_SHIFT                    (16U)
#define FLEXCOMM_PID_ID(x)                       (((uint32_t)(((uint32_t)(x)) << FLEXCOMM_PID_ID_SHIFT)) & FLEXCOMM_PID_ID_MASK)


/*!
 * @}
 */ /* end of group FLEXCOMM_Register_Masks */


/* FLEXCOMM - Peripheral instance base addresses */
/** Peripheral FLEXCOMM0 base address */
#define FLEXCOMM0_BASE                           (0x40086000u)
/** Peripheral FLEXCOMM0 base pointer */
#define FLEXCOMM0                                ((FLEXCOMM_Type *)FLEXCOMM0_BASE)
/** Peripheral FLEXCOMM1 base address */
#define FLEXCOMM1_BASE                           (0x40087000u)
/** Peripheral FLEXCOMM1 base pointer */
#define FLEXCOMM1                                ((FLEXCOMM_Type *)FLEXCOMM1_BASE)
/** Peripheral FLEXCOMM2 base address */
#define FLEXCOMM2_BASE                           (0x40088000u)
/** Peripheral FLEXCOMM2 base pointer */
#define FLEXCOMM2                                ((FLEXCOMM_Type *)FLEXCOMM2_BASE)
/** Peripheral FLEXCOMM3 base address */
#define FLEXCOMM3_BASE                           (0x40089000u)
/** Peripheral FLEXCOMM3 base pointer */
#define FLEXCOMM3                                ((FLEXCOMM_Type *)FLEXCOMM3_BASE)
/** Peripheral FLEXCOMM4 base address */
#define FLEXCOMM4_BASE                           (0x4008A000u)
/** Peripheral FLEXCOMM4 base pointer */
#define FLEXCOMM4                                ((FLEXCOMM_Type *)FLEXCOMM4_BASE)
/** Peripheral FLEXCOMM5 base address */
#define FLEXCOMM5_BASE                           (0x40096000u)
/** Peripheral FLEXCOMM5 base pointer */
#define FLEXCOMM5                                ((FLEXCOMM_Type *)FLEXCOMM5_BASE)
/** Peripheral FLEXCOMM6 base address */
#define FLEXCOMM6_BASE                           (0x40097000u)
/** Peripheral FLEXCOMM6 base pointer */
#define FLEXCOMM6                                ((FLEXCOMM_Type *)FLEXCOMM6_BASE)
/** Peripheral FLEXCOMM7 base address */
#define FLEXCOMM7_BASE                           (0x40098000u)
/** Peripheral FLEXCOMM7 base pointer */
#define FLEXCOMM7                                ((FLEXCOMM_Type *)FLEXCOMM7_BASE)
/** Array initializer of FLEXCOMM peripheral base addresses */
#define FLEXCOMM_BASE_ADDRS                      { FLEXCOMM0_BASE, FLEXCOMM1_BASE, FLEXCOMM2_BASE, FLEXCOMM3_BASE, FLEXCOMM4_BASE, FLEXCOMM5_BASE, FLEXCOMM6_BASE, FLEXCOMM7_BASE }
/** Array initializer of FLEXCOMM peripheral base pointers */
#define FLEXCOMM_BASE_PTRS                       { FLEXCOMM0, FLEXCOMM1, FLEXCOMM2, FLEXCOMM3, FLEXCOMM4, FLEXCOMM5, FLEXCOMM6, FLEXCOMM7 }
/** Interrupt vectors for the FLEXCOMM peripheral type */
#define FLEXCOMM_IRQS                            { FLEXCOMM0_IRQn, FLEXCOMM1_IRQn, FLEXCOMM2_IRQn, FLEXCOMM3_IRQn, FLEXCOMM4_IRQn, FLEXCOMM5_IRQn, FLEXCOMM6_IRQn, FLEXCOMM7_IRQn }

/*!
 * @}
 */ /* end of group FLEXCOMM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Peripheral_Access_Layer FMC Peripheral Access Layer
 * @{
 */

/** FMC - Register Layout Typedef */
typedef struct {
  __IO uint32_t FCTR;                              /**< Control register, offset: 0x0 */
       uint8_t RESERVED_0[12];
  __IO uint32_t FBWST;                             /**< Wait state register, offset: 0x10 */
       uint8_t RESERVED_1[12];
  __IO uint32_t FMSSTART;                          /**< Signature start address register, offset: 0x20 */
  __IO uint32_t FMSSTOP;                           /**< Signature stop-address register, offset: 0x24 */
       uint8_t RESERVED_2[4];
  __I  uint32_t FMSW[4];                           /**< Words of 128-bit signature word, array offset: 0x2C, array step: 0x4 */
       uint8_t RESERVED_3[4004];
  __I  uint32_t FMSTAT;                            /**< Signature generation status register, offset: 0xFE0 */
       uint8_t RESERVED_4[4];
  __O  uint32_t FMSTATCLR;                         /**< Signature generation status clear register, offset: 0xFE8 */
} FMC_Type;

/* ----------------------------------------------------------------------------
   -- FMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Register_Masks FMC Register Masks
 * @{
 */

/*! @name FCTR - Control register */
#define FMC_FCTR_FS_RD0_MASK                     (0x8U)
#define FMC_FCTR_FS_RD0_SHIFT                    (3U)
#define FMC_FCTR_FS_RD0(x)                       (((uint32_t)(((uint32_t)(x)) << FMC_FCTR_FS_RD0_SHIFT)) & FMC_FCTR_FS_RD0_MASK)
#define FMC_FCTR_FS_RD1_MASK                     (0x10U)
#define FMC_FCTR_FS_RD1_SHIFT                    (4U)
#define FMC_FCTR_FS_RD1(x)                       (((uint32_t)(((uint32_t)(x)) << FMC_FCTR_FS_RD1_SHIFT)) & FMC_FCTR_FS_RD1_MASK)

/*! @name FBWST - Wait state register */
#define FMC_FBWST_WAITSTATES_MASK                (0xFFU)
#define FMC_FBWST_WAITSTATES_SHIFT               (0U)
#define FMC_FBWST_WAITSTATES(x)                  (((uint32_t)(((uint32_t)(x)) << FMC_FBWST_WAITSTATES_SHIFT)) & FMC_FBWST_WAITSTATES_MASK)

/*! @name FMSSTART - Signature start address register */
#define FMC_FMSSTART_START_MASK                  (0x1FFFFU)
#define FMC_FMSSTART_START_SHIFT                 (0U)
#define FMC_FMSSTART_START(x)                    (((uint32_t)(((uint32_t)(x)) << FMC_FMSSTART_START_SHIFT)) & FMC_FMSSTART_START_MASK)

/*! @name FMSSTOP - Signature stop-address register */
#define FMC_FMSSTOP_STOP_MASK                    (0x1FFFFU)
#define FMC_FMSSTOP_STOP_SHIFT                   (0U)
#define FMC_FMSSTOP_STOP(x)                      (((uint32_t)(((uint32_t)(x)) << FMC_FMSSTOP_STOP_SHIFT)) & FMC_FMSSTOP_STOP_MASK)
#define FMC_FMSSTOP_SIG_START_MASK               (0x20000U)
#define FMC_FMSSTOP_SIG_START_SHIFT              (17U)
#define FMC_FMSSTOP_SIG_START(x)                 (((uint32_t)(((uint32_t)(x)) << FMC_FMSSTOP_SIG_START_SHIFT)) & FMC_FMSSTOP_SIG_START_MASK)

/*! @name FMSW - Words of 128-bit signature word */
#define FMC_FMSW_SW_MASK                         (0xFFFFFFFFU)
#define FMC_FMSW_SW_SHIFT                        (0U)
#define FMC_FMSW_SW(x)                           (((uint32_t)(((uint32_t)(x)) << FMC_FMSW_SW_SHIFT)) & FMC_FMSW_SW_MASK)

/* The count of FMC_FMSW */
#define FMC_FMSW_COUNT                           (4U)

/*! @name FMSTAT - Signature generation status register */
#define FMC_FMSTAT_SIG_DONE_MASK                 (0x4U)
#define FMC_FMSTAT_SIG_DONE_SHIFT                (2U)
#define FMC_FMSTAT_SIG_DONE(x)                   (((uint32_t)(((uint32_t)(x)) << FMC_FMSTAT_SIG_DONE_SHIFT)) & FMC_FMSTAT_SIG_DONE_MASK)

/*! @name FMSTATCLR - Signature generation status clear register */
#define FMC_FMSTATCLR_SIG_DONE_CLR_MASK          (0x4U)
#define FMC_FMSTATCLR_SIG_DONE_CLR_SHIFT         (2U)
#define FMC_FMSTATCLR_SIG_DONE_CLR(x)            (((uint32_t)(((uint32_t)(x)) << FMC_FMSTATCLR_SIG_DONE_CLR_SHIFT)) & FMC_FMSTATCLR_SIG_DONE_CLR_MASK)


/*!
 * @}
 */ /* end of group FMC_Register_Masks */


/* FMC - Peripheral instance base addresses */
/** Peripheral FMC base address */
#define FMC_BASE                                 (0x40034000u)
/** Peripheral FMC base pointer */
#define FMC                                      ((FMC_Type *)FMC_BASE)
/** Array initializer of FMC peripheral base addresses */
#define FMC_BASE_ADDRS                           { FMC_BASE }
/** Array initializer of FMC peripheral base pointers */
#define FMC_BASE_PTRS                            { FMC }

/*!
 * @}
 */ /* end of group FMC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- GINT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GINT_Peripheral_Access_Layer GINT Peripheral Access Layer
 * @{
 */

/** GINT - Register Layout Typedef */
typedef struct {
  __IO uint32_t CTRL;                              /**< GPIO grouped interrupt control register, offset: 0x0 */
       uint8_t RESERVED_0[28];
  __IO uint32_t PORT_POL[2];                       /**< GPIO grouped interrupt port 0 polarity register, array offset: 0x20, array step: 0x4 */
       uint8_t RESERVED_1[24];
  __IO uint32_t PORT_ENA[2];                       /**< GPIO grouped interrupt port 0 enable register, array offset: 0x40, array step: 0x4 */
} GINT_Type;

/* ----------------------------------------------------------------------------
   -- GINT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GINT_Register_Masks GINT Register Masks
 * @{
 */

/*! @name CTRL - GPIO grouped interrupt control register */
#define GINT_CTRL_INT_MASK                       (0x1U)
#define GINT_CTRL_INT_SHIFT                      (0U)
#define GINT_CTRL_INT(x)                         (((uint32_t)(((uint32_t)(x)) << GINT_CTRL_INT_SHIFT)) & GINT_CTRL_INT_MASK)
#define GINT_CTRL_COMB_MASK                      (0x2U)
#define GINT_CTRL_COMB_SHIFT                     (1U)
#define GINT_CTRL_COMB(x)                        (((uint32_t)(((uint32_t)(x)) << GINT_CTRL_COMB_SHIFT)) & GINT_CTRL_COMB_MASK)
#define GINT_CTRL_TRIG_MASK                      (0x4U)
#define GINT_CTRL_TRIG_SHIFT                     (2U)
#define GINT_CTRL_TRIG(x)                        (((uint32_t)(((uint32_t)(x)) << GINT_CTRL_TRIG_SHIFT)) & GINT_CTRL_TRIG_MASK)

/*! @name PORT_POL - GPIO grouped interrupt port 0 polarity register */
#define GINT_PORT_POL_POL_MASK                   (0xFFFFFFFFU)
#define GINT_PORT_POL_POL_SHIFT                  (0U)
#define GINT_PORT_POL_POL(x)                     (((uint32_t)(((uint32_t)(x)) << GINT_PORT_POL_POL_SHIFT)) & GINT_PORT_POL_POL_MASK)

/* The count of GINT_PORT_POL */
#define GINT_PORT_POL_COUNT                      (2U)

/*! @name PORT_ENA - GPIO grouped interrupt port 0 enable register */
#define GINT_PORT_ENA_ENA_MASK                   (0xFFFFFFFFU)
#define GINT_PORT_ENA_ENA_SHIFT                  (0U)
#define GINT_PORT_ENA_ENA(x)                     (((uint32_t)(((uint32_t)(x)) << GINT_PORT_ENA_ENA_SHIFT)) & GINT_PORT_ENA_ENA_MASK)

/* The count of GINT_PORT_ENA */
#define GINT_PORT_ENA_COUNT                      (2U)


/*!
 * @}
 */ /* end of group GINT_Register_Masks */


/* GINT - Peripheral instance base addresses */
/** Peripheral GINT0 base address */
#define GINT0_BASE                               (0x40002000u)
/** Peripheral GINT0 base pointer */
#define GINT0                                    ((GINT_Type *)GINT0_BASE)
/** Peripheral GINT1 base address */
#define GINT1_BASE                               (0x40003000u)
/** Peripheral GINT1 base pointer */
#define GINT1                                    ((GINT_Type *)GINT1_BASE)
/** Array initializer of GINT peripheral base addresses */
#define GINT_BASE_ADDRS                          { GINT0_BASE, GINT1_BASE }
/** Array initializer of GINT peripheral base pointers */
#define GINT_BASE_PTRS                           { GINT0, GINT1 }
/** Interrupt vectors for the GINT peripheral type */
#define GINT_IRQS                                { GINT0_IRQn, GINT1_IRQn }

/*!
 * @}
 */ /* end of group GINT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- GPIO Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Peripheral_Access_Layer GPIO Peripheral Access Layer
 * @{
 */

/** GPIO - Register Layout Typedef */
typedef struct {
  __IO uint8_t B[2][32];                           /**< Byte pin registers for all port 0 and 1 GPIO pins, array offset: 0x0, array step: index*0x20, index2*0x1 */
       uint8_t RESERVED_0[4032];
  __IO uint32_t W[2][32];                          /**< Word pin registers for all port 0 and 1 GPIO pins, array offset: 0x1000, array step: index*0x80, index2*0x4 */
       uint8_t RESERVED_1[3840];
  __IO uint32_t DIR[2];                            /**< Direction registers, array offset: 0x2000, array step: 0x4 */
       uint8_t RESERVED_2[120];
  __IO uint32_t MASK[2];                           /**< Mask register, array offset: 0x2080, array step: 0x4 */
       uint8_t RESERVED_3[120];
  __IO uint32_t PIN[2];                            /**< Port pin register, array offset: 0x2100, array step: 0x4 */
       uint8_t RESERVED_4[120];
  __IO uint32_t MPIN[2];                           /**< Masked port register, array offset: 0x2180, array step: 0x4 */
       uint8_t RESERVED_5[120];
  __IO uint32_t SET[2];                            /**< Write: Set register for port Read: output bits for port, array offset: 0x2200, array step: 0x4 */
       uint8_t RESERVED_6[120];
  __O  uint32_t CLR[2];                            /**< Clear port, array offset: 0x2280, array step: 0x4 */
       uint8_t RESERVED_7[120];
  __O  uint32_t NOT[2];                            /**< Toggle port, array offset: 0x2300, array step: 0x4 */
       uint8_t RESERVED_8[120];
  __O  uint32_t DIRSET[2];                         /**< Set pin direction bits for port, array offset: 0x2380, array step: 0x4 */
       uint8_t RESERVED_9[120];
  __O  uint32_t DIRCLR[2];                         /**< Clear pin direction bits for port, array offset: 0x2400, array step: 0x4 */
       uint8_t RESERVED_10[120];
  __O  uint32_t DIRNOT[2];                         /**< Toggle pin direction bits for port, array offset: 0x2480, array step: 0x4 */
} GPIO_Type;

/* ----------------------------------------------------------------------------
   -- GPIO Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Masks GPIO Register Masks
 * @{
 */

/*! @name B - Byte pin registers for all port 0 and 1 GPIO pins */
#define GPIO_B_PBYTE_MASK                        (0x1U)
#define GPIO_B_PBYTE_SHIFT                       (0U)
#define GPIO_B_PBYTE(x)                          (((uint8_t)(((uint8_t)(x)) << GPIO_B_PBYTE_SHIFT)) & GPIO_B_PBYTE_MASK)

/* The count of GPIO_B */
#define GPIO_B_COUNT                             (2U)

/* The count of GPIO_B */
#define GPIO_B_COUNT2                            (32U)

/*! @name W - Word pin registers for all port 0 and 1 GPIO pins */
#define GPIO_W_PWORD_MASK                        (0xFFFFFFFFU)
#define GPIO_W_PWORD_SHIFT                       (0U)
#define GPIO_W_PWORD(x)                          (((uint32_t)(((uint32_t)(x)) << GPIO_W_PWORD_SHIFT)) & GPIO_W_PWORD_MASK)

/* The count of GPIO_W */
#define GPIO_W_COUNT                             (2U)

/* The count of GPIO_W */
#define GPIO_W_COUNT2                            (32U)

/*! @name DIR - Direction registers */
#define GPIO_DIR_DIRP_MASK                       (0xFFFFFFFFU)
#define GPIO_DIR_DIRP_SHIFT                      (0U)
#define GPIO_DIR_DIRP(x)                         (((uint32_t)(((uint32_t)(x)) << GPIO_DIR_DIRP_SHIFT)) & GPIO_DIR_DIRP_MASK)

/* The count of GPIO_DIR */
#define GPIO_DIR_COUNT                           (2U)

/*! @name MASK - Mask register */
#define GPIO_MASK_MASKP_MASK                     (0xFFFFFFFFU)
#define GPIO_MASK_MASKP_SHIFT                    (0U)
#define GPIO_MASK_MASKP(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_MASK_MASKP_SHIFT)) & GPIO_MASK_MASKP_MASK)

/* The count of GPIO_MASK */
#define GPIO_MASK_COUNT                          (2U)

/*! @name PIN - Port pin register */
#define GPIO_PIN_PORT_MASK                       (0xFFFFFFFFU)
#define GPIO_PIN_PORT_SHIFT                      (0U)
#define GPIO_PIN_PORT(x)                         (((uint32_t)(((uint32_t)(x)) << GPIO_PIN_PORT_SHIFT)) & GPIO_PIN_PORT_MASK)

/* The count of GPIO_PIN */
#define GPIO_PIN_COUNT                           (2U)

/*! @name MPIN - Masked port register */
#define GPIO_MPIN_MPORTP_MASK                    (0xFFFFFFFFU)
#define GPIO_MPIN_MPORTP_SHIFT                   (0U)
#define GPIO_MPIN_MPORTP(x)                      (((uint32_t)(((uint32_t)(x)) << GPIO_MPIN_MPORTP_SHIFT)) & GPIO_MPIN_MPORTP_MASK)

/* The count of GPIO_MPIN */
#define GPIO_MPIN_COUNT                          (2U)

/*! @name SET - Write: Set register for port Read: output bits for port */
#define GPIO_SET_SETP_MASK                       (0xFFFFFFFFU)
#define GPIO_SET_SETP_SHIFT                      (0U)
#define GPIO_SET_SETP(x)                         (((uint32_t)(((uint32_t)(x)) << GPIO_SET_SETP_SHIFT)) & GPIO_SET_SETP_MASK)

/* The count of GPIO_SET */
#define GPIO_SET_COUNT                           (2U)

/*! @name CLR - Clear port */
#define GPIO_CLR_CLRP_MASK                       (0xFFFFFFFFU)
#define GPIO_CLR_CLRP_SHIFT                      (0U)
#define GPIO_CLR_CLRP(x)                         (((uint32_t)(((uint32_t)(x)) << GPIO_CLR_CLRP_SHIFT)) & GPIO_CLR_CLRP_MASK)

/* The count of GPIO_CLR */
#define GPIO_CLR_COUNT                           (2U)

/*! @name NOT - Toggle port */
#define GPIO_NOT_NOTP_MASK                       (0xFFFFFFFFU)
#define GPIO_NOT_NOTP_SHIFT                      (0U)
#define GPIO_NOT_NOTP(x)                         (((uint32_t)(((uint32_t)(x)) << GPIO_NOT_NOTP_SHIFT)) & GPIO_NOT_NOTP_MASK)

/* The count of GPIO_NOT */
#define GPIO_NOT_COUNT                           (2U)

/*! @name DIRSET - Set pin direction bits for port */
#define GPIO_DIRSET_DIRSETP_MASK                 (0xFFFFFFFFU)
#define GPIO_DIRSET_DIRSETP_SHIFT                (0U)
#define GPIO_DIRSET_DIRSETP(x)                   (((uint32_t)(((uint32_t)(x)) << GPIO_DIRSET_DIRSETP_SHIFT)) & GPIO_DIRSET_DIRSETP_MASK)

/* The count of GPIO_DIRSET */
#define GPIO_DIRSET_COUNT                        (2U)

/*! @name DIRCLR - Clear pin direction bits for port */
#define GPIO_DIRCLR_DIRCLRP_MASK                 (0xFFFFFFFFU)
#define GPIO_DIRCLR_DIRCLRP_SHIFT                (0U)
#define GPIO_DIRCLR_DIRCLRP(x)                   (((uint32_t)(((uint32_t)(x)) << GPIO_DIRCLR_DIRCLRP_SHIFT)) & GPIO_DIRCLR_DIRCLRP_MASK)

/* The count of GPIO_DIRCLR */
#define GPIO_DIRCLR_COUNT                        (2U)

/*! @name DIRNOT - Toggle pin direction bits for port */
#define GPIO_DIRNOT_DIRNOTP_MASK                 (0xFFFFFFFFU)
#define GPIO_DIRNOT_DIRNOTP_SHIFT                (0U)
#define GPIO_DIRNOT_DIRNOTP(x)                   (((uint32_t)(((uint32_t)(x)) << GPIO_DIRNOT_DIRNOTP_SHIFT)) & GPIO_DIRNOT_DIRNOTP_MASK)

/* The count of GPIO_DIRNOT */
#define GPIO_DIRNOT_COUNT                        (2U)


/*!
 * @}
 */ /* end of group GPIO_Register_Masks */


/* GPIO - Peripheral instance base addresses */
/** Peripheral GPIO base address */
#define GPIO_BASE                                (0x4008C000u)
/** Peripheral GPIO base pointer */
#define GPIO                                     ((GPIO_Type *)GPIO_BASE)
/** Array initializer of GPIO peripheral base addresses */
#define GPIO_BASE_ADDRS                          { GPIO_BASE }
/** Array initializer of GPIO peripheral base pointers */
#define GPIO_BASE_PTRS                           { GPIO }

/*!
 * @}
 */ /* end of group GPIO_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- I2C Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Peripheral_Access_Layer I2C Peripheral Access Layer
 * @{
 */

/** I2C - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[2048];
  __IO uint32_t CFG;                               /**< Configuration for shared functions., offset: 0x800 */
  __IO uint32_t STAT;                              /**< Status register for Master, Slave, and Monitor functions., offset: 0x804 */
  __IO uint32_t INTENSET;                          /**< Interrupt Enable Set and read register., offset: 0x808 */
  __O  uint32_t INTENCLR;                          /**< Interrupt Enable Clear register., offset: 0x80C */
  __IO uint32_t TIMEOUT;                           /**< Time-out value register., offset: 0x810 */
  __IO uint32_t CLKDIV;                            /**< Clock pre-divider for the entire I2C interface. This determines what time increments are used for the MSTTIME register, and controls some timing of the Slave function., offset: 0x814 */
  __I  uint32_t INTSTAT;                           /**< Interrupt Status register for Master, Slave, and Monitor functions., offset: 0x818 */
       uint8_t RESERVED_1[4];
  __IO uint32_t MSTCTL;                            /**< Master control register., offset: 0x820 */
  __IO uint32_t MSTTIME;                           /**< Master timing configuration., offset: 0x824 */
  __IO uint32_t MSTDAT;                            /**< Combined Master receiver and transmitter data register., offset: 0x828 */
       uint8_t RESERVED_2[20];
  __IO uint32_t SLVCTL;                            /**< Slave control register., offset: 0x840 */
  __IO uint32_t SLVDAT;                            /**< Combined Slave receiver and transmitter data register., offset: 0x844 */
  __IO uint32_t SLVADR[4];                         /**< Slave address register., array offset: 0x848, array step: 0x4 */
  __IO uint32_t SLVQUAL0;                          /**< Slave Qualification for address 0., offset: 0x858 */
       uint8_t RESERVED_3[36];
  __I  uint32_t MONRXDAT;                          /**< Monitor receiver data register., offset: 0x880 */
       uint8_t RESERVED_4[1912];
  __I  uint32_t ID;                                /**< Peripheral identification register., offset: 0xFFC */
} I2C_Type;

/* ----------------------------------------------------------------------------
   -- I2C Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Masks I2C Register Masks
 * @{
 */

/*! @name CFG - Configuration for shared functions. */
#define I2C_CFG_MSTEN_MASK                       (0x1U)
#define I2C_CFG_MSTEN_SHIFT                      (0U)
#define I2C_CFG_MSTEN(x)                         (((uint32_t)(((uint32_t)(x)) << I2C_CFG_MSTEN_SHIFT)) & I2C_CFG_MSTEN_MASK)
#define I2C_CFG_SLVEN_MASK                       (0x2U)
#define I2C_CFG_SLVEN_SHIFT                      (1U)
#define I2C_CFG_SLVEN(x)                         (((uint32_t)(((uint32_t)(x)) << I2C_CFG_SLVEN_SHIFT)) & I2C_CFG_SLVEN_MASK)
#define I2C_CFG_MONEN_MASK                       (0x4U)
#define I2C_CFG_MONEN_SHIFT                      (2U)
#define I2C_CFG_MONEN(x)                         (((uint32_t)(((uint32_t)(x)) << I2C_CFG_MONEN_SHIFT)) & I2C_CFG_MONEN_MASK)
#define I2C_CFG_TIMEOUTEN_MASK                   (0x8U)
#define I2C_CFG_TIMEOUTEN_SHIFT                  (3U)
#define I2C_CFG_TIMEOUTEN(x)                     (((uint32_t)(((uint32_t)(x)) << I2C_CFG_TIMEOUTEN_SHIFT)) & I2C_CFG_TIMEOUTEN_MASK)
#define I2C_CFG_MONCLKSTR_MASK                   (0x10U)
#define I2C_CFG_MONCLKSTR_SHIFT                  (4U)
#define I2C_CFG_MONCLKSTR(x)                     (((uint32_t)(((uint32_t)(x)) << I2C_CFG_MONCLKSTR_SHIFT)) & I2C_CFG_MONCLKSTR_MASK)
#define I2C_CFG_HSCAPABLE_MASK                   (0x20U)
#define I2C_CFG_HSCAPABLE_SHIFT                  (5U)
#define I2C_CFG_HSCAPABLE(x)                     (((uint32_t)(((uint32_t)(x)) << I2C_CFG_HSCAPABLE_SHIFT)) & I2C_CFG_HSCAPABLE_MASK)

/*! @name STAT - Status register for Master, Slave, and Monitor functions. */
#define I2C_STAT_MSTPENDING_MASK                 (0x1U)
#define I2C_STAT_MSTPENDING_SHIFT                (0U)
#define I2C_STAT_MSTPENDING(x)                   (((uint32_t)(((uint32_t)(x)) << I2C_STAT_MSTPENDING_SHIFT)) & I2C_STAT_MSTPENDING_MASK)
#define I2C_STAT_MSTSTATE_MASK                   (0xEU)
#define I2C_STAT_MSTSTATE_SHIFT                  (1U)
#define I2C_STAT_MSTSTATE(x)                     (((uint32_t)(((uint32_t)(x)) << I2C_STAT_MSTSTATE_SHIFT)) & I2C_STAT_MSTSTATE_MASK)
#define I2C_STAT_MSTARBLOSS_MASK                 (0x10U)
#define I2C_STAT_MSTARBLOSS_SHIFT                (4U)
#define I2C_STAT_MSTARBLOSS(x)                   (((uint32_t)(((uint32_t)(x)) << I2C_STAT_MSTARBLOSS_SHIFT)) & I2C_STAT_MSTARBLOSS_MASK)
#define I2C_STAT_MSTSTSTPERR_MASK                (0x40U)
#define I2C_STAT_MSTSTSTPERR_SHIFT               (6U)
#define I2C_STAT_MSTSTSTPERR(x)                  (((uint32_t)(((uint32_t)(x)) << I2C_STAT_MSTSTSTPERR_SHIFT)) & I2C_STAT_MSTSTSTPERR_MASK)
#define I2C_STAT_SLVPENDING_MASK                 (0x100U)
#define I2C_STAT_SLVPENDING_SHIFT                (8U)
#define I2C_STAT_SLVPENDING(x)                   (((uint32_t)(((uint32_t)(x)) << I2C_STAT_SLVPENDING_SHIFT)) & I2C_STAT_SLVPENDING_MASK)
#define I2C_STAT_SLVSTATE_MASK                   (0x600U)
#define I2C_STAT_SLVSTATE_SHIFT                  (9U)
#define I2C_STAT_SLVSTATE(x)                     (((uint32_t)(((uint32_t)(x)) << I2C_STAT_SLVSTATE_SHIFT)) & I2C_STAT_SLVSTATE_MASK)
#define I2C_STAT_SLVNOTSTR_MASK                  (0x800U)
#define I2C_STAT_SLVNOTSTR_SHIFT                 (11U)
#define I2C_STAT_SLVNOTSTR(x)                    (((uint32_t)(((uint32_t)(x)) << I2C_STAT_SLVNOTSTR_SHIFT)) & I2C_STAT_SLVNOTSTR_MASK)
#define I2C_STAT_SLVIDX_MASK                     (0x3000U)
#define I2C_STAT_SLVIDX_SHIFT                    (12U)
#define I2C_STAT_SLVIDX(x)                       (((uint32_t)(((uint32_t)(x)) << I2C_STAT_SLVIDX_SHIFT)) & I2C_STAT_SLVIDX_MASK)
#define I2C_STAT_SLVSEL_MASK                     (0x4000U)
#define I2C_STAT_SLVSEL_SHIFT                    (14U)
#define I2C_STAT_SLVSEL(x)                       (((uint32_t)(((uint32_t)(x)) << I2C_STAT_SLVSEL_SHIFT)) & I2C_STAT_SLVSEL_MASK)
#define I2C_STAT_SLVDESEL_MASK                   (0x8000U)
#define I2C_STAT_SLVDESEL_SHIFT                  (15U)
#define I2C_STAT_SLVDESEL(x)                     (((uint32_t)(((uint32_t)(x)) << I2C_STAT_SLVDESEL_SHIFT)) & I2C_STAT_SLVDESEL_MASK)
#define I2C_STAT_MONRDY_MASK                     (0x10000U)
#define I2C_STAT_MONRDY_SHIFT                    (16U)
#define I2C_STAT_MONRDY(x)                       (((uint32_t)(((uint32_t)(x)) << I2C_STAT_MONRDY_SHIFT)) & I2C_STAT_MONRDY_MASK)
#define I2C_STAT_MONOV_MASK                      (0x20000U)
#define I2C_STAT_MONOV_SHIFT                     (17U)
#define I2C_STAT_MONOV(x)                        (((uint32_t)(((uint32_t)(x)) << I2C_STAT_MONOV_SHIFT)) & I2C_STAT_MONOV_MASK)
#define I2C_STAT_MONACTIVE_MASK                  (0x40000U)
#define I2C_STAT_MONACTIVE_SHIFT                 (18U)
#define I2C_STAT_MONACTIVE(x)                    (((uint32_t)(((uint32_t)(x)) << I2C_STAT_MONACTIVE_SHIFT)) & I2C_STAT_MONACTIVE_MASK)
#define I2C_STAT_MONIDLE_MASK                    (0x80000U)
#define I2C_STAT_MONIDLE_SHIFT                   (19U)
#define I2C_STAT_MONIDLE(x)                      (((uint32_t)(((uint32_t)(x)) << I2C_STAT_MONIDLE_SHIFT)) & I2C_STAT_MONIDLE_MASK)
#define I2C_STAT_EVENTTIMEOUT_MASK               (0x1000000U)
#define I2C_STAT_EVENTTIMEOUT_SHIFT              (24U)
#define I2C_STAT_EVENTTIMEOUT(x)                 (((uint32_t)(((uint32_t)(x)) << I2C_STAT_EVENTTIMEOUT_SHIFT)) & I2C_STAT_EVENTTIMEOUT_MASK)
#define I2C_STAT_SCLTIMEOUT_MASK                 (0x2000000U)
#define I2C_STAT_SCLTIMEOUT_SHIFT                (25U)
#define I2C_STAT_SCLTIMEOUT(x)                   (((uint32_t)(((uint32_t)(x)) << I2C_STAT_SCLTIMEOUT_SHIFT)) & I2C_STAT_SCLTIMEOUT_MASK)

/*! @name INTENSET - Interrupt Enable Set and read register. */
#define I2C_INTENSET_MSTPENDINGEN_MASK           (0x1U)
#define I2C_INTENSET_MSTPENDINGEN_SHIFT          (0U)
#define I2C_INTENSET_MSTPENDINGEN(x)             (((uint32_t)(((uint32_t)(x)) << I2C_INTENSET_MSTPENDINGEN_SHIFT)) & I2C_INTENSET_MSTPENDINGEN_MASK)
#define I2C_INTENSET_MSTARBLOSSEN_MASK           (0x10U)
#define I2C_INTENSET_MSTARBLOSSEN_SHIFT          (4U)
#define I2C_INTENSET_MSTARBLOSSEN(x)             (((uint32_t)(((uint32_t)(x)) << I2C_INTENSET_MSTARBLOSSEN_SHIFT)) & I2C_INTENSET_MSTARBLOSSEN_MASK)
#define I2C_INTENSET_MSTSTSTPERREN_MASK          (0x40U)
#define I2C_INTENSET_MSTSTSTPERREN_SHIFT         (6U)
#define I2C_INTENSET_MSTSTSTPERREN(x)            (((uint32_t)(((uint32_t)(x)) << I2C_INTENSET_MSTSTSTPERREN_SHIFT)) & I2C_INTENSET_MSTSTSTPERREN_MASK)
#define I2C_INTENSET_SLVPENDINGEN_MASK           (0x100U)
#define I2C_INTENSET_SLVPENDINGEN_SHIFT          (8U)
#define I2C_INTENSET_SLVPENDINGEN(x)             (((uint32_t)(((uint32_t)(x)) << I2C_INTENSET_SLVPENDINGEN_SHIFT)) & I2C_INTENSET_SLVPENDINGEN_MASK)
#define I2C_INTENSET_SLVNOTSTREN_MASK            (0x800U)
#define I2C_INTENSET_SLVNOTSTREN_SHIFT           (11U)
#define I2C_INTENSET_SLVNOTSTREN(x)              (((uint32_t)(((uint32_t)(x)) << I2C_INTENSET_SLVNOTSTREN_SHIFT)) & I2C_INTENSET_SLVNOTSTREN_MASK)
#define I2C_INTENSET_SLVDESELEN_MASK             (0x8000U)
#define I2C_INTENSET_SLVDESELEN_SHIFT            (15U)
#define I2C_INTENSET_SLVDESELEN(x)               (((uint32_t)(((uint32_t)(x)) << I2C_INTENSET_SLVDESELEN_SHIFT)) & I2C_INTENSET_SLVDESELEN_MASK)
#define I2C_INTENSET_MONRDYEN_MASK               (0x10000U)
#define I2C_INTENSET_MONRDYEN_SHIFT              (16U)
#define I2C_INTENSET_MONRDYEN(x)                 (((uint32_t)(((uint32_t)(x)) << I2C_INTENSET_MONRDYEN_SHIFT)) & I2C_INTENSET_MONRDYEN_MASK)
#define I2C_INTENSET_MONOVEN_MASK                (0x20000U)
#define I2C_INTENSET_MONOVEN_SHIFT               (17U)
#define I2C_INTENSET_MONOVEN(x)                  (((uint32_t)(((uint32_t)(x)) << I2C_INTENSET_MONOVEN_SHIFT)) & I2C_INTENSET_MONOVEN_MASK)
#define I2C_INTENSET_MONIDLEEN_MASK              (0x80000U)
#define I2C_INTENSET_MONIDLEEN_SHIFT             (19U)
#define I2C_INTENSET_MONIDLEEN(x)                (((uint32_t)(((uint32_t)(x)) << I2C_INTENSET_MONIDLEEN_SHIFT)) & I2C_INTENSET_MONIDLEEN_MASK)
#define I2C_INTENSET_EVENTTIMEOUTEN_MASK         (0x1000000U)
#define I2C_INTENSET_EVENTTIMEOUTEN_SHIFT        (24U)
#define I2C_INTENSET_EVENTTIMEOUTEN(x)           (((uint32_t)(((uint32_t)(x)) << I2C_INTENSET_EVENTTIMEOUTEN_SHIFT)) & I2C_INTENSET_EVENTTIMEOUTEN_MASK)
#define I2C_INTENSET_SCLTIMEOUTEN_MASK           (0x2000000U)
#define I2C_INTENSET_SCLTIMEOUTEN_SHIFT          (25U)
#define I2C_INTENSET_SCLTIMEOUTEN(x)             (((uint32_t)(((uint32_t)(x)) << I2C_INTENSET_SCLTIMEOUTEN_SHIFT)) & I2C_INTENSET_SCLTIMEOUTEN_MASK)

/*! @name INTENCLR - Interrupt Enable Clear register. */
#define I2C_INTENCLR_MSTPENDINGCLR_MASK          (0x1U)
#define I2C_INTENCLR_MSTPENDINGCLR_SHIFT         (0U)
#define I2C_INTENCLR_MSTPENDINGCLR(x)            (((uint32_t)(((uint32_t)(x)) << I2C_INTENCLR_MSTPENDINGCLR_SHIFT)) & I2C_INTENCLR_MSTPENDINGCLR_MASK)
#define I2C_INTENCLR_MSTARBLOSSCLR_MASK          (0x10U)
#define I2C_INTENCLR_MSTARBLOSSCLR_SHIFT         (4U)
#define I2C_INTENCLR_MSTARBLOSSCLR(x)            (((uint32_t)(((uint32_t)(x)) << I2C_INTENCLR_MSTARBLOSSCLR_SHIFT)) & I2C_INTENCLR_MSTARBLOSSCLR_MASK)
#define I2C_INTENCLR_MSTSTSTPERRCLR_MASK         (0x40U)
#define I2C_INTENCLR_MSTSTSTPERRCLR_SHIFT        (6U)
#define I2C_INTENCLR_MSTSTSTPERRCLR(x)           (((uint32_t)(((uint32_t)(x)) << I2C_INTENCLR_MSTSTSTPERRCLR_SHIFT)) & I2C_INTENCLR_MSTSTSTPERRCLR_MASK)
#define I2C_INTENCLR_SLVPENDINGCLR_MASK          (0x100U)
#define I2C_INTENCLR_SLVPENDINGCLR_SHIFT         (8U)
#define I2C_INTENCLR_SLVPENDINGCLR(x)            (((uint32_t)(((uint32_t)(x)) << I2C_INTENCLR_SLVPENDINGCLR_SHIFT)) & I2C_INTENCLR_SLVPENDINGCLR_MASK)
#define I2C_INTENCLR_SLVNOTSTRCLR_MASK           (0x800U)
#define I2C_INTENCLR_SLVNOTSTRCLR_SHIFT          (11U)
#define I2C_INTENCLR_SLVNOTSTRCLR(x)             (((uint32_t)(((uint32_t)(x)) << I2C_INTENCLR_SLVNOTSTRCLR_SHIFT)) & I2C_INTENCLR_SLVNOTSTRCLR_MASK)
#define I2C_INTENCLR_SLVDESELCLR_MASK            (0x8000U)
#define I2C_INTENCLR_SLVDESELCLR_SHIFT           (15U)
#define I2C_INTENCLR_SLVDESELCLR(x)              (((uint32_t)(((uint32_t)(x)) << I2C_INTENCLR_SLVDESELCLR_SHIFT)) & I2C_INTENCLR_SLVDESELCLR_MASK)
#define I2C_INTENCLR_MONRDYCLR_MASK              (0x10000U)
#define I2C_INTENCLR_MONRDYCLR_SHIFT             (16U)
#define I2C_INTENCLR_MONRDYCLR(x)                (((uint32_t)(((uint32_t)(x)) << I2C_INTENCLR_MONRDYCLR_SHIFT)) & I2C_INTENCLR_MONRDYCLR_MASK)
#define I2C_INTENCLR_MONOVCLR_MASK               (0x20000U)
#define I2C_INTENCLR_MONOVCLR_SHIFT              (17U)
#define I2C_INTENCLR_MONOVCLR(x)                 (((uint32_t)(((uint32_t)(x)) << I2C_INTENCLR_MONOVCLR_SHIFT)) & I2C_INTENCLR_MONOVCLR_MASK)
#define I2C_INTENCLR_MONIDLECLR_MASK             (0x80000U)
#define I2C_INTENCLR_MONIDLECLR_SHIFT            (19U)
#define I2C_INTENCLR_MONIDLECLR(x)               (((uint32_t)(((uint32_t)(x)) << I2C_INTENCLR_MONIDLECLR_SHIFT)) & I2C_INTENCLR_MONIDLECLR_MASK)
#define I2C_INTENCLR_EVENTTIMEOUTCLR_MASK        (0x1000000U)
#define I2C_INTENCLR_EVENTTIMEOUTCLR_SHIFT       (24U)
#define I2C_INTENCLR_EVENTTIMEOUTCLR(x)          (((uint32_t)(((uint32_t)(x)) << I2C_INTENCLR_EVENTTIMEOUTCLR_SHIFT)) & I2C_INTENCLR_EVENTTIMEOUTCLR_MASK)
#define I2C_INTENCLR_SCLTIMEOUTCLR_MASK          (0x2000000U)
#define I2C_INTENCLR_SCLTIMEOUTCLR_SHIFT         (25U)
#define I2C_INTENCLR_SCLTIMEOUTCLR(x)            (((uint32_t)(((uint32_t)(x)) << I2C_INTENCLR_SCLTIMEOUTCLR_SHIFT)) & I2C_INTENCLR_SCLTIMEOUTCLR_MASK)

/*! @name TIMEOUT - Time-out value register. */
#define I2C_TIMEOUT_TOMIN_MASK                   (0xFU)
#define I2C_TIMEOUT_TOMIN_SHIFT                  (0U)
#define I2C_TIMEOUT_TOMIN(x)                     (((uint32_t)(((uint32_t)(x)) << I2C_TIMEOUT_TOMIN_SHIFT)) & I2C_TIMEOUT_TOMIN_MASK)
#define I2C_TIMEOUT_TO_MASK                      (0xFFF0U)
#define I2C_TIMEOUT_TO_SHIFT                     (4U)
#define I2C_TIMEOUT_TO(x)                        (((uint32_t)(((uint32_t)(x)) << I2C_TIMEOUT_TO_SHIFT)) & I2C_TIMEOUT_TO_MASK)

/*! @name CLKDIV - Clock pre-divider for the entire I2C interface. This determines what time increments are used for the MSTTIME register, and controls some timing of the Slave function. */
#define I2C_CLKDIV_DIVVAL_MASK                   (0xFFFFU)
#define I2C_CLKDIV_DIVVAL_SHIFT                  (0U)
#define I2C_CLKDIV_DIVVAL(x)                     (((uint32_t)(((uint32_t)(x)) << I2C_CLKDIV_DIVVAL_SHIFT)) & I2C_CLKDIV_DIVVAL_MASK)

/*! @name INTSTAT - Interrupt Status register for Master, Slave, and Monitor functions. */
#define I2C_INTSTAT_MSTPENDING_MASK              (0x1U)
#define I2C_INTSTAT_MSTPENDING_SHIFT             (0U)
#define I2C_INTSTAT_MSTPENDING(x)                (((uint32_t)(((uint32_t)(x)) << I2C_INTSTAT_MSTPENDING_SHIFT)) & I2C_INTSTAT_MSTPENDING_MASK)
#define I2C_INTSTAT_MSTARBLOSS_MASK              (0x10U)
#define I2C_INTSTAT_MSTARBLOSS_SHIFT             (4U)
#define I2C_INTSTAT_MSTARBLOSS(x)                (((uint32_t)(((uint32_t)(x)) << I2C_INTSTAT_MSTARBLOSS_SHIFT)) & I2C_INTSTAT_MSTARBLOSS_MASK)
#define I2C_INTSTAT_MSTSTSTPERR_MASK             (0x40U)
#define I2C_INTSTAT_MSTSTSTPERR_SHIFT            (6U)
#define I2C_INTSTAT_MSTSTSTPERR(x)               (((uint32_t)(((uint32_t)(x)) << I2C_INTSTAT_MSTSTSTPERR_SHIFT)) & I2C_INTSTAT_MSTSTSTPERR_MASK)
#define I2C_INTSTAT_SLVPENDING_MASK              (0x100U)
#define I2C_INTSTAT_SLVPENDING_SHIFT             (8U)
#define I2C_INTSTAT_SLVPENDING(x)                (((uint32_t)(((uint32_t)(x)) << I2C_INTSTAT_SLVPENDING_SHIFT)) & I2C_INTSTAT_SLVPENDING_MASK)
#define I2C_INTSTAT_SLVNOTSTR_MASK               (0x800U)
#define I2C_INTSTAT_SLVNOTSTR_SHIFT              (11U)
#define I2C_INTSTAT_SLVNOTSTR(x)                 (((uint32_t)(((uint32_t)(x)) << I2C_INTSTAT_SLVNOTSTR_SHIFT)) & I2C_INTSTAT_SLVNOTSTR_MASK)
#define I2C_INTSTAT_SLVDESEL_MASK                (0x8000U)
#define I2C_INTSTAT_SLVDESEL_SHIFT               (15U)
#define I2C_INTSTAT_SLVDESEL(x)                  (((uint32_t)(((uint32_t)(x)) << I2C_INTSTAT_SLVDESEL_SHIFT)) & I2C_INTSTAT_SLVDESEL_MASK)
#define I2C_INTSTAT_MONRDY_MASK                  (0x10000U)
#define I2C_INTSTAT_MONRDY_SHIFT                 (16U)
#define I2C_INTSTAT_MONRDY(x)                    (((uint32_t)(((uint32_t)(x)) << I2C_INTSTAT_MONRDY_SHIFT)) & I2C_INTSTAT_MONRDY_MASK)
#define I2C_INTSTAT_MONOV_MASK                   (0x20000U)
#define I2C_INTSTAT_MONOV_SHIFT                  (17U)
#define I2C_INTSTAT_MONOV(x)                     (((uint32_t)(((uint32_t)(x)) << I2C_INTSTAT_MONOV_SHIFT)) & I2C_INTSTAT_MONOV_MASK)
#define I2C_INTSTAT_MONIDLE_MASK                 (0x80000U)
#define I2C_INTSTAT_MONIDLE_SHIFT                (19U)
#define I2C_INTSTAT_MONIDLE(x)                   (((uint32_t)(((uint32_t)(x)) << I2C_INTSTAT_MONIDLE_SHIFT)) & I2C_INTSTAT_MONIDLE_MASK)
#define I2C_INTSTAT_EVENTTIMEOUT_MASK            (0x1000000U)
#define I2C_INTSTAT_EVENTTIMEOUT_SHIFT           (24U)
#define I2C_INTSTAT_EVENTTIMEOUT(x)              (((uint32_t)(((uint32_t)(x)) << I2C_INTSTAT_EVENTTIMEOUT_SHIFT)) & I2C_INTSTAT_EVENTTIMEOUT_MASK)
#define I2C_INTSTAT_SCLTIMEOUT_MASK              (0x2000000U)
#define I2C_INTSTAT_SCLTIMEOUT_SHIFT             (25U)
#define I2C_INTSTAT_SCLTIMEOUT(x)                (((uint32_t)(((uint32_t)(x)) << I2C_INTSTAT_SCLTIMEOUT_SHIFT)) & I2C_INTSTAT_SCLTIMEOUT_MASK)

/*! @name MSTCTL - Master control register. */
#define I2C_MSTCTL_MSTCONTINUE_MASK              (0x1U)
#define I2C_MSTCTL_MSTCONTINUE_SHIFT             (0U)
#define I2C_MSTCTL_MSTCONTINUE(x)                (((uint32_t)(((uint32_t)(x)) << I2C_MSTCTL_MSTCONTINUE_SHIFT)) & I2C_MSTCTL_MSTCONTINUE_MASK)
#define I2C_MSTCTL_MSTSTART_MASK                 (0x2U)
#define I2C_MSTCTL_MSTSTART_SHIFT                (1U)
#define I2C_MSTCTL_MSTSTART(x)                   (((uint32_t)(((uint32_t)(x)) << I2C_MSTCTL_MSTSTART_SHIFT)) & I2C_MSTCTL_MSTSTART_MASK)
#define I2C_MSTCTL_MSTSTOP_MASK                  (0x4U)
#define I2C_MSTCTL_MSTSTOP_SHIFT                 (2U)
#define I2C_MSTCTL_MSTSTOP(x)                    (((uint32_t)(((uint32_t)(x)) << I2C_MSTCTL_MSTSTOP_SHIFT)) & I2C_MSTCTL_MSTSTOP_MASK)
#define I2C_MSTCTL_MSTDMA_MASK                   (0x8U)
#define I2C_MSTCTL_MSTDMA_SHIFT                  (3U)
#define I2C_MSTCTL_MSTDMA(x)                     (((uint32_t)(((uint32_t)(x)) << I2C_MSTCTL_MSTDMA_SHIFT)) & I2C_MSTCTL_MSTDMA_MASK)

/*! @name MSTTIME - Master timing configuration. */
#define I2C_MSTTIME_MSTSCLLOW_MASK               (0x7U)
#define I2C_MSTTIME_MSTSCLLOW_SHIFT              (0U)
#define I2C_MSTTIME_MSTSCLLOW(x)                 (((uint32_t)(((uint32_t)(x)) << I2C_MSTTIME_MSTSCLLOW_SHIFT)) & I2C_MSTTIME_MSTSCLLOW_MASK)
#define I2C_MSTTIME_MSTSCLHIGH_MASK              (0x70U)
#define I2C_MSTTIME_MSTSCLHIGH_SHIFT             (4U)
#define I2C_MSTTIME_MSTSCLHIGH(x)                (((uint32_t)(((uint32_t)(x)) << I2C_MSTTIME_MSTSCLHIGH_SHIFT)) & I2C_MSTTIME_MSTSCLHIGH_MASK)

/*! @name MSTDAT - Combined Master receiver and transmitter data register. */
#define I2C_MSTDAT_DATA_MASK                     (0xFFU)
#define I2C_MSTDAT_DATA_SHIFT                    (0U)
#define I2C_MSTDAT_DATA(x)                       (((uint32_t)(((uint32_t)(x)) << I2C_MSTDAT_DATA_SHIFT)) & I2C_MSTDAT_DATA_MASK)

/*! @name SLVCTL - Slave control register. */
#define I2C_SLVCTL_SLVCONTINUE_MASK              (0x1U)
#define I2C_SLVCTL_SLVCONTINUE_SHIFT             (0U)
#define I2C_SLVCTL_SLVCONTINUE(x)                (((uint32_t)(((uint32_t)(x)) << I2C_SLVCTL_SLVCONTINUE_SHIFT)) & I2C_SLVCTL_SLVCONTINUE_MASK)
#define I2C_SLVCTL_SLVNACK_MASK                  (0x2U)
#define I2C_SLVCTL_SLVNACK_SHIFT                 (1U)
#define I2C_SLVCTL_SLVNACK(x)                    (((uint32_t)(((uint32_t)(x)) << I2C_SLVCTL_SLVNACK_SHIFT)) & I2C_SLVCTL_SLVNACK_MASK)
#define I2C_SLVCTL_SLVDMA_MASK                   (0x8U)
#define I2C_SLVCTL_SLVDMA_SHIFT                  (3U)
#define I2C_SLVCTL_SLVDMA(x)                     (((uint32_t)(((uint32_t)(x)) << I2C_SLVCTL_SLVDMA_SHIFT)) & I2C_SLVCTL_SLVDMA_MASK)
#define I2C_SLVCTL_AUTOACK_MASK                  (0x100U)
#define I2C_SLVCTL_AUTOACK_SHIFT                 (8U)
#define I2C_SLVCTL_AUTOACK(x)                    (((uint32_t)(((uint32_t)(x)) << I2C_SLVCTL_AUTOACK_SHIFT)) & I2C_SLVCTL_AUTOACK_MASK)
#define I2C_SLVCTL_AUTOMATCHREAD_MASK            (0x200U)
#define I2C_SLVCTL_AUTOMATCHREAD_SHIFT           (9U)
#define I2C_SLVCTL_AUTOMATCHREAD(x)              (((uint32_t)(((uint32_t)(x)) << I2C_SLVCTL_AUTOMATCHREAD_SHIFT)) & I2C_SLVCTL_AUTOMATCHREAD_MASK)

/*! @name SLVDAT - Combined Slave receiver and transmitter data register. */
#define I2C_SLVDAT_DATA_MASK                     (0xFFU)
#define I2C_SLVDAT_DATA_SHIFT                    (0U)
#define I2C_SLVDAT_DATA(x)                       (((uint32_t)(((uint32_t)(x)) << I2C_SLVDAT_DATA_SHIFT)) & I2C_SLVDAT_DATA_MASK)

/*! @name SLVADR - Slave address register. */
#define I2C_SLVADR_SADISABLE_MASK                (0x1U)
#define I2C_SLVADR_SADISABLE_SHIFT               (0U)
#define I2C_SLVADR_SADISABLE(x)                  (((uint32_t)(((uint32_t)(x)) << I2C_SLVADR_SADISABLE_SHIFT)) & I2C_SLVADR_SADISABLE_MASK)
#define I2C_SLVADR_SLVADR_MASK                   (0xFEU)
#define I2C_SLVADR_SLVADR_SHIFT                  (1U)
#define I2C_SLVADR_SLVADR(x)                     (((uint32_t)(((uint32_t)(x)) << I2C_SLVADR_SLVADR_SHIFT)) & I2C_SLVADR_SLVADR_MASK)
#define I2C_SLVADR_AUTONACK_MASK                 (0x8000U)
#define I2C_SLVADR_AUTONACK_SHIFT                (15U)
#define I2C_SLVADR_AUTONACK(x)                   (((uint32_t)(((uint32_t)(x)) << I2C_SLVADR_AUTONACK_SHIFT)) & I2C_SLVADR_AUTONACK_MASK)

/* The count of I2C_SLVADR */
#define I2C_SLVADR_COUNT                         (4U)

/*! @name SLVQUAL0 - Slave Qualification for address 0. */
#define I2C_SLVQUAL0_QUALMODE0_MASK              (0x1U)
#define I2C_SLVQUAL0_QUALMODE0_SHIFT             (0U)
#define I2C_SLVQUAL0_QUALMODE0(x)                (((uint32_t)(((uint32_t)(x)) << I2C_SLVQUAL0_QUALMODE0_SHIFT)) & I2C_SLVQUAL0_QUALMODE0_MASK)
#define I2C_SLVQUAL0_SLVQUAL0_MASK               (0xFEU)
#define I2C_SLVQUAL0_SLVQUAL0_SHIFT              (1U)
#define I2C_SLVQUAL0_SLVQUAL0(x)                 (((uint32_t)(((uint32_t)(x)) << I2C_SLVQUAL0_SLVQUAL0_SHIFT)) & I2C_SLVQUAL0_SLVQUAL0_MASK)

/*! @name MONRXDAT - Monitor receiver data register. */
#define I2C_MONRXDAT_MONRXDAT_MASK               (0xFFU)
#define I2C_MONRXDAT_MONRXDAT_SHIFT              (0U)
#define I2C_MONRXDAT_MONRXDAT(x)                 (((uint32_t)(((uint32_t)(x)) << I2C_MONRXDAT_MONRXDAT_SHIFT)) & I2C_MONRXDAT_MONRXDAT_MASK)
#define I2C_MONRXDAT_MONSTART_MASK               (0x100U)
#define I2C_MONRXDAT_MONSTART_SHIFT              (8U)
#define I2C_MONRXDAT_MONSTART(x)                 (((uint32_t)(((uint32_t)(x)) << I2C_MONRXDAT_MONSTART_SHIFT)) & I2C_MONRXDAT_MONSTART_MASK)
#define I2C_MONRXDAT_MONRESTART_MASK             (0x200U)
#define I2C_MONRXDAT_MONRESTART_SHIFT            (9U)
#define I2C_MONRXDAT_MONRESTART(x)               (((uint32_t)(((uint32_t)(x)) << I2C_MONRXDAT_MONRESTART_SHIFT)) & I2C_MONRXDAT_MONRESTART_MASK)
#define I2C_MONRXDAT_MONNACK_MASK                (0x400U)
#define I2C_MONRXDAT_MONNACK_SHIFT               (10U)
#define I2C_MONRXDAT_MONNACK(x)                  (((uint32_t)(((uint32_t)(x)) << I2C_MONRXDAT_MONNACK_SHIFT)) & I2C_MONRXDAT_MONNACK_MASK)

/*! @name ID - Peripheral identification register. */
#define I2C_ID_APERTURE_MASK                     (0xFFU)
#define I2C_ID_APERTURE_SHIFT                    (0U)
#define I2C_ID_APERTURE(x)                       (((uint32_t)(((uint32_t)(x)) << I2C_ID_APERTURE_SHIFT)) & I2C_ID_APERTURE_MASK)
#define I2C_ID_MINOR_REV_MASK                    (0xF00U)
#define I2C_ID_MINOR_REV_SHIFT                   (8U)
#define I2C_ID_MINOR_REV(x)                      (((uint32_t)(((uint32_t)(x)) << I2C_ID_MINOR_REV_SHIFT)) & I2C_ID_MINOR_REV_MASK)
#define I2C_ID_MAJOR_REV_MASK                    (0xF000U)
#define I2C_ID_MAJOR_REV_SHIFT                   (12U)
#define I2C_ID_MAJOR_REV(x)                      (((uint32_t)(((uint32_t)(x)) << I2C_ID_MAJOR_REV_SHIFT)) & I2C_ID_MAJOR_REV_MASK)
#define I2C_ID_ID_MASK                           (0xFFFF0000U)
#define I2C_ID_ID_SHIFT                          (16U)
#define I2C_ID_ID(x)                             (((uint32_t)(((uint32_t)(x)) << I2C_ID_ID_SHIFT)) & I2C_ID_ID_MASK)


/*!
 * @}
 */ /* end of group I2C_Register_Masks */


/* I2C - Peripheral instance base addresses */
/** Peripheral I2C0 base address */
#define I2C0_BASE                                (0x40086000u)
/** Peripheral I2C0 base pointer */
#define I2C0                                     ((I2C_Type *)I2C0_BASE)
/** Peripheral I2C1 base address */
#define I2C1_BASE                                (0x40087000u)
/** Peripheral I2C1 base pointer */
#define I2C1                                     ((I2C_Type *)I2C1_BASE)
/** Peripheral I2C2 base address */
#define I2C2_BASE                                (0x40088000u)
/** Peripheral I2C2 base pointer */
#define I2C2                                     ((I2C_Type *)I2C2_BASE)
/** Peripheral I2C3 base address */
#define I2C3_BASE                                (0x40089000u)
/** Peripheral I2C3 base pointer */
#define I2C3                                     ((I2C_Type *)I2C3_BASE)
/** Peripheral I2C4 base address */
#define I2C4_BASE                                (0x4008A000u)
/** Peripheral I2C4 base pointer */
#define I2C4                                     ((I2C_Type *)I2C4_BASE)
/** Peripheral I2C5 base address */
#define I2C5_BASE                                (0x40096000u)
/** Peripheral I2C5 base pointer */
#define I2C5                                     ((I2C_Type *)I2C5_BASE)
/** Peripheral I2C6 base address */
#define I2C6_BASE                                (0x40097000u)
/** Peripheral I2C6 base pointer */
#define I2C6                                     ((I2C_Type *)I2C6_BASE)
/** Peripheral I2C7 base address */
#define I2C7_BASE                                (0x40098000u)
/** Peripheral I2C7 base pointer */
#define I2C7                                     ((I2C_Type *)I2C7_BASE)
/** Array initializer of I2C peripheral base addresses */
#define I2C_BASE_ADDRS                           { I2C0_BASE, I2C1_BASE, I2C2_BASE, I2C3_BASE, I2C4_BASE, I2C5_BASE, I2C6_BASE, I2C7_BASE }
/** Array initializer of I2C peripheral base pointers */
#define I2C_BASE_PTRS                            { I2C0, I2C1, I2C2, I2C3, I2C4, I2C5, I2C6, I2C7 }
/** Interrupt vectors for the I2C peripheral type */
#define I2C_IRQS                                 { FLEXCOMM0_IRQn, FLEXCOMM1_IRQn, FLEXCOMM2_IRQn, FLEXCOMM3_IRQn, FLEXCOMM4_IRQn, FLEXCOMM5_IRQn, FLEXCOMM6_IRQn, FLEXCOMM7_IRQn }

/*!
 * @}
 */ /* end of group I2C_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- I2S Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2S_Peripheral_Access_Layer I2S Peripheral Access Layer
 * @{
 */

/** I2S - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[3072];
  __IO uint32_t CFG1;                              /**< Configuration register 1 for the primary channel pair., offset: 0xC00 */
  __IO uint32_t CFG2;                              /**< Configuration register 2 for the primary channel pair., offset: 0xC04 */
  __IO uint32_t STAT;                              /**< Status register for the primary channel pair., offset: 0xC08 */
       uint8_t RESERVED_1[16];
  __IO uint32_t DIV;                               /**< Clock divider, used by all channel pairs., offset: 0xC1C */
       uint8_t RESERVED_2[480];
  __IO uint32_t FIFOCFG;                           /**< FIFO configuration and enable register., offset: 0xE00 */
  __IO uint32_t FIFOSTAT;                          /**< FIFO status register., offset: 0xE04 */
  __IO uint32_t FIFOTRIG;                          /**< FIFO trigger settings for interrupt and DMA request., offset: 0xE08 */
       uint8_t RESERVED_3[4];
  __IO uint32_t FIFOINTENSET;                      /**< FIFO interrupt enable set (enable) and read register., offset: 0xE10 */
  __IO uint32_t FIFOINTENCLR;                      /**< FIFO interrupt enable clear (disable) and read register., offset: 0xE14 */
  __I  uint32_t FIFOINTSTAT;                       /**< FIFO interrupt status register., offset: 0xE18 */
       uint8_t RESERVED_4[4];
  __O  uint32_t FIFOWR;                            /**< FIFO write data., offset: 0xE20 */
  __O  uint32_t FIFOWR48H;                         /**< FIFO write data for upper data bits. May only be used if the I2S is configured for 2x 24-bit data and not using DMA., offset: 0xE24 */
       uint8_t RESERVED_5[8];
  __I  uint32_t FIFORD;                            /**< FIFO read data., offset: 0xE30 */
  __I  uint32_t FIFORD48H;                         /**< FIFO read data for upper data bits. May only be used if the I2S is configured for 2x 24-bit data and not using DMA., offset: 0xE34 */
       uint8_t RESERVED_6[8];
  __I  uint32_t FIFORDNOPOP;                       /**< FIFO data read with no FIFO pop., offset: 0xE40 */
  __I  uint32_t FIFORD48HNOPOP;                    /**< FIFO data read for upper data bits with no FIFO pop. May only be used if the I2S is configured for 2x 24-bit data and not using DMA., offset: 0xE44 */
       uint8_t RESERVED_7[436];
  __I  uint32_t ID;                                /**< I2S Module identification, offset: 0xFFC */
} I2S_Type;

/* ----------------------------------------------------------------------------
   -- I2S Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2S_Register_Masks I2S Register Masks
 * @{
 */

/*! @name CFG1 - Configuration register 1 for the primary channel pair. */
#define I2S_CFG1_MAINENABLE_MASK                 (0x1U)
#define I2S_CFG1_MAINENABLE_SHIFT                (0U)
#define I2S_CFG1_MAINENABLE(x)                   (((uint32_t)(((uint32_t)(x)) << I2S_CFG1_MAINENABLE_SHIFT)) & I2S_CFG1_MAINENABLE_MASK)
#define I2S_CFG1_DATAPAUSE_MASK                  (0x2U)
#define I2S_CFG1_DATAPAUSE_SHIFT                 (1U)
#define I2S_CFG1_DATAPAUSE(x)                    (((uint32_t)(((uint32_t)(x)) << I2S_CFG1_DATAPAUSE_SHIFT)) & I2S_CFG1_DATAPAUSE_MASK)
#define I2S_CFG1_PAIRCOUNT_MASK                  (0xCU)
#define I2S_CFG1_PAIRCOUNT_SHIFT                 (2U)
#define I2S_CFG1_PAIRCOUNT(x)                    (((uint32_t)(((uint32_t)(x)) << I2S_CFG1_PAIRCOUNT_SHIFT)) & I2S_CFG1_PAIRCOUNT_MASK)
#define I2S_CFG1_MSTSLVCFG_MASK                  (0x30U)
#define I2S_CFG1_MSTSLVCFG_SHIFT                 (4U)
#define I2S_CFG1_MSTSLVCFG(x)                    (((uint32_t)(((uint32_t)(x)) << I2S_CFG1_MSTSLVCFG_SHIFT)) & I2S_CFG1_MSTSLVCFG_MASK)
#define I2S_CFG1_MODE_MASK                       (0xC0U)
#define I2S_CFG1_MODE_SHIFT                      (6U)
#define I2S_CFG1_MODE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_CFG1_MODE_SHIFT)) & I2S_CFG1_MODE_MASK)
#define I2S_CFG1_RIGHTLOW_MASK                   (0x100U)
#define I2S_CFG1_RIGHTLOW_SHIFT                  (8U)
#define I2S_CFG1_RIGHTLOW(x)                     (((uint32_t)(((uint32_t)(x)) << I2S_CFG1_RIGHTLOW_SHIFT)) & I2S_CFG1_RIGHTLOW_MASK)
#define I2S_CFG1_LEFTJUST_MASK                   (0x200U)
#define I2S_CFG1_LEFTJUST_SHIFT                  (9U)
#define I2S_CFG1_LEFTJUST(x)                     (((uint32_t)(((uint32_t)(x)) << I2S_CFG1_LEFTJUST_SHIFT)) & I2S_CFG1_LEFTJUST_MASK)
#define I2S_CFG1_ONECHANNEL_MASK                 (0x400U)
#define I2S_CFG1_ONECHANNEL_SHIFT                (10U)
#define I2S_CFG1_ONECHANNEL(x)                   (((uint32_t)(((uint32_t)(x)) << I2S_CFG1_ONECHANNEL_SHIFT)) & I2S_CFG1_ONECHANNEL_MASK)
#define I2S_CFG1_SCK_POL_MASK                    (0x1000U)
#define I2S_CFG1_SCK_POL_SHIFT                   (12U)
#define I2S_CFG1_SCK_POL(x)                      (((uint32_t)(((uint32_t)(x)) << I2S_CFG1_SCK_POL_SHIFT)) & I2S_CFG1_SCK_POL_MASK)
#define I2S_CFG1_WS_POL_MASK                     (0x2000U)
#define I2S_CFG1_WS_POL_SHIFT                    (13U)
#define I2S_CFG1_WS_POL(x)                       (((uint32_t)(((uint32_t)(x)) << I2S_CFG1_WS_POL_SHIFT)) & I2S_CFG1_WS_POL_MASK)
#define I2S_CFG1_DATALEN_MASK                    (0x1F0000U)
#define I2S_CFG1_DATALEN_SHIFT                   (16U)
#define I2S_CFG1_DATALEN(x)                      (((uint32_t)(((uint32_t)(x)) << I2S_CFG1_DATALEN_SHIFT)) & I2S_CFG1_DATALEN_MASK)

/*! @name CFG2 - Configuration register 2 for the primary channel pair. */
#define I2S_CFG2_FRAMELEN_MASK                   (0x1FFU)
#define I2S_CFG2_FRAMELEN_SHIFT                  (0U)
#define I2S_CFG2_FRAMELEN(x)                     (((uint32_t)(((uint32_t)(x)) << I2S_CFG2_FRAMELEN_SHIFT)) & I2S_CFG2_FRAMELEN_MASK)
#define I2S_CFG2_POSITION_MASK                   (0x1FF0000U)
#define I2S_CFG2_POSITION_SHIFT                  (16U)
#define I2S_CFG2_POSITION(x)                     (((uint32_t)(((uint32_t)(x)) << I2S_CFG2_POSITION_SHIFT)) & I2S_CFG2_POSITION_MASK)

/*! @name STAT - Status register for the primary channel pair. */
#define I2S_STAT_BUSY_MASK                       (0x1U)
#define I2S_STAT_BUSY_SHIFT                      (0U)
#define I2S_STAT_BUSY(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_STAT_BUSY_SHIFT)) & I2S_STAT_BUSY_MASK)
#define I2S_STAT_SLVFRMERR_MASK                  (0x2U)
#define I2S_STAT_SLVFRMERR_SHIFT                 (1U)
#define I2S_STAT_SLVFRMERR(x)                    (((uint32_t)(((uint32_t)(x)) << I2S_STAT_SLVFRMERR_SHIFT)) & I2S_STAT_SLVFRMERR_MASK)
#define I2S_STAT_LR_MASK                         (0x4U)
#define I2S_STAT_LR_SHIFT                        (2U)
#define I2S_STAT_LR(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_STAT_LR_SHIFT)) & I2S_STAT_LR_MASK)
#define I2S_STAT_DATAPAUSED_MASK                 (0x8U)
#define I2S_STAT_DATAPAUSED_SHIFT                (3U)
#define I2S_STAT_DATAPAUSED(x)                   (((uint32_t)(((uint32_t)(x)) << I2S_STAT_DATAPAUSED_SHIFT)) & I2S_STAT_DATAPAUSED_MASK)

/*! @name DIV - Clock divider, used by all channel pairs. */
#define I2S_DIV_DIV_MASK                         (0xFFFU)
#define I2S_DIV_DIV_SHIFT                        (0U)
#define I2S_DIV_DIV(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_DIV_DIV_SHIFT)) & I2S_DIV_DIV_MASK)

/*! @name FIFOCFG - FIFO configuration and enable register. */
#define I2S_FIFOCFG_ENABLETX_MASK                (0x1U)
#define I2S_FIFOCFG_ENABLETX_SHIFT               (0U)
#define I2S_FIFOCFG_ENABLETX(x)                  (((uint32_t)(((uint32_t)(x)) << I2S_FIFOCFG_ENABLETX_SHIFT)) & I2S_FIFOCFG_ENABLETX_MASK)
#define I2S_FIFOCFG_ENABLERX_MASK                (0x2U)
#define I2S_FIFOCFG_ENABLERX_SHIFT               (1U)
#define I2S_FIFOCFG_ENABLERX(x)                  (((uint32_t)(((uint32_t)(x)) << I2S_FIFOCFG_ENABLERX_SHIFT)) & I2S_FIFOCFG_ENABLERX_MASK)
#define I2S_FIFOCFG_TXI2SSE0_MASK                (0x4U)
#define I2S_FIFOCFG_TXI2SSE0_SHIFT               (2U)
#define I2S_FIFOCFG_TXI2SSE0(x)                  (((uint32_t)(((uint32_t)(x)) << I2S_FIFOCFG_TXI2SSE0_SHIFT)) & I2S_FIFOCFG_TXI2SSE0_MASK)
#define I2S_FIFOCFG_PACK48_MASK                  (0x8U)
#define I2S_FIFOCFG_PACK48_SHIFT                 (3U)
#define I2S_FIFOCFG_PACK48(x)                    (((uint32_t)(((uint32_t)(x)) << I2S_FIFOCFG_PACK48_SHIFT)) & I2S_FIFOCFG_PACK48_MASK)
#define I2S_FIFOCFG_SIZE_MASK                    (0x30U)
#define I2S_FIFOCFG_SIZE_SHIFT                   (4U)
#define I2S_FIFOCFG_SIZE(x)                      (((uint32_t)(((uint32_t)(x)) << I2S_FIFOCFG_SIZE_SHIFT)) & I2S_FIFOCFG_SIZE_MASK)
#define I2S_FIFOCFG_DMATX_MASK                   (0x1000U)
#define I2S_FIFOCFG_DMATX_SHIFT                  (12U)
#define I2S_FIFOCFG_DMATX(x)                     (((uint32_t)(((uint32_t)(x)) << I2S_FIFOCFG_DMATX_SHIFT)) & I2S_FIFOCFG_DMATX_MASK)
#define I2S_FIFOCFG_DMARX_MASK                   (0x2000U)
#define I2S_FIFOCFG_DMARX_SHIFT                  (13U)
#define I2S_FIFOCFG_DMARX(x)                     (((uint32_t)(((uint32_t)(x)) << I2S_FIFOCFG_DMARX_SHIFT)) & I2S_FIFOCFG_DMARX_MASK)
#define I2S_FIFOCFG_WAKETX_MASK                  (0x4000U)
#define I2S_FIFOCFG_WAKETX_SHIFT                 (14U)
#define I2S_FIFOCFG_WAKETX(x)                    (((uint32_t)(((uint32_t)(x)) << I2S_FIFOCFG_WAKETX_SHIFT)) & I2S_FIFOCFG_WAKETX_MASK)
#define I2S_FIFOCFG_WAKERX_MASK                  (0x8000U)
#define I2S_FIFOCFG_WAKERX_SHIFT                 (15U)
#define I2S_FIFOCFG_WAKERX(x)                    (((uint32_t)(((uint32_t)(x)) << I2S_FIFOCFG_WAKERX_SHIFT)) & I2S_FIFOCFG_WAKERX_MASK)
#define I2S_FIFOCFG_EMPTYTX_MASK                 (0x10000U)
#define I2S_FIFOCFG_EMPTYTX_SHIFT                (16U)
#define I2S_FIFOCFG_EMPTYTX(x)                   (((uint32_t)(((uint32_t)(x)) << I2S_FIFOCFG_EMPTYTX_SHIFT)) & I2S_FIFOCFG_EMPTYTX_MASK)
#define I2S_FIFOCFG_EMPTYRX_MASK                 (0x20000U)
#define I2S_FIFOCFG_EMPTYRX_SHIFT                (17U)
#define I2S_FIFOCFG_EMPTYRX(x)                   (((uint32_t)(((uint32_t)(x)) << I2S_FIFOCFG_EMPTYRX_SHIFT)) & I2S_FIFOCFG_EMPTYRX_MASK)

/*! @name FIFOSTAT - FIFO status register. */
#define I2S_FIFOSTAT_TXERR_MASK                  (0x1U)
#define I2S_FIFOSTAT_TXERR_SHIFT                 (0U)
#define I2S_FIFOSTAT_TXERR(x)                    (((uint32_t)(((uint32_t)(x)) << I2S_FIFOSTAT_TXERR_SHIFT)) & I2S_FIFOSTAT_TXERR_MASK)
#define I2S_FIFOSTAT_RXERR_MASK                  (0x2U)
#define I2S_FIFOSTAT_RXERR_SHIFT                 (1U)
#define I2S_FIFOSTAT_RXERR(x)                    (((uint32_t)(((uint32_t)(x)) << I2S_FIFOSTAT_RXERR_SHIFT)) & I2S_FIFOSTAT_RXERR_MASK)
#define I2S_FIFOSTAT_PERINT_MASK                 (0x8U)
#define I2S_FIFOSTAT_PERINT_SHIFT                (3U)
#define I2S_FIFOSTAT_PERINT(x)                   (((uint32_t)(((uint32_t)(x)) << I2S_FIFOSTAT_PERINT_SHIFT)) & I2S_FIFOSTAT_PERINT_MASK)
#define I2S_FIFOSTAT_TXEMPTY_MASK                (0x10U)
#define I2S_FIFOSTAT_TXEMPTY_SHIFT               (4U)
#define I2S_FIFOSTAT_TXEMPTY(x)                  (((uint32_t)(((uint32_t)(x)) << I2S_FIFOSTAT_TXEMPTY_SHIFT)) & I2S_FIFOSTAT_TXEMPTY_MASK)
#define I2S_FIFOSTAT_TXNOTFULL_MASK              (0x20U)
#define I2S_FIFOSTAT_TXNOTFULL_SHIFT             (5U)
#define I2S_FIFOSTAT_TXNOTFULL(x)                (((uint32_t)(((uint32_t)(x)) << I2S_FIFOSTAT_TXNOTFULL_SHIFT)) & I2S_FIFOSTAT_TXNOTFULL_MASK)
#define I2S_FIFOSTAT_RXNOTEMPTY_MASK             (0x40U)
#define I2S_FIFOSTAT_RXNOTEMPTY_SHIFT            (6U)
#define I2S_FIFOSTAT_RXNOTEMPTY(x)               (((uint32_t)(((uint32_t)(x)) << I2S_FIFOSTAT_RXNOTEMPTY_SHIFT)) & I2S_FIFOSTAT_RXNOTEMPTY_MASK)
#define I2S_FIFOSTAT_RXFULL_MASK                 (0x80U)
#define I2S_FIFOSTAT_RXFULL_SHIFT                (7U)
#define I2S_FIFOSTAT_RXFULL(x)                   (((uint32_t)(((uint32_t)(x)) << I2S_FIFOSTAT_RXFULL_SHIFT)) & I2S_FIFOSTAT_RXFULL_MASK)
#define I2S_FIFOSTAT_TXLVL_MASK                  (0x1F00U)
#define I2S_FIFOSTAT_TXLVL_SHIFT                 (8U)
#define I2S_FIFOSTAT_TXLVL(x)                    (((uint32_t)(((uint32_t)(x)) << I2S_FIFOSTAT_TXLVL_SHIFT)) & I2S_FIFOSTAT_TXLVL_MASK)
#define I2S_FIFOSTAT_RXLVL_MASK                  (0x1F0000U)
#define I2S_FIFOSTAT_RXLVL_SHIFT                 (16U)
#define I2S_FIFOSTAT_RXLVL(x)                    (((uint32_t)(((uint32_t)(x)) << I2S_FIFOSTAT_RXLVL_SHIFT)) & I2S_FIFOSTAT_RXLVL_MASK)

/*! @name FIFOTRIG - FIFO trigger settings for interrupt and DMA request. */
#define I2S_FIFOTRIG_TXLVLENA_MASK               (0x1U)
#define I2S_FIFOTRIG_TXLVLENA_SHIFT              (0U)
#define I2S_FIFOTRIG_TXLVLENA(x)                 (((uint32_t)(((uint32_t)(x)) << I2S_FIFOTRIG_TXLVLENA_SHIFT)) & I2S_FIFOTRIG_TXLVLENA_MASK)
#define I2S_FIFOTRIG_RXLVLENA_MASK               (0x2U)
#define I2S_FIFOTRIG_RXLVLENA_SHIFT              (1U)
#define I2S_FIFOTRIG_RXLVLENA(x)                 (((uint32_t)(((uint32_t)(x)) << I2S_FIFOTRIG_RXLVLENA_SHIFT)) & I2S_FIFOTRIG_RXLVLENA_MASK)
#define I2S_FIFOTRIG_TXLVL_MASK                  (0xF00U)
#define I2S_FIFOTRIG_TXLVL_SHIFT                 (8U)
#define I2S_FIFOTRIG_TXLVL(x)                    (((uint32_t)(((uint32_t)(x)) << I2S_FIFOTRIG_TXLVL_SHIFT)) & I2S_FIFOTRIG_TXLVL_MASK)
#define I2S_FIFOTRIG_RXLVL_MASK                  (0xF0000U)
#define I2S_FIFOTRIG_RXLVL_SHIFT                 (16U)
#define I2S_FIFOTRIG_RXLVL(x)                    (((uint32_t)(((uint32_t)(x)) << I2S_FIFOTRIG_RXLVL_SHIFT)) & I2S_FIFOTRIG_RXLVL_MASK)

/*! @name FIFOINTENSET - FIFO interrupt enable set (enable) and read register. */
#define I2S_FIFOINTENSET_TXERR_MASK              (0x1U)
#define I2S_FIFOINTENSET_TXERR_SHIFT             (0U)
#define I2S_FIFOINTENSET_TXERR(x)                (((uint32_t)(((uint32_t)(x)) << I2S_FIFOINTENSET_TXERR_SHIFT)) & I2S_FIFOINTENSET_TXERR_MASK)
#define I2S_FIFOINTENSET_RXERR_MASK              (0x2U)
#define I2S_FIFOINTENSET_RXERR_SHIFT             (1U)
#define I2S_FIFOINTENSET_RXERR(x)                (((uint32_t)(((uint32_t)(x)) << I2S_FIFOINTENSET_RXERR_SHIFT)) & I2S_FIFOINTENSET_RXERR_MASK)
#define I2S_FIFOINTENSET_TXLVL_MASK              (0x4U)
#define I2S_FIFOINTENSET_TXLVL_SHIFT             (2U)
#define I2S_FIFOINTENSET_TXLVL(x)                (((uint32_t)(((uint32_t)(x)) << I2S_FIFOINTENSET_TXLVL_SHIFT)) & I2S_FIFOINTENSET_TXLVL_MASK)
#define I2S_FIFOINTENSET_RXLVL_MASK              (0x8U)
#define I2S_FIFOINTENSET_RXLVL_SHIFT             (3U)
#define I2S_FIFOINTENSET_RXLVL(x)                (((uint32_t)(((uint32_t)(x)) << I2S_FIFOINTENSET_RXLVL_SHIFT)) & I2S_FIFOINTENSET_RXLVL_MASK)

/*! @name FIFOINTENCLR - FIFO interrupt enable clear (disable) and read register. */
#define I2S_FIFOINTENCLR_TXERR_MASK              (0x1U)
#define I2S_FIFOINTENCLR_TXERR_SHIFT             (0U)
#define I2S_FIFOINTENCLR_TXERR(x)                (((uint32_t)(((uint32_t)(x)) << I2S_FIFOINTENCLR_TXERR_SHIFT)) & I2S_FIFOINTENCLR_TXERR_MASK)
#define I2S_FIFOINTENCLR_RXERR_MASK              (0x2U)
#define I2S_FIFOINTENCLR_RXERR_SHIFT             (1U)
#define I2S_FIFOINTENCLR_RXERR(x)                (((uint32_t)(((uint32_t)(x)) << I2S_FIFOINTENCLR_RXERR_SHIFT)) & I2S_FIFOINTENCLR_RXERR_MASK)
#define I2S_FIFOINTENCLR_TXLVL_MASK              (0x4U)
#define I2S_FIFOINTENCLR_TXLVL_SHIFT             (2U)
#define I2S_FIFOINTENCLR_TXLVL(x)                (((uint32_t)(((uint32_t)(x)) << I2S_FIFOINTENCLR_TXLVL_SHIFT)) & I2S_FIFOINTENCLR_TXLVL_MASK)
#define I2S_FIFOINTENCLR_RXLVL_MASK              (0x8U)
#define I2S_FIFOINTENCLR_RXLVL_SHIFT             (3U)
#define I2S_FIFOINTENCLR_RXLVL(x)                (((uint32_t)(((uint32_t)(x)) << I2S_FIFOINTENCLR_RXLVL_SHIFT)) & I2S_FIFOINTENCLR_RXLVL_MASK)

/*! @name FIFOINTSTAT - FIFO interrupt status register. */
#define I2S_FIFOINTSTAT_TXERR_MASK               (0x1U)
#define I2S_FIFOINTSTAT_TXERR_SHIFT              (0U)
#define I2S_FIFOINTSTAT_TXERR(x)                 (((uint32_t)(((uint32_t)(x)) << I2S_FIFOINTSTAT_TXERR_SHIFT)) & I2S_FIFOINTSTAT_TXERR_MASK)
#define I2S_FIFOINTSTAT_RXERR_MASK               (0x2U)
#define I2S_FIFOINTSTAT_RXERR_SHIFT              (1U)
#define I2S_FIFOINTSTAT_RXERR(x)                 (((uint32_t)(((uint32_t)(x)) << I2S_FIFOINTSTAT_RXERR_SHIFT)) & I2S_FIFOINTSTAT_RXERR_MASK)
#define I2S_FIFOINTSTAT_TXLVL_MASK               (0x4U)
#define I2S_FIFOINTSTAT_TXLVL_SHIFT              (2U)
#define I2S_FIFOINTSTAT_TXLVL(x)                 (((uint32_t)(((uint32_t)(x)) << I2S_FIFOINTSTAT_TXLVL_SHIFT)) & I2S_FIFOINTSTAT_TXLVL_MASK)
#define I2S_FIFOINTSTAT_RXLVL_MASK               (0x8U)
#define I2S_FIFOINTSTAT_RXLVL_SHIFT              (3U)
#define I2S_FIFOINTSTAT_RXLVL(x)                 (((uint32_t)(((uint32_t)(x)) << I2S_FIFOINTSTAT_RXLVL_SHIFT)) & I2S_FIFOINTSTAT_RXLVL_MASK)
#define I2S_FIFOINTSTAT_PERINT_MASK              (0x10U)
#define I2S_FIFOINTSTAT_PERINT_SHIFT             (4U)
#define I2S_FIFOINTSTAT_PERINT(x)                (((uint32_t)(((uint32_t)(x)) << I2S_FIFOINTSTAT_PERINT_SHIFT)) & I2S_FIFOINTSTAT_PERINT_MASK)

/*! @name FIFOWR - FIFO write data. */
#define I2S_FIFOWR_TXDATA_MASK                   (0xFFFFFFFFU)
#define I2S_FIFOWR_TXDATA_SHIFT                  (0U)
#define I2S_FIFOWR_TXDATA(x)                     (((uint32_t)(((uint32_t)(x)) << I2S_FIFOWR_TXDATA_SHIFT)) & I2S_FIFOWR_TXDATA_MASK)

/*! @name FIFOWR48H - FIFO write data for upper data bits. May only be used if the I2S is configured for 2x 24-bit data and not using DMA. */
#define I2S_FIFOWR48H_TXDATA_MASK                (0xFFFFFFU)
#define I2S_FIFOWR48H_TXDATA_SHIFT               (0U)
#define I2S_FIFOWR48H_TXDATA(x)                  (((uint32_t)(((uint32_t)(x)) << I2S_FIFOWR48H_TXDATA_SHIFT)) & I2S_FIFOWR48H_TXDATA_MASK)

/*! @name FIFORD - FIFO read data. */
#define I2S_FIFORD_RXDATA_MASK                   (0xFFFFFFFFU)
#define I2S_FIFORD_RXDATA_SHIFT                  (0U)
#define I2S_FIFORD_RXDATA(x)                     (((uint32_t)(((uint32_t)(x)) << I2S_FIFORD_RXDATA_SHIFT)) & I2S_FIFORD_RXDATA_MASK)

/*! @name FIFORD48H - FIFO read data for upper data bits. May only be used if the I2S is configured for 2x 24-bit data and not using DMA. */
#define I2S_FIFORD48H_RXDATA_MASK                (0xFFFFFFU)
#define I2S_FIFORD48H_RXDATA_SHIFT               (0U)
#define I2S_FIFORD48H_RXDATA(x)                  (((uint32_t)(((uint32_t)(x)) << I2S_FIFORD48H_RXDATA_SHIFT)) & I2S_FIFORD48H_RXDATA_MASK)

/*! @name FIFORDNOPOP - FIFO data read with no FIFO pop. */
#define I2S_FIFORDNOPOP_RXDATA_MASK              (0xFFFFFFFFU)
#define I2S_FIFORDNOPOP_RXDATA_SHIFT             (0U)
#define I2S_FIFORDNOPOP_RXDATA(x)                (((uint32_t)(((uint32_t)(x)) << I2S_FIFORDNOPOP_RXDATA_SHIFT)) & I2S_FIFORDNOPOP_RXDATA_MASK)

/*! @name FIFORD48HNOPOP - FIFO data read for upper data bits with no FIFO pop. May only be used if the I2S is configured for 2x 24-bit data and not using DMA. */
#define I2S_FIFORD48HNOPOP_RXDATA_MASK           (0xFFFFFFU)
#define I2S_FIFORD48HNOPOP_RXDATA_SHIFT          (0U)
#define I2S_FIFORD48HNOPOP_RXDATA(x)             (((uint32_t)(((uint32_t)(x)) << I2S_FIFORD48HNOPOP_RXDATA_SHIFT)) & I2S_FIFORD48HNOPOP_RXDATA_MASK)

/*! @name ID - I2S Module identification */
#define I2S_ID_Aperture_MASK                     (0xFFU)
#define I2S_ID_Aperture_SHIFT                    (0U)
#define I2S_ID_Aperture(x)                       (((uint32_t)(((uint32_t)(x)) << I2S_ID_Aperture_SHIFT)) & I2S_ID_Aperture_MASK)
#define I2S_ID_Minor_Rev_MASK                    (0xF00U)
#define I2S_ID_Minor_Rev_SHIFT                   (8U)
#define I2S_ID_Minor_Rev(x)                      (((uint32_t)(((uint32_t)(x)) << I2S_ID_Minor_Rev_SHIFT)) & I2S_ID_Minor_Rev_MASK)
#define I2S_ID_Major_Rev_MASK                    (0xF000U)
#define I2S_ID_Major_Rev_SHIFT                   (12U)
#define I2S_ID_Major_Rev(x)                      (((uint32_t)(((uint32_t)(x)) << I2S_ID_Major_Rev_SHIFT)) & I2S_ID_Major_Rev_MASK)
#define I2S_ID_ID_MASK                           (0xFFFF0000U)
#define I2S_ID_ID_SHIFT                          (16U)
#define I2S_ID_ID(x)                             (((uint32_t)(((uint32_t)(x)) << I2S_ID_ID_SHIFT)) & I2S_ID_ID_MASK)


/*!
 * @}
 */ /* end of group I2S_Register_Masks */


/* I2S - Peripheral instance base addresses */
/** Peripheral I2S0 base address */
#define I2S0_BASE                                (0x40097000u)
/** Peripheral I2S0 base pointer */
#define I2S0                                     ((I2S_Type *)I2S0_BASE)
/** Peripheral I2S1 base address */
#define I2S1_BASE                                (0x40098000u)
/** Peripheral I2S1 base pointer */
#define I2S1                                     ((I2S_Type *)I2S1_BASE)
/** Array initializer of I2S peripheral base addresses */
#define I2S_BASE_ADDRS                           { I2S0_BASE, I2S1_BASE }
/** Array initializer of I2S peripheral base pointers */
#define I2S_BASE_PTRS                            { I2S0, I2S1 }
/** Interrupt vectors for the I2S peripheral type */
#define I2S_IRQS                                 { FLEXCOMM6_IRQn, FLEXCOMM7_IRQn }

/*!
 * @}
 */ /* end of group I2S_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- INPUTMUX Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup INPUTMUX_Peripheral_Access_Layer INPUTMUX Peripheral Access Layer
 * @{
 */

/** INPUTMUX - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[192];
  __IO uint32_t PINTSEL[4];                        /**< Pin interrupt select register, array offset: 0xC0, array step: 0x4 */
       uint8_t RESERVED_1[16];
  __IO uint32_t DMA_ITRIG_INMUX[22];               /**< Trigger select register for DMA channel, array offset: 0xE0, array step: 0x4 */
       uint8_t RESERVED_2[40];
  __IO uint32_t DMA_OTRIG_INMUX[4];                /**< DMA output trigger selection to become DMA trigger, array offset: 0x160, array step: 0x4 */
       uint8_t RESERVED_3[16];
  __IO uint32_t FREQMEAS_REF;                      /**< Selection for frequency measurement reference clock, offset: 0x180 */
  __IO uint32_t FREQMEAS_TARGET;                   /**< Selection for frequency measurement target clock, offset: 0x184 */
} INPUTMUX_Type;

/* ----------------------------------------------------------------------------
   -- INPUTMUX Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup INPUTMUX_Register_Masks INPUTMUX Register Masks
 * @{
 */

/*! @name PINTSEL - Pin interrupt select register */
#define INPUTMUX_PINTSEL_INTPIN_MASK             (0xFFU)
#define INPUTMUX_PINTSEL_INTPIN_SHIFT            (0U)
#define INPUTMUX_PINTSEL_INTPIN(x)               (((uint32_t)(((uint32_t)(x)) << INPUTMUX_PINTSEL_INTPIN_SHIFT)) & INPUTMUX_PINTSEL_INTPIN_MASK)

/* The count of INPUTMUX_PINTSEL */
#define INPUTMUX_PINTSEL_COUNT                   (4U)

/*! @name DMA_ITRIG_INMUX - Trigger select register for DMA channel */
#define INPUTMUX_DMA_ITRIG_INMUX_INP_MASK        (0x1FU)
#define INPUTMUX_DMA_ITRIG_INMUX_INP_SHIFT       (0U)
#define INPUTMUX_DMA_ITRIG_INMUX_INP(x)          (((uint32_t)(((uint32_t)(x)) << INPUTMUX_DMA_ITRIG_INMUX_INP_SHIFT)) & INPUTMUX_DMA_ITRIG_INMUX_INP_MASK)

/* The count of INPUTMUX_DMA_ITRIG_INMUX */
#define INPUTMUX_DMA_ITRIG_INMUX_COUNT           (22U)

/*! @name DMA_OTRIG_INMUX - DMA output trigger selection to become DMA trigger */
#define INPUTMUX_DMA_OTRIG_INMUX_INP_MASK        (0x1FU)
#define INPUTMUX_DMA_OTRIG_INMUX_INP_SHIFT       (0U)
#define INPUTMUX_DMA_OTRIG_INMUX_INP(x)          (((uint32_t)(((uint32_t)(x)) << INPUTMUX_DMA_OTRIG_INMUX_INP_SHIFT)) & INPUTMUX_DMA_OTRIG_INMUX_INP_MASK)

/* The count of INPUTMUX_DMA_OTRIG_INMUX */
#define INPUTMUX_DMA_OTRIG_INMUX_COUNT           (4U)

/*! @name FREQMEAS_REF - Selection for frequency measurement reference clock */
#define INPUTMUX_FREQMEAS_REF_CLKIN_MASK         (0x1FU)
#define INPUTMUX_FREQMEAS_REF_CLKIN_SHIFT        (0U)
#define INPUTMUX_FREQMEAS_REF_CLKIN(x)           (((uint32_t)(((uint32_t)(x)) << INPUTMUX_FREQMEAS_REF_CLKIN_SHIFT)) & INPUTMUX_FREQMEAS_REF_CLKIN_MASK)

/*! @name FREQMEAS_TARGET - Selection for frequency measurement target clock */
#define INPUTMUX_FREQMEAS_TARGET_CLKIN_MASK      (0x1FU)
#define INPUTMUX_FREQMEAS_TARGET_CLKIN_SHIFT     (0U)
#define INPUTMUX_FREQMEAS_TARGET_CLKIN(x)        (((uint32_t)(((uint32_t)(x)) << INPUTMUX_FREQMEAS_TARGET_CLKIN_SHIFT)) & INPUTMUX_FREQMEAS_TARGET_CLKIN_MASK)


/*!
 * @}
 */ /* end of group INPUTMUX_Register_Masks */


/* INPUTMUX - Peripheral instance base addresses */
/** Peripheral INPUTMUX base address */
#define INPUTMUX_BASE                            (0x40005000u)
/** Peripheral INPUTMUX base pointer */
#define INPUTMUX                                 ((INPUTMUX_Type *)INPUTMUX_BASE)
/** Array initializer of INPUTMUX peripheral base addresses */
#define INPUTMUX_BASE_ADDRS                      { INPUTMUX_BASE }
/** Array initializer of INPUTMUX peripheral base pointers */
#define INPUTMUX_BASE_PTRS                       { INPUTMUX }

/*!
 * @}
 */ /* end of group INPUTMUX_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- IOCON Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup IOCON_Peripheral_Access_Layer IOCON Peripheral Access Layer
 * @{
 */

/** IOCON - Register Layout Typedef */
typedef struct {
  __IO uint32_t PIO[2][32];                        /**< Digital I/O control for port 0 pins PIO0_0..Digital I/O control for port 1 pins PIO1_31, array offset: 0x0, array step: index*0x80, index2*0x4 */
} IOCON_Type;

/* ----------------------------------------------------------------------------
   -- IOCON Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup IOCON_Register_Masks IOCON Register Masks
 * @{
 */

/*! @name PIO - Digital I/O control for port 0 pins PIO0_0..Digital I/O control for port 1 pins PIO1_31 */
#define IOCON_PIO_FUNC_MASK                      (0x7U)
#define IOCON_PIO_FUNC_SHIFT                     (0U)
#define IOCON_PIO_FUNC(x)                        (((uint32_t)(((uint32_t)(x)) << IOCON_PIO_FUNC_SHIFT)) & IOCON_PIO_FUNC_MASK)
#define IOCON_PIO_MODE_MASK                      (0x18U)
#define IOCON_PIO_MODE_SHIFT                     (3U)
#define IOCON_PIO_MODE(x)                        (((uint32_t)(((uint32_t)(x)) << IOCON_PIO_MODE_SHIFT)) & IOCON_PIO_MODE_MASK)
#define IOCON_PIO_I2CSLEW_MASK                   (0x20U)
#define IOCON_PIO_I2CSLEW_SHIFT                  (5U)
#define IOCON_PIO_I2CSLEW(x)                     (((uint32_t)(((uint32_t)(x)) << IOCON_PIO_I2CSLEW_SHIFT)) & IOCON_PIO_I2CSLEW_MASK)
#define IOCON_PIO_INVERT_MASK                    (0x40U)
#define IOCON_PIO_INVERT_SHIFT                   (6U)
#define IOCON_PIO_INVERT(x)                      (((uint32_t)(((uint32_t)(x)) << IOCON_PIO_INVERT_SHIFT)) & IOCON_PIO_INVERT_MASK)
#define IOCON_PIO_DIGIMODE_MASK                  (0x80U)
#define IOCON_PIO_DIGIMODE_SHIFT                 (7U)
#define IOCON_PIO_DIGIMODE(x)                    (((uint32_t)(((uint32_t)(x)) << IOCON_PIO_DIGIMODE_SHIFT)) & IOCON_PIO_DIGIMODE_MASK)
#define IOCON_PIO_FILTEROFF_MASK                 (0x100U)
#define IOCON_PIO_FILTEROFF_SHIFT                (8U)
#define IOCON_PIO_FILTEROFF(x)                   (((uint32_t)(((uint32_t)(x)) << IOCON_PIO_FILTEROFF_SHIFT)) & IOCON_PIO_FILTEROFF_MASK)
#define IOCON_PIO_I2CDRIVE_MASK                  (0x200U)
#define IOCON_PIO_I2CDRIVE_SHIFT                 (9U)
#define IOCON_PIO_I2CDRIVE(x)                    (((uint32_t)(((uint32_t)(x)) << IOCON_PIO_I2CDRIVE_SHIFT)) & IOCON_PIO_I2CDRIVE_MASK)
#define IOCON_PIO_SLEW_MASK                      (0x200U)
#define IOCON_PIO_SLEW_SHIFT                     (9U)
#define IOCON_PIO_SLEW(x)                        (((uint32_t)(((uint32_t)(x)) << IOCON_PIO_SLEW_SHIFT)) & IOCON_PIO_SLEW_MASK)
#define IOCON_PIO_OD_MASK                        (0x400U)
#define IOCON_PIO_OD_SHIFT                       (10U)
#define IOCON_PIO_OD(x)                          (((uint32_t)(((uint32_t)(x)) << IOCON_PIO_OD_SHIFT)) & IOCON_PIO_OD_MASK)
#define IOCON_PIO_I2CFILTER_MASK                 (0x400U)
#define IOCON_PIO_I2CFILTER_SHIFT                (10U)
#define IOCON_PIO_I2CFILTER(x)                   (((uint32_t)(((uint32_t)(x)) << IOCON_PIO_I2CFILTER_SHIFT)) & IOCON_PIO_I2CFILTER_MASK)

/* The count of IOCON_PIO */
#define IOCON_PIO_COUNT                          (2U)

/* The count of IOCON_PIO */
#define IOCON_PIO_COUNT2                         (32U)


/*!
 * @}
 */ /* end of group IOCON_Register_Masks */


/* IOCON - Peripheral instance base addresses */
/** Peripheral IOCON base address */
#define IOCON_BASE                               (0x40001000u)
/** Peripheral IOCON base pointer */
#define IOCON                                    ((IOCON_Type *)IOCON_BASE)
/** Array initializer of IOCON peripheral base addresses */
#define IOCON_BASE_ADDRS                         { IOCON_BASE }
/** Array initializer of IOCON peripheral base pointers */
#define IOCON_BASE_PTRS                          { IOCON }

/*!
 * @}
 */ /* end of group IOCON_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MRT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MRT_Peripheral_Access_Layer MRT Peripheral Access Layer
 * @{
 */

/** MRT - Register Layout Typedef */
typedef struct {
  struct {                                         /* offset: 0x0, array step: 0x10 */
    __IO uint32_t INTVAL;                            /**< MRT Time interval value register. This value is loaded into the TIMER register., array offset: 0x0, array step: 0x10 */
    __I  uint32_t TIMER;                             /**< MRT Timer register. This register reads the value of the down-counter., array offset: 0x4, array step: 0x10 */
    __IO uint32_t CTRL;                              /**< MRT Control register. This register controls the MRT modes., array offset: 0x8, array step: 0x10 */
    __IO uint32_t STAT;                              /**< MRT Status register., array offset: 0xC, array step: 0x10 */
  } CHANNEL[4];
       uint8_t RESERVED_0[176];
  __IO uint32_t MODCFG;                            /**< Module Configuration register. This register provides information about this particular MRT instance, and allows choosing an overall mode for the idle channel feature., offset: 0xF0 */
  __I  uint32_t IDLE_CH;                           /**< Idle channel register. This register returns the number of the first idle channel., offset: 0xF4 */
  __IO uint32_t IRQ_FLAG;                          /**< Global interrupt flag register, offset: 0xF8 */
} MRT_Type;

/* ----------------------------------------------------------------------------
   -- MRT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MRT_Register_Masks MRT Register Masks
 * @{
 */

/*! @name CHANNEL_INTVAL - MRT Time interval value register. This value is loaded into the TIMER register. */
#define MRT_CHANNEL_INTVAL_IVALUE_MASK           (0xFFFFFFU)
#define MRT_CHANNEL_INTVAL_IVALUE_SHIFT          (0U)
#define MRT_CHANNEL_INTVAL_IVALUE(x)             (((uint32_t)(((uint32_t)(x)) << MRT_CHANNEL_INTVAL_IVALUE_SHIFT)) & MRT_CHANNEL_INTVAL_IVALUE_MASK)
#define MRT_CHANNEL_INTVAL_LOAD_MASK             (0x80000000U)
#define MRT_CHANNEL_INTVAL_LOAD_SHIFT            (31U)
#define MRT_CHANNEL_INTVAL_LOAD(x)               (((uint32_t)(((uint32_t)(x)) << MRT_CHANNEL_INTVAL_LOAD_SHIFT)) & MRT_CHANNEL_INTVAL_LOAD_MASK)

/* The count of MRT_CHANNEL_INTVAL */
#define MRT_CHANNEL_INTVAL_COUNT                 (4U)

/*! @name CHANNEL_TIMER - MRT Timer register. This register reads the value of the down-counter. */
#define MRT_CHANNEL_TIMER_VALUE_MASK             (0xFFFFFFU)
#define MRT_CHANNEL_TIMER_VALUE_SHIFT            (0U)
#define MRT_CHANNEL_TIMER_VALUE(x)               (((uint32_t)(((uint32_t)(x)) << MRT_CHANNEL_TIMER_VALUE_SHIFT)) & MRT_CHANNEL_TIMER_VALUE_MASK)

/* The count of MRT_CHANNEL_TIMER */
#define MRT_CHANNEL_TIMER_COUNT                  (4U)

/*! @name CHANNEL_CTRL - MRT Control register. This register controls the MRT modes. */
#define MRT_CHANNEL_CTRL_INTEN_MASK              (0x1U)
#define MRT_CHANNEL_CTRL_INTEN_SHIFT             (0U)
#define MRT_CHANNEL_CTRL_INTEN(x)                (((uint32_t)(((uint32_t)(x)) << MRT_CHANNEL_CTRL_INTEN_SHIFT)) & MRT_CHANNEL_CTRL_INTEN_MASK)
#define MRT_CHANNEL_CTRL_MODE_MASK               (0x6U)
#define MRT_CHANNEL_CTRL_MODE_SHIFT              (1U)
#define MRT_CHANNEL_CTRL_MODE(x)                 (((uint32_t)(((uint32_t)(x)) << MRT_CHANNEL_CTRL_MODE_SHIFT)) & MRT_CHANNEL_CTRL_MODE_MASK)

/* The count of MRT_CHANNEL_CTRL */
#define MRT_CHANNEL_CTRL_COUNT                   (4U)

/*! @name CHANNEL_STAT - MRT Status register. */
#define MRT_CHANNEL_STAT_INTFLAG_MASK            (0x1U)
#define MRT_CHANNEL_STAT_INTFLAG_SHIFT           (0U)
#define MRT_CHANNEL_STAT_INTFLAG(x)              (((uint32_t)(((uint32_t)(x)) << MRT_CHANNEL_STAT_INTFLAG_SHIFT)) & MRT_CHANNEL_STAT_INTFLAG_MASK)
#define MRT_CHANNEL_STAT_RUN_MASK                (0x2U)
#define MRT_CHANNEL_STAT_RUN_SHIFT               (1U)
#define MRT_CHANNEL_STAT_RUN(x)                  (((uint32_t)(((uint32_t)(x)) << MRT_CHANNEL_STAT_RUN_SHIFT)) & MRT_CHANNEL_STAT_RUN_MASK)
#define MRT_CHANNEL_STAT_INUSE_MASK              (0x4U)
#define MRT_CHANNEL_STAT_INUSE_SHIFT             (2U)
#define MRT_CHANNEL_STAT_INUSE(x)                (((uint32_t)(((uint32_t)(x)) << MRT_CHANNEL_STAT_INUSE_SHIFT)) & MRT_CHANNEL_STAT_INUSE_MASK)

/* The count of MRT_CHANNEL_STAT */
#define MRT_CHANNEL_STAT_COUNT                   (4U)

/*! @name MODCFG - Module Configuration register. This register provides information about this particular MRT instance, and allows choosing an overall mode for the idle channel feature. */
#define MRT_MODCFG_NOC_MASK                      (0xFU)
#define MRT_MODCFG_NOC_SHIFT                     (0U)
#define MRT_MODCFG_NOC(x)                        (((uint32_t)(((uint32_t)(x)) << MRT_MODCFG_NOC_SHIFT)) & MRT_MODCFG_NOC_MASK)
#define MRT_MODCFG_NOB_MASK                      (0x1F0U)
#define MRT_MODCFG_NOB_SHIFT                     (4U)
#define MRT_MODCFG_NOB(x)                        (((uint32_t)(((uint32_t)(x)) << MRT_MODCFG_NOB_SHIFT)) & MRT_MODCFG_NOB_MASK)
#define MRT_MODCFG_MULTITASK_MASK                (0x80000000U)
#define MRT_MODCFG_MULTITASK_SHIFT               (31U)
#define MRT_MODCFG_MULTITASK(x)                  (((uint32_t)(((uint32_t)(x)) << MRT_MODCFG_MULTITASK_SHIFT)) & MRT_MODCFG_MULTITASK_MASK)

/*! @name IDLE_CH - Idle channel register. This register returns the number of the first idle channel. */
#define MRT_IDLE_CH_CHAN_MASK                    (0xF0U)
#define MRT_IDLE_CH_CHAN_SHIFT                   (4U)
#define MRT_IDLE_CH_CHAN(x)                      (((uint32_t)(((uint32_t)(x)) << MRT_IDLE_CH_CHAN_SHIFT)) & MRT_IDLE_CH_CHAN_MASK)

/*! @name IRQ_FLAG - Global interrupt flag register */
#define MRT_IRQ_FLAG_GFLAG0_MASK                 (0x1U)
#define MRT_IRQ_FLAG_GFLAG0_SHIFT                (0U)
#define MRT_IRQ_FLAG_GFLAG0(x)                   (((uint32_t)(((uint32_t)(x)) << MRT_IRQ_FLAG_GFLAG0_SHIFT)) & MRT_IRQ_FLAG_GFLAG0_MASK)
#define MRT_IRQ_FLAG_GFLAG1_MASK                 (0x2U)
#define MRT_IRQ_FLAG_GFLAG1_SHIFT                (1U)
#define MRT_IRQ_FLAG_GFLAG1(x)                   (((uint32_t)(((uint32_t)(x)) << MRT_IRQ_FLAG_GFLAG1_SHIFT)) & MRT_IRQ_FLAG_GFLAG1_MASK)
#define MRT_IRQ_FLAG_GFLAG2_MASK                 (0x4U)
#define MRT_IRQ_FLAG_GFLAG2_SHIFT                (2U)
#define MRT_IRQ_FLAG_GFLAG2(x)                   (((uint32_t)(((uint32_t)(x)) << MRT_IRQ_FLAG_GFLAG2_SHIFT)) & MRT_IRQ_FLAG_GFLAG2_MASK)
#define MRT_IRQ_FLAG_GFLAG3_MASK                 (0x8U)
#define MRT_IRQ_FLAG_GFLAG3_SHIFT                (3U)
#define MRT_IRQ_FLAG_GFLAG3(x)                   (((uint32_t)(((uint32_t)(x)) << MRT_IRQ_FLAG_GFLAG3_SHIFT)) & MRT_IRQ_FLAG_GFLAG3_MASK)


/*!
 * @}
 */ /* end of group MRT_Register_Masks */


/* MRT - Peripheral instance base addresses */
/** Peripheral MRT0 base address */
#define MRT0_BASE                                (0x4000D000u)
/** Peripheral MRT0 base pointer */
#define MRT0                                     ((MRT_Type *)MRT0_BASE)
/** Array initializer of MRT peripheral base addresses */
#define MRT_BASE_ADDRS                           { MRT0_BASE }
/** Array initializer of MRT peripheral base pointers */
#define MRT_BASE_PTRS                            { MRT0 }
/** Interrupt vectors for the MRT peripheral type */
#define MRT_IRQS                                 { MRT0_IRQn }

/*!
 * @}
 */ /* end of group MRT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PINT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PINT_Peripheral_Access_Layer PINT Peripheral Access Layer
 * @{
 */

/** PINT - Register Layout Typedef */
typedef struct {
  __IO uint32_t ISEL;                              /**< Pin Interrupt Mode register, offset: 0x0 */
  __IO uint32_t IENR;                              /**< Pin interrupt level or rising edge interrupt enable register, offset: 0x4 */
  __O  uint32_t SIENR;                             /**< Pin interrupt level or rising edge interrupt set register, offset: 0x8 */
  __O  uint32_t CIENR;                             /**< Pin interrupt level (rising edge interrupt) clear register, offset: 0xC */
  __IO uint32_t IENF;                              /**< Pin interrupt active level or falling edge interrupt enable register, offset: 0x10 */
  __O  uint32_t SIENF;                             /**< Pin interrupt active level or falling edge interrupt set register, offset: 0x14 */
  __O  uint32_t CIENF;                             /**< Pin interrupt active level or falling edge interrupt clear register, offset: 0x18 */
  __IO uint32_t RISE;                              /**< Pin interrupt rising edge register, offset: 0x1C */
  __IO uint32_t FALL;                              /**< Pin interrupt falling edge register, offset: 0x20 */
  __IO uint32_t IST;                               /**< Pin interrupt status register, offset: 0x24 */
  __IO uint32_t PMCTRL;                            /**< Pattern match interrupt control register, offset: 0x28 */
  __IO uint32_t PMSRC;                             /**< Pattern match interrupt bit-slice source register, offset: 0x2C */
  __IO uint32_t PMCFG;                             /**< Pattern match interrupt bit slice configuration register, offset: 0x30 */
} PINT_Type;

/* ----------------------------------------------------------------------------
   -- PINT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PINT_Register_Masks PINT Register Masks
 * @{
 */

/*! @name ISEL - Pin Interrupt Mode register */
#define PINT_ISEL_PMODE_MASK                     (0xFFU)
#define PINT_ISEL_PMODE_SHIFT                    (0U)
#define PINT_ISEL_PMODE(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_ISEL_PMODE_SHIFT)) & PINT_ISEL_PMODE_MASK)

/*! @name IENR - Pin interrupt level or rising edge interrupt enable register */
#define PINT_IENR_ENRL_MASK                      (0xFFU)
#define PINT_IENR_ENRL_SHIFT                     (0U)
#define PINT_IENR_ENRL(x)                        (((uint32_t)(((uint32_t)(x)) << PINT_IENR_ENRL_SHIFT)) & PINT_IENR_ENRL_MASK)

/*! @name SIENR - Pin interrupt level or rising edge interrupt set register */
#define PINT_SIENR_SETENRL_MASK                  (0xFFU)
#define PINT_SIENR_SETENRL_SHIFT                 (0U)
#define PINT_SIENR_SETENRL(x)                    (((uint32_t)(((uint32_t)(x)) << PINT_SIENR_SETENRL_SHIFT)) & PINT_SIENR_SETENRL_MASK)

/*! @name CIENR - Pin interrupt level (rising edge interrupt) clear register */
#define PINT_CIENR_CENRL_MASK                    (0xFFU)
#define PINT_CIENR_CENRL_SHIFT                   (0U)
#define PINT_CIENR_CENRL(x)                      (((uint32_t)(((uint32_t)(x)) << PINT_CIENR_CENRL_SHIFT)) & PINT_CIENR_CENRL_MASK)

/*! @name IENF - Pin interrupt active level or falling edge interrupt enable register */
#define PINT_IENF_ENAF_MASK                      (0xFFU)
#define PINT_IENF_ENAF_SHIFT                     (0U)
#define PINT_IENF_ENAF(x)                        (((uint32_t)(((uint32_t)(x)) << PINT_IENF_ENAF_SHIFT)) & PINT_IENF_ENAF_MASK)

/*! @name SIENF - Pin interrupt active level or falling edge interrupt set register */
#define PINT_SIENF_SETENAF_MASK                  (0xFFU)
#define PINT_SIENF_SETENAF_SHIFT                 (0U)
#define PINT_SIENF_SETENAF(x)                    (((uint32_t)(((uint32_t)(x)) << PINT_SIENF_SETENAF_SHIFT)) & PINT_SIENF_SETENAF_MASK)

/*! @name CIENF - Pin interrupt active level or falling edge interrupt clear register */
#define PINT_CIENF_CENAF_MASK                    (0xFFU)
#define PINT_CIENF_CENAF_SHIFT                   (0U)
#define PINT_CIENF_CENAF(x)                      (((uint32_t)(((uint32_t)(x)) << PINT_CIENF_CENAF_SHIFT)) & PINT_CIENF_CENAF_MASK)

/*! @name RISE - Pin interrupt rising edge register */
#define PINT_RISE_RDET_MASK                      (0xFFU)
#define PINT_RISE_RDET_SHIFT                     (0U)
#define PINT_RISE_RDET(x)                        (((uint32_t)(((uint32_t)(x)) << PINT_RISE_RDET_SHIFT)) & PINT_RISE_RDET_MASK)

/*! @name FALL - Pin interrupt falling edge register */
#define PINT_FALL_FDET_MASK                      (0xFFU)
#define PINT_FALL_FDET_SHIFT                     (0U)
#define PINT_FALL_FDET(x)                        (((uint32_t)(((uint32_t)(x)) << PINT_FALL_FDET_SHIFT)) & PINT_FALL_FDET_MASK)

/*! @name IST - Pin interrupt status register */
#define PINT_IST_PSTAT_MASK                      (0xFFU)
#define PINT_IST_PSTAT_SHIFT                     (0U)
#define PINT_IST_PSTAT(x)                        (((uint32_t)(((uint32_t)(x)) << PINT_IST_PSTAT_SHIFT)) & PINT_IST_PSTAT_MASK)

/*! @name PMCTRL - Pattern match interrupt control register */
#define PINT_PMCTRL_SEL_PMATCH_MASK              (0x1U)
#define PINT_PMCTRL_SEL_PMATCH_SHIFT             (0U)
#define PINT_PMCTRL_SEL_PMATCH(x)                (((uint32_t)(((uint32_t)(x)) << PINT_PMCTRL_SEL_PMATCH_SHIFT)) & PINT_PMCTRL_SEL_PMATCH_MASK)
#define PINT_PMCTRL_ENA_RXEV_MASK                (0x2U)
#define PINT_PMCTRL_ENA_RXEV_SHIFT               (1U)
#define PINT_PMCTRL_ENA_RXEV(x)                  (((uint32_t)(((uint32_t)(x)) << PINT_PMCTRL_ENA_RXEV_SHIFT)) & PINT_PMCTRL_ENA_RXEV_MASK)
#define PINT_PMCTRL_PMAT_MASK                    (0xFF000000U)
#define PINT_PMCTRL_PMAT_SHIFT                   (24U)
#define PINT_PMCTRL_PMAT(x)                      (((uint32_t)(((uint32_t)(x)) << PINT_PMCTRL_PMAT_SHIFT)) & PINT_PMCTRL_PMAT_MASK)

/*! @name PMSRC - Pattern match interrupt bit-slice source register */
#define PINT_PMSRC_SRC0_MASK                     (0x700U)
#define PINT_PMSRC_SRC0_SHIFT                    (8U)
#define PINT_PMSRC_SRC0(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_PMSRC_SRC0_SHIFT)) & PINT_PMSRC_SRC0_MASK)
#define PINT_PMSRC_SRC1_MASK                     (0x3800U)
#define PINT_PMSRC_SRC1_SHIFT                    (11U)
#define PINT_PMSRC_SRC1(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_PMSRC_SRC1_SHIFT)) & PINT_PMSRC_SRC1_MASK)
#define PINT_PMSRC_SRC2_MASK                     (0x1C000U)
#define PINT_PMSRC_SRC2_SHIFT                    (14U)
#define PINT_PMSRC_SRC2(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_PMSRC_SRC2_SHIFT)) & PINT_PMSRC_SRC2_MASK)
#define PINT_PMSRC_SRC3_MASK                     (0xE0000U)
#define PINT_PMSRC_SRC3_SHIFT                    (17U)
#define PINT_PMSRC_SRC3(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_PMSRC_SRC3_SHIFT)) & PINT_PMSRC_SRC3_MASK)
#define PINT_PMSRC_SRC4_MASK                     (0x700000U)
#define PINT_PMSRC_SRC4_SHIFT                    (20U)
#define PINT_PMSRC_SRC4(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_PMSRC_SRC4_SHIFT)) & PINT_PMSRC_SRC4_MASK)
#define PINT_PMSRC_SRC5_MASK                     (0x3800000U)
#define PINT_PMSRC_SRC5_SHIFT                    (23U)
#define PINT_PMSRC_SRC5(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_PMSRC_SRC5_SHIFT)) & PINT_PMSRC_SRC5_MASK)
#define PINT_PMSRC_SRC6_MASK                     (0x1C000000U)
#define PINT_PMSRC_SRC6_SHIFT                    (26U)
#define PINT_PMSRC_SRC6(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_PMSRC_SRC6_SHIFT)) & PINT_PMSRC_SRC6_MASK)
#define PINT_PMSRC_SRC7_MASK                     (0xE0000000U)
#define PINT_PMSRC_SRC7_SHIFT                    (29U)
#define PINT_PMSRC_SRC7(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_PMSRC_SRC7_SHIFT)) & PINT_PMSRC_SRC7_MASK)

/*! @name PMCFG - Pattern match interrupt bit slice configuration register */
#define PINT_PMCFG_PROD_ENDPTS0_MASK             (0x1U)
#define PINT_PMCFG_PROD_ENDPTS0_SHIFT            (0U)
#define PINT_PMCFG_PROD_ENDPTS0(x)               (((uint32_t)(((uint32_t)(x)) << PINT_PMCFG_PROD_ENDPTS0_SHIFT)) & PINT_PMCFG_PROD_ENDPTS0_MASK)
#define PINT_PMCFG_PROD_ENDPTS1_MASK             (0x2U)
#define PINT_PMCFG_PROD_ENDPTS1_SHIFT            (1U)
#define PINT_PMCFG_PROD_ENDPTS1(x)               (((uint32_t)(((uint32_t)(x)) << PINT_PMCFG_PROD_ENDPTS1_SHIFT)) & PINT_PMCFG_PROD_ENDPTS1_MASK)
#define PINT_PMCFG_PROD_ENDPTS2_MASK             (0x4U)
#define PINT_PMCFG_PROD_ENDPTS2_SHIFT            (2U)
#define PINT_PMCFG_PROD_ENDPTS2(x)               (((uint32_t)(((uint32_t)(x)) << PINT_PMCFG_PROD_ENDPTS2_SHIFT)) & PINT_PMCFG_PROD_ENDPTS2_MASK)
#define PINT_PMCFG_PROD_ENDPTS3_MASK             (0x8U)
#define PINT_PMCFG_PROD_ENDPTS3_SHIFT            (3U)
#define PINT_PMCFG_PROD_ENDPTS3(x)               (((uint32_t)(((uint32_t)(x)) << PINT_PMCFG_PROD_ENDPTS3_SHIFT)) & PINT_PMCFG_PROD_ENDPTS3_MASK)
#define PINT_PMCFG_PROD_ENDPTS4_MASK             (0x10U)
#define PINT_PMCFG_PROD_ENDPTS4_SHIFT            (4U)
#define PINT_PMCFG_PROD_ENDPTS4(x)               (((uint32_t)(((uint32_t)(x)) << PINT_PMCFG_PROD_ENDPTS4_SHIFT)) & PINT_PMCFG_PROD_ENDPTS4_MASK)
#define PINT_PMCFG_PROD_ENDPTS5_MASK             (0x20U)
#define PINT_PMCFG_PROD_ENDPTS5_SHIFT            (5U)
#define PINT_PMCFG_PROD_ENDPTS5(x)               (((uint32_t)(((uint32_t)(x)) << PINT_PMCFG_PROD_ENDPTS5_SHIFT)) & PINT_PMCFG_PROD_ENDPTS5_MASK)
#define PINT_PMCFG_PROD_ENDPTS6_MASK             (0x40U)
#define PINT_PMCFG_PROD_ENDPTS6_SHIFT            (6U)
#define PINT_PMCFG_PROD_ENDPTS6(x)               (((uint32_t)(((uint32_t)(x)) << PINT_PMCFG_PROD_ENDPTS6_SHIFT)) & PINT_PMCFG_PROD_ENDPTS6_MASK)
#define PINT_PMCFG_CFG0_MASK                     (0x700U)
#define PINT_PMCFG_CFG0_SHIFT                    (8U)
#define PINT_PMCFG_CFG0(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_PMCFG_CFG0_SHIFT)) & PINT_PMCFG_CFG0_MASK)
#define PINT_PMCFG_CFG1_MASK                     (0x3800U)
#define PINT_PMCFG_CFG1_SHIFT                    (11U)
#define PINT_PMCFG_CFG1(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_PMCFG_CFG1_SHIFT)) & PINT_PMCFG_CFG1_MASK)
#define PINT_PMCFG_CFG2_MASK                     (0x1C000U)
#define PINT_PMCFG_CFG2_SHIFT                    (14U)
#define PINT_PMCFG_CFG2(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_PMCFG_CFG2_SHIFT)) & PINT_PMCFG_CFG2_MASK)
#define PINT_PMCFG_CFG3_MASK                     (0xE0000U)
#define PINT_PMCFG_CFG3_SHIFT                    (17U)
#define PINT_PMCFG_CFG3(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_PMCFG_CFG3_SHIFT)) & PINT_PMCFG_CFG3_MASK)
#define PINT_PMCFG_CFG4_MASK                     (0x700000U)
#define PINT_PMCFG_CFG4_SHIFT                    (20U)
#define PINT_PMCFG_CFG4(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_PMCFG_CFG4_SHIFT)) & PINT_PMCFG_CFG4_MASK)
#define PINT_PMCFG_CFG5_MASK                     (0x3800000U)
#define PINT_PMCFG_CFG5_SHIFT                    (23U)
#define PINT_PMCFG_CFG5(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_PMCFG_CFG5_SHIFT)) & PINT_PMCFG_CFG5_MASK)
#define PINT_PMCFG_CFG6_MASK                     (0x1C000000U)
#define PINT_PMCFG_CFG6_SHIFT                    (26U)
#define PINT_PMCFG_CFG6(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_PMCFG_CFG6_SHIFT)) & PINT_PMCFG_CFG6_MASK)
#define PINT_PMCFG_CFG7_MASK                     (0xE0000000U)
#define PINT_PMCFG_CFG7_SHIFT                    (29U)
#define PINT_PMCFG_CFG7(x)                       (((uint32_t)(((uint32_t)(x)) << PINT_PMCFG_CFG7_SHIFT)) & PINT_PMCFG_CFG7_MASK)


/*!
 * @}
 */ /* end of group PINT_Register_Masks */


/* PINT - Peripheral instance base addresses */
/** Peripheral PINT base address */
#define PINT_BASE                                (0x40004000u)
/** Peripheral PINT base pointer */
#define PINT                                     ((PINT_Type *)PINT_BASE)
/** Array initializer of PINT peripheral base addresses */
#define PINT_BASE_ADDRS                          { PINT_BASE }
/** Array initializer of PINT peripheral base pointers */
#define PINT_BASE_PTRS                           { PINT }
/** Interrupt vectors for the PINT peripheral type */
#define PINT_IRQS                                { PIN_INT0_IRQn, PIN_INT1_IRQn, PIN_INT2_IRQn, PIN_INT3_IRQn }

/*!
 * @}
 */ /* end of group PINT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- RTC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RTC_Peripheral_Access_Layer RTC Peripheral Access Layer
 * @{
 */

/** RTC - Register Layout Typedef */
typedef struct {
  __IO uint32_t CTRL;                              /**< RTC control register, offset: 0x0 */
  __IO uint32_t MATCH;                             /**< RTC match register, offset: 0x4 */
  __IO uint32_t COUNT;                             /**< RTC counter register, offset: 0x8 */
  __IO uint32_t WAKE;                              /**< High-resolution/wake-up timer control register, offset: 0xC */
} RTC_Type;

/* ----------------------------------------------------------------------------
   -- RTC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RTC_Register_Masks RTC Register Masks
 * @{
 */

/*! @name CTRL - RTC control register */
#define RTC_CTRL_SWRESET_MASK                    (0x1U)
#define RTC_CTRL_SWRESET_SHIFT                   (0U)
#define RTC_CTRL_SWRESET(x)                      (((uint32_t)(((uint32_t)(x)) << RTC_CTRL_SWRESET_SHIFT)) & RTC_CTRL_SWRESET_MASK)
#define RTC_CTRL_ALARM1HZ_MASK                   (0x4U)
#define RTC_CTRL_ALARM1HZ_SHIFT                  (2U)
#define RTC_CTRL_ALARM1HZ(x)                     (((uint32_t)(((uint32_t)(x)) << RTC_CTRL_ALARM1HZ_SHIFT)) & RTC_CTRL_ALARM1HZ_MASK)
#define RTC_CTRL_WAKE1KHZ_MASK                   (0x8U)
#define RTC_CTRL_WAKE1KHZ_SHIFT                  (3U)
#define RTC_CTRL_WAKE1KHZ(x)                     (((uint32_t)(((uint32_t)(x)) << RTC_CTRL_WAKE1KHZ_SHIFT)) & RTC_CTRL_WAKE1KHZ_MASK)
#define RTC_CTRL_ALARMDPD_EN_MASK                (0x10U)
#define RTC_CTRL_ALARMDPD_EN_SHIFT               (4U)
#define RTC_CTRL_ALARMDPD_EN(x)                  (((uint32_t)(((uint32_t)(x)) << RTC_CTRL_ALARMDPD_EN_SHIFT)) & RTC_CTRL_ALARMDPD_EN_MASK)
#define RTC_CTRL_WAKEDPD_EN_MASK                 (0x20U)
#define RTC_CTRL_WAKEDPD_EN_SHIFT                (5U)
#define RTC_CTRL_WAKEDPD_EN(x)                   (((uint32_t)(((uint32_t)(x)) << RTC_CTRL_WAKEDPD_EN_SHIFT)) & RTC_CTRL_WAKEDPD_EN_MASK)
#define RTC_CTRL_RTC1KHZ_EN_MASK                 (0x40U)
#define RTC_CTRL_RTC1KHZ_EN_SHIFT                (6U)
#define RTC_CTRL_RTC1KHZ_EN(x)                   (((uint32_t)(((uint32_t)(x)) << RTC_CTRL_RTC1KHZ_EN_SHIFT)) & RTC_CTRL_RTC1KHZ_EN_MASK)
#define RTC_CTRL_RTC_EN_MASK                     (0x80U)
#define RTC_CTRL_RTC_EN_SHIFT                    (7U)
#define RTC_CTRL_RTC_EN(x)                       (((uint32_t)(((uint32_t)(x)) << RTC_CTRL_RTC_EN_SHIFT)) & RTC_CTRL_RTC_EN_MASK)
#define RTC_CTRL_RTC_OSC_PD_MASK                 (0x100U)
#define RTC_CTRL_RTC_OSC_PD_SHIFT                (8U)
#define RTC_CTRL_RTC_OSC_PD(x)                   (((uint32_t)(((uint32_t)(x)) << RTC_CTRL_RTC_OSC_PD_SHIFT)) & RTC_CTRL_RTC_OSC_PD_MASK)

/*! @name MATCH - RTC match register */
#define RTC_MATCH_MATVAL_MASK                    (0xFFFFFFFFU)
#define RTC_MATCH_MATVAL_SHIFT                   (0U)
#define RTC_MATCH_MATVAL(x)                      (((uint32_t)(((uint32_t)(x)) << RTC_MATCH_MATVAL_SHIFT)) & RTC_MATCH_MATVAL_MASK)

/*! @name COUNT - RTC counter register */
#define RTC_COUNT_VAL_MASK                       (0xFFFFFFFFU)
#define RTC_COUNT_VAL_SHIFT                      (0U)
#define RTC_COUNT_VAL(x)                         (((uint32_t)(((uint32_t)(x)) << RTC_COUNT_VAL_SHIFT)) & RTC_COUNT_VAL_MASK)

/*! @name WAKE - High-resolution/wake-up timer control register */
#define RTC_WAKE_VAL_MASK                        (0xFFFFU)
#define RTC_WAKE_VAL_SHIFT                       (0U)
#define RTC_WAKE_VAL(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_WAKE_VAL_SHIFT)) & RTC_WAKE_VAL_MASK)


/*!
 * @}
 */ /* end of group RTC_Register_Masks */


/* RTC - Peripheral instance base addresses */
/** Peripheral RTC base address */
#define RTC_BASE                                 (0x4002C000u)
/** Peripheral RTC base pointer */
#define RTC                                      ((RTC_Type *)RTC_BASE)
/** Array initializer of RTC peripheral base addresses */
#define RTC_BASE_ADDRS                           { RTC_BASE }
/** Array initializer of RTC peripheral base pointers */
#define RTC_BASE_PTRS                            { RTC }
/** Interrupt vectors for the RTC peripheral type */
#define RTC_IRQS                                 { RTC_IRQn }

/*!
 * @}
 */ /* end of group RTC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SCT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SCT_Peripheral_Access_Layer SCT Peripheral Access Layer
 * @{
 */

/** SCT - Register Layout Typedef */
typedef struct {
  __IO uint32_t CONFIG;                            /**< SCT configuration register, offset: 0x0 */
  __IO uint32_t CTRL;                              /**< SCT control register, offset: 0x4 */
  __IO uint32_t LIMIT;                             /**< SCT limit event select register, offset: 0x8 */
  __IO uint32_t HALT;                              /**< SCT halt event select register, offset: 0xC */
  __IO uint32_t STOP;                              /**< SCT stop event select register, offset: 0x10 */
  __IO uint32_t START;                             /**< SCT start event select register, offset: 0x14 */
       uint8_t RESERVED_0[40];
  __IO uint32_t COUNT;                             /**< SCT counter register, offset: 0x40 */
  __IO uint32_t STATE;                             /**< SCT state register, offset: 0x44 */
  __I  uint32_t INPUT;                             /**< SCT input register, offset: 0x48 */
  __IO uint32_t REGMODE;                           /**< SCT match/capture mode register, offset: 0x4C */
  __IO uint32_t OUTPUT;                            /**< SCT output register, offset: 0x50 */
  __IO uint32_t OUTPUTDIRCTRL;                     /**< SCT output counter direction control register, offset: 0x54 */
  __IO uint32_t RES;                               /**< SCT conflict resolution register, offset: 0x58 */
  __IO uint32_t DMA0REQUEST;                       /**< SCT DMA request 0 register, offset: 0x5C */
  __IO uint32_t DMA1REQUEST;                       /**< SCT DMA request 1 register, offset: 0x60 */
       uint8_t RESERVED_1[140];
  __IO uint32_t EVEN;                              /**< SCT event interrupt enable register, offset: 0xF0 */
  __IO uint32_t EVFLAG;                            /**< SCT event flag register, offset: 0xF4 */
  __IO uint32_t CONEN;                             /**< SCT conflict interrupt enable register, offset: 0xF8 */
  __IO uint32_t CONFLAG;                           /**< SCT conflict flag register, offset: 0xFC */
  union {                                          /* offset: 0x100 */
    __IO uint32_t SCTCAP[10];                        /**< SCT capture register of capture channel, array offset: 0x100, array step: 0x4 */
    __IO uint32_t SCTMATCH[10];                      /**< SCT match value register of match channels, array offset: 0x100, array step: 0x4 */
  };
       uint8_t RESERVED_2[216];
  union {                                          /* offset: 0x200 */
    __IO uint32_t SCTCAPCTRL[10];                    /**< SCT capture control register, array offset: 0x200, array step: 0x4 */
    __IO uint32_t SCTMATCHREL[10];                   /**< SCT match reload value register, array offset: 0x200, array step: 0x4 */
  };
       uint8_t RESERVED_3[216];
  struct {                                         /* offset: 0x300, array step: 0x8 */
    __IO uint32_t STATE;                             /**< SCT event state register 0, array offset: 0x300, array step: 0x8 */
    __IO uint32_t CTRL;                              /**< SCT event control register 0, array offset: 0x304, array step: 0x8 */
  } EVENT[10];
       uint8_t RESERVED_4[432];
  struct {                                         /* offset: 0x500, array step: 0x8 */
    __IO uint32_t SET;                               /**< SCT output 0 set register, array offset: 0x500, array step: 0x8 */
    __IO uint32_t CLR;                               /**< SCT output 0 clear register, array offset: 0x504, array step: 0x8 */
  } OUT[8];
} SCT_Type;

/* ----------------------------------------------------------------------------
   -- SCT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SCT_Register_Masks SCT Register Masks
 * @{
 */

/*! @name CONFIG - SCT configuration register */
#define SCT_CONFIG_UNIFY_MASK                    (0x1U)
#define SCT_CONFIG_UNIFY_SHIFT                   (0U)
#define SCT_CONFIG_UNIFY(x)                      (((uint32_t)(((uint32_t)(x)) << SCT_CONFIG_UNIFY_SHIFT)) & SCT_CONFIG_UNIFY_MASK)
#define SCT_CONFIG_CLKMODE_MASK                  (0x6U)
#define SCT_CONFIG_CLKMODE_SHIFT                 (1U)
#define SCT_CONFIG_CLKMODE(x)                    (((uint32_t)(((uint32_t)(x)) << SCT_CONFIG_CLKMODE_SHIFT)) & SCT_CONFIG_CLKMODE_MASK)
#define SCT_CONFIG_CKSEL_MASK                    (0x78U)
#define SCT_CONFIG_CKSEL_SHIFT                   (3U)
#define SCT_CONFIG_CKSEL(x)                      (((uint32_t)(((uint32_t)(x)) << SCT_CONFIG_CKSEL_SHIFT)) & SCT_CONFIG_CKSEL_MASK)
#define SCT_CONFIG_NORELAOD_L_MASK               (0x80U)
#define SCT_CONFIG_NORELAOD_L_SHIFT              (7U)
#define SCT_CONFIG_NORELAOD_L(x)                 (((uint32_t)(((uint32_t)(x)) << SCT_CONFIG_NORELAOD_L_SHIFT)) & SCT_CONFIG_NORELAOD_L_MASK)
#define SCT_CONFIG_NORELOAD_H_MASK               (0x100U)
#define SCT_CONFIG_NORELOAD_H_SHIFT              (8U)
#define SCT_CONFIG_NORELOAD_H(x)                 (((uint32_t)(((uint32_t)(x)) << SCT_CONFIG_NORELOAD_H_SHIFT)) & SCT_CONFIG_NORELOAD_H_MASK)
#define SCT_CONFIG_INSYNC_MASK                   (0x1E00U)
#define SCT_CONFIG_INSYNC_SHIFT                  (9U)
#define SCT_CONFIG_INSYNC(x)                     (((uint32_t)(((uint32_t)(x)) << SCT_CONFIG_INSYNC_SHIFT)) & SCT_CONFIG_INSYNC_MASK)
#define SCT_CONFIG_AUTOLIMIT_L_MASK              (0x20000U)
#define SCT_CONFIG_AUTOLIMIT_L_SHIFT             (17U)
#define SCT_CONFIG_AUTOLIMIT_L(x)                (((uint32_t)(((uint32_t)(x)) << SCT_CONFIG_AUTOLIMIT_L_SHIFT)) & SCT_CONFIG_AUTOLIMIT_L_MASK)
#define SCT_CONFIG_AUTOLIMIT_H_MASK              (0x40000U)
#define SCT_CONFIG_AUTOLIMIT_H_SHIFT             (18U)
#define SCT_CONFIG_AUTOLIMIT_H(x)                (((uint32_t)(((uint32_t)(x)) << SCT_CONFIG_AUTOLIMIT_H_SHIFT)) & SCT_CONFIG_AUTOLIMIT_H_MASK)

/*! @name CTRL - SCT control register */
#define SCT_CTRL_DOWN_L_MASK                     (0x1U)
#define SCT_CTRL_DOWN_L_SHIFT                    (0U)
#define SCT_CTRL_DOWN_L(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_CTRL_DOWN_L_SHIFT)) & SCT_CTRL_DOWN_L_MASK)
#define SCT_CTRL_STOP_L_MASK                     (0x2U)
#define SCT_CTRL_STOP_L_SHIFT                    (1U)
#define SCT_CTRL_STOP_L(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_CTRL_STOP_L_SHIFT)) & SCT_CTRL_STOP_L_MASK)
#define SCT_CTRL_HALT_L_MASK                     (0x4U)
#define SCT_CTRL_HALT_L_SHIFT                    (2U)
#define SCT_CTRL_HALT_L(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_CTRL_HALT_L_SHIFT)) & SCT_CTRL_HALT_L_MASK)
#define SCT_CTRL_CLRCTR_L_MASK                   (0x8U)
#define SCT_CTRL_CLRCTR_L_SHIFT                  (3U)
#define SCT_CTRL_CLRCTR_L(x)                     (((uint32_t)(((uint32_t)(x)) << SCT_CTRL_CLRCTR_L_SHIFT)) & SCT_CTRL_CLRCTR_L_MASK)
#define SCT_CTRL_BIDIR_L_MASK                    (0x10U)
#define SCT_CTRL_BIDIR_L_SHIFT                   (4U)
#define SCT_CTRL_BIDIR_L(x)                      (((uint32_t)(((uint32_t)(x)) << SCT_CTRL_BIDIR_L_SHIFT)) & SCT_CTRL_BIDIR_L_MASK)
#define SCT_CTRL_PRE_L_MASK                      (0x1FE0U)
#define SCT_CTRL_PRE_L_SHIFT                     (5U)
#define SCT_CTRL_PRE_L(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_CTRL_PRE_L_SHIFT)) & SCT_CTRL_PRE_L_MASK)
#define SCT_CTRL_DOWN_H_MASK                     (0x10000U)
#define SCT_CTRL_DOWN_H_SHIFT                    (16U)
#define SCT_CTRL_DOWN_H(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_CTRL_DOWN_H_SHIFT)) & SCT_CTRL_DOWN_H_MASK)
#define SCT_CTRL_STOP_H_MASK                     (0x20000U)
#define SCT_CTRL_STOP_H_SHIFT                    (17U)
#define SCT_CTRL_STOP_H(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_CTRL_STOP_H_SHIFT)) & SCT_CTRL_STOP_H_MASK)
#define SCT_CTRL_HALT_H_MASK                     (0x40000U)
#define SCT_CTRL_HALT_H_SHIFT                    (18U)
#define SCT_CTRL_HALT_H(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_CTRL_HALT_H_SHIFT)) & SCT_CTRL_HALT_H_MASK)
#define SCT_CTRL_CLRCTR_H_MASK                   (0x80000U)
#define SCT_CTRL_CLRCTR_H_SHIFT                  (19U)
#define SCT_CTRL_CLRCTR_H(x)                     (((uint32_t)(((uint32_t)(x)) << SCT_CTRL_CLRCTR_H_SHIFT)) & SCT_CTRL_CLRCTR_H_MASK)
#define SCT_CTRL_BIDIR_H_MASK                    (0x100000U)
#define SCT_CTRL_BIDIR_H_SHIFT                   (20U)
#define SCT_CTRL_BIDIR_H(x)                      (((uint32_t)(((uint32_t)(x)) << SCT_CTRL_BIDIR_H_SHIFT)) & SCT_CTRL_BIDIR_H_MASK)
#define SCT_CTRL_PRE_H_MASK                      (0x1FE00000U)
#define SCT_CTRL_PRE_H_SHIFT                     (21U)
#define SCT_CTRL_PRE_H(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_CTRL_PRE_H_SHIFT)) & SCT_CTRL_PRE_H_MASK)

/*! @name LIMIT - SCT limit event select register */
#define SCT_LIMIT_LIMMSK_L_MASK                  (0xFFFFU)
#define SCT_LIMIT_LIMMSK_L_SHIFT                 (0U)
#define SCT_LIMIT_LIMMSK_L(x)                    (((uint32_t)(((uint32_t)(x)) << SCT_LIMIT_LIMMSK_L_SHIFT)) & SCT_LIMIT_LIMMSK_L_MASK)
#define SCT_LIMIT_LIMMSK_H_MASK                  (0xFFFF0000U)
#define SCT_LIMIT_LIMMSK_H_SHIFT                 (16U)
#define SCT_LIMIT_LIMMSK_H(x)                    (((uint32_t)(((uint32_t)(x)) << SCT_LIMIT_LIMMSK_H_SHIFT)) & SCT_LIMIT_LIMMSK_H_MASK)

/*! @name HALT - SCT halt event select register */
#define SCT_HALT_HALTMSK_L_MASK                  (0xFFFFU)
#define SCT_HALT_HALTMSK_L_SHIFT                 (0U)
#define SCT_HALT_HALTMSK_L(x)                    (((uint32_t)(((uint32_t)(x)) << SCT_HALT_HALTMSK_L_SHIFT)) & SCT_HALT_HALTMSK_L_MASK)
#define SCT_HALT_HALTMSK_H_MASK                  (0xFFFF0000U)
#define SCT_HALT_HALTMSK_H_SHIFT                 (16U)
#define SCT_HALT_HALTMSK_H(x)                    (((uint32_t)(((uint32_t)(x)) << SCT_HALT_HALTMSK_H_SHIFT)) & SCT_HALT_HALTMSK_H_MASK)

/*! @name STOP - SCT stop event select register */
#define SCT_STOP_STOPMSK_L_MASK                  (0xFFFFU)
#define SCT_STOP_STOPMSK_L_SHIFT                 (0U)
#define SCT_STOP_STOPMSK_L(x)                    (((uint32_t)(((uint32_t)(x)) << SCT_STOP_STOPMSK_L_SHIFT)) & SCT_STOP_STOPMSK_L_MASK)
#define SCT_STOP_STOPMSK_H_MASK                  (0xFFFF0000U)
#define SCT_STOP_STOPMSK_H_SHIFT                 (16U)
#define SCT_STOP_STOPMSK_H(x)                    (((uint32_t)(((uint32_t)(x)) << SCT_STOP_STOPMSK_H_SHIFT)) & SCT_STOP_STOPMSK_H_MASK)

/*! @name START - SCT start event select register */
#define SCT_START_STARTMSK_L_MASK                (0xFFFFU)
#define SCT_START_STARTMSK_L_SHIFT               (0U)
#define SCT_START_STARTMSK_L(x)                  (((uint32_t)(((uint32_t)(x)) << SCT_START_STARTMSK_L_SHIFT)) & SCT_START_STARTMSK_L_MASK)
#define SCT_START_STARTMSK_H_MASK                (0xFFFF0000U)
#define SCT_START_STARTMSK_H_SHIFT               (16U)
#define SCT_START_STARTMSK_H(x)                  (((uint32_t)(((uint32_t)(x)) << SCT_START_STARTMSK_H_SHIFT)) & SCT_START_STARTMSK_H_MASK)

/*! @name COUNT - SCT counter register */
#define SCT_COUNT_CTR_L_MASK                     (0xFFFFU)
#define SCT_COUNT_CTR_L_SHIFT                    (0U)
#define SCT_COUNT_CTR_L(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_COUNT_CTR_L_SHIFT)) & SCT_COUNT_CTR_L_MASK)
#define SCT_COUNT_CTR_H_MASK                     (0xFFFF0000U)
#define SCT_COUNT_CTR_H_SHIFT                    (16U)
#define SCT_COUNT_CTR_H(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_COUNT_CTR_H_SHIFT)) & SCT_COUNT_CTR_H_MASK)

/*! @name STATE - SCT state register */
#define SCT_STATE_STATE_L_MASK                   (0x1FU)
#define SCT_STATE_STATE_L_SHIFT                  (0U)
#define SCT_STATE_STATE_L(x)                     (((uint32_t)(((uint32_t)(x)) << SCT_STATE_STATE_L_SHIFT)) & SCT_STATE_STATE_L_MASK)
#define SCT_STATE_STATE_H_MASK                   (0x1F0000U)
#define SCT_STATE_STATE_H_SHIFT                  (16U)
#define SCT_STATE_STATE_H(x)                     (((uint32_t)(((uint32_t)(x)) << SCT_STATE_STATE_H_SHIFT)) & SCT_STATE_STATE_H_MASK)

/*! @name INPUT - SCT input register */
#define SCT_INPUT_AIN0_MASK                      (0x1U)
#define SCT_INPUT_AIN0_SHIFT                     (0U)
#define SCT_INPUT_AIN0(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_AIN0_SHIFT)) & SCT_INPUT_AIN0_MASK)
#define SCT_INPUT_AIN1_MASK                      (0x2U)
#define SCT_INPUT_AIN1_SHIFT                     (1U)
#define SCT_INPUT_AIN1(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_AIN1_SHIFT)) & SCT_INPUT_AIN1_MASK)
#define SCT_INPUT_AIN2_MASK                      (0x4U)
#define SCT_INPUT_AIN2_SHIFT                     (2U)
#define SCT_INPUT_AIN2(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_AIN2_SHIFT)) & SCT_INPUT_AIN2_MASK)
#define SCT_INPUT_AIN3_MASK                      (0x8U)
#define SCT_INPUT_AIN3_SHIFT                     (3U)
#define SCT_INPUT_AIN3(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_AIN3_SHIFT)) & SCT_INPUT_AIN3_MASK)
#define SCT_INPUT_AIN4_MASK                      (0x10U)
#define SCT_INPUT_AIN4_SHIFT                     (4U)
#define SCT_INPUT_AIN4(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_AIN4_SHIFT)) & SCT_INPUT_AIN4_MASK)
#define SCT_INPUT_AIN5_MASK                      (0x20U)
#define SCT_INPUT_AIN5_SHIFT                     (5U)
#define SCT_INPUT_AIN5(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_AIN5_SHIFT)) & SCT_INPUT_AIN5_MASK)
#define SCT_INPUT_AIN6_MASK                      (0x40U)
#define SCT_INPUT_AIN6_SHIFT                     (6U)
#define SCT_INPUT_AIN6(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_AIN6_SHIFT)) & SCT_INPUT_AIN6_MASK)
#define SCT_INPUT_AIN7_MASK                      (0x80U)
#define SCT_INPUT_AIN7_SHIFT                     (7U)
#define SCT_INPUT_AIN7(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_AIN7_SHIFT)) & SCT_INPUT_AIN7_MASK)
#define SCT_INPUT_AIN8_MASK                      (0x100U)
#define SCT_INPUT_AIN8_SHIFT                     (8U)
#define SCT_INPUT_AIN8(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_AIN8_SHIFT)) & SCT_INPUT_AIN8_MASK)
#define SCT_INPUT_AIN9_MASK                      (0x200U)
#define SCT_INPUT_AIN9_SHIFT                     (9U)
#define SCT_INPUT_AIN9(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_AIN9_SHIFT)) & SCT_INPUT_AIN9_MASK)
#define SCT_INPUT_AIN10_MASK                     (0x400U)
#define SCT_INPUT_AIN10_SHIFT                    (10U)
#define SCT_INPUT_AIN10(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_AIN10_SHIFT)) & SCT_INPUT_AIN10_MASK)
#define SCT_INPUT_AIN11_MASK                     (0x800U)
#define SCT_INPUT_AIN11_SHIFT                    (11U)
#define SCT_INPUT_AIN11(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_AIN11_SHIFT)) & SCT_INPUT_AIN11_MASK)
#define SCT_INPUT_AIN12_MASK                     (0x1000U)
#define SCT_INPUT_AIN12_SHIFT                    (12U)
#define SCT_INPUT_AIN12(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_AIN12_SHIFT)) & SCT_INPUT_AIN12_MASK)
#define SCT_INPUT_AIN13_MASK                     (0x2000U)
#define SCT_INPUT_AIN13_SHIFT                    (13U)
#define SCT_INPUT_AIN13(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_AIN13_SHIFT)) & SCT_INPUT_AIN13_MASK)
#define SCT_INPUT_AIN14_MASK                     (0x4000U)
#define SCT_INPUT_AIN14_SHIFT                    (14U)
#define SCT_INPUT_AIN14(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_AIN14_SHIFT)) & SCT_INPUT_AIN14_MASK)
#define SCT_INPUT_AIN15_MASK                     (0x8000U)
#define SCT_INPUT_AIN15_SHIFT                    (15U)
#define SCT_INPUT_AIN15(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_AIN15_SHIFT)) & SCT_INPUT_AIN15_MASK)
#define SCT_INPUT_SIN0_MASK                      (0x10000U)
#define SCT_INPUT_SIN0_SHIFT                     (16U)
#define SCT_INPUT_SIN0(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_SIN0_SHIFT)) & SCT_INPUT_SIN0_MASK)
#define SCT_INPUT_SIN1_MASK                      (0x20000U)
#define SCT_INPUT_SIN1_SHIFT                     (17U)
#define SCT_INPUT_SIN1(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_SIN1_SHIFT)) & SCT_INPUT_SIN1_MASK)
#define SCT_INPUT_SIN2_MASK                      (0x40000U)
#define SCT_INPUT_SIN2_SHIFT                     (18U)
#define SCT_INPUT_SIN2(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_SIN2_SHIFT)) & SCT_INPUT_SIN2_MASK)
#define SCT_INPUT_SIN3_MASK                      (0x80000U)
#define SCT_INPUT_SIN3_SHIFT                     (19U)
#define SCT_INPUT_SIN3(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_SIN3_SHIFT)) & SCT_INPUT_SIN3_MASK)
#define SCT_INPUT_SIN4_MASK                      (0x100000U)
#define SCT_INPUT_SIN4_SHIFT                     (20U)
#define SCT_INPUT_SIN4(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_SIN4_SHIFT)) & SCT_INPUT_SIN4_MASK)
#define SCT_INPUT_SIN5_MASK                      (0x200000U)
#define SCT_INPUT_SIN5_SHIFT                     (21U)
#define SCT_INPUT_SIN5(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_SIN5_SHIFT)) & SCT_INPUT_SIN5_MASK)
#define SCT_INPUT_SIN6_MASK                      (0x400000U)
#define SCT_INPUT_SIN6_SHIFT                     (22U)
#define SCT_INPUT_SIN6(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_SIN6_SHIFT)) & SCT_INPUT_SIN6_MASK)
#define SCT_INPUT_SIN7_MASK                      (0x800000U)
#define SCT_INPUT_SIN7_SHIFT                     (23U)
#define SCT_INPUT_SIN7(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_SIN7_SHIFT)) & SCT_INPUT_SIN7_MASK)
#define SCT_INPUT_SIN8_MASK                      (0x1000000U)
#define SCT_INPUT_SIN8_SHIFT                     (24U)
#define SCT_INPUT_SIN8(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_SIN8_SHIFT)) & SCT_INPUT_SIN8_MASK)
#define SCT_INPUT_SIN9_MASK                      (0x2000000U)
#define SCT_INPUT_SIN9_SHIFT                     (25U)
#define SCT_INPUT_SIN9(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_SIN9_SHIFT)) & SCT_INPUT_SIN9_MASK)
#define SCT_INPUT_SIN10_MASK                     (0x4000000U)
#define SCT_INPUT_SIN10_SHIFT                    (26U)
#define SCT_INPUT_SIN10(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_SIN10_SHIFT)) & SCT_INPUT_SIN10_MASK)
#define SCT_INPUT_SIN11_MASK                     (0x8000000U)
#define SCT_INPUT_SIN11_SHIFT                    (27U)
#define SCT_INPUT_SIN11(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_SIN11_SHIFT)) & SCT_INPUT_SIN11_MASK)
#define SCT_INPUT_SIN12_MASK                     (0x10000000U)
#define SCT_INPUT_SIN12_SHIFT                    (28U)
#define SCT_INPUT_SIN12(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_SIN12_SHIFT)) & SCT_INPUT_SIN12_MASK)
#define SCT_INPUT_SIN13_MASK                     (0x20000000U)
#define SCT_INPUT_SIN13_SHIFT                    (29U)
#define SCT_INPUT_SIN13(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_SIN13_SHIFT)) & SCT_INPUT_SIN13_MASK)
#define SCT_INPUT_SIN14_MASK                     (0x40000000U)
#define SCT_INPUT_SIN14_SHIFT                    (30U)
#define SCT_INPUT_SIN14(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_SIN14_SHIFT)) & SCT_INPUT_SIN14_MASK)
#define SCT_INPUT_SIN15_MASK                     (0x80000000U)
#define SCT_INPUT_SIN15_SHIFT                    (31U)
#define SCT_INPUT_SIN15(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_INPUT_SIN15_SHIFT)) & SCT_INPUT_SIN15_MASK)

/*! @name REGMODE - SCT match/capture mode register */
#define SCT_REGMODE_REGMOD_L_MASK                (0xFFFFU)
#define SCT_REGMODE_REGMOD_L_SHIFT               (0U)
#define SCT_REGMODE_REGMOD_L(x)                  (((uint32_t)(((uint32_t)(x)) << SCT_REGMODE_REGMOD_L_SHIFT)) & SCT_REGMODE_REGMOD_L_MASK)
#define SCT_REGMODE_REGMOD_H_MASK                (0xFFFF0000U)
#define SCT_REGMODE_REGMOD_H_SHIFT               (16U)
#define SCT_REGMODE_REGMOD_H(x)                  (((uint32_t)(((uint32_t)(x)) << SCT_REGMODE_REGMOD_H_SHIFT)) & SCT_REGMODE_REGMOD_H_MASK)

/*! @name OUTPUT - SCT output register */
#define SCT_OUTPUT_OUT_MASK                      (0xFFFFU)
#define SCT_OUTPUT_OUT_SHIFT                     (0U)
#define SCT_OUTPUT_OUT(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUT_OUT_SHIFT)) & SCT_OUTPUT_OUT_MASK)

/*! @name OUTPUTDIRCTRL - SCT output counter direction control register */
#define SCT_OUTPUTDIRCTRL_SETCLR0_MASK           (0x3U)
#define SCT_OUTPUTDIRCTRL_SETCLR0_SHIFT          (0U)
#define SCT_OUTPUTDIRCTRL_SETCLR0(x)             (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUTDIRCTRL_SETCLR0_SHIFT)) & SCT_OUTPUTDIRCTRL_SETCLR0_MASK)
#define SCT_OUTPUTDIRCTRL_SETCLR1_MASK           (0xCU)
#define SCT_OUTPUTDIRCTRL_SETCLR1_SHIFT          (2U)
#define SCT_OUTPUTDIRCTRL_SETCLR1(x)             (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUTDIRCTRL_SETCLR1_SHIFT)) & SCT_OUTPUTDIRCTRL_SETCLR1_MASK)
#define SCT_OUTPUTDIRCTRL_SETCLR2_MASK           (0x30U)
#define SCT_OUTPUTDIRCTRL_SETCLR2_SHIFT          (4U)
#define SCT_OUTPUTDIRCTRL_SETCLR2(x)             (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUTDIRCTRL_SETCLR2_SHIFT)) & SCT_OUTPUTDIRCTRL_SETCLR2_MASK)
#define SCT_OUTPUTDIRCTRL_SETCLR3_MASK           (0xC0U)
#define SCT_OUTPUTDIRCTRL_SETCLR3_SHIFT          (6U)
#define SCT_OUTPUTDIRCTRL_SETCLR3(x)             (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUTDIRCTRL_SETCLR3_SHIFT)) & SCT_OUTPUTDIRCTRL_SETCLR3_MASK)
#define SCT_OUTPUTDIRCTRL_SETCLR4_MASK           (0x300U)
#define SCT_OUTPUTDIRCTRL_SETCLR4_SHIFT          (8U)
#define SCT_OUTPUTDIRCTRL_SETCLR4(x)             (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUTDIRCTRL_SETCLR4_SHIFT)) & SCT_OUTPUTDIRCTRL_SETCLR4_MASK)
#define SCT_OUTPUTDIRCTRL_SETCLR5_MASK           (0xC00U)
#define SCT_OUTPUTDIRCTRL_SETCLR5_SHIFT          (10U)
#define SCT_OUTPUTDIRCTRL_SETCLR5(x)             (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUTDIRCTRL_SETCLR5_SHIFT)) & SCT_OUTPUTDIRCTRL_SETCLR5_MASK)
#define SCT_OUTPUTDIRCTRL_SETCLR6_MASK           (0x3000U)
#define SCT_OUTPUTDIRCTRL_SETCLR6_SHIFT          (12U)
#define SCT_OUTPUTDIRCTRL_SETCLR6(x)             (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUTDIRCTRL_SETCLR6_SHIFT)) & SCT_OUTPUTDIRCTRL_SETCLR6_MASK)
#define SCT_OUTPUTDIRCTRL_SETCLR7_MASK           (0xC000U)
#define SCT_OUTPUTDIRCTRL_SETCLR7_SHIFT          (14U)
#define SCT_OUTPUTDIRCTRL_SETCLR7(x)             (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUTDIRCTRL_SETCLR7_SHIFT)) & SCT_OUTPUTDIRCTRL_SETCLR7_MASK)
#define SCT_OUTPUTDIRCTRL_SETCLR8_MASK           (0x30000U)
#define SCT_OUTPUTDIRCTRL_SETCLR8_SHIFT          (16U)
#define SCT_OUTPUTDIRCTRL_SETCLR8(x)             (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUTDIRCTRL_SETCLR8_SHIFT)) & SCT_OUTPUTDIRCTRL_SETCLR8_MASK)
#define SCT_OUTPUTDIRCTRL_SETCLR9_MASK           (0xC0000U)
#define SCT_OUTPUTDIRCTRL_SETCLR9_SHIFT          (18U)
#define SCT_OUTPUTDIRCTRL_SETCLR9(x)             (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUTDIRCTRL_SETCLR9_SHIFT)) & SCT_OUTPUTDIRCTRL_SETCLR9_MASK)
#define SCT_OUTPUTDIRCTRL_SETCLR10_MASK          (0x300000U)
#define SCT_OUTPUTDIRCTRL_SETCLR10_SHIFT         (20U)
#define SCT_OUTPUTDIRCTRL_SETCLR10(x)            (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUTDIRCTRL_SETCLR10_SHIFT)) & SCT_OUTPUTDIRCTRL_SETCLR10_MASK)
#define SCT_OUTPUTDIRCTRL_SETCLR11_MASK          (0xC00000U)
#define SCT_OUTPUTDIRCTRL_SETCLR11_SHIFT         (22U)
#define SCT_OUTPUTDIRCTRL_SETCLR11(x)            (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUTDIRCTRL_SETCLR11_SHIFT)) & SCT_OUTPUTDIRCTRL_SETCLR11_MASK)
#define SCT_OUTPUTDIRCTRL_SETCLR12_MASK          (0x3000000U)
#define SCT_OUTPUTDIRCTRL_SETCLR12_SHIFT         (24U)
#define SCT_OUTPUTDIRCTRL_SETCLR12(x)            (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUTDIRCTRL_SETCLR12_SHIFT)) & SCT_OUTPUTDIRCTRL_SETCLR12_MASK)
#define SCT_OUTPUTDIRCTRL_SETCLR13_MASK          (0xC000000U)
#define SCT_OUTPUTDIRCTRL_SETCLR13_SHIFT         (26U)
#define SCT_OUTPUTDIRCTRL_SETCLR13(x)            (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUTDIRCTRL_SETCLR13_SHIFT)) & SCT_OUTPUTDIRCTRL_SETCLR13_MASK)
#define SCT_OUTPUTDIRCTRL_SETCLR14_MASK          (0x30000000U)
#define SCT_OUTPUTDIRCTRL_SETCLR14_SHIFT         (28U)
#define SCT_OUTPUTDIRCTRL_SETCLR14(x)            (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUTDIRCTRL_SETCLR14_SHIFT)) & SCT_OUTPUTDIRCTRL_SETCLR14_MASK)
#define SCT_OUTPUTDIRCTRL_SETCLR15_MASK          (0xC0000000U)
#define SCT_OUTPUTDIRCTRL_SETCLR15_SHIFT         (30U)
#define SCT_OUTPUTDIRCTRL_SETCLR15(x)            (((uint32_t)(((uint32_t)(x)) << SCT_OUTPUTDIRCTRL_SETCLR15_SHIFT)) & SCT_OUTPUTDIRCTRL_SETCLR15_MASK)

/*! @name RES - SCT conflict resolution register */
#define SCT_RES_O0RES_MASK                       (0x3U)
#define SCT_RES_O0RES_SHIFT                      (0U)
#define SCT_RES_O0RES(x)                         (((uint32_t)(((uint32_t)(x)) << SCT_RES_O0RES_SHIFT)) & SCT_RES_O0RES_MASK)
#define SCT_RES_O1RES_MASK                       (0xCU)
#define SCT_RES_O1RES_SHIFT                      (2U)
#define SCT_RES_O1RES(x)                         (((uint32_t)(((uint32_t)(x)) << SCT_RES_O1RES_SHIFT)) & SCT_RES_O1RES_MASK)
#define SCT_RES_O2RES_MASK                       (0x30U)
#define SCT_RES_O2RES_SHIFT                      (4U)
#define SCT_RES_O2RES(x)                         (((uint32_t)(((uint32_t)(x)) << SCT_RES_O2RES_SHIFT)) & SCT_RES_O2RES_MASK)
#define SCT_RES_O3RES_MASK                       (0xC0U)
#define SCT_RES_O3RES_SHIFT                      (6U)
#define SCT_RES_O3RES(x)                         (((uint32_t)(((uint32_t)(x)) << SCT_RES_O3RES_SHIFT)) & SCT_RES_O3RES_MASK)
#define SCT_RES_O4RES_MASK                       (0x300U)
#define SCT_RES_O4RES_SHIFT                      (8U)
#define SCT_RES_O4RES(x)                         (((uint32_t)(((uint32_t)(x)) << SCT_RES_O4RES_SHIFT)) & SCT_RES_O4RES_MASK)
#define SCT_RES_O5RES_MASK                       (0xC00U)
#define SCT_RES_O5RES_SHIFT                      (10U)
#define SCT_RES_O5RES(x)                         (((uint32_t)(((uint32_t)(x)) << SCT_RES_O5RES_SHIFT)) & SCT_RES_O5RES_MASK)
#define SCT_RES_O6RES_MASK                       (0x3000U)
#define SCT_RES_O6RES_SHIFT                      (12U)
#define SCT_RES_O6RES(x)                         (((uint32_t)(((uint32_t)(x)) << SCT_RES_O6RES_SHIFT)) & SCT_RES_O6RES_MASK)
#define SCT_RES_O7RES_MASK                       (0xC000U)
#define SCT_RES_O7RES_SHIFT                      (14U)
#define SCT_RES_O7RES(x)                         (((uint32_t)(((uint32_t)(x)) << SCT_RES_O7RES_SHIFT)) & SCT_RES_O7RES_MASK)
#define SCT_RES_O8RES_MASK                       (0x30000U)
#define SCT_RES_O8RES_SHIFT                      (16U)
#define SCT_RES_O8RES(x)                         (((uint32_t)(((uint32_t)(x)) << SCT_RES_O8RES_SHIFT)) & SCT_RES_O8RES_MASK)
#define SCT_RES_O9RES_MASK                       (0xC0000U)
#define SCT_RES_O9RES_SHIFT                      (18U)
#define SCT_RES_O9RES(x)                         (((uint32_t)(((uint32_t)(x)) << SCT_RES_O9RES_SHIFT)) & SCT_RES_O9RES_MASK)
#define SCT_RES_O10RES_MASK                      (0x300000U)
#define SCT_RES_O10RES_SHIFT                     (20U)
#define SCT_RES_O10RES(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_RES_O10RES_SHIFT)) & SCT_RES_O10RES_MASK)
#define SCT_RES_O11RES_MASK                      (0xC00000U)
#define SCT_RES_O11RES_SHIFT                     (22U)
#define SCT_RES_O11RES(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_RES_O11RES_SHIFT)) & SCT_RES_O11RES_MASK)
#define SCT_RES_O12RES_MASK                      (0x3000000U)
#define SCT_RES_O12RES_SHIFT                     (24U)
#define SCT_RES_O12RES(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_RES_O12RES_SHIFT)) & SCT_RES_O12RES_MASK)
#define SCT_RES_O13RES_MASK                      (0xC000000U)
#define SCT_RES_O13RES_SHIFT                     (26U)
#define SCT_RES_O13RES(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_RES_O13RES_SHIFT)) & SCT_RES_O13RES_MASK)
#define SCT_RES_O14RES_MASK                      (0x30000000U)
#define SCT_RES_O14RES_SHIFT                     (28U)
#define SCT_RES_O14RES(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_RES_O14RES_SHIFT)) & SCT_RES_O14RES_MASK)
#define SCT_RES_O15RES_MASK                      (0xC0000000U)
#define SCT_RES_O15RES_SHIFT                     (30U)
#define SCT_RES_O15RES(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_RES_O15RES_SHIFT)) & SCT_RES_O15RES_MASK)

/*! @name DMA0REQUEST - SCT DMA request 0 register */
#define SCT_DMA0REQUEST_DEV_0_MASK               (0xFFFFU)
#define SCT_DMA0REQUEST_DEV_0_SHIFT              (0U)
#define SCT_DMA0REQUEST_DEV_0(x)                 (((uint32_t)(((uint32_t)(x)) << SCT_DMA0REQUEST_DEV_0_SHIFT)) & SCT_DMA0REQUEST_DEV_0_MASK)
#define SCT_DMA0REQUEST_DRL0_MASK                (0x40000000U)
#define SCT_DMA0REQUEST_DRL0_SHIFT               (30U)
#define SCT_DMA0REQUEST_DRL0(x)                  (((uint32_t)(((uint32_t)(x)) << SCT_DMA0REQUEST_DRL0_SHIFT)) & SCT_DMA0REQUEST_DRL0_MASK)
#define SCT_DMA0REQUEST_DRQ0_MASK                (0x80000000U)
#define SCT_DMA0REQUEST_DRQ0_SHIFT               (31U)
#define SCT_DMA0REQUEST_DRQ0(x)                  (((uint32_t)(((uint32_t)(x)) << SCT_DMA0REQUEST_DRQ0_SHIFT)) & SCT_DMA0REQUEST_DRQ0_MASK)

/*! @name DMA1REQUEST - SCT DMA request 1 register */
#define SCT_DMA1REQUEST_DEV_1_MASK               (0xFFFFU)
#define SCT_DMA1REQUEST_DEV_1_SHIFT              (0U)
#define SCT_DMA1REQUEST_DEV_1(x)                 (((uint32_t)(((uint32_t)(x)) << SCT_DMA1REQUEST_DEV_1_SHIFT)) & SCT_DMA1REQUEST_DEV_1_MASK)
#define SCT_DMA1REQUEST_DRL1_MASK                (0x40000000U)
#define SCT_DMA1REQUEST_DRL1_SHIFT               (30U)
#define SCT_DMA1REQUEST_DRL1(x)                  (((uint32_t)(((uint32_t)(x)) << SCT_DMA1REQUEST_DRL1_SHIFT)) & SCT_DMA1REQUEST_DRL1_MASK)
#define SCT_DMA1REQUEST_DRQ1_MASK                (0x80000000U)
#define SCT_DMA1REQUEST_DRQ1_SHIFT               (31U)
#define SCT_DMA1REQUEST_DRQ1(x)                  (((uint32_t)(((uint32_t)(x)) << SCT_DMA1REQUEST_DRQ1_SHIFT)) & SCT_DMA1REQUEST_DRQ1_MASK)

/*! @name EVEN - SCT event interrupt enable register */
#define SCT_EVEN_IEN_MASK                        (0xFFFFU)
#define SCT_EVEN_IEN_SHIFT                       (0U)
#define SCT_EVEN_IEN(x)                          (((uint32_t)(((uint32_t)(x)) << SCT_EVEN_IEN_SHIFT)) & SCT_EVEN_IEN_MASK)

/*! @name EVFLAG - SCT event flag register */
#define SCT_EVFLAG_FLAG_MASK                     (0xFFFFU)
#define SCT_EVFLAG_FLAG_SHIFT                    (0U)
#define SCT_EVFLAG_FLAG(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_EVFLAG_FLAG_SHIFT)) & SCT_EVFLAG_FLAG_MASK)

/*! @name CONEN - SCT conflict interrupt enable register */
#define SCT_CONEN_NCEN_MASK                      (0xFFFFU)
#define SCT_CONEN_NCEN_SHIFT                     (0U)
#define SCT_CONEN_NCEN(x)                        (((uint32_t)(((uint32_t)(x)) << SCT_CONEN_NCEN_SHIFT)) & SCT_CONEN_NCEN_MASK)

/*! @name CONFLAG - SCT conflict flag register */
#define SCT_CONFLAG_NCFLAG_MASK                  (0xFFFFU)
#define SCT_CONFLAG_NCFLAG_SHIFT                 (0U)
#define SCT_CONFLAG_NCFLAG(x)                    (((uint32_t)(((uint32_t)(x)) << SCT_CONFLAG_NCFLAG_SHIFT)) & SCT_CONFLAG_NCFLAG_MASK)
#define SCT_CONFLAG_BUSERRL_MASK                 (0x40000000U)
#define SCT_CONFLAG_BUSERRL_SHIFT                (30U)
#define SCT_CONFLAG_BUSERRL(x)                   (((uint32_t)(((uint32_t)(x)) << SCT_CONFLAG_BUSERRL_SHIFT)) & SCT_CONFLAG_BUSERRL_MASK)
#define SCT_CONFLAG_BUSERRH_MASK                 (0x80000000U)
#define SCT_CONFLAG_BUSERRH_SHIFT                (31U)
#define SCT_CONFLAG_BUSERRH(x)                   (((uint32_t)(((uint32_t)(x)) << SCT_CONFLAG_BUSERRH_SHIFT)) & SCT_CONFLAG_BUSERRH_MASK)

/*! @name SCTCAP - SCT capture register of capture channel */
#define SCT_SCTCAP_CAPn_L_MASK                   (0xFFFFU)
#define SCT_SCTCAP_CAPn_L_SHIFT                  (0U)
#define SCT_SCTCAP_CAPn_L(x)                     (((uint32_t)(((uint32_t)(x)) << SCT_SCTCAP_CAPn_L_SHIFT)) & SCT_SCTCAP_CAPn_L_MASK)
#define SCT_SCTCAP_CAPn_H_MASK                   (0xFFFF0000U)
#define SCT_SCTCAP_CAPn_H_SHIFT                  (16U)
#define SCT_SCTCAP_CAPn_H(x)                     (((uint32_t)(((uint32_t)(x)) << SCT_SCTCAP_CAPn_H_SHIFT)) & SCT_SCTCAP_CAPn_H_MASK)

/* The count of SCT_SCTCAP */
#define SCT_SCTCAP_COUNT                         (10U)

/*! @name SCTMATCH - SCT match value register of match channels */
#define SCT_SCTMATCH_MATCHn_L_MASK               (0xFFFFU)
#define SCT_SCTMATCH_MATCHn_L_SHIFT              (0U)
#define SCT_SCTMATCH_MATCHn_L(x)                 (((uint32_t)(((uint32_t)(x)) << SCT_SCTMATCH_MATCHn_L_SHIFT)) & SCT_SCTMATCH_MATCHn_L_MASK)
#define SCT_SCTMATCH_MATCHn_H_MASK               (0xFFFF0000U)
#define SCT_SCTMATCH_MATCHn_H_SHIFT              (16U)
#define SCT_SCTMATCH_MATCHn_H(x)                 (((uint32_t)(((uint32_t)(x)) << SCT_SCTMATCH_MATCHn_H_SHIFT)) & SCT_SCTMATCH_MATCHn_H_MASK)

/* The count of SCT_SCTMATCH */
#define SCT_SCTMATCH_COUNT                       (10U)

/*! @name SCTCAPCTRL - SCT capture control register */
#define SCT_SCTCAPCTRL_CAPCONn_L_MASK            (0xFFFFU)
#define SCT_SCTCAPCTRL_CAPCONn_L_SHIFT           (0U)
#define SCT_SCTCAPCTRL_CAPCONn_L(x)              (((uint32_t)(((uint32_t)(x)) << SCT_SCTCAPCTRL_CAPCONn_L_SHIFT)) & SCT_SCTCAPCTRL_CAPCONn_L_MASK)
#define SCT_SCTCAPCTRL_CAPCONn_H_MASK            (0xFFFF0000U)
#define SCT_SCTCAPCTRL_CAPCONn_H_SHIFT           (16U)
#define SCT_SCTCAPCTRL_CAPCONn_H(x)              (((uint32_t)(((uint32_t)(x)) << SCT_SCTCAPCTRL_CAPCONn_H_SHIFT)) & SCT_SCTCAPCTRL_CAPCONn_H_MASK)

/* The count of SCT_SCTCAPCTRL */
#define SCT_SCTCAPCTRL_COUNT                     (10U)

/*! @name SCTMATCHREL - SCT match reload value register */
#define SCT_SCTMATCHREL_RELOADn_L_MASK           (0xFFFFU)
#define SCT_SCTMATCHREL_RELOADn_L_SHIFT          (0U)
#define SCT_SCTMATCHREL_RELOADn_L(x)             (((uint32_t)(((uint32_t)(x)) << SCT_SCTMATCHREL_RELOADn_L_SHIFT)) & SCT_SCTMATCHREL_RELOADn_L_MASK)
#define SCT_SCTMATCHREL_RELOADn_H_MASK           (0xFFFF0000U)
#define SCT_SCTMATCHREL_RELOADn_H_SHIFT          (16U)
#define SCT_SCTMATCHREL_RELOADn_H(x)             (((uint32_t)(((uint32_t)(x)) << SCT_SCTMATCHREL_RELOADn_H_SHIFT)) & SCT_SCTMATCHREL_RELOADn_H_MASK)

/* The count of SCT_SCTMATCHREL */
#define SCT_SCTMATCHREL_COUNT                    (10U)

/*! @name EVENT_STATE - SCT event state register 0 */
#define SCT_EVENT_STATE_STATEMSKn_MASK           (0xFFFFU)
#define SCT_EVENT_STATE_STATEMSKn_SHIFT          (0U)
#define SCT_EVENT_STATE_STATEMSKn(x)             (((uint32_t)(((uint32_t)(x)) << SCT_EVENT_STATE_STATEMSKn_SHIFT)) & SCT_EVENT_STATE_STATEMSKn_MASK)

/* The count of SCT_EVENT_STATE */
#define SCT_EVENT_STATE_COUNT                    (10U)

/*! @name EVENT_CTRL - SCT event control register 0 */
#define SCT_EVENT_CTRL_MATCHSEL_MASK             (0xFU)
#define SCT_EVENT_CTRL_MATCHSEL_SHIFT            (0U)
#define SCT_EVENT_CTRL_MATCHSEL(x)               (((uint32_t)(((uint32_t)(x)) << SCT_EVENT_CTRL_MATCHSEL_SHIFT)) & SCT_EVENT_CTRL_MATCHSEL_MASK)
#define SCT_EVENT_CTRL_HEVENT_MASK               (0x10U)
#define SCT_EVENT_CTRL_HEVENT_SHIFT              (4U)
#define SCT_EVENT_CTRL_HEVENT(x)                 (((uint32_t)(((uint32_t)(x)) << SCT_EVENT_CTRL_HEVENT_SHIFT)) & SCT_EVENT_CTRL_HEVENT_MASK)
#define SCT_EVENT_CTRL_OUTSEL_MASK               (0x20U)
#define SCT_EVENT_CTRL_OUTSEL_SHIFT              (5U)
#define SCT_EVENT_CTRL_OUTSEL(x)                 (((uint32_t)(((uint32_t)(x)) << SCT_EVENT_CTRL_OUTSEL_SHIFT)) & SCT_EVENT_CTRL_OUTSEL_MASK)
#define SCT_EVENT_CTRL_IOSEL_MASK                (0x3C0U)
#define SCT_EVENT_CTRL_IOSEL_SHIFT               (6U)
#define SCT_EVENT_CTRL_IOSEL(x)                  (((uint32_t)(((uint32_t)(x)) << SCT_EVENT_CTRL_IOSEL_SHIFT)) & SCT_EVENT_CTRL_IOSEL_MASK)
#define SCT_EVENT_CTRL_IOCOND_MASK               (0xC00U)
#define SCT_EVENT_CTRL_IOCOND_SHIFT              (10U)
#define SCT_EVENT_CTRL_IOCOND(x)                 (((uint32_t)(((uint32_t)(x)) << SCT_EVENT_CTRL_IOCOND_SHIFT)) & SCT_EVENT_CTRL_IOCOND_MASK)
#define SCT_EVENT_CTRL_COMBMODE_MASK             (0x3000U)
#define SCT_EVENT_CTRL_COMBMODE_SHIFT            (12U)
#define SCT_EVENT_CTRL_COMBMODE(x)               (((uint32_t)(((uint32_t)(x)) << SCT_EVENT_CTRL_COMBMODE_SHIFT)) & SCT_EVENT_CTRL_COMBMODE_MASK)
#define SCT_EVENT_CTRL_STATELD_MASK              (0x4000U)
#define SCT_EVENT_CTRL_STATELD_SHIFT             (14U)
#define SCT_EVENT_CTRL_STATELD(x)                (((uint32_t)(((uint32_t)(x)) << SCT_EVENT_CTRL_STATELD_SHIFT)) & SCT_EVENT_CTRL_STATELD_MASK)
#define SCT_EVENT_CTRL_STATEV_MASK               (0xF8000U)
#define SCT_EVENT_CTRL_STATEV_SHIFT              (15U)
#define SCT_EVENT_CTRL_STATEV(x)                 (((uint32_t)(((uint32_t)(x)) << SCT_EVENT_CTRL_STATEV_SHIFT)) & SCT_EVENT_CTRL_STATEV_MASK)
#define SCT_EVENT_CTRL_MATCHMEM_MASK             (0x100000U)
#define SCT_EVENT_CTRL_MATCHMEM_SHIFT            (20U)
#define SCT_EVENT_CTRL_MATCHMEM(x)               (((uint32_t)(((uint32_t)(x)) << SCT_EVENT_CTRL_MATCHMEM_SHIFT)) & SCT_EVENT_CTRL_MATCHMEM_MASK)
#define SCT_EVENT_CTRL_DIRECTION_MASK            (0x600000U)
#define SCT_EVENT_CTRL_DIRECTION_SHIFT           (21U)
#define SCT_EVENT_CTRL_DIRECTION(x)              (((uint32_t)(((uint32_t)(x)) << SCT_EVENT_CTRL_DIRECTION_SHIFT)) & SCT_EVENT_CTRL_DIRECTION_MASK)

/* The count of SCT_EVENT_CTRL */
#define SCT_EVENT_CTRL_COUNT                     (10U)

/*! @name OUT_SET - SCT output 0 set register */
#define SCT_OUT_SET_SET_MASK                     (0xFFFFU)
#define SCT_OUT_SET_SET_SHIFT                    (0U)
#define SCT_OUT_SET_SET(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_OUT_SET_SET_SHIFT)) & SCT_OUT_SET_SET_MASK)

/* The count of SCT_OUT_SET */
#define SCT_OUT_SET_COUNT                        (8U)

/*! @name OUT_CLR - SCT output 0 clear register */
#define SCT_OUT_CLR_CLR_MASK                     (0xFFFFU)
#define SCT_OUT_CLR_CLR_SHIFT                    (0U)
#define SCT_OUT_CLR_CLR(x)                       (((uint32_t)(((uint32_t)(x)) << SCT_OUT_CLR_CLR_SHIFT)) & SCT_OUT_CLR_CLR_MASK)

/* The count of SCT_OUT_CLR */
#define SCT_OUT_CLR_COUNT                        (8U)


/*!
 * @}
 */ /* end of group SCT_Register_Masks */


/* SCT - Peripheral instance base addresses */
/** Peripheral SCT0 base address */
#define SCT0_BASE                                (0x40085000u)
/** Peripheral SCT0 base pointer */
#define SCT0                                     ((SCT_Type *)SCT0_BASE)
/** Array initializer of SCT peripheral base addresses */
#define SCT_BASE_ADDRS                           { SCT0_BASE }
/** Array initializer of SCT peripheral base pointers */
#define SCT_BASE_PTRS                            { SCT0 }
/** Interrupt vectors for the SCT peripheral type */
#define SCT_IRQS                                 { SCT0_IRQn }

/*!
 * @}
 */ /* end of group SCT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SPI Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Peripheral_Access_Layer SPI Peripheral Access Layer
 * @{
 */

/** SPI - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[1024];
  __IO uint32_t CFG;                               /**< SPI Configuration register, offset: 0x400 */
  __IO uint32_t DLY;                               /**< SPI Delay register, offset: 0x404 */
  __IO uint32_t STAT;                              /**< SPI Status. Some status flags can be cleared by writing a 1 to that bit position., offset: 0x408 */
  __IO uint32_t INTENSET;                          /**< SPI Interrupt Enable read and Set. A complete value may be read from this register. Writing a 1 to any implemented bit position causes that bit to be set., offset: 0x40C */
  __O  uint32_t INTENCLR;                          /**< SPI Interrupt Enable Clear. Writing a 1 to any implemented bit position causes the corresponding bit in INTENSET to be cleared., offset: 0x410 */
       uint8_t RESERVED_1[16];
  __IO uint32_t DIV;                               /**< SPI clock Divider, offset: 0x424 */
  __I  uint32_t INTSTAT;                           /**< SPI Interrupt Status, offset: 0x428 */
       uint8_t RESERVED_2[2516];
  __IO uint32_t FIFOCFG;                           /**< FIFO configuration and enable register., offset: 0xE00 */
  __IO uint32_t FIFOSTAT;                          /**< FIFO status register., offset: 0xE04 */
  __IO uint32_t FIFOTRIG;                          /**< FIFO trigger settings for interrupt and DMA request., offset: 0xE08 */
       uint8_t RESERVED_3[4];
  __IO uint32_t FIFOINTENSET;                      /**< FIFO interrupt enable set (enable) and read register., offset: 0xE10 */
  __IO uint32_t FIFOINTENCLR;                      /**< FIFO interrupt enable clear (disable) and read register., offset: 0xE14 */
  __I  uint32_t FIFOINTSTAT;                       /**< FIFO interrupt status register., offset: 0xE18 */
       uint8_t RESERVED_4[4];
  __O  uint32_t FIFOWR;                            /**< FIFO write data., offset: 0xE20 */
       uint8_t RESERVED_5[12];
  __I  uint32_t FIFORD;                            /**< FIFO read data., offset: 0xE30 */
       uint8_t RESERVED_6[12];
  __I  uint32_t FIFORDNOPOP;                       /**< FIFO data read with no FIFO pop., offset: 0xE40 */
       uint8_t RESERVED_7[440];
  __I  uint32_t ID;                                /**< Peripheral identification register., offset: 0xFFC */
} SPI_Type;

/* ----------------------------------------------------------------------------
   -- SPI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Register_Masks SPI Register Masks
 * @{
 */

/*! @name CFG - SPI Configuration register */
#define SPI_CFG_ENABLE_MASK                      (0x1U)
#define SPI_CFG_ENABLE_SHIFT                     (0U)
#define SPI_CFG_ENABLE(x)                        (((uint32_t)(((uint32_t)(x)) << SPI_CFG_ENABLE_SHIFT)) & SPI_CFG_ENABLE_MASK)
#define SPI_CFG_MASTER_MASK                      (0x4U)
#define SPI_CFG_MASTER_SHIFT                     (2U)
#define SPI_CFG_MASTER(x)                        (((uint32_t)(((uint32_t)(x)) << SPI_CFG_MASTER_SHIFT)) & SPI_CFG_MASTER_MASK)
#define SPI_CFG_LSBF_MASK                        (0x8U)
#define SPI_CFG_LSBF_SHIFT                       (3U)
#define SPI_CFG_LSBF(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_CFG_LSBF_SHIFT)) & SPI_CFG_LSBF_MASK)
#define SPI_CFG_CPHA_MASK                        (0x10U)
#define SPI_CFG_CPHA_SHIFT                       (4U)
#define SPI_CFG_CPHA(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_CFG_CPHA_SHIFT)) & SPI_CFG_CPHA_MASK)
#define SPI_CFG_CPOL_MASK                        (0x20U)
#define SPI_CFG_CPOL_SHIFT                       (5U)
#define SPI_CFG_CPOL(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_CFG_CPOL_SHIFT)) & SPI_CFG_CPOL_MASK)
#define SPI_CFG_LOOP_MASK                        (0x80U)
#define SPI_CFG_LOOP_SHIFT                       (7U)
#define SPI_CFG_LOOP(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_CFG_LOOP_SHIFT)) & SPI_CFG_LOOP_MASK)
#define SPI_CFG_SPOL0_MASK                       (0x100U)
#define SPI_CFG_SPOL0_SHIFT                      (8U)
#define SPI_CFG_SPOL0(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_CFG_SPOL0_SHIFT)) & SPI_CFG_SPOL0_MASK)
#define SPI_CFG_SPOL1_MASK                       (0x200U)
#define SPI_CFG_SPOL1_SHIFT                      (9U)
#define SPI_CFG_SPOL1(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_CFG_SPOL1_SHIFT)) & SPI_CFG_SPOL1_MASK)
#define SPI_CFG_SPOL2_MASK                       (0x400U)
#define SPI_CFG_SPOL2_SHIFT                      (10U)
#define SPI_CFG_SPOL2(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_CFG_SPOL2_SHIFT)) & SPI_CFG_SPOL2_MASK)
#define SPI_CFG_SPOL3_MASK                       (0x800U)
#define SPI_CFG_SPOL3_SHIFT                      (11U)
#define SPI_CFG_SPOL3(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_CFG_SPOL3_SHIFT)) & SPI_CFG_SPOL3_MASK)

/*! @name DLY - SPI Delay register */
#define SPI_DLY_PRE_DELAY_MASK                   (0xFU)
#define SPI_DLY_PRE_DELAY_SHIFT                  (0U)
#define SPI_DLY_PRE_DELAY(x)                     (((uint32_t)(((uint32_t)(x)) << SPI_DLY_PRE_DELAY_SHIFT)) & SPI_DLY_PRE_DELAY_MASK)
#define SPI_DLY_POST_DELAY_MASK                  (0xF0U)
#define SPI_DLY_POST_DELAY_SHIFT                 (4U)
#define SPI_DLY_POST_DELAY(x)                    (((uint32_t)(((uint32_t)(x)) << SPI_DLY_POST_DELAY_SHIFT)) & SPI_DLY_POST_DELAY_MASK)
#define SPI_DLY_FRAME_DELAY_MASK                 (0xF00U)
#define SPI_DLY_FRAME_DELAY_SHIFT                (8U)
#define SPI_DLY_FRAME_DELAY(x)                   (((uint32_t)(((uint32_t)(x)) << SPI_DLY_FRAME_DELAY_SHIFT)) & SPI_DLY_FRAME_DELAY_MASK)
#define SPI_DLY_TRANSFER_DELAY_MASK              (0xF000U)
#define SPI_DLY_TRANSFER_DELAY_SHIFT             (12U)
#define SPI_DLY_TRANSFER_DELAY(x)                (((uint32_t)(((uint32_t)(x)) << SPI_DLY_TRANSFER_DELAY_SHIFT)) & SPI_DLY_TRANSFER_DELAY_MASK)

/*! @name STAT - SPI Status. Some status flags can be cleared by writing a 1 to that bit position. */
#define SPI_STAT_SSA_MASK                        (0x10U)
#define SPI_STAT_SSA_SHIFT                       (4U)
#define SPI_STAT_SSA(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_STAT_SSA_SHIFT)) & SPI_STAT_SSA_MASK)
#define SPI_STAT_SSD_MASK                        (0x20U)
#define SPI_STAT_SSD_SHIFT                       (5U)
#define SPI_STAT_SSD(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_STAT_SSD_SHIFT)) & SPI_STAT_SSD_MASK)
#define SPI_STAT_STALLED_MASK                    (0x40U)
#define SPI_STAT_STALLED_SHIFT                   (6U)
#define SPI_STAT_STALLED(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_STAT_STALLED_SHIFT)) & SPI_STAT_STALLED_MASK)
#define SPI_STAT_ENDTRANSFER_MASK                (0x80U)
#define SPI_STAT_ENDTRANSFER_SHIFT               (7U)
#define SPI_STAT_ENDTRANSFER(x)                  (((uint32_t)(((uint32_t)(x)) << SPI_STAT_ENDTRANSFER_SHIFT)) & SPI_STAT_ENDTRANSFER_MASK)
#define SPI_STAT_MSTIDLE_MASK                    (0x100U)
#define SPI_STAT_MSTIDLE_SHIFT                   (8U)
#define SPI_STAT_MSTIDLE(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_STAT_MSTIDLE_SHIFT)) & SPI_STAT_MSTIDLE_MASK)

/*! @name INTENSET - SPI Interrupt Enable read and Set. A complete value may be read from this register. Writing a 1 to any implemented bit position causes that bit to be set. */
#define SPI_INTENSET_SSAEN_MASK                  (0x10U)
#define SPI_INTENSET_SSAEN_SHIFT                 (4U)
#define SPI_INTENSET_SSAEN(x)                    (((uint32_t)(((uint32_t)(x)) << SPI_INTENSET_SSAEN_SHIFT)) & SPI_INTENSET_SSAEN_MASK)
#define SPI_INTENSET_SSDEN_MASK                  (0x20U)
#define SPI_INTENSET_SSDEN_SHIFT                 (5U)
#define SPI_INTENSET_SSDEN(x)                    (((uint32_t)(((uint32_t)(x)) << SPI_INTENSET_SSDEN_SHIFT)) & SPI_INTENSET_SSDEN_MASK)
#define SPI_INTENSET_MSTIDLEEN_MASK              (0x100U)
#define SPI_INTENSET_MSTIDLEEN_SHIFT             (8U)
#define SPI_INTENSET_MSTIDLEEN(x)                (((uint32_t)(((uint32_t)(x)) << SPI_INTENSET_MSTIDLEEN_SHIFT)) & SPI_INTENSET_MSTIDLEEN_MASK)

/*! @name INTENCLR - SPI Interrupt Enable Clear. Writing a 1 to any implemented bit position causes the corresponding bit in INTENSET to be cleared. */
#define SPI_INTENCLR_SSAEN_MASK                  (0x10U)
#define SPI_INTENCLR_SSAEN_SHIFT                 (4U)
#define SPI_INTENCLR_SSAEN(x)                    (((uint32_t)(((uint32_t)(x)) << SPI_INTENCLR_SSAEN_SHIFT)) & SPI_INTENCLR_SSAEN_MASK)
#define SPI_INTENCLR_SSDEN_MASK                  (0x20U)
#define SPI_INTENCLR_SSDEN_SHIFT                 (5U)
#define SPI_INTENCLR_SSDEN(x)                    (((uint32_t)(((uint32_t)(x)) << SPI_INTENCLR_SSDEN_SHIFT)) & SPI_INTENCLR_SSDEN_MASK)
#define SPI_INTENCLR_MSTIDLE_MASK                (0x100U)
#define SPI_INTENCLR_MSTIDLE_SHIFT               (8U)
#define SPI_INTENCLR_MSTIDLE(x)                  (((uint32_t)(((uint32_t)(x)) << SPI_INTENCLR_MSTIDLE_SHIFT)) & SPI_INTENCLR_MSTIDLE_MASK)

/*! @name DIV - SPI clock Divider */
#define SPI_DIV_DIVVAL_MASK                      (0xFFFFU)
#define SPI_DIV_DIVVAL_SHIFT                     (0U)
#define SPI_DIV_DIVVAL(x)                        (((uint32_t)(((uint32_t)(x)) << SPI_DIV_DIVVAL_SHIFT)) & SPI_DIV_DIVVAL_MASK)

/*! @name INTSTAT - SPI Interrupt Status */
#define SPI_INTSTAT_SSA_MASK                     (0x10U)
#define SPI_INTSTAT_SSA_SHIFT                    (4U)
#define SPI_INTSTAT_SSA(x)                       (((uint32_t)(((uint32_t)(x)) << SPI_INTSTAT_SSA_SHIFT)) & SPI_INTSTAT_SSA_MASK)
#define SPI_INTSTAT_SSD_MASK                     (0x20U)
#define SPI_INTSTAT_SSD_SHIFT                    (5U)
#define SPI_INTSTAT_SSD(x)                       (((uint32_t)(((uint32_t)(x)) << SPI_INTSTAT_SSD_SHIFT)) & SPI_INTSTAT_SSD_MASK)
#define SPI_INTSTAT_MSTIDLE_MASK                 (0x100U)
#define SPI_INTSTAT_MSTIDLE_SHIFT                (8U)
#define SPI_INTSTAT_MSTIDLE(x)                   (((uint32_t)(((uint32_t)(x)) << SPI_INTSTAT_MSTIDLE_SHIFT)) & SPI_INTSTAT_MSTIDLE_MASK)

/*! @name FIFOCFG - FIFO configuration and enable register. */
#define SPI_FIFOCFG_ENABLETX_MASK                (0x1U)
#define SPI_FIFOCFG_ENABLETX_SHIFT               (0U)
#define SPI_FIFOCFG_ENABLETX(x)                  (((uint32_t)(((uint32_t)(x)) << SPI_FIFOCFG_ENABLETX_SHIFT)) & SPI_FIFOCFG_ENABLETX_MASK)
#define SPI_FIFOCFG_ENABLERX_MASK                (0x2U)
#define SPI_FIFOCFG_ENABLERX_SHIFT               (1U)
#define SPI_FIFOCFG_ENABLERX(x)                  (((uint32_t)(((uint32_t)(x)) << SPI_FIFOCFG_ENABLERX_SHIFT)) & SPI_FIFOCFG_ENABLERX_MASK)
#define SPI_FIFOCFG_SIZE_MASK                    (0x30U)
#define SPI_FIFOCFG_SIZE_SHIFT                   (4U)
#define SPI_FIFOCFG_SIZE(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_FIFOCFG_SIZE_SHIFT)) & SPI_FIFOCFG_SIZE_MASK)
#define SPI_FIFOCFG_DMATX_MASK                   (0x1000U)
#define SPI_FIFOCFG_DMATX_SHIFT                  (12U)
#define SPI_FIFOCFG_DMATX(x)                     (((uint32_t)(((uint32_t)(x)) << SPI_FIFOCFG_DMATX_SHIFT)) & SPI_FIFOCFG_DMATX_MASK)
#define SPI_FIFOCFG_DMARX_MASK                   (0x2000U)
#define SPI_FIFOCFG_DMARX_SHIFT                  (13U)
#define SPI_FIFOCFG_DMARX(x)                     (((uint32_t)(((uint32_t)(x)) << SPI_FIFOCFG_DMARX_SHIFT)) & SPI_FIFOCFG_DMARX_MASK)
#define SPI_FIFOCFG_WAKETX_MASK                  (0x4000U)
#define SPI_FIFOCFG_WAKETX_SHIFT                 (14U)
#define SPI_FIFOCFG_WAKETX(x)                    (((uint32_t)(((uint32_t)(x)) << SPI_FIFOCFG_WAKETX_SHIFT)) & SPI_FIFOCFG_WAKETX_MASK)
#define SPI_FIFOCFG_WAKERX_MASK                  (0x8000U)
#define SPI_FIFOCFG_WAKERX_SHIFT                 (15U)
#define SPI_FIFOCFG_WAKERX(x)                    (((uint32_t)(((uint32_t)(x)) << SPI_FIFOCFG_WAKERX_SHIFT)) & SPI_FIFOCFG_WAKERX_MASK)
#define SPI_FIFOCFG_EMPTYTX_MASK                 (0x10000U)
#define SPI_FIFOCFG_EMPTYTX_SHIFT                (16U)
#define SPI_FIFOCFG_EMPTYTX(x)                   (((uint32_t)(((uint32_t)(x)) << SPI_FIFOCFG_EMPTYTX_SHIFT)) & SPI_FIFOCFG_EMPTYTX_MASK)
#define SPI_FIFOCFG_EMPTYRX_MASK                 (0x20000U)
#define SPI_FIFOCFG_EMPTYRX_SHIFT                (17U)
#define SPI_FIFOCFG_EMPTYRX(x)                   (((uint32_t)(((uint32_t)(x)) << SPI_FIFOCFG_EMPTYRX_SHIFT)) & SPI_FIFOCFG_EMPTYRX_MASK)

/*! @name FIFOSTAT - FIFO status register. */
#define SPI_FIFOSTAT_TXERR_MASK                  (0x1U)
#define SPI_FIFOSTAT_TXERR_SHIFT                 (0U)
#define SPI_FIFOSTAT_TXERR(x)                    (((uint32_t)(((uint32_t)(x)) << SPI_FIFOSTAT_TXERR_SHIFT)) & SPI_FIFOSTAT_TXERR_MASK)
#define SPI_FIFOSTAT_RXERR_MASK                  (0x2U)
#define SPI_FIFOSTAT_RXERR_SHIFT                 (1U)
#define SPI_FIFOSTAT_RXERR(x)                    (((uint32_t)(((uint32_t)(x)) << SPI_FIFOSTAT_RXERR_SHIFT)) & SPI_FIFOSTAT_RXERR_MASK)
#define SPI_FIFOSTAT_PERINT_MASK                 (0x8U)
#define SPI_FIFOSTAT_PERINT_SHIFT                (3U)
#define SPI_FIFOSTAT_PERINT(x)                   (((uint32_t)(((uint32_t)(x)) << SPI_FIFOSTAT_PERINT_SHIFT)) & SPI_FIFOSTAT_PERINT_MASK)
#define SPI_FIFOSTAT_TXEMPTY_MASK                (0x10U)
#define SPI_FIFOSTAT_TXEMPTY_SHIFT               (4U)
#define SPI_FIFOSTAT_TXEMPTY(x)                  (((uint32_t)(((uint32_t)(x)) << SPI_FIFOSTAT_TXEMPTY_SHIFT)) & SPI_FIFOSTAT_TXEMPTY_MASK)
#define SPI_FIFOSTAT_TXNOTFULL_MASK              (0x20U)
#define SPI_FIFOSTAT_TXNOTFULL_SHIFT             (5U)
#define SPI_FIFOSTAT_TXNOTFULL(x)                (((uint32_t)(((uint32_t)(x)) << SPI_FIFOSTAT_TXNOTFULL_SHIFT)) & SPI_FIFOSTAT_TXNOTFULL_MASK)
#define SPI_FIFOSTAT_RXNOTEMPTY_MASK             (0x40U)
#define SPI_FIFOSTAT_RXNOTEMPTY_SHIFT            (6U)
#define SPI_FIFOSTAT_RXNOTEMPTY(x)               (((uint32_t)(((uint32_t)(x)) << SPI_FIFOSTAT_RXNOTEMPTY_SHIFT)) & SPI_FIFOSTAT_RXNOTEMPTY_MASK)
#define SPI_FIFOSTAT_RXFULL_MASK                 (0x80U)
#define SPI_FIFOSTAT_RXFULL_SHIFT                (7U)
#define SPI_FIFOSTAT_RXFULL(x)                   (((uint32_t)(((uint32_t)(x)) << SPI_FIFOSTAT_RXFULL_SHIFT)) & SPI_FIFOSTAT_RXFULL_MASK)
#define SPI_FIFOSTAT_TXLVL_MASK                  (0x1F00U)
#define SPI_FIFOSTAT_TXLVL_SHIFT                 (8U)
#define SPI_FIFOSTAT_TXLVL(x)                    (((uint32_t)(((uint32_t)(x)) << SPI_FIFOSTAT_TXLVL_SHIFT)) & SPI_FIFOSTAT_TXLVL_MASK)
#define SPI_FIFOSTAT_RXLVL_MASK                  (0x1F0000U)
#define SPI_FIFOSTAT_RXLVL_SHIFT                 (16U)
#define SPI_FIFOSTAT_RXLVL(x)                    (((uint32_t)(((uint32_t)(x)) << SPI_FIFOSTAT_RXLVL_SHIFT)) & SPI_FIFOSTAT_RXLVL_MASK)

/*! @name FIFOTRIG - FIFO trigger settings for interrupt and DMA request. */
#define SPI_FIFOTRIG_TXLVLENA_MASK               (0x1U)
#define SPI_FIFOTRIG_TXLVLENA_SHIFT              (0U)
#define SPI_FIFOTRIG_TXLVLENA(x)                 (((uint32_t)(((uint32_t)(x)) << SPI_FIFOTRIG_TXLVLENA_SHIFT)) & SPI_FIFOTRIG_TXLVLENA_MASK)
#define SPI_FIFOTRIG_RXLVLENA_MASK               (0x2U)
#define SPI_FIFOTRIG_RXLVLENA_SHIFT              (1U)
#define SPI_FIFOTRIG_RXLVLENA(x)                 (((uint32_t)(((uint32_t)(x)) << SPI_FIFOTRIG_RXLVLENA_SHIFT)) & SPI_FIFOTRIG_RXLVLENA_MASK)
#define SPI_FIFOTRIG_TXLVL_MASK                  (0xF00U)
#define SPI_FIFOTRIG_TXLVL_SHIFT                 (8U)
#define SPI_FIFOTRIG_TXLVL(x)                    (((uint32_t)(((uint32_t)(x)) << SPI_FIFOTRIG_TXLVL_SHIFT)) & SPI_FIFOTRIG_TXLVL_MASK)
#define SPI_FIFOTRIG_RXLVL_MASK                  (0xF0000U)
#define SPI_FIFOTRIG_RXLVL_SHIFT                 (16U)
#define SPI_FIFOTRIG_RXLVL(x)                    (((uint32_t)(((uint32_t)(x)) << SPI_FIFOTRIG_RXLVL_SHIFT)) & SPI_FIFOTRIG_RXLVL_MASK)

/*! @name FIFOINTENSET - FIFO interrupt enable set (enable) and read register. */
#define SPI_FIFOINTENSET_TXERR_MASK              (0x1U)
#define SPI_FIFOINTENSET_TXERR_SHIFT             (0U)
#define SPI_FIFOINTENSET_TXERR(x)                (((uint32_t)(((uint32_t)(x)) << SPI_FIFOINTENSET_TXERR_SHIFT)) & SPI_FIFOINTENSET_TXERR_MASK)
#define SPI_FIFOINTENSET_RXERR_MASK              (0x2U)
#define SPI_FIFOINTENSET_RXERR_SHIFT             (1U)
#define SPI_FIFOINTENSET_RXERR(x)                (((uint32_t)(((uint32_t)(x)) << SPI_FIFOINTENSET_RXERR_SHIFT)) & SPI_FIFOINTENSET_RXERR_MASK)
#define SPI_FIFOINTENSET_TXLVL_MASK              (0x4U)
#define SPI_FIFOINTENSET_TXLVL_SHIFT             (2U)
#define SPI_FIFOINTENSET_TXLVL(x)                (((uint32_t)(((uint32_t)(x)) << SPI_FIFOINTENSET_TXLVL_SHIFT)) & SPI_FIFOINTENSET_TXLVL_MASK)
#define SPI_FIFOINTENSET_RXLVL_MASK              (0x8U)
#define SPI_FIFOINTENSET_RXLVL_SHIFT             (3U)
#define SPI_FIFOINTENSET_RXLVL(x)                (((uint32_t)(((uint32_t)(x)) << SPI_FIFOINTENSET_RXLVL_SHIFT)) & SPI_FIFOINTENSET_RXLVL_MASK)

/*! @name FIFOINTENCLR - FIFO interrupt enable clear (disable) and read register. */
#define SPI_FIFOINTENCLR_TXERR_MASK              (0x1U)
#define SPI_FIFOINTENCLR_TXERR_SHIFT             (0U)
#define SPI_FIFOINTENCLR_TXERR(x)                (((uint32_t)(((uint32_t)(x)) << SPI_FIFOINTENCLR_TXERR_SHIFT)) & SPI_FIFOINTENCLR_TXERR_MASK)
#define SPI_FIFOINTENCLR_RXERR_MASK              (0x2U)
#define SPI_FIFOINTENCLR_RXERR_SHIFT             (1U)
#define SPI_FIFOINTENCLR_RXERR(x)                (((uint32_t)(((uint32_t)(x)) << SPI_FIFOINTENCLR_RXERR_SHIFT)) & SPI_FIFOINTENCLR_RXERR_MASK)
#define SPI_FIFOINTENCLR_TXLVL_MASK              (0x4U)
#define SPI_FIFOINTENCLR_TXLVL_SHIFT             (2U)
#define SPI_FIFOINTENCLR_TXLVL(x)                (((uint32_t)(((uint32_t)(x)) << SPI_FIFOINTENCLR_TXLVL_SHIFT)) & SPI_FIFOINTENCLR_TXLVL_MASK)
#define SPI_FIFOINTENCLR_RXLVL_MASK              (0x8U)
#define SPI_FIFOINTENCLR_RXLVL_SHIFT             (3U)
#define SPI_FIFOINTENCLR_RXLVL(x)                (((uint32_t)(((uint32_t)(x)) << SPI_FIFOINTENCLR_RXLVL_SHIFT)) & SPI_FIFOINTENCLR_RXLVL_MASK)

/*! @name FIFOINTSTAT - FIFO interrupt status register. */
#define SPI_FIFOINTSTAT_TXERR_MASK               (0x1U)
#define SPI_FIFOINTSTAT_TXERR_SHIFT              (0U)
#define SPI_FIFOINTSTAT_TXERR(x)                 (((uint32_t)(((uint32_t)(x)) << SPI_FIFOINTSTAT_TXERR_SHIFT)) & SPI_FIFOINTSTAT_TXERR_MASK)
#define SPI_FIFOINTSTAT_RXERR_MASK               (0x2U)
#define SPI_FIFOINTSTAT_RXERR_SHIFT              (1U)
#define SPI_FIFOINTSTAT_RXERR(x)                 (((uint32_t)(((uint32_t)(x)) << SPI_FIFOINTSTAT_RXERR_SHIFT)) & SPI_FIFOINTSTAT_RXERR_MASK)
#define SPI_FIFOINTSTAT_TXLVL_MASK               (0x4U)
#define SPI_FIFOINTSTAT_TXLVL_SHIFT              (2U)
#define SPI_FIFOINTSTAT_TXLVL(x)                 (((uint32_t)(((uint32_t)(x)) << SPI_FIFOINTSTAT_TXLVL_SHIFT)) & SPI_FIFOINTSTAT_TXLVL_MASK)
#define SPI_FIFOINTSTAT_RXLVL_MASK               (0x8U)
#define SPI_FIFOINTSTAT_RXLVL_SHIFT              (3U)
#define SPI_FIFOINTSTAT_RXLVL(x)                 (((uint32_t)(((uint32_t)(x)) << SPI_FIFOINTSTAT_RXLVL_SHIFT)) & SPI_FIFOINTSTAT_RXLVL_MASK)
#define SPI_FIFOINTSTAT_PERINT_MASK              (0x10U)
#define SPI_FIFOINTSTAT_PERINT_SHIFT             (4U)
#define SPI_FIFOINTSTAT_PERINT(x)                (((uint32_t)(((uint32_t)(x)) << SPI_FIFOINTSTAT_PERINT_SHIFT)) & SPI_FIFOINTSTAT_PERINT_MASK)

/*! @name FIFOWR - FIFO write data. */
#define SPI_FIFOWR_TXDATA_MASK                   (0xFFFFU)
#define SPI_FIFOWR_TXDATA_SHIFT                  (0U)
#define SPI_FIFOWR_TXDATA(x)                     (((uint32_t)(((uint32_t)(x)) << SPI_FIFOWR_TXDATA_SHIFT)) & SPI_FIFOWR_TXDATA_MASK)
#define SPI_FIFOWR_TXSSEL0_N_MASK                (0x10000U)
#define SPI_FIFOWR_TXSSEL0_N_SHIFT               (16U)
#define SPI_FIFOWR_TXSSEL0_N(x)                  (((uint32_t)(((uint32_t)(x)) << SPI_FIFOWR_TXSSEL0_N_SHIFT)) & SPI_FIFOWR_TXSSEL0_N_MASK)
#define SPI_FIFOWR_TXSSEL1_N_MASK                (0x20000U)
#define SPI_FIFOWR_TXSSEL1_N_SHIFT               (17U)
#define SPI_FIFOWR_TXSSEL1_N(x)                  (((uint32_t)(((uint32_t)(x)) << SPI_FIFOWR_TXSSEL1_N_SHIFT)) & SPI_FIFOWR_TXSSEL1_N_MASK)
#define SPI_FIFOWR_TXSSEL2_N_MASK                (0x40000U)
#define SPI_FIFOWR_TXSSEL2_N_SHIFT               (18U)
#define SPI_FIFOWR_TXSSEL2_N(x)                  (((uint32_t)(((uint32_t)(x)) << SPI_FIFOWR_TXSSEL2_N_SHIFT)) & SPI_FIFOWR_TXSSEL2_N_MASK)
#define SPI_FIFOWR_TXSSEL3_N_MASK                (0x80000U)
#define SPI_FIFOWR_TXSSEL3_N_SHIFT               (19U)
#define SPI_FIFOWR_TXSSEL3_N(x)                  (((uint32_t)(((uint32_t)(x)) << SPI_FIFOWR_TXSSEL3_N_SHIFT)) & SPI_FIFOWR_TXSSEL3_N_MASK)
#define SPI_FIFOWR_EOT_MASK                      (0x100000U)
#define SPI_FIFOWR_EOT_SHIFT                     (20U)
#define SPI_FIFOWR_EOT(x)                        (((uint32_t)(((uint32_t)(x)) << SPI_FIFOWR_EOT_SHIFT)) & SPI_FIFOWR_EOT_MASK)
#define SPI_FIFOWR_EOF_MASK                      (0x200000U)
#define SPI_FIFOWR_EOF_SHIFT                     (21U)
#define SPI_FIFOWR_EOF(x)                        (((uint32_t)(((uint32_t)(x)) << SPI_FIFOWR_EOF_SHIFT)) & SPI_FIFOWR_EOF_MASK)
#define SPI_FIFOWR_RXIGNORE_MASK                 (0x400000U)
#define SPI_FIFOWR_RXIGNORE_SHIFT                (22U)
#define SPI_FIFOWR_RXIGNORE(x)                   (((uint32_t)(((uint32_t)(x)) << SPI_FIFOWR_RXIGNORE_SHIFT)) & SPI_FIFOWR_RXIGNORE_MASK)
#define SPI_FIFOWR_LEN_MASK                      (0xF000000U)
#define SPI_FIFOWR_LEN_SHIFT                     (24U)
#define SPI_FIFOWR_LEN(x)                        (((uint32_t)(((uint32_t)(x)) << SPI_FIFOWR_LEN_SHIFT)) & SPI_FIFOWR_LEN_MASK)

/*! @name FIFORD - FIFO read data. */
#define SPI_FIFORD_RXDATA_MASK                   (0xFFFFU)
#define SPI_FIFORD_RXDATA_SHIFT                  (0U)
#define SPI_FIFORD_RXDATA(x)                     (((uint32_t)(((uint32_t)(x)) << SPI_FIFORD_RXDATA_SHIFT)) & SPI_FIFORD_RXDATA_MASK)
#define SPI_FIFORD_RXSSEL0_N_MASK                (0x10000U)
#define SPI_FIFORD_RXSSEL0_N_SHIFT               (16U)
#define SPI_FIFORD_RXSSEL0_N(x)                  (((uint32_t)(((uint32_t)(x)) << SPI_FIFORD_RXSSEL0_N_SHIFT)) & SPI_FIFORD_RXSSEL0_N_MASK)
#define SPI_FIFORD_RXSSEL1_N_MASK                (0x20000U)
#define SPI_FIFORD_RXSSEL1_N_SHIFT               (17U)
#define SPI_FIFORD_RXSSEL1_N(x)                  (((uint32_t)(((uint32_t)(x)) << SPI_FIFORD_RXSSEL1_N_SHIFT)) & SPI_FIFORD_RXSSEL1_N_MASK)
#define SPI_FIFORD_RXSSEL2_N_MASK                (0x40000U)
#define SPI_FIFORD_RXSSEL2_N_SHIFT               (18U)
#define SPI_FIFORD_RXSSEL2_N(x)                  (((uint32_t)(((uint32_t)(x)) << SPI_FIFORD_RXSSEL2_N_SHIFT)) & SPI_FIFORD_RXSSEL2_N_MASK)
#define SPI_FIFORD_RXSSEL3_N_MASK                (0x80000U)
#define SPI_FIFORD_RXSSEL3_N_SHIFT               (19U)
#define SPI_FIFORD_RXSSEL3_N(x)                  (((uint32_t)(((uint32_t)(x)) << SPI_FIFORD_RXSSEL3_N_SHIFT)) & SPI_FIFORD_RXSSEL3_N_MASK)
#define SPI_FIFORD_SOT_MASK                      (0x100000U)
#define SPI_FIFORD_SOT_SHIFT                     (20U)
#define SPI_FIFORD_SOT(x)                        (((uint32_t)(((uint32_t)(x)) << SPI_FIFORD_SOT_SHIFT)) & SPI_FIFORD_SOT_MASK)

/*! @name FIFORDNOPOP - FIFO data read with no FIFO pop. */
#define SPI_FIFORDNOPOP_RXDATA_MASK              (0xFFFFU)
#define SPI_FIFORDNOPOP_RXDATA_SHIFT             (0U)
#define SPI_FIFORDNOPOP_RXDATA(x)                (((uint32_t)(((uint32_t)(x)) << SPI_FIFORDNOPOP_RXDATA_SHIFT)) & SPI_FIFORDNOPOP_RXDATA_MASK)
#define SPI_FIFORDNOPOP_RXSSEL0_N_MASK           (0x10000U)
#define SPI_FIFORDNOPOP_RXSSEL0_N_SHIFT          (16U)
#define SPI_FIFORDNOPOP_RXSSEL0_N(x)             (((uint32_t)(((uint32_t)(x)) << SPI_FIFORDNOPOP_RXSSEL0_N_SHIFT)) & SPI_FIFORDNOPOP_RXSSEL0_N_MASK)
#define SPI_FIFORDNOPOP_RXSSEL1_N_MASK           (0x20000U)
#define SPI_FIFORDNOPOP_RXSSEL1_N_SHIFT          (17U)
#define SPI_FIFORDNOPOP_RXSSEL1_N(x)             (((uint32_t)(((uint32_t)(x)) << SPI_FIFORDNOPOP_RXSSEL1_N_SHIFT)) & SPI_FIFORDNOPOP_RXSSEL1_N_MASK)
#define SPI_FIFORDNOPOP_RXSSEL2_N_MASK           (0x40000U)
#define SPI_FIFORDNOPOP_RXSSEL2_N_SHIFT          (18U)
#define SPI_FIFORDNOPOP_RXSSEL2_N(x)             (((uint32_t)(((uint32_t)(x)) << SPI_FIFORDNOPOP_RXSSEL2_N_SHIFT)) & SPI_FIFORDNOPOP_RXSSEL2_N_MASK)
#define SPI_FIFORDNOPOP_RXSSEL3_N_MASK           (0x80000U)
#define SPI_FIFORDNOPOP_RXSSEL3_N_SHIFT          (19U)
#define SPI_FIFORDNOPOP_RXSSEL3_N(x)             (((uint32_t)(((uint32_t)(x)) << SPI_FIFORDNOPOP_RXSSEL3_N_SHIFT)) & SPI_FIFORDNOPOP_RXSSEL3_N_MASK)
#define SPI_FIFORDNOPOP_SOT_MASK                 (0x100000U)
#define SPI_FIFORDNOPOP_SOT_SHIFT                (20U)
#define SPI_FIFORDNOPOP_SOT(x)                   (((uint32_t)(((uint32_t)(x)) << SPI_FIFORDNOPOP_SOT_SHIFT)) & SPI_FIFORDNOPOP_SOT_MASK)

/*! @name ID - Peripheral identification register. */
#define SPI_ID_APERTURE_MASK                     (0xFFU)
#define SPI_ID_APERTURE_SHIFT                    (0U)
#define SPI_ID_APERTURE(x)                       (((uint32_t)(((uint32_t)(x)) << SPI_ID_APERTURE_SHIFT)) & SPI_ID_APERTURE_MASK)
#define SPI_ID_MINOR_REV_MASK                    (0xF00U)
#define SPI_ID_MINOR_REV_SHIFT                   (8U)
#define SPI_ID_MINOR_REV(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_ID_MINOR_REV_SHIFT)) & SPI_ID_MINOR_REV_MASK)
#define SPI_ID_MAJOR_REV_MASK                    (0xF000U)
#define SPI_ID_MAJOR_REV_SHIFT                   (12U)
#define SPI_ID_MAJOR_REV(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_ID_MAJOR_REV_SHIFT)) & SPI_ID_MAJOR_REV_MASK)
#define SPI_ID_ID_MASK                           (0xFFFF0000U)
#define SPI_ID_ID_SHIFT                          (16U)
#define SPI_ID_ID(x)                             (((uint32_t)(((uint32_t)(x)) << SPI_ID_ID_SHIFT)) & SPI_ID_ID_MASK)


/*!
 * @}
 */ /* end of group SPI_Register_Masks */


/* SPI - Peripheral instance base addresses */
/** Peripheral SPI0 base address */
#define SPI0_BASE                                (0x40086000u)
/** Peripheral SPI0 base pointer */
#define SPI0                                     ((SPI_Type *)SPI0_BASE)
/** Peripheral SPI1 base address */
#define SPI1_BASE                                (0x40087000u)
/** Peripheral SPI1 base pointer */
#define SPI1                                     ((SPI_Type *)SPI1_BASE)
/** Peripheral SPI2 base address */
#define SPI2_BASE                                (0x40088000u)
/** Peripheral SPI2 base pointer */
#define SPI2                                     ((SPI_Type *)SPI2_BASE)
/** Peripheral SPI3 base address */
#define SPI3_BASE                                (0x40089000u)
/** Peripheral SPI3 base pointer */
#define SPI3                                     ((SPI_Type *)SPI3_BASE)
/** Peripheral SPI4 base address */
#define SPI4_BASE                                (0x4008A000u)
/** Peripheral SPI4 base pointer */
#define SPI4                                     ((SPI_Type *)SPI4_BASE)
/** Peripheral SPI5 base address */
#define SPI5_BASE                                (0x40096000u)
/** Peripheral SPI5 base pointer */
#define SPI5                                     ((SPI_Type *)SPI5_BASE)
/** Peripheral SPI6 base address */
#define SPI6_BASE                                (0x40097000u)
/** Peripheral SPI6 base pointer */
#define SPI6                                     ((SPI_Type *)SPI6_BASE)
/** Peripheral SPI7 base address */
#define SPI7_BASE                                (0x40098000u)
/** Peripheral SPI7 base pointer */
#define SPI7                                     ((SPI_Type *)SPI7_BASE)
/** Array initializer of SPI peripheral base addresses */
#define SPI_BASE_ADDRS                           { SPI0_BASE, SPI1_BASE, SPI2_BASE, SPI3_BASE, SPI4_BASE, SPI5_BASE, SPI6_BASE, SPI7_BASE }
/** Array initializer of SPI peripheral base pointers */
#define SPI_BASE_PTRS                            { SPI0, SPI1, SPI2, SPI3, SPI4, SPI5, SPI6, SPI7 }
/** Interrupt vectors for the SPI peripheral type */
#define SPI_IRQS                                 { FLEXCOMM0_IRQn, FLEXCOMM1_IRQn, FLEXCOMM2_IRQn, FLEXCOMM3_IRQn, FLEXCOMM4_IRQn, FLEXCOMM5_IRQn, FLEXCOMM6_IRQn, FLEXCOMM7_IRQn }

/*!
 * @}
 */ /* end of group SPI_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SYSCON Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SYSCON_Peripheral_Access_Layer SYSCON Peripheral Access Layer
 * @{
 */

/** SYSCON - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[16];
  __IO uint32_t AHBMATPRIO;                        /**< AHB multilayer matrix priority control, offset: 0x10 */
       uint8_t RESERVED_1[44];
  __IO uint32_t SYSTCKCAL;                         /**< System tick counter calibration, offset: 0x40 */
       uint8_t RESERVED_2[4];
  __IO uint32_t NMISRC;                            /**< NMI Source Select, offset: 0x48 */
  __IO uint32_t ASYNCAPBCTRL;                      /**< Asynchronous APB Control, offset: 0x4C */
       uint8_t RESERVED_3[112];
  __I  uint32_t PIOPORCAP[2];                      /**< POR captured value of port n, array offset: 0xC0, array step: 0x4 */
       uint8_t RESERVED_4[8];
  __I  uint32_t PIORESCAP[2];                      /**< Reset captured value of port n, array offset: 0xD0, array step: 0x4 */
       uint8_t RESERVED_5[40];
  __IO uint32_t PRESETCTRL[2];                     /**< Peripheral reset control n, array offset: 0x100, array step: 0x4 */
       uint8_t RESERVED_6[24];
  __O  uint32_t PRESETCTRLSET[2];                  /**< Set bits in PRESETCTRLn, array offset: 0x120, array step: 0x4 */
       uint8_t RESERVED_7[24];
  __O  uint32_t PRESETCTRLCLR[2];                  /**< Clear bits in PRESETCTRLn, array offset: 0x140, array step: 0x4 */
       uint8_t RESERVED_8[168];
  __IO uint32_t SYSRSTSTAT;                        /**< System reset status register, offset: 0x1F0 */
       uint8_t RESERVED_9[12];
  __IO uint32_t AHBCLKCTRL[2];                     /**< AHB Clock control n, array offset: 0x200, array step: 0x4 */
       uint8_t RESERVED_10[24];
  __O  uint32_t AHBCLKCTRLSET[2];                  /**< Set bits in AHBCLKCTRLn, array offset: 0x220, array step: 0x4 */
       uint8_t RESERVED_11[24];
  __O  uint32_t AHBCLKCTRLCLR[2];                  /**< Clear bits in AHBCLKCTRLn, array offset: 0x240, array step: 0x4 */
       uint8_t RESERVED_12[56];
  __IO uint32_t MAINCLKSELA;                       /**< Main clock source select A, offset: 0x280 */
  __IO uint32_t MAINCLKSELB;                       /**< Main clock source select B, offset: 0x284 */
  __IO uint32_t CLKOUTSELA;                        /**< CLKOUT clock source select A, offset: 0x288 */
       uint8_t RESERVED_13[4];
  __IO uint32_t SYSPLLCLKSEL;                      /**< PLL clock source select, offset: 0x290 */
       uint8_t RESERVED_14[16];
  __IO uint32_t ADCCLKSEL;                         /**< ADC clock source select, offset: 0x2A4 */
  __IO uint32_t USBCLKSEL;                         /**< USB clock source select, offset: 0x2A8 */
       uint8_t RESERVED_15[4];
  __IO uint32_t FXCOMCLKSEL[8];                    /**< Flexcomm0 clock source select..Flexcomm7 clock source select, array offset: 0x2B0, array step: 0x4 */
       uint8_t RESERVED_16[16];
  __IO uint32_t MCLKCLKSEL;                        /**< MCLK clock source select, offset: 0x2E0 */
       uint8_t RESERVED_17[4];
  __IO uint32_t FRGCLKSEL;                         /**< Fractional Rate Generator clock source select, offset: 0x2E8 */
       uint8_t RESERVED_18[20];
  __IO uint32_t SYSTICKCLKDIV;                     /**< SYSTICK clock divider, offset: 0x300 */
       uint8_t RESERVED_19[124];
  __IO uint32_t AHBCLKDIV;                         /**< AHB clock divider, offset: 0x380 */
  __IO uint32_t CLKOUTDIV;                         /**< CLKOUT clock divider, offset: 0x384 */
       uint8_t RESERVED_20[12];
  __IO uint32_t ADCCLKDIV;                         /**< ADC clock divider, offset: 0x394 */
  __IO uint32_t USBCLKDIV;                         /**< USB clock divider, offset: 0x398 */
       uint8_t RESERVED_21[4];
  __IO uint32_t FRGCTRL;                           /**< Fractional rate divider, offset: 0x3A0 */
       uint8_t RESERVED_22[8];
  __IO uint32_t MCLKDIV;                           /**< I2S MCLK clock divider, offset: 0x3AC */
       uint8_t RESERVED_23[80];
  __IO uint32_t FLASHCFG;                          /**< Flash wait states configuration, offset: 0x400 */
       uint8_t RESERVED_24[8];
  __IO uint32_t USBCLKCTRL;                        /**< USB clock control, offset: 0x40C */
  __IO uint32_t USBCLKSTAT;                        /**< USB clock status, offset: 0x410 */
       uint8_t RESERVED_25[4];
  __IO uint32_t FREQMECTRL;                        /**< Frequency measure register, offset: 0x418 */
       uint8_t RESERVED_26[4];
  __IO uint32_t MCLKIO;                            /**< MCLK input/output control, offset: 0x420 */
       uint8_t RESERVED_27[220];
  __IO uint32_t FROCTRL;                           /**< FRO oscillator control, offset: 0x500 */
       uint8_t RESERVED_28[4];
  __IO uint32_t WDTOSCCTRL;                        /**< Watchdog oscillator control, offset: 0x508 */
  __IO uint32_t RTCOSCCTRL;                        /**< RTC oscillator 32 kHz output control, offset: 0x50C */
       uint8_t RESERVED_29[112];
  __IO uint32_t SYSPLLCTRL;                        /**< PLL control, offset: 0x580 */
  __I  uint32_t SYSPLLSTAT;                        /**< PLL status, offset: 0x584 */
  __IO uint32_t SYSPLLNDEC;                        /**< PLL N decoder, offset: 0x588 */
  __IO uint32_t SYSPLLPDEC;                        /**< PLL P decoder, offset: 0x58C */
  __IO uint32_t SYSPLLSSCTRL0;                     /**< PLL spread spectrum control 0, offset: 0x590 */
  __IO uint32_t SYSPLLSSCTRL1;                     /**< PLL spread spectrum control 1, offset: 0x594 */
       uint8_t RESERVED_30[104];
  __IO uint32_t PDSLEEPCFG[2];                     /**< Sleep configuration register n, array offset: 0x600, array step: 0x4 */
       uint8_t RESERVED_31[8];
  __IO uint32_t PDRUNCFG[1];                       /**< Power configuration register 0, array offset: 0x610, array step: 0x4 */
       uint8_t RESERVED_32[12];
  __O  uint32_t PDRUNCFGSET[1];                    /**< Set bits in PDRUNCFGn, array offset: 0x620, array step: 0x4 */
       uint8_t RESERVED_33[12];
  __O  uint32_t PDRUNCFGCLR[1];                    /**< Clear bits in PDRUNCFGn, array offset: 0x630, array step: 0x4 */
       uint8_t RESERVED_34[76];
  __IO uint32_t STARTER[1];                        /**< Start logic n wake-up enable register, array offset: 0x680, array step: 0x4 */
       uint8_t RESERVED_35[28];
  __O  uint32_t STARTERSET[1];                     /**< Set bits in STARTERn, array offset: 0x6A0, array step: 0x4 */
       uint8_t RESERVED_36[28];
  __O  uint32_t STARTERCLR[1];                     /**< Clear bits in STARTERn, array offset: 0x6C0, array step: 0x4 */
       uint8_t RESERVED_37[188];
  __IO uint32_t HWWAKE;                            /**< Configures special cases of hardware wake-up, offset: 0x780 */
       uint8_t RESERVED_38[2160];
  __I  uint32_t JTAGIDCODE;                        /**< JTAG ID code register, offset: 0xFF4 */
  __I  uint32_t DEVICE_ID0;                        /**< Part ID register, offset: 0xFF8 */
  __I  uint32_t DEVICE_ID1;                        /**< Boot ROM and die revision register, offset: 0xFFC */
       uint8_t RESERVED_39[127044];
  __IO uint32_t BODCTRL;                           /**< Brown-Out Detect control, offset: 0x20044 */
} SYSCON_Type;

/* ----------------------------------------------------------------------------
   -- SYSCON Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SYSCON_Register_Masks SYSCON Register Masks
 * @{
 */

/*! @name AHBMATPRIO - AHB multilayer matrix priority control */
#define SYSCON_AHBMATPRIO_PRI_ICODE_MASK         (0x3U)
#define SYSCON_AHBMATPRIO_PRI_ICODE_SHIFT        (0U)
#define SYSCON_AHBMATPRIO_PRI_ICODE(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBMATPRIO_PRI_ICODE_SHIFT)) & SYSCON_AHBMATPRIO_PRI_ICODE_MASK)
#define SYSCON_AHBMATPRIO_PRI_DCODE_MASK         (0xCU)
#define SYSCON_AHBMATPRIO_PRI_DCODE_SHIFT        (2U)
#define SYSCON_AHBMATPRIO_PRI_DCODE(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBMATPRIO_PRI_DCODE_SHIFT)) & SYSCON_AHBMATPRIO_PRI_DCODE_MASK)
#define SYSCON_AHBMATPRIO_PRI_SYS_MASK           (0x30U)
#define SYSCON_AHBMATPRIO_PRI_SYS_SHIFT          (4U)
#define SYSCON_AHBMATPRIO_PRI_SYS(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBMATPRIO_PRI_SYS_SHIFT)) & SYSCON_AHBMATPRIO_PRI_SYS_MASK)
#define SYSCON_AHBMATPRIO_PRI_M0_MASK            (0xC0U)
#define SYSCON_AHBMATPRIO_PRI_M0_SHIFT           (6U)
#define SYSCON_AHBMATPRIO_PRI_M0(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBMATPRIO_PRI_M0_SHIFT)) & SYSCON_AHBMATPRIO_PRI_M0_MASK)
#define SYSCON_AHBMATPRIO_PRI_USB_MASK           (0x300U)
#define SYSCON_AHBMATPRIO_PRI_USB_SHIFT          (8U)
#define SYSCON_AHBMATPRIO_PRI_USB(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBMATPRIO_PRI_USB_SHIFT)) & SYSCON_AHBMATPRIO_PRI_USB_MASK)
#define SYSCON_AHBMATPRIO_PRI_DMA_MASK           (0xC00U)
#define SYSCON_AHBMATPRIO_PRI_DMA_SHIFT          (10U)
#define SYSCON_AHBMATPRIO_PRI_DMA(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBMATPRIO_PRI_DMA_SHIFT)) & SYSCON_AHBMATPRIO_PRI_DMA_MASK)

/*! @name SYSTCKCAL - System tick counter calibration */
#define SYSCON_SYSTCKCAL_CAL_MASK                (0xFFFFFFU)
#define SYSCON_SYSTCKCAL_CAL_SHIFT               (0U)
#define SYSCON_SYSTCKCAL_CAL(x)                  (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSTCKCAL_CAL_SHIFT)) & SYSCON_SYSTCKCAL_CAL_MASK)
#define SYSCON_SYSTCKCAL_SKEW_MASK               (0x1000000U)
#define SYSCON_SYSTCKCAL_SKEW_SHIFT              (24U)
#define SYSCON_SYSTCKCAL_SKEW(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSTCKCAL_SKEW_SHIFT)) & SYSCON_SYSTCKCAL_SKEW_MASK)
#define SYSCON_SYSTCKCAL_NOREF_MASK              (0x2000000U)
#define SYSCON_SYSTCKCAL_NOREF_SHIFT             (25U)
#define SYSCON_SYSTCKCAL_NOREF(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSTCKCAL_NOREF_SHIFT)) & SYSCON_SYSTCKCAL_NOREF_MASK)

/*! @name NMISRC - NMI Source Select */
#define SYSCON_NMISRC_IRQM4_MASK                 (0x3FU)
#define SYSCON_NMISRC_IRQM4_SHIFT                (0U)
#define SYSCON_NMISRC_IRQM4(x)                   (((uint32_t)(((uint32_t)(x)) << SYSCON_NMISRC_IRQM4_SHIFT)) & SYSCON_NMISRC_IRQM4_MASK)
#define SYSCON_NMISRC_IRQM0_MASK                 (0x3F00U)
#define SYSCON_NMISRC_IRQM0_SHIFT                (8U)
#define SYSCON_NMISRC_IRQM0(x)                   (((uint32_t)(((uint32_t)(x)) << SYSCON_NMISRC_IRQM0_SHIFT)) & SYSCON_NMISRC_IRQM0_MASK)
#define SYSCON_NMISRC_NMIENM0_MASK               (0x40000000U)
#define SYSCON_NMISRC_NMIENM0_SHIFT              (30U)
#define SYSCON_NMISRC_NMIENM0(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_NMISRC_NMIENM0_SHIFT)) & SYSCON_NMISRC_NMIENM0_MASK)
#define SYSCON_NMISRC_NMIENM4_MASK               (0x80000000U)
#define SYSCON_NMISRC_NMIENM4_SHIFT              (31U)
#define SYSCON_NMISRC_NMIENM4(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_NMISRC_NMIENM4_SHIFT)) & SYSCON_NMISRC_NMIENM4_MASK)

/*! @name ASYNCAPBCTRL - Asynchronous APB Control */
#define SYSCON_ASYNCAPBCTRL_ENABLE_MASK          (0x1U)
#define SYSCON_ASYNCAPBCTRL_ENABLE_SHIFT         (0U)
#define SYSCON_ASYNCAPBCTRL_ENABLE(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_ASYNCAPBCTRL_ENABLE_SHIFT)) & SYSCON_ASYNCAPBCTRL_ENABLE_MASK)

/*! @name PIOPORCAP - POR captured value of port n */
#define SYSCON_PIOPORCAP_PIOPORCAP_MASK          (0xFFFFFFFFU)
#define SYSCON_PIOPORCAP_PIOPORCAP_SHIFT         (0U)
#define SYSCON_PIOPORCAP_PIOPORCAP(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_PIOPORCAP_PIOPORCAP_SHIFT)) & SYSCON_PIOPORCAP_PIOPORCAP_MASK)

/* The count of SYSCON_PIOPORCAP */
#define SYSCON_PIOPORCAP_COUNT                   (2U)

/*! @name PIORESCAP - Reset captured value of port n */
#define SYSCON_PIORESCAP_PIORESCAP_MASK          (0xFFFFFFFFU)
#define SYSCON_PIORESCAP_PIORESCAP_SHIFT         (0U)
#define SYSCON_PIORESCAP_PIORESCAP(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_PIORESCAP_PIORESCAP_SHIFT)) & SYSCON_PIORESCAP_PIORESCAP_MASK)

/* The count of SYSCON_PIORESCAP */
#define SYSCON_PIORESCAP_COUNT                   (2U)

/*! @name PRESETCTRL - Peripheral reset control n */
#define SYSCON_PRESETCTRL_MRT0_RST_MASK          (0x1U)
#define SYSCON_PRESETCTRL_MRT0_RST_SHIFT         (0U)
#define SYSCON_PRESETCTRL_MRT0_RST(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_MRT0_RST_SHIFT)) & SYSCON_PRESETCTRL_MRT0_RST_MASK)
#define SYSCON_PRESETCTRL_SCT0_RST_MASK          (0x4U)
#define SYSCON_PRESETCTRL_SCT0_RST_SHIFT         (2U)
#define SYSCON_PRESETCTRL_SCT0_RST(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_SCT0_RST_SHIFT)) & SYSCON_PRESETCTRL_SCT0_RST_MASK)
#define SYSCON_PRESETCTRL_FLASH_RST_MASK         (0x80U)
#define SYSCON_PRESETCTRL_FLASH_RST_SHIFT        (7U)
#define SYSCON_PRESETCTRL_FLASH_RST(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_FLASH_RST_SHIFT)) & SYSCON_PRESETCTRL_FLASH_RST_MASK)
#define SYSCON_PRESETCTRL_FMC_RST_MASK           (0x100U)
#define SYSCON_PRESETCTRL_FMC_RST_SHIFT          (8U)
#define SYSCON_PRESETCTRL_FMC_RST(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_FMC_RST_SHIFT)) & SYSCON_PRESETCTRL_FMC_RST_MASK)
#define SYSCON_PRESETCTRL_UTICK0_RST_MASK        (0x400U)
#define SYSCON_PRESETCTRL_UTICK0_RST_SHIFT       (10U)
#define SYSCON_PRESETCTRL_UTICK0_RST(x)          (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_UTICK0_RST_SHIFT)) & SYSCON_PRESETCTRL_UTICK0_RST_MASK)
#define SYSCON_PRESETCTRL_FC0_RST_MASK           (0x800U)
#define SYSCON_PRESETCTRL_FC0_RST_SHIFT          (11U)
#define SYSCON_PRESETCTRL_FC0_RST(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_FC0_RST_SHIFT)) & SYSCON_PRESETCTRL_FC0_RST_MASK)
#define SYSCON_PRESETCTRL_MUX_RST_MASK           (0x800U)
#define SYSCON_PRESETCTRL_MUX_RST_SHIFT          (11U)
#define SYSCON_PRESETCTRL_MUX_RST(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_MUX_RST_SHIFT)) & SYSCON_PRESETCTRL_MUX_RST_MASK)
#define SYSCON_PRESETCTRL_FC1_RST_MASK           (0x1000U)
#define SYSCON_PRESETCTRL_FC1_RST_SHIFT          (12U)
#define SYSCON_PRESETCTRL_FC1_RST(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_FC1_RST_SHIFT)) & SYSCON_PRESETCTRL_FC1_RST_MASK)
#define SYSCON_PRESETCTRL_FC2_RST_MASK           (0x2000U)
#define SYSCON_PRESETCTRL_FC2_RST_SHIFT          (13U)
#define SYSCON_PRESETCTRL_FC2_RST(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_FC2_RST_SHIFT)) & SYSCON_PRESETCTRL_FC2_RST_MASK)
#define SYSCON_PRESETCTRL_IOCON_RST_MASK         (0x2000U)
#define SYSCON_PRESETCTRL_IOCON_RST_SHIFT        (13U)
#define SYSCON_PRESETCTRL_IOCON_RST(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_IOCON_RST_SHIFT)) & SYSCON_PRESETCTRL_IOCON_RST_MASK)
#define SYSCON_PRESETCTRL_GPIO0_RST_MASK         (0x4000U)
#define SYSCON_PRESETCTRL_GPIO0_RST_SHIFT        (14U)
#define SYSCON_PRESETCTRL_GPIO0_RST(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_GPIO0_RST_SHIFT)) & SYSCON_PRESETCTRL_GPIO0_RST_MASK)
#define SYSCON_PRESETCTRL_FC3_RST_MASK           (0x4000U)
#define SYSCON_PRESETCTRL_FC3_RST_SHIFT          (14U)
#define SYSCON_PRESETCTRL_FC3_RST(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_FC3_RST_SHIFT)) & SYSCON_PRESETCTRL_FC3_RST_MASK)
#define SYSCON_PRESETCTRL_FC4_RST_MASK           (0x8000U)
#define SYSCON_PRESETCTRL_FC4_RST_SHIFT          (15U)
#define SYSCON_PRESETCTRL_FC4_RST(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_FC4_RST_SHIFT)) & SYSCON_PRESETCTRL_FC4_RST_MASK)
#define SYSCON_PRESETCTRL_GPIO1_RST_MASK         (0x8000U)
#define SYSCON_PRESETCTRL_GPIO1_RST_SHIFT        (15U)
#define SYSCON_PRESETCTRL_GPIO1_RST(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_GPIO1_RST_SHIFT)) & SYSCON_PRESETCTRL_GPIO1_RST_MASK)
#define SYSCON_PRESETCTRL_FC5_RST_MASK           (0x10000U)
#define SYSCON_PRESETCTRL_FC5_RST_SHIFT          (16U)
#define SYSCON_PRESETCTRL_FC5_RST(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_FC5_RST_SHIFT)) & SYSCON_PRESETCTRL_FC5_RST_MASK)
#define SYSCON_PRESETCTRL_FC6_RST_MASK           (0x20000U)
#define SYSCON_PRESETCTRL_FC6_RST_SHIFT          (17U)
#define SYSCON_PRESETCTRL_FC6_RST(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_FC6_RST_SHIFT)) & SYSCON_PRESETCTRL_FC6_RST_MASK)
#define SYSCON_PRESETCTRL_PINT_RST_MASK          (0x40000U)
#define SYSCON_PRESETCTRL_PINT_RST_SHIFT         (18U)
#define SYSCON_PRESETCTRL_PINT_RST(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_PINT_RST_SHIFT)) & SYSCON_PRESETCTRL_PINT_RST_MASK)
#define SYSCON_PRESETCTRL_FC7_RST_MASK           (0x40000U)
#define SYSCON_PRESETCTRL_FC7_RST_SHIFT          (18U)
#define SYSCON_PRESETCTRL_FC7_RST(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_FC7_RST_SHIFT)) & SYSCON_PRESETCTRL_FC7_RST_MASK)
#define SYSCON_PRESETCTRL_GINT_RST_MASK          (0x80000U)
#define SYSCON_PRESETCTRL_GINT_RST_SHIFT         (19U)
#define SYSCON_PRESETCTRL_GINT_RST(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_GINT_RST_SHIFT)) & SYSCON_PRESETCTRL_GINT_RST_MASK)
#define SYSCON_PRESETCTRL_DMA0_RST_MASK          (0x100000U)
#define SYSCON_PRESETCTRL_DMA0_RST_SHIFT         (20U)
#define SYSCON_PRESETCTRL_DMA0_RST(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_DMA0_RST_SHIFT)) & SYSCON_PRESETCTRL_DMA0_RST_MASK)
#define SYSCON_PRESETCTRL_CRC_RST_MASK           (0x200000U)
#define SYSCON_PRESETCTRL_CRC_RST_SHIFT          (21U)
#define SYSCON_PRESETCTRL_CRC_RST(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_CRC_RST_SHIFT)) & SYSCON_PRESETCTRL_CRC_RST_MASK)
#define SYSCON_PRESETCTRL_WWDT_RST_MASK          (0x400000U)
#define SYSCON_PRESETCTRL_WWDT_RST_SHIFT         (22U)
#define SYSCON_PRESETCTRL_WWDT_RST(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_WWDT_RST_SHIFT)) & SYSCON_PRESETCTRL_WWDT_RST_MASK)
#define SYSCON_PRESETCTRL_USB0_RST_MASK          (0x2000000U)
#define SYSCON_PRESETCTRL_USB0_RST_SHIFT         (25U)
#define SYSCON_PRESETCTRL_USB0_RST(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_USB0_RST_SHIFT)) & SYSCON_PRESETCTRL_USB0_RST_MASK)
#define SYSCON_PRESETCTRL_CTIMER0_RST_MASK       (0x4000000U)
#define SYSCON_PRESETCTRL_CTIMER0_RST_SHIFT      (26U)
#define SYSCON_PRESETCTRL_CTIMER0_RST(x)         (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_CTIMER0_RST_SHIFT)) & SYSCON_PRESETCTRL_CTIMER0_RST_MASK)
#define SYSCON_PRESETCTRL_CTIMER1_RST_MASK       (0x8000000U)
#define SYSCON_PRESETCTRL_CTIMER1_RST_SHIFT      (27U)
#define SYSCON_PRESETCTRL_CTIMER1_RST(x)         (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_CTIMER1_RST_SHIFT)) & SYSCON_PRESETCTRL_CTIMER1_RST_MASK)
#define SYSCON_PRESETCTRL_ADC0_RST_MASK          (0x8000000U)
#define SYSCON_PRESETCTRL_ADC0_RST_SHIFT         (27U)
#define SYSCON_PRESETCTRL_ADC0_RST(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRL_ADC0_RST_SHIFT)) & SYSCON_PRESETCTRL_ADC0_RST_MASK)

/* The count of SYSCON_PRESETCTRL */
#define SYSCON_PRESETCTRL_COUNT                  (2U)

/*! @name PRESETCTRLSET - Set bits in PRESETCTRLn */
#define SYSCON_PRESETCTRLSET_RST_SET_MASK        (0xFFFFFFFFU)
#define SYSCON_PRESETCTRLSET_RST_SET_SHIFT       (0U)
#define SYSCON_PRESETCTRLSET_RST_SET(x)          (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRLSET_RST_SET_SHIFT)) & SYSCON_PRESETCTRLSET_RST_SET_MASK)

/* The count of SYSCON_PRESETCTRLSET */
#define SYSCON_PRESETCTRLSET_COUNT               (2U)

/*! @name PRESETCTRLCLR - Clear bits in PRESETCTRLn */
#define SYSCON_PRESETCTRLCLR_RST_CLR_MASK        (0xFFFFFFFFU)
#define SYSCON_PRESETCTRLCLR_RST_CLR_SHIFT       (0U)
#define SYSCON_PRESETCTRLCLR_RST_CLR(x)          (((uint32_t)(((uint32_t)(x)) << SYSCON_PRESETCTRLCLR_RST_CLR_SHIFT)) & SYSCON_PRESETCTRLCLR_RST_CLR_MASK)

/* The count of SYSCON_PRESETCTRLCLR */
#define SYSCON_PRESETCTRLCLR_COUNT               (2U)

/*! @name SYSRSTSTAT - System reset status register */
#define SYSCON_SYSRSTSTAT_POR_MASK               (0x1U)
#define SYSCON_SYSRSTSTAT_POR_SHIFT              (0U)
#define SYSCON_SYSRSTSTAT_POR(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSRSTSTAT_POR_SHIFT)) & SYSCON_SYSRSTSTAT_POR_MASK)
#define SYSCON_SYSRSTSTAT_EXTRST_MASK            (0x2U)
#define SYSCON_SYSRSTSTAT_EXTRST_SHIFT           (1U)
#define SYSCON_SYSRSTSTAT_EXTRST(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSRSTSTAT_EXTRST_SHIFT)) & SYSCON_SYSRSTSTAT_EXTRST_MASK)
#define SYSCON_SYSRSTSTAT_WDT_MASK               (0x4U)
#define SYSCON_SYSRSTSTAT_WDT_SHIFT              (2U)
#define SYSCON_SYSRSTSTAT_WDT(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSRSTSTAT_WDT_SHIFT)) & SYSCON_SYSRSTSTAT_WDT_MASK)
#define SYSCON_SYSRSTSTAT_BOD_MASK               (0x8U)
#define SYSCON_SYSRSTSTAT_BOD_SHIFT              (3U)
#define SYSCON_SYSRSTSTAT_BOD(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSRSTSTAT_BOD_SHIFT)) & SYSCON_SYSRSTSTAT_BOD_MASK)
#define SYSCON_SYSRSTSTAT_SYSRST_MASK            (0x10U)
#define SYSCON_SYSRSTSTAT_SYSRST_SHIFT           (4U)
#define SYSCON_SYSRSTSTAT_SYSRST(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSRSTSTAT_SYSRST_SHIFT)) & SYSCON_SYSRSTSTAT_SYSRST_MASK)

/*! @name AHBCLKCTRL - AHB Clock control n */
#define SYSCON_AHBCLKCTRL_MRT0_MASK              (0x1U)
#define SYSCON_AHBCLKCTRL_MRT0_SHIFT             (0U)
#define SYSCON_AHBCLKCTRL_MRT0(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_MRT0_SHIFT)) & SYSCON_AHBCLKCTRL_MRT0_MASK)
#define SYSCON_AHBCLKCTRL_ROM_MASK               (0x2U)
#define SYSCON_AHBCLKCTRL_ROM_SHIFT              (1U)
#define SYSCON_AHBCLKCTRL_ROM(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_ROM_SHIFT)) & SYSCON_AHBCLKCTRL_ROM_MASK)
#define SYSCON_AHBCLKCTRL_SCT0_MASK              (0x4U)
#define SYSCON_AHBCLKCTRL_SCT0_SHIFT             (2U)
#define SYSCON_AHBCLKCTRL_SCT0(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_SCT0_SHIFT)) & SYSCON_AHBCLKCTRL_SCT0_MASK)
#define SYSCON_AHBCLKCTRL_FLASH_MASK             (0x80U)
#define SYSCON_AHBCLKCTRL_FLASH_SHIFT            (7U)
#define SYSCON_AHBCLKCTRL_FLASH(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_FLASH_SHIFT)) & SYSCON_AHBCLKCTRL_FLASH_MASK)
#define SYSCON_AHBCLKCTRL_FMC_MASK               (0x100U)
#define SYSCON_AHBCLKCTRL_FMC_SHIFT              (8U)
#define SYSCON_AHBCLKCTRL_FMC(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_FMC_SHIFT)) & SYSCON_AHBCLKCTRL_FMC_MASK)
#define SYSCON_AHBCLKCTRL_UTICK0_MASK            (0x400U)
#define SYSCON_AHBCLKCTRL_UTICK0_SHIFT           (10U)
#define SYSCON_AHBCLKCTRL_UTICK0(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_UTICK0_SHIFT)) & SYSCON_AHBCLKCTRL_UTICK0_MASK)
#define SYSCON_AHBCLKCTRL_INPUTMUX_MASK          (0x800U)
#define SYSCON_AHBCLKCTRL_INPUTMUX_SHIFT         (11U)
#define SYSCON_AHBCLKCTRL_INPUTMUX(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_INPUTMUX_SHIFT)) & SYSCON_AHBCLKCTRL_INPUTMUX_MASK)
#define SYSCON_AHBCLKCTRL_FLEXCOMM0_MASK         (0x800U)
#define SYSCON_AHBCLKCTRL_FLEXCOMM0_SHIFT        (11U)
#define SYSCON_AHBCLKCTRL_FLEXCOMM0(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_FLEXCOMM0_SHIFT)) & SYSCON_AHBCLKCTRL_FLEXCOMM0_MASK)
#define SYSCON_AHBCLKCTRL_FLEXCOMM1_MASK         (0x1000U)
#define SYSCON_AHBCLKCTRL_FLEXCOMM1_SHIFT        (12U)
#define SYSCON_AHBCLKCTRL_FLEXCOMM1(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_FLEXCOMM1_SHIFT)) & SYSCON_AHBCLKCTRL_FLEXCOMM1_MASK)
#define SYSCON_AHBCLKCTRL_FLEXCOMM2_MASK         (0x2000U)
#define SYSCON_AHBCLKCTRL_FLEXCOMM2_SHIFT        (13U)
#define SYSCON_AHBCLKCTRL_FLEXCOMM2(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_FLEXCOMM2_SHIFT)) & SYSCON_AHBCLKCTRL_FLEXCOMM2_MASK)
#define SYSCON_AHBCLKCTRL_IOCON_MASK             (0x2000U)
#define SYSCON_AHBCLKCTRL_IOCON_SHIFT            (13U)
#define SYSCON_AHBCLKCTRL_IOCON(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_IOCON_SHIFT)) & SYSCON_AHBCLKCTRL_IOCON_MASK)
#define SYSCON_AHBCLKCTRL_FLEXCOMM3_MASK         (0x4000U)
#define SYSCON_AHBCLKCTRL_FLEXCOMM3_SHIFT        (14U)
#define SYSCON_AHBCLKCTRL_FLEXCOMM3(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_FLEXCOMM3_SHIFT)) & SYSCON_AHBCLKCTRL_FLEXCOMM3_MASK)
#define SYSCON_AHBCLKCTRL_GPIO0_MASK             (0x4000U)
#define SYSCON_AHBCLKCTRL_GPIO0_SHIFT            (14U)
#define SYSCON_AHBCLKCTRL_GPIO0(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_GPIO0_SHIFT)) & SYSCON_AHBCLKCTRL_GPIO0_MASK)
#define SYSCON_AHBCLKCTRL_FLEXCOMM4_MASK         (0x8000U)
#define SYSCON_AHBCLKCTRL_FLEXCOMM4_SHIFT        (15U)
#define SYSCON_AHBCLKCTRL_FLEXCOMM4(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_FLEXCOMM4_SHIFT)) & SYSCON_AHBCLKCTRL_FLEXCOMM4_MASK)
#define SYSCON_AHBCLKCTRL_GPIO1_MASK             (0x8000U)
#define SYSCON_AHBCLKCTRL_GPIO1_SHIFT            (15U)
#define SYSCON_AHBCLKCTRL_GPIO1(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_GPIO1_SHIFT)) & SYSCON_AHBCLKCTRL_GPIO1_MASK)
#define SYSCON_AHBCLKCTRL_FLEXCOMM5_MASK         (0x10000U)
#define SYSCON_AHBCLKCTRL_FLEXCOMM5_SHIFT        (16U)
#define SYSCON_AHBCLKCTRL_FLEXCOMM5(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_FLEXCOMM5_SHIFT)) & SYSCON_AHBCLKCTRL_FLEXCOMM5_MASK)
#define SYSCON_AHBCLKCTRL_FLEXCOMM6_MASK         (0x20000U)
#define SYSCON_AHBCLKCTRL_FLEXCOMM6_SHIFT        (17U)
#define SYSCON_AHBCLKCTRL_FLEXCOMM6(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_FLEXCOMM6_SHIFT)) & SYSCON_AHBCLKCTRL_FLEXCOMM6_MASK)
#define SYSCON_AHBCLKCTRL_FLEXCOMM7_MASK         (0x40000U)
#define SYSCON_AHBCLKCTRL_FLEXCOMM7_SHIFT        (18U)
#define SYSCON_AHBCLKCTRL_FLEXCOMM7(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_FLEXCOMM7_SHIFT)) & SYSCON_AHBCLKCTRL_FLEXCOMM7_MASK)
#define SYSCON_AHBCLKCTRL_PINT_MASK              (0x40000U)
#define SYSCON_AHBCLKCTRL_PINT_SHIFT             (18U)
#define SYSCON_AHBCLKCTRL_PINT(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_PINT_SHIFT)) & SYSCON_AHBCLKCTRL_PINT_MASK)
#define SYSCON_AHBCLKCTRL_GINT_MASK              (0x80000U)
#define SYSCON_AHBCLKCTRL_GINT_SHIFT             (19U)
#define SYSCON_AHBCLKCTRL_GINT(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_GINT_SHIFT)) & SYSCON_AHBCLKCTRL_GINT_MASK)
#define SYSCON_AHBCLKCTRL_DMA0_MASK              (0x100000U)
#define SYSCON_AHBCLKCTRL_DMA0_SHIFT             (20U)
#define SYSCON_AHBCLKCTRL_DMA0(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_DMA0_SHIFT)) & SYSCON_AHBCLKCTRL_DMA0_MASK)
#define SYSCON_AHBCLKCTRL_CRC_MASK               (0x200000U)
#define SYSCON_AHBCLKCTRL_CRC_SHIFT              (21U)
#define SYSCON_AHBCLKCTRL_CRC(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_CRC_SHIFT)) & SYSCON_AHBCLKCTRL_CRC_MASK)
#define SYSCON_AHBCLKCTRL_WWDT_MASK              (0x400000U)
#define SYSCON_AHBCLKCTRL_WWDT_SHIFT             (22U)
#define SYSCON_AHBCLKCTRL_WWDT(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_WWDT_SHIFT)) & SYSCON_AHBCLKCTRL_WWDT_MASK)
#define SYSCON_AHBCLKCTRL_RTC_MASK               (0x800000U)
#define SYSCON_AHBCLKCTRL_RTC_SHIFT              (23U)
#define SYSCON_AHBCLKCTRL_RTC(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_RTC_SHIFT)) & SYSCON_AHBCLKCTRL_RTC_MASK)
#define SYSCON_AHBCLKCTRL_USB0_MASK              (0x2000000U)
#define SYSCON_AHBCLKCTRL_USB0_SHIFT             (25U)
#define SYSCON_AHBCLKCTRL_USB0(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_USB0_SHIFT)) & SYSCON_AHBCLKCTRL_USB0_MASK)
#define SYSCON_AHBCLKCTRL_CTIMER0_MASK           (0x4000000U)
#define SYSCON_AHBCLKCTRL_CTIMER0_SHIFT          (26U)
#define SYSCON_AHBCLKCTRL_CTIMER0(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_CTIMER0_SHIFT)) & SYSCON_AHBCLKCTRL_CTIMER0_MASK)
#define SYSCON_AHBCLKCTRL_CTIMER1_MASK           (0x8000000U)
#define SYSCON_AHBCLKCTRL_CTIMER1_SHIFT          (27U)
#define SYSCON_AHBCLKCTRL_CTIMER1(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_CTIMER1_SHIFT)) & SYSCON_AHBCLKCTRL_CTIMER1_MASK)
#define SYSCON_AHBCLKCTRL_ADC0_MASK              (0x8000000U)
#define SYSCON_AHBCLKCTRL_ADC0_SHIFT             (27U)
#define SYSCON_AHBCLKCTRL_ADC0(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRL_ADC0_SHIFT)) & SYSCON_AHBCLKCTRL_ADC0_MASK)

/* The count of SYSCON_AHBCLKCTRL */
#define SYSCON_AHBCLKCTRL_COUNT                  (2U)

/*! @name AHBCLKCTRLSET - Set bits in AHBCLKCTRLn */
#define SYSCON_AHBCLKCTRLSET_CLK_SET_MASK        (0xFFFFFFFFU)
#define SYSCON_AHBCLKCTRLSET_CLK_SET_SHIFT       (0U)
#define SYSCON_AHBCLKCTRLSET_CLK_SET(x)          (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRLSET_CLK_SET_SHIFT)) & SYSCON_AHBCLKCTRLSET_CLK_SET_MASK)

/* The count of SYSCON_AHBCLKCTRLSET */
#define SYSCON_AHBCLKCTRLSET_COUNT               (2U)

/*! @name AHBCLKCTRLCLR - Clear bits in AHBCLKCTRLn */
#define SYSCON_AHBCLKCTRLCLR_CLK_CLR_MASK        (0xFFFFFFFFU)
#define SYSCON_AHBCLKCTRLCLR_CLK_CLR_SHIFT       (0U)
#define SYSCON_AHBCLKCTRLCLR_CLK_CLR(x)          (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKCTRLCLR_CLK_CLR_SHIFT)) & SYSCON_AHBCLKCTRLCLR_CLK_CLR_MASK)

/* The count of SYSCON_AHBCLKCTRLCLR */
#define SYSCON_AHBCLKCTRLCLR_COUNT               (2U)

/*! @name MAINCLKSELA - Main clock source select A */
#define SYSCON_MAINCLKSELA_SEL_MASK              (0x3U)
#define SYSCON_MAINCLKSELA_SEL_SHIFT             (0U)
#define SYSCON_MAINCLKSELA_SEL(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_MAINCLKSELA_SEL_SHIFT)) & SYSCON_MAINCLKSELA_SEL_MASK)

/*! @name MAINCLKSELB - Main clock source select B */
#define SYSCON_MAINCLKSELB_SEL_MASK              (0x3U)
#define SYSCON_MAINCLKSELB_SEL_SHIFT             (0U)
#define SYSCON_MAINCLKSELB_SEL(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_MAINCLKSELB_SEL_SHIFT)) & SYSCON_MAINCLKSELB_SEL_MASK)

/*! @name CLKOUTSELA - CLKOUT clock source select A */
#define SYSCON_CLKOUTSELA_SEL_MASK               (0x7U)
#define SYSCON_CLKOUTSELA_SEL_SHIFT              (0U)
#define SYSCON_CLKOUTSELA_SEL(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_CLKOUTSELA_SEL_SHIFT)) & SYSCON_CLKOUTSELA_SEL_MASK)

/*! @name SYSPLLCLKSEL - PLL clock source select */
#define SYSCON_SYSPLLCLKSEL_SEL_MASK             (0x7U)
#define SYSCON_SYSPLLCLKSEL_SEL_SHIFT            (0U)
#define SYSCON_SYSPLLCLKSEL_SEL(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLCLKSEL_SEL_SHIFT)) & SYSCON_SYSPLLCLKSEL_SEL_MASK)

/*! @name ADCCLKSEL - ADC clock source select */
#define SYSCON_ADCCLKSEL_SEL_MASK                (0x7U)
#define SYSCON_ADCCLKSEL_SEL_SHIFT               (0U)
#define SYSCON_ADCCLKSEL_SEL(x)                  (((uint32_t)(((uint32_t)(x)) << SYSCON_ADCCLKSEL_SEL_SHIFT)) & SYSCON_ADCCLKSEL_SEL_MASK)

/*! @name USBCLKSEL - USB clock source select */
#define SYSCON_USBCLKSEL_SEL_MASK                (0x7U)
#define SYSCON_USBCLKSEL_SEL_SHIFT               (0U)
#define SYSCON_USBCLKSEL_SEL(x)                  (((uint32_t)(((uint32_t)(x)) << SYSCON_USBCLKSEL_SEL_SHIFT)) & SYSCON_USBCLKSEL_SEL_MASK)

/*! @name FXCOMCLKSEL - Flexcomm0 clock source select..Flexcomm7 clock source select */
#define SYSCON_FXCOMCLKSEL_SEL_MASK              (0x7U)
#define SYSCON_FXCOMCLKSEL_SEL_SHIFT             (0U)
#define SYSCON_FXCOMCLKSEL_SEL(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_FXCOMCLKSEL_SEL_SHIFT)) & SYSCON_FXCOMCLKSEL_SEL_MASK)

/* The count of SYSCON_FXCOMCLKSEL */
#define SYSCON_FXCOMCLKSEL_COUNT                 (8U)

/*! @name MCLKCLKSEL - MCLK clock source select */
#define SYSCON_MCLKCLKSEL_SEL_MASK               (0x7U)
#define SYSCON_MCLKCLKSEL_SEL_SHIFT              (0U)
#define SYSCON_MCLKCLKSEL_SEL(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_MCLKCLKSEL_SEL_SHIFT)) & SYSCON_MCLKCLKSEL_SEL_MASK)

/*! @name FRGCLKSEL - Fractional Rate Generator clock source select */
#define SYSCON_FRGCLKSEL_SEL_MASK                (0x7U)
#define SYSCON_FRGCLKSEL_SEL_SHIFT               (0U)
#define SYSCON_FRGCLKSEL_SEL(x)                  (((uint32_t)(((uint32_t)(x)) << SYSCON_FRGCLKSEL_SEL_SHIFT)) & SYSCON_FRGCLKSEL_SEL_MASK)

/*! @name SYSTICKCLKDIV - SYSTICK clock divider */
#define SYSCON_SYSTICKCLKDIV_DIV_MASK            (0xFFU)
#define SYSCON_SYSTICKCLKDIV_DIV_SHIFT           (0U)
#define SYSCON_SYSTICKCLKDIV_DIV(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSTICKCLKDIV_DIV_SHIFT)) & SYSCON_SYSTICKCLKDIV_DIV_MASK)

/*! @name AHBCLKDIV - AHB clock divider */
#define SYSCON_AHBCLKDIV_DIV_MASK                (0xFFU)
#define SYSCON_AHBCLKDIV_DIV_SHIFT               (0U)
#define SYSCON_AHBCLKDIV_DIV(x)                  (((uint32_t)(((uint32_t)(x)) << SYSCON_AHBCLKDIV_DIV_SHIFT)) & SYSCON_AHBCLKDIV_DIV_MASK)

/*! @name CLKOUTDIV - CLKOUT clock divider */
#define SYSCON_CLKOUTDIV_DIV_MASK                (0xFFU)
#define SYSCON_CLKOUTDIV_DIV_SHIFT               (0U)
#define SYSCON_CLKOUTDIV_DIV(x)                  (((uint32_t)(((uint32_t)(x)) << SYSCON_CLKOUTDIV_DIV_SHIFT)) & SYSCON_CLKOUTDIV_DIV_MASK)

/*! @name ADCCLKDIV - ADC clock divider */
#define SYSCON_ADCCLKDIV_DIV_MASK                (0xFFU)
#define SYSCON_ADCCLKDIV_DIV_SHIFT               (0U)
#define SYSCON_ADCCLKDIV_DIV(x)                  (((uint32_t)(((uint32_t)(x)) << SYSCON_ADCCLKDIV_DIV_SHIFT)) & SYSCON_ADCCLKDIV_DIV_MASK)

/*! @name USBCLKDIV - USB clock divider */
#define SYSCON_USBCLKDIV_DIV_MASK                (0xFFU)
#define SYSCON_USBCLKDIV_DIV_SHIFT               (0U)
#define SYSCON_USBCLKDIV_DIV(x)                  (((uint32_t)(((uint32_t)(x)) << SYSCON_USBCLKDIV_DIV_SHIFT)) & SYSCON_USBCLKDIV_DIV_MASK)

/*! @name FRGCTRL - Fractional rate divider */
#define SYSCON_FRGCTRL_DIV_MASK                  (0xFFU)
#define SYSCON_FRGCTRL_DIV_SHIFT                 (0U)
#define SYSCON_FRGCTRL_DIV(x)                    (((uint32_t)(((uint32_t)(x)) << SYSCON_FRGCTRL_DIV_SHIFT)) & SYSCON_FRGCTRL_DIV_MASK)
#define SYSCON_FRGCTRL_MULT_MASK                 (0xFF00U)
#define SYSCON_FRGCTRL_MULT_SHIFT                (8U)
#define SYSCON_FRGCTRL_MULT(x)                   (((uint32_t)(((uint32_t)(x)) << SYSCON_FRGCTRL_MULT_SHIFT)) & SYSCON_FRGCTRL_MULT_MASK)

/*! @name MCLKDIV - I2S MCLK clock divider */
#define SYSCON_MCLKDIV_DIV_MASK                  (0xFFU)
#define SYSCON_MCLKDIV_DIV_SHIFT                 (0U)
#define SYSCON_MCLKDIV_DIV(x)                    (((uint32_t)(((uint32_t)(x)) << SYSCON_MCLKDIV_DIV_SHIFT)) & SYSCON_MCLKDIV_DIV_MASK)

/*! @name FLASHCFG - Flash wait states configuration */
#define SYSCON_FLASHCFG_FETCHCFG_MASK            (0x3U)
#define SYSCON_FLASHCFG_FETCHCFG_SHIFT           (0U)
#define SYSCON_FLASHCFG_FETCHCFG(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_FLASHCFG_FETCHCFG_SHIFT)) & SYSCON_FLASHCFG_FETCHCFG_MASK)
#define SYSCON_FLASHCFG_DATACFG_MASK             (0xCU)
#define SYSCON_FLASHCFG_DATACFG_SHIFT            (2U)
#define SYSCON_FLASHCFG_DATACFG(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_FLASHCFG_DATACFG_SHIFT)) & SYSCON_FLASHCFG_DATACFG_MASK)
#define SYSCON_FLASHCFG_ACCEL_MASK               (0x10U)
#define SYSCON_FLASHCFG_ACCEL_SHIFT              (4U)
#define SYSCON_FLASHCFG_ACCEL(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_FLASHCFG_ACCEL_SHIFT)) & SYSCON_FLASHCFG_ACCEL_MASK)
#define SYSCON_FLASHCFG_PREFEN_MASK              (0x20U)
#define SYSCON_FLASHCFG_PREFEN_SHIFT             (5U)
#define SYSCON_FLASHCFG_PREFEN(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_FLASHCFG_PREFEN_SHIFT)) & SYSCON_FLASHCFG_PREFEN_MASK)
#define SYSCON_FLASHCFG_PREFOVR_MASK             (0x40U)
#define SYSCON_FLASHCFG_PREFOVR_SHIFT            (6U)
#define SYSCON_FLASHCFG_PREFOVR(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_FLASHCFG_PREFOVR_SHIFT)) & SYSCON_FLASHCFG_PREFOVR_MASK)
#define SYSCON_FLASHCFG_FLASHTIM_MASK            (0xF000U)
#define SYSCON_FLASHCFG_FLASHTIM_SHIFT           (12U)
#define SYSCON_FLASHCFG_FLASHTIM(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_FLASHCFG_FLASHTIM_SHIFT)) & SYSCON_FLASHCFG_FLASHTIM_MASK)

/*! @name USBCLKCTRL - USB clock control */
#define SYSCON_USBCLKCTRL_POL_CLK_MASK           (0x2U)
#define SYSCON_USBCLKCTRL_POL_CLK_SHIFT          (1U)
#define SYSCON_USBCLKCTRL_POL_CLK(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_USBCLKCTRL_POL_CLK_SHIFT)) & SYSCON_USBCLKCTRL_POL_CLK_MASK)

/*! @name USBCLKSTAT - USB clock status */
#define SYSCON_USBCLKSTAT_NEED_CLKST_MASK        (0x1U)
#define SYSCON_USBCLKSTAT_NEED_CLKST_SHIFT       (0U)
#define SYSCON_USBCLKSTAT_NEED_CLKST(x)          (((uint32_t)(((uint32_t)(x)) << SYSCON_USBCLKSTAT_NEED_CLKST_SHIFT)) & SYSCON_USBCLKSTAT_NEED_CLKST_MASK)

/*! @name FREQMECTRL - Frequency measure register */
#define SYSCON_FREQMECTRL_CAPVAL_MASK            (0x3FFFU)
#define SYSCON_FREQMECTRL_CAPVAL_SHIFT           (0U)
#define SYSCON_FREQMECTRL_CAPVAL(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_FREQMECTRL_CAPVAL_SHIFT)) & SYSCON_FREQMECTRL_CAPVAL_MASK)
#define SYSCON_FREQMECTRL_PROG_MASK              (0x80000000U)
#define SYSCON_FREQMECTRL_PROG_SHIFT             (31U)
#define SYSCON_FREQMECTRL_PROG(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_FREQMECTRL_PROG_SHIFT)) & SYSCON_FREQMECTRL_PROG_MASK)

/*! @name MCLKIO - MCLK input/output control */
#define SYSCON_MCLKIO_DIR_MASK                   (0x1U)
#define SYSCON_MCLKIO_DIR_SHIFT                  (0U)
#define SYSCON_MCLKIO_DIR(x)                     (((uint32_t)(((uint32_t)(x)) << SYSCON_MCLKIO_DIR_SHIFT)) & SYSCON_MCLKIO_DIR_MASK)

/*! @name FROCTRL - FRO oscillator control */
#define SYSCON_FROCTRL_TRIM_MASK                 (0x3FFFU)
#define SYSCON_FROCTRL_TRIM_SHIFT                (0U)
#define SYSCON_FROCTRL_TRIM(x)                   (((uint32_t)(((uint32_t)(x)) << SYSCON_FROCTRL_TRIM_SHIFT)) & SYSCON_FROCTRL_TRIM_MASK)
#define SYSCON_FROCTRL_SEL_MASK                  (0x4000U)
#define SYSCON_FROCTRL_SEL_SHIFT                 (14U)
#define SYSCON_FROCTRL_SEL(x)                    (((uint32_t)(((uint32_t)(x)) << SYSCON_FROCTRL_SEL_SHIFT)) & SYSCON_FROCTRL_SEL_MASK)
#define SYSCON_FROCTRL_FREQTRIM_MASK             (0xFF0000U)
#define SYSCON_FROCTRL_FREQTRIM_SHIFT            (16U)
#define SYSCON_FROCTRL_FREQTRIM(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_FROCTRL_FREQTRIM_SHIFT)) & SYSCON_FROCTRL_FREQTRIM_MASK)
#define SYSCON_FROCTRL_USBCLKADJ_MASK            (0x1000000U)
#define SYSCON_FROCTRL_USBCLKADJ_SHIFT           (24U)
#define SYSCON_FROCTRL_USBCLKADJ(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_FROCTRL_USBCLKADJ_SHIFT)) & SYSCON_FROCTRL_USBCLKADJ_MASK)
#define SYSCON_FROCTRL_USBMODCHG_MASK            (0x2000000U)
#define SYSCON_FROCTRL_USBMODCHG_SHIFT           (25U)
#define SYSCON_FROCTRL_USBMODCHG(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_FROCTRL_USBMODCHG_SHIFT)) & SYSCON_FROCTRL_USBMODCHG_MASK)
#define SYSCON_FROCTRL_HSPDCLK_MASK              (0x40000000U)
#define SYSCON_FROCTRL_HSPDCLK_SHIFT             (30U)
#define SYSCON_FROCTRL_HSPDCLK(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_FROCTRL_HSPDCLK_SHIFT)) & SYSCON_FROCTRL_HSPDCLK_MASK)
#define SYSCON_FROCTRL_WRTRIM_MASK               (0x80000000U)
#define SYSCON_FROCTRL_WRTRIM_SHIFT              (31U)
#define SYSCON_FROCTRL_WRTRIM(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_FROCTRL_WRTRIM_SHIFT)) & SYSCON_FROCTRL_WRTRIM_MASK)

/*! @name WDTOSCCTRL - Watchdog oscillator control */
#define SYSCON_WDTOSCCTRL_DIVSEL_MASK            (0x1FU)
#define SYSCON_WDTOSCCTRL_DIVSEL_SHIFT           (0U)
#define SYSCON_WDTOSCCTRL_DIVSEL(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_WDTOSCCTRL_DIVSEL_SHIFT)) & SYSCON_WDTOSCCTRL_DIVSEL_MASK)
#define SYSCON_WDTOSCCTRL_FREQSEL_MASK           (0x3E0U)
#define SYSCON_WDTOSCCTRL_FREQSEL_SHIFT          (5U)
#define SYSCON_WDTOSCCTRL_FREQSEL(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT)) & SYSCON_WDTOSCCTRL_FREQSEL_MASK)

/*! @name RTCOSCCTRL - RTC oscillator 32 kHz output control */
#define SYSCON_RTCOSCCTRL_EN_MASK                (0x1U)
#define SYSCON_RTCOSCCTRL_EN_SHIFT               (0U)
#define SYSCON_RTCOSCCTRL_EN(x)                  (((uint32_t)(((uint32_t)(x)) << SYSCON_RTCOSCCTRL_EN_SHIFT)) & SYSCON_RTCOSCCTRL_EN_MASK)

/*! @name SYSPLLCTRL - PLL control */
#define SYSCON_SYSPLLCTRL_SELR_MASK              (0xFU)
#define SYSCON_SYSPLLCTRL_SELR_SHIFT             (0U)
#define SYSCON_SYSPLLCTRL_SELR(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLCTRL_SELR_SHIFT)) & SYSCON_SYSPLLCTRL_SELR_MASK)
#define SYSCON_SYSPLLCTRL_SELI_MASK              (0x3F0U)
#define SYSCON_SYSPLLCTRL_SELI_SHIFT             (4U)
#define SYSCON_SYSPLLCTRL_SELI(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLCTRL_SELI_SHIFT)) & SYSCON_SYSPLLCTRL_SELI_MASK)
#define SYSCON_SYSPLLCTRL_SELP_MASK              (0x7C00U)
#define SYSCON_SYSPLLCTRL_SELP_SHIFT             (10U)
#define SYSCON_SYSPLLCTRL_SELP(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLCTRL_SELP_SHIFT)) & SYSCON_SYSPLLCTRL_SELP_MASK)
#define SYSCON_SYSPLLCTRL_BYPASS_MASK            (0x8000U)
#define SYSCON_SYSPLLCTRL_BYPASS_SHIFT           (15U)
#define SYSCON_SYSPLLCTRL_BYPASS(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLCTRL_BYPASS_SHIFT)) & SYSCON_SYSPLLCTRL_BYPASS_MASK)
#define SYSCON_SYSPLLCTRL_BYPASSCCODIV2_MASK     (0x10000U)
#define SYSCON_SYSPLLCTRL_BYPASSCCODIV2_SHIFT    (16U)
#define SYSCON_SYSPLLCTRL_BYPASSCCODIV2(x)       (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLCTRL_BYPASSCCODIV2_SHIFT)) & SYSCON_SYSPLLCTRL_BYPASSCCODIV2_MASK)
#define SYSCON_SYSPLLCTRL_UPLIMOFF_MASK          (0x20000U)
#define SYSCON_SYSPLLCTRL_UPLIMOFF_SHIFT         (17U)
#define SYSCON_SYSPLLCTRL_UPLIMOFF(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLCTRL_UPLIMOFF_SHIFT)) & SYSCON_SYSPLLCTRL_UPLIMOFF_MASK)
#define SYSCON_SYSPLLCTRL_BANDSEL_MASK           (0x40000U)
#define SYSCON_SYSPLLCTRL_BANDSEL_SHIFT          (18U)
#define SYSCON_SYSPLLCTRL_BANDSEL(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLCTRL_BANDSEL_SHIFT)) & SYSCON_SYSPLLCTRL_BANDSEL_MASK)
#define SYSCON_SYSPLLCTRL_DIRECTI_MASK           (0x80000U)
#define SYSCON_SYSPLLCTRL_DIRECTI_SHIFT          (19U)
#define SYSCON_SYSPLLCTRL_DIRECTI(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLCTRL_DIRECTI_SHIFT)) & SYSCON_SYSPLLCTRL_DIRECTI_MASK)
#define SYSCON_SYSPLLCTRL_DIRECTO_MASK           (0x100000U)
#define SYSCON_SYSPLLCTRL_DIRECTO_SHIFT          (20U)
#define SYSCON_SYSPLLCTRL_DIRECTO(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLCTRL_DIRECTO_SHIFT)) & SYSCON_SYSPLLCTRL_DIRECTO_MASK)

/*! @name SYSPLLSTAT - PLL status */
#define SYSCON_SYSPLLSTAT_LOCK_MASK              (0x1U)
#define SYSCON_SYSPLLSTAT_LOCK_SHIFT             (0U)
#define SYSCON_SYSPLLSTAT_LOCK(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLSTAT_LOCK_SHIFT)) & SYSCON_SYSPLLSTAT_LOCK_MASK)

/*! @name SYSPLLNDEC - PLL N decoder */
#define SYSCON_SYSPLLNDEC_NDEC_MASK              (0x3FFU)
#define SYSCON_SYSPLLNDEC_NDEC_SHIFT             (0U)
#define SYSCON_SYSPLLNDEC_NDEC(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLNDEC_NDEC_SHIFT)) & SYSCON_SYSPLLNDEC_NDEC_MASK)
#define SYSCON_SYSPLLNDEC_NREQ_MASK              (0x400U)
#define SYSCON_SYSPLLNDEC_NREQ_SHIFT             (10U)
#define SYSCON_SYSPLLNDEC_NREQ(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLNDEC_NREQ_SHIFT)) & SYSCON_SYSPLLNDEC_NREQ_MASK)

/*! @name SYSPLLPDEC - PLL P decoder */
#define SYSCON_SYSPLLPDEC_PDEC_MASK              (0x7FU)
#define SYSCON_SYSPLLPDEC_PDEC_SHIFT             (0U)
#define SYSCON_SYSPLLPDEC_PDEC(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLPDEC_PDEC_SHIFT)) & SYSCON_SYSPLLPDEC_PDEC_MASK)
#define SYSCON_SYSPLLPDEC_PREQ_MASK              (0x80U)
#define SYSCON_SYSPLLPDEC_PREQ_SHIFT             (7U)
#define SYSCON_SYSPLLPDEC_PREQ(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLPDEC_PREQ_SHIFT)) & SYSCON_SYSPLLPDEC_PREQ_MASK)

/*! @name SYSPLLSSCTRL0 - PLL spread spectrum control 0 */
#define SYSCON_SYSPLLSSCTRL0_MDEC_MASK           (0x1FFFFU)
#define SYSCON_SYSPLLSSCTRL0_MDEC_SHIFT          (0U)
#define SYSCON_SYSPLLSSCTRL0_MDEC(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLSSCTRL0_MDEC_SHIFT)) & SYSCON_SYSPLLSSCTRL0_MDEC_MASK)
#define SYSCON_SYSPLLSSCTRL0_MREQ_MASK           (0x20000U)
#define SYSCON_SYSPLLSSCTRL0_MREQ_SHIFT          (17U)
#define SYSCON_SYSPLLSSCTRL0_MREQ(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLSSCTRL0_MREQ_SHIFT)) & SYSCON_SYSPLLSSCTRL0_MREQ_MASK)
#define SYSCON_SYSPLLSSCTRL0_SEL_EXT_MASK        (0x40000U)
#define SYSCON_SYSPLLSSCTRL0_SEL_EXT_SHIFT       (18U)
#define SYSCON_SYSPLLSSCTRL0_SEL_EXT(x)          (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLSSCTRL0_SEL_EXT_SHIFT)) & SYSCON_SYSPLLSSCTRL0_SEL_EXT_MASK)

/*! @name SYSPLLSSCTRL1 - PLL spread spectrum control 1 */
#define SYSCON_SYSPLLSSCTRL1_MD_MASK             (0x7FFFFU)
#define SYSCON_SYSPLLSSCTRL1_MD_SHIFT            (0U)
#define SYSCON_SYSPLLSSCTRL1_MD(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLSSCTRL1_MD_SHIFT)) & SYSCON_SYSPLLSSCTRL1_MD_MASK)
#define SYSCON_SYSPLLSSCTRL1_MDREQ_MASK          (0x80000U)
#define SYSCON_SYSPLLSSCTRL1_MDREQ_SHIFT         (19U)
#define SYSCON_SYSPLLSSCTRL1_MDREQ(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLSSCTRL1_MDREQ_SHIFT)) & SYSCON_SYSPLLSSCTRL1_MDREQ_MASK)
#define SYSCON_SYSPLLSSCTRL1_MF_MASK             (0x700000U)
#define SYSCON_SYSPLLSSCTRL1_MF_SHIFT            (20U)
#define SYSCON_SYSPLLSSCTRL1_MF(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLSSCTRL1_MF_SHIFT)) & SYSCON_SYSPLLSSCTRL1_MF_MASK)
#define SYSCON_SYSPLLSSCTRL1_MR_MASK             (0x3800000U)
#define SYSCON_SYSPLLSSCTRL1_MR_SHIFT            (23U)
#define SYSCON_SYSPLLSSCTRL1_MR(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLSSCTRL1_MR_SHIFT)) & SYSCON_SYSPLLSSCTRL1_MR_MASK)
#define SYSCON_SYSPLLSSCTRL1_MC_MASK             (0xC000000U)
#define SYSCON_SYSPLLSSCTRL1_MC_SHIFT            (26U)
#define SYSCON_SYSPLLSSCTRL1_MC(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLSSCTRL1_MC_SHIFT)) & SYSCON_SYSPLLSSCTRL1_MC_MASK)
#define SYSCON_SYSPLLSSCTRL1_PD_MASK             (0x10000000U)
#define SYSCON_SYSPLLSSCTRL1_PD_SHIFT            (28U)
#define SYSCON_SYSPLLSSCTRL1_PD(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLSSCTRL1_PD_SHIFT)) & SYSCON_SYSPLLSSCTRL1_PD_MASK)
#define SYSCON_SYSPLLSSCTRL1_DITHER_MASK         (0x20000000U)
#define SYSCON_SYSPLLSSCTRL1_DITHER_SHIFT        (29U)
#define SYSCON_SYSPLLSSCTRL1_DITHER(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_SYSPLLSSCTRL1_DITHER_SHIFT)) & SYSCON_SYSPLLSSCTRL1_DITHER_MASK)

/*! @name PDSLEEPCFG - Sleep configuration register n */
#define SYSCON_PDSLEEPCFG_PD_SLEEP_MASK          (0xFFFFFFFFU)
#define SYSCON_PDSLEEPCFG_PD_SLEEP_SHIFT         (0U)
#define SYSCON_PDSLEEPCFG_PD_SLEEP(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_PDSLEEPCFG_PD_SLEEP_SHIFT)) & SYSCON_PDSLEEPCFG_PD_SLEEP_MASK)

/* The count of SYSCON_PDSLEEPCFG */
#define SYSCON_PDSLEEPCFG_COUNT                  (2U)

/*! @name PDRUNCFG - Power configuration register 0 */
#define SYSCON_PDRUNCFG_PDEN_FRO_MASK            (0x10U)
#define SYSCON_PDRUNCFG_PDEN_FRO_SHIFT           (4U)
#define SYSCON_PDRUNCFG_PDEN_FRO(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_PDRUNCFG_PDEN_FRO_SHIFT)) & SYSCON_PDRUNCFG_PDEN_FRO_MASK)
#define SYSCON_PDRUNCFG_PDEN_TS_MASK             (0x40U)
#define SYSCON_PDRUNCFG_PDEN_TS_SHIFT            (6U)
#define SYSCON_PDRUNCFG_PDEN_TS(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_PDRUNCFG_PDEN_TS_SHIFT)) & SYSCON_PDRUNCFG_PDEN_TS_MASK)
#define SYSCON_PDRUNCFG_PDEN_BOD_RST_MASK        (0x80U)
#define SYSCON_PDRUNCFG_PDEN_BOD_RST_SHIFT       (7U)
#define SYSCON_PDRUNCFG_PDEN_BOD_RST(x)          (((uint32_t)(((uint32_t)(x)) << SYSCON_PDRUNCFG_PDEN_BOD_RST_SHIFT)) & SYSCON_PDRUNCFG_PDEN_BOD_RST_MASK)
#define SYSCON_PDRUNCFG_PDEN_BOD_INTR_MASK       (0x100U)
#define SYSCON_PDRUNCFG_PDEN_BOD_INTR_SHIFT      (8U)
#define SYSCON_PDRUNCFG_PDEN_BOD_INTR(x)         (((uint32_t)(((uint32_t)(x)) << SYSCON_PDRUNCFG_PDEN_BOD_INTR_SHIFT)) & SYSCON_PDRUNCFG_PDEN_BOD_INTR_MASK)
#define SYSCON_PDRUNCFG_PDEN_ADC0_MASK           (0x400U)
#define SYSCON_PDRUNCFG_PDEN_ADC0_SHIFT          (10U)
#define SYSCON_PDRUNCFG_PDEN_ADC0(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PDRUNCFG_PDEN_ADC0_SHIFT)) & SYSCON_PDRUNCFG_PDEN_ADC0_MASK)
#define SYSCON_PDRUNCFG_PDEN_SRAM0_MASK          (0x2000U)
#define SYSCON_PDRUNCFG_PDEN_SRAM0_SHIFT         (13U)
#define SYSCON_PDRUNCFG_PDEN_SRAM0(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_PDRUNCFG_PDEN_SRAM0_SHIFT)) & SYSCON_PDRUNCFG_PDEN_SRAM0_MASK)
#define SYSCON_PDRUNCFG_PDEN_SRAMX_MASK          (0x10000U)
#define SYSCON_PDRUNCFG_PDEN_SRAMX_SHIFT         (16U)
#define SYSCON_PDRUNCFG_PDEN_SRAMX(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_PDRUNCFG_PDEN_SRAMX_SHIFT)) & SYSCON_PDRUNCFG_PDEN_SRAMX_MASK)
#define SYSCON_PDRUNCFG_PDEN_ROM_MASK            (0x20000U)
#define SYSCON_PDRUNCFG_PDEN_ROM_SHIFT           (17U)
#define SYSCON_PDRUNCFG_PDEN_ROM(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_PDRUNCFG_PDEN_ROM_SHIFT)) & SYSCON_PDRUNCFG_PDEN_ROM_MASK)
#define SYSCON_PDRUNCFG_PDEN_VDDA_MASK           (0x80000U)
#define SYSCON_PDRUNCFG_PDEN_VDDA_SHIFT          (19U)
#define SYSCON_PDRUNCFG_PDEN_VDDA(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PDRUNCFG_PDEN_VDDA_SHIFT)) & SYSCON_PDRUNCFG_PDEN_VDDA_MASK)
#define SYSCON_PDRUNCFG_PDEN_WDT_OSC_MASK        (0x100000U)
#define SYSCON_PDRUNCFG_PDEN_WDT_OSC_SHIFT       (20U)
#define SYSCON_PDRUNCFG_PDEN_WDT_OSC(x)          (((uint32_t)(((uint32_t)(x)) << SYSCON_PDRUNCFG_PDEN_WDT_OSC_SHIFT)) & SYSCON_PDRUNCFG_PDEN_WDT_OSC_MASK)
#define SYSCON_PDRUNCFG_PDEN_USB_PHY_MASK        (0x200000U)
#define SYSCON_PDRUNCFG_PDEN_USB_PHY_SHIFT       (21U)
#define SYSCON_PDRUNCFG_PDEN_USB_PHY(x)          (((uint32_t)(((uint32_t)(x)) << SYSCON_PDRUNCFG_PDEN_USB_PHY_SHIFT)) & SYSCON_PDRUNCFG_PDEN_USB_PHY_MASK)
#define SYSCON_PDRUNCFG_PDEN_SYS_PLL_MASK        (0x400000U)
#define SYSCON_PDRUNCFG_PDEN_SYS_PLL_SHIFT       (22U)
#define SYSCON_PDRUNCFG_PDEN_SYS_PLL(x)          (((uint32_t)(((uint32_t)(x)) << SYSCON_PDRUNCFG_PDEN_SYS_PLL_SHIFT)) & SYSCON_PDRUNCFG_PDEN_SYS_PLL_MASK)
#define SYSCON_PDRUNCFG_PDEN_VREFP_MASK          (0x800000U)
#define SYSCON_PDRUNCFG_PDEN_VREFP_SHIFT         (23U)
#define SYSCON_PDRUNCFG_PDEN_VREFP(x)            (((uint32_t)(((uint32_t)(x)) << SYSCON_PDRUNCFG_PDEN_VREFP_SHIFT)) & SYSCON_PDRUNCFG_PDEN_VREFP_MASK)

/* The count of SYSCON_PDRUNCFG */
#define SYSCON_PDRUNCFG_COUNT                    (1U)

/*! @name PDRUNCFGSET - Set bits in PDRUNCFGn */
#define SYSCON_PDRUNCFGSET_PD_SET_MASK           (0xFFFFFFFFU)
#define SYSCON_PDRUNCFGSET_PD_SET_SHIFT          (0U)
#define SYSCON_PDRUNCFGSET_PD_SET(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PDRUNCFGSET_PD_SET_SHIFT)) & SYSCON_PDRUNCFGSET_PD_SET_MASK)

/* The count of SYSCON_PDRUNCFGSET */
#define SYSCON_PDRUNCFGSET_COUNT                 (1U)

/*! @name PDRUNCFGCLR - Clear bits in PDRUNCFGn */
#define SYSCON_PDRUNCFGCLR_PD_CLR_MASK           (0xFFFFFFFFU)
#define SYSCON_PDRUNCFGCLR_PD_CLR_SHIFT          (0U)
#define SYSCON_PDRUNCFGCLR_PD_CLR(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_PDRUNCFGCLR_PD_CLR_SHIFT)) & SYSCON_PDRUNCFGCLR_PD_CLR_MASK)

/* The count of SYSCON_PDRUNCFGCLR */
#define SYSCON_PDRUNCFGCLR_COUNT                 (1U)

/*! @name STARTER - Start logic n wake-up enable register */
#define SYSCON_STARTER_WDT_BOD_MASK              (0x1U)
#define SYSCON_STARTER_WDT_BOD_SHIFT             (0U)
#define SYSCON_STARTER_WDT_BOD(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_WDT_BOD_SHIFT)) & SYSCON_STARTER_WDT_BOD_MASK)
#define SYSCON_STARTER_DMA0_MASK                 (0x2U)
#define SYSCON_STARTER_DMA0_SHIFT                (1U)
#define SYSCON_STARTER_DMA0(x)                   (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_DMA0_SHIFT)) & SYSCON_STARTER_DMA0_MASK)
#define SYSCON_STARTER_GINT0_MASK                (0x4U)
#define SYSCON_STARTER_GINT0_SHIFT               (2U)
#define SYSCON_STARTER_GINT0(x)                  (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_GINT0_SHIFT)) & SYSCON_STARTER_GINT0_MASK)
#define SYSCON_STARTER_GINT1_MASK                (0x8U)
#define SYSCON_STARTER_GINT1_SHIFT               (3U)
#define SYSCON_STARTER_GINT1(x)                  (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_GINT1_SHIFT)) & SYSCON_STARTER_GINT1_MASK)
#define SYSCON_STARTER_PIN_INT0_MASK             (0x10U)
#define SYSCON_STARTER_PIN_INT0_SHIFT            (4U)
#define SYSCON_STARTER_PIN_INT0(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_PIN_INT0_SHIFT)) & SYSCON_STARTER_PIN_INT0_MASK)
#define SYSCON_STARTER_PIN_INT1_MASK             (0x20U)
#define SYSCON_STARTER_PIN_INT1_SHIFT            (5U)
#define SYSCON_STARTER_PIN_INT1(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_PIN_INT1_SHIFT)) & SYSCON_STARTER_PIN_INT1_MASK)
#define SYSCON_STARTER_PIN_INT2_MASK             (0x40U)
#define SYSCON_STARTER_PIN_INT2_SHIFT            (6U)
#define SYSCON_STARTER_PIN_INT2(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_PIN_INT2_SHIFT)) & SYSCON_STARTER_PIN_INT2_MASK)
#define SYSCON_STARTER_PIN_INT3_MASK             (0x80U)
#define SYSCON_STARTER_PIN_INT3_SHIFT            (7U)
#define SYSCON_STARTER_PIN_INT3(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_PIN_INT3_SHIFT)) & SYSCON_STARTER_PIN_INT3_MASK)
#define SYSCON_STARTER_UTICK0_MASK               (0x100U)
#define SYSCON_STARTER_UTICK0_SHIFT              (8U)
#define SYSCON_STARTER_UTICK0(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_UTICK0_SHIFT)) & SYSCON_STARTER_UTICK0_MASK)
#define SYSCON_STARTER_MRT0_MASK                 (0x200U)
#define SYSCON_STARTER_MRT0_SHIFT                (9U)
#define SYSCON_STARTER_MRT0(x)                   (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_MRT0_SHIFT)) & SYSCON_STARTER_MRT0_MASK)
#define SYSCON_STARTER_CTIMER0_MASK              (0x400U)
#define SYSCON_STARTER_CTIMER0_SHIFT             (10U)
#define SYSCON_STARTER_CTIMER0(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_CTIMER0_SHIFT)) & SYSCON_STARTER_CTIMER0_MASK)
#define SYSCON_STARTER_CTIMER1_MASK              (0x800U)
#define SYSCON_STARTER_CTIMER1_SHIFT             (11U)
#define SYSCON_STARTER_CTIMER1(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_CTIMER1_SHIFT)) & SYSCON_STARTER_CTIMER1_MASK)
#define SYSCON_STARTER_SCT0_MASK                 (0x1000U)
#define SYSCON_STARTER_SCT0_SHIFT                (12U)
#define SYSCON_STARTER_SCT0(x)                   (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_SCT0_SHIFT)) & SYSCON_STARTER_SCT0_MASK)
#define SYSCON_STARTER_CTIMER3_MASK              (0x2000U)
#define SYSCON_STARTER_CTIMER3_SHIFT             (13U)
#define SYSCON_STARTER_CTIMER3(x)                (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_CTIMER3_SHIFT)) & SYSCON_STARTER_CTIMER3_MASK)
#define SYSCON_STARTER_FLEXCOMM0_MASK            (0x4000U)
#define SYSCON_STARTER_FLEXCOMM0_SHIFT           (14U)
#define SYSCON_STARTER_FLEXCOMM0(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_FLEXCOMM0_SHIFT)) & SYSCON_STARTER_FLEXCOMM0_MASK)
#define SYSCON_STARTER_FLEXCOMM1_MASK            (0x8000U)
#define SYSCON_STARTER_FLEXCOMM1_SHIFT           (15U)
#define SYSCON_STARTER_FLEXCOMM1(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_FLEXCOMM1_SHIFT)) & SYSCON_STARTER_FLEXCOMM1_MASK)
#define SYSCON_STARTER_FLEXCOMM2_MASK            (0x10000U)
#define SYSCON_STARTER_FLEXCOMM2_SHIFT           (16U)
#define SYSCON_STARTER_FLEXCOMM2(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_FLEXCOMM2_SHIFT)) & SYSCON_STARTER_FLEXCOMM2_MASK)
#define SYSCON_STARTER_FLEXCOMM3_MASK            (0x20000U)
#define SYSCON_STARTER_FLEXCOMM3_SHIFT           (17U)
#define SYSCON_STARTER_FLEXCOMM3(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_FLEXCOMM3_SHIFT)) & SYSCON_STARTER_FLEXCOMM3_MASK)
#define SYSCON_STARTER_FLEXCOMM4_MASK            (0x40000U)
#define SYSCON_STARTER_FLEXCOMM4_SHIFT           (18U)
#define SYSCON_STARTER_FLEXCOMM4(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_FLEXCOMM4_SHIFT)) & SYSCON_STARTER_FLEXCOMM4_MASK)
#define SYSCON_STARTER_FLEXCOMM5_MASK            (0x80000U)
#define SYSCON_STARTER_FLEXCOMM5_SHIFT           (19U)
#define SYSCON_STARTER_FLEXCOMM5(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_FLEXCOMM5_SHIFT)) & SYSCON_STARTER_FLEXCOMM5_MASK)
#define SYSCON_STARTER_FLEXCOMM6_MASK            (0x100000U)
#define SYSCON_STARTER_FLEXCOMM6_SHIFT           (20U)
#define SYSCON_STARTER_FLEXCOMM6(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_FLEXCOMM6_SHIFT)) & SYSCON_STARTER_FLEXCOMM6_MASK)
#define SYSCON_STARTER_FLEXCOMM7_MASK            (0x200000U)
#define SYSCON_STARTER_FLEXCOMM7_SHIFT           (21U)
#define SYSCON_STARTER_FLEXCOMM7(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_FLEXCOMM7_SHIFT)) & SYSCON_STARTER_FLEXCOMM7_MASK)
#define SYSCON_STARTER_USB0_NEEDCLK_MASK         (0x8000000U)
#define SYSCON_STARTER_USB0_NEEDCLK_SHIFT        (27U)
#define SYSCON_STARTER_USB0_NEEDCLK(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_USB0_NEEDCLK_SHIFT)) & SYSCON_STARTER_USB0_NEEDCLK_MASK)
#define SYSCON_STARTER_USB0_MASK                 (0x10000000U)
#define SYSCON_STARTER_USB0_SHIFT                (28U)
#define SYSCON_STARTER_USB0(x)                   (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_USB0_SHIFT)) & SYSCON_STARTER_USB0_MASK)
#define SYSCON_STARTER_RTC_MASK                  (0x20000000U)
#define SYSCON_STARTER_RTC_SHIFT                 (29U)
#define SYSCON_STARTER_RTC(x)                    (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTER_RTC_SHIFT)) & SYSCON_STARTER_RTC_MASK)

/* The count of SYSCON_STARTER */
#define SYSCON_STARTER_COUNT                     (1U)

/*! @name STARTERSET - Set bits in STARTERn */
#define SYSCON_STARTERSET_START_SET_MASK         (0xFFFFFFFFU)
#define SYSCON_STARTERSET_START_SET_SHIFT        (0U)
#define SYSCON_STARTERSET_START_SET(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTERSET_START_SET_SHIFT)) & SYSCON_STARTERSET_START_SET_MASK)

/* The count of SYSCON_STARTERSET */
#define SYSCON_STARTERSET_COUNT                  (1U)

/*! @name STARTERCLR - Clear bits in STARTERn */
#define SYSCON_STARTERCLR_START_CLR_MASK         (0xFFFFFFFFU)
#define SYSCON_STARTERCLR_START_CLR_SHIFT        (0U)
#define SYSCON_STARTERCLR_START_CLR(x)           (((uint32_t)(((uint32_t)(x)) << SYSCON_STARTERCLR_START_CLR_SHIFT)) & SYSCON_STARTERCLR_START_CLR_MASK)

/* The count of SYSCON_STARTERCLR */
#define SYSCON_STARTERCLR_COUNT                  (1U)

/*! @name HWWAKE - Configures special cases of hardware wake-up */
#define SYSCON_HWWAKE_FORCEWAKE_MASK             (0x1U)
#define SYSCON_HWWAKE_FORCEWAKE_SHIFT            (0U)
#define SYSCON_HWWAKE_FORCEWAKE(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_HWWAKE_FORCEWAKE_SHIFT)) & SYSCON_HWWAKE_FORCEWAKE_MASK)
#define SYSCON_HWWAKE_FCWAKE_MASK                (0x2U)
#define SYSCON_HWWAKE_FCWAKE_SHIFT               (1U)
#define SYSCON_HWWAKE_FCWAKE(x)                  (((uint32_t)(((uint32_t)(x)) << SYSCON_HWWAKE_FCWAKE_SHIFT)) & SYSCON_HWWAKE_FCWAKE_MASK)
#define SYSCON_HWWAKE_WAKEDMA_MASK               (0x8U)
#define SYSCON_HWWAKE_WAKEDMA_SHIFT              (3U)
#define SYSCON_HWWAKE_WAKEDMA(x)                 (((uint32_t)(((uint32_t)(x)) << SYSCON_HWWAKE_WAKEDMA_SHIFT)) & SYSCON_HWWAKE_WAKEDMA_MASK)

/*! @name JTAGIDCODE - JTAG ID code register */
#define SYSCON_JTAGIDCODE_JTAGID_MASK            (0xFFFFFFFFU)
#define SYSCON_JTAGIDCODE_JTAGID_SHIFT           (0U)
#define SYSCON_JTAGIDCODE_JTAGID(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_JTAGIDCODE_JTAGID_SHIFT)) & SYSCON_JTAGIDCODE_JTAGID_MASK)

/*! @name DEVICE_ID0 - Part ID register */
#define SYSCON_DEVICE_ID0_PARTID_MASK            (0xFFFFFFFFU)
#define SYSCON_DEVICE_ID0_PARTID_SHIFT           (0U)
#define SYSCON_DEVICE_ID0_PARTID(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_DEVICE_ID0_PARTID_SHIFT)) & SYSCON_DEVICE_ID0_PARTID_MASK)

/*! @name DEVICE_ID1 - Boot ROM and die revision register */
#define SYSCON_DEVICE_ID1_REVID_MASK             (0xFFFFFFFFU)
#define SYSCON_DEVICE_ID1_REVID_SHIFT            (0U)
#define SYSCON_DEVICE_ID1_REVID(x)               (((uint32_t)(((uint32_t)(x)) << SYSCON_DEVICE_ID1_REVID_SHIFT)) & SYSCON_DEVICE_ID1_REVID_MASK)

/*! @name BODCTRL - Brown-Out Detect control */
#define SYSCON_BODCTRL_BODRSTLEV_MASK            (0x3U)
#define SYSCON_BODCTRL_BODRSTLEV_SHIFT           (0U)
#define SYSCON_BODCTRL_BODRSTLEV(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_BODCTRL_BODRSTLEV_SHIFT)) & SYSCON_BODCTRL_BODRSTLEV_MASK)
#define SYSCON_BODCTRL_BODRSTENA_MASK            (0x4U)
#define SYSCON_BODCTRL_BODRSTENA_SHIFT           (2U)
#define SYSCON_BODCTRL_BODRSTENA(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_BODCTRL_BODRSTENA_SHIFT)) & SYSCON_BODCTRL_BODRSTENA_MASK)
#define SYSCON_BODCTRL_BODINTLEV_MASK            (0x18U)
#define SYSCON_BODCTRL_BODINTLEV_SHIFT           (3U)
#define SYSCON_BODCTRL_BODINTLEV(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_BODCTRL_BODINTLEV_SHIFT)) & SYSCON_BODCTRL_BODINTLEV_MASK)
#define SYSCON_BODCTRL_BODINTENA_MASK            (0x20U)
#define SYSCON_BODCTRL_BODINTENA_SHIFT           (5U)
#define SYSCON_BODCTRL_BODINTENA(x)              (((uint32_t)(((uint32_t)(x)) << SYSCON_BODCTRL_BODINTENA_SHIFT)) & SYSCON_BODCTRL_BODINTENA_MASK)
#define SYSCON_BODCTRL_BODRSTSTAT_MASK           (0x40U)
#define SYSCON_BODCTRL_BODRSTSTAT_SHIFT          (6U)
#define SYSCON_BODCTRL_BODRSTSTAT(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_BODCTRL_BODRSTSTAT_SHIFT)) & SYSCON_BODCTRL_BODRSTSTAT_MASK)
#define SYSCON_BODCTRL_BODINTSTAT_MASK           (0x80U)
#define SYSCON_BODCTRL_BODINTSTAT_SHIFT          (7U)
#define SYSCON_BODCTRL_BODINTSTAT(x)             (((uint32_t)(((uint32_t)(x)) << SYSCON_BODCTRL_BODINTSTAT_SHIFT)) & SYSCON_BODCTRL_BODINTSTAT_MASK)


/*!
 * @}
 */ /* end of group SYSCON_Register_Masks */


/* SYSCON - Peripheral instance base addresses */
/** Peripheral SYSCON base address */
#define SYSCON_BASE                              (0x40000000u)
/** Peripheral SYSCON base pointer */
#define SYSCON                                   ((SYSCON_Type *)SYSCON_BASE)
/** Array initializer of SYSCON peripheral base addresses */
#define SYSCON_BASE_ADDRS                        { SYSCON_BASE }
/** Array initializer of SYSCON peripheral base pointers */
#define SYSCON_BASE_PTRS                         { SYSCON }

/*!
 * @}
 */ /* end of group SYSCON_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- USART Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USART_Peripheral_Access_Layer USART Peripheral Access Layer
 * @{
 */

/** USART - Register Layout Typedef */
typedef struct {
  __IO uint32_t CFG;                               /**< USART Configuration register. Basic USART configuration settings that typically are not changed during operation., offset: 0x0 */
  __IO uint32_t CTL;                               /**< USART Control register. USART control settings that are more likely to change during operation., offset: 0x4 */
  __IO uint32_t STAT;                              /**< USART Status register. The complete status value can be read here. Writing ones clears some bits in the register. Some bits can be cleared by writing a 1 to them., offset: 0x8 */
  __IO uint32_t INTENSET;                          /**< Interrupt Enable read and Set register for USART (not FIFO) status. Contains individual interrupt enable bits for each potential USART interrupt. A complete value may be read from this register. Writing a 1 to any implemented bit position causes that bit to be set., offset: 0xC */
  __O  uint32_t INTENCLR;                          /**< Interrupt Enable Clear register. Allows clearing any combination of bits in the INTENSET register. Writing a 1 to any implemented bit position causes the corresponding bit to be cleared., offset: 0x10 */
       uint8_t RESERVED_0[12];
  __IO uint32_t BRG;                               /**< Baud Rate Generator register. 16-bit integer baud rate divisor value., offset: 0x20 */
  __I  uint32_t INTSTAT;                           /**< Interrupt status register. Reflects interrupts that are currently enabled., offset: 0x24 */
  __IO uint32_t OSR;                               /**< Oversample selection register for asynchronous communication., offset: 0x28 */
  __IO uint32_t ADDR;                              /**< Address register for automatic address matching., offset: 0x2C */
       uint8_t RESERVED_1[3536];
  __IO uint32_t FIFOCFG;                           /**< FIFO configuration and enable register., offset: 0xE00 */
  __IO uint32_t FIFOSTAT;                          /**< FIFO status register., offset: 0xE04 */
  __IO uint32_t FIFOTRIG;                          /**< FIFO trigger settings for interrupt and DMA request., offset: 0xE08 */
       uint8_t RESERVED_2[4];
  __IO uint32_t FIFOINTENSET;                      /**< FIFO interrupt enable set (enable) and read register., offset: 0xE10 */
  __IO uint32_t FIFOINTENCLR;                      /**< FIFO interrupt enable clear (disable) and read register., offset: 0xE14 */
  __I  uint32_t FIFOINTSTAT;                       /**< FIFO interrupt status register., offset: 0xE18 */
       uint8_t RESERVED_3[4];
  __O  uint32_t FIFOWR;                            /**< FIFO write data., offset: 0xE20 */
       uint8_t RESERVED_4[12];
  __I  uint32_t FIFORD;                            /**< FIFO read data., offset: 0xE30 */
       uint8_t RESERVED_5[12];
  __I  uint32_t FIFORDNOPOP;                       /**< FIFO data read with no FIFO pop., offset: 0xE40 */
       uint8_t RESERVED_6[440];
  __I  uint32_t ID;                                /**< Peripheral identification register., offset: 0xFFC */
} USART_Type;

/* ----------------------------------------------------------------------------
   -- USART Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USART_Register_Masks USART Register Masks
 * @{
 */

/*! @name CFG - USART Configuration register. Basic USART configuration settings that typically are not changed during operation. */
#define USART_CFG_ENABLE_MASK                    (0x1U)
#define USART_CFG_ENABLE_SHIFT                   (0U)
#define USART_CFG_ENABLE(x)                      (((uint32_t)(((uint32_t)(x)) << USART_CFG_ENABLE_SHIFT)) & USART_CFG_ENABLE_MASK)
#define USART_CFG_DATALEN_MASK                   (0xCU)
#define USART_CFG_DATALEN_SHIFT                  (2U)
#define USART_CFG_DATALEN(x)                     (((uint32_t)(((uint32_t)(x)) << USART_CFG_DATALEN_SHIFT)) & USART_CFG_DATALEN_MASK)
#define USART_CFG_PARITYSEL_MASK                 (0x30U)
#define USART_CFG_PARITYSEL_SHIFT                (4U)
#define USART_CFG_PARITYSEL(x)                   (((uint32_t)(((uint32_t)(x)) << USART_CFG_PARITYSEL_SHIFT)) & USART_CFG_PARITYSEL_MASK)
#define USART_CFG_STOPLEN_MASK                   (0x40U)
#define USART_CFG_STOPLEN_SHIFT                  (6U)
#define USART_CFG_STOPLEN(x)                     (((uint32_t)(((uint32_t)(x)) << USART_CFG_STOPLEN_SHIFT)) & USART_CFG_STOPLEN_MASK)
#define USART_CFG_MODE32K_MASK                   (0x80U)
#define USART_CFG_MODE32K_SHIFT                  (7U)
#define USART_CFG_MODE32K(x)                     (((uint32_t)(((uint32_t)(x)) << USART_CFG_MODE32K_SHIFT)) & USART_CFG_MODE32K_MASK)
#define USART_CFG_LINMODE_MASK                   (0x100U)
#define USART_CFG_LINMODE_SHIFT                  (8U)
#define USART_CFG_LINMODE(x)                     (((uint32_t)(((uint32_t)(x)) << USART_CFG_LINMODE_SHIFT)) & USART_CFG_LINMODE_MASK)
#define USART_CFG_CTSEN_MASK                     (0x200U)
#define USART_CFG_CTSEN_SHIFT                    (9U)
#define USART_CFG_CTSEN(x)                       (((uint32_t)(((uint32_t)(x)) << USART_CFG_CTSEN_SHIFT)) & USART_CFG_CTSEN_MASK)
#define USART_CFG_SYNCEN_MASK                    (0x800U)
#define USART_CFG_SYNCEN_SHIFT                   (11U)
#define USART_CFG_SYNCEN(x)                      (((uint32_t)(((uint32_t)(x)) << USART_CFG_SYNCEN_SHIFT)) & USART_CFG_SYNCEN_MASK)
#define USART_CFG_CLKPOL_MASK                    (0x1000U)
#define USART_CFG_CLKPOL_SHIFT                   (12U)
#define USART_CFG_CLKPOL(x)                      (((uint32_t)(((uint32_t)(x)) << USART_CFG_CLKPOL_SHIFT)) & USART_CFG_CLKPOL_MASK)
#define USART_CFG_SYNCMST_MASK                   (0x4000U)
#define USART_CFG_SYNCMST_SHIFT                  (14U)
#define USART_CFG_SYNCMST(x)                     (((uint32_t)(((uint32_t)(x)) << USART_CFG_SYNCMST_SHIFT)) & USART_CFG_SYNCMST_MASK)
#define USART_CFG_LOOP_MASK                      (0x8000U)
#define USART_CFG_LOOP_SHIFT                     (15U)
#define USART_CFG_LOOP(x)                        (((uint32_t)(((uint32_t)(x)) << USART_CFG_LOOP_SHIFT)) & USART_CFG_LOOP_MASK)
#define USART_CFG_OETA_MASK                      (0x40000U)
#define USART_CFG_OETA_SHIFT                     (18U)
#define USART_CFG_OETA(x)                        (((uint32_t)(((uint32_t)(x)) << USART_CFG_OETA_SHIFT)) & USART_CFG_OETA_MASK)
#define USART_CFG_AUTOADDR_MASK                  (0x80000U)
#define USART_CFG_AUTOADDR_SHIFT                 (19U)
#define USART_CFG_AUTOADDR(x)                    (((uint32_t)(((uint32_t)(x)) << USART_CFG_AUTOADDR_SHIFT)) & USART_CFG_AUTOADDR_MASK)
#define USART_CFG_OESEL_MASK                     (0x100000U)
#define USART_CFG_OESEL_SHIFT                    (20U)
#define USART_CFG_OESEL(x)                       (((uint32_t)(((uint32_t)(x)) << USART_CFG_OESEL_SHIFT)) & USART_CFG_OESEL_MASK)
#define USART_CFG_OEPOL_MASK                     (0x200000U)
#define USART_CFG_OEPOL_SHIFT                    (21U)
#define USART_CFG_OEPOL(x)                       (((uint32_t)(((uint32_t)(x)) << USART_CFG_OEPOL_SHIFT)) & USART_CFG_OEPOL_MASK)
#define USART_CFG_RXPOL_MASK                     (0x400000U)
#define USART_CFG_RXPOL_SHIFT                    (22U)
#define USART_CFG_RXPOL(x)                       (((uint32_t)(((uint32_t)(x)) << USART_CFG_RXPOL_SHIFT)) & USART_CFG_RXPOL_MASK)
#define USART_CFG_TXPOL_MASK                     (0x800000U)
#define USART_CFG_TXPOL_SHIFT                    (23U)
#define USART_CFG_TXPOL(x)                       (((uint32_t)(((uint32_t)(x)) << USART_CFG_TXPOL_SHIFT)) & USART_CFG_TXPOL_MASK)

/*! @name CTL - USART Control register. USART control settings that are more likely to change during operation. */
#define USART_CTL_TXBRKEN_MASK                   (0x2U)
#define USART_CTL_TXBRKEN_SHIFT                  (1U)
#define USART_CTL_TXBRKEN(x)                     (((uint32_t)(((uint32_t)(x)) << USART_CTL_TXBRKEN_SHIFT)) & USART_CTL_TXBRKEN_MASK)
#define USART_CTL_ADDRDET_MASK                   (0x4U)
#define USART_CTL_ADDRDET_SHIFT                  (2U)
#define USART_CTL_ADDRDET(x)                     (((uint32_t)(((uint32_t)(x)) << USART_CTL_ADDRDET_SHIFT)) & USART_CTL_ADDRDET_MASK)
#define USART_CTL_TXDIS_MASK                     (0x40U)
#define USART_CTL_TXDIS_SHIFT                    (6U)
#define USART_CTL_TXDIS(x)                       (((uint32_t)(((uint32_t)(x)) << USART_CTL_TXDIS_SHIFT)) & USART_CTL_TXDIS_MASK)
#define USART_CTL_CC_MASK                        (0x100U)
#define USART_CTL_CC_SHIFT                       (8U)
#define USART_CTL_CC(x)                          (((uint32_t)(((uint32_t)(x)) << USART_CTL_CC_SHIFT)) & USART_CTL_CC_MASK)
#define USART_CTL_CLRCCONRX_MASK                 (0x200U)
#define USART_CTL_CLRCCONRX_SHIFT                (9U)
#define USART_CTL_CLRCCONRX(x)                   (((uint32_t)(((uint32_t)(x)) << USART_CTL_CLRCCONRX_SHIFT)) & USART_CTL_CLRCCONRX_MASK)
#define USART_CTL_AUTOBAUD_MASK                  (0x10000U)
#define USART_CTL_AUTOBAUD_SHIFT                 (16U)
#define USART_CTL_AUTOBAUD(x)                    (((uint32_t)(((uint32_t)(x)) << USART_CTL_AUTOBAUD_SHIFT)) & USART_CTL_AUTOBAUD_MASK)

/*! @name STAT - USART Status register. The complete status value can be read here. Writing ones clears some bits in the register. Some bits can be cleared by writing a 1 to them. */
#define USART_STAT_RXIDLE_MASK                   (0x2U)
#define USART_STAT_RXIDLE_SHIFT                  (1U)
#define USART_STAT_RXIDLE(x)                     (((uint32_t)(((uint32_t)(x)) << USART_STAT_RXIDLE_SHIFT)) & USART_STAT_RXIDLE_MASK)
#define USART_STAT_TXIDLE_MASK                   (0x8U)
#define USART_STAT_TXIDLE_SHIFT                  (3U)
#define USART_STAT_TXIDLE(x)                     (((uint32_t)(((uint32_t)(x)) << USART_STAT_TXIDLE_SHIFT)) & USART_STAT_TXIDLE_MASK)
#define USART_STAT_CTS_MASK                      (0x10U)
#define USART_STAT_CTS_SHIFT                     (4U)
#define USART_STAT_CTS(x)                        (((uint32_t)(((uint32_t)(x)) << USART_STAT_CTS_SHIFT)) & USART_STAT_CTS_MASK)
#define USART_STAT_DELTACTS_MASK                 (0x20U)
#define USART_STAT_DELTACTS_SHIFT                (5U)
#define USART_STAT_DELTACTS(x)                   (((uint32_t)(((uint32_t)(x)) << USART_STAT_DELTACTS_SHIFT)) & USART_STAT_DELTACTS_MASK)
#define USART_STAT_TXDISSTAT_MASK                (0x40U)
#define USART_STAT_TXDISSTAT_SHIFT               (6U)
#define USART_STAT_TXDISSTAT(x)                  (((uint32_t)(((uint32_t)(x)) << USART_STAT_TXDISSTAT_SHIFT)) & USART_STAT_TXDISSTAT_MASK)
#define USART_STAT_RXBRK_MASK                    (0x400U)
#define USART_STAT_RXBRK_SHIFT                   (10U)
#define USART_STAT_RXBRK(x)                      (((uint32_t)(((uint32_t)(x)) << USART_STAT_RXBRK_SHIFT)) & USART_STAT_RXBRK_MASK)
#define USART_STAT_DELTARXBRK_MASK               (0x800U)
#define USART_STAT_DELTARXBRK_SHIFT              (11U)
#define USART_STAT_DELTARXBRK(x)                 (((uint32_t)(((uint32_t)(x)) << USART_STAT_DELTARXBRK_SHIFT)) & USART_STAT_DELTARXBRK_MASK)
#define USART_STAT_START_MASK                    (0x1000U)
#define USART_STAT_START_SHIFT                   (12U)
#define USART_STAT_START(x)                      (((uint32_t)(((uint32_t)(x)) << USART_STAT_START_SHIFT)) & USART_STAT_START_MASK)
#define USART_STAT_FRAMERRINT_MASK               (0x2000U)
#define USART_STAT_FRAMERRINT_SHIFT              (13U)
#define USART_STAT_FRAMERRINT(x)                 (((uint32_t)(((uint32_t)(x)) << USART_STAT_FRAMERRINT_SHIFT)) & USART_STAT_FRAMERRINT_MASK)
#define USART_STAT_PARITYERRINT_MASK             (0x4000U)
#define USART_STAT_PARITYERRINT_SHIFT            (14U)
#define USART_STAT_PARITYERRINT(x)               (((uint32_t)(((uint32_t)(x)) << USART_STAT_PARITYERRINT_SHIFT)) & USART_STAT_PARITYERRINT_MASK)
#define USART_STAT_RXNOISEINT_MASK               (0x8000U)
#define USART_STAT_RXNOISEINT_SHIFT              (15U)
#define USART_STAT_RXNOISEINT(x)                 (((uint32_t)(((uint32_t)(x)) << USART_STAT_RXNOISEINT_SHIFT)) & USART_STAT_RXNOISEINT_MASK)
#define USART_STAT_ABERR_MASK                    (0x10000U)
#define USART_STAT_ABERR_SHIFT                   (16U)
#define USART_STAT_ABERR(x)                      (((uint32_t)(((uint32_t)(x)) << USART_STAT_ABERR_SHIFT)) & USART_STAT_ABERR_MASK)

/*! @name INTENSET - Interrupt Enable read and Set register for USART (not FIFO) status. Contains individual interrupt enable bits for each potential USART interrupt. A complete value may be read from this register. Writing a 1 to any implemented bit position causes that bit to be set. */
#define USART_INTENSET_TXIDLEEN_MASK             (0x8U)
#define USART_INTENSET_TXIDLEEN_SHIFT            (3U)
#define USART_INTENSET_TXIDLEEN(x)               (((uint32_t)(((uint32_t)(x)) << USART_INTENSET_TXIDLEEN_SHIFT)) & USART_INTENSET_TXIDLEEN_MASK)
#define USART_INTENSET_DELTACTSEN_MASK           (0x20U)
#define USART_INTENSET_DELTACTSEN_SHIFT          (5U)
#define USART_INTENSET_DELTACTSEN(x)             (((uint32_t)(((uint32_t)(x)) << USART_INTENSET_DELTACTSEN_SHIFT)) & USART_INTENSET_DELTACTSEN_MASK)
#define USART_INTENSET_TXDISEN_MASK              (0x40U)
#define USART_INTENSET_TXDISEN_SHIFT             (6U)
#define USART_INTENSET_TXDISEN(x)                (((uint32_t)(((uint32_t)(x)) << USART_INTENSET_TXDISEN_SHIFT)) & USART_INTENSET_TXDISEN_MASK)
#define USART_INTENSET_DELTARXBRKEN_MASK         (0x800U)
#define USART_INTENSET_DELTARXBRKEN_SHIFT        (11U)
#define USART_INTENSET_DELTARXBRKEN(x)           (((uint32_t)(((uint32_t)(x)) << USART_INTENSET_DELTARXBRKEN_SHIFT)) & USART_INTENSET_DELTARXBRKEN_MASK)
#define USART_INTENSET_STARTEN_MASK              (0x1000U)
#define USART_INTENSET_STARTEN_SHIFT             (12U)
#define USART_INTENSET_STARTEN(x)                (((uint32_t)(((uint32_t)(x)) << USART_INTENSET_STARTEN_SHIFT)) & USART_INTENSET_STARTEN_MASK)
#define USART_INTENSET_FRAMERREN_MASK            (0x2000U)
#define USART_INTENSET_FRAMERREN_SHIFT           (13U)
#define USART_INTENSET_FRAMERREN(x)              (((uint32_t)(((uint32_t)(x)) << USART_INTENSET_FRAMERREN_SHIFT)) & USART_INTENSET_FRAMERREN_MASK)
#define USART_INTENSET_PARITYERREN_MASK          (0x4000U)
#define USART_INTENSET_PARITYERREN_SHIFT         (14U)
#define USART_INTENSET_PARITYERREN(x)            (((uint32_t)(((uint32_t)(x)) << USART_INTENSET_PARITYERREN_SHIFT)) & USART_INTENSET_PARITYERREN_MASK)
#define USART_INTENSET_RXNOISEEN_MASK            (0x8000U)
#define USART_INTENSET_RXNOISEEN_SHIFT           (15U)
#define USART_INTENSET_RXNOISEEN(x)              (((uint32_t)(((uint32_t)(x)) << USART_INTENSET_RXNOISEEN_SHIFT)) & USART_INTENSET_RXNOISEEN_MASK)
#define USART_INTENSET_ABERREN_MASK              (0x10000U)
#define USART_INTENSET_ABERREN_SHIFT             (16U)
#define USART_INTENSET_ABERREN(x)                (((uint32_t)(((uint32_t)(x)) << USART_INTENSET_ABERREN_SHIFT)) & USART_INTENSET_ABERREN_MASK)

/*! @name INTENCLR - Interrupt Enable Clear register. Allows clearing any combination of bits in the INTENSET register. Writing a 1 to any implemented bit position causes the corresponding bit to be cleared. */
#define USART_INTENCLR_TXIDLECLR_MASK            (0x8U)
#define USART_INTENCLR_TXIDLECLR_SHIFT           (3U)
#define USART_INTENCLR_TXIDLECLR(x)              (((uint32_t)(((uint32_t)(x)) << USART_INTENCLR_TXIDLECLR_SHIFT)) & USART_INTENCLR_TXIDLECLR_MASK)
#define USART_INTENCLR_DELTACTSCLR_MASK          (0x20U)
#define USART_INTENCLR_DELTACTSCLR_SHIFT         (5U)
#define USART_INTENCLR_DELTACTSCLR(x)            (((uint32_t)(((uint32_t)(x)) << USART_INTENCLR_DELTACTSCLR_SHIFT)) & USART_INTENCLR_DELTACTSCLR_MASK)
#define USART_INTENCLR_TXDISCLR_MASK             (0x40U)
#define USART_INTENCLR_TXDISCLR_SHIFT            (6U)
#define USART_INTENCLR_TXDISCLR(x)               (((uint32_t)(((uint32_t)(x)) << USART_INTENCLR_TXDISCLR_SHIFT)) & USART_INTENCLR_TXDISCLR_MASK)
#define USART_INTENCLR_DELTARXBRKCLR_MASK        (0x800U)
#define USART_INTENCLR_DELTARXBRKCLR_SHIFT       (11U)
#define USART_INTENCLR_DELTARXBRKCLR(x)          (((uint32_t)(((uint32_t)(x)) << USART_INTENCLR_DELTARXBRKCLR_SHIFT)) & USART_INTENCLR_DELTARXBRKCLR_MASK)
#define USART_INTENCLR_STARTCLR_MASK             (0x1000U)
#define USART_INTENCLR_STARTCLR_SHIFT            (12U)
#define USART_INTENCLR_STARTCLR(x)               (((uint32_t)(((uint32_t)(x)) << USART_INTENCLR_STARTCLR_SHIFT)) & USART_INTENCLR_STARTCLR_MASK)
#define USART_INTENCLR_FRAMERRCLR_MASK           (0x2000U)
#define USART_INTENCLR_FRAMERRCLR_SHIFT          (13U)
#define USART_INTENCLR_FRAMERRCLR(x)             (((uint32_t)(((uint32_t)(x)) << USART_INTENCLR_FRAMERRCLR_SHIFT)) & USART_INTENCLR_FRAMERRCLR_MASK)
#define USART_INTENCLR_PARITYERRCLR_MASK         (0x4000U)
#define USART_INTENCLR_PARITYERRCLR_SHIFT        (14U)
#define USART_INTENCLR_PARITYERRCLR(x)           (((uint32_t)(((uint32_t)(x)) << USART_INTENCLR_PARITYERRCLR_SHIFT)) & USART_INTENCLR_PARITYERRCLR_MASK)
#define USART_INTENCLR_RXNOISECLR_MASK           (0x8000U)
#define USART_INTENCLR_RXNOISECLR_SHIFT          (15U)
#define USART_INTENCLR_RXNOISECLR(x)             (((uint32_t)(((uint32_t)(x)) << USART_INTENCLR_RXNOISECLR_SHIFT)) & USART_INTENCLR_RXNOISECLR_MASK)
#define USART_INTENCLR_ABERRCLR_MASK             (0x10000U)
#define USART_INTENCLR_ABERRCLR_SHIFT            (16U)
#define USART_INTENCLR_ABERRCLR(x)               (((uint32_t)(((uint32_t)(x)) << USART_INTENCLR_ABERRCLR_SHIFT)) & USART_INTENCLR_ABERRCLR_MASK)

/*! @name BRG - Baud Rate Generator register. 16-bit integer baud rate divisor value. */
#define USART_BRG_BRGVAL_MASK                    (0xFFFFU)
#define USART_BRG_BRGVAL_SHIFT                   (0U)
#define USART_BRG_BRGVAL(x)                      (((uint32_t)(((uint32_t)(x)) << USART_BRG_BRGVAL_SHIFT)) & USART_BRG_BRGVAL_MASK)

/*! @name INTSTAT - Interrupt status register. Reflects interrupts that are currently enabled. */
#define USART_INTSTAT_TXIDLE_MASK                (0x8U)
#define USART_INTSTAT_TXIDLE_SHIFT               (3U)
#define USART_INTSTAT_TXIDLE(x)                  (((uint32_t)(((uint32_t)(x)) << USART_INTSTAT_TXIDLE_SHIFT)) & USART_INTSTAT_TXIDLE_MASK)
#define USART_INTSTAT_DELTACTS_MASK              (0x20U)
#define USART_INTSTAT_DELTACTS_SHIFT             (5U)
#define USART_INTSTAT_DELTACTS(x)                (((uint32_t)(((uint32_t)(x)) << USART_INTSTAT_DELTACTS_SHIFT)) & USART_INTSTAT_DELTACTS_MASK)
#define USART_INTSTAT_TXDISINT_MASK              (0x40U)
#define USART_INTSTAT_TXDISINT_SHIFT             (6U)
#define USART_INTSTAT_TXDISINT(x)                (((uint32_t)(((uint32_t)(x)) << USART_INTSTAT_TXDISINT_SHIFT)) & USART_INTSTAT_TXDISINT_MASK)
#define USART_INTSTAT_DELTARXBRK_MASK            (0x800U)
#define USART_INTSTAT_DELTARXBRK_SHIFT           (11U)
#define USART_INTSTAT_DELTARXBRK(x)              (((uint32_t)(((uint32_t)(x)) << USART_INTSTAT_DELTARXBRK_SHIFT)) & USART_INTSTAT_DELTARXBRK_MASK)
#define USART_INTSTAT_START_MASK                 (0x1000U)
#define USART_INTSTAT_START_SHIFT                (12U)
#define USART_INTSTAT_START(x)                   (((uint32_t)(((uint32_t)(x)) << USART_INTSTAT_START_SHIFT)) & USART_INTSTAT_START_MASK)
#define USART_INTSTAT_FRAMERRINT_MASK            (0x2000U)
#define USART_INTSTAT_FRAMERRINT_SHIFT           (13U)
#define USART_INTSTAT_FRAMERRINT(x)              (((uint32_t)(((uint32_t)(x)) << USART_INTSTAT_FRAMERRINT_SHIFT)) & USART_INTSTAT_FRAMERRINT_MASK)
#define USART_INTSTAT_PARITYERRINT_MASK          (0x4000U)
#define USART_INTSTAT_PARITYERRINT_SHIFT         (14U)
#define USART_INTSTAT_PARITYERRINT(x)            (((uint32_t)(((uint32_t)(x)) << USART_INTSTAT_PARITYERRINT_SHIFT)) & USART_INTSTAT_PARITYERRINT_MASK)
#define USART_INTSTAT_RXNOISEINT_MASK            (0x8000U)
#define USART_INTSTAT_RXNOISEINT_SHIFT           (15U)
#define USART_INTSTAT_RXNOISEINT(x)              (((uint32_t)(((uint32_t)(x)) << USART_INTSTAT_RXNOISEINT_SHIFT)) & USART_INTSTAT_RXNOISEINT_MASK)
#define USART_INTSTAT_ABERRINT_MASK              (0x10000U)
#define USART_INTSTAT_ABERRINT_SHIFT             (16U)
#define USART_INTSTAT_ABERRINT(x)                (((uint32_t)(((uint32_t)(x)) << USART_INTSTAT_ABERRINT_SHIFT)) & USART_INTSTAT_ABERRINT_MASK)

/*! @name OSR - Oversample selection register for asynchronous communication. */
#define USART_OSR_OSRVAL_MASK                    (0xFU)
#define USART_OSR_OSRVAL_SHIFT                   (0U)
#define USART_OSR_OSRVAL(x)                      (((uint32_t)(((uint32_t)(x)) << USART_OSR_OSRVAL_SHIFT)) & USART_OSR_OSRVAL_MASK)

/*! @name ADDR - Address register for automatic address matching. */
#define USART_ADDR_ADDRESS_MASK                  (0xFFU)
#define USART_ADDR_ADDRESS_SHIFT                 (0U)
#define USART_ADDR_ADDRESS(x)                    (((uint32_t)(((uint32_t)(x)) << USART_ADDR_ADDRESS_SHIFT)) & USART_ADDR_ADDRESS_MASK)

/*! @name FIFOCFG - FIFO configuration and enable register. */
#define USART_FIFOCFG_ENABLETX_MASK              (0x1U)
#define USART_FIFOCFG_ENABLETX_SHIFT             (0U)
#define USART_FIFOCFG_ENABLETX(x)                (((uint32_t)(((uint32_t)(x)) << USART_FIFOCFG_ENABLETX_SHIFT)) & USART_FIFOCFG_ENABLETX_MASK)
#define USART_FIFOCFG_ENABLERX_MASK              (0x2U)
#define USART_FIFOCFG_ENABLERX_SHIFT             (1U)
#define USART_FIFOCFG_ENABLERX(x)                (((uint32_t)(((uint32_t)(x)) << USART_FIFOCFG_ENABLERX_SHIFT)) & USART_FIFOCFG_ENABLERX_MASK)
#define USART_FIFOCFG_SIZE_MASK                  (0x30U)
#define USART_FIFOCFG_SIZE_SHIFT                 (4U)
#define USART_FIFOCFG_SIZE(x)                    (((uint32_t)(((uint32_t)(x)) << USART_FIFOCFG_SIZE_SHIFT)) & USART_FIFOCFG_SIZE_MASK)
#define USART_FIFOCFG_DMATX_MASK                 (0x1000U)
#define USART_FIFOCFG_DMATX_SHIFT                (12U)
#define USART_FIFOCFG_DMATX(x)                   (((uint32_t)(((uint32_t)(x)) << USART_FIFOCFG_DMATX_SHIFT)) & USART_FIFOCFG_DMATX_MASK)
#define USART_FIFOCFG_DMARX_MASK                 (0x2000U)
#define USART_FIFOCFG_DMARX_SHIFT                (13U)
#define USART_FIFOCFG_DMARX(x)                   (((uint32_t)(((uint32_t)(x)) << USART_FIFOCFG_DMARX_SHIFT)) & USART_FIFOCFG_DMARX_MASK)
#define USART_FIFOCFG_WAKETX_MASK                (0x4000U)
#define USART_FIFOCFG_WAKETX_SHIFT               (14U)
#define USART_FIFOCFG_WAKETX(x)                  (((uint32_t)(((uint32_t)(x)) << USART_FIFOCFG_WAKETX_SHIFT)) & USART_FIFOCFG_WAKETX_MASK)
#define USART_FIFOCFG_WAKERX_MASK                (0x8000U)
#define USART_FIFOCFG_WAKERX_SHIFT               (15U)
#define USART_FIFOCFG_WAKERX(x)                  (((uint32_t)(((uint32_t)(x)) << USART_FIFOCFG_WAKERX_SHIFT)) & USART_FIFOCFG_WAKERX_MASK)
#define USART_FIFOCFG_EMPTYTX_MASK               (0x10000U)
#define USART_FIFOCFG_EMPTYTX_SHIFT              (16U)
#define USART_FIFOCFG_EMPTYTX(x)                 (((uint32_t)(((uint32_t)(x)) << USART_FIFOCFG_EMPTYTX_SHIFT)) & USART_FIFOCFG_EMPTYTX_MASK)
#define USART_FIFOCFG_EMPTYRX_MASK               (0x20000U)
#define USART_FIFOCFG_EMPTYRX_SHIFT              (17U)
#define USART_FIFOCFG_EMPTYRX(x)                 (((uint32_t)(((uint32_t)(x)) << USART_FIFOCFG_EMPTYRX_SHIFT)) & USART_FIFOCFG_EMPTYRX_MASK)

/*! @name FIFOSTAT - FIFO status register. */
#define USART_FIFOSTAT_TXERR_MASK                (0x1U)
#define USART_FIFOSTAT_TXERR_SHIFT               (0U)
#define USART_FIFOSTAT_TXERR(x)                  (((uint32_t)(((uint32_t)(x)) << USART_FIFOSTAT_TXERR_SHIFT)) & USART_FIFOSTAT_TXERR_MASK)
#define USART_FIFOSTAT_RXERR_MASK                (0x2U)
#define USART_FIFOSTAT_RXERR_SHIFT               (1U)
#define USART_FIFOSTAT_RXERR(x)                  (((uint32_t)(((uint32_t)(x)) << USART_FIFOSTAT_RXERR_SHIFT)) & USART_FIFOSTAT_RXERR_MASK)
#define USART_FIFOSTAT_PERINT_MASK               (0x8U)
#define USART_FIFOSTAT_PERINT_SHIFT              (3U)
#define USART_FIFOSTAT_PERINT(x)                 (((uint32_t)(((uint32_t)(x)) << USART_FIFOSTAT_PERINT_SHIFT)) & USART_FIFOSTAT_PERINT_MASK)
#define USART_FIFOSTAT_TXEMPTY_MASK              (0x10U)
#define USART_FIFOSTAT_TXEMPTY_SHIFT             (4U)
#define USART_FIFOSTAT_TXEMPTY(x)                (((uint32_t)(((uint32_t)(x)) << USART_FIFOSTAT_TXEMPTY_SHIFT)) & USART_FIFOSTAT_TXEMPTY_MASK)
#define USART_FIFOSTAT_TXNOTFULL_MASK            (0x20U)
#define USART_FIFOSTAT_TXNOTFULL_SHIFT           (5U)
#define USART_FIFOSTAT_TXNOTFULL(x)              (((uint32_t)(((uint32_t)(x)) << USART_FIFOSTAT_TXNOTFULL_SHIFT)) & USART_FIFOSTAT_TXNOTFULL_MASK)
#define USART_FIFOSTAT_RXNOTEMPTY_MASK           (0x40U)
#define USART_FIFOSTAT_RXNOTEMPTY_SHIFT          (6U)
#define USART_FIFOSTAT_RXNOTEMPTY(x)             (((uint32_t)(((uint32_t)(x)) << USART_FIFOSTAT_RXNOTEMPTY_SHIFT)) & USART_FIFOSTAT_RXNOTEMPTY_MASK)
#define USART_FIFOSTAT_RXFULL_MASK               (0x80U)
#define USART_FIFOSTAT_RXFULL_SHIFT              (7U)
#define USART_FIFOSTAT_RXFULL(x)                 (((uint32_t)(((uint32_t)(x)) << USART_FIFOSTAT_RXFULL_SHIFT)) & USART_FIFOSTAT_RXFULL_MASK)
#define USART_FIFOSTAT_TXLVL_MASK                (0x1F00U)
#define USART_FIFOSTAT_TXLVL_SHIFT               (8U)
#define USART_FIFOSTAT_TXLVL(x)                  (((uint32_t)(((uint32_t)(x)) << USART_FIFOSTAT_TXLVL_SHIFT)) & USART_FIFOSTAT_TXLVL_MASK)
#define USART_FIFOSTAT_RXLVL_MASK                (0x1F0000U)
#define USART_FIFOSTAT_RXLVL_SHIFT               (16U)
#define USART_FIFOSTAT_RXLVL(x)                  (((uint32_t)(((uint32_t)(x)) << USART_FIFOSTAT_RXLVL_SHIFT)) & USART_FIFOSTAT_RXLVL_MASK)

/*! @name FIFOTRIG - FIFO trigger settings for interrupt and DMA request. */
#define USART_FIFOTRIG_TXLVLENA_MASK             (0x1U)
#define USART_FIFOTRIG_TXLVLENA_SHIFT            (0U)
#define USART_FIFOTRIG_TXLVLENA(x)               (((uint32_t)(((uint32_t)(x)) << USART_FIFOTRIG_TXLVLENA_SHIFT)) & USART_FIFOTRIG_TXLVLENA_MASK)
#define USART_FIFOTRIG_RXLVLENA_MASK             (0x2U)
#define USART_FIFOTRIG_RXLVLENA_SHIFT            (1U)
#define USART_FIFOTRIG_RXLVLENA(x)               (((uint32_t)(((uint32_t)(x)) << USART_FIFOTRIG_RXLVLENA_SHIFT)) & USART_FIFOTRIG_RXLVLENA_MASK)
#define USART_FIFOTRIG_TXLVL_MASK                (0xF00U)
#define USART_FIFOTRIG_TXLVL_SHIFT               (8U)
#define USART_FIFOTRIG_TXLVL(x)                  (((uint32_t)(((uint32_t)(x)) << USART_FIFOTRIG_TXLVL_SHIFT)) & USART_FIFOTRIG_TXLVL_MASK)
#define USART_FIFOTRIG_RXLVL_MASK                (0xF0000U)
#define USART_FIFOTRIG_RXLVL_SHIFT               (16U)
#define USART_FIFOTRIG_RXLVL(x)                  (((uint32_t)(((uint32_t)(x)) << USART_FIFOTRIG_RXLVL_SHIFT)) & USART_FIFOTRIG_RXLVL_MASK)

/*! @name FIFOINTENSET - FIFO interrupt enable set (enable) and read register. */
#define USART_FIFOINTENSET_TXERR_MASK            (0x1U)
#define USART_FIFOINTENSET_TXERR_SHIFT           (0U)
#define USART_FIFOINTENSET_TXERR(x)              (((uint32_t)(((uint32_t)(x)) << USART_FIFOINTENSET_TXERR_SHIFT)) & USART_FIFOINTENSET_TXERR_MASK)
#define USART_FIFOINTENSET_RXERR_MASK            (0x2U)
#define USART_FIFOINTENSET_RXERR_SHIFT           (1U)
#define USART_FIFOINTENSET_RXERR(x)              (((uint32_t)(((uint32_t)(x)) << USART_FIFOINTENSET_RXERR_SHIFT)) & USART_FIFOINTENSET_RXERR_MASK)
#define USART_FIFOINTENSET_TXLVL_MASK            (0x4U)
#define USART_FIFOINTENSET_TXLVL_SHIFT           (2U)
#define USART_FIFOINTENSET_TXLVL(x)              (((uint32_t)(((uint32_t)(x)) << USART_FIFOINTENSET_TXLVL_SHIFT)) & USART_FIFOINTENSET_TXLVL_MASK)
#define USART_FIFOINTENSET_RXLVL_MASK            (0x8U)
#define USART_FIFOINTENSET_RXLVL_SHIFT           (3U)
#define USART_FIFOINTENSET_RXLVL(x)              (((uint32_t)(((uint32_t)(x)) << USART_FIFOINTENSET_RXLVL_SHIFT)) & USART_FIFOINTENSET_RXLVL_MASK)

/*! @name FIFOINTENCLR - FIFO interrupt enable clear (disable) and read register. */
#define USART_FIFOINTENCLR_TXERR_MASK            (0x1U)
#define USART_FIFOINTENCLR_TXERR_SHIFT           (0U)
#define USART_FIFOINTENCLR_TXERR(x)              (((uint32_t)(((uint32_t)(x)) << USART_FIFOINTENCLR_TXERR_SHIFT)) & USART_FIFOINTENCLR_TXERR_MASK)
#define USART_FIFOINTENCLR_RXERR_MASK            (0x2U)
#define USART_FIFOINTENCLR_RXERR_SHIFT           (1U)
#define USART_FIFOINTENCLR_RXERR(x)              (((uint32_t)(((uint32_t)(x)) << USART_FIFOINTENCLR_RXERR_SHIFT)) & USART_FIFOINTENCLR_RXERR_MASK)
#define USART_FIFOINTENCLR_TXLVL_MASK            (0x4U)
#define USART_FIFOINTENCLR_TXLVL_SHIFT           (2U)
#define USART_FIFOINTENCLR_TXLVL(x)              (((uint32_t)(((uint32_t)(x)) << USART_FIFOINTENCLR_TXLVL_SHIFT)) & USART_FIFOINTENCLR_TXLVL_MASK)
#define USART_FIFOINTENCLR_RXLVL_MASK            (0x8U)
#define USART_FIFOINTENCLR_RXLVL_SHIFT           (3U)
#define USART_FIFOINTENCLR_RXLVL(x)              (((uint32_t)(((uint32_t)(x)) << USART_FIFOINTENCLR_RXLVL_SHIFT)) & USART_FIFOINTENCLR_RXLVL_MASK)

/*! @name FIFOINTSTAT - FIFO interrupt status register. */
#define USART_FIFOINTSTAT_TXERR_MASK             (0x1U)
#define USART_FIFOINTSTAT_TXERR_SHIFT            (0U)
#define USART_FIFOINTSTAT_TXERR(x)               (((uint32_t)(((uint32_t)(x)) << USART_FIFOINTSTAT_TXERR_SHIFT)) & USART_FIFOINTSTAT_TXERR_MASK)
#define USART_FIFOINTSTAT_RXERR_MASK             (0x2U)
#define USART_FIFOINTSTAT_RXERR_SHIFT            (1U)
#define USART_FIFOINTSTAT_RXERR(x)               (((uint32_t)(((uint32_t)(x)) << USART_FIFOINTSTAT_RXERR_SHIFT)) & USART_FIFOINTSTAT_RXERR_MASK)
#define USART_FIFOINTSTAT_TXLVL_MASK             (0x4U)
#define USART_FIFOINTSTAT_TXLVL_SHIFT            (2U)
#define USART_FIFOINTSTAT_TXLVL(x)               (((uint32_t)(((uint32_t)(x)) << USART_FIFOINTSTAT_TXLVL_SHIFT)) & USART_FIFOINTSTAT_TXLVL_MASK)
#define USART_FIFOINTSTAT_RXLVL_MASK             (0x8U)
#define USART_FIFOINTSTAT_RXLVL_SHIFT            (3U)
#define USART_FIFOINTSTAT_RXLVL(x)               (((uint32_t)(((uint32_t)(x)) << USART_FIFOINTSTAT_RXLVL_SHIFT)) & USART_FIFOINTSTAT_RXLVL_MASK)
#define USART_FIFOINTSTAT_PERINT_MASK            (0x10U)
#define USART_FIFOINTSTAT_PERINT_SHIFT           (4U)
#define USART_FIFOINTSTAT_PERINT(x)              (((uint32_t)(((uint32_t)(x)) << USART_FIFOINTSTAT_PERINT_SHIFT)) & USART_FIFOINTSTAT_PERINT_MASK)

/*! @name FIFOWR - FIFO write data. */
#define USART_FIFOWR_TXDATA_MASK                 (0x1FFU)
#define USART_FIFOWR_TXDATA_SHIFT                (0U)
#define USART_FIFOWR_TXDATA(x)                   (((uint32_t)(((uint32_t)(x)) << USART_FIFOWR_TXDATA_SHIFT)) & USART_FIFOWR_TXDATA_MASK)

/*! @name FIFORD - FIFO read data. */
#define USART_FIFORD_RXDATA_MASK                 (0x1FFU)
#define USART_FIFORD_RXDATA_SHIFT                (0U)
#define USART_FIFORD_RXDATA(x)                   (((uint32_t)(((uint32_t)(x)) << USART_FIFORD_RXDATA_SHIFT)) & USART_FIFORD_RXDATA_MASK)
#define USART_FIFORD_FRAMERR_MASK                (0x2000U)
#define USART_FIFORD_FRAMERR_SHIFT               (13U)
#define USART_FIFORD_FRAMERR(x)                  (((uint32_t)(((uint32_t)(x)) << USART_FIFORD_FRAMERR_SHIFT)) & USART_FIFORD_FRAMERR_MASK)
#define USART_FIFORD_PARITYERR_MASK              (0x4000U)
#define USART_FIFORD_PARITYERR_SHIFT             (14U)
#define USART_FIFORD_PARITYERR(x)                (((uint32_t)(((uint32_t)(x)) << USART_FIFORD_PARITYERR_SHIFT)) & USART_FIFORD_PARITYERR_MASK)
#define USART_FIFORD_RXNOISE_MASK                (0x8000U)
#define USART_FIFORD_RXNOISE_SHIFT               (15U)
#define USART_FIFORD_RXNOISE(x)                  (((uint32_t)(((uint32_t)(x)) << USART_FIFORD_RXNOISE_SHIFT)) & USART_FIFORD_RXNOISE_MASK)

/*! @name FIFORDNOPOP - FIFO data read with no FIFO pop. */
#define USART_FIFORDNOPOP_RXDATA_MASK            (0x1FFU)
#define USART_FIFORDNOPOP_RXDATA_SHIFT           (0U)
#define USART_FIFORDNOPOP_RXDATA(x)              (((uint32_t)(((uint32_t)(x)) << USART_FIFORDNOPOP_RXDATA_SHIFT)) & USART_FIFORDNOPOP_RXDATA_MASK)
#define USART_FIFORDNOPOP_FRAMERR_MASK           (0x2000U)
#define USART_FIFORDNOPOP_FRAMERR_SHIFT          (13U)
#define USART_FIFORDNOPOP_FRAMERR(x)             (((uint32_t)(((uint32_t)(x)) << USART_FIFORDNOPOP_FRAMERR_SHIFT)) & USART_FIFORDNOPOP_FRAMERR_MASK)
#define USART_FIFORDNOPOP_PARITYERR_MASK         (0x4000U)
#define USART_FIFORDNOPOP_PARITYERR_SHIFT        (14U)
#define USART_FIFORDNOPOP_PARITYERR(x)           (((uint32_t)(((uint32_t)(x)) << USART_FIFORDNOPOP_PARITYERR_SHIFT)) & USART_FIFORDNOPOP_PARITYERR_MASK)
#define USART_FIFORDNOPOP_RXNOISE_MASK           (0x8000U)
#define USART_FIFORDNOPOP_RXNOISE_SHIFT          (15U)
#define USART_FIFORDNOPOP_RXNOISE(x)             (((uint32_t)(((uint32_t)(x)) << USART_FIFORDNOPOP_RXNOISE_SHIFT)) & USART_FIFORDNOPOP_RXNOISE_MASK)

/*! @name ID - Peripheral identification register. */
#define USART_ID_APERTURE_MASK                   (0xFFU)
#define USART_ID_APERTURE_SHIFT                  (0U)
#define USART_ID_APERTURE(x)                     (((uint32_t)(((uint32_t)(x)) << USART_ID_APERTURE_SHIFT)) & USART_ID_APERTURE_MASK)
#define USART_ID_MINOR_REV_MASK                  (0xF00U)
#define USART_ID_MINOR_REV_SHIFT                 (8U)
#define USART_ID_MINOR_REV(x)                    (((uint32_t)(((uint32_t)(x)) << USART_ID_MINOR_REV_SHIFT)) & USART_ID_MINOR_REV_MASK)
#define USART_ID_MAJOR_REV_MASK                  (0xF000U)
#define USART_ID_MAJOR_REV_SHIFT                 (12U)
#define USART_ID_MAJOR_REV(x)                    (((uint32_t)(((uint32_t)(x)) << USART_ID_MAJOR_REV_SHIFT)) & USART_ID_MAJOR_REV_MASK)
#define USART_ID_ID_MASK                         (0xFFFF0000U)
#define USART_ID_ID_SHIFT                        (16U)
#define USART_ID_ID(x)                           (((uint32_t)(((uint32_t)(x)) << USART_ID_ID_SHIFT)) & USART_ID_ID_MASK)


/*!
 * @}
 */ /* end of group USART_Register_Masks */


/* USART - Peripheral instance base addresses */
/** Peripheral USART0 base address */
#define USART0_BASE                              (0x40086000u)
/** Peripheral USART0 base pointer */
#define USART0                                   ((USART_Type *)USART0_BASE)
/** Peripheral USART1 base address */
#define USART1_BASE                              (0x40087000u)
/** Peripheral USART1 base pointer */
#define USART1                                   ((USART_Type *)USART1_BASE)
/** Peripheral USART2 base address */
#define USART2_BASE                              (0x40088000u)
/** Peripheral USART2 base pointer */
#define USART2                                   ((USART_Type *)USART2_BASE)
/** Peripheral USART3 base address */
#define USART3_BASE                              (0x40089000u)
/** Peripheral USART3 base pointer */
#define USART3                                   ((USART_Type *)USART3_BASE)
/** Peripheral USART4 base address */
#define USART4_BASE                              (0x4008A000u)
/** Peripheral USART4 base pointer */
#define USART4                                   ((USART_Type *)USART4_BASE)
/** Peripheral USART5 base address */
#define USART5_BASE                              (0x40096000u)
/** Peripheral USART5 base pointer */
#define USART5                                   ((USART_Type *)USART5_BASE)
/** Peripheral USART6 base address */
#define USART6_BASE                              (0x40097000u)
/** Peripheral USART6 base pointer */
#define USART6                                   ((USART_Type *)USART6_BASE)
/** Peripheral USART7 base address */
#define USART7_BASE                              (0x40098000u)
/** Peripheral USART7 base pointer */
#define USART7                                   ((USART_Type *)USART7_BASE)
/** Array initializer of USART peripheral base addresses */
#define USART_BASE_ADDRS                         { USART0_BASE, USART1_BASE, USART2_BASE, USART3_BASE, USART4_BASE, USART5_BASE, USART6_BASE, USART7_BASE }
/** Array initializer of USART peripheral base pointers */
#define USART_BASE_PTRS                          { USART0, USART1, USART2, USART3, USART4, USART5, USART6, USART7 }
/** Interrupt vectors for the USART peripheral type */
#define USART_IRQS                               { FLEXCOMM0_IRQn, FLEXCOMM1_IRQn, FLEXCOMM2_IRQn, FLEXCOMM3_IRQn, FLEXCOMM4_IRQn, FLEXCOMM5_IRQn, FLEXCOMM6_IRQn, FLEXCOMM7_IRQn }

/*!
 * @}
 */ /* end of group USART_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- USB Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USB_Peripheral_Access_Layer USB Peripheral Access Layer
 * @{
 */

/** USB - Register Layout Typedef */
typedef struct {
  __IO uint32_t DEVCMDSTAT;                        /**< USB Device Command/Status register, offset: 0x0 */
  __IO uint32_t INFO;                              /**< USB Info register, offset: 0x4 */
  __IO uint32_t EPLISTSTART;                       /**< USB EP Command/Status List start address, offset: 0x8 */
  __IO uint32_t DATABUFSTART;                      /**< USB Data buffer start address, offset: 0xC */
  __IO uint32_t LPM;                               /**< USB Link Power Management register, offset: 0x10 */
  __IO uint32_t EPSKIP;                            /**< USB Endpoint skip, offset: 0x14 */
  __IO uint32_t EPINUSE;                           /**< USB Endpoint Buffer in use, offset: 0x18 */
  __IO uint32_t EPBUFCFG;                          /**< USB Endpoint Buffer Configuration register, offset: 0x1C */
  __IO uint32_t INTSTAT;                           /**< USB interrupt status register, offset: 0x20 */
  __IO uint32_t INTEN;                             /**< USB interrupt enable register, offset: 0x24 */
  __IO uint32_t INTSETSTAT;                        /**< USB set interrupt status register, offset: 0x28 */
       uint8_t RESERVED_0[8];
  __I  uint32_t EPTOGGLE;                          /**< USB Endpoint toggle register, offset: 0x34 */
} USB_Type;

/* ----------------------------------------------------------------------------
   -- USB Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USB_Register_Masks USB Register Masks
 * @{
 */

/*! @name DEVCMDSTAT - USB Device Command/Status register */
#define USB_DEVCMDSTAT_DEV_ADDR_MASK             (0x7FU)
#define USB_DEVCMDSTAT_DEV_ADDR_SHIFT            (0U)
#define USB_DEVCMDSTAT_DEV_ADDR(x)               (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_DEV_ADDR_SHIFT)) & USB_DEVCMDSTAT_DEV_ADDR_MASK)
#define USB_DEVCMDSTAT_DEV_EN_MASK               (0x80U)
#define USB_DEVCMDSTAT_DEV_EN_SHIFT              (7U)
#define USB_DEVCMDSTAT_DEV_EN(x)                 (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_DEV_EN_SHIFT)) & USB_DEVCMDSTAT_DEV_EN_MASK)
#define USB_DEVCMDSTAT_SETUP_MASK                (0x100U)
#define USB_DEVCMDSTAT_SETUP_SHIFT               (8U)
#define USB_DEVCMDSTAT_SETUP(x)                  (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_SETUP_SHIFT)) & USB_DEVCMDSTAT_SETUP_MASK)
#define USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK        (0x200U)
#define USB_DEVCMDSTAT_FORCE_NEEDCLK_SHIFT       (9U)
#define USB_DEVCMDSTAT_FORCE_NEEDCLK(x)          (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_FORCE_NEEDCLK_SHIFT)) & USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK)
#define USB_DEVCMDSTAT_LPM_SUP_MASK              (0x800U)
#define USB_DEVCMDSTAT_LPM_SUP_SHIFT             (11U)
#define USB_DEVCMDSTAT_LPM_SUP(x)                (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_LPM_SUP_SHIFT)) & USB_DEVCMDSTAT_LPM_SUP_MASK)
#define USB_DEVCMDSTAT_INTONNAK_AO_MASK          (0x1000U)
#define USB_DEVCMDSTAT_INTONNAK_AO_SHIFT         (12U)
#define USB_DEVCMDSTAT_INTONNAK_AO(x)            (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_INTONNAK_AO_SHIFT)) & USB_DEVCMDSTAT_INTONNAK_AO_MASK)
#define USB_DEVCMDSTAT_INTONNAK_AI_MASK          (0x2000U)
#define USB_DEVCMDSTAT_INTONNAK_AI_SHIFT         (13U)
#define USB_DEVCMDSTAT_INTONNAK_AI(x)            (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_INTONNAK_AI_SHIFT)) & USB_DEVCMDSTAT_INTONNAK_AI_MASK)
#define USB_DEVCMDSTAT_INTONNAK_CO_MASK          (0x4000U)
#define USB_DEVCMDSTAT_INTONNAK_CO_SHIFT         (14U)
#define USB_DEVCMDSTAT_INTONNAK_CO(x)            (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_INTONNAK_CO_SHIFT)) & USB_DEVCMDSTAT_INTONNAK_CO_MASK)
#define USB_DEVCMDSTAT_INTONNAK_CI_MASK          (0x8000U)
#define USB_DEVCMDSTAT_INTONNAK_CI_SHIFT         (15U)
#define USB_DEVCMDSTAT_INTONNAK_CI(x)            (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_INTONNAK_CI_SHIFT)) & USB_DEVCMDSTAT_INTONNAK_CI_MASK)
#define USB_DEVCMDSTAT_DCON_MASK                 (0x10000U)
#define USB_DEVCMDSTAT_DCON_SHIFT                (16U)
#define USB_DEVCMDSTAT_DCON(x)                   (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_DCON_SHIFT)) & USB_DEVCMDSTAT_DCON_MASK)
#define USB_DEVCMDSTAT_DSUS_MASK                 (0x20000U)
#define USB_DEVCMDSTAT_DSUS_SHIFT                (17U)
#define USB_DEVCMDSTAT_DSUS(x)                   (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_DSUS_SHIFT)) & USB_DEVCMDSTAT_DSUS_MASK)
#define USB_DEVCMDSTAT_LPM_SUS_MASK              (0x80000U)
#define USB_DEVCMDSTAT_LPM_SUS_SHIFT             (19U)
#define USB_DEVCMDSTAT_LPM_SUS(x)                (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_LPM_SUS_SHIFT)) & USB_DEVCMDSTAT_LPM_SUS_MASK)
#define USB_DEVCMDSTAT_LPM_REWP_MASK             (0x100000U)
#define USB_DEVCMDSTAT_LPM_REWP_SHIFT            (20U)
#define USB_DEVCMDSTAT_LPM_REWP(x)               (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_LPM_REWP_SHIFT)) & USB_DEVCMDSTAT_LPM_REWP_MASK)
#define USB_DEVCMDSTAT_DCON_C_MASK               (0x1000000U)
#define USB_DEVCMDSTAT_DCON_C_SHIFT              (24U)
#define USB_DEVCMDSTAT_DCON_C(x)                 (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_DCON_C_SHIFT)) & USB_DEVCMDSTAT_DCON_C_MASK)
#define USB_DEVCMDSTAT_DSUS_C_MASK               (0x2000000U)
#define USB_DEVCMDSTAT_DSUS_C_SHIFT              (25U)
#define USB_DEVCMDSTAT_DSUS_C(x)                 (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_DSUS_C_SHIFT)) & USB_DEVCMDSTAT_DSUS_C_MASK)
#define USB_DEVCMDSTAT_DRES_C_MASK               (0x4000000U)
#define USB_DEVCMDSTAT_DRES_C_SHIFT              (26U)
#define USB_DEVCMDSTAT_DRES_C(x)                 (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_DRES_C_SHIFT)) & USB_DEVCMDSTAT_DRES_C_MASK)
#define USB_DEVCMDSTAT_VBUSDEBOUNCED_MASK        (0x10000000U)
#define USB_DEVCMDSTAT_VBUSDEBOUNCED_SHIFT       (28U)
#define USB_DEVCMDSTAT_VBUSDEBOUNCED(x)          (((uint32_t)(((uint32_t)(x)) << USB_DEVCMDSTAT_VBUSDEBOUNCED_SHIFT)) & USB_DEVCMDSTAT_VBUSDEBOUNCED_MASK)

/*! @name INFO - USB Info register */
#define USB_INFO_FRAME_NR_MASK                   (0x7FFU)
#define USB_INFO_FRAME_NR_SHIFT                  (0U)
#define USB_INFO_FRAME_NR(x)                     (((uint32_t)(((uint32_t)(x)) << USB_INFO_FRAME_NR_SHIFT)) & USB_INFO_FRAME_NR_MASK)
#define USB_INFO_ERR_CODE_MASK                   (0x7800U)
#define USB_INFO_ERR_CODE_SHIFT                  (11U)
#define USB_INFO_ERR_CODE(x)                     (((uint32_t)(((uint32_t)(x)) << USB_INFO_ERR_CODE_SHIFT)) & USB_INFO_ERR_CODE_MASK)

/*! @name EPLISTSTART - USB EP Command/Status List start address */
#define USB_EPLISTSTART_EP_LIST_MASK             (0xFFFFFF00U)
#define USB_EPLISTSTART_EP_LIST_SHIFT            (8U)
#define USB_EPLISTSTART_EP_LIST(x)               (((uint32_t)(((uint32_t)(x)) << USB_EPLISTSTART_EP_LIST_SHIFT)) & USB_EPLISTSTART_EP_LIST_MASK)

/*! @name DATABUFSTART - USB Data buffer start address */
#define USB_DATABUFSTART_DA_BUF_MASK             (0xFFC00000U)
#define USB_DATABUFSTART_DA_BUF_SHIFT            (22U)
#define USB_DATABUFSTART_DA_BUF(x)               (((uint32_t)(((uint32_t)(x)) << USB_DATABUFSTART_DA_BUF_SHIFT)) & USB_DATABUFSTART_DA_BUF_MASK)

/*! @name LPM - USB Link Power Management register */
#define USB_LPM_HIRD_HW_MASK                     (0xFU)
#define USB_LPM_HIRD_HW_SHIFT                    (0U)
#define USB_LPM_HIRD_HW(x)                       (((uint32_t)(((uint32_t)(x)) << USB_LPM_HIRD_HW_SHIFT)) & USB_LPM_HIRD_HW_MASK)
#define USB_LPM_HIRD_SW_MASK                     (0xF0U)
#define USB_LPM_HIRD_SW_SHIFT                    (4U)
#define USB_LPM_HIRD_SW(x)                       (((uint32_t)(((uint32_t)(x)) << USB_LPM_HIRD_SW_SHIFT)) & USB_LPM_HIRD_SW_MASK)
#define USB_LPM_DATA_PENDING_MASK                (0x100U)
#define USB_LPM_DATA_PENDING_SHIFT               (8U)
#define USB_LPM_DATA_PENDING(x)                  (((uint32_t)(((uint32_t)(x)) << USB_LPM_DATA_PENDING_SHIFT)) & USB_LPM_DATA_PENDING_MASK)

/*! @name EPSKIP - USB Endpoint skip */
#define USB_EPSKIP_SKIP_MASK                     (0x3FFFFFFFU)
#define USB_EPSKIP_SKIP_SHIFT                    (0U)
#define USB_EPSKIP_SKIP(x)                       (((uint32_t)(((uint32_t)(x)) << USB_EPSKIP_SKIP_SHIFT)) & USB_EPSKIP_SKIP_MASK)

/*! @name EPINUSE - USB Endpoint Buffer in use */
#define USB_EPINUSE_BUF_MASK                     (0x3FCU)
#define USB_EPINUSE_BUF_SHIFT                    (2U)
#define USB_EPINUSE_BUF(x)                       (((uint32_t)(((uint32_t)(x)) << USB_EPINUSE_BUF_SHIFT)) & USB_EPINUSE_BUF_MASK)

/*! @name EPBUFCFG - USB Endpoint Buffer Configuration register */
#define USB_EPBUFCFG_BUF_SB_MASK                 (0x3FCU)
#define USB_EPBUFCFG_BUF_SB_SHIFT                (2U)
#define USB_EPBUFCFG_BUF_SB(x)                   (((uint32_t)(((uint32_t)(x)) << USB_EPBUFCFG_BUF_SB_SHIFT)) & USB_EPBUFCFG_BUF_SB_MASK)

/*! @name INTSTAT - USB interrupt status register */
#define USB_INTSTAT_EP0OUT_MASK                  (0x1U)
#define USB_INTSTAT_EP0OUT_SHIFT                 (0U)
#define USB_INTSTAT_EP0OUT(x)                    (((uint32_t)(((uint32_t)(x)) << USB_INTSTAT_EP0OUT_SHIFT)) & USB_INTSTAT_EP0OUT_MASK)
#define USB_INTSTAT_EP0IN_MASK                   (0x2U)
#define USB_INTSTAT_EP0IN_SHIFT                  (1U)
#define USB_INTSTAT_EP0IN(x)                     (((uint32_t)(((uint32_t)(x)) << USB_INTSTAT_EP0IN_SHIFT)) & USB_INTSTAT_EP0IN_MASK)
#define USB_INTSTAT_EP1OUT_MASK                  (0x4U)
#define USB_INTSTAT_EP1OUT_SHIFT                 (2U)
#define USB_INTSTAT_EP1OUT(x)                    (((uint32_t)(((uint32_t)(x)) << USB_INTSTAT_EP1OUT_SHIFT)) & USB_INTSTAT_EP1OUT_MASK)
#define USB_INTSTAT_EP1IN_MASK                   (0x8U)
#define USB_INTSTAT_EP1IN_SHIFT                  (3U)
#define USB_INTSTAT_EP1IN(x)                     (((uint32_t)(((uint32_t)(x)) << USB_INTSTAT_EP1IN_SHIFT)) & USB_INTSTAT_EP1IN_MASK)
#define USB_INTSTAT_EP2OUT_MASK                  (0x10U)
#define USB_INTSTAT_EP2OUT_SHIFT                 (4U)
#define USB_INTSTAT_EP2OUT(x)                    (((uint32_t)(((uint32_t)(x)) << USB_INTSTAT_EP2OUT_SHIFT)) & USB_INTSTAT_EP2OUT_MASK)
#define USB_INTSTAT_EP2IN_MASK                   (0x20U)
#define USB_INTSTAT_EP2IN_SHIFT                  (5U)
#define USB_INTSTAT_EP2IN(x)                     (((uint32_t)(((uint32_t)(x)) << USB_INTSTAT_EP2IN_SHIFT)) & USB_INTSTAT_EP2IN_MASK)
#define USB_INTSTAT_EP3OUT_MASK                  (0x40U)
#define USB_INTSTAT_EP3OUT_SHIFT                 (6U)
#define USB_INTSTAT_EP3OUT(x)                    (((uint32_t)(((uint32_t)(x)) << USB_INTSTAT_EP3OUT_SHIFT)) & USB_INTSTAT_EP3OUT_MASK)
#define USB_INTSTAT_EP3IN_MASK                   (0x80U)
#define USB_INTSTAT_EP3IN_SHIFT                  (7U)
#define USB_INTSTAT_EP3IN(x)                     (((uint32_t)(((uint32_t)(x)) << USB_INTSTAT_EP3IN_SHIFT)) & USB_INTSTAT_EP3IN_MASK)
#define USB_INTSTAT_EP4OUT_MASK                  (0x100U)
#define USB_INTSTAT_EP4OUT_SHIFT                 (8U)
#define USB_INTSTAT_EP4OUT(x)                    (((uint32_t)(((uint32_t)(x)) << USB_INTSTAT_EP4OUT_SHIFT)) & USB_INTSTAT_EP4OUT_MASK)
#define USB_INTSTAT_EP4IN_MASK                   (0x200U)
#define USB_INTSTAT_EP4IN_SHIFT                  (9U)
#define USB_INTSTAT_EP4IN(x)                     (((uint32_t)(((uint32_t)(x)) << USB_INTSTAT_EP4IN_SHIFT)) & USB_INTSTAT_EP4IN_MASK)
#define USB_INTSTAT_FRAME_INT_MASK               (0x40000000U)
#define USB_INTSTAT_FRAME_INT_SHIFT              (30U)
#define USB_INTSTAT_FRAME_INT(x)                 (((uint32_t)(((uint32_t)(x)) << USB_INTSTAT_FRAME_INT_SHIFT)) & USB_INTSTAT_FRAME_INT_MASK)
#define USB_INTSTAT_DEV_INT_MASK                 (0x80000000U)
#define USB_INTSTAT_DEV_INT_SHIFT                (31U)
#define USB_INTSTAT_DEV_INT(x)                   (((uint32_t)(((uint32_t)(x)) << USB_INTSTAT_DEV_INT_SHIFT)) & USB_INTSTAT_DEV_INT_MASK)

/*! @name INTEN - USB interrupt enable register */
#define USB_INTEN_EP_INT_EN_MASK                 (0x3FFU)
#define USB_INTEN_EP_INT_EN_SHIFT                (0U)
#define USB_INTEN_EP_INT_EN(x)                   (((uint32_t)(((uint32_t)(x)) << USB_INTEN_EP_INT_EN_SHIFT)) & USB_INTEN_EP_INT_EN_MASK)
#define USB_INTEN_FRAME_INT_EN_MASK              (0x40000000U)
#define USB_INTEN_FRAME_INT_EN_SHIFT             (30U)
#define USB_INTEN_FRAME_INT_EN(x)                (((uint32_t)(((uint32_t)(x)) << USB_INTEN_FRAME_INT_EN_SHIFT)) & USB_INTEN_FRAME_INT_EN_MASK)
#define USB_INTEN_DEV_INT_EN_MASK                (0x80000000U)
#define USB_INTEN_DEV_INT_EN_SHIFT               (31U)
#define USB_INTEN_DEV_INT_EN(x)                  (((uint32_t)(((uint32_t)(x)) << USB_INTEN_DEV_INT_EN_SHIFT)) & USB_INTEN_DEV_INT_EN_MASK)

/*! @name INTSETSTAT - USB set interrupt status register */
#define USB_INTSETSTAT_EP_SET_INT_MASK           (0x3FFU)
#define USB_INTSETSTAT_EP_SET_INT_SHIFT          (0U)
#define USB_INTSETSTAT_EP_SET_INT(x)             (((uint32_t)(((uint32_t)(x)) << USB_INTSETSTAT_EP_SET_INT_SHIFT)) & USB_INTSETSTAT_EP_SET_INT_MASK)
#define USB_INTSETSTAT_FRAME_SET_INT_MASK        (0x40000000U)
#define USB_INTSETSTAT_FRAME_SET_INT_SHIFT       (30U)
#define USB_INTSETSTAT_FRAME_SET_INT(x)          (((uint32_t)(((uint32_t)(x)) << USB_INTSETSTAT_FRAME_SET_INT_SHIFT)) & USB_INTSETSTAT_FRAME_SET_INT_MASK)
#define USB_INTSETSTAT_DEV_SET_INT_MASK          (0x80000000U)
#define USB_INTSETSTAT_DEV_SET_INT_SHIFT         (31U)
#define USB_INTSETSTAT_DEV_SET_INT(x)            (((uint32_t)(((uint32_t)(x)) << USB_INTSETSTAT_DEV_SET_INT_SHIFT)) & USB_INTSETSTAT_DEV_SET_INT_MASK)

/*! @name EPTOGGLE - USB Endpoint toggle register */
#define USB_EPTOGGLE_TOGGLE_MASK                 (0x3FFU)
#define USB_EPTOGGLE_TOGGLE_SHIFT                (0U)
#define USB_EPTOGGLE_TOGGLE(x)                   (((uint32_t)(((uint32_t)(x)) << USB_EPTOGGLE_TOGGLE_SHIFT)) & USB_EPTOGGLE_TOGGLE_MASK)


/*!
 * @}
 */ /* end of group USB_Register_Masks */


/* USB - Peripheral instance base addresses */
/** Peripheral USB0 base address */
#define USB0_BASE                                (0x40084000u)
/** Peripheral USB0 base pointer */
#define USB0                                     ((USB_Type *)USB0_BASE)
/** Array initializer of USB peripheral base addresses */
#define USB_BASE_ADDRS                           { USB0_BASE }
/** Array initializer of USB peripheral base pointers */
#define USB_BASE_PTRS                            { USB0 }
/** Interrupt vectors for the USB peripheral type */
#define USB_IRQS                                 { USB0_IRQn }
#define USB_NEEDCLK_IRQS                         { USB0_NEEDCLK_IRQn }

/*!
 * @}
 */ /* end of group USB_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- UTICK Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup UTICK_Peripheral_Access_Layer UTICK Peripheral Access Layer
 * @{
 */

/** UTICK - Register Layout Typedef */
typedef struct {
  __IO uint32_t CTRL;                              /**< Control register., offset: 0x0 */
  __IO uint32_t STAT;                              /**< Status register., offset: 0x4 */
  __IO uint32_t CFG;                               /**< Capture configuration register., offset: 0x8 */
  __O  uint32_t CAPCLR;                            /**< Capture clear register., offset: 0xC */
  __I  uint32_t CAP[4];                            /**< Capture register ., array offset: 0x10, array step: 0x4 */
} UTICK_Type;

/* ----------------------------------------------------------------------------
   -- UTICK Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup UTICK_Register_Masks UTICK Register Masks
 * @{
 */

/*! @name CTRL - Control register. */
#define UTICK_CTRL_DELAYVAL_MASK                 (0x7FFFFFFFU)
#define UTICK_CTRL_DELAYVAL_SHIFT                (0U)
#define UTICK_CTRL_DELAYVAL(x)                   (((uint32_t)(((uint32_t)(x)) << UTICK_CTRL_DELAYVAL_SHIFT)) & UTICK_CTRL_DELAYVAL_MASK)
#define UTICK_CTRL_REPEAT_MASK                   (0x80000000U)
#define UTICK_CTRL_REPEAT_SHIFT                  (31U)
#define UTICK_CTRL_REPEAT(x)                     (((uint32_t)(((uint32_t)(x)) << UTICK_CTRL_REPEAT_SHIFT)) & UTICK_CTRL_REPEAT_MASK)

/*! @name STAT - Status register. */
#define UTICK_STAT_INTR_MASK                     (0x1U)
#define UTICK_STAT_INTR_SHIFT                    (0U)
#define UTICK_STAT_INTR(x)                       (((uint32_t)(((uint32_t)(x)) << UTICK_STAT_INTR_SHIFT)) & UTICK_STAT_INTR_MASK)
#define UTICK_STAT_ACTIVE_MASK                   (0x2U)
#define UTICK_STAT_ACTIVE_SHIFT                  (1U)
#define UTICK_STAT_ACTIVE(x)                     (((uint32_t)(((uint32_t)(x)) << UTICK_STAT_ACTIVE_SHIFT)) & UTICK_STAT_ACTIVE_MASK)

/*! @name CFG - Capture configuration register. */
#define UTICK_CFG_CAPEN0_MASK                    (0x1U)
#define UTICK_CFG_CAPEN0_SHIFT                   (0U)
#define UTICK_CFG_CAPEN0(x)                      (((uint32_t)(((uint32_t)(x)) << UTICK_CFG_CAPEN0_SHIFT)) & UTICK_CFG_CAPEN0_MASK)
#define UTICK_CFG_CAPEN1_MASK                    (0x2U)
#define UTICK_CFG_CAPEN1_SHIFT                   (1U)
#define UTICK_CFG_CAPEN1(x)                      (((uint32_t)(((uint32_t)(x)) << UTICK_CFG_CAPEN1_SHIFT)) & UTICK_CFG_CAPEN1_MASK)
#define UTICK_CFG_CAPEN2_MASK                    (0x4U)
#define UTICK_CFG_CAPEN2_SHIFT                   (2U)
#define UTICK_CFG_CAPEN2(x)                      (((uint32_t)(((uint32_t)(x)) << UTICK_CFG_CAPEN2_SHIFT)) & UTICK_CFG_CAPEN2_MASK)
#define UTICK_CFG_CAPEN3_MASK                    (0x8U)
#define UTICK_CFG_CAPEN3_SHIFT                   (3U)
#define UTICK_CFG_CAPEN3(x)                      (((uint32_t)(((uint32_t)(x)) << UTICK_CFG_CAPEN3_SHIFT)) & UTICK_CFG_CAPEN3_MASK)
#define UTICK_CFG_CAPPOL0_MASK                   (0x100U)
#define UTICK_CFG_CAPPOL0_SHIFT                  (8U)
#define UTICK_CFG_CAPPOL0(x)                     (((uint32_t)(((uint32_t)(x)) << UTICK_CFG_CAPPOL0_SHIFT)) & UTICK_CFG_CAPPOL0_MASK)
#define UTICK_CFG_CAPPOL1_MASK                   (0x200U)
#define UTICK_CFG_CAPPOL1_SHIFT                  (9U)
#define UTICK_CFG_CAPPOL1(x)                     (((uint32_t)(((uint32_t)(x)) << UTICK_CFG_CAPPOL1_SHIFT)) & UTICK_CFG_CAPPOL1_MASK)
#define UTICK_CFG_CAPPOL2_MASK                   (0x400U)
#define UTICK_CFG_CAPPOL2_SHIFT                  (10U)
#define UTICK_CFG_CAPPOL2(x)                     (((uint32_t)(((uint32_t)(x)) << UTICK_CFG_CAPPOL2_SHIFT)) & UTICK_CFG_CAPPOL2_MASK)
#define UTICK_CFG_CAPPOL3_MASK                   (0x800U)
#define UTICK_CFG_CAPPOL3_SHIFT                  (11U)
#define UTICK_CFG_CAPPOL3(x)                     (((uint32_t)(((uint32_t)(x)) << UTICK_CFG_CAPPOL3_SHIFT)) & UTICK_CFG_CAPPOL3_MASK)

/*! @name CAPCLR - Capture clear register. */
#define UTICK_CAPCLR_CAPCLR0_MASK                (0x1U)
#define UTICK_CAPCLR_CAPCLR0_SHIFT               (0U)
#define UTICK_CAPCLR_CAPCLR0(x)                  (((uint32_t)(((uint32_t)(x)) << UTICK_CAPCLR_CAPCLR0_SHIFT)) & UTICK_CAPCLR_CAPCLR0_MASK)
#define UTICK_CAPCLR_CAPCLR1_MASK                (0x2U)
#define UTICK_CAPCLR_CAPCLR1_SHIFT               (1U)
#define UTICK_CAPCLR_CAPCLR1(x)                  (((uint32_t)(((uint32_t)(x)) << UTICK_CAPCLR_CAPCLR1_SHIFT)) & UTICK_CAPCLR_CAPCLR1_MASK)
#define UTICK_CAPCLR_CAPCLR2_MASK                (0x4U)
#define UTICK_CAPCLR_CAPCLR2_SHIFT               (2U)
#define UTICK_CAPCLR_CAPCLR2(x)                  (((uint32_t)(((uint32_t)(x)) << UTICK_CAPCLR_CAPCLR2_SHIFT)) & UTICK_CAPCLR_CAPCLR2_MASK)
#define UTICK_CAPCLR_CAPCLR3_MASK                (0x8U)
#define UTICK_CAPCLR_CAPCLR3_SHIFT               (3U)
#define UTICK_CAPCLR_CAPCLR3(x)                  (((uint32_t)(((uint32_t)(x)) << UTICK_CAPCLR_CAPCLR3_SHIFT)) & UTICK_CAPCLR_CAPCLR3_MASK)

/*! @name CAP - Capture register . */
#define UTICK_CAP_CAP_VALUE_MASK                 (0x7FFFFFFFU)
#define UTICK_CAP_CAP_VALUE_SHIFT                (0U)
#define UTICK_CAP_CAP_VALUE(x)                   (((uint32_t)(((uint32_t)(x)) << UTICK_CAP_CAP_VALUE_SHIFT)) & UTICK_CAP_CAP_VALUE_MASK)
#define UTICK_CAP_VALID_MASK                     (0x80000000U)
#define UTICK_CAP_VALID_SHIFT                    (31U)
#define UTICK_CAP_VALID(x)                       (((uint32_t)(((uint32_t)(x)) << UTICK_CAP_VALID_SHIFT)) & UTICK_CAP_VALID_MASK)

/* The count of UTICK_CAP */
#define UTICK_CAP_COUNT                          (4U)


/*!
 * @}
 */ /* end of group UTICK_Register_Masks */


/* UTICK - Peripheral instance base addresses */
/** Peripheral UTICK0 base address */
#define UTICK0_BASE                              (0x4000E000u)
/** Peripheral UTICK0 base pointer */
#define UTICK0                                   ((UTICK_Type *)UTICK0_BASE)
/** Array initializer of UTICK peripheral base addresses */
#define UTICK_BASE_ADDRS                         { UTICK0_BASE }
/** Array initializer of UTICK peripheral base pointers */
#define UTICK_BASE_PTRS                          { UTICK0 }
/** Interrupt vectors for the UTICK peripheral type */
#define UTICK_IRQS                               { UTICK0_IRQn }

/*!
 * @}
 */ /* end of group UTICK_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- WWDT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WWDT_Peripheral_Access_Layer WWDT Peripheral Access Layer
 * @{
 */

/** WWDT - Register Layout Typedef */
typedef struct {
  __IO uint32_t MOD;                               /**< Watchdog mode register. This register contains the basic mode and status of the Watchdog Timer., offset: 0x0 */
  __IO uint32_t TC;                                /**< Watchdog timer constant register. This 24-bit register determines the time-out value., offset: 0x4 */
  __O  uint32_t FEED;                              /**< Watchdog feed sequence register. Writing 0xAA followed by 0x55 to this register reloads the Watchdog timer with the value contained in TC., offset: 0x8 */
  __I  uint32_t TV;                                /**< Watchdog timer value register. This 24-bit register reads out the current value of the Watchdog timer., offset: 0xC */
       uint8_t RESERVED_0[4];
  __IO uint32_t WARNINT;                           /**< Watchdog Warning Interrupt compare value., offset: 0x14 */
  __IO uint32_t WINDOW;                            /**< Watchdog Window compare value., offset: 0x18 */
} WWDT_Type;

/* ----------------------------------------------------------------------------
   -- WWDT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WWDT_Register_Masks WWDT Register Masks
 * @{
 */

/*! @name MOD - Watchdog mode register. This register contains the basic mode and status of the Watchdog Timer. */
#define WWDT_MOD_WDEN_MASK                       (0x1U)
#define WWDT_MOD_WDEN_SHIFT                      (0U)
#define WWDT_MOD_WDEN(x)                         (((uint32_t)(((uint32_t)(x)) << WWDT_MOD_WDEN_SHIFT)) & WWDT_MOD_WDEN_MASK)
#define WWDT_MOD_WDRESET_MASK                    (0x2U)
#define WWDT_MOD_WDRESET_SHIFT                   (1U)
#define WWDT_MOD_WDRESET(x)                      (((uint32_t)(((uint32_t)(x)) << WWDT_MOD_WDRESET_SHIFT)) & WWDT_MOD_WDRESET_MASK)
#define WWDT_MOD_WDTOF_MASK                      (0x4U)
#define WWDT_MOD_WDTOF_SHIFT                     (2U)
#define WWDT_MOD_WDTOF(x)                        (((uint32_t)(((uint32_t)(x)) << WWDT_MOD_WDTOF_SHIFT)) & WWDT_MOD_WDTOF_MASK)
#define WWDT_MOD_WDINT_MASK                      (0x8U)
#define WWDT_MOD_WDINT_SHIFT                     (3U)
#define WWDT_MOD_WDINT(x)                        (((uint32_t)(((uint32_t)(x)) << WWDT_MOD_WDINT_SHIFT)) & WWDT_MOD_WDINT_MASK)
#define WWDT_MOD_WDPROTECT_MASK                  (0x10U)
#define WWDT_MOD_WDPROTECT_SHIFT                 (4U)
#define WWDT_MOD_WDPROTECT(x)                    (((uint32_t)(((uint32_t)(x)) << WWDT_MOD_WDPROTECT_SHIFT)) & WWDT_MOD_WDPROTECT_MASK)
#define WWDT_MOD_LOCK_MASK                       (0x20U)
#define WWDT_MOD_LOCK_SHIFT                      (5U)
#define WWDT_MOD_LOCK(x)                         (((uint32_t)(((uint32_t)(x)) << WWDT_MOD_LOCK_SHIFT)) & WWDT_MOD_LOCK_MASK)

/*! @name TC - Watchdog timer constant register. This 24-bit register determines the time-out value. */
#define WWDT_TC_COUNT_MASK                       (0xFFFFFFU)
#define WWDT_TC_COUNT_SHIFT                      (0U)
#define WWDT_TC_COUNT(x)                         (((uint32_t)(((uint32_t)(x)) << WWDT_TC_COUNT_SHIFT)) & WWDT_TC_COUNT_MASK)

/*! @name FEED - Watchdog feed sequence register. Writing 0xAA followed by 0x55 to this register reloads the Watchdog timer with the value contained in TC. */
#define WWDT_FEED_FEED_MASK                      (0xFFU)
#define WWDT_FEED_FEED_SHIFT                     (0U)
#define WWDT_FEED_FEED(x)                        (((uint32_t)(((uint32_t)(x)) << WWDT_FEED_FEED_SHIFT)) & WWDT_FEED_FEED_MASK)

/*! @name TV - Watchdog timer value register. This 24-bit register reads out the current value of the Watchdog timer. */
#define WWDT_TV_COUNT_MASK                       (0xFFFFFFU)
#define WWDT_TV_COUNT_SHIFT                      (0U)
#define WWDT_TV_COUNT(x)                         (((uint32_t)(((uint32_t)(x)) << WWDT_TV_COUNT_SHIFT)) & WWDT_TV_COUNT_MASK)

/*! @name WARNINT - Watchdog Warning Interrupt compare value. */
#define WWDT_WARNINT_WARNINT_MASK                (0x3FFU)
#define WWDT_WARNINT_WARNINT_SHIFT               (0U)
#define WWDT_WARNINT_WARNINT(x)                  (((uint32_t)(((uint32_t)(x)) << WWDT_WARNINT_WARNINT_SHIFT)) & WWDT_WARNINT_WARNINT_MASK)

/*! @name WINDOW - Watchdog Window compare value. */
#define WWDT_WINDOW_WINDOW_MASK                  (0xFFFFFFU)
#define WWDT_WINDOW_WINDOW_SHIFT                 (0U)
#define WWDT_WINDOW_WINDOW(x)                    (((uint32_t)(((uint32_t)(x)) << WWDT_WINDOW_WINDOW_SHIFT)) & WWDT_WINDOW_WINDOW_MASK)


/*!
 * @}
 */ /* end of group WWDT_Register_Masks */


/* WWDT - Peripheral instance base addresses */
/** Peripheral WWDT base address */
#define WWDT_BASE                                (0x4000C000u)
/** Peripheral WWDT base pointer */
#define WWDT                                     ((WWDT_Type *)WWDT_BASE)
/** Array initializer of WWDT peripheral base addresses */
#define WWDT_BASE_ADDRS                          { WWDT_BASE }
/** Array initializer of WWDT peripheral base pointers */
#define WWDT_BASE_PTRS                           { WWDT }
/** Interrupt vectors for the WWDT peripheral type */
#define WWDT_IRQS                                { WDT_BOD_IRQn }

/*!
 * @}
 */ /* end of group WWDT_Peripheral_Access_Layer */


/*
** End of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #if (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
  #else
    #pragma pop
  #endif
#elif defined(__GNUC__)
  /* leave anonymous unions enabled */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=default
#else
  #error Not supported compiler type
#endif

/*!
 * @}
 */ /* end of group Peripheral_access_layer */


/* ----------------------------------------------------------------------------
   -- Macros for use with bit field definitions (xxx_SHIFT, xxx_MASK).
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Bit_Field_Generic_Macros Macros for use with bit field definitions (xxx_SHIFT, xxx_MASK).
 * @{
 */

#if defined(__ARMCC_VERSION)
  #if (__ARMCC_VERSION >= 6010050)
    #pragma clang system_header
  #endif
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma system_include
#endif

/**
 * @brief Mask and left-shift a bit field value for use in a register bit range.
 * @param field Name of the register bit field.
 * @param value Value of the bit field.
 * @return Masked and shifted value.
 */
#define NXP_VAL2FLD(field, value)    (((value) << (field ## _SHIFT)) & (field ## _MASK))
/**
 * @brief Mask and right-shift a register value to extract a bit field value.
 * @param field Name of the register bit field.
 * @param value Value of the register.
 * @return Masked and shifted bit field value.
 */
#define NXP_FLD2VAL(field, value)    (((value) & (field ## _MASK)) >> (field ## _SHIFT))

/*!
 * @}
 */ /* end of group Bit_Field_Generic_Macros */


/* ----------------------------------------------------------------------------
   -- SDK Compatibility
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SDK_Compatibility_Symbols SDK Compatibility
 * @{
 */

/* No SDK compatibility issues. */

/*!
 * @}
 */ /* end of group SDK_Compatibility_Symbols */


#endif  /* _LPC51U68_H_ */

