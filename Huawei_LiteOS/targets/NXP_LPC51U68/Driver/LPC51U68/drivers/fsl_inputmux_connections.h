/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright (c) 2016, NXP
 * All rights reserved.
 *
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _FSL_INPUTMUX_CONNECTIONS_
#define _FSL_INPUTMUX_CONNECTIONS_

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @addtogroup inputmux_driver
 * @{
 */

/*! @brief Periphinmux IDs */
#define PINTSEL_PMUX_ID 0xC0U
#define DMA_TRIG0_PMUX_ID 0xE0U
#define DMA_OTRIG_PMUX_ID 0x160U
#define FREQMEAS_PMUX_ID 0x180U
#define PMUX_SHIFT 20U

/*! @brief INPUTMUX connections type */
typedef enum _inputmux_connection_t
{
    /*!< Frequency measure. */
    kINPUTMUX_MainOscToFreqmeas = 0U + (FREQMEAS_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Fro12MhzToFreqmeas = 1U + (FREQMEAS_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_WdtOscToFreqmeas = 2U + (FREQMEAS_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_32KhzOscToFreqmeas = 3U + (FREQMEAS_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_MainClkToFreqmeas = 4U + (FREQMEAS_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin4ToFreqmeas = 5U + (FREQMEAS_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin20ToFreqmeas = 6U + (FREQMEAS_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin24ToFreqmeas = 7U + (FREQMEAS_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin4ToFreqmeas = 8U + (FREQMEAS_PMUX_ID << PMUX_SHIFT),
    /*!< Pin Interrupt. */
    kINPUTMUX_GpioPort0Pin0ToPintsel = 0U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin1ToPintsel = 1U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin2ToPintsel = 2U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin3ToPintsel = 3U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin4ToPintsel = 4U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin5ToPintsel = 5U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin6ToPintsel = 6U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin7ToPintsel = 7U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin8ToPintsel = 8U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin9ToPintsel = 9U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin10ToPintsel = 10U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin11ToPintsel = 11U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin12ToPintsel = 12U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin13ToPintsel = 13U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin14ToPintsel = 14U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin15ToPintsel = 15U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin16ToPintsel = 16U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin17ToPintsel = 17U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin18ToPintsel = 18U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin19ToPintsel = 19U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin20ToPintsel = 20U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin21ToPintsel = 21U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin22ToPintsel = 22U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin23ToPintsel = 23U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin24ToPintsel = 24U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin25ToPintsel = 25U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin26ToPintsel = 26U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin27ToPintsel = 27U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin28ToPintsel = 28U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin29ToPintsel = 29U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin30ToPintsel = 30U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin31ToPintsel = 31U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin0ToPintsel = 32U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin1ToPintsel = 33U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin2ToPintsel = 34U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin3ToPintsel = 35U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin4ToPintsel = 36U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin5ToPintsel = 37U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin6ToPintsel = 38U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin7ToPintsel = 39U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin8ToPintsel = 40U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin9ToPintsel = 41U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin10ToPintsel = 42U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin11ToPintsel = 43U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin12ToPintsel = 44U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin13ToPintsel = 45U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin14ToPintsel = 46U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin15ToPintsel = 47U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin16ToPintsel = 48U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin17ToPintsel = 49U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin18ToPintsel = 50U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin19ToPintsel = 51U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin20ToPintsel = 52U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin21ToPintsel = 53U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin22ToPintsel = 54U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin23ToPintsel = 55U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin24ToPintsel = 56U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin25ToPintsel = 57U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin26ToPintsel = 58U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin27ToPintsel = 59U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin28ToPintsel = 60U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin29ToPintsel = 61U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin30ToPintsel = 62U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort1Pin31ToPintsel = 63U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    /*!< DMA ITRIG. */
    kINPUTMUX_Adc0SeqaIrqToDma = 0U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_ADC0SeqbIrqToDma = 1U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Sct0DmaReq0ToDma = 2U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Sct0DmaReq1ToDma = 3U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Ctimer0M0ToDma = 4U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Ctimer0M1ToDma = 5U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Ctimer1M0ToDma = 6U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Ctimer3M0ToDma = 9U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_PinInt0ToDma = 12U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_PinInt1ToDma = 13U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_PinInt2ToDma = 14U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_PinInt3ToDma = 15U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Otrig0ToDma = 16U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Otrig1ToDma = 17U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Otrig2ToDma = 18U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Otrig3ToDma = 19U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    /*!< DMA OTRIG. */
    kINPUTMUX_DmaFlexcomm0RxTrigoutToTriginChannels = 0U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm0TxTrigoutToTriginChannels = 1U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm1RxTrigoutToTriginChannels = 2U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm1TxTrigoutToTriginChannels = 3U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm2RxTrigoutToTriginChannels = 4U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm2TxTrigoutToTriginChannels = 5U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm3RxTrigoutToTriginChannels = 6U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm3TxTrigoutToTriginChannels = 7U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm4RxTrigoutToTriginChannels = 8U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm4TxTrigoutToTriginChannels = 9U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm5RxTrigoutToTriginChannels = 10U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm5TxTrigoutToTriginChannels = 11U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm6RxTrigoutToTriginChannels = 12U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm6TxTrigoutToTriginChannels = 13U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm7RxTrigoutToTriginChannels = 14U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm7TxTrigoutToTriginChannels = 15U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaChannel18_TrigoutToTriginChannels = 18U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaChannel19_TrigoutToTriginChannels = 19U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
} inputmux_connection_t;

/*@}*/

#endif /* _FSL_INPUTMUX_CONNECTIONS_ */
