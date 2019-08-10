/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
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

#ifndef __FSL_ADC_H__
#define __FSL_ADC_H__

#include "fsl_common.h"

/*!
 * @addtogroup lpc_adc
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief ADC driver version 2.2.0. */
#define LPC_ADC_DRIVER_VERSION (MAKE_VERSION(2, 2, 0))
/*@}*/

/*!
 * @brief Flags
 */
enum _adc_status_flags
{
    kADC_ThresholdCompareFlagOnChn0 = 1U << 0U,   /*!< Threshold comparison event on Channel 0. */
    kADC_ThresholdCompareFlagOnChn1 = 1U << 1U,   /*!< Threshold comparison event on Channel 1. */
    kADC_ThresholdCompareFlagOnChn2 = 1U << 2U,   /*!< Threshold comparison event on Channel 2. */
    kADC_ThresholdCompareFlagOnChn3 = 1U << 3U,   /*!< Threshold comparison event on Channel 3. */
    kADC_ThresholdCompareFlagOnChn4 = 1U << 4U,   /*!< Threshold comparison event on Channel 4. */
    kADC_ThresholdCompareFlagOnChn5 = 1U << 5U,   /*!< Threshold comparison event on Channel 5. */
    kADC_ThresholdCompareFlagOnChn6 = 1U << 6U,   /*!< Threshold comparison event on Channel 6. */
    kADC_ThresholdCompareFlagOnChn7 = 1U << 7U,   /*!< Threshold comparison event on Channel 7. */
    kADC_ThresholdCompareFlagOnChn8 = 1U << 8U,   /*!< Threshold comparison event on Channel 8. */
    kADC_ThresholdCompareFlagOnChn9 = 1U << 9U,   /*!< Threshold comparison event on Channel 9. */
    kADC_ThresholdCompareFlagOnChn10 = 1U << 10U, /*!< Threshold comparison event on Channel 10. */
    kADC_ThresholdCompareFlagOnChn11 = 1U << 11U, /*!< Threshold comparison event on Channel 11. */
    kADC_OverrunFlagForChn0 =
        1U << 12U, /*!< Mirror the OVERRUN status flag from the result register for ADC channel 0. */
    kADC_OverrunFlagForChn1 =
        1U << 13U, /*!< Mirror the OVERRUN status flag from the result register for ADC channel 1. */
    kADC_OverrunFlagForChn2 =
        1U << 14U, /*!< Mirror the OVERRUN status flag from the result register for ADC channel 2. */
    kADC_OverrunFlagForChn3 =
        1U << 15U, /*!< Mirror the OVERRUN status flag from the result register for ADC channel 3. */
    kADC_OverrunFlagForChn4 =
        1U << 16U, /*!< Mirror the OVERRUN status flag from the result register for ADC channel 4. */
    kADC_OverrunFlagForChn5 =
        1U << 17U, /*!< Mirror the OVERRUN status flag from the result register for ADC channel 5. */
    kADC_OverrunFlagForChn6 =
        1U << 18U, /*!< Mirror the OVERRUN status flag from the result register for ADC channel 6. */
    kADC_OverrunFlagForChn7 =
        1U << 19U, /*!< Mirror the OVERRUN status flag from the result register for ADC channel 7. */
    kADC_OverrunFlagForChn8 =
        1U << 20U, /*!< Mirror the OVERRUN status flag from the result register for ADC channel 8. */
    kADC_OverrunFlagForChn9 =
        1U << 21U, /*!< Mirror the OVERRUN status flag from the result register for ADC channel 9. */
    kADC_OverrunFlagForChn10 =
        1U << 22U, /*!< Mirror the OVERRUN status flag from the result register for ADC channel 10. */
    kADC_OverrunFlagForChn11 =
        1U << 23U, /*!< Mirror the OVERRUN status flag from the result register for ADC channel 11. */
    kADC_GlobalOverrunFlagForSeqA = 1U << 24U, /*!< Mirror the glabal OVERRUN status flag for conversion sequence A. */
    kADC_GlobalOverrunFlagForSeqB = 1U << 25U, /*!< Mirror the global OVERRUN status flag for conversion sequence B. */
    kADC_ConvSeqAInterruptFlag = 1U << 28U,    /*!< Sequence A interrupt/DMA trigger. */
    kADC_ConvSeqBInterruptFlag = 1U << 29U,    /*!< Sequence B interrupt/DMA trigger. */
    kADC_ThresholdCompareInterruptFlag = 1U << 30U, /*!< Threshold comparision interrupt flag. */
    kADC_OverrunInterruptFlag = 1U << 31U,          /*!< Overrun interrupt flag. */
};

/*!
 * @brief Interrupts
 * @note Not all the interrupt options are listed here
 */
enum _adc_interrupt_enable
{
    kADC_ConvSeqAInterruptEnable = ADC_INTEN_SEQA_INTEN_MASK, /*!< Enable interrupt upon completion of each individual
                                                                   conversion in sequence A, or entire sequence. */
    kADC_ConvSeqBInterruptEnable = ADC_INTEN_SEQB_INTEN_MASK, /*!< Enable interrupt upon completion of each individual
                                                                   conversion in sequence B, or entire sequence. */
    kADC_OverrunInterruptEnable = ADC_INTEN_OVR_INTEN_MASK, /*!< Enable the detection of an overrun condition on any of
                                                                 the channel data registers will cause an overrun
                                                                 interrupt/DMA trigger. */
};

#if defined(FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE) & FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE
/*!
 * @brief Define selection of clock mode.
 */
typedef enum _adc_clock_mode
{
    kADC_ClockSynchronousMode =
        0U, /*!< The ADC clock would be derived from the system clock based on "clockDividerNumber". */
    kADC_ClockAsynchronousMode = 1U, /*!< The ADC clock would be based on the SYSCON block's divider. */
} adc_clock_mode_t;
#endif/* FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE. */


#if defined(FSL_FEATURE_ADC_HAS_CTRL_RESOL) & FSL_FEATURE_ADC_HAS_CTRL_RESOL
/*!
 * @brief Define selection of resolution.
 */
typedef enum _adc_resolution
{
    kADC_Resolution6bit = 0U,  /*!< 6-bit resolution. */
    kADC_Resolution8bit = 1U,  /*!< 8-bit resolution. */
    kADC_Resolution10bit = 2U, /*!< 10-bit resolution. */
    kADC_Resolution12bit = 3U, /*!< 12-bit resolution. */
} adc_resolution_t;
#endif/* FSL_FEATURE_ADC_HAS_CTRL_RESOL. */

#if defined(FSL_FEATURE_ADC_HAS_TRIM_REG) & FSL_FEATURE_ADC_HAS_TRIM_REG
/*!
* @brief Definfe range of the analog supply voltage VDDA.
*/
typedef enum _adc_voltage_range
{
    kADC_HighVoltageRange = 0U,  /* High voltage. VDD = 2.7 V to 3.6 V. */
    kADC_LowVoltageRange = 1U,   /* Low voltage. VDD = 2.4 V to 2.7 V. */
} adc_vdda_range_t;
#endif/* FSL_FEATURE_ADC_HAS_TRIM_REG. */

/*!
 * @brief Define selection of polarity of selected input trigger for conversion sequence.
 */
typedef enum _adc_trigger_polarity
{
    kADC_TriggerPolarityNegativeEdge = 0U, /*!< A negative edge launches the conversion sequence on the trigger(s). */
    kADC_TriggerPolarityPositiveEdge = 1U, /*!< A positive edge launches the conversion sequence on the trigger(s). */
} adc_trigger_polarity_t;

/*!
 * @brief Define selection of conversion sequence's priority.
 */
typedef enum _adc_priority
{
    kADC_PriorityLow = 0U,  /*!< This sequence would be preempted when another sequence is started. */
    kADC_PriorityHigh = 1U, /*!< This sequence would preempt other sequence even when it is started. */
} adc_priority_t;

/*!
 * @brief Define selection of conversion sequence's interrupt.
 */
typedef enum _adc_seq_interrupt_mode
{
    kADC_InterruptForEachConversion = 0U, /*!< The sequence interrupt/DMA trigger will be set at the end of each
                                               individual ADC conversion inside this conversion sequence. */
    kADC_InterruptForEachSequence = 1U,   /*!< The sequence interrupt/DMA trigger will be set when the entire set of
                                               this sequence conversions completes. */
} adc_seq_interrupt_mode_t;

/*!
 * @brief Define status of threshold compare result.
 */
typedef enum _adc_threshold_compare_status
{
    kADC_ThresholdCompareInRange = 0U,    /*!< LOW threshold <= conversion value <= HIGH threshold. */
    kADC_ThresholdCompareBelowRange = 1U, /*!< conversion value < LOW threshold. */
    kADC_ThresholdCompareAboveRange = 2U, /*!< conversion value > HIGH threshold. */
} adc_threshold_compare_status_t;

/*!
 * @brief Define status of threshold crossing detection result.
 */
typedef enum _adc_threshold_crossing_status
{
    /* The conversion on this channel had the same relationship (above or below) to the threshold value established by
     * the designated LOW threshold value as did the previous conversion on this channel. */
    kADC_ThresholdCrossingNoDetected = 0U, /*!< No threshold Crossing detected. */

    /* Indicates that a threshold crossing in the downward direction has occurred - i.e. the previous sample on this
     * channel was above the threshold value established by the designated LOW threshold value and the current sample is
     * below that threshold. */
    kADC_ThresholdCrossingDownward = 2U, /*!< Downward Threshold Crossing detected. */

    /* Indicates that a thre shold crossing in the upward direction has occurred - i.e. the previous sample on this
     * channel was below the threshold value established by the designated LOW threshold value and the current sample is
     * above that threshold. */
    kADC_ThresholdCrossingUpward = 3U, /*!< Upward Threshold Crossing Detected. */
} adc_threshold_crossing_status_t;

/*!
 * @brief Define interrupt mode for threshold compare event.
 */
typedef enum _adc_threshold_interrupt_mode
{
    kADC_ThresholdInterruptDisabled = 0U,   /*!< Threshold comparison interrupt is disabled. */
    kADC_ThresholdInterruptOnOutside = 1U,  /*!< Threshold comparison interrupt is enabled on outside threshold. */
    kADC_ThresholdInterruptOnCrossing = 2U, /*!< Threshold comparison interrupt is enabled on crossing threshold. */
} adc_threshold_interrupt_mode_t;

/*!
 * @brief Define structure for configuring the block.
 */
typedef struct _adc_config
{
#if defined(FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE) & FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE
    adc_clock_mode_t clockMode;   /*!< Select the clock mode for ADC converter. */
#endif/* FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE. */
    uint32_t clockDividerNumber;  /*!< This field is only available when using kADC_ClockSynchronousMode for "clockMode"
                                       field. The divider would be plused by 1 based on the value in this field. The
                                       available range is in 8 bits. */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_RESOL) & FSL_FEATURE_ADC_HAS_CTRL_RESOL
    adc_resolution_t resolution;  /*!< Select the conversion bits. */
#endif/* FSL_FEATURE_ADC_HAS_CTRL_RESOL. */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL) & FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL
    bool enableBypassCalibration; /*!< By default, a calibration cycle must be performed each time the chip is
                                       powered-up. Re-calibration may be warranted periodically - especially if
                                       operating conditions have changed. To enable this option would avoid the need to
                                       calibrate if offset error is not a concern in the application. */
#endif/* FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL. */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_TSAMP) & FSL_FEATURE_ADC_HAS_CTRL_TSAMP
    uint32_t sampleTimeNumber;    /*!< By default, with value as "0U", the sample period would be 2.5 ADC clocks. Then,
                                       to plus the "sampleTimeNumber" value here. The available value range is in 3 bits.*/
#endif/* FSL_FEATURE_ADC_HAS_CTRL_TSAMP. */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE) & FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE
    bool enableLowPowerMode;    /*!< If disable low-power mode, ADC remains activated even when no conversions are requested.
                                 If enable low-power mode, The ADC is automatically powered-down when no conversions are
                                 taking place. */
#endif/* FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE. */
#if defined(FSL_FEATURE_ADC_HAS_TRIM_REG) & FSL_FEATURE_ADC_HAS_TRIM_REG
adc_vdda_range_t voltageRange;    /*!<  Configure the ADC for the appropriate operating range of the analog supply voltage VDDA.
                                        Failure to set the area correctly causes the ADC to return incorrect conversion results. */
#endif/* FSL_FEATURE_ADC_HAS_TRIM_REG. */
} adc_config_t;

/*!
 * @brief Define structure for configuring conversion sequence.
 */
typedef struct _adc_conv_seq_config
{
    uint32_t channelMask; /*!< Selects which one or more of the ADC channels will be sampled and converted when this
                               sequence is launched. The masked channels would be involved in current conversion
                               sequence, beginning with the lowest-order. The available range is in 12-bit. */
    uint32_t triggerMask; /*!< Selects which one or more of the available hardware trigger sources will cause this
                               conversion sequence to be initiated. The available range is 6-bit.*/
    adc_trigger_polarity_t triggerPolarity; /*!< Select the trigger to lauch conversion sequence. */
    bool enableSyncBypass; /*!< To enable this feature allows the hardware trigger input to bypass synchronization
                                flip-flop stages and therefore shorten the time between the trigger input signal and the
                                start of a conversion. */
    bool enableSingleStep; /*!< When enabling this feature, a trigger will launch a single conversion on the next
                                channel in the sequence instead of the default response of launching an entire sequence
                                of conversions. */
    adc_seq_interrupt_mode_t interruptMode; /*!< Select the interrpt/DMA trigger mode. */
} adc_conv_seq_config_t;

/*!
 * @brief Define structure of keeping conversion result information.
 */
typedef struct _adc_result_info
{
    uint32_t result;                                         /*!< Keep the conversion data value. */
    adc_threshold_compare_status_t thresholdCompareStatus;   /*!< Keep the threshold compare status. */
    adc_threshold_crossing_status_t thresholdCorssingStatus; /*!< Keep the threshold crossing status. */
    uint32_t channelNumber;                                  /*!< Keep the channel number for this conversion. */
    bool overrunFlag; /*!< Keep the status whether the conversion is overrun or not. */
    /* The data available flag would be returned by the reading result API. */
} adc_result_info_t;

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @name Initialization and Deinitialization
 * @{
 */

/*!
 * @brief Initialize the ADC module.
 *
 * @param base ADC peripheral base address.
 * @param config Pointer to configuration structure, see to #adc_config_t.
 */
void ADC_Init(ADC_Type *base, const adc_config_t *config);

/*!
 * @brief Deinitialize the ADC module.
 *
 * @param base ADC peripheral base address.
 */
void ADC_Deinit(ADC_Type *base);

/*!
 * @brief Gets an available pre-defined settings for initial configuration.
 *
 * This function initializes the initial configuration structure with an available settings. The default values are:
 * @code
 *   config->clockMode = kADC_ClockSynchronousMode;
 *   config->clockDividerNumber = 0U;
 *   config->resolution = kADC_Resolution12bit;
 *   config->enableBypassCalibration = false;
 *   config->sampleTimeNumber = 0U;
 * @endcode
 * @param config Pointer to configuration structure.
 */
void ADC_GetDefaultConfig(adc_config_t *config);

#if defined(FSL_FEATURE_ADC_HAS_CALIB_REG) & FSL_FEATURE_ADC_HAS_CALIB_REG
/*!
 * @brief Do the self hardware calibration.
 *
 * @param base ADC peripheral base address.
 * @retval true  Calibration succeed.
 * @retval false Calibration failed.
 */
bool ADC_DoSelfCalibration(ADC_Type *base);
#else
/*!
 * @brief Do the self calibration. To calibrate the ADC, set the ADC clock to 500 kHz.
 *        In order to achieve the specified ADC accuracy, the A/D converter must be recalibrated, at a minimum,
 *        following every chip reset before initiating normal ADC operation.
 *
 * @param base ADC peripheral base address.
 * @param frequency The ststem clock frequency to ADC.
 * @retval true  Calibration succeed.
 * @retval false Calibration failed.
 */
bool ADC_DoSelfCalibration(ADC_Type *base, uint32_t frequency);
#endif/* FSL_FEATURE_ADC_HAS_CALIB_REG */    

#if !(defined(FSL_FEATURE_ADC_HAS_NO_INSEL) && FSL_FEATURE_ADC_HAS_NO_INSEL)
/*!
 * @brief Enable the internal temperature sensor measurement.
 *
 * When enabling the internal temperature sensor measurement, the channel 0 would be connected to internal sensor
 * instead of external pin.
 *
 * @param base ADC peripheral base address.
 * @param enable Switcher to enable the feature or not.
 */
static inline void ADC_EnableTemperatureSensor(ADC_Type *base, bool enable)
{
    if (enable)
    {
        base->INSEL = (base->INSEL & ~ADC_INSEL_SEL_MASK) | ADC_INSEL_SEL(0x3);
    }
    else
    {
        base->INSEL = (base->INSEL & ~ADC_INSEL_SEL_MASK) | ADC_INSEL_SEL(0);
    }
}
#endif /* FSL_FEATURE_ADC_HAS_NO_INSEL. */
/* @} */

/*!
 * @name Control conversion sequence A.
 * @{
 */

/*!
 * @brief Enable the conversion sequence A.
 *
 * In order to avoid spuriously triggering the sequence, the trigger to conversion sequence should be ready before the
 * sequence is ready. when the sequence is disabled, the trigger would be ignored. Also, it is suggested to disable the
 * sequence during changing the sequence's setting.
 *
 * @param base ADC peripheral base address.
 * @param enable Switcher to enable the feature or not.
 */
static inline void ADC_EnableConvSeqA(ADC_Type *base, bool enable)
{
    if (enable)
    {
        base->SEQ_CTRL[0] |= ADC_SEQ_CTRL_SEQ_ENA_MASK;
    }
    else
    {
        base->SEQ_CTRL[0] &= ~ADC_SEQ_CTRL_SEQ_ENA_MASK;
    }
}

/*!
 * @brief Configure the conversion sequence A.
 *
 * @param base ADC peripheral base address.
 * @param config Pointer to configuration structure, see to #adc_conv_seq_config_t.
 */
void ADC_SetConvSeqAConfig(ADC_Type *base, const adc_conv_seq_config_t *config);

/*!
 * @brief Do trigger the sequence's conversion by software.
 *
 * @param base ADC peripheral base address.
 */
static inline void ADC_DoSoftwareTriggerConvSeqA(ADC_Type *base)
{
    base->SEQ_CTRL[0] |= ADC_SEQ_CTRL_START_MASK;
}

/*!
 * @brief Enable the burst conversion of sequence A.
 *
 * Enable the burst mode would cause the conversion sequence to be cntinuously cycled through. Other triggers would be
 * ignored while this mode is enabled. Repeated conversions could be halted by disabling this mode. And the sequence
 * currently in process will be completed before cnversions are terminated.
 * Note that a new sequence could begin just before the burst mode is disabled.
 *
 * @param base ADC peripheral base address.
 * @param enable Switcher to enable this feature.
 */
static inline void ADC_EnableConvSeqABurstMode(ADC_Type *base, bool enable)
{
    if (enable)
    {
        base->SEQ_CTRL[0] |= ADC_SEQ_CTRL_BURST_MASK;
    }
    else
    {
        base->SEQ_CTRL[0] &= ~ADC_SEQ_CTRL_BURST_MASK;
    }
}

/*!
 * @brief Set the high priority for conversion sequence A.
 *
 * @param base ADC peripheral bass address.
 */
static inline void ADC_SetConvSeqAHighPriority(ADC_Type *base)
{
    base->SEQ_CTRL[0] |= ADC_SEQ_CTRL_LOWPRIO_MASK;
}

/* @} */

/*!
 * @name Control conversion sequence B.
 * @{
 */

/*!
 * @brief Enable the conversion sequence B.
 *
 * In order to avoid spuriously triggering the sequence, the trigger to conversion sequence should be ready before the
 * sequence is ready. when the sequence is disabled, the trigger would be ignored. Also, it is suggested to disable the
 * sequence during changing the sequence's setting.
 *
 * @param base ADC peripheral base address.
 * @param enable Switcher to enable the feature or not.
 */
static inline void ADC_EnableConvSeqB(ADC_Type *base, bool enable)
{
    if (enable)
    {
        base->SEQ_CTRL[1] |= ADC_SEQ_CTRL_SEQ_ENA_MASK;
    }
    else
    {
        base->SEQ_CTRL[1] &= ~ADC_SEQ_CTRL_SEQ_ENA_MASK;
    }
}

/*!
 * @brief Configure the conversion sequence B.
 *
 * @param base ADC peripheral base address.
 * @param config Pointer to configuration structure, see to #adc_conv_seq_config_t.
 */
void ADC_SetConvSeqBConfig(ADC_Type *base, const adc_conv_seq_config_t *config);

/*!
 * @brief Do trigger the sequence's conversion by software.
 *
 * @param base ADC peripheral base address.
 */
static inline void ADC_DoSoftwareTriggerConvSeqB(ADC_Type *base)
{
    base->SEQ_CTRL[1] |= ADC_SEQ_CTRL_START_MASK;
}

/*!
 * @brief Enable the burst conversion of sequence B.
 *
 * Enable the burst mode would cause the conversion sequence to be continuously cycled through. Other triggers would be
 * ignored while this mode is enabled. Repeated conversions could be halted by disabling this mode. And the sequence
 * currently in process will be completed before cnversions are terminated.
 * Note that a new sequence could begin just before the burst mode is disabled.
 *
 * @param base ADC peripheral base address.
 * @param enable Switcher to enable this feature.
 */
static inline void ADC_EnableConvSeqBBurstMode(ADC_Type *base, bool enable)
{
    if (enable)
    {
        base->SEQ_CTRL[1] |= ADC_SEQ_CTRL_BURST_MASK;
    }
    else
    {
        base->SEQ_CTRL[1] &= ~ADC_SEQ_CTRL_BURST_MASK;
    }
}

/*!
 * @brief Set the high priority for conversion sequence B.
 *
 * @param base ADC peripheral bass address.
 */
static inline void ADC_SetConvSeqBHighPriority(ADC_Type *base)
{
    base->SEQ_CTRL[0] &= ~ADC_SEQ_CTRL_LOWPRIO_MASK;
}

/* @} */

/*!
 * @name Data result.
 * @{
 */

/*!
 * @brief Get the global ADC conversion infomation of sequence A.
 *
 * @param base ADC peripheral base address.
 * @param info Pointer to information structure, see to #adc_result_info_t;
 * @retval true  The conversion result is ready.
 * @retval false The conversion result is not ready yet.
 */
bool ADC_GetConvSeqAGlobalConversionResult(ADC_Type *base, adc_result_info_t *info);

/*!
 * @brief Get the global ADC conversion infomation of sequence B.
 *
 * @param base ADC peripheral base address.
 * @param info Pointer to information structure, see to #adc_result_info_t;
 * @retval true  The conversion result is ready.
 * @retval false The conversion result is not ready yet.
 */
bool ADC_GetConvSeqBGlobalConversionResult(ADC_Type *base, adc_result_info_t *info);

/*!
 * @brief Get the channel's ADC conversion completed under each conversion sequence.
 *
 * @param base ADC peripheral base address.
 * @param channel The indicated channel number.
 * @param info Pointer to information structure, see to #adc_result_info_t;
 * @retval true  The conversion result is ready.
 * @retval false The conversion result is not ready yet.
 */
bool ADC_GetChannelConversionResult(ADC_Type *base, uint32_t channel, adc_result_info_t *info);

/* @} */

/*!
 * @name Threshold function.
 * @{
 */

/*!
 * @brief Set the threshhold pair 0 with low and high value.
 *
 * @param base ADC peripheral base address.
 * @param lowValue LOW threshold value.
 * @param highValue HIGH threshold value.
 */
static inline void ADC_SetThresholdPair0(ADC_Type *base, uint32_t lowValue, uint32_t highValue)
{
    base->THR0_LOW = ADC_THR0_LOW_THRLOW(lowValue);
    base->THR0_HIGH = ADC_THR0_HIGH_THRHIGH(highValue);
}

/*!
 * @brief Set the threshhold pair 1 with low and high value.
 *
 * @param base ADC peripheral base address.
 * @param lowValue LOW threshold value. The available value is with 12-bit.
 * @param highValue HIGH threshold value. The available value is with 12-bit.
 */
static inline void ADC_SetThresholdPair1(ADC_Type *base, uint32_t lowValue, uint32_t highValue)
{
    base->THR1_LOW = ADC_THR1_LOW_THRLOW(lowValue);
    base->THR1_HIGH = ADC_THR1_HIGH_THRHIGH(highValue);
}

/*!
 * @brief Set given channels to apply the threshold pare 0.
 *
 * @param base ADC peripheral base address.
 * @param channelMask Indicated channels' mask.
 */
static inline void ADC_SetChannelWithThresholdPair0(ADC_Type *base, uint32_t channelMask)
{
    base->CHAN_THRSEL &= ~(channelMask);
}

/*!
 * @brief Set given channels to apply the threshold pare 1.
 *
 * @param base ADC peripheral base address.
 * @param channelMask Indicated channels' mask.
 */
static inline void ADC_SetChannelWithThresholdPair1(ADC_Type *base, uint32_t channelMask)
{
    base->CHAN_THRSEL |= channelMask;
}

/* @} */

/*!
 * @name Interrupts.
 * @{
 */

/*!
 * @brief Enable interrupts for conversion sequences.
 *
 * @param base ADC peripheral base address.
 * @param mask Mask of interrupt mask value for global block except each channal, see to #_adc_interrupt_enable.
 */
static inline void ADC_EnableInterrupts(ADC_Type *base, uint32_t mask)
{
    base->INTEN |= (0x7 & mask);
}

/*!
 * @brief Disable interrupts for conversion sequence.
 *
 * @param base ADC peripheral base address.
 * @param mask Mask of interrupt mask value for global block except each channel, see to #_adc_interrupt_enable.
 */
static inline void ADC_DisableInterrupts(ADC_Type *base, uint32_t mask)
{
    base->INTEN &= ~(0x7 & mask);
}

/*!
 * @brief Enable the interrupt of threshold compare event for each channel.
 * @deprecated Do not use this function.  It has been superceded by @ADC_EnableThresholdCompareInterrupt
 */
static inline void ADC_EnableShresholdCompareInterrupt(ADC_Type *base,
                                                       uint32_t channel,
                                                       adc_threshold_interrupt_mode_t mode)
{
    base->INTEN = (base->INTEN & ~(0x3U << ((channel << 1U) + 3U))) | ((uint32_t)(mode) << ((channel << 1U) + 3U));
}

/*!
 * @brief Enable the interrupt of threshold compare event for each channel.
 *
 * @param base ADC peripheral base address.
 * @param channel Channel number.
 * @param mode Interrupt mode for threshold compare event, see to #adc_threshold_interrupt_mode_t.
 */
static inline void ADC_EnableThresholdCompareInterrupt(ADC_Type *base,
                                                       uint32_t channel,
                                                       adc_threshold_interrupt_mode_t mode)
{
    base->INTEN = (base->INTEN & ~(0x3U << ((channel << 1U) + 3U))) | ((uint32_t)(mode) << ((channel << 1U) + 3U));
}

/* @} */

/*!
 * @name Status.
 * @{
 */

/*!
 * @brief Get status flags of ADC module.
 *
 * @param base ADC peripheral base address.
 * @return Mask of status flags of module, see to #_adc_status_flags.
 */
static inline uint32_t ADC_GetStatusFlags(ADC_Type *base)
{
    return base->FLAGS;
}

/*!
 * @brief Clear status flags of ADC module.
 *
 * @param base ADC peripheral base address.
 * @param mask Mask of status flags of module, see to #_adc_status_flags.
 */
static inline void ADC_ClearStatusFlags(ADC_Type *base, uint32_t mask)
{
    base->FLAGS = mask; /* Write 1 to clear. */
}

/* @} */

#if defined(__cplusplus)
}
#endif

/* @} */

#endif /* __FSL_ADC_H__ */
