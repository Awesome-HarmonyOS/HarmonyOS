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

#include "fsl_adc.h"
#include "fsl_clock.h"

static ADC_Type *const s_adcBases[] = ADC_BASE_PTRS;
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
static const clock_ip_name_t s_adcClocks[] = ADC_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

static uint32_t ADC_GetInstance(ADC_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_adcBases); instance++)
    {
        if (s_adcBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_adcBases));

    return instance;
}

void ADC_Init(ADC_Type *base, const adc_config_t *config)
{
    assert(config != NULL);

    uint32_t tmp32 = 0U;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable clock. */
    CLOCK_EnableClock(s_adcClocks[ADC_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    /* Disable the interrupts. */
    base->INTEN = 0U; /* Quickly disable all the interrupts. */

    /* Configure the ADC block. */
    tmp32 = ADC_CTRL_CLKDIV(config->clockDividerNumber);

#if defined(FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE) & FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE
    /* Async or Sync clock mode. */
    switch (config->clockMode)
    {
        case kADC_ClockAsynchronousMode:
            tmp32 |= ADC_CTRL_ASYNMODE_MASK;
            break;
        default: /* kADC_ClockSynchronousMode */
            break;
    }
#endif /* FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE. */

#if defined(FSL_FEATURE_ADC_HAS_CTRL_RESOL) & FSL_FEATURE_ADC_HAS_CTRL_RESOL
    /* Resolution. */
    tmp32 |= ADC_CTRL_RESOL(config->resolution);
#endif/* FSL_FEATURE_ADC_HAS_CTRL_RESOL. */

#if defined(FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL) & FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL
    /* Bypass calibration. */
    if (config->enableBypassCalibration)
    {
        tmp32 |= ADC_CTRL_BYPASSCAL_MASK;
    }
#endif/* FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL. */

#if defined(FSL_FEATURE_ADC_HAS_CTRL_TSAMP) & FSL_FEATURE_ADC_HAS_CTRL_TSAMP
    /* Sample time clock count. */
    tmp32 |= ADC_CTRL_TSAMP(config->sampleTimeNumber);
#endif/* FSL_FEATURE_ADC_HAS_CTRL_TSAMP. */

#if defined(FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE) & FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE
    if(config->enableLowPowerMode)
    {
         tmp32 |= ADC_CTRL_LPWRMODE_MASK;  
    } 
#endif/* FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE. */

    base->CTRL = tmp32;
    
#if defined(FSL_FEATURE_ADC_HAS_TRIM_REG) & FSL_FEATURE_ADC_HAS_TRIM_REG
    base->ADTRIM &= ~ADC_ADTRIM_VRANGE_MASK;
    base->ADTRIM |= ADC_ADTRIM_VRANGE(config->voltageRange);
#endif/* FSL_FEATURE_ADC_HAS_TRIM_REG. */
}

void ADC_GetDefaultConfig(adc_config_t *config)
{
#if defined(FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE) & FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE
    config->clockMode = kADC_ClockSynchronousMode;
#endif/* FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE. */
    
    config->clockDividerNumber = 0U;
#if defined(FSL_FEATURE_ADC_HAS_CTRL_RESOL) & FSL_FEATURE_ADC_HAS_CTRL_RESOL
    config->resolution = kADC_Resolution12bit;
#endif/* FSL_FEATURE_ADC_HAS_CTRL_RESOL. */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL) & FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL
    config->enableBypassCalibration = false;
#endif/* FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL. */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_TSAMP) & FSL_FEATURE_ADC_HAS_CTRL_TSAMP
    config->sampleTimeNumber = 0U;
#endif/* FSL_FEATURE_ADC_HAS_CTRL_TSAMP. */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE) & FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE    
    config->enableLowPowerMode = false;
#endif/* FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE. */
#if defined(FSL_FEATURE_ADC_HAS_TRIM_REG) & FSL_FEATURE_ADC_HAS_TRIM_REG    
    config->voltageRange = kADC_HighVoltageRange;
#endif/* FSL_FEATURE_ADC_HAS_TRIM_REG. */
}

void ADC_Deinit(ADC_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Disable the clock. */
    CLOCK_DisableClock(s_adcClocks[ADC_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

#if defined(FSL_FEATURE_ADC_HAS_CALIB_REG) & FSL_FEATURE_ADC_HAS_CALIB_REG
bool ADC_DoSelfCalibration(ADC_Type *base)
{
    uint32_t i;

    /* Enable the converter. */
    /* This bit acn only be set 1 by software. It is cleared automatically whenever the ADC is powered down.
       This bit should be set after at least 10 ms after the ADC is powered on. */
    base->STARTUP = ADC_STARTUP_ADC_ENA_MASK;
    for (i = 0U; i < 0x10; i++) /* Wait a few clocks to startup up. */
    {
        __ASM("NOP");
    }
    if (!(base->STARTUP & ADC_STARTUP_ADC_ENA_MASK))
    {
        return false; /* ADC is not powered up. */
    }

    /* If not in by-pass mode, do the calibration. */
    if ((ADC_CALIB_CALREQD_MASK == (base->CALIB & ADC_CALIB_CALREQD_MASK)) &&
        (0U == (base->CTRL & ADC_CTRL_BYPASSCAL_MASK)))
    {
        /* Calibration is needed, do it now. */
        base->CALIB = ADC_CALIB_CALIB_MASK;
        i = 0xF0000;
        while ((ADC_CALIB_CALIB_MASK == (base->CALIB & ADC_CALIB_CALIB_MASK)) && (--i))
        {
        }
        if (i == 0U)
        {
            return false; /* Calibration timeout. */
        }
    }

    /* A dummy conversion cycle will be performed. */
    base->STARTUP |= ADC_STARTUP_ADC_INIT_MASK;
    i = 0x7FFFF;
    while ((ADC_STARTUP_ADC_INIT_MASK == (base->STARTUP & ADC_STARTUP_ADC_INIT_MASK)) && (--i))
    {
    }
    if (i == 0U)
    {
        return false;
    }

    return true;
}
#else
bool ADC_DoSelfCalibration(ADC_Type *base, uint32_t frequency)
{
    uint32_t tmp32; 
    uint32_t i = 0xF0000;

    /* Store the current contents of the ADC CTRL register. */
    tmp32 = base->CTRL;
    
    /* Start ADC self-calibration. */
    base->CTRL |= ADC_CTRL_CALMODE_MASK;

    /* Divide the system clock to yield an ADC clock of about 500 kHz. */
    base->CTRL &= ~ADC_CTRL_CLKDIV_MASK;
    base->CTRL |= ADC_CTRL_CLKDIV((frequency / 500000U) - 1U);
    
    /* Clear the LPWR bit. */
    base->CTRL &= ~ADC_CTRL_LPWRMODE_MASK;
    
    /* Wait for the completion of calibration. */
    while ((ADC_CTRL_CALMODE_MASK == (base->CTRL & ADC_CTRL_CALMODE_MASK)) && (--i))
    {
    }
    /* Restore the contents of the ADC CTRL register. */
    base->CTRL = tmp32;
    
    /* Judge whether the calibration is overtime.  */
    if (i == 0U)
    {
        return false; /* Calibration timeout. */
    }
    
    return true;
}
#endif/* FSL_FEATURE_ADC_HAS_CALIB_REG */

void ADC_SetConvSeqAConfig(ADC_Type *base, const adc_conv_seq_config_t *config)
{
    assert(config != NULL);

    uint32_t tmp32;

    tmp32 = ADC_SEQ_CTRL_CHANNELS(config->channelMask)   /* Channel mask. */
            | ADC_SEQ_CTRL_TRIGGER(config->triggerMask); /* Trigger mask. */

    /* Polarity for tirgger signal. */
    switch (config->triggerPolarity)
    {
        case kADC_TriggerPolarityPositiveEdge:
            tmp32 |= ADC_SEQ_CTRL_TRIGPOL_MASK;
            break;
        default: /* kADC_TriggerPolarityNegativeEdge */
            break;
    }

    /* Bypass the clock Sync. */
    if (config->enableSyncBypass)
    {
        tmp32 |= ADC_SEQ_CTRL_SYNCBYPASS_MASK;
    }

    /* Interrupt point. */
    switch (config->interruptMode)
    {
        case kADC_InterruptForEachSequence:
            tmp32 |= ADC_SEQ_CTRL_MODE_MASK;
            break;
        default: /* kADC_InterruptForEachConversion */
            break;
    }

    /* One trigger for a conversion, or for a sequence. */
    if (config->enableSingleStep)
    {
        tmp32 |= ADC_SEQ_CTRL_SINGLESTEP_MASK;
    }

    base->SEQ_CTRL[0] = tmp32;
}

void ADC_SetConvSeqBConfig(ADC_Type *base, const adc_conv_seq_config_t *config)
{
    assert(config != NULL);

    uint32_t tmp32;

    tmp32 = ADC_SEQ_CTRL_CHANNELS(config->channelMask)   /* Channel mask. */
            | ADC_SEQ_CTRL_TRIGGER(config->triggerMask); /* Trigger mask. */

    /* Polarity for tirgger signal. */
    switch (config->triggerPolarity)
    {
        case kADC_TriggerPolarityPositiveEdge:
            tmp32 |= ADC_SEQ_CTRL_TRIGPOL_MASK;
            break;
        default: /* kADC_TriggerPolarityPositiveEdge */
            break;
    }

    /* Bypass the clock Sync. */
    if (config->enableSyncBypass)
    {
        tmp32 |= ADC_SEQ_CTRL_SYNCBYPASS_MASK;
    }

    /* Interrupt point. */
    switch (config->interruptMode)
    {
        case kADC_InterruptForEachSequence:
            tmp32 |= ADC_SEQ_CTRL_MODE_MASK;
            break;
        default: /* kADC_InterruptForEachConversion */
            break;
    }

    /* One trigger for a conversion, or for a sequence. */
    if (config->enableSingleStep)
    {
        tmp32 |= ADC_SEQ_CTRL_SINGLESTEP_MASK;
    }

    base->SEQ_CTRL[1] = tmp32;
}

bool ADC_GetConvSeqAGlobalConversionResult(ADC_Type *base, adc_result_info_t *info)
{
    assert(info != NULL);

    uint32_t tmp32 = base->SEQ_GDAT[0]; /* Read to clear the status. */

    if (0U == (ADC_SEQ_GDAT_DATAVALID_MASK & tmp32))
    {
        return false;
    }

    info->result = (tmp32 & ADC_SEQ_GDAT_RESULT_MASK) >> ADC_SEQ_GDAT_RESULT_SHIFT;
    info->thresholdCompareStatus =
        (adc_threshold_compare_status_t)((tmp32 & ADC_SEQ_GDAT_THCMPRANGE_MASK) >> ADC_SEQ_GDAT_THCMPRANGE_SHIFT);
    info->thresholdCorssingStatus =
        (adc_threshold_crossing_status_t)((tmp32 & ADC_SEQ_GDAT_THCMPCROSS_MASK) >> ADC_SEQ_GDAT_THCMPCROSS_SHIFT);
    info->channelNumber = (tmp32 & ADC_SEQ_GDAT_CHN_MASK) >> ADC_SEQ_GDAT_CHN_SHIFT;
    info->overrunFlag = ((tmp32 & ADC_SEQ_GDAT_OVERRUN_MASK) == ADC_SEQ_GDAT_OVERRUN_MASK);

    return true;
}

bool ADC_GetConvSeqBGlobalConversionResult(ADC_Type *base, adc_result_info_t *info)
{
    assert(info != NULL);

    uint32_t tmp32 = base->SEQ_GDAT[1]; /* Read to clear the status. */

    if (0U == (ADC_SEQ_GDAT_DATAVALID_MASK & tmp32))
    {
        return false;
    }

    info->result = (tmp32 & ADC_SEQ_GDAT_RESULT_MASK) >> ADC_SEQ_GDAT_RESULT_SHIFT;
    info->thresholdCompareStatus =
        (adc_threshold_compare_status_t)((tmp32 & ADC_SEQ_GDAT_THCMPRANGE_MASK) >> ADC_SEQ_GDAT_THCMPRANGE_SHIFT);
    info->thresholdCorssingStatus =
        (adc_threshold_crossing_status_t)((tmp32 & ADC_SEQ_GDAT_THCMPCROSS_MASK) >> ADC_SEQ_GDAT_THCMPCROSS_SHIFT);
    info->channelNumber = (tmp32 & ADC_SEQ_GDAT_CHN_MASK) >> ADC_SEQ_GDAT_CHN_SHIFT;
    info->overrunFlag = ((tmp32 & ADC_SEQ_GDAT_OVERRUN_MASK) == ADC_SEQ_GDAT_OVERRUN_MASK);

    return true;
}

bool ADC_GetChannelConversionResult(ADC_Type *base, uint32_t channel, adc_result_info_t *info)
{
    assert(info != NULL);
    assert(channel < ADC_DAT_COUNT);

    uint32_t tmp32 = base->DAT[channel]; /* Read to clear the status. */

    if (0U == (ADC_DAT_DATAVALID_MASK & tmp32))
    {
        return false;
    }

    info->result = (tmp32 & ADC_DAT_RESULT_MASK) >> ADC_DAT_RESULT_SHIFT;
    info->thresholdCompareStatus =
        (adc_threshold_compare_status_t)((tmp32 & ADC_DAT_THCMPRANGE_MASK) >> ADC_DAT_THCMPRANGE_SHIFT);
    info->thresholdCorssingStatus =
        (adc_threshold_crossing_status_t)((tmp32 & ADC_DAT_THCMPCROSS_MASK) >> ADC_DAT_THCMPCROSS_SHIFT);
    info->channelNumber = (tmp32 & ADC_DAT_CHANNEL_MASK) >> ADC_DAT_CHANNEL_SHIFT;
    info->overrunFlag = ((tmp32 & ADC_DAT_OVERRUN_MASK) == ADC_DAT_OVERRUN_MASK);

    return true;
}
