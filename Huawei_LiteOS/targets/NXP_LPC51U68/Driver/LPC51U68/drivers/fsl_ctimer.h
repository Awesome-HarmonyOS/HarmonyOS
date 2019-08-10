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
#ifndef _FSL_CTIMER_H_
#define _FSL_CTIMER_H_

#include "fsl_common.h"

/*!
 * @addtogroup ctimer
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_CTIMER_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0 */
/*@}*/

/*! @brief List of Timer capture channels */
typedef enum _ctimer_capture_channel
{
    kCTIMER_Capture_0 = 0U, /*!< Timer capture channel 0 */
    kCTIMER_Capture_1,      /*!< Timer capture channel 1 */
    kCTIMER_Capture_2,      /*!< Timer capture channel 2 */
#if defined(FSL_FEATURE_CTIMER_HAS_CCR_CAP3) && FSL_FEATURE_CTIMER_HAS_CCR_CAP3
    kCTIMER_Capture_3 /*!< Timer capture channel 3 */
#endif                /* FSL_FEATURE_CTIMER_HAS_IR_CR3INT */
} ctimer_capture_channel_t;

/*! @brief List of capture edge options */
typedef enum _ctimer_capture_edge
{
    kCTIMER_Capture_RiseEdge = 1U, /*!< Capture on rising edge */
    kCTIMER_Capture_FallEdge = 2U, /*!< Capture on falling edge */
    kCTIMER_Capture_BothEdge = 3U, /*!< Capture on rising and falling edge */
} ctimer_capture_edge_t;

/*! @brief List of Timer match registers */
typedef enum _ctimer_match
{
    kCTIMER_Match_0 = 0U, /*!< Timer match register 0 */
    kCTIMER_Match_1,      /*!< Timer match register 1 */
    kCTIMER_Match_2,      /*!< Timer match register 2 */
    kCTIMER_Match_3       /*!< Timer match register 3 */
} ctimer_match_t;

/*! @brief List of output control options */
typedef enum _ctimer_match_output_control
{
    kCTIMER_Output_NoAction = 0U, /*!< No action is taken */
    kCTIMER_Output_Clear,         /*!< Clear the EM bit/output to 0 */
    kCTIMER_Output_Set,           /*!< Set the EM bit/output to 1 */
    kCTIMER_Output_Toggle         /*!< Toggle the EM bit/output */
} ctimer_match_output_control_t;

/*! @brief List of Timer modes */
typedef enum _ctimer_timer_mode
{
    kCTIMER_TimerMode = 0U,     /* TC is incremented every rising APB bus clock edge */
    kCTIMER_IncreaseOnRiseEdge, /* TC is incremented on rising edge of input signal */
    kCTIMER_IncreaseOnFallEdge, /* TC is incremented on falling edge of input signal */
    kCTIMER_IncreaseOnBothEdge  /* TC is incremented on both edges of input signal */
} ctimer_timer_mode_t;

/*! @brief List of Timer interrupts */
typedef enum _ctimer_interrupt_enable
{
    kCTIMER_Match0InterruptEnable = CTIMER_MCR_MR0I_MASK,    /*!< Match 0 interrupt */
    kCTIMER_Match1InterruptEnable = CTIMER_MCR_MR1I_MASK,    /*!< Match 1 interrupt */
    kCTIMER_Match2InterruptEnable = CTIMER_MCR_MR2I_MASK,    /*!< Match 2 interrupt */
    kCTIMER_Match3InterruptEnable = CTIMER_MCR_MR3I_MASK,    /*!< Match 3 interrupt */
    kCTIMER_Capture0InterruptEnable = CTIMER_CCR_CAP0I_MASK, /*!< Capture 0 interrupt */
    kCTIMER_Capture1InterruptEnable = CTIMER_CCR_CAP1I_MASK, /*!< Capture 1 interrupt */
    kCTIMER_Capture2InterruptEnable = CTIMER_CCR_CAP2I_MASK, /*!< Capture 2 interrupt */
#if defined(FSL_FEATURE_CTIMER_HAS_CCR_CAP3) && FSL_FEATURE_CTIMER_HAS_CCR_CAP3
    kCTIMER_Capture3InterruptEnable = CTIMER_CCR_CAP3I_MASK, /*!< Capture 3 interrupt */
#endif                                                       /* FSL_FEATURE_CTIMER_HAS_CCR_CAP3 */
} ctimer_interrupt_enable_t;

/*! @brief List of Timer flags */
typedef enum _ctimer_status_flags
{
    kCTIMER_Match0Flag = CTIMER_IR_MR0INT_MASK,   /*!< Match 0 interrupt flag */
    kCTIMER_Match1Flag = CTIMER_IR_MR1INT_MASK,   /*!< Match 1 interrupt flag */
    kCTIMER_Match2Flag = CTIMER_IR_MR2INT_MASK,   /*!< Match 2 interrupt flag */
    kCTIMER_Match3Flag = CTIMER_IR_MR3INT_MASK,   /*!< Match 3 interrupt flag */
    kCTIMER_Capture0Flag = CTIMER_IR_CR0INT_MASK, /*!< Capture 0 interrupt flag */
    kCTIMER_Capture1Flag = CTIMER_IR_CR1INT_MASK, /*!< Capture 1 interrupt flag */
    kCTIMER_Capture2Flag = CTIMER_IR_CR2INT_MASK, /*!< Capture 2 interrupt flag */
#if defined(FSL_FEATURE_CTIMER_HAS_IR_CR3INT) && FSL_FEATURE_CTIMER_HAS_IR_CR3INT
    kCTIMER_Capture3Flag = CTIMER_IR_CR3INT_MASK, /*!< Capture 3 interrupt flag */
#endif                                            /* FSL_FEATURE_CTIMER_HAS_IR_CR3INT */
} ctimer_status_flags_t;

typedef void (*ctimer_callback_t)(uint32_t flags);

/*! @brief Callback type when registering for a callback. When registering a callback
 *         an array of function pointers is passed the size could be 1 or 8, the callback
 *         type will tell that.
 */
typedef enum
{
    kCTIMER_SingleCallback,  /*!< Single Callback type where there is only one callback for the timer.
                                 based on the status flags different channels needs to be handled differently */
    kCTIMER_MultipleCallback /*!< Multiple Callback type where there can be 8 valid callbacks, one per channel.
                                 for both match/capture */
} ctimer_callback_type_t;

/*!
 * @brief Match configuration
 *
 * This structure holds the configuration settings for each match register.
 */
typedef struct _ctimer_match_config
{
    uint32_t matchValue;                      /*!< This is stored in the match register */
    bool enableCounterReset;                  /*!< true: Match will reset the counter
                                                   false: Match will not reser the counter */
    bool enableCounterStop;                   /*!< true: Match will stop the counter
                                                   false: Match will not stop the counter */
    ctimer_match_output_control_t outControl; /*!< Action to be taken on a match on the EM bit/output */
    bool outPinInitState;                     /*!< Initial value of the EM bit/output */
    bool enableInterrupt;                     /*!< true: Generate interrupt upon match
                                                   false: Do not generate interrupt on match */

} ctimer_match_config_t;

/*!
 * @brief Timer configuration structure
 *
 * This structure holds the configuration settings for the Timer peripheral. To initialize this
 * structure to reasonable defaults, call the CTIMER_GetDefaultConfig() function and pass a
 * pointer to the configuration structure instance.
 *
 * The configuration structure can be made constant so as to reside in flash.
 */
typedef struct _ctimer_config
{
    ctimer_timer_mode_t mode;       /*!< Timer mode */
    ctimer_capture_channel_t input; /*!< Input channel to increment the timer, used only in timer
                                        modes that rely on this input signal to increment TC */
    uint32_t prescale;              /*!< Prescale value */
} ctimer_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
 * @brief Ungates the clock and configures the peripheral for basic operation.
 *
 * @note This API should be called at the beginning of the application before using the driver.
 *
 * @param base   Ctimer peripheral base address
 * @param config Pointer to the user configuration structure.
 */
void CTIMER_Init(CTIMER_Type *base, const ctimer_config_t *config);

/*!
 * @brief Gates the timer clock.
 *
 * @param base Ctimer peripheral base address
 */
void CTIMER_Deinit(CTIMER_Type *base);

/*!
 * @brief  Fills in the timers configuration structure with the default settings.
 *
 * The default values are:
 * @code
 *   config->mode = kCTIMER_TimerMode;
 *   config->input = kCTIMER_Capture_0;
 *   config->prescale = 0;
 * @endcode
 * @param config Pointer to the user configuration structure.
 */
void CTIMER_GetDefaultConfig(ctimer_config_t *config);

/*! @}*/

/*!
 * @name PWM setup operations
 * @{
 */

/*!
 * @brief Configures the PWM signal parameters.
 *
 * Enables PWM mode on the match channel passed in and will then setup the match value
 * and other match parameters to generate a PWM signal.
 * This function will assign match channel 3 to set the PWM cycle.
 *
 * @note When setting PWM output from multiple output pins, all should use the same PWM
 * frequency
 *
 * @param base             Ctimer peripheral base address
 * @param matchChannel     Match pin to be used to output the PWM signal
 * @param dutyCyclePercent PWM pulse width; the value should be between 0 to 100
 * @param pwmFreq_Hz       PWM signal frequency in Hz
 * @param srcClock_Hz      Timer counter clock in Hz
 * @param enableInt        Enable interrupt when the timer value reaches the match value of the PWM pulse,
 *                         if it is 0 then no interrupt is generated
 *
 * @return kStatus_Success on success
 *         kStatus_Fail If matchChannel passed in is 3; this channel is reserved to set the PWM cycle
 */
status_t CTIMER_SetupPwm(CTIMER_Type *base,
                         ctimer_match_t matchChannel,
                         uint8_t dutyCyclePercent,
                         uint32_t pwmFreq_Hz,
                         uint32_t srcClock_Hz,
                         bool enableInt);

/*!
 * @brief Updates the duty cycle of an active PWM signal.
 *
 * @param base             Ctimer peripheral base address
 * @param matchChannel     Match pin to be used to output the PWM signal
 * @param dutyCyclePercent New PWM pulse width; the value should be between 0 to 100
 */
void CTIMER_UpdatePwmDutycycle(CTIMER_Type *base, ctimer_match_t matchChannel, uint8_t dutyCyclePercent);

/*! @}*/

/*!
 * @brief Setup the match register.
 *
 * User configuration is used to setup the match value and action to be taken when a match occurs.
 *
 * @param base         Ctimer peripheral base address
 * @param matchChannel Match register to configure
 * @param config       Pointer to the match configuration structure
 */
void CTIMER_SetupMatch(CTIMER_Type *base, ctimer_match_t matchChannel, const ctimer_match_config_t *config);

/*!
 * @brief Setup the capture.
 *
 * @param base      Ctimer peripheral base address
 * @param capture   Capture channel to configure
 * @param edge      Edge on the channel that will trigger a capture
 * @param enableInt Flag to enable channel interrupts, if enabled then the registered call back
 *                  is called upon capture
 */
void CTIMER_SetupCapture(CTIMER_Type *base,
                         ctimer_capture_channel_t capture,
                         ctimer_capture_edge_t edge,
                         bool enableInt);

/*!
 * @brief Register callback.
 *
 * @param base      Ctimer peripheral base address
 * @param cb_func   callback function
 * @param cb_type   callback function type, singular or multiple
 */
void CTIMER_RegisterCallBack(CTIMER_Type *base, ctimer_callback_t *cb_func, ctimer_callback_type_t cb_type);

/*!
 * @name Interrupt Interface
 * @{
 */

/*!
 * @brief Enables the selected Timer interrupts.
 *
 * @param base Ctimer peripheral base address
 * @param mask The interrupts to enable. This is a logical OR of members of the
 *             enumeration ::ctimer_interrupt_enable_t
 */
static inline void CTIMER_EnableInterrupts(CTIMER_Type *base, uint32_t mask)
{
    /* Enable match interrupts */
    base->MCR |= mask & (CTIMER_MCR_MR0I_MASK | CTIMER_MCR_MR1I_MASK | CTIMER_MCR_MR2I_MASK | CTIMER_MCR_MR3I_MASK);

    /* Enable capture interrupts */
    base->CCR |= mask & (CTIMER_CCR_CAP0I_MASK | CTIMER_CCR_CAP1I_MASK | CTIMER_CCR_CAP2I_MASK
#if defined(FSL_FEATURE_CTIMER_HAS_CCR_CAP3) && FSL_FEATURE_CTIMER_HAS_CCR_CAP3
                         | CTIMER_CCR_CAP3I_MASK
#endif /* FSL_FEATURE_CTIMER_HAS_CCR_CAP3 */
                         );
}

/*!
 * @brief Disables the selected Timer interrupts.
 *
 * @param base Ctimer peripheral base address
 * @param mask The interrupts to enable. This is a logical OR of members of the
 *             enumeration ::ctimer_interrupt_enable_t
 */
static inline void CTIMER_DisableInterrupts(CTIMER_Type *base, uint32_t mask)
{
    /* Disable match interrupts */
    base->MCR &= ~(mask & (CTIMER_MCR_MR0I_MASK | CTIMER_MCR_MR1I_MASK | CTIMER_MCR_MR2I_MASK | CTIMER_MCR_MR3I_MASK));

    /* Disable capture interrupts */
    base->CCR &= ~(mask & (CTIMER_CCR_CAP0I_MASK | CTIMER_CCR_CAP1I_MASK | CTIMER_CCR_CAP2I_MASK
#if defined(FSL_FEATURE_CTIMER_HAS_CCR_CAP3) && FSL_FEATURE_CTIMER_HAS_CCR_CAP3
                           | CTIMER_CCR_CAP3I_MASK
#endif /* FSL_FEATURE_CTIMER_HAS_CCR_CAP3 */
                           ));
}

/*!
 * @brief Gets the enabled Timer interrupts.
 *
 * @param base Ctimer peripheral base address
 *
 * @return The enabled interrupts. This is the logical OR of members of the
 *         enumeration ::ctimer_interrupt_enable_t
 */
static inline uint32_t CTIMER_GetEnabledInterrupts(CTIMER_Type *base)
{
    uint32_t enabledIntrs = 0;

    /* Get all the match interrupts enabled */
    enabledIntrs =
        base->MCR & (CTIMER_MCR_MR0I_MASK | CTIMER_MCR_MR1I_MASK | CTIMER_MCR_MR2I_MASK | CTIMER_MCR_MR3I_MASK);

    /* Get all the capture interrupts enabled */
    enabledIntrs |= base->CCR & (CTIMER_CCR_CAP0I_MASK | CTIMER_CCR_CAP1I_MASK | CTIMER_CCR_CAP2I_MASK
#if defined(FSL_FEATURE_CTIMER_HAS_CCR_CAP3) && FSL_FEATURE_CTIMER_HAS_CCR_CAP3
                                 | CTIMER_CCR_CAP3I_MASK
#endif /* FSL_FEATURE_CTIMER_HAS_CCR_CAP3 */
                                 );

    return enabledIntrs;
}

/*! @}*/

/*!
 * @name Status Interface
 * @{
 */

/*!
 * @brief Gets the Timer status flags.
 *
 * @param base Ctimer peripheral base address
 *
 * @return The status flags. This is the logical OR of members of the
 *         enumeration ::ctimer_status_flags_t
 */
static inline uint32_t CTIMER_GetStatusFlags(CTIMER_Type *base)
{
    return base->IR;
}

/*!
 * @brief Clears the Timer status flags.
 *
 * @param base Ctimer peripheral base address
 * @param mask The status flags to clear. This is a logical OR of members of the
 *             enumeration ::ctimer_status_flags_t
 */
static inline void CTIMER_ClearStatusFlags(CTIMER_Type *base, uint32_t mask)
{
    base->IR = mask;
}

/*! @}*/

/*!
 * @name Counter Start and Stop
 * @{
 */

/*!
 * @brief Starts the Timer counter.
 *
 * @param base Ctimer peripheral base address
 */
static inline void CTIMER_StartTimer(CTIMER_Type *base)
{
    base->TCR |= CTIMER_TCR_CEN_MASK;
}

/*!
 * @brief Stops the Timer counter.
 *
 * @param base Ctimer peripheral base address
 */
static inline void CTIMER_StopTimer(CTIMER_Type *base)
{
    base->TCR &= ~CTIMER_TCR_CEN_MASK;
}

/*! @}*/

/*!
 * @brief Reset the counter.
 *
 * The timer counter and prescale counter are reset on the next positive edge of the APB clock.
 *
 * @param base Ctimer peripheral base address
 */
static inline void CTIMER_Reset(CTIMER_Type *base)
{
    base->TCR |= CTIMER_TCR_CRST_MASK;
    base->TCR &= ~CTIMER_TCR_CRST_MASK;
}

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_CTIMER_H_ */
