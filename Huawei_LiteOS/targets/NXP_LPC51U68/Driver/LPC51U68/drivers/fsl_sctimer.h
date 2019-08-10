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
#ifndef _FSL_SCTIMER_H_
#define _FSL_SCTIMER_H_

#include "fsl_common.h"

/*!
 * @addtogroup sctimer
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_SCTIMER_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0 */
/*@}*/

/*! @brief SCTimer PWM operation modes */
typedef enum _sctimer_pwm_mode
{
    kSCTIMER_EdgeAlignedPwm = 0U, /*!< Edge-aligned PWM */
    kSCTIMER_CenterAlignedPwm     /*!< Center-aligned PWM */
} sctimer_pwm_mode_t;

/*! @brief SCTimer counters when working as two independent 16-bit counters */
typedef enum _sctimer_counter
{
    kSCTIMER_Counter_L = 0U, /*!< Counter L */
    kSCTIMER_Counter_H       /*!< Counter H */
} sctimer_counter_t;

/*! @brief List of SCTimer input pins */
typedef enum _sctimer_input
{
    kSCTIMER_Input_0 = 0U, /*!< SCTIMER input 0 */
    kSCTIMER_Input_1,      /*!< SCTIMER input 1 */
    kSCTIMER_Input_2,      /*!< SCTIMER input 2 */
    kSCTIMER_Input_3,      /*!< SCTIMER input 3 */
    kSCTIMER_Input_4,      /*!< SCTIMER input 4 */
    kSCTIMER_Input_5,      /*!< SCTIMER input 5 */
    kSCTIMER_Input_6,      /*!< SCTIMER input 6 */
    kSCTIMER_Input_7       /*!< SCTIMER input 7 */
} sctimer_input_t;

/*! @brief List of SCTimer output pins */
typedef enum _sctimer_out
{
    kSCTIMER_Out_0 = 0U, /*!< SCTIMER output 0*/
    kSCTIMER_Out_1,      /*!< SCTIMER output 1 */
    kSCTIMER_Out_2,      /*!< SCTIMER output 2 */
    kSCTIMER_Out_3,      /*!< SCTIMER output 3 */
    kSCTIMER_Out_4,      /*!< SCTIMER output 4 */
    kSCTIMER_Out_5,      /*!< SCTIMER output 5 */
    kSCTIMER_Out_6,      /*!< SCTIMER output 6 */
    kSCTIMER_Out_7       /*!< SCTIMER output 7 */
} sctimer_out_t;

/*! @brief SCTimer PWM output pulse mode: high-true, low-true or no output */
typedef enum _sctimer_pwm_level_select
{
    kSCTIMER_LowTrue = 0U, /*!< Low true pulses */
    kSCTIMER_HighTrue      /*!< High true pulses */
} sctimer_pwm_level_select_t;

/*! @brief Options to configure a SCTimer PWM signal */
typedef struct _sctimer_pwm_signal_param
{
    sctimer_out_t output;             /*!< The output pin to use to generate the PWM signal */
    sctimer_pwm_level_select_t level; /*!< PWM output active level select. */
    uint8_t dutyCyclePercent;         /*!< PWM pulse width, value should be between 1 to 100
                                           100 = always active signal (100% duty cycle).*/
} sctimer_pwm_signal_param_t;

/*! @brief SCTimer clock mode options */
typedef enum _sctimer_clock_mode
{
    kSCTIMER_System_ClockMode = 0U, /*!< System Clock Mode */
    kSCTIMER_Sampled_ClockMode,     /*!< Sampled System Clock Mode */
    kSCTIMER_Input_ClockMode,       /*!< SCT Input Clock Mode */
    kSCTIMER_Asynchronous_ClockMode /*!< Asynchronous Mode */
} sctimer_clock_mode_t;

/*! @brief SCTimer clock select options */
typedef enum _sctimer_clock_select
{
    kSCTIMER_Clock_On_Rise_Input_0 = 0U, /*!< Rising edges on input 0 */
    kSCTIMER_Clock_On_Fall_Input_0,      /*!< Falling edges on input 0 */
    kSCTIMER_Clock_On_Rise_Input_1,      /*!< Rising edges on input 1 */
    kSCTIMER_Clock_On_Fall_Input_1,      /*!< Falling edges on input 1 */
    kSCTIMER_Clock_On_Rise_Input_2,      /*!< Rising edges on input 2 */
    kSCTIMER_Clock_On_Fall_Input_2,      /*!< Falling edges on input 2 */
    kSCTIMER_Clock_On_Rise_Input_3,      /*!< Rising edges on input 3 */
    kSCTIMER_Clock_On_Fall_Input_3,      /*!< Falling edges on input 3 */
    kSCTIMER_Clock_On_Rise_Input_4,      /*!< Rising edges on input 4 */
    kSCTIMER_Clock_On_Fall_Input_4,      /*!< Falling edges on input 4 */
    kSCTIMER_Clock_On_Rise_Input_5,      /*!< Rising edges on input 5 */
    kSCTIMER_Clock_On_Fall_Input_5,      /*!< Falling edges on input 5 */
    kSCTIMER_Clock_On_Rise_Input_6,      /*!< Rising edges on input 6 */
    kSCTIMER_Clock_On_Fall_Input_6,      /*!< Falling edges on input 6 */
    kSCTIMER_Clock_On_Rise_Input_7,      /*!< Rising edges on input 7 */
    kSCTIMER_Clock_On_Fall_Input_7       /*!< Falling edges on input 7 */
} sctimer_clock_select_t;

/*!
 * @brief SCTimer output conflict resolution options.
 *
 * Specifies what action should be taken if multiple events dictate that a given output should be
 * both set and cleared at the same time
 */
typedef enum _sctimer_conflict_resolution
{
    kSCTIMER_ResolveNone = 0U, /*!< No change */
    kSCTIMER_ResolveSet,       /*!< Set output */
    kSCTIMER_ResolveClear,     /*!< Clear output */
    kSCTIMER_ResolveToggle     /*!< Toggle output */
} sctimer_conflict_resolution_t;

/*! @brief List of SCTimer event types */
typedef enum _sctimer_event
{
    kSCTIMER_InputLowOrMatchEvent =
        (0 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (0 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (0 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_InputRiseOrMatchEvent =
        (0 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (1 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (0 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_InputFallOrMatchEvent =
        (0 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (2 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (0 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_InputHighOrMatchEvent =
        (0 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (3 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (0 << SCT_EVENT_CTRL_OUTSEL_SHIFT),

    kSCTIMER_MatchEventOnly =
        (1 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (0 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (0 << SCT_EVENT_CTRL_OUTSEL_SHIFT),

    kSCTIMER_InputLowEvent =
        (2 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (0 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (0 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_InputRiseEvent =
        (2 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (1 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (0 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_InputFallEvent =
        (2 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (2 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (0 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_InputHighEvent =
        (2 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (3 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (0 << SCT_EVENT_CTRL_OUTSEL_SHIFT),

    kSCTIMER_InputLowAndMatchEvent =
        (3 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (0 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (0 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_InputRiseAndMatchEvent =
        (3 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (1 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (0 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_InputFallAndMatchEvent =
        (3 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (2 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (0 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_InputHighAndMatchEvent =
        (3 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (3 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (0 << SCT_EVENT_CTRL_OUTSEL_SHIFT),

    kSCTIMER_OutputLowOrMatchEvent =
        (0 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (0 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (1 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_OutputRiseOrMatchEvent =
        (0 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (1 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (1 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_OutputFallOrMatchEvent =
        (0 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (2 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (1 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_OutputHighOrMatchEvent =
        (0 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (3 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (1 << SCT_EVENT_CTRL_OUTSEL_SHIFT),

    kSCTIMER_OutputLowEvent =
        (2 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (0 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (1 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_OutputRiseEvent =
        (2 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (1 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (1 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_OutputFallEvent =
        (2 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (2 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (1 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_OutputHighEvent =
        (2 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (3 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (1 << SCT_EVENT_CTRL_OUTSEL_SHIFT),

    kSCTIMER_OutputLowAndMatchEvent =
        (3 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (0 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (1 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_OutputRiseAndMatchEvent =
        (3 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (1 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (1 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_OutputFallAndMatchEvent =
        (3 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (2 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (1 << SCT_EVENT_CTRL_OUTSEL_SHIFT),
    kSCTIMER_OutputHighAndMatchEvent =
        (3 << SCT_EVENT_CTRL_COMBMODE_SHIFT) + (3 << SCT_EVENT_CTRL_IOCOND_SHIFT) + (1 << SCT_EVENT_CTRL_OUTSEL_SHIFT)
} sctimer_event_t;

/*! @brief SCTimer callback typedef. */
typedef void (*sctimer_event_callback_t)(void);

/*! @brief List of SCTimer interrupts */
typedef enum _sctimer_interrupt_enable
{
    kSCTIMER_Event0InterruptEnable = (1U << 0),   /*!< Event 0 interrupt */
    kSCTIMER_Event1InterruptEnable = (1U << 1),   /*!< Event 1 interrupt */
    kSCTIMER_Event2InterruptEnable = (1U << 2),   /*!< Event 2 interrupt */
    kSCTIMER_Event3InterruptEnable = (1U << 3),   /*!< Event 3 interrupt */
    kSCTIMER_Event4InterruptEnable = (1U << 4),   /*!< Event 4 interrupt */
    kSCTIMER_Event5InterruptEnable = (1U << 5),   /*!< Event 5 interrupt */
    kSCTIMER_Event6InterruptEnable = (1U << 6),   /*!< Event 6 interrupt */
    kSCTIMER_Event7InterruptEnable = (1U << 7),   /*!< Event 7 interrupt */
    kSCTIMER_Event8InterruptEnable = (1U << 8),   /*!< Event 8 interrupt */
    kSCTIMER_Event9InterruptEnable = (1U << 9),   /*!< Event 9 interrupt */
    kSCTIMER_Event10InterruptEnable = (1U << 10), /*!< Event 10 interrupt */
    kSCTIMER_Event11InterruptEnable = (1U << 11), /*!< Event 11 interrupt */
    kSCTIMER_Event12InterruptEnable = (1U << 12), /*!< Event 12 interrupt */
} sctimer_interrupt_enable_t;

/*! @brief List of SCTimer flags */
typedef enum _sctimer_status_flags
{
    kSCTIMER_Event0Flag = (1U << 0),   /*!< Event 0 Flag */
    kSCTIMER_Event1Flag = (1U << 1),   /*!< Event 1 Flag */
    kSCTIMER_Event2Flag = (1U << 2),   /*!< Event 2 Flag */
    kSCTIMER_Event3Flag = (1U << 3),   /*!< Event 3 Flag */
    kSCTIMER_Event4Flag = (1U << 4),   /*!< Event 4 Flag */
    kSCTIMER_Event5Flag = (1U << 5),   /*!< Event 5 Flag */
    kSCTIMER_Event6Flag = (1U << 6),   /*!< Event 6 Flag */
    kSCTIMER_Event7Flag = (1U << 7),   /*!< Event 7 Flag */
    kSCTIMER_Event8Flag = (1U << 8),   /*!< Event 8 Flag */
    kSCTIMER_Event9Flag = (1U << 9),   /*!< Event 9 Flag */
    kSCTIMER_Event10Flag = (1U << 10), /*!< Event 10 Flag */
    kSCTIMER_Event11Flag = (1U << 11), /*!< Event 11 Flag */
    kSCTIMER_Event12Flag = (1U << 12), /*!< Event 12 Flag */
    kSCTIMER_BusErrorLFlag =
        (1U << SCT_CONFLAG_BUSERRL_SHIFT), /*!< Bus error due to write when L counter was not halted */
    kSCTIMER_BusErrorHFlag =
        (1U << SCT_CONFLAG_BUSERRH_SHIFT) /*!< Bus error due to write when H counter was not halted */
} sctimer_status_flags_t;

/*!
 * @brief SCTimer configuration structure
 *
 * This structure holds the configuration settings for the SCTimer peripheral. To initialize this
 * structure to reasonable defaults, call the SCTMR_GetDefaultConfig() function and pass a
 * pointer to the configuration structure instance.
 *
 * The configuration structure can be made constant so as to reside in flash.
 */
typedef struct _sctimer_config
{
    bool enableCounterUnify;            /*!< true: SCT operates as a unified 32-bit counter;
                                             false: SCT operates as two 16-bit counters */
    sctimer_clock_mode_t clockMode;     /*!< SCT clock mode value */
    sctimer_clock_select_t clockSelect; /*!< SCT clock select value */
    bool enableBidirection_l;           /*!< true: Up-down count mode for the L or unified counter
                                             false: Up count mode only for the L or unified counter */
    bool enableBidirection_h;           /*!< true: Up-down count mode for the H or unified counter
                                             false: Up count mode only for the H or unified counter.
                                             This field is used only if the enableCounterUnify is set
                                             to false */
    uint8_t prescale_l;                 /*!< Prescale value to produce the L or unified counter clock */
    uint8_t prescale_h;                 /*!< Prescale value to produce the H counter clock.
                                             This field is used only if the enableCounterUnify is set
                                             to false */
    uint8_t outInitState;               /*!< Defines the initial output value */
} sctimer_config_t;

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
 * @brief Ungates the SCTimer clock and configures the peripheral for basic operation.
 *
 * @note This API should be called at the beginning of the application using the SCTimer driver.
 *
 * @param base   SCTimer peripheral base address
 * @param config Pointer to the user configuration structure.
 *
 * @return kStatus_Success indicates success; Else indicates failure.
 */
status_t SCTIMER_Init(SCT_Type *base, const sctimer_config_t *config);

/*!
 * @brief Gates the SCTimer clock.
 *
 * @param base SCTimer peripheral base address
 */
void SCTIMER_Deinit(SCT_Type *base);

/*!
 * @brief  Fills in the SCTimer configuration structure with the default settings.
 *
 * The default values are:
 * @code
 *  config->enableCounterUnify = true;
 *  config->clockMode = kSCTIMER_System_ClockMode;
 *  config->clockSelect = kSCTIMER_Clock_On_Rise_Input_0;
 *  config->enableBidirection_l = false;
 *  config->enableBidirection_h = false;
 *  config->prescale_l = 0;
 *  config->prescale_h = 0;
 *  config->outInitState = 0;
 * @endcode
 * @param config Pointer to the user configuration structure.
 */
void SCTIMER_GetDefaultConfig(sctimer_config_t *config);

/*! @}*/

/*!
 * @name PWM setup operations
 * @{
 */

/*!
 * @brief Configures the PWM signal parameters.
 *
 * Call this function to configure the PWM signal period, mode, duty cycle, and edge. This
 * function will create 2 events; one of the events will trigger on match with the pulse value
 * and the other will trigger when the counter matches the PWM period. The PWM period event is
 * also used as a limit event to reset the counter or change direction. Both events are enabled
 * for the same state. The state number can be retrieved by calling the function
 * SCTIMER_GetCurrentStateNumber().
 * The counter is set to operate as one 32-bit counter (unify bit is set to 1).
 * The counter operates in bi-directional mode when generating a center-aligned PWM.
 *
 * @note When setting PWM output from multiple output pins, they all should use the same PWM mode.
 * For example, all PWM's should be either edge-aligned or center-aligned.
 * When using this API, the PWM signal frequency of all the initialized channels must be the same.
 * Otherwise all the initialized channels' PWM signal frequency is equal to the last call to the 
 * API's pwmFreq_Hz.
 *
 * @param base        SCTimer peripheral base address
 * @param pwmParams   PWM parameters to configure the output
 * @param mode        PWM operation mode, options available in enumeration ::sctimer_pwm_mode_t
 * @param pwmFreq_Hz  PWM signal frequency in Hz
 * @param srcClock_Hz SCTimer counter clock in Hz
 * @param event       Pointer to a variable where the PWM period event number is stored
 *
 * @return kStatus_Success on success
 *         kStatus_Fail If we have hit the limit in terms of number of events created or if
 *                      an incorrect PWM dutycylce is passed in.
 */
status_t SCTIMER_SetupPwm(SCT_Type *base,
                          const sctimer_pwm_signal_param_t *pwmParams,
                          sctimer_pwm_mode_t mode,
                          uint32_t pwmFreq_Hz,
                          uint32_t srcClock_Hz,
                          uint32_t *event);

/*!
 * @brief Updates the duty cycle of an active PWM signal.
 *
 * @param base              SCTimer peripheral base address
 * @param output            The output to configure
 * @param dutyCyclePercent  New PWM pulse width; the value should be between 1 to 100
 * @param event             Event number associated with this PWM signal. This was returned to the user by the
 *                          function SCTIMER_SetupPwm().
 */
void SCTIMER_UpdatePwmDutycycle(SCT_Type *base, sctimer_out_t output, uint8_t dutyCyclePercent, uint32_t event);

/*!
 * @name Interrupt Interface
 * @{
 */

/*!
 * @brief Enables the selected SCTimer interrupts.
 *
 * @param base SCTimer peripheral base address
 * @param mask The interrupts to enable. This is a logical OR of members of the
 *             enumeration ::sctimer_interrupt_enable_t
 */
static inline void SCTIMER_EnableInterrupts(SCT_Type *base, uint32_t mask)
{
    base->EVEN |= mask;
}

/*!
 * @brief Disables the selected SCTimer interrupts.
 *
 * @param base SCTimer peripheral base address
 * @param mask The interrupts to enable. This is a logical OR of members of the
 *             enumeration ::sctimer_interrupt_enable_t
 */
static inline void SCTIMER_DisableInterrupts(SCT_Type *base, uint32_t mask)
{
    base->EVEN &= ~mask;
}

/*!
 * @brief Gets the enabled SCTimer interrupts.
 *
 * @param base SCTimer peripheral base address
 *
 * @return The enabled interrupts. This is the logical OR of members of the
 *         enumeration ::sctimer_interrupt_enable_t
 */
static inline uint32_t SCTIMER_GetEnabledInterrupts(SCT_Type *base)
{
    return (base->EVEN & 0xFFFFU);
}

/*! @}*/

/*!
 * @name Status Interface
 * @{
 */

/*!
 * @brief Gets the SCTimer status flags.
 *
 * @param base SCTimer peripheral base address
 *
 * @return The status flags. This is the logical OR of members of the
 *         enumeration ::sctimer_status_flags_t
 */
static inline uint32_t SCTIMER_GetStatusFlags(SCT_Type *base)
{
    uint32_t statusFlags = 0;

    /* Add the recorded events */
    statusFlags = (base->EVFLAG & 0xFFFFU);

    /* Add bus error flags */
    statusFlags |= (base->CONFLAG & (SCT_CONFLAG_BUSERRL_MASK | SCT_CONFLAG_BUSERRH_MASK));

    return statusFlags;
}

/*!
 * @brief Clears the SCTimer status flags.
 *
 * @param base SCTimer peripheral base address
 * @param mask The status flags to clear. This is a logical OR of members of the
 *             enumeration ::sctimer_status_flags_t
 */
static inline void SCTIMER_ClearStatusFlags(SCT_Type *base, uint32_t mask)
{
    /* Write to the flag registers */
    base->EVFLAG = (mask & 0xFFFFU);
    base->CONFLAG = (mask & (SCT_CONFLAG_BUSERRL_MASK | SCT_CONFLAG_BUSERRH_MASK));
}

/*! @}*/

/*!
 * @name Counter Start and Stop
 * @{
 */

/*!
 * @brief Starts the SCTimer counter.
 *
 * @param base           SCTimer peripheral base address
 * @param countertoStart SCTimer counter to start; if unify mode is set then function always
 *                       writes to HALT_L bit
 */
static inline void SCTIMER_StartTimer(SCT_Type *base, sctimer_counter_t countertoStart)
{
    /* Clear HALT_L bit if counter is operating in 32-bit mode or user wants to start L counter */
    if ((base->CONFIG & SCT_CONFIG_UNIFY_MASK) || (countertoStart == kSCTIMER_Counter_L))
    {
        base->CTRL &= ~(SCT_CTRL_HALT_L_MASK);
    }
    else
    {
        /* Start H counter */
        base->CTRL &= ~(SCT_CTRL_HALT_H_MASK);
    }
}

/*!
 * @brief Halts the SCTimer counter.
 *
 * @param base          SCTimer peripheral base address
 * @param countertoStop SCTimer counter to stop; if unify mode is set then function always
 *                      writes to HALT_L bit
 */
static inline void SCTIMER_StopTimer(SCT_Type *base, sctimer_counter_t countertoStop)
{
    /* Set HALT_L bit if counter is operating in 32-bit mode or user wants to stop L counter */
    if ((base->CONFIG & SCT_CONFIG_UNIFY_MASK) || (countertoStop == kSCTIMER_Counter_L))
    {
        base->CTRL |= (SCT_CTRL_HALT_L_MASK);
    }
    else
    {
        /* Stop H counter */
        base->CTRL |= (SCT_CTRL_HALT_H_MASK);
    }
}

/*! @}*/

/*!
 * @name Functions to create a new event and manage the state logic
 * @{
 */

/*!
 * @brief Create an event that is triggered on a match or IO and schedule in current state.
 *
 * This function will configure an event using the options provided by the user. If the event type uses
 * the counter match, then the function will set the user provided match value into a match register
 * and put this match register number into the event control register.
 * The event is enabled for the current state and the event number is increased by one at the end.
 * The function returns the event number; this event number can be used to configure actions to be
 * done when this event is triggered.
 *
 * @param base         SCTimer peripheral base address
 * @param howToMonitor Event type; options are available in the enumeration ::sctimer_interrupt_enable_t
 * @param matchValue   The match value that will be programmed to a match register
 * @param whichIO      The input or output that will be involved in event triggering. This field
 *                     is ignored if the event type is "match only"
 * @param whichCounter SCTimer counter to use when operating in 16-bit mode. In 32-bit mode, this
 *                     field has no meaning as we have only 1 unified counter; hence ignored.
 * @param event        Pointer to a variable where the new event number is stored
 *
 * @return kStatus_Success on success
 *         kStatus_Error if we have hit the limit in terms of number of events created or
                         if we have reached the limit in terms of number of match registers
 */
status_t SCTIMER_CreateAndScheduleEvent(SCT_Type *base,
                                        sctimer_event_t howToMonitor,
                                        uint32_t matchValue,
                                        uint32_t whichIO,
                                        sctimer_counter_t whichCounter,
                                        uint32_t *event);

/*!
 * @brief Enable an event in the current state.
 *
 * This function will allow the event passed in to trigger in the current state. The event must
 * be created earlier by either calling the function SCTIMER_SetupPwm() or function
 * SCTIMER_CreateAndScheduleEvent() .
 *
 * @param base  SCTimer peripheral base address
 * @param event Event number to enable in the current state
 *
 */
void SCTIMER_ScheduleEvent(SCT_Type *base, uint32_t event);

/*!
 * @brief Increase the state by 1
 *
 * All future events created by calling the function SCTIMER_ScheduleEvent() will be enabled in this new
 * state.
 *
 * @param base  SCTimer peripheral base address
 *
 * @return kStatus_Success on success
 *         kStatus_Error if we have hit the limit in terms of states used

 */
status_t SCTIMER_IncreaseState(SCT_Type *base);

/*!
 * @brief Provides the current state
 *
 * User can use this to set the next state by calling the function SCTIMER_SetupNextStateAction().
 *
 * @param base SCTimer peripheral base address
 *
 * @return The current state
 */
uint32_t SCTIMER_GetCurrentState(SCT_Type *base);

/*! @}*/

/*!
 * @name Actions to take in response to an event
 * @{
 */

/*!
 * @brief Setup capture of the counter value on trigger of a selected event
 *
 * @param base            SCTimer peripheral base address
 * @param whichCounter    SCTimer counter to use when operating in 16-bit mode. In 32-bit mode, this
 *                        field has no meaning as only the Counter_L bits are used.
 * @param captureRegister Pointer to a variable where the capture register number will be returned. User
 *                        can read the captured value from this register when the specified event is triggered.
 * @param event           Event number that will trigger the capture
 *
 * @return kStatus_Success on success
 *         kStatus_Error if we have hit the limit in terms of number of match/capture registers available
 */
status_t SCTIMER_SetupCaptureAction(SCT_Type *base,
                                    sctimer_counter_t whichCounter,
                                    uint32_t *captureRegister,
                                    uint32_t event);

/*!
 * @brief Receive noticification when the event trigger an interrupt.
 *
 * If the interrupt for the event is enabled by the user, then a callback can be registered
 * which will be invoked when the event is triggered
 *
 * @param base     SCTimer peripheral base address
 * @param event    Event number that will trigger the interrupt
 * @param callback Function to invoke when the event is triggered
 */

void SCTIMER_SetCallback(SCT_Type *base, sctimer_event_callback_t callback, uint32_t event);

/*!
 * @brief Transition to the specified state.
 *
 * This transition will be triggered by the event number that is passed in by the user.
 *
 * @param base      SCTimer peripheral base address
 * @param nextState The next state SCTimer will transition to
 * @param event     Event number that will trigger the state transition
 */
static inline void SCTIMER_SetupNextStateAction(SCT_Type *base, uint32_t nextState, uint32_t event)
{
    uint32_t reg = base->EVENT[event].CTRL;

    reg &= ~(SCT_EVENT_CTRL_STATEV_MASK);
    /* Load the STATEV value when the event occurs to be the next state */
    reg |= SCT_EVENT_CTRL_STATEV(nextState) | SCT_EVENT_CTRL_STATELD_MASK;

    base->EVENT[event].CTRL = reg;
}

/*!
 * @brief Set the Output.
 *
 * This output will be set when the event number that is passed in by the user is triggered.
 *
 * @param base    SCTimer peripheral base address
 * @param whichIO The output to set
 * @param event   Event number that will trigger the output change
 */
static inline void SCTIMER_SetupOutputSetAction(SCT_Type *base, uint32_t whichIO, uint32_t event)
{
    base->OUT[whichIO].SET |= (1U << event);
}

/*!
 * @brief Clear the Output.
 *
 * This output will be cleared when the event number that is passed in by the user is triggered.
 *
 * @param base    SCTimer peripheral base address
 * @param whichIO The output to clear
 * @param event   Event number that will trigger the output change
 */
static inline void SCTIMER_SetupOutputClearAction(SCT_Type *base, uint32_t whichIO, uint32_t event)
{
    base->OUT[whichIO].CLR |= (1U << event);
}

/*!
 * @brief Toggle the output level.
 *
 * This change in the output level is triggered by the event number that is passed in by the user.
 *
 * @param base    SCTimer peripheral base address
 * @param whichIO The output to toggle
 * @param event   Event number that will trigger the output change
 */
void SCTIMER_SetupOutputToggleAction(SCT_Type *base, uint32_t whichIO, uint32_t event);

/*!
 * @brief Limit the running counter.
 *
 * The counter is limited when the event number that is passed in by the user is triggered.
 *
 * @param base         SCTimer peripheral base address
 * @param whichCounter SCTimer counter to use when operating in 16-bit mode. In 32-bit mode, this
 *                     field has no meaning as only the Counter_L bits are used.
 * @param event        Event number that will trigger the counter to be limited
 */
static inline void SCTIMER_SetupCounterLimitAction(SCT_Type *base, sctimer_counter_t whichCounter, uint32_t event)
{
    /* Use Counter_L bits if counter is operating in 32-bit mode or user wants to setup the L counter */
    if ((base->CONFIG & SCT_CONFIG_UNIFY_MASK) || (whichCounter == kSCTIMER_Counter_L))
    {
        base->LIMIT |= SCT_LIMIT_LIMMSK_L(1U << event);
    }
    else
    {
        base->LIMIT |= SCT_LIMIT_LIMMSK_H(1U << event);
    }
}

/*!
 * @brief Stop the running counter.
 *
 * The counter is stopped when the event number that is passed in by the user is triggered.
 *
 * @param base         SCTimer peripheral base address
 * @param whichCounter SCTimer counter to use when operating in 16-bit mode. In 32-bit mode, this
 *                     field has no meaning as only the Counter_L bits are used.
 * @param event        Event number that will trigger the counter to be stopped
 */
static inline void SCTIMER_SetupCounterStopAction(SCT_Type *base, sctimer_counter_t whichCounter, uint32_t event)
{
    /* Use Counter_L bits if counter is operating in 32-bit mode or user wants to setup the L counter */
    if ((base->CONFIG & SCT_CONFIG_UNIFY_MASK) || (whichCounter == kSCTIMER_Counter_L))
    {
        base->STOP |= SCT_STOP_STOPMSK_L(1U << event);
    }
    else
    {
        base->STOP |= SCT_STOP_STOPMSK_H(1U << event);
    }
}

/*!
 * @brief Re-start the stopped counter.
 *
 * The counter will re-start when the event number that is passed in by the user is triggered.
 *
 * @param base         SCTimer peripheral base address
 * @param whichCounter SCTimer counter to use when operating in 16-bit mode. In 32-bit mode, this
 *                     field has no meaning as only the Counter_L bits are used.
 * @param event        Event number that will trigger the counter to re-start
 */
static inline void SCTIMER_SetupCounterStartAction(SCT_Type *base, sctimer_counter_t whichCounter, uint32_t event)
{
    /* Use Counter_L bits if counter is operating in 32-bit mode or user wants to setup the L counter */
    if ((base->CONFIG & SCT_CONFIG_UNIFY_MASK) || (whichCounter == kSCTIMER_Counter_L))
    {
        base->START |= SCT_START_STARTMSK_L(1U << event);
    }
    else
    {
        base->START |= SCT_START_STARTMSK_H(1U << event);
    }
}

/*!
 * @brief Halt the running counter.
 *
 * The counter is disabled (halted) when the event number that is passed in by the user is
 * triggered. When the counter is halted, all further events are disabled. The HALT condition
 * can only be removed by calling the SCTIMER_StartTimer() function.
 *
 * @param base         SCTimer peripheral base address
 * @param whichCounter SCTimer counter to use when operating in 16-bit mode. In 32-bit mode, this
 *                     field has no meaning as only the Counter_L bits are used.
 * @param event        Event number that will trigger the counter to be halted
 */
static inline void SCTIMER_SetupCounterHaltAction(SCT_Type *base, sctimer_counter_t whichCounter, uint32_t event)
{
    /* Use Counter_L bits if counter is operating in 32-bit mode or user wants to setup the L counter */
    if ((base->CONFIG & SCT_CONFIG_UNIFY_MASK) || (whichCounter == kSCTIMER_Counter_L))
    {
        base->HALT |= SCT_HALT_HALTMSK_L(1U << event);
    }
    else
    {
        base->HALT |= SCT_HALT_HALTMSK_H(1U << event);
    }
}

/*!
 * @brief Generate a DMA request.
 *
 * DMA request will be triggered by the event number that is passed in by the user.
 *
 * @param base      SCTimer peripheral base address
 * @param dmaNumber The DMA request to generate
 * @param event     Event number that will trigger the DMA request
 */
static inline void SCTIMER_SetupDmaTriggerAction(SCT_Type *base, uint32_t dmaNumber, uint32_t event)
{
    if (dmaNumber == 0)
    {
        base->DMA0REQUEST |= (1U << event);
    }
    else
    {
        base->DMA1REQUEST |= (1U << event);
    }
}

/*!
 * @brief SCTimer interrupt handler.
 *
 * @param base SCTimer peripheral base address.
 */
void SCTIMER_EventHandleIRQ(SCT_Type *base);

/*! @}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_SCTIMER_H_ */
