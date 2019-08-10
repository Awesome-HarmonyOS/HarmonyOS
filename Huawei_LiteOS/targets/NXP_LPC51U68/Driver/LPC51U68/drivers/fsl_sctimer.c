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

#include "fsl_sctimer.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Typedef for interrupt handler. */
typedef void (*sctimer_isr_t)(SCT_Type *base);

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Gets the instance from the base address
 *
 * @param base SCTimer peripheral base address
 *
 * @return The SCTimer instance
 */
static uint32_t SCTIMER_GetInstance(SCT_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to SCT bases for each instance. */
static SCT_Type *const s_sctBases[] = SCT_BASE_PTRS;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to SCT clocks for each instance. */
static const clock_ip_name_t s_sctClocks[] = SCT_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*! @brief Pointers to SCT resets for each instance. */
static const reset_ip_name_t s_sctResets[] = SCT_RSTS;

/*!< @brief SCTimer event Callback function. */
static sctimer_event_callback_t s_eventCallback[FSL_FEATURE_SCT_NUMBER_OF_EVENTS];

/*!< @brief Keep track of SCTimer event number */
static uint32_t s_currentEvent;

/*!< @brief Keep track of SCTimer state number */
static uint32_t s_currentState;

/*!< @brief Keep track of SCTimer match/capture register number */
static uint32_t s_currentMatch;

/*! @brief Pointer to SCTimer IRQ handler */
static sctimer_isr_t s_sctimerIsr;

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t SCTIMER_GetInstance(SCT_Type *base)
{
    uint32_t instance;
    uint32_t sctArrayCount = (sizeof(s_sctBases) / sizeof(s_sctBases[0]));

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < sctArrayCount; instance++)
    {
        if (s_sctBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < sctArrayCount);

    return instance;
}

status_t SCTIMER_Init(SCT_Type *base, const sctimer_config_t *config)
{
    assert(config);
    uint32_t i;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable the SCTimer clock*/
    CLOCK_EnableClock(s_sctClocks[SCTIMER_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    /* Reset the module */
    RESET_PeripheralReset(s_sctResets[SCTIMER_GetInstance(base)]);

    /* Setup the counter operation */
    base->CONFIG = SCT_CONFIG_CKSEL(config->clockSelect) | SCT_CONFIG_CLKMODE(config->clockMode) |
                   SCT_CONFIG_UNIFY(config->enableCounterUnify);

    /* Write to the control register, clear the counter and keep the counters halted */
    base->CTRL = SCT_CTRL_BIDIR_L(config->enableBidirection_l) | SCT_CTRL_PRE_L(config->prescale_l) |
                 SCT_CTRL_CLRCTR_L_MASK | SCT_CTRL_HALT_L_MASK;

    if (!(config->enableCounterUnify))
    {
        base->CTRL |= SCT_CTRL_BIDIR_H(config->enableBidirection_h) | SCT_CTRL_PRE_H(config->prescale_h) |
                      SCT_CTRL_CLRCTR_H_MASK | SCT_CTRL_HALT_H_MASK;
    }

    /* Initial state of channel output */
    base->OUTPUT = config->outInitState;

    /* Clear the global variables */
    s_currentEvent = 0;
    s_currentState = 0;
    s_currentMatch = 0;

    /* Clear the callback array */
    for (i = 0; i < FSL_FEATURE_SCT_NUMBER_OF_EVENTS; i++)
    {
        s_eventCallback[i] = NULL;
    }

    /* Save interrupt handler */
    s_sctimerIsr = SCTIMER_EventHandleIRQ;

    return kStatus_Success;
}

void SCTIMER_Deinit(SCT_Type *base)
{
    /* Halt the counters */
    base->CTRL |= (SCT_CTRL_HALT_L_MASK | SCT_CTRL_HALT_H_MASK);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Disable the SCTimer clock*/
    CLOCK_DisableClock(s_sctClocks[SCTIMER_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void SCTIMER_GetDefaultConfig(sctimer_config_t *config)
{
    assert(config);

    /* SCT operates as a unified 32-bit counter */
    config->enableCounterUnify = true;
    /* System clock clocks the entire SCT module */
    config->clockMode = kSCTIMER_System_ClockMode;
    /* This is used only by certain clock modes */
    config->clockSelect = kSCTIMER_Clock_On_Rise_Input_0;
    /* Up count mode only for the unified counter */
    config->enableBidirection_l = false;
    /* Up count mode only for Counte_H */
    config->enableBidirection_h = false;
    /* Prescale factor of 1 */
    config->prescale_l = 0;
    /* Prescale factor of 1 for Counter_H*/
    config->prescale_h = 0;
    /* Clear outputs */
    config->outInitState = 0;
}

status_t SCTIMER_SetupPwm(SCT_Type *base,
                          const sctimer_pwm_signal_param_t *pwmParams,
                          sctimer_pwm_mode_t mode,
                          uint32_t pwmFreq_Hz,
                          uint32_t srcClock_Hz,
                          uint32_t *event)
{
    assert(pwmParams);
    assert(srcClock_Hz);
    assert(pwmFreq_Hz);

    uint32_t period, pulsePeriod = 0;
    uint32_t sctClock = srcClock_Hz / (((base->CTRL & SCT_CTRL_PRE_L_MASK) >> SCT_CTRL_PRE_L_SHIFT) + 1);
    uint32_t periodEvent, pulseEvent;
    uint32_t reg;

    /* This function will create 2 events, return an error if we do not have enough events available */
    if ((s_currentEvent + 2) > FSL_FEATURE_SCT_NUMBER_OF_EVENTS)
    {
        return kStatus_Fail;
    }

    if (pwmParams->dutyCyclePercent == 0)
    {
        return kStatus_Fail;
    }

    /* Set unify bit to operate in 32-bit counter mode */
    base->CONFIG |= SCT_CONFIG_UNIFY_MASK;

    /* Use bi-directional mode for center-aligned PWM */
    if (mode == kSCTIMER_CenterAlignedPwm)
    {
        base->CTRL |= SCT_CTRL_BIDIR_L_MASK;
    }

    /* Calculate PWM period match value */
    if (mode == kSCTIMER_EdgeAlignedPwm)
    {
        period = (sctClock / pwmFreq_Hz) - 1;
    }
    else
    {
        period = sctClock / (pwmFreq_Hz * 2);
    }

    /* Calculate pulse width match value */
    pulsePeriod = (period * pwmParams->dutyCyclePercent) / 100;

    /* For 100% dutycyle, make pulse period greater than period so the event will never occur */
    if (pwmParams->dutyCyclePercent >= 100)
    {
        pulsePeriod = period + 2;
    }

    /* Schedule an event when we reach the PWM period */
    SCTIMER_CreateAndScheduleEvent(base, kSCTIMER_MatchEventOnly, period, 0, kSCTIMER_Counter_L, &periodEvent);

    /* Schedule an event when we reach the pulse width */
    SCTIMER_CreateAndScheduleEvent(base, kSCTIMER_MatchEventOnly, pulsePeriod, 0, kSCTIMER_Counter_L, &pulseEvent);

    /* Reset the counter when we reach the PWM period */
    SCTIMER_SetupCounterLimitAction(base, kSCTIMER_Counter_L, periodEvent);

    /* Return the period event to the user */
    *event = periodEvent;

    /* For high-true level */
    if (pwmParams->level == kSCTIMER_HighTrue)
    {
        /* Set the initial output level to low which is the inactive state */
        base->OUTPUT &= ~(1U << pwmParams->output);

        if (mode == kSCTIMER_EdgeAlignedPwm)
        {
            /* Set the output when we reach the PWM period */
            SCTIMER_SetupOutputSetAction(base, pwmParams->output, periodEvent);
            /* Clear the output when we reach the PWM pulse value */
            SCTIMER_SetupOutputClearAction(base, pwmParams->output, pulseEvent);
        }
        else
        {
            /* Clear the output when we reach the PWM pulse event */
            SCTIMER_SetupOutputClearAction(base, pwmParams->output, pulseEvent);
            /* Reverse output when down counting */
            reg = base->OUTPUTDIRCTRL;
            reg &= ~(SCT_OUTPUTDIRCTRL_SETCLR0_MASK << (2 * pwmParams->output));
            reg |= (1U << (2 * pwmParams->output));
            base->OUTPUTDIRCTRL = reg;
        }
    }
    /* For low-true level */
    else
    {
        /* Set the initial output level to high which is the inactive state */
        base->OUTPUT |= (1U << pwmParams->output);

        if (mode == kSCTIMER_EdgeAlignedPwm)
        {
            /* Clear the output when we reach the PWM period */
            SCTIMER_SetupOutputClearAction(base, pwmParams->output, periodEvent);
            /* Set the output when we reach the PWM pulse value */
            SCTIMER_SetupOutputSetAction(base, pwmParams->output, pulseEvent);
        }
        else
        {
            /* Set the output when we reach the PWM pulse event */
            SCTIMER_SetupOutputSetAction(base, pwmParams->output, pulseEvent);
            /* Reverse output when down counting */
            reg = base->OUTPUTDIRCTRL;
            reg &= ~(SCT_OUTPUTDIRCTRL_SETCLR0_MASK << (2 * pwmParams->output));
            reg |= (1U << (2 * pwmParams->output));
            base->OUTPUTDIRCTRL = reg;
        }
    }

    return kStatus_Success;
}

void SCTIMER_UpdatePwmDutycycle(SCT_Type *base, sctimer_out_t output, uint8_t dutyCyclePercent, uint32_t event)

{
    assert(dutyCyclePercent > 0);

    uint32_t periodMatchReg, pulseMatchReg;
    uint32_t pulsePeriod = 0, period;

    /* Retrieve the match register number for the PWM period */
    periodMatchReg = base->EVENT[event].CTRL & SCT_EVENT_CTRL_MATCHSEL_MASK;

    /* Retrieve the match register number for the PWM pulse period */
    pulseMatchReg = base->EVENT[event + 1].CTRL & SCT_EVENT_CTRL_MATCHSEL_MASK;

    period = base->SCTMATCH[periodMatchReg];

    /* Calculate pulse width match value */
    pulsePeriod = (period * dutyCyclePercent) / 100;

    /* For 100% dutycyle, make pulse period greater than period so the event will never occur */
    if (dutyCyclePercent >= 100)
    {
        pulsePeriod = period + 2;
    }

    /* Stop the counter before updating match register */
    SCTIMER_StopTimer(base, kSCTIMER_Counter_L);

    /* Update dutycycle */
    base->SCTMATCH[pulseMatchReg] = SCT_SCTMATCH_MATCHn_L(pulsePeriod);
    base->SCTMATCHREL[pulseMatchReg] = SCT_SCTMATCHREL_RELOADn_L(pulsePeriod);

    /* Restart the counter */
    SCTIMER_StartTimer(base, kSCTIMER_Counter_L);
}

status_t SCTIMER_CreateAndScheduleEvent(SCT_Type *base,
                                        sctimer_event_t howToMonitor,
                                        uint32_t matchValue,
                                        uint32_t whichIO,
                                        sctimer_counter_t whichCounter,
                                        uint32_t *event)
{
    uint32_t combMode = (((uint32_t)howToMonitor & SCT_EVENT_CTRL_COMBMODE_MASK) >> SCT_EVENT_CTRL_COMBMODE_SHIFT);
    uint32_t currentCtrlVal = howToMonitor;

    /* Return an error if we have hit the limit in terms of number of events created */
    if (s_currentEvent >= FSL_FEATURE_SCT_NUMBER_OF_EVENTS)
    {
        return kStatus_Fail;
    }

    /* IO only mode */
    if (combMode == 0x2U)
    {
        base->EVENT[s_currentEvent].CTRL = currentCtrlVal | SCT_EVENT_CTRL_IOSEL(whichIO);
    }
    /* Match mode only */
    else if (combMode == 0x1U)
    {
        /* Return an error if we have hit the limit in terms of number of number of match registers */
        if (s_currentMatch >= FSL_FEATURE_SCT_NUMBER_OF_MATCH_CAPTURE)
        {
            return kStatus_Fail;
        }

        currentCtrlVal |= SCT_EVENT_CTRL_MATCHSEL(s_currentMatch);
        /* Use Counter_L bits if counter is operating in 32-bit mode or user wants to setup the L counter */
        if ((base->CONFIG & SCT_CONFIG_UNIFY_MASK) || (whichCounter == kSCTIMER_Counter_L))
        {
            base->SCTMATCH[s_currentMatch] = SCT_SCTMATCH_MATCHn_L(matchValue);
            base->SCTMATCHREL[s_currentMatch] = SCT_SCTMATCHREL_RELOADn_L(matchValue);
        }
        else
        {
            /* Select the counter, no need for this if operating in 32-bit mode */
            currentCtrlVal |= SCT_EVENT_CTRL_HEVENT(whichCounter);
            base->SCTMATCH[s_currentMatch] = SCT_SCTMATCH_MATCHn_H(matchValue);
            base->SCTMATCHREL[s_currentMatch] = SCT_SCTMATCHREL_RELOADn_H(matchValue);
        }
        base->EVENT[s_currentEvent].CTRL = currentCtrlVal;
        /* Increment the match register number */
        s_currentMatch++;
    }
    /* Use both Match & IO */
    else
    {
        /* Return an error if we have hit the limit in terms of number of number of match registers */
        if (s_currentMatch >= FSL_FEATURE_SCT_NUMBER_OF_MATCH_CAPTURE)
        {
            return kStatus_Fail;
        }

        currentCtrlVal |= SCT_EVENT_CTRL_MATCHSEL(s_currentMatch) | SCT_EVENT_CTRL_IOSEL(whichIO);
        /* Use Counter_L bits if counter is operating in 32-bit mode or user wants to setup the L counter */
        if ((base->CONFIG & SCT_CONFIG_UNIFY_MASK) || (whichCounter == kSCTIMER_Counter_L))
        {
            base->SCTMATCH[s_currentMatch] = SCT_SCTMATCH_MATCHn_L(matchValue);
            base->SCTMATCHREL[s_currentMatch] = SCT_SCTMATCHREL_RELOADn_L(matchValue);
        }
        else
        {
            /* Select the counter, no need for this if operating in 32-bit mode */
            currentCtrlVal |= SCT_EVENT_CTRL_HEVENT(whichCounter);
            base->SCTMATCH[s_currentMatch] = SCT_SCTMATCH_MATCHn_H(matchValue);
            base->SCTMATCHREL[s_currentMatch] = SCT_SCTMATCHREL_RELOADn_H(matchValue);
        }
        base->EVENT[s_currentEvent].CTRL = currentCtrlVal;
        /* Increment the match register number */
        s_currentMatch++;
    }

    /* Enable the event in the current state */
    base->EVENT[s_currentEvent].STATE = (1U << s_currentState);

    /* Return the event number */
    *event = s_currentEvent;

    /* Increment the event number */
    s_currentEvent++;

    return kStatus_Success;
}

void SCTIMER_ScheduleEvent(SCT_Type *base, uint32_t event)
{
    /* Enable event in the current state */
    base->EVENT[event].STATE |= (1U << s_currentState);
}

status_t SCTIMER_IncreaseState(SCT_Type *base)
{
    /* Return an error if we have hit the limit in terms of states used */
    if (s_currentState >= FSL_FEATURE_SCT_NUMBER_OF_STATES)
    {
        return kStatus_Fail;
    }

    s_currentState++;

    return kStatus_Success;
}

uint32_t SCTIMER_GetCurrentState(SCT_Type *base)
{
    return s_currentState;
}

void SCTIMER_SetupOutputToggleAction(SCT_Type *base, uint32_t whichIO, uint32_t event)
{
    uint32_t reg;

    /* Set the same event to set and clear the output */
    base->OUT[whichIO].CLR |= (1U << event);
    base->OUT[whichIO].SET |= (1U << event);

    /* Set the conflict resolution to toggle output */
    reg = base->RES;
    reg &= ~(SCT_RES_O0RES_MASK << (2 * whichIO));
    reg |= (uint32_t)(kSCTIMER_ResolveToggle << (2 * whichIO));
    base->RES = reg;
}

status_t SCTIMER_SetupCaptureAction(SCT_Type *base,
                                    sctimer_counter_t whichCounter,
                                    uint32_t *captureRegister,
                                    uint32_t event)
{
    /* Return an error if we have hit the limit in terms of number of capture/match registers used */
    if (s_currentMatch >= FSL_FEATURE_SCT_NUMBER_OF_MATCH_CAPTURE)
    {
        return kStatus_Fail;
    }

    /* Use Counter_L bits if counter is operating in 32-bit mode or user wants to setup the L counter */
    if ((base->CONFIG & SCT_CONFIG_UNIFY_MASK) || (whichCounter == kSCTIMER_Counter_L))
    {
        /* Set the bit to enable event */
        base->SCTCAPCTRL[s_currentMatch] |= SCT_SCTCAPCTRL_CAPCONn_L(1 << event);

        /* Set this resource to be a capture rather than match */
        base->REGMODE |= SCT_REGMODE_REGMOD_L(1 << s_currentMatch);
    }
    else
    {
        /* Set bit to enable event */
        base->SCTCAPCTRL[s_currentMatch] |= SCT_SCTCAPCTRL_CAPCONn_H(1 << event);

        /* Set this resource to be a capture rather than match */
        base->REGMODE |= SCT_REGMODE_REGMOD_H(1 << s_currentMatch);
    }

    /* Return the match register number */
    *captureRegister = s_currentMatch;

    /* Increase the match register number */
    s_currentMatch++;

    return kStatus_Success;
}

void SCTIMER_SetCallback(SCT_Type *base, sctimer_event_callback_t callback, uint32_t event)
{
    s_eventCallback[event] = callback;
}

void SCTIMER_EventHandleIRQ(SCT_Type *base)
{
    uint32_t eventFlag = SCT0->EVFLAG;
    /* Only clear the flags whose interrupt field is enabled */
    uint32_t clearFlag = (eventFlag & SCT0->EVEN);
    uint32_t mask = eventFlag;
    int i = 0;

    /* Invoke the callback for certain events */
    for (i = 0; (i < FSL_FEATURE_SCT_NUMBER_OF_EVENTS) && (mask != 0); i++)
    {
        if (mask & 0x1)
        {
            if (s_eventCallback[i] != NULL)
            {
                s_eventCallback[i]();
            }
        }
        mask >>= 1;
    }

    /* Clear event interrupt flag */
    SCT0->EVFLAG = clearFlag;
}

void SCT0_IRQHandler(void)
{
    s_sctimerIsr(SCT0);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
