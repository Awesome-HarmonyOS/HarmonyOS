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

#include "fsl_wwdt.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Gets the instance from the base address
 *
 * @param base WWDT peripheral base address
 *
 * @return The WWDT instance
 */
static uint32_t WWDT_GetInstance(WWDT_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to WWDT bases for each instance. */
static WWDT_Type *const s_wwdtBases[] = WWDT_BASE_PTRS;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to WWDT clocks for each instance. */
static const clock_ip_name_t s_wwdtClocks[] = WWDT_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*! @brief Pointers to WWDT resets for each instance. */
static const reset_ip_name_t s_wwdtResets[] = WWDT_RSTS;

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t WWDT_GetInstance(WWDT_Type *base)
{
    uint32_t instance;
    uint32_t wwdtArrayCount = (sizeof(s_wwdtBases) / sizeof(s_wwdtBases[0]));

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < wwdtArrayCount; instance++)
    {
        if (s_wwdtBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < wwdtArrayCount);

    return instance;
}

/*******************************************************************************
 * Code
 ******************************************************************************/

void WWDT_GetDefaultConfig(wwdt_config_t *config)
{
    assert(config);

    /* Enable the watch dog */
    config->enableWwdt = true;
    /* Disable the watchdog timeout reset */
    config->enableWatchdogReset = false;
    /* Disable the watchdog protection for updating the timeout value */
    config->enableWatchdogProtect = false;
    /* Do not lock the watchdog oscillator */
    config->enableLockOscillator = false;
    /* Windowing is not in effect */
    config->windowValue = 0xFFFFFFU;
    /* Set the timeout value to the max */
    config->timeoutValue = 0xFFFFFFU;
    /* No warning is provided */
    config->warningValue = 0;
}

void WWDT_Init(WWDT_Type *base, const wwdt_config_t *config)
{
    assert(config);

    uint32_t value = 0U;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable the WWDT clock */
    CLOCK_EnableClock(s_wwdtClocks[WWDT_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    /* Reset the WWDT module */
    RESET_PeripheralReset(s_wwdtResets[WWDT_GetInstance(base)]);

    value = WWDT_MOD_WDEN(config->enableWwdt) | WWDT_MOD_WDRESET(config->enableWatchdogReset) |
            WWDT_MOD_WDPROTECT(config->enableWatchdogProtect) | WWDT_MOD_LOCK(config->enableLockOscillator);
    /* Set configruation */
    base->WINDOW = WWDT_WINDOW_WINDOW(config->windowValue);
    base->TC = WWDT_TC_COUNT(config->timeoutValue);
    base->WARNINT = WWDT_WARNINT_WARNINT(config->warningValue);
    base->MOD = value;
}

void WWDT_Deinit(WWDT_Type *base)
{
    WWDT_Disable(base);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Disable the WWDT clock */
    CLOCK_DisableClock(s_wwdtClocks[WWDT_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void WWDT_Refresh(WWDT_Type *base)
{
    uint32_t primaskValue = 0U;

    /* Disable the global interrupt to protect refresh sequence */
    primaskValue = DisableGlobalIRQ();
    base->FEED = WWDT_FIRST_WORD_OF_REFRESH;
    base->FEED = WWDT_SECOND_WORD_OF_REFRESH;
    EnableGlobalIRQ(primaskValue);
}

void WWDT_ClearStatusFlags(WWDT_Type *base, uint32_t mask)
{
    /* Clear the WDINT bit so that we don't accidentally clear it */
    uint32_t reg = (base->MOD & (~WWDT_MOD_WDINT_MASK));

    /* Clear timeout by writing a zero */
    if (mask & kWWDT_TimeoutFlag)
    {
        reg &= ~WWDT_MOD_WDTOF_MASK;
    }

    /* Clear warning interrupt flag by writing a one */
    if (mask & kWWDT_WarningFlag)
    {
        reg |= WWDT_MOD_WDINT_MASK;
    }

    base->MOD = reg;
}
