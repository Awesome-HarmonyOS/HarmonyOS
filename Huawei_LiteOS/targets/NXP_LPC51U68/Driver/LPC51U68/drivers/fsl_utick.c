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

#include "fsl_utick.h"
#include "fsl_power.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Typedef for interrupt handler. */
typedef void (*utick_isr_t)(UTICK_Type *base, utick_callback_t cb);

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Gets the instance from the base address
 *
 * @param base UTICK peripheral base address
 *
 * @return The UTICK instance
 */
static uint32_t UTICK_GetInstance(UTICK_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Array of UTICK handle. */
static utick_callback_t s_utickHandle[FSL_FEATURE_SOC_UTICK_COUNT];
/* Array of UTICK peripheral base address. */
static UTICK_Type *const s_utickBases[] = UTICK_BASE_PTRS;
/* Array of UTICK IRQ number. */
static const IRQn_Type s_utickIRQ[] = UTICK_IRQS;
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/* Array of UTICK clock name. */
static const clock_ip_name_t s_utickClock[] = UTICK_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
/* UTICK ISR for transactional APIs. */
static utick_isr_t s_utickIsr;

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t UTICK_GetInstance(UTICK_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_utickBases); instance++)
    {
        if (s_utickBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_utickBases));

    return instance;
}

void UTICK_SetTick(UTICK_Type *base, utick_mode_t mode, uint32_t count, utick_callback_t cb)
{
    uint32_t instance;

    /* Get instance from peripheral base address. */
    instance = UTICK_GetInstance(base);

    /* Save the handle in global variables to support the double weak mechanism. */
    s_utickHandle[instance] = cb;
    EnableDeepSleepIRQ(s_utickIRQ[instance]);
    base->CTRL = count | UTICK_CTRL_REPEAT(mode);
}

void UTICK_Init(UTICK_Type *base)
{
    /* Enable utick clock */
    CLOCK_EnableClock(s_utickClock[UTICK_GetInstance(base)]);
    /* Power up Watchdog oscillator*/
    POWER_DisablePD(kPDRUNCFG_PD_WDT_OSC);
    s_utickIsr = UTICK_HandleIRQ;
}

void UTICK_Deinit(UTICK_Type *base)
{
    /* Turn off utick */
    base->CTRL = 0;
    /* Disable utick clock */
    CLOCK_DisableClock(s_utickClock[UTICK_GetInstance(base)]);
}

uint32_t UTICK_GetStatusFlags(UTICK_Type *base)
{
    return (base->STAT);
}

void UTICK_ClearStatusFlags(UTICK_Type *base)
{
    base->STAT = UTICK_STAT_INTR_MASK;
}

void UTICK_HandleIRQ(UTICK_Type *base, utick_callback_t cb)
{
    UTICK_ClearStatusFlags(base);
    if (cb)
    {
        cb();
    }
}

#if defined(UTICK0)
void UTICK0_DriverIRQHandler(void)
{
    s_utickIsr(UTICK0, s_utickHandle[0]);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif
#if defined(UTICK1)
void UTICK1_DriverIRQHandler(void)
{
    s_utickIsr(UTICK1, s_utickHandle[1]);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif
#if defined(UTICK2)
void UTICK2_DriverIRQHandler(void)
{
    s_utickIsr(UTICK2, s_utickHandle[2]);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif
