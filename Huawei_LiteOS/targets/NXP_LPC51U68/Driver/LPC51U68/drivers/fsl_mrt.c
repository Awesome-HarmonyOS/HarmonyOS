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

#include "fsl_mrt.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Gets the instance from the base address
 *
 * @param base Multi-Rate timer peripheral base address
 *
 * @return The MRT instance
 */
static uint32_t MRT_GetInstance(MRT_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to MRT bases for each instance. */
static MRT_Type *const s_mrtBases[] = MRT_BASE_PTRS;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to MRT clocks for each instance. */
static const clock_ip_name_t s_mrtClocks[] = MRT_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*! @brief Pointers to MRT resets for each instance. */
static const reset_ip_name_t s_mrtResets[] = MRT_RSTS;

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t MRT_GetInstance(MRT_Type *base)
{
    uint32_t instance;
    uint32_t mrtArrayCount = (sizeof(s_mrtBases) / sizeof(s_mrtBases[0]));

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < mrtArrayCount; instance++)
    {
        if (s_mrtBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < mrtArrayCount);

    return instance;
}

void MRT_Init(MRT_Type *base, const mrt_config_t *config)
{
    assert(config);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Ungate the MRT clock */
    CLOCK_EnableClock(s_mrtClocks[MRT_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    /* Reset the module */
    RESET_PeripheralReset(s_mrtResets[MRT_GetInstance(base)]);

    /* Set timer operating mode */
    base->MODCFG = MRT_MODCFG_MULTITASK(config->enableMultiTask);
}

void MRT_Deinit(MRT_Type *base)
{
    /* Stop all the timers */
    MRT_StopTimer(base, kMRT_Channel_0);
    MRT_StopTimer(base, kMRT_Channel_1);
    MRT_StopTimer(base, kMRT_Channel_2);
    MRT_StopTimer(base, kMRT_Channel_3);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Gate the MRT clock*/
    CLOCK_DisableClock(s_mrtClocks[MRT_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void MRT_UpdateTimerPeriod(MRT_Type *base, mrt_chnl_t channel, uint32_t count, bool immediateLoad)
{
    uint32_t newValue = count;
    if (((base->CHANNEL[channel].CTRL & MRT_CHANNEL_CTRL_MODE_MASK) == kMRT_OneShotMode) || (immediateLoad))
    {
        /* For one-shot interrupt mode, load the new value immediately even if user forgot to enable */
        newValue |= MRT_CHANNEL_INTVAL_LOAD_MASK;
    }

    /* Update the timer interval value */
    base->CHANNEL[channel].INTVAL = newValue;
}
