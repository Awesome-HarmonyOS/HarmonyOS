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

#include "fsl_fmeas.h"

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/*! @brief Target clock counter value.
 * According to user manual, 2 has to be subtracted from captured value (CAPVAL). */
#define TARGET_CLOCK_COUNT(base) \
    ((uint32_t)(                 \
        ((((SYSCON_Type *)base)->FREQMECTRL & SYSCON_FREQMECTRL_CAPVAL_MASK) >> SYSCON_FREQMECTRL_CAPVAL_SHIFT) - 2))

/*! @brief Reference clock counter value. */
#define REFERENCE_CLOCK_COUNT ((uint32_t)((SYSCON_FREQMECTRL_CAPVAL_MASK >> SYSCON_FREQMECTRL_CAPVAL_SHIFT) + 1))

/*******************************************************************************
 * Code
 ******************************************************************************/

uint32_t FMEAS_GetFrequency(SYSCON_Type *base, uint32_t refClockRate)
{
    uint32_t targetClockCount = TARGET_CLOCK_COUNT(base);
    uint64_t clkrate = 0;

    if (targetClockCount > 0)
    {
        clkrate = (((uint64_t)targetClockCount) * (uint64_t)refClockRate) / REFERENCE_CLOCK_COUNT;
    }

    return (uint32_t)clkrate;
}
