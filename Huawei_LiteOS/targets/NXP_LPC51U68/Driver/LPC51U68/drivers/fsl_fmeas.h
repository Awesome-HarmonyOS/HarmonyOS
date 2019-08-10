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
#ifndef _FSL_FMEAS_H_
#define _FSL_FMEAS_H_

#include "fsl_common.h"

/*!
 * @addtogroup fmeas
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief Defines LPC Frequency Measure driver version 2.0.0.
 *
 * Change log:
 * - Version 2.0.0
 *   - initial version
 */
#define FSL_FMEAS_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*******************************************************************************
 * API
 *******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @name FMEAS Functional Operation
 * @{
 */

/*!
 * @brief    Starts a frequency measurement cycle.
 *
 * @param    base : SYSCON peripheral base address.
 */
static inline void FMEAS_StartMeasure(SYSCON_Type *base)
{
    base->FREQMECTRL = 0;
    base->FREQMECTRL = (1UL << 31);
}

/*!
 * @brief    Indicates when a frequency measurement cycle is complete.
 *
 * @param    base : SYSCON peripheral base address.
 * @return   true if a measurement cycle is active, otherwise false.
 */
static inline bool FMEAS_IsMeasureComplete(SYSCON_Type *base)
{
    return (bool)((base->FREQMECTRL & (1UL << 31)) == 0);
}

/*!
 * @brief    Returns the computed value for a frequency measurement cycle
 *
 * @param    base         : SYSCON peripheral base address.
 * @param    refClockRate : Reference clock rate used during the frequency measurement cycle.
 *
 * @return   Frequency in Hz.
 */
uint32_t FMEAS_GetFrequency(SYSCON_Type *base, uint32_t refClockRate);

/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @}*/

#endif /* _FSL_FMEAS_H_ */
