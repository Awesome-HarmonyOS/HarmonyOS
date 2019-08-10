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

#ifndef _FSL_GINT_H_
#define _FSL_GINT_H_

#include "fsl_common.h"

/*!
 * @addtogroup gint_driver
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_GINT_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0. */
/*@}*/

/*! @brief GINT combine inputs type */
typedef enum _gint_comb
{
    kGINT_CombineOr = 0U, /*!< A grouped interrupt is generated when any one of the enabled inputs is active */
    kGINT_CombineAnd = 1U /*!< A grouped interrupt is generated when all enabled inputs are active */
} gint_comb_t;

/*! @brief GINT trigger type */
typedef enum _gint_trig
{
    kGINT_TrigEdge = 0U, /*!< Edge triggered based on polarity */
    kGINT_TrigLevel = 1U /*!< Level triggered based on polarity */
} gint_trig_t;

/* @brief GINT port type */
typedef enum _gint_port
{
    kGINT_Port0 = 0U,
    kGINT_Port1 = 1U,
#if defined(FSL_FEATURE_GINT_PORT_COUNT) && (FSL_FEATURE_GINT_PORT_COUNT > 2U)
    kGINT_Port2 = 2U,
#endif
#if defined(FSL_FEATURE_GINT_PORT_COUNT) && (FSL_FEATURE_GINT_PORT_COUNT > 3U)
    kGINT_Port3 = 3U,
#endif
#if defined(FSL_FEATURE_GINT_PORT_COUNT) && (FSL_FEATURE_GINT_PORT_COUNT > 4U)
    kGINT_Port4 = 4U,
#endif
#if defined(FSL_FEATURE_GINT_PORT_COUNT) && (FSL_FEATURE_GINT_PORT_COUNT > 5U)
    kGINT_Port5 = 5U,
#endif
#if defined(FSL_FEATURE_GINT_PORT_COUNT) && (FSL_FEATURE_GINT_PORT_COUNT > 6U)
    kGINT_Port6 = 6U,
#endif
#if defined(FSL_FEATURE_GINT_PORT_COUNT) && (FSL_FEATURE_GINT_PORT_COUNT > 7U)
    kGINT_Port7 = 7U,
#endif
} gint_port_t;

/*! @brief GINT Callback function. */
typedef void (*gint_cb_t)(void);

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief	Initialize GINT peripheral.

 * This function initializes the GINT peripheral and enables the clock.
 *
 * @param base Base address of the GINT peripheral.
 *
 * @retval None.
 */
void GINT_Init(GINT_Type *base);

/*!
 * @brief	Setup GINT peripheral control parameters.

 * This function sets the control parameters of GINT peripheral.
 *
 * @param base Base address of the GINT peripheral.
 * @param comb Controls if the enabled inputs are logically ORed or ANDed for interrupt generation.
 * @param trig Controls if the enabled inputs are level or edge sensitive based on polarity.
 * @param callback This function is called when configured group interrupt is generated.
 *
 * @retval None.
 */
void GINT_SetCtrl(GINT_Type *base, gint_comb_t comb, gint_trig_t trig, gint_cb_t callback);

/*!
 * @brief	Get GINT peripheral control parameters.

 * This function returns the control parameters of GINT peripheral.
 *
 * @param base Base address of the GINT peripheral.
 * @param comb Pointer to store combine input value.
 * @param trig Pointer to store trigger value.
 * @param callback Pointer to store callback function.
 *
 * @retval None.
 */
void GINT_GetCtrl(GINT_Type *base, gint_comb_t *comb, gint_trig_t *trig, gint_cb_t *callback);

/*!
 * @brief	Configure GINT peripheral pins.

 * This function enables and controls the polarity of enabled pin(s) of a given port.
 *
 * @param base Base address of the GINT peripheral.
 * @param port Port number.
 * @param polarityMask Each bit position selects the polarity of the corresponding enabled pin.
 *        0 = The pin is active LOW. 1 = The pin is active HIGH.
 * @param enableMask Each bit position selects if the corresponding pin is enabled or not.
 *        0 = The pin is disabled. 1 = The pin is enabled.
 *
 * @retval None.
 */
void GINT_ConfigPins(GINT_Type *base, gint_port_t port, uint32_t polarityMask, uint32_t enableMask);

/*!
 * @brief	Get GINT peripheral pin configuration.

 * This function returns the pin configuration of a given port.
 *
 * @param base Base address of the GINT peripheral.
 * @param port Port number.
 * @param polarityMask Pointer to store the polarity mask Each bit position indicates the polarity of the corresponding
 enabled pin.
 *        0 = The pin is active LOW. 1 = The pin is active HIGH.
 * @param enableMask Pointer to store the enable mask. Each bit position indicates if the corresponding pin is enabled
 or not.
 *        0 = The pin is disabled. 1 = The pin is enabled.
 *
 * @retval None.
 */
void GINT_GetConfigPins(GINT_Type *base, gint_port_t port, uint32_t *polarityMask, uint32_t *enableMask);

/*!
 * @brief	Enable callback.

 * This function enables the interrupt for the selected GINT peripheral. Although the pin(s) are monitored
 * as soon as they are enabled, the callback function is not enabled until this function is called.
 *
 * @param base Base address of the GINT peripheral.
 *
 * @retval None.
 */
void GINT_EnableCallback(GINT_Type *base);

/*!
 * @brief	Disable callback.

 * This function disables the interrupt for the selected GINT peripheral. Although the pins are still
 * being monitored but the callback function is not called.
 *
 * @param base Base address of the peripheral.
 *
 * @retval None.
 */
void GINT_DisableCallback(GINT_Type *base);

/*!
 * @brief	Clear GINT status.

 * This function clears the GINT status bit.
 *
 * @param base Base address of the GINT peripheral.
 *
 * @retval None.
 */
static inline void GINT_ClrStatus(GINT_Type *base)
{
    base->CTRL |= GINT_CTRL_INT_MASK;
}

/*!
 * @brief	Get GINT status.

 * This function returns the GINT status.
 *
 * @param base Base address of the GINT peripheral.
 *
 * @retval status = 0 No group interrupt request.  = 1 Group interrupt request active.
 */
static inline uint32_t GINT_GetStatus(GINT_Type *base)
{
    return (base->CTRL & GINT_CTRL_INT_MASK);
}

/*!
 * @brief	Deinitialize GINT peripheral.

 * This function disables the GINT clock.
 *
 * @param base Base address of the GINT peripheral.
 *
 * @retval None.
 */
void GINT_Deinit(GINT_Type *base);

#ifdef __cplusplus
}
#endif

/*@}*/

#endif /* _FSL_GINT_H_ */
