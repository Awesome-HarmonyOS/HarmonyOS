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

#ifndef _FSL_INPUTMUX_H_
#define _FSL_INPUTMUX_H_

#include "fsl_inputmux_connections.h"
#include "fsl_common.h"

/*!
 * @addtogroup inputmux_driver
 * @{
 */

/*! @file */
/*! @file fsl_inputmux_connections.h */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief Group interrupt driver version for SDK */
#define FSL_INPUTMUX_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0. */
                                                            /*@}*/

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief	Initialize INPUTMUX peripheral.

 * This function enables the INPUTMUX clock.
 *
 * @param base Base address of the INPUTMUX peripheral.
 *
 * @retval None.
 */
void INPUTMUX_Init(INPUTMUX_Type *base);

/*!
 * @brief Attaches a signal
 *
 * This function gates the INPUTPMUX clock.
 *
 * @param base Base address of the INPUTMUX peripheral.
 * @param index Destination peripheral to attach the signal to.
 * @param connection Selects connection.
 *
 * @retval None.
*/
void INPUTMUX_AttachSignal(INPUTMUX_Type *base, uint32_t index, inputmux_connection_t connection);

/*!
 * @brief	Deinitialize INPUTMUX peripheral.

 * This function disables the INPUTMUX clock.
 *
 * @param base Base address of the INPUTMUX peripheral.
 *
 * @retval None.
 */
void INPUTMUX_Deinit(INPUTMUX_Type *base);

#ifdef __cplusplus
}
#endif

/*@}*/

#endif /* _FSL_INPUTMUX_H_ */
