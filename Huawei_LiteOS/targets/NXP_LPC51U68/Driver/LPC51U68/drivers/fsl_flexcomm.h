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
#ifndef _FSL_FLEXCOMM_H_
#define _FSL_FLEXCOMM_H_

#include "fsl_common.h"

/*!
 * @addtogroup flexcomm_driver
 * @{
 */

/*! @name Driver version */
/*@{*/
/*! @brief FlexCOMM driver version 2.0.0. */
#define FSL_FLEXCOMM_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*! @brief FLEXCOMM peripheral modes. */
typedef enum
{
    FLEXCOMM_PERIPH_NONE,   /*!< No peripheral */
    FLEXCOMM_PERIPH_USART,  /*!< USART peripheral */
    FLEXCOMM_PERIPH_SPI,    /*!< SPI Peripheral */
    FLEXCOMM_PERIPH_I2C,    /*!< I2C Peripheral */
    FLEXCOMM_PERIPH_I2S_TX, /*!< I2S TX Peripheral */
    FLEXCOMM_PERIPH_I2S_RX, /*!< I2S RX Peripheral */
} FLEXCOMM_PERIPH_T;

/*! @brief Typedef for interrupt handler. */
typedef void (*flexcomm_irq_handler_t)(void *base, void *handle);

/*! @brief Array with IRQ number for each FLEXCOMM module. */
extern IRQn_Type const kFlexcommIrqs[];

/*! @brief Returns instance number for FLEXCOMM module with given base address. */
uint32_t FLEXCOMM_GetInstance(void *base);

/*! @brief Initializes FLEXCOMM and selects peripheral mode according to the second parameter. */
status_t FLEXCOMM_Init(void *base, FLEXCOMM_PERIPH_T periph);

/*! @brief Sets IRQ handler for given FLEXCOMM module. It is used by drivers register IRQ handler according to FLEXCOMM
 * mode */
void FLEXCOMM_SetIRQHandler(void *base, flexcomm_irq_handler_t handler, void *handle);

/*@}*/

#endif /* _FSL_FLEXCOMM_H_*/
