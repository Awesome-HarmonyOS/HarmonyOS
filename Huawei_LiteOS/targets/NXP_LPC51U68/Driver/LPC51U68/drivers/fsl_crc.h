/*
 * The Clear BSD License
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
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

#ifndef _FSL_CRC_H_
#define _FSL_CRC_H_

#include "fsl_common.h"

/*!
 * @addtogroup crc
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief CRC driver version. Version 2.0.1.
 *
 * Current version: 2.0.1
 *
 * Change log:
 * - Version 2.0.0
 *   - initial version
 * - Version 2.0.1
 *   - add explicit type cast when writing to WR_DATA
 */
#define FSL_CRC_DRIVER_VERSION (MAKE_VERSION(2, 0, 1))
/*@}*/

#ifndef CRC_DRIVER_CUSTOM_DEFAULTS
/*! @brief Default configuration structure filled by CRC_GetDefaultConfig(). Uses CRC-16/CCITT-FALSE as default. */
#define CRC_DRIVER_USE_CRC16_CCITT_FALSE_AS_DEFAULT 1
#endif

/*! @brief CRC polynomials to use. */
typedef enum _crc_polynomial
{
    kCRC_Polynomial_CRC_CCITT = 0U, /*!< x^16+x^12+x^5+1 */
    kCRC_Polynomial_CRC_16 = 1U,    /*!< x^16+x^15+x^2+1 */
    kCRC_Polynomial_CRC_32 = 2U     /*!< x^32+x^26+x^23+x^22+x^16+x^12+x^11+x^10+x^8+x^7+x^5+x^4+x^2+x+1 */
} crc_polynomial_t;

/*!
* @brief CRC protocol configuration.
*
* This structure holds the configuration for the CRC protocol.
*
*/
typedef struct _crc_config
{
    crc_polynomial_t polynomial; /*!< CRC polynomial. */
    bool reverseIn;              /*!< Reverse bits on input. */
    bool complementIn;           /*!< Perform 1's complement on input. */
    bool reverseOut;             /*!< Reverse bits on output. */
    bool complementOut;          /*!< Perform 1's complement on output. */
    uint32_t seed;               /*!< Starting checksum value. */
} crc_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Enables and configures the CRC peripheral module.
 *
 * This functions enables the CRC peripheral clock in the LPC SYSCON block.
 * It also configures the CRC engine and starts checksum computation by writing the seed.
 *
 * @param base   CRC peripheral address.
 * @param config CRC module configuration structure.
 */
void CRC_Init(CRC_Type *base, const crc_config_t *config);

/*!
 * @brief Disables the CRC peripheral module.
 *
 * This functions disables the CRC peripheral clock in the LPC SYSCON block.
 *
 * @param base CRC peripheral address.
 */
static inline void CRC_Deinit(CRC_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* disable clock to CRC */
    CLOCK_DisableClock(kCLOCK_Crc);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

/*!
 * @brief resets CRC peripheral module.
 *
 * @param base   CRC peripheral address.
 */
void CRC_Reset(CRC_Type *base);

/*!
 * @brief Loads default values to CRC protocol configuration structure.
 *
 * Loads default values to CRC protocol configuration structure. The default values are:
 * @code
 *   config->polynomial = kCRC_Polynomial_CRC_CCITT;
 *   config->reverseIn = false;
 *   config->complementIn = false;
 *   config->reverseOut = false;
 *   config->complementOut = false;
 *   config->seed = 0xFFFFU;
 * @endcode
 *
 * @param config CRC protocol configuration structure
 */
void CRC_GetDefaultConfig(crc_config_t *config);

/*!
 * @brief Loads actual values configured in CRC peripheral to CRC protocol configuration structure.
 *
 * The values, including seed, can be used to resume CRC calculation later.

 * @param base   CRC peripheral address.
 * @param config CRC protocol configuration structure
 */
void CRC_GetConfig(CRC_Type *base, crc_config_t *config);

/*!
 * @brief Writes data to the CRC module.
 *
 * Writes input data buffer bytes to CRC data register.
 *
 * @param base     CRC peripheral address.
 * @param data     Input data stream, MSByte in data[0].
 * @param dataSize Size of the input data buffer in bytes.
 */
void CRC_WriteData(CRC_Type *base, const uint8_t *data, size_t dataSize);

/*!
 * @brief Reads 32-bit checksum from the CRC module.
 *
 * Reads CRC data register.
 *
 * @param base CRC peripheral address.
 * @return final 32-bit checksum, after configured bit reverse and complement operations.
 */
static inline uint32_t CRC_Get32bitResult(CRC_Type *base)
{
    return base->SUM;
}

/*!
 * @brief Reads 16-bit checksum from the CRC module.
 *
 * Reads CRC data register.
 *
 * @param base CRC peripheral address.
 * @return final 16-bit checksum, after configured bit reverse and complement operations.
 */
static inline uint16_t CRC_Get16bitResult(CRC_Type *base)
{
    return (uint16_t)base->SUM;
}

#if defined(__cplusplus)
}
#endif

/*!
 *@}
 */

#endif /* _FSL_CRC_H_ */
