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
#include "fsl_crc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if defined(CRC_DRIVER_USE_CRC16_CCITT_FALSE_AS_DEFAULT) && CRC_DRIVER_USE_CRC16_CCITT_FALSE_AS_DEFAULT
/* @brief Default user configuration structure for CRC-CCITT */
#define CRC_DRIVER_DEFAULT_POLYNOMIAL kCRC_Polynomial_CRC_CCITT
/*< CRC-CCIT polynomial x^16 + x^12 + x^5 + x^0 */
#define CRC_DRIVER_DEFAULT_REVERSE_IN false
/*< Default is no bit reverse */
#define CRC_DRIVER_DEFAULT_COMPLEMENT_IN false
/*< Default is without complement of written data */
#define CRC_DRIVER_DEFAULT_REVERSE_OUT false
/*< Default is no bit reverse */
#define CRC_DRIVER_DEFAULT_COMPLEMENT_OUT false
/*< Default is without complement of CRC data register read data */
#define CRC_DRIVER_DEFAULT_SEED 0xFFFFU
/*< Default initial checksum */
#endif /* CRC_DRIVER_USE_CRC16_CCITT_FALSE_AS_DEFAULT */

/*******************************************************************************
 * Code
 ******************************************************************************/

void CRC_Init(CRC_Type *base, const crc_config_t *config)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* enable clock to CRC */
    CLOCK_EnableClock(kCLOCK_Crc);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    /* configure CRC module and write the seed */
    base->MODE = 0 | CRC_MODE_CRC_POLY(config->polynomial) | CRC_MODE_BIT_RVS_WR(config->reverseIn) |
                 CRC_MODE_CMPL_WR(config->complementIn) | CRC_MODE_BIT_RVS_SUM(config->reverseOut) |
                 CRC_MODE_CMPL_SUM(config->complementOut);
    base->SEED = config->seed;
}

void CRC_GetDefaultConfig(crc_config_t *config)
{
    static const crc_config_t default_config = {CRC_DRIVER_DEFAULT_POLYNOMIAL,     CRC_DRIVER_DEFAULT_REVERSE_IN,
                                                CRC_DRIVER_DEFAULT_COMPLEMENT_IN,  CRC_DRIVER_DEFAULT_REVERSE_OUT,
                                                CRC_DRIVER_DEFAULT_COMPLEMENT_OUT, CRC_DRIVER_DEFAULT_SEED};

    *config = default_config;
}

void CRC_Reset(CRC_Type *base)
{
    crc_config_t config;
    CRC_GetDefaultConfig(&config);
    CRC_Init(base, &config);
}

void CRC_GetConfig(CRC_Type *base, crc_config_t *config)
{
    /* extract CRC mode settings */
    uint32_t mode = base->MODE;
    config->polynomial = (crc_polynomial_t)((mode & CRC_MODE_CRC_POLY_MASK) >> CRC_MODE_CRC_POLY_SHIFT);
    config->reverseIn = (bool)(mode & CRC_MODE_BIT_RVS_WR_MASK);
    config->complementIn = (bool)(mode & CRC_MODE_CMPL_WR_MASK);
    config->reverseOut = (bool)(mode & CRC_MODE_BIT_RVS_SUM_MASK);
    config->complementOut = (bool)(mode & CRC_MODE_CMPL_SUM_MASK);

    /* reset CRC sum bit reverse and 1's complement setting, so its value can be used as a seed */
    base->MODE = mode & ~((1U << CRC_MODE_BIT_RVS_SUM_SHIFT) | (1U << CRC_MODE_CMPL_SUM_SHIFT));

    /* now we can obtain intermediate raw CRC sum value */
    config->seed = base->SUM;

    /* restore original CRC sum bit reverse and 1's complement setting */
    base->MODE = mode;
}

void CRC_WriteData(CRC_Type *base, const uint8_t *data, size_t dataSize)
{
    const uint32_t *data32;

    /* 8-bit reads and writes till source address is aligned 4 bytes */
    while ((dataSize) && ((uint32_t)data & 3U))
    {
        *((__O uint8_t *)&(base->WR_DATA)) = *data;
        data++;
        dataSize--;
    }

    /* use 32-bit reads and writes as long as possible */
    data32 = (const uint32_t *)data;
    while (dataSize >= sizeof(uint32_t))
    {
        *((__O uint32_t *)&(base->WR_DATA)) = *data32;
        data32++;
        dataSize -= sizeof(uint32_t);
    }

    data = (const uint8_t *)data32;

    /* 8-bit reads and writes till end of data buffer */
    while (dataSize)
    {
        *((__O uint8_t *)&(base->WR_DATA)) = *data;
        data++;
        dataSize--;
    }
}
