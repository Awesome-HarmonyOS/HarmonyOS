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

#ifndef _FSL_IOCON_H_
#define _FSL_IOCON_H_

#include "fsl_common.h"

/*!
 * @addtogroup lpc_iocon
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief IOCON driver version 2.0.0. */
#define LPC_IOCON_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/**
 * @brief Array of IOCON pin definitions passed to IOCON_SetPinMuxing() must be in this format
 */
typedef struct _iocon_group
{
    uint32_t port : 8;      /* Pin port */
    uint32_t pin : 8;       /* Pin number */
    uint32_t ionumber : 8;  /* IO number */
    uint32_t modefunc : 16; /* Function and mode */
} iocon_group_t;

/**
 * @brief IOCON function and mode selection definitions
 * @note See the User Manual for specific modes and functions supported by the various pins.
 */
#if defined(FSL_FEATURE_IOCON_FUNC_FIELD_WIDTH) && (FSL_FEATURE_IOCON_FUNC_FIELD_WIDTH == 4)
#define IOCON_FUNC0 0x0                   /*!< Selects pin function 0 */
#define IOCON_FUNC1 0x1                   /*!< Selects pin function 1 */
#define IOCON_FUNC2 0x2                   /*!< Selects pin function 2 */
#define IOCON_FUNC3 0x3                   /*!< Selects pin function 3 */
#define IOCON_FUNC4 0x4                   /*!< Selects pin function 4 */
#define IOCON_FUNC5 0x5                   /*!< Selects pin function 5 */
#define IOCON_FUNC6 0x6                   /*!< Selects pin function 6 */
#define IOCON_FUNC7 0x7                   /*!< Selects pin function 7 */
#define IOCON_FUNC8 0x8                   /*!< Selects pin function 8 */
#define IOCON_FUNC9 0x9                   /*!< Selects pin function 9 */
#define IOCON_FUNC10 0xA                  /*!< Selects pin function 10 */
#define IOCON_FUNC11 0xB                  /*!< Selects pin function 11 */
#define IOCON_FUNC12 0xC                  /*!< Selects pin function 12 */
#define IOCON_FUNC13 0xD                  /*!< Selects pin function 13 */
#define IOCON_FUNC14 0xE                  /*!< Selects pin function 14 */
#define IOCON_FUNC15 0xF                  /*!< Selects pin function 15 */
#define IOCON_MODE_INACT (0x0 << 4)       /*!< No addition pin function */
#define IOCON_MODE_PULLDOWN (0x1 << 4)    /*!< Selects pull-down function */
#define IOCON_MODE_PULLUP (0x2 << 4)      /*!< Selects pull-up function */
#define IOCON_MODE_REPEATER (0x3 << 4)    /*!< Selects pin repeater function */
#define IOCON_HYS_EN (0x1 << 6)           /*!< Enables hysteresis */
#define IOCON_GPIO_MODE (0x1 << 6)        /*!< GPIO Mode */
#define IOCON_I2C_SLEW (0x0 << 6)         /*!< I2C Slew Rate Control */
#define IOCON_INV_EN (0x1 << 7)           /*!< Enables invert function on input */
#define IOCON_ANALOG_EN (0x0 << 8)        /*!< Enables analog function by setting 0 to bit 7 */
#define IOCON_DIGITAL_EN (0x1 << 8)       /*!< Enables digital function by setting 1 to bit 7(default) */
#define IOCON_STDI2C_EN (0x1 << 9)        /*!< I2C standard mode/fast-mode */
#define IOCON_FASTI2C_EN (0x3 << 9)       /*!< I2C Fast-mode Plus and high-speed slave */
#define IOCON_INPFILT_OFF (0x1 << 9)      /*!< Input filter Off for GPIO pins */
#define IOCON_INPFILT_ON (0x0 << 9)       /*!< Input filter On for GPIO pins */
#define IOCON_OPENDRAIN_EN (0x1 << 11)    /*!< Enables open-drain function */
#define IOCON_S_MODE_0CLK (0x0 << 12)     /*!< Bypass input filter */
#define IOCON_S_MODE_1CLK (0x1 << 12)     /*!< Input pulses shorter than 1 filter clock are rejected */
#define IOCON_S_MODE_2CLK (0x2 << 12)     /*!< Input pulses shorter than 2 filter clock2 are rejected */
#define IOCON_S_MODE_3CLK (0x3 << 12)     /*!< Input pulses shorter than 3 filter clock2 are rejected */
#define IOCON_S_MODE(clks) ((clks) << 12) /*!< Select clocks for digital input filter mode */
#define IOCON_CLKDIV(div) \
    ((div) << 14) /*!< Select peripheral clock divider for input filter sampling clock, 2^n, n=0-6 */
#else
#define IOCON_FUNC0 0x0                   /*!< Selects pin function 0 */
#define IOCON_FUNC1 0x1                   /*!< Selects pin function 1 */
#define IOCON_FUNC2 0x2                   /*!< Selects pin function 2 */
#define IOCON_FUNC3 0x3                   /*!< Selects pin function 3 */
#define IOCON_FUNC4 0x4                   /*!< Selects pin function 4 */
#define IOCON_FUNC5 0x5                   /*!< Selects pin function 5 */
#define IOCON_FUNC6 0x6                   /*!< Selects pin function 6 */
#define IOCON_FUNC7 0x7                   /*!< Selects pin function 7 */
#define IOCON_MODE_INACT (0x0 << 3)       /*!< No addition pin function */
#define IOCON_MODE_PULLDOWN (0x1 << 3)    /*!< Selects pull-down function */
#define IOCON_MODE_PULLUP (0x2 << 3)      /*!< Selects pull-up function */
#define IOCON_MODE_REPEATER (0x3 << 3)    /*!< Selects pin repeater function */
#define IOCON_HYS_EN (0x1 << 5)           /*!< Enables hysteresis */
#define IOCON_GPIO_MODE (0x1 << 5)        /*!< GPIO Mode */
#define IOCON_I2C_SLEW (0x0 << 5)         /*!< I2C Slew Rate Control */
#define IOCON_INV_EN (0x1 << 6)           /*!< Enables invert function on input */
#define IOCON_ANALOG_EN (0x0 << 7)        /*!< Enables analog function by setting 0 to bit 7 */
#define IOCON_DIGITAL_EN (0x1 << 7)       /*!< Enables digital function by setting 1 to bit 7(default) */
#define IOCON_STDI2C_EN (0x1 << 8)        /*!< I2C standard mode/fast-mode */
#define IOCON_FASTI2C_EN (0x3 << 8)       /*!< I2C Fast-mode Plus and high-speed slave */
#define IOCON_INPFILT_OFF (0x1 << 8)      /*!< Input filter Off for GPIO pins */
#define IOCON_INPFILT_ON (0x0 << 8)       /*!< Input filter On for GPIO pins */
#define IOCON_OPENDRAIN_EN (0x1 << 10)    /*!< Enables open-drain function */
#define IOCON_S_MODE_0CLK (0x0 << 11)     /*!< Bypass input filter */
#define IOCON_S_MODE_1CLK (0x1 << 11)     /*!< Input pulses shorter than 1 filter clock are rejected */
#define IOCON_S_MODE_2CLK (0x2 << 11)     /*!< Input pulses shorter than 2 filter clock2 are rejected */
#define IOCON_S_MODE_3CLK (0x3 << 11)     /*!< Input pulses shorter than 3 filter clock2 are rejected */
#define IOCON_S_MODE(clks) ((clks) << 11) /*!< Select clocks for digital input filter mode */
#define IOCON_CLKDIV(div) \
    ((div) << 13) /*!< Select peripheral clock divider for input filter sampling clock, 2^n, n=0-6 */
#endif
#if defined(__cplusplus)
extern "C" {
#endif

#if (defined(FSL_FEATURE_IOCON_ONE_DIMENSION) && (FSL_FEATURE_IOCON_ONE_DIMENSION == 1))
/**
 * @brief   Sets I/O Control pin mux
 * @param   base        : The base of IOCON peripheral on the chip
 * @param   ionumber    : GPIO number to mux
 * @param   modefunc    : OR'ed values of type IOCON_*
 * @return  Nothing
 */
__STATIC_INLINE void IOCON_PinMuxSet(IOCON_Type *base, uint8_t ionumber, uint32_t modefunc)
{
    base->PIO[ionumber] = modefunc;
}
#else
/**
 * @brief   Sets I/O Control pin mux
 * @param   base        : The base of IOCON peripheral on the chip
 * @param   port        : GPIO port to mux
 * @param   pin         : GPIO pin to mux
 * @param   modefunc    : OR'ed values of type IOCON_*
 * @return  Nothing
 */
__STATIC_INLINE void IOCON_PinMuxSet(IOCON_Type *base, uint8_t port, uint8_t pin, uint32_t modefunc)
{
    base->PIO[port][pin] = modefunc;
}
#endif

/**
 * @brief   Set all I/O Control pin muxing
 * @param   base        : The base of IOCON peripheral on the chip
 * @param   pinArray    : Pointer to array of pin mux selections
 * @param   arrayLength : Number of entries in pinArray
 * @return  Nothing
 */
__STATIC_INLINE void IOCON_SetPinMuxing(IOCON_Type *base, const iocon_group_t *pinArray, uint32_t arrayLength)
{
    uint32_t i;

    for (i = 0; i < arrayLength; i++)
    {
#if (defined(FSL_FEATURE_IOCON_ONE_DIMENSION) && (FSL_FEATURE_IOCON_ONE_DIMENSION == 1))
        IOCON_PinMuxSet(base, pinArray[i].ionumber, pinArray[i].modefunc);
#else
        IOCON_PinMuxSet(base, pinArray[i].port, pinArray[i].pin, pinArray[i].modefunc);
#endif /* FSL_FEATURE_IOCON_ONE_DIMENSION */
    }
}

/* @} */

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_IOCON_H_ */
