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

#ifndef _FSL_PINT_H_
#define _FSL_PINT_H_

#include "fsl_common.h"

/*!
 * @addtogroup pint_driver
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_PINT_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0 */
/*@}*/

/* Number of interrupt line supported by PINT */
#define PINT_PIN_INT_COUNT 8U

/* Number of input sources supported by PINT */
#define PINT_INPUT_COUNT 8U

/* PININT Bit slice source register bits */
#define PININT_BITSLICE_SRC_START 8U
#define PININT_BITSLICE_SRC_MASK 7U

/* PININT Bit slice configuration register bits */
#define PININT_BITSLICE_CFG_START 8U
#define PININT_BITSLICE_CFG_MASK 7U
#define PININT_BITSLICE_ENDP_MASK 7U

#define PINT_PIN_INT_LEVEL 0x10U
#define PINT_PIN_INT_EDGE 0x00U
#define PINT_PIN_INT_FALL_OR_HIGH_LEVEL 0x02U
#define PINT_PIN_INT_RISE 0x01U
#define PINT_PIN_RISE_EDGE (PINT_PIN_INT_EDGE | PINT_PIN_INT_RISE)
#define PINT_PIN_FALL_EDGE (PINT_PIN_INT_EDGE | PINT_PIN_INT_FALL_OR_HIGH_LEVEL)
#define PINT_PIN_BOTH_EDGE (PINT_PIN_INT_EDGE | PINT_PIN_INT_RISE | PINT_PIN_INT_FALL_OR_HIGH_LEVEL)
#define PINT_PIN_LOW_LEVEL (PINT_PIN_INT_LEVEL)
#define PINT_PIN_HIGH_LEVEL (PINT_PIN_INT_LEVEL | PINT_PIN_INT_FALL_OR_HIGH_LEVEL)

/*! @brief PINT Pin Interrupt enable type */
typedef enum _pint_pin_enable
{
    kPINT_PinIntEnableNone = 0U,                      /*!< Do not generate Pin Interrupt */
    kPINT_PinIntEnableRiseEdge = PINT_PIN_RISE_EDGE,  /*!< Generate Pin Interrupt on rising edge */
    kPINT_PinIntEnableFallEdge = PINT_PIN_FALL_EDGE,  /*!< Generate Pin Interrupt on falling edge */
    kPINT_PinIntEnableBothEdges = PINT_PIN_BOTH_EDGE, /*!< Generate Pin Interrupt on both edges */
    kPINT_PinIntEnableLowLevel = PINT_PIN_LOW_LEVEL,  /*!< Generate Pin Interrupt on low level */
    kPINT_PinIntEnableHighLevel = PINT_PIN_HIGH_LEVEL /*!< Generate Pin Interrupt on high level */
} pint_pin_enable_t;

/*! @brief PINT Pin Interrupt type */
typedef enum _pint_int
{
    kPINT_PinInt0 = 0U, /*!< Pin Interrupt  0 */
#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 1U)
    kPINT_PinInt1 = 1U, /*!< Pin Interrupt  1 */
#endif
#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 2U)
    kPINT_PinInt2 = 2U, /*!< Pin Interrupt  2 */
#endif
#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 3U)
    kPINT_PinInt3 = 3U, /*!< Pin Interrupt  3 */
#endif
#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 4U)
    kPINT_PinInt4 = 4U, /*!< Pin Interrupt  4 */
#endif
#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 5U)
    kPINT_PinInt5 = 5U, /*!< Pin Interrupt  5 */
#endif
#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 6U)
    kPINT_PinInt6 = 6U, /*!< Pin Interrupt  6 */
#endif
#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 7U)
    kPINT_PinInt7 = 7U, /*!< Pin Interrupt  7 */
#endif
} pint_pin_int_t;

/*! @brief PINT Pattern Match bit slice input source type */
typedef enum _pint_pmatch_input_src
{
    kPINT_PatternMatchInp0Src = 0U, /*!< Input source 0 */
    kPINT_PatternMatchInp1Src = 1U, /*!< Input source 1 */
    kPINT_PatternMatchInp2Src = 2U, /*!< Input source 2 */
    kPINT_PatternMatchInp3Src = 3U, /*!< Input source 3 */
    kPINT_PatternMatchInp4Src = 4U, /*!< Input source 4 */
    kPINT_PatternMatchInp5Src = 5U, /*!< Input source 5 */
    kPINT_PatternMatchInp6Src = 6U, /*!< Input source 6 */
    kPINT_PatternMatchInp7Src = 7U, /*!< Input source 7 */
} pint_pmatch_input_src_t;

/*! @brief PINT Pattern Match bit slice type */
typedef enum _pint_pmatch_bslice
{
    kPINT_PatternMatchBSlice0 = 0U, /*!< Bit slice 0 */
#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 1U)
    kPINT_PatternMatchBSlice1 = 1U, /*!< Bit slice 1 */
#endif
#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 2U)
    kPINT_PatternMatchBSlice2 = 2U, /*!< Bit slice 2 */
#endif
#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 3U)
    kPINT_PatternMatchBSlice3 = 3U, /*!< Bit slice 3 */
#endif
#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 4U)
    kPINT_PatternMatchBSlice4 = 4U, /*!< Bit slice 4 */
#endif
#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 5U)
    kPINT_PatternMatchBSlice5 = 5U, /*!< Bit slice 5 */
#endif
#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 6U)
    kPINT_PatternMatchBSlice6 = 6U, /*!< Bit slice 6 */
#endif
#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 7U)
    kPINT_PatternMatchBSlice7 = 7U, /*!< Bit slice 7 */
#endif
} pint_pmatch_bslice_t;

/*! @brief PINT Pattern Match configuration type */
typedef enum _pint_pmatch_bslice_cfg
{
    kPINT_PatternMatchAlways = 0U,          /*!< Always Contributes to product term match */
    kPINT_PatternMatchStickyRise = 1U,      /*!< Sticky Rising edge */
    kPINT_PatternMatchStickyFall = 2U,      /*!< Sticky Falling edge */
    kPINT_PatternMatchStickyBothEdges = 3U, /*!< Sticky Rising or Falling edge */
    kPINT_PatternMatchHigh = 4U,            /*!< High level */
    kPINT_PatternMatchLow = 5U,             /*!< Low level */
    kPINT_PatternMatchNever = 6U,           /*!< Never contributes to product term match */
    kPINT_PatternMatchBothEdges = 7U,       /*!< Either rising or falling edge */
} pint_pmatch_bslice_cfg_t;

/*! @brief PINT Callback function. */
typedef void (*pint_cb_t)(pint_pin_int_t pintr, uint32_t pmatch_status);

typedef struct _pint_pmatch_cfg
{
    pint_pmatch_input_src_t bs_src;
    pint_pmatch_bslice_cfg_t bs_cfg;
    bool end_point;
    pint_cb_t callback;
} pint_pmatch_cfg_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief	Initialize PINT peripheral.

 * This function initializes the PINT peripheral and enables the clock.
 *
 * @param base Base address of the PINT peripheral.
 *
 * @retval None.
 */
void PINT_Init(PINT_Type *base);

/*!
 * @brief	Configure PINT peripheral pin interrupt.

 * This function configures a given pin interrupt.
 *
 * @param base Base address of the PINT peripheral.
 * @param intr Pin interrupt.
 * @param enable Selects detection logic.
 * @param callback Callback.
 *
 * @retval None.
 */
void PINT_PinInterruptConfig(PINT_Type *base, pint_pin_int_t intr, pint_pin_enable_t enable, pint_cb_t callback);

/*!
 * @brief	Get PINT peripheral pin interrupt configuration.

 * This function returns the configuration of a given pin interrupt.
 *
 * @param base Base address of the PINT peripheral.
 * @param pintr Pin interrupt.
 * @param enable Pointer to store the detection logic.
 * @param callback Callback.
 *
 * @retval None.
 */
void PINT_PinInterruptGetConfig(PINT_Type *base, pint_pin_int_t pintr, pint_pin_enable_t *enable, pint_cb_t *callback);

/*!
 * @brief	Clear Selected pin interrupt status.

 * This function clears the selected pin interrupt status.
 *
 * @param base Base address of the PINT peripheral.
 * @param pintr Pin interrupt.
 *
 * @retval None.
 */
static inline void PINT_PinInterruptClrStatus(PINT_Type *base, pint_pin_int_t pintr)
{
    base->IST = (1U << pintr);
}

/*!
 * @brief	Get Selected pin interrupt status.

 * This function returns the selected pin interrupt status.
 *
 * @param base Base address of the PINT peripheral.
 * @param pintr Pin interrupt.
 *
 * @retval status = 0 No pin interrupt request.  = 1 Selected Pin interrupt request active.
 */
static inline uint32_t PINT_PinInterruptGetStatus(PINT_Type *base, pint_pin_int_t pintr)
{
    return ((base->IST & (1U << pintr)) ? 1U : 0U);
}

/*!
 * @brief	Clear all pin interrupts status.

 * This function clears the status of all pin interrupts.
 *
 * @param base Base address of the PINT peripheral.
 *
 * @retval None.
 */
static inline void PINT_PinInterruptClrStatusAll(PINT_Type *base)
{
    base->IST = PINT_IST_PSTAT_MASK;
}

/*!
 * @brief	Get all pin interrupts status.

 * This function returns the status of all pin interrupts.
 *
 * @param base Base address of the PINT peripheral.
 *
 * @retval status Each bit position indicates the status of corresponding pin interrupt.
 * = 0 No pin interrupt request. = 1 Pin interrupt request active.
 */
static inline uint32_t PINT_PinInterruptGetStatusAll(PINT_Type *base)
{
    return (base->IST);
}

/*!
 * @brief	Clear Selected pin interrupt fall flag.

 * This function clears the selected pin interrupt fall flag.
 *
 * @param base Base address of the PINT peripheral.
 * @param pintr Pin interrupt.
 *
 * @retval None.
 */
static inline void PINT_PinInterruptClrFallFlag(PINT_Type *base, pint_pin_int_t pintr)
{
    base->FALL = (1U << pintr);
}

/*!
 * @brief	Get selected pin interrupt fall flag.

 * This function returns the selected pin interrupt fall flag.
 *
 * @param base Base address of the PINT peripheral.
 * @param pintr Pin interrupt.
 *
 * @retval flag = 0 Falling edge has not been detected.  = 1 Falling edge has been detected.
 */
static inline uint32_t PINT_PinInterruptGetFallFlag(PINT_Type *base, pint_pin_int_t pintr)
{
    return ((base->FALL & (1U << pintr)) ? 1U : 0U);
}

/*!
 * @brief	Clear all pin interrupt fall flags.

 * This function clears the fall flag for all pin interrupts.
 *
 * @param base Base address of the PINT peripheral.
 *
 * @retval None.
 */
static inline void PINT_PinInterruptClrFallFlagAll(PINT_Type *base)
{
    base->FALL = PINT_FALL_FDET_MASK;
}

/*!
 * @brief	Get all pin interrupt fall flags.

 * This function returns the fall flag of all pin interrupts.
 *
 * @param base Base address of the PINT peripheral.
 *
 * @retval flags Each bit position indicates the falling edge detection of the corresponding pin interrupt.
 * 0 Falling edge has not been detected.  = 1 Falling edge has been detected.
 */
static inline uint32_t PINT_PinInterruptGetFallFlagAll(PINT_Type *base)
{
    return (base->FALL);
}

/*!
 * @brief	Clear Selected pin interrupt rise flag.

 * This function clears the selected pin interrupt rise flag.
 *
 * @param base Base address of the PINT peripheral.
 * @param pintr Pin interrupt.
 *
 * @retval None.
 */
static inline void PINT_PinInterruptClrRiseFlag(PINT_Type *base, pint_pin_int_t pintr)
{
    base->RISE = (1U << pintr);
}

/*!
 * @brief	Get selected pin interrupt rise flag.

 * This function returns the selected pin interrupt rise flag.
 *
 * @param base Base address of the PINT peripheral.
 * @param pintr Pin interrupt.
 *
 * @retval flag = 0 Rising edge has not been detected.  = 1 Rising edge has been detected.
 */
static inline uint32_t PINT_PinInterruptGetRiseFlag(PINT_Type *base, pint_pin_int_t pintr)
{
    return ((base->RISE & (1U << pintr)) ? 1U : 0U);
}

/*!
 * @brief	Clear all pin interrupt rise flags.

 * This function clears the rise flag for all pin interrupts.
 *
 * @param base Base address of the PINT peripheral.
 *
 * @retval None.
 */
static inline void PINT_PinInterruptClrRiseFlagAll(PINT_Type *base)
{
    base->RISE = PINT_RISE_RDET_MASK;
}

/*!
 * @brief	Get all pin interrupt rise flags.

 * This function returns the rise flag of all pin interrupts.
 *
 * @param base Base address of the PINT peripheral.
 *
 * @retval flags Each bit position indicates the rising edge detection of the corresponding pin interrupt.
 * 0 Rising edge has not been detected.  = 1 Rising edge has been detected.
 */
static inline uint32_t PINT_PinInterruptGetRiseFlagAll(PINT_Type *base)
{
    return (base->RISE);
}

/*!
 * @brief	Configure PINT pattern match.

 * This function configures a given pattern match bit slice.
 *
 * @param base Base address of the PINT peripheral.
 * @param bslice Pattern match bit slice number.
 * @param cfg Pointer to bit slice configuration.
 *
 * @retval None.
 */
void PINT_PatternMatchConfig(PINT_Type *base, pint_pmatch_bslice_t bslice, pint_pmatch_cfg_t *cfg);

/*!
 * @brief	Get PINT pattern match configuration.

 * This function returns the configuration of a given pattern match bit slice.
 *
 * @param base Base address of the PINT peripheral.
 * @param bslice Pattern match bit slice number.
 * @param cfg Pointer to bit slice configuration.
 *
 * @retval None.
 */
void PINT_PatternMatchGetConfig(PINT_Type *base, pint_pmatch_bslice_t bslice, pint_pmatch_cfg_t *cfg);

/*!
 * @brief	Get pattern match bit slice status.

 * This function returns the status of selected bit slice.
 *
 * @param base Base address of the PINT peripheral.
 * @param bslice Pattern match bit slice number.
 *
 * @retval status = 0 Match has not been detected.  = 1 Match has been detected.
 */
static inline uint32_t PINT_PatternMatchGetStatus(PINT_Type *base, pint_pmatch_bslice_t bslice)
{
    return ((base->PMCTRL >> PINT_PMCTRL_PMAT_SHIFT) & (0x1U << bslice)) >> bslice;
}

/*!
 * @brief	Get status of all pattern match bit slices.

 * This function returns the status of all bit slices.
 *
 * @param base Base address of the PINT peripheral.
 *
 * @retval status Each bit position indicates the match status of corresponding bit slice.
 * = 0 Match has not been detected.  = 1 Match has been detected.
 */
static inline uint32_t PINT_PatternMatchGetStatusAll(PINT_Type *base)
{
    return base->PMCTRL >> PINT_PMCTRL_PMAT_SHIFT;
}

/*!
 * @brief	Reset pattern match detection logic.

 * This function resets the pattern match detection logic if any of the product term is matching.
 *
 * @param base Base address of the PINT peripheral.
 *
 * @retval pmstatus Each bit position indicates the match status of corresponding bit slice.
 * = 0 Match was detected.  = 1 Match was not detected.
 */
uint32_t PINT_PatternMatchResetDetectLogic(PINT_Type *base);

/*!
 * @brief	Enable pattern match function.

 * This function enables the pattern match function.
 *
 * @param base Base address of the PINT peripheral.
 *
 * @retval	None.
 */
static inline void PINT_PatternMatchEnable(PINT_Type *base)
{
    base->PMCTRL = (base->PMCTRL & PINT_PMCTRL_ENA_RXEV_MASK) | PINT_PMCTRL_SEL_PMATCH_MASK;
}

/*!
 * @brief	Disable pattern match function.

 * This function disables the pattern match function.
 *
 * @param base Base address of the PINT peripheral.
 *
 * @retval	None.
 */
static inline void PINT_PatternMatchDisable(PINT_Type *base)
{
    base->PMCTRL = (base->PMCTRL & PINT_PMCTRL_ENA_RXEV_MASK) & ~PINT_PMCTRL_SEL_PMATCH_MASK;
}

/*!
 * @brief	Enable RXEV output.

 * This function enables the pattern match RXEV output.
 *
 * @param base Base address of the PINT peripheral.
 *
 * @retval	None.
 */
static inline void PINT_PatternMatchEnableRXEV(PINT_Type *base)
{
    base->PMCTRL = (base->PMCTRL & PINT_PMCTRL_SEL_PMATCH_MASK) | PINT_PMCTRL_ENA_RXEV_MASK;
}

/*!
 * @brief	Disable RXEV output.

 * This function disables the pattern match RXEV output.
 *
 * @param base Base address of the PINT peripheral.
 *
 * @retval	None.
 */
static inline void PINT_PatternMatchDisableRXEV(PINT_Type *base)
{
    base->PMCTRL = (base->PMCTRL & PINT_PMCTRL_SEL_PMATCH_MASK) & ~PINT_PMCTRL_ENA_RXEV_MASK;
}

/*!
 * @brief	Enable callback.

 * This function enables the interrupt for the selected PINT peripheral. Although the pin(s) are monitored
 * as soon as they are enabled, the callback function is not enabled until this function is called.
 *
 * @param base Base address of the PINT peripheral.
 *
 * @retval None.
 */
void PINT_EnableCallback(PINT_Type *base);

/*!
 * @brief	Disable callback.

 * This function disables the interrupt for the selected PINT peripheral. Although the pins are still
 * being monitored but the callback function is not called.
 *
 * @param base Base address of the peripheral.
 *
 * @retval None.
 */
void PINT_DisableCallback(PINT_Type *base);

/*!
 * @brief	Deinitialize PINT peripheral.

 * This function disables the PINT clock.
 *
 * @param base Base address of the PINT peripheral.
 *
 * @retval None.
 */
void PINT_Deinit(PINT_Type *base);

#ifdef __cplusplus
}
#endif

/*@}*/

#endif /* _FSL_PINT_H_ */
