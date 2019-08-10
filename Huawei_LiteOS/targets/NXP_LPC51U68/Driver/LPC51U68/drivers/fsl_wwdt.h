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
#ifndef _FSL_WWDT_H_
#define _FSL_WWDT_H_

#include "fsl_common.h"

/*!
 * @addtogroup wwdt
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief Defines WWDT driver version 2.0.0. */
#define FSL_WWDT_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*! @name Refresh sequence */
/*@{*/
#define WWDT_FIRST_WORD_OF_REFRESH (0xAAU)  /*!< First word of refresh sequence */
#define WWDT_SECOND_WORD_OF_REFRESH (0x55U) /*!< Second word of refresh sequence */
/*@}*/

/*! @brief Describes WWDT configuration structure. */
typedef struct _wwdt_config
{
    bool enableWwdt;            /*!< Enables or disables WWDT */
    bool enableWatchdogReset;   /*!< true: Watchdog timeout will cause a chip reset
                                     false: Watchdog timeout will not cause a chip reset */
    bool enableWatchdogProtect; /*!< true: Enable watchdog protect; timeout value can only be
                                           changed after counter is below warning & window values
                                     false: Disable watchdog protect; timeout value can be changed
                                            at any time */
    bool enableLockOscillator;  /*!< true: Disabling or powering down the watchdog oscillator is prevented
                                           Once set, this bit can only be cleared by a reset
                                     false: Do not lock oscillator */
    uint32_t windowValue;       /*!< Window value, set this to 0xFFFFFF if windowing is not in effect */
    uint32_t timeoutValue;      /*!< Timeout value */
    uint32_t warningValue;      /*!< Watchdog time counter value that will generate a
                                     warning interrupt. Set this to 0 for no warning */

} wwdt_config_t;

/*!
 * @brief WWDT status flags.
 *
 * This structure contains the WWDT status flags for use in the WWDT functions.
 */
enum _wwdt_status_flags_t
{
    kWWDT_TimeoutFlag = WWDT_MOD_WDTOF_MASK, /*!< Time-out flag, set when the timer times out */
    kWWDT_WarningFlag = WWDT_MOD_WDINT_MASK  /*!< Warning interrupt flag, set when timer is below the value WDWARNINT */
};

/*******************************************************************************
 * API
 *******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @name WWDT Initialization and De-initialization
 * @{
 */

/*!
 * @brief Initializes WWDT configure sturcture.
 *
 * This function initializes the WWDT configure structure to default value. The default
 * value are:
 * @code
 *  config->enableWwdt = true;
 *  config->enableWatchdogReset = false;
 *  config->enableWatchdogProtect = false;
 *  config->enableLockOscillator = false;
 *  config->windowValue = 0xFFFFFFU;
 *  config->timeoutValue = 0xFFFFFFU;
 *  config->warningValue = 0;
 * @endcode
 *
 * @param config Pointer to WWDT config structure.
 * @see wwdt_config_t
 */
void WWDT_GetDefaultConfig(wwdt_config_t *config);

/*!
 * @brief Initializes the WWDT.
 *
 * This function initializes the WWDT. When called, the WWDT runs according to the configuration.
 *
 * Example:
 * @code
 *   wwdt_config_t config;
 *   WWDT_GetDefaultConfig(&config);
 *   config.timeoutValue = 0x7ffU;
 *   WWDT_Init(wwdt_base,&config);
 * @endcode
 *
 * @param base   WWDT peripheral base address
 * @param config The configuration of WWDT
 */
void WWDT_Init(WWDT_Type *base, const wwdt_config_t *config);

/*!
 * @brief Shuts down the WWDT.
 *
 * This function shuts down the WWDT.
 *
 * @param base WWDT peripheral base address
 */
void WWDT_Deinit(WWDT_Type *base);

/* @} */

/*!
 * @name WWDT Functional Operation
 * @{
 */

/*!
 * @brief Enables the WWDT module.
 *
 * This function write value into WWDT_MOD register to enable the WWDT, it is a write-once bit;
 * once this bit is set to one and a watchdog feed is performed, the watchdog timer will run
 * permanently.
 *
 * @param base WWDT peripheral base address
 */
static inline void WWDT_Enable(WWDT_Type *base)
{
    base->MOD |= WWDT_MOD_WDEN_MASK;
}

/*!
 * @brief Disables the WWDT module.
 *
 * This function write value into WWDT_MOD register to disable the WWDT.
 *
 * @param base WWDT peripheral base address
 */
static inline void WWDT_Disable(WWDT_Type *base)
{
    base->MOD &= ~WWDT_MOD_WDEN_MASK;
}

/*!
 * @brief Gets all WWDT status flags.
 *
 * This function gets all status flags.
 *
 * Example for getting Timeout Flag:
 * @code
 *   uint32_t status;
 *   status = WWDT_GetStatusFlags(wwdt_base) & kWWDT_TimeoutFlag;
 * @endcode
 * @param base        WWDT peripheral base address
 * @return The status flags. This is the logical OR of members of the
 *         enumeration ::_wwdt_status_flags_t
 */
static inline uint32_t WWDT_GetStatusFlags(WWDT_Type *base)
{
    return (base->MOD & (WWDT_MOD_WDTOF_MASK | WWDT_MOD_WDINT_MASK));
}

/*!
 * @brief Clear WWDT flag.
 *
 * This function clears WWDT status flag.
 *
 * Example for clearing warning flag:
 * @code
 *   WWDT_ClearStatusFlags(wwdt_base, kWWDT_WarningFlag);
 * @endcode
 * @param base WWDT peripheral base address
 * @param mask The status flags to clear. This is a logical OR of members of the
 *             enumeration ::_wwdt_status_flags_t
 */
void WWDT_ClearStatusFlags(WWDT_Type *base, uint32_t mask);

/*!
 * @brief Set the WWDT warning value.
 *
 * The WDWARNINT register determines the watchdog timer counter value that will generate a watchdog
 * interrupt. When the watchdog timer counter is no longer greater than the value defined by
 * WARNINT, an interrupt will be generated after the subsequent WDCLK.
 *
 * @param base         WWDT peripheral base address
 * @param warningValue WWDT warning value.
 */
static inline void WWDT_SetWarningValue(WWDT_Type *base, uint32_t warningValue)
{
    base->WARNINT = WWDT_WARNINT_WARNINT(warningValue);
}

/*!
 * @brief Set the WWDT timeout value.
 *
 * This function sets the timeout value. Every time a feed sequence occurs the value in the TC
 * register is loaded into the Watchdog timer. Writing a value below 0xFF will cause 0xFF to be
 * loaded into the TC register. Thus the minimum time-out interval is TWDCLK*256*4.
 * If enableWatchdogProtect flag is true in wwdt_config_t config structure, any attempt to change
 * the timeout value before the watchdog counter is below the warning and window values
 * will cause a watchdog reset and set the WDTOF flag.
 *
 * @param base WWDT peripheral base address
 * @param timeoutCount WWDT timeout value, count of WWDT clock tick.
 */
static inline void WWDT_SetTimeoutValue(WWDT_Type *base, uint32_t timeoutCount)
{
    base->TC = WWDT_TC_COUNT(timeoutCount);
}

/*!
 * @brief Sets the WWDT window value.
 *
 * The WINDOW register determines the highest TV value allowed when a watchdog feed is performed.
 * If a feed sequence occurs when timer value is greater than the value in WINDOW, a watchdog
 * event will occur. To disable windowing, set windowValue to 0xFFFFFF (maximum possible timer
 * value) so windowing is not in effect.
 *
 * @param base        WWDT peripheral base address
 * @param windowValue WWDT window value.
 */
static inline void WWDT_SetWindowValue(WWDT_Type *base, uint32_t windowValue)
{
    base->WINDOW = WWDT_WINDOW_WINDOW(windowValue);
}

/*!
 * @brief Refreshes the WWDT timer.
 *
 * This function feeds the WWDT.
 * This function should be called before WWDT timer is in timeout. Otherwise, a reset is asserted.
 *
 * @param base WWDT peripheral base address
 */
void WWDT_Refresh(WWDT_Type *base);

/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @}*/

#endif /* _FSL_WWDT_H_ */
