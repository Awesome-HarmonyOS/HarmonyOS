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
#ifndef _FSL_RTC_H_
#define _FSL_RTC_H_

#include "fsl_common.h"

/*!
 * @addtogroup rtc
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_RTC_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0 */
/*@}*/

/*! @brief List of RTC interrupts */
typedef enum _rtc_interrupt_enable
{
    kRTC_AlarmInterruptEnable = RTC_CTRL_ALARMDPD_EN_MASK, /*!< Alarm interrupt.*/
    kRTC_WakeupInterruptEnable = RTC_CTRL_WAKEDPD_EN_MASK  /*!< Wake-up interrupt.*/
} rtc_interrupt_enable_t;

/*! @brief List of RTC flags */
typedef enum _rtc_status_flags
{
    kRTC_AlarmFlag = RTC_CTRL_ALARM1HZ_MASK, /*!< Alarm flag*/
    kRTC_WakeupFlag = RTC_CTRL_WAKE1KHZ_MASK /*!< 1kHz wake-up timer flag*/
} rtc_status_flags_t;

/*! @brief Structure is used to hold the date and time */
typedef struct _rtc_datetime
{
    uint16_t year;  /*!< Range from 1970 to 2099.*/
    uint8_t month;  /*!< Range from 1 to 12.*/
    uint8_t day;    /*!< Range from 1 to 31 (depending on month).*/
    uint8_t hour;   /*!< Range from 0 to 23.*/
    uint8_t minute; /*!< Range from 0 to 59.*/
    uint8_t second; /*!< Range from 0 to 59.*/
} rtc_datetime_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
 * @brief Ungates the RTC clock and enables the RTC oscillator.
 *
 * @note This API should be called at the beginning of the application using the RTC driver.
 *
 * @param base RTC peripheral base address
 */
void RTC_Init(RTC_Type *base);

/*!
 * @brief Stop the timer and gate the RTC clock
 *
 * @param base RTC peripheral base address
 */
static inline void RTC_Deinit(RTC_Type *base)
{
    /* Stop the RTC timer */
    base->CTRL &= ~RTC_CTRL_RTC_EN_MASK;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Gate the module clock */
    CLOCK_DisableClock(kCLOCK_Rtc);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

/*! @}*/

/*!
 * @name Current Time & Alarm
 * @{
 */

/*!
 * @brief Sets the RTC date and time according to the given time structure.
 *
 * The RTC counter must be stopped prior to calling this function as writes to the RTC
 * seconds register will fail if the RTC counter is running.
 *
 * @param base     RTC peripheral base address
 * @param datetime Pointer to structure where the date and time details to set are stored
 *
 * @return kStatus_Success: Success in setting the time and starting the RTC
 *         kStatus_InvalidArgument: Error because the datetime format is incorrect
 */
status_t RTC_SetDatetime(RTC_Type *base, const rtc_datetime_t *datetime);

/*!
 * @brief Gets the RTC time and stores it in the given time structure.
 *
 * @param base     RTC peripheral base address
 * @param datetime Pointer to structure where the date and time details are stored.
 */
void RTC_GetDatetime(RTC_Type *base, rtc_datetime_t *datetime);

/*!
 * @brief Sets the RTC alarm time
 *
 * The function checks whether the specified alarm time is greater than the present
 * time. If not, the function does not set the alarm and returns an error.
 *
 * @param base      RTC peripheral base address
 * @param alarmTime Pointer to structure where the alarm time is stored.
 *
 * @return kStatus_Success: success in setting the RTC alarm
 *         kStatus_InvalidArgument: Error because the alarm datetime format is incorrect
 *         kStatus_Fail: Error because the alarm time has already passed
 */
status_t RTC_SetAlarm(RTC_Type *base, const rtc_datetime_t *alarmTime);

/*!
 * @brief Returns the RTC alarm time.
 *
 * @param base     RTC peripheral base address
 * @param datetime Pointer to structure where the alarm date and time details are stored.
 */
void RTC_GetAlarm(RTC_Type *base, rtc_datetime_t *datetime);

/*! @}*/

/*!
 * @brief Enable the RTC high resolution timer and set the wake-up time.
 *
 * @param base        RTC peripheral base address
 * @param wakeupValue The value to be loaded into the RTC WAKE register
 */
static inline void RTC_SetWakeupCount(RTC_Type *base, uint16_t wakeupValue)
{
    /* Enable the 1kHz RTC timer */
    base->CTRL |= RTC_CTRL_RTC1KHZ_EN_MASK;

    /* Set the start count value into the wake-up timer */
    base->WAKE = wakeupValue;
}

/*!
 * @brief Read actual RTC counter value.
 *
 * @param base        RTC peripheral base address
 */
static inline uint16_t RTC_GetWakeupCount(RTC_Type *base)
{
    /* Read wake-up counter */
    return RTC_WAKE_VAL(base->WAKE);
}

/*!
 * @name Interrupt Interface
 * @{
 */

/*!
 * @brief Enables the selected RTC interrupts.
 *
 * @param base RTC peripheral base address
 * @param mask The interrupts to enable. This is a logical OR of members of the
 *             enumeration ::rtc_interrupt_enable_t
 */
static inline void RTC_EnableInterrupts(RTC_Type *base, uint32_t mask)
{
    uint32_t reg = base->CTRL;

    /* Clear flag bits to prevent accidentally clearing anything when writing back */
    reg &= ~(RTC_CTRL_ALARM1HZ_MASK | RTC_CTRL_WAKE1KHZ_MASK);
    reg |= mask;

    base->CTRL = reg;
}

/*!
 * @brief Disables the selected RTC interrupts.
 *
 * @param base RTC peripheral base address
 * @param mask The interrupts to enable. This is a logical OR of members of the
 *             enumeration ::rtc_interrupt_enable_t
 */
static inline void RTC_DisableInterrupts(RTC_Type *base, uint32_t mask)
{
    uint32_t reg = base->CTRL;

    /* Clear flag bits to prevent accidentally clearing anything when writing back */
    reg &= ~(RTC_CTRL_ALARM1HZ_MASK | RTC_CTRL_WAKE1KHZ_MASK | mask);

    base->CTRL = reg;
}

/*!
 * @brief Gets the enabled RTC interrupts.
 *
 * @param base RTC peripheral base address
 *
 * @return The enabled interrupts. This is the logical OR of members of the
 *         enumeration ::rtc_interrupt_enable_t
 */
static inline uint32_t RTC_GetEnabledInterrupts(RTC_Type *base)
{
    return (base->CTRL & (RTC_CTRL_ALARMDPD_EN_MASK | RTC_CTRL_WAKEDPD_EN_MASK));
}

/*! @}*/

/*!
 * @name Status Interface
 * @{
 */

/*!
 * @brief Gets the RTC status flags
 *
 * @param base RTC peripheral base address
 *
 * @return The status flags. This is the logical OR of members of the
 *         enumeration ::rtc_status_flags_t
 */
static inline uint32_t RTC_GetStatusFlags(RTC_Type *base)
{
    return (base->CTRL & (RTC_CTRL_ALARM1HZ_MASK | RTC_CTRL_WAKE1KHZ_MASK));
}

/*!
 * @brief  Clears the RTC status flags.
 *
 * @param base RTC peripheral base address
 * @param mask The status flags to clear. This is a logical OR of members of the
 *             enumeration ::rtc_status_flags_t
 */
static inline void RTC_ClearStatusFlags(RTC_Type *base, uint32_t mask)
{
    uint32_t reg = base->CTRL;

    /* Clear flag bits to prevent accidentally clearing anything when writing back */
    reg &= ~(RTC_CTRL_ALARM1HZ_MASK | RTC_CTRL_WAKE1KHZ_MASK);

    /* Write 1 to the flags we wish to clear */
    reg |= mask;

    base->CTRL = reg;
}

/*! @}*/

/*!
 * @name Timer Start and Stop
 * @{
 */

/*!
 * @brief Starts the RTC time counter.
 *
 * After calling this function, the timer counter increments once a second provided SR[TOF] or
 * SR[TIF] are not set.
 *
 * @param base RTC peripheral base address
 */
static inline void RTC_StartTimer(RTC_Type *base)
{
    base->CTRL |= RTC_CTRL_RTC_EN_MASK;
}

/*!
 * @brief Stops the RTC time counter.
 *
 * RTC's seconds register can be written to only when the timer is stopped.
 *
 * @param base RTC peripheral base address
 */
static inline void RTC_StopTimer(RTC_Type *base)
{
    base->CTRL &= ~RTC_CTRL_RTC_EN_MASK;
}

/*! @}*/

/*!
 * @brief Performs a software reset on the RTC module.
 *
 * This resets all RTC registers to their reset value. The bit is cleared by software explicitly clearing it.
 *
 * @param base RTC peripheral base address
 */
static inline void RTC_Reset(RTC_Type *base)
{
    base->CTRL |= RTC_CTRL_SWRESET_MASK;
    base->CTRL &= ~RTC_CTRL_SWRESET_MASK;
}

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_RTC_H_ */
