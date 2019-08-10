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

#include "fsl_rtc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SECONDS_IN_A_DAY (86400U)
#define SECONDS_IN_A_HOUR (3600U)
#define SECONDS_IN_A_MINUTE (60U)
#define DAYS_IN_A_YEAR (365U)
#define YEAR_RANGE_START (1970U)
#define YEAR_RANGE_END (2099U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Checks whether the date and time passed in is valid
 *
 * @param datetime Pointer to structure where the date and time details are stored
 *
 * @return Returns false if the date & time details are out of range; true if in range
 */
static bool RTC_CheckDatetimeFormat(const rtc_datetime_t *datetime);

/*!
 * @brief Converts time data from datetime to seconds
 *
 * @param datetime Pointer to datetime structure where the date and time details are stored
 *
 * @return The result of the conversion in seconds
 */
static uint32_t RTC_ConvertDatetimeToSeconds(const rtc_datetime_t *datetime);

/*!
 * @brief Converts time data from seconds to a datetime structure
 *
 * @param seconds  Seconds value that needs to be converted to datetime format
 * @param datetime Pointer to the datetime structure where the result of the conversion is stored
 */
static void RTC_ConvertSecondsToDatetime(uint32_t seconds, rtc_datetime_t *datetime);

/*******************************************************************************
 * Code
 ******************************************************************************/
static bool RTC_CheckDatetimeFormat(const rtc_datetime_t *datetime)
{
    assert(datetime);

    /* Table of days in a month for a non leap year. First entry in the table is not used,
     * valid months start from 1
     */
    uint8_t daysPerMonth[] = {0U, 31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};

    /* Check year, month, hour, minute, seconds */
    if ((datetime->year < YEAR_RANGE_START) || (datetime->year > YEAR_RANGE_END) || (datetime->month > 12U) ||
        (datetime->month < 1U) || (datetime->hour >= 24U) || (datetime->minute >= 60U) || (datetime->second >= 60U))
    {
        /* If not correct then error*/
        return false;
    }

    /* Adjust the days in February for a leap year */
    if ((((datetime->year & 3U) == 0) && (datetime->year % 100 != 0)) || (datetime->year % 400 == 0))
    {
        daysPerMonth[2] = 29U;
    }

    /* Check the validity of the day */
    if ((datetime->day > daysPerMonth[datetime->month]) || (datetime->day < 1U))
    {
        return false;
    }

    return true;
}

static uint32_t RTC_ConvertDatetimeToSeconds(const rtc_datetime_t *datetime)
{
    assert(datetime);

    /* Number of days from begin of the non Leap-year*/
    /* Number of days from begin of the non Leap-year*/
    uint16_t monthDays[] = {0U, 0U, 31U, 59U, 90U, 120U, 151U, 181U, 212U, 243U, 273U, 304U, 334U};
    uint32_t seconds;

    /* Compute number of days from 1970 till given year*/
    seconds = (datetime->year - 1970U) * DAYS_IN_A_YEAR;
    /* Add leap year days */
    seconds += ((datetime->year / 4) - (1970U / 4));
    /* Add number of days till given month*/
    seconds += monthDays[datetime->month];
    /* Add days in given month. We subtract the current day as it is
     * represented in the hours, minutes and seconds field*/
    seconds += (datetime->day - 1);
    /* For leap year if month less than or equal to Febraury, decrement day counter*/
    if ((!(datetime->year & 3U)) && (datetime->month <= 2U))
    {
        seconds--;
    }

    seconds = (seconds * SECONDS_IN_A_DAY) + (datetime->hour * SECONDS_IN_A_HOUR) +
              (datetime->minute * SECONDS_IN_A_MINUTE) + datetime->second;

    return seconds;
}

static void RTC_ConvertSecondsToDatetime(uint32_t seconds, rtc_datetime_t *datetime)
{
    assert(datetime);

    uint32_t x;
    uint32_t secondsRemaining, days;
    uint16_t daysInYear;
    /* Table of days in a month for a non leap year. First entry in the table is not used,
     * valid months start from 1
     */
    uint8_t daysPerMonth[] = {0U, 31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};

    /* Start with the seconds value that is passed in to be converted to date time format */
    secondsRemaining = seconds;

    /* Calcuate the number of days, we add 1 for the current day which is represented in the
     * hours and seconds field
     */
    days = secondsRemaining / SECONDS_IN_A_DAY + 1;

    /* Update seconds left*/
    secondsRemaining = secondsRemaining % SECONDS_IN_A_DAY;

    /* Calculate the datetime hour, minute and second fields */
    datetime->hour = secondsRemaining / SECONDS_IN_A_HOUR;
    secondsRemaining = secondsRemaining % SECONDS_IN_A_HOUR;
    datetime->minute = secondsRemaining / 60U;
    datetime->second = secondsRemaining % SECONDS_IN_A_MINUTE;

    /* Calculate year */
    daysInYear = DAYS_IN_A_YEAR;
    datetime->year = YEAR_RANGE_START;
    while (days > daysInYear)
    {
        /* Decrease day count by a year and increment year by 1 */
        days -= daysInYear;
        datetime->year++;

        /* Adjust the number of days for a leap year */
        if (datetime->year & 3U)
        {
            daysInYear = DAYS_IN_A_YEAR;
        }
        else
        {
            daysInYear = DAYS_IN_A_YEAR + 1;
        }
    }

    /* Adjust the days in February for a leap year */
    if (!(datetime->year & 3U))
    {
        daysPerMonth[2] = 29U;
    }

    for (x = 1U; x <= 12U; x++)
    {
        if (days <= daysPerMonth[x])
        {
            datetime->month = x;
            break;
        }
        else
        {
            days -= daysPerMonth[x];
        }
    }

    datetime->day = days;
}

void RTC_Init(RTC_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable the RTC peripheral clock */
    CLOCK_EnableClock(kCLOCK_Rtc);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    /* Make sure the reset bit is cleared */
    base->CTRL &= ~RTC_CTRL_SWRESET_MASK;

#if !(defined(FSL_FEATURE_RTC_HAS_NO_OSC_PD) && FSL_FEATURE_RTC_HAS_NO_OSC_PD)
    /* Make sure the RTC OSC is powered up */
    base->CTRL &= ~RTC_CTRL_RTC_OSC_PD_MASK;
#endif
}

status_t RTC_SetDatetime(RTC_Type *base, const rtc_datetime_t *datetime)
{
    assert(datetime);

    /* Return error if the time provided is not valid */
    if (!(RTC_CheckDatetimeFormat(datetime)))
    {
        return kStatus_InvalidArgument;
    }

    /* Set time in seconds */
    base->COUNT = RTC_ConvertDatetimeToSeconds(datetime);

    return kStatus_Success;
}

void RTC_GetDatetime(RTC_Type *base, rtc_datetime_t *datetime)
{
    assert(datetime);

    uint32_t seconds = 0;

    seconds = base->COUNT;
    RTC_ConvertSecondsToDatetime(seconds, datetime);
}

status_t RTC_SetAlarm(RTC_Type *base, const rtc_datetime_t *alarmTime)
{
    assert(alarmTime);

    uint32_t alarmSeconds = 0;
    uint32_t currSeconds = 0;

    /* Return error if the alarm time provided is not valid */
    if (!(RTC_CheckDatetimeFormat(alarmTime)))
    {
        return kStatus_InvalidArgument;
    }

    alarmSeconds = RTC_ConvertDatetimeToSeconds(alarmTime);

    /* Get the current time */
    currSeconds = base->COUNT;

    /* Return error if the alarm time has passed */
    if (alarmSeconds < currSeconds)
    {
        return kStatus_Fail;
    }

    /* Set alarm in seconds*/
    base->MATCH = alarmSeconds;

    return kStatus_Success;
}

void RTC_GetAlarm(RTC_Type *base, rtc_datetime_t *datetime)
{
    assert(datetime);

    uint32_t alarmSeconds = 0;

    /* Get alarm in seconds  */
    alarmSeconds = base->MATCH;

    RTC_ConvertSecondsToDatetime(alarmSeconds, datetime);
}
