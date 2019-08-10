/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
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
 *
 * Debug console shall provide input and output functions to scan and print formatted data.
 * o Support a format specifier for PRINTF follows this prototype "%[flags][width][.precision][length]specifier"
 *   - [flags] :'-', '+', '#', ' ', '0'
 *   - [width]:  number (0,1...)
 *   - [.precision]: number (0,1...)
 *   - [length]: do not support
 *   - [specifier]: 'd', 'i', 'f', 'F', 'x', 'X', 'o', 'p', 'u', 'c', 's', 'n'
 * o Support a format specifier for SCANF follows this prototype " %[*][width][length]specifier"
 *   - [*]: is supported.
 *   - [width]: number (0,1...)
 *   - [length]: 'h', 'hh', 'l','ll','L'. ignore ('j','z','t')
 *   - [specifier]: 'd', 'i', 'u', 'f', 'F', 'e', 'E', 'g', 'G', 'a', 'A', 'o', 'c', 's'
 */

#ifndef _FSL_DEBUGCONSOLE_H_
#define _FSL_DEBUGCONSOLE_H_

#include "fsl_common.h"
/*!
 * @addtogroup debugconsole
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Definition to select sdk or toolchain printf, scanf. */
#ifndef SDK_DEBUGCONSOLE
#define SDK_DEBUGCONSOLE 1U
#endif

#if defined(SDK_DEBUGCONSOLE) && !(SDK_DEBUGCONSOLE)
#include <stdio.h>
#endif

#if SDK_DEBUGCONSOLE /* Select printf, scanf, putchar, getchar of SDK version. */
#define PRINTF DbgConsole_Printf
#define SCANF DbgConsole_Scanf
#define PUTCHAR DbgConsole_Putchar
#define GETCHAR DbgConsole_Getchar
#else /* Select printf, scanf, putchar, getchar of toolchain. */
#define PRINTF printf
#define SCANF scanf
#define PUTCHAR putchar
#define GETCHAR getchar
#endif /* SDK_DEBUGCONSOLE */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*! @name Initialization*/
/* @{ */

/*!
 * @brief Initializes the the peripheral used for debug messages.
 *
 * Call this function to enable debug log messages to be output via the specified peripheral,
 * frequency of peripheral source clock, and base address at the specified baud rate.
 * After this function has returned, stdout and stdin are connected to the selected peripheral.
 *
 * @param baseAddr      Indicates the address of the peripheral used to send debug messages.
 * @param baudRate      The desired baud rate in bits per second.
 * @param device        Low level device type for the debug console, can be one of the following.
 *                      @arg DEBUG_CONSOLE_DEVICE_TYPE_UART,
 *                      @arg DEBUG_CONSOLE_DEVICE_TYPE_LPUART,
 *                      @arg DEBUG_CONSOLE_DEVICE_TYPE_LPSCI,
 *                      @arg DEBUG_CONSOLE_DEVICE_TYPE_USBCDC.
 * @param clkSrcFreq    Frequency of peripheral source clock.
 *
 * @return              Indicates whether initialization was successful or not.
 * @retval kStatus_Success          Execution successfully
 * @retval kStatus_Fail             Execution failure
 * @retval kStatus_InvalidArgument  Invalid argument existed
 */
status_t DbgConsole_Init(uint32_t baseAddr, uint32_t baudRate, uint8_t device, uint32_t clkSrcFreq);

/*!
 * @brief De-initializes the peripheral used for debug messages.
 *
 * Call this function to disable debug log messages to be output via the specified peripheral
 * base address and at the specified baud rate.
 *
 * @return Indicates whether de-initialization was successful or not.
 */
status_t DbgConsole_Deinit(void);

#if SDK_DEBUGCONSOLE
/*!
 * @brief Writes formatted output to the standard output stream.
 *
 * Call this function to write a formatted output to the standard output stream.
 *
 * @param   fmt_s Format control string.
 * @return  Returns the number of characters printed or a negative value if an error occurs.
 */
int DbgConsole_Printf(const char *fmt_s, ...);

/*!
 * @brief Writes a character to stdout.
 *
 * Call this function to write a character to stdout.
 *
 * @param   ch Character to be written.
 * @return  Returns the character written.
 */
int DbgConsole_Putchar(int ch);

/*!
 * @brief Reads formatted data from the standard input stream.
 *
 * Call this function to read formatted data from the standard input stream.
 *
 * @param   fmt_ptr Format control string.
 * @return  Returns the number of fields successfully converted and assigned.
 */
int DbgConsole_Scanf(char *fmt_ptr, ...);

/*!
 * @brief Reads a character from standard input.
 *
 * Call this function to read a character from standard input.
 *
 * @return Returns the character read.
 */
int DbgConsole_Getchar(void);

/*!
 * @brief Debug console flush log.
 *
 * Call this function to wait the buffer empty and io idle before.
 * If interrupt transfer is using, make sure the global IRQ is enable before call this function
 * This function should be called when
 * 1, before enter power down mode
 * 2, log is required to print to terminal immediately
 * @return Indicates whether wait idle was successful or not.
 */
status_t DbgConsole_Flush(void);

#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
/*!
 * @brief Debug console try to get char
 * This function provide a api which will not block current task, if character is
 * avaliable return it , otherwise return fail.
 * @param ch the address of char to receive
 * @return Indicates get char was successful or not.
 */
status_t DbgConsole_TryGetchar(char *ch);
#endif

#endif /* SDK_DEBUGCONSOLE */

/*! @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @} */

#endif /* _FSL_DEBUGCONSOLE_H_ */
