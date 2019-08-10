/*
 * The Clear BSD License
 * Copyright 2017 NXP
 * All rights reserved.
 *
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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
 */

#ifndef _FSL_LOG_H
#define _FSL_LOG_H

#include "fsl_common.h"

/*!
 * @addtogroup debugconsole
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*************************************************************************************************
 * Prototypes
 ************************************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @brief Initializes
 *
 * Call this function to init the buffer
 * @param baseAddr, device base address
 * @param device, device type
 * @param baudRate, device communicate baudrate
 * @param clkSrcFreq, device source clock freq
 *
 * @return              	Indicates whether initialization was successful or not.
 * @retval kStatus_Success  Execution successfully
 * @retval kStatus_Fail     Execution failure
 */
status_t LOG_Init(uint32_t baseAddr, uint8_t device, uint32_t baudRate, uint32_t clkSrcFreq);

/*!
 * @brief De-Initializes
 *
 * Call this function to deinit the buffer
 *
 * @return Indicates whether Deinit was successful or not.
 */
void LOG_Deinit(void);

/*!
 * @brief log push interface
 *
 * Call this function to print log
 * @param fmt, buffer pointer
 * @param size, avaliable size
 * @return indicate the push size
 * @retval-1 indicate buffer is full or transfer fail.
 * @retval size return the push log size.
 */
int LOG_Push(uint8_t *buf, size_t size);

/*!
 * @brief log read one line function
 *
 * Call this function to print log
 * @param fmt, buffer pointer
 * @param size, avaliable size
 * @reutrn the number of the recieved character
 */
int LOG_ReadLine(uint8_t *buf, size_t size);

/*!
 * @brief log read one character function
 *
 * Call this function to GETCHAR
 * @param ch receive address
 * @reutrn the number of the recieved character
 */
int LOG_ReadCharacter(uint8_t *ch);

/*!
 * @brief wait log and io idle
 *
 * Call this function to wait log buffer empty and io idle before enter low power mode.
 * @return Indicates whether wait idle was successful or not.
 */
status_t LOG_WaitIdle(void);

#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
/*!
 * @brief log try read one character
 * Call this function to check character avaliable or not, if not return fail, otherwise return it.
 * @param ch the address of char to receive
 * @return Indicates try getchar was successful or not.
 */
status_t LOG_TryReadCharacter(uint8_t *ch);
#endif

/*!
 * @brief log pop function
 *
 * Call this function to pop log from buffer.
 * @param buf buffer address to pop
 * @param size log size to pop
 * @return pop log size.
 */
int LOG_Pop(uint8_t *buf, size_t size);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @} */

#endif
