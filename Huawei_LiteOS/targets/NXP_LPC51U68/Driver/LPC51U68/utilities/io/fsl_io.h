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

#ifndef _FSL_IO_H
#define _FSL_IO_H

#include "fsl_common.h"

/*!
 * @addtogroup debugconsole
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief define a notify callback for IO
* @param size , transfer data size.
* @param rx, indicate a rx transfer is success.
* @param tx, indicate a tx transfer is success.
*/
typedef void (*notify)(size_t *size, bool rx, bool tx);

/*! @brief State structure storing io. */
typedef struct io_State
{
    void *ioBase;   /*!< Base of the IP register. */
    uint8_t ioType; /*!< device type */
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
    notify callBack; /*!< define the callback function for buffer */
#endif

} io_state_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @brief io init function.
 *
 * Call this function to init IO.
 *
 * @param io configuration pointer
 * @param baudRate baud rate
 * @param clkSrcFreq clock freq
 * @param ringbuffer used to receive character
 */
void IO_Init(io_state_t *io, uint32_t baudRate, uint32_t clkSrcFreq, uint8_t *ringBuffer);

/*!
 * @brief Deinit IO.
 *
 * Call this function to Deinit IO.
 *
 * @return deinit status
 */
status_t IO_Deinit(void);

/*!
 * @brief io transfer function.
 *
 * Call this function to transfer log.
 * Print log:
 * @code
 * IO_Transfer(ch, size, true);
 * @endcode
 * Scanf log:
 * @code
 * IO_Transfer(ch, size, false);
 * @endcode
 *
 * @param   ch  transfer buffer pointer
 * @param	size transfer size
 * @param   tx indicate the transfer is TX or RX
 */
status_t IO_Transfer(uint8_t *ch, size_t size, bool tx);

/*!
 * @brief io wait idle.
 *
 * Call this function to wait the io idle
 *
 * @return Indicates whether wait idle was successful or not.
 */
status_t IO_WaitIdle(void);

#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
/*!
 * @brief io try to receive one character.
 *
 * Call this function try to receive character
 * @param ch the address of char to receive
 * @return Indicates try getchar was successful or not.
 */
status_t IO_TryReceiveCharacter(uint8_t *ch);
#endif

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @} */

#endif /* _FSL_IO_H */
