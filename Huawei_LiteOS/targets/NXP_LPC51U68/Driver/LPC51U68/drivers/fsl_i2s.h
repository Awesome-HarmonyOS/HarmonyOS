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
#ifndef _FSL_I2S_H_
#define _FSL_I2S_H_

#include "fsl_device_registers.h"
#include "fsl_common.h"
#include "fsl_flexcomm.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @addtogroup i2s_driver
 * @{
 */

/*! @file */

/*! @name Driver version */
/*@{*/
/*! @brief I2S driver version 2.0.0.
 *
 * Current version: 2.0.0
 *
 * Change log:
 * - Version 2.0.0
 *   - initial version
 */
#define FSL_I2S_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

#ifndef I2S_NUM_BUFFERS

/*! @brief Number of buffers . */
#define I2S_NUM_BUFFERS (4)

#endif

/*! @brief I2S status codes. */
enum _i2s_status
{
    kStatus_I2S_BufferComplete =
        MAKE_STATUS(kStatusGroup_I2S, 0),                /*!< Transfer from/into a single buffer has completed */
    kStatus_I2S_Done = MAKE_STATUS(kStatusGroup_I2S, 1), /*!< All buffers transfers have completed */
    kStatus_I2S_Busy =
        MAKE_STATUS(kStatusGroup_I2S, 2), /*!< Already performing a transfer and cannot queue another buffer */
};

/*!
 * @brief I2S flags.
 *
 * @note These enums are meant to be OR'd together to form a bit mask.
 */
typedef enum _i2s_flags
{
    kI2S_TxErrorFlag = I2S_FIFOINTENSET_TXERR_MASK, /*!< TX error interrupt */
    kI2S_TxLevelFlag = I2S_FIFOINTENSET_TXLVL_MASK, /*!< TX level interrupt */
    kI2S_RxErrorFlag = I2S_FIFOINTENSET_RXERR_MASK, /*!< RX error interrupt */
    kI2S_RxLevelFlag = I2S_FIFOINTENSET_RXLVL_MASK  /*!< RX level interrupt */
} i2s_flags_t;

/*! @brief Master / slave mode. */
typedef enum _i2s_master_slave
{
    kI2S_MasterSlaveNormalSlave = 0x0,  /*!< Normal slave */
    kI2S_MasterSlaveWsSyncMaster = 0x1, /*!< WS synchronized master */
    kI2S_MasterSlaveExtSckMaster = 0x2, /*!< Master using existing SCK */
    kI2S_MasterSlaveNormalMaster = 0x3  /*!< Normal master */
} i2s_master_slave_t;

/*! @brief I2S mode. */
typedef enum _i2s_mode
{
    kI2S_ModeI2sClassic = 0x0, /*!< I2S classic mode */
    kI2S_ModeDspWs50 = 0x1,    /*!< DSP mode, WS having 50% duty cycle */
    kI2S_ModeDspWsShort = 0x2, /*!< DSP mode, WS having one clock long pulse */
    kI2S_ModeDspWsLong = 0x3   /*!< DSP mode, WS having one data slot long pulse */
} i2s_mode_t;

/*! @brief I2S configuration structure. */
typedef struct _i2s_config
{
    i2s_master_slave_t masterSlave; /*!< Master / slave configuration */
    i2s_mode_t mode;                /*!< I2S mode */
    bool rightLow;                  /*!< Right channel data in low portion of FIFO */
    bool leftJust;                  /*!< Left justify data in FIFO */
#if defined(I2S_CFG1_PDMDATA)
    bool pdmData;                   /*!< Data source is the D-Mic subsystem */
#endif
    bool sckPol;                    /*!< SCK polarity */
    bool wsPol;                     /*!< WS polarity */
    uint16_t divider;               /*!< Flexcomm function clock divider (1 - 4096) */
    bool oneChannel;                /*!< true mono, false stereo */
    uint8_t dataLength;             /*!< Data length (4 - 32) */
    uint16_t frameLength;           /*!< Frame width (4 - 512) */
    uint16_t position;              /*!< Data position in the frame */
    uint8_t watermark;              /*!< FIFO trigger level */
    bool txEmptyZero;               /*!< Transmit zero when buffer becomes empty or last item */
    bool pack48; /*!< Packing format for 48-bit data (false - 24 bit values, true - alternating 32-bit and 16-bit
                    values) */
} i2s_config_t;

/*! @brief Buffer to transfer from or receive audio data into. */
typedef struct _i2s_transfer
{
    volatile uint8_t *data;   /*!< Pointer to data buffer. */
    volatile size_t dataSize; /*!< Buffer size in bytes. */
} i2s_transfer_t;

/*! @brief Transactional state of the intialized transfer or receive I2S operation. */
typedef struct _i2s_handle i2s_handle_t;

/*!
 * @brief Callback function invoked from transactional API
 *        on completion of a single buffer transfer.
 *
 * @param base I2S base pointer.
 * @param handle pointer to I2S transaction.
 * @param completionStatus status of the transaction.
 * @param userData optional pointer to user arguments data.
 */
typedef void (*i2s_transfer_callback_t)(I2S_Type *base,
                                        i2s_handle_t *handle,
                                        status_t completionStatus,
                                        void *userData);

/*! @brief Members not to be accessed / modified outside of the driver. */
struct _i2s_handle
{
    uint32_t state;                             /*!< State of transfer */
    i2s_transfer_callback_t completionCallback; /*!< Callback function pointer */
    void *userData;                             /*!< Application data passed to callback */
    bool oneChannel;                            /*!< true mono, false stereo */
    uint8_t dataLength;                         /*!< Data length (4 - 32) */
    bool pack48;     /*!< Packing format for 48-bit data (false - 24 bit values, true - alternating 32-bit and 16-bit
                        values) */
    bool useFifo48H; /*!< When dataLength 17-24: true use FIFOWR48H, false use FIFOWR */
    volatile i2s_transfer_t i2sQueue[I2S_NUM_BUFFERS]; /*!< Transfer queue storing transfer buffers */
    volatile uint8_t queueUser;                        /*!< Queue index where user's next transfer will be stored */
    volatile uint8_t queueDriver;                      /*!< Queue index of buffer actually used by the driver */
    volatile uint32_t errorCount;                      /*!< Number of buffer underruns/overruns */
    volatile uint32_t transferCount;                   /*!< Number of bytes transferred */
    volatile uint8_t watermark;                        /*!< FIFO trigger level */
};

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
 * @brief Initializes the FLEXCOMM peripheral for I2S transmit functionality.
 *
 * Ungates the FLEXCOMM clock and configures the module
 * for I2S transmission using a configuration structure.
 * The configuration structure can be custom filled or set with default values by
 * I2S_TxGetDefaultConfig().
 *
 * @note This API should be called at the beginning of the application to use
 * the I2S driver.
 *
 * @param base I2S base pointer.
 * @param config pointer to I2S configuration structure.
 */
void I2S_TxInit(I2S_Type *base, const i2s_config_t *config);

/*!
 * @brief Initializes the FLEXCOMM peripheral for I2S receive functionality.
 *
 * Ungates the FLEXCOMM clock and configures the module
 * for I2S receive using a configuration structure.
 * The configuration structure can be custom filled or set with default values by
 * I2S_RxGetDefaultConfig().
 *
 * @note This API should be called at the beginning of the application to use
 * the I2S driver.
 *
 * @param base I2S base pointer.
 * @param config pointer to I2S configuration structure.
 */
void I2S_RxInit(I2S_Type *base, const i2s_config_t *config);

/*!
 * @brief Sets the I2S Tx configuration structure to default values.
 *
 * This API initializes the configuration structure for use in I2S_TxInit().
 * The initialized structure can remain unchanged in I2S_TxInit(), or it can be modified
 * before calling I2S_TxInit().
 * Example:
   @code
   i2s_config_t config;
   I2S_TxGetDefaultConfig(&config);
   @endcode
 *
 * Default values:
 * @code
 *   config->masterSlave = kI2S_MasterSlaveNormalMaster;
 *   config->mode = kI2S_ModeI2sClassic;
 *   config->rightLow = false;
 *   config->leftJust = false;
 *   config->pdmData = false;
 *   config->sckPol = false;
 *   config->wsPol = false;
 *   config->divider = 1;
 *   config->oneChannel = false;
 *   config->dataLength = 16;
 *   config->frameLength = 32;
 *   config->position = 0;
 *   config->watermark = 4;
 *   config->txEmptyZero = true;
 *   config->pack48 = false;
 * @endcode
 *
 * @param config pointer to I2S configuration structure.
 */
void I2S_TxGetDefaultConfig(i2s_config_t *config);

/*!
 * @brief Sets the I2S Rx configuration structure to default values.
 *
 * This API initializes the configuration structure for use in I2S_RxInit().
 * The initialized structure can remain unchanged in I2S_RxInit(), or it can be modified
 * before calling I2S_RxInit().
 * Example:
   @code
   i2s_config_t config;
   I2S_RxGetDefaultConfig(&config);
   @endcode
 *
 * Default values:
 * @code
 *   config->masterSlave = kI2S_MasterSlaveNormalSlave;
 *   config->mode = kI2S_ModeI2sClassic;
 *   config->rightLow = false;
 *   config->leftJust = false;
 *   config->pdmData = false;
 *   config->sckPol = false;
 *   config->wsPol = false;
 *   config->divider = 1;
 *   config->oneChannel = false;
 *   config->dataLength = 16;
 *   config->frameLength = 32;
 *   config->position = 0;
 *   config->watermark = 4;
 *   config->txEmptyZero = false;
 *   config->pack48 = false;
 * @endcode
 *
 * @param config pointer to I2S configuration structure.
 */
void I2S_RxGetDefaultConfig(i2s_config_t *config);

/*!
 * @brief De-initializes the I2S peripheral.
 *
 * This API gates the FLEXCOMM clock. The I2S module can't operate unless I2S_TxInit
 * or I2S_RxInit is called to enable the clock.
 *
 * @param base I2S base pointer.
 */
void I2S_Deinit(I2S_Type *base);

/*! @} */

/*!
 * @name Non-blocking API
 * @{
 */

/*!
 * @brief Initializes handle for transfer of audio data.
 *
 * @param base I2S base pointer.
 * @param handle pointer to handle structure.
 * @param callback function to be called back when transfer is done or fails.
 * @param userData pointer to data passed to callback.
 */
void I2S_TxTransferCreateHandle(I2S_Type *base, i2s_handle_t *handle, i2s_transfer_callback_t callback, void *userData);

/*!
 * @brief Begins or queue sending of the given data.
 *
 * @param base I2S base pointer.
 * @param handle pointer to handle structure.
 * @param transfer data buffer.
 *
 * @retval kStatus_Success
 * @retval kStatus_I2S_Busy if all queue slots are occupied with unsent buffers.
 */
status_t I2S_TxTransferNonBlocking(I2S_Type *base, i2s_handle_t *handle, i2s_transfer_t transfer);

/*!
 * @brief Aborts sending of data.
 *
 * @param base I2S base pointer.
 * @param handle pointer to handle structure.
 */
void I2S_TxTransferAbort(I2S_Type *base, i2s_handle_t *handle);

/*!
 * @brief Initializes handle for reception of audio data.
 *
 * @param base I2S base pointer.
 * @param handle pointer to handle structure.
 * @param callback function to be called back when transfer is done or fails.
 * @param userData pointer to data passed to callback.
 */
void I2S_RxTransferCreateHandle(I2S_Type *base, i2s_handle_t *handle, i2s_transfer_callback_t callback, void *userData);

/*!
 * @brief Begins or queue reception of data into given buffer.
 *
 * @param base I2S base pointer.
 * @param handle pointer to handle structure.
 * @param transfer data buffer.
 *
 * @retval kStatus_Success
 * @retval kStatus_I2S_Busy if all queue slots are occupied with buffers which are not full.
 */
status_t I2S_RxTransferNonBlocking(I2S_Type *base, i2s_handle_t *handle, i2s_transfer_t transfer);

/*!
 * @brief Aborts receiving of data.
 *
 * @param base I2S base pointer.
 * @param handle pointer to handle structure.
 */
void I2S_RxTransferAbort(I2S_Type *base, i2s_handle_t *handle);

/*!
 * @brief Returns number of bytes transferred so far.
 *
 * @param base I2S base pointer.
 * @param handle pointer to handle structure.
 * @param[out] count number of bytes transferred so far by the non-blocking transaction.
 *
 * @retval kStatus_Success
 * @retval kStatus_NoTransferInProgress there is no non-blocking transaction currently in progress.
 */
status_t I2S_TransferGetCount(I2S_Type *base, i2s_handle_t *handle, size_t *count);

/*!
 * @brief Returns number of buffer underruns or overruns.
 *
 * @param base I2S base pointer.
 * @param handle pointer to handle structure.
 * @param[out] count number of transmit errors encountered so far by the non-blocking transaction.
 *
 * @retval kStatus_Success
 * @retval kStatus_NoTransferInProgress there is no non-blocking transaction currently in progress.
 */
status_t I2S_TransferGetErrorCount(I2S_Type *base, i2s_handle_t *handle, size_t *count);

/*! @} */

/*!
 * @name Enable / disable
 * @{
 */

/*!
 * @brief Enables I2S operation.
 *
 * @param base I2S base pointer.
 */
static inline void I2S_Enable(I2S_Type *base)
{
    base->CFG1 |= I2S_CFG1_MAINENABLE(1U);
}

/*!
 * @brief Disables I2S operation.
 *
 * @param base I2S base pointer.
 */
static inline void I2S_Disable(I2S_Type *base)
{
    base->CFG1 &= (~I2S_CFG1_MAINENABLE(1U));
}

/*! @} */

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enables I2S FIFO interrupts.
 *
 * @param base I2S base pointer.
 * @param interruptMask bit mask of interrupts to enable. See #i2s_flags_t for the set
 *      of constants that should be OR'd together to form the bit mask.
 */
static inline void I2S_EnableInterrupts(I2S_Type *base, uint32_t interruptMask)
{
    base->FIFOINTENSET = interruptMask;
}

/*!
 * @brief Disables I2S FIFO interrupts.
 *
 * @param base I2S base pointer.
 * @param interruptMask bit mask of interrupts to enable. See #i2s_flags_t for the set
 *      of constants that should be OR'd together to form the bit mask.
 */
static inline void I2S_DisableInterrupts(I2S_Type *base, uint32_t interruptMask)
{
    base->FIFOINTENCLR = interruptMask;
}

/*!
 * @brief Returns the set of currently enabled I2S FIFO interrupts.
 *
 * @param base I2S base pointer.
 *
 * @return A bitmask composed of #i2s_flags_t enumerators OR'd together
 *         to indicate the set of enabled interrupts.
 */
static inline uint32_t I2S_GetEnabledInterrupts(I2S_Type *base)
{
    return base->FIFOINTENSET;
}

/*!
 * @brief Invoked from interrupt handler when transmit FIFO level decreases.
 *
 * @param base I2S base pointer.
 * @param handle pointer to handle structure.
 */
void I2S_TxHandleIRQ(I2S_Type *base, i2s_handle_t *handle);

/*!
 * @brief Invoked from interrupt handler when receive FIFO level decreases.
 *
 * @param base I2S base pointer.
 * @param handle pointer to handle structure.
 */
void I2S_RxHandleIRQ(I2S_Type *base, i2s_handle_t *handle);

/*! @} */

/*! @} */

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_I2S_H_ */
