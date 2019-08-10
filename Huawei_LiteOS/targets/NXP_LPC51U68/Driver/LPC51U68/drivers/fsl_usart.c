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

#include "fsl_usart.h"
#include "fsl_device_registers.h"
#include "fsl_flexcomm.h"

enum _usart_transfer_states
{
    kUSART_TxIdle, /* TX idle. */
    kUSART_TxBusy, /* TX busy. */
    kUSART_RxIdle, /* RX idle. */
    kUSART_RxBusy  /* RX busy. */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief IRQ name array */
static const IRQn_Type s_usartIRQ[] = USART_IRQS;

/*! @brief Array to map USART instance number to base address. */
static const uint32_t s_usartBaseAddrs[FSL_FEATURE_SOC_USART_COUNT] = USART_BASE_ADDRS;

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Get the index corresponding to the USART */
uint32_t USART_GetInstance(USART_Type *base)
{
    int i;

    for (i = 0; i < FSL_FEATURE_SOC_USART_COUNT; i++)
    {
        if ((uint32_t)base == s_usartBaseAddrs[i])
        {
            return i;
        }
    }

    assert(false);
    return 0;
}

size_t USART_TransferGetRxRingBufferLength(usart_handle_t *handle)
{
    size_t size;

    /* Check arguments */
    assert(NULL != handle);

    if (handle->rxRingBufferTail > handle->rxRingBufferHead)
    {
        size = (size_t)(handle->rxRingBufferHead + handle->rxRingBufferSize - handle->rxRingBufferTail);
    }
    else
    {
        size = (size_t)(handle->rxRingBufferHead - handle->rxRingBufferTail);
    }
    return size;
}

static bool USART_TransferIsRxRingBufferFull(usart_handle_t *handle)
{
    bool full;

    /* Check arguments */
    assert(NULL != handle);

    if (USART_TransferGetRxRingBufferLength(handle) == (handle->rxRingBufferSize - 1U))
    {
        full = true;
    }
    else
    {
        full = false;
    }
    return full;
}

void USART_TransferStartRingBuffer(USART_Type *base, usart_handle_t *handle, uint8_t *ringBuffer, size_t ringBufferSize)
{
    /* Check arguments */
    assert(NULL != base);
    assert(NULL != handle);
    assert(NULL != ringBuffer);

    /* Setup the ringbuffer address */
    handle->rxRingBuffer = ringBuffer;
    handle->rxRingBufferSize = ringBufferSize;
    handle->rxRingBufferHead = 0U;
    handle->rxRingBufferTail = 0U;
    /* ring buffer is ready we can start receiving data */
    base->FIFOINTENSET |= USART_FIFOINTENSET_RXLVL_MASK | USART_FIFOINTENSET_RXERR_MASK;
}

void USART_TransferStopRingBuffer(USART_Type *base, usart_handle_t *handle)
{
    /* Check arguments */
    assert(NULL != base);
    assert(NULL != handle);

    if (handle->rxState == kUSART_RxIdle)
    {
        base->FIFOINTENCLR = USART_FIFOINTENCLR_RXLVL_MASK | USART_FIFOINTENCLR_RXERR_MASK;
    }
    handle->rxRingBuffer = NULL;
    handle->rxRingBufferSize = 0U;
    handle->rxRingBufferHead = 0U;
    handle->rxRingBufferTail = 0U;
}

status_t USART_Init(USART_Type *base, const usart_config_t *config, uint32_t srcClock_Hz)
{
    int result;

    /* check arguments */
    assert(!((NULL == base) || (NULL == config) || (0 == srcClock_Hz)));
    if ((NULL == base) || (NULL == config) || (0 == srcClock_Hz))
    {
        return kStatus_InvalidArgument;
    }

    /* initialize flexcomm to USART mode */
    result = FLEXCOMM_Init(base, FLEXCOMM_PERIPH_USART);
    if (kStatus_Success != result)
    {
        return result;
    }

    /* setup baudrate */
    result = USART_SetBaudRate(base, config->baudRate_Bps, srcClock_Hz);
    if (kStatus_Success != result)
    {
        return result;
    }

    if (config->enableTx)
    {
        /* empty and enable txFIFO */
        base->FIFOCFG |= USART_FIFOCFG_EMPTYTX_MASK | USART_FIFOCFG_ENABLETX_MASK;
        /* setup trigger level */
        base->FIFOTRIG &= ~(USART_FIFOTRIG_TXLVL_MASK);
        base->FIFOTRIG |= USART_FIFOTRIG_TXLVL(config->txWatermark);
        /* enable trigger interrupt */
        base->FIFOTRIG |= USART_FIFOTRIG_TXLVLENA_MASK;
    }

    /* empty and enable rxFIFO */
    if (config->enableRx)
    {
        base->FIFOCFG |= USART_FIFOCFG_EMPTYRX_MASK | USART_FIFOCFG_ENABLERX_MASK;
        /* setup trigger level */
        base->FIFOTRIG &= ~(USART_FIFOTRIG_RXLVL_MASK);
        base->FIFOTRIG |= USART_FIFOTRIG_RXLVL(config->rxWatermark);
        /* enable trigger interrupt */
        base->FIFOTRIG |= USART_FIFOTRIG_RXLVLENA_MASK;
    }
    /* setup configuration and enable USART */
    base->CFG = USART_CFG_PARITYSEL(config->parityMode) | USART_CFG_STOPLEN(config->stopBitCount) |
                USART_CFG_DATALEN(config->bitCountPerChar) | USART_CFG_LOOP(config->loopback) | USART_CFG_ENABLE_MASK;
    return kStatus_Success;
}

void USART_Deinit(USART_Type *base)
{
    /* Check arguments */
    assert(NULL != base);
    while (!(base->STAT & USART_STAT_TXIDLE_MASK))
    {
    }
    /* Disable interrupts, disable dma requests, disable peripheral */
    base->FIFOINTENCLR = USART_FIFOINTENCLR_TXERR_MASK | USART_FIFOINTENCLR_RXERR_MASK | USART_FIFOINTENCLR_TXLVL_MASK |
                         USART_FIFOINTENCLR_RXLVL_MASK;
    base->FIFOCFG &= ~(USART_FIFOCFG_DMATX_MASK | USART_FIFOCFG_DMARX_MASK);
    base->CFG &= ~(USART_CFG_ENABLE_MASK);
}

void USART_GetDefaultConfig(usart_config_t *config)
{
    /* Check arguments */
    assert(NULL != config);

    /* Set always all members ! */
    config->baudRate_Bps = 115200U;
    config->parityMode = kUSART_ParityDisabled;
    config->stopBitCount = kUSART_OneStopBit;
    config->bitCountPerChar = kUSART_8BitsPerChar;
    config->loopback = false;
    config->enableRx = false;
    config->enableTx = false;
    config->txWatermark = kUSART_TxFifo0;
    config->rxWatermark = kUSART_RxFifo1;
}

status_t USART_SetBaudRate(USART_Type *base, uint32_t baudrate_Bps, uint32_t srcClock_Hz)
{
    uint32_t best_diff = (uint32_t)-1, best_osrval = 0xf, best_brgval = (uint32_t)-1;
    uint32_t osrval, brgval, diff, baudrate;

    /* check arguments */
    assert(!((NULL == base) || (0 == baudrate_Bps) || (0 == srcClock_Hz)));
    if ((NULL == base) || (0 == baudrate_Bps) || (0 == srcClock_Hz))
    {
        return kStatus_InvalidArgument;
    }

    /*
     * Smaller values of OSR can make the sampling position within a data bit less accurate and may
     * potentially cause more noise errors or incorrect data.
     */
    for (osrval = best_osrval; osrval >= 8; osrval--)
    {
        brgval = (srcClock_Hz / ((osrval + 1) * baudrate_Bps)) - 1;
        if (brgval > 0xFFFF)
        {
            continue;
        }
        baudrate = srcClock_Hz / ((osrval + 1) * (brgval + 1));
        diff = baudrate_Bps < baudrate ? baudrate - baudrate_Bps : baudrate_Bps - baudrate;
        if (diff < best_diff)
        {
            best_diff = diff;
            best_osrval = osrval;
            best_brgval = brgval;
        }
    }

    /* value over range */
    if (best_brgval > 0xFFFF)
    {
        return kStatus_USART_BaudrateNotSupport;
    }

    base->OSR = best_osrval;
    base->BRG = best_brgval;
    return kStatus_Success;
}

void USART_WriteBlocking(USART_Type *base, const uint8_t *data, size_t length)
{
    /* Check arguments */
    assert(!((NULL == base) || (NULL == data)));
    if ((NULL == base) || (NULL == data))
    {
        return;
    }
    /* Check whether txFIFO is enabled */
    if (!(base->FIFOCFG & USART_FIFOCFG_ENABLETX_MASK))
    {
        return;
    }
    for (; length > 0; length--)
    {
        /* Loop until txFIFO get some space for new data */
        while (!(base->FIFOSTAT & USART_FIFOSTAT_TXNOTFULL_MASK))
        {
        }
        base->FIFOWR = *data;
        data++;
    }
    /* Wait to finish transfer */
    while (!(base->STAT & USART_STAT_TXIDLE_MASK))
    {
    }
}

status_t USART_ReadBlocking(USART_Type *base, uint8_t *data, size_t length)
{
    uint32_t status;

    /* check arguments */
    assert(!((NULL == base) || (NULL == data)));
    if ((NULL == base) || (NULL == data))
    {
        return kStatus_InvalidArgument;
    }

    /* Check whether rxFIFO is enabled */
    if (!(base->FIFOCFG & USART_FIFOCFG_ENABLERX_MASK))
    {
        return kStatus_Fail;
    }
    for (; length > 0; length--)
    {
        /* loop until rxFIFO have some data to read */
        while (!(base->FIFOSTAT & USART_FIFOSTAT_RXNOTEMPTY_MASK))
        {
        }
        /* check receive status */
        status = base->STAT;
        if (status & USART_STAT_FRAMERRINT_MASK)
        {
            base->STAT |= USART_STAT_FRAMERRINT_MASK;
            return kStatus_USART_FramingError;
        }
        if (status & USART_STAT_PARITYERRINT_MASK)
        {
            base->STAT |= USART_STAT_PARITYERRINT_MASK;
            return kStatus_USART_ParityError;
        }
        if (status & USART_STAT_RXNOISEINT_MASK)
        {
            base->STAT |= USART_STAT_RXNOISEINT_MASK;
            return kStatus_USART_NoiseError;
        }
        /* check rxFIFO status */
        if (base->FIFOSTAT & USART_FIFOSTAT_RXERR_MASK)
        {
            base->FIFOCFG |= USART_FIFOCFG_EMPTYRX_MASK;
            base->FIFOSTAT |= USART_FIFOSTAT_RXERR_MASK;
            return kStatus_USART_RxError;
        }

        *data = base->FIFORD;
        data++;
    }
    return kStatus_Success;
}

status_t USART_TransferCreateHandle(USART_Type *base,
                                    usart_handle_t *handle,
                                    usart_transfer_callback_t callback,
                                    void *userData)
{
    int32_t instance = 0;

    /* Check 'base' */
    assert(!((NULL == base) || (NULL == handle)));
    if ((NULL == base) || (NULL == handle))
    {
        return kStatus_InvalidArgument;
    }

    instance = USART_GetInstance(base);

    memset(handle, 0, sizeof(*handle));
    /* Set the TX/RX state. */
    handle->rxState = kUSART_RxIdle;
    handle->txState = kUSART_TxIdle;
    /* Set the callback and user data. */
    handle->callback = callback;
    handle->userData = userData;
    handle->rxWatermark = (usart_rxfifo_watermark_t)USART_FIFOTRIG_RXLVL_GET(base);
    handle->txWatermark = (usart_txfifo_watermark_t)USART_FIFOTRIG_TXLVL_GET(base);

    FLEXCOMM_SetIRQHandler(base, (flexcomm_irq_handler_t)(uintptr_t)USART_TransferHandleIRQ, handle);

    /* Enable interrupt in NVIC. */
    EnableIRQ(s_usartIRQ[instance]);

    return kStatus_Success;
}

status_t USART_TransferSendNonBlocking(USART_Type *base, usart_handle_t *handle, usart_transfer_t *xfer)
{
    /* Check arguments */
    assert(!((NULL == base) || (NULL == handle) || (NULL == xfer)));
    if ((NULL == base) || (NULL == handle) || (NULL == xfer))
    {
        return kStatus_InvalidArgument;
    }
    /* Check xfer members */
    assert(!((0 == xfer->dataSize) || (NULL == xfer->data)));
    if ((0 == xfer->dataSize) || (NULL == xfer->data))
    {
        return kStatus_InvalidArgument;
    }

    /* Return error if current TX busy. */
    if (kUSART_TxBusy == handle->txState)
    {
        return kStatus_USART_TxBusy;
    }
    else
    {
        handle->txData = xfer->data;
        handle->txDataSize = xfer->dataSize;
        handle->txDataSizeAll = xfer->dataSize;
        handle->txState = kUSART_TxBusy;
        /* Enable transmiter interrupt. */
        base->FIFOINTENSET |= USART_FIFOINTENSET_TXLVL_MASK;
    }
    return kStatus_Success;
}

void USART_TransferAbortSend(USART_Type *base, usart_handle_t *handle)
{
    assert(NULL != handle);

    /* Disable interrupts */
    base->FIFOINTENSET &= ~USART_FIFOINTENSET_TXLVL_MASK;
    /* Empty txFIFO */
    base->FIFOCFG |= USART_FIFOCFG_EMPTYTX_MASK;

    handle->txDataSize = 0;
    handle->txState = kUSART_TxIdle;
}

status_t USART_TransferGetSendCount(USART_Type *base, usart_handle_t *handle, uint32_t *count)
{
    assert(NULL != handle);
    assert(NULL != count);

    if (kUSART_TxIdle == handle->txState)
    {
        return kStatus_NoTransferInProgress;
    }

    *count = handle->txDataSizeAll - handle->txDataSize;

    return kStatus_Success;
}

status_t USART_TransferReceiveNonBlocking(USART_Type *base,
                                          usart_handle_t *handle,
                                          usart_transfer_t *xfer,
                                          size_t *receivedBytes)
{
    uint32_t i;
    /* How many bytes to copy from ring buffer to user memory. */
    size_t bytesToCopy = 0U;
    /* How many bytes to receive. */
    size_t bytesToReceive;
    /* How many bytes currently have received. */
    size_t bytesCurrentReceived;
    uint32_t regPrimask = 0U;

    /* Check arguments */
    assert(!((NULL == base) || (NULL == handle) || (NULL == xfer)));
    if ((NULL == base) || (NULL == handle) || (NULL == xfer))
    {
        return kStatus_InvalidArgument;
    }
    /* Check xfer members */
    assert(!((0 == xfer->dataSize) || (NULL == xfer->data)));
    if ((0 == xfer->dataSize) || (NULL == xfer->data))
    {
        return kStatus_InvalidArgument;
    }

    /* How to get data:
       1. If RX ring buffer is not enabled, then save xfer->data and xfer->dataSize
          to uart handle, enable interrupt to store received data to xfer->data. When
          all data received, trigger callback.
       2. If RX ring buffer is enabled and not empty, get data from ring buffer first.
          If there are enough data in ring buffer, copy them to xfer->data and return.
          If there are not enough data in ring buffer, copy all of them to xfer->data,
          save the xfer->data remained empty space to uart handle, receive data
          to this empty space and trigger callback when finished. */
    if (kUSART_RxBusy == handle->rxState)
    {
        return kStatus_USART_RxBusy;
    }
    else
    {
        bytesToReceive = xfer->dataSize;
        bytesCurrentReceived = 0U;
        /* If RX ring buffer is used. */
        if (handle->rxRingBuffer)
        {
            /* Disable IRQ, protect ring buffer. */
            regPrimask = DisableGlobalIRQ();
            /* How many bytes in RX ring buffer currently. */
            bytesToCopy = USART_TransferGetRxRingBufferLength(handle);
            if (bytesToCopy)
            {
                bytesToCopy = MIN(bytesToReceive, bytesToCopy);
                bytesToReceive -= bytesToCopy;
                /* Copy data from ring buffer to user memory. */
                for (i = 0U; i < bytesToCopy; i++)
                {
                    xfer->data[bytesCurrentReceived++] = handle->rxRingBuffer[handle->rxRingBufferTail];
                    /* Wrap to 0. Not use modulo (%) because it might be large and slow. */
                    if (handle->rxRingBufferTail + 1U == handle->rxRingBufferSize)
                    {
                        handle->rxRingBufferTail = 0U;
                    }
                    else
                    {
                        handle->rxRingBufferTail++;
                    }
                }
            }
            /* If ring buffer does not have enough data, still need to read more data. */
            if (bytesToReceive)
            {
                /* No data in ring buffer, save the request to UART handle. */
                handle->rxData = xfer->data + bytesCurrentReceived;
                handle->rxDataSize = bytesToReceive;
                handle->rxDataSizeAll = bytesToReceive;
                handle->rxState = kUSART_RxBusy;
            }
            /* Enable IRQ if previously enabled. */
            EnableGlobalIRQ(regPrimask);
            /* Call user callback since all data are received. */
            if (0 == bytesToReceive)
            {
                if (handle->callback)
                {
                    handle->callback(base, handle, kStatus_USART_RxIdle, handle->userData);
                }
            }
        }
        /* Ring buffer not used. */
        else
        {
            handle->rxData = xfer->data + bytesCurrentReceived;
            handle->rxDataSize = bytesToReceive;
            handle->rxDataSizeAll = bytesToReceive;
            handle->rxState = kUSART_RxBusy;

            /* Enable RX interrupt. */
            base->FIFOINTENSET |= USART_FIFOINTENSET_RXLVL_MASK;
        }
        /* Return the how many bytes have read. */
        if (receivedBytes)
        {
            *receivedBytes = bytesCurrentReceived;
        }
    }
    return kStatus_Success;
}

void USART_TransferAbortReceive(USART_Type *base, usart_handle_t *handle)
{
    assert(NULL != handle);

    /* Only abort the receive to handle->rxData, the RX ring buffer is still working. */
    if (!handle->rxRingBuffer)
    {
        /* Disable interrupts */
        base->FIFOINTENSET &= ~USART_FIFOINTENSET_RXLVL_MASK;
        /* Empty rxFIFO */
        base->FIFOCFG |= USART_FIFOCFG_EMPTYRX_MASK;
    }

    handle->rxDataSize = 0U;
    handle->rxState = kUSART_RxIdle;
}

status_t USART_TransferGetReceiveCount(USART_Type *base, usart_handle_t *handle, uint32_t *count)
{
    assert(NULL != handle);
    assert(NULL != count);

    if (kUSART_RxIdle == handle->rxState)
    {
        return kStatus_NoTransferInProgress;
    }

    *count = handle->rxDataSizeAll - handle->rxDataSize;

    return kStatus_Success;
}

void USART_TransferHandleIRQ(USART_Type *base, usart_handle_t *handle)
{
    /* Check arguments */
    assert((NULL != base) && (NULL != handle));

    bool receiveEnabled = (handle->rxDataSize) || (handle->rxRingBuffer);
    bool sendEnabled = handle->txDataSize;

    /* If RX overrun. */
    if (base->FIFOSTAT & USART_FIFOSTAT_RXERR_MASK)
    {
        /* Clear rx error state. */
        base->FIFOSTAT |= USART_FIFOSTAT_RXERR_MASK;
        /* clear rxFIFO */
        base->FIFOCFG |= USART_FIFOCFG_EMPTYRX_MASK;
        /* Trigger callback. */
        if (handle->callback)
        {
            handle->callback(base, handle, kStatus_USART_RxError, handle->userData);
        }
    }
    while ((receiveEnabled && (base->FIFOSTAT & USART_FIFOSTAT_RXNOTEMPTY_MASK)) ||
           (sendEnabled && (base->FIFOSTAT & USART_FIFOSTAT_TXNOTFULL_MASK)))
    {
        /* Receive data */
        if (receiveEnabled && (base->FIFOSTAT & USART_FIFOSTAT_RXNOTEMPTY_MASK))
        {
            /* Receive to app bufffer if app buffer is present */
            if (handle->rxDataSize)
            {
                *handle->rxData = base->FIFORD;
                handle->rxDataSize--;
                handle->rxData++;
                receiveEnabled = ((handle->rxDataSize != 0) || (handle->rxRingBuffer));
                if (!handle->rxDataSize)
                {
                    if (!handle->rxRingBuffer)
                    {
                        base->FIFOINTENCLR = USART_FIFOINTENCLR_RXLVL_MASK | USART_FIFOINTENSET_RXERR_MASK;
                    }
                    handle->rxState = kUSART_RxIdle;
                    if (handle->callback)
                    {
                        handle->callback(base, handle, kStatus_USART_RxIdle, handle->userData);
                    }
                }
            }
            /* Otherwise receive to ring buffer if ring buffer is present */
            else
            {
                if (handle->rxRingBuffer)
                {
                    /* If RX ring buffer is full, trigger callback to notify over run. */
                    if (USART_TransferIsRxRingBufferFull(handle))
                    {
                        if (handle->callback)
                        {
                            handle->callback(base, handle, kStatus_USART_RxRingBufferOverrun, handle->userData);
                        }
                    }
                    /* If ring buffer is still full after callback function, the oldest data is overrided. */
                    if (USART_TransferIsRxRingBufferFull(handle))
                    {
                        /* Increase handle->rxRingBufferTail to make room for new data. */
                        if (handle->rxRingBufferTail + 1U == handle->rxRingBufferSize)
                        {
                            handle->rxRingBufferTail = 0U;
                        }
                        else
                        {
                            handle->rxRingBufferTail++;
                        }
                    }
                    /* Read data. */
                    handle->rxRingBuffer[handle->rxRingBufferHead] = base->FIFORD;
                    /* Increase handle->rxRingBufferHead. */
                    if (handle->rxRingBufferHead + 1U == handle->rxRingBufferSize)
                    {
                        handle->rxRingBufferHead = 0U;
                    }
                    else
                    {
                        handle->rxRingBufferHead++;
                    }
                }
            }
        }
        /* Send data */
        if (sendEnabled && (base->FIFOSTAT & USART_FIFOSTAT_TXNOTFULL_MASK))
        {
            base->FIFOWR = *handle->txData;
            handle->txDataSize--;
            handle->txData++;
            sendEnabled = handle->txDataSize != 0;
            if (!sendEnabled)
            {
                base->FIFOINTENCLR = USART_FIFOINTENCLR_TXLVL_MASK;
                handle->txState = kUSART_TxIdle;
                if (handle->callback)
                {
                    handle->callback(base, handle, kStatus_USART_TxIdle, handle->userData);
                }
            }
        }
    }

    /* ring buffer is not used */
    if (NULL == handle->rxRingBuffer)
    {
        /* restore if rx transfer ends and rxLevel is different from default value */
        if ((handle->rxDataSize == 0) && (USART_FIFOTRIG_RXLVL_GET(base) != handle->rxWatermark))
        {
            base->FIFOTRIG =
                (base->FIFOTRIG & (~USART_FIFOTRIG_RXLVL_MASK)) | USART_FIFOTRIG_RXLVL(handle->rxWatermark);
        }
        /* decrease level if rx transfer is bellow */
        if ((handle->rxDataSize != 0) && (handle->rxDataSize < (USART_FIFOTRIG_RXLVL_GET(base) + 1)))
        {
            base->FIFOTRIG =
                (base->FIFOTRIG & (~USART_FIFOTRIG_RXLVL_MASK)) | (USART_FIFOTRIG_RXLVL(handle->rxDataSize - 1));
        }
    }
}
