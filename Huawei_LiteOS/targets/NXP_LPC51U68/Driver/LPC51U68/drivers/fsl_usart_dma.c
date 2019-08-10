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
#include "fsl_dma.h"
#include "fsl_flexcomm.h"
#include "fsl_usart_dma.h"

/*<! Structure definition for uart_dma_handle_t. The structure is private. */
typedef struct _usart_dma_private_handle
{
    USART_Type *base;
    usart_dma_handle_t *handle;
} usart_dma_private_handle_t;

enum _usart_transfer_states
{
    kUSART_TxIdle, /* TX idle. */
    kUSART_TxBusy, /* TX busy. */
    kUSART_RxIdle, /* RX idle. */
    kUSART_RxBusy  /* RX busy. */
};

/*<! Private handle only used for internally. */
static usart_dma_private_handle_t s_dmaPrivateHandle[FSL_FEATURE_SOC_USART_COUNT];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void USART_TransferSendDMACallback(dma_handle_t *handle, void *param, bool transferDone, uint32_t intmode)
{
    assert(handle);
    assert(param);

    usart_dma_private_handle_t *usartPrivateHandle = (usart_dma_private_handle_t *)param;

    /* Disable UART TX DMA. */
    USART_EnableTxDMA(usartPrivateHandle->base, false);

    usartPrivateHandle->handle->txState = kUSART_TxIdle;

    if (usartPrivateHandle->handle->callback)
    {
        usartPrivateHandle->handle->callback(usartPrivateHandle->base, usartPrivateHandle->handle, kStatus_USART_TxIdle,
                                             usartPrivateHandle->handle->userData);
    }
}

static void USART_TransferReceiveDMACallback(dma_handle_t *handle, void *param, bool transferDone, uint32_t intmode)
{
    assert(handle);
    assert(param);

    usart_dma_private_handle_t *usartPrivateHandle = (usart_dma_private_handle_t *)param;

    /* Disable UART RX DMA. */
    USART_EnableRxDMA(usartPrivateHandle->base, false);

    usartPrivateHandle->handle->rxState = kUSART_RxIdle;

    if (usartPrivateHandle->handle->callback)
    {
        usartPrivateHandle->handle->callback(usartPrivateHandle->base, usartPrivateHandle->handle, kStatus_USART_RxIdle,
                                             usartPrivateHandle->handle->userData);
    }
}

status_t USART_TransferCreateHandleDMA(USART_Type *base,
                                       usart_dma_handle_t *handle,
                                       usart_dma_transfer_callback_t callback,
                                       void *userData,
                                       dma_handle_t *txDmaHandle,
                                       dma_handle_t *rxDmaHandle)
{
    int32_t instance = 0;

    /* check 'base' */
    assert(!(NULL == base));
    if (NULL == base)
    {
        return kStatus_InvalidArgument;
    }
    /* check 'handle' */
    assert(!(NULL == handle));
    if (NULL == handle)
    {
        return kStatus_InvalidArgument;
    }

    instance = USART_GetInstance(base);

    memset(handle, 0, sizeof(*handle));
    /* assign 'base' and 'handle' */
    s_dmaPrivateHandle[instance].base = base;
    s_dmaPrivateHandle[instance].handle = handle;

    /* set tx/rx 'idle' state */
    handle->rxState = kUSART_RxIdle;
    handle->txState = kUSART_TxIdle;

    handle->callback = callback;
    handle->userData = userData;

    handle->rxDmaHandle = rxDmaHandle;
    handle->txDmaHandle = txDmaHandle;

    /* Configure TX. */
    if (txDmaHandle)
    {
        DMA_SetCallback(txDmaHandle, USART_TransferSendDMACallback, &s_dmaPrivateHandle[instance]);
    }

    /* Configure RX. */
    if (rxDmaHandle)
    {
        DMA_SetCallback(rxDmaHandle, USART_TransferReceiveDMACallback, &s_dmaPrivateHandle[instance]);
    }

    return kStatus_Success;
}

status_t USART_TransferSendDMA(USART_Type *base, usart_dma_handle_t *handle, usart_transfer_t *xfer)
{
    assert(handle);
    assert(handle->txDmaHandle);
    assert(xfer);
    assert(xfer->data);
    assert(xfer->dataSize);

    dma_transfer_config_t xferConfig;
    status_t status;

    /* If previous TX not finished. */
    if (kUSART_TxBusy == handle->txState)
    {
        status = kStatus_USART_TxBusy;
    }
    else
    {
        handle->txState = kUSART_TxBusy;
        handle->txDataSizeAll = xfer->dataSize;

        /* Enable DMA request from txFIFO */
        USART_EnableTxDMA(base, true);

        /* Prepare transfer. */
        DMA_PrepareTransfer(&xferConfig, xfer->data, (void *)&base->FIFOWR, sizeof(uint8_t), xfer->dataSize,
                            kDMA_MemoryToPeripheral, NULL);

        /* Submit transfer. */
        DMA_SubmitTransfer(handle->txDmaHandle, &xferConfig);
        DMA_StartTransfer(handle->txDmaHandle);

        status = kStatus_Success;
    }

    return status;
}

status_t USART_TransferReceiveDMA(USART_Type *base, usart_dma_handle_t *handle, usart_transfer_t *xfer)
{
    assert(handle);
    assert(handle->rxDmaHandle);
    assert(xfer);
    assert(xfer->data);
    assert(xfer->dataSize);

    dma_transfer_config_t xferConfig;
    status_t status;

    /* If previous RX not finished. */
    if (kUSART_RxBusy == handle->rxState)
    {
        status = kStatus_USART_RxBusy;
    }
    else
    {
        handle->rxState = kUSART_RxBusy;
        handle->rxDataSizeAll = xfer->dataSize;

        /* Enable DMA request from rxFIFO */
        USART_EnableRxDMA(base, true);

        /* Prepare transfer. */
        DMA_PrepareTransfer(&xferConfig, (void *)&base->FIFORD, xfer->data, sizeof(uint8_t), xfer->dataSize,
                            kDMA_PeripheralToMemory, NULL);

        /* Submit transfer. */
        DMA_SubmitTransfer(handle->rxDmaHandle, &xferConfig);
        DMA_StartTransfer(handle->rxDmaHandle);

        status = kStatus_Success;
    }

    return status;
}

void USART_TransferAbortSendDMA(USART_Type *base, usart_dma_handle_t *handle)
{
    assert(NULL != handle);
    assert(NULL != handle->txDmaHandle);

    /* Stop transfer. */
    DMA_AbortTransfer(handle->txDmaHandle);
    handle->txState = kUSART_TxIdle;
}

void USART_TransferAbortReceiveDMA(USART_Type *base, usart_dma_handle_t *handle)
{
    assert(NULL != handle);
    assert(NULL != handle->rxDmaHandle);

    /* Stop transfer. */
    DMA_AbortTransfer(handle->rxDmaHandle);
    handle->rxState = kUSART_RxIdle;
}

status_t USART_TransferGetReceiveCountDMA(USART_Type *base, usart_dma_handle_t *handle, uint32_t *count)
{
    assert(handle);
    assert(handle->rxDmaHandle);
    assert(count);

    if (kUSART_RxIdle == handle->rxState)
    {
        return kStatus_NoTransferInProgress;
    }

    *count = handle->rxDataSizeAll - DMA_GetRemainingBytes(handle->rxDmaHandle->base, handle->rxDmaHandle->channel);

    return kStatus_Success;
}
