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

#include "fsl_dma.h"
#include "fsl_i2s_dma.h"
#include "fsl_flexcomm.h"
#include <string.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define DMA_MAX_TRANSFER_BYTES (DMA_MAX_TRANSFER_COUNT * sizeof(uint32_t))
#define DMA_DESCRIPTORS (2U)

/*<! @brief Structure for statically allocated private data. */
typedef struct _i2s_dma_private_handle
{
    I2S_Type *base;           /*!< I2S base address */
    i2s_dma_handle_t *handle; /*!< I2S handle */

    volatile uint16_t enqueuedBytes[DMA_DESCRIPTORS]; /*!< Number of bytes being transferred by DMA descriptors */
    volatile uint8_t enqueuedBytesStart;              /*!< First item in enqueuedBytes (for reading) */
    volatile uint8_t enqueuedBytesEnd;                /*!< Last item in enqueuedBytes (for adding) */

    volatile uint8_t
        dmaDescriptorsUsed; /*!< Number of DMA descriptors with valid data (in queue, excluding initial descriptor) */
    volatile uint8_t
        descriptor; /*!< Index of next DMA descriptor in s_DmaDescriptors to be configured with data (does not include
                       I2S instance offset) */

    volatile uint8_t queueDescriptor;                         /*!< Queue index of buffer to be actually consumed by DMA
                                                                * (queueUser - advanced when user adds a buffer,
                                                                *  queueDescriptor - advanced when user buffer queued to DMA,
                                                                *  queueDriver - advanced when DMA queued buffer sent out to I2S) */
    volatile i2s_transfer_t descriptorQueue[I2S_NUM_BUFFERS]; /*!< Transfer data to be queued to DMA */

    volatile bool intA; /*!< If next scheduled DMA transfer will cause interrupt A or B */
} i2s_dma_private_handle_t;

/*! @brief I2S DMA transfer private state. */
enum _i2s_dma_state
{
    kI2S_DmaStateIdle = 0x0U, /*!< I2S is in idle state */
    kI2S_DmaStateTx,          /*!< I2S is busy transmitting data */
    kI2S_DmaStateRx,          /*!< I2S is busy receiving data */
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static status_t I2S_EnqueueUserBuffer(I2S_Type *base, i2s_dma_handle_t *handle, i2s_transfer_t transfer);
static uint32_t I2S_GetInstance(I2S_Type *base);
static inline void I2S_DisableDMAInterrupts(i2s_dma_handle_t *handle);
static inline void I2S_EnableDMAInterrupts(i2s_dma_handle_t *handle);
static void I2S_TxEnableDMA(I2S_Type *base, bool enable);
static void I2S_RxEnableDMA(I2S_Type *base, bool enable);
static uint16_t I2S_GetTransferBytes(volatile i2s_transfer_t *transfer);
static status_t I2S_StartTransferDMA(I2S_Type *base, i2s_dma_handle_t *handle);
static void I2S_AddTransferDMA(I2S_Type *base, i2s_dma_handle_t *handle);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*<! @brief DMA transfer descriptors. */
#if defined(__ICCARM__)
#pragma data_alignment = 16
static dma_descriptor_t s_DmaDescriptors[DMA_DESCRIPTORS * FSL_FEATURE_SOC_I2S_COUNT];
#elif defined(__CC_ARM)
__attribute__((aligned(16))) static dma_descriptor_t s_DmaDescriptors[DMA_DESCRIPTORS * FSL_FEATURE_SOC_I2S_COUNT];
#elif defined(__GNUC__)
__attribute__((aligned(16))) static dma_descriptor_t s_DmaDescriptors[DMA_DESCRIPTORS * FSL_FEATURE_SOC_I2S_COUNT];
#endif

/*<! @brief Buffer with dummy TX data. */
#if defined(__ICCARM__)
#pragma data_alignment = 4
static uint32_t s_DummyBufferTx = 0U;
#elif defined(__CC_ARM)
__attribute__((aligned(4))) static uint32_t s_DummyBufferTx = 0U;
#elif defined(__GNUC__)
__attribute__((aligned(4))) static uint32_t s_DummyBufferTx = 0U;
#endif

/*<! @brief Buffer to fill with RX data to discard. */
#if defined(__ICCARM__)
#pragma data_alignment = 4
static uint32_t s_DummyBufferRx = 0U;
#elif defined(__CC_ARM)
__attribute__((aligned(4))) static uint32_t s_DummyBufferRx = 0U;
#elif defined(__GNUC__)
__attribute__((aligned(4))) static uint32_t s_DummyBufferRx = 0U;
#endif

/*<! @brief Private array of data associated with available I2S peripherals. */
static i2s_dma_private_handle_t s_DmaPrivateHandle[FSL_FEATURE_SOC_I2S_COUNT];

/*<! @brief Base addresses of available I2S peripherals. */
static const uint32_t s_I2sBaseAddrs[FSL_FEATURE_SOC_I2S_COUNT] = I2S_BASE_ADDRS;

/*******************************************************************************
 * Code
 ******************************************************************************/

static status_t I2S_EnqueueUserBuffer(I2S_Type *base, i2s_dma_handle_t *handle, i2s_transfer_t transfer)
{
    uint32_t instance = I2S_GetInstance(base);
    i2s_dma_private_handle_t *privateHandle = &(s_DmaPrivateHandle[instance]);

    /* Validate input data and tranfer buffer */

    assert(handle);
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    assert((((uint32_t)transfer.data) % 4U) == 0U);
    if ((((uint32_t)transfer.data) % 4U) != 0U)
    {
        /* Data not 4-bytes aligned */
        return kStatus_InvalidArgument;
    }

    assert(transfer.dataSize != 0U);
    if (transfer.dataSize == 0U)
    {
        /* No data to send or receive */
        return kStatus_InvalidArgument;
    }

    assert((transfer.dataSize % 4U) == 0U);
    if ((transfer.dataSize % 4U) != 0U)
    {
        /* Data length not multiply of 4 bytes */
        return kStatus_InvalidArgument;
    }

    if (handle->i2sQueue[handle->queueUser].dataSize)
    {
        /* Previously prepared buffers not processed yet, reject request */
        return kStatus_I2S_Busy;
    }

    /* Enqueue data */
    privateHandle->descriptorQueue[handle->queueUser].data = transfer.data;
    privateHandle->descriptorQueue[handle->queueUser].dataSize = transfer.dataSize;
    handle->i2sQueue[handle->queueUser].data = transfer.data;
    handle->i2sQueue[handle->queueUser].dataSize = transfer.dataSize;
    handle->queueUser = (handle->queueUser + 1U) % I2S_NUM_BUFFERS;

    return kStatus_Success;
}

static uint32_t I2S_GetInstance(I2S_Type *base)
{
    uint32_t i;

    for (i = 0U; i < ARRAY_SIZE(s_I2sBaseAddrs); i++)
    {
        if ((uint32_t)base == s_I2sBaseAddrs[i])
        {
            return i;
        }
    }

    assert(false);
    return 0U;
}

static inline void I2S_DisableDMAInterrupts(i2s_dma_handle_t *handle)
{
    DMA_DisableChannelInterrupts(handle->dmaHandle->base, handle->dmaHandle->channel);
}

static inline void I2S_EnableDMAInterrupts(i2s_dma_handle_t *handle)
{
    if (handle->state != kI2S_DmaStateIdle)
    {
        DMA_EnableChannelInterrupts(handle->dmaHandle->base, handle->dmaHandle->channel);
    }
}

void I2S_TxTransferCreateHandleDMA(I2S_Type *base,
                                   i2s_dma_handle_t *handle,
                                   dma_handle_t *dmaHandle,
                                   i2s_dma_transfer_callback_t callback,
                                   void *userData)
{
    assert(handle);
    assert(dmaHandle);

    uint32_t instance = I2S_GetInstance(base);
    i2s_dma_private_handle_t *privateHandle = &(s_DmaPrivateHandle[instance]);

    memset(handle, 0U, sizeof(*handle));
    handle->state = kI2S_DmaStateIdle;
    handle->dmaHandle = dmaHandle;
    handle->completionCallback = callback;
    handle->userData = userData;

    memset(privateHandle, 0U, sizeof(*privateHandle));
    privateHandle->base = base;
    privateHandle->handle = handle;

    DMA_SetCallback(dmaHandle, I2S_DMACallback, privateHandle);
}

status_t I2S_TxTransferSendDMA(I2S_Type *base, i2s_dma_handle_t *handle, i2s_transfer_t transfer)
{
    status_t status;

    I2S_DisableDMAInterrupts(handle);

    /* Enqueue transfer buffer */
    status = I2S_EnqueueUserBuffer(base, handle, transfer);
    if (status != kStatus_Success)
    {
        I2S_EnableDMAInterrupts(handle);
        return status;
    }

    /* Initialize DMA transfer */
    if (handle->state == kI2S_DmaStateIdle)
    {
        handle->state = kI2S_DmaStateTx;
        status = I2S_StartTransferDMA(base, handle);
        if (status != kStatus_Success)
        {
            I2S_EnableDMAInterrupts(handle);
            return status;
        }
    }

    I2S_AddTransferDMA(base, handle);
    I2S_EnableDMAInterrupts(handle);

    return kStatus_Success;
}

void I2S_TransferAbortDMA(I2S_Type *base, i2s_dma_handle_t *handle)
{
    assert(handle);
    assert(handle->dmaHandle);

    uint32_t instance = I2S_GetInstance(base);
    i2s_dma_private_handle_t *privateHandle = &(s_DmaPrivateHandle[instance]);

    I2S_DisableDMAInterrupts(handle);

    /* Abort operation */
    DMA_AbortTransfer(handle->dmaHandle);

    if (handle->state == kI2S_DmaStateTx)
    {
        /* Wait until all transmitted data get out of FIFO */
        while ((base->FIFOSTAT & I2S_FIFOSTAT_TXEMPTY_MASK) == 0U)
        {
        }
        /* The last piece of valid data can be still being transmitted from I2S at this moment */

        /* Write additional data to FIFO */
        base->FIFOWR = 0U;
        while ((base->FIFOSTAT & I2S_FIFOSTAT_TXEMPTY_MASK) == 0U)
        {
        }
        /* At this moment the additional data are out of FIFO, starting being transmitted.
         * This means the preceding valid data has been just transmitted and we can stop I2S. */
        I2S_TxEnableDMA(base, false);
    }
    else
    {
        I2S_RxEnableDMA(base, false);
    }

    I2S_Disable(base);

    /* Reset state */
    handle->state = kI2S_DmaStateIdle;

    /* Clear transfer queue */
    memset((void *)&(handle->i2sQueue), 0U, sizeof(handle->i2sQueue));
    handle->queueDriver = 0U;
    handle->queueUser = 0U;

    /* Clear internal state */
    memset((void *)&(privateHandle->descriptorQueue), 0U, sizeof(privateHandle->descriptorQueue));
    memset((void *)&(privateHandle->enqueuedBytes), 0U, sizeof(privateHandle->enqueuedBytes));
    privateHandle->enqueuedBytesStart = 0U;
    privateHandle->enqueuedBytesEnd = 0U;
    privateHandle->dmaDescriptorsUsed = 0U;
    privateHandle->descriptor = 0U;
    privateHandle->queueDescriptor = 0U;
    privateHandle->intA = false;
}

void I2S_RxTransferCreateHandleDMA(I2S_Type *base,
                                   i2s_dma_handle_t *handle,
                                   dma_handle_t *dmaHandle,
                                   i2s_dma_transfer_callback_t callback,
                                   void *userData)
{
    I2S_TxTransferCreateHandleDMA(base, handle, dmaHandle, callback, userData);
}

status_t I2S_RxTransferReceiveDMA(I2S_Type *base, i2s_dma_handle_t *handle, i2s_transfer_t transfer)
{
    status_t status;

    I2S_DisableDMAInterrupts(handle);

    /* Enqueue transfer buffer */
    status = I2S_EnqueueUserBuffer(base, handle, transfer);
    if (status != kStatus_Success)
    {
        I2S_EnableDMAInterrupts(handle);
        return status;
    }

    /* Initialize DMA transfer */
    if (handle->state == kI2S_DmaStateIdle)
    {
        handle->state = kI2S_DmaStateRx;
        status = I2S_StartTransferDMA(base, handle);
        if (status != kStatus_Success)
        {
            I2S_EnableDMAInterrupts(handle);
            return status;
        }
    }

    I2S_AddTransferDMA(base, handle);
    I2S_EnableDMAInterrupts(handle);

    return kStatus_Success;
}

static void I2S_TxEnableDMA(I2S_Type *base, bool enable)
{
    if (enable)
    {
        base->FIFOCFG |= I2S_FIFOCFG_DMATX_MASK;
    }
    else
    {
        base->FIFOCFG &= (~I2S_FIFOCFG_DMATX_MASK);
        base->FIFOCFG |= I2S_FIFOCFG_EMPTYTX_MASK;
    }
}

static void I2S_RxEnableDMA(I2S_Type *base, bool enable)
{
    if (enable)
    {
        base->FIFOCFG |= I2S_FIFOCFG_DMARX_MASK;
    }
    else
    {
        base->FIFOCFG &= (~I2S_FIFOCFG_DMARX_MASK);
        base->FIFOCFG |= I2S_FIFOCFG_EMPTYRX_MASK;
    }
}

static uint16_t I2S_GetTransferBytes(volatile i2s_transfer_t *transfer)
{
    assert(transfer);

    uint16_t transferBytes;

    if (transfer->dataSize >= (2 * DMA_MAX_TRANSFER_BYTES))
    {
        transferBytes = DMA_MAX_TRANSFER_BYTES;
    }
    else if (transfer->dataSize > DMA_MAX_TRANSFER_BYTES)
    {
        transferBytes = transfer->dataSize / 2U;
        if ((transferBytes % 4U) != 0U)
        {
            transferBytes -= (transferBytes % 4U);
        }
    }
    else
    {
        transferBytes = transfer->dataSize;
    }

    return transferBytes;
}

static status_t I2S_StartTransferDMA(I2S_Type *base, i2s_dma_handle_t *handle)
{
    status_t status;
    dma_transfer_config_t xferConfig = {0};
    i2s_dma_private_handle_t *privateHandle;
    volatile i2s_transfer_t *transfer;
    uint16_t transferBytes;
    uint32_t instance;
    int i;
    dma_descriptor_t *descriptor;
    dma_descriptor_t *nextDescriptor;
    dma_xfercfg_t xfercfg;

    instance = I2S_GetInstance(base);
    privateHandle = &(s_DmaPrivateHandle[instance]);
    transfer = &(privateHandle->descriptorQueue[privateHandle->queueDescriptor]);

    transferBytes = I2S_GetTransferBytes(transfer);

    /* Prepare transfer of data via initial DMA transfer descriptor */
    DMA_PrepareTransfer(
        &xferConfig, (handle->state == kI2S_DmaStateTx) ? (void *)transfer->data : (void *)&(base->FIFORD),
        (handle->state == kI2S_DmaStateTx) ? (void *)&(base->FIFOWR) : (void *)transfer->data, sizeof(uint32_t),
        transferBytes, (handle->state == kI2S_DmaStateTx) ? kDMA_MemoryToPeripheral : kDMA_PeripheralToMemory,
        (void *)&(s_DmaDescriptors[(instance * DMA_DESCRIPTORS) + 0U]));

    /* Initial descriptor is stored in another place in memory, but treat it as another descriptor for simplicity */
    privateHandle->dmaDescriptorsUsed = 1U;
    privateHandle->intA = false;

    privateHandle->enqueuedBytes[privateHandle->enqueuedBytesEnd] = transferBytes;
    privateHandle->enqueuedBytesEnd = (privateHandle->enqueuedBytesEnd + 1U) % DMA_DESCRIPTORS;

    transfer->dataSize -= transferBytes;
    transfer->data += transferBytes;

    if (transfer->dataSize == 0U)
    {
        transfer->data = NULL;
        privateHandle->queueDescriptor = (privateHandle->queueDescriptor + 1U) % I2S_NUM_BUFFERS;
    }

    /* Link the DMA descriptors for the case when no additional transfer is queued before the initial one finishes */
    for (i = 0; i < DMA_DESCRIPTORS; i++)
    {
        descriptor = &(s_DmaDescriptors[(instance * DMA_DESCRIPTORS) + i]);
        nextDescriptor = &(s_DmaDescriptors[(instance * DMA_DESCRIPTORS) + ((i + 1) % DMA_DESCRIPTORS)]);

        xfercfg.valid = true;
        xfercfg.reload = true;
        xfercfg.swtrig = false;
        xfercfg.clrtrig = false;
        xfercfg.intA = false;
        xfercfg.intB = false;
        xfercfg.byteWidth = sizeof(uint32_t);
        xfercfg.srcInc = 0U;
        xfercfg.dstInc = 0U;
        xfercfg.transferCount = 8U;

        DMA_CreateDescriptor(descriptor, &xfercfg,
                             (handle->state == kI2S_DmaStateTx) ? (void *)&s_DummyBufferTx : (void *)&(base->FIFORD),
                             (handle->state == kI2S_DmaStateTx) ? (void *)&(base->FIFOWR) : (void *)&s_DummyBufferRx,
                             (void *)nextDescriptor);
    }

    /* Submit and start initial DMA transfer */

    if (handle->state == kI2S_DmaStateTx)
    {
        I2S_TxEnableDMA(base, true);
    }
    else
    {
        I2S_RxEnableDMA(base, true);
    }

    status = DMA_SubmitTransfer(handle->dmaHandle, &xferConfig);
    if (status != kStatus_Success)
    {
        return status;
    }

    DMA_StartTransfer(handle->dmaHandle);

    I2S_Enable(base);
    return kStatus_Success;
}

static void I2S_AddTransferDMA(I2S_Type *base, i2s_dma_handle_t *handle)
{
    dma_xfercfg_t xfercfg;
    volatile i2s_transfer_t *transfer;
    uint16_t transferBytes;
    uint32_t instance;
    i2s_dma_private_handle_t *privateHandle;
    dma_descriptor_t *descriptor;
    dma_descriptor_t *nextDescriptor;

    instance = I2S_GetInstance(base);
    privateHandle = &(s_DmaPrivateHandle[instance]);

    while (privateHandle->dmaDescriptorsUsed < DMA_DESCRIPTORS)
    {
        transfer = &(privateHandle->descriptorQueue[privateHandle->queueDescriptor]);

        if (transfer->dataSize == 0U)
        {
            /* Nothing to be added */
            return;
        }

        /* Determine currently configured descriptor and the other which it will link to */
        descriptor = &(s_DmaDescriptors[(instance * DMA_DESCRIPTORS) + privateHandle->descriptor]);
        privateHandle->descriptor = (privateHandle->descriptor + 1U) % DMA_DESCRIPTORS;
        nextDescriptor = &(s_DmaDescriptors[(instance * DMA_DESCRIPTORS) + privateHandle->descriptor]);

        transferBytes = I2S_GetTransferBytes(transfer);
        privateHandle->enqueuedBytes[privateHandle->enqueuedBytesEnd] = transferBytes;
        privateHandle->enqueuedBytesEnd = (privateHandle->enqueuedBytesEnd + 1U) % DMA_DESCRIPTORS;

        /* Configure descriptor */

        xfercfg.valid = true;
        xfercfg.reload = true;
        xfercfg.swtrig = false;
        xfercfg.clrtrig = false;
        xfercfg.intA = privateHandle->intA;
        xfercfg.intB = !privateHandle->intA;
        xfercfg.byteWidth = sizeof(uint32_t);
        xfercfg.srcInc = (handle->state == kI2S_DmaStateTx) ? 1U : 0U;
        xfercfg.dstInc = (handle->state == kI2S_DmaStateTx) ? 0U : 1U;
        xfercfg.transferCount = transferBytes / sizeof(uint32_t);

        DMA_CreateDescriptor(descriptor, &xfercfg,
                             (handle->state == kI2S_DmaStateTx) ? (void *)transfer->data : (void *)&(base->FIFORD),
                             (handle->state == kI2S_DmaStateTx) ? (void *)&(base->FIFOWR) : (void *)transfer->data,
                             (void *)nextDescriptor);

        /* Advance internal state */

        privateHandle->dmaDescriptorsUsed++;
        privateHandle->intA = !privateHandle->intA;

        transfer->dataSize -= transferBytes;
        transfer->data += transferBytes;
        if (transfer->dataSize == 0U)
        {
            transfer->data = NULL;
            privateHandle->queueDescriptor = (privateHandle->queueDescriptor + 1U) % I2S_NUM_BUFFERS;
        }
    }
}

void I2S_DMACallback(dma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds)
{
    i2s_dma_private_handle_t *privateHandle = (i2s_dma_private_handle_t *)userData;
    i2s_dma_handle_t *i2sHandle = privateHandle->handle;
    I2S_Type *base = privateHandle->base;

    if ((!transferDone) || (i2sHandle->state == kI2S_DmaStateIdle))
    {
        return;
    }

    if (privateHandle->dmaDescriptorsUsed > 0U)
    {
        /* Finished descriptor, decrease amount of data to be processed */

        i2sHandle->i2sQueue[i2sHandle->queueDriver].dataSize -=
            privateHandle->enqueuedBytes[privateHandle->enqueuedBytesStart];
        i2sHandle->i2sQueue[i2sHandle->queueDriver].data +=
            privateHandle->enqueuedBytes[privateHandle->enqueuedBytesStart];
        privateHandle->enqueuedBytes[privateHandle->enqueuedBytesStart] = 0U;
        privateHandle->enqueuedBytesStart = (privateHandle->enqueuedBytesStart + 1U) % DMA_DESCRIPTORS;

        privateHandle->dmaDescriptorsUsed--;

        if (i2sHandle->i2sQueue[i2sHandle->queueDriver].dataSize == 0U)
        {
            /* Entire user buffer sent or received - advance to next one */
            i2sHandle->i2sQueue[i2sHandle->queueDriver].data = NULL;
            i2sHandle->queueDriver = (i2sHandle->queueDriver + 1U) % I2S_NUM_BUFFERS;

            /* Notify user about buffer completion */
            if (i2sHandle->completionCallback)
            {
                (i2sHandle->completionCallback)(base, i2sHandle, kStatus_I2S_BufferComplete, i2sHandle->userData);
            }
        }
    }

    if (i2sHandle->i2sQueue[i2sHandle->queueDriver].dataSize == 0U)
    {
        /* All user buffers processed */
        I2S_TransferAbortDMA(base, i2sHandle);

        /* Notify user about completion of the final buffer */
        if (i2sHandle->completionCallback)
        {
            (i2sHandle->completionCallback)(base, i2sHandle, kStatus_I2S_Done, i2sHandle->userData);
        }
    }
    else
    {
        /* Enqueue another user buffer to DMA if it could not be done when in I2S_Rx/TxTransferSendDMA */
        I2S_AddTransferDMA(base, i2sHandle);
    }
}
