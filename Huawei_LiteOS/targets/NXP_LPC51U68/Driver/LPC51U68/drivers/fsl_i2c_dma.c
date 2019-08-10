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

#include "fsl_i2c_dma.h"
#include "fsl_flexcomm.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*<! @brief Structure definition for i2c_master_dma_handle_t. The structure is private. */
typedef struct _i2c_master_dma_private_handle
{
    I2C_Type *base;
    i2c_master_dma_handle_t *handle;
} i2c_master_dma_private_handle_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief DMA callback for I2C master DMA driver.
 *
 * @param handle DMA handler for I2C master DMA driver
 * @param userData user param passed to the callback function
 */
static void I2C_MasterTransferCallbackDMA(dma_handle_t *handle, void *userData);

/*!
 * @brief Set up master transfer, send slave address and sub address(if any), wait until the
 * wait until address sent status return.
 *
 * @param base I2C peripheral base address.
 * @param handle pointer to i2c_master_dma_handle_t structure which stores the transfer state.
 * @param xfer pointer to i2c_master_transfer_t structure.
 */
static status_t I2C_InitTransferStateMachineDMA(I2C_Type *base,
                                                i2c_master_dma_handle_t *handle,
                                                i2c_master_transfer_t *xfer);

/*!
 * @brief Get the I2C instance from peripheral base address.
 *
 * @param base I2C peripheral base address.
 * @return I2C instance.
 */
extern uint32_t I2C_GetInstance(I2C_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*<! Private handle only used for internally. */
static i2c_master_dma_private_handle_t s_dmaPrivateHandle[FSL_FEATURE_SOC_I2C_COUNT];

/*! @brief IRQ name array */
static const IRQn_Type s_i2cIRQ[] = I2C_IRQS;

/*******************************************************************************
 * Codes
 ******************************************************************************/

/*!
 * @brief Prepares the transfer state machine and fills in the command buffer.
 * @param handle Master nonblocking driver handle.
 */
static status_t I2C_InitTransferStateMachineDMA(I2C_Type *base,
                                                i2c_master_dma_handle_t *handle,
                                                i2c_master_transfer_t *xfer)
{
    struct _i2c_master_transfer *transfer;

    handle->transfer = *xfer;
    transfer = &(handle->transfer);

    handle->transferCount = 0;
    handle->remainingBytesDMA = 0;
    handle->buf = (uint8_t *)transfer->data;
    handle->remainingSubaddr = 0;

    if (transfer->flags & kI2C_TransferNoStartFlag)
    {
        /* Start condition shall be ommited, switch directly to next phase */
        if (transfer->dataSize == 0)
        {
            handle->state = kStopState;
        }
        else if (handle->transfer.direction == kI2C_Write)
        {
            handle->state = xfer->dataSize = kTransmitDataState;
        }
        else if (handle->transfer.direction == kI2C_Read)
        {
            handle->state = (xfer->dataSize == 1) ? kReceiveLastDataState : kReceiveDataState;
        }
        else
        {
            return kStatus_I2C_InvalidParameter;
        }
    }
    else
    {
        if (transfer->subaddressSize != 0)
        {
            int i;
            uint32_t subaddress;

            if (transfer->subaddressSize > sizeof(handle->subaddrBuf))
            {
                return kStatus_I2C_InvalidParameter;
            }

            /* Prepare subaddress transmit buffer, most significant byte is stored at the lowest address */
            subaddress = xfer->subaddress;
            for (i = xfer->subaddressSize - 1; i >= 0; i--)
            {
                handle->subaddrBuf[i] = subaddress & 0xff;
                subaddress >>= 8;
            }
            handle->remainingSubaddr = transfer->subaddressSize;
        }

        handle->state = kStartState;
    }

    return kStatus_Success;
}

static void I2C_RunDMATransfer(I2C_Type *base, i2c_master_dma_handle_t *handle)
{
    int transfer_size;
    dma_transfer_config_t xferConfig;

    /* Update transfer count */
    handle->transferCount = handle->buf - (uint8_t *)handle->transfer.data;

    /* Check if there is anything to be transferred at all */
    if (handle->remainingBytesDMA == 0)
    {
        /* No data to be transferrred, disable DMA */
        base->MSTCTL = 0;
        return;
    }

    /* Calculate transfer size */
    transfer_size = handle->remainingBytesDMA;
    if (transfer_size > I2C_MAX_DMA_TRANSFER_COUNT)
    {
        transfer_size = I2C_MAX_DMA_TRANSFER_COUNT;
    }

    switch (handle->transfer.direction)
    {
        case kI2C_Write:
            DMA_PrepareTransfer(&xferConfig, handle->buf, (void *)&base->MSTDAT, sizeof(uint8_t), transfer_size,
                                kDMA_MemoryToPeripheral, NULL);
            break;

        case kI2C_Read:
            DMA_PrepareTransfer(&xferConfig, (void *)&base->MSTDAT, handle->buf, sizeof(uint8_t), transfer_size,
                                kDMA_PeripheralToMemory, NULL);
            break;

        default:
            /* This should never happen */
            assert(0);
            break;
    }

    DMA_SubmitTransfer(handle->dmaHandle, &xferConfig);
    DMA_StartTransfer(handle->dmaHandle);

    handle->remainingBytesDMA -= transfer_size;
    handle->buf += transfer_size;
}

/*!
 * @brief Execute states until the transfer is done.
 * @param handle Master nonblocking driver handle.
 * @param[out] isDone Set to true if the transfer has completed.
 * @retval #kStatus_Success
 * @retval #kStatus_I2C_ArbitrationLost
 * @retval #kStatus_I2C_Nak
 */
static status_t I2C_RunTransferStateMachineDMA(I2C_Type *base, i2c_master_dma_handle_t *handle, bool *isDone)
{
    uint32_t status;
    uint32_t master_state;
    struct _i2c_master_transfer *transfer;
    dma_transfer_config_t xferConfig;
    status_t err;
    uint32_t start_flag = 0;

    transfer = &(handle->transfer);

    *isDone = false;

    status = I2C_GetStatusFlags(base);

    if (status & I2C_STAT_MSTARBLOSS_MASK)
    {
        I2C_MasterClearStatusFlags(base, I2C_STAT_MSTARBLOSS_MASK);
        DMA_AbortTransfer(handle->dmaHandle);
        base->MSTCTL = 0;
        return kStatus_I2C_ArbitrationLost;
    }

    if (status & I2C_STAT_MSTSTSTPERR_MASK)
    {
        I2C_MasterClearStatusFlags(base, I2C_STAT_MSTSTSTPERR_MASK);
        DMA_AbortTransfer(handle->dmaHandle);
        base->MSTCTL = 0;
        return kStatus_I2C_StartStopError;
    }

    if ((status & I2C_STAT_MSTPENDING_MASK) == 0)
    {
        return kStatus_I2C_Busy;
    }

    /* Get the state of the I2C module */
    master_state = (status & I2C_STAT_MSTSTATE_MASK) >> I2C_STAT_MSTSTATE_SHIFT;

    if ((master_state == I2C_STAT_MSTCODE_NACKADR) || (master_state == I2C_STAT_MSTCODE_NACKDAT))
    {
        /* Slave NACKed last byte, issue stop and return error */
        DMA_AbortTransfer(handle->dmaHandle);
        base->MSTCTL = I2C_MSTCTL_MSTSTOP_MASK;
        handle->state = kWaitForCompletionState;
        return kStatus_I2C_Nak;
    }

    err = kStatus_Success;

    if (handle->state == kStartState)
    {
        /* set start flag for later use */
        start_flag = I2C_MSTCTL_MSTSTART_MASK;

        if (handle->remainingSubaddr)
        {
            base->MSTDAT = (uint32_t)transfer->slaveAddress << 1;
            handle->state = kTransmitSubaddrState;
        }
        else if (transfer->direction == kI2C_Write)
        {
            base->MSTDAT = (uint32_t)transfer->slaveAddress << 1;
            if (transfer->dataSize == 0)
            {
                /* No data to be transferred, initiate start and schedule stop */
                base->MSTCTL = I2C_MSTCTL_MSTSTART_MASK;
                handle->state = kStopState;
                return err;
            }
            handle->state = kTransmitDataState;
        }
        else if ((transfer->direction == kI2C_Read) && (transfer->dataSize > 0))
        {
            base->MSTDAT = ((uint32_t)transfer->slaveAddress << 1) | 1u;
            if (transfer->dataSize == 1)
            {
                /* The very last byte is always received by means of SW */
                base->MSTCTL = I2C_MSTCTL_MSTSTART_MASK;
                handle->state = kReceiveLastDataState;
                return err;
            }
            handle->state = kReceiveDataState;
        }
        else
        {
            handle->state = kIdleState;
            err = kStatus_I2C_UnexpectedState;
            return err;
        }
    }

    switch (handle->state)
    {
        case kTransmitSubaddrState:
            if ((master_state != I2C_STAT_MSTCODE_TXREADY) && (!start_flag))
            {
                return kStatus_I2C_UnexpectedState;
            }

            base->MSTCTL = start_flag | I2C_MSTCTL_MSTDMA_MASK;

            /* Prepare and submit DMA transfer. */
            DMA_PrepareTransfer(&xferConfig, handle->subaddrBuf, (void *)&base->MSTDAT, sizeof(uint8_t),
                                handle->remainingSubaddr, kDMA_MemoryToPeripheral, NULL);
            DMA_SubmitTransfer(handle->dmaHandle, &xferConfig);
            DMA_StartTransfer(handle->dmaHandle);
            handle->remainingSubaddr = 0;
            if (transfer->dataSize)
            {
                /* There is data to be transferred, if there is write to read turnaround it is necessary to perform
                 * repeated start */
                handle->state = (transfer->direction == kI2C_Read) ? kStartState : kTransmitDataState;
            }
            else
            {
                /* No more data, schedule stop condition */
                handle->state = kStopState;
            }
            break;

        case kTransmitDataState:
            if ((master_state != I2C_STAT_MSTCODE_TXREADY) && (!start_flag))
            {
                return kStatus_I2C_UnexpectedState;
            }

            base->MSTCTL = start_flag | I2C_MSTCTL_MSTDMA_MASK;
            handle->remainingBytesDMA = handle->transfer.dataSize;

            I2C_RunDMATransfer(base, handle);

            /* Schedule stop condition */
            handle->state = kStopState;
            break;

        case kReceiveDataState:
            if ((master_state != I2C_STAT_MSTCODE_RXREADY) && (!start_flag))
            {
                return kStatus_I2C_UnexpectedState;
            }

            base->MSTCTL = start_flag | I2C_MSTCTL_MSTDMA_MASK;
            handle->remainingBytesDMA = handle->transfer.dataSize - 1;

            I2C_RunDMATransfer(base, handle);

            /* Schedule reception of last data byte */
            handle->state = kReceiveLastDataState;
            break;

        case kReceiveLastDataState:
            if (master_state != I2C_STAT_MSTCODE_RXREADY)
            {
                return kStatus_I2C_UnexpectedState;
            }

            ((uint8_t *)transfer->data)[transfer->dataSize - 1] = base->MSTDAT;
            handle->transferCount++;

            /* No more data expected, issue NACK and STOP right away */
            base->MSTCTL = I2C_MSTCTL_MSTSTOP_MASK;
            handle->state = kWaitForCompletionState;
            break;

        case kStopState:
            if (transfer->flags & kI2C_TransferNoStopFlag)
            {
                /* Stop condition is omitted, we are done */
                *isDone = true;
                handle->state = kIdleState;
                break;
            }
            /* Send stop condition */
            base->MSTCTL = I2C_MSTCTL_MSTSTOP_MASK;
            handle->state = kWaitForCompletionState;
            break;

        case kWaitForCompletionState:
            *isDone = true;
            handle->state = kIdleState;
            break;

        case kStartState:
        case kIdleState:
        default:
            /* State machine shall not be invoked again once it enters the idle state */
            err = kStatus_I2C_UnexpectedState;
            break;
    }

    return err;
}

void I2C_MasterTransferDMAHandleIRQ(I2C_Type *base, i2c_master_dma_handle_t *handle)
{
    bool isDone;
    status_t result;

    /* Don't do anything if we don't have a valid handle. */
    if (!handle)
    {
        return;
    }

    result = I2C_RunTransferStateMachineDMA(base, handle, &isDone);

    if (isDone || (result != kStatus_Success))
    {
        /* Disable internal IRQ enables. */
        I2C_DisableInterrupts(base,
                              I2C_INTSTAT_MSTPENDING_MASK | I2C_INTSTAT_MSTARBLOSS_MASK | I2C_INTSTAT_MSTSTSTPERR_MASK);

        /* Invoke callback. */
        if (handle->completionCallback)
        {
            handle->completionCallback(base, handle, result, handle->userData);
        }
    }
}

static void I2C_MasterTransferCallbackDMA(dma_handle_t *handle, void *userData)
{
    i2c_master_dma_private_handle_t *dmaPrivateHandle;

    /* Don't do anything if we don't have a valid handle. */
    if (!handle)
    {
        return;
    }

    dmaPrivateHandle = (i2c_master_dma_private_handle_t *)userData;
    I2C_RunDMATransfer(dmaPrivateHandle->base, dmaPrivateHandle->handle);
}

void I2C_MasterTransferCreateHandleDMA(I2C_Type *base,
                                       i2c_master_dma_handle_t *handle,
                                       i2c_master_dma_transfer_callback_t callback,
                                       void *userData,
                                       dma_handle_t *dmaHandle)
{
    uint32_t instance;

    assert(handle);
    assert(dmaHandle);

    /* Zero handle. */
    memset(handle, 0, sizeof(*handle));

    /* Look up instance number */
    instance = I2C_GetInstance(base);

    /* Set the user callback and userData. */
    handle->completionCallback = callback;
    handle->userData = userData;

    FLEXCOMM_SetIRQHandler(base, (flexcomm_irq_handler_t)(uintptr_t)I2C_MasterTransferDMAHandleIRQ, handle);

    /* Clear internal IRQ enables and enable NVIC IRQ. */
    I2C_DisableInterrupts(base,
                          I2C_INTSTAT_MSTPENDING_MASK | I2C_INTSTAT_MSTARBLOSS_MASK | I2C_INTSTAT_MSTSTSTPERR_MASK);
    EnableIRQ(s_i2cIRQ[instance]);

    /* Set the handle for DMA. */
    handle->dmaHandle = dmaHandle;

    s_dmaPrivateHandle[instance].base = base;
    s_dmaPrivateHandle[instance].handle = handle;

    DMA_SetCallback(dmaHandle, (dma_callback)(uintptr_t)I2C_MasterTransferCallbackDMA, &s_dmaPrivateHandle[instance]);
}

status_t I2C_MasterTransferDMA(I2C_Type *base, i2c_master_dma_handle_t *handle, i2c_master_transfer_t *xfer)
{
    status_t result;

    assert(handle);
    assert(xfer);
    assert(xfer->subaddressSize <= sizeof(xfer->subaddress));

    /* Return busy if another transaction is in progress. */
    if (handle->state != kIdleState)
    {
        return kStatus_I2C_Busy;
    }

    /* Prepare transfer state machine. */
    result = I2C_InitTransferStateMachineDMA(base, handle, xfer);

    /* Clear error flags. */
    I2C_MasterClearStatusFlags(base, I2C_STAT_MSTARBLOSS_MASK | I2C_STAT_MSTSTSTPERR_MASK);

    /* Enable I2C internal IRQ sources */
    I2C_EnableInterrupts(base,
                         I2C_INTSTAT_MSTARBLOSS_MASK | I2C_INTSTAT_MSTSTSTPERR_MASK | I2C_INTSTAT_MSTPENDING_MASK);

    return result;
}

status_t I2C_MasterTransferGetCountDMA(I2C_Type *base, i2c_master_dma_handle_t *handle, size_t *count)
{
    assert(handle);

    if (!count)
    {
        return kStatus_InvalidArgument;
    }

    /* Catch when there is not an active transfer. */
    if (handle->state == kIdleState)
    {
        *count = 0;
        return kStatus_NoTransferInProgress;
    }

    /* There is no necessity to disable interrupts as we read a single integer value */
    *count = handle->transferCount;
    return kStatus_Success;
}

void I2C_MasterTransferAbortDMA(I2C_Type *base, i2c_master_dma_handle_t *handle)
{
    uint32_t status;
    uint32_t master_state;

    if (handle->state != kIdleState)
    {
        DMA_AbortTransfer(handle->dmaHandle);

        /* Disable DMA */
        base->MSTCTL = 0;

        /* Disable internal IRQ enables. */
        I2C_DisableInterrupts(base,
                              I2C_INTSTAT_MSTPENDING_MASK | I2C_INTSTAT_MSTARBLOSS_MASK | I2C_INTSTAT_MSTSTSTPERR_MASK);

        /* Wait until module is ready */
        do
        {
            status = I2C_GetStatusFlags(base);
        } while ((status & I2C_STAT_MSTPENDING_MASK) == 0);

        /* Clear controller state. */
        I2C_MasterClearStatusFlags(base, I2C_STAT_MSTARBLOSS_MASK | I2C_STAT_MSTSTSTPERR_MASK);

        /* Get the state of the I2C module */
        master_state = (status & I2C_STAT_MSTSTATE_MASK) >> I2C_STAT_MSTSTATE_SHIFT;

        if (master_state != I2C_STAT_MSTCODE_IDLE)
        {
            /* Send a stop command to finalize the transfer. */
            base->MSTCTL = I2C_MSTCTL_MSTSTOP_MASK;

            /* Wait until module is ready */
            do
            {
                status = I2C_GetStatusFlags(base);
            } while ((status & I2C_STAT_MSTPENDING_MASK) == 0);

            /* Clear controller state. */
            I2C_MasterClearStatusFlags(base, I2C_STAT_MSTARBLOSS_MASK | I2C_STAT_MSTSTSTPERR_MASK);
        }

        /* Reset the state to idle. */
        handle->state = kIdleState;
    }
}
