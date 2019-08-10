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
 * ANY EPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "fsl_io.h"
#include "fsl_debug_console_conf.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* check avaliable device  */
#if (defined(FSL_FEATURE_SOC_UART_COUNT) && (FSL_FEATURE_SOC_UART_COUNT != 0))
#define DEBUG_CONSOLE_IO_UART
#endif

#if (defined(FSL_FEATURE_SOC_IUART_COUNT) && (FSL_FEATURE_SOC_IUART_COUNT != 0))
#define DEBUG_CONSOLE_IO_IUART
#endif

#if (defined(FSL_FEATURE_SOC_LPUART_COUNT) && (FSL_FEATURE_SOC_LPUART_COUNT != 0))
#define DEBUG_CONSOLE_IO_LPUART
#endif

#if (defined(FSL_FEATURE_SOC_LPSCI_COUNT) && (FSL_FEATURE_SOC_LPSCI_COUNT != 0))
#define DEBUG_CONSOLE_IO_LPSCI
#endif

#if ((defined(FSL_FEATURE_SOC_USB_COUNT) && (FSL_FEATURE_SOC_USB_COUNT != 0)) && defined(BOARD_USE_VIRTUALCOM))
#define DEBUG_CONSOLE_IO_USBCDC
#endif

#if (defined(FSL_FEATURE_SOC_FLEXCOMM_COUNT) && (FSL_FEATURE_SOC_FLEXCOMM_COUNT != 0))
#define DEBUG_CONSOLE_IO_FLEXCOMM
#endif

#if (defined(FSL_FEATURE_SOC_VFIFO_COUNT) && (FSL_FEATURE_SOC_VFIFO_COUNT != 0))
#define DEBUG_CONSOLE_IO_VUSART
#endif

/* configuration for debug console device */
/* If new device is required as the low level device for debug console,
 * Add the #elif branch and add the preprocessor macro to judge whether
 * this kind of device exist in this SOC. */
#if (defined DEBUG_CONSOLE_IO_UART) || (defined DEBUG_CONSOLE_IO_IUART)
#include "fsl_uart.h"
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
static uart_handle_t s_ioUartHandler;
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */
#endif /* defined DEBUG_CONSOLE_IO_UART) || (defined DEBUG_CONSOLE_IO_IUART */

#if defined DEBUG_CONSOLE_IO_LPUART
#include "fsl_lpuart.h"
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
static lpuart_handle_t s_ioLpuartHandler;
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */
#endif /* DEBUG_CONSOLE_IO_LPUART */

#if defined DEBUG_CONSOLE_IO_LPSCI
#include "fsl_lpsci.h"
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
static lpsci_handle_t s_ioLpsciHandler;
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */
#endif /* DEBUG_CONSOLE_IO_LPSCI */

#if defined DEBUG_CONSOLE_IO_USBCDC
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device_cdc_acm.h"
#include "usb_device_ch9.h"
#include "virtual_com.h"
#endif /* DEBUG_CONSOLE_IO_USBCDC */

#if (defined DEBUG_CONSOLE_IO_FLEXCOMM) || (defined DEBUG_CONSOLE_IO_VUSART)
#include "fsl_usart.h"
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
static usart_handle_t s_ioUsartHandler;
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */
#endif /* defined DEBUG_CONSOLE_IO_FLEXCOMM) || (defined DEBUG_CONSOLE_IO_VUSART */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Debug console IO state information. */
static io_state_t s_debugConsoleIO = {
    .ioBase = NULL,
    .ioType = DEBUG_CONSOLE_DEVICE_TYPE_NONE,
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
    .callBack = NULL,
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */
};

/*******************************************************************************
 * Code
 ******************************************************************************/

#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING

#if (defined DEBUG_CONSOLE_IO_UART) || (defined DEBUG_CONSOLE_IO_IUART)
static void UART_Callback(UART_Type *base, uart_handle_t *handle, status_t status, void *userData)
{
    bool tx = false, rx = false;
    size_t size = 0U;

    if (status == kStatus_UART_RxIdle)
    {
        rx = true;
        size = handle->txDataSizeAll;
    }

    if (status == kStatus_UART_TxIdle)
    {
        tx = true;
        size = handle->txDataSizeAll;
    }

    /* inform the buffer layer that transfer is complete */
    if (s_debugConsoleIO.callBack != NULL)
    {
        /* call buffer callback function */
        s_debugConsoleIO.callBack(&size, rx, tx);
    }
}
#endif /* defined DEBUG_CONSOLE_IO_UART) || (defined DEBUG_CONSOLE_IO_IUART */

#if defined DEBUG_CONSOLE_IO_LPSCI
static void LPSCI_Callback(UART0_Type *base, lpsci_handle_t *handle, status_t status, void *userData)
{
    bool tx = false, rx = false;
    size_t size = 0U;

    if (status == kStatus_LPSCI_RxIdle)
    {
        rx = true;
        size = handle->txDataSizeAll;
    }

    if (status == kStatus_LPSCI_TxIdle)
    {
        tx = true;
        size = handle->txDataSizeAll;
    }

    /* inform the buffer layer that transfer is complete */
    if (s_debugConsoleIO.callBack != NULL)
    {
        /* call buffer callback function */
        s_debugConsoleIO.callBack(&size, rx, tx);
    }
}
#endif /* DEBUG_CONSOLE_IO_LPSCI */

#if defined DEBUG_CONSOLE_IO_LPUART
static void LPUART_Callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
    bool tx = false, rx = false;
    size_t size = 0U;

    if (status == kStatus_LPUART_RxIdle)
    {
        rx = true;
        size = handle->txDataSizeAll;
    }

    if (status == kStatus_LPUART_TxIdle)
    {
        tx = true;
        size = handle->txDataSizeAll;
    }

    /* inform the buffer layer that transfer is complete */
    if (s_debugConsoleIO.callBack != NULL)
    {
        /* call buffer callback function */
        s_debugConsoleIO.callBack(&size, rx, tx);
    }
}
#endif /* DEBUG_CONSOLE_IO_LPUART */

#if (defined DEBUG_CONSOLE_IO_FLEXCOMM) || (defined DEBUG_CONSOLE_IO_VUSART)
static void USART_Callback(USART_Type *base, usart_handle_t *handle, status_t status, void *userData)
{
    bool tx = false, rx = false;
    size_t size = 0U;

    if (status == kStatus_USART_RxIdle)
    {
        rx = true;
        size = handle->txDataSizeAll;
    }

    if (status == kStatus_USART_TxIdle)
    {
        tx = true;
        size = handle->txDataSizeAll;
    }

    /* inform the buffer layer that transfer is complete */
    if (s_debugConsoleIO.callBack != NULL)
    {
        /* call buffer callback function */
        s_debugConsoleIO.callBack(&size, rx, tx);
    }
}
#endif /* defined DEBUG_CONSOLE_IO_FLEXCOMM) || (defined DEBUG_CONSOLE_IO_VUSART */
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */

void IO_Init(io_state_t *io, uint32_t baudRate, uint32_t clkSrcFreq, uint8_t *ringBuffer)
{
    assert(NULL != io);

    /* record device type/base */
    s_debugConsoleIO.ioType = io->ioType;
    s_debugConsoleIO.ioBase = (void *)(io->ioBase);

    switch (s_debugConsoleIO.ioType)
    {
#if (defined DEBUG_CONSOLE_IO_UART) || (defined DEBUG_CONSOLE_IO_IUART)
        case DEBUG_CONSOLE_DEVICE_TYPE_UART:
        case DEBUG_CONSOLE_DEVICE_TYPE_IUART:
        {
            uart_config_t uart_config;
            UART_GetDefaultConfig(&uart_config);
            uart_config.baudRate_Bps = baudRate;
            /* Enable clock and initial UART module follow user configure structure. */
            UART_Init((UART_Type *)s_debugConsoleIO.ioBase, &uart_config, clkSrcFreq);
            UART_EnableTx(s_debugConsoleIO.ioBase, true);
            UART_EnableRx(s_debugConsoleIO.ioBase, true);
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
            s_debugConsoleIO.callBack = io->callBack;
            /* create handler for interrupt transfer */
            UART_TransferCreateHandle(s_debugConsoleIO.ioBase, &s_ioUartHandler, UART_Callback, NULL);
            /* start ring buffer */
            UART_TransferStartRingBuffer(s_debugConsoleIO.ioBase, &s_ioUartHandler, ringBuffer,
                                         DEBUG_CONSOLE_RECEIVE_BUFFER_LEN);
#endif
        }
        break;
#endif

#if defined DEBUG_CONSOLE_IO_LPUART
        case DEBUG_CONSOLE_DEVICE_TYPE_LPUART:
        {
            lpuart_config_t lpuart_config;
            LPUART_GetDefaultConfig(&lpuart_config);
            lpuart_config.baudRate_Bps = baudRate;
            /* Enable clock and initial UART module follow user configure structure. */
            LPUART_Init((LPUART_Type *)s_debugConsoleIO.ioBase, &lpuart_config, clkSrcFreq);
            LPUART_EnableTx(s_debugConsoleIO.ioBase, true);
            LPUART_EnableRx(s_debugConsoleIO.ioBase, true);
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
            s_debugConsoleIO.callBack = io->callBack;
            /* create handler for interrupt transfer */
            LPUART_TransferCreateHandle(s_debugConsoleIO.ioBase, &s_ioLpuartHandler, LPUART_Callback, NULL);
            /* start ring buffer */
            LPUART_TransferStartRingBuffer(s_debugConsoleIO.ioBase, &s_ioLpuartHandler, ringBuffer,
                                           DEBUG_CONSOLE_RECEIVE_BUFFER_LEN);
#endif
        }
        break;
#endif
#if defined DEBUG_CONSOLE_IO_LPSCI
        case DEBUG_CONSOLE_DEVICE_TYPE_LPSCI:
        {
            lpsci_config_t lpsci_config;
            LPSCI_GetDefaultConfig(&lpsci_config);
            lpsci_config.baudRate_Bps = baudRate;
            /* Enable clock and initial UART module follow user configure structure. */
            LPSCI_Init((UART0_Type *)s_debugConsoleIO.ioBase, &lpsci_config, clkSrcFreq);
            LPSCI_EnableTx(s_debugConsoleIO.ioBase, true);
            LPSCI_EnableRx(s_debugConsoleIO.ioBase, true);
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
            s_debugConsoleIO.callBack = io->callBack;
            /* create handler for interrupt transfer */
            LPSCI_TransferCreateHandle(s_debugConsoleIO.ioBase, &s_ioLpsciHandler, LPSCI_Callback, NULL);
            /* start ring buffer */
            LPSCI_TransferStartRingBuffer(s_debugConsoleIO.ioBase, &s_ioLpsciHandler, ringBuffer,
                                          DEBUG_CONSOLE_RECEIVE_BUFFER_LEN);
#endif
        }
        break;
#endif
#if defined DEBUG_CONSOLE_IO_USBCDC
        case DEBUG_CONSOLE_DEVICE_TYPE_USBCDC:
        {
            s_debugConsoleIO.ioBase = USB_VcomInit();
        }
        break;
#endif
#if defined DEBUG_CONSOLE_IO_FLEXCOMM
        case DEBUG_CONSOLE_DEVICE_TYPE_FLEXCOMM:
        {
            usart_config_t usart_config;
            USART_GetDefaultConfig(&usart_config);
            usart_config.baudRate_Bps = baudRate;
            /* Enable clock and initial UART module follow user configure structure. */
            USART_Init((USART_Type *)s_debugConsoleIO.ioBase, &usart_config, clkSrcFreq);
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
            s_debugConsoleIO.callBack = io->callBack;
            /* create handler for interrupt transfer */
            USART_TransferCreateHandle(s_debugConsoleIO.ioBase, &s_ioUsartHandler, USART_Callback, NULL);
            /* start ring buffer */
            USART_TransferStartRingBuffer(s_debugConsoleIO.ioBase, &s_ioUsartHandler, ringBuffer,
                                          DEBUG_CONSOLE_RECEIVE_BUFFER_LEN);
#endif
        }
        break;
#endif
#if defined DEBUG_CONSOLE_IO_VUSART
        case DEBUG_CONSOLE_DEVICE_TYPE_VUSART:
        {
            usart_config_t usart_config;
            USART_GetDefaultConfig(&usart_config);
            usart_config.baudRate_Bps = baudRate;
            usart_config.enableRx = true;
            usart_config.enableTx = true;
            /* Enable rx fifo for user's continously input */
            usart_config.fifoConfig.enableRxFifo = true;
            usart_config.fifoConfig.rxFifoSize = 8;
            /* Enable clock and initial UART module follow user configure structure. */
            USART_Init((USART_Type *)s_debugConsoleIO.ioBase, &usart_config, clkSrcFreq);
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
            s_debugConsoleIO.callBack = io->callBack;
            /* create handler for interrupt transfer */
            USART_TransferCreateHandle(s_debugConsoleIO.ioBase, &s_ioUsartHandler, USART_Callback, NULL);
            /* start ring buffer */
            USART_TransferStartRingBuffer(s_debugConsoleIO.ioBase, &s_ioUsartHandler, ringBuffer,
                                          DEBUG_CONSOLE_RECEIVE_BUFFER_LEN);
#endif
        }
        break;
#endif
        default:
            break;
    }
}

status_t IO_Deinit(void)
{
    if (s_debugConsoleIO.ioType == DEBUG_CONSOLE_DEVICE_TYPE_NONE)
    {
        return kStatus_Success;
    }

    switch (s_debugConsoleIO.ioType)
    {
#if (defined DEBUG_CONSOLE_IO_UART) || (defined DEBUG_CONSOLE_IO_IUART)
        case DEBUG_CONSOLE_DEVICE_TYPE_UART:
        case DEBUG_CONSOLE_DEVICE_TYPE_IUART:
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
            /* stop ring buffer */
            UART_TransferStopRingBuffer(s_debugConsoleIO.ioBase, &s_ioUartHandler);
#endif
            /* Disable UART module. */
            UART_Deinit((UART_Type *)s_debugConsoleIO.ioBase);

            break;
#endif
#if defined DEBUG_CONSOLE_IO_LPSCI
        case DEBUG_CONSOLE_DEVICE_TYPE_LPSCI:
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
            /* stop ring buffer */
            LPSCI_TransferStopRingBuffer(s_debugConsoleIO.ioBase, &s_ioLpsciHandler);
#endif
            /* Disable LPSCI module. */
            LPSCI_Deinit((UART0_Type *)s_debugConsoleIO.ioBase);

            break;
#endif
#if defined DEBUG_CONSOLE_IO_LPUART
        case DEBUG_CONSOLE_DEVICE_TYPE_LPUART:
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
            /* stop ring buffer */
            LPUART_TransferStopRingBuffer(s_debugConsoleIO.ioBase, &s_ioLpuartHandler);
#endif
            /* Disable LPUART module. */
            LPUART_Deinit((LPUART_Type *)s_debugConsoleIO.ioBase);

            break;
#endif
#if defined DEBUG_CONSOLE_IO_USBCDC
        case DEBUG_CONSOLE_DEVICE_TYPE_USBCDC:
            /* Disable USBCDC module. */
            USB_VcomDeinit(s_debugConsoleIO.ioBase);
            break;
#endif
#if (defined DEBUG_CONSOLE_IO_FLEXCOMM) || (defined DEBUG_CONSOLE_IO_VUSART)
        case DEBUG_CONSOLE_DEVICE_TYPE_FLEXCOMM:
        case DEBUG_CONSOLE_DEVICE_TYPE_VUSART:
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
            /* stop ring buffer */
            USART_TransferStopRingBuffer(s_debugConsoleIO.ioBase, &s_ioUsartHandler);
#endif
            /* deinit IO */
            USART_Deinit((USART_Type *)s_debugConsoleIO.ioBase);

            break;
#endif
        default:
            s_debugConsoleIO.ioType = DEBUG_CONSOLE_DEVICE_TYPE_NONE;
            break;
    }

    s_debugConsoleIO.ioType = DEBUG_CONSOLE_DEVICE_TYPE_NONE;

    return kStatus_Success;
}

status_t IO_WaitIdle(void)
{
    switch (s_debugConsoleIO.ioType)
    {
#if (defined DEBUG_CONSOLE_IO_UART)
        case DEBUG_CONSOLE_DEVICE_TYPE_UART:
            /* wait transfer complete flag */
            while (!(UART_GetStatusFlags(s_debugConsoleIO.ioBase) & kUART_TransmissionCompleteFlag))
            {
            }

            break;
#endif

#if (defined DEBUG_CONSOLE_IO_IUART)
        case DEBUG_CONSOLE_DEVICE_TYPE_IUART:
            /* wait transfer complete flag */
            while (!(UART_GetStatusFlag(s_debugConsoleIO.ioBase, kUART_TxCompleteFlag)))
            {
            }

            break;
#endif

#if defined DEBUG_CONSOLE_IO_LPSCI
        case DEBUG_CONSOLE_DEVICE_TYPE_LPSCI:
            /* wait transfer complete flag */
            while (!(LPSCI_GetStatusFlags(s_debugConsoleIO.ioBase) & kLPSCI_TransmissionCompleteFlag))
            {
            }

            break;
#endif

#if defined DEBUG_CONSOLE_IO_LPUART
        case DEBUG_CONSOLE_DEVICE_TYPE_LPUART:
            /* wait transfer complete flag */
            while (!(LPUART_GetStatusFlags(s_debugConsoleIO.ioBase) & kLPUART_TransmissionCompleteFlag))
            {
            }
            break;
#endif

#if (defined DEBUG_CONSOLE_IO_FLEXCOMM) || (defined DEBUG_CONSOLE_IO_VUSART)
        case DEBUG_CONSOLE_DEVICE_TYPE_FLEXCOMM:
        case DEBUG_CONSOLE_DEVICE_TYPE_VUSART:
            /* wait transfer complete flag */
            while (!(USART_GetStatusFlags(s_debugConsoleIO.ioBase) & kUSART_TxFifoEmptyFlag))
            {
            }
            break;
#endif
        default:
            break;
    }

    return kStatus_Success;
}

#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING

status_t IO_Transfer(uint8_t *ch, size_t size, bool tx)
{
    status_t status = kStatus_Fail;

    switch (s_debugConsoleIO.ioType)
    {
#if (defined DEBUG_CONSOLE_IO_UART) || (defined DEBUG_CONSOLE_IO_IUART)
        case DEBUG_CONSOLE_DEVICE_TYPE_UART:
        case DEBUG_CONSOLE_DEVICE_TYPE_IUART:
        {
            uart_transfer_t transfer = {0U};
            transfer.data = ch;
            transfer.dataSize = size;
            /* transfer data */
            if (tx)
            {
                status = UART_TransferSendNonBlocking(s_debugConsoleIO.ioBase, &s_ioUartHandler, &transfer);
            }
            else
            {
                status = UART_TransferReceiveNonBlocking(s_debugConsoleIO.ioBase, &s_ioUartHandler, &transfer, NULL);
            }
        }
        break;
#endif
#if defined DEBUG_CONSOLE_IO_LPSCI
        case DEBUG_CONSOLE_DEVICE_TYPE_LPSCI:
        {
            lpsci_transfer_t transfer = {0U};
            transfer.data = ch;
            transfer.dataSize = size;
            /* transfer data */
            if (tx)
            {
                status = LPSCI_TransferSendNonBlocking(s_debugConsoleIO.ioBase, &s_ioLpsciHandler, &transfer);
            }
            else
            {
                status = LPSCI_TransferReceiveNonBlocking(s_debugConsoleIO.ioBase, &s_ioLpsciHandler, &transfer, NULL);
            }
        }
        break;
#endif

#if defined DEBUG_CONSOLE_IO_LPUART
        case DEBUG_CONSOLE_DEVICE_TYPE_LPUART:
        {
            lpuart_transfer_t transfer = {0U};
            transfer.data = ch;
            transfer.dataSize = size;
            /* transfer data */
            if (tx)
            {
                status = LPUART_TransferSendNonBlocking(s_debugConsoleIO.ioBase, &s_ioLpuartHandler, &transfer);
            }
            else
            {
                status =
                    LPUART_TransferReceiveNonBlocking(s_debugConsoleIO.ioBase, &s_ioLpuartHandler, &transfer, NULL);
            }
        }
        break;
#endif

#if defined DEBUG_CONSOLE_IO_USBCDC
        case DEBUG_CONSOLE_DEVICE_TYPE_USBCDC:
        {
            if (tx)
            {
                USB_VcomWriteBlocking(s_debugConsoleIO.ioBase, ch, size);
            }
            else
            {
                USB_VcomReadBlocking(s_debugConsoleIO.ioBase, ch, size);
            }
        }
        break;
#endif

#if (defined DEBUG_CONSOLE_IO_FLEXCOMM) || (defined DEBUG_CONSOLE_IO_VUSART)
        case DEBUG_CONSOLE_DEVICE_TYPE_FLEXCOMM:
        case DEBUG_CONSOLE_DEVICE_TYPE_VUSART:
        {
            usart_transfer_t transfer = {0U};
            transfer.data = ch;
            transfer.dataSize = size;
            /* transfer data */
            if (tx)
            {
                status = USART_TransferSendNonBlocking(s_debugConsoleIO.ioBase, &s_ioUsartHandler, &transfer);
            }
            else
            {
                status = USART_TransferReceiveNonBlocking(s_debugConsoleIO.ioBase, &s_ioUsartHandler, &transfer, NULL);
            }
        }
        break;
#endif
        default:
            break;
    }

    return status;
}

status_t IO_TryReceiveCharacter(uint8_t *ch)
{
    status_t status = kStatus_Fail;
    uint32_t size = 1U;

    switch (s_debugConsoleIO.ioType)
    {
#if (defined DEBUG_CONSOLE_IO_UART) || (defined DEBUG_CONSOLE_IO_IUART)
        case DEBUG_CONSOLE_DEVICE_TYPE_UART:
        case DEBUG_CONSOLE_DEVICE_TYPE_IUART:
        {
            uart_transfer_t transfer = {0U};
            transfer.data = ch;
            transfer.dataSize = size;
            if (UART_TransferGetRxRingBufferLength(&s_ioUartHandler) >= size)
            {
                /* transfer data */
                status = UART_TransferReceiveNonBlocking(s_debugConsoleIO.ioBase, &s_ioUartHandler, &transfer, NULL);
            }
        }
        break;
#endif
#if defined DEBUG_CONSOLE_IO_LPSCI
        case DEBUG_CONSOLE_DEVICE_TYPE_LPSCI:
        {
            lpsci_transfer_t transfer = {0U};
            transfer.data = ch;
            transfer.dataSize = size;
            if (LPSCI_TransferGetRxRingBufferLength(&s_ioLpsciHandler) >= size)
            {
                /* transfer data */
                status = LPSCI_TransferReceiveNonBlocking(s_debugConsoleIO.ioBase, &s_ioLpsciHandler, &transfer, NULL);
            }
        }
        break;
#endif

#if defined DEBUG_CONSOLE_IO_LPUART
        case DEBUG_CONSOLE_DEVICE_TYPE_LPUART:
        {
            lpuart_transfer_t transfer = {0U};
            transfer.data = ch;
            transfer.dataSize = size;
            if (LPUART_TransferGetRxRingBufferLength(s_debugConsoleIO.ioBase, &s_ioLpuartHandler) >= size)
            {
                /* transfer data */
                status =
                    LPUART_TransferReceiveNonBlocking(s_debugConsoleIO.ioBase, &s_ioLpuartHandler, &transfer, NULL);
            }
        }
        break;
#endif

#if defined DEBUG_CONSOLE_IO_USBCDC
        case DEBUG_CONSOLE_DEVICE_TYPE_USBCDC:
            break;
#endif

#if (defined DEBUG_CONSOLE_IO_FLEXCOMM) || (defined DEBUG_CONSOLE_IO_VUSART)
        case DEBUG_CONSOLE_DEVICE_TYPE_FLEXCOMM:
        case DEBUG_CONSOLE_DEVICE_TYPE_VUSART:
        {
            usart_transfer_t transfer = {0U};
            transfer.data = ch;
            transfer.dataSize = size;
            if (USART_TransferGetRxRingBufferLength(&s_ioUsartHandler) >= size)
            {
                /* transfer data */
                status = USART_TransferReceiveNonBlocking(s_debugConsoleIO.ioBase, &s_ioUsartHandler, &transfer, NULL);
            }
        }
        break;
#endif
        default:
            break;
    }

    return status;
}

#else

status_t IO_Transfer(uint8_t *ch, size_t size, bool tx)
{
    status_t status = kStatus_Success;
    switch (s_debugConsoleIO.ioType)
    {
#if (defined DEBUG_CONSOLE_IO_UART) || (defined DEBUG_CONSOLE_IO_IUART)
        case DEBUG_CONSOLE_DEVICE_TYPE_UART:
        case DEBUG_CONSOLE_DEVICE_TYPE_IUART:
        {
            if (tx)
            {
                UART_WriteBlocking(s_debugConsoleIO.ioBase, ch, size);
            }
            else
            {
                status = UART_ReadBlocking(s_debugConsoleIO.ioBase, ch, size);
            }
        }
        break;
#endif
#if defined DEBUG_CONSOLE_IO_LPSCI
        case DEBUG_CONSOLE_DEVICE_TYPE_LPSCI:
        {
            if (tx)
            {
                LPSCI_WriteBlocking(s_debugConsoleIO.ioBase, ch, size);
            }
            else
            {
                status = LPSCI_ReadBlocking(s_debugConsoleIO.ioBase, ch, size);
            }
        }
        break;
#endif

#if defined DEBUG_CONSOLE_IO_LPUART
        case DEBUG_CONSOLE_DEVICE_TYPE_LPUART:
        {
            if (tx)
            {
                LPUART_WriteBlocking(s_debugConsoleIO.ioBase, ch, size);
            }
            else
            {
                status = LPUART_ReadBlocking(s_debugConsoleIO.ioBase, ch, size);
            }
        }
        break;
#endif

#if defined DEBUG_CONSOLE_IO_USBCDC
        case DEBUG_CONSOLE_DEVICE_TYPE_USBCDC:
        {
            if (tx)
            {
                USB_VcomWriteBlocking(s_debugConsoleIO.ioBase, ch, size);
            }
            else
            {
                status = USB_VcomReadBlocking(s_debugConsoleIO.ioBase, ch, size);
            }
        }
        break;
#endif

#if (defined DEBUG_CONSOLE_IO_FLEXCOMM) || (defined DEBUG_CONSOLE_IO_VUSART)
        case DEBUG_CONSOLE_DEVICE_TYPE_FLEXCOMM:
        case DEBUG_CONSOLE_DEVICE_TYPE_VUSART:
        {
            if (tx)
            {
                USART_WriteBlocking(s_debugConsoleIO.ioBase, ch, size);
            }
            else
            {
                status = USART_ReadBlocking(s_debugConsoleIO.ioBase, ch, size);
            }
        }
        break;
#endif
        default:
            status = kStatus_Fail;
            break;
    }

    return status;
}

#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */
