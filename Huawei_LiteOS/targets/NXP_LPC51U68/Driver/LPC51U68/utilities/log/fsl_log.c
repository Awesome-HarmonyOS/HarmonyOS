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
#include "fsl_log.h"
#include "fsl_debug_console_conf.h"
#include "fsl_io.h"
#ifdef FSL_RTOS_FREE_RTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#endif
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifndef BACKSPACE
/*! @brief character backspace ASCII value */
#define BACKSPACE 127
#endif

#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
/*! @brief increase pop member */
#define LOG_CHECK_BUFFER_INDEX_OVERFLOW(index)          \
    {                                                   \
        if (index >= DEBUG_CONSOLE_TRANSMIT_BUFFER_LEN) \
        {                                               \
            index -= DEBUG_CONSOLE_TRANSMIT_BUFFER_LEN; \
        }                                               \
    \
\
}

/*! @brief get current runing environment is ISR or not */
#ifdef __CA7_REV
#define IS_RUNNING_IN_ISR() SystemGetIRQNestingLevel()
#else
#define IS_RUNNING_IN_ISR() __get_IPSR()
#endif /* __CA7_REV */

#else
#define IS_RUNNING_IN_ISR() (0U)
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */

/* define for rtos */
#if (DEBUG_CONSOLE_SYNCHRONIZATION_MODE == DEBUG_CONSOLE_SYNCHRONIZATION_FREERTOS)
/* metex semaphore */
#define LOG_CREATE_MUTEX_SEMAPHORE(mutex) (mutex = xSemaphoreCreateMutex())

#define LOG_GIVE_MUTEX_SEMAPHORE(mutex) \
    \
{                                \
        if (IS_RUNNING_IN_ISR() == 0U)  \
        {                               \
            xSemaphoreGive(mutex);      \
        }                               \
    \
}

#define LOG_TAKE_MUTEX_SEMAPHORE_BLOCKING(mutex)  \
    \
{                                          \
        if (IS_RUNNING_IN_ISR() == 0U)            \
        {                                         \
            xSemaphoreTake(mutex, portMAX_DELAY); \
        }                                         \
    \
}

#define LOG_TAKE_MUTEX_SEMAPHORE_NONBLOCKING(mutex, result) \
    \
{                                                    \
        if (IS_RUNNING_IN_ISR() == 0U)                      \
        {                                                   \
            result = xSemaphoreTake(mutex, 0U);             \
        }                                                   \
        else                                                \
        {                                                   \
            result = 1U;                                    \
        }                                                   \
    \
}

/* Binary semaphore */
#define LOG_CREATE_BINARY_SEMAPHORE(binary) (binary = xSemaphoreCreateBinary())
#define LOG_TAKE_BINARY_SEMAPHORE_BLOCKING(binary) (xSemaphoreTake(binary, portMAX_DELAY))
#define LOG_GIVE_BINARY_SEMAPHORE_FROM_ISR(binary) (xSemaphoreGiveFromISR(binary, NULL))

#elif(DEBUG_CONSOLE_SYNCHRONIZATION_MODE == DEBUG_CONSOLE_SYNCHRONIZATION_BM)

#define LOG_CREATE_MUTEX_SEMAPHORE(mutex)
#define LOG_TAKE_MUTEX_SEMAPHORE_BLOCKING(mutex)
#define LOG_GIVE_MUTEX_SEMAPHORE(mutex)
#define LOG_CREATE_BINARY_SEMAPHORE(binary)
#define LOG_TAKE_MUTEX_SEMAPHORE_NONBLOCKING(mutex, result) (result = 1U)
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
#define LOG_TAKE_BINARY_SEMAPHORE_BLOCKING(binary) \
    \
{                                           \
        while (!binary)                            \
            ;                                      \
        binary = false;                            \
    \
\
}
#define LOG_GIVE_BINARY_SEMAPHORE_FROM_ISR(binary) (binary = true)
#else
#define LOG_TAKE_BINARY_SEMAPHORE_BLOCKING(binary)
#define LOG_GIVE_BINARY_SEMAPHORE_FROM_ISR(binary)
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */

/* add other implementation here
*such as :
* #elif(DEBUG_CONSOLE_SYNCHRONIZATION_MODE == DEBUG_CONSOLE_SYNCHRONIZATION_xxx)
*/

#else

#define LOG_CREATE_MUTEX_SEMAPHORE(mutex)
#define LOG_TAKE_MUTEX_SEMAPHORE_BLOCKING(mutex)
#define LOG_TAKE_MUTEX_SEMAPHORE_NONBLOCKING(mutex, result) (result = 1U)
#define LOG_GIVE_MUTEX_SEMAPHORE(mutex)
#define LOG_CREATE_BINARY_SEMAPHORE(binary)
#define LOG_TAKE_BINARY_SEMAPHORE_BLOCKING(binary)
#endif /* DEBUG_CONSOLE_SYNCHRONIZATION_MODE == DEBUG_CONSOLE_SYNCHRONIZATION_FREERTOS */

#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
/*! @brief Define the buffer
* The total buffer size should be calucate as (BUFFER_SUPPORT_LOG_LENGTH + 1) * BUFFER_SUPPORT_LOG_NUM * 4
*/
typedef struct _log_buffer
{
    volatile uint16_t totalIndex;                     /*!< indicate the total usage of the buffer */
    volatile uint16_t pushIndex;                      /*!< indicate the next push index */
    volatile uint16_t popIndex;                       /*!< indicate the pop index */
    uint8_t txBuf[DEBUG_CONSOLE_TRANSMIT_BUFFER_LEN]; /*!< buffer to store printf log */

    uint8_t rxBuf[DEBUG_CONSOLE_RECEIVE_BUFFER_LEN]; /*!< buffer to store scanf log */
} log_buffer_t;
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */

/*******************************************************************************
 * Variables
 ******************************************************************************/
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
/* A global log buffer */
static log_buffer_t s_log_buffer;
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */

/* lock definition */
#if (DEBUG_CONSOLE_SYNCHRONIZATION_MODE == DEBUG_CONSOLE_SYNCHRONIZATION_FREERTOS)
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
static SemaphoreHandle_t s_logPushSemaphore = NULL;
static SemaphoreHandle_t s_logReadSemaphore = NULL;
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */
static SemaphoreHandle_t s_logPopSemaphore = NULL;
static SemaphoreHandle_t s_logReadWaitSemaphore = NULL;

#elif(DEBUG_CONSOLE_SYNCHRONIZATION_MODE == DEBUG_CONSOLE_SYNCHRONIZATION_BM)

#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING

static volatile bool s_logReadWaitSemaphore = false; /* transferred event from ISR for bare-metal + interrupt */

#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */

#else
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */

/*******************************************************************************
* Prototypes
******************************************************************************/
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
/*!
 * @brief callback function for IO layer to notify LOG
 *
 * @param size last transfer data size
 * @param receive indicate a RX transfer
 * @param transmit indicate a TX transfer
 *
 */
static void LOG_Transferred(size_t *size, bool receive, bool transmit);

/*!
 * @brief log push function
 *
 * @param buf target buffer
 * @param size log size
 *
 */
static int LOG_BufPush(uint8_t *buf, size_t size);

/*!
 * @brief Get next avaliable log
 *
 * @param next avaliable size
 * @return next avaliable address
 */
static uint8_t *LOG_BufGetNextAvaliableLog(size_t *size);

/*!
 * @brief buf pop
 *
 * @param size log size popped and next available log size
 * @return next avaliable address
 */
static uint8_t *LOG_BufPop(size_t *size);

#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */

/*!
 * @brief read one character
 *
 * @param ch character address
 * @return indicate the read status
 *
 */
static status_t LOG_ReadOneCharacter(uint8_t *ch);

#if DEBUG_CONSOLE_ENABLE_ECHO_FUNCTION
/*!
 * @brief echo one character
 *
 * @param ch character address
 * @param isGetchar flag to distinguish getchar from scanf
 * @param index special for scanf to support backspace
 * @return indicate the read status
 *
 */
static status_t LOG_EchoCharacter(uint8_t *ch, bool isGetChar, int *index);
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
status_t LOG_Init(uint32_t baseAddr, uint8_t device, uint32_t baudRate, uint32_t clkSrcFreq)
{
    io_state_t io;
    /* init io */
    io.ioBase = (void *)baseAddr;
    io.ioType = device;

#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
    /* memset the global queue */
    memset(&s_log_buffer, 0U, sizeof(s_log_buffer));
    /* init callback for NON-BLOCKING */
    io.callBack = LOG_Transferred;
    /* io init function */
    IO_Init(&io, baudRate, clkSrcFreq, s_log_buffer.rxBuf);
    /* Debug console buffer push lock create */
    LOG_CREATE_MUTEX_SEMAPHORE(s_logPushSemaphore);
    /* Debug console get/scanf mutex lock create */
    LOG_CREATE_MUTEX_SEMAPHORE(s_logReadSemaphore);
#else
    IO_Init(&io, baudRate, clkSrcFreq, NULL);
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */

    /* Debug console lock create */
    LOG_CREATE_MUTEX_SEMAPHORE(s_logPopSemaphore);
    LOG_CREATE_BINARY_SEMAPHORE(s_logReadWaitSemaphore);

    return kStatus_Success;
}

void LOG_Deinit(void)
{
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
    /* memset the global queue */
    memset(&s_log_buffer, 0U, sizeof(s_log_buffer));
#endif /*DEBUG_CONSOLE_TRANSFER_NON_BLOCKING*/
    /* Deinit IO */
    IO_Deinit();
}

status_t LOG_WaitIdle(void)
{
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
    /* wait buffer empty */
    while (!(s_log_buffer.totalIndex == 0U))
        ;
#endif /*DEBUG_CONSOLE_TRANSFER_NON_BLOCKING*/
    /* wait IO idle */
    IO_WaitIdle();

    return kStatus_Success;
}

int LOG_Push(uint8_t *buf, size_t size)
{
    assert(buf != NULL);

#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
    /* push to buffer */
    LOG_BufPush(buf, size);
    buf = LOG_BufGetNextAvaliableLog(&size);
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */
    /* pop log */
    return LOG_Pop(buf, size);
}

int LOG_Pop(uint8_t *buf, size_t size)
{
    uint8_t getLock = 0U;

    if ((0 != size) && (NULL != buf))
    {
        /* take POP lock, should be non-blocking */
        LOG_TAKE_MUTEX_SEMAPHORE_NONBLOCKING(s_logPopSemaphore, getLock);

        if (getLock)
        {
            /* call IO transfer function */
            if (IO_Transfer(buf, size, true) != kStatus_Success)
            {
                size = 0U;
            }
            /* release POP lock */
            LOG_GIVE_MUTEX_SEMAPHORE(s_logPopSemaphore);
        }
    }

    return size;
}

int LOG_ReadLine(uint8_t *buf, size_t size)
{
    assert(buf != NULL);

    int i = 0;

    /* take mutex lock function */
    LOG_TAKE_MUTEX_SEMAPHORE_BLOCKING(s_logReadSemaphore);

    for (i = 0; i < size; i++)
    {
        /* recieve one char every time */
        if (LOG_ReadOneCharacter(&buf[i]) != kStatus_Success)
        {
            return -1;
        }
#if DEBUG_CONSOLE_ENABLE_ECHO_FUNCTION
        LOG_EchoCharacter(&buf[i], false, &i);
#endif
        /* analysis data */
        if ((buf[i] == '\r') || (buf[i] == '\n'))
        {
            /* End of Line. */
            if (i == 0)
            {
                buf[i] = '\0';
                i = -1;
            }
            else
            {
                break;
            }
        }
    }
    /* get char should not add '\0'*/
    if (i == size)
    {
        buf[i] = '\0';
    }
    else
    {
        buf[i + 1] = '\0';
    }

    /* release mutex lock function */
    LOG_GIVE_MUTEX_SEMAPHORE(s_logReadSemaphore);

    return i;
}

int LOG_ReadCharacter(uint8_t *ch)
{
    assert(ch != NULL);
    int ret = 0;

    /* take mutex lock function */
    LOG_TAKE_MUTEX_SEMAPHORE_BLOCKING(s_logReadSemaphore);
    /* read one character */
    if (LOG_ReadOneCharacter(ch) == kStatus_Success)
    {
        ret = 1;
#if DEBUG_CONSOLE_ENABLE_ECHO_FUNCTION
        LOG_EchoCharacter(ch, true, NULL);
#endif
    }
    else
    {
        ret = -1;
    }

    /* release mutex lock function */
    LOG_GIVE_MUTEX_SEMAPHORE(s_logReadSemaphore);

    return ret;
}

static status_t LOG_ReadOneCharacter(uint8_t *ch)
{
    /* recieve one char every time */
    if (IO_Transfer(ch, 1U, false) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    /* wait release from ISR */
    LOG_TAKE_BINARY_SEMAPHORE_BLOCKING(s_logReadWaitSemaphore);

    return kStatus_Success;
}

#if DEBUG_CONSOLE_ENABLE_ECHO_FUNCTION
static status_t LOG_EchoCharacter(uint8_t *ch, bool isGetChar, int *index)
{
    /* Due to scanf take \n and \r as end of string,should not echo */
    if (((*ch != '\r') && (*ch != '\n')) || (isGetChar))
    {
        /* recieve one char every time */
        if (IO_Transfer(ch, 1U, true) != kStatus_Success)
        {
            return kStatus_Fail;
        }
    }

    if (!isGetChar)
    {
        if ((*index > 0) && (*ch == BACKSPACE))
        {
            *index -= 2;
        }
    }

    return kStatus_Success;
}
#endif

#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
static int LOG_BufPush(uint8_t *buf, size_t size)
{
    uint32_t pushIndex = 0U, i = 0U;
    bool pushAvaliable = false;

    /* take mutex lock function */
    LOG_TAKE_MUTEX_SEMAPHORE_BLOCKING(s_logPushSemaphore);
    if (size <= (DEBUG_CONSOLE_TRANSMIT_BUFFER_LEN - s_log_buffer.totalIndex))
    {
        /* get push index */
        pushIndex = s_log_buffer.pushIndex;
        s_log_buffer.pushIndex += size;
        /* check index overflow */
        LOG_CHECK_BUFFER_INDEX_OVERFLOW(s_log_buffer.pushIndex);
        /* update push/total index value */
        s_log_buffer.totalIndex += size;
        pushAvaliable = true;
    }
    /* release mutex lock function */
    LOG_GIVE_MUTEX_SEMAPHORE(s_logPushSemaphore);

    /* check the buffer if have enough space to store the log */
    if (pushAvaliable)
    {
        for (i = size; i > 0; i--)
        {
            /* copy log to buffer, the buffer only support a fixed length argument, if the log argument
            is longer than the fixed length, the left argument will be losed */
            s_log_buffer.txBuf[pushIndex] = *buf++;
            /* increase index */
            pushIndex++;
            /* check index overflow */
            LOG_CHECK_BUFFER_INDEX_OVERFLOW(pushIndex);
        }
    }
    else
    {
        size = 0U;
    }

    return size;
}

static uint8_t *LOG_BufGetNextAvaliableLog(size_t *size)
{
    uint16_t popIndex = s_log_buffer.popIndex;

    /* get avaliable size */
    if (s_log_buffer.totalIndex > (DEBUG_CONSOLE_TRANSMIT_BUFFER_LEN - popIndex))
    {
        *size = (DEBUG_CONSOLE_TRANSMIT_BUFFER_LEN - popIndex);
    }
    else
    {
        *size = s_log_buffer.totalIndex;
    }

    /* return address */
    return (&(s_log_buffer.txBuf[popIndex]));
}

static uint8_t *LOG_BufPop(size_t *size)
{
    if (s_log_buffer.totalIndex >= *size)
    {
        /* decrease the log total member */
        s_log_buffer.totalIndex -= *size;
        /* there is more log in the queue to be pushed */
        if (s_log_buffer.totalIndex > 0U)
        {
            /* update the pop index */
            s_log_buffer.popIndex += *size;
            /* check index overflow */
            LOG_CHECK_BUFFER_INDEX_OVERFLOW(s_log_buffer.popIndex);

            return LOG_BufGetNextAvaliableLog(size);
        }
        else
        {
            /* reset push and pop */
            s_log_buffer.popIndex = 0U;
            s_log_buffer.pushIndex = 0U;
            *size = 0U;
        }
    }

    return NULL;
}

static void LOG_Transferred(size_t *size, bool receive, bool transmit)
{
    uint8_t *addr = NULL;

    if (transmit)
    {
        addr = LOG_BufPop(size);
        /* continue pop log from buffer */
        LOG_Pop(addr, *size);
    }

    if (receive)
    {
        /* release from ISR */
        LOG_GIVE_BINARY_SEMAPHORE_FROM_ISR(s_logReadWaitSemaphore);
    }
}
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */

#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
status_t LOG_TryReadCharacter(uint8_t *ch)
{
    if (NULL != ch)
    {
        return IO_TryReceiveCharacter(ch);
    }
    return kStatus_Fail;
}
#endif
