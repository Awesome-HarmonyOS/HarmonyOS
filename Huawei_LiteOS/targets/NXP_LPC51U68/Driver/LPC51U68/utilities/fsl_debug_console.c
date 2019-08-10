/*
 * This is a modified version of the file printf.c, which was distributed
 * by Motorola as part of the M5407C3BOOT.zip package used to initialize
 * the M5407C3 evaluation board.
 *
 * Copyright:
 *      1999-2000 MOTOROLA, INC. All Rights Reserved.
 *  You are hereby granted a copyright license to use, modify, and
 *  distribute the SOFTWARE so long as this entire notice is
 *  retained without alteration in any modified and/or redistributed
 *  versions, and that such modified versions are clearly identified
 *  as such. No licenses are granted by implication, estoppel or
 *  otherwise under any patents or trademarks of Motorola, Inc. This
 *  software is provided on an "AS IS" basis and without warranty.
 *
 *  To the maximum extent permitted by applicable law, MOTOROLA
 *  DISCLAIMS ALL WARRANTIES WHETHER EXPRESS OR IMPLIED, INCLUDING
 *  IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR
 *  PURPOSE AND ANY WARRANTY AGAINST INFRINGEMENT WITH REGARD TO THE
 *  SOFTWARE (INCLUDING ANY MODIFIED VERSIONS THEREOF) AND ANY
 *  ACCOMPANYING WRITTEN MATERIALS.
 *
 *  To the maximum extent permitted by applicable law, IN NO EVENT
 *  SHALL MOTOROLA BE LIABLE FOR ANY DAMAGES WHATSOEVER (INCLUDING
 *  WITHOUT LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS
 *  INTERRUPTION, LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY
 *  LOSS) ARISING OF THE USE OR INABILITY TO USE THE SOFTWARE.
 *
 *  Motorola assumes no responsibility for the maintenance and support
 *  of this software

 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
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

#include <stdarg.h>
#include <stdlib.h>
#if defined(__CC_ARM)
#include <stdio.h>
#endif

#include "fsl_debug_console.h"
#include "fsl_debug_console_conf.h"
#include "fsl_log.h"
#include "fsl_str.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief This is a printf call back function which is used to relocate the log to buffer
 * or print the log immediately when the local buffer is full.
 *
 * @param[in] buf   Buffer to store log.
 * @param[in] indicator Buffer index.
 * @param[in] val Target character to store.
 * @param[in] len length of the character
 *
 */
#if SDK_DEBUGCONSOLE
static void DbgConsole_RelocateLog(char *buf, int32_t *indicator, char val, int len);
#endif
/*******************************************************************************
 * Code
 ******************************************************************************/

/*************Code for DbgConsole Init, Deinit, Printf, Scanf *******************************/

/* See fsl_debug_console.h for documentation of this function. */
status_t DbgConsole_Init(uint32_t baseAddr, uint32_t baudRate, uint8_t device, uint32_t clkSrcFreq)
{
    assert(device != DEBUG_CONSOLE_DEVICE_TYPE_NONE);

    return LOG_Init(baseAddr, device, baudRate, clkSrcFreq);
}

/* See fsl_debug_console.h for documentation of this function. */
status_t DbgConsole_Deinit(void)
{
    /* LOG deinit */
    LOG_Deinit();

    return kStatus_Success;
}

status_t DbgConsole_Flush(void)
{
    /* wait log and io idle */
    return LOG_WaitIdle();
}

#if SDK_DEBUGCONSOLE
/* See fsl_debug_console.h for documentation of this function. */
int DbgConsole_Printf(const char *fmt_s, ...)
{
    va_list ap;
    int logLength = 0U, result = 0U;
    char printBuf[DEBUG_CONSOLE_PRINTF_MAX_LOG_LEN] = {0U};

    va_start(ap, fmt_s);
    /* format print log first */
    logLength = StrFormatPrintf(fmt_s, ap, printBuf, DbgConsole_RelocateLog);
    /* print log */
    result = LOG_Push((uint8_t *)printBuf, logLength);

    va_end(ap);

    return result;
}

/* See fsl_debug_console.h for documentation of this function. */
int DbgConsole_Putchar(int ch)
{
    /* print char */
    return LOG_Push((uint8_t *)&ch, 1U);
}

/* See fsl_debug_console.h for documentation of this function. */
int DbgConsole_Scanf(char *fmt_ptr, ...)
{
    va_list ap;
    int result;
    char scanfBuf[DEBUG_CONSOLE_SCANF_MAX_LOG_LEN + 1U] = {0U};

    /* scanf log */
    LOG_ReadLine((uint8_t *)scanfBuf, DEBUG_CONSOLE_SCANF_MAX_LOG_LEN);
    /* get va_list */
    va_start(ap, fmt_ptr);
    /* format scanf log */
    result = StrFormatScanf(scanfBuf, fmt_ptr, ap);

    va_end(ap);

    return result;
}

#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
status_t DbgConsole_TryGetchar(char *ch)
{
    if (NULL != ch)
    {
        return LOG_TryReadCharacter((uint8_t *)ch);
    }

    return kStatus_Fail;
}
#endif

/* See fsl_debug_console.h for documentation of this function. */
int DbgConsole_Getchar(void)
{
    uint8_t ch;

    /* Get char */
    LOG_ReadCharacter(&ch);

    return ch;
}

static void DbgConsole_RelocateLog(char *buf, int32_t *indicator, char val, int len)
{
    int i = 0;

    for (i = 0; i < len; i++)
    {
        if ((*indicator + 1) >= DEBUG_CONSOLE_PRINTF_MAX_LOG_LEN)
        {
            LOG_Push((uint8_t *)buf, *indicator);
            *indicator = 0U;
        }

        buf[*indicator] = val;
        (*indicator)++;
    }
}

#endif /* SDK_DEBUGCONSOLE */
/*************Code to support toolchain's printf, scanf *******************************/
/* These function __write and __read is used to support IAR toolchain to printf and scanf*/
#if (defined(__ICCARM__))
#pragma weak __write
size_t __write(int handle, const unsigned char *buffer, size_t size)
{
    if (buffer == 0)
    {
        /*
         * This means that we should flush internal buffers.  Since we don't we just return.
         * (Remember, "handle" == -1 means that all handles should be flushed.)
         */
        return 0;
    }

    /* This function only writes to "standard out" and "standard err" for all other file handles it returns failure. */
    if ((handle != 1) && (handle != 2))
    {
        return ((size_t)-1);
    }

    /* Send data. */
    LOG_Push((uint8_t *)buffer, 1U);

    return size;
}

#pragma weak __read
size_t __read(int handle, unsigned char *buffer, size_t size)
{
    /* This function only reads from "standard in", for all other file  handles it returns failure. */
    if (handle != 0)
    {
        return ((size_t)-1);
    }

    /* Receive data.*/
    LOG_ReadLine(buffer, size);

    return size;
}

/* support LPC Xpresso with RedLib */
#elif(defined(__REDLIB__))

#if (!SDK_DEBUGCONSOLE) && (defined(SDK_DEBUGCONSOLE_UART))
int __attribute__((weak)) __sys_write(int handle, char *buffer, int size)
{
    if (buffer == 0)
    {
        /* return -1 if error. */
        return -1;
    }

    /* This function only writes to "standard out" and "standard err" for all other file handles it returns failure. */
    if ((handle != 1) && (handle != 2))
    {
        return -1;
    }

    /* Send data. */
    LOG_Push((uint8_t *)buffer, size);

    return 0;
}

int __attribute__((weak)) __sys_readc(void)
{
    char tmp;

    /* Receive data. */
    LOG_ReadCharacter((uint8_t *)&tmp);

    return tmp;
}
#endif

/* These function __write and __read is used to support ARM_GCC, KDS, Atollic toolchains to printf and scanf*/
#elif(defined(__GNUC__))

#if ((defined(__GNUC__) && (!defined(__MCUXPRESSO))) || \
     (defined(__MCUXPRESSO) && (!SDK_DEBUGCONSOLE) && (defined(SDK_DEBUGCONSOLE_UART))))

int __attribute__((weak)) _write(int handle, char *buffer, int size)
{
    if (buffer == 0)
    {
        /* return -1 if error. */
        return -1;
    }

    /* This function only writes to "standard out" and "standard err" for all other file handles it returns failure. */
    if ((handle != 1) && (handle != 2))
    {
        return -1;
    }

    /* Send data. */
    LOG_Push((uint8_t *)buffer, size);

    return size;
}

int __attribute__((weak)) _read(int handle, char *buffer, int size)
{
    /* This function only reads from "standard in", for all other file handles it returns failure. */
    if (handle != 0)
    {
        return -1;
    }

    /* Receive data. */
    return LOG_ReadLine((uint8_t *)buffer, size);
}
#endif

/* These function fputc and fgetc is used to support KEIL toolchain to printf and scanf*/
#elif defined(__CC_ARM)
struct __FILE
{
    int handle;
    /*
     * Whatever you require here. If the only file you are using is standard output using printf() for debugging,
     * no file handling is required.
     */
};

/* FILE is typedef in stdio.h. */
#pragma weak __stdout
#pragma weak __stdin
FILE __stdout;
FILE __stdin;

#pragma weak fputc
int fputc(int ch, FILE *f)
{
    /* Send data. */
    return LOG_Push((uint8_t *)(&ch), 1);
}

#pragma weak fgetc
int fgetc(FILE *f)
{
    char ch;

    /* Receive data. */
    LOG_ReadCharacter((uint8_t *)&ch);

    return ch;
}
#endif /* __ICCARM__ */
