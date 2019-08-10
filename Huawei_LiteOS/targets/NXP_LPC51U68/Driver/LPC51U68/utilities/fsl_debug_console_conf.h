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
#ifndef _FSL_DEBUG_CONSOLE_CONF_H_
#define _FSL_DEBUG_CONSOLE_CONF_H_

/****************Debug console configuration********************/

/*! @brief If Non-blocking mode is needed, please define it at project setting,
* otherwise blocking mode is the default transfer mode.
* Warning: If you want to use non-blocking transfer,please make sure the corresponding
* IO interrupt is enable, otherwise there is no output.
* And non-blocking is combine with buffer, no matter bare-metal or rtos.
*/
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
/*! @brief define the transmit buffer length which is used to store the multi task log, buffer is enabled automatically
* when
* non-blocking transfer is using,
* This value will affect the RAM's ultilization, should be set per paltform's capability and software requirement.
* If it is configured too small, log maybe missed , because the log will not be
* buffered if the buffer is full, and the print will return immediately with -1.
* And this value should be multiple of 4 to meet memory alignment.
*
*/
#ifndef DEBUG_CONSOLE_TRANSMIT_BUFFER_LEN
#define DEBUG_CONSOLE_TRANSMIT_BUFFER_LEN (512U)
#endif /* DEBUG_CONSOLE_TRANSMIT_BUFFER_LEN */

/*! @brief define the receive buffer length which is used to store the user input, buffer is enabled automatically when
* non-blocking transfer is using,
* This value will affect the RAM's ultilization, should be set per paltform's capability and software requirement.
* If it is configured too small, log maybe missed, because buffer will be overwrited if buffer is too small.
* And this value should be multiple of 4 to meet memory alignment.
*
*/
#ifndef DEBUG_CONSOLE_RECEIVE_BUFFER_LEN
#define DEBUG_CONSOLE_RECEIVE_BUFFER_LEN (512U)
#endif /* DEBUG_CONSOLE_RECEIVE_BUFFER_LEN */

#else
#define DEBUG_CONSOLE_TRANSFER_BLOCKING
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */

/*!@ brief define the MAX log length debug console support , that is when you call printf("log", x);, the log
* length can not bigger than this value.
* This macro decide the local log buffer length, the buffer locate at stack, the stack maybe overflow if
* the buffer is too big and current task stack size not big enough.
*/
#ifndef DEBUG_CONSOLE_PRINTF_MAX_LOG_LEN
#define DEBUG_CONSOLE_PRINTF_MAX_LOG_LEN (128U)
#endif /* DEBUG_CONSOLE_PRINTF_MAX_LOG_LEN */

/*!@ brief define the buffer support buffer scanf log length, that is when you call scanf("log", &x);, the log
* length can not bigger than this value.
* As same as the DEBUG_CONSOLE_BUFFER_PRINTF_MAX_LOG_LEN.
*/
#ifndef DEBUG_CONSOLE_SCANF_MAX_LOG_LEN
#define DEBUG_CONSOLE_SCANF_MAX_LOG_LEN (20U)
#endif /* DEBUG_CONSOLE_SCANF_MAX_LOG_LEN */

/*! @brief Debug console synchronization
* User should not change these macro for synchronization mode, but add the
* corresponding synchronization mechanism per different software environment.
* Such as, if another RTOS is used,
* add:
*  #define DEBUG_CONSOLE_SYNCHRONIZATION_XXXX 3
* in this configuration file and implement the synchronization in fsl.log.c.
*/
/*! @brief synchronization for baremetal software */
#define DEBUG_CONSOLE_SYNCHRONIZATION_BM 0
/*! @brief synchronization for freertos software */
#define DEBUG_CONSOLE_SYNCHRONIZATION_FREERTOS 1

/*! @brief RTOS synchronization mechanism disable
* If not defined, default is enable, to avoid multitask log print mess.
* If other RTOS is used, you can implement the RTOS's specific synchronization mechanism in fsl.log.c
* If synchronization is disabled, log maybe messed on terminal.
*/
#ifndef DEBUG_CONSOLE_DISABLE_RTOS_SYNCHRONIZATION
#ifdef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
#ifdef FSL_RTOS_FREE_RTOS
#define DEBUG_CONSOLE_SYNCHRONIZATION_MODE DEBUG_CONSOLE_SYNCHRONIZATION_FREERTOS
#else
#define DEBUG_CONSOLE_SYNCHRONIZATION_MODE DEBUG_CONSOLE_SYNCHRONIZATION_BM
#endif /* FSL_RTOS_FREE_RTOS */
#else
#define DEBUG_CONSOLE_SYNCHRONIZATION_MODE DEBUG_CONSOLE_SYNCHRONIZATION_BM
#endif /* DEBUG_CONSOLE_TRANSFER_NON_BLOCKING */
#endif /* DEBUG_CONSOLE_DISABLE_RTOS_SYNCHRONIZATION */

/*! @brief echo function support
* If you want to use the echo function,please define DEBUG_CONSOLE_ENABLE_ECHO
* at your project setting.
*/
#ifndef DEBUG_CONSOLE_ENABLE_ECHO
#define DEBUG_CONSOLE_ENABLE_ECHO_FUNCTION 0
#else
#define DEBUG_CONSOLE_ENABLE_ECHO_FUNCTION 1
#endif /* DEBUG_CONSOLE_ENABLE_ECHO */

/*********************************************************************/

/***************Debug console other configuration*********************/
/*! @brief Definition to printf the float number. */
#ifndef PRINTF_FLOAT_ENABLE
#define PRINTF_FLOAT_ENABLE 0U
#endif /* PRINTF_FLOAT_ENABLE */

/*! @brief Definition to scanf the float number. */
#ifndef SCANF_FLOAT_ENABLE
#define SCANF_FLOAT_ENABLE 0U
#endif /* SCANF_FLOAT_ENABLE */

/*! @brief Definition to support advanced format specifier for printf. */
#ifndef PRINTF_ADVANCED_ENABLE
#define PRINTF_ADVANCED_ENABLE 0U
#endif /* PRINTF_ADVANCED_ENABLE */

/*! @brief Definition to support advanced format specifier for scanf. */
#ifndef SCANF_ADVANCED_ENABLE
#define SCANF_ADVANCED_ENABLE 0U
#endif /* SCANF_ADVANCED_ENABLE */

/*******************************************************************/

#endif /* _FSL_DEBUG_CONSOLE_CONF_H_ */
