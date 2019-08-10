/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#ifndef _FSL_SHELL_H_
#define _FSL_SHELL_H_

#include "fsl_common.h"

/*!
 * @addtogroup SHELL
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Macro to set on/off history feature. */
#ifndef SHELL_USE_HISTORY
#define SHELL_USE_HISTORY (0U)
#endif

/*! @brief Macro to set on/off history feature. */
#ifndef SHELL_SEARCH_IN_HIST
#define SHELL_SEARCH_IN_HIST (1U)
#endif

/*! @brief Macro to select method stream. */
#ifndef SHELL_USE_FILE_STREAM
#define SHELL_USE_FILE_STREAM (0U)
#endif

/*! @brief Macro to set on/off auto-complete feature. */
#ifndef SHELL_AUTO_COMPLETE
#define SHELL_AUTO_COMPLETE (1U)
#endif

/*! @brief Macro to set console buffer size. */
#ifndef SHELL_BUFFER_SIZE
#define SHELL_BUFFER_SIZE (64U)
#endif

/*! @brief Macro to set maximum arguments in command. */
#ifndef SHELL_MAX_ARGS
#define SHELL_MAX_ARGS (8U)
#endif

/*! @brief Macro to set maximum count of history commands. */
#ifndef SHELL_HIST_MAX
#define SHELL_HIST_MAX (3U)
#endif

/*! @brief Macro to set maximum count of commands. */
#ifndef SHELL_MAX_CMD
#define SHELL_MAX_CMD (20U)
#endif

/*! @brief Macro to bypass arguments check */
#define SHELL_OPTIONAL_PARAMS (0xFF)

/*! @brief Shell user send data callback prototype.*/
typedef void (*send_data_cb_t)(uint8_t *buf, uint32_t len);

/*! @brief Shell user receiver data callback prototype.*/
typedef void (*recv_data_cb_t)(uint8_t *buf, uint32_t len);

/*! @brief Shell user printf data prototype.*/
typedef int (*printf_data_t)(const char *format, ...);

/*! @brief A type for the handle special key. */
typedef enum _fun_key_status
{
    kSHELL_Normal = 0U,   /*!< Normal key */
    kSHELL_Special = 1U,  /*!< Special key */
    kSHELL_Function = 2U, /*!< Function key */
} fun_key_status_t;

/*! @brief Data structure for Shell environment. */
typedef struct _shell_context_struct
{
    char *prompt;                 /*!< Prompt string */
    enum _fun_key_status stat;    /*!< Special key status */
    char line[SHELL_BUFFER_SIZE]; /*!< Consult buffer */
    uint8_t cmd_num;              /*!< Number of user commands */
    uint8_t l_pos;                /*!< Total line position */
    uint8_t c_pos;                /*!< Current line position */
#if SHELL_USE_FILE_STREAM
    FILE *STDOUT, *STDIN, *STDERR;
#else
    send_data_cb_t send_data_func; /*!< Send data interface operation */
    recv_data_cb_t recv_data_func; /*!< Receive data interface operation */
    printf_data_t printf_data_func;
#endif
    uint16_t hist_current;                            /*!< Current history command in hist buff*/
    uint16_t hist_count;                              /*!< Total history command in hist buff*/
    char hist_buf[SHELL_HIST_MAX][SHELL_BUFFER_SIZE]; /*!< History buffer*/
    bool exit;                                        /*!< Exit Flag*/
} shell_context_struct, *p_shell_context_t;

/*! @brief User command function prototype. */
typedef int32_t (*cmd_function_t)(p_shell_context_t context, int32_t argc, char **argv);

/*! @brief User command data structure. */
typedef struct _shell_command_context
{
    const char *pcCommand; /*!< The command that is executed.  For example "help".  It must be all lower case. */
    char *pcHelpString;    /*!< String that describes how to use the command.  It should start with the command itself,
                                    and end with "\r\n".  For example "help: Returns a list of all the commands\r\n". */
    const cmd_function_t
        pFuncCallBack; /*!< A pointer to the callback function that returns the output generated by the command. */
    uint8_t cExpectedNumberOfParameters; /*!< Commands expect a fixed number of parameters, which may be zero. */
} shell_command_context_t;

/*! @brief Structure list command. */
typedef struct _shell_command_context_list
{
    const shell_command_context_t *CommandList[SHELL_MAX_CMD]; /*!< The command table list */
    uint8_t numberOfCommandInList;                             /*!< The total command in list */
} shell_command_context_list_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* _cplusplus */

/*!
 * @name Shell functional operation
 * @{
 */

/*!
* @brief Enables the clock gate and configures the Shell module according to the configuration structure.
*
* This function must be called before calling all other Shell functions.
* Call operation the Shell commands with user-defined settings.
* The example below shows how to set up the middleware Shell and
* how to call the SHELL_Init function by passing in these parameters.
* This is an example.
* @code
*   shell_context_struct user_context;
*   SHELL_Init(&user_context, SendDataFunc, ReceiveDataFunc, "SHELL>> ");
* @endcode
* @param context The pointer to the Shell environment and  runtime states.
* @param send_cb The pointer to call back send data function.
* @param recv_cb The pointer to call back receive data function.
* @param prompt  The string prompt of Shell
*/
void SHELL_Init(p_shell_context_t context,
                send_data_cb_t send_cb,
                recv_data_cb_t recv_cb,
                printf_data_t shell_printf,
                char *prompt);

/*!
 * @brief Shell register command.
 * @param   command_context The pointer to the command data structure.
 * @return  -1 if error or 0 if success
 */
int32_t SHELL_RegisterCommand(const shell_command_context_t *command_context);

/*!
 * @brief Main loop for Shell.
 * Main loop for Shell; After this function is called, Shell begins to initialize the basic variables and starts to
 * work.
 * @param    context The pointer to the Shell environment and  runtime states.
 * @return   This function does not return until Shell command exit was called.
 */
int32_t SHELL_Main(p_shell_context_t context);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_SHELL_H_ */
