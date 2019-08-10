/*----------------------------------------------------------------------------
 * Copyright (c) <2016-2018>, <Huawei Technologies Co., Ltd>
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or other materials
 * provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific prior written
 * permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *---------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------
 * Notice of Export Control Law
 * ===============================================
 * Huawei LiteOS may be subject to applicable export control laws and regulations, which might
 * include those applicable to Huawei LiteOS of U.S. and the country in which you are located.
 * Import, export and usage of Huawei LiteOS in any manner by you shall be in compliance with such
 * applicable export control laws and regulations.
 *---------------------------------------------------------------------------*/

#ifndef _ATINY_CONTEXT_H
#define _ATINY_CONTEXT_H
#include "liblwm2m.h"
#include "atiny_lwm2m/agenttiny.h"
#include "object_comm.h"

#define SERVER_URI_MAX_LEN      (64)
#define MAX_PACKET_SIZE         (1024)
#define SERVER_ID               (123)
#define BIND_TIMEOUT            (10)


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/**

 *@ingroup handle_data_t

 *Structure of agentiny hanle.

 */
 
 enum
{
    OBJ_SECURITY_INDEX = 0,
    OBJ_SERVER_INDEX,
    OBJ_ACCESS_CONTROL_INDEX,
    OBJ_DEVICE_INDEX,
    OBJ_CONNECT_INDEX,
    OBJ_FIRMWARE_INDEX,
    OBJ_LOCATION_INDEX,
    OBJ_APP_INDEX,
    OBJ_MAX_NUM,
};
 
typedef struct
{
    lwm2m_context_t  *lwm2m_context;
    atiny_param_t     atiny_params;
    client_data_t     client_data;
    lwm2m_object_t   *obj_array[OBJ_MAX_NUM];
    int atiny_quit;
    int reconnect_flag;
    void *quit_sem;
    int reboot_flag;
    uint8_t *recv_buffer;
} handle_data_t;


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _ATINY_INNER_H */
