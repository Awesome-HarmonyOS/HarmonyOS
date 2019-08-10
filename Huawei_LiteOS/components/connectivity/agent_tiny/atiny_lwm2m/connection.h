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

/*******************************************************************************
 *
 * Copyright (c) 2015 Intel Corporation and others.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * The Eclipse Distribution License is available at
 *    http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Simon Bernard - initial API and implementation
 *    Christian Renz - Please refer to git log
 *
 *******************************************************************************/

#ifndef DTLS_CONN_H_
#define DTLS_CONN_H_
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "util_timer.h"
#include "liblwm2m.h"


// after 40sec of inactivity we rehandshake
#define DTLS_NAT_TIMEOUT 40

#define LWM2M_IS_CLIENT                   0
#define LWM2M_IS_SERVER                   1

/* timeout for udp client shakehand,the unit is second */
#ifndef DTLS_UDP_CLIENT_SHAKEHAND_TIMEOUT
#define DTLS_UDP_CLIENT_SHAKEHAND_TIMEOUT 60
#endif

typedef enum
{
    CONNECTION_SEND_ERR,
    CONNECTION_RECV_ERR,
    CONNECTION_ERR_MAX
} connection_err_e;

typedef struct _connection_t
{
    struct _connection_t*   next;
    void* net_context;
    lwm2m_object_t* securityObj;
    int securityInstId;
    uint16_t dtls_flag;
    uint16_t bootstrap_flag;
    lwm2m_context_t* lwm2mH;
    uint16_t errs[CONNECTION_ERR_MAX];
#ifdef LWM2M_BOOTSTRAP
    util_timer_t server_triger_timer;
#endif
} connection_t;

typedef void (*lwm2m_connection_err_notify_t)(lwm2m_context_t* context, connection_err_e err_type, bool boostrap_flag);


int lwm2m_buffer_recv(void* sessionH, uint8_t* buffer, size_t length, uint32_t timeout);
void lwm2m_register_connection_err_notify(lwm2m_connection_err_notify_t nofiy);


#ifdef LWM2M_BOOTSTRAP
void lwm2m_step_striger_server_initiated_bs(connection_t * sessionH);
void lwm2m_stop_striger_server_initiated_bs(connection_t * sessionH);
bool lwm2m_is_sec_obj_uri_valid(uint16_t secObjInstID, void *userData);
#endif

#endif
