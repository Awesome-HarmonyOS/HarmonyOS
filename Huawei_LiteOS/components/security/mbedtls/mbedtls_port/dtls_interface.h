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

/*
 *  Simple DTLS client demonstration program
 *
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */
#ifndef DTLS_INTERFACE_H
#define DTLS_INTERFACE_H


#if !defined(MBEDTLS_CONFIG_FILE)
#include "los_mbedtls_config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#define mbedtls_printf     printf
#endif

#include "mbedtls/net_sockets.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#include "mbedtls/timing.h"

#ifdef __cplusplus
extern "C" {
#endif


#ifndef TLS_SHAKEHAND_TIMEOUT
#define TLS_SHAKEHAND_TIMEOUT 1000
#endif

typedef enum
{
    VERIFY_WITH_PSK = 0,
    VERIFY_WITH_CERT,
}verify_type_e;

typedef struct
{
    union
    {
        struct
        {
            const char *host;
            const char *port;
        }c;
        struct
        {
            const char *local_port;
        }s;
    }u;
    uint32_t timeout;
    int client_or_server;
    int udp_or_tcp;
    verify_type_e psk_or_cert;
    void (*step_notify)(void *param);
    void (*finish_notify)(void *param);
    void *param;
}dtls_shakehand_info_s;

typedef struct
{
    union
    {
        struct
        {
            const unsigned char *psk;
            uint32_t psk_len;
            const unsigned char *psk_identity;
        }p;
        struct
        {
            const unsigned char *ca_cert;
            uint32_t cert_len;
        }c;
    }v;
    verify_type_e psk_or_cert;
    int udp_or_tcp;
}dtls_establish_info_s;

void dtls_init(void);

mbedtls_ssl_context *dtls_ssl_new(dtls_establish_info_s *info, char plat_type);

int dtls_shakehand(mbedtls_ssl_context *ssl, const dtls_shakehand_info_s *info);

void dtls_ssl_destroy(mbedtls_ssl_context* ssl);

int dtls_write(mbedtls_ssl_context* ssl, const unsigned char* buf, size_t len);

int dtls_read(mbedtls_ssl_context* ssl, unsigned char* buf, size_t len, uint32_t timeout);

int dtls_accept( mbedtls_net_context *bind_ctx,
                            mbedtls_net_context *client_ctx,
                            void *client_ip, size_t buf_size, size_t *ip_len );

#ifdef __cplusplus
}
#endif

#endif
