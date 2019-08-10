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
 *  Minimal configuration for TLS 1.2 with PSK and AES-CCM ciphersuites
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
/*
 * Minimal configuration for TLS 1.2 with PSK and AES-CCM ciphersuites
 * Distinguishing features:
 * - no bignum, no PK, no X509
 * - fully modern and secure (provided the pre-shared keys have high entropy)
 * - very low record overhead with CCM-8
 * - optimized for low RAM usage
 *
 * See README.txt for usage instructions.
 */
#ifndef MBEDTLS_CONFIG_H
#define MBEDTLS_CONFIG_H

/* System support */
//#define MBEDTLS_HAVE_TIME /* Optionally used in Hello messages */
/* Other MBEDTLS_HAVE_XXX flags irrelevant for this configuration */

/* mbed TLS feature support */
#define MBEDTLS_KEY_EXCHANGE_PSK_ENABLED
#define MBEDTLS_SSL_PROTO_TLS1_2
#define MBEDTLS_SSL_PROTO_DTLS
#define MBEDTLS_SSL_DTLS_HELLO_VERIFY

/* mbed TLS modules */
#define MBEDTLS_AES_C
#define MBEDTLS_CCM_C
#define MBEDTLS_CIPHER_C
#define MBEDTLS_CTR_DRBG_C
#define MBEDTLS_ENTROPY_C
#define MBEDTLS_MD_C
//#define MBEDTLS_NET_C
#define MBEDTLS_SHA256_C
#define MBEDTLS_SSL_CLI_C
#define MBEDTLS_SSL_SRV_C
#define MBEDTLS_SSL_TLS_C
#if !defined ( __GNUC__ )  /* GCC*/
#define MBEDTLS_DEBUG_C
#endif

#ifdef USE_MRVL_SDIO_WIFI
#define MBEDTLS_ARC4_C
#define MBEDTLS_MD5_C
#define MBEDTLS_SHA1_C
#endif

#define MBEDTLS_PLATFORM_C
#define MBEDTLS_ENTROPY_HARDWARE_ALT
#define MBEDTLS_NO_PLATFORM_ENTROPY
#define MBEDTLS_PLATFORM_PRINTF_ALT
#define MBEDTLS_PLATFORM_SNPRINTF_ALT

#define MBEDTLS_PLATFORM_MEMORY
//#define MBEDTLS_PLATFORM_CALLOC_MACRO        calloc /**< Default allocator macro to use, can be undefined */
//#define MBEDTLS_PLATFORM_FREE_MACRO            free /**< Default free macro to use, can be undefined */

/* Save RAM at the expense of ROM */
#if !defined ( __GNUC__ )  /* GCC*/
#define MBEDTLS_AES_ROM_TABLES
#endif

/* Save some RAM by adjusting to your exact needs */
#define MBEDTLS_PSK_MAX_LEN    32 /* 128-bits keys are generally enough */

/*
 * You should adjust this to the exact number of sources you're using: default
 * is the "platform_entropy_poll" source, but you may want to add other ones
 * Minimum is 2 for the entropy test suite.
 */
#define MBEDTLS_ENTROPY_MAX_SOURCES 1

/*
 * Save RAM at the expense of interoperability: do this only if you control
 * both ends of the connection!  (See comments in "mbedtls/ssl.h".)
 * The optimal size here depends on the typical size of records.
 */
#define MBEDTLS_SSL_MAX_CONTENT_LEN             5000
#define MBEDTLS_SSL_MAX_FRAGMENT_LENGTH
#define MBEDTLS_SSL_DTLS_ANTI_REPLAY
#define MBEDTLS_SSL_ALL_ALERT_MESSAGES
#define MBEDTLS_SSL_RENEGOTIATION
#define MBEDTLS_SSL_CACHE_C
#define MBEDTLS_CIPHER_PADDING_ZEROS_AND_LEN
#if 0 // We should support two encryption algorithm
#define MBEDTLS_CCM_C
#define MBEDTLS_SSL_CIPHERSUITES                        \
        MBEDTLS_TLS_PSK_WITH_AES_128_CCM_8
#define MBEDTLS_WITH_ONLY_AEAD_CHIPERS
#define MBEDTLS_ONLY_CCM_8_CHIPERS
#else
#define MBEDTLS_CIPHER_MODE_CBC
#define MBEDTLS_SSL_CIPHERSUITES                        \
        MBEDTLS_TLS_PSK_WITH_AES_128_CBC_SHA256
#endif

#if defined(CONFIG_FEATURE_FOTA) ||  defined(WITH_SOTA)
#define MBEDTLS_RSA_C
#define MBEDTLS_BIGNUM_C
#define MBEDTLS_OID_C
#define MBEDTLS_PKCS1_V21
#define MBEDTLS_ASN1_PARSE_C
#endif

#include "mbedtls/check_config.h"

#endif /* MBEDTLS_CONFIG_H */
