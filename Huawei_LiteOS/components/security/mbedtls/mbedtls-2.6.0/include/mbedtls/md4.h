/**
 * \file md4.h
 *
 * \brief MD4 message digest algorithm (hash function)
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
#ifndef MBEDTLS_MD4_H
#define MBEDTLS_MD4_H

#if !defined(MBEDTLS_CONFIG_FILE)
#include "config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#include <stddef.h>
#include <stdint.h>

#if !defined(MBEDTLS_MD4_ALT)
// Regular implementation
//

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief          MD4 context structure
 */
typedef struct
{
    uint32_t total[2];          /*!< number of bytes processed  */
    uint32_t state[4];          /*!< intermediate digest state  */
    unsigned char buffer[64];   /*!< data block being processed */
}
mbedtls_md4_context;

/**
 * \brief          Initialize MD4 context
 *
 * \param ctx      MD4 context to be initialized
 */
void mbedtls_md4_init( mbedtls_md4_context *ctx );

/**
 * \brief          Clear MD4 context
 *
 * \param ctx      MD4 context to be cleared
 */
void mbedtls_md4_free( mbedtls_md4_context *ctx );

/**
 * \brief          Clone (the state of) an MD4 context
 *
 * \param dst      The destination context
 * \param src      The context to be cloned
 */
void mbedtls_md4_clone( mbedtls_md4_context *dst,
                        const mbedtls_md4_context *src );

/**
 * \brief          MD4 context setup
 *
 * \param ctx      context to be initialized
 */
void mbedtls_md4_starts( mbedtls_md4_context *ctx );

/**
 * \brief          MD4 process buffer
 *
 * \param ctx      MD4 context
 * \param input    buffer holding the  data
 * \param ilen     length of the input data
 */
void mbedtls_md4_update( mbedtls_md4_context *ctx, const unsigned char *input, size_t ilen );

/**
 * \brief          MD4 final digest
 *
 * \param ctx      MD4 context
 * \param output   MD4 checksum result
 */
void mbedtls_md4_finish( mbedtls_md4_context *ctx, unsigned char output[16] );

#ifdef __cplusplus
}
#endif

#else  /* MBEDTLS_MD4_ALT */
#include "md4_alt.h"
#endif /* MBEDTLS_MD4_ALT */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief          Output = MD4( input buffer )
 *
 * \param input    buffer holding the  data
 * \param ilen     length of the input data
 * \param output   MD4 checksum result
 */
void mbedtls_md4( const unsigned char *input, size_t ilen, unsigned char output[16] );

/**
 * \brief          Checkup routine
 *
 * \return         0 if successful, or 1 if the test failed
 */
int mbedtls_md4_self_test( int verbose );

/* Internal use */
void mbedtls_md4_process( mbedtls_md4_context *ctx, const unsigned char data[64] );

#ifdef __cplusplus
}
#endif

#endif /* mbedtls_md4.h */
