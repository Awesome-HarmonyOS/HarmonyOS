/**
 * \file ecdh.h
 *
 * \brief Elliptic curve Diffie-Hellman
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
#ifndef MBEDTLS_ECDH_H
#define MBEDTLS_ECDH_H

#include "ecp.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * When importing from an EC key, select if it is our key or the peer's key
 */
typedef enum
{
    MBEDTLS_ECDH_OURS,
    MBEDTLS_ECDH_THEIRS,
} mbedtls_ecdh_side;

/**
 * \brief           ECDH context structure
 */
typedef struct
{
    mbedtls_ecp_group grp;      /*!<  elliptic curve used                           */
    mbedtls_mpi d;              /*!<  our secret value (private key)                */
    mbedtls_ecp_point Q;        /*!<  our public value (public key)                 */
    mbedtls_ecp_point Qp;       /*!<  peer's public value (public key)              */
    mbedtls_mpi z;              /*!<  shared secret                                 */
    int point_format;   /*!<  format for point export in TLS messages       */
    mbedtls_ecp_point Vi;       /*!<  blinding value (for later)                    */
    mbedtls_ecp_point Vf;       /*!<  un-blinding value (for later)                 */
    mbedtls_mpi _d;             /*!<  previous d (for later)                        */
}
mbedtls_ecdh_context;

/**
 * \brief           Generate a public key.
 *                  Raw function that only does the core computation.
 *
 * \param grp       ECP group
 * \param d         Destination MPI (secret exponent, aka private key)
 * \param Q         Destination point (public key)
 * \param f_rng     RNG function
 * \param p_rng     RNG parameter
 *
 * \return          0 if successful,
 *                  or a MBEDTLS_ERR_ECP_XXX or MBEDTLS_MPI_XXX error code
 */
int mbedtls_ecdh_gen_public( mbedtls_ecp_group *grp, mbedtls_mpi *d, mbedtls_ecp_point *Q,
                     int (*f_rng)(void *, unsigned char *, size_t),
                     void *p_rng );

/**
 * \brief           Compute shared secret
 *                  Raw function that only does the core computation.
 *
 * \param grp       ECP group
 * \param z         Destination MPI (shared secret)
 * \param Q         Public key from other party
 * \param d         Our secret exponent (private key)
 * \param f_rng     RNG function (see notes)
 * \param p_rng     RNG parameter
 *
 * \return          0 if successful,
 *                  or a MBEDTLS_ERR_ECP_XXX or MBEDTLS_MPI_XXX error code
 *
 * \note            If f_rng is not NULL, it is used to implement
 *                  countermeasures against potential elaborate timing
 *                  attacks, see \c mbedtls_ecp_mul() for details.
 */
int mbedtls_ecdh_compute_shared( mbedtls_ecp_group *grp, mbedtls_mpi *z,
                         const mbedtls_ecp_point *Q, const mbedtls_mpi *d,
                         int (*f_rng)(void *, unsigned char *, size_t),
                         void *p_rng );

/**
 * \brief           Initialize context
 *
 * \param ctx       Context to initialize
 */
void mbedtls_ecdh_init( mbedtls_ecdh_context *ctx );

/**
 * \brief           Free context
 *
 * \param ctx       Context to free
 */
void mbedtls_ecdh_free( mbedtls_ecdh_context *ctx );

/**
 * \brief           Generate a public key and a TLS ServerKeyExchange payload.
 *                  (First function used by a TLS server for ECDHE.)
 *
 * \param ctx       ECDH context
 * \param olen      number of chars written
 * \param buf       destination buffer
 * \param blen      length of buffer
 * \param f_rng     RNG function
 * \param p_rng     RNG parameter
 *
 * \note            This function assumes that ctx->grp has already been
 *                  properly set (for example using mbedtls_ecp_group_load).
 *
 * \return          0 if successful, or an MBEDTLS_ERR_ECP_XXX error code
 */
int mbedtls_ecdh_make_params( mbedtls_ecdh_context *ctx, size_t *olen,
                      unsigned char *buf, size_t blen,
                      int (*f_rng)(void *, unsigned char *, size_t),
                      void *p_rng );

/**
 * \brief           Parse and procress a TLS ServerKeyExhange payload.
 *                  (First function used by a TLS client for ECDHE.)
 *
 * \param ctx       ECDH context
 * \param buf       pointer to start of input buffer
 * \param end       one past end of buffer
 *
 * \return          0 if successful, or an MBEDTLS_ERR_ECP_XXX error code
 */
int mbedtls_ecdh_read_params( mbedtls_ecdh_context *ctx,
                      const unsigned char **buf, const unsigned char *end );

/**
 * \brief           Setup an ECDH context from an EC key.
 *                  (Used by clients and servers in place of the
 *                  ServerKeyEchange for static ECDH: import ECDH parameters
 *                  from a certificate's EC key information.)
 *
 * \param ctx       ECDH constext to set
 * \param key       EC key to use
 * \param side      Is it our key (1) or the peer's key (0) ?
 *
 * \return          0 if successful, or an MBEDTLS_ERR_ECP_XXX error code
 */
int mbedtls_ecdh_get_params( mbedtls_ecdh_context *ctx, const mbedtls_ecp_keypair *key,
                     mbedtls_ecdh_side side );

/**
 * \brief           Generate a public key and a TLS ClientKeyExchange payload.
 *                  (Second function used by a TLS client for ECDH(E).)
 *
 * \param ctx       ECDH context
 * \param olen      number of bytes actually written
 * \param buf       destination buffer
 * \param blen      size of destination buffer
 * \param f_rng     RNG function
 * \param p_rng     RNG parameter
 *
 * \return          0 if successful, or an MBEDTLS_ERR_ECP_XXX error code
 */
int mbedtls_ecdh_make_public( mbedtls_ecdh_context *ctx, size_t *olen,
                      unsigned char *buf, size_t blen,
                      int (*f_rng)(void *, unsigned char *, size_t),
                      void *p_rng );

/**
 * \brief           Parse and process a TLS ClientKeyExchange payload.
 *                  (Second function used by a TLS server for ECDH(E).)
 *
 * \param ctx       ECDH context
 * \param buf       start of input buffer
 * \param blen      length of input buffer
 *
 * \return          0 if successful, or an MBEDTLS_ERR_ECP_XXX error code
 */
int mbedtls_ecdh_read_public( mbedtls_ecdh_context *ctx,
                      const unsigned char *buf, size_t blen );

/**
 * \brief           Derive and export the shared secret.
 *                  (Last function used by both TLS client en servers.)
 *
 * \param ctx       ECDH context
 * \param olen      number of bytes written
 * \param buf       destination buffer
 * \param blen      buffer length
 * \param f_rng     RNG function, see notes for \c mbedtls_ecdh_compute_shared()
 * \param p_rng     RNG parameter
 *
 * \return          0 if successful, or an MBEDTLS_ERR_ECP_XXX error code
 */
int mbedtls_ecdh_calc_secret( mbedtls_ecdh_context *ctx, size_t *olen,
                      unsigned char *buf, size_t blen,
                      int (*f_rng)(void *, unsigned char *, size_t),
                      void *p_rng );

#ifdef __cplusplus
}
#endif

#endif /* ecdh.h */
