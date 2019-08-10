/**
 * \file dhm.h
 *
 * \brief Diffie-Hellman-Merkle key exchange
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
#ifndef MBEDTLS_DHM_H
#define MBEDTLS_DHM_H

#include "bignum.h"

/*
 * DHM Error codes
 */
#define MBEDTLS_ERR_DHM_BAD_INPUT_DATA                    -0x3080  /**< Bad input parameters to function. */
#define MBEDTLS_ERR_DHM_READ_PARAMS_FAILED                -0x3100  /**< Reading of the DHM parameters failed. */
#define MBEDTLS_ERR_DHM_MAKE_PARAMS_FAILED                -0x3180  /**< Making of the DHM parameters failed. */
#define MBEDTLS_ERR_DHM_READ_PUBLIC_FAILED                -0x3200  /**< Reading of the public values failed. */
#define MBEDTLS_ERR_DHM_MAKE_PUBLIC_FAILED                -0x3280  /**< Making of the public value failed. */
#define MBEDTLS_ERR_DHM_CALC_SECRET_FAILED                -0x3300  /**< Calculation of the DHM secret failed. */
#define MBEDTLS_ERR_DHM_INVALID_FORMAT                    -0x3380  /**< The ASN.1 data is not formatted correctly. */
#define MBEDTLS_ERR_DHM_ALLOC_FAILED                      -0x3400  /**< Allocation of memory failed. */
#define MBEDTLS_ERR_DHM_FILE_IO_ERROR                     -0x3480  /**< Read/write of file failed. */

/**
 * RFC 3526 defines a number of standardized Diffie-Hellman groups
 * for IKE.
 * RFC 5114 defines a number of standardized Diffie-Hellman groups
 * that can be used.
 *
 * Some are included here for convenience.
 *
 * Included are:
 *  RFC 3526 3.    2048-bit MODP Group
 *  RFC 3526 4.    3072-bit MODP Group
 *  RFC 3526 5.    4096-bit MODP Group
 *  RFC 5114 2.2.  2048-bit MODP Group with 224-bit Prime Order Subgroup
 */
#define MBEDTLS_DHM_RFC3526_MODP_2048_P               \
    "FFFFFFFFFFFFFFFFC90FDAA22168C234C4C6628B80DC1CD1" \
    "29024E088A67CC74020BBEA63B139B22514A08798E3404DD" \
    "EF9519B3CD3A431B302B0A6DF25F14374FE1356D6D51C245" \
    "E485B576625E7EC6F44C42E9A637ED6B0BFF5CB6F406B7ED" \
    "EE386BFB5A899FA5AE9F24117C4B1FE649286651ECE45B3D" \
    "C2007CB8A163BF0598DA48361C55D39A69163FA8FD24CF5F" \
    "83655D23DCA3AD961C62F356208552BB9ED529077096966D" \
    "670C354E4ABC9804F1746C08CA18217C32905E462E36CE3B" \
    "E39E772C180E86039B2783A2EC07A28FB5C55DF06F4C52C9" \
    "DE2BCBF6955817183995497CEA956AE515D2261898FA0510" \
    "15728E5A8AACAA68FFFFFFFFFFFFFFFF"

#define MBEDTLS_DHM_RFC3526_MODP_2048_G          "02"

#define MBEDTLS_DHM_RFC3526_MODP_3072_P               \
    "FFFFFFFFFFFFFFFFC90FDAA22168C234C4C6628B80DC1CD1" \
    "29024E088A67CC74020BBEA63B139B22514A08798E3404DD" \
    "EF9519B3CD3A431B302B0A6DF25F14374FE1356D6D51C245" \
    "E485B576625E7EC6F44C42E9A637ED6B0BFF5CB6F406B7ED" \
    "EE386BFB5A899FA5AE9F24117C4B1FE649286651ECE45B3D" \
    "C2007CB8A163BF0598DA48361C55D39A69163FA8FD24CF5F" \
    "83655D23DCA3AD961C62F356208552BB9ED529077096966D" \
    "670C354E4ABC9804F1746C08CA18217C32905E462E36CE3B" \
    "E39E772C180E86039B2783A2EC07A28FB5C55DF06F4C52C9" \
    "DE2BCBF6955817183995497CEA956AE515D2261898FA0510" \
    "15728E5A8AAAC42DAD33170D04507A33A85521ABDF1CBA64" \
    "ECFB850458DBEF0A8AEA71575D060C7DB3970F85A6E1E4C7" \
    "ABF5AE8CDB0933D71E8C94E04A25619DCEE3D2261AD2EE6B" \
    "F12FFA06D98A0864D87602733EC86A64521F2B18177B200C" \
    "BBE117577A615D6C770988C0BAD946E208E24FA074E5AB31" \
    "43DB5BFCE0FD108E4B82D120A93AD2CAFFFFFFFFFFFFFFFF"

#define MBEDTLS_DHM_RFC3526_MODP_3072_G          "02"

#define MBEDTLS_DHM_RFC3526_MODP_4096_P                \
    "FFFFFFFFFFFFFFFFC90FDAA22168C234C4C6628B80DC1CD1" \
    "29024E088A67CC74020BBEA63B139B22514A08798E3404DD" \
    "EF9519B3CD3A431B302B0A6DF25F14374FE1356D6D51C245" \
    "E485B576625E7EC6F44C42E9A637ED6B0BFF5CB6F406B7ED" \
    "EE386BFB5A899FA5AE9F24117C4B1FE649286651ECE45B3D" \
    "C2007CB8A163BF0598DA48361C55D39A69163FA8FD24CF5F" \
    "83655D23DCA3AD961C62F356208552BB9ED529077096966D" \
    "670C354E4ABC9804F1746C08CA18217C32905E462E36CE3B" \
    "E39E772C180E86039B2783A2EC07A28FB5C55DF06F4C52C9" \
    "DE2BCBF6955817183995497CEA956AE515D2261898FA0510" \
    "15728E5A8AAAC42DAD33170D04507A33A85521ABDF1CBA64" \
    "ECFB850458DBEF0A8AEA71575D060C7DB3970F85A6E1E4C7" \
    "ABF5AE8CDB0933D71E8C94E04A25619DCEE3D2261AD2EE6B" \
    "F12FFA06D98A0864D87602733EC86A64521F2B18177B200C" \
    "BBE117577A615D6C770988C0BAD946E208E24FA074E5AB31" \
    "43DB5BFCE0FD108E4B82D120A92108011A723C12A787E6D7" \
    "88719A10BDBA5B2699C327186AF4E23C1A946834B6150BDA" \
    "2583E9CA2AD44CE8DBBBC2DB04DE8EF92E8EFC141FBECAA6" \
    "287C59474E6BC05D99B2964FA090C3A2233BA186515BE7ED" \
    "1F612970CEE2D7AFB81BDD762170481CD0069127D5B05AA9" \
    "93B4EA988D8FDDC186FFB7DC90A6C08F4DF435C934063199" \
    "FFFFFFFFFFFFFFFF"

#define MBEDTLS_DHM_RFC3526_MODP_4096_G          "02"

#define MBEDTLS_DHM_RFC5114_MODP_2048_P               \
    "AD107E1E9123A9D0D660FAA79559C51FA20D64E5683B9FD1" \
    "B54B1597B61D0A75E6FA141DF95A56DBAF9A3C407BA1DF15" \
    "EB3D688A309C180E1DE6B85A1274A0A66D3F8152AD6AC212" \
    "9037C9EDEFDA4DF8D91E8FEF55B7394B7AD5B7D0B6C12207" \
    "C9F98D11ED34DBF6C6BA0B2C8BBC27BE6A00E0A0B9C49708" \
    "B3BF8A317091883681286130BC8985DB1602E714415D9330" \
    "278273C7DE31EFDC7310F7121FD5A07415987D9ADC0A486D" \
    "CDF93ACC44328387315D75E198C641A480CD86A1B9E587E8" \
    "BE60E69CC928B2B9C52172E413042E9B23F10B0E16E79763" \
    "C9B53DCF4BA80A29E3FB73C16B8E75B97EF363E2FFA31F71" \
    "CF9DE5384E71B81C0AC4DFFE0C10E64F"

#define MBEDTLS_DHM_RFC5114_MODP_2048_G              \
    "AC4032EF4F2D9AE39DF30B5C8FFDAC506CDEBE7B89998CAF"\
    "74866A08CFE4FFE3A6824A4E10B9A6F0DD921F01A70C4AFA"\
    "AB739D7700C29F52C57DB17C620A8652BE5E9001A8D66AD7"\
    "C17669101999024AF4D027275AC1348BB8A762D0521BC98A"\
    "E247150422EA1ED409939D54DA7460CDB5F6C6B250717CBE"\
    "F180EB34118E98D119529A45D6F834566E3025E316A330EF"\
    "BB77A86F0C1AB15B051AE3D428C8F8ACB70A8137150B8EEB"\
    "10E183EDD19963DDD9E263E4770589EF6AA21E7F5F2FF381"\
    "B539CCE3409D13CD566AFBB48D6C019181E1BCFE94B30269"\
    "EDFE72FE9B6AA4BD7B5A0F1C71CFFF4C19C418E1F6EC0179"\
    "81BC087F2A7065B384B890D3191F2BFA"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief          DHM context structure
 */
typedef struct
{
    size_t len; /*!<  size(P) in chars  */
    mbedtls_mpi P;      /*!<  prime modulus     */
    mbedtls_mpi G;      /*!<  generator         */
    mbedtls_mpi X;      /*!<  secret value      */
    mbedtls_mpi GX;     /*!<  self = G^X mod P  */
    mbedtls_mpi GY;     /*!<  peer = G^Y mod P  */
    mbedtls_mpi K;      /*!<  key = GY^X mod P  */
    mbedtls_mpi RP;     /*!<  cached R^2 mod P  */
    mbedtls_mpi Vi;     /*!<  blinding value    */
    mbedtls_mpi Vf;     /*!<  un-blinding value */
    mbedtls_mpi pX;     /*!<  previous X        */
}
mbedtls_dhm_context;

/**
 * \brief          Initialize DHM context
 *
 * \param ctx      DHM context to be initialized
 */
void mbedtls_dhm_init( mbedtls_dhm_context *ctx );

/**
 * \brief          Parse the ServerKeyExchange parameters
 *
 * \param ctx      DHM context
 * \param p        &(start of input buffer)
 * \param end      end of buffer
 *
 * \return         0 if successful, or an MBEDTLS_ERR_DHM_XXX error code
 */
int mbedtls_dhm_read_params( mbedtls_dhm_context *ctx,
                     unsigned char **p,
                     const unsigned char *end );

/**
 * \brief          Setup and write the ServerKeyExchange parameters
 *
 * \param ctx      DHM context
 * \param x_size   private value size in bytes
 * \param output   destination buffer
 * \param olen     number of chars written
 * \param f_rng    RNG function
 * \param p_rng    RNG parameter
 *
 * \note           This function assumes that ctx->P and ctx->G
 *                 have already been properly set (for example
 *                 using mbedtls_mpi_read_string or mbedtls_mpi_read_binary).
 *
 * \return         0 if successful, or an MBEDTLS_ERR_DHM_XXX error code
 */
int mbedtls_dhm_make_params( mbedtls_dhm_context *ctx, int x_size,
                     unsigned char *output, size_t *olen,
                     int (*f_rng)(void *, unsigned char *, size_t),
                     void *p_rng );

/**
 * \brief          Import the peer's public value G^Y
 *
 * \param ctx      DHM context
 * \param input    input buffer
 * \param ilen     size of buffer
 *
 * \return         0 if successful, or an MBEDTLS_ERR_DHM_XXX error code
 */
int mbedtls_dhm_read_public( mbedtls_dhm_context *ctx,
                     const unsigned char *input, size_t ilen );

/**
 * \brief          Create own private value X and export G^X
 *
 * \param ctx      DHM context
 * \param x_size   private value size in bytes
 * \param output   destination buffer
 * \param olen     must be at least equal to the size of P, ctx->len
 * \param f_rng    RNG function
 * \param p_rng    RNG parameter
 *
 * \return         0 if successful, or an MBEDTLS_ERR_DHM_XXX error code
 */
int mbedtls_dhm_make_public( mbedtls_dhm_context *ctx, int x_size,
                     unsigned char *output, size_t olen,
                     int (*f_rng)(void *, unsigned char *, size_t),
                     void *p_rng );

/**
 * \brief          Derive and export the shared secret (G^Y)^X mod P
 *
 * \param ctx      DHM context
 * \param output   destination buffer
 * \param output_size   size of the destination buffer
 * \param olen     on exit, holds the actual number of bytes written
 * \param f_rng    RNG function, for blinding purposes
 * \param p_rng    RNG parameter
 *
 * \return         0 if successful, or an MBEDTLS_ERR_DHM_XXX error code
 *
 * \note           If non-NULL, f_rng is used to blind the input as
 *                 countermeasure against timing attacks. Blinding is
 *                 automatically used if and only if our secret value X is
 *                 re-used and costs nothing otherwise, so it is recommended
 *                 to always pass a non-NULL f_rng argument.
 */
int mbedtls_dhm_calc_secret( mbedtls_dhm_context *ctx,
                     unsigned char *output, size_t output_size, size_t *olen,
                     int (*f_rng)(void *, unsigned char *, size_t),
                     void *p_rng );

/**
 * \brief          Free and clear the components of a DHM key
 *
 * \param ctx      DHM context to free and clear
 */
void mbedtls_dhm_free( mbedtls_dhm_context *ctx );

#if defined(MBEDTLS_ASN1_PARSE_C)
/** \ingroup x509_module */
/**
 * \brief          Parse DHM parameters in PEM or DER format
 *
 * \param dhm      DHM context to be initialized
 * \param dhmin    input buffer
 * \param dhminlen size of the buffer
 *                 (including the terminating null byte for PEM data)
 *
 * \return         0 if successful, or a specific DHM or PEM error code
 */
int mbedtls_dhm_parse_dhm( mbedtls_dhm_context *dhm, const unsigned char *dhmin,
                   size_t dhminlen );

#if defined(MBEDTLS_FS_IO)
/** \ingroup x509_module */
/**
 * \brief          Load and parse DHM parameters
 *
 * \param dhm      DHM context to be initialized
 * \param path     filename to read the DHM Parameters from
 *
 * \return         0 if successful, or a specific DHM or PEM error code
 */
int mbedtls_dhm_parse_dhmfile( mbedtls_dhm_context *dhm, const char *path );
#endif /* MBEDTLS_FS_IO */
#endif /* MBEDTLS_ASN1_PARSE_C */

/**
 * \brief          Checkup routine
 *
 * \return         0 if successful, or 1 if the test failed
 */
int mbedtls_dhm_self_test( int verbose );

#ifdef __cplusplus
}
#endif

#endif /* dhm.h */
