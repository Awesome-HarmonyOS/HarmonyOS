/*
 *  RSA simple data encryption program
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

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#include <stdlib.h>
#define mbedtls_fprintf    fprintf
#define mbedtls_printf     printf
#define mbedtls_exit       exit
#define MBEDTLS_EXIT_SUCCESS EXIT_SUCCESS
#define MBEDTLS_EXIT_FAILURE EXIT_FAILURE
#endif

#if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_RSA_C) && \
    defined(MBEDTLS_ENTROPY_C) && defined(MBEDTLS_FS_IO) && \
    defined(MBEDTLS_CTR_DRBG_C)
#include "mbedtls/rsa.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"

#include <string.h>
#endif

#if !defined(MBEDTLS_BIGNUM_C) || !defined(MBEDTLS_RSA_C) ||  \
    !defined(MBEDTLS_ENTROPY_C) || !defined(MBEDTLS_FS_IO) || \
    !defined(MBEDTLS_CTR_DRBG_C)
int main( void )
{
    mbedtls_printf("MBEDTLS_BIGNUM_C and/or MBEDTLS_RSA_C and/or "
           "MBEDTLS_ENTROPY_C and/or MBEDTLS_FS_IO and/or "
           "MBEDTLS_CTR_DRBG_C not defined.\n");
    return( 0 );
}
#else
int main( int argc, char *argv[] )
{
    FILE *f;
    int return_val, exit_val;
    size_t i;
    mbedtls_rsa_context rsa;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    unsigned char input[1024];
    unsigned char buf[512];
    const char *pers = "rsa_encrypt";

    exit_val = MBEDTLS_EXIT_SUCCESS;

    if( argc != 2 )
    {
        mbedtls_printf( "usage: rsa_encrypt <string of max 100 characters>\n" );

#if defined(_WIN32)
        mbedtls_printf( "\n" );
#endif

        mbedtls_exit( MBEDTLS_EXIT_FAILURE );
    }

    mbedtls_printf( "\n  . Seeding the random number generator..." );
    fflush( stdout );

    mbedtls_rsa_init( &rsa, MBEDTLS_RSA_PKCS_V15, 0 );
    mbedtls_ctr_drbg_init( &ctr_drbg );
    mbedtls_entropy_init( &entropy );

    return_val = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func,
                                        &entropy, (const unsigned char *) pers,
                                        strlen( pers ) );
    if( return_val != 0 )
    {
        exit_val = MBEDTLS_EXIT_FAILURE;
        mbedtls_printf( " failed\n  ! mbedtls_ctr_drbg_seed returned %d\n",
                        return_val );
        goto exit;
    }

    mbedtls_printf( "\n  . Reading public key from rsa_pub.txt" );
    fflush( stdout );

    if( ( f = fopen( "rsa_pub.txt", "rb" ) ) == NULL )
    {
        exit_val = MBEDTLS_EXIT_FAILURE;
        mbedtls_printf( " failed\n  ! Could not open rsa_pub.txt\n" \
                "  ! Please run rsa_genkey first\n\n" );
        goto exit;
    }

    if( ( return_val = mbedtls_mpi_read_file( &rsa.N, 16, f ) ) != 0 ||
        ( return_val = mbedtls_mpi_read_file( &rsa.E, 16, f ) ) != 0 )
    {
        exit_val = MBEDTLS_EXIT_FAILURE;
        mbedtls_printf( " failed\n  ! mbedtls_mpi_read_file returned %d\n\n",
                        return_val );
        fclose( f );
        goto exit;
    }

    rsa.len = ( mbedtls_mpi_bitlen( &rsa.N ) + 7 ) >> 3;

    fclose( f );

    if( strlen( argv[1] ) > 100 )
    {
        exit_val = MBEDTLS_EXIT_FAILURE;
        mbedtls_printf( " Input data larger than 100 characters.\n\n" );
        goto exit;
    }

    memcpy( input, argv[1], strlen( argv[1] ) );

    /*
     * Calculate the RSA encryption of the hash.
     */
    mbedtls_printf( "\n  . Generating the RSA encrypted value" );
    fflush( stdout );

    return_val = mbedtls_rsa_pkcs1_encrypt( &rsa, mbedtls_ctr_drbg_random,
                                            &ctr_drbg, MBEDTLS_RSA_PUBLIC,
                                            strlen( argv[1] ), input, buf );
    if( return_val != 0 )
    {
        exit_val = MBEDTLS_EXIT_FAILURE;
        mbedtls_printf( " failed\n  ! mbedtls_rsa_pkcs1_encrypt returned %d\n\n",
                        return_val );
        goto exit;
    }

    /*
     * Write the signature into result-enc.txt
     */
    if( ( f = fopen( "result-enc.txt", "wb+" ) ) == NULL )
    {
        exit_val = MBEDTLS_EXIT_FAILURE;
        mbedtls_printf( " failed\n  ! Could not create %s\n\n", "result-enc.txt" );
        goto exit;
    }

    for( i = 0; i < rsa.len; i++ )
        mbedtls_fprintf( f, "%02X%s", buf[i],
                 ( i + 1 ) % 16 == 0 ? "\r\n" : " " );

    fclose( f );

    mbedtls_printf( "\n  . Done (created \"%s\")\n\n", "result-enc.txt" );

exit:
    mbedtls_ctr_drbg_free( &ctr_drbg );
    mbedtls_entropy_free( &entropy );
    mbedtls_rsa_free( &rsa );

#if defined(_WIN32)
    mbedtls_printf( "  + Press Enter to exit this program.\n" );
    fflush( stdout ); getchar();
#endif

    return( exit_val );
}
#endif /* MBEDTLS_BIGNUM_C && MBEDTLS_RSA_C && MBEDTLS_ENTROPY_C &&
          MBEDTLS_FS_IO && MBEDTLS_CTR_DRBG_C */
