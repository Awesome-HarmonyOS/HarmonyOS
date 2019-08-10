/*
 *  RSA simple decryption program
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
#define mbedtls_printf     printf
#define mbedtls_exit       exit
#define MBEDTLS_EXIT_SUCCESS EXIT_SUCCESS
#define MBEDTLS_EXIT_FAILURE EXIT_FAILURE
#endif

#if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_RSA_C) && \
    defined(MBEDTLS_FS_IO) && defined(MBEDTLS_ENTROPY_C) && \
    defined(MBEDTLS_CTR_DRBG_C)
#include "mbedtls/rsa.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"

#include <string.h>

#endif

#if !defined(MBEDTLS_BIGNUM_C) || !defined(MBEDTLS_RSA_C) ||  \
    !defined(MBEDTLS_FS_IO) || !defined(MBEDTLS_ENTROPY_C) || \
    !defined(MBEDTLS_CTR_DRBG_C)
int main( void )
{
    mbedtls_printf("MBEDTLS_BIGNUM_C and/or MBEDTLS_RSA_C and/or "
           "MBEDTLS_FS_IO and/or MBEDTLS_ENTROPY_C and/or "
           "MBEDTLS_CTR_DRBG_C not defined.\n");
    return( 0 );
}
#else
int main( int argc, char *argv[] )
{
    FILE *f;
    int return_val, exit_val, c;
    size_t i;
    mbedtls_rsa_context rsa;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    unsigned char result[1024];
    unsigned char buf[512];
    const char *pers = "rsa_decrypt";
    ((void) argv);

    memset(result, 0, sizeof( result ) );
    exit_val = MBEDTLS_EXIT_SUCCESS;

    if( argc != 1 )
    {
        mbedtls_printf( "usage: rsa_decrypt\n" );

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

    mbedtls_printf( "\n  . Reading private key from rsa_priv.txt" );
    fflush( stdout );

    if( ( f = fopen( "rsa_priv.txt", "rb" ) ) == NULL )
    {
        exit_val = MBEDTLS_EXIT_FAILURE;
        mbedtls_printf( " failed\n  ! Could not open rsa_priv.txt\n" \
                "  ! Please run rsa_genkey first\n\n" );
        goto exit;
    }

    if( ( return_val = mbedtls_mpi_read_file( &rsa.N , 16, f ) ) != 0 ||
        ( return_val = mbedtls_mpi_read_file( &rsa.E , 16, f ) ) != 0 ||
        ( return_val = mbedtls_mpi_read_file( &rsa.D , 16, f ) ) != 0 ||
        ( return_val = mbedtls_mpi_read_file( &rsa.P , 16, f ) ) != 0 ||
        ( return_val = mbedtls_mpi_read_file( &rsa.Q , 16, f ) ) != 0 ||
        ( return_val = mbedtls_mpi_read_file( &rsa.DP, 16, f ) ) != 0 ||
        ( return_val = mbedtls_mpi_read_file( &rsa.DQ, 16, f ) ) != 0 ||
        ( return_val = mbedtls_mpi_read_file( &rsa.QP, 16, f ) ) != 0 )
    {
        exit_val = MBEDTLS_EXIT_FAILURE;
        mbedtls_printf( " failed\n  ! mbedtls_mpi_read_file returned %d\n\n",
                        return_val );
        fclose( f );
        goto exit;
    }

    rsa.len = ( mbedtls_mpi_bitlen( &rsa.N ) + 7 ) >> 3;

    fclose( f );

    /*
     * Extract the RSA encrypted value from the text file
     */
    if( ( f = fopen( "result-enc.txt", "rb" ) ) == NULL )
    {
        exit_val = MBEDTLS_EXIT_FAILURE;
        mbedtls_printf( "\n  ! Could not open %s\n\n", "result-enc.txt" );
        goto exit;
    }

    i = 0;

    while( fscanf( f, "%02X", &c ) > 0 &&
           i < (int) sizeof( buf ) )
        buf[i++] = (unsigned char) c;

    fclose( f );

    if( i != rsa.len )
    {
        exit_val = MBEDTLS_EXIT_FAILURE;
        mbedtls_printf( "\n  ! Invalid RSA signature format\n\n" );
        goto exit;
    }

    /*
     * Decrypt the encrypted RSA data and print the result.
     */
    mbedtls_printf( "\n  . Decrypting the encrypted data" );
    fflush( stdout );

    return_val = mbedtls_rsa_pkcs1_decrypt( &rsa, mbedtls_ctr_drbg_random,
                                            &ctr_drbg, MBEDTLS_RSA_PRIVATE, &i,
                                            buf, result, 1024 );
    if( return_val != 0 )
    {
        exit_val = MBEDTLS_EXIT_FAILURE;
        mbedtls_printf( " failed\n  ! mbedtls_rsa_pkcs1_decrypt returned %d\n\n",
                        return_val );
        goto exit;
    }

    mbedtls_printf( "\n  . OK\n\n" );

    mbedtls_printf( "The decrypted result is: '%s'\n\n", result );

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
#endif /* MBEDTLS_BIGNUM_C && MBEDTLS_RSA_C && MBEDTLS_FS_IO */

