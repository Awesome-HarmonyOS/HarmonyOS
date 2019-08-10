/*
 *  Example RSA key generation program
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
#define mbedtls_printf     printf
#endif

#if defined(MBEDTLS_BIGNUM_C) && defined(MBEDTLS_ENTROPY_C) && \
    defined(MBEDTLS_RSA_C) && defined(MBEDTLS_GENPRIME) && \
    defined(MBEDTLS_FS_IO) && defined(MBEDTLS_CTR_DRBG_C)
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/bignum.h"
#include "mbedtls/x509.h"
#include "mbedtls/rsa.h"

#include <stdio.h>
#include <string.h>
#endif

#define KEY_SIZE 2048
#define EXPONENT 65537

#if !defined(MBEDTLS_BIGNUM_C) || !defined(MBEDTLS_ENTROPY_C) ||   \
    !defined(MBEDTLS_RSA_C) || !defined(MBEDTLS_GENPRIME) ||      \
    !defined(MBEDTLS_FS_IO) || !defined(MBEDTLS_CTR_DRBG_C)
int main( void )
{
    mbedtls_printf("MBEDTLS_BIGNUM_C and/or MBEDTLS_ENTROPY_C and/or "
           "MBEDTLS_RSA_C and/or MBEDTLS_GENPRIME and/or "
           "MBEDTLS_FS_IO and/or MBEDTLS_CTR_DRBG_C not defined.\n");
    return( 0 );
}
#else
int main( void )
{
    int ret;
    mbedtls_rsa_context rsa;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    FILE *fpub  = NULL;
    FILE *fpriv = NULL;
    const char *pers = "rsa_genkey";

    mbedtls_ctr_drbg_init( &ctr_drbg );

    mbedtls_printf( "\n  . Seeding the random number generator..." );
    fflush( stdout );

    mbedtls_entropy_init( &entropy );
    if( ( ret = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy,
                               (const unsigned char *) pers,
                               strlen( pers ) ) ) != 0 )
    {
        mbedtls_printf( " failed\n  ! mbedtls_ctr_drbg_seed returned %d\n", ret );
        goto exit;
    }

    mbedtls_printf( " ok\n  . Generating the RSA key [ %d-bit ]...", KEY_SIZE );
    fflush( stdout );

    mbedtls_rsa_init( &rsa, MBEDTLS_RSA_PKCS_V15, 0 );

    if( ( ret = mbedtls_rsa_gen_key( &rsa, mbedtls_ctr_drbg_random, &ctr_drbg, KEY_SIZE,
                             EXPONENT ) ) != 0 )
    {
        mbedtls_printf( " failed\n  ! mbedtls_rsa_gen_key returned %d\n\n", ret );
        goto exit;
    }

    mbedtls_printf( " ok\n  . Exporting the public  key in rsa_pub.txt...." );
    fflush( stdout );

    if( ( fpub = fopen( "rsa_pub.txt", "wb+" ) ) == NULL )
    {
        mbedtls_printf( " failed\n  ! could not open rsa_pub.txt for writing\n\n" );
        ret = 1;
        goto exit;
    }

    if( ( ret = mbedtls_mpi_write_file( "N = ", &rsa.N, 16, fpub ) ) != 0 ||
        ( ret = mbedtls_mpi_write_file( "E = ", &rsa.E, 16, fpub ) ) != 0 )
    {
        mbedtls_printf( " failed\n  ! mbedtls_mpi_write_file returned %d\n\n", ret );
        goto exit;
    }

    mbedtls_printf( " ok\n  . Exporting the private key in rsa_priv.txt..." );
    fflush( stdout );

    if( ( fpriv = fopen( "rsa_priv.txt", "wb+" ) ) == NULL )
    {
        mbedtls_printf( " failed\n  ! could not open rsa_priv.txt for writing\n" );
        ret = 1;
        goto exit;
    }

    if( ( ret = mbedtls_mpi_write_file( "N = " , &rsa.N , 16, fpriv ) ) != 0 ||
        ( ret = mbedtls_mpi_write_file( "E = " , &rsa.E , 16, fpriv ) ) != 0 ||
        ( ret = mbedtls_mpi_write_file( "D = " , &rsa.D , 16, fpriv ) ) != 0 ||
        ( ret = mbedtls_mpi_write_file( "P = " , &rsa.P , 16, fpriv ) ) != 0 ||
        ( ret = mbedtls_mpi_write_file( "Q = " , &rsa.Q , 16, fpriv ) ) != 0 ||
        ( ret = mbedtls_mpi_write_file( "DP = ", &rsa.DP, 16, fpriv ) ) != 0 ||
        ( ret = mbedtls_mpi_write_file( "DQ = ", &rsa.DQ, 16, fpriv ) ) != 0 ||
        ( ret = mbedtls_mpi_write_file( "QP = ", &rsa.QP, 16, fpriv ) ) != 0 )
    {
        mbedtls_printf( " failed\n  ! mbedtls_mpi_write_file returned %d\n\n", ret );
        goto exit;
    }
/*
    mbedtls_printf( " ok\n  . Generating the certificate..." );

    x509write_init_raw( &cert );
    x509write_add_pubkey( &cert, &rsa );
    x509write_add_subject( &cert, "CN='localhost'" );
    x509write_add_validity( &cert, "2007-09-06 17:00:32",
                                   "2010-09-06 17:00:32" );
    x509write_create_selfsign( &cert, &rsa );
    x509write_crtfile( &cert, "cert.der", X509_OUTPUT_DER );
    x509write_crtfile( &cert, "cert.pem", X509_OUTPUT_PEM );
    x509write_free_raw( &cert );
*/
    mbedtls_printf( " ok\n\n" );

exit:

    if( fpub  != NULL )
        fclose( fpub );

    if( fpriv != NULL )
        fclose( fpriv );

    mbedtls_rsa_free( &rsa );
    mbedtls_ctr_drbg_free( &ctr_drbg );
    mbedtls_entropy_free( &entropy );

#if defined(_WIN32)
    mbedtls_printf( "  Press Enter to exit this program.\n" );
    fflush( stdout ); getchar();
#endif

    return( ret );
}
#endif /* MBEDTLS_BIGNUM_C && MBEDTLS_ENTROPY_C && MBEDTLS_RSA_C &&
          MBEDTLS_GENPRIME && MBEDTLS_FS_IO && MBEDTLS_CTR_DRBG_C */
