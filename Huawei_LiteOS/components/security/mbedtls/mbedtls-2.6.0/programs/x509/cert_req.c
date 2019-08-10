/*
 *  Certificate request generation
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

#if !defined(MBEDTLS_X509_CSR_WRITE_C) || !defined(MBEDTLS_FS_IO) ||  \
    !defined(MBEDTLS_PK_PARSE_C) || !defined(MBEDTLS_SHA256_C) || \
    !defined(MBEDTLS_ENTROPY_C) || !defined(MBEDTLS_CTR_DRBG_C) || \
    !defined(MBEDTLS_PEM_WRITE_C)
int main( void )
{
    mbedtls_printf( "MBEDTLS_X509_CSR_WRITE_C and/or MBEDTLS_FS_IO and/or "
            "MBEDTLS_PK_PARSE_C and/or MBEDTLS_SHA256_C and/or "
            "MBEDTLS_ENTROPY_C and/or MBEDTLS_CTR_DRBG_C "
            "not defined.\n");
    return( 0 );
}
#else

#include "mbedtls/x509_csr.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DFL_FILENAME            "keyfile.key"
#define DFL_DEBUG_LEVEL         0
#define DFL_OUTPUT_FILENAME     "cert.req"
#define DFL_SUBJECT_NAME        "CN=Cert,O=mbed TLS,C=UK"
#define DFL_KEY_USAGE           0
#define DFL_NS_CERT_TYPE        0

#define USAGE \
    "\n usage: cert_req param=<>...\n"                  \
    "\n acceptable parameters:\n"                       \
    "    filename=%%s         default: keyfile.key\n"   \
    "    debug_level=%%d      default: 0 (disabled)\n"  \
    "    output_file=%%s      default: cert.req\n"      \
    "    subject_name=%%s     default: CN=Cert,O=mbed TLS,C=UK\n"   \
    "    key_usage=%%s        default: (empty)\n"       \
    "                        Comma-separated-list of values:\n"     \
    "                          digital_signature\n"     \
    "                          non_repudiation\n"       \
    "                          key_encipherment\n"      \
    "                          data_encipherment\n"     \
    "                          key_agreement\n"         \
    "                          key_cert_sign\n"  \
    "                          crl_sign\n"              \
    "    ns_cert_type=%%s     default: (empty)\n"       \
    "                        Comma-separated-list of values:\n"     \
    "                          ssl_client\n"            \
    "                          ssl_server\n"            \
    "                          email\n"                 \
    "                          object_signing\n"        \
    "                          ssl_ca\n"                \
    "                          email_ca\n"              \
    "                          object_signing_ca\n"     \
    "\n"

/*
 * global options
 */
struct options
{
    const char *filename;       /* filename of the key file             */
    int debug_level;            /* level of debugging                   */
    const char *output_file;    /* where to store the constructed key file  */
    const char *subject_name;   /* subject name for certificate request */
    unsigned char key_usage;    /* key usage flags                      */
    unsigned char ns_cert_type; /* NS cert type                         */
} opt;

int write_certificate_request( mbedtls_x509write_csr *req, const char *output_file,
                               int (*f_rng)(void *, unsigned char *, size_t),
                               void *p_rng )
{
    int ret;
    FILE *f;
    unsigned char output_buf[4096];
    size_t len = 0;

    memset( output_buf, 0, 4096 );
    if( ( ret = mbedtls_x509write_csr_pem( req, output_buf, 4096, f_rng, p_rng ) ) < 0 )
        return( ret );

    len = strlen( (char *) output_buf );

    if( ( f = fopen( output_file, "w" ) ) == NULL )
        return( -1 );

    if( fwrite( output_buf, 1, len, f ) != len )
    {
        fclose( f );
        return( -1 );
    }

    fclose( f );

    return( 0 );
}

int main( int argc, char *argv[] )
{
    int ret = 0;
    mbedtls_pk_context key;
    char buf[1024];
    int i;
    char *p, *q, *r;
    mbedtls_x509write_csr req;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    const char *pers = "csr example app";

    /*
     * Set to sane values
     */
    mbedtls_x509write_csr_init( &req );
    mbedtls_x509write_csr_set_md_alg( &req, MBEDTLS_MD_SHA256 );
    mbedtls_pk_init( &key );
    mbedtls_ctr_drbg_init( &ctr_drbg );
    memset( buf, 0, sizeof( buf ) );

    if( argc == 0 )
    {
    usage:
        mbedtls_printf( USAGE );
        ret = 1;
        goto exit;
    }

    opt.filename            = DFL_FILENAME;
    opt.debug_level         = DFL_DEBUG_LEVEL;
    opt.output_file         = DFL_OUTPUT_FILENAME;
    opt.subject_name        = DFL_SUBJECT_NAME;
    opt.key_usage           = DFL_KEY_USAGE;
    opt.ns_cert_type        = DFL_NS_CERT_TYPE;

    for( i = 1; i < argc; i++ )
    {

        p = argv[i];
        if( ( q = strchr( p, '=' ) ) == NULL )
            goto usage;
        *q++ = '\0';

        if( strcmp( p, "filename" ) == 0 )
            opt.filename = q;
        else if( strcmp( p, "output_file" ) == 0 )
            opt.output_file = q;
        else if( strcmp( p, "debug_level" ) == 0 )
        {
            opt.debug_level = atoi( q );
            if( opt.debug_level < 0 || opt.debug_level > 65535 )
                goto usage;
        }
        else if( strcmp( p, "subject_name" ) == 0 )
        {
            opt.subject_name = q;
        }
        else if( strcmp( p, "key_usage" ) == 0 )
        {
            while( q != NULL )
            {
                if( ( r = strchr( q, ',' ) ) != NULL )
                    *r++ = '\0';

                if( strcmp( q, "digital_signature" ) == 0 )
                    opt.key_usage |= MBEDTLS_X509_KU_DIGITAL_SIGNATURE;
                else if( strcmp( q, "non_repudiation" ) == 0 )
                    opt.key_usage |= MBEDTLS_X509_KU_NON_REPUDIATION;
                else if( strcmp( q, "key_encipherment" ) == 0 )
                    opt.key_usage |= MBEDTLS_X509_KU_KEY_ENCIPHERMENT;
                else if( strcmp( q, "data_encipherment" ) == 0 )
                    opt.key_usage |= MBEDTLS_X509_KU_DATA_ENCIPHERMENT;
                else if( strcmp( q, "key_agreement" ) == 0 )
                    opt.key_usage |= MBEDTLS_X509_KU_KEY_AGREEMENT;
                else if( strcmp( q, "key_cert_sign" ) == 0 )
                    opt.key_usage |= MBEDTLS_X509_KU_KEY_CERT_SIGN;
                else if( strcmp( q, "crl_sign" ) == 0 )
                    opt.key_usage |= MBEDTLS_X509_KU_CRL_SIGN;
                else
                    goto usage;

                q = r;
            }
        }
        else if( strcmp( p, "ns_cert_type" ) == 0 )
        {
            while( q != NULL )
            {
                if( ( r = strchr( q, ',' ) ) != NULL )
                    *r++ = '\0';

                if( strcmp( q, "ssl_client" ) == 0 )
                    opt.ns_cert_type |= MBEDTLS_X509_NS_CERT_TYPE_SSL_CLIENT;
                else if( strcmp( q, "ssl_server" ) == 0 )
                    opt.ns_cert_type |= MBEDTLS_X509_NS_CERT_TYPE_SSL_SERVER;
                else if( strcmp( q, "email" ) == 0 )
                    opt.ns_cert_type |= MBEDTLS_X509_NS_CERT_TYPE_EMAIL;
                else if( strcmp( q, "object_signing" ) == 0 )
                    opt.ns_cert_type |= MBEDTLS_X509_NS_CERT_TYPE_OBJECT_SIGNING;
                else if( strcmp( q, "ssl_ca" ) == 0 )
                    opt.ns_cert_type |= MBEDTLS_X509_NS_CERT_TYPE_SSL_CA;
                else if( strcmp( q, "email_ca" ) == 0 )
                    opt.ns_cert_type |= MBEDTLS_X509_NS_CERT_TYPE_EMAIL_CA;
                else if( strcmp( q, "object_signing_ca" ) == 0 )
                    opt.ns_cert_type |= MBEDTLS_X509_NS_CERT_TYPE_OBJECT_SIGNING_CA;
                else
                    goto usage;

                q = r;
            }
        }
        else
            goto usage;
    }

    if( opt.key_usage )
        mbedtls_x509write_csr_set_key_usage( &req, opt.key_usage );

    if( opt.ns_cert_type )
        mbedtls_x509write_csr_set_ns_cert_type( &req, opt.ns_cert_type );

    /*
     * 0. Seed the PRNG
     */
    mbedtls_printf( "  . Seeding the random number generator..." );
    fflush( stdout );

    mbedtls_entropy_init( &entropy );
    if( ( ret = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy,
                               (const unsigned char *) pers,
                               strlen( pers ) ) ) != 0 )
    {
        mbedtls_printf( " failed\n  !  mbedtls_ctr_drbg_seed returned %d", ret );
        goto exit;
    }

    mbedtls_printf( " ok\n" );

    /*
     * 1.0. Check the subject name for validity
     */
    mbedtls_printf( "  . Checking subject name..." );
    fflush( stdout );

    if( ( ret = mbedtls_x509write_csr_set_subject_name( &req, opt.subject_name ) ) != 0 )
    {
        mbedtls_printf( " failed\n  !  mbedtls_x509write_csr_set_subject_name returned %d", ret );
        goto exit;
    }

    mbedtls_printf( " ok\n" );

    /*
     * 1.1. Load the key
     */
    mbedtls_printf( "  . Loading the private key ..." );
    fflush( stdout );

    ret = mbedtls_pk_parse_keyfile( &key, opt.filename, NULL );

    if( ret != 0 )
    {
        mbedtls_printf( " failed\n  !  mbedtls_pk_parse_keyfile returned %d", ret );
        goto exit;
    }

    mbedtls_x509write_csr_set_key( &req, &key );

    mbedtls_printf( " ok\n" );

    /*
     * 1.2. Writing the request
     */
    mbedtls_printf( "  . Writing the certificate request ..." );
    fflush( stdout );

    if( ( ret = write_certificate_request( &req, opt.output_file,
                                           mbedtls_ctr_drbg_random, &ctr_drbg ) ) != 0 )
    {
        mbedtls_printf( " failed\n  !  write_certifcate_request %d", ret );
        goto exit;
    }

    mbedtls_printf( " ok\n" );

exit:

    if( ret != 0 && ret != 1)
    {
#ifdef MBEDTLS_ERROR_C
        mbedtls_strerror( ret, buf, sizeof( buf ) );
        mbedtls_printf( " - %s\n", buf );
#else
        mbedtls_printf("\n");
#endif
    }

    mbedtls_x509write_csr_free( &req );
    mbedtls_pk_free( &key );
    mbedtls_ctr_drbg_free( &ctr_drbg );
    mbedtls_entropy_free( &entropy );

#if defined(_WIN32)
    mbedtls_printf( "  + Press Enter to exit this program.\n" );
    fflush( stdout ); getchar();
#endif

    return( ret );
}
#endif /* MBEDTLS_X509_CSR_WRITE_C && MBEDTLS_PK_PARSE_C && MBEDTLS_FS_IO &&
          MBEDTLS_ENTROPY_C && MBEDTLS_CTR_DRBG_C && MBEDTLS_PEM_WRITE_C */
