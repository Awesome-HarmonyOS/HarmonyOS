/*
 *  Benchmark demonstration program
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

#if !defined(TARGET_LIKE_MBED)

#include <stdio.h>

int main() {
    printf("this version of this program only works on mbed OS\n");
    return 0;
}

#else

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#define mbedtls_exit       exit
#define mbedtls_printf     printf
#define mbedtls_snprintf   snprintf
#define mbedtls_free       free
#endif

#include <string.h>

#include "mbedtls/md4.h"
#include "mbedtls/md5.h"
#include "mbedtls/ripemd160.h"
#include "mbedtls/sha1.h"
#include "mbedtls/sha256.h"
#include "mbedtls/sha512.h"
#include "mbedtls/arc4.h"
#include "mbedtls/des.h"
#include "mbedtls/aes.h"
#include "mbedtls/blowfish.h"
#include "mbedtls/camellia.h"
#include "mbedtls/gcm.h"
#include "mbedtls/ccm.h"
#include "mbedtls/havege.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/hmac_drbg.h"
#include "mbedtls/rsa.h"
#include "mbedtls/pk.h"
#include "mbedtls/dhm.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/error.h"

#include "mbed-drivers/mbed.h"

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
#include "mbedtls/memory_buffer_alloc.h"
#endif

#define RSA_PRIVATE_KEY_2048                                            \
"-----BEGIN RSA PRIVATE KEY-----\r\n"                                   \
"MIIEogIBAAKCAQEA2dwVr+IMGEtA2/MCP6fA5eb/6B18Bq6e7gw8brNPkm3E6LyR\r\n"  \
"4DnMJVxZmw3bPDKBDoKzfntkMESi/Yw5UopLtVfjGfWeQWPClqffLZBsZ60BRAsg\r\n"  \
"/g+ID5tgzxSuxzftypK59uexOVCAm7hCKZHGO3DbI7bLY27j7VAgEP7d/yuaz5Fx\r\n"  \
"Kl/vu7shqrBoz6ABJVJD3KC8nUiMRUCXRINmxbyUUjA4DnicZv6+xrGKr36r6M8h\r\n"  \
"VYLa5msKc8WzbnBWzpUsrpb4/r7ML+qp92gdSfVJ8/bLiU7h2C7faDA59uaqrFK9\r\n"  \
"xmDdx7FaWhGQs3LWW6w1UNgkPS0FDYUslpsnsQIDAQABAoIBAC7IJNwM5V3+IuJY\r\n"  \
"T35Nzo1PyloUosJokvY5KGz5Ejg2XBdCDu0gXCcVqqQyGIbXrYDpLhQV+RCoXHun\r\n"  \
"tdN0oQdC5SB47s/J1Uo2qCUHo0+sBd6PqTkFKsl3KxWssk9TQjvCwC412IefMs69\r\n"  \
"hW+ZvwCanmQP56LleApIr2oW4KLfW8Ry/QfZlua+dizctdN7+H1mWwgZQTY9T27J\r\n"  \
"6RtGRA5NVkKVPzIHVJfdpKoO7xGg1g06aEbPB/VmGvZaaFWWnaf7uRvFjLZecBLu\r\n"  \
"QSx2DA/GDjirlDYj99PJb7DtB4xRtKzsyw0o+xapC8w6OtIl/3xFt9moCu2jGrsx\r\n"  \
"vpjHdfECgYEA7fSACRseIs9gAIVX8wq6gayTpA47DHYWAD6IQfIj35SJ+AgsvbFF\r\n"  \
"4AmrwDhcJVPmDy1N4nLBfyGAMt/2CfiYkdkW6QFX/ULRMMBL/G7kWV8hYQDICB2g\r\n"  \
"xaMRN1lPCmFq6BkSWjwIYTnYDFBDWVm1GVT8TMtJoM8Erej9qC0PeFUCgYEA6mF3\r\n"  \
"bigO3t8f5sig+XepaftEUbkJMzo72TVRnIR2ycdR2ihelPQ+25g9dwV0ZA5XXhBS\r\n"  \
"DKOABWjMM739Mwmy9v26Dlmu9R01zHQktMvtEAyfz7lk2NF0aMuj8285OJUBf9bz\r\n"  \
"Cq3MjtMCD+4CZ6iaEqCdUKOuxfpx5cWVJV+qve0CgYBhD1YaYMFOGaBjFgDl1f51\r\n"  \
"Xltqk5NqZdBbkSYrIAWZ8RDF5y+4wFJsLAWuhk6vuyUgE66tK3nZzWRpXAkT0B8L\r\n"  \
"fq1lpXKqj1KcvBNCiEkEW1VWJ+dvyAYIF5eyJ++hoFLnETL3M32HivyhKSwPihPg\r\n"  \
"nVW8TT9fJJIYDe1JZ/fjcQKBgHJfv7UsrR0LSvkG3K8AOtbx+8PZhOjPuRbk0v+L\r\n"  \
"EKCkuIe5/XW4vtfQMeZb7hFJgk7vrepm+vkoy8VQKDf4urGW3W1VTHBmobM01hi4\r\n"  \
"DuYvEul+Mf0wMRtWjJolo4m+BO5KiW2jpFfqFm6JmfjVqOIAKOSKC6am8V/MDF0h\r\n"  \
"kyN9AoGAT9oOiEXMolbkDZw/QCaBiRoAGlGlNYUkJ+58U6OjIZLISw6aFv+Y2uE0\r\n"  \
"mEImItjuYZtSYKblWikp6ldPoKlt9bwEFe3c6IZ8kJ3+xyEyAGrvjXjEY7PzP6dp\r\n"  \
"Ajbjp9X9uocEBv9W/KsBLdQ7yizcL/toHwdBO4vQqmqTvAc5IIw=\r\n"              \
"-----END RSA PRIVATE KEY-----\r\n"

#define RSA_PRIVATE_KEY_4096                                            \
"-----BEGIN RSA PRIVATE KEY-----\r\n"                                   \
"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\r\n"  \
"-----END RSA PRIVATE KEY-----\r\n"

#if defined _MSC_VER && !defined snprintf
#define snprintf _snprintf
#endif

/*
 * For heap usage estimates, we need an estimate of the overhead per allocated
 * block. ptmalloc2/3 (used in gnu libc for instance) uses 2 size_t per block,
 * so use that as our baseline.
 */
#define MEM_BLOCK_OVERHEAD  ( 2 * sizeof( size_t ) )

/*
 * Size to use for the malloc buffer if MEMORY_BUFFER_ALLOC_C is defined.
 */
#define HEAP_SIZE       (1u << 16)  // 64k

#define BUFSIZE         1024
#define HEADER_FORMAT   "  %-24s :  "
#define TITLE_LEN       25

#define OPTIONS                                                         \
    "md4, md5, ripemd160, sha1, sha256, sha512,\r\n"                      \
    "arc4, des3, des, aes_cbc, aes_gcm, aes_ccm, camellia, blowfish,\r\n" \
    "havege, ctr_drbg, hmac_drbg\r\n"                                     \
    "rsa, dhm, ecdsa, ecdh.\r\n"

#if defined(MBEDTLS_ERROR_C)
#define PRINT_ERROR                                                     \
        mbedtls_strerror( ret, ( char * )tmp, sizeof( tmp ) );         \
        mbedtls_printf( "FAILED: %s\r\n", tmp );
#else
#define PRINT_ERROR                                                     \
        mbedtls_printf( "FAILED: -0x%04x\r\n", -ret );
#endif

static unsigned long mbedtls_timing_hardclock( void )
{
    static int dwt_started = 0;

    if( dwt_started == 0 )
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    return( DWT->CYCCNT );
}

static volatile int alarmed;
static void alarm() { alarmed = 1; }

#define TIME_AND_TSC( TITLE, CODE )                                     \
do {                                                                    \
    unsigned long i, j, tsc;                                            \
    Timeout t; \
                                                                        \
    mbedtls_printf( HEADER_FORMAT, TITLE );                            \
    fflush( stdout );                                                   \
                                                                        \
    for( i = 1, alarmed = 0, t.attach( alarm, 1.0 ); !alarmed; i++ )                                        \
    {                                                                   \
        CODE;                                                           \
    }                                                                   \
                                                                        \
    tsc = mbedtls_timing_hardclock();                                                  \
    for( j = 0; j < 1024; j++ )                                         \
    {                                                                   \
        CODE;                                                           \
    }                                                                   \
                                                                        \
    mbedtls_printf( "%9lu Kb/s,  %9lu cycles/byte\r\n",                  \
                     i * BUFSIZE / 1024,                                \
                     ( mbedtls_timing_hardclock() - tsc ) / ( j * BUFSIZE ) );         \
} while( 0 )

#if defined(MBEDTLS_ERROR_C)
#define PRINT_ERROR                                                     \
        mbedtls_strerror( ret, ( char * )tmp, sizeof( tmp ) );         \
        mbedtls_printf( "FAILED: %s\r\n", tmp );
#else
#define PRINT_ERROR                                                     \
        mbedtls_printf( "FAILED: -0x%04x\r\n", -ret );
#endif

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C) && defined(MBEDTLS_MEMORY_DEBUG)

#define MEMORY_MEASURE_INIT                                             \
    size_t max_used, max_blocks, max_bytes;                             \
    size_t prv_used, prv_blocks;                                        \
    mbedtls_memory_buffer_alloc_cur_get( &prv_used, &prv_blocks );              \
    mbedtls_memory_buffer_alloc_max_reset( );

#define MEMORY_MEASURE_PRINT( title_len )                               \
    mbedtls_memory_buffer_alloc_max_get( &max_used, &max_blocks );              \
    for( i = 12 - title_len; i != 0; i-- ) mbedtls_printf( " " );      \
    max_used -= prv_used;                                               \
    max_blocks -= prv_blocks;                                           \
    max_bytes = max_used + MEM_BLOCK_OVERHEAD * max_blocks;             \
    mbedtls_printf( "%6u heap bytes", (unsigned) max_bytes );

#else
#define MEMORY_MEASURE_INIT
#define MEMORY_MEASURE_PRINT( title_len )
#endif

#define TIME_PUBLIC( TITLE, TYPE, CODE )                                \
do {                                                                    \
    unsigned long ms;                                                    \
    int ret = 0;                                                            \
    Timer t; \
    MEMORY_MEASURE_INIT;                                                \
                                                                        \
    mbedtls_printf( HEADER_FORMAT, TITLE );                            \
    fflush( stdout );                                                   \
    \
    t.start(); \
    CODE; \
    t.stop(); \
    ms = t.read_ms(); \
                                                                        \
    if( ret != 0 )                                                      \
    {                                                                   \
        PRINT_ERROR;                                                    \
    }                                                                   \
    else                                                                \
    {                                                                   \
        mbedtls_printf( "%6lu ms/" TYPE, ms );                    \
        MEMORY_MEASURE_PRINT( sizeof( TYPE ) + 1 );                     \
        mbedtls_printf( "\r\n" );                                        \
    }                                                                   \
} while( 0 )

static int myrand( void *rng_state, unsigned char *output, size_t len )
{
    size_t use_len;
    int rnd;

    if( rng_state != NULL )
        rng_state  = NULL;

    while( len > 0 )
    {
        use_len = len;
        if( use_len > sizeof(int) )
            use_len = sizeof(int);

        rnd = rand();
        memcpy( output, &rnd, use_len );
        output += use_len;
        len -= use_len;
    }

    return( 0 );
}

/*
 * Clear some memory that was used to prepare the context
 */
#if defined(MBEDTLS_ECP_C)
void ecp_clear_precomputed( mbedtls_ecp_group *grp )
{
    if( grp->T != NULL )
    {
        size_t i;
        for( i = 0; i < grp->T_size; i++ )
            mbedtls_ecp_point_free( &grp->T[i] );
        mbedtls_free( grp->T );
    }
    grp->T = NULL;
    grp->T_size = 0;
}
#else
#define ecp_clear_precomputed( g )
#endif

unsigned char buf[BUFSIZE];

typedef struct {
    char md4, md5, ripemd160, sha1, sha256, sha512,
         arc4, des3, des, aes_cbc, aes_gcm, aes_ccm, camellia, blowfish,
         havege, ctr_drbg, hmac_drbg,
         rsa, dhm, ecdsa, ecdh;
} todo_list;

int benchmark( int argc, char *argv[] )
{
    int i;
    unsigned char tmp[200];
    char title[TITLE_LEN];
    todo_list todo;
#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
    unsigned char malloc_buf[HEAP_SIZE] = { 0 };
#endif

    if( argc <= 1 )
    {
        memset( &todo, 1, sizeof( todo ) );
    }
    else
    {
        memset( &todo, 0, sizeof( todo ) );

        for( i = 1; i < argc; i++ )
        {
            if( strcmp( argv[i], "md4" ) == 0 )
                todo.md4 = 1;
            else if( strcmp( argv[i], "md5" ) == 0 )
                todo.md5 = 1;
            else if( strcmp( argv[i], "ripemd160" ) == 0 )
                todo.ripemd160 = 1;
            else if( strcmp( argv[i], "sha1" ) == 0 )
                todo.sha1 = 1;
            else if( strcmp( argv[i], "sha256" ) == 0 )
                todo.sha256 = 1;
            else if( strcmp( argv[i], "sha512" ) == 0 )
                todo.sha512 = 1;
            else if( strcmp( argv[i], "arc4" ) == 0 )
                todo.arc4 = 1;
            else if( strcmp( argv[i], "des3" ) == 0 )
                todo.des3 = 1;
            else if( strcmp( argv[i], "des" ) == 0 )
                todo.des = 1;
            else if( strcmp( argv[i], "aes_cbc" ) == 0 )
                todo.aes_cbc = 1;
            else if( strcmp( argv[i], "aes_gcm" ) == 0 )
                todo.aes_gcm = 1;
            else if( strcmp( argv[i], "aes_ccm" ) == 0 )
                todo.aes_ccm = 1;
            else if( strcmp( argv[i], "camellia" ) == 0 )
                todo.camellia = 1;
            else if( strcmp( argv[i], "blowfish" ) == 0 )
                todo.blowfish = 1;
            else if( strcmp( argv[i], "havege" ) == 0 )
                todo.havege = 1;
            else if( strcmp( argv[i], "ctr_drbg" ) == 0 )
                todo.ctr_drbg = 1;
            else if( strcmp( argv[i], "hmac_drbg" ) == 0 )
                todo.hmac_drbg = 1;
            else if( strcmp( argv[i], "rsa" ) == 0 )
                todo.rsa = 1;
            else if( strcmp( argv[i], "dhm" ) == 0 )
                todo.dhm = 1;
            else if( strcmp( argv[i], "ecdsa" ) == 0 )
                todo.ecdsa = 1;
            else if( strcmp( argv[i], "ecdh" ) == 0 )
                todo.ecdh = 1;
            else
            {
                mbedtls_printf( "Unrecognized option: %s\r\n", argv[i] );
                mbedtls_printf( "Available options: " OPTIONS );
            }
        }
    }

    mbedtls_printf( "\r\n\r\n" );

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
    mbedtls_memory_buffer_alloc_init( malloc_buf, sizeof( malloc_buf ) );
#endif
    memset( buf, 0xAA, sizeof( buf ) );
    memset( tmp, 0xBB, sizeof( tmp ) );

#if defined(MBEDTLS_MD4_C)
    if( todo.md4 )
        TIME_AND_TSC( "MD4", mbedtls_md4( buf, BUFSIZE, tmp ) );
#endif

#if defined(MBEDTLS_MD5_C)
    if( todo.md5 )
        TIME_AND_TSC( "MD5", mbedtls_md5( buf, BUFSIZE, tmp ) );
#endif

#if defined(MBEDTLS_RIPEMD160_C)
    if( todo.ripemd160 )
        TIME_AND_TSC( "RIPEMD160", mbedtls_ripemd160( buf, BUFSIZE, tmp ) );
#endif

#if defined(MBEDTLS_SHA1_C)
    if( todo.sha1 )
        TIME_AND_TSC( "SHA-1", mbedtls_sha1( buf, BUFSIZE, tmp ) );
#endif

#if defined(MBEDTLS_SHA256_C)
    if( todo.sha256 )
        TIME_AND_TSC( "SHA-256", mbedtls_sha256( buf, BUFSIZE, tmp, 0 ) );
#endif

#if defined(MBEDTLS_SHA512_C)
    if( todo.sha512 )
        TIME_AND_TSC( "SHA-512", mbedtls_sha512( buf, BUFSIZE, tmp, 0 ) );
#endif

#if defined(MBEDTLS_ARC4_C)
    if( todo.arc4 )
    {
        mbedtls_arc4_context arc4;
        mbedtls_arc4_init( &arc4 );
        mbedtls_arc4_setup( &arc4, tmp, 32 );
        TIME_AND_TSC( "ARC4", mbedtls_arc4_crypt( &arc4, BUFSIZE, buf, buf ) );
        mbedtls_arc4_free( &arc4 );
    }
#endif

#if defined(MBEDTLS_DES_C) && defined(MBEDTLS_CIPHER_MODE_CBC)
    if( todo.des3 )
    {
        mbedtls_des3_context des3;
        mbedtls_des3_init( &des3 );
        mbedtls_des3_set3key_enc( &des3, tmp );
        TIME_AND_TSC( "3DES",
                mbedtls_des3_crypt_cbc( &des3, MBEDTLS_DES_ENCRYPT, BUFSIZE, tmp, buf, buf ) );
        mbedtls_des3_free( &des3 );
    }

    if( todo.des )
    {
        mbedtls_des_context des;
        mbedtls_des_init( &des );
        mbedtls_des_setkey_enc( &des, tmp );
        TIME_AND_TSC( "DES",
                mbedtls_des_crypt_cbc( &des, MBEDTLS_DES_ENCRYPT, BUFSIZE, tmp, buf, buf ) );
        mbedtls_des_free( &des );
    }
#endif

#if defined(MBEDTLS_AES_C)
#if defined(MBEDTLS_CIPHER_MODE_CBC)
    if( todo.aes_cbc )
    {
        int keysize;
        mbedtls_aes_context aes;
        mbedtls_aes_init( &aes );
        for( keysize = 128; keysize <= 256; keysize += 64 )
        {
            mbedtls_snprintf( title, sizeof( title ), "AES-CBC-%d", keysize );

            memset( buf, 0, sizeof( buf ) );
            memset( tmp, 0, sizeof( tmp ) );
            mbedtls_aes_setkey_enc( &aes, tmp, keysize );

            TIME_AND_TSC( title,
                mbedtls_aes_crypt_cbc( &aes, MBEDTLS_AES_ENCRYPT, BUFSIZE, tmp, buf, buf ) );
        }
        mbedtls_aes_free( &aes );
    }
#endif
#if defined(MBEDTLS_GCM_C)
    if( todo.aes_gcm )
    {
        int keysize;
        mbedtls_gcm_context gcm;

        mbedtls_gcm_init( &gcm );
        for( keysize = 128; keysize <= 256; keysize += 64 )
        {
            mbedtls_snprintf( title, sizeof( title ), "AES-GCM-%d", keysize );

            memset( buf, 0, sizeof( buf ) );
            memset( tmp, 0, sizeof( tmp ) );
            mbedtls_gcm_setkey( &gcm, MBEDTLS_CIPHER_ID_AES, tmp, keysize );

            TIME_AND_TSC( title,
                    mbedtls_gcm_crypt_and_tag( &gcm, MBEDTLS_GCM_ENCRYPT, BUFSIZE, tmp,
                        12, NULL, 0, buf, buf, 16, tmp ) );

            mbedtls_gcm_free( &gcm );
        }
    }
#endif
#if defined(MBEDTLS_CCM_C)
    if( todo.aes_ccm )
    {
        int keysize;
        mbedtls_ccm_context ccm;

        mbedtls_ccm_init( &ccm );
        for( keysize = 128; keysize <= 256; keysize += 64 )
        {
            mbedtls_snprintf( title, sizeof( title ), "AES-CCM-%d", keysize );

            memset( buf, 0, sizeof( buf ) );
            memset( tmp, 0, sizeof( tmp ) );
            mbedtls_ccm_setkey( &ccm, MBEDTLS_CIPHER_ID_AES, tmp, keysize );

            TIME_AND_TSC( title,
                    mbedtls_ccm_encrypt_and_tag( &ccm, BUFSIZE, tmp,
                        12, NULL, 0, buf, buf, tmp, 16 ) );

            mbedtls_ccm_free( &ccm );
        }
    }
#endif
#endif

#if defined(MBEDTLS_CAMELLIA_C) && defined(MBEDTLS_CIPHER_MODE_CBC)
    if( todo.camellia )
    {
        int keysize;
        mbedtls_camellia_context camellia;
        mbedtls_camellia_init( &camellia );
        for( keysize = 128; keysize <= 256; keysize += 64 )
        {
            mbedtls_snprintf( title, sizeof( title ), "CAMELLIA-CBC-%d", keysize );

            memset( buf, 0, sizeof( buf ) );
            memset( tmp, 0, sizeof( tmp ) );
            mbedtls_camellia_setkey_enc( &camellia, tmp, keysize );

            TIME_AND_TSC( title,
                    mbedtls_camellia_crypt_cbc( &camellia, MBEDTLS_CAMELLIA_ENCRYPT,
                        BUFSIZE, tmp, buf, buf ) );
        }
        mbedtls_camellia_free( &camellia );
    }
#endif

#if defined(MBEDTLS_BLOWFISH_C) && defined(MBEDTLS_CIPHER_MODE_CBC)
    if( todo.blowfish )
    {
        int keysize;
        mbedtls_blowfish_context blowfish;
        mbedtls_blowfish_init( &blowfish );

        for( keysize = 128; keysize <= 256; keysize += 64 )
        {
            mbedtls_snprintf( title, sizeof( title ), "BLOWFISH-CBC-%d", keysize );

            memset( buf, 0, sizeof( buf ) );
            memset( tmp, 0, sizeof( tmp ) );
            mbedtls_blowfish_setkey( &blowfish, tmp, keysize );

            TIME_AND_TSC( title,
                    mbedtls_blowfish_crypt_cbc( &blowfish, MBEDTLS_BLOWFISH_ENCRYPT, BUFSIZE,
                        tmp, buf, buf ) );
        }

        mbedtls_blowfish_free( &blowfish );
    }
#endif

#if defined(MBEDTLS_HAVEGE_C)
    if( todo.havege )
    {
        mbedtls_havege_state hs;
        mbedtls_havege_init( &hs );
        TIME_AND_TSC( "HAVEGE", mbedtls_havege_random( &hs, buf, BUFSIZE ) );
        mbedtls_havege_free( &hs );
    }
#endif

#if defined(MBEDTLS_CTR_DRBG_C)
    if( todo.ctr_drbg )
    {
        mbedtls_ctr_drbg_context ctr_drbg;

        mbedtls_ctr_drbg_init( &ctr_drbg );

        if( mbedtls_ctr_drbg_seed( &ctr_drbg, myrand, NULL, NULL, 0 ) != 0 )
            mbedtls_exit(1);
        TIME_AND_TSC( "CTR_DRBG (NOPR)",
                if( mbedtls_ctr_drbg_random( &ctr_drbg, buf, BUFSIZE ) != 0 )
                mbedtls_exit(1) );

        if( mbedtls_ctr_drbg_seed( &ctr_drbg, myrand, NULL, NULL, 0 ) != 0 )
            mbedtls_exit(1);
        mbedtls_ctr_drbg_set_prediction_resistance( &ctr_drbg, MBEDTLS_CTR_DRBG_PR_ON );
        TIME_AND_TSC( "CTR_DRBG (PR)",
                if( mbedtls_ctr_drbg_random( &ctr_drbg, buf, BUFSIZE ) != 0 )
                mbedtls_exit(1) );
        mbedtls_ctr_drbg_free( &ctr_drbg );
    }
#endif

#if defined(MBEDTLS_HMAC_DRBG_C)
    if( todo.hmac_drbg )
    {
        mbedtls_hmac_drbg_context hmac_drbg;
        const mbedtls_md_info_t *md_info;

        mbedtls_hmac_drbg_init( &hmac_drbg );

#if defined(MBEDTLS_SHA1_C)
        if( ( md_info = mbedtls_md_info_from_type( MBEDTLS_MD_SHA1 ) ) == NULL )
            mbedtls_exit(1);

        if( mbedtls_hmac_drbg_seed( &hmac_drbg, md_info, myrand, NULL, NULL, 0 ) != 0 )
            mbedtls_exit(1);
        TIME_AND_TSC( "HMAC_DRBG SHA-1 (NOPR)",
                if( mbedtls_hmac_drbg_random( &hmac_drbg, buf, BUFSIZE ) != 0 )
                mbedtls_exit(1) );
        mbedtls_hmac_drbg_free( &hmac_drbg );

        if( mbedtls_hmac_drbg_seed( &hmac_drbg, md_info, myrand, NULL, NULL, 0 ) != 0 )
            mbedtls_exit(1);
        mbedtls_hmac_drbg_set_prediction_resistance( &hmac_drbg,
                                             MBEDTLS_HMAC_DRBG_PR_ON );
        TIME_AND_TSC( "HMAC_DRBG SHA-1 (PR)",
                if( mbedtls_hmac_drbg_random( &hmac_drbg, buf, BUFSIZE ) != 0 )
                mbedtls_exit(1) );
        mbedtls_hmac_drbg_free( &hmac_drbg );
#endif

#if defined(MBEDTLS_SHA256_C)
        if( ( md_info = mbedtls_md_info_from_type( MBEDTLS_MD_SHA256 ) ) == NULL )
            mbedtls_exit(1);

        if( mbedtls_hmac_drbg_seed( &hmac_drbg, md_info, myrand, NULL, NULL, 0 ) != 0 )
            mbedtls_exit(1);
        TIME_AND_TSC( "HMAC_DRBG SHA-256 (NOPR)",
                if( mbedtls_hmac_drbg_random( &hmac_drbg, buf, BUFSIZE ) != 0 )
                mbedtls_exit(1) );
        mbedtls_hmac_drbg_free( &hmac_drbg );

        if( mbedtls_hmac_drbg_seed( &hmac_drbg, md_info, myrand, NULL, NULL, 0 ) != 0 )
            mbedtls_exit(1);
        mbedtls_hmac_drbg_set_prediction_resistance( &hmac_drbg,
                                             MBEDTLS_HMAC_DRBG_PR_ON );
        TIME_AND_TSC( "HMAC_DRBG SHA-256 (PR)",
                if( mbedtls_hmac_drbg_random( &hmac_drbg, buf, BUFSIZE ) != 0 )
                mbedtls_exit(1) );
        mbedtls_hmac_drbg_free( &hmac_drbg );
#endif
    }
#endif

#if defined(MBEDTLS_RSA_C) && \
    defined(MBEDTLS_PEM_PARSE_C) && defined(MBEDTLS_PK_PARSE_C)
    if( todo.rsa )
    {
        mbedtls_pk_context pk;
        mbedtls_rsa_context *rsa;
        const char *rsa_keys[] = { RSA_PRIVATE_KEY_2048, RSA_PRIVATE_KEY_4096 };
        size_t i;

        for( i = 0; i < sizeof( rsa_keys ) / sizeof( rsa_keys[0] ); i++ )
        {
            mbedtls_pk_init( &pk );
            mbedtls_pk_parse_key( &pk, (const unsigned char *) rsa_keys[i],
                                                       strlen( rsa_keys[i] ) + 1, NULL, 0 );
            rsa = mbedtls_pk_rsa( pk );

            mbedtls_snprintf( title, sizeof( title ), "RSA-%d", mbedtls_pk_get_bitlen( &pk ) );

            TIME_PUBLIC( title, " public",
                    buf[0] = 0;
                    ret = mbedtls_rsa_public( rsa, buf, buf ) );

            TIME_PUBLIC( title, "private",
                    buf[0] = 0;
                    ret = mbedtls_rsa_private( rsa, myrand, NULL, buf, buf ) );

            mbedtls_pk_free( &pk );
        }
    }
#endif

#if defined(MBEDTLS_DHM_C) && defined(MBEDTLS_BIGNUM_C)
    if( todo.dhm )
    {
        int dhm_sizes[] = { 2048, 3072 };
        const char *dhm_P[] = {
            MBEDTLS_DHM_RFC3526_MODP_2048_P,
            MBEDTLS_DHM_RFC3526_MODP_3072_P,
        };
        const char *dhm_G[] = {
            MBEDTLS_DHM_RFC3526_MODP_2048_G,
            MBEDTLS_DHM_RFC3526_MODP_3072_G,
        };

        mbedtls_dhm_context dhm;
        size_t olen;
        for( i = 0; (size_t) i < sizeof( dhm_sizes ) / sizeof( dhm_sizes[0] ); i++ )
        {
            mbedtls_dhm_init( &dhm );

            if( mbedtls_mpi_read_string( &dhm.P, 16, dhm_P[i] ) != 0 ||
                mbedtls_mpi_read_string( &dhm.G, 16, dhm_G[i] ) != 0 )
            {
                mbedtls_exit( 1 );
            }

            dhm.len = mbedtls_mpi_size( &dhm.P );
            mbedtls_dhm_make_public( &dhm, (int) dhm.len, buf, dhm.len, myrand, NULL );
            if( mbedtls_mpi_copy( &dhm.GY, &dhm.GX ) != 0 )
                mbedtls_exit( 1 );

            mbedtls_snprintf( title, sizeof( title ), "DHE-%d", dhm_sizes[i] );
            TIME_PUBLIC( title, "handshake",
                    ret |= mbedtls_dhm_make_public( &dhm, (int) dhm.len, buf, dhm.len,
                                            myrand, NULL );
                    ret |= mbedtls_dhm_calc_secret( &dhm, buf, sizeof( buf ), &olen, myrand, NULL ) );

            mbedtls_snprintf( title, sizeof( title ), "DH-%d", dhm_sizes[i] );
            TIME_PUBLIC( title, "handshake",
                    ret |= mbedtls_dhm_calc_secret( &dhm, buf, sizeof( buf ), &olen, myrand, NULL ) );

            mbedtls_dhm_free( &dhm );
        }
    }
#endif

#if defined(MBEDTLS_ECDSA_C) && defined(MBEDTLS_SHA256_C)
    if( todo.ecdsa )
    {
        mbedtls_ecdsa_context ecdsa;
        const mbedtls_ecp_curve_info *curve_info;
        size_t sig_len;

        memset( buf, 0x2A, sizeof( buf ) );

        for( curve_info = mbedtls_ecp_curve_list();
             curve_info->grp_id != MBEDTLS_ECP_DP_NONE;
             curve_info++ )
        {
            mbedtls_ecdsa_init( &ecdsa );

            if( mbedtls_ecdsa_genkey( &ecdsa, curve_info->grp_id, myrand, NULL ) != 0 )
                mbedtls_exit( 1 );
            ecp_clear_precomputed( &ecdsa.grp );

            mbedtls_snprintf( title, sizeof( title ), "ECDSA-%s",
                                              curve_info->name );
            TIME_PUBLIC( title, "sign",
                    ret = mbedtls_ecdsa_write_signature( &ecdsa, MBEDTLS_MD_SHA256, buf, curve_info->bit_size,
                                                tmp, &sig_len, myrand, NULL ) );

            mbedtls_ecdsa_free( &ecdsa );
        }

        for( curve_info = mbedtls_ecp_curve_list();
             curve_info->grp_id != MBEDTLS_ECP_DP_NONE;
             curve_info++ )
        {
            mbedtls_ecdsa_init( &ecdsa );

            if( mbedtls_ecdsa_genkey( &ecdsa, curve_info->grp_id, myrand, NULL ) != 0 ||
                mbedtls_ecdsa_write_signature( &ecdsa, MBEDTLS_MD_SHA256, buf, curve_info->bit_size,
                                               tmp, &sig_len, myrand, NULL ) != 0 )
            {
                mbedtls_exit( 1 );
            }
            ecp_clear_precomputed( &ecdsa.grp );

            mbedtls_snprintf( title, sizeof( title ), "ECDSA-%s",
                                              curve_info->name );
            TIME_PUBLIC( title, "verify",
                    ret = mbedtls_ecdsa_read_signature( &ecdsa, buf, curve_info->bit_size,
                                                tmp, sig_len ) );

            mbedtls_ecdsa_free( &ecdsa );
        }
    }
#endif

#if defined(MBEDTLS_ECDH_C)
    if( todo.ecdh )
    {
        mbedtls_ecdh_context ecdh;
#if defined(MBEDTLS_ECP_DP_CURVE25519_ENABLED)
        mbedtls_mpi z;
#endif
        const mbedtls_ecp_curve_info *curve_info;
        size_t olen;

        for( curve_info = mbedtls_ecp_curve_list();
             curve_info->grp_id != MBEDTLS_ECP_DP_NONE;
             curve_info++ )
        {
            mbedtls_ecdh_init( &ecdh );

            if( mbedtls_ecp_group_load( &ecdh.grp, curve_info->grp_id ) != 0 ||
                mbedtls_ecdh_make_public( &ecdh, &olen, buf, sizeof( buf),
                                  myrand, NULL ) != 0 ||
                mbedtls_ecp_copy( &ecdh.Qp, &ecdh.Q ) != 0 )
            {
                mbedtls_exit( 1 );
            }
            ecp_clear_precomputed( &ecdh.grp );

            mbedtls_snprintf( title, sizeof( title ), "ECDHE-%s",
                                              curve_info->name );
            TIME_PUBLIC( title, "handshake",
                    ret |= mbedtls_ecdh_make_public( &ecdh, &olen, buf, sizeof( buf),
                                             myrand, NULL );
                    ret |= mbedtls_ecdh_calc_secret( &ecdh, &olen, buf, sizeof( buf ),
                                             myrand, NULL ) );
            mbedtls_ecdh_free( &ecdh );
        }

        /* Curve25519 needs to be handled separately */
#if defined(MBEDTLS_ECP_DP_CURVE25519_ENABLED)
        mbedtls_ecdh_init( &ecdh );
        mbedtls_mpi_init( &z );

        if( mbedtls_ecp_group_load( &ecdh.grp, MBEDTLS_ECP_DP_CURVE25519 ) != 0 ||
            mbedtls_ecdh_gen_public( &ecdh.grp, &ecdh.d, &ecdh.Qp, myrand, NULL ) != 0 )
        {
            mbedtls_exit( 1 );
        }

        TIME_PUBLIC(  "ECDHE-Curve25519", "handshake",
                ret |= mbedtls_ecdh_gen_public( &ecdh.grp, &ecdh.d, &ecdh.Q,
                                        myrand, NULL );
                ret |= mbedtls_ecdh_compute_shared( &ecdh.grp, &z, &ecdh.Qp, &ecdh.d,
                                            myrand, NULL ) );

        mbedtls_ecdh_free( &ecdh );
        mbedtls_mpi_free( &z );
#endif

        for( curve_info = mbedtls_ecp_curve_list();
             curve_info->grp_id != MBEDTLS_ECP_DP_NONE;
             curve_info++ )
        {
            mbedtls_ecdh_init( &ecdh );

            if( mbedtls_ecp_group_load( &ecdh.grp, curve_info->grp_id ) != 0 ||
                mbedtls_ecdh_make_public( &ecdh, &olen, buf, sizeof( buf),
                                  myrand, NULL ) != 0 ||
                mbedtls_ecp_copy( &ecdh.Qp, &ecdh.Q ) != 0 ||
                mbedtls_ecdh_make_public( &ecdh, &olen, buf, sizeof( buf),
                                  myrand, NULL ) != 0 )
            {
                mbedtls_exit( 1 );
            }
            ecp_clear_precomputed( &ecdh.grp );

            mbedtls_snprintf( title, sizeof( title ), "ECDH-%s",
                                              curve_info->name );
            TIME_PUBLIC( title, "handshake",
                    ret |= mbedtls_ecdh_calc_secret( &ecdh, &olen, buf, sizeof( buf ),
                                             myrand, NULL ) );
            mbedtls_ecdh_free( &ecdh );
        }

        /* Curve25519 needs to be handled separately */
#if defined(MBEDTLS_ECP_DP_CURVE25519_ENABLED)
        mbedtls_ecdh_init( &ecdh );
        mbedtls_mpi_init( &z );

        if( mbedtls_ecp_group_load( &ecdh.grp, MBEDTLS_ECP_DP_CURVE25519 ) != 0 ||
            mbedtls_ecdh_gen_public( &ecdh.grp, &ecdh.d, &ecdh.Qp,
                             myrand, NULL ) != 0 ||
            mbedtls_ecdh_gen_public( &ecdh.grp, &ecdh.d, &ecdh.Q, myrand, NULL ) != 0 )
        {
            mbedtls_exit( 1 );
        }

        TIME_PUBLIC(  "ECDH-Curve25519", "handshake",
                ret |= mbedtls_ecdh_compute_shared( &ecdh.grp, &z, &ecdh.Qp, &ecdh.d,
                                            myrand, NULL ) );

        mbedtls_ecdh_free( &ecdh );
        mbedtls_mpi_free( &z );
#endif
    }
#endif

    mbedtls_printf( "\r\n" );

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
    mbedtls_memory_buffer_alloc_free();
#endif

#if defined(_WIN32)
    mbedtls_printf( "  Press Enter to exit this program.\r\n" );
    fflush( stdout ); getchar();
#endif

    return( 0 );
}

#include "mbed-drivers/test_env.h"
#include "minar/minar.h"

static void run() {
    MBED_HOSTTEST_TIMEOUT(150);
    MBED_HOSTTEST_SELECT(default);
    MBED_HOSTTEST_DESCRIPTION(mbed TLS benchmark program);
    MBED_HOSTTEST_START("MBEDTLS_BENCHMARK");
    MBED_HOSTTEST_RESULT(benchmark(0, NULL) == 0);
}

void app_start(int, char*[]) {
    /* Use 115200 bps for consistency with other examples */
    get_stdio_serial().baud(115200);
    minar::Scheduler::postCallback(mbed::util::FunctionPointer0<void>(run).bind());
}

#endif /* TARGET_LIKE_MBED */
