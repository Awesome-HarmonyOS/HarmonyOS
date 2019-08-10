/*
 *  Entropy accumulator implementation
 *
 *  Copyright (C) 2006-2016, ARM Limited, All Rights Reserved
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

#if defined(MBEDTLS_ENTROPY_C)

#if defined(MBEDTLS_TEST_NULL_ENTROPY)
#warning "**** WARNING!  MBEDTLS_TEST_NULL_ENTROPY defined! "
#warning "**** THIS BUILD HAS NO DEFINED ENTROPY SOURCES "
#warning "**** THIS BUILD IS *NOT* SUITABLE FOR PRODUCTION USE "
#endif

#include "mbedtls/entropy.h"
#include "mbedtls/entropy_poll.h"

#include <string.h>

#if defined(MBEDTLS_FS_IO)
#include <stdio.h>
#endif

#if defined(MBEDTLS_ENTROPY_NV_SEED)
#include "mbedtls/platform.h"
#endif

#if defined(MBEDTLS_SELF_TEST)
#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#define mbedtls_printf     printf
#endif /* MBEDTLS_PLATFORM_C */
#endif /* MBEDTLS_SELF_TEST */

#if defined(MBEDTLS_HAVEGE_C)
#include "mbedtls/havege.h"
#endif

/* Implementation that should never be optimized out by the compiler */
static void mbedtls_zeroize( void *v, size_t n ) {
    volatile unsigned char *p = v; while( n-- ) *p++ = 0;
}

#define ENTROPY_MAX_LOOP    256     /**< Maximum amount to loop before error */

void mbedtls_entropy_init( mbedtls_entropy_context *ctx )
{
    memset( ctx, 0, sizeof(mbedtls_entropy_context) );

#if defined(MBEDTLS_THREADING_C)
    mbedtls_mutex_init( &ctx->mutex );
#endif

#if defined(MBEDTLS_ENTROPY_SHA512_ACCUMULATOR)
    mbedtls_sha512_starts( &ctx->accumulator, 0 );
#else
    mbedtls_sha256_starts( &ctx->accumulator, 0 );
#endif
#if defined(MBEDTLS_HAVEGE_C)
    mbedtls_havege_init( &ctx->havege_data );
#endif

#if defined(MBEDTLS_TEST_NULL_ENTROPY)
    mbedtls_entropy_add_source( ctx, mbedtls_null_entropy_poll, NULL,
                                1, MBEDTLS_ENTROPY_SOURCE_STRONG );
#endif

#if !defined(MBEDTLS_NO_DEFAULT_ENTROPY_SOURCES)
#if !defined(MBEDTLS_NO_PLATFORM_ENTROPY)
    mbedtls_entropy_add_source( ctx, mbedtls_platform_entropy_poll, NULL,
                                MBEDTLS_ENTROPY_MIN_PLATFORM,
                                MBEDTLS_ENTROPY_SOURCE_STRONG );
#endif
#if defined(MBEDTLS_TIMING_C)
    mbedtls_entropy_add_source( ctx, mbedtls_hardclock_poll, NULL,
                                MBEDTLS_ENTROPY_MIN_HARDCLOCK,
                                MBEDTLS_ENTROPY_SOURCE_WEAK );
#endif
#if defined(MBEDTLS_HAVEGE_C)
    mbedtls_entropy_add_source( ctx, mbedtls_havege_poll, &ctx->havege_data,
                                MBEDTLS_ENTROPY_MIN_HAVEGE,
                                MBEDTLS_ENTROPY_SOURCE_STRONG );
#endif
#if defined(MBEDTLS_ENTROPY_HARDWARE_ALT)
    mbedtls_entropy_add_source( ctx, mbedtls_hardware_poll, NULL,
                                MBEDTLS_ENTROPY_MIN_HARDWARE,
                                MBEDTLS_ENTROPY_SOURCE_STRONG );
#endif
#if defined(MBEDTLS_ENTROPY_NV_SEED)
    mbedtls_entropy_add_source( ctx, mbedtls_nv_seed_poll, NULL,
                                MBEDTLS_ENTROPY_BLOCK_SIZE,
                                MBEDTLS_ENTROPY_SOURCE_STRONG );
#endif
#endif /* MBEDTLS_NO_DEFAULT_ENTROPY_SOURCES */
}

void mbedtls_entropy_free( mbedtls_entropy_context *ctx )
{
#if defined(MBEDTLS_HAVEGE_C)
    mbedtls_havege_free( &ctx->havege_data );
#endif
#if defined(MBEDTLS_THREADING_C)
    mbedtls_mutex_free( &ctx->mutex );
#endif
    mbedtls_zeroize( ctx, sizeof( mbedtls_entropy_context ) );
}

int mbedtls_entropy_add_source( mbedtls_entropy_context *ctx,
                        mbedtls_entropy_f_source_ptr f_source, void *p_source,
                        size_t threshold, int strong )
{
    int idx, ret = 0;

#if defined(MBEDTLS_THREADING_C)
    if( ( ret = mbedtls_mutex_lock( &ctx->mutex ) ) != 0 )
        return( ret );
#endif

    idx = ctx->source_count;
    if( idx >= MBEDTLS_ENTROPY_MAX_SOURCES )
    {
        ret = MBEDTLS_ERR_ENTROPY_MAX_SOURCES;
        goto exit;
    }

    ctx->source[idx].f_source  = f_source;
    ctx->source[idx].p_source  = p_source;
    ctx->source[idx].threshold = threshold;
    ctx->source[idx].strong    = strong;

    ctx->source_count++;

exit:
#if defined(MBEDTLS_THREADING_C)
    if( mbedtls_mutex_unlock( &ctx->mutex ) != 0 )
        return( MBEDTLS_ERR_THREADING_MUTEX_ERROR );
#endif

    return( ret );
}

/*
 * Entropy accumulator update
 */
static int entropy_update( mbedtls_entropy_context *ctx, unsigned char source_id,
                           const unsigned char *data, size_t len )
{
    unsigned char header[2];
    unsigned char tmp[MBEDTLS_ENTROPY_BLOCK_SIZE];
    size_t use_len = len;
    const unsigned char *p = data;

    if( use_len > MBEDTLS_ENTROPY_BLOCK_SIZE )
    {
#if defined(MBEDTLS_ENTROPY_SHA512_ACCUMULATOR)
        mbedtls_sha512( data, len, tmp, 0 );
#else
        mbedtls_sha256( data, len, tmp, 0 );
#endif
        p = tmp;
        use_len = MBEDTLS_ENTROPY_BLOCK_SIZE;
    }

    header[0] = source_id;
    header[1] = use_len & 0xFF;

#if defined(MBEDTLS_ENTROPY_SHA512_ACCUMULATOR)
    mbedtls_sha512_update( &ctx->accumulator, header, 2 );
    mbedtls_sha512_update( &ctx->accumulator, p, use_len );
#else
    mbedtls_sha256_update( &ctx->accumulator, header, 2 );
    mbedtls_sha256_update( &ctx->accumulator, p, use_len );
#endif

    return( 0 );
}

int mbedtls_entropy_update_manual( mbedtls_entropy_context *ctx,
                           const unsigned char *data, size_t len )
{
    int ret;

#if defined(MBEDTLS_THREADING_C)
    if( ( ret = mbedtls_mutex_lock( &ctx->mutex ) ) != 0 )
        return( ret );
#endif

    ret = entropy_update( ctx, MBEDTLS_ENTROPY_SOURCE_MANUAL, data, len );

#if defined(MBEDTLS_THREADING_C)
    if( mbedtls_mutex_unlock( &ctx->mutex ) != 0 )
        return( MBEDTLS_ERR_THREADING_MUTEX_ERROR );
#endif

    return( ret );
}

/*
 * Run through the different sources to add entropy to our accumulator
 */
static int entropy_gather_internal( mbedtls_entropy_context *ctx )
{
    int ret, i, have_one_strong = 0;
    unsigned char buf[MBEDTLS_ENTROPY_MAX_GATHER];
    size_t olen;

    if( ctx->source_count == 0 )
        return( MBEDTLS_ERR_ENTROPY_NO_SOURCES_DEFINED );

    /*
     * Run through our entropy sources
     */
    for( i = 0; i < ctx->source_count; i++ )
    {
        if( ctx->source[i].strong == MBEDTLS_ENTROPY_SOURCE_STRONG )
            have_one_strong = 1;

        olen = 0;
        if( ( ret = ctx->source[i].f_source( ctx->source[i].p_source,
                        buf, MBEDTLS_ENTROPY_MAX_GATHER, &olen ) ) != 0 )
        {
            return( ret );
        }

        /*
         * Add if we actually gathered something
         */
        if( olen > 0 )
        {
            entropy_update( ctx, (unsigned char) i, buf, olen );
            ctx->source[i].size += olen;
        }
    }

    if( have_one_strong == 0 )
        return( MBEDTLS_ERR_ENTROPY_NO_STRONG_SOURCE );

    return( 0 );
}

/*
 * Thread-safe wrapper for entropy_gather_internal()
 */
int mbedtls_entropy_gather( mbedtls_entropy_context *ctx )
{
    int ret;

#if defined(MBEDTLS_THREADING_C)
    if( ( ret = mbedtls_mutex_lock( &ctx->mutex ) ) != 0 )
        return( ret );
#endif

    ret = entropy_gather_internal( ctx );

#if defined(MBEDTLS_THREADING_C)
    if( mbedtls_mutex_unlock( &ctx->mutex ) != 0 )
        return( MBEDTLS_ERR_THREADING_MUTEX_ERROR );
#endif

    return( ret );
}

int mbedtls_entropy_func( void *data, unsigned char *output, size_t len )
{
    int ret, count = 0, i, done;
    mbedtls_entropy_context *ctx = (mbedtls_entropy_context *) data;
    unsigned char buf[MBEDTLS_ENTROPY_BLOCK_SIZE];

    if( len > MBEDTLS_ENTROPY_BLOCK_SIZE )
        return( MBEDTLS_ERR_ENTROPY_SOURCE_FAILED );

#if defined(MBEDTLS_ENTROPY_NV_SEED)
    /* Update the NV entropy seed before generating any entropy for outside
     * use.
     */
    if( ctx->initial_entropy_run == 0 )
    {
        ctx->initial_entropy_run = 1;
        if( ( ret = mbedtls_entropy_update_nv_seed( ctx ) ) != 0 )
            return( ret );
    }
#endif

#if defined(MBEDTLS_THREADING_C)
    if( ( ret = mbedtls_mutex_lock( &ctx->mutex ) ) != 0 )
        return( ret );
#endif

    /*
     * Always gather extra entropy before a call
     */
    do
    {
        if( count++ > ENTROPY_MAX_LOOP )
        {
            ret = MBEDTLS_ERR_ENTROPY_SOURCE_FAILED;
            goto exit;
        }

        if( ( ret = entropy_gather_internal( ctx ) ) != 0 )
            goto exit;

        done = 1;
        for( i = 0; i < ctx->source_count; i++ )
            if( ctx->source[i].size < ctx->source[i].threshold )
                done = 0;
    }
    while( ! done );

    memset( buf, 0, MBEDTLS_ENTROPY_BLOCK_SIZE );

#if defined(MBEDTLS_ENTROPY_SHA512_ACCUMULATOR)
    mbedtls_sha512_finish( &ctx->accumulator, buf );

    /*
     * Reset accumulator and counters and recycle existing entropy
     */
    memset( &ctx->accumulator, 0, sizeof( mbedtls_sha512_context ) );
    mbedtls_sha512_starts( &ctx->accumulator, 0 );
    mbedtls_sha512_update( &ctx->accumulator, buf, MBEDTLS_ENTROPY_BLOCK_SIZE );

    /*
     * Perform second SHA-512 on entropy
     */
    mbedtls_sha512( buf, MBEDTLS_ENTROPY_BLOCK_SIZE, buf, 0 );
#else /* MBEDTLS_ENTROPY_SHA512_ACCUMULATOR */
    mbedtls_sha256_finish( &ctx->accumulator, buf );

    /*
     * Reset accumulator and counters and recycle existing entropy
     */
    memset( &ctx->accumulator, 0, sizeof( mbedtls_sha256_context ) );
    mbedtls_sha256_starts( &ctx->accumulator, 0 );
    mbedtls_sha256_update( &ctx->accumulator, buf, MBEDTLS_ENTROPY_BLOCK_SIZE );

    /*
     * Perform second SHA-256 on entropy
     */
    mbedtls_sha256( buf, MBEDTLS_ENTROPY_BLOCK_SIZE, buf, 0 );
#endif /* MBEDTLS_ENTROPY_SHA512_ACCUMULATOR */

    for( i = 0; i < ctx->source_count; i++ )
        ctx->source[i].size = 0;

    memcpy( output, buf, len );

    ret = 0;

exit:
#if defined(MBEDTLS_THREADING_C)
    if( mbedtls_mutex_unlock( &ctx->mutex ) != 0 )
        return( MBEDTLS_ERR_THREADING_MUTEX_ERROR );
#endif

    return( ret );
}

#if defined(MBEDTLS_ENTROPY_NV_SEED)
int mbedtls_entropy_update_nv_seed( mbedtls_entropy_context *ctx )
{
    int ret = MBEDTLS_ERR_ENTROPY_FILE_IO_ERROR;
    unsigned char buf[ MBEDTLS_ENTROPY_MAX_SEED_SIZE ];

    /* Read new seed  and write it to NV */
    if( ( ret = mbedtls_entropy_func( ctx, buf, MBEDTLS_ENTROPY_BLOCK_SIZE ) ) != 0 )
        return( ret );

    if( mbedtls_nv_seed_write( buf, MBEDTLS_ENTROPY_BLOCK_SIZE ) < 0 )
        return( MBEDTLS_ERR_ENTROPY_FILE_IO_ERROR );

    /* Manually update the remaining stream with a separator value to diverge */
    memset( buf, 0, MBEDTLS_ENTROPY_BLOCK_SIZE );
    mbedtls_entropy_update_manual( ctx, buf, MBEDTLS_ENTROPY_BLOCK_SIZE );

    return( 0 );
}
#endif /* MBEDTLS_ENTROPY_NV_SEED */

#if defined(MBEDTLS_FS_IO)
int mbedtls_entropy_write_seed_file( mbedtls_entropy_context *ctx, const char *path )
{
    int ret = MBEDTLS_ERR_ENTROPY_FILE_IO_ERROR;
    FILE *f;
    unsigned char buf[MBEDTLS_ENTROPY_BLOCK_SIZE];

    if( ( f = fopen( path, "wb" ) ) == NULL )
        return( MBEDTLS_ERR_ENTROPY_FILE_IO_ERROR );

    if( ( ret = mbedtls_entropy_func( ctx, buf, MBEDTLS_ENTROPY_BLOCK_SIZE ) ) != 0 )
        goto exit;

    if( fwrite( buf, 1, MBEDTLS_ENTROPY_BLOCK_SIZE, f ) != MBEDTLS_ENTROPY_BLOCK_SIZE )
    {
        ret = MBEDTLS_ERR_ENTROPY_FILE_IO_ERROR;
        goto exit;
    }

    ret = 0;

exit:
    fclose( f );
    return( ret );
}

int mbedtls_entropy_update_seed_file( mbedtls_entropy_context *ctx, const char *path )
{
    FILE *f;
    size_t n;
    unsigned char buf[ MBEDTLS_ENTROPY_MAX_SEED_SIZE ];

    if( ( f = fopen( path, "rb" ) ) == NULL )
        return( MBEDTLS_ERR_ENTROPY_FILE_IO_ERROR );

    fseek( f, 0, SEEK_END );
    n = (size_t) ftell( f );
    fseek( f, 0, SEEK_SET );

    if( n > MBEDTLS_ENTROPY_MAX_SEED_SIZE )
        n = MBEDTLS_ENTROPY_MAX_SEED_SIZE;

    if( fread( buf, 1, n, f ) != n )
    {
        fclose( f );
        return( MBEDTLS_ERR_ENTROPY_FILE_IO_ERROR );
    }

    fclose( f );

    mbedtls_entropy_update_manual( ctx, buf, n );

    return( mbedtls_entropy_write_seed_file( ctx, path ) );
}
#endif /* MBEDTLS_FS_IO */

#if defined(MBEDTLS_SELF_TEST)
#if !defined(MBEDTLS_TEST_NULL_ENTROPY)
/*
 * Dummy source function
 */
static int entropy_dummy_source( void *data, unsigned char *output,
                                 size_t len, size_t *olen )
{
    ((void) data);

    memset( output, 0x2a, len );
    *olen = len;

    return( 0 );
}
#endif /* !MBEDTLS_TEST_NULL_ENTROPY */

#if defined(MBEDTLS_ENTROPY_HARDWARE_ALT)

static int mbedtls_entropy_source_self_test_gather( unsigned char *buf, size_t buf_len )
{
    int ret = 0;
    size_t entropy_len = 0;
    size_t olen = 0;
    size_t attempts = buf_len;

    while( attempts > 0 && entropy_len < buf_len )
    {
        if( ( ret = mbedtls_hardware_poll( NULL, buf + entropy_len,
            buf_len - entropy_len, &olen ) ) != 0 )
            return( ret );

        entropy_len += olen;
        attempts--;
    }

    if( entropy_len < buf_len )
    {
        ret = 1;
    }

    return( ret );
}


static int mbedtls_entropy_source_self_test_check_bits( const unsigned char *buf,
                                                        size_t buf_len )
{
    unsigned char set= 0xFF;
    unsigned char unset = 0x00;
    size_t i;

    for( i = 0; i < buf_len; i++ )
    {
        set &= buf[i];
        unset |= buf[i];
    }

    return( set == 0xFF || unset == 0x00 );
}

/*
 * A test to ensure hat the entropy sources are functioning correctly
 * and there is no obvious failure. The test performs the following checks:
 *  - The entropy source is not providing only 0s (all bits unset) or 1s (all
 *    bits set).
 *  - The entropy source is not providing values in a pattern. Because the
 *    hardware could be providing data in an arbitrary length, this check polls
 *    the hardware entropy source twice and compares the result to ensure they
 *    are not equal.
 *  - The error code returned by the entropy source is not an error.
 */
int mbedtls_entropy_source_self_test( int verbose )
{
    int ret = 0;
    unsigned char buf0[2 * sizeof( unsigned long long int )];
    unsigned char buf1[2 * sizeof( unsigned long long int )];

    if( verbose != 0 )
        mbedtls_printf( "  ENTROPY_BIAS test: " );

    memset( buf0, 0x00, sizeof( buf0 ) );
    memset( buf1, 0x00, sizeof( buf1 ) );

    if( ( ret = mbedtls_entropy_source_self_test_gather( buf0, sizeof( buf0 ) ) ) != 0 )
        goto cleanup;
    if( ( ret = mbedtls_entropy_source_self_test_gather( buf1, sizeof( buf1 ) ) ) != 0 )
        goto cleanup;

    /* Make sure that the returned values are not all 0 or 1 */
    if( ( ret = mbedtls_entropy_source_self_test_check_bits( buf0, sizeof( buf0 ) ) ) != 0 )
        goto cleanup;
    if( ( ret = mbedtls_entropy_source_self_test_check_bits( buf1, sizeof( buf1 ) ) ) != 0 )
        goto cleanup;

    /* Make sure that the entropy source is not returning values in a
     * pattern */
    ret = memcmp( buf0, buf1, sizeof( buf0 ) ) == 0;

cleanup:
    if( verbose != 0 )
    {
        if( ret != 0 )
            mbedtls_printf( "failed\n" );
        else
            mbedtls_printf( "passed\n" );

        mbedtls_printf( "\n" );
    }

    return( ret != 0 );
}

#endif /* MBEDTLS_ENTROPY_HARDWARE_ALT */

/*
 * The actual entropy quality is hard to test, but we can at least
 * test that the functions don't cause errors and write the correct
 * amount of data to buffers.
 */
int mbedtls_entropy_self_test( int verbose )
{
    int ret = 1;
#if !defined(MBEDTLS_TEST_NULL_ENTROPY)
    mbedtls_entropy_context ctx;
    unsigned char buf[MBEDTLS_ENTROPY_BLOCK_SIZE] = { 0 };
    unsigned char acc[MBEDTLS_ENTROPY_BLOCK_SIZE] = { 0 };
    size_t i, j;
#endif /* !MBEDTLS_TEST_NULL_ENTROPY */

    if( verbose != 0 )
        mbedtls_printf( "  ENTROPY test: " );

#if !defined(MBEDTLS_TEST_NULL_ENTROPY)
    mbedtls_entropy_init( &ctx );

    /* First do a gather to make sure we have default sources */
    if( ( ret = mbedtls_entropy_gather( &ctx ) ) != 0 )
        goto cleanup;

    ret = mbedtls_entropy_add_source( &ctx, entropy_dummy_source, NULL, 16,
                                      MBEDTLS_ENTROPY_SOURCE_WEAK );
    if( ret != 0 )
        goto cleanup;

    if( ( ret = mbedtls_entropy_update_manual( &ctx, buf, sizeof buf ) ) != 0 )
        goto cleanup;

    /*
     * To test that mbedtls_entropy_func writes correct number of bytes:
     * - use the whole buffer and rely on ASan to detect overruns
     * - collect entropy 8 times and OR the result in an accumulator:
     *   any byte should then be 0 with probably 2^(-64), so requiring
     *   each of the 32 or 64 bytes to be non-zero has a false failure rate
     *   of at most 2^(-58) which is acceptable.
     */
    for( i = 0; i < 8; i++ )
    {
        if( ( ret = mbedtls_entropy_func( &ctx, buf, sizeof( buf ) ) ) != 0 )
            goto cleanup;

        for( j = 0; j < sizeof( buf ); j++ )
            acc[j] |= buf[j];
    }

    for( j = 0; j < sizeof( buf ); j++ )
    {
        if( acc[j] == 0 )
        {
            ret = 1;
            goto cleanup;
        }
    }

#if defined(MBEDTLS_ENTROPY_HARDWARE_ALT)
    if( ( ret = mbedtls_entropy_source_self_test( 0 ) ) != 0 )
        goto cleanup;
#endif

cleanup:
    mbedtls_entropy_free( &ctx );
#endif /* !MBEDTLS_TEST_NULL_ENTROPY */

    if( verbose != 0 )
    {
        if( ret != 0 )
            mbedtls_printf( "failed\n" );
        else
            mbedtls_printf( "passed\n" );

        mbedtls_printf( "\n" );
    }

    return( ret != 0 );
}
#endif /* MBEDTLS_SELF_TEST */

#endif /* MBEDTLS_ENTROPY_C */
