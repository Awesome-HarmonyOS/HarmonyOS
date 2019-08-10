/**
 *  \brief Generate random data into a file
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
#define mbedtls_fprintf    fprintf
#define mbedtls_printf     printf
#endif

#if defined(MBEDTLS_HAVEGE_C) && defined(MBEDTLS_FS_IO)
#include "mbedtls/havege.h"

#include <stdio.h>
#include <time.h>
#endif

#if !defined(MBEDTLS_HAVEGE_C) || !defined(MBEDTLS_FS_IO)
int main( void )
{
    mbedtls_printf("MBEDTLS_HAVEGE_C not defined.\n");
    return( 0 );
}
#else
int main( int argc, char *argv[] )
{
    FILE *f;
    time_t t;
    int i, k, ret = 0;
    mbedtls_havege_state hs;
    unsigned char buf[1024];

    if( argc < 2 )
    {
        mbedtls_fprintf( stderr, "usage: %s <output filename>\n", argv[0] );
        return( 1 );
    }

    if( ( f = fopen( argv[1], "wb+" ) ) == NULL )
    {
        mbedtls_printf( "failed to open '%s' for writing.\n", argv[1] );
        return( 1 );
    }

    mbedtls_havege_init( &hs );

    t = time( NULL );

    for( i = 0, k = 768; i < k; i++ )
    {
        if( mbedtls_havege_random( &hs, buf, sizeof( buf ) ) != 0 )
        {
            mbedtls_printf( "Failed to get random from source.\n" );

            ret = 1;
            goto exit;
        }

        fwrite( buf, sizeof( buf ), 1, f );

        mbedtls_printf( "Generating %ldkb of data in file '%s'... %04.1f" \
                "%% done\r", (long)(sizeof(buf) * k / 1024), argv[1], (100 * (float) (i + 1)) / k );
        fflush( stdout );
    }

    if( t == time( NULL ) )
        t--;

    mbedtls_printf(" \n ");

exit:
    mbedtls_havege_free( &hs );
    fclose( f );
    return( ret );
}
#endif /* MBEDTLS_HAVEGE_C */
