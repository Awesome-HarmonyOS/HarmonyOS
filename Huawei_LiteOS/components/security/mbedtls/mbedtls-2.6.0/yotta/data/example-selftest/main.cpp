/*
 *  Self-test demonstration program
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

#if !defined(POLARSSL_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#include "mbedtls/entropy.h"
#include "mbedtls/hmac_drbg.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/dhm.h"
#include "mbedtls/gcm.h"
#include "mbedtls/ccm.h"
#include "mbedtls/md2.h"
#include "mbedtls/md4.h"
#include "mbedtls/md5.h"
#include "mbedtls/ripemd160.h"
#include "mbedtls/sha1.h"
#include "mbedtls/sha256.h"
#include "mbedtls/sha512.h"
#include "mbedtls/arc4.h"
#include "mbedtls/des.h"
#include "mbedtls/aes.h"
#include "mbedtls/camellia.h"
#include "mbedtls/base64.h"
#include "mbedtls/bignum.h"
#include "mbedtls/rsa.h"
#include "mbedtls/x509.h"
#include "mbedtls/xtea.h"
#include "mbedtls/pkcs5.h"
#include "mbedtls/ecp.h"

#include <stdio.h>
#include <string.h>

#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#define mbedtls_printf     printf
#endif

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
#include "mbedtls/memory_buffer_alloc.h"
#endif

int selftest( int argc, char *argv[] )
{
    int ret = 0, v;
#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
    unsigned char buf[1000000];
#endif

    if( argc == 2 && strcmp( argv[1], "-quiet" ) == 0 )
        v = 0;
    else
    {
        v = 1;
        mbedtls_printf( "\n" );
    }

#if defined(MBEDTLS_SELF_TEST)

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
    mbedtls_memory_buffer_alloc_init( buf, sizeof(buf) );
#endif

#if defined(MBEDTLS_MD2_C)
    if( ( ret = mbedtls_md2_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_MD4_C)
    if( ( ret = mbedtls_md4_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_MD5_C)
    if( ( ret = mbedtls_md5_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_RIPEMD160_C)
    if( ( ret = mbedtls_ripemd160_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_SHA1_C)
    if( ( ret = mbedtls_sha1_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_SHA256_C)
    if( ( ret = mbedtls_sha256_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_SHA512_C)
    if( ( ret = mbedtls_sha512_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_ARC4_C)
    if( ( ret = mbedtls_arc4_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_DES_C)
    if( ( ret = mbedtls_des_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_AES_C)
    if( ( ret = mbedtls_aes_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_GCM_C) && defined(MBEDTLS_AES_C)
    if( ( ret = mbedtls_gcm_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_CCM_C) && defined(MBEDTLS_AES_C)
    if( ( ret = mbedtls_ccm_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_BASE64_C)
    if( ( ret = mbedtls_base64_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_BIGNUM_C)
    if( ( ret = mbedtls_mpi_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_RSA_C)
    if( ( ret = mbedtls_rsa_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_X509_USE_C)
    if( ( ret = mbedtls_x509_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_XTEA_C)
    if( ( ret = mbedtls_xtea_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_CAMELLIA_C)
    if( ( ret = mbedtls_camellia_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_CTR_DRBG_C)
    if( ( ret = mbedtls_ctr_drbg_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_HMAC_DRBG_C)
    if( ( ret = mbedtls_hmac_drbg_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_ECP_C)
    if( ( ret = mbedtls_ecp_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_DHM_C)
    if( ( ret = mbedtls_dhm_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_ENTROPY_C)
    if( ( ret = mbedtls_entropy_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_PKCS5_C)
    if( ( ret = mbedtls_pkcs5_self_test( v ) ) != 0 )
        return( ret );
#endif

#if defined(MBEDTLS_TIMING_C)
    if( ( ret = mbedtls_timing_self_test( v ) ) != 0 )
        return( ret );
#endif

#else
    mbedtls_printf( " POLARSSL_SELF_TEST not defined.\n" );
#endif

    if( v != 0 )
    {
#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C) && defined(MBEDTLS_MEMORY_DEBUG)
        mbedtls_memory_buffer_alloc_status();
#endif
    }

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
    mbedtls_memory_buffer_alloc_free();

    if( ( ret = mbedtls_memory_buffer_alloc_self_test( v ) ) != 0 )
        return( ret );
#endif

    if( v != 0 )
    {
        mbedtls_printf( "  [ All tests passed ]\n\n" );
#if defined(_WIN32)
        mbedtls_printf( "  Press Enter to exit this program.\n" );
        fflush( stdout ); getchar();
#endif
    }

    return( ret );
}

#if defined(TARGET_LIKE_MBED)

#include "mbed-drivers/test_env.h"
#include "minar/minar.h"

static void run() {
    MBED_HOSTTEST_TIMEOUT(40);
    MBED_HOSTTEST_SELECT(default);
    MBED_HOSTTEST_DESCRIPTION(mbed TLS selftest program);
    MBED_HOSTTEST_START("MBEDTLS_SELFTEST");
    MBED_HOSTTEST_RESULT(selftest(0, NULL) == 0);
}

void app_start(int, char*[]) {
    /* Use 115200 bps for consistency with other examples */
    get_stdio_serial().baud(115200);
    minar::Scheduler::postCallback(mbed::util::FunctionPointer0<void>(run).bind());
}

#else

int main() {
    return selftest(0, NULL);
}

#endif
