/*
 *  Hello world example of using the hashing functions of mbed TLS
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

/*
 * This program illustrates various ways of hashing a buffer.
 * You normally need only one of these two includes.
 */
#include "mbedtls/sha256.h" /* SHA-256 only */
#include "mbedtls/md.h"     /* generic interface */

#if defined(TARGET_LIKE_MBED)
#include "mbed-drivers/mbed.h"
#endif
#include <cstdio>

static void print_hex(const char *title, const unsigned char buf[], size_t len)
{
    printf("%s: ", title);

    for (size_t i = 0; i < len; i++)
        printf("%02x", buf[i]);

    printf("\r\n");
}

static const char hello_str[] = "Hello, world!";
static const unsigned char *hello_buffer = (const unsigned char *) hello_str;
static const size_t hello_len = sizeof hello_str - 1;

int example(void)
{
    printf( "\r\n\r\n" );

    /*
     * Method 1: use all-in-one function of a specific SHA-xxx module
     */
    unsigned char output1[32]; /* SHA-256 outputs 32 bytes */

    /* 0 here means use the full SHA-256, not the SHA-224 variant */
    mbedtls_sha256(hello_buffer, hello_len, output1, 0);

    print_hex("Method 1", output1, sizeof output1);


    /*
     * Method 2: use the streaming interface of a specific SHA-xxx module
     * This is useful if we get our input piecewise.
     */
    unsigned char output2[32];
    mbedtls_sha256_context ctx2;

    mbedtls_sha256_init(&ctx2);
    mbedtls_sha256_starts(&ctx2, 0); /* SHA-256, not 224 */

    /* Simulating multiple fragments */
    mbedtls_sha256_update(&ctx2, hello_buffer, 1);
    mbedtls_sha256_update(&ctx2, hello_buffer + 1, 1);
    mbedtls_sha256_update(&ctx2, hello_buffer + 2, hello_len - 2);

    mbedtls_sha256_finish(&ctx2, output2);
    print_hex("Method 2", output2, sizeof output2);

    /* Or you could re-use the context by doing mbedtls_sha256_starts() again */
    mbedtls_sha256_free(&ctx2);

    /*
     * Method 3: use all-in-one function of the generice interface
     */
    unsigned char output3[MBEDTLS_MD_MAX_SIZE]; /* Enough for any hash */

    /* Can easily pick any hash you want, by identifier */
    const mbedtls_md_info_t *md_info3 = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

    if (md_info3 == NULL)
    {
        printf("SHA256 not available\r\n");
        return 1;
    }

    int ret3 = mbedtls_md(md_info3, hello_buffer, hello_len, output3);

    if (ret3 != 0)
    {
        printf("md() returned -0x%04X\r\n", -ret3);
        return 1;
    }

    print_hex("Method 3", output3, mbedtls_md_get_size(md_info3));


    /*
     * Method 4: streaming & generic interface
     */
    unsigned char output4[MBEDTLS_MD_MAX_SIZE]; /* Enough for any hash */

    const mbedtls_md_info_t *md_info4 = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

    if (md_info4 == NULL)
    {
        printf("SHA256 not available\r\n");
        return 1;
    }

    mbedtls_md_context_t ctx4;

    mbedtls_md_init(&ctx4);

    int ret4 = mbedtls_md_init_ctx(&ctx4, md_info4);
    if (ret4 != 0)
    {
        printf("md_init_ctx() returned -0x%04X\r\n", -ret4);
        return 1;
    }

    mbedtls_md_starts(&ctx4);

    /* Simulating multiple fragments */
    mbedtls_md_update(&ctx4, hello_buffer, 1);
    mbedtls_md_update(&ctx4, hello_buffer + 1, 1);
    mbedtls_md_update(&ctx4, hello_buffer + 2, hello_len - 2);

    mbedtls_md_finish(&ctx4, output4);
    print_hex("Method 4", output4, mbedtls_md_get_size(md_info4));

    /* Or you could re-use the context by doing mbedtls_md_starts() again */
    mbedtls_md_free(&ctx4);


    printf("\r\nDONE\r\n");

    return 0;
}

#if defined(TARGET_LIKE_MBED)

#include "mbed-drivers/test_env.h"
#include "minar/minar.h"

static void run() {
    MBED_HOSTTEST_TIMEOUT(10);
    MBED_HOSTTEST_SELECT(default);
    MBED_HOSTTEST_DESCRIPTION(mbed TLS example on hashing);
    MBED_HOSTTEST_START("MBEDTLS_EX_HASHING");
    MBED_HOSTTEST_RESULT(example() == 0);
}

void app_start(int, char*[]) {
    /* Use 115200 bps for consistency with other examples */
    get_stdio_serial().baud(115200);
    minar::Scheduler::postCallback(mbed::util::FunctionPointer0<void>(run).bind());
}

#else

int main() {
    return example();
}

#endif
