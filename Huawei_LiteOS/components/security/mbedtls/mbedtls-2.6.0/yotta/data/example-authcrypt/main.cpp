/*
 *  Hello world example of using the authenticated encryption with mbed TLS
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

#include "mbedtls/cipher.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"

#include <stdio.h>
#include <string.h>

static void print_hex(const char *title, const unsigned char buf[], size_t len)
{
    printf("%s: ", title);

    for (size_t i = 0; i < len; i++)
        printf("%02x", buf[i]);

    printf("\r\n");
}

/*
 * The pre-shared key. Should be generated randomly and be unique to the
 * device/channel/etc. Just used a fixed on here for simplicity.
 */
static const unsigned char secret_key[16] = {
    0xf4, 0x82, 0xc6, 0x70, 0x3c, 0xc7, 0x61, 0x0a,
    0xb9, 0xa0, 0xb8, 0xe9, 0x87, 0xb8, 0xc1, 0x72,
};

static int example(void)
{
    /* message that should be protected */
    const char message[] = "Some things are better left unread";
    /* metadata transmitted in the clear but authenticated */
    const char metadata[] = "eg sequence number, routing info";
    /* ciphertext buffer large enough to hold message + nonce + tag */
    unsigned char ciphertext[128] = { 0 };
    int ret;

    printf("\r\n\r\n");
    print_hex("plaintext message", (unsigned char *) message, sizeof message);

    /*
     * Setup random number generator
     * (Note: later this might be done automatically.)
     */
    mbedtls_entropy_context entropy;    /* entropy pool for seeding PRNG */
    mbedtls_ctr_drbg_context drbg;      /* pseudo-random generator */

    mbedtls_entropy_init(&entropy);
    mbedtls_ctr_drbg_init(&drbg);

    /* Seed the PRNG using the entropy pool, and throw in our secret key as an
     * additional source of randomness. */
    ret = mbedtls_ctr_drbg_seed(&drbg, mbedtls_entropy_func, &entropy,
                                       secret_key, sizeof (secret_key));
    if (ret != 0) {
        printf("mbedtls_ctr_drbg_init() returned -0x%04X\r\n", -ret);
        return 1;
    }

    /*
     * Setup AES-CCM contex
     */
    mbedtls_cipher_context_t ctx;

    mbedtls_cipher_init(&ctx);

    ret = mbedtls_cipher_setup(&ctx, mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_CCM));
    if (ret != 0) {
        printf("mbedtls_cipher_setup() returned -0x%04X\r\n", -ret);
        return 1;
    }

    ret = mbedtls_cipher_setkey(&ctx, secret_key, 8 * sizeof secret_key, MBEDTLS_ENCRYPT);
    if (ret != 0) {
        printf("mbedtls_cipher_setkey() returned -0x%04X\r\n", -ret);
        return 1;
    }

    /*
     * Encrypt-authenticate the message and authenticate additional data
     *
     * First generate a random 8-byte nonce.
     * Put it directly in the output buffer as the recipient will need it.
     *
     * Warning: you must never re-use the same (key, nonce) pair. One of the
     * best ways to ensure this to use a counter for the nonce. However this
     * means you should save the counter accross rebots, if the key is a
     * long-term one. The alternative we choose here is to generate the nonce
     * randomly. However it only works if you have a good source of
     * randomness.
     */
    const size_t nonce_len = 8;
    mbedtls_ctr_drbg_random(&drbg, ciphertext, nonce_len);

    size_t ciphertext_len = 0;
    /* Go for a conservative 16-byte (128-bit) tag
     * and append it to the ciphertext */
    const size_t tag_len = 16;
    ret = mbedtls_cipher_auth_encrypt(&ctx, ciphertext, nonce_len,
                              (const unsigned char *) metadata, sizeof metadata,
                              (const unsigned char *) message, sizeof message,
                              ciphertext + nonce_len, &ciphertext_len,
                              ciphertext + nonce_len + sizeof message, tag_len );
    if (ret != 0) {
        printf("mbedtls_cipher_auth_encrypt() returned -0x%04X\r\n", -ret);
        return 1;
    }
    ciphertext_len += nonce_len + tag_len;

    /*
     * The following information should now be transmitted:
     * - first ciphertext_len bytes of ciphertext buffer
     * - metadata if not already transmitted elsewhere
     */
    print_hex("ciphertext", ciphertext, ciphertext_len);

    /*
     * Decrypt-authenticate
     */
    unsigned char decrypted[128] = { 0 };
    size_t decrypted_len = 0;

    ret = mbedtls_cipher_setkey(&ctx, secret_key, 8 * sizeof secret_key, MBEDTLS_DECRYPT);
    if (ret != 0) {
        printf("mbedtls_cipher_setkey() returned -0x%04X\r\n", -ret);
        return 1;
    }

    ret = mbedtls_cipher_auth_decrypt(&ctx,
                              ciphertext, nonce_len,
                              (const unsigned char *) metadata, sizeof metadata,
                              ciphertext + nonce_len, ciphertext_len - nonce_len - tag_len,
                              decrypted, &decrypted_len,
                              ciphertext + ciphertext_len - tag_len, tag_len );
    /* Checking the return code is CRITICAL for security here */
    if (ret == MBEDTLS_ERR_CIPHER_AUTH_FAILED) {
        printf("Something bad is happening! Data is not authentic!\r\n");
        return 1;
    }
    if (ret != 0) {
        printf("mbedtls_cipher_authdecrypt() returned -0x%04X\r\n", -ret);
        return 1;
    }

    print_hex("decrypted", decrypted, decrypted_len);

    printf("\r\nDONE\r\n");

    return 0;
}

#if defined(TARGET_LIKE_MBED)

#include "mbed-drivers/test_env.h"
#include "minar/minar.h"

static void run() {
    MBED_HOSTTEST_TIMEOUT(10);
    MBED_HOSTTEST_SELECT(default);
    MBED_HOSTTEST_DESCRIPTION(mbed TLS example authcrypt);
    MBED_HOSTTEST_START("MBEDTLS_EX_AUTHCRYPT");
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
