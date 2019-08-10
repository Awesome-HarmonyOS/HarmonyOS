/*----------------------------------------------------------------------------
 * Copyright (c) <2016-2018>, <Huawei Technologies Co., Ltd>
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or other materials
 * provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific prior written
 * permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *---------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------
 * Notice of Export Control Law
 * ===============================================
 * Huawei LiteOS may be subject to applicable export control laws and regulations, which might
 * include those applicable to Huawei LiteOS of U.S. and the country in which you are located.
 * Import, export and usage of Huawei LiteOS in any manner by you shall be in compliance with such
 * applicable export control laws and regulations.
 *---------------------------------------------------------------------------*/
#ifdef INCLUDE_PACK_OPTION_FILE
#include "package_sha256.h"
#include <string.h>
#include "../package_head.h"/*lint !e451*/

static void pack_sha256_reset(pack_checksum_alg_s *thi)
{
    pack_sha256_s *sha256 = (pack_sha256_s *)thi;
    mbedtls_sha256_init(&sha256->sha256_context);
    mbedtls_sha256_starts(&sha256->sha256_context, false);
}
static int pack_sha256_update(pack_checksum_alg_s *thi, const uint8_t *buff, uint16_t len)
{
    pack_sha256_s *sha256 = (pack_sha256_s *)thi;
    mbedtls_sha256_update(&sha256->sha256_context, buff, len);
    return PACK_OK;
}
static int pack_sha256_check(pack_checksum_alg_s *thi, const uint8_t  *checksum, uint16_t checksum_len)
{
    uint8_t real_value[32];
    pack_sha256_s *sha256 = (pack_sha256_s *)thi;

    ASSERT_THIS(return PACK_ERR);

    if(sizeof(real_value) != checksum_len)
    {
        PACK_LOG("len %d not the same", checksum_len);
        return PACK_ERR;
    }
    mbedtls_sha256_finish(&sha256->sha256_context, real_value);
    if(memcmp(real_value, checksum, checksum_len) != 0)
    {
        PACK_LOG("checksum err");
        return PACK_ERR;
    }
    return PACK_OK;
}

static void pack_sha256_destroy(struct pack_checksum_alg_tag_s *thi)
{
    pack_sha256_s *sha256 = (pack_sha256_s *)thi;
    mbedtls_sha256_free(&sha256->sha256_context);
}

int pack_sha256_init(pack_sha256_s *thi)
{
    thi->base.reset = pack_sha256_reset;
    thi->base.update = pack_sha256_update;
    thi->base.check = pack_sha256_check;
    thi->base.destroy = pack_sha256_destroy;
    pack_sha256_reset(&thi->base);
    return PACK_OK;
}
#endif

