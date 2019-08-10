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

#include "package_checksum.h"
#include "package_head.h"

#if (PACK_CHECKSUM != PACK_NO_CHECKSUM)
#include <string.h>

#if (PACK_CHECKSUM == PACK_SHA256_RSA2048)
#include "opt/package_sha256_rsa2048.h"
#elif (PACK_CHECKSUM == PACK_SHA256)
#include "opt/package_sha256.h"
#else
#error PACK_CHECKSUM not define
#endif




struct pack_checksum_tag_s
{
    uint32_t offset;
    bool offset_flag;
    pack_head_s *head;
#if (PACK_CHECKSUM == PACK_SHA256_RSA2048)
    pack_sha256_rsa2048_s alg;
#elif (PACK_CHECKSUM == PACK_SHA256)
    pack_sha256_s alg;
#endif

};

static inline pack_checksum_alg_s *pack_checksum_get_alg(pack_checksum_s *thi)
{
#if (PACK_CHECKSUM == PACK_SHA256_RSA2048)
    return &thi->alg.sha256.base;
#elif (PACK_CHECKSUM == PACK_SHA256)
    return &thi->alg.base;
#endif
}

static void pack_checksum_init(pack_checksum_s *thi, pack_head_s *head)
{
    memset(thi, 0, sizeof(*thi));
    thi->head = head;
#if (PACK_CHECKSUM == PACK_SHA256_RSA2048)
    (void)pack_sha256_rsa2048_init(&thi->alg, thi->head);
#elif (PACK_CHECKSUM == PACK_SHA256)
    (void)pack_sha256_init(&thi->alg);
#endif
}




void pack_checksum_delete(pack_checksum_s *thi)
{
    if(NULL == thi)
    {
        return;
    }
    pack_checksum_get_alg(thi)->destroy(pack_checksum_get_alg(thi));
    PACK_FREE(thi);
}

static int pack_checksum_init_head_data(pack_checksum_s *thi)
{
    int32_t len;
    const uint8_t *buff;

    pack_checksum_get_alg(thi)->reset(pack_checksum_get_alg(thi));
    len = pack_head_get_head_len(thi->head);
    if(0 == len)
    {
        return PACK_OK;
    }

    buff = pack_head_get_head_info(thi->head);
    if(NULL == buff)
    {
        PACK_LOG("buff null");
        return PACK_ERR;
    }

    return pack_checksum_get_alg(thi)->update(pack_checksum_get_alg(thi), buff, len);
}
pack_checksum_s *pack_checksum_create(pack_head_s *head)
{
    pack_checksum_s *thi = PACK_MALLOC(sizeof(pack_checksum_s));
    if(NULL == thi)
    {
        PACK_LOG("PACK_MALLOC fail");
        return NULL;
    }
    pack_checksum_init(thi, head);
    (void)pack_checksum_init_head_data(thi);
    return thi;
}

static int pack_checksum_restore_checksum(pack_checksum_s *thi, uint32_t offset, pack_hardware_s *hardware)
{
    uint8_t *buff  = NULL;
    const uint32_t max_size = hardware->get_block_size(hardware);
    uint32_t total_size = 0;
    uint32_t left_size;
    uint32_t read_size;
    int ret = PACK_ERR;

    buff = PACK_MALLOC(max_size);
    if(NULL == buff)
    {
        PACK_LOG("malloc null");
        return PACK_ERR;
    }
    do
    {
        ret = PACK_ERR;
        left_size = offset - total_size;
        read_size = MIN(left_size, max_size);
        ret = hardware->read_software(hardware, total_size, buff, read_size);
        if(ret != PACK_OK)
        {
            PACK_LOG("read_software fail, ret %d, offset %d, read_size %d", ret, total_size, read_size);
            break;
        }
        ret = pack_checksum_get_alg(thi)->update(pack_checksum_get_alg(thi), buff, read_size);
        if(ret != PACK_OK)
        {
            break;
        }
        total_size += read_size;
    }
    while(total_size < offset);

    if(buff)
    {
        PACK_FREE(buff);
    }

    return ret;
}

int pack_checksum_update_data(pack_checksum_s *thi, uint32_t offset, const uint8_t *buff,
                                   uint16_t len, pack_hardware_s *hardware)
{
    int ret;

    ASSERT_THIS(return PACK_ERR);

    if(0 == len)
    {
        return PACK_OK;
    }

    if(NULL == buff)
    {
        PACK_LOG("buff null");
        return PACK_ERR;
    }

    if(((thi->offset_flag) && (thi->offset == offset))
            || (pack_head_get_head_len(thi->head) == offset))
    {
        if(pack_checksum_get_alg(thi)->update(pack_checksum_get_alg(thi), buff, len) != PACK_OK)
        {
            return PACK_ERR;
        }
        thi->offset_flag = true;
        thi->offset = offset + len;
        return PACK_OK;
    }
    /*lint -e525*/
    if((NULL == hardware) || (NULL == hardware->read_software))
    {
        PACK_LOG("hardware null");
        return PACK_ERR;
    }
    /*lint +e525*/

    ret = pack_checksum_init_head_data(thi);
    if(ret != PACK_OK)
    {
        return ret;
    }

    ret = pack_checksum_restore_checksum(thi, offset, hardware);
    if(ret != PACK_OK)
    {
        return ret;
    }

    if(pack_checksum_get_alg(thi)->update(pack_checksum_get_alg(thi), buff, len) != PACK_OK)
    {
        return PACK_ERR;
    }

    thi->offset_flag = true;
    thi->offset = offset + len;
    return PACK_OK;
}

int pack_checksum_check(pack_checksum_s *thi, const uint8_t *expected_value, uint16_t len)
{
    ASSERT_THIS(return PACK_ERR);
    return pack_checksum_get_alg(thi)->check(pack_checksum_get_alg(thi), expected_value, len);
}
#define INCLUDE_PACK_OPTION_FILE
#if (PACK_CHECKSUM == PACK_SHA256_RSA2048)
#include "opt/package_sha256.c"
#include "opt/package_sha256_rsa2048.c"
#elif (PACK_CHECKSUM == PACK_SHA256)
#include "opt/package_sha256.c"
#endif

#else
pack_checksum_s * pack_checksum_create(struct pack_head_tag_s *head)
{
    (void)head;
    return NULL;
}
void pack_checksum_delete(pack_checksum_s * thi)
{
    (void)thi;
}
int pack_checksum_update_data(pack_checksum_s *thi, uint32_t offset, const uint8_t *buff, uint16_t len,  pack_hardware_s *hardware)
{
    (void)thi;
    (void)offset;
    (void)buff;
    (void)len;
    (void)hardware;
    return PACK_ERR;
}
int pack_checksum_check(pack_checksum_s *thi, const uint8_t *expected_value, uint16_t len)
{
    (void)thi;
    (void)expected_value;
    (void)len;
    return PACK_ERR;
}

#endif

