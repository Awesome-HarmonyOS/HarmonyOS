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

#include "package_head.h"
#include "package_writer.h"
#include "stddef.h"

#define MAKE_DWORD(a, b, c, d) ((((uint32_t)(a)) << 24) | (((uint32_t)(b)) << 16) | (((uint32_t)(c)) <<8) | ((uint32_t)(d)))
#define MAKE_WORD(a, b)  ((((uint32_t)(a)) <<8) | ((uint32_t)(b)))

#define GET_DWORD(buf, pos) MAKE_DWORD(buf[pos], buf[pos + 1], buf[pos + 2], buf[pos + 3])
#define GET_WORD(buf, pos) MAKE_WORD(buf[pos], buf[pos + 1])


#define PACK_HEADER_MIN_LEN 12
#define PACK_TLV_START_POS PACK_HEADER_MIN_LEN

#define PACK_VERSION_POS 0
#define PACK_HEADER_HEAD_LEN_POS 4
#define PACK_HEADER_TOTAL_LEN_POS 8

#define PACK_TLV_T_LEN 2
#define PACK_TLV_L_LEN 2

#define PACK_TLV_T_SHA256 1
#define PACK_TLV_T_SHA256_RSA2048 3
#define PACK_INVALID_TLV_T 0xffff
#define PACK_TLV_T_BIN_TYPE 4



#define VERSION_NO 0




void pack_head_init(pack_head_s *head)
{
    (void)memset(head, 0 , sizeof(*head));
}


void pack_head_destroy(pack_head_s *head)
{
    if(head->buff)
    {
        PACK_FREE(head->buff);
        head->buff = NULL;
    }

    if(head->checksum_pos)
    {
        PACK_FREE(head->checksum_pos);
        head->checksum_pos = NULL;
    }

    if(head->checksum)
    {
        pack_checksum_delete(head->checksum);
        head->checksum = NULL;
    }

    memset(&(head->buff), 0, sizeof(*head) - (((uint8_t *) & (head->buff)) - ((uint8_t *)head)));
}

int pack_head_parse_head_len(pack_head_s *head, uint32_t offset, const uint8_t *buff,
                                  uint16_t len, uint16_t *used_len)
{
    *used_len = 0;
    if(offset < PACK_HEADER_MIN_LEN)
    {
        uint32_t copy_len;
        uint32_t head_len;
        uint32_t total_len;
        uint32_t version;
        if(offset > head->stored_len)
        {
            PACK_LOG("head not continuous, len %u offset %u", head->stored_len, offset);
            return PACK_ERR;
        }

        if(NULL == head->buff)
        {
            head->buff = PACK_MALLOC(PACK_HEADER_MIN_LEN);
            if(NULL == head->buff)
            {
                PACK_LOG("PACK_MALLOC fail");
                return PACK_ERR;
            }
            head->head_len = PACK_HEADER_MIN_LEN;
        }
        copy_len = MIN(PACK_HEADER_MIN_LEN - offset, len);
        memcpy(head->buff + offset, buff, copy_len);
        head->stored_len = offset + copy_len;
        *used_len = copy_len;
        if(head->stored_len < PACK_HEADER_MIN_LEN)
        {
            return PACK_OK;
        }

        version = GET_DWORD(head->buff, PACK_VERSION_POS);
        if(version != VERSION_NO)
        {
            PACK_LOG("invalid version %d", version);
            head->stored_len = 0;
            return PACK_ERR;
        }

        head_len = GET_DWORD(head->buff, PACK_HEADER_HEAD_LEN_POS);
        total_len = GET_DWORD(head->buff, PACK_HEADER_TOTAL_LEN_POS);
        if(head_len < PACK_HEADER_MIN_LEN || (head_len >= total_len)
            || (head_len > PACK_MAX_HEAD_LEN))
        {
            PACK_LOG("invalid head len %d, total len %d", head_len, total_len);
            head->stored_len = 0;
            return PACK_ERR;
        }

        if(head_len > head->head_len)
        {
            uint8_t *new_buff = PACK_MALLOC(head_len);
            if(NULL == new_buff)
            {
                PACK_LOG("PACK_MALLOC fail");
                return PACK_ERR;
            }
            memcpy(new_buff, head->buff, head->stored_len);
            PACK_FREE(head->buff);
            head->buff = new_buff;
            head->head_len = head_len;
        }

    }
    return PACK_OK;
}


static uint32_t pack_head_get_checksum_attribute(void)
{
#if (PACK_CHECKSUM == PACK_SHA256_RSA2048)
    return PACK_TLV_T_SHA256_RSA2048;
#elif (PACK_CHECKSUM == PACK_SHA256)
    return PACK_TLV_T_SHA256;
#else
    return PACK_INVALID_TLV_T;
#endif
}

static int pack_head_handle_checksum_tlv(pack_head_s *head, uint8_t *value, uint32_t len)
{
    if(head->checksum_pos)
    {
        PACK_FREE(head->checksum_pos);
        head->checksum_pos = NULL;
        head->checksum_len = 0;
    }

    if(len > 0)
    {

        head->checksum_pos = PACK_MALLOC(len);
        if(NULL == head->checksum_pos)
        {
            PACK_LOG("PACK_MALLOC %d fail", len);
            return PACK_ERR;
        }
        memcpy(head->checksum_pos, value, len);
    }

    head->checksum_len = len;
    memset(value, 0, len);

    return PACK_OK;
}

static int pack_head_handle_bin_type_tlv(pack_head_s *head, uint8_t *value, uint32_t len)
{
    if (head->hardware->set_flash_type)
    {
        uint32_t flash_type =  GET_DWORD(value, 0);
        if (flash_type >= OTA_UPDATE_INFO)
        {
            PACK_LOG("flash_type %d invalid", flash_type);
            return PACK_ERR;
        }
        head->hardware->set_flash_type(head->hardware, (ota_flash_type_e)flash_type);
    }

    return PACK_OK;
}


static int pack_head_parse_tlvs(pack_head_s *head, uint8_t *buff, uint32_t len)
{
    uint32_t attribute;
    uint32_t tlv_len;
    uint8_t *cur = buff + PACK_TLV_START_POS;
    uint32_t left_len = len - PACK_TLV_START_POS;

    int (*tlv_handles[2])(pack_head_s *head, uint8_t *value, uint32_t len) =
                                {pack_head_handle_checksum_tlv, pack_head_handle_bin_type_tlv};
    uint32_t attributes[array_size(tlv_handles)] = {PACK_INVALID_TLV_T, PACK_TLV_T_BIN_TYPE};

    attributes[0] = pack_head_get_checksum_attribute();
    while(left_len > 0)
    {
        uint32_t i;

        attribute = GET_WORD(cur, 0);
        cur += PACK_TLV_T_LEN;
        tlv_len = GET_WORD(cur, 0);
        cur += PACK_TLV_L_LEN;
        if(left_len < (PACK_TLV_T_LEN  + PACK_TLV_L_LEN + tlv_len))
        {
            PACK_LOG("tvl err attribute %d, tlv_len %d", attribute, tlv_len);
            return PACK_ERR;
        }

        for (i = 0; i < array_size(tlv_handles); i++)
        {
            if (attributes[i] == attribute)
            {
                if(tlv_handles[i](head, cur, tlv_len) != PACK_OK)
                {
                    return PACK_ERR;
                }
                break;
            }
        }

        cur += tlv_len ;
        left_len -= (PACK_TLV_T_LEN  + PACK_TLV_L_LEN + tlv_len);
    }

    if(NULL == head->checksum_pos)
    {
        PACK_LOG("head empty checksum info");
#if (PACK_CHECKSUM != PACK_NO_CHECKSUM)
        return PACK_ERR;
#else
        return PACK_OK;
#endif
    }


    if(head->checksum)
    {
        pack_checksum_delete(head->checksum);
        head->checksum = NULL;
    }
    head->checksum = pack_checksum_create(head);
    if(head->checksum == NULL)
    {
        PACK_LOG("pack_checksum_create fail");
        return PACK_ERR;
    }
    return PACK_OK;

}


int pack_head_parse(pack_head_s *head, uint32_t offset, const uint8_t *buff,
                         uint16_t len, uint16_t *used_len)
{
    int ret;
    uint16_t tmp_len = 0;

    *used_len = 0;
    ret = pack_head_parse_head_len(head, offset, buff, len, &tmp_len);
    if(ret != PACK_OK)
    {
        return ret;
    }

    len -= tmp_len;
    if((0 == len) && (head->stored_len < head->head_len))
    {
        *used_len = tmp_len;
        return PACK_OK;
    }

    offset += tmp_len;
    buff += tmp_len;

    if((head->head_len < PACK_HEADER_MIN_LEN)
            || (NULL == head->buff)
            || ((head->stored_len < head->head_len) && (offset > head->stored_len)))
    {
        PACK_LOG("head not continuous, len %u offset %u", head->stored_len, offset);
        return PACK_ERR;
    }

    if(offset < head->head_len)
    {
        uint16_t copy_len;

        copy_len = MIN(len, head->head_len - offset);
        (void)memcpy(head->buff + offset, buff, copy_len);
        *used_len = copy_len + tmp_len;
        head->stored_len = offset + copy_len;
        if(head->stored_len >= head->head_len)
        {
            uint32_t save_len;
            save_len = GET_DWORD(head->buff, PACK_HEADER_TOTAL_LEN_POS);
            if(save_len <= head->head_len)
            {
                PACK_LOG("head len err, save len %d head len %u", save_len, head->head_len);
                return PACK_ERR;
            }

            if(head->hardware && head->hardware->get_max_size)
            {
                uint32_t max_len = head->hardware->get_max_size(head->hardware);
                if(max_len < save_len - head->head_len)
                {
                    PACK_LOG("size exceed, save len %u head len %u max_len %u", save_len, head->head_len, max_len);
                    return PACK_ERR;
                }
            }

            if(head->update_check)
            {
                if(head->update_check(head->buff, head->head_len, head->param) != PACK_OK)
                {
                    return PACK_ERR;
                }
            }

            return pack_head_parse_tlvs(head, head->buff, head->head_len);
        }
    }

    return PACK_OK;
}

static bool pack_head_is_done(const pack_head_s *head)
{
    return ((head->head_len >= PACK_HEADER_MIN_LEN) && (head->stored_len == head->head_len));
}

int pack_head_check(const pack_head_s *head, uint32_t len)
{
    uint32_t save_len;

    if(!pack_head_is_done(head))
    {
        PACK_LOG("head invalid get len, stored len %u, head len %u", head->stored_len, head->head_len);
        return PACK_ERR;
    }

    save_len = GET_DWORD(head->buff, PACK_HEADER_TOTAL_LEN_POS);
    if(len != save_len)
    {
        PACK_LOG("len err save len %u, rcv len %u", save_len, len);
        return PACK_ERR;
    }

    if((head->checksum_pos == NULL) || (head->checksum_len == 0))
    {
        PACK_LOG("fota no checksum exist");
        return PACK_OK;
    }

    if (head->checksum)
    {
        return pack_checksum_check(head->checksum, head->checksum_pos, head->checksum_len);
    }
    return PACK_OK;
}

uint32_t pack_head_get_head_len(const pack_head_s *head)
{
    return pack_head_is_done(head) ? (uint32_t)head->head_len : 0;
}

const uint8_t *pack_head_get_head_info(const pack_head_s *head)
{
    return pack_head_is_done(head) ? head->buff : 0;
}


int pack_head_set_head_info(pack_head_s *head, pack_device_info_s *device_info)
{
    head->hardware = device_info->hardware;
    head->update_check = NULL;
    head->param = NULL;
    (void)memcpy(&head->key, &device_info->key, sizeof(head->key));
    return PACK_OK;
}

pack_checksum_s *pack_head_get_checksum(pack_head_s *head)
{
    return head->checksum;
}

ota_key_s  *pack_head_get_key(pack_head_s *head)
{
    return &head->key;
}



