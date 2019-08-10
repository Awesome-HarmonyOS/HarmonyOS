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

#include "ota/package.h"
#include "package_device.h"
#if (PACK_CHECKSUM != PACK_NO_CHECKSUM)
#include "mbedtls/platform.h"
#endif

static inline pack_storage_device_s *pack_storage_get_storage_device(pack_storage_device_api_s *this)
{
    return (pack_storage_device_s *)this;
}

static int pack_storage_write_software_end(pack_storage_device_api_s *this, pack_download_result_e result, uint32_t total_len)
{
    pack_storage_device_s *device;
    int ret = PACK_OK;

    if(NULL == this)
    {
        PACK_LOG("null pointer");
        return PACK_ERR;
    }

    device = pack_storage_get_storage_device(this);

    do
    {

        if(pack_wr_write_end(&device->writer) != PACK_OK)
        {
            ret = PACK_ERR;
            break;
        }

        if(result != PACK_DOWNLOAD_OK)
        {
            break;
        }

        if(pack_head_check(&device->head, total_len) != PACK_OK)
        {
            ret = PACK_ERR;
            PACK_LOG("check err,len %u", total_len);
            break;
        }

    }
    while(0);

    device->total_len = total_len;

    //current download finish, clear the save data
    if(PACK_DOWNLOAD_OK == result)
    {
        pack_head_destroy(&device->head);
        pack_wr_destroy(&device->writer);
    }

    return ret;
}


static int pack_storage_write_software(pack_storage_device_api_s *this,
        uint32_t offset, const uint8_t *buffer, uint32_t len)
{
    pack_storage_device_s *device;
    uint16_t used_len = 0;
    int ret;

    if(NULL == this)
    {
        PACK_LOG("null pointer");
        return PACK_ERR;
    }
    if(0 == len || buffer == NULL)
    {
        PACK_LOG("write 0 len");
        return PACK_ERR;
    }


    device = pack_storage_get_storage_device(this);

    ret = pack_head_parse(&device->head, offset, buffer, (uint16_t)len, &used_len);
    if(ret != PACK_OK)
    {
        PACK_LOG("pack_head_parse fail %d", ret);
        return ret;
    }

    if(used_len == len)
    {
        return PACK_OK;
    }

    len -= used_len;
    offset += used_len;
    buffer += used_len;

    ret = pack_wr_write(&device->writer, offset - pack_head_get_head_len(&device->head), buffer, len);
    if(ret != PACK_OK)
    {
        PACK_LOG("pack_wr_write fail %d", ret);
        return ret;
    }

    if (pack_head_get_checksum(&device->head))
    {
        ret = pack_checksum_update_data(pack_head_get_checksum(&device->head), offset,
                                             buffer, len, &device->hardware);
        if(ret != PACK_OK)
        {
            PACK_LOG("pack_checksum_update_data fail %d", ret);
        }
    }

    return PACK_OK;

}

int pack_storage_active_software(pack_storage_device_api_s *thi)
{
    pack_storage_device_s *device = pack_storage_get_storage_device(thi);

    if(NULL == thi)
    {
        PACK_LOG("null err");
        return PACK_ERR;
    }

    return flag_set_info((OTA_FULL_SOFTWARE == device->type) ? UPGRADE_FULL : UPGRADE_DIFF,
                        device->total_len);
    }

static void pack_init_pack_device(pack_storage_device_s *device)
{
    if(device->init_flag)
    {
        return;
    }

    (void)memset(device, 0, sizeof(*device));
    pack_wr_init(&device->writer);
    pack_head_init(&device->head);

    device->interface.write_software = pack_storage_write_software;
    device->interface.write_software_end = pack_storage_write_software_end;
    device->interface.active_software = pack_storage_active_software;
    device->init_flag = true;
}


static pack_storage_device_s g_pack_storage_device;

pack_storage_device_api_s *pack_get_device(void)
{

    pack_storage_device_s *device  = &g_pack_storage_device;
    pack_init_pack_device(device);
    return &device->interface;
}


static int pack_read_software(struct pack_hardware_tag_s *thi, uint32_t offset, uint8_t *buffer, uint32_t len)
{
    pack_storage_device_s *device = (pack_storage_device_s *)pack_get_device();

    (void)thi;
    if (device->params.ota_opt.read_flash)
    {
        return device->params.ota_opt.read_flash(device->type, buffer, len, offset);
    }
    PACK_LOG("read_flash null");

    return PACK_ERR;
}

static int pack_write_software(struct pack_hardware_tag_s *thi, uint32_t offset, const uint8_t *buffer, uint32_t len)
{
    pack_storage_device_s *device = (pack_storage_device_s *)pack_get_device();

    (void)thi;
    if (device->params.ota_opt.write_flash)
    {
        return device->params.ota_opt.write_flash(device->type, buffer, len, offset);
    }
    PACK_LOG("write_flash null");

    return PACK_ERR;
}

static void pack_set_flash_type(struct pack_hardware_tag_s *thi, ota_flash_type_e type)
{
    pack_storage_device_s *device = (pack_storage_device_s *)pack_get_device();

    device->type = type;
    PACK_LOG("pack_set_flash_type type %d", type);
}

static uint32_t pack_get_block_size(struct pack_hardware_tag_s *thi)
{
    pack_storage_device_s *device = (pack_storage_device_s *)pack_get_device();
    return device->params.ota_opt.flash_block_size;
}

static void* local_calloc(size_t n, size_t size)
{
    void *p = pack_malloc(n * size);
    if (NULL != p)
    {
        memset(p, 0, n * size);
    }
    return p;
}

int pack_init_device(const pack_params_s *params)
{
    pack_storage_device_s *device = (pack_storage_device_s *)pack_get_device();
    pack_device_info_s device_info;

    if ((params == NULL) || (params->malloc == NULL)
        || (params->free == NULL))
    {
        PACK_LOG("params null");
        return PACK_ERR;
    }

    memcpy(&device->params, params, sizeof(device->params));

    device->hardware.read_software = pack_read_software;
    device->hardware.write_software = pack_write_software;
    device->hardware.set_flash_type = pack_set_flash_type;
    device->hardware.get_block_size = pack_get_block_size;

    device_info.hardware = &device->hardware;
    memcpy(&device_info.key, &device->params.ota_opt.key, sizeof(device_info.key));

    if(pack_head_set_head_info(&device->head,  &device_info) != PACK_OK)
    {
        return PACK_ERR;
    }

    pack_wr_set_device(&device->writer, device_info.hardware);

#if (PACK_CHECKSUM != PACK_NO_CHECKSUM)
    (void)mbedtls_platform_set_calloc_free(local_calloc, params->free);
#endif

    return PACK_OK;
}

pack_params_s * pack_get_params(void)
{
    pack_storage_device_s *device = (pack_storage_device_s *)pack_get_device();
    return &device->params;
}

void * pack_malloc(size_t size)
{
    pack_params_s *params = pack_get_params();

    return (params->malloc == NULL) ? NULL : params->malloc(size);
}

void pack_free(void *ptr)
{
    pack_params_s *params = pack_get_params();
    if(params->free != NULL)
    {
        params->free(ptr);
    }
}



