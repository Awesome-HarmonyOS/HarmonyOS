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

#include "package_writer.h"
#include "package_head.h"
#include "upgrade_flag.h"


void pack_wr_init(pack_writer_s *writer)
{
    (void)memset(writer, 0, sizeof(*writer));
}
void pack_wr_free_save_buffer(pack_writer_s *writer)
{
    if(writer->buffer)
    {
        PACK_FREE(writer->buffer);
        writer->buffer = NULL;
    }
    writer->buffer_stored_len = 0;
    writer->block_size = 0;
}

void pack_wr_destroy(pack_writer_s *writer)
{
    writer->offset_flag = false;
    pack_wr_free_save_buffer(writer);

}


void pack_wr_set_device(pack_writer_s *writer, pack_hardware_s *hardware)
{
    writer->hardware = hardware;
}

int pack_wr_check(pack_writer_s *writer)
{
    if(NULL == writer->hardware)
    {
        PACK_LOG("null poiter");
        return PACK_ERR;
    }
    return PACK_OK;
}

static int pack_write_data(pack_writer_s *writer, uint32_t offset, const uint8_t *buffer, uint32_t len)
{

    int ret = writer->hardware->write_software(writer->hardware,  offset, buffer, len);

    if (0 == offset)
    {
        (void)flag_enable_hwpatch(buffer, len);
    }
    return ret;
}
int pack_wr_write_stored_data(pack_writer_s *writer)
{
    int ret;

#if (PACK_COMBINE_TO_WRITE_LAST_BLOCK == PACK_YES)
    if (writer->buffer_stored_len < writer->block_size)
    {
        ret = writer->hardware->read_software(writer->hardware,  writer->offset,
                    writer->buffer + writer->buffer_stored_len, writer->block_size - writer->buffer_stored_len);
        if (ret != PACK_OK)
        {
            PACK_LOG("read_software fail offset %ld, size %ld ret %d", writer->offset,
                                writer->block_size - writer->buffer_stored_len, ret);
            return ret;
        }

        writer->buffer_stored_len = writer->block_size;
    }
#endif

    ret = pack_write_data(writer,  writer->offset, writer->buffer, writer->buffer_stored_len);
    if(ret != PACK_OK)
    {
        PACK_LOG("write_software err ret %d, offset %u, len %u", ret, writer->offset, writer->buffer_stored_len);
        return PACK_ERR;
    }

    writer->offset_flag = false;
    return PACK_OK;
}



static int pack_wr_begin_not_aligned(pack_writer_s *writer, uint32_t block_begin_offset, uint32_t block_size,
                                uint32_t offset, const uint8_t *buff, uint16_t len , uint16_t *out_len)
{
       uint32_t block_left_len = (block_begin_offset + block_size) - offset;
       uint32_t write_len = MIN(len, block_left_len);
       int ret;

        if (writer->offset_flag && writer->buffer)
        {
            memcpy(writer->buffer + writer->buffer_stored_len, buff, write_len);
            writer->buffer_stored_len += write_len;
        }
        else
        {
            if (writer->buffer == NULL)
            {
                writer->buffer = PACK_MALLOC(block_size);
                if (writer->buffer == NULL)
                {
                    PACK_LOG("malloc null %d", block_size);
                    return PACK_ERR;
                }
                writer->block_size = block_size;
            }

            writer->buffer_stored_len = (offset - block_begin_offset);
            ret = writer->hardware->read_software(writer->hardware, block_begin_offset, writer->buffer,
                    writer->buffer_stored_len);
            if (ret != PACK_OK)
            {
                PACK_LOG("read_software fail offset %ld, size %ld ret %d", block_begin_offset, writer->buffer_stored_len, ret);
                return ret;
            }
            memcpy(writer->buffer + writer->buffer_stored_len, buff, write_len);
            writer->offset_flag = true;
            writer->offset = block_begin_offset;
            writer->buffer_stored_len += write_len;
        }


        if (writer->buffer_stored_len >= block_size)
        {
            ret = pack_write_data(writer, writer->offset, writer->buffer, block_size);
            if (ret != PACK_OK)
            {
                PACK_LOG("pack_write_data fail,offset %ld, size %ld, ret %ld",
                        writer->offset, block_size, ret);
                return ret;
            }
            writer->offset_flag = false;
        }

        *out_len = (uint16_t)write_len;

        return PACK_OK;
}

static int pack_wr_entire_blocks(pack_writer_s *writer, uint32_t block_begin_offset,
                                    uint32_t block_size, const uint8_t *buff, uint16_t len, uint16_t *out_len)
{
    int ret;

    *out_len = 0;

    for (; len >= block_size; len -= block_size, block_begin_offset += block_size,
                                buff += block_size, *out_len += block_size)
    {
        ret = pack_write_data(writer, block_begin_offset, buff, block_size);
        if (ret != PACK_OK)
        {
            PACK_LOG("pack_write_data fail,offset %ld, size %ld, ret %ld",
                    writer->offset, block_size, ret);
            return ret;
        }
    }

    return PACK_OK;
}


static int pack_wr_end_not_aligned_block(pack_writer_s *writer, uint32_t block_begin_offset,
                                            uint32_t block_size, const uint8_t *buff, uint16_t len)
{
    if (writer->buffer == NULL)
    {
        writer->buffer = PACK_MALLOC(block_size);
        if (writer->buffer == NULL)
        {
            PACK_LOG("malloc null %d", block_size);
            return PACK_ERR;
        }
        writer->block_size = block_size;
    }

    memcpy(writer->buffer, buff, len);
    writer->buffer_stored_len = len;
    writer->offset_flag = true;
    writer->offset = block_begin_offset;
    return PACK_OK;
}

int pack_wr_write(pack_writer_s *writer, uint32_t offset, const uint8_t *buff, uint16_t len)
{
    uint16_t write_len = 0;
    int ret;
    uint32_t block_size;

    if(pack_wr_check(writer) != PACK_OK)
    {
        return PACK_ERR;
    }


    if(writer->offset_flag && (writer->offset + writer->buffer_stored_len != offset)
        && (writer->buffer_stored_len > 0))
    {

        ret = pack_wr_write_stored_data(writer);
        if (ret != PACK_OK)
        {
            return ret;
        }
    }

    block_size = writer->hardware->get_block_size(writer->hardware);
    if (block_size == 0)
    {
        PACK_LOG("err block_size is 0");
        return PACK_ERR;
    }

    if (offset % block_size)
    {
        uint32_t block_begin;

        block_begin = ((offset / block_size) * block_size);
        ret = pack_wr_begin_not_aligned(writer, block_begin, block_size, offset, buff, len, &write_len);
        if (ret != PACK_OK)
        {
            return ret;
        }

        buff += write_len;
        len -= write_len;
        offset += write_len;
    }

    if (len <= 0)
    {
        return PACK_OK;
    }

    ret = pack_wr_entire_blocks(writer, offset, block_size, buff, len, &write_len);
    if (ret != PACK_OK)
    {
        return ret;
    }

    buff += write_len;
    len -= write_len;
    offset += write_len;

    if (len <= 0)
    {
        return PACK_OK;
    }

    return pack_wr_end_not_aligned_block(writer, offset, block_size, buff, len);

}

int pack_wr_write_end(pack_writer_s *writer)
{
    if(0 == writer->buffer_stored_len)
    {
        return PACK_OK;
    }

    return pack_wr_write_stored_data(writer);
}




