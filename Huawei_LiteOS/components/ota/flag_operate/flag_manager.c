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

#include <stddef.h>
#include <string.h>
#include "flag_manager.h"

static flag_op_s g_flag_op;

#define FLASH_UNIT_SIZE (256)
#define FLASH_FLAG_SIZE (512)

int flag_init(flag_op_s *flag)
{
    if (NULL == flag
        || NULL == flag->func_flag_read
        || NULL == flag->func_flag_write)
    return -1;

    g_flag_op.func_flag_read = flag->func_flag_read;
    g_flag_op.func_flag_write = flag->func_flag_write;

    return 0;
}

int flag_read(flag_type_e flag_type, void *buf, int32_t len)
{
    uint8_t flag_buf[FLASH_FLAG_SIZE];

    if (NULL == buf
        || len < 0
        || len > FLASH_UNIT_SIZE
        || g_flag_op.func_flag_read(flag_buf, FLASH_FLAG_SIZE) != 0)
        return -1;

    switch (flag_type)
    {
    case FLAG_BOOTLOADER:
        memcpy(buf, flag_buf, len);
        break;
    case FLAG_APP:
        memcpy(buf, flag_buf + FLASH_UNIT_SIZE, len);
        break;
    default:
        break;
    }

    return 0;
}

int flag_write(flag_type_e flag_type, const void *buf, int32_t len)
{
    uint8_t flag_buf[FLASH_FLAG_SIZE];

    if (NULL == buf
        || len < 0
        || len > FLASH_UNIT_SIZE
        || g_flag_op.func_flag_read(flag_buf, FLASH_FLAG_SIZE) != 0)
        return -1;

    switch (flag_type)
    {
    case FLAG_BOOTLOADER:
        memcpy(flag_buf, buf, len);
        break;
    case FLAG_APP:
        memcpy(flag_buf + FLASH_UNIT_SIZE, buf, len);
        break;
    default:
        break;
    }

    return g_flag_op.func_flag_write(flag_buf, FLASH_FLAG_SIZE);
}