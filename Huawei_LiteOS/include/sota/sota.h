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
#ifndef __SOTA_H__
#define __SOTA_H__

#include<stdint.h>
#include"ota/ota_api.h"
#include<stddef.h>

#define SOTA_DEBUG 1

typedef enum
{
    APPLICATION = 0,
    BOOTLOADER = 1,
} sota_run_mode_e;

typedef enum
{
    SOTA_OK = 0,
    SOTA_DOWNLOADING = 1,
    SOTA_UPDATING    = 2,
    SOTA_UPDATED     = 3,
    SOTA_FAILED             = 101,
    SOTA_EXIT               = 102,
    SOTA_INVALID_PACKET     = 103,
    SOTA_UNEXPECT_PACKET    = 104,
    SOTA_WRITE_FLASH_FAILED = 105
} sota_ret_e;

typedef struct
{
    int (*get_ver)(char* buf, uint32_t len);
    int (*sota_send)(const char* buf, int len);
    void* (*sota_malloc)(size_t size);
    void (*sota_free)(void *ptr);
    int  (*sota_printf)(const char *fmt, ...);
    sota_run_mode_e  firmware_download_stage;
    sota_run_mode_e  current_run_stage;
    ota_opt_s ota_info;
} sota_arg_s;


int32_t sota_init(const sota_arg_s* sota_arg);
int32_t sota_parse(const int8_t *in_buf, int32_t in_len, int8_t * out_buf,  int32_t out_len);
int32_t sota_process(void *arg, const int8_t *buf, int32_t buf_len);
void    sota_timeout_handler(void);
#endif
