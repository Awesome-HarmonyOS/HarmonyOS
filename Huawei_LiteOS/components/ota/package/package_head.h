/*----------------------------------------------------------------------------
 * Copyright (c) <2018>, <Huawei Technologies Co., Ltd>
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

/**@defgroup atiny_adapter Agenttiny Adapter
 * @ingroup agent
 */

#ifndef PACKAGE_HEAD_H
#define PACKAGE_HEAD_H
#include "ota/package.h"
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "package_checksum.h"
#include "upgrade_flag.h"


#define PACK_MALLOC(size) pack_malloc(size)
#define PACK_FREE(ptr) pack_free(ptr)

#define PACK_LOG_ENABLE
#ifdef PACK_LOG_ENABLE
#define PACK_LOG(fmt, ...) \
do\
{\
    pack_params_s *__pack_params__ = pack_get_params();\
    if (__pack_params__->printf != NULL)\
    {\
        (void)__pack_params__->printf("[%s:%d]" fmt "\r\n",  __FUNCTION__, __LINE__,  ##__VA_ARGS__);\
    }\
}while(0)
#else
#define PACK_LOG(fmt, ...) ((void)0)
#endif


#define ASSERT_THIS(do_something) \
        if(NULL == thi)\
        {\
            PACK_LOG("this null pointer");\
            do_something;\
        }

#ifndef MIN
#define MIN(a, b) (((a) <= (b)) ? (a) : (b))
#endif

#ifndef array_size
#define array_size(a) (sizeof(a)/sizeof(*(a)))
#endif

enum
{
    PACK_OK,
    PACK_ERR
};


typedef struct
{
    pack_hardware_s *hardware;
    ota_key_s key;
}pack_device_info_s;

struct pack_head_tag_s;
typedef int (*head_update_check)(const uint8_t *head_buff , uint16_t len, void *param);

typedef struct pack_head_tag_s
{
    pack_hardware_s *hardware;
    head_update_check update_check;
    void *param;
    ota_key_s key;

    /* following data will be memset  when destroy */
    uint8_t *buff;
    uint16_t stored_len;
    uint16_t head_len;
    pack_checksum_s *checksum;
    uint8_t *checksum_pos;
    uint32_t checksum_len;
}pack_head_s;

#if defined(__cplusplus)
extern "C" {
#endif


void pack_head_init(pack_head_s *head);
void pack_head_destroy(pack_head_s *head);
int pack_head_parse(pack_head_s *head, uint32_t offset, const uint8_t *buff, uint16_t len,
                    uint16_t *used_len);
int pack_head_check(const pack_head_s *head, uint32_t len);
uint32_t pack_head_get_head_len(const pack_head_s *head);
const uint8_t* pack_head_get_head_info(const pack_head_s *head);

int pack_head_set_head_info(pack_head_s *head, pack_device_info_s *device_info);
pack_checksum_s *pack_head_get_checksum(pack_head_s *head);
ota_key_s  *pack_head_get_key(pack_head_s *head);


pack_params_s * pack_get_params(void);
void * pack_malloc(size_t size);
void pack_free(void *ptr);

#if defined(__cplusplus)
}
#endif

#endif //PACKAGE_HEAD_H


