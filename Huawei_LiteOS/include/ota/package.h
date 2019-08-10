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

/**@defgroup Agenttiny
 * @ingroup agent
 */

#ifndef PACKAGE_H
#define PACKAGE_H

//#ifdef WITH_SOTA
#include "ota_api.h"
#include <stdio.h>



//#endif

/* use sha256 rsa2048 for checksum */
#define PACK_SHA256_RSA2048 0
/* use sha256 for checksum */
#define PACK_SHA256 1
/* no checksum info */
#define PACK_NO_CHECKSUM 2

#define PACK_NO 0
#define PACK_YES 1

/* should define checksum first */
#ifndef PACK_CHECKSUM
#define PACK_CHECKSUM PACK_SHA256_RSA2048
#endif

/* PACK_COMBINE_TO_WRITE_LAST_BLOCK is set, the last writing software will read the not writing data from
flash to combine a entire block, it can save one block to buffer, cause the port write callback will only
write entire block size and need no buffer. but it is not suitable to write to fs system */
#ifndef PACK_COMBINE_TO_WRITE_LAST_BLOCK
#define PACK_COMBINE_TO_WRITE_LAST_BLOCK PACK_NO
#endif


/* package head len should not bigger than this */
#ifndef PACK_MAX_HEAD_LEN
#define PACK_MAX_HEAD_LEN (4 * 1024)
#endif


#if defined(__cplusplus)
extern "C" {
#endif

typedef struct
{
    ota_opt_s ota_opt;
    void* (*malloc)(size_t size);
    void (*free)(void *ptr);
    int (*printf)(const char *fmt, ...);
}pack_params_s;

typedef struct pack_storage_device_api_tag_s pack_storage_device_api_s;

typedef enum
{
    PACK_DOWNLOAD_OK,
    PACK_DOWNLOAD_FAIL
}pack_download_result_e;
struct pack_storage_device_api_tag_s
{
    int (*write_software)(pack_storage_device_api_s *thi, uint32_t offset, const uint8_t *buffer, uint32_t len);
    int (*write_software_end)(pack_storage_device_api_s *thi, pack_download_result_e result, uint32_t total_len);
    int (*active_software)(pack_storage_device_api_s *thi);
};


/**
 *@ingroup agenttiny
 *@brief get storage device.
 *
 *@par Description:
 *This API is used to get storage device.
 *@attention none.
 *
 *@param none.
 *
 *@retval #pack_storage_device_api_s *     storage device.
 *@par Dependency: none.
 *@see none
 */
pack_storage_device_api_s *pack_get_device(void);

/**
 *@ingroup agenttiny
 *@brief initiate storage device.
 *
 *@par Description:
 *This API is used to initiate storage device.
 *@attention none.
 *
 *@param ato_opt        [IN] Ota option.
 *
 *@retval #int          0 if succeed, or error.
 *@par Dependency: none.
 *@see none
 */
int pack_init_device(const pack_params_s *params);


#if defined(__cplusplus)
}
#endif

#endif //PACKAGE_H


