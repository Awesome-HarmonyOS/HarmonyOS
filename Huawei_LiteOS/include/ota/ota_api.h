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

/**@defgroup agent AgentTiny
 * @defgroup agenttiny Agenttiny Definition
 * @ingroup agent
 */
#ifndef OTA_API_H
#define OTA_API_H

#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


typedef enum
{
    OTA_FULL_SOFTWARE,
    OTA_DIFF_SOFTWARE,
    OTA_UPDATE_INFO
} ota_flash_type_e;


typedef struct
{
    const char *rsa_N; /* RSA public key N, should valid all the time */
    const char *rsa_E; /* RSA public key E, should valid all the time */
}ota_key_s;

typedef enum
{
    OTA_DOWNLOAD_SUCCESS,
    OTA_DOWNLOAD_FAIL
} ota_download_result_e;


typedef struct
{
    int (*read_flash)(ota_flash_type_e type, void *buf, int32_t len, uint32_t location);
    int (*write_flash)(ota_flash_type_e type, const void *buf, int32_t len, uint32_t location);
    uint32_t flash_block_size;
    ota_key_s key;
}ota_opt_s;


#ifdef __cplusplus
}
#endif

#endif

