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

#ifndef UPGRADE_FLAG_H
#define UPGRADE_FLAG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    UPGRADE_FULL = 0,
    UPGRADE_DIFF,
    UPGRADE_NONE,
} upgrade_type_e;

typedef enum
{
    OTA_IDLE = 0,
    OTA_NEED_UPGRADE,
    OTA_SUCCEED,
    OTA_FAILED,
} upgrade_state_e;

int flag_upgrade_init(void);

int flag_set_info(upgrade_type_e upgrade_type, uint32_t image_size);

void flag_get_info(upgrade_type_e *upgrade_type, uint32_t *image_size,
                   uint32_t *old_image_size, upgrade_state_e *upgrade_state);

int flag_upgrade_set_result(upgrade_state_e state, uint32_t image_size);

int flag_upgrade_get_result(upgrade_state_e *state);

int flag_set_recover_verify(uint32_t recover_verify, uint32_t verify_length);

void flag_get_recover_verify(uint32_t *recover_verify, uint32_t *verify_length);

int flag_enable_hwpatch(const uint8_t *patch_head, int32_t patch_len);

#ifdef __cplusplus
}
#endif

#endif /* UPGRADE_FLAG_H */