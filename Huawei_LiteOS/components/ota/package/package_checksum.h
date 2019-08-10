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

#ifndef PACKAGE_CHECKSUM_H
#define PACKAGE_CHECKSUM_H

#include "ota/package.h"


typedef struct pack_hardware_tag_s
{
    int (*read_software)(struct pack_hardware_tag_s *thi, uint32_t offset, uint8_t *buffer, uint32_t len);
    int (*write_software)(struct pack_hardware_tag_s *thi, uint32_t offset, const uint8_t *buffer, uint32_t len);
    void (*set_flash_type)(struct pack_hardware_tag_s *thi, ota_flash_type_e type);
    uint32_t (*get_block_size)(struct pack_hardware_tag_s *thi);
    uint32_t (*get_max_size)(struct pack_hardware_tag_s *thi);
}pack_hardware_s;

typedef struct pack_checksum_tag_s pack_checksum_s;

struct pack_head_tag_s;

typedef struct pack_checksum_alg_tag_s
{
    void (*reset)(struct pack_checksum_alg_tag_s *thi);
    int (*update)(struct pack_checksum_alg_tag_s *thi, const uint8_t *buff, uint16_t len);
    int (*check)(struct pack_checksum_alg_tag_s *thi, const uint8_t  *checksum, uint16_t checksum_len);
    void (*destroy)(struct pack_checksum_alg_tag_s *thi);
}pack_checksum_alg_s;




#if defined(__cplusplus)
extern "C" {
#endif


pack_checksum_s * pack_checksum_create(struct pack_head_tag_s *head);
void pack_checksum_delete(pack_checksum_s * thi);
int pack_checksum_update_data(pack_checksum_s *thi, uint32_t offset, const uint8_t *buff, uint16_t len,  pack_hardware_s *hardware);
int pack_checksum_check(pack_checksum_s *thi, const uint8_t *expected_value, uint16_t len);


#if defined(__cplusplus)
}
#endif

#endif //PACKAGE_CHECKSUM_H


