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

#ifndef RECOVER_IMAGE_H
#define RECOVER_IMAGE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int8_t recover_on_oldimage; // Whether recover on oldimage area or not.
    int32_t max_old_image_size; // Max size of oldimage area.
    int32_t max_new_image_size; // Max size of newimage area.
    int32_t max_patch_size;     // Max size of patch area.
    int32_t flash_erase_unit;   // Smallest unit of erase operation on flash.
    uint32_t old_image_addr;    // Head address of oldimage.
    uint32_t new_image_addr;    // Head address of newimage.
    uint32_t patch_addr;        // Head address of patch.
} recover_info_s;

typedef struct
{
    int (*func_printf)(const char *fmt, ...);
    void *(*func_malloc)(size_t size);
    void (*func_free)(void *ptr);
} recover_assist_s;

typedef enum
{
    FLASH_OLDBIN_READ = 0, // Function to read oldbin area.
    FLASH_NEWBIN_WRITE,    // Function to write newbin area.
    FLASH_PATCH,           // Function to read/write patch area.
    FLASH_UPDATE_INFO,     // Function to read/write update info area.
} flash_type_e;

typedef struct
{
    int (*func_flash_read)(flash_type_e flash_type, void *buf, int32_t len, uint32_t offset);
    int (*func_flash_write)(flash_type_e flash_type, const void *buf, int32_t len, uint32_t offset);
} recover_flash_s;

typedef enum
{
    RECOVER_UPGRADE_NONE = 0, // Normal startup
    RECOVER_UPGRADE_DIFF,     // Diff upgrade
    RECOVER_UPGRADE_FULL,     // Full upgrade
} recover_upgrade_type_e;

/**
 *@ingroup recover_image
 *@brief register info of recover module.
 *
 *@par Description:
 *This API is used to register info of recover module.
 *@attention none.
 *
 *@param info           [IN] Necessary information needed by recover module.
 *@param assist         [IN] Assist functions includes malloc, free and pringf.
 *@param flash          [IN] Read/Write functions of flash.
 *
 *@retval #int          error code @ref hwpatch_errno.h.
 *
 *@par Dependency: none.
 *@see none.
 */
int recover_init(recover_info_s *info, recover_assist_s *assist, recover_flash_s *flash);

/**
 *@ingroup recover_image
 *@brief do recover action.
 *
 *@par Description:
 *This API is used to do recover action.
 *@attention none.
 *
 *@param recover_upgrade_type [OUT] Upgrade type @ref recover_upgrade_type_e.
 *@param newbin_size          [OUT] Newbin size of this upgrade time.
 *@param oldbin_size          [OUT] Oldbin size of this upgrade time.
 *
 *@retval #int                error code @ref hwpatch_errno.h.
 *
 *@par Dependency: none.
 *@see none.
 */
int recover_image(recover_upgrade_type_e *recover_upgrade_type, uint32_t *newbin_size, uint32_t *oldbin_size);

/**
 *@ingroup recover_image
 *@brief set update result to be failed.
 *
 *@par Description:
 *This API is used to set update result to be failed.
 *@attention none.
 *
 *@retval #int                error code @ref hwpatch_errno.h.
 *
 *@par Dependency: none.
 *@see none.
 */
int recover_set_update_fail(void);

#ifdef __cplusplus
}
#endif

#endif /* RECOVER_IMAGE_H */