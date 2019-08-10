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

#include <stdio.h>
#include <string.h>

#if defined (__GNUC__) || defined (__CC_ARM)
#include "fs/sys/fcntl.h"
#include <los_printf.h>
#endif

#include "fs/los_vfs.h"
#include "fs/los_spiffs.h"

#include <hal_spi_flash.h>

#define PHYS_ERASE_SIZE     64 * 1024
#define LOG_BLOCK_SIZE      64 * 1024
#define LOG_PAGE_SIZE       256

static s32_t stm32f4xx_spiffs_read (struct spiffs_t *fs, u32_t addr, u32_t size, u8_t *buff)
{
    (void)hal_spi_flash_read ((void *) buff, size, addr);

    return SPIFFS_OK;
}

static s32_t stm32f4xx_spiffs_write (struct spiffs_t *fs, u32_t addr, u32_t size, u8_t *buff)
{
    (void)hal_spi_flash_write ((void *) buff, size, (uint32_t *)&addr);

    return SPIFFS_OK;
}

static s32_t stm32f4xx_spiffs_erase (struct spiffs_t *fs, u32_t addr, u32_t size)
{
    (void)hal_spi_flash_erase (addr, size);

    return SPIFFS_OK;
}

int stm32f4xx_spiffs_init (int need_erase)
{
    hal_spi_flash_config();
    if (need_erase)
    {
        (void)hal_spi_flash_erase(SPIFFS_PHYS_ADDR, SPIFFS_PHYS_SIZE);
    }

    (void)spiffs_init ();

    if (spiffs_mount ("/spiffs/", SPIFFS_PHYS_ADDR, SPIFFS_PHYS_SIZE, PHYS_ERASE_SIZE,
                      LOG_BLOCK_SIZE, LOG_PAGE_SIZE, stm32f4xx_spiffs_read,
                      stm32f4xx_spiffs_write, stm32f4xx_spiffs_erase) != LOS_OK)
    {
        PRINT_ERR ("failed to mount spiffs!\n");
        return LOS_NOK;
    }

    return LOS_OK;
}

