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

/* Includes -----------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#if defined (__GNUC__) || defined (__CC_ARM)
#include "fs/sys/fcntl.h"
#include <los_printf.h>
#endif

#include "fs/los_vfs.h"
#include "fs/los_fatfs.h"

#include <hal_spi_flash.h>
/* Defines ------------------------------------------------------------------*/
#define SPI_FLASH_ID            0xEF4018

#define SPI_FLASH_SECTOR_SIZE   (4 * 1024)
#define SPI_FLASH_PAGE_SIZE     256
/* Typedefs -----------------------------------------------------------------*/
/* Macros -------------------------------------------------------------------*/
/* Local variables ----------------------------------------------------------*/
/* Extern variables ---------------------------------------------------------*/
/* Global variables ---------------------------------------------------------*/
/* Private function prototypes ----------------------------------------------*/
/* Public functions ---------------------------------------------------------*/
static DSTATUS stm32f4xx_fatfs_status(BYTE lun)
{
    DSTATUS status = STA_NOINIT;

    if(SPI_FLASH_ID == hal_spi_flash_get_id())
    {
        status &= ~STA_NOINIT;
    }
    return status;
}

static DSTATUS stm32f4xx_fatfs_initialize(BYTE lun)
{
    DSTATUS status = STA_NOINIT;

    hal_spi_flash_config();
    hal_spi_flash_wake_up();
    status = stm32f4xx_fatfs_status(lun);
    return status;
}

static DRESULT stm32f4xx_fatfs_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
    int ret;
    ret = hal_spi_flash_read(buff, count * SPI_FLASH_SECTOR_SIZE,
            FF_PHYS_ADDR + sector * SPI_FLASH_SECTOR_SIZE);
    if(ret != 0)
        return RES_ERROR;
    return RES_OK;
}

static DRESULT stm32f4xx_fatfs_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
    int ret;
    ret = hal_spi_flash_erase_write(buff, count * SPI_FLASH_SECTOR_SIZE,
            FF_PHYS_ADDR + sector * SPI_FLASH_SECTOR_SIZE);
    if(ret != 0)
        return RES_ERROR;
    return RES_OK;
}

static DRESULT stm32f4xx_fatfs_ioctl(BYTE lun, BYTE cmd, void *buff)
{
    DRESULT res = RES_PARERR;

    switch (cmd)
    {
    case GET_SECTOR_COUNT:
        *(DWORD *)buff = FF_PHYS_SIZE / SPI_FLASH_SECTOR_SIZE;
        break;
    case GET_SECTOR_SIZE:
        *(WORD *)buff = SPI_FLASH_SECTOR_SIZE;
        break;
    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 1;
        break;
    }
    res = RES_OK;
    return res;
}

static struct diskio_drv spi_drv =
{
    stm32f4xx_fatfs_initialize,
    stm32f4xx_fatfs_status,
    stm32f4xx_fatfs_read,
    stm32f4xx_fatfs_write,
    stm32f4xx_fatfs_ioctl
};

int stm32f4xx_fatfs_init(int need_erase)
{
    int8_t drive = -1;

    if (need_erase)
    {
        (void)hal_spi_flash_config();
        (void)hal_spi_flash_erase(FF_PHYS_ADDR, FF_PHYS_SIZE);
    }

    (void)fatfs_init();

    if(fatfs_mount("/fatfs/", &spi_drv, (uint8_t *)&drive) < 0)
    {
        PRINT_ERR ("failed to mount fatfs!\n");
    }

    return drive;
}

DWORD get_fattime (void)
{
    return 0;
}

/* Private functions --------------------------------------------------------*/

