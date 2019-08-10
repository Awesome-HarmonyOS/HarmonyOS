/*----------------------------------------------------------------------------
 * Copyright (c) <2013-2018>, <Huawei Technologies Co., Ltd>
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

/* Define to prevent recursive inclusion ------------------------------------*/
#ifndef __LOS_FATFS_H__
#define __LOS_FATFS_H__


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

/* Includes -----------------------------------------------------------------*/
#include "ff.h"
#include "diskio.h"
#include <stdint.h>
/* Defines ------------------------------------------------------------------*/
#define DISK_STATE_INITIALIZED  1
/* Macros -------------------------------------------------------------------*/
/* Typedefs -----------------------------------------------------------------*/
struct diskio_drv
{
    DSTATUS (*initialize)   (BYTE);                             /*!< Initialize Disk Drive  */
    DSTATUS (*status)       (BYTE);                             /*!< Get Disk Status        */
    DRESULT (*read)         (BYTE, BYTE *, DWORD, UINT);        /*!< Read Sector(s)         */
    DRESULT (*write)        (BYTE, const BYTE *, DWORD, UINT);  /*!< Write Sector(s)        */
    DRESULT (*ioctl)        (BYTE, BYTE, void *);               /*!< I/O control operation  */
};

struct disk_dev
{
    uint8_t                 state;
    const struct diskio_drv *drv;
    uint8_t                 lun;
};

struct disk_mnt
{
    struct disk_dev     dev[FF_VOLUMES];
    volatile uint8_t    num;
};

/* Extern variables ---------------------------------------------------------*/
/* Functions API ------------------------------------------------------------*/

int fatfs_init (void);
int fatfs_mount(const char *path, struct diskio_drv *drv, uint8_t *drive);
int fatfs_unmount(const char *path, uint8_t drive);



#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __LOS_FATFS_H__ */
