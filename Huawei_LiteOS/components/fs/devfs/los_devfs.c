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

#include <los_config.h>
#include <los_devfs.h>

#if (LOSCFG_ENABLE_DEVFS == YES)

static void *devfs_root = NULL;

UINT32 los_devfs_init (void)
{
    if (devfs_root != NULL)
    {
        return LOS_OK;
    }

    if (los_kifs_init () != LOS_OK)
    {
        return LOS_NOK;
    }

    devfs_root = los_kifs_mount ("/dev/");

    return (devfs_root == NULL) ? LOS_NOK : LOS_OK;
}

UINT32 los_devfs_create (const char *name, uint32_t flags,
                         struct devfs_ops *devops, void *arg)
{
    int ret;

    if (devfs_root == NULL)
    {
        return LOS_NOK;
    }

    ret = los_kifs_create (devfs_root, name, flags, &devops->kiops, arg);

    return ret == 0 ? LOS_OK : LOS_NOK;
}

UINT32 los_devfs_link (const char *path_in_mp, uint32_t flags,
                       void *buff, size_t size)
{
    int ret;

    if (devfs_root == NULL)
    {
        return LOS_NOK;
    }

    ret = los_kifs_link (devfs_root, path_in_mp, flags, buff, size);

    return ret == 0 ? LOS_OK : LOS_NOK;
}

#endif

