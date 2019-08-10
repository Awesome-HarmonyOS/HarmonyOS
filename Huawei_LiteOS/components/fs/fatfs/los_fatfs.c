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

/* Includes -----------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fs/sys/errno.h"
#include "fs/sys/fcntl.h"
#include "fs/sys/stat.h"
#include "fs/los_vfs.h"
#include "fs/los_fatfs.h"
#include "los_printf.h"


/* Defines ------------------------------------------------------------------*/
/* Typedefs -----------------------------------------------------------------*/

/* Macros -------------------------------------------------------------------*/
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef POINTER_ASSERT
#define POINTER_ASSERT(p) \
    if(p == NULL) \
    { \
        return -EINVAL; \
    }
#endif
/* Local variables ----------------------------------------------------------*/
struct disk_mnt disk;

/* Extern variables ---------------------------------------------------------*/
/* Global variables ---------------------------------------------------------*/
/* Private function prototypes ----------------------------------------------*/
/* Public functions ---------------------------------------------------------*/

static int ret_to_errno(FRESULT result)
{
    int err = 0;

    switch (result)
    {
    case FR_OK:
        return 0;

    case FR_NO_PATH:
        err = ENOTDIR;
        break;

    case FR_NO_FILE:
        err = ENOENT;
        break;

    case FR_NO_FILESYSTEM:
        err = ENODEV;
        break;

    case FR_TOO_MANY_OPEN_FILES:
        err = ENFILE;
        break;

    case FR_INVALID_NAME:
        err = ENAMETOOLONG;
        break;

    case FR_INVALID_PARAMETER:
    case FR_INVALID_OBJECT:
    case FR_INT_ERR:
        err = EINVAL;
        break;

    case FR_INVALID_DRIVE:
    case FR_NOT_ENABLED:
        err = ENXIO;
        break;

    case FR_EXIST:
        err = EEXIST;
        break;

    case FR_DISK_ERR:
    case FR_NOT_READY:
        err = EIO;
        break;

    case FR_WRITE_PROTECTED:
        err = EROFS;
        break;

    case FR_LOCKED:
    	err = EBUSY;
    	break;

    case FR_DENIED:
        err = EISDIR;
        break;

    case FR_MKFS_ABORTED:
        err = EBUSY;
        break;

    case FR_NOT_ENOUGH_CORE:
        err = ENOMEM;
        break;

    case FR_TIMEOUT:
        err = ETIMEDOUT;
        break;

    default:
        err = EIO;
        break;
    }

    VFS_ERRNO_SET (err);
    return -err;
}

/**
  * @brief  Links a compatible diskio driver/lun id and increments the number of active
  *         linked drivers.
  * @note   The number of linked drivers (volumes) is up to 10 due to FatFs limits.
  * @param  drv: pointer to the disk IO Driver structure
  * @param  lun : only used for USB Key Disk to add multi-lun management
            else the parameter must be equal to 0
  * @retval Returns -1 in case of failure, otherwise return the drive (0 to volumes).
  */
static int fatfs_link_driver(const struct diskio_drv *drv, uint8_t lun)
{
    int ret = -1;
    int i;

    if(disk.num >= FF_VOLUMES)
        return ret;

    for(i = 0; i < FF_VOLUMES; i++)
    {
        if(disk.dev[i].drv != 0)
            continue;

        disk.dev[disk.num].state = 0;
        disk.dev[disk.num].drv = drv;
        disk.dev[disk.num].lun = lun;
        disk.num++;
        return i;
    }
    return ret;
}

/**
  * @brief  Unlinks a diskio driver and decrements the number of active linked
  *         drivers.
  * @param  drive: the disk drive (0 to volumes)
  * @param  lun : not used
  * @retval Returns -1 in case of failure, otherwise return the drive (0 to volumes).
  */
static int fatfs_unlink_driver(uint8_t drive, uint8_t lun)
{
    int ret = -1;

    if(disk.num >= 1 && drive < FF_VOLUMES)
    {
        if(disk.dev[drive].drv != 0)
        {
            disk.dev[drive].state= 0;
            disk.dev[drive].drv = 0;
            disk.dev[drive].lun = 0;
            disk.num--;
            return drive;
        }
    }

    return ret;
}

int fatfs_register(const struct diskio_drv *drv)
{
    return fatfs_link_driver(drv, 0);
}

int fatfs_unregister(uint8_t drive)
{
    return fatfs_unlink_driver(drive, 0);
}

static int fatfs_flags_get (int oflags)
{
    int flags = 0;

    switch (oflags & O_ACCMODE)
    {
    case O_RDONLY:
        flags |= FA_READ;
        break;
    case O_WRONLY:
        flags |= FA_WRITE;
        break;
    case O_RDWR:
        flags |= FA_READ | FA_WRITE;
        break;
    default:
        break;
    }

    if (oflags & O_CREAT)
    {
        flags |= FA_OPEN_ALWAYS;
    }

    if ((oflags & O_CREAT) && (oflags & O_EXCL))
    {
        flags |= FA_CREATE_NEW;
    }

    if (oflags & O_TRUNC)
    {
        flags |= FA_CREATE_ALWAYS;
    }

    if (oflags & O_APPEND)
    {
        flags |= FA_READ | FA_WRITE | FA_OPEN_APPEND;
    }

    return flags;
}

static int fatfs_op_open (struct file *file, const char *path_in_mp, int flags)
{
    FRESULT res;
    FIL     *fp;
    FILINFO info = {0};

    fp = (FIL *) malloc (sizeof(FIL));
    if (fp == NULL)
    {
        PRINT_ERR ("fail to malloc memory in FATFS, <malloc.c> is needed,"
                   "make sure it is added\n");
        return -EINVAL;
    }

    if (!(flags & O_CREAT) && (flags & O_TRUNC))
    {
        res = f_stat(path_in_mp, &info);
        if(res != FR_OK)
        {
            free(fp);
            return res;
        }
    }

    res = f_open (fp, path_in_mp, fatfs_flags_get (flags));
    if(res == FR_OK)
    {
        file->f_data = (void *) fp;
    }
    else
    {
        free(fp);
    }
    if (FR_LOCKED == res)
    {
        int err = 0;
        VFS_ERRNO_SET (EACCES);
        err = EACCES;
        return -err;
    }
    else
    {
        return ret_to_errno(res);
    }
}

static int fatfs_op_close (struct file *file)
{
    FRESULT res;
    FIL     *fp = (FIL *)file->f_data;

    POINTER_ASSERT(fp);

    res = f_close(fp);
    if(res == FR_OK)
    {
        free(fp);
        file->f_data = NULL;
    }

    return ret_to_errno(res);
}

static ssize_t fatfs_op_read (struct file *file, char *buff, size_t bytes)
{
    ssize_t size = 0;
    FRESULT res;
    FIL     *fp = (FIL *)file->f_data;

    if (buff == NULL || bytes == 0)
        return -EINVAL;

    POINTER_ASSERT(fp);
    res = f_read (fp, buff, bytes, (UINT *)&size);
    if(res != FR_OK)
    {
        PRINT_ERR ("failed to read, res=%d\n", res);
        return ret_to_errno(res);
    }
    return size;
}

static ssize_t fatfs_op_write (struct file *file, const char *buff, size_t bytes)
{
    ssize_t  size = 0;
    FRESULT  res;
    FIL     *fp = (FIL *)file->f_data;

    if (buff == NULL || bytes == 0)
        return -EINVAL;

    POINTER_ASSERT(fp);
    res = f_write (fp, buff, bytes, (UINT *)&size);
    if(res != FR_OK || size == 0)
    {
        PRINT_ERR ("failed to write, res=%d\n", res);
        return ret_to_errno(res);
    }
    return size;
}

static off_t fatfs_op_lseek (struct file *file, off_t off, int whence)
{
    FIL *fp = (FIL *)file->f_data;

    POINTER_ASSERT(fp);
    switch (whence)
    {
    case 0: // SEEK_SET
        break;
    case 1: // SEEK_CUR
        off += f_tell(fp);
        break;
    case 2: // SEEK_END
        off += f_size(fp);
        break;
    default:
    	ret_to_errno(FR_INVALID_PARAMETER);
        return -1;
    }
    
    if (off < 0)
    {
        return ret_to_errno(FR_INVALID_PARAMETER);
    }
    
    FRESULT res = f_lseek(fp, off);
    if (res == FR_OK)
    {

    	return off;
    }
    else
        return ret_to_errno(res);
}

int fatfs_op_stat (struct mount_point *mp, const char *path_in_mp, struct stat *stat)
{
    FRESULT res;
    FILINFO info = {0};

    memset(stat, 0, sizeof(*stat));
    res = f_stat(path_in_mp, &info);
    if (res == FR_OK)
    {
        stat->st_size = info.fsize;
        if (info.fattrib & AM_DIR)
        {
            stat->st_mode = S_IFDIR;
        }
        else
        {
            stat->st_mode = S_IFREG;
        }
    }

    return ret_to_errno(res);
}

static int fatfs_op_unlink (struct mount_point *mp, const char *path_in_mp)
{
    FRESULT res = f_unlink(path_in_mp);
    if (FR_NO_PATH == res)
    {
        int err = 0;
        VFS_ERRNO_SET (ENOENT);
        err = ENOENT;
        return -err;
    }
    else
    {
    	return ret_to_errno(res);
    }

}

static int fatfs_op_rename (struct mount_point *mp, const char *path_in_mp_old,
                             const char *path_in_mp_new)
{
    FRESULT res = f_rename(path_in_mp_old, path_in_mp_new);
    return ret_to_errno(res);
}

static int fatfs_op_sync (struct file *file)
{
    FIL *fp = (FIL *)file->f_data;
    FRESULT res;

    POINTER_ASSERT(fp);

    res = f_sync(fp);
    return ret_to_errno(res);
}

static int fatfs_op_opendir (struct dir *dir, const char *path)
{
    FRESULT  res;
    DIR     *dp;

    dp = (DIR *) malloc (sizeof (DIR));

    if (dp == NULL)
    {
        PRINT_ERR ("fail to malloc memory in SPIFFS, <malloc.c> is needed,"
                   "make sure it is added\n");
        return -ENOMEM;
    }

    res = f_opendir(dp, path);
    if(res != FR_OK)
    {
        free(dp);
        return ret_to_errno(res);
    }

    dir->d_data   = dp;
    dir->d_offset = 0;

    return FR_OK;
}

static int fatfs_op_readdir (struct dir *dir, struct dirent *dent)
{
    FRESULT  res;
    DIR     *dp = (DIR *) dir->d_data;
    FILINFO  e;
    int     len;

    POINTER_ASSERT(dp);

    res = f_readdir(dp, &e);
    if (res != FR_OK)
    {
        return ret_to_errno(res);
    }

    len = MIN(sizeof(e.fname), LOS_MAX_DIR_NAME_LEN+1) - 1;
    strncpy ((char *)dent->name, (const char *) e.fname, len);
    dent->name [len] = '\0';
    dent->size = e.fsize;

    if (e.fattrib == AM_DIR)
    {
        dent->type = VFS_TYPE_DIR;
    }
    else
    {
        dent->type = VFS_TYPE_FILE;
    }

    return FR_OK;
}

static int fatfs_op_closedir (struct dir *dir)
{
    FRESULT  res;
    DIR     *dp = (DIR *) dir->d_data;

    POINTER_ASSERT(dp);

    res = f_closedir (dp);
    if(res == FR_OK)
    {
        free (dp);
        dir->d_data = NULL;
    }

    return ret_to_errno(res);
}

static int fatfs_op_mkdir(struct mount_point *mp, const char *path)
{
    FRESULT res = f_mkdir(path);
    if (FR_NO_PATH == res)
    {
        int err = 0;
        VFS_ERRNO_SET (ENOENT);
        err = ENOENT;
        return -err;
    }
    else
    {
    	return ret_to_errno(res);
    }
}

static struct file_ops fatfs_ops =
{
    fatfs_op_open,
    fatfs_op_close,
    fatfs_op_read,
    fatfs_op_write,
    fatfs_op_lseek,
    fatfs_op_stat,
    fatfs_op_unlink,
    fatfs_op_rename,
    NULL,               /* ioctl not supported for now */
    fatfs_op_sync,
    fatfs_op_opendir,
    fatfs_op_readdir,
    fatfs_op_closedir,
    fatfs_op_mkdir
};

static struct file_system fatfs_fs =
{
    "fatfs",
    &fatfs_ops,
    NULL,
    0
};

int fatfs_init (void)
{
    static int fatfs_inited = FALSE;

    if (fatfs_inited)
    {
        return LOS_OK;
    }

    if (los_vfs_init () != LOS_OK)
    {
        return LOS_NOK;
    }

    if (los_fs_register (&fatfs_fs) != LOS_OK)
    {
        PRINT_ERR ("failed to register fs!\n");
        return LOS_NOK;
    }

    fatfs_inited = TRUE;

    PRINT_INFO ("register fatfs done!\n");

    return LOS_OK;
}

static FATFS *fatfs_ptr = NULL;

int fatfs_mount(const char *path, struct diskio_drv *drv, uint8_t *drive)
{
    int s_drive;
    char dpath[10] = {0};
    int ret = -1;
    BYTE *work_buff = NULL;
    FRESULT res;
    FATFS   *fs = NULL;

    s_drive = fatfs_register(drv);
    if(s_drive < 0)
    {
        PRINT_ERR("failed to register diskio!\n");
        return s_drive;
    }
    fs = (FATFS *) malloc (sizeof (FATFS));
    if (fs == NULL)
    {
        PRINT_ERR ("fail to malloc memory in FATFS, <malloc.c> is needed,"
                   "make sure it is added\n");
        goto err;
    }
    memset(fs, 0, sizeof(FATFS));
    sprintf(dpath, "%d:/", s_drive);
    res = f_mount(fs, (const TCHAR *)dpath, 1);
    if(res == FR_NO_FILESYSTEM)
    {
        work_buff = (BYTE *)malloc(FF_MAX_SS);
        if(work_buff == NULL)
            goto err_free;
        memset(work_buff, 0, FF_MAX_SS);
        res = f_mkfs((const TCHAR *)dpath, FM_ANY, 0, work_buff, FF_MAX_SS);
        if(res == FR_OK)
        {
            res = f_mount(NULL, (const TCHAR *)dpath, 1);
            res = f_mount(fs, (const TCHAR *)dpath, 1);
        }
        free(work_buff);
    }
    if(res != FR_OK)
    {
        PRINT_ERR("failed to mount fatfs, res=%d!\n", res);
        goto err_free;
    }

    ret = los_fs_mount ("fatfs", path, fs);

    if (ret == LOS_OK)
    {
        PRINT_INFO ("fatfs mount at %s done!\n", path);
        *drive = s_drive;
        fatfs_ptr = fs;
        return LOS_OK;
    }

    PRINT_ERR ("failed to mount!\n");

err_free:
    if(fs != NULL)
        free(fs);
err:
    fatfs_unregister(s_drive);
    return ret;
}

int fatfs_unmount(const char *path, uint8_t drive)
{
    char dpath[10] = {0};

    sprintf(dpath, "%d:/", drive);
    fatfs_unregister(drive);
    f_mount(NULL, (const TCHAR *)dpath, 1);
    los_fs_unmount(path);
    if (fatfs_ptr)
    {
        free(fatfs_ptr);
        fatfs_ptr = NULL;
    }

    return 0;
}

/* Private functions --------------------------------------------------------*/

