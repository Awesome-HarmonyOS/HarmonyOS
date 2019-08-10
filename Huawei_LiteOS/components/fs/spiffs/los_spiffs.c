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

/* NOTE: this is a demo for adapting the SPIFFS */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "fs/sys/errno.h"
#include "fs/sys/fcntl.h"
#include "fs/sys/stat.h"
#include "fs/los_vfs.h"
#include "los_printf.h"

#include "spiffs.h"
#include "spiffs_nucleus.h"


static int ret_to_errno(int ret)
{
    int err = 0;

    switch (ret)
    {
    case SPIFFS_OK:
        return 0;

    case SPIFFS_ERR_DATA_SPAN_MISMATCH:
    case SPIFFS_ERR_IS_INDEX:
    case SPIFFS_ERR_INDEX_REF_FREE:
    case SPIFFS_ERR_INDEX_REF_LU:
    case SPIFFS_ERR_INDEX_REF_INVALID:
    case SPIFFS_ERR_INDEX_FREE:
    case SPIFFS_ERR_INDEX_LU:
    case SPIFFS_ERR_INDEX_INVALID:
    case SPIFFS_ERR_CONFLICTING_NAME:
    case SPIFFS_ERR_INVALID_PARA:
        err = EINVAL;
        break;

    case SPIFFS_ERR_NOT_WRITABLE:
    case SPIFFS_ERR_NOT_READABLE:
    case SPIFFS_ERR_NOT_CONFIGURED:
        err = EACCES;
        break;

    case SPIFFS_ERR_NOT_MOUNTED:
    case SPIFFS_ERR_NOT_A_FS:
    case SPIFFS_ERR_PROBE_NOT_A_FS:
    case SPIFFS_ERR_MAGIC_NOT_POSSIBLE:
        err = ENODEV;
        break;

    case SPIFFS_ERR_FULL:
    case SPIFFS_ERR_PROBE_TOO_FEW_BLOCKS:
        err = ENOSPC;
        break;

    case SPIFFS_ERR_BAD_DESCRIPTOR:
    case SPIFFS_ERR_OUT_OF_FILE_DESCS:
        err = EBADF;
        break;

    case SPIFFS_ERR_MOUNTED:
    case SPIFFS_ERR_FILE_EXISTS:
        err = EEXIST;
        break;

    case SPIFFS_ERR_NOT_FOUND:
    case SPIFFS_ERR_NOT_A_FILE:
    case SPIFFS_ERR_DELETED:
    case SPIFFS_ERR_FILE_DELETED:
    case SPIFFS_ERR_NOT_FINALIZED:
    case SPIFFS_ERR_NOT_INDEX:
    case SPIFFS_ERR_IS_FREE:
    case SPIFFS_ERR_INDEX_SPAN_MISMATCH:
    case SPIFFS_ERR_FILE_CLOSED:
        err = ENOENT;
        break;

    case SPIFFS_ERR_NAME_TOO_LONG:
        err = ENAMETOOLONG;
        break;

    case SPIFFS_ERR_RO_NOT_IMPL:
    case SPIFFS_ERR_RO_ABORTED_OPERATION:
        err = EROFS;
        break;

    case SPIFFS_ERR_SEEK_BOUNDS:
        err = EOVERFLOW;
        break;

    case SPIFFS_ERR_END_OF_OBJECT:
    case SPIFFS_ERR_NO_DELETED_BLOCKS:
        err = ENODATA;
        break;

    case SPIFFS_ERR_ERASE_FAIL:
        err = EIO;
        break;

    default:
        err = EIO;
        break;
    }

    VFS_ERRNO_SET (err);
    return -err;
}

static int spiffs_flags_get (int oflags)
{
    int flags = 0;

    switch (oflags & O_ACCMODE)
    {
    case O_RDONLY:
        flags |= SPIFFS_O_RDONLY;
        break;
    case O_WRONLY:
        flags |= SPIFFS_O_WRONLY;
        break;
    case O_RDWR:
        flags |= SPIFFS_O_RDWR;
        break;
    default:
        break;
    }

    if (oflags & O_CREAT)
    {
        flags |= SPIFFS_O_CREAT;
    }

    if (oflags & O_EXCL)
    {
        flags |= SPIFFS_O_EXCL;
    }

    if (oflags & O_TRUNC)
    {
        flags |= SPIFFS_O_TRUNC;
    }

    if (oflags & O_APPEND)
    {
        flags |= SPIFFS_O_CREAT | SPIFFS_O_APPEND;
    }

    return flags;
}

static int spiffs_op_open (struct file *file, const char *path_in_mp, int flags)
{
    spiffs              *fs = (spiffs *) file->f_mp->m_data;
    spiffs_file          s_file;

    s_file = SPIFFS_open (fs, path_in_mp, spiffs_flags_get (flags), 0);

    if (s_file < SPIFFS_OK)
    {
        return ret_to_errno(s_file);
    }

    file->f_data = (void *) (uintptr_t) s_file;

    return 0;
}

static spiffs_file spifd_from_file (struct file *file)
{
    return (spiffs_file) (uintptr_t) file->f_data;
}

static int spiffs_op_close (struct file *file)
{
    spiffs_file  s_file = spifd_from_file (file);
    spiffs      *fs     = (spiffs *) file->f_mp->m_data;
    s32_t res = SPIFFS_close (fs, s_file);

    return ret_to_errno(res);
}

static ssize_t spiffs_op_read (struct file *file, char *buff, size_t bytes)
{
    if (buff == NULL || bytes == 0)
        return -EINVAL;

    spiffs_file  s_file = spifd_from_file (file);
    spiffs      *fs     = (spiffs *) file->f_mp->m_data;
    s32_t res = SPIFFS_read (fs, s_file, buff, bytes);

    return res < 0 ? ret_to_errno(res) : res;
}

static ssize_t spiffs_op_write (struct file *file, const char *buff, size_t bytes)
{
    if (buff == NULL || bytes == 0)
        return -EINVAL;

    spiffs_file  s_file = spifd_from_file (file);
    spiffs      *fs     = (spiffs *) file->f_mp->m_data;
    s32_t res = SPIFFS_write (fs, s_file, (void *) buff, bytes);

    return res < 0 ? ret_to_errno(res) : res;
}

static off_t spiffs_op_lseek (struct file *file, off_t off, int whence)
{
    spiffs_file  s_file = spifd_from_file (file);
    spiffs      *fs     = (spiffs *) file->f_mp->m_data;
    s32_t res = SPIFFS_lseek (fs, s_file, off, whence);

    return res < 0 ? ret_to_errno(res) : res;
}

int spiffs_op_stat (struct mount_point *mp, const char *path_in_mp, struct stat *stat)
{
    spiffs_stat s = {0};

    memset(stat, 0, sizeof(*stat));
    s32_t res = SPIFFS_stat((spiffs *)mp->m_data, path_in_mp, &s);
    if (res == SPIFFS_OK)
    {
        stat->st_size = s.size;
        if (s.type == SPIFFS_TYPE_DIR)
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

static int spiffs_op_unlink (struct mount_point *mp, const char *path_in_mp)
{
    s32_t res = SPIFFS_remove ((spiffs *) mp->m_data, path_in_mp);
    return ret_to_errno(res);
}

static int spiffs_op_rename (struct mount_point *mp, const char *path_in_mp_old,
                             const char *path_in_mp_new)
{
    s32_t res = SPIFFS_rename ((spiffs *) mp->m_data, path_in_mp_old, path_in_mp_new);
    return ret_to_errno(res);
}

static int spiffs_op_sync (struct file *file)
{
    spiffs_file  s_file = spifd_from_file (file);
    spiffs      *fs     = (spiffs *) file->f_mp->m_data;
    s32_t res = SPIFFS_fflush (fs, s_file);

    return res < 0 ? ret_to_errno(res) : res;
}

static int spiffs_op_opendir (struct dir *dir, const char *path)
{
    spiffs      *fs     = (spiffs *) dir->d_mp->m_data;
    spiffs_DIR *sdir;

    sdir = (spiffs_DIR *) malloc (sizeof (spiffs_DIR));

    if (sdir == NULL)
    {
        PRINT_ERR ("fail to malloc memory in SPIFFS, <malloc.c> is needed,"
                   "make sure it is added\n");
        return -ENOMEM;
    }

    dir->d_data   = (void *) SPIFFS_opendir (fs, path, sdir);
    dir->d_offset = 0;

    if (dir->d_data == 0)
    {
        free (sdir);
        return -ENOENT;
    }

    return LOS_OK;
}

int spiffs_op_readdir (struct dir *dir, struct dirent *dent)
{
    struct spiffs_dirent e;

    if (NULL == SPIFFS_readdir ((spiffs_DIR *) dir->d_data, &e))
    {
        return -ENOENT;
    }

    strncpy (dent->name, (const char *) e.name, LOS_MAX_FILE_NAME_LEN - 1);
    dent->name [LOS_MAX_FILE_NAME_LEN - 1] = '\0';
    dent->size = e.size;

    if (e.type == SPIFFS_TYPE_DIR)
    {
        dent->type = VFS_TYPE_DIR;
    }
    else
    {
        dent->type = VFS_TYPE_FILE;
    }

    return LOS_OK;
}

static int spiffs_op_closedir (struct dir *dir)
{
    spiffs_DIR *sdir = (spiffs_DIR *) dir->d_data;

    s32_t res = SPIFFS_closedir (sdir);

    free (sdir);

    return ret_to_errno(res);
}

static struct file_ops spiffs_ops =
{
    spiffs_op_open,
    spiffs_op_close,
    spiffs_op_read,
    spiffs_op_write,
    spiffs_op_lseek,
    spiffs_op_stat,
    spiffs_op_unlink,
    spiffs_op_rename,
    NULL,               /* ioctl not supported for now */
    spiffs_op_sync,
    spiffs_op_opendir,
    spiffs_op_readdir,
    spiffs_op_closedir,
    NULL                /* spiffs do not support mkdir */
};

static struct file_system spiffs_fs =
{
    "spiffs",
    &spiffs_ops,
    NULL,
    0
};

static spiffs *fs_ptr = NULL;
static u8_t *wbuf_ptr = NULL;
static u8_t *fds_ptr = NULL;
static u8_t *cache_ptr = NULL;

int spiffs_mount (const char *path, u32_t phys_addr, u32_t phys_size,
                  u32_t phys_erase_block, u32_t log_block_size,
                  u32_t log_page_size,
                  s32_t (*spi_rd) (struct spiffs_t *, u32_t, u32_t, u8_t *),
                  s32_t (*spi_wr) (struct spiffs_t *, u32_t, u32_t, u8_t *),
                  s32_t (*spi_er) (struct spiffs_t *, u32_t, u32_t))
{
    spiffs         *fs;
    spiffs_config   c;
    u8_t           *wbuf;
    u8_t           *fds;
    u8_t           *cache;
    int             ret = -1;

#define LOS_SPIFFS_FD_SIZE      (sizeof (spiffs_fd) * 8)
#define LOS_SPIFFS_CACHE_SIZE   (((log_page_size + 32) * 4) + 40)

    fs    = (spiffs *) malloc (sizeof (spiffs));
    wbuf  = (u8_t *)   malloc (log_page_size * 2);
    fds   = (u8_t *)   malloc (LOS_SPIFFS_FD_SIZE);
    cache = (u8_t *)   malloc (LOS_SPIFFS_CACHE_SIZE);

    if ((fs == NULL) || (wbuf == NULL) || (fds == NULL) || (cache == NULL))
    {
        PRINT_ERR ("fail to malloc memory in SPIFFS, <malloc.c> is needed,"
                   "make sure it is added\n");
        goto err_free;
    }

    memset (fs, 0, sizeof (spiffs));

    c.hal_read_f       = spi_rd;
    c.hal_write_f      = spi_wr;
    c.hal_erase_f      = spi_er;
    c.log_block_size   = log_block_size;
    c.log_page_size    = log_page_size;
    c.phys_addr        = phys_addr;
    c.phys_erase_block = phys_erase_block;
    c.phys_size        = phys_size;
    c.fh_ix_offset     = TEST_SPIFFS_FILEHDL_OFFSET;

    ret = SPIFFS_mount (fs, &c, wbuf, fds, LOS_SPIFFS_FD_SIZE, cache,
                        LOS_SPIFFS_CACHE_SIZE, NULL);

    if (ret == SPIFFS_ERR_NOT_A_FS)
    {
        PRINT_INFO ("formating fs...\n");

        SPIFFS_format (fs);

        ret = SPIFFS_mount (fs, &c, wbuf, fds, LOS_SPIFFS_FD_SIZE, cache,
                            LOS_SPIFFS_CACHE_SIZE, NULL);
    }

    if (ret != SPIFFS_OK)
    {
        PRINT_ERR ("format fail!\n");
        goto err_unmount;
    }

    ret = los_fs_mount ("spiffs", path, fs);

    if (ret == LOS_OK)
    {
        fs_ptr = fs;
        wbuf_ptr = wbuf;
        fds_ptr = fds;
        cache_ptr = cache;
        PRINT_INFO ("spiffs mount at %s done!\n", path);
        return LOS_OK;
    }

    PRINT_ERR ("failed to mount!\n");

err_unmount:
    SPIFFS_unmount (fs);
err_free:
    if (fs != NULL)
        free (fs);
    if (wbuf != NULL)
        free (wbuf);
    if (fds != NULL)
        free (fds);
    if (cache != NULL)
        free (cache);

    return ret;
}

int spiffs_unmount(const char *path)
{
    if (fs_ptr)
    {
        SPIFFS_unmount(fs_ptr);
        free(fs_ptr);
        fs_ptr = NULL;
    }
    if (wbuf_ptr)
    {
        free(wbuf_ptr);
        wbuf_ptr = NULL;
    }
    if (fds_ptr)
    {
        free(fds_ptr);
        fds_ptr = NULL;
    }
    if (cache_ptr)
    {
        free(cache_ptr);
        cache_ptr = NULL;
    }

    los_fs_unmount(path);

    return 0;
}

int spiffs_init (void)
{
    static int spiffs_inited = FALSE;

    if (spiffs_inited)
    {
        return LOS_OK;
    }

    if (los_vfs_init () != LOS_OK)
    {
        return LOS_NOK;
    }

    if (los_fs_register (&spiffs_fs) != LOS_OK)
    {
        PRINT_ERR ("failed to register fs!\n");
        return LOS_NOK;
    }

    spiffs_inited = TRUE;

    PRINT_INFO ("register spiffs done!\n");

    return LOS_OK;
}
