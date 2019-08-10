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

#ifndef _LOS_VFS_H
#define _LOS_VFS_H

#include <stdio.h>
#include <stdint.h>
#include <los_typedef.h>
#include "fs/sys/stat.h"

#define LOS_MAX_DIR_NAME_LEN                 255
#define LOS_MAX_FILE_NAME_LEN               16
#define LOS_FS_MAX_NAME_LEN                 LOS_MAX_FILE_NAME_LEN
#define LOS_MAX_FILES                       8

struct file;
struct mount_point;
struct dir;
struct dirent;

//#ifdef __CC_ARM
typedef int                                 ssize_t;
typedef long                                off_t;
//#endif

#if defined (__GNUC__) || defined (__CC_ARM)
#define VFS_ERRNO_SET(err)                  (errno = (err))
#else
#define VFS_ERRNO_SET(err)
#endif

/*lint -e145*/
struct file_ops
{
    int     (*open)     (struct file *, const char *, int);
    int     (*close)    (struct file *);
    ssize_t (*read)     (struct file *, char *, size_t);
    ssize_t (*write)    (struct file *, const char *, size_t);
    off_t   (*lseek)    (struct file *, off_t, int);
    int     (*stat)     (struct mount_point *, const char *, struct stat *);
    int     (*unlink)   (struct mount_point *, const char *);
    int     (*rename)   (struct mount_point *, const char *, const char *);
    int     (*ioctl)    (struct file *, int, unsigned long);
    int     (*sync)     (struct file *);
    int     (*opendir)  (struct dir *, const char *);
    int     (*readdir)  (struct dir *, struct dirent *);
    int     (*closedir) (struct dir *);
    int     (*mkdir)    (struct mount_point *, const char *);
};
/*lint +e145*/

struct file_system
{
    const char            fs_name [LOS_FS_MAX_NAME_LEN];
    struct file_ops     * fs_fops;
    struct file_system  * fs_next;
    volatile uint32_t     fs_refs;
};

struct mount_point
{
    struct file_system  * m_fs;
    struct mount_point  * m_next;
    const char          * m_path;
    volatile uint32_t     m_refs;
    UINT32                m_mutex;
    void                * m_data;   /* used by fs private data for this mount point (like /sdb1, /sdb2), */
};

#define FILE_STATUS_NOT_USED        0
#define FILE_STATUS_INITING         1
#define FILE_STATUS_READY           2
#define FILE_STATUS_CLOSING         3

#define VFS_TYPE_FILE               0
#define VFS_TYPE_DIR                1

struct file
{
    struct file_ops    * f_fops;
    UINT32               f_flags;
    UINT32               f_status;
    off_t                f_offset;
    struct mount_point * f_mp;      /* can get private mount data here */
    UINT32               f_owner;   /* the task that openned this file */
    void               * f_data;
};

struct dirent
{
    char                 name [LOS_MAX_DIR_NAME_LEN+1];
    UINT32               type;
    UINT32               size;
};

struct dir
{
    struct mount_point * d_mp;      /* can get private mount data here */
    struct dirent        d_dent;
    off_t                d_offset;
    void               * d_data;
};

extern int      los_open (const char *, int);
extern int      los_close (int);
extern ssize_t  los_read (int, char *, size_t);
extern ssize_t  los_write (int, const void *, size_t);
extern off_t    los_lseek (int, off_t, int);
extern int      los_stat (const char *, struct stat *);
extern int      los_unlink (const char *);
extern int      los_rename (const char *, const char *);
extern int      los_ioctl (int, int, ...);
int             los_sync (int fd);
struct dir    * los_opendir (const char * path);
struct dirent * los_readdir (struct dir * dir);
int             los_closedir (struct dir * dir);
int             los_mkdir (const char * path, int mode);

extern int      los_fs_register (struct file_system *);
extern int      los_fs_unregister (struct file_system *);
extern int      los_fs_mount (const char *, const char *, void *);
extern int      los_fs_unmount (const char *);
extern int      los_vfs_init (void);

extern int      open (const char *path, int flags,...);
extern int      close (int fd);
extern ssize_t  read (int fd, void *buff, size_t bytes);
extern ssize_t  write (int fd, const void *buff, size_t bytes);
extern off_t    lseek (int fd, off_t off, int whence);
extern int      stat (const char *path, struct stat *stat);
extern int      unlink (const char *path);
extern int      rename (const char *oldpath, const char *newpath);
extern int      fsync (int fd);
extern struct dir *opendir (const char *path);
extern struct dirent *readdir (struct dir *dir);
extern int      closedir (struct dir *dir);
extern int      mkdir (const char *path, int mode);

#endif
