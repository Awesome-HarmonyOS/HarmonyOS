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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef __GNUC__
#include <sys/errno.h>
#endif

#if defined (__GNUC__) || defined (__CC_ARM)
#include <sys/fcntl.h>
#include <los_memory.h>
#endif

#include "los_vfs.h"

#define RAMFS_TYPE_DIR          VFS_TYPE_DIR
#define RAMFS_TYPE_FILE         VFS_TYPE_FILE

struct ramfs_element
{
    char                           name [LOS_MAX_FILE_NAME_LEN];
    uint32_t                       type;
    struct ramfs_element          *sabling;
    struct ramfs_element          *parent;
    volatile uint32_t              refs;
    union
    {
        struct
        {
            size_t                 size;
            char                  *content;
        } f;
        struct
        {
            struct ramfs_element *child;
        } d;
    };
};

struct ramfs_mount_point
{
    struct ramfs_element       root;
    void                      *memory;
};

static struct ramfs_element *ramfs_file_find (struct mount_point *mp,
        const char   *path_in_mp,
        const char **path_unresolved)
{
    struct ramfs_element *walk;

    /* walk every dir */

    walk = &((struct ramfs_mount_point *) mp->m_data)->root;

    while (1)
    {
        const char            *c;
        struct ramfs_element *t;
        int                    l;

        if (walk->type != RAMFS_TYPE_DIR)
        {
            VFS_ERRNO_SET (ENOTDIR);

            return NULL;
        }

        while (*path_in_mp == '/') path_in_mp++;

        c = strchr (path_in_mp, '/');

        if (c == NULL)
        {
            l = strlen (path_in_mp);
        }
        else
        {
            l = c - path_in_mp;
        }

        if (l >= LOS_MAX_FILE_NAME_LEN)
        {
            VFS_ERRNO_SET (ENAMETOOLONG);

            return NULL;
        }

        for (t = walk->d.child; t != NULL; t = t->sabling)
        {
            if ((strncmp (t->name, path_in_mp, l) == 0) &&
                    (t->name [l] == '\0'))
            {
                break;
            }
        }

        if (t == NULL)
        {
            break;  /* no match */
        }

        path_in_mp += l;
        walk        = t;

        if (c == NULL)
        {
            break;
        }
    }

    *path_unresolved = path_in_mp;

    return walk;
}

static int ramfs_open (struct file *file, const char *path_in_mp, int flags)
{
    struct ramfs_element *ramfs_file;
    struct ramfs_element *walk;
    int                    ret = -1;

    /* openning dir like "/romfs/ not support " */

    if (*path_in_mp == '\0')
    {
        VFS_ERRNO_SET (EISDIR);
        return ret;
    }

    walk = ramfs_file_find (file->f_mp, path_in_mp, &path_in_mp);

    if (walk == NULL)
    {
        /* errno set by ramfs_file_find */
        return ret;
    }

    if ((walk->type == RAMFS_TYPE_DIR) && (*path_in_mp == '\0'))
    {
        VFS_ERRNO_SET (EISDIR);
        return -1;
    }

    if (*path_in_mp == '\0')    /* file already exist, we found it */
    {
        ramfs_file = walk;

        if (ramfs_file->type != RAMFS_TYPE_FILE)
        {
            VFS_ERRNO_SET (EISDIR);
            return -1;
        }

        if ((flags & O_CREAT) && (flags & O_EXCL))
        {
            VFS_ERRNO_SET (EEXIST);
            return -1;
        }

        if (flags & O_APPEND)
        {
            file->f_offset = ramfs_file->f.size;
        }

        ramfs_file->refs++;

        file->f_data = (void *) ramfs_file;

        return 0;
    }

    /*
     * file not found, ramfs_file holds the most dir matched, path_in_mp holds
     * the left path not resolved
     */

    if ((flags & O_CREAT) == 0)
    {
        VFS_ERRNO_SET (ENOENT);
        return -1;
    }

    if (walk->type != RAMFS_TYPE_DIR)
    {
        /* if here, BUG! */
        VFS_ERRNO_SET (ENOTDIR);
        return -1;
    }

    if (strchr (path_in_mp, '/') != NULL)
    {
        VFS_ERRNO_SET (ENOENT);             /* parent dir not exist */
        return -1;
    }

    if (strlen (path_in_mp) >= LOS_MAX_FILE_NAME_LEN)
    {
        VFS_ERRNO_SET (ENAMETOOLONG);
        return -1;
    }

    ramfs_file = malloc (sizeof (struct ramfs_element));
    if (ramfs_file == NULL)
    {
        PRINT_ERR ("fail to malloc memory in RAMFS, <malloc.c> is needed,"
                   "make sure it is added\n");
        VFS_ERRNO_SET (ENOMEM);
        return -1;
    }

    strcpy (ramfs_file->name, path_in_mp);  /* length of path_in_mp is already verified */

    ramfs_file->refs = 1;

    ramfs_file->type = RAMFS_TYPE_FILE;
    ramfs_file->sabling = walk->d.child;
    walk->d.child = ramfs_file;
    ramfs_file->f.content = NULL;
    ramfs_file->f.size = 0;
    ramfs_file->parent = walk;

    file->f_data = (void *) ramfs_file;

    return 0;
}

static int ramfs_close (struct file *file)
{
    struct ramfs_element *ramfs_file = (struct ramfs_element *) file->f_data;

    ramfs_file->refs--;

    return 0;           /* not file delete, do not free the content */
}

static ssize_t ramfs_read (struct file *file, char *buff, size_t bytes)
{
    struct ramfs_element *ramfs_file = (struct ramfs_element *) file->f_data;

    if (file->f_offset < 0)
    {
        file->f_offset = 0;
    }

    if (ramfs_file->f.size <= (size_t) file->f_offset)  /* nothing to read */
    {
        return 0;
    }

    if (ramfs_file->f.size - file->f_offset < bytes)
    {
        bytes = ramfs_file->f.size - file->f_offset;
    }

    memcpy (buff, ramfs_file->f.content + file->f_offset, bytes);

    file->f_offset += bytes;

    return bytes;
}

static ssize_t ramfs_write (struct file *file, const char *buff, size_t bytes)
{
    struct mount_point    *mp = file->f_mp;
    struct ramfs_element *ramfs_file = (struct ramfs_element *) file->f_data;

    if (file->f_offset < 0)
    {
        file->f_offset = 0;
    }

    if (file->f_offset + bytes > ramfs_file->f.size)
    {
        char *p;

        p = LOS_MemRealloc (((struct ramfs_mount_point *) mp->m_data)->memory,
                            ramfs_file->f.content, file->f_offset + bytes);

        if (p != NULL)
        {
            ramfs_file->f.content = p;
            ramfs_file->f.size = file->f_offset + bytes;
        }
        else
        {
            if (ramfs_file->f.size <= (size_t) file->f_offset)
            {
                VFS_ERRNO_SET (ENOMEM);
                return (ssize_t) - 1;
            }

            bytes = ramfs_file->f.size - file->f_offset;
        }
    }

    memcpy (ramfs_file->f.content + file->f_offset, buff, bytes);

    file->f_offset += bytes;

    return bytes;
}

static off_t ramfs_lseek (struct file *file, off_t off, int whence)
{
    struct ramfs_element *ramfs_file = (struct ramfs_element *) file->f_data;

    switch (whence)
    {
    case SEEK_SET:
        file->f_offset  = off;
        break;
    case SEEK_CUR:
        file->f_offset += off;
        break;
    case SEEK_END:
        file->f_offset  = ramfs_file->f.size;
        break;
    default:
        VFS_ERRNO_SET (EINVAL);
        return -1;
    }

    if (file->f_offset < 0)
    {
        file->f_offset = 0;
    }

    if ((size_t) file->f_offset > ramfs_file->f.size)
    {
        file->f_offset = ramfs_file->f.size;
    }

    return file->f_offset;
}

static void ramfs_del (struct ramfs_element *e)
{
    struct ramfs_element *dir;
    struct ramfs_element *t;

    if (e->parent == NULL)          /* root element, do not delete */
    {
        return;
    }

    dir = e->parent;

    t = dir->d.child;

    if (t == e)
    {
        dir->d.child = e->sabling;
    }
    else
    {
        while (t->sabling != e)
        {
            t = t->sabling;
        }

        t->sabling = e->sabling;
    }

    free (e);
}

static int ramfs_unlink (struct mount_point *mp, const char *path_in_mp)
{
    struct ramfs_element *ramfs_file;

    ramfs_file = ramfs_file_find (mp, path_in_mp, &path_in_mp);

    if ((ramfs_file == NULL) || (*path_in_mp != '\0'))
    {
        VFS_ERRNO_SET (ENOENT);
        return -1;
    }

    if (ramfs_file->refs != 0)
    {
        VFS_ERRNO_SET (EBUSY);
        return -1;
    }

    if (ramfs_file->type == RAMFS_TYPE_DIR)
    {
        if (ramfs_file->d.child != NULL)
        {
            VFS_ERRNO_SET (EBUSY);      /* have file under it busy */
            return -1;
        }
    }
    else
    {
        if (ramfs_file->f.content != NULL)
        {
            LOS_MemFree (((struct ramfs_mount_point *) mp->m_data)->memory,
                         ramfs_file->f.content);
            ramfs_file->f.content = NULL;
        }
    }

    ramfs_del (ramfs_file);

    return 0;
}

static int ramfs_rename (struct mount_point *mp, const char *path_in_mp_old,
                         const char *path_in_mp_new)
{
    struct ramfs_element *ramfs_file_old;
    struct ramfs_element *ramfs_file_new;

    ramfs_file_old = ramfs_file_find (mp, path_in_mp_old, &path_in_mp_old);

    if ((ramfs_file_old == NULL) || (*path_in_mp_old != '\0'))
    {
        VFS_ERRNO_SET (ENOENT);
        return -1;
    }

    ramfs_file_new = ramfs_file_find (mp, path_in_mp_new, &path_in_mp_new);

    /*
     * ramfs_file_new == NULL means at least parent dir not found
     * *path_in_mp_new == '\0' means file already exist
     */

    if ((ramfs_file_new == NULL) || (*path_in_mp_new == '\0'))
    {
        VFS_ERRNO_SET (ENOENT);
        return -1;
    }

    /* must in the same dir */

    if (strchr (path_in_mp_new, '/') != NULL)
    {
        VFS_ERRNO_SET (EISDIR);
        return -1;
    }

    /* must in the same dir */

    if (ramfs_file_new != ramfs_file_old->parent)
    {
        VFS_ERRNO_SET (EISDIR);
        return -1;
    }

    if (strlen (path_in_mp_new) >= LOS_MAX_FILE_NAME_LEN)
    {
        VFS_ERRNO_SET (ENAMETOOLONG);
        return -1;
    }

    strcpy (ramfs_file_old->name, path_in_mp_new);

    return 0;
}

static int ramfs_opendir (struct dir *dir, const char *path_in_mp)
{
    struct ramfs_element *ramfs_dir;
    struct mount_point    *mp = dir->d_mp;

    ramfs_dir = ramfs_file_find (mp, path_in_mp, &path_in_mp);

    if ((ramfs_dir == NULL) || (*path_in_mp != '\0'))
    {
        VFS_ERRNO_SET (ENOENT);
        return -1;
    }

    if (ramfs_dir->type != RAMFS_TYPE_DIR)
    {
        VFS_ERRNO_SET (ENOTDIR);
        return -1;
    }

    ramfs_dir->refs++;

    dir->d_data   = (void *) ramfs_dir;
    dir->d_offset = 0;

    return 0;
}

static int ramfs_readdir (struct dir *dir, struct dirent *dent)
{
    struct ramfs_element *ramfs_dir = (struct ramfs_element *) dir->d_data;
    struct ramfs_element *child;
    off_t                  i;

    for (i = 0, child = ramfs_dir->d.child;
            i < dir->d_offset && child != NULL;
            i++, child = child->sabling)
    {
        /* nop */
    }

    if (NULL == child)
    {
        VFS_ERRNO_SET (ENOENT);
        return -1;
    }

    strncpy (dent->name, child->name, LOS_MAX_FILE_NAME_LEN - 1);
    dent->name [LOS_MAX_FILE_NAME_LEN - 1] = '\0';
    dent->size = 0;

    if (child->type == RAMFS_TYPE_DIR)
    {
        dent->type = VFS_TYPE_DIR;
    }
    else
    {
        dent->type = VFS_TYPE_FILE;
        dent->size = child->f.size;
    }

    dir->d_offset++;

    return 0;
}

static int ramfs_closedir (struct dir *dir)
{
    struct ramfs_element *ramfs_dir = (struct ramfs_element *) dir->d_data;

    ramfs_dir->refs--;

    return 0;
}

static int ramfs_mkdir (struct mount_point *mp, const char *path_in_mp)
{
    struct ramfs_element *ramfs_parent;
    struct ramfs_element *ramfs_dir;
    const char            *t;
    int                    len;

    ramfs_parent = ramfs_file_find (mp, path_in_mp, &path_in_mp);

    if ((ramfs_parent == NULL) || (*path_in_mp == '\0'))
    {
        return -1;      /* dir already exist */
    }

    t = strchr (path_in_mp, '/');

    if (t != NULL)
    {
        len = t - path_in_mp;

        while (*t == '/') t++;

        if (*t != '\0')
        {
            return -1;  /* creating dir under non-existed dir */
        }
    }
    else
    {
        len = strlen (path_in_mp);
    }

    if (len >= LOS_MAX_FILE_NAME_LEN)
    {
        return -1;
    }

    ramfs_dir = (struct ramfs_element *) malloc (sizeof (struct ramfs_element));

    if (ramfs_dir == NULL)
    {
        PRINT_ERR ("fail to malloc memory in RAMFS, <malloc.c> is needed,"
                   "make sure it is added\n");
        return -1;
    }

    memset (ramfs_dir, 0, sizeof (struct ramfs_element));

    strncpy (ramfs_dir->name, path_in_mp, len);
    ramfs_dir->type       = RAMFS_TYPE_DIR;
    ramfs_dir->sabling    = ramfs_parent->d.child;
    ramfs_parent->d.child = ramfs_dir;
    ramfs_dir->parent     = ramfs_parent;

    return 0;
}

static struct file_ops ramfs_ops =
{
    ramfs_open,
    ramfs_close,
    ramfs_read,
    ramfs_write,
    ramfs_lseek,
    NULL,           /* stat not supported */
    ramfs_unlink,
    ramfs_rename,
    NULL,           /* ioctl not supported */
    NULL,           /* sync not supported */
    ramfs_opendir,
    ramfs_readdir,
    ramfs_closedir,
    ramfs_mkdir
};

static struct file_system ramfs_fs =
{
    "ramfs",
    &ramfs_ops,
    NULL,
    0
};

int ramfs_mount (const char *path, size_t block_size)
{
    struct ramfs_mount_point *rmp;

    if (strlen (path) >= LOS_MAX_FILE_NAME_LEN)
    {
        return LOS_NOK;
    }

    rmp = (struct ramfs_mount_point *) malloc (sizeof (struct ramfs_mount_point));

    if (rmp == NULL)
    {
        PRINT_ERR ("fail to malloc memory in RAMFS, <malloc.c> is needed,"
                   "make sure it is added\n");
        return LOS_NOK;
    }

    memset (rmp, 0, sizeof (struct ramfs_mount_point));
    rmp->root.type = RAMFS_TYPE_DIR;
    strncpy (rmp->root.name, path, LOS_MAX_FILE_NAME_LEN);
    rmp->memory = malloc (block_size);

    if (rmp->memory == NULL)
    {
        PRINT_ERR ("fail to malloc memory in RAMFS, <malloc.c> is needed,"
                   "make sure it is added\n");
        PRINT_ERR ("failed to allocate memory\n");
        return LOS_NOK;
    }

    if (LOS_MemInit (rmp->memory, block_size) != LOS_OK)
    {
        PRINT_ERR ("failed to init pool\n");
        free (rmp->memory);
        return LOS_NOK;
    }

    if (los_fs_mount ("ramfs", path, rmp) == LOS_OK)
    {
        PRINT_INFO ("ramfs mount at %s done!\n", path);
        return LOS_OK;
    }

    PRINT_ERR ("failed to register fs!\n");

    free (rmp->memory);
    free (rmp);

    return LOS_NOK;
}

int ramfs_init (void)
{
    static int ramfs_inited = FALSE;

    if (ramfs_inited)
    {
        return LOS_OK;
    }

    if (los_vfs_init () != LOS_OK)
    {
        PRINT_ERR ("vfs init fail!\n");
        return LOS_NOK;
    }

    if (los_fs_register (&ramfs_fs) != LOS_OK)
    {
        PRINT_ERR ("failed to register fs!\n");
        return LOS_NOK;
    }

    /* alloc 16KB memory as "disk" */

    if (ramfs_mount ("/ramfs/", 16 * 1024) != LOS_OK)
    {
        PRINT_ERR ("failed to mount ramfs!\n");
        return LOS_NOK;
    }

    PRINT_INFO ("register fs done!\n");

    ramfs_inited = TRUE;

    return LOS_OK;
}

#ifdef DEBUG
void ramfs_ls (struct ramfs_element *dir, int level)
{
    struct ramfs_element *itr;
    int                    i;

    if (dir->type != RAMFS_TYPE_DIR)
    {
        return;
    }

    for (itr = dir->d.child; itr != NULL; itr = itr->sabling)
    {
        for (i = 0; i < level; i++)
            PRINTK ("  ");

        PRINTK ("%s%c\n", itr->name, itr->type == RAMFS_TYPE_DIR ? '/' : '\0');

        if (itr->type == RAMFS_TYPE_DIR)
        {
            ramfs_ls (itr, level + 1);
        }
    }
}

extern struct mount_point *los_mp_find (const char *, const char **);

void ramfs_tree (const char *mount_path)
{
    struct mount_point    *mp = los_mp_find (mount_path, NULL);
    struct ramfs_element *walk;

    if (mp == NULL)
    {
        PRINT_ERR ("can not find mount point info for %s\n", mount_path);
        return;
    }

    walk = (struct ramfs_element *) mp->m_data;

    ramfs_ls (walk, 0);
}
#endif

