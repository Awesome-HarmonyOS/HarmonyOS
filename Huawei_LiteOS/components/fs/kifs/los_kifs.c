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

#include <los_printf.h>

#include <los_vfs.h>
#include <los_kifs.h>

struct kifs_node
{
    char                        name [LOS_MAX_FILE_NAME_LEN];
    uint32_t                    attr;       /* R(readable)/W(writable)/E(exclusive)/D(dir)/B(buffer) */
    struct kifs_node           *sabling;
    struct kifs_node           *parent;
    union
    {
        struct kifs_ops        *kiops;      /* kiops if is file with ops */
        void                   *buff;       /* buff addr, if file is linked to buffer */
        struct kifs_node       *child;      /* child if is dir */
    };
    union
    {
        void                   *arg;        /* arg for ops if is file with ops */
        size_t                  size;       /* buff size, if file is linked to buffer */
    };
};

static struct kifs_node *kifs_file_find (struct kifs_node *root,
        const char   *path_in_mp,
        const char **path_unresolved)
{
    struct kifs_node *dir = root;

    while (1)
    {
        const char        *c;
        struct kifs_node *t;
        int                l;

        if ((dir->attr & KIFS_ATTR_D) == 0)
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

        for (t = dir->child; t != NULL; t = t->sabling)
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
        dir        = t;

        if (c == NULL)
        {
            break;
        }
    }

    *path_unresolved = path_in_mp;

    return dir;
}

static int kifs_open (struct file *file, const char *path_in_mp, int flags)
{
    struct kifs_node *node;

    node = kifs_file_find ((struct kifs_node *) file->f_mp->m_data, path_in_mp,
                           &path_in_mp);

    if (node == NULL)
    {
        return -1;
    }

    if (*path_in_mp != '\0')
    {
        VFS_ERRNO_SET (ENOENT);
        return -1;
    }

    if (node->attr & KIFS_ATTR_D)
    {
        VFS_ERRNO_SET (EISDIR);
        return -1;
    }

    file->f_data = (void *) node;

    if ((node->attr & KIFS_ATTR_B) != 0)
    {
        /* linked buffer do not have kiops */
        return 0;
    }

    if (node->kiops->open == NULL)
    {
        return 0;   /* if open is NULL, means the file do not need it! */
    }

    return node->kiops->open (node->arg, flags);
}

static int kifs_close (struct file *file)
{
    struct kifs_node *node = (struct kifs_node *) file->f_data;

    if (node == NULL)
    {
        return -1;
    }

    if (node->attr & KIFS_ATTR_B)
    {
        return 0;
    }

    if (node->kiops->close == NULL)
    {
        return 0;   /* if close is NULL, means the file do not need it! */
    }

    return node->kiops->close (node->arg);
}

static ssize_t kifs_read (struct file *file, char *buff, size_t bytes)
{
    struct kifs_node *node = (struct kifs_node *) file->f_data;

    if ((node->attr & KIFS_ATTR_R) == 0)
    {
        VFS_ERRNO_SET (EACCES);
        return (ssize_t) - 1;
    }

    if (node->attr & KIFS_ATTR_B)
    {
        bytes = bytes > node->size ? node->size : bytes;

        memcpy (buff, node->buff, bytes);

        return bytes;
    }

    if (node->kiops->read == NULL)
    {
        VFS_ERRNO_SET (EACCES);
        return (ssize_t) - 1;
    }

    return node->kiops->read (node->arg, buff, bytes);
}

static ssize_t kifs_write (struct file *file, const char *buff, size_t bytes)
{
    struct kifs_node *node = (struct kifs_node *) file->f_data;

    if ((node->attr & KIFS_ATTR_W) == 0)
    {
        VFS_ERRNO_SET (EACCES);
        return (ssize_t) - 1;
    }

    if (node->attr & KIFS_ATTR_B)
    {
        bytes = bytes > node->size ? node->size : bytes;

        memcpy (node->buff, buff, bytes);

        return bytes;
    }

    if (node->kiops->write == NULL)
    {
        VFS_ERRNO_SET (EACCES);
        return (ssize_t) - 1;
    }

    return node->kiops->write (node->arg, buff, bytes);
}

static int kifs_ioctl (struct file *file, int func, unsigned long arg)
{
    struct kifs_node *node = (struct kifs_node *) file->f_data;

    if (node->attr & KIFS_ATTR_B)
    {
        return -1;
    }

    if (node->kiops->ioctl == NULL)
    {
        return -1;
    }

    /* <node->arg> is the private data for this kifile, the <arg> is the one
     * of ioctl */

    return node->kiops->ioctl (node->arg, func, arg);
}

static int kifs_opendir (struct dir *dir, const char *path_in_mp)
{
    struct kifs_node *node;

    node = kifs_file_find ((struct kifs_node *) dir->d_mp->m_data, path_in_mp,
                           &path_in_mp);

    if ((node == NULL) || (*path_in_mp != '\0'))
    {
        VFS_ERRNO_SET (ENOENT);
        return -1;
    }

    if ((node->attr & KIFS_ATTR_D) == 0)
    {
        VFS_ERRNO_SET (ENOTDIR);
        return -1;
    }

    dir->d_data   = (void *) node;
    dir->d_offset = 0;

    return 0;
}

static int kifs_readdir (struct dir *dir, struct dirent *dent)
{
    struct kifs_node *node = (struct kifs_node *) dir->d_data;
    struct kifs_node *child;
    off_t              i;

    if (node == NULL)
    {
        return -1;
    }

    for (i = 0, child = node->child;
            i < dir->d_offset && child != NULL;
            i++, child = child->sabling)
    {
        /* nop */
    }

    if (child == NULL)
    {
        VFS_ERRNO_SET (ENOENT);
        return -1;
    }

    strncpy (dent->name, child->name, LOS_MAX_FILE_NAME_LEN - 1);
    dent->name [LOS_MAX_FILE_NAME_LEN - 1] = '\0';
    dent->size = 0;

    if ((child->attr & KIFS_ATTR_D) != 0)
    {
        dent->type = VFS_TYPE_DIR;
    }
    else
    {
        dent->type = VFS_TYPE_FILE;
        dent->size = child->size;
    }

    dir->d_offset++;

    return 0;
}

static int kifs_closedir (struct dir *dir)
{
    return 0;
}

static struct file_ops kifs_ops =
{
    kifs_open,
    kifs_close,
    kifs_read,
    kifs_write,
    NULL,           /* lseek not suported */
    NULL,           /* stat not supported */
    NULL,           /* unlink not supported */
    NULL,           /* rename not supported */
    kifs_ioctl,     /* ioctl not supported */
    NULL,           /* sync not supported */
    kifs_opendir,
    kifs_readdir,
    kifs_closedir,
    NULL            /* mkdir not supported */
};

static struct file_system kifs_fs =
{
    "kifs",
    &kifs_ops,
    NULL,
    0
};

static struct kifs_node *kifs_file_creat (void *root,
        const char *path_in_mp,
        uint32_t flags)
{
    struct kifs_node *dir;
    struct kifs_node *node;
    const char        *t;

    if (path_in_mp [strlen (path_in_mp) - 1] == '/')
    {
        return NULL;
    }

    dir = kifs_file_find ((struct kifs_node *) root, path_in_mp, &path_in_mp);

    if (dir == NULL)   /* impossible */
    {
        return NULL;
    }

    if (*path_in_mp == '\0')
    {
        return NULL;
    }

    if ((dir->attr & KIFS_ATTR_D) == 0)
    {
        return NULL;
    }

    while ((t = strchr (path_in_mp, '/')) != NULL)
    {
        if ((t - path_in_mp) >= LOS_MAX_FILE_NAME_LEN)
        {
            return NULL;
        }

        node = (struct kifs_node *) malloc (sizeof (struct kifs_node));

        if (node == NULL)
        {
            PRINT_ERR ("fail to malloc memory in KIFS, <malloc.c> is needed,"
                       "make sure it is added\n");
            return NULL;
        }

        memset (node, 0, sizeof (struct kifs_node));
        strncpy (node->name, path_in_mp, t - path_in_mp);

        node->parent  = dir;
        node->sabling = dir->child;
        dir->child    = node;
        node->attr    = KIFS_ATTR_D;

        dir           = node;
        path_in_mp    = t + 1;

        while (*path_in_mp == '/') path_in_mp++;
    }

    if (*path_in_mp == '\0')
    {
        return NULL;
    }

    node = (struct kifs_node *) malloc (sizeof (struct kifs_node));

    if (node == NULL)
    {
        PRINT_ERR ("fail to malloc memory in KIFS, <malloc.c> is needed,"
                   "make sure it is added\n");
        return NULL;
    }

    memset (node, 0, sizeof (struct kifs_node));
    strcpy (node->name, path_in_mp);

    node->parent  = dir;
    node->sabling = dir->child;
    dir->child    = node;
    node->attr    = flags;

    return node;
}

int los_kifs_create (void *root, const char *path_in_mp, uint32_t flags,
                     struct kifs_ops *kiops, void *arg)
{
    struct kifs_node *node;

    if ((kiops == NULL) || ((flags & (KIFS_ATTR_R | KIFS_ATTR_W)) == 0))
    {
        return -1;
    }

    node = kifs_file_creat (root, path_in_mp, flags);

    if (node == NULL)
    {
        return -1;
    }

    node->kiops = kiops;
    node->arg   = arg;

    return 0;
}

int los_kifs_link (void *root, const char *path_in_mp, uint32_t flags,
                   void *buff, size_t size)
{
    struct kifs_node *node;

    if ((buff == NULL) || ((flags & (KIFS_ATTR_R | KIFS_ATTR_W)) == 0))
    {
        return -1;
    }

    node = kifs_file_creat (root, path_in_mp, flags);

    if (node == NULL)
    {
        return -1;
    }

    node->buff  = buff;
    node->size  = size;
    node->attr |= KIFS_ATTR_B;

    return 0;
}

void *los_kifs_mount (const char *path)
{
    struct kifs_node *root;

    if (los_vfs_init () != LOS_OK)
    {
        PRINT_ERR ("vfs init fail!\n");
        return NULL;
    }

    if (strlen (path) >= LOS_MAX_FILE_NAME_LEN)
    {
        return NULL;
    }

    root = (struct kifs_node *) malloc (sizeof (struct kifs_node));

    if (root == NULL)
    {
        PRINT_ERR ("fail to malloc memory in KIFS, <malloc.c> is needed,"
                   "make sure it is added\n");
        return NULL;
    }

    memset (root, 0, sizeof (struct kifs_node));

    strcpy (root->name, path);

    root->attr = KIFS_ATTR_D;

    if (los_fs_mount ("kifs", path, root) == LOS_OK)
    {
        return (void *) root;
    }

    free (root);

    return NULL;
}

int los_kifs_init (void)
{
    static int kifs_inited = FALSE;

    if (kifs_inited)
    {
        return LOS_OK;
    }

    if (los_fs_register (&kifs_fs) != LOS_OK)
    {
        PRINT_ERR ("kifs fs register fail!\n");
        return LOS_NOK;
    }

    kifs_inited = TRUE;

    return LOS_OK;
}

