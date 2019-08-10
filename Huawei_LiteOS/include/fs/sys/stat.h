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

#ifndef _STAT_H
#define _STAT_H

struct  stat 
{
  unsigned long  st_dev;
  unsigned long  st_ino;
  int            st_mode;
  unsigned long  st_nlink;
  unsigned short st_uid;
  unsigned short st_gid;
  unsigned long  st_rdev;
  unsigned long  st_size;
  unsigned long  st_atime;
  unsigned long  st_mtime;
  unsigned long  st_ctime;
  unsigned long  st_blksize;
  unsigned long  st_blocks;
};

#define _IFMT           0170000 /* type of file */
#define _IFDIR          0040000 /* directory */
#define _IFCHR          0020000 /* character special */
#define _IFBLK          0060000 /* block special */
#define _IFREG          0100000 /* regular */
#define _IFLNK          0120000 /* symbolic link */
#define _IFSOCK         0140000 /* socket */
#define _IFIFO          0010000 /* fifo */

#define S_BLKSIZE       1024 /* size of a block */

#define S_ISUID         0004000 /* set user id on execution */
#define S_ISGID         0002000 /* set group id on execution */
#define S_ISVTX         0001000 /* save swapped text even after use */

#define S_IFMT          _IFMT
#define S_IFDIR         _IFDIR
#define S_IFCHR         _IFCHR
#define S_IFBLK         _IFBLK
#define S_IFREG         _IFREG
#define S_IFLNK         _IFLNK
#define S_IFSOCK        _IFSOCK
#define S_IFIFO         _IFIFO

#define S_IRWXU         (S_IRUSR | S_IWUSR | S_IXUSR)
#define S_IRUSR         0000400 /* read permission, owner */
#define S_IWUSR         0000200 /* write permission, owner */
#define S_IXUSR         0000100 /* execute/search permission, owner */
#define S_IRWXG         (S_IRGRP | S_IWGRP | S_IXGRP)
#define S_IRGRP         0000040 /* read permission, group */
#define S_IWGRP         0000020 /* write permission, grougroup */
#define S_IXGRP         0000010 /* execute/search permission, group */
#define S_IRWXO         (S_IROTH | S_IWOTH | S_IXOTH)
#define S_IROTH         0000004 /* read permission, other */
#define S_IWOTH         0000002 /* write permission, other */
#define S_IXOTH         0000001 /* execute/search permission, other */

#define S_ISBLK(m)      (((m)&_IFMT) == _IFBLK)
#define S_ISCHR(m)      (((m)&_IFMT) == _IFCHR)
#define S_ISDIR(m)      (((m)&_IFMT) == _IFDIR)
#define S_ISFIFO(m)     (((m)&_IFMT) == _IFIFO)
#define S_ISREG(m)      (((m)&_IFMT) == _IFREG)
#define S_ISLNK(m)      (((m)&_IFMT) == _IFLNK)
#define S_ISSOCK(m)     (((m)&_IFMT) == _IFSOCK)

int mkdir (const char *_path, int _mode);
int stat (const char *__restrict __path, struct stat *__restrict __sbuf);

#endif /* _STAT_H */
