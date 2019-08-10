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
#include "fs_common.h"


static char s_ucaWriteBuffer[] = "hello world";
static char s_ucaReadBuffer[100];


void fs_demo(void)
{
    printf("Huawei LiteOS File System Demo.\n");

#if defined(FS_SPIFFS)
    extern void spiffs_demo();
    spiffs_demo();
#endif

#if defined(FS_FATFS)
    extern void fatfs_demo();
    fatfs_demo();
#endif
}


int write_file(const char *name, char *buff, int len)
{
    int fd;
    int ret;

    if(name == NULL || buff == NULL || len <= 0)
    {
        FS_LOG_ERR("invalid parameter.");
        return -1;
    }
    fd = los_open(name, O_CREAT | O_WRONLY | O_TRUNC);
    if(fd < 0)
    {
        FS_LOG_ERR("los_open file %s failed.", name);
        return -1;
    }
    ret = los_write(fd, buff, len);
    if(ret < 0)
    {
        FS_LOG_ERR("los_write file %s failed.", name);
        los_close(fd);
        return -1;
    }
    los_close(fd);
    return 0;
}

int read_file(const char *name, char *buff, int len)
{
    int fd;
    int ret;

    if(name == NULL || buff == NULL || len <= 0)
    {
        FS_LOG_ERR("invalid parameter.");
        return -1;
    }
    fd = los_open(name, O_RDONLY);
    if(fd < 0)
    {
        FS_LOG_ERR("los_open file %s failed.", name);
        return -1;
    }
    ret  = los_read(fd, buff, len);
    if(ret <= 0)
    {
        FS_LOG_ERR("los_read file %s failed.", name);
        los_close(fd);
        return -1;
    }
    los_close(fd);
    return 0;
}

int open_dir(const char *name, struct dir **dir)
{
    int ret = 0;
    int counter = 3;

    if(name == NULL || dir == NULL)
    {
        FS_LOG_ERR("invalid parameter.");
        return -1;
    }

    do
    {
        *dir = los_opendir(name);
        if(*dir == NULL)
        {
            FS_LOG_ERR("los_opendir %s failed, ret=%d.", name, ret);
            ret = los_mkdir(name, 0);
            if(ret != 0)
            {
                FS_LOG_ERR("los_mkdir %s failed, ret=%d.", name, ret);
            }
            else
            {
                FS_LOG_ERR("los_mkdir %s successfully.", name);
            }
        }
    }while(*dir == NULL && --counter > 0);

    if(counter <= 0)
    {
        FS_LOG_ERR("los_opendir/los_mkdir %s failed, ret=%d.", name, ret);
        return -1;
    }
    return 0;
}

int read_dir(const char *name, struct dir *dir)
{
    int flag = 1;
    struct dirent *pDirent = NULL;

    if(name == NULL || dir == NULL)
    {
        FS_LOG_ERR("invalid parameter.");
        return -1;
    }

    while(1)
    {
        pDirent = los_readdir(dir);
        if(pDirent == NULL || pDirent->name[0] == '\0')
        {
            if(flag == 1)
            {
                FS_LOG_ERR("los_readdir %s failed.", name);
                return -1;
            }
            else break;
        }
        flag = 0;
        printf("los_readdir %s: name=%s, type=%d, size=%d\n", name, pDirent->name,
            pDirent->type, pDirent->size);
    }
    return 0;
}

void los_vfs_io(char *file_name, char *dir_name)
{
    int ret = 0;
    struct dir *pDir = NULL;
    int wrlen = sizeof(s_ucaWriteBuffer) - 1;
    int rdlen = sizeof(s_ucaReadBuffer);

    rdlen = MIN(wrlen, rdlen);

    /**************************
     *  file operation
     **************************/
    ret = write_file(file_name, s_ucaWriteBuffer, wrlen);
    if(ret < 0)
    {
       (void)los_unlink(file_name);
        return;
    }

    ret = read_file(file_name, s_ucaReadBuffer, rdlen);
    if(ret < 0)
    {
        (void)los_unlink(file_name);
        return;
    }
    printf("*********** readed %d data ***********\r\n%s\r\n"
           "**************************************\r\n", rdlen, s_ucaReadBuffer);

    /****************************
     *  dir operation
     ****************************/
    sprintf(file_name, "%s/%s", (char *)dir_name, LOS_FILE);
    ret = open_dir(dir_name, &pDir);
    if(ret < 0)
    {
        (void)los_unlink(file_name);
        return;
    }


    ret = write_file(file_name, s_ucaWriteBuffer, wrlen);
    if(ret < 0)
    {
        (void)los_closedir(pDir);
        (void)los_unlink(file_name);
        return;
    }

    ret = read_dir(dir_name, pDir);
    if(ret < 0)
    {
        (void)los_closedir(pDir);
        (void)los_unlink(file_name);
        return;
    }

    ret = los_closedir(pDir);
    if(ret < 0)
    {
        FS_LOG_ERR("los_closedir %s failed.", dir_name);
        (void)los_unlink(file_name); // remove file_name
        return;
    }
    (void)los_unlink(file_name); // remove file_name
}

void make_dir(const char *name)
{
    int count = 0;
    char tmp_dir[128];

    int num = snprintf(tmp_dir, sizeof(tmp_dir)-2, "%s", name);
    if (num <= 0)
    {
        return;
    }
    else if (tmp_dir[num-1] != '/')
    {
        tmp_dir[num] = '/';
        tmp_dir[num+1] = 0;
    }

    for (int i = 0; tmp_dir[i] != 0; i++)
    {
        if (tmp_dir[i] == '/')
        {
            count++;
            if (count > 2)
            {
                tmp_dir[i] = 0;

                (void)los_mkdir(tmp_dir, 0);

                tmp_dir[i] = '/';
            }
        }
    }
}

void print_dir(const char *name, int level)
{
    if (level <= 1)
        printf("%s\n", name);
    else if (level > 10)
        return;

    struct dir *dir = los_opendir(name);
    if(dir == NULL)
    {
        FS_LOG_ERR("los_opendir %s failed", name);
        return;
    }

    while(1)
    {
        struct dirent *dirent = los_readdir(dir);
        if(dirent == NULL || dirent->name[0] == 0)
        {
            break;
        }

        if (dirent->type == VFS_TYPE_DIR
            && strcmp(dirent->name, ".")
            && strcmp(dirent->name, ".."))
        {
            char tmp_path[LOS_MAX_DIR_NAME_LEN+2];
            printf("|%*s%s/\n", level*4, "--->", dirent->name);
            snprintf(tmp_path, sizeof(tmp_path), "%s/%s", name, dirent->name);
            print_dir(tmp_path, level+1);
        }
        else
        {
            printf("|%*s%s\n", level*4, "--->", dirent->name);
        }
    }

    if (los_closedir(dir) < 0)
    {
        FS_LOG_ERR("los_closedir %s failed", name);
        return;
    }
}


/* Private functions --------------------------------------------------------*/

