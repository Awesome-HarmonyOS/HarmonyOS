/*
 * Copyright 2008 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include "cmockery.h"

#if defined (__GNUC__) || defined (__CC_ARM)
#include "fs/sys/fcntl.h"
#include <los_printf.h>
#endif

#include "fs/los_vfs.h"


/* Defines ------------------------------------------------------------------*/
#define TEST_FS_SPIFFS      0
#define TEST_FS_FATFS       1

#define SPIFFS_PATH         "/spiffs"
#define FATFS_PATH          "/fatfs"

#define LOS_FILE            "f.txt"
#define LOS_FILE_RN         "file.txt"
#define LOS_DIR             "d"


/* Typedefs -----------------------------------------------------------------*/
/* Macros -------------------------------------------------------------------*/
#ifndef FS_PRINTF
#define FS_PRINTF(fmt, arg...)  printf("[%s:%d]" fmt "\n", __func__, __LINE__, ##arg)
#endif

/* Local variables ----------------------------------------------------------*/
static char write_buf[] = "IP:192.168.0.100\nMASK:255.255.255.0\nGW:192.168.0.1";
static char read_buf[100] = {"fs read failed"};

#define MAX_NAME_LEN 64
static char file_name[MAX_NAME_LEN];
static char file_rename[MAX_NAME_LEN];
static char dir_name[MAX_NAME_LEN];
static char path_name[MAX_NAME_LEN];

static int fs_type;


/* Extern variables ---------------------------------------------------------*/
extern int stm32f4xx_spiffs_init(int need_erase);
extern int stm32f4xx_fatfs_init(int need_erase);
extern int spiffs_unmount(const char *path);
extern int fatfs_unmount(const char *path, uint8_t drive);

/* Global variables ---------------------------------------------------------*/
/* Private function prototypes ----------------------------------------------*/
/* Public functions ---------------------------------------------------------*/

static void print_dir(const char *name, int level)
{
    if (level <= 1)
        printf("%s\n", name);
    else if (level > 10)
        return;

    struct dir *dir = los_opendir(name);
    if(dir == NULL)
    {
        FS_PRINTF("los_opendir %s failed", name);
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
        FS_PRINTF("los_closedir %s failed", name);
        return;
    }
}


static void test_file_open_normal(void **state)
{
    int fd;
    int ret;

    fd = los_open(file_name, O_CREAT);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_close(fd);
    assert_int_equal(ret, 0);

    fd = los_open(file_name, O_RDWR);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_close(fd);
    assert_int_equal(ret, 0);

    fd = los_open(file_name, O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_close(fd);
    assert_int_equal(ret, 0);
}

static void test_file_open_exception(void **state)
{
    int fd;

    (void)los_unlink(file_name);

    fd = los_open(NULL, O_CREAT | O_RDWR | O_TRUNC);
    assert_int_equal(fd, -1);

    fd = los_open(file_name, O_RDONLY);
    assert_int_equal(fd, -1);

    fd = los_open(file_name, 0);
    assert_int_equal(fd, -1);
}

static void test_file_read_normal(void **state)
{
    int fd;
    int ret;
    size_t wrlen = sizeof(write_buf);

    fd = los_open(file_name, O_CREAT | O_RDWR | O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_read(fd, read_buf, sizeof(read_buf));
    assert_true(ret <= 0); // fatfs return -1, unreasonable

    ret = los_write(fd, write_buf, wrlen);
    assert_int_equal(ret, wrlen);

    ret = los_lseek(fd, 0, 0);
    assert_int_equal(ret, 0);

    ret = los_read(fd, read_buf, sizeof(read_buf));
    assert_int_equal(ret, wrlen);
    assert_string_equal(read_buf, write_buf);

    ret = los_close(fd);
    assert_int_equal(ret, 0);
}

static void test_file_read_exception(void **state)
{
    int fd;
    int ret;

    fd = los_open(file_name, O_CREAT | O_RDWR | O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_read(fd, 0, sizeof(read_buf));
    assert_true(ret < 0);

    ret = los_read(fd, read_buf, 0);
    assert_true(ret < 0);

    ret = los_close(fd);
    assert_int_equal(ret, 0);

    fd = los_open(file_name, O_CREAT | O_WRONLY | O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_read(fd, read_buf, sizeof(read_buf));
    assert_int_equal(ret, -1);

    ret = los_close(fd);
    assert_int_equal(ret, 0);
}

static void test_file_write_normal(void **state)
{
    int fd;
    int ret;
    size_t wrlen = sizeof(write_buf);

    fd = los_open(file_name, O_CREAT | O_RDWR | O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_write(fd, write_buf, wrlen);
    assert_int_equal(ret, wrlen);

    ret = los_close(fd);
    assert_int_equal(ret, 0);
}

static void test_file_write_exception(void **state)
{
    int fd;
    int ret;
    size_t wrlen = sizeof(write_buf);

    fd = los_open(file_name, O_CREAT | O_RDWR | O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_write(fd, 0, wrlen);
    assert_true(ret <= 0);

    ret = los_write(fd, write_buf, 0);
    assert_true(ret <= 0);

    ret = los_close(fd);
    assert_int_equal(ret, 0);

    fd = los_open(file_name, O_CREAT | O_RDONLY | O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_write(fd, write_buf, wrlen);
    assert_true(ret <= 0);

    ret = los_close(fd);
    assert_int_equal(ret, 0);
}

static void test_file_seek_normal(void **state)
{
    int fd;
    int ret;
    size_t wrlen = sizeof(write_buf);

    fd = los_open(file_name, O_CREAT | O_RDWR | O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_write(fd, write_buf, wrlen);
    assert_int_equal(ret, wrlen);

    ret = los_lseek(fd, 0, 1);
    assert_int_equal(ret, wrlen);

    ret = los_lseek(fd, -10, 2);
    assert_int_equal(ret, wrlen-10);

    ret = los_lseek(fd, 0, 0);
    assert_int_equal(ret, 0);

    // spiffs does not support seek offset > file_size
    if (fs_type != TEST_FS_SPIFFS)
    {
        ret = los_lseek(fd, 1, 2);
        assert_int_equal(ret, wrlen+1);
    }

    ret = los_close(fd);
    assert_int_equal(ret, 0);
}

static void test_file_seek_exception(void **state)
{
    int fd;
    int ret;

    fd = los_open(file_name, O_CREAT | O_RDWR | O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_lseek(fd, -1, 0);
    assert_true(ret < 0);

    ret = los_lseek(fd, 0, -1);
    assert_true(ret <= 0);

    ret = los_lseek(fd, 0, 3);
    assert_true(ret <= 0);

    ret = los_close(fd);
    assert_int_equal(ret, 0);
}

static void test_file_sync_normal(void **state)
{
    int fd;
    int ret;
    size_t wrlen = sizeof(write_buf);

    fd = los_open(file_name, O_CREAT | O_RDWR | O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_write(fd, write_buf, wrlen);
    assert_int_equal(ret, wrlen);

    ret = los_sync(fd);
    assert_true(ret >= 0);

    ret = los_close(fd);
    assert_int_equal(ret, 0);
}

static void test_file_sync_exception(void **state)
{
    int fd;
    int ret;

    fd = los_open(file_name, O_CREAT | O_RDWR | O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_sync(fd);
    assert_int_equal(ret, 0);

    ret = los_close(fd);
    assert_int_equal(ret, 0);
}

static void test_file_close_normal(void **state)
{
    int fd;
    int ret;

    fd = los_open(file_name, O_CREAT | O_RDWR | O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_close(fd);
    assert_int_equal(ret, 0);
}

static void test_file_close_exception(void **state)
{
    int ret;

    ret = los_close(-1);
    assert_int_equal(ret, -1);

    ret = los_close(LOS_MAX_FILES);
    assert_int_equal(ret, -1);
}

static void test_file_stat_normal(void **state)
{
    int fd;
    int ret;
    size_t wrlen = sizeof(write_buf);

    (void)los_unlink(file_name);

    struct stat s;
    ret = los_stat(file_rename, &s);
    assert_true(ret < 0);

    fd = los_open(file_name, O_CREAT | O_RDWR | O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_write(fd, write_buf, wrlen);
    assert_int_equal(ret, wrlen);

    ret = los_close(fd);
    assert_int_equal(ret, 0);

    ret = los_stat(file_name, &s);
    assert_int_equal(ret, 0);
    assert_int_equal(s.st_size, wrlen);
}

static void test_file_stat_exception(void **state)
{
    int ret;
    struct stat s;
    ret = los_stat(NULL, &s);
    assert_int_equal(ret, -1);
}

static void test_file_rename_normal(void **state)
{
    int fd;
    int ret;

    (void)los_unlink(file_rename);

    fd = los_open(file_name, O_CREAT | O_RDWR | O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_close(fd);
    assert_int_equal(ret, 0);

    ret = los_rename(file_name, file_rename);
    assert_int_equal(ret, 0);

    ret = los_rename(file_rename, file_name);
    assert_int_equal(ret, 0);
}

static void test_file_rename_exception(void **state)
{
    int fd;
    int ret;

    fd = los_open(file_name, O_CREAT | O_RDWR | O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_close(fd);
    assert_int_equal(ret, 0);

    ret = los_rename(file_name, file_name);
    if (fs_type == TEST_FS_SPIFFS)
    {
        assert_int_not_equal(ret, 0);
    }
    else
    {
        assert_int_equal(ret, 0);
    }

    ret = los_rename(file_name, NULL);
    assert_int_equal(ret, -1);

    ret = los_rename(NULL, NULL);
    assert_int_equal(ret, -1);
}

static void test_file_unlink_normal(void **state)
{
    int fd;
    int ret;

    (void)los_unlink(file_name);
    (void)los_unlink(file_rename);

    fd = los_open(file_name, O_CREAT | O_RDWR | O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_close(fd);
    assert_int_equal(ret, 0);

    ret = los_unlink(file_name);
    assert_int_equal(ret, 0);
}

static void test_file_unlink_exception(void **state)
{
    int ret;

    (void)los_unlink(file_name);
    (void)los_unlink(file_rename);

    ret = los_unlink(file_name);
    assert_int_not_equal(ret, 0);

    ret = los_unlink(NULL);
    assert_int_equal(ret, -1);
}


static void test_dir_make_normal(void **state)
{
    int ret;

    // spiffs does not support real directories, so unable to delete or make
    if (fs_type != TEST_FS_SPIFFS)
    {
        (void)los_unlink(dir_name);

        ret = los_mkdir(dir_name, 0);
        assert_int_equal(ret, 0);

        ret = los_mkdir(dir_name, 0);
        assert_int_not_equal(ret, 0);
    }
}

static void test_dir_make_exception(void **state)
{
    int ret;

    ret = los_mkdir(NULL, 0);
    assert_int_equal(ret, -1);

    ret = los_mkdir("unknown", 0);
    assert_int_equal(ret, -1);
}

static void test_dir_open_normal(void **state)
{
    int ret;

    if (fs_type != TEST_FS_SPIFFS)
    {
        (void)los_unlink(dir_name);

        ret = los_mkdir(dir_name, 0);
        assert_int_equal(ret, 0);
    }

    struct dir *dir = los_opendir(dir_name);
    assert_true(dir != NULL);

    ret = los_closedir(dir);
    assert_int_equal(ret, 0);
}

static void test_dir_open_exception(void **state)
{
    struct dir *dir = NULL;

    (void)los_unlink(dir_name);
    (void)los_unlink(path_name);

    // spiffs can open/read/close any valid path name as a directory
    if (fs_type != TEST_FS_SPIFFS)
    {
        dir = los_opendir(dir_name);
        assert_true(dir == NULL);

        dir = los_opendir(file_name);
        assert_true(dir == NULL);

        dir = los_opendir(path_name);
        assert_true(dir == NULL);
    }

    dir = los_opendir(NULL);
    assert_true(dir == NULL);

    dir = los_opendir("unknown");
    assert_true(dir == NULL);
}

static void test_dir_read_normal(void **state)
{
    int fd;
    int ret;

    if (fs_type != TEST_FS_SPIFFS)
    {
        (void)los_unlink(dir_name);

        ret = los_mkdir(dir_name, 0);
        assert_int_equal(ret, 0);
    }

    fd = los_open(path_name, O_CREAT | O_RDWR | O_TRUNC);
    assert_in_range(fd, 0, LOS_MAX_FILES);

    ret = los_close(fd);
    assert_int_equal(ret, 0);

    struct dir *dir = los_opendir(dir_name);
    assert_true(dir != NULL);

    struct dirent *dirent = los_readdir(dir);
    assert_true(dirent != NULL && dirent->name[0] != 0);
    if (fs_type == TEST_FS_FATFS)
    {
        assert_true(strcasecmp(dirent->name, LOS_FILE) == 0);
    }

    if (fs_type != TEST_FS_SPIFFS)
    {
        dirent = los_readdir(dir);
        assert_true(dirent == NULL || dirent->name[0] == 0);
    }

    ret = los_closedir(dir);
    assert_int_equal(ret, 0);

    ret = los_unlink(path_name);
    assert_int_equal(ret, 0);

    dir = los_opendir(dir_name);
    assert_true(dir != NULL);

    if (fs_type != TEST_FS_SPIFFS)
    {
        dirent = los_readdir(dir);
        // when reading a empty directory, jffs2 returns NULL,
        // and fatfs returns dirent with name ""
        assert_true(dirent == NULL || dirent->name[0] == 0);
    }

    ret = los_closedir(dir);
    assert_int_equal(ret, 0);
}

static void test_dir_read_exception(void **state)
{
    struct dirent *dirent = los_readdir(NULL);
    assert_true(dirent == NULL);
}

static void test_dir_close_normal(void **state)
{
    int ret;

    if (fs_type != TEST_FS_SPIFFS)
    {
        (void)los_unlink(dir_name);

        ret = los_mkdir(dir_name, 0);
        assert_int_equal(ret, 0);
    }

    struct dir *dir = los_opendir(dir_name);
    assert_true(dir != NULL);

    ret = los_closedir(dir);
    assert_int_equal(ret, 0);
}

static void test_dir_close_exception(void **state)
{
    int ret = los_closedir(NULL);
    assert_int_not_equal(ret, 0);
}


int fs_test_main(void)
{
    const UnitTest tests[] =
    {
        unit_test(test_file_open_normal),
        unit_test(test_file_open_exception),
        unit_test(test_file_read_normal),
        unit_test(test_file_read_exception),
        unit_test(test_file_write_normal),
        unit_test(test_file_write_exception),
        unit_test(test_file_seek_normal),
        unit_test(test_file_seek_exception),
        unit_test(test_file_sync_normal),
        unit_test(test_file_sync_exception),
        unit_test(test_file_close_normal),
        unit_test(test_file_close_exception),
        unit_test(test_file_stat_normal),
        unit_test(test_file_stat_exception),
        unit_test(test_file_rename_normal),
        unit_test(test_file_rename_exception),
        unit_test(test_file_unlink_normal),
        unit_test(test_file_unlink_exception),
        unit_test(test_dir_make_normal),
        unit_test(test_dir_make_exception),
        unit_test(test_dir_open_normal),
        unit_test(test_dir_open_exception),
        unit_test(test_dir_read_normal),
        unit_test(test_dir_read_exception),
        unit_test(test_dir_close_normal),
        unit_test(test_dir_close_exception),
    };

    printf("Huawei LiteOS File System Test\n");

    // spiffs
    int ret = stm32f4xx_spiffs_init(0);
    if(ret < 0)
    {
        FS_PRINTF("stm32f4xx_spiffs_init failed: %d", ret);
        return -1;
    }

    snprintf(file_name, sizeof(file_name), "%s/%s", SPIFFS_PATH, LOS_FILE);
    snprintf(file_rename, sizeof(file_rename), "%s/%s", SPIFFS_PATH, LOS_FILE_RN);
    snprintf(dir_name, sizeof(dir_name), "%s/%s", SPIFFS_PATH, LOS_DIR);
    snprintf(path_name, sizeof(path_name), "%s/%s", dir_name, LOS_FILE);

    print_dir("/spiffs", 1);
    fs_type = TEST_FS_SPIFFS;
    run_tests(tests);
    spiffs_unmount("/spiffs/");

    // fatfs
    int drive = stm32f4xx_fatfs_init(0);
    if(drive < 0)
    {
        FS_PRINTF("stm32f4xx_fatfs_init failed, drive: %d", drive);
    }

    snprintf(file_name, sizeof(file_name), "%s/%d:/%s", FATFS_PATH, drive, LOS_FILE);
    snprintf(file_rename, sizeof(file_rename), "%s/%d:/%s", FATFS_PATH, drive, LOS_FILE_RN);
    snprintf(dir_name, sizeof(dir_name), "%s/%d:/%s", FATFS_PATH, drive, LOS_DIR);
    snprintf(path_name, sizeof(path_name), "%s/%s", dir_name, LOS_FILE);

    print_dir("/fatfs/0:", 1);
    fs_type = TEST_FS_FATFS;
    run_tests(tests);
    fatfs_unmount("/fatfs/", drive);

    return 0;
}
