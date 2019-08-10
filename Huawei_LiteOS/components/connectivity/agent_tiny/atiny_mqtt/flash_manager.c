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

#include "flash_manager.h"
#include "log/atiny_log.h"

#define FLASH_MAGIC_NUM 0x57381232

#define MAGIC_NUM_POS 0
#define TOTAL_LEN_POS 4
#define DATA_POS 8

#define MAX_FLASH_INFO ((STRING_MAX_LEN + 1) * (MAX_DATA_ITEM) + DATA_POS)


static int (*g_cmd_ioctl)(mqtt_cmd_e cmd, void *arg, int32_t len);

static int flash_manager_parse(const char *buf, uint32_t len, flash_info_s *flash_info)
{
    uint32_t i;
    uint32_t tmp_len;
    int ret;

    for (i = 0; i < MAX_DATA_ITEM; i++)
    {
        tmp_len = strnlen(buf, len);
        if ((tmp_len >= len) || (tmp_len > STRING_MAX_LEN))
        {
           ATINY_LOG(LOG_FATAL, "string len err tmp_len %ld,len %ld", tmp_len, len);
           break;
        }

        flash_info->items[i] = atiny_strdup(buf);
        if (flash_info->items[i] == NULL)
        {
            ATINY_LOG(LOG_FATAL, "atiny_strdup fail");
            ret = ATINY_MALLOC_FAILED;
            goto EXIT_ERR;
        }
        buf += (tmp_len + 1);
        len -= (tmp_len + 1);
    }

    if ((i != MAX_DATA_ITEM) || (len != 0))
    {
        ATINY_LOG(LOG_FATAL, "data info err item num %ld,left len %ld", i, len);
        ret = ATINY_ERR;
        goto EXIT_ERR;
    }
    ATINY_LOG(LOG_INFO, "deviceid and password exist");
    return ATINY_OK;

EXIT_ERR:
    flash_manager_destroy_flash_info(flash_info);
    return ret;

}

static int flash_manager_get_need_len(const flash_info_s *flash_info, uint32_t *len)
{
   uint32_t i;

   *len  = DATA_POS;
   for (i = 0; i < MAX_DATA_ITEM; i++)
   {
       uint32_t tmp_len = strnlen(flash_info->items[i], MAX_FLASH_INFO);
       *len += (tmp_len + 1);
       if (*len > MAX_FLASH_INFO)
       {
           ATINY_LOG(LOG_ERR, "write info exceed max");
           return ATINY_ERR;
       }

   }
   return ATINY_OK;
}

static void flash_manager_write_buffer(const flash_info_s *flash_info, char *buf, uint32_t len)
{
   uint32_t i;
   *((uint32_t *)buf) = FLASH_MAGIC_NUM;
   *((uint32_t *)&buf[TOTAL_LEN_POS]) = len;

   buf += DATA_POS;
   len -= DATA_POS;
   for (i = 0; i < MAX_DATA_ITEM; i++)
   {
     int32_t tmp_len;
     tmp_len = snprintf(buf, len, "%s", flash_info->items[i]);
     if ((tmp_len >= len) || (tmp_len < 0))
     {
        ATINY_LOG(LOG_FATAL, "snprintf err,tmp_len %ld, len %d", tmp_len, len);
        return;
     }
     buf += (tmp_len + 1);
     len -= (tmp_len + 1);
   }
}

void flash_manager_init(int (*cmd_ioctl)(mqtt_cmd_e cmd, void *arg, int32_t len))
{
    g_cmd_ioctl = cmd_ioctl;
}

int flash_manager_read(flash_info_s *flash_info)
{
    int ret;
    uint32_t len;
    uint32_t magic_num;
    uint8_t *buf;

    memset(flash_info, 0, sizeof(*flash_info));
    buf = atiny_malloc(MAX_FLASH_INFO);
    if (buf == NULL)
    {
        ATINY_LOG(LOG_ERR, "atiny_malloc fail len %d", MAX_FLASH_INFO);
        return ATINY_MALLOC_FAILED;
    }

    ret = g_cmd_ioctl(MQTT_READ_SECRET_INFO, buf, MAX_FLASH_INFO);
    if (ret != ATINY_OK)
    {
        ATINY_LOG(LOG_FATAL, "g_cmd_ioctl MQTT_READ_SECRET_INFO fail ret %ld", ret);
         goto EXIT;
    }

    magic_num = *((uint32_t*)buf);
    if (magic_num != FLASH_MAGIC_NUM)
    {
        ATINY_LOG(LOG_INFO, "mqtt flash info not valid");
         goto EXIT;
    }


    len = *((uint32_t*)&buf[TOTAL_LEN_POS]);
    if ((len > MAX_FLASH_INFO) || (len <= DATA_POS))
    {
        ATINY_LOG(LOG_ERR, "mqtt flash info len %d invalid", len);
         goto EXIT;
    }

    ret = flash_manager_parse((char *)(buf + DATA_POS), len - DATA_POS, flash_info);

EXIT:
    atiny_free(buf);
    return ret;
}

int flash_manager_write(const flash_info_s *flash_info)
{
    uint32_t len;
    int ret;
    uint8_t *buf;

    ret = flash_manager_get_need_len(flash_info, &len);
    if (ret != ATINY_OK)
    {
        return ret;
    }

    buf = atiny_malloc(MAX_FLASH_INFO);
    if (buf == NULL)
    {
        ATINY_LOG(LOG_ERR, "atiny_malloc fail len %d", len);
        return ATINY_MALLOC_FAILED;
    }

    flash_manager_write_buffer(flash_info, (char *)buf, len);
    ret = g_cmd_ioctl(MQTT_SAVE_SECRET_INFO, buf, MAX_FLASH_INFO);
    atiny_free(buf);
    if (ret != ATINY_OK)
    {
        ATINY_LOG(LOG_FATAL, "g_cmd_ioctl MQTT_SAVE_SECRET_INFO fail ret %d, len ld", ret, len);

    }
    return ret;
}
void flash_manager_destroy_flash_info(flash_info_s *flash_info)
{
   uint32_t i;

   for (i = 0; i < MAX_DATA_ITEM; i++)
   {
       TRY_FREE_MEM(flash_info->items[i]);
   }
}



