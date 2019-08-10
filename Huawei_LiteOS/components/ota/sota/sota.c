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
#include "sota/sota.h"
#include "sota_hal.h"
#include "flag_manager.h"

#include<stdio.h>
#include<string.h>
#include<stdint.h>

#include "ota/package.h"
#include "upgrade_flag.h"

#define VER_LEN      16
#define SEND_BUF_LEN 128

typedef  uint8_t   BYTE;
typedef  uint16_t  WORD;

#ifdef SOTA_DEBUG
#define SOTA_LOG(fmt, ...) \
    do \
    { \
        if (NULL != g_flash_op.sota_printf) \
        { \
            (void)g_flash_op.sota_printf("[%s:%d][I]"fmt"\n", \
                                  __func__, __LINE__, ##__VA_ARGS__); \
        } \
    } while (0)
#else
#define SOTA_LOG(fmt, ...) ((void)0)
#endif

typedef enum
{
    MSG_GET_VER = 19,
    MSG_NOTIFY_NEW_VER,
    MSG_GET_BLOCK,
    MSG_DOWNLOAD_STATE,
    MSG_EXC_UPDATE,
    MSG_NOTIFY_STATE = 24
}msg_code_e;

typedef enum
{
    IDLE = 0,
    DOWNLOADING,
    UPDATING,
    UPDATED,
}sota_state;

typedef enum
{
    DEV_OK = 0x0,
    DEV_BUSYING = 0x1,
    DEV_WEAK_SIGNAL = 0x2,
    DEV_LATEST_VER = 0x3,
    DEV_LOW_BATTERY = 0x4,
    DEV_NO_SPACE = 0x5,
    DOWNLOAD_TIME_OUT = 0x6,
    FIRMWARE_CHECK_ERROR = 0x7,
    FIRMWARE_NOT_MATCH = 0x8,
    DEV_MEMORY_EXHAUSTED = 0x9,
    DEV_INNER_ERROR = 0x7f,
    UPDATE_TASK_EXIT = 0x80,
    REQUEST_BLOCK_INVALID = 0x81
}response_code_e;

typedef struct ota_pcp_head_t
{
    WORD ori_id;
    BYTE ver_num;
    BYTE msg_code;
    WORD chk_code;
    WORD data_len;
}ota_pcp_head_s;

typedef struct ota_ver_notify
{
    BYTE ver[VER_LEN];
    WORD block_size;
    WORD block_totalnum;
    WORD ver_chk_code;
}ota_ver_notify_t;

typedef struct sota_update_info
{
    uint16_t block_size;
    uint32_t block_num;
    uint32_t block_offset;
    uint32_t block_totalnum;
    uint32_t block_tolen;
    uint32_t ver_chk_code;
    char     ver[VER_LEN];
    uint8_t  state;
} sota_update_info_t;

sota_arg_s                         g_flash_op;
static sota_update_info_t          g_at_update_record;
static pack_storage_device_api_s * g_storage_device;

#define LITTLE_DNEIAN
#ifdef LITTLE_DNEIAN
#define htons_ota(x) ((((x) & 0x00ff) << 8) | (((x) & 0xff00) >> 8))
#else
#define htons_ota(x) (x)
#endif
#define PCP_HEAD 0xFFFE
#define BLOCK_HEAD 3

static int chartoint(const char* port)
{
	int tmp=0;
	while (*port >= '0' && *port <= '9')
	{
		tmp = tmp*10+*port-'0';
		port++;
	}
	return tmp;
}

int32_t sota_parse(const int8_t *in_buf, int32_t in_len, int8_t * out_buf,  int32_t out_len)
{
    ota_pcp_head_s *phead;
    char *databuf;
    char *rlen;
    int buflen;
    int ret,cmd_crc_num;
    char *buf;

    if (in_buf == NULL || in_len < (sizeof(ota_pcp_head_s) - sizeof(WORD)) || out_buf == NULL)
    {
        SOTA_LOG("in_buf:%p len:%d, out_buf:%p",in_buf,(int)in_len, out_buf);
        goto END;
    }

    rlen = strstr((const char*)in_buf,":");/*lint !e158*/
    if (rlen == NULL)/*lint !e158*/
    {
        SOTA_LOG("buflen invalid");
        goto END;
    }
    buflen = chartoint(rlen+1);
    if (out_len < buflen)
    {
        SOTA_LOG("out buf not enough");
        goto END; 
    }
    
    buflen = buflen * 2;
    databuf = strstr(rlen,",");
    if (databuf == NULL)
    {
        SOTA_LOG("buf invalid");
        goto END;
    }
    buf = databuf + 1;

    memset(out_buf, 0, out_len);
    HexStrToByte((const unsigned char *)buf, (unsigned char *)out_buf, buflen);
    phead = (ota_pcp_head_s *)out_buf;

    cmd_crc_num = htons_ota(phead->chk_code);
    phead->chk_code = 0;
    ret = crc_check((const unsigned char *)out_buf, buflen/2);
    phead->ori_id = htons_ota(phead->ori_id);
    if (phead->data_len > BLOCK_HEAD && phead->msg_code == MSG_GET_BLOCK)
    {
        phead->data_len = htons_ota(phead->data_len) - BLOCK_HEAD;
    }
    if (phead->ori_id != PCP_HEAD || (ret != cmd_crc_num) || \
            (phead->msg_code < MSG_GET_VER || phead->msg_code > MSG_NOTIFY_STATE))
    {
        SOTA_LOG("head wrong! head magic:%X msg_code:%X ver_num:%X ret:%X crc:%X",
            phead->ori_id,phead->msg_code,phead->ver_num, ret, cmd_crc_num);
        goto END;
    }

    return SOTA_OK;
END:
    return SOTA_FAILED;
}

static int ver_to_hex(const char *bufin, int len, char *bufout)
{
    int i = 0;
    if (NULL == bufin || len <= 0 || NULL == bufout)
    {
        return -1;
    }
    for (i = 0; i < len; i++)
    {
        sprintf(bufout + i * 2, "%02X", bufin[i]);
    }
    return 0;
}

static int sota_at_send(msg_code_e msg_code, char *buf, int len)
{
    uint32_t ret;
    char crcretbuf[5] = {0};
    char tmpbuf[SEND_BUF_LEN + VER_LEN] = {0};
    ota_pcp_head_s pcp_head = {0};
    unsigned char atwbuf[SEND_BUF_LEN + VER_LEN] = {0};
    unsigned char hbuf[64] = {0};
    if (len >= SEND_BUF_LEN)
    {
        SOTA_LOG("payload too long");
        return SOTA_FAILED;
    }
    pcp_head.ori_id = htons_ota(PCP_HEAD);
    pcp_head.ver_num = 1;
    pcp_head.msg_code = msg_code;
    pcp_head.data_len = htons_ota(len / 2);
    (void)ver_to_hex((const char *)&pcp_head, sizeof(ota_pcp_head_s), (char *)hbuf);

    memcpy(atwbuf, hbuf, VER_LEN);
    memcpy(atwbuf + VER_LEN, buf, len);

    HexStrToByte(atwbuf, (unsigned char*)tmpbuf, len + VER_LEN); //strlen(atwbuf)
    ret = (uint32_t)crc_check((unsigned char*)tmpbuf, (len + VER_LEN) / 2);
    (void)snprintf(crcretbuf, sizeof(crcretbuf), "%04X", (unsigned int)ret);

    memcpy(atwbuf + 8, crcretbuf, 4);
    return g_flash_op.sota_send((char *)atwbuf, len + VER_LEN);
}

static void sota_send_request_block(char *ver)
{
     char ver_ret[VER_LEN + 2] = {0};
     char sbuf[64] = {0};
 
     if (g_flash_op.firmware_download_stage == BOOTLOADER
        && g_flash_op.current_run_stage == APPLICATION)
     {
        return;
     }
        
     memcpy(ver_ret, ver, VER_LEN);
     ver_ret[VER_LEN] = (g_at_update_record.block_num >> 8) & 0XFF;
     ver_ret[VER_LEN + 1] = g_at_update_record.block_num & 0XFF;
     (void)ver_to_hex(ver_ret, (VER_LEN + 2), sbuf);
     (void)sota_at_send(MSG_GET_BLOCK, sbuf, (VER_LEN + 2) * 2);
}

static void sota_send_response_code(msg_code_e msg_code, response_code_e code)
{
    char ret_buf[1];
    char sbuf[2];
    
    ret_buf[0] = code;
    (void)ver_to_hex(ret_buf, 1, (char *)sbuf);
    (void)sota_at_send(msg_code, (char *)sbuf, 2);
}

static void sota_reset_record_info(ota_ver_notify_t *notify)
{
    g_at_update_record.block_offset = 0;
    g_at_update_record.block_size = htons_ota(notify->block_size);
    g_at_update_record.block_totalnum = htons_ota(notify->block_totalnum);
    g_at_update_record.block_num = 0;
    g_at_update_record.block_tolen = 0;
    g_at_update_record.ver_chk_code = notify->ver_chk_code;
    memcpy(g_at_update_record.ver, notify->ver, VER_LEN);
    g_at_update_record.state = DOWNLOADING;
    (void)flag_write(FLAG_APP, (void*)&g_at_update_record, sizeof(sota_update_info_t));  
}

static int32_t sota_new_ver_process(const ota_pcp_head_s *head, const uint8_t *pbuf)
{
    char ver[VER_LEN];
    ota_ver_notify_t *notify = (ota_ver_notify_t *)pbuf;
    
    (void)g_flash_op.get_ver(ver, VER_LEN);
    if (strncmp(ver, (const char*)notify->ver, VER_LEN) == 0)
    {
        SOTA_LOG("Already latest version %s", notify->ver);
        sota_send_response_code(MSG_NOTIFY_NEW_VER, DEV_LATEST_VER);
        g_at_update_record.state = IDLE;
        return SOTA_OK;
    }

    SOTA_LOG("Notify ver %s,%x, record ver:%s,%x", notify->ver, notify->ver_chk_code, 
        g_at_update_record.ver,g_at_update_record.ver_chk_code);
    if ((strncmp(g_at_update_record.ver, (const char *)notify->ver, VER_LEN) == 0)
        && (notify->ver_chk_code == g_at_update_record.ver_chk_code))
    {
        SOTA_LOG("state %d, downloaded %d blocks", g_at_update_record.state, g_at_update_record.block_num);
        if (g_at_update_record.block_num < g_at_update_record.block_totalnum
            && g_at_update_record.state == DOWNLOADING)
        {
            sota_send_request_block((char*)notify->ver);
            return SOTA_DOWNLOADING;
        }
        else if (g_at_update_record.block_num == g_at_update_record.block_totalnum
            && g_at_update_record.state == UPDATING)
        {
            sota_send_response_code(MSG_DOWNLOAD_STATE, DEV_OK);
            return SOTA_UPDATING;
        }
        else if (g_at_update_record.block_num == g_at_update_record.block_totalnum
            && g_at_update_record.state == UPDATED)
        {
            return SOTA_UPDATED;
        }
    }
    
    sota_reset_record_info(notify);
    sota_send_request_block((char*)notify->ver);
    return SOTA_DOWNLOADING;
}

static int32_t sota_data_block_process(const ota_pcp_head_s *head, const uint8_t *pbuf)
{
    uint16_t block_seq = 0;
    int ret = SOTA_OK;
    
    if (g_at_update_record.state != DOWNLOADING)
    {
       return SOTA_UNEXPECT_PACKET;
    }
    
    if (*pbuf == UPDATE_TASK_EXIT)
    {
        g_at_update_record.state = IDLE;
        return SOTA_EXIT;
    }

    block_seq = ((*(pbuf + 1) << 8) & 0XFF00) | (*(pbuf + 2) & 0XFF);
    if (g_at_update_record.block_num != block_seq)
    {
        SOTA_LOG("Download wrong,we need block %X, but rx %X:",(int)g_at_update_record.block_num, (int)block_seq);
        return SOTA_UNEXPECT_PACKET;
    }
    SOTA_LOG("off:%lx size:%x ",g_at_update_record.block_offset,head->data_len);
    ret = g_storage_device->write_software(g_storage_device, g_at_update_record.block_offset,(const uint8_t *)(pbuf + BLOCK_HEAD), head->data_len);
    if (ret != SOTA_OK)
    {
        SOTA_LOG("write software failed. ret:%d", ret);
        sota_send_response_code(MSG_DOWNLOAD_STATE, DEV_NO_SPACE);
        return SOTA_WRITE_FLASH_FAILED;
    }
    
    g_at_update_record.block_offset += g_at_update_record.block_size;
    g_at_update_record.block_tolen += head->data_len;
    g_at_update_record.block_num++;
    
    if ((g_at_update_record.block_num) < g_at_update_record.block_totalnum)
    {
    	SOTA_LOG("Rx total %d bytes downloading\r\n", g_at_update_record.block_tolen);
    	sota_send_request_block(g_at_update_record.ver);
        return SOTA_DOWNLOADING;
    } 
    else
    { 
        SOTA_LOG("Rx total %d bytes, UPDATING...\r\n", g_at_update_record.block_tolen);
        ret = g_storage_device->write_software_end(g_storage_device, PACK_DOWNLOAD_OK, g_at_update_record.block_tolen);
        if (ret != SOTA_OK)
        {
            SOTA_LOG("write software end ret:%d", ret);
            sota_send_response_code(MSG_DOWNLOAD_STATE, FIRMWARE_CHECK_ERROR);
            return SOTA_WRITE_FLASH_FAILED;
        }
        else
        {
            g_at_update_record.state = UPDATING;
            sota_send_response_code(MSG_DOWNLOAD_STATE, DEV_OK);
            return SOTA_UPDATING;
        }
    } 
}

static int32_t sota_update_exc_process(const ota_pcp_head_s *head, const uint8_t *pbuf)
{
    int ret = SOTA_OK;

    SOTA_LOG("Begin excute update");
    if (g_at_update_record.state != UPDATING)
    {
        return SOTA_UNEXPECT_PACKET;
    }
   
    ret = g_storage_device->active_software(g_storage_device);
    if (ret != SOTA_OK)
    {
        SOTA_LOG("Active software failed ret:%d.", ret);
        sota_send_response_code(MSG_EXC_UPDATE, DEV_INNER_ERROR);
        return SOTA_WRITE_FLASH_FAILED;
    }
    else
    {
        g_at_update_record.state = UPDATED;
       (void)flag_write(FLAG_APP, (void*)&g_at_update_record, sizeof(sota_update_info_t));
        sota_send_response_code(MSG_EXC_UPDATE, DEV_OK);
        return SOTA_UPDATED;
    }
}
int32_t sota_process(void *arg, const int8_t *buf, int32_t buflen)
{
    char sbuf[64] = {0};
    const uint8_t *pbuf = NULL;
    int ret = SOTA_OK;
    ota_pcp_head_s *phead;
    unsigned char  msg_code;

    phead =(ota_pcp_head_s *)buf;
    msg_code = phead->msg_code;

    if (phead->data_len > 0)
    {
        pbuf = (uint8_t *)buf + VER_LEN/2;
    }

    SOTA_LOG("process sota msg %d", msg_code);
    
    switch (msg_code)
    {
        case MSG_GET_VER:
        { 
            char ver_ret[VER_LEN + 1] = {0};
            (void)g_flash_op.get_ver(ver_ret+1, VER_LEN);
            (void)ver_to_hex(ver_ret, (VER_LEN + 1), (char *)sbuf);
            (void)sota_at_send(MSG_GET_VER, (char *)sbuf, (VER_LEN + 1) * 2);
            ret = SOTA_OK;
            break;
        }
        case MSG_NOTIFY_NEW_VER:
        {
            if (phead->data_len > sizeof(ota_ver_notify_t))
            {
               ret = sota_new_ver_process(phead, pbuf);
            }
            else
            {
                ret = SOTA_INVALID_PACKET;
            }
            break;
        }  
        case MSG_GET_BLOCK:
        {
            if (phead->data_len > 0)
            {
               ret = sota_data_block_process(phead, pbuf);
            }
            else
            {
                ret = SOTA_INVALID_PACKET;
            }
            break;
        }
        case MSG_EXC_UPDATE:
        {
             ret = sota_update_exc_process(phead, pbuf);
             break;
        }
        default:
        {
            SOTA_LOG("Rx invalid packet");
            ret = SOTA_INVALID_PACKET;
            break;
        }
    }
    return ret;
}

void sota_timeout_handler(void)
{
    if (g_at_update_record.state == DOWNLOADING)
    {
        SOTA_LOG("Download block %d over time", g_at_update_record.block_num);
        sota_send_response_code(MSG_EXC_UPDATE, DOWNLOAD_TIME_OUT);
        sota_send_request_block(g_at_update_record.ver);
    }
    else if (g_at_update_record.state == UPDATING)
    {
        SOTA_LOG("Download finish. excute over time");
        sota_send_response_code(MSG_EXC_UPDATE, DOWNLOAD_TIME_OUT);
        sota_send_response_code(MSG_DOWNLOAD_STATE, DEV_OK);
    }
}

static int sota_status_check(void)
{
    upgrade_state_e state;
    char sbuf[64] = {0};
    char tmpbuf[VER_LEN+1] = {0};

    memset(&g_at_update_record, 0, sizeof(sota_update_info_t));
    if (flag_read(FLAG_APP, (char*)&g_at_update_record, sizeof(sota_update_info_t)))
    {
        SOTA_LOG("flag read err");
        return SOTA_FAILED;
    }
    SOTA_LOG("state:%d flash ver:%s",g_at_update_record.state, g_at_update_record.ver);

    if (g_flash_op.firmware_download_stage == BOOTLOADER
        && g_flash_op.current_run_stage == BOOTLOADER)
    {
        if (g_at_update_record.state == DOWNLOADING)
        {
            sota_send_request_block(g_at_update_record.ver);
            return SOTA_DOWNLOADING;
        }
    }
    else
    {
	    (void)flag_upgrade_get_result(&state);
        SOTA_LOG("upgrade result: %d", state);
        if (state == OTA_SUCCEED)
        {
            SOTA_LOG("Update version %s success", g_at_update_record.ver);
            memcpy(tmpbuf + 1, g_at_update_record.ver, VER_LEN);
            (void)ver_to_hex(tmpbuf, VER_LEN+1, sbuf);
            (void)sota_at_send(MSG_NOTIFY_STATE, sbuf, (VER_LEN+1) * 2);
        }
    }

    memset(&g_at_update_record, 0, sizeof(sota_update_info_t));
    (void)flag_write(FLAG_APP, (const void*)&g_at_update_record, sizeof(sota_update_info_t));
    return SOTA_OK;
}

static int func_flag_read(void *buf, int32_t len)
{
    return g_flash_op.ota_info.read_flash(OTA_UPDATE_INFO,buf, len, 0);
}

static int func_flag_write(const void *buf, int32_t len)
{
    return g_flash_op.ota_info.write_flash(OTA_UPDATE_INFO,buf, len, 0);
}

int32_t sota_init(const sota_arg_s* sota_arg)
{
    int  ret;
    flag_op_s flag_op;
    pack_params_s pack_param;

    if (sota_arg == NULL || sota_arg->sota_malloc == NULL || sota_arg->sota_free == NULL)
    {
        return SOTA_FAILED;
    }

    memcpy(&pack_param.ota_opt, &sota_arg->ota_info, sizeof(pack_param.ota_opt));
    pack_param.malloc = sota_arg->sota_malloc;
    pack_param.free = sota_arg->sota_free;
    pack_param.printf = sota_arg->sota_printf;
    ret = pack_init_device(&pack_param);
    if (ret != SOTA_OK)
    {
        return SOTA_FAILED;
    }

    g_storage_device = pack_get_device();

    memcpy(&g_flash_op, sota_arg, sizeof(sota_arg_s));

    flag_op.func_flag_read = func_flag_read;
    flag_op.func_flag_write = func_flag_write;
    (void)flag_init(&flag_op);
    (void)flag_upgrade_init();

    return sota_status_check();
}

