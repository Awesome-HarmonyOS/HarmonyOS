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
#include <string.h>
#include <ctype.h>
#include "bg36.h"

extern at_task at;
#define MAX_BG36_SOCK_NUM 11

at_config at_user_conf = {
    .name = AT_MODU_NAME,
    .usart_port = AT_USART_PORT,
    .buardrate = AT_BUARDRATE,
    .linkid_num = MAX_BG36_SOCK_NUM,
    .user_buf_len = MAX_AT_USERDATA_LEN,
    .cmd_begin = AT_CMD_BEGIN,
    .line_end = AT_LINE_END,
    .mux_mode = 1, //support multi connection mode
    .timeout = BG36_TIMEOUT,   //  ms
};

emtc_socket_info sockinfo[MAX_BG36_SOCK_NUM];

int bg36_cmd(char *cmd, int32_t len, const char *suffix, char *resp_buf, int* resp_len)
{
    AT_LOG("bg36 cmd:%s len:%d",cmd, (int)len);
	return at.cmd((int8_t *)cmd, len, suffix, resp_buf, resp_len);
}

static int bg36_close_sock(int sockid)
{
    char *cmd = "AT+QICLOSE=";
    char buf[64];
    int cmd_len;

    cmd_len = snprintf(buf, sizeof(buf), "%s%d\r", cmd, sockid);
    return bg36_cmd(buf, cmd_len, "OK", NULL,NULL);
}

//Direct Push Mode
int32_t bg36_data_handler(void *arg, int8_t *buf, int32_t len)
{
    int32_t sockid = 0;
    int32_t data_len = 0;
    const char *p1 = NULL;
    const char *p2 = NULL;
    QUEUE_BUFF qbuf;
    int32_t ret = 0;
    int32_t offset = 0;

    while(offset < len)
    {
        p1 = strstr((char *)(buf+offset), "recv");
        if (p1 == NULL)
        {
            AT_LOG("buf done, offset:%ld len:%ld",offset, len);
            return AT_OK;
        }
        p1 += strlen("\"recv\"");
        sockid = chartoint(p1+1);

        if (sockid >= MAX_BG36_SOCK_NUM || sockinfo[sockid].used_flag == false)
        {
            AT_LOG("invalid sock id %ld", sockid);
            return AT_FAILED;
        }

        p2 = strstr((char *)(p1+1), ",");
        if (p2 == NULL)
        {
            AT_LOG("invalid data %ld", sockid);
            return AT_FAILED;
        }
        data_len = chartoint(p2+1);
        if (data_len > AT_DATA_LEN*2 || data_len <= 0)
        {
            AT_LOG("datalen too long:%ld", data_len);
            return AT_FAILED;
        }

        qbuf.addr = at_malloc(data_len);
        if (qbuf.addr == NULL)
        {
            AT_LOG("at_malloc null");
            return AT_OK;
        }

        p1 = strstr(p2, "\r\n");
        if (p1 != NULL)
        {
            memcpy(qbuf.addr, p1+2, data_len);
            qbuf.len = data_len;
            ret = LOS_QueueWriteCopy(at.linkid[sockid].qid, &qbuf, sizeof(qbuf), 0);
            if (LOS_OK != ret)
            {
                AT_LOG("LOS_QueueWriteCopy failed! ret %ld", ret);
                at_free(qbuf.addr);
            }
            offset+=data_len+strlen("+QIURC: \"recv\",");
        }
        else
        {
            AT_LOG("recv data null!");
            return AT_FAILED;
        }
    }

    return AT_OK;
}

int32_t bg36_cmd_match(const char *buf, char* featurestr,int len)
{
    return (strstr((char *)buf, featurestr) != NULL) ? 0: -1;
}

int32_t bg36_create_socket(const int8_t * host, const int8_t *port, int32_t proto, char* service_type)
{
    int rbuflen = 64;
    char inbuf[64] = {0};
    char tmpbuf[32] = {0};
    int conid = 0;
    int err = 0;
    char* str = NULL;
    int id = 0;
    int ret = 0;
    char cmd[64] = {0};

    AT_LOG("port:%s\r\n", port);

    if (at.mux_mode != AT_MUXMODE_MULTI)
    {
        AT_LOG("Only support in multi mode!\r\n");
        return AT_FAILED;
    }

    id = at.get_id();
    if (id >= MAX_BG36_SOCK_NUM)
    {
        AT_LOG("sock num exceeded,socket is %d", id);
        return AT_FAILED;
    }

    (void)snprintf(cmd, 64, "%s,%d,\"%s\",\"%s\",%s,0,1\r", QIOPEN_SOCKET, id, service_type, host, port);
    ret = bg36_cmd(cmd, strlen(cmd), "+QIOPEN:", inbuf, &rbuflen);
    str = strstr(inbuf, "+QIOPEN:");
    if (str == NULL || ret != AT_OK)
    {
        AT_LOG("QIOPEN no reply, sockid:%d", id);
        (void)bg36_close_sock((int)id);
        (void)LOS_TaskDelay(10000);
        ret = bg36_cmd(cmd, strlen(cmd), "+QIOPEN:", inbuf, &rbuflen);
        str = strstr(inbuf, "+QIOPEN:");
        if (str == NULL || ret != AT_OK)
        {
            at.linkid[id].usable = AT_LINK_UNUSE;
            return AT_FAILED;
        }
    }

    ret = sscanf(str,"+QIOPEN: %d,%d%s", &conid, &err, tmpbuf);
    if(ret == -1 || err != 0 || conid != id)
    {
        AT_LOG("Create socket %d failed. conid:%d, ret %d, err:%d", id, conid, ret, err);
        (void)bg36_close_sock(conid);
        at.linkid[id].usable = AT_LINK_UNUSE;
        return AT_FAILED;
    }

    if (LOS_QueueCreate("dataQueue", 16, &at.linkid[id].qid, 0, sizeof(QUEUE_BUFF)) != LOS_OK)
    {
        AT_LOG("init dataQueue failed!");
        (void)bg36_close_sock(conid);
        at.linkid[id].usable = AT_LINK_UNUSE;
        return AT_FAILED;
    }

    sockinfo[id].used_flag = true;
    AT_LOG("create socket %d success!",id);

    return id;
}

int32_t bg36_bind(const int8_t * host, const int8_t *port, int32_t proto)
{
    return bg36_create_socket(host, port, proto, "TCP");
}

int32_t bg36_connect(const int8_t * host, const int8_t *port, int32_t proto)
{
    char *cmd2 = "AT+QISTATE=1,";
    char cmd[64] = {0};
    int sockid;
    sockid = bg36_create_socket(host, port, proto, "TCP");
    if(sockid < 0 || sockid >= MAX_BG36_SOCK_NUM)
    {
        return AT_FAILED;
    }
    (void)snprintf(cmd, 64, "%s%d\r", cmd2, sockid);
    (void)bg36_cmd(cmd, strlen(cmd), "+QISTATE:", NULL, NULL);
    return sockid;
}

int32_t bg36_send(int32_t id , const uint8_t *buf, uint32_t len)
{
    char *cmd1 = "AT+QISEND=";
    char cmd[64] = {0};
    int ret;
	if (id < 0 || id >= MAX_BG36_SOCK_NUM || len >= MAX_SEND_DATA_LEN)
    {
        AT_LOG("invalid args sockid:%d len:%d",(int)id, (int)len);
        return AT_FAILED;
    }
    (void)snprintf(cmd, sizeof(cmd),"%s%d,%d%c",cmd1, (int)id, (int)len,'\r');
    ret = bg36_cmd(cmd, strlen(cmd), ">", NULL, NULL);
    if(ret)
    {
        AT_LOG("socket invalid,no >");
        return AT_FAILED;
    }
    ret = bg36_cmd((char *)buf, len, "OK", NULL, NULL);
    if(ret)
    {
        AT_LOG("data send failed");
        return AT_FAILED;
    }
    return len;
}

static int32_t bg36_recv_timeout(int32_t id , uint8_t  *buf, uint32_t len,char* ipaddr,int* port, int32_t timeout)
{
    int rlen =0;
    int copylen =0;
    int ret;
    QUEUE_BUFF  qbuf;
    UINT32 qlen = sizeof(QUEUE_BUFF);

    if (id  >= MAX_BG36_SOCK_NUM)
    {
        AT_LOG("link id %d invalid", (int)id);
        return AT_FAILED;
    }

    if (sockinfo[id].buf == NULL)
    {
        ret = LOS_QueueReadCopy(at.linkid[id].qid, &qbuf, &qlen, timeout);
        if (ret != LOS_OK)
        {
             return AT_TIMEOUT;
        }
        AT_LOG("Read queue len:%d",(int)qbuf.len);
        sockinfo[id].buf = (char*)qbuf.addr;
        sockinfo[id].len = qbuf.len;
        sockinfo[id].offset = 0;
    }

    if(sockinfo[id].len - sockinfo[id].offset > len)
    {
        memcpy(buf, sockinfo[id].buf + sockinfo[id].offset, len);
        sockinfo[id].offset += len;
        return len;
    }
    else
    {
        copylen = sockinfo[id].len - sockinfo[id].offset;
        memcpy(buf, sockinfo[id].buf + sockinfo[id].offset, copylen);
        at_free(sockinfo[id].buf);
        sockinfo[id].offset = 0;
        sockinfo[id].buf = NULL;
        sockinfo[id].len = 0;
        rlen = copylen;
        while(rlen < len)
        {
            ret = LOS_QueueReadCopy(at.linkid[id].qid, &qbuf, &qlen, 0);
            if (ret == LOS_OK)
            {
                sockinfo[id].buf = (char*)qbuf.addr;
                sockinfo[id].len = qbuf.len;
                sockinfo[id].offset = 0;
                if(len-rlen < qbuf.len)
                {
                    memcpy(buf+rlen, sockinfo[id].buf, len - rlen);
                    sockinfo[id].offset = len - rlen;
                    return len;
                }
                else
                {
                    memcpy(buf+rlen, sockinfo[id].buf, qbuf.len);
                    rlen += qbuf.len;
                    at_free(sockinfo[id].buf);
                    sockinfo[id].offset = 0;
                    sockinfo[id].buf = NULL;
                    sockinfo[id].len = 0;
                }
            }
            else
            {
                return rlen;
            }
        }

        return rlen;
    }
}

static int32_t bg36_recv(int32_t id , uint8_t  *buf, uint32_t len)
{
    return bg36_recv_timeout(id, buf, len, NULL, NULL, LOS_WAIT_FOREVER);/*lint !e569*/
}

static int32_t bg36_close(int32_t id)
{
    int ret;
    QUEUE_BUFF  qbuf = {0};
    char *cmd2 = "AT+QISTATE=1,";
    char cmd[64] = {0};

    UINT32 qlen = sizeof(QUEUE_BUFF);

    (void)snprintf(cmd, 64, "%s%d\r", cmd2, (int)id);
    (void)bg36_cmd(cmd, strlen(cmd), "+QISTATE:", NULL, NULL);
    ret = bg36_close_sock((int)id);
    if (ret != AT_OK)
    {
        AT_LOG("close no reply,continue. sockid:%d", (int)id);
        (void)LOS_TaskDelay(10000);
        (void)bg36_close_sock((int)id);
        (void)LOS_TaskDelay(10000);
        (void)bg36_cmd(cmd, strlen(cmd), "+QISTATE:", NULL, NULL);
    }

    sockinfo[id].used_flag = false;
    at.linkid[id].usable = false;

    if(sockinfo[id].buf != NULL)
    {
        at_free(sockinfo[id].buf);
    }

    do
    {
        qbuf.addr = NULL;
        ret = LOS_QueueReadCopy(at.linkid[id].qid, &qbuf, &qlen, 0);
        if (ret == LOS_OK && qbuf.addr != NULL)
        {
            at_free(qbuf.addr);
        }
    }while(ret == LOS_OK);

    ret = LOS_QueueDelete(at.linkid[id].qid);
    if (ret != LOS_OK)
    {
        AT_LOG("LOS_QueueDelete failed, ret is %d!,qid %d", ret, at.linkid[id].qid);
    }

    return ret;
}

static int32_t bg36_init(void)
{
    int rbuflen = 64;
    char inbuf[64] = {0};
    char tmpbuf[64] = {0};
    int creg = 0;
    int i = 0;
    int ret;

    at.init(&at_user_conf);
    memset(sockinfo, 0, sizeof(emtc_socket_info) * MAX_BG36_SOCK_NUM);/*lint !e545*/

    (void)at.oob_register(AT_DATAF_PREFIX, strlen(AT_DATAF_PREFIX), bg36_data_handler, bg36_cmd_match);
    (void)bg36_cmd(ATI, strlen(ATI), "OK", NULL, NULL);
    (void)bg36_cmd(ATE0, strlen(ATE0), "OK", NULL, NULL);
    (void)bg36_cmd(CPIN, strlen(CPIN), "+CPIN: READY", NULL, NULL);
    while(1)
    {
        (void)bg36_cmd(QUERYCFATT, strlen(QUERYCFATT), "+CGATT", inbuf,&rbuflen);
        if (strlen(inbuf)!=0)
        {
            ret = sscanf(inbuf,"\r\n+CGATT: %d\r\n%s",&creg,tmpbuf);
            if(ret == -1)
            {
                continue;
            }
            if (creg == 1)
            {
                break;
            }
        }
        (void)LOS_TaskDelay(100);
        memset(inbuf, 0, sizeof(inbuf));
    }

    for( i = 0; i < MAX_BG36_SOCK_NUM; i++)
    {
        (void)bg36_close((int32_t)i);/*lint !e534*/
    }

    return bg36_cmd(QIACTQUERY, strlen(QIACTQUERY), "OK", NULL, NULL);
}

at_adaptor_api emtc_bg36_interface =
{
    .init = bg36_init,
    .bind = bg36_bind,
    .connect = bg36_connect,
    .send = bg36_send,
    .sendto = NULL,
    .recv_timeout = bg36_recv_timeout,
    .recv = bg36_recv,
    .recvfrom = NULL,
    .close = bg36_close,
    .recv_cb = NULL,
    .deinit = NULL,
};
