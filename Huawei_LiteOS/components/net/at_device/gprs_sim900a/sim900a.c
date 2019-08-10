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

#if defined(WITH_AT_FRAMEWORK)
#include "sim900a.h"

extern at_task at;
at_adaptor_api sim900a_interface;
char prefix_name[15];

int32_t sim900a_echo_off(void)
{
    return at.cmd((int8_t *)AT_CMD_ECHO_OFF, strlen(AT_CMD_ECHO_OFF), "OK\r\n", NULL,NULL);
}
int32_t sim900a_echo_on(void)
{
    return at.cmd((int8_t *)AT_CMD_ECHO_ON, strlen(AT_CMD_ECHO_OFF), "OK\r\n", NULL,NULL);
}
int32_t sim900a_reset(void)
{
    int32_t ret = 0;
    //at.cmd((int8_t*)AT_CMD_CLOSE,strlen(AT_CMD_CLOSE),"CLOSE OK","ERROR");
    ret = at.cmd((int8_t *)AT_CMD_SHUT, strlen(AT_CMD_SHUT), "SHUT OK", NULL,NULL);
    return ret;
}

int32_t sim900a_set_mux_mode(int32_t m)
{
    char cmd[64] = {0};
    snprintf(cmd, 64, "%s=%d", AT_CMD_MUX, (int)m);
    return at.cmd((int8_t *)cmd, strlen(cmd), "OK", NULL,NULL);
}

int32_t sim900a_connect(const int8_t *host, const int8_t *port, int32_t proto)
{
    int32_t ret = AT_FAILED;
    int32_t id = at.get_id();
    sim900a_reset();
    char cmd1[64] = {0};
    snprintf(cmd1, 64, "%s=\"B\"", AT_CMD_CLASS);
    at.cmd((int8_t *)cmd1, strlen(cmd1), "OK", NULL,NULL);
    char cmd2[64] = {0};
    snprintf(cmd2, 64, "%s=1,\"IP\",\"CMNET\"", AT_CMD_PDP_CONT);
    at.cmd((int8_t *)cmd2, strlen(cmd2), "OK", NULL,NULL);
    char cmd3[64] = {0};
    snprintf(cmd3, 64, "%s=1", AT_CMD_PDP_ATT);
    at.cmd((int8_t *)cmd3, strlen(cmd3), "OK", NULL,NULL);
    char cmd4[64] = {0};
    snprintf(cmd4, 64, "%s=1", AT_CMD_CIPHEAD);
    at.cmd((int8_t *)cmd4, strlen(cmd4), "OK", NULL,NULL);
    char cmd5[64] = {0};

    AT_LOG_DEBUG("host:%s, port:%s", host, port);

    if (AT_MUXMODE_SINGLE == at.mux_mode)
    {
        snprintf(cmd5, 64, "%s=\"%s\",\"%s\",\"%s\"", AT_CMD_CONN, proto == ATINY_PROTO_UDP ? "UDP" : "TCP", host, port);
    }
    else
    {
        at.cmd((int8_t *)(AT_CMD_PDP_ACT"=1,1"), strlen(AT_CMD_PDP_ACT"=1,1"), "OK", NULL,NULL);
        at.cmd((int8_t *)AT_CMD_CSTT, strlen(AT_CMD_CSTT), "OK", NULL,NULL);
        at.cmd((int8_t *)AT_CMD_CIICR, strlen(AT_CMD_CIICR), "OK", NULL,NULL);
        at.cmd((int8_t *)AT_CMD_CIFSR, strlen(AT_CMD_CIFSR), "", NULL,NULL);
        snprintf(cmd5, 64, "%s=%ld,\"%s\",\"%s\",\"%s\"", AT_CMD_CONN, id, proto == ATINY_PROTO_UDP ? "UDP" : "TCP", host, port);
    }
    if (id < 0 || id >= AT_MAX_LINK_NUM)
    {
        AT_LOG("no vailed linkid for use(id = %ld)", id);
        return -1;
    }
    ret = LOS_QueueCreate("dataQueue", 16, &at.linkid[id].qid, 0, sizeof(QUEUE_BUFF));
    if (ret != LOS_OK)
    {
        AT_LOG("init dataQueue failed!");
        at.linkid[id].usable = AT_LINK_UNUSE;
        return  -1;
    }
    at.cmd((int8_t *)cmd5, strlen(cmd5), "CONNECT OK", NULL,NULL);
    return id;
}

int32_t  sim900a_recv_timeout(int32_t id, uint8_t *buf, uint32_t len, char* ipaddr,int* port, int32_t timeout)
{
    uint32_t qlen = sizeof(QUEUE_BUFF);
    uint32_t rxlen = 0;

    (void)ipaddr; //gprs not need remote ip
    (void)port;   //gprs not need remote port

    QUEUE_BUFF  qbuf = {0, NULL};
    AT_LOG("****at.linkid[id].qid=%d***\n", at.linkid[id].qid);
    int ret = LOS_QueueReadCopy(at.linkid[id].qid, (void *)&qbuf, (UINT32 *)&qlen, timeout);
    AT_LOG("ret = %x, len = %ld, id = %ld", ret, qbuf.len, id);
    if (ret != LOS_OK)
    {
        return AT_FAILED;
    }

    if (qbuf.len)
    {
        rxlen = (len < qbuf.len) ? len : qbuf.len;
        memcpy(buf, qbuf.addr, rxlen);
        at_free(qbuf.addr);
    }
    return rxlen;
}

int32_t  sim900a_recv(int32_t id, uint8_t *buf, uint32_t len)
{
    return sim900a_recv_timeout(id, buf, len, NULL,NULL,LOS_WAIT_FOREVER);
}

int32_t sim900a_send(int32_t id , const uint8_t  *buf, uint32_t len)
{
    int32_t ret = -1;
    char cmd[64] = {0};
    if (AT_MUXMODE_SINGLE == at.mux_mode)
    {
        snprintf(cmd, 64, "%s=%ld", AT_CMD_SEND, len);
    }
    else
    {
        snprintf(cmd, 64, "%s=%ld,%ld", AT_CMD_SEND, id, len);
    }

    ret = at.write((int8_t *)cmd, (int8_t *)"SEND OK", (int8_t *)buf, len);

    return ret;
}

void sim900a_check(void)
{
    //check module response
    while(AT_FAILED == at.cmd((int8_t *)AT_CMD_AT, strlen(AT_CMD_AT), "OK", NULL,NULL))
    {
        printf("\r\ncheck module response unnormal\r\n");
        printf("\r\nplease check the module pin connection and the power switch\r\n");
        SIM900A_DELAY(500);
    }
    if(AT_FAILED != at.cmd((int8_t *)AT_CMD_CPIN, strlen(AT_CMD_CPIN), "OK", NULL,NULL))
    {
        printf("detected sim card\n");
    }
    if(AT_FAILED != at.cmd((int8_t *)AT_CMD_COPS, strlen(AT_CMD_COPS), "CHINA MOBILE", NULL,NULL))
    {
        printf("registerd to the network\n");
    }
}

int32_t sim900a_recv_cb(int32_t id)
{
    return AT_FAILED;
}

int32_t sim900a_close(int32_t id)
{
    char cmd[64] = {0};
    if (AT_MUXMODE_SINGLE == at.mux_mode)
    {
        snprintf(cmd, 64, "%s", AT_CMD_CLOSE);
    }
    else
    {
        uint32_t qlen = sizeof(QUEUE_BUFF);
        QUEUE_BUFF  qbuf = {0, NULL};
        while(LOS_OK == LOS_QueueReadCopy(at.linkid[id].qid, (void *)&qbuf, (UINT32 *)&qlen, 10))
        {
            if (qbuf.len)
            {
                at_free(qbuf.addr);
                memset(&qbuf, 0, sizeof(QUEUE_BUFF)); // don't use qlen
            }
        }
        (void)LOS_QueueDelete(at.linkid[id].qid);
        at.linkid[id].usable = 0;
        snprintf(cmd, 64, "%s=%ld", AT_CMD_CLOSE, id);
    }
    return at.cmd((int8_t *)cmd, strlen(cmd), "OK", NULL,NULL);
}
int32_t sim900a_data_handler(void *arg, int8_t *buf, int32_t len)
{
    if (NULL == buf || len <= 0)
    {
        AT_LOG("param invailed!");
        return -1;
    }
    AT_LOG("entry!");

    //process data frame ,like +IPD,linkid,len:data
    int32_t ret = 0;
    int32_t linkid = 0, data_len = 0;
    char *p1, *p2;
    QUEUE_BUFF qbuf;
    p1 = (char *)buf;

    if (0 == memcmp(p1, prefix_name, strlen(prefix_name)))
    {
        p2 = strstr(p1, ",");
        if (NULL == p2)
        {
            AT_LOG("got data prefix invailed!");
            goto END;
        }

        if (AT_MUXMODE_MULTI == at.mux_mode)
        {
            linkid = 0;
            for (p2++; *p2 <= '9' && *p2 >= '0'; p2++)
            {
                linkid = linkid * 10 + (*p2 - '0');
            }
        }

        for (p2++; *p2 <= '9' && *p2 >= '0' ; p2++)
        {
            data_len = (data_len * 10 + (*p2 - '0'));
        }
        p2++; //over ':'

        if (data_len > len)
        {
            AT_LOG("error !! receive data not complete data_len:%ld len:%ld",data_len,len);
            goto END;
        }

        qbuf.addr = at_malloc(data_len);
        if (NULL == qbuf.addr)
        {
            AT_LOG("malloc for qbuf failed!");
            goto END;
        }

        qbuf.len = data_len;
        if(AT_MUXMODE_MULTI == at.mux_mode)
        {
            p2++;
            p2++;//multi-connect prefix is +RECEIVE,0,13:\r\n+packet content
        }
        memcpy(qbuf.addr, p2, data_len);

        if (LOS_OK != (ret = LOS_QueueWriteCopy(at.linkid[linkid].qid, &qbuf, sizeof(QUEUE_BUFF), 0)))
        {
            AT_LOG("LOS_QueueWriteCopy  failed! ret = %lx", ret);
            at_free(qbuf.addr);
            goto END;
        }
        ret = (p2 + data_len - (char *)buf);
    }
END:
    return ret;
}

int32_t sim900a_cmd_match(const char *buf, char* featurestr,int len)
{
    return memcmp(buf,featurestr,len);
}

int32_t sim900a_ini()
{
    at_config at_user_conf =
    {
        .name = AT_MODU_NAME,
        .usart_port = AT_USART_PORT,
        .buardrate = AT_BUARDRATE,
        .linkid_num = AT_MAX_LINK_NUM,
        .user_buf_len = MAX_AT_USERDATA_LEN,
        .cmd_begin = AT_CMD_BEGIN,
        .line_end = AT_LINE_END,
        .mux_mode = 1, //support multi connection mode
        .timeout = AT_CMD_TIMEOUT,   //  ms
    };
    at.init(&at_user_conf);
    //single and multi connect prefix is different
    if (AT_MUXMODE_MULTI == at.mux_mode)
    {
        memcpy(prefix_name, AT_DATAF_PREFIX_MULTI, sizeof(AT_DATAF_PREFIX_MULTI));
    }
    else
    {
        memcpy(prefix_name, AT_DATAF_PREFIX, sizeof(AT_DATAF_PREFIX));
    }
    at.oob_register((char *)prefix_name, strlen((char *)prefix_name), sim900a_data_handler,sim900a_cmd_match);
    sim900a_echo_off();
    sim900a_check();
    sim900a_reset();
    sim900a_set_mux_mode(at.mux_mode);
    at.cmd((int8_t *)("AT+CIPMUX?"), strlen("AT+CIPMUX?"), "OK", NULL,NULL);
    return AT_OK;
}

int32_t sim900a_deinit(void)
{
    int id = 0;

    if(NULL != at.linkid)
    {
        for(id = 0; id < AT_MAX_LINK_NUM; id++)
        {
            if(AT_LINK_INUSE == at.linkid[id].usable)
            {
                if(AT_OK != sim900a_close(id))
                {
                    AT_LOG("sim900a_close(%d) failed", id);
                }
            }
        }
    }

    at.deinit();
    return AT_OK;
}



at_adaptor_api sim900a_interface =
{
    .init = sim900a_ini,
    .connect = sim900a_connect, /*TCP or UDP connect*/
    .send = sim900a_send, /*send data, if no response, retrun error*/
    .recv_timeout = sim900a_recv_timeout,
    .recv = sim900a_recv,
    .close = sim900a_close,/*close connect*/
    .recv_cb = sim900a_recv_cb,/*receive event handle, no available by now */
    .deinit = sim900a_deinit,
};

#endif //#if NETWORK_TYPE == SIM_900A
