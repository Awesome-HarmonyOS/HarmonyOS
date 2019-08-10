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
#if defined(WITH_AT_FRAMEWORK)
#include "at_device/bc95.h"
#include "at_hal.h"

//#include "bc95_test.h"




extern at_task at;
at_adaptor_api bc95_interface;
extern char rbuf[AT_DATA_LEN];
extern char wbuf[AT_DATA_LEN];


typedef struct
{
    uint32_t data_len;
    int link_idx;
    bool valid_flag;
}nb_data_ind_info_s;

char tmpbuf[AT_DATA_LEN]={0}; //transform to hex

socket_info sockinfo[MAX_SOCK_NUM];
static nb_data_ind_info_s g_data_ind_info;

#if defined ( __CC_ARM ) || defined ( __ICCARM__ )
static char *strnstr(const char *s1, const char *s2, size_t len)
{
    size_t l2;

    l2 = strlen(s2);
    if (!l2)
        return (char *)s1;
    while (len >= l2) {
        len--;
        if (!memcmp(s1, s2, l2))
            return (char *)s1;
        s1++;
    }
    return NULL;
}
#endif

static int nb_alloc_sock(int socket)
{
    int idx;

    for (uint32_t i = 0; i < MAX_SOCK_NUM; ++i)
    {
        if (sockinfo[i].used_flag  && (sockinfo[i].socket == socket))
        {
            return i;
        }
    }

    idx  = (socket % MAX_SOCK_NUM);
    if (!sockinfo[idx].used_flag)
    {
        return idx;
    }

    for (uint32_t i = 0; i < MAX_SOCK_NUM; ++i)
    {
        if (!sockinfo[i].used_flag)
        {
            return i;
        }
    }
    AT_LOG("save socket fail %d", socket);
    return MAX_SOCK_NUM;
}

static int nb_sock_to_idx(int socket)
{
    int idx;

    idx  = (socket % MAX_SOCK_NUM);

    if (sockinfo[idx].used_flag && (socket == sockinfo[idx].socket))
    {
        return idx;
    }

    for (uint32_t i = 0; i < MAX_SOCK_NUM; ++i)
    {
        if (sockinfo[i].used_flag && (socket == sockinfo[i].socket))
        {
            return i;
        }
    }

    return MAX_SOCK_NUM;
}

int str_to_hex(const char *bufin, int len, char *bufout)
{
    int i = 0;
    if (NULL == bufin || len <= 0 || NULL == bufout)
    {
        return -1;
    }
    for(i = 0; i < len; i++)
    {
        sprintf(bufout+i*2, "%02X", bufin[i]);
    }
    return 0;
}

void HexStrToStr(const unsigned char *source, unsigned char *dest, int sourceLen)
{
    short i;
    unsigned char highByte, lowByte;
    for (i = 0; i < sourceLen; i += 2)
    {
        highByte = toupper(source[i]);
        lowByte  = toupper(source[i + 1]);
        if (highByte > 0x39)
            highByte -= 0x37;
        else
            highByte -= 0x30;
        if (lowByte > 0x39)
            lowByte -= 0x37;
        else
            lowByte -= 0x30;
        dest[i / 2] = (highByte << 4) | lowByte;
    }
    return ;
}

int32_t nb_reboot(void)
{
   // memset(sockinfo, 0, MAX_SOCK_NUM * sizeof(struct _socket_info_t));
    return at.cmd((int8_t*)AT_NB_reboot, strlen(AT_NB_reboot), "OK", NULL,NULL);
}

int32_t nb_hw_detect(void)//"AT+CFUN?\r"
{
    return at.cmd((int8_t*)AT_NB_hw_detect, strlen(AT_NB_hw_detect), "+CFUN:1", NULL,NULL);
}

int32_t nb_check_csq(void)
{
    char *cmd = "AT+CSQ\r";
    return at.cmd((int8_t*)cmd, strlen(cmd), "+CSQ:", NULL,NULL);
}

int32_t nb_set_cdpserver(char* host, char* port)
{
    char *cmd = "AT+NCDP=";
    char *cmd2 = "AT+NCDP?";
	char *cmdNNMI = "AT+NNMI=1\r";
    char *cmdCMEE = "AT+CMEE=1\r";
	//char *cmdCGP = "AT+CGPADDR";
	char tmpbuf[128] = {0};
	int ret = -1;
    char ipaddr[100] = {0};
    if(strlen(host) > 70 || strlen(port) > 20 || host==NULL || port == NULL)
    {
        ret = at.cmd((int8_t*)cmdNNMI, strlen(cmdNNMI), "OK", NULL,NULL);
        ret = at.cmd((int8_t*)cmdCMEE, strlen(cmdCMEE), "OK", NULL,NULL);
        return ret;
    }

    snprintf(ipaddr, sizeof(ipaddr) - 1, "%s,%s\r", host, port);
	snprintf(tmpbuf, sizeof(tmpbuf) - 1, "%s%s%c", cmd, ipaddr, '\r');

    ret = at.cmd((int8_t*)tmpbuf, strlen(tmpbuf), "OK", NULL,NULL);
	if(ret < 0)
	{
		return ret;
	}
	ret = at.cmd((int8_t*)cmd2, strlen(cmd2), ipaddr, NULL,NULL);
	//LOS_TaskDelay(1000);
	ret = at.cmd((int8_t*)cmdNNMI, strlen(cmdNNMI), "OK", NULL,NULL);
	//at.cmd((int8_t*)cmdCMEE, strlen(cmdCMEE), "OK", NULL, NULL);
    //ret = at.cmd((int8_t*)cmdCGP, strlen(cmdCGP), NULL, NULL);
	return ret;
}

int32_t nb_send_psk(char* pskid, char* psk)
{
    char* cmds = "AT+QSECSWT";//AT+QSECSWT=1,100    OK
    char* cmdp = "AT+QSETPSK";//AT+QSETPSK=86775942,E6F4C799   OK
    sprintf(wbuf, "%s=%d,%d\r", cmds, 1, 100);//min
    at.cmd((int8_t*)wbuf, strlen(wbuf), "OK", NULL,NULL);
    snprintf(wbuf, AT_DATA_LEN, "%s=%s,%s\r", cmdp, pskid, psk);
    return at.cmd((int8_t*)wbuf, strlen(wbuf), "OK", NULL,NULL);
}

int32_t nb_set_no_encrypt(void)
{
    char* cmd = "AT+QSECSWT=0\r";
    return at.cmd((int8_t*)cmd, strlen(cmd), "OK", NULL,NULL);
}

#ifdef WITH_SOTA
int sota_cmd(int8_t *cmd, int32_t len, const char *suffix, char *resp_buf, int* resp_len)
{
    AT_LOG("sota_cmd:%s", cmd);
    LOS_MuxPend(at.cmd_mux, LOS_WAIT_FOREVER);
    at_transmit((uint8_t *)cmd, len, 1);
    LOS_MuxPost(at.cmd_mux);

    return AT_OK;

}

int nb_send_str(const char* buf, int len)
{
    char *cmd1 = "AT+NMGS=";
    memset(wbuf, 0, AT_DATA_LEN);
    memset(rbuf, 0, AT_DATA_LEN);
    snprintf(wbuf, AT_DATA_LEN, "%s%d,%s%c",cmd1,(int)len/2,buf,'\r');
    return sota_cmd((int8_t*)wbuf, strlen(wbuf), "OK", NULL,NULL);

}
#endif
int32_t nb_send_payload(const char* buf, int len)
{
    char *cmd1 = "AT+NMGS=";
    char *cmd2 = "AT+NQMGS\r";
    int ret;
    char* str = NULL;
    int curcnt = 0;
    int rbuflen;
    static int sndcnt = 0;
    if(buf == NULL || len > AT_MAX_PAYLOADLEN)
    {
        AT_LOG("payload too long");
        return -1;
    }
    memset(tmpbuf, 0, AT_DATA_LEN);
    memset(wbuf, 0, AT_DATA_LEN);
    str_to_hex(buf, len, tmpbuf);
    memset(rbuf, 0, AT_DATA_LEN);
    snprintf(wbuf, AT_DATA_LEN,"%s%d,%s%c",cmd1,(int)len,tmpbuf,'\r');
    ret = at.cmd((int8_t*)wbuf, strlen(wbuf), "OK", NULL,NULL);
    if(ret < 0)
        return -1;
    ret = at.cmd((int8_t*)cmd2, strlen(cmd2), "SENT=", rbuf,&rbuflen);
    if(ret < 0)
        return -1;
    str = strstr(rbuf,"SENT=");
    if(str == NULL)
        return -1;
    sscanf(str,"SENT=%d,%s",&curcnt,wbuf);
    if(curcnt == sndcnt)
        return -1;
    sndcnt = curcnt;
    return ret;
}

int nb_query_ip(void)
{
	char *cmd = "AT+CGPADDR\r";
    return at.cmd((int8_t*)cmd, strlen(cmd), "+CGPADDR:0,", NULL,NULL);
}

int32_t nb_get_netstat(void)
{
	char *cmd = "AT+CGATT?\r";
    return at.cmd((int8_t*)cmd, strlen(cmd), "CGATT:1", NULL,NULL);
}

static int32_t nb_cmd_with_2_suffix(const int8_t *cmd, int  len,
                        const char* suffix_ok, const char* suffix_err,  char *resp_buf, uint32_t* resp_len)
{

    const char *suffix[2] = {0};
    at_cmd_info_s cmd_info = {0};

    suffix[0] = suffix_ok;
    suffix[1] = suffix_err;

    cmd_info.suffix = suffix;
    cmd_info.suffix_num = array_size(suffix);

    cmd_info.resp_buf = resp_buf;
    cmd_info.resp_len = resp_len;

	if (at.cmd_multi_suffix(cmd, len, &cmd_info) != AT_OK)
    {
        return AT_FAILED;
    }

    if (cmd_info.match_idx != 0)
    {
        AT_LOG("cmd_info.match_idx %d", cmd_info.match_idx);
        return AT_FAILED;
    }

    return AT_OK;
}


int32_t nb_create_sock(int port,int proto)
{
	int socket;
    int rbuflen = AT_DATA_LEN;
	const char *cmdudp = "AT+NSOCR=DGRAM,17,";//udp
	const char *cmdtcp = "AT+NSOCR=STREAM,6,";//tcp
	int ret;
    char buf[64];
    int cmd_len;

	if(proto!=17 && proto!=6)
    {
        AT_LOG("proto invalid!");
        return -1;
    }
    memset(rbuf, 0, AT_DATA_LEN);

    if (proto == 17)
    {
        cmd_len = snprintf(buf, sizeof(buf), "%s%d,1\r", cmdudp, port);//udp
    }
    else
    {
        cmd_len = snprintf(buf, sizeof(buf), "%s%d,1\r", cmdtcp, port);
    }

	nb_cmd_with_2_suffix((int8_t*)buf, cmd_len, "OK", "ERROR", rbuf, (uint32_t *)&rbuflen);
	ret = sscanf(rbuf, "%d\r%s",&socket, tmpbuf);
    if ((2 == ret) && (socket >= 0)
        && (strnstr(tmpbuf, "OK", sizeof(tmpbuf))))
    {
        return socket;
    }

    AT_LOG("sscanf fail,ret=%d,socket=%d", ret, socket);
    return -1;
}

static bool nb_is_addr_valid(const char *addr)
{
    const int size = 4;
    int tmp[4];
    int ret;

    ret = sscanf(addr, "%d.%d.%d.%d", &tmp[0], &tmp[1], &tmp[2], &tmp[3]);
    return  (size == ret);
}

int nb_decompose_str(const char* str, int *readleft, int *out_sockid)
{
    const char *tmp,*trans;
    int sockid;
    QUEUE_BUFF qbuf;
    int ret = AT_FAILED;
    int rlen;
    int link_id;


    tmp = strstr(str,",");
    if(tmp == NULL)
    {
        return AT_FAILED;
    }

    sockid = chartoint(str);
    trans = strstr(tmp+1,",");
    if(trans == NULL)
    {
        return AT_FAILED;
    }
    strncpy(qbuf.ipaddr,tmp+1,MIN((trans-tmp),AT_DATA_LEN/2));
    qbuf.ipaddr[trans-tmp-1] = '\0';
    if (!nb_is_addr_valid(qbuf.ipaddr))
    {
        return AT_FAILED;
    }

    qbuf.port = chartoint((char*)(trans+1));
    tmp = strstr(trans+1,",");
    if(tmp == NULL)
    {
        return AT_FAILED;
    }
    rlen = chartoint((char*)(tmp+1));
    if(rlen >= AT_DATA_LEN/2 || rlen < 0)
    {
        AT_LOG("rlen %d", rlen);
        return AT_FAILED;
    }

    trans = strstr(tmp+1,",");
    if(trans == NULL)
    {
        return AT_FAILED;
    }

    tmp = strstr(trans+1,",");
    if (tmp == NULL)
    {
        return AT_FAILED;
    }

    *readleft = chartoint((char*)(tmp+1));

    *out_sockid = sockid;

    link_id = nb_sock_to_idx(sockid);
    if (link_id >= MAX_SOCK_NUM)
    {
        AT_LOG("sockid invalid %d", sockid);
        return AT_OK;
    }

    qbuf.addr = at_malloc(rlen);
    if (qbuf.addr == NULL)
    {
        AT_LOG("at_malloc null");
        return AT_OK;
    }

    HexStrToStr((const unsigned char*)(trans+1), qbuf.addr, (rlen)*2);
    qbuf.len = rlen;

    ret = LOS_QueueWriteCopy(at.linkid[link_id].qid, &qbuf, sizeof(qbuf), 0);
    if (LOS_OK != ret)
    {
        AT_LOG("LOS_QueueWriteCopy  failed! ret %d", ret);
        at_free(qbuf.addr);
    }

    //AT_LOG("wwww write data,qid %d, len %ld, ret %d", at.linkid[link_id].qid, qbuf.len, ret);


    return AT_OK;
}

static void nb_close_sock(int sock)
{
    const char *cmd = "AT+NSOCL=";
    char buf[64];
    int cmd_len;

	cmd_len = snprintf(buf, sizeof(buf), "%s%d\r", cmd, sock);
	nb_cmd_with_2_suffix((int8_t*)buf, cmd_len, "OK", "ERROR", NULL,NULL);
}


static int nb_create_sock_link(int portnum, int *link_id)
{
    int ret = 0;
    int sock;

    sock = nb_create_sock(portnum, UDP_PROTO);
	if(sock < 0)
	{
		AT_LOG("sock num exceeded,ret is %d", sock);
		return AT_FAILED;
	}

    ret = nb_alloc_sock(sock);
    if (ret >= MAX_SOCK_NUM)
	{

        AT_LOG("sock num exceeded,socket is %d", sock);
        goto CLOSE_SOCk;
    }

    if (LOS_QueueCreate("dataQueue", 16, &at.linkid[ret].qid, 0, sizeof(QUEUE_BUFF)) != LOS_OK)
    {
        AT_LOG("init dataQueue failed, ret is %d!",ret);
        goto CLOSE_SOCk;
    }

    *link_id = ret;
    sockinfo[ret].socket = sock;
    sockinfo[ret].used_flag = true;
    return AT_OK;

CLOSE_SOCk:
    nb_close_sock(sock);

    return AT_FAILED;

}

int32_t nb_bind(const int8_t * host, const int8_t *port, int32_t proto)
{
	int ret = 0;
	int portnum;

    (void)host;
    (void)proto;
	portnum = chartoint((char*)port);

    if (nb_create_sock_link(portnum, &ret) != AT_OK)
    {
        return AT_FAILED;
    }

    sockinfo[ret].localport = *(unsigned short*)portnum;

    return AT_OK;
}

int32_t nb_connect(const int8_t * host, const int8_t *port, int32_t proto)
{
	int ret = 0;
	static uint16_t localport = NB_STAT_LOCALPORT;
    const int COAP_SEVER_PORT = 5683;

    if (nb_create_sock_link(localport, &ret) != AT_OK)
    {
        return AT_FAILED;
    }

	localport++;
    if (localport == COAP_SEVER_PORT || localport == (COAP_SEVER_PORT + 1))
    {
        localport = 5685;
    }

	strncpy(sockinfo[ret].remoteip, (const char *)host, sizeof(sockinfo[ret].remoteip));
    sockinfo[ret].remoteip[sizeof(sockinfo[ret].remoteip) - 1] = '\0';
	sockinfo[ret].remoteport = chartoint((char*)port);

    AT_LOG("ret:%d remoteip:%s port:%d",ret,sockinfo[ret].remoteip,sockinfo[ret].remoteport);

    return ret;

}


int32_t nb_sendto(int32_t id , const uint8_t  *buf, uint32_t len, char* ipaddr, int port)
{
	char *cmd = "AT+NSOST=";
	int data_len = len/2;
    int cmd_len;

    if(buf == NULL || data_len > AT_MAX_PAYLOADLEN || id >= MAX_SOCK_NUM)
    {
        AT_LOG("invalid args");
        return -1;
    }

    AT_LOG("id:%d remoteip:%s port:%d",(int)sockinfo[id].socket, ipaddr, port);
	memset(wbuf, 0, AT_DATA_LEN);
	memset(tmpbuf, 0, AT_DATA_LEN);
	str_to_hex((const char *)buf, len, tmpbuf);

	cmd_len = snprintf(wbuf, AT_DATA_LEN, "%s%d,%s,%d,%d,%s\r",cmd,(int)sockinfo[id].socket,
        ipaddr, port, (int)len, tmpbuf);


	if (nb_cmd_with_2_suffix((int8_t*)wbuf, cmd_len, "OK", "ERROR",
                NULL, NULL) != AT_OK)
    {
        return AT_FAILED;
    }

    return len;
}


int32_t nb_send(int32_t id , const uint8_t *buf, uint32_t len)
{
	if (id >= MAX_SOCK_NUM)
    {
        AT_LOG("invalid args");
        return AT_FAILED;
    }
    return nb_sendto(id , buf, len, sockinfo[id].remoteip,(int)sockinfo[id].remoteport);
}

int32_t nb_recv(int32_t id , uint8_t  *buf, uint32_t len)
{
    return nb_recv_timeout(id, buf, len,NULL,NULL, LOS_WAIT_FOREVER);
}

int32_t nb_recvfrom(int32_t id , uint8_t  *buf, uint32_t len,char* ipaddr,int* port)
{
    return nb_recv_timeout(id, buf, len, ipaddr,port,LOS_WAIT_FOREVER);
}

int32_t nb_recv_timeout(int32_t id , uint8_t  *buf, uint32_t len,char* ipaddr,int* port, int32_t timeout)
{
    int rlen = 0;
    int ret;
    QUEUE_BUFF	qbuf;
    UINT32 qlen = sizeof(QUEUE_BUFF);

    if (id  >= MAX_SOCK_NUM)
    {
        AT_LOG("link id %ld invalid", id);
        return AT_FAILED;
    }


    ret = LOS_QueueReadCopy(at.linkid[id].qid, &qbuf, &qlen, timeout);
    //AT_LOG("wwww LOS_QueueReadCopy data,qid %d, len %ld, ret %d", at.linkid[id].qid, qbuf.len, ret);
    if (ret != LOS_OK)
    {
        return AT_TIMEOUT;
    }


    if (('\0' == sockinfo[id].remoteip[0])
        || (0 == sockinfo[id].remoteport))
    {
        AT_LOG("update ip and port for link %ld", id);
        strncpy(sockinfo[id].remoteip, qbuf.ipaddr, sizeof(sockinfo[id].remoteip));
        sockinfo[id].remoteip[sizeof(sockinfo[id].remoteip) - 1] = '\0';
        sockinfo[id].remoteport = qbuf.port;
    }

	if(ipaddr != NULL)
	{
	    memcpy(ipaddr,qbuf.ipaddr,strlen(qbuf.ipaddr));
        *port = qbuf.port;
	}

    rlen = MIN(qbuf.len, len);

    //AT_LOG("recv data, %d", rlen);

    if (rlen){
        memcpy(buf, qbuf.addr, rlen);
        at_free(qbuf.addr);
    }
    return rlen;

}


int32_t nb_close(int32_t id)
{
    int ret;

    if ((id  >= MAX_SOCK_NUM)
        || (!sockinfo[id].used_flag))
    {
        AT_LOG("link id %ld invalid", id);
        return AT_FAILED;
    }

    nb_close_sock(sockinfo[id].socket);

    do
    {
        QUEUE_BUFF	qbuf = {0};
        UINT32 qlen = sizeof(QUEUE_BUFF);
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
    (void)memset(&sockinfo[id], 0, sizeof(sockinfo[id]));

    return AT_OK;
}

int32_t nb_recv_cb(int32_t id)
{
    return AT_FAILED;
}

static int32_t nb_init(void)
{
    at_config at_user_conf = {
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
    
    at_set_config(&at_user_conf);
    memset(&sockinfo, 0, sizeof(sockinfo));
    memset(&g_data_ind_info, 0, sizeof(g_data_ind_info));
    at_reg_step_callback(&at, nb_step);

    return AT_OK;
}

int32_t nb_deinit(void)
{

    for (int i = 0; i < MAX_SOCK_NUM; ++i)
    {
        if (sockinfo[i].used_flag)
        {
            nb_close(i);
        }
    }
    return nb_reboot();
}

at_adaptor_api bc95_interface =
{
    .init = nb_init,

    .bind = nb_bind,

    .connect = nb_connect,
    .send = nb_send,
    .sendto = nb_sendto,

    .recv_timeout = nb_recv_timeout,
    .recv = nb_recv,
    .recvfrom = nb_recvfrom,

    .close = nb_close,
    .recv_cb = nb_recv_cb,

    .deinit = nb_deinit,
};

void nb_reattach(void)
{
    (void)nb_cmd_with_2_suffix((int8_t*)CGATT, strlen(CGATT), "OK", "ERROR", NULL, NULL);
     (void)nb_cmd_with_2_suffix((int8_t*)CGATT_DEATTACH, strlen(CGATT_DEATTACH), "OK", "ERROR", NULL, NULL);
     LOS_TaskDelay(1000);
     (void)nb_cmd_with_2_suffix((int8_t*)CGATT_ATTACH, strlen(CGATT_ATTACH), "OK", "ERROR", NULL, NULL);
}

static int nb_cmd_rcv_data(int sockid, int readleft);


static int32_t nb_handle_sock_data(const int8_t *data, uint32_t len)
{
    (void) len;
    char *curr = (char *)data;

    if (strstr((char *) data, "ERROR") != NULL)
    {
        return AT_OK;
    }

    do
    {

        int readleft;
        int sockid;

        char *next = strstr(curr, "\r\n");

        if (next == curr)
        {
            curr += 2;
        }

        if (next != NULL)
        {
            next += 2;
        }

        if (nb_decompose_str(curr, &readleft, &sockid) == AT_OK)
        {
        /*
            if (readleft != 0)
            {
                nb_cmd_rcv_data(sockid, readleft);
            }*/
            return AT_OK;
        }
        curr = next;
    }while(curr);

    return AT_FAILED;
}


static int nb_cmd_rcv_data(int sockid, int readleft)
{
    int cmdlen;
    char cmdbuf[40];
    const char* cmd = "AT+NSORF=";
    const uint32_t timeout = 10;

    cmdlen = snprintf(cmdbuf, sizeof(cmdbuf), "%s%d,%d\r", cmd, sockid, readleft);
    return at_cmd_in_callback((int8_t*)cmdbuf, cmdlen, nb_handle_sock_data, timeout);
}

static int32_t nb_handle_data_ind(const char *buf)
{
    int32_t sockid;
    int32_t data_len;
    const char *p1, *p2;
    int link_idx;

    p2 = strstr(buf, AT_DATAF_PREFIX);
    if (NULL == p2)
    {
        return AT_FAILED;
    }
    p2+=strlen(AT_DATAF_PREFIX);

    p1 = strstr(p2, ",");
    if (p1 == NULL)
    {
        return AT_FAILED;
    }
    sockid = chartoint(p2);
    data_len = chartoint(p1 + 1);
    link_idx = nb_sock_to_idx(sockid);
    if (link_idx >= MAX_SOCK_NUM)
    {
        AT_LOG("invalid sock id %ld", sockid);


        return AT_OK;
    }

    if (nb_cmd_rcv_data(sockid, data_len) != AT_OK)
    {
        g_data_ind_info.data_len = (uint32_t)data_len;
        g_data_ind_info.link_idx = link_idx;
        g_data_ind_info.valid_flag = true;
    }
    else
    {
        g_data_ind_info.valid_flag = false;
    }

    return AT_OK;

}


int32_t nb_cmd_match(const char *buf, char* featurestr,int len)
{
    if (buf == NULL)
    {
        return AT_FAILED;
    }

    nb_handle_data_ind(buf);

    return AT_FAILED;
}

void nb_step(void)
{
    if ((!g_data_ind_info.valid_flag)
        || (!sockinfo[g_data_ind_info.link_idx].used_flag))
    {
        return;
    }
    if (nb_cmd_rcv_data(sockinfo[g_data_ind_info.link_idx].socket, g_data_ind_info.data_len) == AT_OK)
    {
        g_data_ind_info.valid_flag = false;
    }
}

#endif
