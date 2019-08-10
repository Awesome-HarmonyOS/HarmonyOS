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

#include "los_memory.h"
#include "at_frame/at_main.h"
#include "at_hal.h"
#ifdef WITH_SOTA
#include "sota/sota.h"
#endif
#include "los_sys.ph"
#include "los_tick.ph"

static at_config at_user_conf;

/* FUNCTION */
void at_set_config(at_config *config);
at_config *at_get_config(void);

int32_t at_init(at_config *config);
//int32_t at_read(int32_t id, int8_t * buf, uint32_t len, int32_t timeout);
int32_t at_write(int8_t *cmd, int8_t *suffix, int8_t *buf, int32_t len);
int32_t at_get_unuse_linkid(void);
void at_listener_list_add(at_listener *p);
void at_listner_list_del(at_listener *p);
int32_t at_cmd(int8_t *cmd, int32_t len, const char *suffix, char *resp_buf, int* resp_len);
int32_t at_oob_register(char *featurestr, int cmdlen, oob_callback callback, oob_cmd_match cmd_match);

void at_deinit(void);
//init function for at struct

at_oob_t at_oob;
char rbuf[AT_DATA_LEN] = {0};
char wbuf[AT_DATA_LEN] = {0};

uint32_t at_get_time(void)
{
    return ((uint32_t)(LOS_TickCountGet()* (OS_SYS_MS_PER_SECOND / LOSCFG_BASE_CORE_TICK_PER_SECOND)) )/ 1000;
}


int chartoint(const char* port)
{
	int tmp=0;
	while(*port >= '0' && *port <= '9')
	{
		tmp = tmp*10+*port-'0';
		port++;
	}
	return tmp;
}

//add p to tail;
void at_listener_list_add(at_listener *p)
{

    at_listener *head = at.head;
    at_listener *cur;

    //AT_LOG("www add listener %p, callback %p", p, p->handle_data);

    p->next = NULL;
    if (NULL == head)
    {
        at.head = p;
        return;
    }

    cur = head;
    while(cur->next)
    {
        cur = cur->next;
    }

    cur->next = p;
 }

void at_listner_list_del(at_listener *p)
{
    at_listener *head = at.head;
    at_listener *cur;

    //AT_LOG("www del listener %p callback  %p", p, p->handle_data);
    if (p == head)
    {
        at.head = head->next;
        return;
    }


    cur = head;
    while(cur->next)
    {
        if (cur->next == p)
        {
            cur->next = p->next;
            break;
        }
    }
}

void at_listner_list_destroy(at_task *at_tsk)
{
    at_listener *head;
    while(at_tsk->head != NULL)
    {
        head = at_tsk->head;
        at_tsk->head = head->next;
        if (head->handle_data != NULL)
        {
            at_free(head);
        }
    }
}


static void at_rm_node(at_listener *listener, at_listener *pre)
{
    //AT_LOG("www del listener %p callback %p", listener, listener->handle_data);
    if (at.head == listener)
    {
        at.head = listener->next;
    }
    else
    {
        if (pre)
        {
            pre->next = listener->next;
        }
    }
    at_free(listener);

    LOS_MuxPost(at.trx_mux);
}

static void at_rm_timeout_nodes()
{
    at_listener *pre = NULL;
    at_listener *next = NULL;

    for (at_listener *listener = at.head; listener != NULL;  listener = next)
    {
        next = listener->next;

        if (listener->handle_data == NULL)
        {
            pre = listener;
            continue;
        }

        if (listener->expire_time <= at_get_time())
        {
            AT_LOG("get recv data timeout");
            at_rm_node(listener, pre);
        }
        else
        {
            pre = listener;
        }
    }

}


int32_t at_get_unuse_linkid(void)
{
    int i = 0;

    if (AT_MUXMODE_MULTI == at.mux_mode)
    {
        for (i = 0; i < at_user_conf.linkid_num; i++)
        {
            if (AT_LINK_UNUSE == at.linkid[i].usable)
            {
                break;
            }
        }
    }

    if (i < at_user_conf.linkid_num)
        at.linkid[i].usable = AT_LINK_INUSE;

    return i;
}


void store_resp_buf(int8_t *resp_buf, const int8_t *src, uint32_t src_len, uint32_t* maxlen)
{

    int copy_len;

    copy_len = MIN(*maxlen, src_len);
    memcpy((char *)resp_buf, (char *)src, copy_len);
    *maxlen = copy_len;
}

int32_t at_cmd_in_callback(const int8_t *cmd, int32_t len,
                    int32_t (*handle_data)(const int8_t *data, uint32_t len),  uint32_t timeout)
{
    int32_t ret = AT_FAILED;

    if (handle_data != NULL)
    {
        if (LOS_MuxPend(at.trx_mux, 0) != AT_OK)
        {
            return AT_FAILED;
        }
    }

    LOS_MuxPend(at.cmd_mux, LOS_WAIT_FOREVER);
    //at_rm_timeout_nodes();
    at_transmit((uint8_t *)cmd, len, 1);

    if (handle_data != NULL)
    {
        at_listener *listener = NULL;

        listener = at_malloc(sizeof(*listener));
        if (listener == NULL)
        {
            AT_LOG("at_malloc fail");
            goto EXIT;
        }
        memset(listener, 0 ,sizeof(*listener));

        listener->handle_data = handle_data;
        listener->expire_time = at_get_time() + timeout;
        at_listener_list_add(listener);
    }
    ret = AT_OK;


EXIT:
    LOS_MuxPost(at.cmd_mux);

    AT_LOG("len %ld,cmd %s", len, cmd);

    return ret;
}


int32_t at_cmd_multi_suffix(const int8_t *cmd, int  len, at_cmd_info_s *cmd_info)
{
    at_listener listener;
    int ret;
    int print_len;

    if ((cmd_info == NULL)
        || (cmd == NULL))
    {
        return AT_FAILED;
    }

    memset(&listener, 0, sizeof(listener));
    listener.cmd_info = *cmd_info;
    print_len = ((cmd_info->resp_buf && cmd_info->resp_len) ? (int)*(cmd_info->resp_len) : -1);
    AT_LOG("cmd len %d, %p,%d", print_len,cmd_info->resp_buf,(int)cmd_info->resp_len);

    LOS_MuxPend(at.trx_mux, LOS_WAIT_FOREVER);

    LOS_MuxPend(at.cmd_mux, LOS_WAIT_FOREVER);
    //at_rm_timeout_nodes();
    at_listener_list_add(&listener);
    at_transmit((uint8_t *)cmd, len, 1);
    LOS_MuxPost(at.cmd_mux);

    ret = LOS_SemPend(at.resp_sem, at.timeout);

    LOS_MuxPend(at.cmd_mux, LOS_WAIT_FOREVER);
    at_listner_list_del(&listener);
    LOS_MuxPost(at.cmd_mux);

    LOS_MuxPost(at.trx_mux);
    write_at_task_msg(AT_SENT_DONE);

    if (ret != LOS_OK)
    {
        AT_LOG("LOS_SemPend for listener.resp_sem failed(ret = %x, cmd = %s)!", ret,cmd);
        return AT_FAILED;
    }

    *cmd_info = listener.cmd_info;
    return AT_OK;

}

int32_t at_cmd(int8_t *cmd, int32_t len, const char *suffix, char *resp_buf, int* resp_len)
{
    const char *suffix_array[1] = {suffix};
    at_cmd_info_s cmd_info;

    memset(&cmd_info, 0, sizeof(cmd_info));
    cmd_info.suffix = suffix_array;
    cmd_info.suffix_num = array_size(suffix_array);
    cmd_info.resp_buf = resp_buf;
    cmd_info.resp_len = (uint32_t *)resp_len;

    return at_cmd_multi_suffix(cmd, len, &cmd_info);
}

int32_t at_write(int8_t *cmd, int8_t *suffix, int8_t *buf, int32_t len)
{
    const char *suffix_array[1];
    at_listener listener;
    int ret = AT_FAILED;

    memset(&listener, 0, sizeof(listener));
    listener.cmd_info.suffix_num = 1;
    suffix_array[0] = ">";
    listener.cmd_info.suffix = suffix_array;

    LOS_MuxPend(at.trx_mux, LOS_WAIT_FOREVER);

    LOS_MuxPend(at.cmd_mux, LOS_WAIT_FOREVER);
    //at_rm_timeout_nodes();
    at_listener_list_add(&listener);
    at_transmit((uint8_t *)cmd, strlen((char *)cmd), 1);
    LOS_MuxPost(at.cmd_mux);

    (void)LOS_SemPend(at.resp_sem, 200);

    LOS_MuxPend(at.cmd_mux, LOS_WAIT_FOREVER);
    suffix_array[0] = (char *)suffix;
    at_listner_list_del(&listener);
    at_listener_list_add(&listener);
    at_transmit((uint8_t *)buf, len, 0);
    LOS_MuxPost(at.cmd_mux);

    ret = LOS_SemPend(at.resp_sem, at.timeout);

    LOS_MuxPend(at.cmd_mux, LOS_WAIT_FOREVER);
    at_listner_list_del(&listener);
    LOS_MuxPost(at.cmd_mux);

    LOS_MuxPost(at.trx_mux);
    write_at_task_msg(AT_SENT_DONE);

    if (ret != LOS_OK)
    {
        AT_LOG("LOS_SemPend for listener.resp_sem failed(ret = %x)!", ret);
        return AT_FAILED;
    }
    return len;
}

int cloud_cmd_matching(int8_t *buf, int32_t len)
{
    int ret = 0;
    char *cmp = NULL;
    int i;
    //    int rlen;
    //    memset(wbuf, 0, AT_DATA_LEN);

    for(i = 0; i < at_oob.oob_num; i++)
    {
        //cmp = strstr((char *)buf, at_oob.oob[i].featurestr);
        ret = at_oob.oob[i].cmd_match((const char *)buf, at_oob.oob[i].featurestr,at_oob.oob[i].len);
        if(ret == 0)
        {
            cmp += at_oob.oob[i].len;
            //            sscanf(cmp,"%d,%s",&rlen,wbuf);
            if(at_oob.oob[i].callback != NULL)
            {
                (void)at_oob.oob[i].callback(at_oob.oob[i].arg, (int8_t *)buf, (int32_t)len);
            }
            return len;
        }
    }
    return 0;
}


static int at_handle_callback_cmd_resp(at_listener *listener, int8_t *resp_buf, uint32_t resp_len)
{

    if (listener->handle_data == NULL)
    {
        return AT_FAILED;
    }

    if (listener->handle_data(resp_buf, resp_len) == AT_OK)
    {
        at_rm_node(listener, NULL);
        return AT_OK;
    }

    return AT_FAILED;
}

static void at_handle_resp(int8_t *resp_buf, uint32_t resp_len)
{
    at_listener *listener = NULL;

    listener = at.head;
    if (NULL == listener)
        return;

    if (at_handle_callback_cmd_resp(listener, resp_buf, resp_len) == AT_OK)
    {
        return;
    }

    if(listener->cmd_info.suffix == NULL)
    {

        //store_resp_buf((int8_t *)listener->resp, (int8_t*)p1, p2 - p1);
        (void)LOS_SemPost(at.resp_sem);
        listener = NULL;
        return;
    }

    for (uint32_t i = 0;  i < listener->cmd_info.suffix_num; i++)
    {
        char *suffix;

        if (listener->cmd_info.suffix[i] == NULL)
        {
            continue;
        }

        suffix = strstr((char *)resp_buf, (const char *)listener->cmd_info.suffix[i]);
        if (suffix != NULL)
        {
            if ((NULL != listener->cmd_info.resp_buf) && (NULL != listener->cmd_info.resp_len) && (resp_len > 0))
            {
                store_resp_buf((int8_t *)listener->cmd_info.resp_buf, resp_buf, resp_len, listener->cmd_info.resp_len);//suffix + strlen((char *)listener->suffix) - p1
            }
            listener->cmd_info.match_idx = i;
            (void)LOS_SemPost(at.resp_sem);
            break;
        }

    }
}

static uint32_t at_get_queue_wait_time()
{
    uint32_t ret;

    if (at.step_callback == NULL)
    {
        return LOS_WAIT_FOREVER;
    }

    LOS_MuxPend(at.cmd_mux, LOS_WAIT_FOREVER);
    if ((at.head == NULL) || (at.head->handle_data == NULL))
    {
        ret = LOS_WAIT_FOREVER;
    }
    else
    {
        const uint32_t min_wait_time = 100;
        uint32_t  current = at_get_time();
        ret = ((current >= at.head->expire_time) ? min_wait_time :((at.head->expire_time - current) * 1000));
    }

    LOS_MuxPost(at.cmd_mux);
    return ret;
}

void at_recv_task()
{
    uint32_t recv_len = 0;
    uint8_t *tmp = at.userdata;  //[MAX_USART_BUF_LEN] = {0};
    int ret = 0;
    recv_buff recv_buf;
    UINT32 rlen = sizeof(recv_buff);

    while(1)
    {
        uint32_t wait_time = at_get_queue_wait_time();
        //AT_LOG("www wait time %ld", wait_time);
        ret = LOS_QueueReadCopy(at.rid, &recv_buf, &rlen, wait_time);
        if(ret != LOS_OK)
        {
            continue;
        }

        if (recv_buf.msg_type == AT_TASK_QUIT)
        {
            AT_LOG("at recv task quit");
            break;
        }

        LOS_MuxPend(at.cmd_mux, LOS_WAIT_FOREVER);
        at_rm_timeout_nodes();
		LOS_MuxPost(at.cmd_mux);

        if (at.step_callback)
        {
            at.step_callback();
        }

        if (recv_buf.msg_type != AT_USART_RX)
        {
            AT_LOG("at recv msg sent done");
            continue;
        }

        memset(tmp, 0, at_user_conf.user_buf_len);
        recv_len = read_resp(tmp, &recv_buf);

        if (recv_len <= 0)
        {
            AT_LOG("err, recv_len = %ld", recv_len);
            continue;
        }

        AT_LOG_DEBUG("recv len = %lu buf = %s ", recv_len, tmp);

        ret = cloud_cmd_matching((int8_t *)tmp, recv_len);
        if(ret > 0)
        {
            continue;
        }

        LOS_MuxPend(at.cmd_mux, LOS_WAIT_FOREVER);
        at_rm_timeout_nodes();
        at_handle_resp((int8_t *)tmp, recv_len);
        LOS_MuxPost(at.cmd_mux);

    }
}

uint32_t create_at_recv_task()
{
    uint32_t uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    task_init_param.usTaskPrio = 0;
    task_init_param.pcName = "at_recv_task";
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)at_recv_task;
    task_init_param.uwStackSize = 0x1000;

    uwRet = LOS_TaskCreate((UINT32 *)&at.tsk_hdl, &task_init_param);
    if(LOS_OK != uwRet)
    {
        return uwRet;
    }
    return uwRet;

}

void at_init_oob(void)
{
    at_oob.oob_num = 0;
    memset(at_oob.oob, 0, OOB_MAX_NUM * sizeof(struct oob_s));
}

int32_t at_struct_init(at_task *at)
{
    int ret = -1;
    if (NULL == at)
    {
        AT_LOG("invaild param!");
        return ret;
    }

    ret = LOS_QueueCreate("recvQueue", 32, (UINT32 *)&at->rid, 0, sizeof(recv_buff));
    if (ret != LOS_OK)
    {
        AT_LOG("init recvQueue failed!");
        return AT_FAILED;
    }
    at->rid_flag = true;


    ret = LOS_SemCreate(0, (UINT32 *)&at->recv_sem);
    if (ret != LOS_OK)
    {
        AT_LOG("init at_recv_sem failed!");
        goto at_recv_sem_failed;
    }

    ret = LOS_MuxCreate((UINT32 *)&at->cmd_mux);
    if (ret != LOS_OK)
    {
        AT_LOG("init cmd_mux failed!");
        goto at_cmd_mux_failed;
    }

    ret = LOS_MuxCreate((UINT32 *)&at->trx_mux);
    if (ret != LOS_OK)
    {
        AT_LOG("init cmd_mux failed!");
        goto at_cmd_mux_failed;
    }
    at->trx_mux_flag = true;

    ret = LOS_SemCreate(0, (UINT32 *)&at->resp_sem);
    if (ret != LOS_OK)
    {
        AT_LOG("init resp_sem failed!");
        goto at_resp_sem_failed;
    }
#ifndef USE_USARTRX_DMA
    at->recv_buf = at_malloc(at_user_conf.user_buf_len);
    if (NULL == at->recv_buf)
    {
        AT_LOG("malloc recv_buf failed!");
        goto malloc_recv_buf;
    }
#else
    at->recv_buf = at_malloc(at_user_conf.recv_buf_len);
    if (NULL == at->recv_buf)
    {
        AT_LOG("malloc recv_buf failed!");
        goto malloc_recv_buf;
    }
#endif

    at->cmdresp = at_malloc(at_user_conf.user_buf_len);
    if (NULL == at->cmdresp)
    {
        AT_LOG("malloc cmdresp failed!");
        goto malloc_resp_buf;
    }

    at->userdata = at_malloc(at_user_conf.user_buf_len);
    if (NULL == at->userdata)
    {
        AT_LOG("malloc userdata failed!");
        goto malloc_userdata_buf;
    }
    at->saveddata = at_malloc(at_user_conf.user_buf_len);
    if (NULL == at->saveddata)
    {
        AT_LOG("malloc saveddata failed!");
        goto malloc_saveddata_buf;
    }

    at->linkid = (at_link *)at_malloc(at_user_conf.linkid_num * sizeof(at_link));
    if (NULL == at->linkid)
    {
        AT_LOG("malloc for at linkid array failed!");
        goto malloc_linkid_failed;
    }
    memset(at->linkid, 0, at_user_conf.linkid_num * sizeof(at_link));

    at->head = NULL;
    at->mux_mode = at_user_conf.mux_mode;
    at->timeout = at_user_conf.timeout;
    return AT_OK;

    //        atiny_free(at->linkid);
malloc_linkid_failed:
    at_free(at->saveddata);
malloc_saveddata_buf:
    at_free(at->userdata);
malloc_userdata_buf:
    at_free(at->cmdresp);
malloc_resp_buf:
    at_free(at->recv_buf);
malloc_recv_buf:
    (void)LOS_SemDelete(at->resp_sem);
at_resp_sem_failed:
    (void)LOS_MuxDelete(at->cmd_mux);
at_cmd_mux_failed:
    (void)LOS_SemDelete(at->recv_sem);
at_recv_sem_failed:

    if (at->trx_mux_flag)
    {
        (void)LOS_MuxDelete(at->trx_mux);
        at->trx_mux_flag = false;
    }

    if (at->rid_flag)
    {
        (VOID)LOS_QueueDelete(at->rid);
        at->rid_flag = false;
    }
    return AT_FAILED;
}

int32_t at_struct_deinit(at_task *at)
{
    int32_t ret = AT_OK;

    if(at == NULL)
    {
        AT_LOG("invaild param!");
        return AT_FAILED;
    }

    LOS_MuxPend(at->cmd_mux, LOS_WAIT_FOREVER);
    at_listner_list_destroy(at);
    LOS_MuxPost(at->cmd_mux);

    if (LOS_SemDelete(at->recv_sem) != LOS_OK)
    {
        AT_LOG("delete at.recv_sem failed!");
        ret = AT_FAILED;
    }

    if (LOS_MuxDelete(at->cmd_mux) != LOS_OK)
    {
        AT_LOG("delete at.cmd_mux failed!");
        ret = AT_FAILED;
    }

    if (LOS_SemDelete(at->resp_sem) != LOS_OK)
    {
        AT_LOG("delete at.resp_sem failed!");
        ret = AT_FAILED;
    }

    if (at->trx_mux_flag)
    {
        (void)LOS_MuxDelete(at->trx_mux);
        at->trx_mux_flag = false;
    }

    if (at->rid_flag)
    {
        (VOID)LOS_QueueDelete(at->rid);
        at->rid_flag = false;
    }

    if (NULL != at->recv_buf)
    {
        at_free(at->recv_buf);
        at->recv_buf = NULL;
    }

    if (NULL != at->cmdresp)
    {
        at_free(at->cmdresp);
        at->cmdresp = NULL;
    }

    if (NULL != at->userdata)
    {
        at_free(at->userdata);
        at->userdata = NULL;
    }

    if (NULL != at->saveddata)
    {
        at_free(at->saveddata);
        at->saveddata = NULL;
    }

    if (NULL != at->linkid)
    {
        at_free(at->linkid);
        at->linkid = NULL;
    }

    at->tsk_hdl = 0xFFFF;
    at->head = NULL;
    at->mux_mode = AT_MUXMODE_SINGLE;
    at->timeout = 0;

    return ret;
}

void at_set_config(at_config *config)
{
    if(NULL != config){
        memcpy(&at_user_conf,config,sizeof(at_config));
    }
}

at_config *at_get_config(void)
{
    return &at_user_conf;
}


int32_t at_init(at_config *config)
{

    if(NULL == config)
    {
        AT_LOG("Config is NULL, failed!!\n");
        return AT_FAILED;
    }

    memcpy(&at_user_conf,config,sizeof(at_config));

    AT_LOG_DEBUG("Config %s(buffer total is %lu)......\n", at_user_conf.name, at_user_conf.user_buf_len);

    //LOS_TaskDelay(200);
    if (AT_OK != at_struct_init(&at))
    {
        AT_LOG("prepare AT struct failed!");
        return AT_FAILED;
    }
    at_init_oob();

    if(AT_OK != at_usart_init())
    {
        AT_LOG("at_usart_init failed!");
        (void)at_struct_deinit(&at);
        return AT_FAILED;
    }
    if(LOS_OK != create_at_recv_task())
    {
        AT_LOG("create_at_recv_task failed!");
        at_usart_deinit();
        (void)at_struct_deinit(&at);
        return AT_FAILED;
    }

    AT_LOG("Config complete!!\n");
    return AT_OK;
}


void at_deinit(void)
{


    int cnt = 0;
    const int max_try_num = 10;


    while(LOS_TaskNameGet(at.tsk_hdl) != NULL && cnt < max_try_num)
    {
        write_at_task_msg(AT_TASK_QUIT);
        LOS_TaskDelay(1000);
        cnt++;
    }


    if (LOS_TaskNameGet(at.tsk_hdl) != NULL)
    {
        if(LOS_OK != LOS_TaskDelete(at.tsk_hdl))
        {
            AT_LOG("at_recv_task delete failed!");
        }
    }

    at_usart_deinit();
    if(AT_OK != at_struct_deinit(&at))
    {
        AT_LOG("at_struct_deinit failed!");
    }
    at_init_oob();

}


int32_t at_oob_register(char *featurestr, int cmdlen, oob_callback callback, oob_cmd_match cmd_match)
{
    oob_t *oob;
    if(featurestr == NULL || cmd_match == NULL || at_oob.oob_num == OOB_MAX_NUM || cmdlen >= OOB_CMD_LEN - 1)
        return -1;
    oob = &(at_oob.oob[at_oob.oob_num++]);
    memcpy(oob->featurestr, featurestr, cmdlen);
    oob->len = strlen(featurestr);
    oob->callback = callback;
    oob->cmd_match = cmd_match;
    return 0;
}

void *at_malloc(size_t size)
{
    void *pMem = LOS_MemAlloc(m_aucSysMem0, size);
    if(NULL != pMem)
    {
        memset(pMem, 0, size);
    }
    return pMem;
}

void at_free(void *ptr)
{
    (void)LOS_MemFree(m_aucSysMem0, ptr);
}

void at_reg_step_callback(at_task *at_tsk, void (*step_callback)(void))
{
    at_tsk->step_callback = step_callback;
}


at_task at =
{
    .tsk_hdl = 0xFFFF,
    .recv_buf = NULL,
    .cmdresp = NULL,
    .userdata = NULL,
    .linkid = NULL,
    .head = NULL,

    .init = at_init,
    .deinit = at_deinit,
    .cmd = at_cmd,
    .write = at_write,
    .oob_register = at_oob_register,
    .get_id = at_get_unuse_linkid,
    .cmd_multi_suffix = at_cmd_multi_suffix
};


#endif
