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

#include "internals.h"
#include "atiny_lwm2m/agenttiny.h"
#include "atiny_context.h"
#include "connection.h"
#include "log/atiny_log.h"
#include "atiny_rpt.h"
#include "osdepends/atiny_osdep.h"
#ifdef CONFIG_FEATURE_FOTA
#include "atiny_fota_manager.h"
#endif



int g_reboot = 0;


void observe_handle_ack(lwm2m_transaction_t *transacP, void *message);
static int atiny_check_bootstrap_init_param(atiny_param_t *atiny_params);

static handle_data_t g_atiny_handle;

/*
 * modify date:   2018-06-20
 * description: in order to check the params for the bootstrap, expecialy for the mode and the ip/port
 * return:
 *              success: ATINY_OK
 *              fail:    ATINY_ARG_INVALID
 *
 */
static int atiny_check_bootstrap_init_param(atiny_param_t *atiny_params)
{
    if(NULL == atiny_params)
    {
        return ATINY_ARG_INVALID;
    }

    if(BOOTSTRAP_FACTORY == atiny_params->server_params.bootstrap_mode)
    {
        if((NULL == atiny_params->security_params[0].server_ip) || (NULL == atiny_params->security_params[0].server_port))
        {
            LOG("[bootstrap_tag]: BOOTSTRAP_FACTORY mode's params is wrong, should have iot server ip/port");
            return ATINY_ARG_INVALID;
        }
    }
    else if(BOOTSTRAP_CLIENT_INITIATED == atiny_params->server_params.bootstrap_mode)
    {
        if((NULL == atiny_params->security_params[1].server_ip) || (NULL == atiny_params->security_params[1].server_port))
        {
            LOG("[bootstrap_tag]: BOOTSTRAP_CLIENT_INITIATED mode's params is wrong, should have bootstrap server ip/port");
            return ATINY_ARG_INVALID;
        }
    }
    else if(BOOTSTRAP_SEQUENCE == atiny_params->server_params.bootstrap_mode)
    {
        return ATINY_OK;
    }
    else
    {
        //it is ok? if the mode value is not 0,1,2, we all set it to 2 ?
        LOG("[bootstrap_tag]: BOOTSTRAP only have three mode, should been :0,1,2");
        return ATINY_ARG_INVALID;
    }


    return ATINY_OK;
}

#ifdef LWM2M_BOOTSTRAP
static int atiny_check_psk_init_param(atiny_param_t *atiny_params)
{
    int i = 0;
    int psk_id_len = 0;
    int psk_len = 0;
    const int PSK_ID_LIMIT_LEN = 128;
    const int PSK_LIMIT_LEN = 64;
    int total_element = 0;

    if(NULL == atiny_params)
    {
        return ATINY_ARG_INVALID;
    }

    //security_params have 2 element, we have 2 pair psk.
    total_element = (sizeof(atiny_params->security_params)) / (sizeof(atiny_params->security_params[0]));

    for(i = 0; i < total_element; i++)
    {
        //if there are null, we could run not in security mode
        if((atiny_params->security_params[i].psk_Id != NULL) && (atiny_params->security_params[i].psk != NULL))
        {
            psk_id_len = strlen(atiny_params->security_params[i].psk_Id);
            psk_len = strlen(atiny_params->security_params[i].psk);

            //the limit of the len, please read RFC4279  or OMA-TS-LightweightM2M E.1.1
            if((psk_id_len > PSK_ID_LIMIT_LEN) || (psk_len > PSK_LIMIT_LEN))
            {
                LOG("[bootstrap_tag]: psk_Id len over 128 or psk len over 64");
                return ATINY_ARG_INVALID;
            }
        }
    }

    return ATINY_OK;
}
#endif


int  atiny_init(atiny_param_t *atiny_params, void **phandle)
{
    int result;
    
    result = atiny_init_rpt();
    if (result != ATINY_OK)
    {
        ATINY_LOG(LOG_FATAL, "atiny_init_rpt fail,ret=%d", result);
        return result;
    }

    if (NULL == atiny_params || NULL == phandle)
    {
        ATINY_LOG(LOG_FATAL, "Invalid args");
        return ATINY_ARG_INVALID;
    }

    if(ATINY_OK != atiny_check_bootstrap_init_param(atiny_params))
    {
        LOG("[bootstrap_tag]: BOOTSTRAP's params are wrong");
        return ATINY_ARG_INVALID;
    }

#ifdef LWM2M_BOOTSTRAP
    if(ATINY_OK != atiny_check_psk_init_param(atiny_params))
    {
        LOG("[bootstrap_tag]: psk params are wrong");
    }
#endif

    memset((void *)&g_atiny_handle, 0, sizeof(handle_data_t));

    g_atiny_handle.quit_sem = atiny_mutex_create();
    if (NULL == g_atiny_handle.quit_sem)
    {
        ATINY_LOG(LOG_FATAL, "atiny_mutex_create fail");
        return ATINY_RESOURCE_NOT_ENOUGH;
    }
    atiny_mutex_lock(g_atiny_handle.quit_sem);
    g_atiny_handle.atiny_params = *atiny_params;
    *phandle = &g_atiny_handle;

#ifdef CONFIG_FEATURE_FOTA

    return atiny_fota_manager_set_storage_device(atiny_fota_manager_get_instance());
#else
    return ATINY_OK;
#endif

}

/*
 * add date:     2018-06-05
 * description:  get bootstrap info from atiny_params which from user, set bs_sequence_state and bs_server_uri for lwm2m_context.
 *
 * return:       none
 * param:
 *     in:  atiny_params
 *     out: lwm2m_context
 */
void atiny_set_bootstrap_sequence_state(atiny_param_t *atiny_params, lwm2m_context_t *lwm2m_context)
{
    (void)lwm2m_initBootStrap(lwm2m_context, atiny_params->server_params.bootstrap_mode);
}



/*
* modify info:
*     date:    2018-05-30
*     reason:  modify for bootstrap mode, origin code only support FACTORY mode.
*/
int atiny_init_objects(atiny_param_t *atiny_params, const atiny_device_info_t *device_info, handle_data_t *handle)
{
    int result;
    client_data_t *pdata;
    lwm2m_context_t *lwm2m_context = NULL;
    uint16_t serverId = SERVER_ID;
    char *epname = (char *)device_info->endpoint_name;



    pdata = &handle->client_data;
    memset(pdata, 0, sizeof(client_data_t));

    ATINY_LOG(LOG_INFO, "Trying to init objects");

    lwm2m_context = lwm2m_init(pdata);
    if (NULL == lwm2m_context)
    {
        ATINY_LOG(LOG_FATAL, "lwm2m_init fail");
        return ATINY_MALLOC_FAILED;
    }
    lwm2m_context->observe_mutex = atiny_mutex_create();
    if (NULL == lwm2m_context->observe_mutex)
    {
        ATINY_LOG(LOG_FATAL, "atiny_mutex_create fail");
        lwm2m_free(lwm2m_context);
        return ATINY_RESOURCE_NOT_ENOUGH;
    }

    pdata->lwm2mH = lwm2m_context;

    handle->lwm2m_context = lwm2m_context;

    //even if not in bootstrap sequence mode, still set it NO_BS_SEQUENCE_STATE
    atiny_set_bootstrap_sequence_state(atiny_params, lwm2m_context);

    handle->obj_array[OBJ_SECURITY_INDEX] = get_security_object(serverId, atiny_params, lwm2m_context);

    if (NULL ==  handle->obj_array[OBJ_SECURITY_INDEX])
    {
        ATINY_LOG(LOG_FATAL, "Failed to create security object");
        return ATINY_MALLOC_FAILED;
    }
    pdata->securityObjP = handle->obj_array[OBJ_SECURITY_INDEX];

    handle->obj_array[OBJ_SERVER_INDEX] = get_server_object(serverId, atiny_params->server_params.binding,
                                          atiny_params->server_params.life_time, atiny_params->server_params.storing_cnt != 0);
    if (NULL == handle->obj_array[OBJ_SERVER_INDEX])
    {
        ATINY_LOG(LOG_FATAL, "Failed to create server object");
        return ATINY_MALLOC_FAILED;
    }

    handle->obj_array[OBJ_ACCESS_CONTROL_INDEX] = acc_ctrl_create_object();
    if (NULL == handle->obj_array[OBJ_ACCESS_CONTROL_INDEX])
    {
        ATINY_LOG(LOG_FATAL, "Failed to create access control object");
        return ATINY_MALLOC_FAILED;
    }

    handle->obj_array[OBJ_DEVICE_INDEX] = get_object_device(atiny_params, device_info->manufacturer);
    if (NULL == handle->obj_array[OBJ_DEVICE_INDEX])
    {
        ATINY_LOG(LOG_FATAL, "Failed to create device object");
        return ATINY_MALLOC_FAILED;
    }

    handle->obj_array[OBJ_CONNECT_INDEX] = get_object_conn_m(atiny_params);
    if (NULL == handle->obj_array[OBJ_CONNECT_INDEX])
    {
        ATINY_LOG(LOG_FATAL, "Failed to create connect object");
        return ATINY_MALLOC_FAILED;
    }

    handle->obj_array[OBJ_FIRMWARE_INDEX] = get_object_firmware(atiny_params);
#ifdef CONFIG_FEATURE_FOTA
    if (NULL == handle->obj_array[OBJ_FIRMWARE_INDEX])
    {
        ATINY_LOG(LOG_FATAL, "Failed to create firmware object");
        return ATINY_MALLOC_FAILED;
    }
#endif

    handle->obj_array[OBJ_LOCATION_INDEX] = get_object_location();
    if (NULL == handle->obj_array[OBJ_LOCATION_INDEX])
    {
        ATINY_LOG(LOG_FATAL, "Failed to create location object");
        return ATINY_MALLOC_FAILED;
    }

    handle->obj_array[OBJ_APP_INDEX] = get_binary_app_data_object(atiny_params);
    if (NULL == handle->obj_array[OBJ_APP_INDEX])
    {
        ATINY_LOG(LOG_FATAL, "Failed to create app object");
        return ATINY_MALLOC_FAILED;
    }

    result = lwm2m_configure(lwm2m_context, epname, NULL, NULL, OBJ_MAX_NUM, handle->obj_array);
    if (result != 0)
    {
        return ATINY_RESOURCE_NOT_FOUND;
    }

    return ATINY_OK;
}

static int lwm2m_poll(handle_data_t *phandle, uint32_t *timeout)
{
    client_data_t *dataP;
    int numBytes;
    connection_t *connP;
    lwm2m_context_t *contextP = phandle->lwm2m_context;
    uint8_t *recv_buffer = phandle->recv_buffer;

    dataP = (client_data_t *)(contextP->userData);
    connP = dataP->connList;

    while (connP != NULL)
    {
        numBytes = lwm2m_buffer_recv(connP, recv_buffer, MAX_PACKET_SIZE, *timeout);
        if (numBytes <= 0)
        {
            ATINY_LOG(LOG_INFO, "no packet arrived!");
        }
        else
        {
            output_buffer(stderr, recv_buffer, numBytes, 0);
            lwm2m_handle_packet(contextP, recv_buffer, numBytes, connP);
        }
        connP = connP->next;
    }

    return ATINY_OK;
}

void atiny_destroy(void *handle)
{
    handle_data_t *handle_data = (handle_data_t *)handle;

    if (handle_data == NULL)
    {
        return;
    }
#ifdef CONFIG_FEATURE_FOTA
    atiny_fota_manager_destroy(atiny_fota_manager_get_instance());
#endif
    if(handle_data->recv_buffer != NULL)
    {
        lwm2m_free(handle_data->recv_buffer);
    }
    if (handle_data->obj_array[OBJ_SECURITY_INDEX] != NULL)
    {
        clean_security_object(handle_data->obj_array[OBJ_SECURITY_INDEX]);
    }

    if (handle_data->obj_array[OBJ_SERVER_INDEX] != NULL)
    {
        clean_server_object(handle_data->obj_array[OBJ_SERVER_INDEX]);
    }

    if (handle_data->obj_array[OBJ_ACCESS_CONTROL_INDEX] != NULL)
    {
        acl_ctrl_free_object(handle_data->obj_array[OBJ_ACCESS_CONTROL_INDEX]);
    }

    if (handle_data->obj_array[OBJ_DEVICE_INDEX] != NULL)
    {
        free_object_device(handle_data->obj_array[OBJ_DEVICE_INDEX]);
    }

    if (handle_data->obj_array[OBJ_CONNECT_INDEX] != NULL)
    {
        free_object_conn_m(handle_data->obj_array[OBJ_CONNECT_INDEX]);
    }

    if (handle_data->obj_array[OBJ_FIRMWARE_INDEX] != NULL)
    {
        free_object_firmware(handle_data->obj_array[OBJ_FIRMWARE_INDEX]);
    }

    if (handle_data->obj_array[OBJ_LOCATION_INDEX] != NULL)
    {
        free_object_location(handle_data->obj_array[OBJ_LOCATION_INDEX]);
    }

    if (handle_data->obj_array[OBJ_APP_INDEX] != NULL)
    {
        free_binary_app_data_object(handle_data->obj_array[OBJ_APP_INDEX]);
    }
    atiny_destroy_rpt();

    if (handle_data->lwm2m_context != NULL)
    {
        if (handle_data->lwm2m_context->observe_mutex != NULL)
        {
            atiny_mutex_destroy(handle_data->lwm2m_context->observe_mutex);
        }
        lwm2m_close(handle_data->lwm2m_context);
    }
    atiny_mutex_unlock(handle_data->quit_sem);
}

void atiny_event_handle(module_type_t type, int code, const char *arg, int arg_len)
{
    switch (type)
    {
    case MODULE_LWM2M:
    {
        if (code == STATE_REGISTERED)
        {
            atiny_event_notify(ATINY_REG_OK, NULL, 0);
#ifdef CONFIG_FEATURE_FOTA
            (void)atiny_fota_manager_repot_result(atiny_fota_manager_get_instance());
#endif
        }
        else if (code == STATE_REG_FAILED)
        {
            atiny_event_notify(ATINY_REG_FAIL, NULL, 0);

        }
        break;
    }
    case MODULE_NET:
    {
        break;
    }
    case MODULE_URI:
    {
        if ((arg == NULL) || (arg_len < sizeof(lwm2m_uri_t)))
        {
            break;
        }

        if (code == OBSERVE_UNSUBSCRIBE)
        {
            if (dm_isUriOpaqueHandle((lwm2m_uri_t *)arg))
            {
                atiny_report_type_e rpt_type = APP_DATA;
                atiny_event_notify(ATINY_DATA_UNSUBSCRIBLE, (char *)&rpt_type, sizeof(rpt_type));
            }
            (void)atiny_clear_rpt_data((lwm2m_uri_t *)arg, SENT_FAIL);
        }
        else if (code == OBSERVE_SUBSCRIBE)
        {
            if (dm_isUriOpaqueHandle((lwm2m_uri_t *)arg))
            {
                atiny_report_type_e rpt_type = APP_DATA;
                atiny_event_notify(ATINY_DATA_SUBSCRIBLE, (char *)&rpt_type, sizeof(rpt_type));
            }
        }

        break;
    }
    default:
    {
        break;
    }
    }

}

void reboot_check(void)
{
    if(g_reboot == 1)
    {
        (void)atiny_cmd_ioctl(ATINY_DO_DEV_REBOOT, NULL, 0);
    }
}

static void atiny_connection_err_notify(lwm2m_context_t *context, connection_err_e err_type, bool boostrap_flag)
{
    handle_data_t *handle = NULL;

    if((NULL == context) || (NULL == context->userData))
    {
        ATINY_LOG(LOG_ERR, "null point");
        return;
    }

    if(!boostrap_flag)
    {
        handle = ATINY_FIELD_TO_STRUCT(context->userData, handle_data_t, client_data);
        (void)atiny_reconnect(handle);
    }
    ATINY_LOG(LOG_INFO, "connection err type %d bootstrap %d", err_type, boostrap_flag);
}


static void atiny_handle_reconnect(handle_data_t *handle)
{
    if(handle->reconnect_flag)
    {
        (void)lwm2m_reconnect(handle->lwm2m_context);
        handle->reconnect_flag = false;
        ATINY_LOG(LOG_INFO, "lwm2m reconnect");
    }
}

int atiny_bind(atiny_device_info_t *device_info, void *phandle)
{
    handle_data_t *handle = (handle_data_t *)phandle;
    uint32_t timeout;
    int ret;

    if ((NULL == device_info) || (NULL == phandle))
    {
        ATINY_LOG(LOG_FATAL, "Parameter null");
        atiny_deinit(phandle);
        return ATINY_ARG_INVALID;
    }

    if (NULL == device_info->endpoint_name)
    {
        ATINY_LOG(LOG_FATAL, "Endpoint name null");
        atiny_deinit(phandle);
        return ATINY_ARG_INVALID;
    }

    if (NULL == device_info->manufacturer)
    {
        ATINY_LOG(LOG_FATAL, "Manufacturer name null");
        atiny_deinit(phandle);
        return ATINY_ARG_INVALID;
    }

    ret = atiny_init_objects(&handle->atiny_params, device_info, handle);
    if (ret != ATINY_OK)
    {
        ATINY_LOG(LOG_FATAL, "atiny_init_object fail %d", ret);
        atiny_destroy(handle);
        return ret;
    }
#ifdef CONFIG_FEATURE_FOTA
    (void)atiny_fota_manager_set_lwm2m_context(atiny_fota_manager_get_instance(), handle->lwm2m_context);
#endif
    lwm2m_register_observe_ack_call_back(observe_handle_ack);
    lwm2m_register_event_handler(atiny_event_handle);
    lwm2m_register_connection_err_notify(atiny_connection_err_notify);

    handle->recv_buffer = (uint8_t *)lwm2m_malloc(MAX_PACKET_SIZE);
    if(handle->recv_buffer == NULL)
    {
        ATINY_LOG(LOG_FATAL, "memory not enough");
        return ATINY_MALLOC_FAILED;
    }

    while (!handle->atiny_quit)
    {
        timeout = BIND_TIMEOUT;

        (void)atiny_step_rpt(handle->lwm2m_context);
        atiny_handle_reconnect(handle);
        (void)lwm2m_step(handle->lwm2m_context, (time_t *)&timeout);
        reboot_check();
        (void)lwm2m_poll(handle, &timeout);
    }

    atiny_destroy(phandle);

    return ATINY_OK;
}

void atiny_deinit(void *phandle)
{
    handle_data_t *handle;
    void *sem = NULL;

    if (phandle == NULL)
    {
        return;
    }

    handle = (handle_data_t *)phandle;
    handle->atiny_quit = 1;
    sem = handle->quit_sem;
    atiny_mutex_lock(sem);
    atiny_mutex_destroy(sem);
}

int atiny_data_report(void *phandle, data_report_t *report_data)
{
    lwm2m_uri_t uri;
    int ret;
    data_report_t data;


    if (NULL == phandle || NULL == report_data || report_data->len <= 0
            || report_data->len > MAX_REPORT_DATA_LEN || NULL == report_data->buf)
    {
        ATINY_LOG(LOG_ERR, "invalid args");
        return ATINY_ARG_INVALID;
    }

    memset((void *)&uri, 0, sizeof(uri));

    switch (report_data->type)
    {
    case FIRMWARE_UPDATE_STATE:
        (void)lwm2m_stringToUri("/5/0/3", 6, &uri);
        break;
    case APP_DATA:
        get_resource_uri(BINARY_APP_DATA_OBJECT_ID, 0, BINARY_APP_DATA_RES_ID, &uri);
        break;
    default:
        return ATINY_RESOURCE_NOT_FOUND;
    }

    memcpy(&data, report_data, sizeof(data));
    data.buf = lwm2m_malloc(report_data->len);
    if (NULL == data.buf)
    {
        ATINY_LOG(LOG_ERR, "lwm2m_malloc fail,len %d", data.len);
        return ATINY_MALLOC_FAILED;;
    }
    memcpy(data.buf, report_data->buf, report_data->len);

    ret = atiny_queue_rpt_data(&uri, &data);

    if (ATINY_OK != ret)
    {
        if (data.buf != NULL)
        {
            lwm2m_free(data.buf);
        }
    }

    return ret;
}

int atiny_data_change(void *phandle, const char *data_type)
{
    lwm2m_uri_t uri;
    handle_data_t *handle;

    if (NULL == phandle || NULL == data_type)
    {
        ATINY_LOG(LOG_ERR, "invalid args");
        return ATINY_ARG_INVALID;
    }

    memset((void *)&uri, 0, sizeof(uri));
    handle = (handle_data_t *)phandle;

    if (handle->lwm2m_context->state != STATE_READY)
    {
        ATINY_LOG(LOG_INFO, "not registered");
        return ATINY_CLIENT_UNREGISTERED;
    }

    (void)lwm2m_stringToUri(data_type, strlen(data_type), &uri);

    atiny_mutex_lock(handle->lwm2m_context->observe_mutex);
    lwm2m_resource_value_changed(handle->lwm2m_context, &uri);
    atiny_mutex_unlock(handle->lwm2m_context->observe_mutex);

    return ATINY_OK;
}

void observe_handle_ack(lwm2m_transaction_t *transacP, void *message)
{
    atiny_ack_callback ack_callback = (atiny_ack_callback)transacP->cfg.callback;
    if (transacP->ack_received)
    {
        ack_callback((atiny_report_type_e)(transacP->cfg.type), transacP->cfg.cookie, SENT_SUCCESS);
    }
    else if (transacP->retrans_counter > COAP_MAX_RETRANSMIT)
    {
        ack_callback((atiny_report_type_e)(transacP->cfg.type), transacP->cfg.cookie, SENT_TIME_OUT);
    }
    else
    {
        ack_callback((atiny_report_type_e)(transacP->cfg.type), transacP->cfg.cookie, SENT_FAIL);
    }
}

int atiny_reconnect(void *phandle)
{
    handle_data_t *handle = (handle_data_t *)phandle;


    if (NULL == phandle)
    {
        ATINY_LOG(LOG_FATAL, "Parameter null");
        return ATINY_ARG_INVALID;
    }
    handle->reconnect_flag = true;

    return ATINY_OK;
}

void atiny_set_reboot_flag()
{
    g_reboot = true;
}

