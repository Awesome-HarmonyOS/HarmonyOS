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

#include "atiny_mqtt/mqtt_client.h"
#include "los_base.h"
#include "los_task.ph"
#include "los_typedef.h"
#include "los_sys.h"
#include "log/atiny_log.h"
#include "MQTTClient.h"
#include "flash_manager.h"
#include "dtls_interface.h"
#include "cJSON.h"
#include "hmac.h"

#define MQTT_VERSION_3_1 (3)
#define MQTT_VERSION_3_1_1 (4)

#define VARIABLE_SIZE (4 + 1)
#define CMD_TOPIC_FMT "/huawei/v1/devices/%s/command/%s"
#define DATA_TOPIC_FMT "/huawei/v1/devices/%s/data/%s"
#define SECRET_NOTIFY_TOPIC_FMT "/huawei/v1/products/%s/sn/%s/secretNotify"
#define SECRET_ACK_TOPIC_FMT "/huawei/v1/products/%s/sn/%s/secretACK"

#define MQTT_TIME_BUF_LEN 11

#define IS_VALID_NAME_LEN(name) (strnlen((name), STRING_MAX_LEN + 1) <= STRING_MAX_LEN)

typedef enum
{
    MQTT_CONNECT_WITH_PRODUCT_ID,
    MQTT_CONNECT_WITH_DEVICE_ID
}mqtt_dynamic_connect_state_e;

typedef struct
{
    mqtt_static_connection_info_s save_info;
    char *  got_password;
    mqtt_dynamic_connect_state_e state;
    uint8_t connection_update_flag;
    uint8_t has_device_id; //
    uint8_t reserve[2];
}mqtt_dynamic_info_s;

struct mqtt_client_tag_s
{
    mqtt_device_info_s device_info;
    MQTTClient client;
    mqtt_param_s params;
    mqtt_dynamic_info_s dynamic_info;
    char *sub_topic;
    uint8_t init_flag;
    uint8_t reserve[3];
};

static uint8_t g_mqtt_sendbuf[MQTT_SENDBUF_SIZE];

/* reserve 1 byte for string end 0 for jason */
static uint8_t g_mqtt_readbuf[MQTT_READBUF_SIZE + 1];

static mqtt_client_s g_mqtt_client;

static int mqtt_cmd_ioctl(mqtt_cmd_e cmd, void *arg, int32_t len)
{
    mqtt_client_s* handle = &g_mqtt_client;

    if (handle->params.cmd_ioctl != NULL)
    {
        return handle->params.cmd_ioctl(cmd, arg, len);
    }
    ATINY_LOG(LOG_FATAL, "cmd_ioctl null");
    return ATINY_ERR;
}

static void mqtt_free_params(mqtt_param_s *param)
{

    TRY_FREE_MEM(param->server_ip);
    TRY_FREE_MEM(param->server_port);
    switch(param->info.security_type)
    {
    case MQTT_SECURITY_TYPE_PSK:
        TRY_FREE_MEM(param->info.u.psk.psk_id);
        TRY_FREE_MEM(param->info.u.psk.psk);
        break;
    case MQTT_SECURITY_TYPE_CA:
        TRY_FREE_MEM(param->info.u.ca.ca_crt);
        break;
    default:
        break;
    }
}

static int mqtt_check_param(const mqtt_param_s *param)
{
    if ((param->server_ip == NULL)
        || (param->server_port == NULL)
        || (param->info.security_type >= MQTT_SECURITY_TYPE_MAX)
        || (param->cmd_ioctl == NULL))
    {
        ATINY_LOG(LOG_FATAL, "invalid param, sec type %d", param->info.security_type);
        return ATINY_ARG_INVALID;
    }
    if (param->info.security_type == MQTT_SECURITY_TYPE_PSK)
    {
        if ((param->info.u.psk.psk == NULL)
            || (param->info.u.psk.psk_len <= 0)
            || (param->info.u.psk.psk_id == NULL)
            || (param->info.u.psk.psk_id_len <= 0))
        {
            ATINY_LOG(LOG_FATAL, "invalid psk");
            return ATINY_ARG_INVALID;
        }
    }

    if (param->info.security_type == MQTT_SECURITY_TYPE_CA)
    {
        if (param->info.u.ca.ca_crt == NULL)
        {
            ATINY_LOG(LOG_FATAL, "invalid ca");
            return ATINY_ARG_INVALID;
        }
    }

    return ATINY_OK;
}

static int mqtt_dup_param(mqtt_param_s *dest, const mqtt_param_s *src)
{
    memset(dest, 0, sizeof(*dest));

    dest->info.security_type = src->info.security_type;
    dest->cmd_ioctl = src->cmd_ioctl;

    dest->server_ip = atiny_strdup(src->server_ip);
    if(NULL == dest->server_ip)
    {
        ATINY_LOG(LOG_FATAL, "atiny_strdup NULL");
        return ATINY_MALLOC_FAILED;
    }

    dest->server_port = atiny_strdup(src->server_port);
    if(NULL == dest->server_port)
    {
        ATINY_LOG(LOG_FATAL, "atiny_strdup NULL");
        goto mqtt_param_dup_failed;
    }

    switch(src->info.security_type)
    {
        case MQTT_SECURITY_TYPE_PSK:
            dest->info.u.psk.psk_id = (uint8_t *)atiny_malloc(src->info.u.psk.psk_id_len);
            if(NULL == dest->info.u.psk.psk_id)
            {
                ATINY_LOG(LOG_FATAL, "atiny_strdup NULL");
                goto mqtt_param_dup_failed;
            }
            memcpy(dest->info.u.psk.psk_id, src->info.u.psk.psk_id, src->info.u.psk.psk_id_len);
            dest->info.u.psk.psk_id_len = src->info.u.psk.psk_id_len;

            dest->info.u.psk.psk = (unsigned char *)atiny_malloc(src->info.u.psk.psk_len);
            if(NULL == dest->info.u.psk.psk)
            {
                ATINY_LOG(LOG_FATAL, "atiny_strdup NULL");
                goto mqtt_param_dup_failed;
            }
            memcpy(dest->info.u.psk.psk, src->info.u.psk.psk, src->info.u.psk.psk_len);
            dest->info.u.psk.psk_len = src->info.u.psk.psk_len;
            break;

        case MQTT_SECURITY_TYPE_CA:
            dest->info.u.ca.ca_crt = (char *)atiny_malloc(src->info.u.ca.ca_len);
            if(NULL == dest->info.u.ca.ca_crt)
            {
                ATINY_LOG(LOG_FATAL, "atiny_strdup NULL");
                goto mqtt_param_dup_failed;
            }
            memcpy(dest->info.u.ca.ca_crt, src->info.u.ca.ca_crt, src->info.u.ca.ca_len);
            dest->info.u.ca.ca_len = src->info.u.ca.ca_len;
            break;
        default:
            break;
    }


    return ATINY_OK;

mqtt_param_dup_failed:
    mqtt_free_params(dest);
    return ATINY_MALLOC_FAILED;
}

static void mqtt_free_device_info(mqtt_device_info_s *info)
{

    TRY_FREE_MEM(info->password);
    if(MQTT_STATIC_CONNECT == info->connection_type)
    {
        TRY_FREE_MEM(info->u.s_info.deviceid);
    }
    else
    {
        TRY_FREE_MEM(info->u.d_info.productid);
        TRY_FREE_MEM(info->u.d_info.nodeid);
    }
}

static void mqtt_free_dynamic_info(mqtt_client_s* handle)
{
    if (handle->sub_topic)
    {
        (void)MQTTSetMessageHandler(&handle->client, handle->sub_topic, NULL);
        atiny_free(handle->sub_topic);
        handle->sub_topic = NULL;
    }
    TRY_FREE_MEM(handle->dynamic_info.save_info.deviceid);
    TRY_FREE_MEM(handle->dynamic_info.got_password);
}

static int mqtt_check_device_info(const mqtt_device_info_s *info)
{
    if((info->connection_type >= MQTT_MAX_CONNECTION_TYPE)
        || (info->codec_mode >= MQTT_MAX_CODEC_MODE)
        || (info->sign_type >= MQTT_MAX_SIGN_TYPE)
        || (NULL == info->password)
        || (!IS_VALID_NAME_LEN(info->password)))
    {
        ATINY_LOG(LOG_FATAL, "invalid device info con_type %d codec_mode %d signe type %d",
            info->connection_type, info->codec_mode, info->sign_type);
        return ATINY_ARG_INVALID;
    }

    if ((info->connection_type == MQTT_STATIC_CONNECT)
        && ((NULL == info->u.s_info.deviceid)
        || (!IS_VALID_NAME_LEN(info->u.s_info.deviceid))))
    {
        ATINY_LOG(LOG_FATAL, "invalid static device info con_type %d codec_mode %d signe type %d",
            info->connection_type, info->codec_mode, info->sign_type);
        return ATINY_ARG_INVALID;
    }

    if ((info->connection_type == MQTT_DYNAMIC_CONNECT)
        &&((NULL == info->u.d_info.productid)
        || (!IS_VALID_NAME_LEN(info->u.d_info.productid))
        || (NULL == info->u.d_info.nodeid)
        || !(IS_VALID_NAME_LEN(info->u.d_info.nodeid))))
    {
        ATINY_LOG(LOG_FATAL, "invalid dynamic device info con_type %d codec_mode %d signe type %d",
            info->connection_type, info->codec_mode, info->sign_type);
        return ATINY_ARG_INVALID;
    }

    return ATINY_OK;

}

static int mqtt_dup_device_info(mqtt_device_info_s *dest, const mqtt_device_info_s *src)
{
    memset(dest, 0, sizeof(*dest));
    dest->connection_type = src->connection_type;
    dest->sign_type = src->sign_type;
    dest->codec_mode = src->codec_mode;
    dest->password = atiny_strdup(src->password);
    if (NULL == dest->password)
    {
        ATINY_LOG(LOG_FATAL, "atiny_strdup fail");
        return ATINY_MALLOC_FAILED;
    }

    if(MQTT_STATIC_CONNECT == src->connection_type)
    {
        dest->u.s_info.deviceid = atiny_strdup(src->u.s_info.deviceid);
        if (NULL == dest->u.s_info.deviceid)
        {
            ATINY_LOG(LOG_FATAL, "atiny_strdup fail");
            goto MALLOC_FAIL;
        }
    }
    else
    {

        dest->u.d_info.productid = atiny_strdup(src->u.d_info.productid);
        if (NULL == dest->u.d_info.productid)
        {
            ATINY_LOG(LOG_FATAL, "atiny_strdup fail");
            goto MALLOC_FAIL;
        }
        dest->u.d_info.nodeid = atiny_strdup(src->u.d_info.nodeid);
        if (NULL == dest->u.d_info.nodeid)
        {
            ATINY_LOG(LOG_FATAL, "atiny_strdup fail");
            goto MALLOC_FAIL;
        }
    }

    return ATINY_OK;

MALLOC_FAIL:
    mqtt_free_device_info(dest);
    return ATINY_MALLOC_FAILED;
}

static bool mqtt_is_connectting_with_deviceid(const mqtt_client_s* handle)
{
    return (MQTT_STATIC_CONNECT == handle->device_info.connection_type)
           || (handle->dynamic_info.state == MQTT_CONNECT_WITH_DEVICE_ID);
}

static char *mqtt_add_strings(const char* strs[], uint32_t tmp_len[], uint32_t str_num)
{
    uint32_t i;
    const char hyphen = '_';
    uint32_t len = 0;
    char *result;
    char *cur;

    for (i = 0; i < str_num; i++)
    {
        tmp_len[i] = strnlen(strs[i], STRING_MAX_LEN);
        len += (tmp_len[i] + 1);
    }

    result = atiny_malloc(len);
    if (result == NULL)
    {
        ATINY_LOG(LOG_FATAL, "mqtt_cmd_ioctl fail");
        return NULL;
    }

    cur = result;
    for (i = 0; i < str_num; i++)
    {
        memcpy(cur, strs[i], tmp_len[i]);
        cur += tmp_len[i];
        if (i != str_num - 1)
        {
            *cur = hyphen;
            cur++;
        }
    }
    *cur = '\0';

    return result;
}

static const char *mqtt_connection_type_to_str(mqtt_connection_type_e type)
{
    return MQTT_STATIC_CONNECT == type ? "0" : "1";
}

static const char *mqtt_sign_type_to_str(mqtt_password_sign_type_e type)
{
    return MQTT_SIGN_TYPE_HMACSHA256_NO_CHECK_TIME == type ? "0" : "1";
}

static void mqtt_bin_to_str(const uint8_t *bin_buf, char *str_buf, uint32_t bin_len)
{
    uint32_t i;

    for(i = 0; i < bin_len; i++)
    {
        (void)snprintf(str_buf + i * 2, 3, "%02x", bin_buf[i]);
    }
}
static char *mqtt_get_send_password(char *password, char *time)
{
    uint8_t hmac[32];
    const uint32_t len = sizeof(hmac) * 2 + 1;
    char *result;
    int ret;

    mbedtls_hmac_t hmac_info;

    hmac_info.secret = (uint8_t *)time;
    hmac_info.secret_len = strnlen(time, MQTT_TIME_BUF_LEN);
    hmac_info.input = (uint8_t *)password;
    hmac_info.input_len = strnlen(password, STRING_MAX_LEN);
    hmac_info.digest = hmac;
    hmac_info.digest_len = sizeof(hmac);
    hmac_info.hmac_type = MBEDTLS_MD_SHA256;
    ret = mbedtls_hmac_calc(&hmac_info);
    if (ret != ATINY_OK)
    {
        ATINY_LOG(LOG_FATAL, "mbedtls_hmac_calc fail,ret %d", ret);
        return NULL;
    }

    result = atiny_malloc(len);
    if (result == NULL)
    {
        ATINY_LOG(LOG_FATAL, "atiny_malloc fail");
        return NULL;
    }

    mqtt_bin_to_str(hmac, result, sizeof(hmac));
    return result;
}

static void mqtt_destroy_data_connection_info(MQTTPacket_connectData *data)
{
    TRY_FREE_MEM(data->clientID.cstring);
    TRY_FREE_MEM(data->password.cstring);
}

static int mqtt_get_connection_info(mqtt_client_s* handle, MQTTPacket_connectData *data)
{
    char *strs[5];
    uint32_t tmp[array_size(strs)];
    uint32_t str_num = 0;
    char time[MQTT_TIME_BUF_LEN];
    char *password;

    if (mqtt_is_connectting_with_deviceid(handle))
    {
        if (handle->device_info.connection_type == MQTT_STATIC_CONNECT)
        {
            strs[0] = handle->device_info.u.s_info.deviceid;
            password = handle->device_info.password;
        }
        else
        {
            strs[0] = handle->dynamic_info.save_info.deviceid;
            password = handle->dynamic_info.got_password;
        }
        str_num = 1;
        ATINY_LOG(LOG_INFO, "try static connect");
    }
    else
    {
        strs[0] = handle->device_info.u.d_info.productid;
        strs[1] = handle->device_info.u.d_info.nodeid;
        str_num = 2;
        password = handle->device_info.password;
        ATINY_LOG(LOG_INFO, "try dynamic connect");
    }

    strs[str_num++] = (char *)mqtt_connection_type_to_str(
            mqtt_is_connectting_with_deviceid(handle) ? MQTT_STATIC_CONNECT : MQTT_DYNAMIC_CONNECT);
    strs[str_num++] = (char *)mqtt_sign_type_to_str(handle->device_info.sign_type);
    if (mqtt_cmd_ioctl(MQTT_GET_TIME, time, sizeof(time)) != ATINY_OK)
    {
        ATINY_LOG(LOG_FATAL, "mqtt_cmd_ioctl fail");
        return ATINY_ERR;
    }
    time[sizeof(time) - 1] = '\0';
    strs[str_num++] = time;

    data->clientID.cstring = mqtt_add_strings((const char **)strs, tmp, str_num);
    if (data->clientID.cstring == NULL)
    {
        return ATINY_MALLOC_FAILED;
    }

    data->username.cstring = strs[0]; //deviceid or pruoductid
    data->password.cstring = mqtt_get_send_password(password, time);

    if (data->password.cstring == NULL)
    {
        return ATINY_ERR;
    }

    ATINY_LOG(LOG_DEBUG, "send user %s client %s", data->username.cstring,
                data->clientID.cstring);

    return ATINY_OK;
}

static char *mqtt_get_topic(const mqtt_client_s* handle, const char *fmt, uint32_t fixed_size,
            const char *deviceid_or_productid, const char *sn_or_codec_mode)
{
    uint32_t len;
    char *topic;

    len = fixed_size + strnlen(deviceid_or_productid, STRING_MAX_LEN) + strnlen(sn_or_codec_mode, STRING_MAX_LEN) + 1;
    topic = atiny_malloc(len);
    if (topic == NULL)
    {
        ATINY_LOG(LOG_FATAL, "atiny_malloc fail, len %d", len);
        return NULL;
    }

    (void)snprintf(topic, len, fmt, deviceid_or_productid, sn_or_codec_mode);

    return topic;
}


static char *mqtt_get_device_topic(const mqtt_client_s* handle, const char *fmt, uint32_t fixed_size)
{
    char *deviceid;
    const char *codec_mode[MQTT_MAX_CODEC_MODE] = {"binary", "json"};

    deviceid = ((handle->device_info.connection_type == MQTT_STATIC_CONNECT) ?
                handle->device_info.u.s_info.deviceid : handle->dynamic_info.save_info.deviceid);

    return mqtt_get_topic(handle, fmt, fixed_size, deviceid, codec_mode[handle->device_info.codec_mode]);

}

static char *mqtt_get_secret_topic(const mqtt_client_s* handle, const char *fmt, uint32_t fixed_size)
{
    return mqtt_get_topic(handle, fmt, fixed_size,
            handle->device_info.u.d_info.productid, handle->device_info.u.d_info.nodeid);
}

static int mqtt_parse_secret_topic(mqtt_client_s* handle, const char *payload, uint32_t len)
{

    //TODO:parse json, log
    cJSON *msg_type;
    cJSON *deviceid;
    cJSON *secret;
    cJSON * root = NULL;
    int ret = ATINY_ERR;


    root = cJSON_Parse(payload);
    if (root == NULL)
    {
        ATINY_LOG(LOG_ERR, "err secret notify, len %d, msg %s", len, payload);
         goto EXIT;
    }

    msg_type = cJSON_GetObjectItem(root, MQTT_MSG_TYPE);
    if ((msg_type == NULL) || (msg_type->valuestring == NULL)
        || (strncmp(msg_type->valuestring, "cloudSendSecret", STRING_MAX_LEN + 1) != 0))
    {
        ATINY_LOG(LOG_ERR, "msg_type not right");
        goto EXIT;
    }

    deviceid = cJSON_GetObjectItem(root, "deviceid");
    if ((deviceid == NULL) || (deviceid->valuestring == NULL)
        || (!IS_VALID_NAME_LEN(deviceid->valuestring)))
    {
        ATINY_LOG(LOG_ERR, "deviceid not right");
        goto EXIT;
    }

    secret = cJSON_GetObjectItem(root, "secret");
    if ((secret == NULL) || (secret->valuestring == NULL)
        || (!IS_VALID_NAME_LEN(secret->valuestring)))
    {
        ATINY_LOG(LOG_ERR, "secret not right");
        goto EXIT;
    }

    handle->dynamic_info.has_device_id = false;
    TRY_FREE_MEM(handle->dynamic_info.save_info.deviceid);
    handle->dynamic_info.save_info.deviceid = atiny_strdup(deviceid->valuestring);
    if (handle->dynamic_info.save_info.deviceid == NULL)
    {
        ATINY_LOG(LOG_INFO, "atiny_strdup null");
        goto EXIT;
    }
    TRY_FREE_MEM(handle->dynamic_info.got_password);
    handle->dynamic_info.got_password = atiny_strdup(secret->valuestring);
    if (handle->dynamic_info.got_password == NULL)
    {
        ATINY_LOG(LOG_INFO, "atiny_strdup null");
        goto EXIT;
    }
    handle->dynamic_info.has_device_id = true;
    handle->dynamic_info.connection_update_flag = true;
    ret = ATINY_OK;
    ATINY_LOG(LOG_INFO, "get secret info right");
EXIT:
    if (root)
    {
        cJSON_Delete(root);
    }
    return ret;
}

static void mqtt_send_secret_ack(mqtt_client_s* handle)
{
    MQTTMessage message;
    int rc;

    char* topic = mqtt_get_secret_topic(handle, SECRET_ACK_TOPIC_FMT, sizeof(SECRET_ACK_TOPIC_FMT) - VARIABLE_SIZE);
    if (topic == NULL)
    {
        return;
    }
    memset(&message, 0, sizeof(message));
    message.qos = QOS1;
    rc = MQTTPublish(&handle->client, topic, &message);
    atiny_free(topic);
    if (rc != MQTT_SUCCESS)
    {
        ATINY_LOG(LOG_FATAL, "MQTTPublish fail,rc %d", rc);
    }
}

/*lint -e529*/
static int mqtt_modify_payload(MessageData *md)
{
    char *end = ((char *)md->message->payload) + md->message->payloadlen;
    static uint32_t callback_err;

    /* add for jason parse,then not need to copy in callback */
    if ((end >= (char *)g_mqtt_readbuf) && (end < (char *)(g_mqtt_readbuf + sizeof(g_mqtt_readbuf))))
    {
         *end = '\0';
         return ATINY_OK;
    }

    /*  should not happen */
    ATINY_LOG(LOG_ERR, "not expect msg callback err, pl %p, len %ld, err num %ld", md->message->payload, md->message->payloadlen, ++callback_err);

    return ATINY_ERR;
}
/*lint +e529*/

static void mqtt_recv_secret_topic(MessageData *md)
{
    mqtt_client_s* handle = &g_mqtt_client;

    if ((md == NULL) || (md->message == NULL)
        || (md->message->payload == NULL)
        || (md->message->payloadlen == 0)
        || (mqtt_modify_payload(md) != ATINY_OK))
    {
        ATINY_LOG(LOG_FATAL, "null point or msg err, len %ld", md->message->payloadlen);
        return;
    }

    if (mqtt_parse_secret_topic(handle, md->message->payload, md->message->payloadlen) == ATINY_OK)
    {
        flash_info_s flash_info;
        flash_info.items[PRODUCT_IDX] = handle->device_info.u.d_info.productid;
        flash_info.items[NODEID_IDX] = handle->device_info.u.d_info.nodeid;
        flash_info.items[DEVICEID_IDX] = handle->dynamic_info.save_info.deviceid;
        flash_info.items[PASSWORD_IDX] = handle->dynamic_info.got_password;
        if (flash_manager_write(&flash_info) != ATINY_OK)
        {
            return;
        }
        mqtt_send_secret_ack(handle);
    }
}

static void mqtt_recv_cmd_topic(MessageData *md)
{
    if ((md == NULL) || (md->message == NULL)
        || (mqtt_modify_payload(md) != ATINY_OK))
    {
        ATINY_LOG(LOG_FATAL, "null point");
        return;
    }
    (void)mqtt_cmd_ioctl(MQTT_RCV_MSG, md->message->payload, md->message->payloadlen);
}

static int mqtt_subscribe_topic(mqtt_client_s* handle)
{
    char *topic;
    void (*topi_callback)(MessageData *md);
    int rc;

    if (handle->sub_topic)
    {
        (void)MQTTSetMessageHandler(&handle->client, handle->sub_topic, NULL);
        atiny_free(handle->sub_topic);
        handle->sub_topic = NULL;
    }

    if (mqtt_is_connectting_with_deviceid(handle))
    {
        topic = mqtt_get_device_topic(handle, CMD_TOPIC_FMT, sizeof(CMD_TOPIC_FMT) - VARIABLE_SIZE);
        topi_callback = mqtt_recv_cmd_topic;
        ATINY_LOG(LOG_INFO, "try subcribe static topic");
    }
    else
    {
        topic = mqtt_get_secret_topic(handle, SECRET_NOTIFY_TOPIC_FMT, sizeof(SECRET_NOTIFY_TOPIC_FMT) - VARIABLE_SIZE);;
        topi_callback = mqtt_recv_secret_topic;
         ATINY_LOG(LOG_INFO, "try subcribe dynamic topic");
    }

    if (topic == NULL)
    {
        return ATINY_ERR;
    }


    rc = MQTTSubscribe(&handle->client, topic, QOS1, topi_callback);
    if (rc != MQTT_SUCCESS)
    {
         ATINY_LOG(LOG_FATAL, "MQTTSubscribe fail,rc=%d, topic=%s", rc, topic);
         atiny_free(topic);
    }
    else
    {
        handle->sub_topic = topic;
    }

    return rc;
}

static void mqtt_disconnect( MQTTClient *client, Network *n)
{
    if (MQTTIsConnected(client))
    {
        (void)MQTTDisconnect(client);
    }
    NetworkDisconnect(n);

    ATINY_LOG(LOG_ERR, "mqtt_disconnect");
}

static inline void mqtt_inc_fail_cnt(int32_t *conn_failed_cnt)
{
    if(*conn_failed_cnt < MQTT_CONN_FAILED_MAX_TIMES)
    {
        (*conn_failed_cnt)++;
    }
}

static void mqtt_proc_connect_err( MQTTClient *client, Network *n, int32_t *conn_failed_cnt)
{
    mqtt_inc_fail_cnt(conn_failed_cnt);
    mqtt_disconnect(client, n);
}

static void mqtt_proc_connect_nack(mqtt_client_s* handle)
{
    if (handle->device_info.connection_type == MQTT_DYNAMIC_CONNECT)
    {
        if (handle->dynamic_info.state == MQTT_CONNECT_WITH_DEVICE_ID)
        {
            handle->dynamic_info.state = MQTT_CONNECT_WITH_PRODUCT_ID;
        }
        else
        {
            if (handle->dynamic_info.has_device_id)
            {
                handle->dynamic_info.state = MQTT_CONNECT_WITH_DEVICE_ID;
            }
        }
    }
}

static mqtt_security_info_s *mqtt_get_security_info(void)
{
    mqtt_client_s* handle = &g_mqtt_client;
    return &handle->params.info;
}

static void mqtt_read_flash_info(mqtt_client_s* handle)
{
    flash_info_s flash_info;

    if (handle->device_info.connection_type == MQTT_STATIC_CONNECT)
    {
        return;
    }

    memset(&handle->dynamic_info, 0, sizeof(handle->dynamic_info));
    memset(&flash_info, 0, sizeof(flash_info));
    if (flash_manager_read(&flash_info) != ATINY_OK)
    {
        return;
    }

    if ((strcmp(flash_info.items[PRODUCT_IDX], handle->device_info.u.d_info.productid) != 0)
        || (strcmp(flash_info.items[NODEID_IDX], handle->device_info.u.d_info.nodeid) != 0))
    {
        ATINY_LOG(LOG_INFO, "flash info for the nodeid and not use");
        flash_manager_destroy_flash_info(&flash_info);
        return;
    }

    ATINY_LOG(LOG_DEBUG, "mqtt read info deviceid %s,procid %s,nodid %s",
            flash_info.items[DEVICEID_IDX], flash_info.items[PRODUCT_IDX], flash_info.items[NODEID_IDX]);

    handle->dynamic_info.save_info.deviceid = flash_info.items[DEVICEID_IDX];
    flash_info.items[DEVICEID_IDX] = NULL;
    handle->dynamic_info.got_password =  flash_info.items[PASSWORD_IDX];
    flash_info.items[PASSWORD_IDX] = NULL;
    handle->dynamic_info.has_device_id = true;
    handle->dynamic_info.state = MQTT_CONNECT_WITH_DEVICE_ID;
    flash_manager_destroy_flash_info(&flash_info);
}

int  atiny_mqtt_init(const mqtt_param_s *params, mqtt_client_s **phandle)
{
    cJSON_InitHooks(NULL);
    if (params == NULL || phandle == NULL
        || params->info.security_type != MQTT_SECURITY_TYPE_CA
        || mqtt_check_param(params) != ATINY_OK)
    {
        ATINY_LOG(LOG_FATAL, "Invalid args");
        return ATINY_ARG_INVALID;
    }

    if (g_mqtt_client.init_flag)
    {
        ATINY_LOG(LOG_FATAL, "mqtt reinit");
        return ATINY_ERR;
    }

    memset(&g_mqtt_client, 0, sizeof(g_mqtt_client));

    if (ATINY_OK != mqtt_dup_param(&(g_mqtt_client.params), params))
    {
        return ATINY_MALLOC_FAILED;
    }

    flash_manager_init(mqtt_cmd_ioctl);

    *phandle = &g_mqtt_client;

    g_mqtt_client.init_flag = true;

    return ATINY_OK;
}

int atiny_mqtt_bind(const mqtt_device_info_s* device_info, mqtt_client_s* handle)
{
    Network n;
    MQTTClient *client = NULL;
    mqtt_param_s *params;
    int rc;
    int32_t conn_failed_cnt = 0;
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    Timer timer;
    int result = ATINY_ERR;

    if (NULL == handle)
    {
        ATINY_LOG(LOG_FATAL, "handle null");
        return ATINY_ARG_INVALID;
    }

    if((device_info == NULL)
        || (mqtt_check_device_info(device_info) != ATINY_OK))
    {
        ATINY_LOG(LOG_FATAL, "parameter invalid");
        result = ATINY_ARG_INVALID;
        goto  atiny_bind_quit;
    }

    dtls_init();

    client = &(handle->client);
    params = &(handle->params);

    rc = mqtt_dup_device_info(&(handle->device_info), device_info);
    if (rc != ATINY_OK)
    {
        goto  atiny_bind_quit;
    }

    mqtt_read_flash_info(handle);

    NetworkInit(&n, mqtt_get_security_info);

    memset(client, 0x0, sizeof(MQTTClient));
    rc = MQTTClientInit(client, &n, MQTT_COMMAND_TIMEOUT_MS, g_mqtt_sendbuf, MQTT_SENDBUF_SIZE, g_mqtt_readbuf, MQTT_READBUF_SIZE);
    if (rc != MQTT_SUCCESS)
    {
        ATINY_LOG(LOG_FATAL, "MQTTClientInit fail,rc %d", rc);
        goto  atiny_bind_quit;
    }

    data.willFlag = 0;
    data.MQTTVersion = MQTT_VERSION_3_1_1;
    data.keepAliveInterval = MQTT_KEEPALIVE_INTERVAL_S;
    data.cleansession = true;

    while(true)
    {
        if(conn_failed_cnt > 0)
        {
            ATINY_LOG(LOG_INFO, "reconnect delay : %d", conn_failed_cnt);
            (void)LOS_TaskDelay(MQTT_CONN_FAILED_BASE_DELAY << conn_failed_cnt);
        }

        rc = NetworkConnect(&n, params->server_ip, atoi(params->server_port));
        if(rc != 0)
        {
            ATINY_LOG(LOG_ERR, "NetworkConnect fail: %d", rc);
            mqtt_inc_fail_cnt(&conn_failed_cnt);
            continue;
        }

        if(mqtt_get_connection_info(handle, &data) != ATINY_OK)
        {
            mqtt_destroy_data_connection_info(&data);
            mqtt_proc_connect_err(client, &n, &conn_failed_cnt);
            continue;
        }

        rc = MQTTConnect(client, &data);
        mqtt_destroy_data_connection_info(&data);
        ATINY_LOG(LOG_DEBUG, "CONNACK : %d", rc);
        if(MQTT_SUCCESS != rc)
        {
            // receive connection nack value
            if (rc != MQTT_SUCCESS)
            {
                mqtt_proc_connect_nack(handle);
            }
            ATINY_LOG(LOG_ERR, "MQTTConnect failed %d", rc);
            mqtt_proc_connect_err(client, &n, &conn_failed_cnt);
            continue;
        }

        if(ATINY_OK != mqtt_subscribe_topic(handle))
        {
            ATINY_LOG(LOG_ERR, "mqtt_subscribe_topic failed");
            mqtt_proc_connect_err(client, &n, &conn_failed_cnt);
            continue;
        }

        conn_failed_cnt = 0;
        if (!mqtt_is_connectting_with_deviceid(handle))
        {
            TimerInit(&timer);
            TimerCountdownMS(&timer, MQTT_WRITE_FOR_SECRET_TIMEOUT);
        }
        while (rc >= 0 && MQTTIsConnected(client))
        {
            rc = MQTTYield(client, MQTT_EVENTS_HANDLE_PERIOD_MS);

            // receive secret info
            if (handle->dynamic_info.connection_update_flag)
            {
                ATINY_LOG(LOG_INFO, "recv secret info");
                ATINY_LOG(LOG_DEBUG, "secret info deviceid %s", handle->dynamic_info.save_info.deviceid);
                handle->dynamic_info.connection_update_flag = false;
                handle->dynamic_info.state = MQTT_CONNECT_WITH_DEVICE_ID;
                break;
            }

            // wait secret info timeout.
            if (!mqtt_is_connectting_with_deviceid(handle) && (TimerIsExpired(&timer)))
            {
                if (handle->dynamic_info.has_device_id)
                {
                    handle->dynamic_info.state = MQTT_CONNECT_WITH_DEVICE_ID;
                }
                break;
            }
        }

        mqtt_disconnect(client, &n);
    }

    result = ATINY_OK;
atiny_bind_quit:
    mqtt_free_dynamic_info(handle);
    mqtt_free_params(&(handle->params));
    (void)atiny_task_mutex_lock(&client->mutex);
    mqtt_free_device_info(&(handle->device_info));
    (void)atiny_task_mutex_unlock(&client->mutex);
    MQTTClientDeInit(client);
    handle->init_flag = false;
    return result;
}

int atiny_mqtt_data_send(mqtt_client_s *phandle, const char *msg,  uint32_t msg_len, mqtt_qos_e qos)
{
    MQTTMessage message;
    int rc;
    char* topic;
    size_t payloadlen;

    if ((phandle == NULL) || (qos >= MQTT_QOS_MAX))
    {
        ATINY_LOG(LOG_FATAL, "Parameter invalid");
        return ATINY_ARG_INVALID;
    }
    if (phandle->device_info.codec_mode == MQTT_CODEC_MODE_JSON)
    {
        if (msg == NULL || msg_len <= 0)
        {
            ATINY_LOG(LOG_FATAL, "msg invalid");
            return ATINY_ARG_INVALID;
        }
        payloadlen = strnlen(msg, msg_len);
    }
    else
    {
        if (msg == NULL && msg_len > 0)
        {
            ATINY_LOG(LOG_FATAL, "msg invalid");
            return ATINY_ARG_INVALID;
        }
        payloadlen = msg_len;
    }

    if (!atiny_mqtt_isconnected(phandle))
    {
        ATINY_LOG(LOG_FATAL, "not connected");
        return ATINY_ERR;
    }

    topic = mqtt_get_device_topic(phandle, DATA_TOPIC_FMT, sizeof(DATA_TOPIC_FMT) - VARIABLE_SIZE);
    if (topic == NULL)
    {
        return ATINY_MALLOC_FAILED;
    }
    memset(&message, 0, sizeof(message));
    message.qos = (enum QoS)qos;
    message.payload = (void *)msg;
    message.payloadlen = payloadlen;
    rc = MQTTPublish(&phandle->client, topic, &message);
    atiny_free(topic);
    if (rc != MQTT_SUCCESS)
    {
        ATINY_LOG(LOG_FATAL, "MQTTPublish fail,rc %d", rc);
        return ATINY_ERR;
    }
    return ATINY_OK;
}

int atiny_mqtt_isconnected(mqtt_client_s* phandle)
{
    if (NULL == phandle)
    {
        ATINY_LOG(LOG_ERR, "invalid args");
        return false;
    }
    return mqtt_is_connectting_with_deviceid(phandle) && MQTTIsConnected(&(phandle->client));
}

