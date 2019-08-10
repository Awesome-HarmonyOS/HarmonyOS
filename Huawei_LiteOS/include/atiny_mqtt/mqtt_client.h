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

#ifndef __MQTT_CLIENT_H__
#define __MQTT_CLIENT_H__
#include "atiny_error.h"
#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

#ifndef MQTT_COMMAND_TIMEOUT_MS
#define MQTT_COMMAND_TIMEOUT_MS (10 * 1000)
#endif

#ifndef MQTT_EVENTS_HANDLE_PERIOD_MS
#define MQTT_EVENTS_HANDLE_PERIOD_MS (1*1000)
#endif

#ifndef MQTT_KEEPALIVE_INTERVAL_S
#define MQTT_KEEPALIVE_INTERVAL_S (100)
#endif

#ifndef MQTT_SENDBUF_SIZE
#define MQTT_SENDBUF_SIZE (1024 * 2)
#endif

#ifndef MQTT_READBUF_SIZE
#define MQTT_READBUF_SIZE (1024 * 2)
#endif

/* the unit is milisecond */
#ifndef MQTT_WRITE_FOR_SECRET_TIMEOUT
#define MQTT_WRITE_FOR_SECRET_TIMEOUT (30 * 1000)
#endif

/* MQTT retry connection delay interval. The delay inteval is
(MQTT_CONN_FAILED_BASE_DELAY << MIN(coutinious_fail_count, MQTT_CONN_FAILED_MAX_TIMES)) */
#ifndef MQTT_CONN_FAILED_MAX_TIMES
#define MQTT_CONN_FAILED_MAX_TIMES  6
#endif

/*The unit is millisecond*/
#ifndef MQTT_CONN_FAILED_BASE_DELAY
#define MQTT_CONN_FAILED_BASE_DELAY 1000
#endif

/* deviceReq data msg jason format example to server
the message can be decoded in json or binary.
{
        "msgType":      "deviceReq",
        "hasMore":      0,
        "data": [{
                        "serviceId":    "serviceIdValue",
                        "data": {
                                "defineData": "defineValue"
                        },
                        "eventTime":    "20161219T114920Z"
                }]
}

cloudReq data msg jason format example from server
{
	"msgType":"cloudReq",
	"serviceId":"serviceIdValue",
	"paras":{
		"paraName":"paraValue"
    },
	"cmd":"cmdValue",
	"hasMore":0,
	"mid":0
}

deviceRsp data msg jason format example to server
{
	"msgType":	"deviceRsp",
	"mid":	0,
	"errcode":	0,
	"hasMore":	0,
	"body":	{
		"bodyParaName":	"bodyParaValue"
	}
}


*/

/* msg type, json name, its value is string*/
#define MQTT_MSG_TYPE "msgType"
/* msg type report data, json value */
#define MQTT_DEVICE_REQ "deviceReq"

#define MQTT_CLOUD_REQ "cloudReq"

#define MQTT_DEVICE_RSP "deviceRsp"

/* more data, json name, its value is int, 0 for no more data, 1 for more data */
#define MQTT_HAS_MORE "hasMore"
#define MQTT_NO_MORE_DATA 0
#define MQTT_MORE_DATA 1


/* ServiceData array, json name, its value is ServiceData array */
#define MQTT_DATA "data"

/* ServiceData */
/* service id, json name, its value is string */
#define MQTT_SERVICEID "serviceId"

/* service data, json name, its value is an object defined in profile for the device */
#define MQTT_SERVICE_DATA "serviceData"

/* service data, json name, its value is string, format yyyyMMddTHHmmssZ such as 20161219T114920Z */
#define MQTT_EVENT_TIME "eventTime"

#define MQTT_CMD "cmd"
#define MQTT_PARAS "paras"
#define MQTT_MID "mid"

#define MQTT_ERR_CODE "errcode"
#define MQTT_ERR_CODE_OK 0
#define MQTT_ERR_CODE_ERR 1

#define MQTT_BODY "body"



typedef struct mqtt_client_tag_s mqtt_client_s;

typedef enum
{
    MQTT_SECURITY_TYPE_NONE,
    MQTT_SECURITY_TYPE_PSK,
    MQTT_SECURITY_TYPE_CA,
    MQTT_SECURITY_TYPE_MAX
}mqtt_security_type_e;


typedef struct
{
    uint8_t *psk_id;
    uint32_t psk_id_len;
    uint8_t *psk;
    uint32_t psk_len;
}mqtt_security_psk_s;


typedef struct
{
    char *ca_crt;
    uint32_t ca_len;
}mqtt_security_ca_s;

typedef struct
{
    mqtt_security_type_e security_type;
    union
    {
        mqtt_security_psk_s psk;
        mqtt_security_ca_s ca;
    }u;
}mqtt_security_info_s;

typedef enum
{
    MQTT_GET_TIME, // get the system time, the format is YYYYMMDDHH
    MQTT_RCV_MSG, // notify user a message received.
    MQTT_SAVE_SECRET_INFO, // write the connection secret info for dynamic connection, the info length is fixed, may be encrypted.
    MQTT_READ_SECRET_INFO, // read the connection secret info for dynamic connection, the info length is fixed.
}mqtt_cmd_e;


typedef struct
{
    char *server_ip;
    char *server_port;
    mqtt_security_info_s info;
    int (*cmd_ioctl)(mqtt_cmd_e cmd, void *arg, int32_t len); //command io control
}mqtt_param_s;

typedef enum
{
    MQTT_STATIC_CONNECT, //static connection, one device one password mode
    MQTT_DYNAMIC_CONNECT,//dynamic connection, one product type one password mode
    MQTT_MAX_CONNECTION_TYPE
}mqtt_connection_type_e;


typedef struct
{
    char *deviceid;
}mqtt_static_connection_info_s;

typedef struct
{
    char *productid;
    char *nodeid;
}mqtt_dynamic_connection_info_s;


typedef enum
{
    MQTT_QOS_MOST_ONCE,  //MQTT QOS 0
    MQTT_QOS_LEAST_ONCE, //MQTT QOS 1
    MQTT_QOS_ONLY_ONCE,  //MQTT QOS 2
    MQTT_QOS_MAX
}mqtt_qos_e;

typedef enum
{
    MQTT_CODEC_MODE_BINARY,
    MQTT_CODEC_MODE_JSON,
    MQTT_MAX_CODEC_MODE
}mqtt_codec_mode_e;

typedef enum
{
    MQTT_SIGN_TYPE_HMACSHA256_NO_CHECK_TIME, //use HMACSHA256 to encode password but no check current time
    MQTT_SIGN_TYPE_HMACSHA256_CHECK_TIME, //use HMACSHA256 to encode password and check current time
    MQTT_MAX_SIGN_TYPE
}mqtt_password_sign_type_e;


typedef struct
{
    mqtt_connection_type_e connection_type;
    mqtt_codec_mode_e codec_mode;
    mqtt_password_sign_type_e sign_type;
    char *password;
    union
    {
        mqtt_static_connection_info_s s_info;
        mqtt_dynamic_connection_info_s d_info;
    }u;
}mqtt_device_info_s;


/**
 *@ingroup agenttiny
 *@brief initialize the MQTT protocal.
 *
 *@par Description:
 *This API is used to initialize the MQTT protocal.
 *@attention none.
 *
 *@param mqtt_param_s   [IN]  Configure parameters of MQTT.
 *@param phandle        [OUT] The handle of the agent_tiny.
 *
 *@retval #int          0 if succeed, or the error number @ref mqtt_error_e if failed.
 *@par Dependency: none.
 *@see atiny_bind | atiny_deinit.
 */
int  atiny_mqtt_init(const mqtt_param_s* atiny_params, mqtt_client_s** phandle);

/**
 *@ingroup agenttiny
 *@brief main task of the MQTT protocal.
 *
 *@par Description:
 *This API is used to implement the MQTT protocal, and interactive with MQTT broker.
 *atiny_mqtt_init and atiny_mqtt_bind must be called in one task or thread.
 *@attention none.
 *
 *@param device_info    [IN] The information of devices to be bound.
 *@param phandle        [IN] The handle of the agent_tiny.
 *
 *@retval #int          0 if succeed, or the error number @ref mqtt_error_e if failed.
 *@par Dependency: none.
 *@see atiny_init | atiny_deinit.
 */
int atiny_mqtt_bind(const mqtt_device_info_s* device_info, mqtt_client_s* phandle);

/**
 *@ingroup agenttiny
 *@brief send the data to the broker.
 *
 *@par Description:
 *This API is used to send the data to the broker.
 *atiny_mqtt_init and atiny_mqtt_bind must be called in one task or thread.
 *@attention none.
 *
 *@param phandle        [IN] The handle of the agent_tiny.
 *@param msg    [IN] Message to be sended.
 *@param msg_len              [IN] Message length.
 *@param qos              [IN] quality of service used in MQTT protocol.
 *
 *@retval #int           0 if succeed, or the error number @ref mqtt_error_e if failed.
 *@par Dependency: none.
 *@see none.
 */
int atiny_mqtt_data_send(mqtt_client_s* phandle, const char *msg,  uint32_t msg_len, mqtt_qos_e qos);

/**
 *@ingroup agenttiny
 *@brief to judge if the MQTT client has connected to broker.
 *
 *@par Description:
 *This API is used to judge if the MQTT client has connected to broker.
 *@attention none.
 *
 *@param phandle        [OUT] The handle of the agent_tiny.
 *
 *@retval #int          1 if connected, 0 if not connected, or the error number @ref mqtt_error_e if failed.
 *@par Dependency: none.
 *@see atiny_bind | atiny_deinit.
 */
int atiny_mqtt_isconnected(mqtt_client_s* phandle);


#ifdef __cplusplus
}
#endif

#endif /* __MQTT_CLIENT_H__ */

