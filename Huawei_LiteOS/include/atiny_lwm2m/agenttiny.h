/*----------------------------------------------------------------------------
 * Copyright (c) <2018>, <Huawei Technologies Co., Ltd>
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

/**@defgroup agent AgentTiny
 * @defgroup agenttiny Agenttiny Definition
 * @ingroup agent
 */
#ifndef AGENT_TINY_H
#define AGENT_TINY_H
#include "atiny_error.h"
#include <stdbool.h>
#include <stdint.h>
#include "log/atiny_log.h"
#ifdef CONFIG_FEATURE_FOTA
#include "ota/ota_api.h"
#endif
#include "liblwm2m_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************The following interfaces are implemented by user***********************/

typedef enum
{
    ATINY_GET_MANUFACTURER,
    ATINY_GET_MODEL_NUMBER,
    ATINY_GET_SERIAL_NUMBER,
    ATINY_GET_FIRMWARE_VER,
    ATINY_DO_DEV_REBOOT,
    ATINY_DO_FACTORY_RESET,
    ATINY_GET_POWER_SOURCE,
    ATINY_GET_SOURCE_VOLTAGE,
    ATINY_GET_POWER_CURRENT,
    ATINY_GET_BATERRY_LEVEL,
    ATINY_GET_MEMORY_FREE,
    ATINY_GET_DEV_ERR,
    ATINY_DO_RESET_DEV_ERR,
    ATINY_GET_CURRENT_TIME,
    ATINY_SET_CURRENT_TIME,
    ATINY_GET_UTC_OFFSET,
    ATINY_SET_UTC_OFFSET,
    ATINY_GET_TIMEZONE,
    ATINY_SET_TIMEZONE,
    ATINY_GET_BINDING_MODES,
    ATINY_GET_FIRMWARE_STATE,
    ATINY_GET_NETWORK_BEARER,
    ATINY_GET_SIGNAL_STRENGTH,
    ATINY_GET_CELL_ID,
    ATINY_GET_LINK_QUALITY,
    ATINY_GET_LINK_UTILIZATION,
    ATINY_WRITE_APP_DATA,
    ATINY_UPDATE_PSK,
    ATINY_GET_LATITUDE,
    ATINY_GET_LONGITUDE,
    ATINY_GET_ALTITUDE,
    ATINY_GET_RADIUS,
    ATINY_GET_SPEED,
    ATINY_GET_TIMESTAMP,
    ATINY_GET_VELOCITY,
    ATINY_GET_OTA_OPT,
    ATINY_TRIGER_SERVER_INITIATED_BS
} atiny_cmd_e;

#define MAX_VELOCITY_LEN 16

typedef struct
{
    uint8_t opaque[MAX_VELOCITY_LEN];
    int length;
} atiny_velocity_s;

/**
 *@ingroup agenttiny
 *@brief issue the command.
 *
 *@par Description:
 *This API is used to issue the command.
 *@attention none.
 *
 *@param cmd            [IN]     The command to be issued. @ref atiny_cmd_e.
 *@param arg            [IN/OUT] Buffer to store the command parameters. Agent_tiny is responsible
                                 for memory allocation. if cmd is GET_XXX, arg is outcoming parameter
                                 and this buffer is filled by device; else arg is incoming parameter
                                 and this buffer is filled by agent_tiny.
 *@param len            [IN]     The length of the argument.
 *
 *@retval #int          0 if succeed, or the error number @ref atiny_error_e if failed.
 *@par Dependency: none.
 *@see none.
 */
int atiny_cmd_ioctl(atiny_cmd_e cmd, char* arg, int len);

typedef enum
{
    ATINY_REG_OK,
    ATINY_REG_FAIL,
    ATINY_DATA_SUBSCRIBLE,
    ATINY_DATA_UNSUBSCRIBLE,
    ATINY_FOTA_STATE
} atiny_event_e;

/**
 *@ingroup agenttiny
 *@brief issue the command.
 *
 *@par Description:
 *This API is used to issue the command.
 *@attention none.
 *
 *@param stat           [IN] The event to be issued. @ref atiny_event_e.
 *@param arg            [IN] Buffer to store the event parameters.
 *@param len            [IN] The length of the argument.
 *
 *@retval #int          0 if succeed, or the error number @ref atiny_error_e if failed.
 *@par Dependency: none.
 *@see none.
 */
void atiny_event_notify(atiny_event_e event, const char* arg, int len);

/****************The following interfaces are implemented by agent_tiny*******************/

typedef lwm2m_bootstrap_type_e   atiny_bootstrap_type_e;

typedef struct
{
    char* binding;               /*目前支持U或者UQ*/
    int   life_time;             /*必选，默认50000,如过短，则频繁发送update报文，如过长，在线状态更新时间长*/
    unsigned int  storing_cnt;   /*storing为true时，lwm2m缓存区总字节个数*/
 
    atiny_bootstrap_type_e  bootstrap_mode; /* bootstrap mode  */
    int   hold_off_time; /* bootstrap hold off time for server initiated bootstrap */
} atiny_server_param_t;





typedef struct
{
    char* server_ip;
    char* server_port;

    char* psk_Id;
    char* psk;
    unsigned short psk_len;

} atiny_security_param_t;



typedef enum
{
    FIRMWARE_UPDATE_STATE = 0,
    APP_DATA
} atiny_report_type_e;

typedef struct
{

    atiny_server_param_t   server_params;
    //both iot_server and bs_server have psk & pskID, index 0 for iot_server, and index 1 for bs_server
    atiny_security_param_t security_params[2];
} atiny_param_t;

typedef struct
{
    char* endpoint_name;
    char* manufacturer;
    char* dev_type;
} atiny_device_info_t;

/**
 *@ingroup agenttiny
 *@brief initialize the lwm2m protocal.
 *
 *@par Description:
 *This API is used to initialize the lwm2m protocal.
 *@attention none.
 *
 *@param atiny_params   [IN]  Configure parameters of lwm2m.
 *@param phandle        [OUT] The handle of the agent_tiny.
 *
 *@retval #int          0 if succeed, or the error number @ref atiny_error_e if failed.
 *@par Dependency: none.
 *@see atiny_bind | atiny_deinit.
 */
int atiny_init(atiny_param_t* atiny_params, void** phandle);

/**
 *@ingroup agenttiny
 *@brief main task of the lwm2m protocal.
 *
 *@par Description:
 *This API is used to implement the lwm2m protocal, and interactive with lwm2m server.
 *@attention none.
 *
 *@param device_info    [IN] The information of devices to be bound.
 *@param phandle        [IN] The handle of the agent_tiny.
 *
 *@retval #int          0 if succeed, or the error number @ref atiny_error_e if failed.
 *@par Dependency: none.
 *@see atiny_init | atiny_deinit.
 */
int atiny_bind(atiny_device_info_t* device_info, void* phandle);

/**
 *@ingroup agenttiny
 *@brief stop the device and release resources.
 *
 *@par Description:
 *This API is used to stop the device and release resources.
 *@attention none.
 *
 *@param phandle        [IN] The handle of the agent_tiny.
 *
 *@retval none.
 *@par Dependency: none.
 *@see atiny_init | atiny_bind.
 */
void atiny_deinit(void* phandle);

#define MAX_REPORT_DATA_LEN      1024
#define MAX_BUFFER_REPORT_CNT    8
#define MAX_SEND_ERR_NUM 10
#define MAX_RECV_ERR_NUM 10

typedef enum
{
    NOT_SENT = 0,
    SENT_WAIT_RESPONSE,
    SENT_FAIL,
    SENT_TIME_OUT,
    SENT_SUCCESS,
    SENT_GET_RST,
    SEND_PENDING,
} data_send_status_e;

typedef void (*atiny_ack_callback) (atiny_report_type_e type, int cookie, data_send_status_e status);

typedef struct _data_report_t
{
    atiny_report_type_e type;     /*数据上报类型*/
    int cookie;                   /*数据cookie,用以在ack回调中，区分不同的数据*/
    int len;                      /*数据长度，不应大于MAX_REPORT_DATA_LEN*/
    uint8_t* buf;                 /*数据缓冲区首地址*/
    atiny_ack_callback callback;  /*ack回调*/
} data_report_t;

/**
 *@ingroup agenttiny
 *@brief report the data to the server.
 *
 *@par Description:
 *This API is used to report the data to the server.
 *@attention none.
 *
 *@param phandle        [IN] The handle of the agent_tiny.
 *@param report_data    [IN] Data to be reported.
 *
 *@retval #ATINY_OK           Report succeed.
 *@retval #ATINY_MSG_CONGEST  The lwm2m protocal stack is congested. It is occured when
                              the number of report messages are greater than storing_cnt
                              in atiny_server_param_t.
 *@retval #ATINY_BUF_OVERFLOW The length of the buffer in report_data is exceeded
                              MAX_REPORT_DATA_LEN.
 *@par Dependency: none.
 *@see none.
 */
int atiny_data_report(void* phandle, data_report_t* report_data);

#define DEVICE_AVL_POWER_SOURCES    "/3/0/6"
#define DEVICE_POWER_SOURCE_VOLTAGE "/3/0/7"
#define DEVICE_POWER_SOURCE_CURRENT "/3/0/8"
#define DEVICE_BATTERY_LEVEL        "/3/0/9"
#define DEVICE_MEMORY_FREE          "/3/0/10"

/**
 *@ingroup agenttiny
 *@brief
 *
 *@par Description:
 *This API is used to
 *@attention none.
 *
 *@param phandle        [IN] The handle of the agent_tiny.
 *@param data_type      [IN] Data type of the changed resource.
 *
 *@retval #int          ATINY_OK if succeed or error code @ref atiny_error_e if failed.
 *@par Dependency: none.
 *@see none.
 */
int atiny_data_change(void* phandle, const char* data_type);

/**
 *@ingroup agenttiny
 *@brief reconnect lwm2m server.
 *
 *@par Description:
 *This API is used to reconnect lwm2m server.
 *@attention none.
 *
 *@param phandle        [IN] The handle of the agent_tiny.
 *
 *@retval #int          0 if succeed, or the error number @ref atiny_error_e if failed.
 *@par Dependency: none.
 *@see atiny_init | atiny_deinit.
 */
int atiny_reconnect(void* phandle);



#ifdef __cplusplus
}
#endif

#endif

