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

/*******************************************************************************
 *
 * Copyright (c) 2014 Bosch Software Innovations GmbH, Germany.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * The Eclipse Distribution License is available at
 *    http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Bosch Software Innovations GmbH - Please refer to git log
 *
 *******************************************************************************/
/*
 * object_comm.h
 *
 *  General functions of lwm2m test client.
 *
 *  Created on: 22.01.2015
 *  Author: Achim Kraus
 *  Copyright (c) 2015 Bosch Software Innovations GmbH, Germany. All rights reserved.
 */

#ifndef OBJECT_COMM_H_
#define OBJECT_COMM_H_

#include "internals.h"
#include "atiny_lwm2m/agenttiny.h"
#include "connection.h"
#include "agent_list.h"

#ifndef MAX
#define MAX(a, b) ((a) >= (b)? (a) : (b))
#endif

extern int g_reboot;

typedef struct
{
    lwm2m_object_t * securityObjP;
    lwm2m_object_t * serverObject;
    connection_t  *connList;
    lwm2m_context_t * lwm2mH;
} client_data_t;

typedef struct _security_instance_
{
    struct _security_instance_ * next;        // matches lwm2m_list_t::next
    uint16_t                     instanceId;  // matches lwm2m_list_t::id
    char *                       uri;
    bool                         isBootstrap;
    uint8_t                      securityMode;
    char *                       publicIdentity;
    uint16_t                     publicIdLen;
    char *                       serverPublicKey;
    uint16_t                     serverPublicKeyLen;
    char *                       secretKey;
    uint16_t                     secretKeyLen;
    uint8_t                      smsSecurityMode;
    char *                       smsParams; // SMS binding key parameters
    uint16_t                     smsParamsLen;
    char *                       smsSecret; // SMS binding secret key
    uint16_t                     smsSecretLen;
    uint16_t                     shortID;
    uint32_t                     clientHoldOffTime;
    uint32_t                     bootstrapServerAccountTimeout;
} security_instance_t;

typedef struct _data_node_t
{
    atiny_dl_list list;
    data_report_t data;
}data_node_t;
/*
 * object_device.c
 */
lwm2m_object_t * get_object_device(atiny_param_t *atiny_params, const char* manufacturer);
void free_object_device(lwm2m_object_t * objectP);
void display_device_object(lwm2m_object_t * objectP);
/*
 * object_firmware.c
 */
lwm2m_object_t * get_object_firmware(atiny_param_t *atiny_params);
void free_object_firmware(lwm2m_object_t * objectP);
/*
 * object_location.c
 */
lwm2m_object_t * get_object_location(void);
void free_object_location(lwm2m_object_t * object);
void display_location_object(lwm2m_object_t * objectP);
/*
 * object_test.c
 */
#define TEST_OBJECT_ID 31024
lwm2m_object_t * get_test_object(void);
void free_test_object(lwm2m_object_t * object);
void display_test_object(lwm2m_object_t * objectP);

/*
 * binary_app_data_object.c
 */
#define BINARY_APP_DATA_OBJECT_ID 19
#define BINARY_APP_DATA_RES_ID 0
lwm2m_object_t * get_binary_app_data_object(atiny_param_t *atiny_params);
void free_binary_app_data_object(lwm2m_object_t * object);
void display_binary_app_data_object(lwm2m_object_t * objectP);
void set_binary_app_data_object_rpt_max_cnt(uint32_t max_rpt_cnt);


/*
 * object_server.c
 */
lwm2m_object_t * get_server_object(int serverId, const char* binding, int lifetime, bool storing);
void clean_server_object(lwm2m_object_t * object);
void display_server_object(lwm2m_object_t * objectP);
void copy_server_object(lwm2m_object_t * objectDest, lwm2m_object_t * objectSrc);

/*
 * object_connectivity_moni.c
 */
lwm2m_object_t * get_object_conn_m(atiny_param_t *atiny_params);
void free_object_conn_m(lwm2m_object_t * objectP);
uint8_t connectivity_moni_change(lwm2m_data_t * dataArray, lwm2m_object_t * objectP);

/*
 * object_connectivity_stat.c
 */
extern lwm2m_object_t * get_object_conn_s(void);
void free_object_conn_s(lwm2m_object_t * objectP);
extern void conn_s_updateTxStatistic(lwm2m_object_t * objectP, uint16_t txDataByte, bool smsBased);
extern void conn_s_updateRxStatistic(lwm2m_object_t * objectP, uint16_t rxDataByte, bool smsBased);

/*
 * object_access_control.c
 */
typedef enum
{
    OBJ_ACC_READ       = 0,
    OBJ_ACC_WRITE_ATTR = 0,
    OBJ_ACC_OBSERVE    = 0,
    OBJ_ACC_NOTIFY     = 0,
    OBJ_ACC_WRITE      = 1,
    OBJ_ACC_EXCUTE     = 2,
    OBJ_ACC_DELETE     = 3,
    OBJ_ACC_CREATE     = 4,
    OBJ_ACC_DISCOVER   = 5,

} OBJ_ACC_OPERATE;

#define ACC_AUTH_R     (0x01 << OBJ_ACC_READ)
#define ACC_AUTH_W     (0x01 << OBJ_ACC_WRITE)
#define ACC_AUTH_E     (0x01 << OBJ_ACC_EXCUTE)
#define ACC_AUTH_D     (0x01 << OBJ_ACC_DELETE)
#define ACC_AUTH_C     (0x01 << OBJ_ACC_CREATE)

lwm2m_object_t* acc_ctrl_create_object(void);
void acl_ctrl_free_object(lwm2m_object_t * objectP);
bool  acc_ctrl_obj_add_inst (lwm2m_object_t* accCtrlObjP, uint16_t instId,
                 uint16_t acObjectId, uint16_t acObjInstId, uint16_t acOwner);
bool  acc_ctrl_oi_add_ac_val(lwm2m_object_t* accCtrlObjP, uint16_t instId,
                 uint16_t aclResId, uint16_t acValue);
uint8_t acc_auth_operate(lwm2m_context_t* contextP, lwm2m_uri_t* uri,
                 OBJ_ACC_OPERATE op, uint16_t serverId);

/*
 * object_comm.c
 */
void handle_value_changed(lwm2m_context_t* lwm2mH, lwm2m_uri_t* uri, const char * value, size_t valueLength);

/*
 * object_security.c
 */
lwm2m_object_t * get_security_object(uint16_t serverId,atiny_param_t* atiny_params,lwm2m_context_t* lwm2m_context);
void clean_security_object(lwm2m_object_t * objectP);
char * get_server_uri(lwm2m_object_t * objectP, uint16_t secObjInstID);
void display_security_object(lwm2m_object_t * objectP);
void copy_security_object(lwm2m_object_t * objectDest, lwm2m_object_t * objectSrc);

int prv_refreshServerList(lwm2m_context_t * contextP);
void output_buffer(FILE * stream, uint8_t * buffer, int length, int indent);
void atiny_set_reboot_flag(void);
#endif /* LWM2MCLIENT_H_ */
