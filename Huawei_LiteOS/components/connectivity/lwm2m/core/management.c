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
 * Copyright (c) 2013, 2014 Intel Corporation and others.
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
 *    David Navarro, Intel Corporation - initial API and implementation
 *    domedambrosio - Please refer to git log
 *    Toby Jaffey - Please refer to git log
 *    Bosch Software Innovations GmbH - Please refer to git log
 *    Pascal Rieux - Please refer to git log
 *
 *******************************************************************************/
/*
 Copyright (c) 2013, 2014 Intel Corporation

 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
     * Neither the name of Intel Corporation nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 THE POSSIBILITY OF SUCH DAMAGE.

 David Navarro <david.navarro@intel.com>

*/

#include "internals.h"
#include "object_comm.h"
#include <stdio.h>


#ifdef LWM2M_CLIENT_MODE
static int prv_readAttributes(multi_option_t *query,
                              lwm2m_attributes_t *attrP)
{
    int64_t intValue;
    double floatValue;

    memset(attrP, 0, sizeof(lwm2m_attributes_t));

    while (query != NULL)
    {
        if (lwm2m_strncmp((char *)query->data, ATTR_MIN_PERIOD_STR, ATTR_MIN_PERIOD_LEN) == 0)
        {
            if (0 != ((attrP->toSet | attrP->toClear) & LWM2M_ATTR_FLAG_MIN_PERIOD)) return -1;
            if (query->len == ATTR_MIN_PERIOD_LEN) return -1;

            if (1 != utils_textToInt(query->data + ATTR_MIN_PERIOD_LEN, query->len - ATTR_MIN_PERIOD_LEN, &intValue)) return -1;
            if (intValue < 0) return -1;

            attrP->toSet |= LWM2M_ATTR_FLAG_MIN_PERIOD;
            attrP->minPeriod = intValue;
        }
        else if (lwm2m_strncmp((char *)query->data, ATTR_MIN_PERIOD_STR, ATTR_MIN_PERIOD_LEN - 1) == 0)
        {
            if (0 != ((attrP->toSet | attrP->toClear) & LWM2M_ATTR_FLAG_MIN_PERIOD)) return -1;
            if (query->len != ATTR_MIN_PERIOD_LEN - 1) return -1;

            attrP->toClear |= LWM2M_ATTR_FLAG_MIN_PERIOD;
        }
        else if (lwm2m_strncmp((char *)query->data, ATTR_MAX_PERIOD_STR, ATTR_MAX_PERIOD_LEN) == 0)
        {
            if (0 != ((attrP->toSet | attrP->toClear) & LWM2M_ATTR_FLAG_MAX_PERIOD)) return -1;
            if (query->len == ATTR_MAX_PERIOD_LEN) return -1;

            if (1 != utils_textToInt(query->data + ATTR_MAX_PERIOD_LEN, query->len - ATTR_MAX_PERIOD_LEN, &intValue)) return -1;
            if (intValue < 0) return -1;

            attrP->toSet |= LWM2M_ATTR_FLAG_MAX_PERIOD;
            attrP->maxPeriod = intValue;
        }
        else if (lwm2m_strncmp((char *)query->data, ATTR_MAX_PERIOD_STR, ATTR_MAX_PERIOD_LEN - 1) == 0)
        {
            if (0 != ((attrP->toSet | attrP->toClear) & LWM2M_ATTR_FLAG_MAX_PERIOD)) return -1;
            if (query->len != ATTR_MAX_PERIOD_LEN - 1) return -1;

            attrP->toClear |= LWM2M_ATTR_FLAG_MAX_PERIOD;
        }
        else if (lwm2m_strncmp((char *)query->data, ATTR_GREATER_THAN_STR, ATTR_GREATER_THAN_LEN) == 0)
        {
            if (0 != ((attrP->toSet | attrP->toClear) & LWM2M_ATTR_FLAG_GREATER_THAN)) return -1;
            if (query->len == ATTR_GREATER_THAN_LEN) return -1;

            if (1 != utils_textToFloat(query->data + ATTR_GREATER_THAN_LEN, query->len - ATTR_GREATER_THAN_LEN, &floatValue)) return -1;

            attrP->toSet |= LWM2M_ATTR_FLAG_GREATER_THAN;
            attrP->greaterThan = floatValue;
        }
        else if (lwm2m_strncmp((char *)query->data, ATTR_GREATER_THAN_STR, ATTR_GREATER_THAN_LEN - 1) == 0)
        {
            if (0 != ((attrP->toSet | attrP->toClear) & LWM2M_ATTR_FLAG_GREATER_THAN)) return -1;
            if (query->len != ATTR_GREATER_THAN_LEN - 1) return -1;

            attrP->toClear |= LWM2M_ATTR_FLAG_GREATER_THAN;
        }
        else if (lwm2m_strncmp((char *)query->data, ATTR_LESS_THAN_STR, ATTR_LESS_THAN_LEN) == 0)
        {
            if (0 != ((attrP->toSet | attrP->toClear) & LWM2M_ATTR_FLAG_LESS_THAN)) return -1;
            if (query->len == ATTR_LESS_THAN_LEN) return -1;

            if (1 != utils_textToFloat(query->data + ATTR_LESS_THAN_LEN, query->len - ATTR_LESS_THAN_LEN, &floatValue)) return -1;

            attrP->toSet |= LWM2M_ATTR_FLAG_LESS_THAN;
            attrP->lessThan = floatValue;
        }
        else if (lwm2m_strncmp((char *)query->data, ATTR_LESS_THAN_STR, ATTR_LESS_THAN_LEN - 1) == 0)
        {
            if (0 != ((attrP->toSet | attrP->toClear) & LWM2M_ATTR_FLAG_LESS_THAN)) return -1;
            if (query->len != ATTR_LESS_THAN_LEN - 1) return -1;

            attrP->toClear |= LWM2M_ATTR_FLAG_LESS_THAN;
        }
        else if (lwm2m_strncmp((char *)query->data, ATTR_STEP_STR, ATTR_STEP_LEN) == 0)
        {
            if (0 != ((attrP->toSet | attrP->toClear) & LWM2M_ATTR_FLAG_STEP)) return -1;
            if (query->len == ATTR_STEP_LEN) return -1;

            if (1 != utils_textToFloat(query->data + ATTR_STEP_LEN, query->len - ATTR_STEP_LEN, &floatValue)) return -1;
            if (floatValue < 0) return -1;

            attrP->toSet |= LWM2M_ATTR_FLAG_STEP;
            attrP->step = floatValue;
        }
        else if (lwm2m_strncmp((char *)query->data, ATTR_STEP_STR, ATTR_STEP_LEN - 1) == 0)
        {
            if (0 != ((attrP->toSet | attrP->toClear) & LWM2M_ATTR_FLAG_STEP)) return -1;
            if (query->len != ATTR_STEP_LEN - 1) return -1;

            attrP->toClear |= LWM2M_ATTR_FLAG_STEP;
        }
        else return -1;

        query = query->next;
    }

    return 0;
}

uint8_t dm_handleRequest(lwm2m_context_t *contextP,
                         lwm2m_uri_t *uriP,
                         lwm2m_server_t *serverP,
                         coap_packet_t *message,
                         coap_packet_t *response)
{
    uint8_t result;
    lwm2m_media_type_t format;

    LOG_ARG("Code: %02X, server status: %s", message->code, STR_STATUS(serverP->status));
    LOG_URI(uriP);

    if (IS_OPTION(message, COAP_OPTION_CONTENT_TYPE))
    {
        format = utils_convertMediaType(message->content_type);
    }
    else
    {
        format = (dm_isUriOpaqueHandle(uriP) ? LWM2M_CONTENT_OPAQUE : LWM2M_CONTENT_TLV);

    }

    if (uriP->objectId == LWM2M_SECURITY_OBJECT_ID)
    {
        return COAP_404_NOT_FOUND;
    }

    if (serverP->status != STATE_REGISTERED
            && serverP->status != STATE_REG_UPDATE_NEEDED
            && serverP->status != STATE_REG_FULL_UPDATE_NEEDED
            && serverP->status != STATE_REG_UPDATE_PENDING)
    {
        return COAP_IGNORE;
    }

    switch (message->code)
    {
    case COAP_GET:
    {
        uint8_t *buffer = NULL;
        size_t length = 0;
        int res;

        if (IS_OPTION(message, COAP_OPTION_OBSERVE))
        {
            lwm2m_data_t *dataP = NULL;
            int size = 0;

            if ((result = acc_auth_operate(contextP, uriP, OBJ_ACC_OBSERVE, serverP->shortID)) != COAP_NO_ERROR)
            {
                return result;
            }

            result = object_readData(contextP, uriP, &size, &dataP, NULL, serverP->shortID);
            if (COAP_205_CONTENT == result)
            {
                result = observe_handleRequest(contextP, uriP, serverP, size, dataP, message, response);
                if (COAP_205_CONTENT == result)
                {
                    if (IS_OPTION(message, COAP_OPTION_ACCEPT))
                    {
                        format = utils_convertMediaType((coap_content_type_t)message->accept[0]);
                    }
                    else
                    {
                        format = LWM2M_CONTENT_TLV;
                    }

                    if(dm_isUriOpaqueHandle(uriP))
                    {
                        buffer = NULL;
                        length = 0;
                        format = LWM2M_CONTENT_OPAQUE;
                    }
                    else
                    {
                        res = lwm2m_data_serialize(uriP, size, dataP, &format, &buffer);
                        if (res < 0)
                        {
                            result = COAP_500_INTERNAL_SERVER_ERROR;
                        }
                        else
                        {
                            length = (size_t)res;
                            LOG_ARG("Observe Request[/%d/%d/%d]: %.*s\n", uriP->objectId, uriP->instanceId, uriP->resourceId, length, buffer);
                        }
                    }
                }
                lwm2m_data_free(size, dataP);
            }
        }
        else if (IS_OPTION(message, COAP_OPTION_ACCEPT)
                 && message->accept_num == 1
                 && message->accept[0] == APPLICATION_LINK_FORMAT)
        {
            if ((result = acc_auth_operate(contextP, uriP, OBJ_ACC_DISCOVER, serverP->shortID)) != COAP_NO_ERROR)
            {
                return result;
            }
            format = LWM2M_CONTENT_LINK;
            result = object_discover(contextP, uriP, serverP, &buffer, &length);
        }
        else
        {
            if ((result = acc_auth_operate(contextP, uriP, OBJ_ACC_READ, serverP->shortID)) != COAP_NO_ERROR)
            {
                return result;
            }
            if (IS_OPTION(message, COAP_OPTION_ACCEPT))
            {
                format = utils_convertMediaType((coap_content_type_t)message->accept[0]);
            }

            result = object_read(contextP, uriP, &format, &buffer, &length, serverP->shortID);
        }
        if (COAP_205_CONTENT == result)
        {
            coap_set_header_content_type(response, format);
            coap_set_payload(response, buffer, length);
            // lwm2m_handle_packet will free buffer
        }
        else
        {
            lwm2m_free(buffer);
        }
    }
    break;

    case COAP_POST:
    {
        if (!LWM2M_URI_IS_SET_INSTANCE(uriP))
        {
            if ((result = acc_auth_operate(contextP, uriP, OBJ_ACC_CREATE, serverP->shortID)) != COAP_NO_ERROR)
            {
                return result;
            }
            result = object_create(contextP, uriP, format, message->payload, message->payload_len);
            if (result == COAP_201_CREATED)
            {
                //longest uri is /65535/65535 = 12 + 1 (null) chars
                char location_path[13] = "";
                //instanceId expected
                if ((uriP->flag & LWM2M_URI_FLAG_INSTANCE_ID) == 0)
                {
                    result = COAP_500_INTERNAL_SERVER_ERROR;
                    break;
                }

                if (sprintf(location_path, "/%d/%d", uriP->objectId, uriP->instanceId) < 0)
                {
                    result = COAP_500_INTERNAL_SERVER_ERROR;
                    break;
                }
                coap_set_header_location_path(response, location_path);

                lwm2m_update_registration(contextP, 0, true);
            }
        }
        else if (!LWM2M_URI_IS_SET_RESOURCE(uriP))
        {
            if ((result = acc_auth_operate(contextP, uriP, OBJ_ACC_WRITE, serverP->shortID)) != COAP_NO_ERROR)
            {
                return result;
            }
            result = object_write(contextP, uriP, format, message->payload, message->payload_len);
        }
        else
        {
            if ((result = acc_auth_operate(contextP, uriP, OBJ_ACC_EXCUTE, serverP->shortID)) != COAP_NO_ERROR)
            {
                return result;
            }
            result = object_execute(contextP, uriP, message->payload, message->payload_len);
        }

    }
    break;

    case COAP_PUT:
    {
        if (IS_OPTION(message, COAP_OPTION_URI_QUERY))
        {
            lwm2m_attributes_t attr;

            if ((result = acc_auth_operate(contextP, uriP, OBJ_ACC_WRITE_ATTR, serverP->shortID)) != COAP_NO_ERROR)
            {
                return result;
            }
            if (0 != prv_readAttributes(message->uri_query, &attr))
            {
                result = COAP_400_BAD_REQUEST;
            }
            else
            {
                result = observe_setParameters(contextP, uriP, serverP, &attr);
            }
        }
        else if (LWM2M_URI_IS_SET_INSTANCE(uriP))
        {
            if ((result = acc_auth_operate(contextP, uriP, OBJ_ACC_WRITE, serverP->shortID)) != COAP_NO_ERROR)
            {
                return result;
            }
            result = object_write(contextP, uriP, format, message->payload, message->payload_len);
        }
        else
        {
            result = COAP_400_BAD_REQUEST;
        }
    }
    break;

    case COAP_DELETE:
    {
        if (!LWM2M_URI_IS_SET_INSTANCE(uriP) || LWM2M_URI_IS_SET_RESOURCE(uriP))
        {
            result = COAP_400_BAD_REQUEST;
        }
        else
        {
            if ((result = acc_auth_operate(contextP, uriP, OBJ_ACC_DELETE, serverP->shortID)) != COAP_NO_ERROR)
            {
                return result;
            }
            result = object_delete(contextP, uriP);
            if (result == COAP_202_DELETED)
            {
                lwm2m_update_registration(contextP, 0, true);
            }
        }
    }
    break;

    default:
        result = COAP_400_BAD_REQUEST;
        break;
    }

    return result;
}

#endif

#ifdef LWM2M_SERVER_MODE

#define ID_AS_STRING_MAX_LEN 8

static void prv_resultCallback(lwm2m_transaction_t *transacP,
                               void *message)
{
    dm_data_t *dataP = (dm_data_t *)transacP->userData;

    if (message == NULL)
    {
        dataP->callback(dataP->clientID,
                        &dataP->uri,
                        COAP_503_SERVICE_UNAVAILABLE,
                        LWM2M_CONTENT_TEXT, NULL, 0,
                        dataP->userData);
    }
    else
    {
        coap_packet_t *packet = (coap_packet_t *)message;

        //if packet is a CREATE response and the instanceId was assigned by the client
        if (packet->code == COAP_201_CREATED
                && packet->location_path != NULL)
        {
            char *locationString = NULL;
            int result = 0;
            lwm2m_uri_t locationUri;

            locationString = coap_get_multi_option_as_string(packet->location_path);
            if (locationString == NULL)
            {
                LOG("Error: coap_get_multi_option_as_string() failed for Location_path option in prv_resultCallback()");
                return;
            }

            result = lwm2m_stringToUri(locationString, strlen(locationString), &locationUri);
            if (result == 0)
            {
                LOG("Error: lwm2m_stringToUri() failed for Location_path option in prv_resultCallback()");
                lwm2m_free(locationString);
                return;
            }

            ((dm_data_t *)transacP->userData)->uri.instanceId = locationUri.instanceId;
            ((dm_data_t *)transacP->userData)->uri.flag = locationUri.flag;

            lwm2m_free(locationString);
        }

        dataP->callback(dataP->clientID,
                        &dataP->uri,
                        packet->code,
                        utils_convertMediaType(packet->content_type),
                        packet->payload,
                        packet->payload_len,
                        dataP->userData);
    }
    lwm2m_free(dataP);
}

static int prv_makeOperation(lwm2m_context_t *contextP,
                             uint16_t clientID,
                             lwm2m_uri_t *uriP,
                             coap_method_t method,
                             lwm2m_media_type_t format,
                             uint8_t *buffer,
                             int length,
                             lwm2m_result_callback_t callback,
                             void *userData)
{
    lwm2m_client_t *clientP;
    lwm2m_transaction_t *transaction;
    dm_data_t *dataP;

    clientP = (lwm2m_client_t *)lwm2m_list_find((lwm2m_list_t *)contextP->clientList, clientID);
    if (clientP == NULL) return COAP_404_NOT_FOUND;

    transaction = transaction_new(clientP->sessionH, method, clientP->altPath, uriP, contextP->nextMID++, 4, NULL);
    if (transaction == NULL) return COAP_500_INTERNAL_SERVER_ERROR;

    if (method == COAP_GET)
    {
        coap_set_header_accept(transaction->message, format);
    }
    else if (buffer != NULL)
    {
        coap_set_header_content_type(transaction->message, format);
        // TODO: Take care of fragmentation
        coap_set_payload(transaction->message, buffer, length);
    }

    if (callback != NULL)
    {
        dataP = (dm_data_t *)lwm2m_malloc(sizeof(dm_data_t));
        if (dataP == NULL)
        {
            transaction_free(transaction);
            return COAP_500_INTERNAL_SERVER_ERROR;
        }
        memcpy(&dataP->uri, uriP, sizeof(lwm2m_uri_t));
        dataP->clientID = clientP->internalID;
        dataP->callback = callback;
        dataP->userData = userData;

        transaction->callback = prv_resultCallback;
        transaction->userData = (void *)dataP;
    }

    contextP->transactionList = (lwm2m_transaction_t *)LWM2M_LIST_ADD(contextP->transactionList, transaction);

    return transaction_send(contextP, transaction);
}

int lwm2m_dm_read(lwm2m_context_t *contextP,
                  uint16_t clientID,
                  lwm2m_uri_t *uriP,
                  lwm2m_result_callback_t callback,
                  void *userData)
{
    lwm2m_client_t *clientP;
    lwm2m_media_type_t format;

    LOG_ARG("clientID: %d", clientID);
    LOG_URI(uriP);

    clientP = (lwm2m_client_t *)lwm2m_list_find((lwm2m_list_t *)contextP->clientList, clientID);
    if (clientP == NULL) return COAP_404_NOT_FOUND;

    if (clientP->supportJSON == true)
    {
        format = LWM2M_CONTENT_JSON;
    }
    else
    {
        format = LWM2M_CONTENT_TLV;
    }

    return prv_makeOperation(contextP, clientID, uriP,
                             COAP_GET,
                             format,
                             NULL, 0,
                             callback, userData);
}

int lwm2m_dm_write(lwm2m_context_t *contextP,
                   uint16_t clientID,
                   lwm2m_uri_t *uriP,
                   lwm2m_media_type_t format,
                   uint8_t *buffer,
                   int length,
                   lwm2m_result_callback_t callback,
                   void *userData)
{
    LOG_ARG("clientID: %d, format: %s, length: %d", clientID, STR_MEDIA_TYPE(format), length);
    LOG_URI(uriP);
    if (!LWM2M_URI_IS_SET_INSTANCE(uriP)
            || length == 0)
    {
        return COAP_400_BAD_REQUEST;
    }

    if (LWM2M_URI_IS_SET_RESOURCE(uriP))
    {
        return prv_makeOperation(contextP, clientID, uriP,
                                 COAP_PUT,
                                 format, buffer, length,
                                 callback, userData);
    }
    else
    {
        return prv_makeOperation(contextP, clientID, uriP,
                                 COAP_POST,
                                 format, buffer, length,
                                 callback, userData);
    }
}

int lwm2m_dm_execute(lwm2m_context_t *contextP,
                     uint16_t clientID,
                     lwm2m_uri_t *uriP,
                     lwm2m_media_type_t format,
                     uint8_t *buffer,
                     int length,
                     lwm2m_result_callback_t callback,
                     void *userData)
{
    LOG_ARG("clientID: %d, format: %s, length: %d", clientID, STR_MEDIA_TYPE(format), length);
    LOG_URI(uriP);
    if (!LWM2M_URI_IS_SET_RESOURCE(uriP))
    {
        return COAP_400_BAD_REQUEST;
    }

    return prv_makeOperation(contextP, clientID, uriP,
                             COAP_POST,
                             format, buffer, length,
                             callback, userData);
}

int lwm2m_dm_create(lwm2m_context_t *contextP,
                    uint16_t clientID,
                    lwm2m_uri_t *uriP,
                    lwm2m_media_type_t format,
                    uint8_t *buffer,
                    int length,
                    lwm2m_result_callback_t callback,
                    void *userData)
{
    LOG_ARG("clientID: %d, format: %s, length: %d", clientID, STR_MEDIA_TYPE(format), length);
    LOG_URI(uriP);

    if (LWM2M_URI_IS_SET_INSTANCE(uriP)
            || length == 0)
    {
        return COAP_400_BAD_REQUEST;
    }

    return prv_makeOperation(contextP, clientID, uriP,
                             COAP_POST,
                             format, buffer, length,
                             callback, userData);
}

int lwm2m_dm_delete(lwm2m_context_t *contextP,
                    uint16_t clientID,
                    lwm2m_uri_t *uriP,
                    lwm2m_result_callback_t callback,
                    void *userData)
{
    LOG_ARG("clientID: %d", clientID);
    LOG_URI(uriP);
    if (!LWM2M_URI_IS_SET_INSTANCE(uriP)
            || LWM2M_URI_IS_SET_RESOURCE(uriP))
    {
        return COAP_400_BAD_REQUEST;
    }

    return prv_makeOperation(contextP, clientID, uriP,
                             COAP_DELETE,
                             LWM2M_CONTENT_TEXT, NULL, 0,
                             callback, userData);
}

int lwm2m_dm_write_attributes(lwm2m_context_t *contextP,
                              uint16_t clientID,
                              lwm2m_uri_t *uriP,
                              lwm2m_attributes_t *attrP,
                              lwm2m_result_callback_t callback,
                              void *userData)
{
#define _PRV_BUFFER_SIZE 32
    lwm2m_client_t *clientP;
    lwm2m_transaction_t *transaction;
    coap_packet_t *coap_pkt;
    uint8_t buffer[_PRV_BUFFER_SIZE];
    size_t length;

    LOG_ARG("clientID: %d", clientID);
    LOG_URI(uriP);
    if (attrP == NULL) return COAP_400_BAD_REQUEST;

    if (0 != (attrP->toSet & attrP->toClear)) return COAP_400_BAD_REQUEST;
    if (0 != (attrP->toSet & ATTR_FLAG_NUMERIC) && !LWM2M_URI_IS_SET_RESOURCE(uriP)) return COAP_400_BAD_REQUEST;
    if (ATTR_FLAG_NUMERIC == (attrP->toSet & ATTR_FLAG_NUMERIC)
            && (attrP->lessThan + 2 * attrP->step >= attrP->greaterThan)) return COAP_400_BAD_REQUEST;

    clientP = (lwm2m_client_t *)lwm2m_list_find((lwm2m_list_t *)contextP->clientList, clientID);
    if (clientP == NULL) return COAP_404_NOT_FOUND;

    transaction = transaction_new(clientP->sessionH, COAP_PUT, clientP->altPath, uriP, contextP->nextMID++, 4, NULL);
    if (transaction == NULL) return COAP_500_INTERNAL_SERVER_ERROR;

    if (callback != NULL)
    {
        dm_data_t *dataP;

        dataP = (dm_data_t *)lwm2m_malloc(sizeof(dm_data_t));
        if (dataP == NULL)
        {
            transaction_free(transaction);
            return COAP_500_INTERNAL_SERVER_ERROR;
        }
        memcpy(&dataP->uri, uriP, sizeof(lwm2m_uri_t));
        dataP->clientID = clientP->internalID;
        dataP->callback = callback;
        dataP->userData = userData;

        transaction->callback = prv_resultCallback;
        transaction->userData = (void *)dataP;
    }

    coap_pkt = (coap_packet_t *)transaction->message;
    free_multi_option(coap_pkt->uri_query);
    if (attrP->toSet & LWM2M_ATTR_FLAG_MIN_PERIOD)
    {
        memcpy(buffer, ATTR_MIN_PERIOD_STR, ATTR_MIN_PERIOD_LEN);
        length = utils_intToText(attrP->minPeriod, buffer + ATTR_MIN_PERIOD_LEN, _PRV_BUFFER_SIZE - ATTR_MIN_PERIOD_LEN);
        if (length == 0)
        {
            transaction_free(transaction);
            return COAP_500_INTERNAL_SERVER_ERROR;
        }
        coap_add_multi_option(&(coap_pkt->uri_query), buffer, ATTR_MIN_PERIOD_LEN + length, 0);
        SET_OPTION(coap_pkt, COAP_OPTION_URI_QUERY);
    }
    if (attrP->toSet & LWM2M_ATTR_FLAG_MAX_PERIOD)
    {
        memcpy(buffer, ATTR_MAX_PERIOD_STR, ATTR_MAX_PERIOD_LEN);
        length = utils_intToText(attrP->maxPeriod, buffer + ATTR_MAX_PERIOD_LEN, _PRV_BUFFER_SIZE - ATTR_MAX_PERIOD_LEN);
        if (length == 0)
        {
            transaction_free(transaction);
            return COAP_500_INTERNAL_SERVER_ERROR;
        }
        coap_add_multi_option(&(coap_pkt->uri_query), buffer, ATTR_MAX_PERIOD_LEN + length, 0);
        SET_OPTION(coap_pkt, COAP_OPTION_URI_QUERY);
    }
    if (attrP->toSet & LWM2M_ATTR_FLAG_GREATER_THAN)
    {
        memcpy(buffer, ATTR_GREATER_THAN_STR, ATTR_GREATER_THAN_LEN);
        length = utils_floatToText(attrP->greaterThan, buffer + ATTR_GREATER_THAN_LEN, _PRV_BUFFER_SIZE - ATTR_GREATER_THAN_LEN);
        if (length == 0)
        {
            transaction_free(transaction);
            return COAP_500_INTERNAL_SERVER_ERROR;
        }
        coap_add_multi_option(&(coap_pkt->uri_query), buffer, ATTR_GREATER_THAN_LEN + length, 0);
        SET_OPTION(coap_pkt, COAP_OPTION_URI_QUERY);
    }
    if (attrP->toSet & LWM2M_ATTR_FLAG_LESS_THAN)
    {
        memcpy(buffer, ATTR_LESS_THAN_STR, ATTR_LESS_THAN_LEN);
        length = utils_floatToText(attrP->lessThan, buffer + ATTR_LESS_THAN_LEN, _PRV_BUFFER_SIZE - ATTR_LESS_THAN_LEN);
        if (length == 0)
        {
            transaction_free(transaction);
            return COAP_500_INTERNAL_SERVER_ERROR;
        }
        coap_add_multi_option(&(coap_pkt->uri_query), buffer, ATTR_LESS_THAN_LEN + length, 0);
        SET_OPTION(coap_pkt, COAP_OPTION_URI_QUERY);
    }
    if (attrP->toSet & LWM2M_ATTR_FLAG_STEP)
    {
        memcpy(buffer, ATTR_STEP_STR, ATTR_STEP_LEN);
        length = utils_floatToText(attrP->step, buffer + ATTR_STEP_LEN, _PRV_BUFFER_SIZE - ATTR_STEP_LEN);
        if (length == 0)
        {
            transaction_free(transaction);
            return COAP_500_INTERNAL_SERVER_ERROR;
        }
        coap_add_multi_option(&(coap_pkt->uri_query), buffer, ATTR_STEP_LEN + length, 0);
        SET_OPTION(coap_pkt, COAP_OPTION_URI_QUERY);
    }
    if (attrP->toClear & LWM2M_ATTR_FLAG_MIN_PERIOD)
    {
        coap_add_multi_option(&(coap_pkt->uri_query), (uint8_t *)ATTR_MIN_PERIOD_STR, ATTR_MIN_PERIOD_LEN - 1, 0);
        SET_OPTION(coap_pkt, COAP_OPTION_URI_QUERY);
    }
    if (attrP->toClear & LWM2M_ATTR_FLAG_MAX_PERIOD)
    {
        coap_add_multi_option(&(coap_pkt->uri_query), (uint8_t *)ATTR_MAX_PERIOD_STR, ATTR_MAX_PERIOD_LEN - 1, 0);
        SET_OPTION(coap_pkt, COAP_OPTION_URI_QUERY);
    }
    if (attrP->toClear & LWM2M_ATTR_FLAG_GREATER_THAN)
    {
        coap_add_multi_option(&(coap_pkt->uri_query), (uint8_t *)ATTR_GREATER_THAN_STR, ATTR_GREATER_THAN_LEN - 1, 0);
        SET_OPTION(coap_pkt, COAP_OPTION_URI_QUERY);
    }
    if (attrP->toClear & LWM2M_ATTR_FLAG_LESS_THAN)
    {
        coap_add_multi_option(&(coap_pkt->uri_query), (uint8_t *)ATTR_LESS_THAN_STR, ATTR_LESS_THAN_LEN - 1, 0);
        SET_OPTION(coap_pkt, COAP_OPTION_URI_QUERY);
    }
    if (attrP->toClear & LWM2M_ATTR_FLAG_STEP)
    {
        coap_add_multi_option(&(coap_pkt->uri_query), (uint8_t *)ATTR_STEP_STR, ATTR_STEP_LEN - 1, 0);
        SET_OPTION(coap_pkt, COAP_OPTION_URI_QUERY);
    }

    contextP->transactionList = (lwm2m_transaction_t *)LWM2M_LIST_ADD(contextP->transactionList, transaction);

    return transaction_send(contextP, transaction);
}

int lwm2m_dm_discover(lwm2m_context_t *contextP,
                      uint16_t clientID,
                      lwm2m_uri_t *uriP,
                      lwm2m_result_callback_t callback,
                      void *userData)
{
    lwm2m_client_t *clientP;
    lwm2m_transaction_t *transaction;
    dm_data_t *dataP;

    LOG_ARG("clientID: %d", clientID);
    LOG_URI(uriP);
    clientP = (lwm2m_client_t *)lwm2m_list_find((lwm2m_list_t *)contextP->clientList, clientID);
    if (clientP == NULL) return COAP_404_NOT_FOUND;

    transaction = transaction_new(clientP->sessionH, COAP_GET, clientP->altPath, uriP, contextP->nextMID++, 4, NULL);
    if (transaction == NULL) return COAP_500_INTERNAL_SERVER_ERROR;

    coap_set_header_accept(transaction->message, LWM2M_CONTENT_LINK);

    if (callback != NULL)
    {
        dataP = (dm_data_t *)lwm2m_malloc(sizeof(dm_data_t));
        if (dataP == NULL)
        {
            transaction_free(transaction);
            return COAP_500_INTERNAL_SERVER_ERROR;
        }
        memcpy(&dataP->uri, uriP, sizeof(lwm2m_uri_t));
        dataP->clientID = clientP->internalID;
        dataP->callback = callback;
        dataP->userData = userData;

        transaction->callback = prv_resultCallback;
        transaction->userData = (void *)dataP;
    }

    contextP->transactionList = (lwm2m_transaction_t *)LWM2M_LIST_ADD(contextP->transactionList, transaction);

    return transaction_send(contextP, transaction);
}

#endif
