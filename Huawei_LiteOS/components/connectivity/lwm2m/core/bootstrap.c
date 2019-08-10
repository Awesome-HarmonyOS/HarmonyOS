/*******************************************************************************
 *
 * Copyright (c) 2015 Sierra Wireless and others.
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
 *    Pascal Rieux - Please refer to git log
 *    Bosch Software Innovations GmbH - Please refer to git log
 *    David Navarro, Intel Corporation - Please refer to git log
 *
 *******************************************************************************/

#include "internals.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifdef LWM2M_CLIENT_MODE
#ifdef LWM2M_BOOTSTRAP

#define PRV_QUERY_BUFFER_LENGTH 200


static void prv_handleResponse(lwm2m_server_t *bootstrapServer,
                               coap_packet_t *message)
{
    if (COAP_204_CHANGED == message->code)
    {
        LOG("Received ACK/2.04, Bootstrap pending, waiting for DEL/PUT from BS server...");
        bootstrapServer->status = STATE_BS_PENDING;
        bootstrapServer->registration = lwm2m_gettime() + COAP_EXCHANGE_LIFETIME;
    }
    else
    {
        bootstrapServer->status = STATE_BS_FAILING;
    }
}

static void prv_handleBootstrapReply(lwm2m_transaction_t *transaction,
                                     void *message)
{
    lwm2m_server_t *bootstrapServer = (lwm2m_server_t *)transaction->userData;
    coap_packet_t *coapMessage = (coap_packet_t *)message;

    LOG("Entering");

    if (bootstrapServer->status == STATE_BS_INITIATED)
    {
        if (NULL != coapMessage && COAP_TYPE_RST != coapMessage->type)
        {
            prv_handleResponse(bootstrapServer, coapMessage);
        }
        else
        {
            bootstrapServer->status = STATE_BS_FAILING;
        }
    }
}

// start a device initiated bootstrap
static void prv_requestBootstrap(lwm2m_context_t *context,
                                 lwm2m_server_t *bootstrapServer)
{
    char query[PRV_QUERY_BUFFER_LENGTH];
    int query_length = 0;
    int res;

    LOG("Entering");

    query_length = utils_stringCopy(query, PRV_QUERY_BUFFER_LENGTH, QUERY_STARTER QUERY_NAME);
    if (query_length < 0)
    {
        bootstrapServer->status = STATE_BS_FAILING;
        return;
    }
    res = utils_stringCopy(query + query_length, PRV_QUERY_BUFFER_LENGTH - query_length, context->endpointName);
    if (res < 0)
    {
        bootstrapServer->status = STATE_BS_FAILING;
        return;
    }
    query_length += res;

    if (bootstrapServer->sessionH == NULL)
    {
        bootstrapServer->sessionH = lwm2m_connect_server(bootstrapServer->secObjInstID, context->userData, false);
    }

    if (bootstrapServer->sessionH != NULL)
    {
        lwm2m_transaction_t *transaction = NULL;

        LOG("Bootstrap server connection opened");

        transaction = transaction_new(bootstrapServer->sessionH, COAP_POST, NULL, NULL, context->nextMID++, 4, NULL);
        if (transaction == NULL)
        {
            bootstrapServer->status = STATE_BS_FAILING;
            return;
        }

        coap_set_header_uri_path(transaction->message, "/"URI_BOOTSTRAP_SEGMENT);
        coap_set_header_uri_query(transaction->message, query);
        transaction->callback = prv_handleBootstrapReply;
        transaction->userData = (void *)bootstrapServer;
        context->transactionList = (lwm2m_transaction_t *)LWM2M_LIST_ADD(context->transactionList, transaction);
        if (transaction_send(context, transaction) == 0)
        {
            LOG("CI bootstrap requested to BS server");
            bootstrapServer->status = STATE_BS_INITIATED;
        }
    }
    else
    {
        LOG("Connecting bootstrap server failed");
        bootstrapServer->status = STATE_BS_FAILED;
    }
}


static void prv_resetBootstrapServer(lwm2m_server_t *serverP, void *userData)
{
     if (serverP->sessionH != NULL)
     {
         lwm2m_close_connection(serverP->sessionH, userData);
         serverP->sessionH = NULL;
     }
     free_block1_buffer(serverP->block1Data);
     serverP->block1Data = NULL;
}


static void prv_createBsConnection(lwm2m_context_t *contextP, lwm2m_server_t *targetP)
{

    prv_resetBootstrapServer(targetP, contextP->userData);

    if (lwm2m_isBsCtrlInServerInitiatedBs(contextP))
    {
        if (targetP != NULL)
        {
            targetP->sessionH = lwm2m_connect_server(targetP->secObjInstID, contextP->userData, true);
        }
        return;
    }

    targetP->sessionH = lwm2m_connect_server(targetP->secObjInstID, contextP->userData, false);
}



/*
 * modify info:
 * modify date:     2019-06-04
 * modify place:    STATE_BS_HOLD_OFF, STATE_BS_PENDING
 * pay attention:
 *                  the targetP->status are important, change one, should take care of other status.
 *
 */
void bootstrap_step(lwm2m_context_t *contextP,
                    uint32_t currentTime,
                    time_t *timeoutP)
{
    lwm2m_server_t *targetP;

    LOG("entering");
    targetP = contextP->bootstrapServerList;
    while (targetP != NULL)
    {
        time_t timeOut;

        LOG_ARG("Initial status: %s", STR_STATUS(targetP->status));
        switch (targetP->status)
        {
        case STATE_DEREGISTERED:
            //pay attention: targetP->lifetime come from securityobj instance, it's init value is 10. too small, too short for
            //STATE_BS_HOLD_OFF. of course, the value could be wrote by the bs server. encode decode used for securityobj/serverobj instance

            targetP->registration =  currentTime;
            if(lwm2m_isBsCtrlInServerInitiatedBs(contextP))
            {
                targetP->registration = (currentTime + targetP->lifetime);
            }
            prv_createBsConnection(contextP, targetP);
            currentTime = lwm2m_gettime();
            targetP->status = STATE_BS_HOLD_OFF;

            timeOut = (targetP->registration - currentTime);
            if (timeOut < 0)
            {
                timeOut = 0;
            }
            if (*timeoutP > timeOut)
            {
                *timeoutP = timeOut;
            }
            break;

        case STATE_BS_HOLD_OFF:
            if (targetP->registration <= currentTime)
            {
                if (lwm2m_isBsCtrlInServerInitiatedBs(contextP))
                {
                    lwm2m_stop_striger_server_initiated_bs(targetP->sessionH);
                    targetP->status = STATE_REG_FAILED;
                }
                else
                {
                    prv_requestBootstrap(contextP, targetP);
                }
            }
            else if (*timeoutP > targetP->registration - currentTime)
            {
                *timeoutP = targetP->registration - currentTime;
                 lwm2m_step_striger_server_initiated_bs(targetP->sessionH);
            }
            break;


        case STATE_BS_INITIATED:
            // waiting
            break;

        case STATE_BS_PENDING:
            if (targetP->registration <= currentTime)
            {
               targetP->status = STATE_BS_FAILING;
               *timeoutP = 0;
            }
            else if (*timeoutP > targetP->registration - currentTime)
            {
                *timeoutP = targetP->registration - currentTime;
            }
            break;

        case STATE_BS_FINISHING:
            if (targetP->sessionH != NULL)
            {
                lwm2m_close_connection(targetP->sessionH, contextP->userData);
                targetP->sessionH = NULL;
            }
            targetP->status = STATE_BS_FINISHED;
            *timeoutP = 0;
            break;

        case STATE_BS_FAILING:
            if (targetP->sessionH != NULL)
            {
                lwm2m_close_connection(targetP->sessionH, contextP->userData);
                targetP->sessionH = NULL;
            }
            targetP->status = STATE_BS_FAILED;
            *timeoutP = 0;
            break;

        default:
            break;
        }
        LOG_ARG("Finalal status: %s", STR_STATUS(targetP->status));

        if (lwm2m_isBsCtrlInServerInitiatedBs(contextP))
        {
            return;
        }
        targetP = targetP->next;
    }


}

uint8_t bootstrap_handleFinish(lwm2m_context_t *context,
                               void *fromSessionH)
{
    lwm2m_server_t *bootstrapServer;

    LOG("Entering");
    bootstrapServer = utils_findBootstrapServer(context, fromSessionH);
    if (bootstrapServer != NULL
            && bootstrapServer->status == STATE_BS_PENDING)
    {
        if (object_getServers(context, true) == 0)
        {
            LOG("Bootstrap server status changed to STATE_BS_FINISHING");
            bootstrapServer->status = STATE_BS_FINISHING;
            return COAP_204_CHANGED;
        }
        else
        {
            return COAP_406_NOT_ACCEPTABLE;
        }
    }

    return COAP_IGNORE;
}
bool bootstrap_isBsServerIpValid(const lwm2m_context_t *contextP)
{
    lwm2m_server_t *targetP;

    targetP = contextP->bootstrapServerList;
    while (targetP != NULL)
    {
        if(lwm2m_is_sec_obj_uri_valid(targetP->secObjInstID, contextP->userData))
        {
            return true;
        }
        targetP = targetP->next;
    }

    return false;
}


/*
 * Reset the bootstrap servers statuses
 *
 * TODO: handle LWM2M Servers the client is registered to ?
 *
 */
void bootstrap_start(lwm2m_context_t *contextP)
{
    lwm2m_server_t *targetP;

    LOG("Entering");
    targetP = contextP->bootstrapServerList;
    while (targetP != NULL)
    {
        targetP->status = STATE_DEREGISTERED;
        targetP = targetP->next;
    }
}

/*
 * Returns STATE_BS_PENDING if at least one bootstrap is still pending
 * Returns STATE_BS_FINISHED if at least one bootstrap succeeded and no bootstrap is pending
 * Returns STATE_BS_FAILED if all bootstrap failed.
 */
lwm2m_status_t bootstrap_getStatus(lwm2m_context_t *contextP)
{
    lwm2m_server_t *targetP;
    lwm2m_status_t bs_status;

    LOG("Entering");
    targetP = contextP->bootstrapServerList;
    bs_status = STATE_BS_FAILED;

    while (targetP != NULL)
    {
        switch (targetP->status)
        {
        case STATE_BS_FINISHED:
            if (bs_status == STATE_BS_FAILED)
            {
                bs_status = STATE_BS_FINISHED;
            }
            break;

        case STATE_BS_HOLD_OFF:
        case STATE_BS_INITIATED:
        case STATE_BS_PENDING:
        case STATE_BS_FINISHING:
        case STATE_BS_FAILING:
            bs_status = STATE_BS_PENDING;
            break;

        default:
            break;
        }
        targetP = targetP->next;
    }

    LOG_ARG("Returned status: %s", STR_STATUS(bs_status));

    return bs_status;
}

static uint8_t prv_checkServerStatus(lwm2m_server_t *serverP)
{
    LOG_ARG("Initial status: %s", STR_STATUS(serverP->status));

    switch (serverP->status)
    {
    case STATE_BS_HOLD_OFF:
        serverP->status = STATE_BS_PENDING;
        LOG_ARG("Status changed to: %s", STR_STATUS(serverP->status));
        break;

    case STATE_BS_INITIATED:
        // The ACK was probably lost
        serverP->status = STATE_BS_PENDING;
        LOG_ARG("Status changed to: %s", STR_STATUS(serverP->status));
        break;

    case STATE_DEREGISTERED:
    // server initiated bootstrap
    case STATE_BS_PENDING:
        serverP->registration = lwm2m_gettime() + COAP_EXCHANGE_LIFETIME;
        break;

    case STATE_BS_FINISHED:
    case STATE_BS_FINISHING:
    case STATE_BS_FAILING:
    case STATE_BS_FAILED:
    default:
        LOG("Returning COAP_IGNORE");
        return COAP_IGNORE;
    }

    return COAP_NO_ERROR;
}

static void prv_tagServer(lwm2m_context_t *contextP,
                          uint16_t id)
{
    lwm2m_server_t *targetP;

    targetP = (lwm2m_server_t *)LWM2M_LIST_FIND(contextP->bootstrapServerList, id);
    if (targetP == NULL)
    {
        targetP = (lwm2m_server_t *)LWM2M_LIST_FIND(contextP->serverList, id);
    }
    if (targetP != NULL)
    {
        targetP->dirty = true;
    }
}

static void prv_tagAllServer(lwm2m_context_t *contextP,
                             lwm2m_server_t *serverP)
{
    lwm2m_server_t *targetP;

    targetP = contextP->bootstrapServerList;
    while (targetP != NULL)
    {
        if (targetP != serverP)
        {
            targetP->dirty = true;
        }
        targetP = targetP->next;
    }
    targetP = contextP->serverList;
    while (targetP != NULL)
    {
        targetP->dirty = true;
        targetP = targetP->next;
    }
}

uint8_t bootstrap_handleCommand(lwm2m_context_t *contextP,
                                lwm2m_uri_t *uriP,
                                lwm2m_server_t *serverP,
                                coap_packet_t *message,
                                coap_packet_t *response)
{
    uint8_t result;
    lwm2m_media_type_t format;

    LOG_ARG("Code: %02X", message->code);
    LOG_URI(uriP);
    format = utils_convertMediaType(message->content_type);

    result = prv_checkServerStatus(serverP);
    if (result != COAP_NO_ERROR) return result;

    switch (message->code)
    {
    case COAP_PUT:
    {
        if (LWM2M_URI_IS_SET_INSTANCE(uriP))
        {
            if (object_isInstanceNew(contextP, uriP->objectId, uriP->instanceId))
            {
                result = object_create(contextP, uriP, format, message->payload, message->payload_len);
                if (COAP_201_CREATED == result)
                {
                    result = COAP_204_CHANGED;
                }
            }
            else
            {
                result = object_write(contextP, uriP, format, message->payload, message->payload_len);
                if (uriP->objectId == LWM2M_SECURITY_OBJECT_ID
                        && result == COAP_204_CHANGED)
                {
                    prv_tagServer(contextP, uriP->instanceId);
                }
            }
        }
        else
        {
            lwm2m_data_t *dataP = NULL;
            int size = 0;
            int i;

            if (message->payload_len == 0 || message->payload == 0)
            {
                result = COAP_400_BAD_REQUEST;
            }
            else
            {
                size = lwm2m_data_parse(uriP, message->payload, message->payload_len, format, &dataP);
                if (size == 0)
                {
                    result = COAP_500_INTERNAL_SERVER_ERROR;
                    break;
                }

                for (i = 0 ; i < size ; i++)
                {
                    if(dataP[i].type == LWM2M_TYPE_OBJECT_INSTANCE)
                    {
                        if (object_isInstanceNew(contextP, uriP->objectId, dataP[i].id))
                        {
                            result = object_createInstance(contextP, uriP, &dataP[i]);
                            if (COAP_201_CREATED == result)
                            {
                                result = COAP_204_CHANGED;
                            }
                        }
                        else
                        {
                            result = object_writeInstance(contextP, uriP, &dataP[i]);
                            if (uriP->objectId == LWM2M_SECURITY_OBJECT_ID
                                    && result == COAP_204_CHANGED)
                            {
                                prv_tagServer(contextP, dataP[i].id);
                            }
                        }

                        if(result != COAP_204_CHANGED) // Stop object create or write when result is error
                        {
                            break;
                        }
                    }
                    else
                    {
                        result = COAP_400_BAD_REQUEST;
                    }
                }
                lwm2m_data_free(size, dataP);
            }
        }
    }
    break;

    case COAP_DELETE:
    {
        if (LWM2M_URI_IS_SET_RESOURCE(uriP))
        {
            result = COAP_400_BAD_REQUEST;
        }
        else
        {
            result = object_delete(contextP, uriP);
            if (uriP->objectId == LWM2M_SECURITY_OBJECT_ID
                    && result == COAP_202_DELETED)
            {
                if (LWM2M_URI_IS_SET_INSTANCE(uriP))
                {
                    prv_tagServer(contextP, uriP->instanceId);
                }
                else
                {
                    prv_tagAllServer(contextP, NULL);
                }
            }
        }
    }
    break;

    case COAP_GET:
    case COAP_POST:
    default:
        result = COAP_400_BAD_REQUEST;
        break;
    }

    if (result == COAP_202_DELETED
            || result == COAP_204_CHANGED)
    {
        if (serverP->status != STATE_BS_PENDING)
        {
            serverP->status = STATE_BS_PENDING;
            contextP->state = STATE_BOOTSTRAPPING;
        }
        lwm2m_stop_striger_server_initiated_bs(serverP->sessionH);    }
    LOG_ARG("Server status: %s", STR_STATUS(serverP->status));

    return result;
}

uint8_t bootstrap_handleDeleteAll(lwm2m_context_t *contextP,
                                  void *fromSessionH)
{
    lwm2m_server_t *serverP;
    uint8_t result;
    lwm2m_object_t *objectP;

    LOG("Entering");
    serverP = utils_findBootstrapServer(contextP, fromSessionH);
    if (serverP == NULL) return COAP_IGNORE;
    result = prv_checkServerStatus(serverP);
    if (result != COAP_NO_ERROR) return result;

    result = COAP_202_DELETED;
    for (objectP = contextP->objectList; objectP != NULL; objectP = objectP->next)
    {
        lwm2m_uri_t uri;

        memset(&uri, 0, sizeof(lwm2m_uri_t));
        uri.flag = LWM2M_URI_FLAG_OBJECT_ID;
        uri.objectId = objectP->objID;

        if (objectP->objID == LWM2M_SECURITY_OBJECT_ID)
        {
            lwm2m_list_t *instanceP;

            instanceP = objectP->instanceList;
            while (NULL != instanceP
                    && result == COAP_202_DELETED)
            {
                if (instanceP->id == serverP->secObjInstID)
                {
                    instanceP = instanceP->next;
                }
                else
                {
                    uri.flag = LWM2M_URI_FLAG_OBJECT_ID | LWM2M_URI_FLAG_INSTANCE_ID;
                    uri.instanceId = instanceP->id;
                    result = object_delete(contextP, &uri);
                    instanceP = objectP->instanceList;
                }
            }
            if (result == COAP_202_DELETED)
            {
                prv_tagAllServer(contextP, serverP);
            }
        }
        else
        {
            result = object_delete(contextP, &uri);
            if (result == COAP_405_METHOD_NOT_ALLOWED)
            {
                // Fake a successful deletion for static objects like the Device object.
                result = COAP_202_DELETED;
            }
        }
    }

    return result;
}
#endif
#endif

#ifdef LWM2M_BOOTSTRAP_SERVER_MODE
uint8_t bootstrap_handleRequest(lwm2m_context_t *contextP,
                                lwm2m_uri_t *uriP,
                                void *fromSessionH,
                                coap_packet_t *message,
                                coap_packet_t *response)
{
    uint8_t result;
    char *name;

    LOG_URI(uriP);
    if (contextP->bootstrapCallback == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
    if (message->code != COAP_POST) return COAP_400_BAD_REQUEST;
    if (message->uri_query == NULL) return COAP_400_BAD_REQUEST;
    if (message->payload != NULL) return COAP_400_BAD_REQUEST;

    if (lwm2m_strncmp((char *)message->uri_query->data, QUERY_NAME, QUERY_NAME_LEN) != 0)
    {
        return COAP_400_BAD_REQUEST;
    }

    if (message->uri_query->len == QUERY_NAME_LEN) return COAP_400_BAD_REQUEST;
    if (message->uri_query->next != NULL) return COAP_400_BAD_REQUEST;

    name = (char *)lwm2m_malloc(message->uri_query->len - QUERY_NAME_LEN + 1);
    if (name == NULL) return COAP_500_INTERNAL_SERVER_ERROR;

    memcpy(name, message->uri_query->data + QUERY_NAME_LEN, message->uri_query->len - QUERY_NAME_LEN);
    name[message->uri_query->len - QUERY_NAME_LEN] = 0;

    result = contextP->bootstrapCallback(fromSessionH, COAP_NO_ERROR, NULL, name, contextP->bootstrapUserData);

    lwm2m_free(name);

    return result;
}

void lwm2m_set_bootstrap_callback(lwm2m_context_t *contextP,
                                  lwm2m_bootstrap_callback_t callback,
                                  void *userData)
{
    LOG("Entering");
    contextP->bootstrapCallback = callback;
    contextP->bootstrapUserData = userData;
}

static void prv_resultCallback(lwm2m_transaction_t *transacP,
                               void *message)
{
    bs_data_t *dataP = (bs_data_t *)transacP->userData;
    lwm2m_uri_t *uriP;

    if (dataP->isUri == true)
    {
        uriP = &dataP->uri;
    }
    else
    {
        uriP = NULL;
    }

    if (message == NULL)
    {
        dataP->callback(transacP->peerH,
                        COAP_503_SERVICE_UNAVAILABLE,
                        uriP,
                        NULL,
                        dataP->userData);
    }
    else
    {
        coap_packet_t *packet = (coap_packet_t *)message;

        dataP->callback(transacP->peerH,
                        packet->code,
                        uriP,
                        NULL,
                        dataP->userData);
    }
    lwm2m_free(dataP);
}

int lwm2m_bootstrap_delete(lwm2m_context_t *contextP,
                           void *sessionH,
                           lwm2m_uri_t *uriP)
{
    lwm2m_transaction_t *transaction;
    bs_data_t *dataP;

    LOG_URI(uriP);
    transaction = transaction_new(sessionH, COAP_DELETE, NULL, uriP, contextP->nextMID++, 4, NULL);
    if (transaction == NULL) return COAP_500_INTERNAL_SERVER_ERROR;

    dataP = (bs_data_t *)lwm2m_malloc(sizeof(bs_data_t));
    if (dataP == NULL)
    {
        transaction_free(transaction);
        return COAP_500_INTERNAL_SERVER_ERROR;
    }
    if (uriP == NULL)
    {
        dataP->isUri = false;
    }
    else
    {
        dataP->isUri = true;
        memcpy(&dataP->uri, uriP, sizeof(lwm2m_uri_t));
    }
    dataP->callback = contextP->bootstrapCallback;
    dataP->userData = contextP->bootstrapUserData;

    transaction->callback = prv_resultCallback;
    transaction->userData = (void *)dataP;

    contextP->transactionList = (lwm2m_transaction_t *)LWM2M_LIST_ADD(contextP->transactionList, transaction);

    return transaction_send(contextP, transaction);
}

int lwm2m_bootstrap_write(lwm2m_context_t *contextP,
                          void *sessionH,
                          lwm2m_uri_t *uriP,
                          lwm2m_media_type_t format,
                          uint8_t *buffer,
                          size_t length)
{
    lwm2m_transaction_t *transaction;
    bs_data_t *dataP;

    LOG_URI(uriP);
    if (uriP == NULL
            || buffer == NULL
            || length == 0)
    {
        return COAP_400_BAD_REQUEST;
    }

    transaction = transaction_new(sessionH, COAP_PUT, NULL, uriP, contextP->nextMID++, 4, NULL);
    if (transaction == NULL) return COAP_500_INTERNAL_SERVER_ERROR;

    coap_set_header_content_type(transaction->message, format);
    coap_set_payload(transaction->message, buffer, length);

    dataP = (bs_data_t *)lwm2m_malloc(sizeof(bs_data_t));
    if (dataP == NULL)
    {
        transaction_free(transaction);
        return COAP_500_INTERNAL_SERVER_ERROR;
    }
    dataP->isUri = true;
    memcpy(&dataP->uri, uriP, sizeof(lwm2m_uri_t));
    dataP->callback = contextP->bootstrapCallback;
    dataP->userData = contextP->bootstrapUserData;

    transaction->callback = prv_resultCallback;
    transaction->userData = (void *)dataP;

    contextP->transactionList = (lwm2m_transaction_t *)LWM2M_LIST_ADD(contextP->transactionList, transaction);

    return transaction_send(contextP, transaction);
}

int lwm2m_bootstrap_finish(lwm2m_context_t *contextP,
                           void *sessionH)
{
    lwm2m_transaction_t *transaction;
    bs_data_t *dataP;

    LOG("Entering");
    transaction = transaction_new(sessionH, COAP_POST, NULL, NULL, contextP->nextMID++, 4, NULL);
    if (transaction == NULL) return COAP_500_INTERNAL_SERVER_ERROR;

    coap_set_header_uri_path(transaction->message, "/"URI_BOOTSTRAP_SEGMENT);

    dataP = (bs_data_t *)lwm2m_malloc(sizeof(bs_data_t));
    if (dataP == NULL)
    {
        transaction_free(transaction);
        return COAP_500_INTERNAL_SERVER_ERROR;
    }
    dataP->isUri = false;
    dataP->callback = contextP->bootstrapCallback;
    dataP->userData = contextP->bootstrapUserData;

    transaction->callback = prv_resultCallback;
    transaction->userData = (void *)dataP;

    contextP->transactionList = (lwm2m_transaction_t *)LWM2M_LIST_ADD(contextP->transactionList, transaction);

    return transaction_send(contextP, transaction);
}

#endif
