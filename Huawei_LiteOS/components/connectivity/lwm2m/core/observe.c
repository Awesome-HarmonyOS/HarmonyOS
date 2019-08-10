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
 *    Toby Jaffey - Please refer to git log
 *    Bosch Software Innovations GmbH - Please refer to git log
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
#include "osdepends/atiny_osdep.h"
#include <stdio.h>


#ifdef LWM2M_CLIENT_MODE

#define RES_M_STATE  3

static lwm2m_transaction_callback_t  observe_call_back;

void lwm2m_register_observe_ack_call_back(lwm2m_transaction_callback_t callback)
{
    observe_call_back = callback;
}


static lwm2m_observed_t *prv_findObserved(lwm2m_context_t *contextP,
        lwm2m_uri_t *uriP)
{
    lwm2m_observed_t *targetP;

    targetP = contextP->observedList;
    while (targetP != NULL
            && (targetP->uri.objectId != uriP->objectId
                || targetP->uri.flag != uriP->flag
                || (LWM2M_URI_IS_SET_INSTANCE(uriP) && targetP->uri.instanceId != uriP->instanceId)
                || (LWM2M_URI_IS_SET_RESOURCE(uriP) && targetP->uri.resourceId != uriP->resourceId)))
    {
        targetP = targetP->next;
    }

    return targetP;
}

static void prv_unlinkObserved(lwm2m_context_t *contextP,
                               lwm2m_observed_t *observedP)
{
    if (contextP->observedList == observedP)
    {
        contextP->observedList = contextP->observedList->next;
    }
    else
    {
        lwm2m_observed_t *parentP;

        parentP = contextP->observedList;
        while (parentP->next != NULL
                && parentP->next != observedP)
        {
            parentP = parentP->next;
        }
        if (parentP->next != NULL)
        {
            parentP->next = parentP->next->next;
        }
    }
}

static lwm2m_watcher_t *prv_findWatcher(lwm2m_observed_t *observedP,
                                        lwm2m_server_t *serverP)
{
    lwm2m_watcher_t *targetP;

    targetP = observedP->watcherList;
    while (targetP != NULL
            && targetP->server != serverP)
    {
        targetP = targetP->next;
    }

    return targetP;
}

static lwm2m_watcher_t *prv_getWatcher(lwm2m_context_t *contextP,
                                       lwm2m_uri_t *uriP,
                                       lwm2m_server_t *serverP)
{
    lwm2m_observed_t *observedP;
    bool allocatedObserver;
    lwm2m_watcher_t *watcherP;

    allocatedObserver = false;

    observedP = prv_findObserved(contextP, uriP);
    if (observedP == NULL)
    {
        observedP = (lwm2m_observed_t *)lwm2m_malloc(sizeof(lwm2m_observed_t));
        if (observedP == NULL) return NULL;
        allocatedObserver = true;
        memset(observedP, 0, sizeof(lwm2m_observed_t));
        memcpy(&(observedP->uri), uriP, sizeof(lwm2m_uri_t));
        atiny_mutex_lock(contextP->observe_mutex);
        observedP->next = contextP->observedList;
        contextP->observedList = observedP;
        atiny_mutex_unlock(contextP->observe_mutex);
    }

    watcherP = prv_findWatcher(observedP, serverP);
    if (watcherP == NULL)
    {
        watcherP = (lwm2m_watcher_t *)lwm2m_malloc(sizeof(lwm2m_watcher_t));
        if (watcherP == NULL)
        {
            if (allocatedObserver == true)
            {
                lwm2m_free(observedP);
            }
            return NULL;
        }
        memset(watcherP, 0, sizeof(lwm2m_watcher_t));
        watcherP->active = false;
        watcherP->server = serverP;
        atiny_mutex_lock(contextP->observe_mutex);
        watcherP->next = observedP->watcherList;
        observedP->watcherList = watcherP;
        atiny_mutex_unlock(contextP->observe_mutex);
    }

    return watcherP;
}

uint8_t observe_handleRequest(lwm2m_context_t *contextP,
                              lwm2m_uri_t *uriP,
                              lwm2m_server_t *serverP,
                              int size,
                              lwm2m_data_t *dataP,
                              coap_packet_t *message,
                              coap_packet_t *response)
{
    lwm2m_observed_t * observedP;
    lwm2m_watcher_t *watcherP;
    uint32_t count;

    LOG_ARG("Code: %02X, server status: %s", message->code, STR_STATUS(serverP->status));
    LOG_URI(uriP);

    coap_get_header_observe(message, &count);

    switch (count)
    {
    case 0:
        if (!LWM2M_URI_IS_SET_INSTANCE(uriP) && LWM2M_URI_IS_SET_RESOURCE(uriP)) return COAP_400_BAD_REQUEST;
        if (message->token_len == 0) return COAP_400_BAD_REQUEST;

        watcherP = prv_getWatcher(contextP, uriP, serverP);
        if (watcherP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;

        watcherP->tokenLen = message->token_len;
        memcpy(watcherP->token, message->token, message->token_len);
        watcherP->active = true;
        watcherP->lastTime = lwm2m_gettime();
        watcherP->lastMid = response->mid;
        if (IS_OPTION(message, COAP_OPTION_ACCEPT))
        {
            watcherP->format = utils_convertMediaType((coap_content_type_t)message->accept[0]);
        }
        else
        {
            watcherP->format = LWM2M_CONTENT_TLV;
        }

        if (LWM2M_URI_IS_SET_RESOURCE(uriP))
        {
            switch (dataP->type)
            {
            case LWM2M_TYPE_INTEGER:
                if (1 != lwm2m_data_decode_int(dataP, &(watcherP->lastValue.asInteger))) return COAP_500_INTERNAL_SERVER_ERROR;
                break;
            case LWM2M_TYPE_FLOAT:
                if (1 != lwm2m_data_decode_float(dataP, &(watcherP->lastValue.asFloat))) return COAP_500_INTERNAL_SERVER_ERROR;
                break;
            default:
                break;
            }
        }

        coap_set_header_observe(response, watcherP->counter++);

        return COAP_205_CONTENT;

    case 1:
        // cancellation
        observedP = prv_findObserved(contextP, uriP);
        if (observedP)
        {
            watcherP = prv_findWatcher(observedP, serverP);
            if (watcherP)
            {
                observe_cancel(contextP, watcherP->lastMid, serverP->sessionH);
            }
        }
        return COAP_205_CONTENT;

    default:
        return COAP_400_BAD_REQUEST;
    }
}

void observe_cancel(lwm2m_context_t *contextP,
                    uint16_t mid,
                    void *fromSessionH)
{
    lwm2m_observed_t *observedP;

    LOG_ARG("mid: %d", mid);

    for (observedP = contextP->observedList;
            observedP != NULL;
            observedP = observedP->next)
    {
        lwm2m_watcher_t *targetP = NULL;

        if (observedP->watcherList->lastMid == mid
                && lwm2m_session_is_equal(observedP->watcherList->server->sessionH, fromSessionH, contextP->userData))
        {
            targetP = observedP->watcherList;
            observedP->watcherList = observedP->watcherList->next;
        }
        else
        {
            lwm2m_watcher_t *parentP;

            parentP = observedP->watcherList;
            while (parentP->next != NULL
                    && (parentP->next->lastMid != mid
                 || !lwm2m_session_is_equal(parentP->next->server->sessionH, fromSessionH, contextP->userData)))
            {
                parentP = parentP->next;
            }
            if (parentP->next != NULL)
            {
                targetP = parentP->next;
                parentP->next = parentP->next->next;
            }
        }
        if (targetP != NULL)
        {
            atiny_mutex_lock(contextP->observe_mutex);
            lwm2m_notify_even(MODULE_URI, OBSERVE_UNSUBSCRIBE, (char *) & (observedP->uri), sizeof(observedP->uri));
            if (targetP->parameters != NULL) lwm2m_free(targetP->parameters);
            lwm2m_free(targetP);
            if (observedP->watcherList == NULL)
            {
                prv_unlinkObserved(contextP, observedP);
                lwm2m_free(observedP);
            }
            atiny_mutex_unlock(contextP->observe_mutex);
            return;
        }
    }
}

void observe_clear(lwm2m_context_t *contextP,
                   lwm2m_uri_t *uriP)
{
    lwm2m_observed_t *observedP;

    LOG_URI(uriP);

    observedP = contextP->observedList;
    while(observedP != NULL)
    {
        if (observedP->uri.objectId == uriP->objectId
                && (LWM2M_URI_IS_SET_INSTANCE(uriP) == false
                    || observedP->uri.instanceId == uriP->instanceId))
        {
            lwm2m_observed_t *nextP;
            lwm2m_watcher_t *watcherP;

            atiny_mutex_lock(contextP->observe_mutex);
            nextP = observedP->next;

            for (watcherP = observedP->watcherList; watcherP != NULL; watcherP = watcherP->next)
            {
                if (watcherP->parameters != NULL) lwm2m_free(watcherP->parameters);
            }
            LWM2M_LIST_FREE(observedP->watcherList);

            prv_unlinkObserved(contextP, observedP);
            lwm2m_free(observedP);

            observedP = nextP;
            atiny_mutex_unlock(contextP->observe_mutex);
        }
        else
        {
            observedP = observedP->next;
        }
    }
}

uint8_t observe_setParameters(lwm2m_context_t *contextP,
                              lwm2m_uri_t *uriP,
                              lwm2m_server_t *serverP,
                              lwm2m_attributes_t *attrP)
{
    uint8_t result;
    lwm2m_watcher_t *watcherP;

    LOG_URI(uriP);
    LOG_ARG("toSet: %08X, toClear: %08X, minPeriod: %d, maxPeriod: %d, greaterThan: %f, lessThan: %f, step: %f",
            attrP->toSet, attrP->toClear, attrP->minPeriod, attrP->maxPeriod, attrP->greaterThan, attrP->lessThan, attrP->step);

    if (!LWM2M_URI_IS_SET_INSTANCE(uriP) && LWM2M_URI_IS_SET_RESOURCE(uriP)) return COAP_400_BAD_REQUEST;

    result = object_checkReadable(contextP, uriP);
    if (COAP_205_CONTENT != result) return result;

    if (0 != (attrP->toSet & ATTR_FLAG_NUMERIC))
    {
        result = object_checkNumeric(contextP, uriP);
        if (COAP_205_CONTENT != result) return result;
    }

    watcherP = prv_getWatcher(contextP, uriP, serverP);
    if (watcherP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;

    // Check rule ‚Äúlt‚Ä?value + 2*‚Äùstp‚Ä?values < ‚Äúgt‚Ä?value
    if ((((attrP->toSet | (watcherP->parameters ? watcherP->parameters->toSet : 0)) & ~attrP->toClear) & ATTR_FLAG_NUMERIC) == ATTR_FLAG_NUMERIC)
    {
        float gt;
        float lt;
        float stp;

        if (0 != (attrP->toSet & LWM2M_ATTR_FLAG_GREATER_THAN))
        {
            gt = attrP->greaterThan;
        }
        else
        {
            gt = watcherP->parameters->greaterThan;
        }
        if (0 != (attrP->toSet & LWM2M_ATTR_FLAG_LESS_THAN))
        {
            lt = attrP->lessThan;
        }
        else
        {
            lt = watcherP->parameters->lessThan;
        }
        if (0 != (attrP->toSet & LWM2M_ATTR_FLAG_STEP))
        {
            stp = attrP->step;
        }
        else
        {
            stp = watcherP->parameters->step;
        }

        if (lt + (2 * stp) >= gt) return COAP_400_BAD_REQUEST;
    }

    if (watcherP->parameters == NULL)
    {
        if (attrP->toSet != 0)
        {
            watcherP->parameters = (lwm2m_attributes_t *)lwm2m_malloc(sizeof(lwm2m_attributes_t));
            if (watcherP->parameters == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
            memcpy(watcherP->parameters, attrP, sizeof(lwm2m_attributes_t));
        }
    }
    else
    {
        watcherP->parameters->toSet &= ~attrP->toClear;
        if (attrP->toSet & LWM2M_ATTR_FLAG_MIN_PERIOD)
        {
            watcherP->parameters->minPeriod = attrP->minPeriod;
        }
        if (attrP->toSet & LWM2M_ATTR_FLAG_MAX_PERIOD)
        {
            watcherP->parameters->maxPeriod = attrP->maxPeriod;
        }
        if (attrP->toSet & LWM2M_ATTR_FLAG_GREATER_THAN)
        {
            watcherP->parameters->greaterThan = attrP->greaterThan;
        }
        if (attrP->toSet & LWM2M_ATTR_FLAG_LESS_THAN)
        {
            watcherP->parameters->lessThan = attrP->lessThan;
        }
        if (attrP->toSet & LWM2M_ATTR_FLAG_STEP)
        {
            watcherP->parameters->step = attrP->step;
        }
    }

    LOG_ARG("Final toSet: %08X, minPeriod: %d, maxPeriod: %d, greaterThan: %f, lessThan: %f, step: %f",
            watcherP->parameters->toSet, watcherP->parameters->minPeriod, watcherP->parameters->maxPeriod, watcherP->parameters->greaterThan, watcherP->parameters->lessThan, watcherP->parameters->step);

    return COAP_204_CHANGED;
}

lwm2m_observed_t *observe_findByUri(lwm2m_context_t *contextP,
                                    lwm2m_uri_t *uriP)
{
    lwm2m_observed_t *targetP;

    LOG_URI(uriP);
    targetP = contextP->observedList;
    while (targetP != NULL)
    {
        if (targetP->uri.objectId == uriP->objectId)
        {
            if ((!LWM2M_URI_IS_SET_INSTANCE(uriP) && !LWM2M_URI_IS_SET_INSTANCE(&(targetP->uri)))
                    || (LWM2M_URI_IS_SET_INSTANCE(uriP) && LWM2M_URI_IS_SET_INSTANCE(&(targetP->uri)) && (uriP->instanceId == targetP->uri.instanceId)))
            {
                if ((!LWM2M_URI_IS_SET_RESOURCE(uriP) && !LWM2M_URI_IS_SET_RESOURCE(&(targetP->uri)))
                        || (LWM2M_URI_IS_SET_RESOURCE(uriP) && LWM2M_URI_IS_SET_RESOURCE(&(targetP->uri)) && (uriP->resourceId == targetP->uri.resourceId)))
                {
                    LOG_ARG("Found one with%s observers.", targetP->watcherList ? "" : " no");
                    LOG_URI(&(targetP->uri));
                    return targetP;
                }
            }
        }
        targetP = targetP->next;
    }

    LOG("Found nothing");
    return NULL;
}

void lwm2m_resource_value_changed(lwm2m_context_t *contextP,
                                  lwm2m_uri_t *uriP)
{
    lwm2m_observed_t *targetP;

    LOG_URI(uriP);
    targetP = contextP->observedList;
    while (targetP != NULL)
    {
        if (targetP->uri.objectId == uriP->objectId)
        {
            if (!LWM2M_URI_IS_SET_INSTANCE(uriP)
                    || (targetP->uri.flag & LWM2M_URI_FLAG_INSTANCE_ID) == 0
                    || uriP->instanceId == targetP->uri.instanceId)
            {
                if (!LWM2M_URI_IS_SET_RESOURCE(uriP)
                        || (targetP->uri.flag & LWM2M_URI_FLAG_RESOURCE_ID) == 0
                        || uriP->resourceId == targetP->uri.resourceId)
                {
                    lwm2m_watcher_t *watcherP;

                    LOG("Found an observation");
                    LOG_URI(&(targetP->uri));

                    for (watcherP = targetP->watcherList ; watcherP != NULL ; watcherP = watcherP->next)
                    {
                        if (watcherP->active == true)
                        {
                            LOG("Tagging a watcher");
                            watcherP->update = true;
                        }
                    }
                }
            }
        }
        targetP = targetP->next;
    }

}

int observe_send_transaction(lwm2m_context_t *contextP, lwm2m_data_cfg_t  *cfg, lwm2m_watcher_t *watcherP,
                             uint8_t *buffer, size_t length)
{
    lwm2m_transaction_t *transaction;
    int ret;

    transaction = transaction_new(watcherP->server->sessionH, COAP_205_CONTENT, NULL, NULL, watcherP->lastMid, watcherP->tokenLen, watcherP->token);
    if(NULL == transaction)
    {
        LOG("transaction_new null");
        return COAP_500_INTERNAL_SERVER_ERROR;
    }
    transaction->cfg.callback = cfg->callback;
    transaction->cfg.cookie = cfg->cookie;
    transaction->cfg.type = cfg->type;
    transaction->callback = observe_call_back;
    coap_set_header_content_type(transaction->message, watcherP->format);
    coap_set_header_observe(transaction->message, watcherP->counter++);
    coap_set_payload(transaction->message, buffer, length);
    contextP->transactionList = (lwm2m_transaction_t *)LWM2M_LIST_ADD(contextP->transactionList, transaction);
    ret = transaction_send(contextP, transaction);
    LOG_ARG("notify con msg, cookie:%d type:%d,ret:%d", transaction->cfg.cookie, transaction->cfg.type, ret);
    return ret;
}

void observe_app_step(lwm2m_context_t *contextP,
                      lwm2m_observed_t *targetP,
                      time_t currentTime,
                      time_t *timeoutP)
{
    LOG("Entering");
    lwm2m_watcher_t *watcherP;
    uint8_t *buffer = NULL;
    size_t length = 0;
    lwm2m_data_t *dataP = NULL;
    lwm2m_data_cfg_t  cfg = {0, 0, NULL};
    int size = 0;

    while (COAP_205_CONTENT == object_readData(contextP, &targetP->uri, &size, &dataP, &cfg, 0))
    {

        for (watcherP = targetP->watcherList ; watcherP != NULL ; watcherP = watcherP->next)
        {
            int res;
            if (buffer == NULL)
            {
                res = lwm2m_data_serialize(&targetP->uri, size, dataP, &(watcherP->format), &buffer);
                if (res < 0)
                {
                    if (dataP != NULL)
                    {
                        lwm2m_data_free(size, dataP);
                        dataP = NULL;
                    }
                    if (buffer != NULL)
                    {
                        lwm2m_free(buffer);
                    }
                    break;
                }
                else
                {
                    length = (size_t)res;
                }
            }

            if (watcherP->active == true)
            {
                watcherP->lastTime = currentTime;
                watcherP->lastMid = contextP->nextMID++;

                if (cfg.callback == NULL)
                {
                    coap_packet_t message[1];
                    coap_init_message(message, COAP_TYPE_NON, COAP_205_CONTENT, 0);
                    coap_set_header_content_type(message, watcherP->format);
                    coap_set_payload(message, buffer, length);
                    message->mid = watcherP->lastMid;
                    coap_set_header_token(message, watcherP->token, watcherP->tokenLen);
                    coap_set_header_observe(message, watcherP->counter++);
                    (void)message_send(contextP, message, watcherP->server->sessionH);
                    LOG_ARG("notify no con msg, msgid:", message->mid);
                }
                else
                {
                    (void)observe_send_transaction(contextP, &cfg, watcherP, buffer, length);
                }
            }
        }

        if (dataP != NULL)
        {
            lwm2m_data_free(size, dataP);
            dataP = NULL;
        }
        if (buffer != NULL)
        {
            lwm2m_free(buffer);
            buffer = NULL;
        }
    }

    if (dataP != NULL)
    {
        lwm2m_data_free(size, dataP);
    }
}

void observe_step(lwm2m_context_t *contextP,
                  time_t currentTime,
                  time_t *timeoutP)
{
    lwm2m_observed_t *targetP;

    LOG("Entering");
    for (targetP = contextP->observedList ; targetP != NULL ; targetP = targetP->next)
    {
        lwm2m_watcher_t *watcherP;
        uint8_t *buffer = NULL;
        size_t length = 0;
        lwm2m_data_t *dataP = NULL;
        lwm2m_data_cfg_t  cfg = {0, 0, NULL};
        int size = 0;
        double floatValue = 0;
        int64_t integerValue = 0;
        bool storeValue = false;
        coap_packet_t message[1];
        time_t interval;

        LOG_URI(&(targetP->uri));


        if (dm_isUriOpaqueHandle(&(targetP->uri)))
        {
            observe_app_step(contextP, targetP, currentTime, timeoutP);
            continue;
        }

        cfg.callback = NULL;
        if (LWM2M_URI_IS_SET_RESOURCE(&targetP->uri))
        {
            if (COAP_205_CONTENT != object_readData(contextP, &targetP->uri, &size, &dataP, &cfg, 0)) continue;
            switch (dataP->type)
            {
            case LWM2M_TYPE_INTEGER:
                if (1 != lwm2m_data_decode_int(dataP, &integerValue))
                {
                    lwm2m_data_free(size, dataP);
                    continue;
                }
                storeValue = true;
                break;
            case LWM2M_TYPE_FLOAT:
                if (1 != lwm2m_data_decode_float(dataP, &floatValue))
                {
                    lwm2m_data_free(size, dataP);
                    continue;
                }
                storeValue = true;
                break;
            default:
                break;
            }
        }
        for (watcherP = targetP->watcherList ; watcherP != NULL ; watcherP = watcherP->next)
        {
            if (watcherP->active == true)
            {
                bool notify = false;

                if (watcherP->update == true)
                {
                    // value changed, should we notify the server ?

                    if (watcherP->parameters == NULL || watcherP->parameters->toSet == 0)
                    {
                        // no conditions
                        notify = true;
                        LOG("Notify with no conditions");
                        LOG_URI(&(targetP->uri));
                    }

                    if (notify == false
                            && watcherP->parameters != NULL
                            && (watcherP->parameters->toSet & ATTR_FLAG_NUMERIC) != 0)
                    {
                        if ((watcherP->parameters->toSet & LWM2M_ATTR_FLAG_LESS_THAN) != 0)
                        {
                            LOG("Checking lower threshold");
                            // Did we cross the lower threshold ?
                            switch (dataP->type)
                            {
                            case LWM2M_TYPE_INTEGER:
                                if ((integerValue <= watcherP->parameters->lessThan
                                        && watcherP->lastValue.asInteger > watcherP->parameters->lessThan)
                                        || (integerValue >= watcherP->parameters->lessThan
                                            && watcherP->lastValue.asInteger < watcherP->parameters->lessThan))
                                {
                                    LOG("Notify on lower threshold crossing");
                                    notify = true;
                                }
                                break;
                            case LWM2M_TYPE_FLOAT:
                                if ((floatValue <= watcherP->parameters->lessThan
                                        && watcherP->lastValue.asFloat > watcherP->parameters->lessThan)
                                        || (floatValue >= watcherP->parameters->lessThan
                                            && watcherP->lastValue.asFloat < watcherP->parameters->lessThan))
                                {
                                    LOG("Notify on lower threshold crossing");
                                    notify = true;
                                }
                                break;
                            default:
                                break;
                            }
                        }
                        if ((watcherP->parameters->toSet & LWM2M_ATTR_FLAG_GREATER_THAN) != 0)
                        {
                            LOG("Checking upper threshold");
                            // Did we cross the upper threshold ?
                            switch (dataP->type)
                            {
                            case LWM2M_TYPE_INTEGER:
                                if ((integerValue <= watcherP->parameters->greaterThan
                                        && watcherP->lastValue.asInteger > watcherP->parameters->greaterThan)
                                        || (integerValue >= watcherP->parameters->greaterThan
                                            && watcherP->lastValue.asInteger < watcherP->parameters->greaterThan))
                                {
                                    LOG("Notify on lower upper crossing");
                                    notify = true;
                                }
                                break;
                            case LWM2M_TYPE_FLOAT:
                                if ((floatValue <= watcherP->parameters->greaterThan
                                        && watcherP->lastValue.asFloat > watcherP->parameters->greaterThan)
                                        || (floatValue >= watcherP->parameters->greaterThan
                                            && watcherP->lastValue.asFloat < watcherP->parameters->greaterThan))
                                {
                                    LOG("Notify on lower upper crossing");
                                    notify = true;
                                }
                                break;
                            default:
                                break;
                            }
                        }
                        if ((watcherP->parameters->toSet & LWM2M_ATTR_FLAG_STEP) != 0)
                        {
                            LOG("Checking step");

                            switch (dataP->type)
                            {
                            case LWM2M_TYPE_INTEGER:
                            {
                                int64_t diff;

                                diff = integerValue - watcherP->lastValue.asInteger;
                                if ((diff < 0 && (0 - diff) >= watcherP->parameters->step)
                                        || (diff >= 0 && diff >= watcherP->parameters->step))
                                {
                                    LOG("Notify on step condition");
                                    notify = true;
                                }
                            }
                            break;
                            case LWM2M_TYPE_FLOAT:
                            {
                                double diff;

                                diff = floatValue - watcherP->lastValue.asFloat;
                                if ((diff < 0 && (0 - diff) >= watcherP->parameters->step)
                                        || (diff >= 0 && diff >= watcherP->parameters->step))
                                {
                                    LOG("Notify on step condition");
                                    notify = true;
                                }
                            }
                            break;
                            default:
                                break;
                            }
                        }
                    }

                    if (watcherP->parameters != NULL
                            && (watcherP->parameters->toSet & LWM2M_ATTR_FLAG_MIN_PERIOD) != 0)
                    {
                        LOG_ARG("Checking minimal period (%d s)", watcherP->parameters->minPeriod);

                        if (watcherP->lastTime + watcherP->parameters->minPeriod > currentTime)
                        {
                            // Minimum Period did not elapse yet
                            interval = watcherP->lastTime + watcherP->parameters->minPeriod - currentTime;
                            if (*timeoutP > interval) *timeoutP = interval;
                            notify = false;
                        }
                        else
                        {
                            LOG("Notify on minimal period");
                            notify = true;
                        }
                    }
                }

                // Is the Maximum Period reached ?
                if (notify == false
                        && watcherP->parameters != NULL
                        && (watcherP->parameters->toSet & LWM2M_ATTR_FLAG_MAX_PERIOD) != 0)
                {
                    LOG_ARG("Checking maximal period (%d s)", watcherP->parameters->maxPeriod);

                    if (watcherP->lastTime + watcherP->parameters->maxPeriod <= currentTime)
                    {
                        LOG("Notify on maximal period");
                        notify = true;
                    }
                }

                if (notify == true)
                {
                    if (buffer == NULL)
                    {
                        if (dataP != NULL)
                        {
                            int res;

                            res = lwm2m_data_serialize(&targetP->uri, size, dataP, &(watcherP->format), &buffer);
                            if (res < 0)
                            {
                                break;
                            }
                            else
                            {
                                length = (size_t)res;
                            }

                        }
                        else
                        {
                            if (COAP_205_CONTENT != object_read(contextP, &targetP->uri, &(watcherP->format), &buffer, &length, 0))
                            {
                                buffer = NULL;
                                break;
                            }
                        }

                        if(NULL == cfg.callback)
                        {
                            coap_init_message(message, COAP_TYPE_NON, COAP_205_CONTENT, 0);
                            coap_set_header_content_type(message, watcherP->format);
                            coap_set_payload(message, buffer, length);
                            LOG_ARG("non observe, msgid:", message->mid);
                        }
                    }

                    watcherP->lastTime = currentTime;
                    watcherP->lastMid = contextP->nextMID++;
                    if(NULL == cfg.callback)
                    {

                        message->mid = watcherP->lastMid;
                        coap_set_header_token(message, watcherP->token, watcherP->tokenLen);
                        coap_set_header_observe(message, watcherP->counter++);
                        (void)message_send(contextP, message, watcherP->server->sessionH);

                    }
                    else
                    {
                        (void)observe_send_transaction(contextP, &cfg, watcherP, buffer, length);
                    }
                    atiny_mutex_lock(contextP->observe_mutex);
                    watcherP->update = false;
                    atiny_mutex_unlock(contextP->observe_mutex);
                }

                // Store this value
                if (notify == true && storeValue == true)
                {
                    switch (dataP->type)
                    {
                    case LWM2M_TYPE_INTEGER:
                        watcherP->lastValue.asInteger = integerValue;
                        break;
                    case LWM2M_TYPE_FLOAT:
                        watcherP->lastValue.asFloat = floatValue;
                        break;
                    default:
                        break;
                    }
                }

                if (watcherP->parameters != NULL && (watcherP->parameters->toSet & LWM2M_ATTR_FLAG_MAX_PERIOD) != 0)
                {
                    // update timers
                    interval = watcherP->lastTime + watcherP->parameters->maxPeriod - currentTime;
                    if (*timeoutP > interval) *timeoutP = interval;
                }
            }
        }
        if (dataP != NULL) lwm2m_data_free(size, dataP);
        if (buffer != NULL) lwm2m_free(buffer);
    }
}



uint8_t lwm2m_get_observe_info(lwm2m_context_t *contextP, lwm2m_observe_info_t *observe_info)
{
    lwm2m_observed_t *targetP;
    lwm2m_watcher_t *watcherP;

    if((NULL == observe_info) || (NULL == contextP))
    {
        LOG("null pointer\n");
        return COAP_500_INTERNAL_SERVER_ERROR;
    }

    for (targetP = contextP->observedList ; targetP != NULL ; targetP = targetP->next)
    {
        if((!LWM2M_URI_IS_SET_RESOURCE(&targetP->uri))
                || (targetP->uri.objectId != LWM2M_FIRMWARE_UPDATE_OBJECT_ID)
                || (targetP->uri.instanceId != 0)
                || (targetP->uri.resourceId != RES_M_STATE))
        {
            continue;
        }
        watcherP = targetP->watcherList;
        observe_info->counter = watcherP->counter;
        memcpy(observe_info->token, watcherP->token, sizeof(observe_info->token));
        observe_info->tokenLen = watcherP->tokenLen;
        observe_info->format = watcherP->format;
        return COAP_NO_ERROR;
    }
    return COAP_500_INTERNAL_SERVER_ERROR;
}

uint8_t lwm2m_send_notify(lwm2m_context_t *contextP, lwm2m_observe_info_t *observe_info, int firmware_update_state,  lwm2m_data_cfg_t  *cfg)
{
    lwm2m_uri_t uri;
    int res;
    lwm2m_data_t data;
    lwm2m_media_type_t format;
    uint8_t *buffer = NULL;
    lwm2m_server_t *server;
    lwm2m_watcher_t watcherP;


    if((NULL == observe_info) || (NULL == contextP) || (NULL == cfg))
    {
        LOG("null pointer\n");
        return COAP_500_INTERNAL_SERVER_ERROR;
    }

    server = registration_get_registered_server(contextP);
    if(NULL == server)
    {
        LOG("registration_get_registered_server fail\n");
        return COAP_500_INTERNAL_SERVER_ERROR;
    }

    uri.objectId = LWM2M_FIRMWARE_UPDATE_OBJECT_ID;
    uri.instanceId = 0;
    uri.resourceId = RES_M_STATE;
    uri.flag = (LWM2M_URI_FLAG_OBJECT_ID | LWM2M_URI_FLAG_INSTANCE_ID | LWM2M_URI_FLAG_RESOURCE_ID);

    format = (lwm2m_media_type_t)observe_info->format;
    memset(&data, 0, sizeof(data));
    data.id = uri.resourceId;
    lwm2m_data_encode_int(firmware_update_state, &data);
    res = lwm2m_data_serialize(&uri, 1, &data, &format, &buffer);
    if (res < 0)
    {
        LOG("lwm2m_data_serialize fail\n");
        if (buffer != NULL)
        {
            lwm2m_free(buffer);
        }
        return COAP_500_INTERNAL_SERVER_ERROR;
    }

    memset(&watcherP, 0, sizeof(watcherP));
    watcherP.lastMid = contextP->nextMID++;
    watcherP.tokenLen = observe_info->tokenLen;
    memcpy(watcherP.token, observe_info->token, sizeof(watcherP.token));
    watcherP.format = format;
    watcherP.counter = observe_info->counter;
    watcherP.server = server;

    return (uint8_t)observe_send_transaction(contextP, cfg, &watcherP, buffer, res);
}

#endif

#ifdef LWM2M_SERVER_MODE

typedef struct
{
    lwm2m_observation_t *observationP;
    lwm2m_result_callback_t callbackP;
    void *userDataP;
} cancellation_data_t;

static lwm2m_observation_t *prv_findObservationByURI(lwm2m_client_t *clientP,
        lwm2m_uri_t *uriP)
{
    lwm2m_observation_t *targetP;

    targetP = clientP->observationList;
    while (targetP != NULL)
    {
        if (targetP->uri.objectId == uriP->objectId
                && targetP->uri.flag == uriP->flag
                && targetP->uri.instanceId == uriP->instanceId
                && targetP->uri.resourceId == uriP->resourceId)
        {
            return targetP;
        }

        targetP = targetP->next;
    }

    return targetP;
}

void observe_remove(lwm2m_observation_t *observationP)
{
    LOG("Entering");
    observationP->clientP->observationList = (lwm2m_observation_t *) LWM2M_LIST_RM(observationP->clientP->observationList, observationP->id, NULL);
    lwm2m_free(observationP);
}

static void prv_obsRequestCallback(lwm2m_transaction_t *transacP,
                                   void *message)
{
    lwm2m_observation_t *observationP = (lwm2m_observation_t *)transacP->userData;
    coap_packet_t *packet = (coap_packet_t *)message;
    uint8_t code;

    switch (observationP->status)
    {
    case STATE_DEREG_PENDING:
        // Observation was canceled by the user.
        observe_remove(observationP);
        return;

    case STATE_REG_PENDING:
        observationP->status = STATE_REGISTERED;
        break;

    default:
        break;
    }

    if (message == NULL)
    {
        code = COAP_503_SERVICE_UNAVAILABLE;
    }
    else if (packet->code == COAP_205_CONTENT
             && !IS_OPTION(packet, COAP_OPTION_OBSERVE))
    {
        code = COAP_405_METHOD_NOT_ALLOWED;
    }
    else
    {
        code = packet->code;
    }

    if (code != COAP_205_CONTENT)
    {
        observationP->callback(observationP->clientP->internalID,
                               &observationP->uri,
                               code,
                               LWM2M_CONTENT_TEXT, NULL, 0,
                               observationP->userData);
        observe_remove(observationP);
    }
    else
    {
        observationP->callback(observationP->clientP->internalID,
                               &observationP->uri,
                               0,
                               packet->content_type, packet->payload, packet->payload_len,
                               observationP->userData);
    }
}


static void prv_obsCancelRequestCallback(lwm2m_transaction_t *transacP,
        void *message)
{
    cancellation_data_t *cancelP = (cancellation_data_t *)transacP->userData;
    coap_packet_t *packet = (coap_packet_t *)message;
    uint8_t code;

    if (message == NULL)
    {
        code = COAP_503_SERVICE_UNAVAILABLE;
    }
    else
    {
        code = packet->code;
    }

    if (code != COAP_205_CONTENT)
    {
        cancelP->callbackP(cancelP->observationP->clientP->internalID,
                           &cancelP->observationP->uri,
                           code,
                           LWM2M_CONTENT_TEXT, NULL, 0,
                           cancelP->userDataP);
    }
    else
    {
        cancelP->callbackP(cancelP->observationP->clientP->internalID,
                           &cancelP->observationP->uri,
                           0,
                           packet->content_type, packet->payload, packet->payload_len,
                           cancelP->userDataP);
    }

    observe_remove(cancelP->observationP);

    lwm2m_free(cancelP);
}


int lwm2m_observe(lwm2m_context_t *contextP,
                  uint16_t clientID,
                  lwm2m_uri_t *uriP,
                  lwm2m_result_callback_t callback,
                  void *userData)
{
    lwm2m_client_t *clientP;
    lwm2m_transaction_t *transactionP;
    lwm2m_observation_t *observationP;
    uint8_t token[4];

    LOG_ARG("clientID: %d", clientID);
    LOG_URI(uriP);

    if (!LWM2M_URI_IS_SET_INSTANCE(uriP) && LWM2M_URI_IS_SET_RESOURCE(uriP)) return COAP_400_BAD_REQUEST;

    clientP = (lwm2m_client_t *)lwm2m_list_find((lwm2m_list_t *)contextP->clientList, clientID);
    if (clientP == NULL) return COAP_404_NOT_FOUND;

    for (observationP = clientP->observationList; observationP != NULL; observationP = observationP->next)
    {
        if (uriP->objectId == observationP->uri.objectId
                && (LWM2M_URI_IS_SET_INSTANCE(uriP) == false
                    || observationP->uri.instanceId == uriP->instanceId)
                && (LWM2M_URI_IS_SET_RESOURCE(uriP) == false
                    || observationP->uri.resourceId == uriP->resourceId))
        {
            break;
        }
    }
    if (observationP == NULL)
    {
        observationP = (lwm2m_observation_t *)lwm2m_malloc(sizeof(lwm2m_observation_t));
        if (observationP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        memset(observationP, 0, sizeof(lwm2m_observation_t));

        observationP->id = lwm2m_list_newId((lwm2m_list_t *)clientP->observationList);
        memcpy(&observationP->uri, uriP, sizeof(lwm2m_uri_t));
        observationP->clientP = clientP;

        observationP->clientP->observationList = (lwm2m_observation_t *)LWM2M_LIST_ADD(observationP->clientP->observationList, observationP);
    }
    observationP->status = STATE_REG_PENDING;
    observationP->callback = callback;
    observationP->userData = userData;

    token[0] = clientP->internalID >> 8;
    token[1] = clientP->internalID & 0xFF;
    token[2] = observationP->id >> 8;
    token[3] = observationP->id & 0xFF;

    transactionP = transaction_new(clientP->sessionH, COAP_GET, clientP->altPath, uriP, contextP->nextMID++, 4, token);
    if (transactionP == NULL)
    {
        observationP->clientP->observationList = (lwm2m_observation_t *)LWM2M_LIST_RM(observationP->clientP->observationList, observationP->id, NULL);
        lwm2m_free(observationP);
        return COAP_500_INTERNAL_SERVER_ERROR;
    }

    coap_set_header_observe(transactionP->message, 0);
    if (clientP->supportJSON == true)
    {
        coap_set_header_accept(transactionP->message, LWM2M_CONTENT_JSON);
    }
    else
    {
        coap_set_header_accept(transactionP->message, LWM2M_CONTENT_TLV);
    }

    transactionP->callback = prv_obsRequestCallback;
    transactionP->userData = (void *)observationP;

    contextP->transactionList = (lwm2m_transaction_t *)LWM2M_LIST_ADD(contextP->transactionList, transactionP);

    return transaction_send(contextP, transactionP);
}

int lwm2m_observe_cancel(lwm2m_context_t *contextP,
                         uint16_t clientID,
                         lwm2m_uri_t *uriP,
                         lwm2m_result_callback_t callback,
                         void *userData)
{
    lwm2m_client_t *clientP;
    lwm2m_observation_t *observationP;

    LOG_ARG("clientID: %d", clientID);
    LOG_URI(uriP);

    clientP = (lwm2m_client_t *)lwm2m_list_find((lwm2m_list_t *)contextP->clientList, clientID);
    if (clientP == NULL) return COAP_404_NOT_FOUND;

    observationP = prv_findObservationByURI(clientP, uriP);
    if (observationP == NULL) return COAP_404_NOT_FOUND;

    switch (observationP->status)
    {
    case STATE_REGISTERED:
    {
        lwm2m_transaction_t *transactionP;
        cancellation_data_t *cancelP;
        uint8_t token[4];

        token[0] = clientP->internalID >> 8;
        token[1] = clientP->internalID & 0xFF;
        token[2] = observationP->id >> 8;
        token[3] = observationP->id & 0xFF;

        transactionP = transaction_new(clientP->sessionH, COAP_GET, clientP->altPath, uriP, contextP->nextMID++, 4, token);
        if (transactionP == NULL)
        {
            return COAP_500_INTERNAL_SERVER_ERROR;
        }
        cancelP = (cancellation_data_t *)lwm2m_malloc(sizeof(cancellation_data_t));
        if (cancelP == NULL)
        {
            lwm2m_free(transactionP);
            return COAP_500_INTERNAL_SERVER_ERROR;
        }

        coap_set_header_observe(transactionP->message, 1);

        cancelP->observationP = observationP;
        cancelP->callbackP = callback;
        cancelP->userDataP = userData;

        transactionP->callback = prv_obsCancelRequestCallback;
        transactionP->userData = (void *)cancelP;

        contextP->transactionList = (lwm2m_transaction_t *)LWM2M_LIST_ADD(contextP->transactionList, transactionP);

        return transaction_send(contextP, transactionP);
    }

    case STATE_REG_PENDING:
        observationP->status = STATE_DEREG_PENDING;
        break;

    default:
        // Should not happen
        break;
    }

    return COAP_NO_ERROR;
}

bool observe_handleNotify(lwm2m_context_t *contextP,
                          void *fromSessionH,
                          coap_packet_t *message,
                          coap_packet_t *response)
{
    uint8_t *tokenP;
    int token_len;
    uint16_t clientID;
    uint16_t obsID;
    lwm2m_client_t *clientP;
    lwm2m_observation_t *observationP;
    uint32_t count;

    LOG("Entering");
    token_len = coap_get_header_token(message, (const uint8_t **)&tokenP);
    if (token_len != sizeof(uint32_t)) return false;

    if (1 != coap_get_header_observe(message, &count)) return false;

    clientID = (tokenP[0] << 8) | tokenP[1];
    obsID = (tokenP[2] << 8) | tokenP[3];

    clientP = (lwm2m_client_t *)lwm2m_list_find((lwm2m_list_t *)contextP->clientList, clientID);
    if (clientP == NULL) return false;

    observationP = (lwm2m_observation_t *)lwm2m_list_find((lwm2m_list_t *)clientP->observationList, obsID);
    if (observationP == NULL)
    {
        coap_init_message(response, COAP_TYPE_RST, 0, message->mid);
        message_send(contextP, response, fromSessionH);
    }
    else
    {
        if (message->type == COAP_TYPE_CON )
        {
            coap_init_message(response, COAP_TYPE_ACK, 0, message->mid);
            message_send(contextP, response, fromSessionH);
        }
        observationP->callback(clientID,
                               &observationP->uri,
                               (int)count,
                               message->content_type, message->payload, message->payload_len,
                               observationP->userData);
    }
    return true;
}
#endif
