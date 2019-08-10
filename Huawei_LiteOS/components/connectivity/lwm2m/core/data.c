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
*    Fabien Fleutot - Please refer to git log
*    Bosch Software Innovations GmbH - Please refer to git log
*
*******************************************************************************/

#include "internals.h"
#include <float.h>

#define _PRV_STR_LENGTH 32


void lwm2m_data_decode_opaque(const lwm2m_data_t *dataP,
                              int8_t *valueP)
{
    memcpy(valueP, dataP->value.asBuffer.buffer, dataP->value.asBuffer.length);
}

// dataP array length is assumed to be 1.
static int prv_textSerialize(lwm2m_data_t *dataP,
                             uint8_t **bufferP)
{
    size_t res;

    switch (dataP->type)
    {
    case LWM2M_TYPE_STRING:
        *bufferP = (uint8_t *)lwm2m_malloc(dataP->value.asBuffer.length);
        if (*bufferP == NULL) return 0;
        memcpy(*bufferP, dataP->value.asBuffer.buffer, dataP->value.asBuffer.length);
        return (int)dataP->value.asBuffer.length;

    case LWM2M_TYPE_INTEGER:
    {
        uint8_t intString[_PRV_STR_LENGTH];

        res = utils_intToText(dataP->value.asInteger, intString, _PRV_STR_LENGTH);
        if (res == 0) return -1;

        *bufferP = (uint8_t *)lwm2m_malloc(res);
        if (NULL == *bufferP) return -1;

        memcpy(*bufferP, intString, res);

        return (int)res;
    }

    case LWM2M_TYPE_FLOAT:
    {
        uint8_t floatString[_PRV_STR_LENGTH * 2];

        res = utils_floatToText(dataP->value.asFloat, floatString, _PRV_STR_LENGTH * 2);
        if (res == 0) return -1;

        *bufferP = (uint8_t *)lwm2m_malloc(res);
        if (NULL == *bufferP) return -1;

        memcpy(*bufferP, floatString, res);

        return (int)res;
    }

    case LWM2M_TYPE_BOOLEAN:
        *bufferP = (uint8_t *)lwm2m_malloc(1);
        if (NULL == *bufferP) return -1;

        *bufferP[0] = dataP->value.asBoolean ? '1' : '0';

        return 1;

    case LWM2M_TYPE_OBJECT_LINK:
    {
        char stringBuffer[11];
        size_t length;

        length = utils_intToText(dataP->value.asObjLink.objectId, (uint8_t *)stringBuffer, 5);
        if (length == 0) return -1;

        stringBuffer[5] = ':';
        res = length + 1;

        length = utils_intToText(dataP->value.asObjLink.objectInstanceId, (uint8_t *)stringBuffer + res, 5);
        if (length == 0) return -1;

        res += length;

        *bufferP = (uint8_t *)lwm2m_malloc(res);
        if (*bufferP == NULL) return -1;

        memcpy(*bufferP, stringBuffer, res);

        return res;
    }

    case LWM2M_TYPE_OPAQUE:
    {
        size_t length;

        length = utils_base64GetSize(dataP->value.asBuffer.length);
        *bufferP = (uint8_t *)lwm2m_malloc(length);
        if (*bufferP == NULL) return 0;
        length = utils_base64Encode(dataP->value.asBuffer.buffer, dataP->value.asBuffer.length, *bufferP, length);
        if (length == 0)
        {
            lwm2m_free(*bufferP);
            *bufferP = NULL;
            return 0;
        }
        return (int)length;
    }

    case LWM2M_TYPE_UNDEFINED:
    default:
        return -1;
    }
}

static int prv_setBuffer(lwm2m_data_t *dataP,
                         uint8_t *buffer,
                         size_t bufferLen)
{
    dataP->value.asBuffer.buffer = (uint8_t *)lwm2m_malloc(bufferLen);
    if (dataP->value.asBuffer.buffer == NULL)
    {
        return 0;
    }
    dataP->value.asBuffer.length = bufferLen;
    memcpy(dataP->value.asBuffer.buffer, buffer, bufferLen);

    return 1;
}

lwm2m_data_t *lwm2m_data_new(int size)
{
    lwm2m_data_t *dataP;

    LOG_ARG("size: %d", size);
    if (size <= 0) return NULL;

    dataP = (lwm2m_data_t *)lwm2m_malloc(size * sizeof(lwm2m_data_t));

    if (dataP != NULL)
    {
        memset(dataP, 0, size * sizeof(lwm2m_data_t));
    }

    return dataP;
}

void lwm2m_data_free(int size,
                     lwm2m_data_t *dataP)
{
    int i;

    LOG_ARG("size: %d", size);
    if (size == 0 || dataP == NULL) return;

    for (i = 0; i < size; i++)
    {
        switch (dataP[i].type)
        {
        case LWM2M_TYPE_MULTIPLE_RESOURCE:
        case LWM2M_TYPE_OBJECT_INSTANCE:
        case LWM2M_TYPE_OBJECT:
            lwm2m_data_free(dataP[i].value.asChildren.count, dataP[i].value.asChildren.array);
            break;

        case LWM2M_TYPE_STRING:
        case LWM2M_TYPE_OPAQUE:
            if (dataP[i].value.asBuffer.buffer != NULL)
            {
                lwm2m_free(dataP[i].value.asBuffer.buffer);
            }

        default:
            // do nothing
            break;
        }
    }
    lwm2m_free(dataP);
}

void lwm2m_data_encode_string(const char *string,
                              lwm2m_data_t *dataP)
{
    size_t len;
    int res;

    LOG_ARG("\"%s\"", string);
    if (string == NULL)
    {
        len = 0;
    }
    else
    {
        for (len = 0; string[len] != 0; len++);
    }

    if (len == 0)
    {
        dataP->value.asBuffer.length = 0;
        dataP->value.asBuffer.buffer = NULL;
        res = 1;
    }
    else
    {
        res = prv_setBuffer(dataP, (uint8_t *)string, len);
    }

    if (res == 1)
    {
        dataP->type = LWM2M_TYPE_STRING;
    }
    else
    {
        dataP->type = LWM2M_TYPE_UNDEFINED;
    }
}

void lwm2m_data_encode_opaque(uint8_t *buffer,
                              size_t length,
                              lwm2m_data_t *dataP)
{
    int res;

    LOG_ARG("length: %d", length);
    if (length == 0)
    {
        dataP->value.asBuffer.length = 0;
        dataP->value.asBuffer.buffer = NULL;
        res = 1;
    }
    else
    {
        res = prv_setBuffer(dataP, buffer, length);
    }

    if (res == 1)
    {
        dataP->type = LWM2M_TYPE_OPAQUE;
    }
    else
    {
        dataP->type = LWM2M_TYPE_UNDEFINED;
    }
}

void lwm2m_data_encode_nstring(const char *string,
                               size_t length,
                               lwm2m_data_t *dataP)
{
    LOG_ARG("length: %d, string: \"%s\"", length, string);
    lwm2m_data_encode_opaque((uint8_t *)string, length, dataP);

    if (dataP->type == LWM2M_TYPE_OPAQUE)
    {
        dataP->type = LWM2M_TYPE_STRING;
    }
}

void lwm2m_data_encode_int(int64_t value,
                           lwm2m_data_t *dataP)
{
    LOG_ARG("value: %" PRId64 "", value);
    dataP->type = LWM2M_TYPE_INTEGER;
    dataP->value.asInteger = value;
}

int lwm2m_data_decode_int(const lwm2m_data_t *dataP,
                          int64_t *valueP)
{
    int result;

    LOG("Entering");
    switch (dataP->type)
    {
    case LWM2M_TYPE_INTEGER:
        *valueP = dataP->value.asInteger;
        result = 1;
        break;

    case LWM2M_TYPE_STRING:
        result = utils_textToInt(dataP->value.asBuffer.buffer, dataP->value.asBuffer.length, valueP);
        break;

    case LWM2M_TYPE_OPAQUE:
        switch (dataP->value.asBuffer.length)
        {
        case 1:
            *valueP = (int8_t)dataP->value.asBuffer.buffer[0];
            result = 1;
            break;

        case 2:
        {
            int16_t value;

            utils_copyValue(&value, dataP->value.asBuffer.buffer, dataP->value.asBuffer.length);

            *valueP = value;
            result = 1;
            break;
        }

        case 4:
        {
            int32_t value;

            utils_copyValue(&value, dataP->value.asBuffer.buffer, dataP->value.asBuffer.length);

            *valueP = value;
            result = 1;
            break;
        }

        case 8:
            utils_copyValue(valueP, dataP->value.asBuffer.buffer, dataP->value.asBuffer.length);
            result = 1;
            break;

        default:
            result = 0;
        }
        break;

    default:
        return 0;
    }
    LOG_ARG("result: %d, value: %" PRId64, result, *valueP);

    return result;
}

void lwm2m_data_encode_float(double value,
                             lwm2m_data_t *dataP)
{
    LOG_ARG("value: %f", value);
    dataP->type = LWM2M_TYPE_FLOAT;
    dataP->value.asFloat = value;
}

int lwm2m_data_decode_float(const lwm2m_data_t *dataP,
                            double *valueP)
{
    int result;

    LOG("Entering");
    switch (dataP->type)
    {
    case LWM2M_TYPE_FLOAT:
        *valueP = dataP->value.asFloat;
        result = 1;
        break;

    case LWM2M_TYPE_INTEGER:
        *valueP = (double)dataP->value.asInteger;
        result = 1;
        break;

    case LWM2M_TYPE_STRING:
        result = utils_textToFloat(dataP->value.asBuffer.buffer, dataP->value.asBuffer.length, valueP);
        break;

    case LWM2M_TYPE_OPAQUE:
        switch (dataP->value.asBuffer.length)
        {
        case 4:
        {
            float temp;

            utils_copyValue(&temp, dataP->value.asBuffer.buffer, dataP->value.asBuffer.length);

            *valueP = temp;
            result = 1;
        }
        break;

        case 8:
            utils_copyValue(valueP, dataP->value.asBuffer.buffer, dataP->value.asBuffer.length);
            result = 1;
            break;

        default:
            result = 0;
        }
        break;

    default:
        result = 0;
    }

    LOG_ARG("result: %d, value: %f", result, *valueP);

    return result;
}

void lwm2m_data_encode_bool(bool value,
                            lwm2m_data_t *dataP)
{
    LOG_ARG("value: %s", value ? "true" : "false");
    dataP->type = LWM2M_TYPE_BOOLEAN;
    dataP->value.asBoolean = value;
}

int lwm2m_data_decode_bool(const lwm2m_data_t *dataP,
                           bool *valueP)
{
    int result;

    LOG("Entering");
    switch (dataP->type)
    {
    case LWM2M_TYPE_BOOLEAN:
        *valueP = dataP->value.asBoolean;
        result = 1;
        break;

    case LWM2M_TYPE_STRING:
        if (dataP->value.asBuffer.length != 1) return 0;

        switch (dataP->value.asBuffer.buffer[0])
        {
        case '0':
            *valueP = false;
            result = 1;
            break;
        case '1':
            *valueP = true;
            result = 1;
            break;
        default:
            result = 0;
            break;
        }
        break;

    case LWM2M_TYPE_OPAQUE:
        if (dataP->value.asBuffer.length != 1) return 0;

        switch (dataP->value.asBuffer.buffer[0])
        {
        case 0:
            *valueP = false;
            result = 1;
            break;
        case 1:
            *valueP = true;
            result = 1;
            break;
        default:
            result = 0;
            break;
        }
        break;

    default:
        result = 0;
        break;
    }

    LOG_ARG("result: %d, value: %s", result, *valueP ? "true" : "false");

    return result;
}

void lwm2m_data_encode_objlink(uint16_t objectId,
                               uint16_t objectInstanceId,
                               lwm2m_data_t *dataP)
{
    LOG_ARG("value: %d/%d", objectId, objectInstanceId);
    dataP->type = LWM2M_TYPE_OBJECT_LINK;
    dataP->value.asObjLink.objectId = objectId;
    dataP->value.asObjLink.objectInstanceId = objectInstanceId;
}

void lwm2m_data_include(lwm2m_data_t *subDataP,
                        size_t count,
                        lwm2m_data_t *dataP)
{
    LOG_ARG("count: %d", count);
    if (subDataP == NULL || count == 0) return;

    switch (subDataP[0].type)
    {
    case LWM2M_TYPE_STRING:
    case LWM2M_TYPE_OPAQUE:
    case LWM2M_TYPE_INTEGER:
    case LWM2M_TYPE_FLOAT:
    case LWM2M_TYPE_BOOLEAN:
    case LWM2M_TYPE_OBJECT_LINK:
    case LWM2M_TYPE_MULTIPLE_RESOURCE:
        dataP->type = LWM2M_TYPE_OBJECT_INSTANCE;
        break;
    case LWM2M_TYPE_OBJECT_INSTANCE:
        dataP->type = LWM2M_TYPE_OBJECT;
        break;
    default:
        return;
    }
    dataP->value.asChildren.count = count;
    dataP->value.asChildren.array = subDataP;
}

void lwm2m_data_encode_instances(lwm2m_data_t *subDataP,
                                 size_t count,
                                 lwm2m_data_t *dataP)
{
    LOG_ARG("count: %d", count);
    lwm2m_data_include(subDataP, count, dataP);
    dataP->type = LWM2M_TYPE_MULTIPLE_RESOURCE;
}

int lwm2m_data_parse(lwm2m_uri_t *uriP,
                     uint8_t *buffer,
                     size_t bufferLen,
                     lwm2m_media_type_t format,
                     lwm2m_data_t **dataP)
{
    int res;

    LOG_ARG("format: %s, bufferLen: %d", STR_MEDIA_TYPE(format), bufferLen);
    LOG_URI(uriP);
    switch (format)
    {
    case LWM2M_CONTENT_TEXT:
        if (!LWM2M_URI_IS_SET_RESOURCE(uriP)) return 0;
        *dataP = lwm2m_data_new(1);
        if (*dataP == NULL) return 0;
        (*dataP)->id = uriP->resourceId;
        (*dataP)->type = LWM2M_TYPE_STRING;
        res = prv_setBuffer(*dataP, buffer, bufferLen);
        if (res == 0)
        {
            lwm2m_data_free(1, *dataP);
            *dataP = NULL;
        }
        return res;

    case LWM2M_CONTENT_OPAQUE:
        if (!LWM2M_URI_IS_SET_RESOURCE(uriP)) return 0;
        *dataP = lwm2m_data_new(1);
        if (*dataP == NULL) return 0;
        (*dataP)->id = uriP->resourceId;
        (*dataP)->type = LWM2M_TYPE_OPAQUE;
        res = prv_setBuffer(*dataP, buffer, bufferLen);
        if (res == 0)
        {
            lwm2m_data_free(1, *dataP);
            *dataP = NULL;
        }
        return res;

#ifdef LWM2M_OLD_CONTENT_FORMAT_SUPPORT
    case LWM2M_CONTENT_TLV_OLD:
#endif
    case LWM2M_CONTENT_TLV:
        return tlv_parse(buffer, bufferLen, dataP);

#ifdef LWM2M_SUPPORT_JSON
#ifdef LWM2M_OLD_CONTENT_FORMAT_SUPPORT
    case LWM2M_CONTENT_JSON_OLD:
#endif
    case LWM2M_CONTENT_JSON:
        return json_parse(uriP, buffer, bufferLen, dataP);
#endif

    default:
        return 0;
    }
}

int lwm2m_data_serialize(lwm2m_uri_t *uriP,
                         int size,
                         lwm2m_data_t *dataP,
                         lwm2m_media_type_t *formatP,
                         uint8_t **bufferP)
{
    LOG_URI(uriP);
    LOG_ARG("size: %d, formatP: %s", size, STR_MEDIA_TYPE(*formatP));

    // Check format
    if (*formatP == LWM2M_CONTENT_TEXT
            || *formatP == LWM2M_CONTENT_OPAQUE)
    {
        if (size != 1
                || (uriP != NULL && !LWM2M_URI_IS_SET_RESOURCE(uriP))
                || dataP->type == LWM2M_TYPE_OBJECT
                || dataP->type == LWM2M_TYPE_OBJECT_INSTANCE
                || dataP->type == LWM2M_TYPE_MULTIPLE_RESOURCE)
        {
#ifdef LWM2M_SUPPORT_JSON
            *formatP = LWM2M_CONTENT_JSON;
#else
            *formatP = LWM2M_CONTENT_TLV;
#endif
        }
    }

    if (*formatP == LWM2M_CONTENT_OPAQUE
            && dataP->type != LWM2M_TYPE_OPAQUE)
    {
        LOG("Opaque format is reserved to opaque resources.");
        return -1;
    }

    if (dm_isUriOpaqueHandle(uriP))
    {
        *formatP = LWM2M_CONTENT_OPAQUE;
    }

    LOG_ARG("Final format: %s", STR_MEDIA_TYPE(*formatP));

    switch (*formatP)
    {
    case LWM2M_CONTENT_TEXT:
        return prv_textSerialize(dataP, bufferP);

    case LWM2M_CONTENT_OPAQUE:
        *bufferP = (uint8_t *)lwm2m_malloc(dataP->value.asBuffer.length);
        if (*bufferP == NULL) return -1;
        memcpy(*bufferP, dataP->value.asBuffer.buffer, dataP->value.asBuffer.length);
        return (int)dataP->value.asBuffer.length;

    case LWM2M_CONTENT_TLV:
    case LWM2M_CONTENT_TLV_OLD:
    {
        bool isResourceInstance;

        if (uriP != NULL && LWM2M_URI_IS_SET_RESOURCE(uriP)
                && (size != 1 || dataP->id != uriP->resourceId))
        {
            isResourceInstance = true;
        }
        else
        {
            isResourceInstance = false;
        }
        return tlv_serialize(isResourceInstance, size, dataP, bufferP);
    }

#ifdef LWM2M_CLIENT_MODE
    case LWM2M_CONTENT_LINK:
        return discover_serialize(NULL, uriP, NULL, size, dataP, bufferP);
#endif
#ifdef LWM2M_SUPPORT_JSON
    case LWM2M_CONTENT_JSON:
    case LWM2M_CONTENT_JSON_OLD:
        return json_serialize(uriP, size, dataP, bufferP);
#endif

    default:
        return -1;
    }
}

