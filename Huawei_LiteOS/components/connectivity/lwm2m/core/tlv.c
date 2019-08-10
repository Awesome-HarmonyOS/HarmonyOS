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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <float.h>

#ifndef LWM2M_BIG_ENDIAN
#ifndef LWM2M_LITTLE_ENDIAN
#error Please define LWM2M_BIG_ENDIAN or LWM2M_LITTLE_ENDIAN
#endif
#endif

#define _PRV_TLV_TYPE_MASK 0xC0
#define _PRV_TLV_HEADER_MAX_LENGTH 6

#define _PRV_TLV_TYPE_UNKNOWN           (uint8_t)0xFF
#define _PRV_TLV_TYPE_OBJECT            (uint8_t)0x10
#define _PRV_TLV_TYPE_OBJECT_INSTANCE   (uint8_t)0x00
#define _PRV_TLV_TYPE_RESOURCE          (uint8_t)0xC0
#define _PRV_TLV_TYPE_MULTIPLE_RESOURCE (uint8_t)0x80
#define _PRV_TLV_TYPE_RESOURCE_INSTANCE (uint8_t)0x40

static size_t prv_encodeFloat(double data,
                              uint8_t *data_buffer)
{
    size_t length = 0;

    if ((data < 0.0 - (double)FLT_MAX) || (data > (double)FLT_MAX))
    {
        length = 8;
        utils_copyValue(data_buffer, &data, 8);
    }
    else
    {
        float value;

        length = 4;
        value = (float)data;
        utils_copyValue(data_buffer, &value, 4);
    }

    return length;
}

static size_t prv_encodeInt(int64_t data,
                            uint8_t *data_buffer)
{
    size_t length = 0;

    if (data >= INT8_MIN && data <= INT8_MAX)
    {
        length = 1;
        data_buffer[0] = data;
    }
    else if (data >= INT16_MIN && data <= INT16_MAX)
    {
        int16_t value;

        value = data;
        length = 2;
        data_buffer[0] = (value >> 8) & 0xFF;
        data_buffer[1] = value & 0xFF;
    }
    else if (data >= INT32_MIN && data <= INT32_MAX)
    {
        int32_t value;

        value = data;
        length = 4;
        utils_copyValue(data_buffer, &value, length);
    }
    else if (data >= INT64_MIN && data <= INT64_MAX)
    {
        length = 8;
        utils_copyValue(data_buffer, &data, length);
    }

    return length;
}

static uint8_t prv_getHeaderType(lwm2m_data_type_t type)
{
    switch (type)
    {
    case LWM2M_TYPE_OBJECT:
        return _PRV_TLV_TYPE_OBJECT;

    case LWM2M_TYPE_OBJECT_INSTANCE:
        return _PRV_TLV_TYPE_OBJECT_INSTANCE;

    case LWM2M_TYPE_MULTIPLE_RESOURCE:
        return _PRV_TLV_TYPE_MULTIPLE_RESOURCE;


    case LWM2M_TYPE_STRING:
    case LWM2M_TYPE_INTEGER:
    case LWM2M_TYPE_FLOAT:
    case LWM2M_TYPE_BOOLEAN:
    case LWM2M_TYPE_OPAQUE:
    case LWM2M_TYPE_OBJECT_LINK:
        return _PRV_TLV_TYPE_RESOURCE;

    case LWM2M_TYPE_UNDEFINED:
    default:
        return _PRV_TLV_TYPE_UNKNOWN;
    }
}

static lwm2m_data_type_t prv_getDataType(uint8_t type)
{
    switch (type)
    {
    case _PRV_TLV_TYPE_OBJECT:
        return LWM2M_TYPE_OBJECT;

    case _PRV_TLV_TYPE_OBJECT_INSTANCE:
        return LWM2M_TYPE_OBJECT_INSTANCE;

    case _PRV_TLV_TYPE_MULTIPLE_RESOURCE:
        return LWM2M_TYPE_MULTIPLE_RESOURCE;

    case _PRV_TLV_TYPE_RESOURCE:
    case _PRV_TLV_TYPE_RESOURCE_INSTANCE:
        return LWM2M_TYPE_OPAQUE;

    default:
        return LWM2M_TYPE_UNDEFINED;
    }
}

static int prv_getHeaderLength(uint16_t id,
                               size_t dataLen)
{
    int length;

    length = 2;

    if (id > 0xFF)
    {
        length += 1;
    }

    if (dataLen > 0xFFFF)
    {
        length += 3;
    }
    else if (dataLen > 0xFF)
    {
        length += 2;
    }
    else if (dataLen > 7)
    {
        length += 1;
    }

    return length;
}

static int prv_createHeader(uint8_t *header,
                            bool isInstance,
                            lwm2m_data_type_t type,
                            uint16_t id,
                            size_t data_len)
{
    int header_len;
    int offset;
    uint8_t hdrType;

    header_len = prv_getHeaderLength(id, data_len);
    if (isInstance == true)
    {
        hdrType = _PRV_TLV_TYPE_RESOURCE_INSTANCE;
    }
    else
    {
        hdrType = prv_getHeaderType(type);
    }

    header[0] = 0;
    header[0] |= hdrType & _PRV_TLV_TYPE_MASK;

    if (id > 0xFF)
    {
        header[0] |= 0x20;
        header[1] = (id >> 8) & 0XFF;
        header[2] = id & 0XFF;
        offset = 3;
    }
    else
    {
        header[1] = id;
        offset = 2;
    }
    if (data_len <= 7)
    {
        header[0] += data_len;
    }
    else if (data_len <= 0xFF)
    {
        header[0] |= 0x08;
        header[offset] = data_len;
    }
    else if (data_len <= 0xFFFF)
    {
        header[0] |= 0x10;
        header[offset] = (data_len >> 8) & 0XFF;
        header[offset + 1] = data_len & 0XFF;
    }
    else if (data_len <= 0xFFFFFF)
    {
        header[0] |= 0x18;
        header[offset] = (data_len >> 16) & 0XFF;
        header[offset + 1] = (data_len >> 8) & 0XFF;
        header[offset + 2] = data_len & 0XFF;
    }

    return header_len;
}

int lwm2m_decode_TLV(const uint8_t *buffer,
                     size_t buffer_len,
                     lwm2m_data_type_t *oType,
                     uint16_t *oID,
                     size_t *oDataIndex,
                     size_t *oDataLen)
{

    LOG_ARG("buffer_len: %d", buffer_len);
    ;
    if (buffer_len < 2) return 0;

    *oDataIndex = 2;

    *oType = prv_getDataType(buffer[0] & _PRV_TLV_TYPE_MASK);

    if ((buffer[0] & 0x20) == 0x20)
    {
        // id is 16 bits long
        if (buffer_len < 3) return 0;
        *oDataIndex += 1;
        *oID = (buffer[1] << 8) + buffer[2];
    }
    else
    {
        // id is 8 bits long
        *oID = buffer[1];
    }

    switch (buffer[0] & 0x18)
    {
    case 0x00:
        // no length field
        *oDataLen = buffer[0] & 0x07;
        break;
    case 0x08:
        // length field is 8 bits long
        if (buffer_len < *oDataIndex + 1) return 0;
        *oDataLen = buffer[*oDataIndex];
        *oDataIndex += 1;
        break;
    case 0x10:
        // length field is 16 bits long
        if (buffer_len < *oDataIndex + 2) return 0;
        *oDataLen = (buffer[*oDataIndex] << 8) + buffer[*oDataIndex + 1];
        *oDataIndex += 2;
        break;
    case 0x18:
        // length field is 24 bits long
        if (buffer_len < *oDataIndex + 3) return 0;
        *oDataLen = (buffer[*oDataIndex] << 16) + (buffer[*oDataIndex + 1] << 8) + buffer[*oDataIndex + 2];
        *oDataIndex += 3;
        break;
    default:
        // can't happen
        return 0;
    }

    if (*oDataIndex + *oDataLen > buffer_len) return 0;

    return *oDataIndex + *oDataLen;
}


int tlv_parse(uint8_t *buffer,
              size_t bufferLen,
              lwm2m_data_t **dataP)
{
    lwm2m_data_type_t type;
    uint16_t id;
    size_t dataIndex;
    size_t dataLen;
    int index = 0;
    int result;
    int size = 0;

    LOG_ARG("bufferLen: %d", bufferLen);

    *dataP = NULL;

    while (0 != (result = lwm2m_decode_TLV((uint8_t *)buffer + index, bufferLen - index, &type, &id, &dataIndex, &dataLen)))
    {
        lwm2m_data_t *newTlvP;

        newTlvP = lwm2m_data_new(size + 1);
        if (size >= 1)
        {
            if (newTlvP == NULL)
            {
                lwm2m_data_free(size, *dataP);
                return 0;
            }
            else
            {
                memcpy(newTlvP, *dataP, size * sizeof(lwm2m_data_t));
                lwm2m_free(*dataP);
            }
        }
        *dataP = newTlvP;

        (*dataP)[size].type = type;
        (*dataP)[size].id = id;
        if (type == LWM2M_TYPE_OBJECT_INSTANCE || type == LWM2M_TYPE_MULTIPLE_RESOURCE)
        {
            (*dataP)[size].value.asChildren.count = tlv_parse(buffer + index + dataIndex,
                                                    dataLen,
                                                    &((*dataP)[size].value.asChildren.array));
            if ((*dataP)[size].value.asChildren.count == 0)
            {
                lwm2m_data_free(size + 1, *dataP);
                return 0;
            }
        }
        else
        {
            lwm2m_data_encode_opaque(buffer + index + dataIndex, dataLen, (*dataP) + size);
        }
        size++;
        index += result;
    }

    return size;
}


static int prv_getLength(int size,
                         lwm2m_data_t *dataP)
{
    int length;
    int i;

    length = 0;

    for (i = 0 ; i < size && length != -1 ; i++)
    {
        switch (dataP[i].type)
        {
        case LWM2M_TYPE_OBJECT_INSTANCE:
        case LWM2M_TYPE_MULTIPLE_RESOURCE:
        {
            int subLength;

            subLength = prv_getLength(dataP[i].value.asChildren.count, dataP[i].value.asChildren.array);
            if (subLength == -1)
            {
                length = -1;
            }
            else
            {
                length += prv_getHeaderLength(dataP[i].id, subLength) + subLength;
            }
        }
        break;

        case LWM2M_TYPE_STRING:
        case LWM2M_TYPE_OPAQUE:
            length += prv_getHeaderLength(dataP[i].id, dataP[i].value.asBuffer.length) + dataP[i].value.asBuffer.length;
            break;

        case LWM2M_TYPE_INTEGER:
        {
            size_t data_len;
            uint8_t unused_buffer[_PRV_64BIT_BUFFER_SIZE];

            data_len = prv_encodeInt(dataP[i].value.asInteger, unused_buffer);
            length += prv_getHeaderLength(dataP[i].id, data_len) + data_len;
        }
        break;

        case LWM2M_TYPE_FLOAT:
        {
            size_t data_len;

            if ((dataP[i].value.asFloat < 0.0 - (double)FLT_MAX)
                    || (dataP[i].value.asFloat > (double)FLT_MAX))
            {
                data_len = 8;
            }
            else
            {
                data_len = 4;
            }

            length += prv_getHeaderLength(dataP[i].id, data_len) + data_len;
        }
        break;

        case LWM2M_TYPE_BOOLEAN:
            // Booleans are always encoded on one byte
            length += prv_getHeaderLength(dataP[i].id, 1) + 1;
            break;

        case LWM2M_TYPE_OBJECT_LINK:
            // Object Link are always encoded on four bytes
            length += prv_getHeaderLength(dataP[i].id, 4) + 4;
            break;

        default:
            length = -1;
            break;
        }
    }

    return length;
}


int tlv_serialize(bool isResourceInstance,
                  int size,
                  lwm2m_data_t *dataP,
                  uint8_t **bufferP)
{
    int length;
    int index;
    int i;

    LOG_ARG("isResourceInstance: %s, size: %d", isResourceInstance ? "true" : "false", size);

    *bufferP = NULL;
    length = prv_getLength(size, dataP);
    if (length <= 0) return length;

    *bufferP = (uint8_t *)lwm2m_malloc(length);
    if (*bufferP == NULL) return 0;

    index = 0;
    for (i = 0 ; i < size && length != 0 ; i++)
    {
        int headerLen;
        bool isInstance;

        isInstance = isResourceInstance;
        switch (dataP[i].type)
        {
        case LWM2M_TYPE_MULTIPLE_RESOURCE:
            isInstance = true;
        // fall through
        case LWM2M_TYPE_OBJECT_INSTANCE:
        {
            uint8_t *tmpBuffer;
            int res;

            res = tlv_serialize(isInstance, dataP[i].value.asChildren.count, dataP[i].value.asChildren.array, &tmpBuffer);
            if (res < 0)
            {
                length = -1;
            }
            else
            {
                size_t tmpLength;

                tmpLength = (size_t)res;
                headerLen = prv_createHeader(*bufferP + index, false, dataP[i].type, dataP[i].id, tmpLength);
                index += headerLen;
                if (tmpLength > 0)
                {
                    memcpy(*bufferP + index, tmpBuffer, tmpLength);
                    index += tmpLength;
                    lwm2m_free(tmpBuffer);
                }
            }
        }
        break;

        case LWM2M_TYPE_OBJECT_LINK:
        {
            int k;
            uint8_t buf[4];
            uint32_t v = dataP[i].value.asObjLink.objectId;
            v <<= 16;
            v |= dataP[i].value.asObjLink.objectInstanceId;
            for (k = 3; k >= 0; --k)
            {
                buf[k] = (uint8_t)(v & 0xFF);
                v >>= 8;
            }
            // keep encoding as buffer
            headerLen = prv_createHeader(*bufferP + index, isInstance, dataP[i].type, dataP[i].id, 4);
            index += headerLen;
            memcpy(*bufferP + index, buf, 4);
            index += 4;
        }
        break;

        case LWM2M_TYPE_STRING:
        case LWM2M_TYPE_OPAQUE:
            headerLen = prv_createHeader(*bufferP + index, isInstance, dataP[i].type, dataP[i].id, dataP[i].value.asBuffer.length);
            index += headerLen;
            memcpy(*bufferP + index, dataP[i].value.asBuffer.buffer, dataP[i].value.asBuffer.length);
            index += dataP[i].value.asBuffer.length;
            break;

        case LWM2M_TYPE_INTEGER:
        {
            size_t data_len;
            uint8_t data_buffer[_PRV_64BIT_BUFFER_SIZE];

            data_len = prv_encodeInt(dataP[i].value.asInteger, data_buffer);
            headerLen = prv_createHeader(*bufferP + index, isInstance, dataP[i].type, dataP[i].id, data_len);
            index += headerLen;
            memcpy(*bufferP + index, data_buffer, data_len);
            index += data_len;
        }
        break;

        case LWM2M_TYPE_FLOAT:
        {
            size_t data_len;
            uint8_t data_buffer[_PRV_64BIT_BUFFER_SIZE];

            data_len = prv_encodeFloat(dataP[i].value.asFloat, data_buffer);
            headerLen = prv_createHeader(*bufferP + index, isInstance, dataP[i].type, dataP[i].id, data_len);
            index += headerLen;
            memcpy(*bufferP + index, data_buffer, data_len);
            index += data_len;
        }
        break;

        case LWM2M_TYPE_BOOLEAN:
            headerLen = prv_createHeader(*bufferP + index, isInstance, dataP[i].type, dataP[i].id, 1);
            index += headerLen;
            (*bufferP)[index] = dataP[i].value.asBoolean ? 1 : 0;
            index += 1;
            break;

        default:
            length = -1;
            break;
        }
    }

    if (length < 0)
    {
        lwm2m_free(*bufferP);
        *bufferP = NULL;
    }

    LOG_ARG("returning %u", length);

    return length;
}

