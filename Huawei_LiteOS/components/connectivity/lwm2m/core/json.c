/*******************************************************************************
 *
 * Copyright (c) 2015 Intel Corporation and others.
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
 *
 *******************************************************************************/


#include "internals.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>


#ifdef LWM2M_SUPPORT_JSON

#define PRV_JSON_BUFFER_SIZE 1024

#define JSON_MIN_ARRAY_LEN      21      // e":[{"n":"N","v":X}]}
#define JSON_MIN_BASE_LEN        7      // n":"N",
#define JSON_ITEM_MAX_SIZE      36      // with ten characters for value
#define JSON_MIN_BX_LEN          5      // bt":1

#define JSON_FALSE_STRING  "false"
#define JSON_TRUE_STRING   "true"

#define JSON_RES_ITEM_URI           "{\"n\":\""
#define JSON_RES_ITEM_URI_SIZE      6
#define JSON_ITEM_BOOL_TRUE         "\",\"bv\":true},"
#define JSON_ITEM_BOOL_TRUE_SIZE    13
#define JSON_ITEM_BOOL_FALSE        "\",\"bv\":false},"
#define JSON_ITEM_BOOL_FALSE_SIZE   14
#define JSON_ITEM_NUM               "\",\"v\":"
#define JSON_ITEM_NUM_SIZE          6
#define JSON_ITEM_NUM_END           "},"
#define JSON_ITEM_NUM_END_SIZE      2
#define JSON_ITEM_STRING_BEGIN      "\",\"sv\":\""
#define JSON_ITEM_STRING_BEGIN_SIZE 8
#define JSON_ITEM_STRING_END        "\"},"
#define JSON_ITEM_STRING_END_SIZE   3

#define JSON_BN_HEADER_1        "{\"bn\":\""
#define JSON_BN_HEADER_1_SIZE   7
#define JSON_BN_HEADER_2        "\",\"e\":["
#define JSON_BN_HEADER_2_SIZE   7
#define JSON_HEADER             "{\"e\":["
#define JSON_HEADER_SIZE        6
#define JSON_FOOTER             "]}"
#define JSON_FOOTER_SIZE        2


#define _GO_TO_NEXT_CHAR(I,B,L)         \
    {                                   \
        I++;                            \
        I += prv_skipSpace(B+I, L-I);   \
        if (I == L) goto error;         \
    }

typedef enum
{
    _STEP_START,
    _STEP_TOKEN,
    _STEP_ANY_SEPARATOR,
    _STEP_SEPARATOR,
    _STEP_QUOTED_VALUE,
    _STEP_VALUE,
    _STEP_DONE
} _itemState;

typedef enum
{
    _TYPE_UNSET,
    _TYPE_FALSE,
    _TYPE_TRUE,
    _TYPE_FLOAT,
    _TYPE_STRING
} _type;

typedef struct
{
    uint16_t    ids[4];
    _type       type;
    uint8_t    *value;
    size_t      valueLen;
} _record_t;

static int prv_isReserved(char sign)
{
    if (sign == '['
            || sign == '{'
            || sign == ']'
            || sign == '}'
            || sign == ':'
            || sign == ','
            || sign == '"')
    {
        return 1;
    }

    return 0;
}

static int prv_isWhiteSpace(uint8_t sign)
{
    if (sign == 0x20
            || sign == 0x09
            || sign == 0x0A
            || sign == 0x0D)
    {
        return 1;
    }

    return 0;
}

static size_t prv_skipSpace(uint8_t *buffer,
                            size_t bufferLen)
{
    size_t i;

    i = 0;
    while ((i < bufferLen)
            && prv_isWhiteSpace(buffer[i]))
    {
        i++;
    }

    return i;
}

static int prv_split(uint8_t *buffer,
                     size_t bufferLen,
                     int *tokenStartP,
                     int *tokenLenP,
                     int *valueStartP,
                     int *valueLenP)
{
    size_t index;
    _itemState step;

    index = 0;
    step = _STEP_START;

    index = prv_skipSpace(buffer + index, bufferLen - index);
    if (index == bufferLen) return -1;

    while ((index < bufferLen)
            && (buffer[index] != ','))
    {
        switch (step)
        {
        case _STEP_START:
            if (buffer[index] != '"') return -1;
            *tokenStartP = index + 1;
            step = _STEP_TOKEN;
            break;

        case _STEP_TOKEN:
            if (buffer[index] == '"')
            {
                *tokenLenP = index - *tokenStartP;
                step = _STEP_ANY_SEPARATOR;
            }
            break;

        case _STEP_ANY_SEPARATOR:
            if (buffer[index] != ':') return -1;
            step = _STEP_SEPARATOR;
            break;

        case _STEP_SEPARATOR:
            if (buffer[index] == '"')
            {
                *valueStartP = index;
                step = _STEP_QUOTED_VALUE;
            }
            else if (!prv_isReserved(buffer[index]))
            {
                *valueStartP = index;
                step = _STEP_VALUE;
            }
            else
            {
                return -1;
            }
            break;

        case _STEP_QUOTED_VALUE:
            if (buffer[index] == '"' && buffer[index - 1] != '\\' )
            {
                *valueLenP = index - *valueStartP + 1;
                step = _STEP_DONE;
            }
            break;

        case _STEP_VALUE:
            if (prv_isWhiteSpace(buffer[index]))
            {
                *valueLenP = index - *valueStartP;
                step = _STEP_DONE;
            }
            break;

        case _STEP_DONE:
        default:
            return -1;
        }

        index++;
        if (step == _STEP_START
                || step == _STEP_ANY_SEPARATOR
                || step == _STEP_SEPARATOR
                || step == _STEP_DONE)
        {
            index += prv_skipSpace(buffer + index, bufferLen - index);
        }
    }

    if (step == _STEP_VALUE)
    {
        *valueLenP = index - *valueStartP;
        step = _STEP_DONE;
    }

    if (step != _STEP_DONE) return -1;

    return (int)index;
}

static int prv_countItems(uint8_t *buffer,
                          size_t bufferLen)
{
    int count;
    size_t index;
    int in;

    count = 0;
    index = 0;
    in = 0;

    while (index < bufferLen)
    {
        if (in == 0)
        {
            if (buffer[index] != '{') return -1;
            in = 1;
            _GO_TO_NEXT_CHAR(index, buffer, bufferLen);
        }
        else
        {
            if (buffer[index] == '{') return -1;
            if (buffer[index] == '}')
            {
                in = 0;
                count++;
                _GO_TO_NEXT_CHAR(index, buffer, bufferLen);
                if (buffer[index] == ']')
                {
                    break;
                }
                if (buffer[index] != ',') return -1;
                _GO_TO_NEXT_CHAR(index, buffer, bufferLen);
            }
            else
            {
                _GO_TO_NEXT_CHAR(index, buffer, bufferLen);
            }
        }
    }
    if (in == 1) return -1;

    return count;

error:
    return -1;
}

static int prv_parseItem(uint8_t *buffer,
                         size_t bufferLen,
                         _record_t *recordP)
{
    size_t index;

    memset(recordP->ids, 0xFF, 4 * sizeof(uint16_t));
    recordP->type = _TYPE_UNSET;
    recordP->value = NULL;
    recordP->valueLen = 0;

    index = 0;
    do
    {
        int tokenStart;
        int tokenLen;
        int valueStart;
        int valueLen;
        int next;

        next = prv_split(buffer + index, bufferLen - index, &tokenStart, &tokenLen, &valueStart, &valueLen);
        if (next < 0) return -1;

        switch (tokenLen)
        {
        case 1:
        {
            switch (buffer[index + tokenStart])
            {
            case 'n':
            {
                int i;
                int j;

                if (recordP->ids[0] != LWM2M_MAX_ID) return -1;

                // Check for " around URI
                if (valueLen < 3
                        || buffer[index + valueStart] != '"'
                        || buffer[index + valueStart + valueLen - 1] != '"')
                {
                    return -1;
                }
                // Ignore starting /
                if (buffer[index + valueStart + 1] == '/')
                {
                    if (valueLen < 4)
                    {
                        return -1;
                    }
                    valueStart += 1;
                    valueLen -= 1;
                }
                i = 0;
                j = 0;
                do
                {
                    uint32_t readId;

                    readId = 0;
                    i++;
                    while (i < valueLen - 1 && buffer[index + valueStart + i] != '/')
                    {
                        if (buffer[index + valueStart + i] < '0'
                                || buffer[index + valueStart + i] > '9')
                        {
                            return -1;
                        }
                        readId *= 10;
                        readId += buffer[index + valueStart + i] - '0';
                        if (readId > LWM2M_MAX_ID) return -1;
                        i++;
                    }
                    recordP->ids[j] = readId;
                    j++;
                }
                while (i < valueLen - 1 && j < 4 && buffer[index + valueStart + i] == '/');
                if (i < valueLen - 1 ) return -1;
            }
            break;

            case 'v':
                if (recordP->type != _TYPE_UNSET) return -1;
                recordP->type = _TYPE_FLOAT;
                recordP->value = buffer + index + valueStart;
                recordP->valueLen = valueLen;
                break;

            case 't':
                // TODO: support time
                break;

            default:
                return -1;
            }
        }
        break;

        case 2:
        {
            // "bv", "ov", or "sv"
            if (buffer[index + tokenStart + 1] != 'v') return -1;
            switch (buffer[index + tokenStart])
            {
            case 'b':
                if (recordP->type != _TYPE_UNSET) return -1;
                if (0 == lwm2m_strncmp(JSON_TRUE_STRING, (char *)buffer + index + valueStart, valueLen))
                {
                    recordP->type = _TYPE_TRUE;
                }
                else if (0 == lwm2m_strncmp(JSON_FALSE_STRING, (char *)buffer + index + valueStart, valueLen))
                {
                    recordP->type = _TYPE_FALSE;
                }
                else
                {
                    return -1;
                }
                break;

            case 'o':
                if (recordP->type != _TYPE_UNSET) return -1;
                // TODO: support object link
                break;

            case 's':
                if (recordP->type != _TYPE_UNSET) return -1;
                // Check for " around value
                if (valueLen < 2
                        || buffer[index + valueStart] != '"'
                        || buffer[index + valueStart + valueLen - 1] != '"')
                {
                    return -1;
                }
                recordP->type = _TYPE_STRING;
                recordP->value = buffer + index + valueStart + 1;
                recordP->valueLen = valueLen - 2;
                break;

            default:
                return -1;
            }
        }
        break;

        default:
            return -1;
        }

        index += next + 1;
    }
    while (index < bufferLen);

    return 0;
}

static bool prv_convertValue(_record_t *recordP,
                             lwm2m_data_t *targetP)
{
    switch (recordP->type)
    {
    case _TYPE_FALSE:
        lwm2m_data_encode_bool(false, targetP);
        break;
    case _TYPE_TRUE:
        lwm2m_data_encode_bool(true, targetP);
        break;
    case _TYPE_FLOAT:
    {
        size_t i;

        i = 0;
        while (i < recordP->valueLen
                && recordP->value[i] != '.')
        {
            i++;
        }
        if (i == recordP->valueLen)
        {
            int64_t value;

            if ( 1 != utils_textToInt(recordP->value,
                                      recordP->valueLen,
                                      &value))
            {
                return false;
            }

            lwm2m_data_encode_int(value, targetP);
        }
        else
        {
            double value;

            if ( 1 != utils_textToFloat(recordP->value,
                                        recordP->valueLen,
                                        &value))
            {
                return false;
            }

            lwm2m_data_encode_float(value, targetP);
        }
    }
    break;

    case _TYPE_STRING:
        lwm2m_data_encode_opaque(recordP->value, recordP->valueLen, targetP);
        targetP->type = LWM2M_TYPE_STRING;
        break;

    case _TYPE_UNSET:
    default:
        return false;
    }

    return true;
}

static lwm2m_data_t *prv_findDataItem(lwm2m_data_t *listP,
                                      int count,
                                      uint16_t id)
{
    int i;

    i = 0;
    while (i < count)
    {
        if (listP[i].type != LWM2M_TYPE_UNDEFINED && listP[i].id == id)
        {
            return listP + i;
        }
        i++;
    }

    return NULL;
}

static uri_depth_t prv_decreaseLevel(uri_depth_t level)
{
    switch(level)
    {
    case URI_DEPTH_OBJECT:
        return URI_DEPTH_OBJECT_INSTANCE;
    case URI_DEPTH_OBJECT_INSTANCE:
        return URI_DEPTH_RESOURCE;
    case URI_DEPTH_RESOURCE:
        return URI_DEPTH_RESOURCE_INSTANCE;
    case URI_DEPTH_RESOURCE_INSTANCE:
        return URI_DEPTH_RESOURCE_INSTANCE;
    default:
        return URI_DEPTH_RESOURCE;
    }
}

static lwm2m_data_t *prv_extendData(lwm2m_data_t *parentP)
{
    lwm2m_data_t *newP;

    newP = lwm2m_data_new(parentP->value.asChildren.count + 1);
    if (newP == NULL) return NULL;
    if (parentP->value.asChildren.array != NULL)
    {
        memcpy(newP, parentP->value.asChildren.array, parentP->value.asChildren.count * sizeof(lwm2m_data_t));
        lwm2m_free(parentP->value.asChildren.array);     // do not use lwm2m_data_free() to keep pointed values
    }
    parentP->value.asChildren.array = newP;
    parentP->value.asChildren.count += 1;

    return newP + parentP->value.asChildren.count - 1;
}

static int prv_convertRecord(lwm2m_uri_t *uriP,
                             _record_t *recordArray,
                             int count,
                             lwm2m_data_t **dataP)
{
    int index;
    int freeIndex;
    lwm2m_data_t *rootP;
    int size;
    uri_depth_t rootLevel;

    if (uriP == NULL)
    {
        size = count;
        *dataP = lwm2m_data_new(count);
        if (NULL == *dataP) return -1;
        rootLevel = URI_DEPTH_OBJECT;
        rootP = *dataP;
    }
    else
    {
        lwm2m_data_t *parentP;
        size = 1;

        *dataP = lwm2m_data_new(1);
        if (NULL == *dataP) return -1;
        (*dataP)->type = LWM2M_TYPE_OBJECT;
        (*dataP)->id = uriP->objectId;
        rootLevel = URI_DEPTH_OBJECT_INSTANCE;
        parentP = *dataP;
        if (LWM2M_URI_IS_SET_INSTANCE(uriP))
        {
            parentP->value.asChildren.count = 1;
            parentP->value.asChildren.array = lwm2m_data_new(1);
            if (NULL == parentP->value.asChildren.array) goto error;
            parentP = parentP->value.asChildren.array;
            parentP->type = LWM2M_TYPE_OBJECT_INSTANCE;
            parentP->id = uriP->instanceId;
            rootLevel = URI_DEPTH_RESOURCE;
            if (LWM2M_URI_IS_SET_RESOURCE(uriP))
            {
                parentP->value.asChildren.count = 1;
                parentP->value.asChildren.array = lwm2m_data_new(1);
                if (NULL == parentP->value.asChildren.array) goto error;
                parentP = parentP->value.asChildren.array;
                parentP->type = LWM2M_TYPE_UNDEFINED;
                parentP->id = uriP->resourceId;
                rootLevel = URI_DEPTH_RESOURCE_INSTANCE;
            }
        }
        parentP->value.asChildren.count = count;
        parentP->value.asChildren.array = lwm2m_data_new(count);
        if (NULL == parentP->value.asChildren.array) goto error;
        rootP = parentP->value.asChildren.array;
    }

    freeIndex = 0;
    for (index = 0 ; index < count ; index++)
    {
        lwm2m_data_t *targetP;
        int resSegmentIndex;
        int i;

        // check URI depth
        // resSegmentIndex is set to the resource segment position
        switch(rootLevel)
        {
        case URI_DEPTH_OBJECT:
            resSegmentIndex = 2;
            break;
        case URI_DEPTH_OBJECT_INSTANCE:
            resSegmentIndex = 1;
            break;
        case URI_DEPTH_RESOURCE:
            resSegmentIndex = 0;
            break;
        case URI_DEPTH_RESOURCE_INSTANCE:
            resSegmentIndex = -1;
            break;
        default:
            goto error;
        }
        for (i = 0 ; i <= resSegmentIndex ; i++)
        {
            if (recordArray[index].ids[i] == LWM2M_MAX_ID) goto error;
        }
        if (resSegmentIndex < 2)
        {
            if (recordArray[index].ids[resSegmentIndex + 2] != LWM2M_MAX_ID) goto error;
        }

        targetP = prv_findDataItem(rootP, count, recordArray[index].ids[0]);
        if (targetP == NULL)
        {
            targetP = rootP + freeIndex;
            freeIndex++;
            targetP->id = recordArray[index].ids[0];
            targetP->type = utils_depthToDatatype(rootLevel);
        }
        if (recordArray[index].ids[1] != LWM2M_MAX_ID)
        {
            lwm2m_data_t *parentP;
            uri_depth_t level;

            parentP = targetP;
            level = prv_decreaseLevel(rootLevel);
            for (i = 1 ; i <= resSegmentIndex ; i++)
            {
                targetP = prv_findDataItem(parentP->value.asChildren.array, parentP->value.asChildren.count, recordArray[index].ids[i]);
                if (targetP == NULL)
                {
                    targetP = prv_extendData(parentP);
                    if (targetP == NULL) goto error;
                    targetP->id = recordArray[index].ids[i];
                    targetP->type = utils_depthToDatatype(level);
                }
                level = prv_decreaseLevel(level);
                parentP = targetP;
            }
            if (recordArray[index].ids[resSegmentIndex + 1] != LWM2M_MAX_ID)
            {
                targetP->type = LWM2M_TYPE_MULTIPLE_RESOURCE;
                targetP = prv_extendData(targetP);
                if (targetP == NULL) goto error;
                targetP->id = recordArray[index].ids[resSegmentIndex + 1];
                targetP->type = LWM2M_TYPE_UNDEFINED;
            }
        }

        if (true != prv_convertValue(recordArray + index, targetP)) goto error;
    }

    return size;

error:
    lwm2m_data_free(size, *dataP);
    *dataP = NULL;

    return -1;
}

static int prv_dataStrip(int size,
                         lwm2m_data_t *dataP,
                         lwm2m_data_t **resultP)
{
    int i;
    int j;
    int realSize;

    realSize = 0;
    for (i = 0 ; i < size ; i++)
    {
        if (dataP[i].type != LWM2M_TYPE_UNDEFINED)
        {
            realSize++;
        }
    }

    *resultP = lwm2m_data_new(realSize);
    if (*resultP == NULL) return -1;

    j = 0;
    for (i = 0 ; i < size ; i++)
    {
        if (dataP[i].type != LWM2M_TYPE_UNDEFINED)
        {
            memcpy((*resultP) + j, dataP + i, sizeof(lwm2m_data_t));

            if (dataP[i].type == LWM2M_TYPE_OBJECT
                    || dataP[i].type == LWM2M_TYPE_OBJECT_INSTANCE
                    || dataP[i].type == LWM2M_TYPE_MULTIPLE_RESOURCE)
            {
                int childLen;

                childLen = prv_dataStrip(dataP[i].value.asChildren.count, dataP[i].value.asChildren.array, &((*resultP)[j].value.asChildren.array));
                if (childLen <= 0)
                {
                    // skip this one
                    j--;
                }
                else
                {
                    (*resultP)[j].value.asChildren.count = childLen;
                }
            }
            else
            {
                dataP[i].value.asBuffer.buffer = NULL;
            }

            j++;
        }
    }

    return realSize;
}

int json_parse(lwm2m_uri_t *uriP,
               uint8_t *buffer,
               size_t bufferLen,
               lwm2m_data_t **dataP)
{
    size_t index;
    int count = 0;
    bool eFound = false;
    bool bnFound = false;
    bool btFound = false;
    int bnStart;
    int bnLen;
    _record_t *recordArray;
    lwm2m_data_t *parsedP;

    LOG_ARG("bufferLen: %d, buffer: \"%s\"", bufferLen, (char *)buffer);
    LOG_URI(uriP);
    *dataP = NULL;
    recordArray = NULL;
    parsedP = NULL;

    index = prv_skipSpace(buffer, bufferLen);
    if (index == bufferLen) return -1;

    if (buffer[index] != '{') return -1;
    do
    {
        _GO_TO_NEXT_CHAR(index, buffer, bufferLen);
        if (buffer[index] != '"') goto error;
        if (index++ >= bufferLen) goto error;
        switch (buffer[index])
        {
        case 'e':
        {
            int recordIndex;

            if (bufferLen - index < JSON_MIN_ARRAY_LEN) goto error;
            index++;
            if (buffer[index] != '"') goto error;
            if (eFound == true) goto error;
            eFound = true;

            _GO_TO_NEXT_CHAR(index, buffer, bufferLen);
            if (buffer[index] != ':') goto error;
            _GO_TO_NEXT_CHAR(index, buffer, bufferLen);
            if (buffer[index] != '[') goto error;
            _GO_TO_NEXT_CHAR(index, buffer, bufferLen);
            count = prv_countItems(buffer + index, bufferLen - index);
            if (count <= 0) goto error;
            recordArray = (_record_t *)lwm2m_malloc(count * sizeof(_record_t));
            if (recordArray == NULL) goto error;
            // at this point we are sure buffer[index] is '{' and all { and } are matching
            recordIndex = 0;
            while (recordIndex < count)
            {
                int itemLen;

                if (buffer[index] != '{') goto error;
                itemLen = 0;
                while (buffer[index + itemLen] != '}') itemLen++;
                if (0 != prv_parseItem(buffer + index + 1, itemLen - 1, recordArray + recordIndex))
                {
                    goto error;
                }
                recordIndex++;
                index += itemLen;
                _GO_TO_NEXT_CHAR(index, buffer, bufferLen);
                switch (buffer[index])
                {
                case ',':
                    _GO_TO_NEXT_CHAR(index, buffer, bufferLen);
                    break;
                case ']':
                    if (recordIndex == count) break;
                // else this is an error
                default:
                    goto error;
                }
            }
            if (buffer[index] != ']') goto error;
        }
        break;

        case 'b':
            if (bufferLen - index < JSON_MIN_BX_LEN) goto error;
            index++;
            switch (buffer[index])
            {
            case 't':
                index++;
                if (buffer[index] != '"') goto error;
                if (btFound == true) goto error;
                btFound = true;

                // TODO: handle timed values
                // temp: skip this token
                while(index < bufferLen && buffer[index] != ',' && buffer[index] != '}') index++;
                if (index == bufferLen) goto error;
                index--;
                // end temp
                break;
            case 'n':
            {
                int next;
                int tokenStart;
                int tokenLen;
                int itemLen;

                index++;
                if (buffer[index] != '"') goto error;
                if (bnFound == true) goto error;
                bnFound = true;
                index -= 3;
                itemLen = 0;
                while (buffer[index + itemLen] != '}'
                        && buffer[index + itemLen] != ','
                        && index + itemLen < bufferLen)
                {
                    itemLen++;
                }
                if (index + itemLen == bufferLen) goto error;
                next = prv_split(buffer + index, itemLen, &tokenStart, &tokenLen, &bnStart, &bnLen);
                if (next < 0) goto error;
                bnStart += index;
                index += next - 1;
            }
            break;
            default:
                goto error;
            }
            break;

        default:
            goto error;
        }

        _GO_TO_NEXT_CHAR(index, buffer, bufferLen);
    }
    while (buffer[index] == ',');

    if (buffer[index] != '}') goto error;

    if (eFound == true)
    {
        lwm2m_uri_t baseURI;
        lwm2m_uri_t *baseUriP;
        lwm2m_data_t *resultP;
        int size;

        memset(&baseURI, 0, sizeof(lwm2m_uri_t));
        if (bnFound == false)
        {
            baseUriP = uriP;
        }
        else
        {
            int res;

            // we ignore the request URI and use the bn one.

            // Check for " around URI
            if (bnLen < 3
                    || buffer[bnStart] != '"'
                    || buffer[bnStart + bnLen - 1] != '"')
            {
                goto error;
            }
            bnStart += 1;
            bnLen -= 2;

            if (bnLen == 1)
            {
                if (buffer[bnStart] != '/') goto error;
                baseUriP = NULL;
            }
            else
            {
                res = lwm2m_stringToUri((char *)buffer + bnStart, bnLen, &baseURI);
                if (res < 0 || res != bnLen) goto error;
                baseUriP = &baseURI;
            }
        }

        count = prv_convertRecord(baseUriP, recordArray, count, &parsedP);
        lwm2m_free(recordArray);
        recordArray = NULL;

        if (count > 0 && uriP != NULL)
        {
            if (parsedP->type != LWM2M_TYPE_OBJECT || parsedP->id != uriP->objectId) goto error;
            if (!LWM2M_URI_IS_SET_INSTANCE(uriP))
            {
                size = parsedP->value.asChildren.count;
                resultP = parsedP->value.asChildren.array;
            }
            else
            {
                int i;

                resultP = NULL;
                // be permissive and allow full object JSON when requesting for a single instance
                for (i = 0 ; i < (int)parsedP->value.asChildren.count && resultP == NULL; i++)
                {
                    lwm2m_data_t *targetP;

                    targetP = parsedP->value.asChildren.array + i;
                    if (targetP->id == uriP->instanceId)
                    {
                        resultP = targetP->value.asChildren.array;
                        size = targetP->value.asChildren.count;
                    }
                }
                if (resultP == NULL) goto error;
                if (LWM2M_URI_IS_SET_RESOURCE(uriP))
                {
                    lwm2m_data_t *resP;

                    resP = NULL;
                    for (i = 0 ; i < size && resP == NULL; i++)
                    {
                        lwm2m_data_t *targetP;

                        targetP = resultP + i;
                        if (targetP->id == uriP->resourceId)
                        {
                            if (targetP->type == LWM2M_TYPE_MULTIPLE_RESOURCE)
                            {
                                resP = targetP->value.asChildren.array;
                                size = targetP->value.asChildren.count;
                            }
                            else
                            {
                                size = prv_dataStrip(1, targetP, &resP);
                                if (size <= 0) goto error;
                                lwm2m_data_free(count, parsedP);
                                parsedP = NULL;
                            }
                        }
                    }
                    if (resP == NULL) goto error;
                    resultP = resP;
                }
            }
        }
        else
        {
            resultP = parsedP;
            size = count;
        }

        if (parsedP != NULL)
        {
            lwm2m_data_t *tempP;

            size = prv_dataStrip(size, resultP, &tempP);
            if (size <= 0) goto error;
            lwm2m_data_free(count, parsedP);
            resultP = tempP;
        }
        count = size;
        *dataP = resultP;
    }

    LOG_ARG("Parsing successful. count: %d", count);
    return count;

error:
    LOG("Parsing failed");
    if (parsedP != NULL)
    {
        lwm2m_data_free(count, parsedP);
        parsedP = NULL;
    }
    if (recordArray != NULL)
    {
        lwm2m_free(recordArray);
    }
    return -1;
}

static int prv_serializeValue(lwm2m_data_t *tlvP,
                              uint8_t *buffer,
                              size_t bufferLen)
{
    int res;
    int head;

    switch (tlvP->type)
    {
    case LWM2M_TYPE_STRING:
        if (bufferLen < JSON_ITEM_STRING_BEGIN_SIZE) return -1;
        memcpy(buffer, JSON_ITEM_STRING_BEGIN, JSON_ITEM_STRING_BEGIN_SIZE);
        head = JSON_ITEM_STRING_BEGIN_SIZE;

        if (bufferLen - head < tlvP->value.asBuffer.length) return -1;
        memcpy(buffer + head, tlvP->value.asBuffer.buffer, tlvP->value.asBuffer.length);
        head += tlvP->value.asBuffer.length;

        if (bufferLen - head < JSON_ITEM_STRING_END_SIZE) return -1;
        memcpy(buffer + head, JSON_ITEM_STRING_END, JSON_ITEM_STRING_END_SIZE);
        head += JSON_ITEM_STRING_END_SIZE;

        break;

    case LWM2M_TYPE_INTEGER:
    {
        int64_t value;

        if (0 == lwm2m_data_decode_int(tlvP, &value)) return -1;

        if (bufferLen < JSON_ITEM_NUM_SIZE) return -1;
        memcpy(buffer, JSON_ITEM_NUM, JSON_ITEM_NUM_SIZE);
        head = JSON_ITEM_NUM_SIZE;

        res = utils_intToText(value, buffer + head, bufferLen - head);
        if (res <= 0) return -1;
        head += res;

        if (bufferLen - head < JSON_ITEM_NUM_END_SIZE) return -1;
        memcpy(buffer + head, JSON_ITEM_NUM_END, JSON_ITEM_NUM_END_SIZE);
        head += JSON_ITEM_NUM_END_SIZE;
    }
    break;

    case LWM2M_TYPE_FLOAT:
    {
        double value;

        if (0 == lwm2m_data_decode_float(tlvP, &value)) return -1;

        if (bufferLen < JSON_ITEM_NUM_SIZE) return -1;
        memcpy(buffer, JSON_ITEM_NUM, JSON_ITEM_NUM_SIZE);
        head = JSON_ITEM_NUM_SIZE;

        res = utils_floatToText(value, buffer + head, bufferLen - head);
        if (res <= 0) return -1;
        head += res;

        if (bufferLen - head < JSON_ITEM_NUM_END_SIZE) return -1;
        memcpy(buffer + head, JSON_ITEM_NUM_END, JSON_ITEM_NUM_END_SIZE);
        head += JSON_ITEM_NUM_END_SIZE;
    }
    break;

    case LWM2M_TYPE_BOOLEAN:
    {
        bool value;

        if (0 == lwm2m_data_decode_bool(tlvP, &value)) return -1;

        if (value == true)
        {
            if (bufferLen < JSON_ITEM_BOOL_TRUE_SIZE) return -1;
            memcpy(buffer, JSON_ITEM_BOOL_TRUE, JSON_ITEM_BOOL_TRUE_SIZE);
            head = JSON_ITEM_BOOL_TRUE_SIZE;
        }
        else
        {
            if (bufferLen < JSON_ITEM_BOOL_FALSE_SIZE) return -1;
            memcpy(buffer, JSON_ITEM_BOOL_FALSE, JSON_ITEM_BOOL_FALSE_SIZE);
            head = JSON_ITEM_BOOL_FALSE_SIZE;
        }
    }
    break;

    case LWM2M_TYPE_OPAQUE:
        if (bufferLen < JSON_ITEM_STRING_BEGIN_SIZE) return -1;
        memcpy(buffer, JSON_ITEM_STRING_BEGIN, JSON_ITEM_STRING_BEGIN_SIZE);
        head = JSON_ITEM_STRING_BEGIN_SIZE;

        res = utils_base64Encode(tlvP->value.asBuffer.buffer, tlvP->value.asBuffer.length, buffer + head, bufferLen - head);
        if (res == 0) return -1;
        head += res;

        if (bufferLen - head < JSON_ITEM_STRING_END_SIZE) return -1;
        memcpy(buffer + head, JSON_ITEM_STRING_END, JSON_ITEM_STRING_END_SIZE);
        head += JSON_ITEM_STRING_END_SIZE;
        break;

    case LWM2M_TYPE_OBJECT_LINK:
        // TODO: implement
        return -1;

    default:
        return -1;
    }

    return head;
}

int prv_serializeData(lwm2m_data_t *tlvP,
                      uint8_t *parentUriStr,
                      size_t parentUriLen,
                      uint8_t *buffer,
                      size_t bufferLen)
{
    int head;
    int res;

    head = 0;

    switch (tlvP->type)
    {
    case LWM2M_TYPE_OBJECT:
    case LWM2M_TYPE_OBJECT_INSTANCE:
    case LWM2M_TYPE_MULTIPLE_RESOURCE:
    {
        uint8_t uriStr[URI_MAX_STRING_LEN];
        size_t uriLen;
        size_t index;

        if (parentUriLen > 0)
        {
            if (URI_MAX_STRING_LEN < parentUriLen) return -1;
            memcpy(uriStr, parentUriStr, parentUriLen);
            uriLen = parentUriLen;
        }
        else
        {
            uriLen = 0;
        }
        res = utils_intToText(tlvP->id, uriStr + uriLen, URI_MAX_STRING_LEN - uriLen);
        if (res <= 0) return -1;
        uriLen += res;
        uriStr[uriLen] = '/';
        uriLen++;

        head = 0;
        for (index = 0 ; index < tlvP->value.asChildren.count; index++)
        {
            res = prv_serializeData(tlvP->value.asChildren.array + index, uriStr, uriLen, buffer + head, bufferLen - head);
            if (res < 0) return -1;
            head += res;
        }
    }
    break;

    default:
        if (bufferLen < JSON_RES_ITEM_URI_SIZE) return -1;
        memcpy(buffer, JSON_RES_ITEM_URI, JSON_RES_ITEM_URI_SIZE);
        head = JSON_RES_ITEM_URI_SIZE;

        if (parentUriLen > 0)
        {
            if (bufferLen - head < parentUriLen) return -1;
            memcpy(buffer + head, parentUriStr, parentUriLen);
            head += parentUriLen;
        }

        res = utils_intToText(tlvP->id, buffer + head, bufferLen - head);
        if (res <= 0) return -1;
        head += res;

        res = prv_serializeValue(tlvP, buffer + head, bufferLen - head);
        if (res < 0) return -1;
        head += res;
        break;
    }

    return head;
}

static int prv_findAndCheckData(lwm2m_uri_t *uriP,
                                uri_depth_t level,
                                size_t size,
                                lwm2m_data_t *tlvP,
                                lwm2m_data_t **targetP)
{
    size_t index;
    int result;

    if (size == 0) return 0;

    if (size > 1)
    {
        if (tlvP[0].type == LWM2M_TYPE_OBJECT || tlvP[0].type == LWM2M_TYPE_OBJECT_INSTANCE)
        {
            for (index = 0; index < size; index++)
            {
                if (tlvP[index].type != tlvP[0].type)
                {
                    *targetP = NULL;
                    return -1;
                }
            }
        }
        else
        {
            for (index = 0; index < size; index++)
            {
                if (tlvP[index].type == LWM2M_TYPE_OBJECT || tlvP[index].type == LWM2M_TYPE_OBJECT_INSTANCE)
                {
                    *targetP = NULL;
                    return -1;
                }
            }
        }
    }

    *targetP = NULL;
    result = -1;
    switch (level)
    {
    case URI_DEPTH_OBJECT:
        if (tlvP[0].type == LWM2M_TYPE_OBJECT)
        {
            *targetP = tlvP;
            result = (int)size;
        }
        break;

    case URI_DEPTH_OBJECT_INSTANCE:
        switch (tlvP[0].type)
        {
        case LWM2M_TYPE_OBJECT:
            for (index = 0; index < size; index++)
            {
                if (tlvP[index].id == uriP->objectId)
                {
                    return prv_findAndCheckData(uriP, level, tlvP[index].value.asChildren.count, tlvP[index].value.asChildren.array, targetP);
                }
            }
            break;
        case LWM2M_TYPE_OBJECT_INSTANCE:
            *targetP = tlvP;
            result = (int)size;
            break;
        default:
            break;
        }
        break;

    case URI_DEPTH_RESOURCE:
        switch (tlvP[0].type)
        {
        case LWM2M_TYPE_OBJECT:
            for (index = 0; index < size; index++)
            {
                if (tlvP[index].id == uriP->objectId)
                {
                    return prv_findAndCheckData(uriP, level, tlvP[index].value.asChildren.count, tlvP[index].value.asChildren.array, targetP);
                }
            }
            break;
        case LWM2M_TYPE_OBJECT_INSTANCE:
            for (index = 0; index < size; index++)
            {
                if (tlvP[index].id == uriP->instanceId)
                {
                    return prv_findAndCheckData(uriP, level, tlvP[index].value.asChildren.count, tlvP[index].value.asChildren.array, targetP);
                }
            }
            break;
        default:
            *targetP = tlvP;
            result = (int)size;
            break;
        }
        break;

    case URI_DEPTH_RESOURCE_INSTANCE:
        switch (tlvP[0].type)
        {
        case LWM2M_TYPE_OBJECT:
            for (index = 0; index < size; index++)
            {
                if (tlvP[index].id == uriP->objectId)
                {
                    return prv_findAndCheckData(uriP, level, tlvP[index].value.asChildren.count, tlvP[index].value.asChildren.array, targetP);
                }
            }
            break;
        case LWM2M_TYPE_OBJECT_INSTANCE:
            for (index = 0; index < size; index++)
            {
                if (tlvP[index].id == uriP->instanceId)
                {
                    return prv_findAndCheckData(uriP, level, tlvP[index].value.asChildren.count, tlvP[index].value.asChildren.array, targetP);
                }
            }
            break;
        case LWM2M_TYPE_MULTIPLE_RESOURCE:
            for (index = 0; index < size; index++)
            {
                if (tlvP[index].id == uriP->resourceId)
                {
                    return prv_findAndCheckData(uriP, level, tlvP[index].value.asChildren.count, tlvP[index].value.asChildren.array, targetP);
                }
            }
            break;
        default:
            *targetP = tlvP;
            result = (int)size;
            break;
        }
        break;

    default:
        break;
    }

    return result;
}

int json_serialize(lwm2m_uri_t *uriP,
                   int size,
                   lwm2m_data_t *tlvP,
                   uint8_t **bufferP)
{
    int index;
    size_t head;
    uint8_t bufferJSON[PRV_JSON_BUFFER_SIZE];
    uint8_t baseUriStr[URI_MAX_STRING_LEN];
    int baseUriLen;
    uri_depth_t rootLevel;
    int num;
    lwm2m_data_t *targetP;

    LOG_ARG("size: %d", size);
    LOG_URI(uriP);
    if (size != 0 && tlvP == NULL) return -1;

    baseUriLen = uri_toString(uriP, baseUriStr, URI_MAX_STRING_LEN, &rootLevel);
    if (baseUriLen < 0) return -1;

    num = prv_findAndCheckData(uriP, rootLevel, size, tlvP, &targetP);
    if (num < 0) return -1;

    while (num == 1
            && (targetP->type == LWM2M_TYPE_OBJECT
                || targetP->type == LWM2M_TYPE_OBJECT_INSTANCE
                || targetP->type == LWM2M_TYPE_MULTIPLE_RESOURCE))
    {
        int res;

        res = utils_intToText(targetP->id, baseUriStr + baseUriLen, URI_MAX_STRING_LEN - baseUriLen);
        if (res <= 0) return 0;
        baseUriLen += res;
        if (baseUriLen >= URI_MAX_STRING_LEN - 1) return 0;
        num = targetP->value.asChildren.count;
        targetP = targetP->value.asChildren.array;
        baseUriStr[baseUriLen] = '/';
        baseUriLen++;
    }

    if (baseUriLen > 0)
    {
        memcpy(bufferJSON, JSON_BN_HEADER_1, JSON_BN_HEADER_1_SIZE);
        head = JSON_BN_HEADER_1_SIZE;
        memcpy(bufferJSON + head, baseUriStr, baseUriLen);
        head += baseUriLen;
        memcpy(bufferJSON + head, JSON_BN_HEADER_2, JSON_BN_HEADER_2_SIZE);
        head += JSON_BN_HEADER_2_SIZE;
    }
    else
    {
        memcpy(bufferJSON, JSON_HEADER, JSON_HEADER_SIZE);
        head = JSON_HEADER_SIZE;
    }

    for (index = 0 ; index < num && head < PRV_JSON_BUFFER_SIZE ; index++)
    {
        int res;

        res = prv_serializeData(targetP + index, NULL, 0, bufferJSON + head, PRV_JSON_BUFFER_SIZE - head);
        if (res < 0) return 0;
        head += res;
    }

    if (head + JSON_FOOTER_SIZE - 1 > PRV_JSON_BUFFER_SIZE) return 0;

    if (num > 0) head = head - 1;

    memcpy(bufferJSON + head, JSON_FOOTER, JSON_FOOTER_SIZE);
    head = head + JSON_FOOTER_SIZE;

    *bufferP = (uint8_t *)lwm2m_malloc(head);
    if (*bufferP == NULL) return 0;
    memcpy(*bufferP, bufferJSON, head);

    return head;
}

#endif

