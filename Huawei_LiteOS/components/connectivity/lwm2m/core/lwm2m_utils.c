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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <float.h>


int utils_textToInt(uint8_t *buffer,
                    int length,
                    int64_t *dataP)
{
    uint64_t result = 0;
    int sign = 1;
    int i = 0;

    if (0 == length) return 0;

    if (buffer[0] == '-')
    {
        sign = -1;
        i = 1;
    }

    while (i < length)
    {
        if ('0' <= buffer[i] && buffer[i] <= '9')
        {
            if (result > (UINT64_MAX / 10)) return 0;
            result *= 10;
            result += buffer[i] - '0';
        }
        else
        {
            return 0;
        }
        i++;
    }

    if (result > INT64_MAX) return 0;

    if (sign == -1)
    {
        *dataP = 0 - result;
    }
    else
    {
        *dataP = result;
    }

    return 1;
}

int utils_textToFloat(uint8_t *buffer,
                      int length,
                      double *dataP)
{
    double result;
    int sign;
    int i;

    if (0 == length) return 0;

    if (buffer[0] == '-')
    {
        sign = -1;
        i = 1;
    }
    else
    {
        sign = 1;
        i = 0;
    }

    result = 0;
    while (i < length && buffer[i] != '.')
    {
        if ('0' <= buffer[i] && buffer[i] <= '9')
        {
            if (result > (DBL_MAX / 10)) return 0;
            result *= 10;
            result += (buffer[i] - '0');
        }
        else
        {
            return 0;
        }
        i++;
    }
    if (buffer[i] == '.')
    {
        double dec;

        i++;
        if (i == length) return 0;

        dec = 0.1;
        while (i < length)
        {
            if ('0' <= buffer[i] && buffer[i] <= '9')
            {
                if (result > (DBL_MAX - 1)) return 0;
                result += (buffer[i] - '0') * dec;
                dec /= 10;
            }
            else
            {
                return 0;
            }
            i++;
        }
    }

    *dataP = result * sign;
    return 1;
}

size_t utils_intToText(int64_t data,
                       uint8_t *string,
                       size_t length)
{
    int index;
    bool minus;
    size_t result;

    if (data < 0)
    {
        minus = true;
        data = 0 - data;
    }
    else
    {
        minus = false;
    }

    index = length - 1;
    do
    {
        string[index] = '0' + data % 10;
        data /= 10;
        index --;
    }
    while (index >= 0 && data > 0);

    if (data > 0) return 0;

    if (minus == true)
    {
        if (index == 0) return 0;
        string[index] = '-';
    }
    else
    {
        index++;
    }

    result = length - index;

    if (result < length)
    {
        memmove(string, string + index, result);
    }

    return result;
}

size_t utils_floatToText(double data,
                         uint8_t *string,
                         size_t length)
{
    size_t intLength;
    size_t decLength;
    int64_t intPart;
    double decPart;

    if (data <= (double)INT64_MIN || data >= (double)INT64_MAX) return 0;

    intPart = (int64_t)data;
    decPart = data - intPart;
    if (decPart < 0)
    {
        decPart = 1 - decPart;
    }
    else
    {
        decPart = 1 + decPart;
    }

    if (decPart <= 1 + FLT_EPSILON)
    {
        decPart = 0;
    }

    if (intPart == 0 && data < 0)
    {
        // deal with numbers between -1 and 0
        if (length < 4) return 0;   // "-0.n"
        string[0] = '-';
        string[1] = '0';
        intLength = 2;
    }
    else
    {
        intLength = utils_intToText(intPart, string, length);
        if (intLength == 0) return 0;
    }
    decLength = 0;
    if (decPart >= FLT_EPSILON)
    {
        double noiseFloor;

        if (intLength >= length - 1) return 0;

        noiseFloor = FLT_EPSILON;
        do
        {
            decPart *= 10;
            noiseFloor *= 10;
        }
        while (decPart - (int64_t)decPart > noiseFloor);

        decLength = utils_intToText(decPart, string + intLength, length - intLength);
        if (decLength <= 1) return 0;

        // replace the leading 1 with a dot
        string[intLength] = '.';
    }

    return intLength + decLength;
}

lwm2m_binding_t utils_stringToBinding(uint8_t *buffer,
                                      size_t length)
{
    if (length == 0) return BINDING_UNKNOWN;

    switch (buffer[0])
    {
    case 'U':
        switch (length)
        {
        case 1:
            return BINDING_U;
        case 2:
            switch (buffer[1])
            {
            case 'Q':
                return BINDING_UQ;
            case 'S':
                return BINDING_US;
            default:
                break;
            }
            break;
        case 3:
            if (buffer[1] == 'Q' && buffer[2] == 'S')
            {
                return BINDING_UQS;
            }
            break;
        default:
            break;
        }
        break;

    case 'S':
        switch (length)
        {
        case 1:
            return BINDING_S;
        case 2:
            if (buffer[1] == 'Q')
            {
                return BINDING_SQ;
            }
            break;
        default:
            break;
        }
        break;

    default:
        break;
    }

    return BINDING_UNKNOWN;
}

lwm2m_media_type_t utils_convertMediaType(coap_content_type_t type)
{
    // Here we just check the content type is a valid value for LWM2M
    switch((uint16_t)type)
    {
    case TEXT_PLAIN:
        return LWM2M_CONTENT_TEXT;
    case APPLICATION_OCTET_STREAM:
        return LWM2M_CONTENT_OPAQUE;
    case LWM2M_CONTENT_TLV_OLD:
        return LWM2M_CONTENT_TLV_OLD;
    case LWM2M_CONTENT_TLV:
        return LWM2M_CONTENT_TLV;
    case LWM2M_CONTENT_JSON_OLD:
        return LWM2M_CONTENT_JSON_OLD;
    case LWM2M_CONTENT_JSON:
        return LWM2M_CONTENT_JSON;
    case APPLICATION_LINK_FORMAT:
        return LWM2M_CONTENT_LINK;

    default:
        return LWM2M_CONTENT_TEXT;
    }
}

#ifdef LWM2M_CLIENT_MODE
lwm2m_server_t *utils_findServer(lwm2m_context_t *contextP,
                                 void *fromSessionH)
{
    lwm2m_server_t *targetP;

    targetP = contextP->serverList;
    while (targetP != NULL
            && false == lwm2m_session_is_equal(targetP->sessionH, fromSessionH, contextP->userData))
    {
        targetP = targetP->next;
    }

    return targetP;
}
#endif

#ifdef LWM2M_BOOTSTRAP
lwm2m_server_t *utils_findBootstrapServer(lwm2m_context_t *contextP,
        void *fromSessionH)
{
#ifdef LWM2M_CLIENT_MODE

    lwm2m_server_t *targetP;

    targetP = contextP->bootstrapServerList;
    while (targetP != NULL
            && false == lwm2m_session_is_equal(targetP->sessionH, fromSessionH, contextP->userData))
    {
        targetP = targetP->next;
    }

    return targetP;

#else

    return NULL;

#endif
}
#endif

int utils_isAltPathValid(const char *altPath)
{
    int i;

    if (altPath == NULL) return 0;

    if (altPath[0] != '/') return 0;

    for (i = 1 ; altPath[i] != 0 ; i++)
    {
        // TODO: Support multi-segment alternative path
        if (altPath[i] == '/') return 0;
        // TODO: Check needs for sub-delims, ':' and '@'
        if ((altPath[i] < 'A' || altPath[i] > 'Z')      // ALPHA
                && (altPath[i] < 'a' || altPath[i] > 'z')
                && (altPath[i] < '0' || altPath[i] > '9')      // DIGIT
                && (altPath[i] != '-')                         // Other unreserved
                && (altPath[i] != '.')
                && (altPath[i] != '_')
                && (altPath[i] != '~')
                && (altPath[i] != '%'))                        // pct_encoded
        {
            return 0;
        }

    }
    return 1;
}

// copy a string in a buffer.
// return the number of copied bytes or -1 if the buffer is not large enough
int utils_stringCopy(char *buffer,
                     size_t length,
                     const char *str)
{
    size_t i;

    for (i = 0 ; i < length && str[i] != 0 ; i++)
    {
        buffer[i] = str[i];
    }

    if (i == length) return -1;

    buffer[i] = 0;

    return (int)i;
}

void utils_copyValue(void *dst,
                     const void *src,
                     size_t len)
{
#ifdef LWM2M_BIG_ENDIAN
    memcpy(dst, src, len);
#else
#ifdef LWM2M_LITTLE_ENDIAN
    size_t i;

    for (i = 0; i < len; i++)
    {
        ((uint8_t *)dst)[i] = ((uint8_t *)src)[len - 1 - i];
    }
#endif
#endif
}


#define PRV_B64_PADDING '='

static char b64Alphabet[64] =
{
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
    'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
    'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
    'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '+', '/'
};

static void prv_encodeBlock(uint8_t input[3],
                            uint8_t output[4])
{
    output[0] = b64Alphabet[input[0] >> 2];
    output[1] = b64Alphabet[((input[0] & 0x03) << 4) | (input[1] >> 4)];
    output[2] = b64Alphabet[((input[1] & 0x0F) << 2) | (input[2] >> 6)];
    output[3] = b64Alphabet[input[2] & 0x3F];
}

size_t utils_base64GetSize(size_t dataLen)
{
    size_t result_len;

    result_len = 4 * (dataLen / 3);
    if (dataLen % 3) result_len += 4;

    return result_len;
}

size_t utils_base64Encode(uint8_t *dataP,
                          size_t dataLen,
                          uint8_t *bufferP,
                          size_t bufferLen)
{
    unsigned int data_index;
    unsigned int result_index;
    size_t result_len;

    result_len = utils_base64GetSize(dataLen);

    if (result_len > bufferLen) return 0;

    data_index = 0;
    result_index = 0;
    while (data_index < dataLen)
    {
        switch (dataLen - data_index)
        {
        case 0:
            // should never happen
            break;
        case 1:
            bufferP[result_index] = b64Alphabet[dataP[data_index] >> 2];
            bufferP[result_index + 1] = b64Alphabet[(dataP[data_index] & 0x03) << 4];
            bufferP[result_index + 2] = PRV_B64_PADDING;
            bufferP[result_index + 3] = PRV_B64_PADDING;
            break;
        case 2:
            bufferP[result_index] = b64Alphabet[dataP[data_index] >> 2];
            bufferP[result_index + 1] = b64Alphabet[(dataP[data_index] & 0x03) << 4 | (dataP[data_index + 1] >> 4)];
            bufferP[result_index + 2] = b64Alphabet[(dataP[data_index + 1] & 0x0F) << 2];
            bufferP[result_index + 3] = PRV_B64_PADDING;
            break;
        default:
            prv_encodeBlock(dataP + data_index, bufferP + result_index);
            break;
        }
        data_index += 3;
        result_index += 4;
    }

    return result_len;
}

lwm2m_data_type_t utils_depthToDatatype(uri_depth_t depth)
{
    switch (depth)
    {
    case URI_DEPTH_OBJECT:
        return LWM2M_TYPE_OBJECT;
    case URI_DEPTH_OBJECT_INSTANCE:
        return LWM2M_TYPE_OBJECT_INSTANCE;
    default:
        break;
    }

    return LWM2M_TYPE_UNDEFINED;
}
