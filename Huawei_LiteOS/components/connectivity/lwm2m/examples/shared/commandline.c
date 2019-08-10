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
 * Copyright (c) 2013, 2014, 2015 Intel Corporation and others.
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
 *
 *******************************************************************************/

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <inttypes.h>
#include "liblwm2m.h"
#include "internals.h"
#include "commandline.h"

#define HELP_COMMAND "help"
#define HELP_DESC    "Type '"HELP_COMMAND" [COMMAND]' for more details on a command."
#define UNKNOWN_CMD_MSG "Unknown command. Type '"HELP_COMMAND"' for help."

static command_desc_t *prv_find_command(command_desc_t *commandArray,
                                        char *buffer,
                                        size_t length)
{
    int i;

    if (length == 0) return NULL;

    i = 0;
    while (commandArray[i].name != NULL
            && (strlen(commandArray[i].name) != length || strncmp(buffer, commandArray[i].name, length)))
    {
        i++;
    }

    if (commandArray[i].name == NULL)
    {
        return NULL;
    }
    else
    {
        return &commandArray[i];
    }
}

static void prv_displayHelp(command_desc_t *commandArray,
                            char *buffer)
{
    command_desc_t *cmdP;
    int length;

    // find end of first argument
    length = 0;
    while (buffer[length] != 0 && !isspace(buffer[length] & 0xff))
        length++;

    cmdP = prv_find_command(commandArray, buffer, length);

    if (cmdP == NULL)
    {
        int i;

        fprintf(stdout, HELP_COMMAND"\t"HELP_DESC"\r\n");

        for (i = 0 ; commandArray[i].name != NULL ; i++)
        {
            fprintf(stdout, "%s\t%s\r\n", commandArray[i].name, commandArray[i].shortDesc);
        }
    }
    else
    {
        fprintf(stdout, "%s\r\n", cmdP->longDesc ? cmdP->longDesc : cmdP->shortDesc);
    }
}


void handle_command(command_desc_t *commandArray,
                    char *buffer)
{
    command_desc_t *cmdP;
    int length;

    // find end of command name
    length = 0;
    while (buffer[length] != 0 && !isspace(buffer[length] & 0xFF))
        length++;

    cmdP = prv_find_command(commandArray, buffer, length);
    if (cmdP != NULL)
    {
        while (buffer[length] != 0 && isspace(buffer[length] & 0xFF))
            length++;
        cmdP->callback(buffer + length, cmdP->userData);
    }
    else
    {
        if (!strncmp(buffer, HELP_COMMAND, length))
        {
            while (buffer[length] != 0 && isspace(buffer[length] & 0xFF))
                length++;
            prv_displayHelp(commandArray, buffer + length);
        }
        else
        {
            fprintf(stdout, UNKNOWN_CMD_MSG"\r\n");
        }
    }
}

static char *prv_end_of_space(char *buffer)
{
    while (isspace(buffer[0] & 0xff))
    {
        buffer++;
    }
    return buffer;
}

char *get_end_of_arg(char *buffer)
{
    while (buffer[0] != 0 && !isspace(buffer[0] & 0xFF))
    {
        buffer++;
    }
    return buffer;
}

char *get_next_arg(char *buffer, char **end)
{
    // skip arg
    buffer = get_end_of_arg(buffer);
    // skip space
    buffer = prv_end_of_space(buffer);
    if (NULL != end)
    {
        *end = get_end_of_arg(buffer);
    }

    return buffer;
}

int check_end_of_args(char *buffer)
{
    buffer = prv_end_of_space(buffer);

    return (0 == buffer[0]);
}

/**********************************************************
 * Display Functions
 */

static void print_indent(FILE *stream,
                         int num)
{
    int i;

    for ( i = 0 ; i < num ; i++)
        fprintf(stream, "    ");
}
#ifdef ATING_DEBUG
void output_buffer(FILE *stream,
                   uint8_t *buffer,
                   int length,
                   int indent)
{
    int i;

    if (length == 0) fprintf(stream, "\n");

    if (buffer == NULL) return;

    i = 0;
    while (i < length)
    {
        uint8_t array[16];
        int j;

        print_indent(stream, indent);
        memcpy(array, buffer + i, 16);
        for (j = 0 ; j < 16 && i + j < length; j++)
        {
            fprintf(stream, "%02X ", array[j]);
            if (j % 4 == 3) fprintf(stream, " ");
        }
        if (length > 16)
        {
            while (j < 16)
            {
                fprintf(stream, "   ");
                if (j % 4 == 3) fprintf(stream, " ");
                j++;
            }
        }
        fprintf(stream, " ");
        for (j = 0 ; j < 16 && i + j < length; j++)
        {
            if (isprint(array[j]))
                fprintf(stream, "%c", array[j]);
            else
                fprintf(stream, ".");
        }
        fprintf(stream, "\n");
        i += 16;
    }
}
#else
void output_buffer(FILE *stream,
                   uint8_t *buffer,
                   int length,
                   int indent)
{
}
#endif
void output_tlv(FILE *stream,
                uint8_t *buffer,
                size_t buffer_len,
                int indent)
{
    lwm2m_data_type_t type;
    uint16_t id;
    size_t dataIndex;
    size_t dataLen;
    int length = 0;
    int result;

    while (0 != (result = lwm2m_decode_TLV((uint8_t *)buffer + length, buffer_len - length, &type, &id, &dataIndex, &dataLen)))
    {
        print_indent(stream, indent);
        fprintf(stream, "{\r\n");
        print_indent(stream, indent + 1);
        fprintf(stream, "ID: %d", id);

        fprintf(stream, " type: ");
        switch (type)
        {
        case LWM2M_TYPE_OBJECT_INSTANCE:
            fprintf(stream, "Object Instance");
            break;
        case LWM2M_TYPE_MULTIPLE_RESOURCE:
            fprintf(stream, "Multiple Instances");
            break;
        case LWM2M_TYPE_OPAQUE:
            fprintf(stream, "Resource Value");
            break;
        default:
            printf("unknown (%d)", (int)type);
            break;
        }
        fprintf(stream, "\n");

        print_indent(stream, indent + 1);
        fprintf(stream, "{\n");
        if (type == LWM2M_TYPE_OBJECT_INSTANCE || type == LWM2M_TYPE_MULTIPLE_RESOURCE)
        {
            output_tlv(stream, buffer + length + dataIndex, dataLen, indent + 1);
        }
        else
        {
            int64_t intValue;
            double floatValue;
            uint8_t tmp;

            print_indent(stream, indent + 2);
            fprintf(stream, "data (%d bytes):\r\n", dataLen);
            output_buffer(stream, (uint8_t *)buffer + length + dataIndex, dataLen, indent + 2);

            tmp = buffer[length + dataIndex + dataLen];
            buffer[length + dataIndex + dataLen] = 0;
            if (0 < sscanf((const char *)buffer + length + dataIndex, "%"PRId64, &intValue))
            {
                print_indent(stream, indent + 2);
                fprintf(stream, "data as Integer: %" PRId64 "\r\n", intValue);
            }
            if (0 < sscanf((const char *)buffer + length + dataIndex, "%lg", &floatValue))
            {
                print_indent(stream, indent + 2);
                fprintf(stream, "data as Float: %.16g\r\n", floatValue);
            }
            buffer[length + dataIndex + dataLen] = tmp;
        }
        print_indent(stream, indent + 1);
        fprintf(stream, "}\r\n");
        length += result;
        print_indent(stream, indent);
        fprintf(stream, "}\r\n");
    }
}

void output_data(FILE *stream,
                 lwm2m_media_type_t format,
                 uint8_t *data,
                 int dataLength,
                 int indent)
{
    int i;

    print_indent(stream, indent);
    fprintf(stream, "%d bytes received of type ", dataLength);

    switch (format)
    {
    case LWM2M_CONTENT_TEXT:
        fprintf(stream, "text/plain:\r\n");
        output_buffer(stream, data, dataLength, indent);
        break;

    case LWM2M_CONTENT_OPAQUE:
        fprintf(stream, "application/octet-stream:\r\n");
        output_buffer(stream, data, dataLength, indent);
        break;

    case LWM2M_CONTENT_TLV:
        fprintf(stream, "application/vnd.oma.lwm2m+tlv:\r\n");
        output_tlv(stream, data, dataLength, indent);
        break;

    case LWM2M_CONTENT_JSON:
        fprintf(stream, "application/vnd.oma.lwm2m+json:\r\n");
        print_indent(stream, indent);
        for (i = 0 ; i < dataLength ; i++)
        {
            fprintf(stream, "%c", data[i]);
        }
        fprintf(stream, "\n");
        break;

    case LWM2M_CONTENT_LINK:
        fprintf(stream, "application/link-format:\r\n");
        print_indent(stream, indent);
        for (i = 0 ; i < dataLength ; i++)
        {
            fprintf(stream, "%c", data[i]);
        }
        fprintf(stream, "\n");
        break;

    default:
        fprintf(stream, "Unknown (%d):\r\n", format);
        output_buffer(stream, data, dataLength, indent);
        break;
    }
}

void dump_tlv(FILE *stream,
              int size,
              lwm2m_data_t *dataP,
              int indent)
{
    int i;

    for(i = 0 ; i < size ; i++)
    {
        print_indent(stream, indent);
        fprintf(stream, "{\r\n");
        print_indent(stream, indent + 1);
        fprintf(stream, "id: %d\r\n", dataP[i].id);

        print_indent(stream, indent + 1);
        fprintf(stream, "type: ");
        switch (dataP[i].type)
        {
        case LWM2M_TYPE_OBJECT:
            fprintf(stream, "LWM2M_TYPE_OBJECT\r\n");
            dump_tlv(stream, dataP[i].value.asChildren.count, dataP[i].value.asChildren.array, indent + 1);
            break;
        case LWM2M_TYPE_OBJECT_INSTANCE:
            fprintf(stream, "LWM2M_TYPE_OBJECT_INSTANCE\r\n");
            dump_tlv(stream, dataP[i].value.asChildren.count, dataP[i].value.asChildren.array, indent + 1);
            break;
        case LWM2M_TYPE_MULTIPLE_RESOURCE:
            fprintf(stream, "LWM2M_TYPE_MULTIPLE_RESOURCE\r\n");
            dump_tlv(stream, dataP[i].value.asChildren.count, dataP[i].value.asChildren.array, indent + 1);
            break;
        case LWM2M_TYPE_UNDEFINED:
            fprintf(stream, "LWM2M_TYPE_UNDEFINED\r\n");
            break;
        case LWM2M_TYPE_STRING:
            fprintf(stream, "LWM2M_TYPE_STRING\r\n");
            print_indent(stream, indent + 1);
            fprintf(stream, "\"%.*s\"\r\n", (int)dataP[i].value.asBuffer.length, dataP[i].value.asBuffer.buffer);
            break;
        case LWM2M_TYPE_OPAQUE:
            fprintf(stream, "LWM2M_TYPE_OPAQUE\r\n");
            output_buffer(stream, dataP[i].value.asBuffer.buffer, dataP[i].value.asBuffer.length, indent + 1);
            break;
        case LWM2M_TYPE_INTEGER:
            fprintf(stream, "LWM2M_TYPE_INTEGER: ");
            print_indent(stream, indent + 1);
            fprintf(stream, "%" PRId64, dataP[i].value.asInteger);
            fprintf(stream, "\r\n");
            break;
        case LWM2M_TYPE_FLOAT:
            fprintf(stream, "LWM2M_TYPE_FLOAT: ");
            print_indent(stream, indent + 1);
            fprintf(stream, "%" PRId64, dataP[i].value.asInteger);
            fprintf(stream, "\r\n");
            break;
        case LWM2M_TYPE_BOOLEAN:
            fprintf(stream, "LWM2M_TYPE_BOOLEAN: ");
            fprintf(stream, "%s", dataP[i].value.asBoolean ? "true" : "false");
            fprintf(stream, "\r\n");
            break;
        case LWM2M_TYPE_OBJECT_LINK:
            fprintf(stream, "LWM2M_TYPE_OBJECT_LINK\r\n");
            break;
        default:
            fprintf(stream, "unknown (%d)\r\n", (int)dataP[i].type);
            break;
        }
        print_indent(stream, indent);
        fprintf(stream, "}\r\n");
    }
}

#define CODE_TO_STRING(X)   case X : return #X

static const char *prv_status_to_string(int status)
{
    switch(status)
    {
        CODE_TO_STRING(COAP_NO_ERROR);
        CODE_TO_STRING(COAP_IGNORE);
        CODE_TO_STRING(COAP_201_CREATED);
        CODE_TO_STRING(COAP_202_DELETED);
        CODE_TO_STRING(COAP_204_CHANGED);
        CODE_TO_STRING(COAP_205_CONTENT);
        CODE_TO_STRING(COAP_400_BAD_REQUEST);
        CODE_TO_STRING(COAP_401_UNAUTHORIZED);
        CODE_TO_STRING(COAP_404_NOT_FOUND);
        CODE_TO_STRING(COAP_405_METHOD_NOT_ALLOWED);
        CODE_TO_STRING(COAP_406_NOT_ACCEPTABLE);
        CODE_TO_STRING(COAP_500_INTERNAL_SERVER_ERROR);
        CODE_TO_STRING(COAP_501_NOT_IMPLEMENTED);
        CODE_TO_STRING(COAP_503_SERVICE_UNAVAILABLE);
    default:
        return "";
    }
}

void print_status(FILE *stream,
                  uint8_t status)
{
    fprintf(stream, "%d.%02d (%s)", (status & 0xE0) >> 5, status & 0x1F, prv_status_to_string(status));
}

/**********************************************************
* Base64 decoding function
*
* WARNING: Bugged for input strings with length < 4
*
*/

#define PRV_B64_PADDING '='

static uint8_t prv_b64Revert(uint8_t value)
{
    if (value >= 'A' && value <= 'Z')
    {
        return (value - 'A');
    }
    if (value >= 'a' && value <= 'z')
    {
        return (26 + value - 'a');
    }
    if (value >= '0' && value <= '9')
    {
        return (52 + value - '0');
    }
    switch (value)
    {
    case '+':
        return 62;
    case '/':
        return 63;
    default:
        return 0;
    }
}

static void prv_decodeBlock(uint8_t input[4],
                            uint8_t output[3])
{
    uint8_t tmp[4];
    int i;

    memset(output, 0, 3);

    for (i = 0; i < 4; i++)
    {
        tmp[i] = prv_b64Revert(input[i]);
    }

    output[0] = (tmp[0] << 2) | (tmp[1] >> 4);
    output[1] = (tmp[1] << 4) | (tmp[2] >> 2);
    output[2] = (tmp[2] << 6) | tmp[3];
}

size_t base64_decode(uint8_t *dataP,
                     size_t dataLen,
                     uint8_t **bufferP)
{
    size_t data_index;
    size_t result_index;
    size_t result_len;

    if (dataLen % 4) return 0;

    result_len = (dataLen >> 2) * 3;
    *bufferP = (uint8_t *)lwm2m_malloc(result_len);
    if (NULL == *bufferP) return 0;
    memset(*bufferP, 0, result_len);

    // remove padding
    while (dataP[dataLen - 1] == PRV_B64_PADDING)
    {
        dataLen--;
    }

    data_index = 0;
    result_index = 0;
    while (data_index < dataLen)
    {
        prv_decodeBlock(dataP + data_index, *bufferP + result_index);
        data_index += 4;
        result_index += 3;
    }
    switch (data_index - dataLen)
    {
    case 0:
        break;
    case 2:
    {
        uint8_t tmp[2];

        tmp[0] = prv_b64Revert(dataP[dataLen - 2]);
        tmp[1] = prv_b64Revert(dataP[dataLen - 1]);

        *bufferP[result_index - 3] = (tmp[0] << 2) | (tmp[1] >> 4);
        *bufferP[result_index - 2] = (tmp[1] << 4);
        result_len -= 2;
    }
    break;
    case 3:
    {
        uint8_t tmp[3];

        tmp[0] = prv_b64Revert(dataP[dataLen - 3]);
        tmp[1] = prv_b64Revert(dataP[dataLen - 2]);
        tmp[2] = prv_b64Revert(dataP[dataLen - 1]);

        *bufferP[result_index - 3] = (tmp[0] << 2) | (tmp[1] >> 4);
        *bufferP[result_index - 2] = (tmp[1] << 4) | (tmp[2] >> 2);
        *bufferP[result_index - 1] = (tmp[2] << 6);
        result_len -= 1;
    }
    break;
    default:
        // error
        lwm2m_free(*bufferP);
        *bufferP = NULL;
        result_len = 0;
        break;
    }

    return result_len;
}
