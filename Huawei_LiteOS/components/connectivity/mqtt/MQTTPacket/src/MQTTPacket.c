/*******************************************************************************
 * Copyright (c) 2014 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Ian Craggs - initial API and implementation and/or initial documentation
 *    Sergio R. Caprile - non-blocking packet read functions for stream transport
 *******************************************************************************/

#include "StackTrace.h"
#include "MQTTPacket.h"

#include <string.h>

/**
 * Encodes the message length according to the MQTT algorithm
 * @param buf the buffer into which the encoded data is written
 * @param length the length to be encoded
 * @return the number of bytes written to buffer
 */
int MQTTPacket_encode(unsigned char *buf, int length)
{
    int rc = 0;

    FUNC_ENTRY;
    do
    {
        char d = length % 128;
        length /= 128;
        /* if there are more digits to encode, set the top bit of this digit */
        if (length > 0)
            d |= 0x80;
        buf[rc++] = d;
    }
    while (length > 0);
    FUNC_EXIT_RC(rc);
    return rc;
}


/**
 * Decodes the message length according to the MQTT algorithm
 * @param getcharfn pointer to function to read the next character from the data source
 * @param value the decoded length returned
 * @return the number of bytes read from the socket
 */
int MQTTPacket_decode(int (*getcharfn)(unsigned char *, int), int *value)
{
    unsigned char c;
    int multiplier = 1;
    int len = 0;
#define MAX_NO_OF_REMAINING_LENGTH_BYTES 4

    FUNC_ENTRY;
    *value = 0;
    do
    {
        int rc = MQTTPACKET_READ_ERROR;

        if (++len > MAX_NO_OF_REMAINING_LENGTH_BYTES)
        {
            rc = MQTTPACKET_READ_ERROR;	/* bad data */
            goto exit;
        }
        rc = (*getcharfn)(&c, 1);
        if (rc != 1)
            goto exit;
        *value += (c & 127) * multiplier;
        multiplier *= 128;
    }
    while ((c & 128) != 0);
exit:
    FUNC_EXIT_RC(len);
    return len;
}


int MQTTPacket_len(int rem_len)
{
    rem_len += 1; /* header byte */

    /* now remaining_length field */
    if (rem_len < 128)
        rem_len += 1;
    else if (rem_len < 16384)
        rem_len += 2;
    else if (rem_len < 2097151)
        rem_len += 3;
    else
        rem_len += 4;
    return rem_len;
}


static unsigned char *bufptr;

int bufchar(unsigned char *c, int count)
{
    int i;

    for (i = 0; i < count; ++i)
        *c = *bufptr++;
    return count;
}


int MQTTPacket_decodeBuf(unsigned char *buf, int *value)
{
    bufptr = buf;
    return MQTTPacket_decode(bufchar, value);
}


/**
 * Calculates an integer from two bytes read from the input buffer
 * @param pptr pointer to the input buffer - incremented by the number of bytes used & returned
 * @return the integer value calculated
 */
int readInt(unsigned char **pptr)
{
    unsigned char *ptr = *pptr;
    int len = 256 * (*ptr) + (*(ptr + 1));
    *pptr += 2;
    return len;
}


/**
 * Reads one character from the input buffer.
 * @param pptr pointer to the input buffer - incremented by the number of bytes used & returned
 * @return the character read
 */
char readChar(unsigned char **pptr)
{
    char c = **pptr;
    (*pptr)++;
    return c;
}


/**
 * Writes one character to an output buffer.
 * @param pptr pointer to the output buffer - incremented by the number of bytes used & returned
 * @param c the character to write
 */
void writeChar(unsigned char **pptr, char c)
{
    **pptr = c;
    (*pptr)++;
}


/**
 * Writes an integer as 2 bytes to an output buffer.
 * @param pptr pointer to the output buffer - incremented by the number of bytes used & returned
 * @param anInt the integer to write
 */
void writeInt(unsigned char **pptr, int anInt)
{
    **pptr = (unsigned char)(anInt / 256);
    (*pptr)++;
    **pptr = (unsigned char)(anInt % 256);
    (*pptr)++;
}


/**
 * Writes a "UTF" string to an output buffer.  Converts C string to length-delimited.
 * @param pptr pointer to the output buffer - incremented by the number of bytes used & returned
 * @param string the C string to write
 */
void writeCString(unsigned char **pptr, const char *string)
{
    int len = strlen(string);
    writeInt(pptr, len);
    memcpy(*pptr, string, len);
    *pptr += len;
}


int getLenStringLen(char *ptr)
{
    int len = 256 * ((unsigned char)(*ptr)) + (unsigned char)(*(ptr + 1));
    return len;
}


void writeMQTTString(unsigned char **pptr, MQTTString mqttstring)
{
    if (mqttstring.lenstring.len > 0)
    {
        writeInt(pptr, mqttstring.lenstring.len);
        memcpy(*pptr, mqttstring.lenstring.data, mqttstring.lenstring.len);
        *pptr += mqttstring.lenstring.len;
    }
    else if (mqttstring.cstring)
        writeCString(pptr, mqttstring.cstring);
    else
        writeInt(pptr, 0);
}


/**
 * @param mqttstring the MQTTString structure into which the data is to be read
 * @param pptr pointer to the output buffer - incremented by the number of bytes used & returned
 * @param enddata pointer to the end of the data: do not read beyond
 * @return 1 if successful, 0 if not
 */
int readMQTTLenString(MQTTString *mqttstring, unsigned char **pptr, unsigned char *enddata)
{
    int rc = 0;

    FUNC_ENTRY;
    /* the first two bytes are the length of the string */
    if (enddata - (*pptr) > 1) /* enough length to read the integer? */
    {
        mqttstring->lenstring.len = readInt(pptr); /* increments pptr to point past length */
        if (&(*pptr)[mqttstring->lenstring.len] <= enddata)
        {
            mqttstring->lenstring.data = (char *)*pptr;
            *pptr += mqttstring->lenstring.len;
            rc = 1;
        }
    }
    mqttstring->cstring = NULL;
    FUNC_EXIT_RC(rc);
    return rc;
}


/**
 * Return the length of the MQTTstring - C string if there is one, otherwise the length delimited string
 * @param mqttstring the string to return the length of
 * @return the length of the string
 */
int MQTTstrlen(MQTTString mqttstring)
{
    int rc = 0;

    if (mqttstring.cstring)
        rc = strlen(mqttstring.cstring);
    else
        rc = mqttstring.lenstring.len;
    return rc;
}


/**
 * Compares an MQTTString to a C string
 * @param a the MQTTString to compare
 * @param bptr the C string to compare
 * @return boolean - equal or not
 */
int MQTTPacket_equals(MQTTString *a, char *bptr)
{
    int alen = 0,
        blen = 0;
    char *aptr;

    if (a->cstring)
    {
        aptr = a->cstring;
        alen = strlen(a->cstring);
    }
    else
    {
        aptr = a->lenstring.data;
        alen = a->lenstring.len;
    }
    blen = strlen(bptr);

    return (alen == blen) && (strncmp(aptr, bptr, alen) == 0);
}


/**
 * Helper function to read packet data from some source into a buffer
 * @param buf the buffer into which the packet will be serialized
 * @param buflen the length in bytes of the supplied buffer
 * @param getfn pointer to a function which will read any number of bytes from the needed source
 * @return integer MQTT packet type, or -1 on error
 * @note  the whole message must fit into the caller's buffer
 */
int MQTTPacket_read(unsigned char *buf, int buflen, int (*getfn)(unsigned char *, int))
{
    int rc = -1;
    MQTTHeader header = {0};
    int len = 0;
    int rem_len = 0;

    /* 1. read the header byte.  This has the packet type in it */
    if ((*getfn)(buf, 1) != 1)
        goto exit;

    len = 1;
    /* 2. read the remaining length.  This is variable in itself */
    MQTTPacket_decode(getfn, &rem_len);
    len += MQTTPacket_encode(buf + 1, rem_len); /* put the original remaining length back into the buffer */

    /* 3. read the rest of the buffer using a callback to supply the rest of the data */
    if((rem_len + len) > buflen)
        goto exit;
    if (rem_len && ((*getfn)(buf + len, rem_len) != rem_len))
        goto exit;

    header.byte = buf[0];
    rc = header.bits.type;
exit:
    return rc;
}

/**
 * Decodes the message length according to the MQTT algorithm, non-blocking
 * @param trp pointer to a transport structure holding what is needed to solve getting data from it
 * @param value the decoded length returned
 * @return integer the number of bytes read from the socket, 0 for call again, or -1 on error
 */
static int MQTTPacket_decodenb(MQTTTransport *trp)
{
    unsigned char c;
    int rc = MQTTPACKET_READ_ERROR;

    FUNC_ENTRY;
    if(trp->len == 0) 		/* initialize on first call */
    {
        trp->multiplier = 1;
        trp->rem_len = 0;
    }
    do
    {
        int frc;
        if (trp->len >= MAX_NO_OF_REMAINING_LENGTH_BYTES)
            goto exit;
        if ((frc = (*trp->getfn)(trp->sck, &c, 1)) == -1)
            goto exit;
        if (frc == 0)
        {
            rc = 0;
            goto exit;
        }
        ++(trp->len);
        trp->rem_len += (c & 127) * trp->multiplier;
        trp->multiplier *= 128;
    }
    while ((c & 128) != 0);
    rc = trp->len;
exit:
    FUNC_EXIT_RC(rc);
    return rc;
}

/**
 * Helper function to read packet data from some source into a buffer, non-blocking
 * @param buf the buffer into which the packet will be serialized
 * @param buflen the length in bytes of the supplied buffer
 * @param trp pointer to a transport structure holding what is needed to solve getting data from it
 * @return integer MQTT packet type, 0 for call again, or -1 on error
 * @note  the whole message must fit into the caller's buffer
 */
int MQTTPacket_readnb(unsigned char *buf, int buflen, MQTTTransport *trp)
{
    int rc = -1, frc;
    MQTTHeader header = {0};

    switch(trp->state)
    {
    default:
        trp->state = 0;
    /*FALLTHROUGH*/
    case 0:
        /* read the header byte.  This has the packet type in it */
        if ((frc = (*trp->getfn)(trp->sck, buf, 1)) == -1)
            goto exit;
        if (frc == 0)
            return 0;
        trp->len = 0;
        ++trp->state;
    /*FALLTHROUGH*/
    /* read the remaining length.  This is variable in itself */
    case 1:
        if((frc = MQTTPacket_decodenb(trp)) == MQTTPACKET_READ_ERROR)
            goto exit;
        if(frc == 0)
            return 0;
        trp->len = 1 + MQTTPacket_encode(buf + 1, trp->rem_len); /* put the original remaining length back into the buffer */
        if((trp->rem_len + trp->len) > buflen)
            goto exit;
        ++trp->state;
    /*FALLTHROUGH*/
    case 2:
        if(trp->rem_len)
        {
            /* read the rest of the buffer using a callback to supply the rest of the data */
            if ((frc = (*trp->getfn)(trp->sck, buf + trp->len, trp->rem_len)) == -1)
                goto exit;
            if (frc == 0)
                return 0;
            trp->rem_len -= frc;
            trp->len += frc;
            if(trp->rem_len)
                return 0;
        }
        header.byte = buf[0];
        rc = header.bits.type;
        break;
    }

exit:
    trp->state = 0;
    return rc;
}

