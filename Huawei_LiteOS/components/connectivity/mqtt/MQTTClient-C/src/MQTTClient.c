/*******************************************************************************
 * Copyright (c) 2014, 2017 IBM Corp.
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
 *   Allan Stockdill-Mander/Ian Craggs - initial API and implementation and/or initial documentation
 *   Ian Craggs - fix for #96 - check rem_len in readPacket
 *   Ian Craggs - add ability to set message handler separately #6
 *******************************************************************************/
#include "MQTTClient.h"

static void NewMessageData(MessageData *md, MQTTString *aTopicName, MQTTMessage *aMessage, const char *topic_sub)
{
    md->topicName = aTopicName;
    md->message = aMessage;
}


static int getNextPacketId(MQTTClient *c)
{
    return c->next_packetid = (c->next_packetid == MAX_PACKET_ID) ? 1 : c->next_packetid + 1;
}


static int sendPacket(MQTTClient* c, int length, Timer* timer)
{
    int rc = FAILURE,
        sent = 0;

    while (sent < length && !TimerIsExpired(timer))
    {
        rc = c->ipstack->mqttwrite(c->ipstack, &c->buf[sent], length, TimerLeftMS(timer));
        if (rc < 0)  // there was an error writing the data
            break;
        sent += rc;
    }
    if (sent == length)
    {
        TimerCountdown(&c->last_sent, c->keepAliveInterval); // record the fact that we have successfully sent the packet
        rc = MQTT_SUCCESS;
    }
    else
        rc = FAILURE;
    return rc;
}


int MQTTClientInit(MQTTClient *c, Network *network, unsigned int command_timeout_ms,
                    unsigned char *sendbuf, size_t sendbuf_size, unsigned char *readbuf, size_t readbuf_size)
{
    int i;
    c->ipstack = network;

    for (i = 0; i < MAX_MESSAGE_HANDLERS; ++i)
        c->messageHandlers[i].topicFilter = 0;
    c->command_timeout_ms = command_timeout_ms;
    c->buf = sendbuf;
    c->buf_size = sendbuf_size;
    c->readbuf = readbuf;
    c->readbuf_size = readbuf_size;
    c->isconnected = 0;
    c->cleansession = 0;
    c->ping_outstanding = 0;
    c->defaultMessageHandler = NULL;
	  c->next_packetid = 1;
    TimerInit(&c->last_sent);
    TimerInit(&c->last_received);
#if defined(MQTT_TASK)
    if (MutexInit(&c->mutex) != 0)
    {
        return FAILURE;
    }
#endif
    return MQTT_SUCCESS;
}


void MQTTClientDeInit(MQTTClient *c)
{
    if(!c)
        return;
#if defined(MQTT_TASK)
    MutexDestory(&c->mutex);
#endif
    return;
}


static int decodePacket(MQTTClient* c, int* value, int timeout)
{
    unsigned char i;
    int multiplier = 1;
    int len = 0;
    const int MAX_NO_OF_REMAINING_LENGTH_BYTES = 4;

    *value = 0;
    do
    {
        int rc = MQTTPACKET_READ_ERROR;

        if (++len > MAX_NO_OF_REMAINING_LENGTH_BYTES)
        {
            rc = MQTTPACKET_READ_ERROR; /* bad data */
            goto exit;
        }
        rc = c->ipstack->mqttread(c->ipstack, &i, 1, timeout);
        if (rc != 1)
            goto exit;
        *value += (i & 127) * multiplier;
        multiplier *= 128;
    } while ((i & 128) != 0);
exit:
    return len;
}


static int readPacket(MQTTClient* c, Timer* timer)
{
    MQTTHeader header = {0};
    int len = 0;
    int rem_len = 0;

    /* 1. read the header byte.  This has the packet type in it */
    int rc = c->ipstack->mqttread(c->ipstack, c->readbuf, 1, TimerLeftMS(timer));
    if (rc != 1)
        goto exit;

    len = 1;
    /* 2. read the remaining length.  This is variable in itself */
    decodePacket(c, &rem_len, TimerLeftMS(timer));
    len += MQTTPacket_encode(c->readbuf + 1, rem_len); /* put the original remaining length back into the buffer */

    if (rem_len > (c->readbuf_size - len))
    {
        rc = BUFFER_OVERFLOW;
        goto exit;
    }

    /* 3. read the rest of the buffer using a callback to supply the rest of the data */
    if (rem_len > 0 && (rc = c->ipstack->mqttread(c->ipstack, c->readbuf + len, rem_len, TimerLeftMS(timer)) != rem_len)) {
        rc = 0;
        goto exit;
    }

    header.byte = c->readbuf[0];
    rc = header.bits.type;
    if (c->keepAliveInterval > 0)
        TimerCountdown(&c->last_received, c->keepAliveInterval); // record the fact that we have successfully received a packet
exit:
    return rc;
}

/**
 * Copyright (c) 2009-2018 Roger Light <roger@atchoo.org>
 * licensed under the Eclipse Public License 1.0 and the Eclipse Distribution License 1.0
 */
static int MQTTTopicMatched(const char *sub, MQTTString *topic_name, char *result)
{
    int sublen, topiclen;
    int spos, tpos;
    char multilevel_wildcard = 0;
    char *topic;

    if(!result)
        return -1;
    *result = 0;

    if(!sub || !topic_name)
    {
        return -1;
    }

    sublen = strlen(sub);

    if(topic_name->cstring)
    {
        topic = topic_name->cstring;
        topiclen = strlen(topic_name->cstring);
    }
    else
    {
        topic = topic_name->lenstring.data;
        topiclen = topic_name->lenstring.len;
    }

    if(!sublen || !topiclen)
    {
        *result = 0;
        return -1;
    }

    if(sublen && topiclen)
    {
        if((sub[0] == '$' && topic[0] != '$')
                || (topic[0] == '$' && sub[0] != '$'))
        {
            return 0;
        }
    }

    spos = 0;
    tpos = 0;

    while(spos < sublen && tpos <= topiclen)
    {
        if(sub[spos] == topic[tpos])
        {
            if(tpos == topiclen - 1)
            {
                /* Check for e.g. foo matching foo/# */
                if(spos == sublen - 3 && sub[spos + 1] == '/' && sub[spos + 2] == '#')
                {
                    *result = 1;
                    multilevel_wildcard = 1;
                    return 0;
                }
            }
            spos++;
            tpos++;
            if(spos == sublen && tpos == topiclen)
            {
                *result = 1;
                return 0;
            }
            else if(tpos == topiclen && spos == sublen - 1 && sub[spos] == '+')
            {
                if(spos > 0 && sub[spos - 1] != '/')
                {
                    return -1;
                }
                spos++;
                *result = 1;
                return 0;
            }
        }
        else
        {
            if(sub[spos] == '+')
            {
                /* Check for bad "+foo" or "a/+foo" subscription */
                if(spos > 0 && sub[spos - 1] != '/')
                {
                    return -1;
                }
                /* Check for bad "foo+" or "foo+/a" subscription */
                if(spos < sublen - 1 && sub[spos + 1] != '/')
                {
                    return -1;
                }
                spos++;
                while(tpos < topiclen && topic[tpos] != '/')
                {
                    tpos++;
                }
                if(tpos == topiclen && spos == sublen)
                {
                    *result = 1;
                    return 0;
                }
            }
            else if(sub[spos] == '#')
            {
                if(spos > 0 && sub[spos - 1] != '/')
                {
                    return -1;
                }
                multilevel_wildcard = 1;
                if(spos + 1 != sublen)
                {
                    return -1;
                }
                else
                {
                    *result = 1;
                    return 0;
                }
            }
            else
            {
                /* Check for e.g. foo/bar matching foo/+/# */
                if(spos > 0 && spos + 2 == sublen && tpos == topiclen
                        && sub[spos - 1] == '+' && sub[spos] == '/' && sub[spos + 1] == '#')
                {
                    *result = 1;
                    multilevel_wildcard = 1;
                    return 0;
                }
                return 0;
            }
        }
    }
    if(multilevel_wildcard == 0 && (tpos < topiclen || spos < sublen))
    {
        *result = 0;
    }

    return 0;
}


int deliverMessage(MQTTClient* c, MQTTString* topicName, MQTTMessage* message)
{
    int i;
    int rc = FAILURE;
    char match_rst = 0;

    // we have to find the right message handler - indexed by topic
    for (i = 0; i < MAX_MESSAGE_HANDLERS; ++i)
    {
        if (c->messageHandlers[i].topicFilter != 0)
        {
            (void)MQTTTopicMatched((const char *)c->messageHandlers[i].topicFilter, topicName, &match_rst);
            if (1 == match_rst && c->messageHandlers[i].fp != NULL)
            {
                MessageData md;
                NewMessageData(&md, topicName, message, (const char *)c->messageHandlers[i].topicFilter);
                c->messageHandlers[i].fp(&md);
                rc = MQTT_SUCCESS;
            }
        }
    }

    if (rc == FAILURE && c->defaultMessageHandler != NULL)
    {
        MessageData md;
        NewMessageData(&md, topicName, message, NULL);
        c->defaultMessageHandler(&md);
        rc = MQTT_SUCCESS;
    }

    return rc;
}


int keepalive(MQTTClient* c)
{
    int rc = MQTT_SUCCESS;

    if (c->keepAliveInterval == 0)
        goto exit;

    if (TimerIsExpired(&c->last_sent) && TimerIsExpired(&c->last_received))
    {
        if (c->ping_outstanding)
            rc = FAILURE; /* PINGRESP not received in keepalive interval */
        else
        {
            Timer timer;
            TimerInit(&timer);
            TimerCountdownMS(&timer, 1000);
            int len = MQTTSerialize_pingreq(c->buf, c->buf_size);
            if (len > 0 && (rc = sendPacket(c, len, &timer)) == MQTT_SUCCESS) // send the ping packet
                c->ping_outstanding = 1;
        }
    }

exit:
    return rc;
}


void MQTTCleanSession(MQTTClient* c)
{
    int i = 0;

    for (i = 0; i < MAX_MESSAGE_HANDLERS; ++i)
        c->messageHandlers[i].topicFilter = NULL;
}


void MQTTCloseSession(MQTTClient* c)
{
    c->ping_outstanding = 0;
    c->isconnected = 0;
    if (c->cleansession)
        MQTTCleanSession(c);
}


int cycle(MQTTClient* c, Timer* timer)
{
    int len = 0,
        rc = MQTT_SUCCESS;
    Timer send_timer;

    int packet_type = readPacket(c, timer);     /* read the socket, see what work is due */

    switch (packet_type)
    {
    default:
        /* no more data to read, unrecoverable. Or read packet fails due to unexpected network error */
        rc = packet_type;
        goto exit;
    case 0: /* timed out reading packet */
        break;
    case CONNACK:
    case PUBACK:
    case SUBACK:
    case UNSUBACK:
        break;
    case PUBLISH:
    {
        MQTTString topicName;
        MQTTMessage msg;
        int intQoS;
        msg.payloadlen = 0; /* this is a size_t, but deserialize publish sets this as int */
        if (MQTTDeserialize_publish(&msg.dup, &intQoS, &msg.retained, &msg.id, &topicName,
                                    (unsigned char **)&msg.payload, (int *)&msg.payloadlen, c->readbuf, c->readbuf_size) != 1)
            goto exit;
        msg.qos = (enum QoS)intQoS;
        deliverMessage(c, &topicName, &msg);
        if (msg.qos != QOS0)
        {
            if (msg.qos == QOS1)
                len = MQTTSerialize_ack(c->buf, c->buf_size, PUBACK, 0, msg.id);
            else if (msg.qos == QOS2)
                len = MQTTSerialize_ack(c->buf, c->buf_size, PUBREC, 0, msg.id);
            if (len <= 0)
                rc = FAILURE;
            else
            {
                TimerInit(&send_timer);
                TimerCountdownMS(&send_timer, 1000);
                rc = sendPacket(c, len, &send_timer);
            }
            if (rc == FAILURE)
                goto exit; // there was a problem
        }
        break;
    }
    case PUBREC:
    case PUBREL:
    {
        unsigned short mypacketid;
        unsigned char dup, type;
        if (MQTTDeserialize_ack(&type, &dup, &mypacketid, c->readbuf, c->readbuf_size) != 1)
            rc = FAILURE;
        else if ((len = MQTTSerialize_ack(c->buf, c->buf_size,
                                          (packet_type == PUBREC) ? PUBREL : PUBCOMP, 0, mypacketid)) <= 0)
            rc = FAILURE;
        else
        {
            TimerInit(&send_timer);
            TimerCountdownMS(&send_timer, 1000);
            rc = sendPacket(c, len, &send_timer);
            if(rc != MQTT_SUCCESS)
                rc = FAILURE; // there was a problem
        }
        if (rc == FAILURE)
            goto exit; // there was a problem
        break;
    }

    case PUBCOMP:
        break;
    case PINGRESP:
        c->ping_outstanding = 0;
        break;
    }

    if (keepalive(c) != MQTT_SUCCESS)
    {
        //check only keepalive FAILURE status so that previous FAILURE status can be considered as FAULT
        rc = FAILURE;
    }

exit:
    if (rc == MQTT_SUCCESS)
        rc = packet_type;
    else if (c->isconnected)
        MQTTCloseSession(c);
    return rc;
}


int MQTTYield(MQTTClient* c, int timeout_ms)
{
    int rc = MQTT_SUCCESS;
    Timer timer;

    TimerInit(&timer);
    TimerCountdownMS(&timer, timeout_ms);

    do
    {
#if defined(MQTT_TASK)
        (void)MutexLock(&c->mutex);
#endif
        if ( (rc = cycle(c, &timer) ) < 0)
        {
            rc = FAILURE;
#if defined(MQTT_TASK)
            (void)MutexUnlock(&c->mutex);
#endif
            break;
        }
#if defined(MQTT_TASK)
        (void)MutexUnlock(&c->mutex);
#endif
    }
    while (!TimerIsExpired(&timer));

    return rc;
}


void MQTTRun(void* parm)
{
	Timer timer;
	MQTTClient* c = (MQTTClient*)parm;

	TimerInit(&timer);

	while (1)
	{
#if defined(MQTT_TASK)
		MutexLock(&c->mutex);
#endif
		TimerCountdownMS(&timer, 500); /* Don't wait too long if no traffic is incoming */
		cycle(c, &timer);
#if defined(MQTT_TASK)
		MutexUnlock(&c->mutex);
#endif
	}
}


#if defined(MQTT_TASK)
int MQTTStartTask(MQTTClient* client)
{
	return ThreadStart(&client->thread, &MQTTRun, client);
}
#endif


int waitfor(MQTTClient* c, int packet_type, Timer* timer)
{
    int rc = FAILURE;

    do
    {
        if (TimerIsExpired(timer))
            break; // we timed out
        rc = cycle(c, timer);
    }
    while (rc != packet_type && rc >= 0);

    return rc;
}




int MQTTConnectWithResults(MQTTClient* c, MQTTPacket_connectData* options, MQTTConnackData* data)
{
    Timer connect_timer;
    int rc = FAILURE;
    MQTTPacket_connectData default_options = MQTTPacket_connectData_initializer;
    int len = 0;

#if defined(MQTT_TASK)
	  MutexLock(&c->mutex);
#endif
	  if (c->isconnected) /* don't send connect packet again if we are already connected */
		  goto exit;

    TimerInit(&connect_timer);
    TimerCountdownMS(&connect_timer, c->command_timeout_ms);

    if (options == 0)
        options = &default_options; /* set default options if none were supplied */

    c->keepAliveInterval = options->keepAliveInterval;
    c->cleansession = options->cleansession;
    TimerCountdown(&c->last_received, c->keepAliveInterval);
    if ((len = MQTTSerialize_connect(c->buf, c->buf_size, options)) <= 0)
        goto exit;
    if ((rc = sendPacket(c, len, &connect_timer)) != MQTT_SUCCESS)  // send the connect packet
        goto exit; // there was a problem

    // this will be a blocking call, wait for the connack
    if (waitfor(c, CONNACK, &connect_timer) == CONNACK)
    {
        data->rc = 0;
        data->sessionPresent = 0;
        if (MQTTDeserialize_connack(&data->sessionPresent, &data->rc, c->readbuf, c->readbuf_size) == 1)
            rc = data->rc;
        else
            rc = FAILURE;
    }
    else
        rc = FAILURE;

exit:
    if (rc == MQTT_SUCCESS)
    {
        c->isconnected = 1;
        c->ping_outstanding = 0;
    }

#if defined(MQTT_TASK)
	  MutexUnlock(&c->mutex);
#endif

    return rc;
}


int MQTTConnect(MQTTClient* c, MQTTPacket_connectData* options)
{
    MQTTConnackData data;
    return MQTTConnectWithResults(c, options, &data);
}


int MQTTSetMessageHandler(MQTTClient* c, const char* topicFilter, messageHandler messageHandler)
{
    int rc = FAILURE;
    int i = -1;

    /* first check for an existing matching slot */
    for (i = 0; i < MAX_MESSAGE_HANDLERS; ++i)
    {
        if (c->messageHandlers[i].topicFilter != NULL && strcmp(c->messageHandlers[i].topicFilter, topicFilter) == 0)
        {
            if (messageHandler == NULL) /* remove existing */
            {
                c->messageHandlers[i].topicFilter = NULL;
                c->messageHandlers[i].fp = NULL;
            }
            rc = MQTT_SUCCESS; /* return i when adding new subscription */
            break;
        }
    }
    /* if no existing, look for empty slot (unless we are removing) */
    if (messageHandler != NULL)
    {
        if (rc == FAILURE)
        {
            for (i = 0; i < MAX_MESSAGE_HANDLERS; ++i)
            {
                if (c->messageHandlers[i].topicFilter == NULL)
                {
                    rc = MQTT_SUCCESS;
                    break;
                }
            }
        }
        if (i < MAX_MESSAGE_HANDLERS)
        {
            c->messageHandlers[i].topicFilter = topicFilter;
            c->messageHandlers[i].fp = messageHandler;
        }
    }
    return rc;
}


int MQTTSubscribeWithResults(MQTTClient *c, const char *topicFilter, enum QoS qos,
                             messageHandler messageHandler, MQTTSubackData *data)
{
    int rc = FAILURE;
    Timer timer;
    int len = 0;
    MQTTString topic = MQTTString_initializer;
    topic.cstring = (char *)topicFilter;

#if defined(MQTT_TASK)
    MutexLock(&c->mutex);
#endif
    if (!c->isconnected)
        goto exit;

    TimerInit(&timer);
    TimerCountdownMS(&timer, c->command_timeout_ms);

    len = MQTTSerialize_subscribe(c->buf, c->buf_size, 0, getNextPacketId(c), 1, &topic, (int *)&qos);
    if (len <= 0)
        goto exit;
    if ((rc = sendPacket(c, len, &timer)) != MQTT_SUCCESS) // send the subscribe packet
        goto exit;             // there was a problem

    if (waitfor(c, SUBACK, &timer) == SUBACK)      // wait for suback
    {
        int count = 0;
        unsigned short mypacketid;
        data->grantedQoS = QOS0;
        if (MQTTDeserialize_suback(&mypacketid, 1, &count, (int *)&data->grantedQoS, c->readbuf, c->readbuf_size) == 1)
        {
            if (data->grantedQoS != 0x80)
                rc = MQTTSetMessageHandler(c, topicFilter, messageHandler);
        }
    }
    else
        rc = FAILURE;

exit:
    if (rc == FAILURE)
        MQTTCloseSession(c);
#if defined(MQTT_TASK)
	  MutexUnlock(&c->mutex);
#endif
    return rc;
}


int MQTTSubscribe(MQTTClient* c, const char* topicFilter, enum QoS qos,
       messageHandler messageHandler)
{
    MQTTSubackData data;
    return MQTTSubscribeWithResults(c, topicFilter, qos, messageHandler, &data);
}


int MQTTUnsubscribe(MQTTClient* c, const char* topicFilter)
{
    int rc = FAILURE;
    Timer timer;
    MQTTString topic = MQTTString_initializer;
    topic.cstring = (char *)topicFilter;
    int len = 0;

#if defined(MQTT_TASK)
	  MutexLock(&c->mutex);
#endif
	  if (!c->isconnected)
		  goto exit;

    TimerInit(&timer);
    TimerCountdownMS(&timer, c->command_timeout_ms);

    if ((len = MQTTSerialize_unsubscribe(c->buf, c->buf_size, 0, getNextPacketId(c), 1, &topic)) <= 0)
        goto exit;
    if ((rc = sendPacket(c, len, &timer)) != MQTT_SUCCESS) // send the subscribe packet
        goto exit; // there was a problem

    if (waitfor(c, UNSUBACK, &timer) == UNSUBACK)
    {
        unsigned short mypacketid;  // should be the same as the packetid above
        if (MQTTDeserialize_unsuback(&mypacketid, c->readbuf, c->readbuf_size) == 1)
        {
            /* remove the subscription message handler associated with this topic, if there is one */
            MQTTSetMessageHandler(c, topicFilter, NULL);
        }
    }
    else
        rc = FAILURE;

exit:
    if (rc == FAILURE)
        MQTTCloseSession(c);
#if defined(MQTT_TASK)
	  MutexUnlock(&c->mutex);
#endif
    return rc;
}


int MQTTPublish(MQTTClient* c, const char* topicName, MQTTMessage* message)
{
    int rc = FAILURE;
    Timer timer;
    MQTTString topic = MQTTString_initializer;
    topic.cstring = (char *)topicName;
    int len = 0;

#if defined(MQTT_TASK)
	  MutexLock(&c->mutex);
#endif
	  if (!c->isconnected)
		    goto exit;

    TimerInit(&timer);
    if(message->qos == QOS2)
        TimerCountdownMS(&timer, c->command_timeout_ms * 2);
    else
        TimerCountdownMS(&timer, c->command_timeout_ms);

    if (message->qos == QOS1 || message->qos == QOS2)
        message->id = getNextPacketId(c);

    len = MQTTSerialize_publish(c->buf, c->buf_size, 0, message->qos, message->retained, message->id,
              topic, (unsigned char*)message->payload, message->payloadlen);
    if (len <= 0)
        goto exit;
    if ((rc = sendPacket(c, len, &timer)) != MQTT_SUCCESS) // send the subscribe packet
        goto exit; // there was a problem

    if (message->qos == QOS1)
    {
        if (waitfor(c, PUBACK, &timer) == PUBACK)
        {
            unsigned short mypacketid;
            unsigned char dup, type;
            if (MQTTDeserialize_ack(&type, &dup, &mypacketid, c->readbuf, c->readbuf_size) != 1)
                rc = FAILURE;
        }
        else
            rc = FAILURE;
    }
    else if (message->qos == QOS2)
    {
        if (waitfor(c, PUBCOMP, &timer) == PUBCOMP)
        {
            unsigned short mypacketid;
            unsigned char dup, type;
            if (MQTTDeserialize_ack(&type, &dup, &mypacketid, c->readbuf, c->readbuf_size) != 1)
                rc = FAILURE;
        }
        else
            rc = FAILURE;
    }

exit:
    if (rc == FAILURE)
        MQTTCloseSession(c);
#if defined(MQTT_TASK)
	  MutexUnlock(&c->mutex);
#endif
    return rc;
}


int MQTTDisconnect(MQTTClient* c)
{
    int rc = FAILURE;
    Timer timer;     // we might wait for incomplete incoming publishes to complete
    int len = 0;

#if defined(MQTT_TASK)
	MutexLock(&c->mutex);
#endif
    TimerInit(&timer);
    TimerCountdownMS(&timer, c->command_timeout_ms);

	  len = MQTTSerialize_disconnect(c->buf, c->buf_size);
    if (len > 0)
        rc = sendPacket(c, len, &timer);            // send the disconnect packet
    MQTTCloseSession(c);

#if defined(MQTT_TASK)
	  MutexUnlock(&c->mutex);
#endif
    return rc;
}
