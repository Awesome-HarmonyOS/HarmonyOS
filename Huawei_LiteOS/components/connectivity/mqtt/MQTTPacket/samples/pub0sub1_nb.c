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
 *    Sergio R. Caprile - clarifications and/or documentation extension
 *******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "MQTTPacket.h"
#include "transport.h"

/* This is in order to get an asynchronous signal to stop the sample,
as the code loops waiting for msgs on the subscribed topic.
Your actual code will depend on your hw and approach*/
#include <signal.h>

int toStop = 0;

void cfinish(int sig)
{
    signal(SIGINT, NULL);
    toStop = 1;
}

void stop_init(void)
{
    signal(SIGINT, cfinish);
    signal(SIGTERM, cfinish);
}
/* */

int main(int argc, char *argv[])
{
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    int rc = 0;
    int mysock = 0;
    unsigned char buf[200];
    int buflen = sizeof(buf);
    int msgid = 1;
    MQTTString topicString = MQTTString_initializer;
    int req_qos = 0;
    char *payload = "mypayload";
    int payloadlen = strlen(payload);
    int len = 0;
    char *host = "m2m.eclipse.org";
    int port = 1883;
    MQTTTransport mytransport;

    stop_init();
    if (argc > 1)
        host = argv[1];

    if (argc > 2)
        port = atoi(argv[2]);

    mysock = transport_open(host, port);
    if(mysock < 0)
        return mysock;

    printf("Sending to hostname %s port %d\n", host, port);

    mytransport.sck = &mysock;
    mytransport.getfn = transport_getdatanb;
    mytransport.state = 0;
    data.clientID.cstring = "me";
    data.keepAliveInterval = 20;
    data.cleansession = 1;
    data.username.cstring = "testuser";
    data.password.cstring = "testpassword";

    len = MQTTSerialize_connect(buf, buflen, &data);
    rc = transport_sendPacketBuffer(mysock, buf, len);

    /* wait for connack */
    if (MQTTPacket_read(buf, buflen, transport_getdata) == CONNACK)
    {
        unsigned char sessionPresent, connack_rc;

        if (MQTTDeserialize_connack(&sessionPresent, &connack_rc, buf, buflen) != 1 || connack_rc != 0)
        {
            printf("Unable to connect, return code %d\n", connack_rc);
            goto exit;
        }
    }
    else
        goto exit;

    /* subscribe */
    topicString.cstring = "substopic";
    len = MQTTSerialize_subscribe(buf, buflen, 0, msgid, 1, &topicString, &req_qos);

    rc = transport_sendPacketBuffer(mysock, buf, len);
    do
    {
        int frc;
        if ((frc = MQTTPacket_readnb(buf, buflen, &mytransport)) == SUBACK) /* wait for suback */
        {
            unsigned short submsgid;
            int subcount;
            int granted_qos;

            rc = MQTTDeserialize_suback(&submsgid, 1, &subcount, &granted_qos, buf, buflen);
            if (granted_qos != 0)
            {
                printf("granted qos != 0, %d\n", granted_qos);
                goto exit;
            }
            break;
        }
        else if (frc == -1)
            goto exit;
    }
    while (1);   /* handle timeouts here */
    /* loop getting msgs on subscribed topic */
    topicString.cstring = "pubtopic";
    while (!toStop)
    {
        /* handle timeouts */
        if (MQTTPacket_readnb(buf, buflen, &mytransport) == PUBLISH)
        {
            unsigned char dup;
            int qos;
            unsigned char retained;
            unsigned short msgid;
            int payloadlen_in;
            unsigned char *payload_in;
            int rc;
            MQTTString receivedTopic;

            rc = MQTTDeserialize_publish(&dup, &qos, &retained, &msgid, &receivedTopic,
                                         &payload_in, &payloadlen_in, buf, buflen);
            printf("message arrived %.*s\n", payloadlen_in, payload_in);
            printf("publishing reading\n");
            len = MQTTSerialize_publish(buf, buflen, 0, 0, 0, 0, topicString, (unsigned char *)payload, payloadlen);
            rc = transport_sendPacketBuffer(mysock, buf, len);
        }
    }

    printf("disconnecting\n");
    len = MQTTSerialize_disconnect(buf, buflen);
    rc = transport_sendPacketBuffer(mysock, buf, len);

exit:
    transport_close(mysock);

    return 0;
}
