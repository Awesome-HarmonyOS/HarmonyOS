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
 *    Sergio R. Caprile - port and nonblocking
 *******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "MQTTPacket.h"
#include "transport.h"

#define KEEPALIVE_INTERVAL 20

/* This is to get a timebase in seconds to test the sample */
#include <time.h>
time_t old_t;
void start_ping_timer(void)
{
    time(&old_t);
    old_t += KEEPALIVE_INTERVAL / 2 + 1;
}

int time_to_ping(void)
{
    time_t t;

    time(&t);
    if(t >= old_t)
        return 1;
    return 0;
}

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

enum states { IDLE, GETPONG };

int main(int argc, char *argv[])
{
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    int rc = 0;
    int mysock = 0;
    unsigned char buf[200];
    int buflen = sizeof(buf);
    int len = 0;
    char *host = "m2m.eclipse.org";
    int port = 1883;
    MQTTTransport mytransport;
    int state;

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
    data.keepAliveInterval = KEEPALIVE_INTERVAL;
    data.cleansession = 1;
    data.username.cstring = "testuser";
    data.password.cstring = "testpassword";

    len = MQTTSerialize_connect(buf, buflen, &data);
    rc = transport_sendPacketBuffer(mysock, buf, len);

    printf("Sent MQTT connect\n");
    /* wait for connack */
    do
    {
        int frc;
        if ((frc = MQTTPacket_readnb(buf, buflen, &mytransport)) == CONNACK)
        {
            unsigned char sessionPresent, connack_rc;
            if (MQTTDeserialize_connack(&sessionPresent, &connack_rc, buf, buflen) != 1 || connack_rc != 0)
            {
                printf("Unable to connect, return code %d\n", connack_rc);
                goto exit;
            }
            break;
        }
        else if (frc == -1)
            goto exit;
    }
    while (1);   /* handle timeouts here */

    printf("MQTT connected\n");
    start_ping_timer();

    state = IDLE;
    while (!toStop)
    {
        switch(state)
        {
        case IDLE:
            if(time_to_ping())
            {
                len = MQTTSerialize_pingreq(buf, buflen);
                transport_sendPacketBuffer(mysock, buf, len);
                printf("Ping...");
                state = GETPONG;
            }
            break;
        case GETPONG:
            if((rc = MQTTPacket_readnb(buf, buflen, &mytransport)) == PINGRESP)
            {
                printf("Pong\n");
                start_ping_timer();
                state = IDLE;
            }
            else if(rc == -1)
            {
                printf("OOPS\n");
                goto exit;
            }
            break;
        }
    }

    printf("disconnecting\n");
    len = MQTTSerialize_disconnect(buf, buflen);
    rc = transport_sendPacketBuffer(mysock, buf, len);

exit:
    transport_close(mysock);

    return 0;
}
