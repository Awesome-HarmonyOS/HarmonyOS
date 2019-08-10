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
 *    Sergio R. Caprile - port to the bare metal environment
 *******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "MQTTPacket.h"
#include "transport.h"

/* This is in order to get an asynchronous signal to stop the sample,
as the code loops waiting for msgs on the subscribed topic.
Your actual code will depend on your hw and approach, but this sample can be
run on Linux so debugging of the non-hardware specific bare metal code is easier.
See at bottom of file for details */
#include <signal.h>

int toStop = 0;

void stop_init(void);
/* */

/* Same as above, we provide a set of functions to test/debug on a friendlier system;
the init() and  close() actions on the serial are just for this, you will probably
handle this on whatever handles your media in your application */
void sampleserial_init(void);
void sampleserial_close(void);
int samplesend(unsigned char *address, unsigned int bytes);
int samplerecv(unsigned char *address, unsigned int maxbytes);
/* */

/* You will use your hardware specifics here, see transport.h. */
static transport_iofunctions_t iof = {samplesend, samplerecv};

enum states { READING, PUBLISHING };

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
    MQTTTransport mytransport;
    int state = READING;

    stop_init();
    sampleserial_init();

    mysock = transport_open(&iof);
    if(mysock < 0)
        return mysock;
    /* You will (or already are) 'somehow' connect(ed) to host:port via your hardware specifics. E.g.:
    	you have a serial (RS-232/UART) link
    	you have a cell modem and you issue your AT+ magic
    	you have some TCP/IP which is not lwIP (nor a full-fledged socket compliant one)
    	 and you TCP connect
    */

    mytransport.sck = &mysock;
    mytransport.getfn = transport_getdatanb;
    mytransport.state = 0;
    data.clientID.cstring = "me";
    data.keepAliveInterval = 20;
    data.cleansession = 1;
    data.username.cstring = "testuser";
    data.password.cstring = "testpassword";

    len = MQTTSerialize_connect(buf, buflen, &data);
    /* This one blocks until it finishes sending, you will probably not want this in real life,
    in such a case replace this call by a scheme similar to the one you'll see in the main loop */
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
    /* subscribe */
    topicString.cstring = "substopic";
    len = MQTTSerialize_subscribe(buf, buflen, 0, msgid, 1, &topicString, &req_qos);

    /* This is equivalent to the one above, but using the non-blocking functions. You will probably not want this in real life,
    in such a case replace this call by a scheme similar to the one you'll see in the main loop */
    transport_sendPacketBuffernb_start(mysock, buf, len);
    while((rc = transport_sendPacketBuffernb(mysock)) != TRANSPORT_DONE);
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
    printf("Subscribed\n");

    /* loop getting msgs on subscribed topic */
    topicString.cstring = "pubtopic";
    state = READING;
    while (!toStop)
    {
        /* do other stuff here */
        switch(state)
        {
        case READING:
            if((rc = MQTTPacket_readnb(buf, buflen, &mytransport)) == PUBLISH)
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
                state = PUBLISHING;
                len = MQTTSerialize_publish(buf, buflen, 0, 0, 0, 0, topicString, (unsigned char *)payload, payloadlen);
                transport_sendPacketBuffernb_start(mysock, buf, len);
            }
            else if(rc == -1)
            {
                /* handle I/O errors here */
                goto exit;
            }	/* handle timeouts here */
            break;
        case PUBLISHING:
            switch(transport_sendPacketBuffernb(mysock))
            {
            case TRANSPORT_DONE:
                printf("Published\n");
                state = READING;
                break;
            case TRANSPORT_ERROR:
                /* handle any I/O errors here */
                goto exit;
                break;
            case TRANSPORT_AGAIN:
            default:
                /* handle timeouts here, not probable unless there is a hardware problem */
                break;
            }
            break;
        }
    }

    printf("disconnecting\n");
    len = MQTTSerialize_disconnect(buf, buflen);
    /* Same blocking related stuff here */
    rc = transport_sendPacketBuffer(mysock, buf, len);

exit:
    transport_close(mysock);

    sampleserial_close();
    return 0;
}


/* To stop the sample */
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

/* Serial hack:
Simulate serial transfers on an established TCP connection
 */
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

static int sockfd;

void sampleserial_init(void)
{
    struct sockaddr_in serv_addr;


    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        perror(NULL);
        exit(2);
    }
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr("198.41.30.241");
    serv_addr.sin_port = htons(1883);
    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("ERROR connecting\n");
        exit(-1);
    }
    printf("- TCP Connected to Eclipse\n");
    /* set to non-blocking */
    fcntl(sockfd, F_SETFL, fcntl(sockfd, F_GETFL) | O_NONBLOCK);
}

void sampleserial_close(void)
{
    close(sockfd);
}

int samplesend(unsigned char *address, unsigned int bytes)
{
    int len;

    if(rand() > (RAND_MAX / 2))	// 50% probability of being busy
        return 0;
    if(rand() > (RAND_MAX / 2)) 	// 50% probability of sending half the requested data (no room in buffer)
    {
        if(bytes > 1)
            bytes /= 2;
    }
    if((len = write(sockfd, address, bytes)) >= 0)
        return len;
    if(errno == EAGAIN)
        return 0;
    return -1;
}

int samplerecv(unsigned char *address, unsigned int maxbytes)
{
    int len;

    if(rand() > (RAND_MAX / 2))	// 50% probability of no data
        return 0;
    if(rand() > (RAND_MAX / 2)) 	// 50% probability of getting half the requested data (not arrived yet)
    {
        if(maxbytes > 1)
        {
            maxbytes /= 2;
        }
    }
    if((len = read(sockfd, address, maxbytes)) >= 0)
        return len;
    if(errno == EAGAIN)
        return 0;
    return -1;
}

