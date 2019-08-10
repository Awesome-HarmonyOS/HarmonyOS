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


int main(int argc, char *argv[])
{
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    int rc = 0;
    char buf[200];
    int buflen = sizeof(buf);
    int mysock = 0;
    MQTTString topicString = MQTTString_initializer;
    char *payload = "mypayload";
    int payloadlen = strlen(payload);
    int len = 0;
    char *host = "m2m.eclipse.org";
    int port = 1883;

    if (argc > 1)
        host = argv[1];

    if (argc > 2)
        port = atoi(argv[2]);

    mysock = transport_open(host, port);
    if(mysock < 0)
        return mysock;

    printf("Sending to hostname %s port %d\n", host, port);

    data.clientID.cstring = "me";
    data.keepAliveInterval = 20;
    data.cleansession = 1;
    data.username.cstring = "testuser";
    data.password.cstring = "testpassword";
    data.MQTTVersion = 4;

    len = MQTTSerialize_connect((unsigned char *)buf, buflen, &data);

    topicString.cstring = "mytopic";
    len += MQTTSerialize_publish((unsigned char *)(buf + len), buflen - len, 0, 0, 0, 0, topicString, (unsigned char *)payload, payloadlen);

    len += MQTTSerialize_disconnect((unsigned char *)(buf + len), buflen - len);

    rc = transport_sendPacketBuffer(mysock, (unsigned char *)buf, len);
    if (rc == len)
        printf("Successfully published\n");
    else
        printf("Publish failed\n");

exit:
    transport_close(mysock);

    return 0;
}
