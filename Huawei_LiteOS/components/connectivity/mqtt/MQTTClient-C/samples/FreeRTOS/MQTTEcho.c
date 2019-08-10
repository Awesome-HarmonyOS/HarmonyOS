/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

#define MQTT_TASK 1
#include "MQTTClient.h"


void messageArrived(MessageData *data)
{
    printf("Message arrived on topic %.*s: %.*s\n", data->topicName->lenstring.len, data->topicName->lenstring.data,
           data->message->payloadlen, data->message->payload);
}

static void prvMQTTEchoTask(void *pvParameters)
{
    /* connect to m2m.eclipse.org, subscribe to a topic, send and receive messages regularly every 1 sec */
    MQTTClient client;
    Network network;
    unsigned char sendbuf[80], readbuf[80];
    int rc = 0,
        count = 0;
    MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;

    pvParameters = 0;
    NetworkInit(&network);
    MQTTClientInit(&client, &network, 30000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));

    char *address = "iot.eclipse.org";
    if ((rc = NetworkConnect(&network, address, 1883)) != 0)
        printf("Return code from network connect is %d\n", rc);

#if defined(MQTT_TASK)
    if ((rc = MQTTStartTask(&client)) != pdPASS)
        printf("Return code from start tasks is %d\n", rc);
#endif

    connectData.MQTTVersion = 3;
    connectData.clientID.cstring = "FreeRTOS_sample";

    if ((rc = MQTTConnect(&client, &connectData)) != 0)
        printf("Return code from MQTT connect is %d\n", rc);
    else
        printf("MQTT Connected\n");

    if ((rc = MQTTSubscribe(&client, "FreeRTOS/sample/#", 2, messageArrived)) != 0)
        printf("Return code from MQTT subscribe is %d\n", rc);

    while (++count)
    {
        MQTTMessage message;
        char payload[30];

        message.qos = 1;
        message.retained = 0;
        message.payload = payload;
        sprintf(payload, "message number %d", count);
        message.payloadlen = strlen(payload);

        if ((rc = MQTTPublish(&client, "FreeRTOS/sample/a", &message)) != 0)
            printf("Return code from MQTT publish is %d\n", rc);
#if !defined(MQTT_TASK)
        if ((rc = MQTTYield(&client, 1000)) != 0)
            printf("Return code from yield is %d\n", rc);
#endif
    }

    /* do not return */
}


void vStartMQTTTasks(uint16_t usTaskStackSize, UBaseType_t uxTaskPriority)
{
    BaseType_t x = 0L;

    xTaskCreate(prvMQTTEchoTask,	/* The function that implements the task. */
                "MQTTEcho0",			/* Just a text name for the task to aid debugging. */
                usTaskStackSize,	/* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)x,		/* The task parameter, not used in this case. */
                uxTaskPriority,		/* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);				/* The task handle is not used. */
}
/*-----------------------------------------------------------*/


