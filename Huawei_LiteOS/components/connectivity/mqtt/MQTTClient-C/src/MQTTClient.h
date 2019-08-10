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
 *    Allan Stockdill-Mander/Ian Craggs - initial API and implementation and/or initial documentation
 *    Ian Craggs - documentation and platform specific header
 *    Ian Craggs - add setMessageHandler function
 *******************************************************************************/

#if !defined(MQTT_CLIENT_H)
#define MQTT_CLIENT_H

#if defined(__cplusplus)
 extern "C" {
#endif

#if defined(WIN32_DLL) || defined(WIN64_DLL)
  #define DLLImport __declspec(dllimport)
  #define DLLExport __declspec(dllexport)
#elif defined(LINUX_SO)
  #define DLLImport extern
  #define DLLExport  __attribute__ ((visibility ("default")))
#else
  #define DLLImport
  #define DLLExport
#endif

#include "MQTTPacket.h"
#include "stdio.h"
#define __MQTT_LITE_OS__
#ifdef __MQTT_LITE_OS__
#include "MQTTliteos.h"
#endif


#if defined(MQTTCLIENT_PLATFORM_HEADER)
/* The following sequence of macros converts the MQTTCLIENT_PLATFORM_HEADER value
 * into a string constant suitable for use with include.
 */
#define xstr(s) str(s)
#define str(s) #s
#include xstr(MQTTCLIENT_PLATFORM_HEADER)
#endif

#define MAX_PACKET_ID 65535 /* according to the MQTT specification - do not change! */

#if !defined(MAX_MESSAGE_HANDLERS)
#define MAX_MESSAGE_HANDLERS 5 /* redefinable - how many subscriptions do you want? */
#endif

enum QoS { QOS0, QOS1, QOS2, SUBFAIL=0x80 };

/* all failure return codes must be negative */
enum returnCode { BUFFER_OVERFLOW = -2, FAILURE = -1, MQTT_SUCCESS = 0 };

/* The Platform specific header must define the Network and Timer structures and functions
 * which operate on them.
 *
typedef struct Network
{
	int (*mqttread)(Network*, unsigned char* read_buffer, int, int);
	int (*mqttwrite)(Network*, unsigned char* send_buffer, int, int);
} Network;*/

/* The Timer structure must be defined in the platform specific header,
 * and have the following functions to operate on it.  */
extern void TimerInit(Timer*);
extern char TimerIsExpired(Timer*);
extern void TimerCountdownMS(Timer*, unsigned int);
extern void TimerCountdown(Timer*, unsigned int);
extern int TimerLeftMS(Timer*);

typedef struct MQTTMessage
{
    enum QoS qos;
    unsigned char retained;
    unsigned char dup;
    unsigned short id;
    void *payload;
    size_t payloadlen;
} MQTTMessage;

typedef struct MessageData
{
    MQTTMessage* message;
    MQTTString* topicName;
} MessageData;

typedef struct MQTTConnackData
{
    unsigned char rc;
    unsigned char sessionPresent;
} MQTTConnackData;

typedef struct MQTTSubackData
{
    enum QoS grantedQoS;
} MQTTSubackData;

typedef void (*messageHandler)(MessageData*);

typedef struct MQTTClient
{
    unsigned int next_packetid,
      command_timeout_ms;
    size_t buf_size,
      readbuf_size;
    unsigned char *buf,
      *readbuf;
    unsigned int keepAliveInterval;
    char ping_outstanding;
    int isconnected;
    int cleansession;

    struct MessageHandlers
    {
        const char* topicFilter;
        void (*fp) (MessageData*);
    } messageHandlers[MAX_MESSAGE_HANDLERS];      /* Message handlers are indexed by subscription topic */

    void (*defaultMessageHandler) (MessageData*);

    Network* ipstack;
    Timer last_sent, last_received;
#if defined(MQTT_TASK)
    Mutex mutex;
    Thread thread;
#endif
} MQTTClient;

#define DefaultClient {0, 0, 0, 0, NULL, NULL, 0, 0, 0}


/**
 * Create an MQTT client object
 * @param client
 * @param network
 * @param command_timeout_ms
 * @param
 */
DLLExport int MQTTClientInit(MQTTClient* client, Network* network, unsigned int command_timeout_ms,
		unsigned char* sendbuf, size_t sendbuf_size, unsigned char* readbuf, size_t readbuf_size);

DLLExport void MQTTClientDeInit(MQTTClient* c);

/** MQTT Connect - send an MQTT connect packet down the network and wait for a Connack
 *  The nework object must be connected to the network endpoint before calling this
 *  @param options - connect options
 *  @return success code
 */
DLLExport int MQTTConnectWithResults(MQTTClient* client, MQTTPacket_connectData* options,
    MQTTConnackData* data);

/** MQTT Connect - send an MQTT connect packet down the network and wait for a Connack
 *  The nework object must be connected to the network endpoint before calling this
 *  @param options - connect options
 *  @return success code
 */
DLLExport int MQTTConnect(MQTTClient* client, MQTTPacket_connectData* options);

/** MQTT Publish - send an MQTT publish packet and wait for all acks to complete for all QoSs
 *  @param client - the client object to use
 *  @param topic - the topic to publish to
 *  @param message - the message to send
 *  @return success code
 */
DLLExport int MQTTPublish(MQTTClient* client, const char*, MQTTMessage*);

/** MQTT SetMessageHandler - set or remove a per topic message handler
 *  @param client - the client object to use
 *  @param topicFilter - the topic filter set the message handler for
 *  @param messageHandler - pointer to the message handler function or NULL to remove
 *  @return success code
 */
DLLExport int MQTTSetMessageHandler(MQTTClient* c, const char* topicFilter, messageHandler messageHandler);

/** MQTT Subscribe - send an MQTT subscribe packet and wait for suback before returning.
 *  @param client - the client object to use
 *  @param topicFilter - the topic filter to subscribe to
 *  @param message - the message to send
 *  @return success code
 */
DLLExport int MQTTSubscribe(MQTTClient* client, const char* topicFilter, enum QoS, messageHandler);

/** MQTT Subscribe - send an MQTT subscribe packet and wait for suback before returning.
 *  @param client - the client object to use
 *  @param topicFilter - the topic filter to subscribe to
 *  @param message - the message to send
 *  @param data - suback granted QoS returned
 *  @return success code
 */
DLLExport int MQTTSubscribeWithResults(MQTTClient* client, const char* topicFilter, enum QoS, messageHandler, MQTTSubackData* data);

/** MQTT Subscribe - send an MQTT unsubscribe packet and wait for unsuback before returning.
 *  @param client - the client object to use
 *  @param topicFilter - the topic filter to unsubscribe from
 *  @return success code
 */
DLLExport int MQTTUnsubscribe(MQTTClient* client, const char* topicFilter);

/** MQTT Disconnect - send an MQTT disconnect packet and close the connection
 *  @param client - the client object to use
 *  @return success code
 */
DLLExport int MQTTDisconnect(MQTTClient* client);

/** MQTT Yield - MQTT background
 *  @param client - the client object to use
 *  @param time - the time, in milliseconds, to yield for
 *  @return success code
 */
DLLExport int MQTTYield(MQTTClient* client, int time);

/** MQTT isConnected
 *  @param client - the client object to use
 *  @return truth value indicating whether the client is connected to the server
 */
static inline DLLExport int MQTTIsConnected(MQTTClient* client)
{
  return client->isconnected;
}

#if defined(MQTT_TASK)
/** MQTT start background thread for a client.  After this, MQTTYield should not be called.
*  @param client - the client object to use
*  @return success code
*/
DLLExport int MQTTStartTask(MQTTClient* client);
#endif

#if defined(__cplusplus)
     }
#endif

#endif
