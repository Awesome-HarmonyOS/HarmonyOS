/*******************************************************************************
 * Copyright (c) 2014, 2015 IBM Corp.
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
 *    Allan Stockdill-Mander - initial API and implementation and/or initial documentation
 *    Ian Craggs - convert to FreeRTOS
 *******************************************************************************/

#include "MQTTFreeRTOS.h"


int ThreadStart(Thread *thread, void (*fn)(void *), void *arg)
{
    int rc = 0;
    uint16_t usTaskStackSize = (configMINIMAL_STACK_SIZE * 5);
    UBaseType_t uxTaskPriority = uxTaskPriorityGet(NULL); /* set the priority as the same as the calling task*/

    rc = xTaskCreate(fn,	/* The function that implements the task. */
                     "MQTTTask",			/* Just a text name for the task to aid debugging. */
                     usTaskStackSize,	/* The stack size is defined in FreeRTOSIPConfig.h. */
                     arg,				/* The task parameter, not used in this case. */
                     uxTaskPriority,		/* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                     &thread->task);		/* The task handle is not used. */

    return rc;
}


void MutexInit(Mutex *mutex)
{
    mutex->sem = xSemaphoreCreateMutex();
}

int MutexLock(Mutex *mutex)
{
    return xSemaphoreTake(mutex->sem, portMAX_DELAY);
}

int MutexUnlock(Mutex *mutex)
{
    return xSemaphoreGive(mutex->sem);
}


void TimerCountdownMS(Timer *timer, unsigned int timeout_ms)
{
    timer->xTicksToWait = timeout_ms / portTICK_PERIOD_MS; /* convert milliseconds to ticks */
    vTaskSetTimeOutState(&timer->xTimeOut); /* Record the time at which this function was entered. */
}


void TimerCountdown(Timer *timer, unsigned int timeout)
{
    TimerCountdownMS(timer, timeout * 1000);
}


int TimerLeftMS(Timer *timer)
{
    xTaskCheckForTimeOut(&timer->xTimeOut, &timer->xTicksToWait); /* updates xTicksToWait to the number left */
    return (timer->xTicksToWait < 0) ? 0 : (timer->xTicksToWait * portTICK_PERIOD_MS);
}


char TimerIsExpired(Timer *timer)
{
    return xTaskCheckForTimeOut(&timer->xTimeOut, &timer->xTicksToWait) == pdTRUE;
}


void TimerInit(Timer *timer)
{
    timer->xTicksToWait = 0;
    memset(&timer->xTimeOut, '\0', sizeof(timer->xTimeOut));
}


int FreeRTOS_read(Network *n, unsigned char *buffer, int len, int timeout_ms)
{
    TickType_t xTicksToWait = timeout_ms / portTICK_PERIOD_MS; /* convert milliseconds to ticks */
    TimeOut_t xTimeOut;
    int recvLen = 0;

    vTaskSetTimeOutState(&xTimeOut); /* Record the time at which this function was entered. */
    do
    {
        int rc = 0;

        FreeRTOS_setsockopt(n->my_socket, 0, FREERTOS_SO_RCVTIMEO, &xTicksToWait, sizeof(xTicksToWait));
        rc = FreeRTOS_recv(n->my_socket, buffer + recvLen, len - recvLen, 0);
        if (rc > 0)
            recvLen += rc;
        else if (rc < 0)
        {
            recvLen = rc;
            break;
        }
    }
    while (recvLen < len && xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE);

    return recvLen;
}


int FreeRTOS_write(Network *n, unsigned char *buffer, int len, int timeout_ms)
{
    TickType_t xTicksToWait = timeout_ms / portTICK_PERIOD_MS; /* convert milliseconds to ticks */
    TimeOut_t xTimeOut;
    int sentLen = 0;

    vTaskSetTimeOutState(&xTimeOut); /* Record the time at which this function was entered. */
    do
    {
        int rc = 0;

        FreeRTOS_setsockopt(n->my_socket, 0, FREERTOS_SO_RCVTIMEO, &xTicksToWait, sizeof(xTicksToWait));
        rc = FreeRTOS_send(n->my_socket, buffer + sentLen, len - sentLen, 0);
        if (rc > 0)
            sentLen += rc;
        else if (rc < 0)
        {
            sentLen = rc;
            break;
        }
    }
    while (sentLen < len && xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE);

    return sentLen;
}


void FreeRTOS_disconnect(Network *n)
{
    FreeRTOS_closesocket(n->my_socket);
}


void NetworkInit(Network *n)
{
    n->my_socket = 0;
    n->mqttread = FreeRTOS_read;
    n->mqttwrite = FreeRTOS_write;
    n->disconnect = FreeRTOS_disconnect;
}


int NetworkConnect(Network *n, char *addr, int port)
{
    struct freertos_sockaddr sAddr;
    int retVal = -1;
    uint32_t ipAddress;

    if ((ipAddress = FreeRTOS_gethostbyname(addr)) == 0)
        goto exit;

    sAddr.sin_port = FreeRTOS_htons(port);
    sAddr.sin_addr = ipAddress;

    if ((n->my_socket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP)) < 0)
        goto exit;

    if ((retVal = FreeRTOS_connect(n->my_socket, &sAddr, sizeof(sAddr))) < 0)
    {
        FreeRTOS_closesocket(n->my_socket);
        goto exit;
    }

exit:
    return retVal;
}


#if 0
int NetworkConnectTLS(Network *n, char *addr, int port, SlSockSecureFiles_t *certificates, unsigned char sec_method, unsigned int cipher, char server_verify)
{
    SlSockAddrIn_t sAddr;
    int addrSize;
    int retVal;
    unsigned long ipAddress;

    retVal = sl_NetAppDnsGetHostByName(addr, strlen(addr), &ipAddress, AF_INET);
    if (retVal < 0)
    {
        return -1;
    }

    sAddr.sin_family = AF_INET;
    sAddr.sin_port = sl_Htons((unsigned short)port);
    sAddr.sin_addr.s_addr = sl_Htonl(ipAddress);

    addrSize = sizeof(SlSockAddrIn_t);

    n->my_socket = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, SL_SEC_SOCKET);
    if (n->my_socket < 0)
    {
        return -1;
    }

    SlSockSecureMethod method;
    method.secureMethod = sec_method;
    retVal = sl_SetSockOpt(n->my_socket, SL_SOL_SOCKET, SL_SO_SECMETHOD, &method, sizeof(method));
    if (retVal < 0)
    {
        return retVal;
    }

    SlSockSecureMask mask;
    mask.secureMask = cipher;
    retVal = sl_SetSockOpt(n->my_socket, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &mask, sizeof(mask));
    if (retVal < 0)
    {
        return retVal;
    }

    if (certificates != NULL)
    {
        retVal = sl_SetSockOpt(n->my_socket, SL_SOL_SOCKET, SL_SO_SECURE_FILES, certificates->secureFiles, sizeof(SlSockSecureFiles_t));
        if (retVal < 0)
        {
            return retVal;
        }
    }

    retVal = sl_Connect(n->my_socket, (SlSockAddr_t *)&sAddr, addrSize);
    if (retVal < 0)
    {
        if (server_verify || retVal != -453)
        {
            sl_Close(n->my_socket);
            return retVal;
        }
    }

    SysTickIntRegister(SysTickIntHandler);
    SysTickPeriodSet(80000);
    SysTickEnable();

    return retVal;
}
#endif
