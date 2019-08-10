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

#ifndef MQTT_LITE_OS_H
#define MQTT_LITE_OS_H


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <los_typedef.h>
#include <los_sys.h>
#include "sal/atiny_socket.h"
#include "log/atiny_log.h"
#include "atiny_mqtt/mqtt_client.h"

#define MQTT_TASK 1

typedef struct Timer
{
	unsigned long long end_time;
} Timer;

void TimerInit(Timer*);
char TimerIsExpired(Timer*);
void TimerCountdownMS(Timer*, unsigned int);
void TimerCountdown(Timer*, unsigned int);
int TimerLeftMS(Timer*);

typedef struct atiny_task_mutex_tag_s Mutex;
int MutexInit(Mutex* mutex);
int MutexLock(Mutex* mutex);
int MutexUnlock(Mutex* mutex);
void MutexDestory(Mutex* mutex);

typedef struct
{
	void * no_used;
} Thread;
int ThreadStart(Thread *thread, void (*fn)(void *), void *arg);

typedef struct mqtt_context
{
    int fd;
}mqtt_context_t;

typedef struct Network
{
    void *ctx;
    int (*mqttread) (struct Network*, unsigned char*, int, int);
    int (*mqttwrite) (struct Network*, unsigned char*, int, int);

    mqtt_security_info_s *(*get_security_info)(void);
} Network;

void NetworkInit(Network* n, mqtt_security_info_s *(*get_security_info)(void));
int NetworkConnect(Network*, char*, int);
void NetworkDisconnect(Network*);

#endif
