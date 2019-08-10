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
 *    Allan Stockdill-Mander - initial API and implementation and/or initial documentation
 *******************************************************************************/

#ifndef __MQTT_CC3200_
#define __MQTT_CC3200_

#include "simplelink.h"
#include "netapp.h"
#include "socket.h"
#include "hw_types.h"
#include "systick.h"

typedef struct Timer Timer;

struct Timer {
	unsigned long systick_period;
	unsigned long end_time;
};

typedef struct Network Network;

struct Network
{
	int my_socket;
	int (*mqttread) (Network*, unsigned char*, int, int);
	int (*mqttwrite) (Network*, unsigned char*, int, int);
	void (*disconnect) (Network*);
};

char expired(Timer*);
void countdown_ms(Timer*, unsigned int);
void countdown(Timer*, unsigned int);
int left_ms(Timer*);

void InitTimer(Timer*);

int cc3200_read(Network*, unsigned char*, int, int);
int cc3200_write(Network*, unsigned char*, int, int);
void cc3200_disconnect(Network*);
void NewNetwork(Network*);

int ConnectNetwork(Network*, char*, int);
int TLSConnectNetwork(Network*, char*, int, SlSockSecureFiles_t*, unsigned char, unsigned int, char);

#endif
