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
 *    Sergio R. Caprile - media specifics, nice api doc :^)
 *******************************************************************************/

typedef struct {
	int (*send)(unsigned char *address, unsigned int bytes); 	///< pointer to function to send 'bytes' bytes, returns the actual number of bytes sent
	int (*recv)(unsigned char *address, unsigned int maxbytes); 	///< pointer to function to receive upto 'maxbytes' bytes, returns the actual number of bytes copied
} transport_iofunctions_t;

#define TRANSPORT_DONE	1
#define TRANSPORT_AGAIN	0
#define TRANSPORT_ERROR	-1
/**
@note Blocks until requested buflen is sent
*/
int transport_sendPacketBuffer(int sock, unsigned char* buf, int buflen);
/**
@note Blocks until requested count is received, as MQTTPacket_read() expects
@warning This function is not supported (not implemented)
@warning unless you provide a timeout, this function can block forever. Socket based systems do have
a built in timeout, if your system can provide this, do modify this function, otherwise use getdatanb() instead
@returns number of bytes read
*/
int transport_getdata(unsigned char* buf, int count);

/**
This is a bare metal implementation, so we must have non-blocking functions,
the process of pumping to serial lines can result a bit slow and we don't want to busy wait.
This function starts the process, you will call sendPacketBuffernb() until it reports success (or error)
*/
void transport_sendPacketBuffernb_start(int sock, unsigned char* buf, int buflen);
/**
This is a bare metal implementation, so we must have non-blocking functions,
the process of pumping to serial lines can result a bit slow and we don't want to busy wait
@returns TRANSPORT_DONE if finished, TRANSPORT_AGAIN for call again, or TRANSPORT_ERROR on error
@note you will call again until it finishes (this is stream)
*/
int transport_sendPacketBuffernb(int sock);

/**
This is a bare metal implementation, so we must have non-blocking functions,
the process of sucking from serial lines can result a bit slow and we don't want to busy wait
@return the actual number of bytes read, 0 for none, or TRANSPORT_ERROR on error
@note you will call again until total number of expected bytes is read (this is stream)
*/
int transport_getdatanb(void *sck, unsigned char* buf, int count);

/**
We assume whatever connection needs to be done, it is externally established by the specifics of the hardware
E.g.:
A cell modem: you will call AT+whatever and put the modem in transparent mode, OR, you will embed
the AT+xSENDx / AT+xRECVx commands into the former sendPacketBuffer() and getdatanb() functions
@param	thisio	pointer to a structure containing all necessary stuff to handle direct serial I/O
@returns	whatever indicator the system assigns to this link, if any. (a.k.a. : 'sock'), or TRANSPORT_ERROR for error
*/
int transport_open(transport_iofunctions_t *thisio);
int transport_close(int sock);
