/*----------------------------------------------------------------------------
 * Copyright (c) <2018>, <Huawei Technologies Co., Ltd>
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

/**@defgroup atiny_socket Agenttiny Socket
 * @ingroup agent
 */

#ifndef _ATINY_SOCKET_H_
#define _ATINY_SOCKET_H_

#include <stdio.h>
#include <stdint.h>

#define ATINY_PROTO_TCP 0 /* < The TCP transport protocol */
#define ATINY_PROTO_UDP 1 /* < The UDP transport protocol */

#define ATINY_NET_OK               0
#define ATINY_NET_ERR             -1
#define ATINY_NET_TIMEOUT         -2
#define ATINY_NET_BIND_FAILED     -3
#define ATINY_NET_LISTEN_FAILED   -4
#define ATINY_NET_ACCEPT_FAILED   -5
#define ATINY_NET_BUF_SMALL_FAILED -6
#define ATINY_NET_SOCKET_FAILED    -7


#ifdef __cplusplus
extern "C" {
#endif


int atiny_net_accept( void *bind_ctx,
                        void *client_ctx,
                        void *client_ip, size_t buf_size, size_t *ip_len );

/**
 *@ingroup atiny_socket
 *@brief create socket and bind local ip and port
 *
 *@par Description:
 *This API is used to create socket and bind local ip and port. It will do name resolution with both IPv6 and IPv4, do internal memory allocation for socket structure and release it by the atiny_net_close interface.
 *used only in bootstrap server mode.
 *@attention none.
 *
 *@param host           [IN] Host to bind.
 *@param port           [IN] Port to bind.
 *@param proto          [IN] Protocol: ATINY_PROTO_TCP or ATINY_PROTO_UDP.
 *
 *@retval #pointer      Point to the Socket you created.
 *@retval NULL          Create socket or binded ip or port failed.
 *@par Dependency: none.
 *@see atiny_net_recv | atiny_net_send | atiny_net_recv_timeout | atiny_net_close
 */
void *atiny_net_bind(const char *host, const char *port, int proto);

/**
 *@ingroup atiny_socket
 *@brief create socket and connect to the server
 *
 *@par Description:
 *This API is used to create socket and connect to the server. It will do name resolution with both IPv6 and IPv4, do internal memory allocation for socket structure and release it by the atiny_net_close interface.
 *@attention none.
 *
 *@param host           [IN] Host to connect to.
 *@param port           [IN] Port to connect to.
 *@param proto          [IN] Protocol: ATINY_PROTO_TCP or ATINY_PROTO_UDP.
 *
 *@retval #pointer      Point to the Socket you created.
 *@retval NULL          Create socket or connect to server failed.
 *@par Dependency: none.
 *@see atiny_net_recv | atiny_net_send | atiny_net_recv_timeout | atiny_net_close
 */
void* atiny_net_connect(const char* host, const char* port, int proto);

/**
 *@ingroup atiny_socket
 *@brief read characters from peer.
 *
 *@par Description:
 *This API is used to read at most 'len' characters. If no error occurs, the actual amount read is returned.
 *@attention none.
 *
 *@param ctx            [IN] Pointer to the socket.
 *@param buf            [IN] The buffer to write to.
 *@param len            [IN] Maximum length of the buffer.
 *
 *@retval #int          the number of bytes received.
 *@retval 0             indicates recv() would block.
 *@retval -1            recv failed, you should try to connect again.
 *@par Dependency: none.
 *@see atiny_net_connect | atiny_net_send | atiny_net_recv_timeout | atiny_net_close
 */
int atiny_net_recv(void* ctx, unsigned char* buf, size_t len);

/**
 *@ingroup atiny_socket
 *@brief write characters to peer.
 *
 *@par Description:
 *This API is used to write at most 'len' characters. If no error occurs, the actual amount send is returned.
 *@attention none.
 *
 *@param ctx            [IN] Pointer to the socket.
 *@param buf            [IN] The buffer to read from.
 *@param len            [IN] The length of the buffer.
 *
 *@retval #int          the number of bytes sent.
 *@retval 0             indicates send() would block.
 *@retval -1            send failed, you should try to connect again.
 *@par Dependency: none.
 *@see atiny_net_connect | atiny_net_recv | atiny_net_recv_timeout | atiny_net_close
 */
int atiny_net_send(void* ctx, const unsigned char* buf, size_t len);

/**
 *@ingroup atiny_socket
 *@brief read characters from peer with timeout option.
 *
 *@par Description:
 *This API is used to read at most 'len' characters, blocking for at most 'timeout' seconds. If no error occurs, the actual amount read is returned.
 *@attention
 *<ul>
 *<li>This function will block (until data becomes available or</li>
 *<li>timeout is reached) even if the socket is set to</li>
 *<li>non-blocking. Handling timeouts with non-blocking reads</li>
 *<li>requires a different strategy.</li>
 *</ul>
 *
 *@param ctx            [IN] Pointer to the socket.
 *@param buf            [IN] The buffer to write to.
 *@param len            [IN] Maximum length of the buffer.
 *@param timeout        [IN] Maximum number of milliseconds to wait for data, 0 means no timeout (wait forever).
 *
 *@retval #int          the number of bytes sent.
 *@retval 0             indicates recv() would block.
 *@retval -1            recv failed, you should try to connect again.
 *@retval -2            this operation is timed out.
 *@par Dependency: none.
 *@see atiny_net_connect | atiny_net_recv | atiny_net_send | atiny_net_close
 */
int atiny_net_recv_timeout(void* ctx, unsigned char* buf, size_t len,
                           uint32_t timeout);

int atiny_net_send_timeout(void *ctx, const unsigned char *buf, size_t len,
                          uint32_t timeout);


/**
 *@ingroup atiny_socket
 *@brief gracefully shutdown the connection.
 *
 *@par Description:
 *This API is used to gracefully shutdown the connection and free associated data.
 *@attention none.
 *
 *@param ctx            [IN] Pointer to the socket to free.
 *
 *@retval none.
 *@par Dependency: none.
 *@see atiny_net_connect | atiny_net_recv | atiny_net_send | atiny_net_recv_timeout
 */
void atiny_net_close(void* ctx);

#ifdef __cplusplus
}
#endif

#endif /* _ATINY_SOCKET_H_ */

