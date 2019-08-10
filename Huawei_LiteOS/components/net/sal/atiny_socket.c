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

#include "sal/atiny_socket.h"
#include "osdepends/atiny_osdep.h"


#if defined(WITH_LINUX)
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <netdb.h>
#include <errno.h>
#elif defined(WITH_LWIP)
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/errno.h"
#elif defined(WITH_AT_FRAMEWORK)
#include "at_frame/at_api.h"

#else
// TODO
#endif

#define SOCKET_DEBUG

#if defined(SOCKET_DEBUG)
#define SOCKET_LOG(fmt, ...) \
    do \
    { \
        (void)atiny_printf("[SOCKET][%s:%d] " fmt "\r\n", \
        __FUNCTION__, __LINE__, ##__VA_ARGS__); \
    } while(0)
#else
#define SOCKET_LOG(fmt, ...) ((void)0)
#endif


typedef struct
{
    int fd;
} atiny_net_context;

void *atiny_net_bind(const char *host, const char *port, int proto)
{
#if defined (WITH_LWIP) || defined (WITH_LINUX)
    (void)host;
    (void)port;
    (void)proto;
    return NULL;
/*
    struct sockaddr_in sock_addr;
    int port_i;
    int ret = ATINY_NET_ERR;

    if (NULL == port || (proto != ATINY_PROTO_UDP && proto != ATINY_PROTO_TCP))
        return NULL;

    ctx = atiny_malloc(sizeof(atiny_net_context));

    sscanf(port , "%d", &port_i);
    sock_addr.sin_family = AF_INET;
    sock_addr.sin_port = lwip_htons(port_i);
    sock_addr.sin_addr.s_addr = (host == NULL ? IPADDR_ANY : inet_addr(host));
    sock_addr.sin_len = sizeof(struct sockaddr_in);

    ctx->fd = socket(AF_INET,
        proto == ATINY_PROTO_TCP ? SOCK_STREAM : SOCK_DGRAM,
        proto == ATINY_PROTO_TCP ? IPPROTO_TCP : IPPROTO_UDP);

    if (ctx->fd < 0)
    {
        ret = ATINY_NET_SOCKET_FAILED;
        atiny_free(ctx);
        return NULL;
    }

	int n = 1;
	if( (ret = setsockopt( ctx->fd, SOL_SOCKET, SO_REUSEADDR,
					(const char *) &n, sizeof( n )) ) != 0 )
	{
		ret = ATINY_NET_SOCKET_FAILED;
		goto exit_failed;
	}

    ret = bind(ctx->fd, (struct sockaddr*)&sock_addr, sizeof(struct sockaddr));
    if (ret < 0)
    {
       ret = ATINY_NET_BIND_FAILED;
       goto exit_failed;
    }

    if (proto == ATINY_PROTO_TCP)
    {
        ret = listen(ctx->fd, 20);
        if (ret < 0)
        {
            ret = ATINY_NET_LISTEN_FAILED;
            goto exit_failed;
        }
    }

    return ctx;

exit_failed:
       close(ctx->fd);
       atiny_free(ctx);
       return NULL;
       */


#elif defined(WITH_AT_FRAMEWORK)
    atiny_net_context *ctx;
    ctx = atiny_malloc(sizeof(atiny_net_context));
    if (NULL == ctx)
    {
    	SOCKET_LOG("malloc failed for socket context");
    	return NULL;
    }

    ctx->fd = at_api_bind(host, port, proto);
    if (ctx->fd < 0)
    {
    	SOCKET_LOG("unkown host or port");
    	atiny_free(ctx);
    	ctx = NULL;
    }
		return ctx;
#endif

}
int atiny_net_accept( void *bind_ctx, void *client_ctx, void *client_ip, size_t buf_size, size_t *ip_len )
{
#if defined (WITH_LWIP) || defined (WITH_LINUX)
    int bind_fd = ((atiny_net_context*)bind_ctx)->fd;
    int client_fd = ((atiny_net_context*)client_ctx)->fd;
    int type;
    int ret = ATINY_NET_ERR;
#if LWIP_IPV4 && LWIP_IPV6
#elif LWIP_IPV6
    struct sockaddr_in6 client_addr;
#else
    struct sockaddr_in client_addr;
#endif
    socklen_t type_len, client_addr_len;

    type_len = sizeof(type);
    if (getsockopt(bind_fd, SOL_SOCKET, SO_TYPE, (void*)&type, &type_len) != 0 ||
        (type != SOCK_STREAM && type != SOCK_DGRAM))
    {
        return ATINY_NET_ACCEPT_FAILED;
    }
#if LWIP_IPV4 && LWIP_IPV6
#elif LWIP_IPV6
    client_addr_len = sizeof(struct sockaddr_in6);
#else
    client_addr_len = sizeof(struct sockaddr_in);
#endif
    if (type == SOCK_STREAM)
    {
        ret = client_fd = accept(bind_fd, (struct sockaddr*)&client_addr, &client_addr_len);
    }
    else
    {
        //udp
        char buf[1] = {0};
        ret = recvfrom(bind_fd, buf, sizeof(buf), MSG_PEEK, (struct sockaddr*)&client_addr, &client_addr_len);
    }

    if (ret < 0)
        return ATINY_NET_ACCEPT_FAILED;
    if (type != SOCK_STREAM)
    {
#if LWIP_IPV4 && LWIP_IPV6
#elif LWIP_IPV6
        struct sockaddr_in6  local_addr;
        socklen_t n = sizeof(struct sockaddr_in6);
#else
        struct sockaddr_in  local_addr;
        socklen_t n = sizeof(struct sockaddr_in);
#endif
        char port_s[6] = {0};
        int one = 1; 
        
        ((atiny_net_context*)client_ctx)->fd = client_fd = bind_fd;
        ((atiny_net_context*)bind_ctx)->fd = bind_fd = -1;

        if (connect(client_fd, (struct sockaddr *)&client_addr, client_addr_len) != 0)
            return ATINY_NET_ACCEPT_FAILED;

        ret = getsockname(client_fd, (struct sockaddr*)&local_addr, &n);
#if LWIP_IPV4 && LWIP_IPV6
#elif LWIP_IPV6
        snprintf(port_s, sizeof(port_s), "%d", ntohs(local_addr.sin6_port));
        ((atiny_net_context*)bind_ctx)->fd = socket(local_addr.sin6_family, SOCK_DGRAM, IPPROTO_UDP);
#else
        snprintf(port_s, sizeof(port_s), "%d", ntohs(local_addr.sin_port));
        ((atiny_net_context*)bind_ctx)->fd = socket(local_addr.sin_family, SOCK_DGRAM, IPPROTO_UDP);
#endif 
        if( (ret = setsockopt( ((atiny_net_context*)bind_ctx)->fd, SOL_SOCKET, SO_REUSEADDR,
					(const char *) &one, sizeof( one )) ) != 0 )
    	{
    		ret = ATINY_NET_SOCKET_FAILED;
    	}

        if (ret != 0)
            return ret;
    }

    if (client_ip != NULL)
    {
#if LWIP_IPV4 && LWIP_IPV6
#elif LWIP_IPV6
        if( client_addr.sin6_family == AF_INET6 )
        {
            struct sockaddr_in6 *addr = (struct sockaddr_in6 *) &client_addr;
            *ip_len = sizeof( addr->sin6_addr.s6_addr );

            if( buf_size < *ip_len )
                return( ATINY_NET_BUF_SMALL_FAILED );

            memcpy( client_ip, &addr->sin6_addr.s6_addr, *ip_len );
        }
#else
        if( client_addr.sin_family == AF_INET )
        {
            struct sockaddr_in *addr = (struct sockaddr_in *) &client_addr;
            *ip_len = sizeof( addr->sin_addr.s_addr );

            if( buf_size < *ip_len )
                return( ATINY_NET_BUF_SMALL_FAILED );

            memcpy( client_ip, &addr->sin_addr.s_addr, *ip_len );
        }
#endif
    }
#else
    ((atiny_net_context*)client_ctx)->fd = ((atiny_net_context*)bind_ctx)->fd;
#endif
    return 0;
}

void *atiny_net_connect(const char *host, const char *port, int proto)
{
    atiny_net_context *ctx = NULL;
#if defined(WITH_LINUX) || defined(WITH_LWIP)
    int flags;
    int ret;
    struct addrinfo hints;
    struct addrinfo *addr_list;
    struct addrinfo *cur;
#endif

    //if (NULL == host || NULL == port ||
    if (NULL == port ||
            (proto != ATINY_PROTO_UDP && proto != ATINY_PROTO_TCP))
    {
        SOCKET_LOG("ilegal incoming parameters,(%p,%p,%d)",host,port,proto);
        return NULL;
    }

#if defined(WITH_LINUX) || defined(WITH_LWIP)

    /* Do name resolution with both IPv6 and IPv4 */
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = proto == ATINY_PROTO_UDP ? SOCK_DGRAM : SOCK_STREAM;
    hints.ai_protocol = proto == ATINY_PROTO_UDP ? IPPROTO_UDP : IPPROTO_TCP;

    SOCKET_LOG("try to do name resolution now...");

    if ((ret = getaddrinfo(host, port, &hints, &addr_list)) != 0)
    {
        SOCKET_LOG("getaddrinfo failed: 0x%x", ret);
        return NULL;
    }

    SOCKET_LOG("do name resolution succeed");

    ctx = atiny_malloc(sizeof(atiny_net_context));

    if (NULL == ctx)
    {
        SOCKET_LOG("malloc failed for socket context");
        freeaddrinfo(addr_list);
        return NULL;
    }

    ctx->fd = -1;

    /* Try the sockaddrs until a connection succeeds */
    for (cur = addr_list; cur != NULL; cur = cur->ai_next)
    {
        ctx->fd = socket(cur->ai_family, cur->ai_socktype, cur->ai_protocol);

        if (ctx->fd < 0)
        {
            continue;
        }

        if (proto == ATINY_PROTO_UDP)
        {
            flags = fcntl(ctx->fd, F_GETFL, 0);

            if (flags < 0 || fcntl(ctx->fd, F_SETFL, flags | O_NONBLOCK) < 0)
            {
                close(ctx->fd);
                ctx->fd = -1;
                continue;
            }
        }

        if (connect(ctx->fd, cur->ai_addr, cur->ai_addrlen) == 0)
        {
            break;
        }

        close(ctx->fd);
        ctx->fd = -1;
    }

    freeaddrinfo(addr_list);

    if (ctx->fd < 0)
    {
        SOCKET_LOG("unkown host or port");
        atiny_free(ctx);
        return NULL;
    }

    if (proto == ATINY_PROTO_UDP)
    {
        SOCKET_LOG("UDP create socket and bind to server finished");
    }
    else /* proto == ATINY_PROTO_TCP */
    {
        SOCKET_LOG("TCP connect to server succeed");
    }
#elif defined(WITH_AT_FRAMEWORK)
    ctx = atiny_malloc(sizeof(atiny_net_context));
    if (NULL == ctx)
    {
        SOCKET_LOG("malloc failed for socket context");
        return NULL;
    }

    ctx->fd = at_api_connect(host, port, proto);
    if (ctx->fd < 0)
    {
        SOCKET_LOG("unkown host or port");
        atiny_free(ctx);
        ctx = NULL;
    }
#elif defined(WITH_WIZNET)
    ctx = atiny_malloc(sizeof(atiny_net_context));
    if (NULL == ctx)
    {
        SOCKET_LOG("malloc failed for socket context");
        return NULL;
    }

    ctx->fd = wiznet_connect(host, port, proto);
    if (ctx->fd < 0)
    {
        SOCKET_LOG("unkown host or port");
        atiny_free(ctx);
        ctx = NULL;
    }
#else
#endif
    return ctx;
}

int atiny_net_recv(void *ctx, unsigned char *buf, size_t len)
{
    int ret = -1;
    int fd = ((atiny_net_context *)ctx)->fd;
#if defined(WITH_LINUX) || defined(WITH_LWIP)
    ret = recv(fd, buf, len, 0);
#elif defined(WITH_AT_FRAMEWORK)
    ret = at_api_recv(fd, buf, len);
#elif defined(WITH_WIZNET)
    ret = wiznet_recv(fd, buf, len);
#else
    (void)fd; //clear unuse warning
#endif

#if defined(WITH_LINUX) || defined(WITH_LWIP)
    if (ret < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
        {
            SOCKET_LOG("no data available for now");
            return 0;
        }
        else
        {
            SOCKET_LOG("error accured when recv: 0x%x", errno);
            return -1;
        }
    }

    else if (ret == 0)
    {
        SOCKET_LOG("socket was closed by peer");
        return -1;
    }
#endif
    return ret;
}

int atiny_net_recv_timeout(void *ctx, unsigned char *buf, size_t len,
                           uint32_t timeout)
{
    int ret = -1;
#if defined(WITH_LINUX) || defined(WITH_LWIP)
    struct timeval tv;
    fd_set read_fds;
#endif

    int fd = ((atiny_net_context *)ctx)->fd;

#if defined(WITH_LINUX) || defined(WITH_LWIP)
    if (fd < 0)
    {
        SOCKET_LOG("ilegal socket(%d)", fd);
        return ATINY_NET_ERR;
    }

    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);

    tv.tv_sec  = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;

   ret = select(fd + 1, &read_fds, NULL, NULL, &tv);

    if (ret == 0)
    {
       // SOCKET_LOG("recv timeout");
        return ATINY_NET_TIMEOUT;
    }

    if(ret < 0)
    {
        SOCKET_LOG("select error ret=%d,err 0x%x", ret, errno);
        return ATINY_NET_ERR;
    }

    ret = atiny_net_recv(ctx, buf, len);

#elif defined(WITH_AT_FRAMEWORK)
    ret = at_api_recv_timeout(fd, buf, len, NULL,NULL,timeout);
#elif defined(WITH_WIZNET)
    ret = wiznet_recv_timeout(fd, buf, len, timeout);
#else
    (void)fd; //clear unuse warning
#endif
    return ret;
}

int atiny_net_send(void *ctx, const unsigned char *buf, size_t len)
{
    int ret = -1;
    int fd = ((atiny_net_context *)ctx)->fd;

    if (fd < 0)
    {
        SOCKET_LOG("ilegal socket(%d)", fd);
        return -1;
    }

#if defined(WITH_LINUX) || defined(WITH_LWIP)
    ret = send(fd, buf, len, 0);
#elif defined(WITH_AT_FRAMEWORK)
    ret = at_api_send(fd, buf, len);
#elif defined(WITH_WIZNET)
    ret = wiznet_send(fd, buf, len);
#else
#endif

#if defined(WITH_LINUX) || defined(WITH_LWIP)
    if (ret < 0)
    {
        /* no data available for now */
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
        {
            return 0;
        }
        else
        {
            SOCKET_LOG("error accured when send: 0x%x", errno);
            return -1;
        }
    }
#endif

    return ret;
}

void atiny_net_close(void *ctx)
{
    int fd = ((atiny_net_context *)ctx)->fd;

    if (fd >= 0)
    {
#if defined(WITH_LINUX) || defined(WITH_LWIP)
        close(fd);
#elif defined(WITH_AT_FRAMEWORK)
        at_api_close(fd);
#elif defined(WITH_WIZNET)
        wiznet_close(fd);
#endif
    }

    atiny_free(ctx);
}

#if defined(WITH_LINUX) || defined(WITH_LWIP)
static int atiny_net_write_sock(void *ctx, const unsigned char *buffer, int len, uint32_t timeout_ms)
{
    int fd;
    struct timeval interval = {timeout_ms / 1000, (timeout_ms % 1000) * 1000};

    fd = ((atiny_net_context *)ctx)->fd;
    if (interval.tv_sec < 0 || (interval.tv_sec == 0 && interval.tv_usec <= 0))
    {
        interval.tv_sec = 0;
        interval.tv_usec = 100;
    }

    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (char *)&interval, sizeof(struct timeval));
    return write(fd, buffer, len);
}
#endif

int atiny_net_send_timeout(void *ctx, const unsigned char *buf, size_t len,
                          uint32_t timeout)
{
#if defined(WITH_LINUX) || defined(WITH_LWIP)
    return atiny_net_write_sock(ctx, buf, len, timeout);
#elif defined(WITH_AT_FRAMEWORK)
        int fd;
        fd = ((atiny_net_context *)ctx)->fd;
        return at_api_send(fd , buf, (uint32_t)len);
#endif

}

