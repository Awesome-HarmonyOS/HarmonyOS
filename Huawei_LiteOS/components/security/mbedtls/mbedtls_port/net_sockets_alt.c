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

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if !defined(MBEDTLS_NET_C)


#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdlib.h>
#endif

#include "mbedtls/net_sockets.h"

#include "osdepends/atiny_osdep.h"
#include "sal/atiny_socket.h"


void mbedtls_net_init(mbedtls_net_context *ctx)
{
    ctx->fd = -1;
}

void *mbedtls_net_connect(const char *host, const char *port, int proto)
{
    return atiny_net_connect(host, port, proto);
}

void mbedtls_net_usleep(unsigned long usec)
{
    atiny_usleep(usec);
}

int mbedtls_net_recv(void *ctx, unsigned char *buf, size_t len)
{
    int ret = atiny_net_recv(ctx, buf, len);

    if (ret == 0)
    {
        return MBEDTLS_ERR_SSL_WANT_READ;
    }
    else if (ret < 0)
    {
        return MBEDTLS_ERR_NET_RECV_FAILED;
    }

    return ret;
}

int mbedtls_net_recv_timeout(void *ctx, unsigned char *buf, size_t len,
                             uint32_t timeout)
{
    int ret = atiny_net_recv_timeout(ctx, buf, len, timeout);

    if (ret < 0)
    {
        return MBEDTLS_ERR_SSL_TIMEOUT;
    }
    else if (ret == 0)
    {
        return MBEDTLS_ERR_SSL_WANT_READ;
    }

    return ret;
}

int mbedtls_net_send(void *ctx, const unsigned char *buf, size_t len)
{
    int ret = atiny_net_send(ctx, buf, len);

    if (ret == 0)
    {
        return MBEDTLS_ERR_SSL_WANT_WRITE;
    }
    else if (ret < 0)
    {
        return MBEDTLS_ERR_NET_SEND_FAILED;
    }

    return ret;
}

void mbedtls_net_free(mbedtls_net_context *ctx)
{
    atiny_net_close(ctx);
}

int mbedtls_net_accept( mbedtls_net_context *bind_ctx,
                        mbedtls_net_context *client_ctx,
                        void *client_ip, size_t buf_size, size_t *ip_len )
{
    int ret = atiny_net_accept(bind_ctx, client_ctx, client_ip, buf_size, ip_len);

    if (ret == ATINY_NET_ERR)
       return MBEDTLS_ERR_NET_UNKNOWN_HOST;
    else if (ret == ATINY_NET_SOCKET_FAILED)
       return MBEDTLS_ERR_NET_SOCKET_FAILED;
    else if (ret == ATINY_NET_BIND_FAILED)
      return MBEDTLS_ERR_NET_BIND_FAILED;
    else if (ret == ATINY_NET_LISTEN_FAILED)
        return MBEDTLS_ERR_NET_LISTEN_FAILED;
    else if (ret == ATINY_NET_ACCEPT_FAILED)
        return MBEDTLS_ERR_NET_ACCEPT_FAILED;
    else if(ret == ATINY_NET_BUF_SMALL_FAILED)
        return MBEDTLS_ERR_NET_BUFFER_TOO_SMALL;

    return ret;
}
#endif /* MBEDTLS_NET_C */

