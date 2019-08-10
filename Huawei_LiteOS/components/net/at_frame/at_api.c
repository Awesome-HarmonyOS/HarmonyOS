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

#if defined(WITH_AT_FRAMEWORK)
#include "at_frame/at_api.h"

static at_adaptor_api  *gp_at_adaptor_api = NULL;

int32_t at_api_register(at_adaptor_api *api)
{
    if (NULL == gp_at_adaptor_api)
    {
        gp_at_adaptor_api = api;
        if (gp_at_adaptor_api && gp_at_adaptor_api->init)
        {
            return gp_at_adaptor_api->init();
        }
    }

    return 0;
}

int32_t at_api_bind(const char *host, const char *port, int proto)
{
    int32_t ret = -1;

    if (gp_at_adaptor_api && gp_at_adaptor_api->bind)
    {
        ret = gp_at_adaptor_api->bind((int8_t *)host, (int8_t *)port, proto);
    }
    return ret;
}

int32_t at_api_connect(const char *host, const char *port, int proto)
{
    int32_t ret = -1;

    if (gp_at_adaptor_api && gp_at_adaptor_api->connect)
    {
        ret = gp_at_adaptor_api->connect((int8_t *)host, (int8_t *)port, proto);
    }
    return ret;
}

int32_t at_api_send(int32_t id , const unsigned char *buf, uint32_t len)
{
    if (gp_at_adaptor_api && gp_at_adaptor_api->send)
    {
        return gp_at_adaptor_api->send(id, buf, len);
    }
    return -1;
}

int32_t at_api_sendto(int32_t id , uint8_t  *buf, uint32_t len,char* ipaddr,int port)
{
    if (gp_at_adaptor_api && gp_at_adaptor_api->sendto)
    {
        return gp_at_adaptor_api->sendto(id, buf, len,ipaddr, port);
    }
    return -1;
}

int32_t at_api_recv(int32_t id, unsigned char *buf, size_t len)
{
    if (gp_at_adaptor_api && gp_at_adaptor_api->recv)
    {
        return gp_at_adaptor_api->recv(id, buf, len);
    }
    return -1;
}

int32_t at_api_recv_timeout(int32_t id , uint8_t  *buf, uint32_t len,char* ipaddr,int* port, int32_t timeout)
{
    if (gp_at_adaptor_api && gp_at_adaptor_api->recv_timeout)
    {
        return gp_at_adaptor_api->recv_timeout(id , buf, len, ipaddr, port, timeout);
    }
    return -1;
}

int32_t at_api_close(int32_t fd)
{
    if (gp_at_adaptor_api && gp_at_adaptor_api->close)
    {
        return gp_at_adaptor_api->close(fd);
    }
    return -1;
}

#endif
