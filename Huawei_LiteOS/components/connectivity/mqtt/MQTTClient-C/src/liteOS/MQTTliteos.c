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
#include "MQTTliteos.h"

#include "mbedtls/net_sockets.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/platform.h"
#include "dtls_interface.h"

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
//#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/errno.h"
#endif
#include "osdepends/atiny_osdep.h"


#define get_time_ms atiny_gettime_ms

void TimerInit(Timer *timer)
{
    timer->end_time = get_time_ms();
}

char TimerIsExpired(Timer *timer)
{
    unsigned long long now = get_time_ms();
    return now >= timer->end_time;
}

void TimerCountdownMS(Timer *timer, unsigned int timeout)
{
    unsigned long long now = get_time_ms();
    timer->end_time = now + timeout;
}

void TimerCountdown(Timer *timer, unsigned int timeout)
{
    unsigned long long now = get_time_ms();
    timer->end_time = now + timeout * 1000;
}

int TimerLeftMS(Timer *timer)
{
    UINT64 now = get_time_ms();
    return timer->end_time <= now ? 0 : timer->end_time - now;
}

int MutexInit(Mutex* mutex)
{
    int ret = atiny_task_mutex_create(mutex);
    if (ret != LOS_OK)
    {
        ATINY_LOG(LOG_ERR, "create mutex fail, ret %d.", ret);
    }
    return ret;
}
int MutexLock(Mutex* mutex)
{
    int ret = atiny_task_mutex_lock(mutex);
    if (ret != LOS_OK)
    {
        ATINY_LOG(LOG_ERR, "lock mutex fail,mutex %d,ret %d.", mutex->mutex, ret);
    }
    return ret;
}

int MutexUnlock(Mutex* mutex)
{
    int ret = atiny_task_mutex_unlock(mutex);
    if (ret != LOS_OK)
    {
        ATINY_LOG(LOG_ERR, "unlock mutex fail,mutex %d,ret %d.", mutex->mutex, ret);
    }
    return ret;
}

void MutexDestory(Mutex* mutex)
{
    int ret = atiny_task_mutex_delete(mutex);
    if (ret != LOS_OK)
    {
        ATINY_LOG(LOG_ERR, "delete mutex fail,mutex %d,ret %d.", mutex->mutex, ret);
    }
}

int ThreadStart(Thread *thread, void (*fn)(void *), void *arg)
{
    (void)thread;
    (void)fn;
    (void)arg;
    return -1;
}

static int los_mqtt_read(void *ctx, unsigned char *buffer, int len, int timeout_ms)
{
    int ret = atiny_net_recv_timeout(ctx, buffer, len, timeout_ms);
    /* 0 is timeout for mqtt for normal select */
    if (ret == ATINY_NET_TIMEOUT)
    {
        ret = 0;
    }
    return ret;
}

static int los_mqtt_tls_read(mbedtls_ssl_context *ssl, unsigned char *buffer, int len, int timeout_ms)
{
    int ret;

    if(NULL == ssl || NULL == buffer)
    {
        ATINY_LOG(LOG_FATAL, "invalid params.");
        return -1;
    }

    mbedtls_ssl_conf_read_timeout(ssl->conf, timeout_ms);

    ret = mbedtls_ssl_read(ssl, buffer, len);
    if(ret == MBEDTLS_ERR_SSL_WANT_READ
            || ret == MBEDTLS_ERR_SSL_WANT_WRITE
            || ret == MBEDTLS_ERR_SSL_TIMEOUT)
        ret = 0;

    return ret;
}


static int los_read(Network *n, unsigned char *buffer, int len, int timeout_ms)
{
    int ret = -1;
    mqtt_security_info_s *info;

    if(NULL == n || NULL == buffer)
    {
        ATINY_LOG(LOG_FATAL, "invalid params.");
        return -1;
    }

    info = n->get_security_info();

    switch(info->security_type)
    {
        case MQTT_SECURITY_TYPE_NONE :
            ret = los_mqtt_read(n->ctx, buffer, len, timeout_ms);
            break;
        case MQTT_SECURITY_TYPE_PSK:
        case MQTT_SECURITY_TYPE_CA:
            ret = los_mqtt_tls_read(n->ctx, buffer, len, timeout_ms);
            break;
        default :
            ATINY_LOG(LOG_WARNING, "unknow proto : %d", info->security_type);
            break;
    }

    return ret;
}


static int los_write(Network *n, unsigned char *buffer, int len, int timeout_ms)
{
    int ret = -1;
    mqtt_security_info_s *info;

    if(NULL == n || NULL == buffer)
    {
        ATINY_LOG(LOG_FATAL, "invalid params.");
        return -1;
    }

    info = n->get_security_info();

    switch(info->security_type)
    {
    case MQTT_SECURITY_TYPE_NONE :
        ret = atiny_net_send_timeout(n->ctx, buffer, len, timeout_ms);
        break;
    case MQTT_SECURITY_TYPE_PSK:
    case MQTT_SECURITY_TYPE_CA:
        ret = dtls_write(n->ctx, buffer, len);
        break;
    default :
        ATINY_LOG(LOG_WARNING, "unknow proto : %d", info->security_type);
        break;
    }

    return ret;
}

void NetworkInit(Network *n, mqtt_security_info_s *(*get_security_info)(void))
{
    if((NULL == n) ||
       (NULL == get_security_info))
    {
        ATINY_LOG(LOG_FATAL, "invalid params.");
        return;
    }
    memset(n, 0x0, sizeof(Network));
    n->mqttread = los_read;
    n->mqttwrite = los_write;
    n->get_security_info = get_security_info;

    return;
}

static int los_mqtt_connect(Network *n, char *addr, int port)
{
    char port_str[16];

    (void)snprintf(port_str, sizeof(port_str), "%u", port);
    port_str[sizeof(port_str) - 1] = '\0';
    n->ctx = atiny_net_connect(addr, port_str, ATINY_PROTO_TCP);
    if (n->ctx == NULL)
    {
        ATINY_LOG(LOG_FATAL, "atiny_net_connect fail");
        return ATINY_NET_ERR;
    }

    return ATINY_OK;
}

#define PORT_BUF_LEN 16
static int los_mqtt_tls_connect(Network *n, char *addr, int port)
{
    int ret;
    mbedtls_ssl_context *ssl;
    dtls_shakehand_info_s shakehand_info;
    dtls_establish_info_s establish_info;
    mqtt_security_info_s *security_info;
    char port_buf[PORT_BUF_LEN];

    security_info = n->get_security_info();

    if (security_info->security_type == MQTT_SECURITY_TYPE_PSK)
    {
        establish_info.psk_or_cert = VERIFY_WITH_PSK;
        establish_info.v.p.psk = (unsigned char *)security_info->u.psk.psk;
        establish_info.v.p.psk_len = security_info->u.psk.psk_len;
        establish_info.v.p.psk_identity = (unsigned char *)security_info->u.psk.psk_id;
    }
    else
    {
        establish_info.psk_or_cert = VERIFY_WITH_CERT;
        establish_info.v.c.ca_cert = (const unsigned char *)security_info->u.ca.ca_crt;
        establish_info.v.c.cert_len = security_info->u.ca.ca_len;
    }
    establish_info.udp_or_tcp = MBEDTLS_NET_PROTO_TCP;

    ssl = (void *)dtls_ssl_new(&establish_info, MBEDTLS_SSL_IS_CLIENT);
    if (NULL == ssl)
    {
        ATINY_LOG(LOG_ERR, "create ssl context failed");
        goto exit;
    }

    memset(&shakehand_info, 0, sizeof(dtls_shakehand_info_s));

    if (security_info->security_type == MQTT_SECURITY_TYPE_PSK)
    {
        shakehand_info.psk_or_cert = VERIFY_WITH_PSK;
    }
    else
    {
        shakehand_info.psk_or_cert = VERIFY_WITH_CERT;
    }
    shakehand_info.client_or_server = MBEDTLS_SSL_IS_CLIENT;
    shakehand_info.udp_or_tcp = MBEDTLS_NET_PROTO_TCP;
    shakehand_info.u.c.host = addr;
    atiny_snprintf(port_buf, PORT_BUF_LEN, "%d", port);
    shakehand_info.u.c.port = port_buf;

    ret = dtls_shakehand(ssl, &shakehand_info);
    if (ret != 0)
    {
        ATINY_LOG(LOG_ERR, "ssl shake hand failed");
        goto exit;
    }

    n->ctx = ssl;
    return 0;
exit:
    dtls_ssl_destroy(ssl);
    return -1;
}

int NetworkConnect(Network *n, char *addr, int port)
{
    int ret = -1;
    mqtt_security_info_s *info;

    if(NULL == n || NULL == addr)
    {
        ATINY_LOG(LOG_FATAL, "invalid params.\n");
        return -1;
    }

    info = n->get_security_info();
    switch(info->security_type)
    {
    case MQTT_SECURITY_TYPE_NONE :
        ret = los_mqtt_connect(n, addr, port);
        break;
    case MQTT_SECURITY_TYPE_PSK:
    case MQTT_SECURITY_TYPE_CA:
        ret = los_mqtt_tls_connect(n, addr, port);
        break;
    default :
        ATINY_LOG(LOG_WARNING, "unknow proto : %d\n", info->security_type);
        break;
    }

    return ret;
}

static void los_mqtt_disconnect(void *ctx)
{
    if(NULL == ctx)
    {
        ATINY_LOG(LOG_FATAL, "invalid params.\n");
        return;
    }
    atiny_net_close(ctx);

    return;
}

void NetworkDisconnect(Network *n)
{
    mqtt_security_info_s *info;
    if(NULL == n)
    {
        ATINY_LOG(LOG_FATAL, "invalid params.\n");
        return;
    }

    info = n->get_security_info();
    switch(info->security_type)
    {
    case MQTT_SECURITY_TYPE_NONE :
        los_mqtt_disconnect(n->ctx);
        break;
    case MQTT_SECURITY_TYPE_PSK:
    case MQTT_SECURITY_TYPE_CA:
        dtls_ssl_destroy(n->ctx);
        break;
    default :
        ATINY_LOG(LOG_WARNING, "unknow proto : %d\n", info->security_type);
        break;
    }

    n->ctx = NULL;

    return;
}

