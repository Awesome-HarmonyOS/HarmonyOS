/*
 *  UDP proxy: emulate an unreliable UDP connexion for DTLS testing
 *
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */

/*
 * Warning: this is an internal utility program we use for tests.
 * It does break some abstractions from the NET layer, and is thus NOT an
 * example of good general usage.
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#define mbedtls_time       time
#define mbedtls_time_t     time_t
#define mbedtls_printf     printf
#endif

#if !defined(MBEDTLS_NET_C)
int main( void )
{
    mbedtls_printf( "MBEDTLS_NET_C not defined.\n" );
    return( 0 );
}
#else

#include "mbedtls/net_sockets.h"
#include "mbedtls/error.h"
#include "mbedtls/ssl.h"

#include <string.h>

/* For select() */
#if (defined(_WIN32) || defined(_WIN32_WCE)) && !defined(EFIX64) && \
    !defined(EFI32)
#include <winsock2.h>
#include <windows.h>
#if defined(_MSC_VER)
#if defined(_WIN32_WCE)
#pragma comment( lib, "ws2.lib" )
#else
#pragma comment( lib, "ws2_32.lib" )
#endif
#endif /* _MSC_VER */
#else /* ( _WIN32 || _WIN32_WCE ) && !EFIX64 && !EFI32 */
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#endif /* ( _WIN32 || _WIN32_WCE ) && !EFIX64 && !EFI32 */

/* For gettimeofday() */
#if !defined(_WIN32)
#include <sys/time.h>
#endif

#define MAX_MSG_SIZE            16384 + 2048 /* max record/datagram size */

#define DFL_SERVER_ADDR         "localhost"
#define DFL_SERVER_PORT         "4433"
#define DFL_LISTEN_ADDR         "localhost"
#define DFL_LISTEN_PORT         "5556"

#define USAGE                                                               \
    "\n usage: udp_proxy param=<>...\n"                                     \
    "\n acceptable parameters:\n"                                           \
    "    server_addr=%%s      default: localhost\n"                         \
    "    server_port=%%d      default: 4433\n"                              \
    "    listen_addr=%%s      default: localhost\n"                         \
    "    listen_port=%%d      default: 4433\n"                              \
    "\n"                                                                    \
    "    duplicate=%%d        default: 0 (no duplication)\n"                \
    "                        duplicate about 1:N packets randomly\n"        \
    "    delay=%%d            default: 0 (no delayed packets)\n"            \
    "                        delay about 1:N packets randomly\n"            \
    "    delay_ccs=0/1       default: 0 (don't delay ChangeCipherSpec)\n"   \
    "    drop=%%d             default: 0 (no dropped packets)\n"            \
    "                        drop about 1:N packets randomly\n"             \
    "    mtu=%%d              default: 0 (unlimited)\n"                     \
    "                        drop packets larger than N bytes\n"            \
    "    bad_ad=0/1          default: 0 (don't add bad ApplicationData)\n"  \
    "    protect_hvr=0/1     default: 0 (don't protect HelloVerifyRequest)\n" \
    "    protect_len=%%d     default: (don't protect packets of this size)\n" \
    "\n"                                                                    \
    "    seed=%%d             default: (use current time)\n"                \
    "\n"

/*
 * global options
 */
static struct options
{
    const char *server_addr;    /* address to forward packets to            */
    const char *server_port;    /* port to forward packets to               */
    const char *listen_addr;    /* address for accepting client connections */
    const char *listen_port;    /* port for accepting client connections    */

    int duplicate;              /* duplicate 1 in N packets (none if 0)     */
    int delay;                  /* delay 1 packet in N (none if 0)          */
    int delay_ccs;              /* delay ChangeCipherSpec                   */
    int drop;                   /* drop 1 packet in N (none if 0)           */
    int mtu;                    /* drop packets larger than this            */
    int bad_ad;                 /* inject corrupted ApplicationData record  */
    int protect_hvr;            /* never drop or delay HelloVerifyRequest   */
    int protect_len;            /* never drop/delay packet of the given size*/

    unsigned int seed;          /* seed for "random" events                 */
} opt;

static void exit_usage( const char *name, const char *value )
{
    if( value == NULL )
        mbedtls_printf( " unknown option or missing value: %s\n", name );
    else
        mbedtls_printf( " option %s: illegal value: %s\n", name, value );

    mbedtls_printf( USAGE );
    exit( 1 );
}

static void get_options( int argc, char *argv[] )
{
    int i;
    char *p, *q;

    opt.server_addr    = DFL_SERVER_ADDR;
    opt.server_port    = DFL_SERVER_PORT;
    opt.listen_addr    = DFL_LISTEN_ADDR;
    opt.listen_port    = DFL_LISTEN_PORT;
    /* Other members default to 0 */

    for( i = 1; i < argc; i++ )
    {
        p = argv[i];
        if( ( q = strchr( p, '=' ) ) == NULL )
            exit_usage( p, NULL );
        *q++ = '\0';

        if( strcmp( p, "server_addr" ) == 0 )
            opt.server_addr = q;
        else if( strcmp( p, "server_port" ) == 0 )
            opt.server_port = q;
        else if( strcmp( p, "listen_addr" ) == 0 )
            opt.listen_addr = q;
        else if( strcmp( p, "listen_port" ) == 0 )
            opt.listen_port = q;
        else if( strcmp( p, "duplicate" ) == 0 )
        {
            opt.duplicate = atoi( q );
            if( opt.duplicate < 0 || opt.duplicate > 20 )
                exit_usage( p, q );
        }
        else if( strcmp( p, "delay" ) == 0 )
        {
            opt.delay = atoi( q );
            if( opt.delay < 0 || opt.delay > 20 || opt.delay == 1 )
                exit_usage( p, q );
        }
        else if( strcmp( p, "delay_ccs" ) == 0 )
        {
            opt.delay_ccs = atoi( q );
            if( opt.delay_ccs < 0 || opt.delay_ccs > 1 )
                exit_usage( p, q );
        }
        else if( strcmp( p, "drop" ) == 0 )
        {
            opt.drop = atoi( q );
            if( opt.drop < 0 || opt.drop > 20 || opt.drop == 1 )
                exit_usage( p, q );
        }
        else if( strcmp( p, "mtu" ) == 0 )
        {
            opt.mtu = atoi( q );
            if( opt.mtu < 0 || opt.mtu > MAX_MSG_SIZE )
                exit_usage( p, q );
        }
        else if( strcmp( p, "bad_ad" ) == 0 )
        {
            opt.bad_ad = atoi( q );
            if( opt.bad_ad < 0 || opt.bad_ad > 1 )
                exit_usage( p, q );
        }
        else if( strcmp( p, "protect_hvr" ) == 0 )
        {
            opt.protect_hvr = atoi( q );
            if( opt.protect_hvr < 0 || opt.protect_hvr > 1 )
                exit_usage( p, q );
        }
        else if( strcmp( p, "protect_len" ) == 0 )
        {
            opt.protect_len = atoi( q );
            if( opt.protect_len < 0 )
                exit_usage( p, q );
        }
        else if( strcmp( p, "seed" ) == 0 )
        {
            opt.seed = atoi( q );
            if( opt.seed == 0 )
                exit_usage( p, q );
        }
        else
            exit_usage( p, NULL );
    }
}

static const char *msg_type( unsigned char *msg, size_t len )
{
    if( len < 1 )                           return( "Invalid" );
    switch( msg[0] )
    {
        case MBEDTLS_SSL_MSG_CHANGE_CIPHER_SPEC:    return( "ChangeCipherSpec" );
        case MBEDTLS_SSL_MSG_ALERT:                 return( "Alert" );
        case MBEDTLS_SSL_MSG_APPLICATION_DATA:      return( "ApplicationData" );
        case MBEDTLS_SSL_MSG_HANDSHAKE:             break; /* See below */
        default:                            return( "Unknown" );
    }

    if( len < 13 + 12 )                     return( "Invalid handshake" );

    /*
     * Our handshake message are less than 2^16 bytes long, so they should
     * have 0 as the first byte of length, frag_offset and frag_length.
     * Otherwise, assume they are encrypted.
     */
    if( msg[14] || msg[19] || msg[22] )     return( "Encrypted handshake" );

    switch( msg[13] )
    {
        case MBEDTLS_SSL_HS_HELLO_REQUEST:          return( "HelloRequest" );
        case MBEDTLS_SSL_HS_CLIENT_HELLO:           return( "ClientHello" );
        case MBEDTLS_SSL_HS_SERVER_HELLO:           return( "ServerHello" );
        case MBEDTLS_SSL_HS_HELLO_VERIFY_REQUEST:   return( "HelloVerifyRequest" );
        case MBEDTLS_SSL_HS_NEW_SESSION_TICKET:     return( "NewSessionTicket" );
        case MBEDTLS_SSL_HS_CERTIFICATE:            return( "Certificate" );
        case MBEDTLS_SSL_HS_SERVER_KEY_EXCHANGE:    return( "ServerKeyExchange" );
        case MBEDTLS_SSL_HS_CERTIFICATE_REQUEST:    return( "CertificateRequest" );
        case MBEDTLS_SSL_HS_SERVER_HELLO_DONE:      return( "ServerHelloDone" );
        case MBEDTLS_SSL_HS_CERTIFICATE_VERIFY:     return( "CertificateVerify" );
        case MBEDTLS_SSL_HS_CLIENT_KEY_EXCHANGE:    return( "ClientKeyExchange" );
        case MBEDTLS_SSL_HS_FINISHED:               return( "Finished" );
        default:                            return( "Unknown handshake" );
    }
}

/* Return elapsed time in milliseconds since the first call */
static unsigned long ellapsed_time( void )
{
#if defined(_WIN32)
    return( 0 );
#else
    static struct timeval ref = { 0, 0 };
    struct timeval now;

    if( ref.tv_sec == 0 && ref.tv_usec == 0 )
    {
        gettimeofday( &ref, NULL );
        return( 0 );
    }

    gettimeofday( &now, NULL );
    return( 1000 * ( now.tv_sec  - ref.tv_sec )
                 + ( now.tv_usec - ref.tv_usec ) / 1000 );
#endif
}

typedef struct
{
    mbedtls_net_context *dst;
    const char *way;
    const char *type;
    unsigned len;
    unsigned char buf[MAX_MSG_SIZE];
} packet;

/* Print packet. Outgoing packets come with a reason (forward, dupl, etc.) */
void print_packet( const packet *p, const char *why )
{
    if( why == NULL )
        mbedtls_printf( "  %05lu %s %s (%u bytes)\n",
                ellapsed_time(), p->way, p->type, p->len );
    else
        mbedtls_printf( "        %s %s (%u bytes): %s\n",
                p->way, p->type, p->len, why );
    fflush( stdout );
}

int send_packet( const packet *p, const char *why )
{
    int ret;
    mbedtls_net_context *dst = p->dst;

    /* insert corrupted ApplicationData record? */
    if( opt.bad_ad &&
        strcmp( p->type, "ApplicationData" ) == 0 )
    {
        unsigned char buf[MAX_MSG_SIZE];
        memcpy( buf, p->buf, p->len );
        ++buf[p->len - 1];

        print_packet( p, "corrupted" );
        if( ( ret = mbedtls_net_send( dst, buf, p->len ) ) <= 0 )
        {
            mbedtls_printf( "  ! mbedtls_net_send returned %d\n", ret );
            return( ret );
        }
    }

    print_packet( p, why );
    if( ( ret = mbedtls_net_send( dst, p->buf, p->len ) ) <= 0 )
    {
        mbedtls_printf( "  ! mbedtls_net_send returned %d\n", ret );
        return( ret );
    }

    /* Don't duplicate Application Data, only handshake covered */
    if( opt.duplicate != 0 &&
        strcmp( p->type, "ApplicationData" ) != 0 &&
        rand() % opt.duplicate == 0 )
    {
        print_packet( p, "duplicated" );

        if( ( ret = mbedtls_net_send( dst, p->buf, p->len ) ) <= 0 )
        {
            mbedtls_printf( "  ! mbedtls_net_send returned %d\n", ret );
            return( ret );
        }
    }

    return( 0 );
}

static packet prev;

void clear_pending( void )
{
    memset( &prev, 0, sizeof( packet ) );
}

/*
 * Avoid dropping or delaying a packet that was already dropped twice: this
 * only results in uninteresting timeouts. We can't rely on type to identify
 * packets, since during renegotiation they're all encrypted.  So, rely on
 * size mod 2048 (which is usually just size).
 */
static unsigned char dropped[2048] = { 0 };
#define DROP_MAX 2

/*
 * OpenSSL groups packets in a datagram the first time it sends them, but not
 * when it resends them. Count every record as seen the first time.
 */
void update_dropped( const packet *p )
{
    size_t id = p->len % sizeof( dropped );
    const unsigned char *end = p->buf + p->len;
    const unsigned char *cur = p->buf;
    size_t len = ( ( cur[11] << 8 ) | cur[12] ) + 13;

    ++dropped[id];

    /* Avoid counting single record twice */
    if( len == p->len )
        return;

    while( cur < end )
    {
        len = ( ( cur[11] << 8 ) | cur[12] ) + 13;

        id = len % sizeof( dropped );
        ++dropped[id];

        cur += len;
    }
}

int handle_message( const char *way,
                    mbedtls_net_context *dst,
                    mbedtls_net_context *src )
{
    int ret;
    packet cur;
    size_t id;

    /* receive packet */
    if( ( ret = mbedtls_net_recv( src, cur.buf, sizeof( cur.buf ) ) ) <= 0 )
    {
        mbedtls_printf( "  ! mbedtls_net_recv returned %d\n", ret );
        return( ret );
    }

    cur.len  = ret;
    cur.type = msg_type( cur.buf, cur.len );
    cur.way  = way;
    cur.dst  = dst;
    print_packet( &cur, NULL );

    id = cur.len % sizeof( dropped );

    /* do we want to drop, delay, or forward it? */
    if( ( opt.mtu != 0 &&
          cur.len > (unsigned) opt.mtu ) ||
        ( opt.drop != 0 &&
          strcmp( cur.type, "ApplicationData" ) != 0 &&
          ! ( opt.protect_hvr &&
              strcmp( cur.type, "HelloVerifyRequest" ) == 0 ) &&
          cur.len != (size_t) opt.protect_len &&
          dropped[id] < DROP_MAX &&
          rand() % opt.drop == 0 ) )
    {
        update_dropped( &cur );
    }
    else if( ( opt.delay_ccs == 1 &&
               strcmp( cur.type, "ChangeCipherSpec" ) == 0 ) ||
             ( opt.delay != 0 &&
               strcmp( cur.type, "ApplicationData" ) != 0 &&
               ! ( opt.protect_hvr &&
                   strcmp( cur.type, "HelloVerifyRequest" ) == 0 ) &&
               prev.dst == NULL &&
               cur.len != (size_t) opt.protect_len &&
               dropped[id] < DROP_MAX &&
               rand() % opt.delay == 0 ) )
    {
        memcpy( &prev, &cur, sizeof( packet ) );
    }
    else
    {
        /* forward and possibly duplicate */
        if( ( ret = send_packet( &cur, "forwarded" ) ) != 0 )
            return( ret );

        /* send previously delayed message if any */
        if( prev.dst != NULL )
        {
            ret = send_packet( &prev, "delayed" );
            memset( &prev, 0, sizeof( packet ) );
            if( ret != 0 )
                return( ret );
        }
    }

    return( 0 );
}

int main( int argc, char *argv[] )
{
    int ret;

    mbedtls_net_context listen_fd, client_fd, server_fd;

    int nb_fds;
    fd_set read_fds;

    mbedtls_net_init( &listen_fd );
    mbedtls_net_init( &client_fd );
    mbedtls_net_init( &server_fd );

    get_options( argc, argv );

    /*
     * Decisions to drop/delay/duplicate packets are pseudo-random: dropping
     * exactly 1 in N packets would lead to problems when a flight has exactly
     * N packets: the same packet would be dropped on every resend.
     *
     * In order to be able to reproduce problems reliably, the seed may be
     * specified explicitly.
     */
    if( opt.seed == 0 )
    {
        opt.seed = (unsigned int) time( NULL );
        mbedtls_printf( "  . Pseudo-random seed: %u\n", opt.seed );
    }

    srand( opt.seed );

    /*
     * 0. "Connect" to the server
     */
    mbedtls_printf( "  . Connect to server on UDP/%s/%s ...",
            opt.server_addr, opt.server_port );
    fflush( stdout );

    if( ( ret = mbedtls_net_connect( &server_fd, opt.server_addr, opt.server_port,
                             MBEDTLS_NET_PROTO_UDP ) ) != 0 )
    {
        mbedtls_printf( " failed\n  ! mbedtls_net_connect returned %d\n\n", ret );
        goto exit;
    }

    mbedtls_printf( " ok\n" );

    /*
     * 1. Setup the "listening" UDP socket
     */
    mbedtls_printf( "  . Bind on UDP/%s/%s ...",
            opt.listen_addr, opt.listen_port );
    fflush( stdout );

    if( ( ret = mbedtls_net_bind( &listen_fd, opt.listen_addr, opt.listen_port,
                          MBEDTLS_NET_PROTO_UDP ) ) != 0 )
    {
        mbedtls_printf( " failed\n  ! mbedtls_net_bind returned %d\n\n", ret );
        goto exit;
    }

    mbedtls_printf( " ok\n" );

    /*
     * 2. Wait until a client connects
     */
accept:
    mbedtls_net_free( &client_fd );

    mbedtls_printf( "  . Waiting for a remote connection ..." );
    fflush( stdout );

    if( ( ret = mbedtls_net_accept( &listen_fd, &client_fd,
                                    NULL, 0, NULL ) ) != 0 )
    {
        mbedtls_printf( " failed\n  ! mbedtls_net_accept returned %d\n\n", ret );
        goto exit;
    }

    mbedtls_printf( " ok\n" );

    /*
     * 3. Forward packets forever (kill the process to terminate it)
     */
    clear_pending();
    memset( dropped, 0, sizeof( dropped ) );

    nb_fds = client_fd.fd;
    if( nb_fds < server_fd.fd )
        nb_fds = server_fd.fd;
    if( nb_fds < listen_fd.fd )
        nb_fds = listen_fd.fd;
    ++nb_fds;

    while( 1 )
    {
        FD_ZERO( &read_fds );
        FD_SET( server_fd.fd, &read_fds );
        FD_SET( client_fd.fd, &read_fds );
        FD_SET( listen_fd.fd, &read_fds );

        if( ( ret = select( nb_fds, &read_fds, NULL, NULL, NULL ) ) <= 0 )
        {
            perror( "select" );
            goto exit;
        }

        if( FD_ISSET( listen_fd.fd, &read_fds ) )
            goto accept;

        if( FD_ISSET( client_fd.fd, &read_fds ) )
        {
            if( ( ret = handle_message( "S <- C",
                                        &server_fd, &client_fd ) ) != 0 )
                goto accept;
        }

        if( FD_ISSET( server_fd.fd, &read_fds ) )
        {
            if( ( ret = handle_message( "S -> C",
                                        &client_fd, &server_fd ) ) != 0 )
                goto accept;
        }
    }

exit:

#ifdef MBEDTLS_ERROR_C
    if( ret != 0 )
    {
        char error_buf[100];
        mbedtls_strerror( ret, error_buf, 100 );
        mbedtls_printf( "Last error was: -0x%04X - %s\n\n", - ret, error_buf );
        fflush( stdout );
    }
#endif

    mbedtls_net_free( &client_fd );
    mbedtls_net_free( &server_fd );
    mbedtls_net_free( &listen_fd );

#if defined(_WIN32)
    mbedtls_printf( "  Press Enter to exit this program.\n" );
    fflush( stdout ); getchar();
#endif

    return( ret != 0 );
}

#endif /* MBEDTLS_NET_C */
