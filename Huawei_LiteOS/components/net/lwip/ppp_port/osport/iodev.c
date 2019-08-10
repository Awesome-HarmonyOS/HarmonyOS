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
#include "osport.h"


typedef struct
{
    u32_t  debugrxmode: 2;  //1means ascii 2 hex while others means no debug
    u32_t  debugtxmode: 2;  //1means ascii 2 hex while others means no debug
} tagIOCB;

static tagIOCB  gIOCB;
//import the uart here
extern s32_t uart_read(u8_t *buf, s32_t len, s32_t timeout);
extern s32_t uart_write(u8_t *buf, s32_t len, s32_t timeout);


#pragma weak uart_read
s32_t uart_read(u8_t *buf, s32_t len, s32_t timeout)
{
    return 0;
}
#pragma weak uart_write
s32_t uart_write(u8_t *buf, s32_t len, s32_t timeout)
{
    return 0;
}


//we do some port here:we port the uart
s32_t iodev_open(const char *name, s32_t flags, s32_t mode)
{
    s32_t fd = 1;
    return fd;
}

s32_t iodev_read(s32_t fd, u8_t *buf, s32_t len, s32_t timeout)
{
    s32_t i = 0;
    s32_t ret;
    ret = uart_read(buf, len, timeout);
    if(ret > 0)
    {
        printf("RCV:%02x Bytes:", ret);
        for(i = 0; i < ret; i++)
        {
            if(gIOCB.debugrxmode == 1)
            {
                printf("%c", buf[i]);
            }
            else if(gIOCB.debugrxmode  == 2)
            {
                printf(" %02x", buf[i]);
            }
            else
            {

            }
        }
        printf("\n\r");
    }
    return ret;
}
s32_t iodev_write(s32_t fd, u8_t *buf, s32_t len, s32_t timeout)
{
    s32_t ret;
    s32_t i;
    printf("SND:%02x Bytes:", len);
    for(i = 0; i < len; i++)
    {
        if(gIOCB.debugtxmode == 1)
        {
            printf("%c", buf[i]);
        }
        else if(gIOCB.debugtxmode == 2)
        {
            printf(" %02x", buf[i]);
        }
        else
        {

        }
    }
    printf("\n\r");
    ret = uart_write(buf, len, timeout);
    return ret;
}
s32_t iodev_close(s32_t fd)
{
    return 0;
}
s32_t iodev_flush(s32_t fd)
{
    unsigned char buf;
    s32_t ret;
    do
    {
        ret = iodev_read(fd, &buf, 1, 0);
    }
    while(ret > 0);
    return 0;
}
void  iodev_debugmode(s32_t rxtx, u32_t mode)
{
    if(rxtx == 0)
    {
        gIOCB.debugrxmode = mode;
    }
    else if(rxtx == 1)
    {
        gIOCB.debugtxmode = mode;
    }
    else if(rxtx == 2)
    {
        gIOCB.debugrxmode = mode;
        gIOCB.debugtxmode = mode;
    }
    else  //do nothing here
    {
    }
    return;
}

