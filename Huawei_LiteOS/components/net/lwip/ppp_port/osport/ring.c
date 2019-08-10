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
#include <string.h>
#include "osport.h"
//ring:which to be initialized
//buf:you supplied for the ring
//len:the buf length
//return:0 means ok while -1 failed
s32_t ring_init(tagRingBuf *ring, u8_t *buf, s32_t buflen, s32_t offset, s32_t datalen)
{
    int ret = -1;
    if((NULL == ret))
    {
        return ret;
    }
    ring->buf     = buf;
    ring->buflen  = buflen;
    ring->datalen = datalen;
    ring->dataoff = offset;
    ret = 0;
    return ret;
}
//write len bytes data to the ring
//return:how many bytes has been written while -1 means something err
//write only changes the datalen and  data in  the ring
s32_t ring_write(tagRingBuf *ring, u8_t *buf, s32_t len)
{
    int ret = -1;
    int cpylen;  //the current time we should move
    int lenleft;  //and how many data still left to move
    int offset;
    unsigned char *src;
    unsigned char *dst;
    if((NULL == ring) || (NULL == buf) || (0 == len))
    {
        return ret;//which means parameters error
    }
    if(ring->datalen == ring->buflen)
    {
        ret = 0;
        return  ret;//which means you could copy nothing here
    }
    ret = len > (ring->buflen - ring->datalen) ? (ring->buflen - ring->datalen) : len;
    //now let us think the method to fill the data,take care of the roll back
    lenleft = ret;
    src = buf;
    if((ring->dataoff + ring->datalen) > ring->buflen) //which means the data has roll back
    {
        offset = (ring->dataoff + ring->datalen) % ring->buflen; //we could move it one time
        cpylen = lenleft;
        dst = ring->buf + offset;
        memcpy(dst, src, cpylen);
        ring->datalen += cpylen;
        lenleft -= cpylen;
    }
    else if((ring->dataoff + ring->datalen + lenleft) > ring->buflen) //which means the data will be roll back
    {
        //which means roll back,we should copy some here to the tail
        offset = ring->dataoff + ring->datalen;
        cpylen = ring->buflen - offset;
        dst = ring->buf + offset;
        memcpy(dst, src, cpylen);
        src += cpylen;
        ring->datalen += cpylen;
        lenleft -= cpylen;
    }
    //here means we could move it by one time
    if(lenleft > 0)
    {
        offset = (ring->dataoff + ring->datalen) % ring->buflen; //we could move it one time
        cpylen = lenleft;
        dst = ring->buf + offset;
        memcpy(dst, src, cpylen);
        ring->datalen += cpylen;
    }
    return ret;
}
//read len bytes data from the ring
//return:how many bytes has been read while -1 means something err
//read effect the offset datalen and data in the ring
s32_t ring_read(tagRingBuf *ring, u8_t *buf, s32_t len)
{
    int ret = -1;
    int cpylen;  //the current time we should move
    int lenleft;  //and how many data still left to move
    int offset;
    unsigned char *src;
    unsigned char *dst;
    if((NULL == ring) || (NULL == buf) || (0 == len))
    {
        return ret;//which means parameters error
    }
    if(ring->datalen == 0)
    {
        ret = 0;
        return  ret;//which means you could copy nothing here
    }
    ret = len > ring->datalen ? ring->datalen : len;
    //now let us think the method to fill the data,take care of the roll back
    lenleft = ret;
    dst = buf;
    if((ring->dataoff + lenleft) >= ring->buflen) //which means the data has roll back
    {
        offset = ring->dataoff; //we cpy part
        cpylen = ring->buflen - ring->dataoff;
        src = ring->buf + offset;
        memcpy(dst, src, cpylen);
        ring->dataoff = (ring->dataoff + cpylen) % ring->buflen;
        ring->datalen -= cpylen;
        lenleft -= cpylen;
        dst += cpylen;
    }
    //here means we could move it by one time
    if(lenleft > 0)
    {
        offset = ring->dataoff; //we cpy part
        cpylen = lenleft;
        src = ring->buf + offset;
        memcpy(dst, src, cpylen);
        ring->dataoff = ring->dataoff + cpylen;
        ring->datalen -= cpylen;
    }
    return ret;
}
//check how many data in the ring while -1 means someting error
s32_t ring_datalen(tagRingBuf *ring)
{
    int ret = -1;
    if(NULL != ring)
    {
        ret = ring->datalen;
    }
    return ret;
}
//reset the ring
s32_t ring_reset(tagRingBuf *ring)
{
    int ret = -1;
    if(NULL != ring)
    {
        ring->datalen = 0;
        ring->dataoff = 0;
        ret = 0;
    }
    return ret;
}
//deinit the ring:-1 means something error while 0 ok
s32_t ring_deinit(tagRingBuf *ring)
{
    int ret = -1;
    if(NULL != ring)
    {
        memset(ring, 0, sizeof(tagRingBuf));
        ret = 0;
    }
    return ret;
}

