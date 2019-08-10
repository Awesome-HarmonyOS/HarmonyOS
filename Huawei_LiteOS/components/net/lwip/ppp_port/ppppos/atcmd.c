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
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <string.h>

#include "osport.h"

#define CN_IODEV_WTIMEOUT   100
#define CN_IODEV_RTIMEOUT   100
#ifndef bool_t
#define bool_t int
#define false 0
#define true  1
#endif

#define CN_AT_LEN  128

//we use this for the at command
//usage:we use this function to deal the at result as the args format
static int __getpara(char *text, char *seperate, char *argv[], int argc)
{
    int result;
    char *s;
    int len, i;
    s = seperate;
    len = strlen(text);
    while(*s != '\0') //make all the charactor in text matching the seperate to 0
    {
        for(i = 0; i < len; i++)
        {
            if(text[i] == *s)
            {
                text[i] = '\0';
            }
        }
        s++;
    }
    //ok now check the para start
    result = 0;
    s = text;
    while(result < argc)
    {
        //jump the NULL
        while(*s == '\0')
        {
            s++;
        }
        //the s is the start
        if(s < (text + len))
        {
            argv[result] = s;
            result++;
            s = s + strlen(s);
        }
        else
        {
            break;
        }
    }
    return result;
}

////////////////////////////////DO THE AT COMMAND HERE//////////////////////////////////////
//-----------------------------------------------------------------------------
//??:use this function to do the at command and analyze
//??:devname, the at device name,cmd:at command buf/buflen:storage the reply result(argc and argv in args mode)
//??:
//??:
//??:zhangqf@??10:51:51/2017?3?23?
//-----------------------------------------------------------------------------
//usage:send the at command to the serial device
int AtCmd(const char *devname, char *cmd, char *buf, int buflen, int argc, char *argv[])
{
    char   cmdbuf[CN_AT_LEN];
    int    result = 0;
    int    len = 0;
    int    lenleft;
    int    offset;
    int    dev;
    //open the at command device
    dev = iodev_open(devname, 0, 0);
    if(-1 == dev)
    {
        printf("%s:open %s dev failed \n\r", __FUNCTION__, devname);
        goto EXIT_OPENFAILED;
    }
    //flush the device
    iodev_flush(dev);
    //initialize the buf with the specified at command
    memset(cmdbuf, 0, CN_AT_LEN);
    snprintf(cmdbuf, CN_AT_LEN, "%s\r\n", cmd);//AT+CGMI
    //write the command to the device
    len = strlen(cmdbuf);
    result = iodev_write(dev, (unsigned char *)cmdbuf, len, CN_IODEV_WTIMEOUT);
    if(result != len)
    {
        printf("%s:only write %d/%d to %s \n\r", __FUNCTION__, result, len, devname);
        goto EXIT_WRITEFAILED;
    }

    //if need the result,then we will wait for the timeout
    if((NULL != buf) && (buflen > 0))
    {
        //initialize the buf
        memset(buf, 0, buflen);
        offset = 0;
        lenleft = buflen;
        while(1)
        {
            len = iodev_read(dev, (unsigned char *)&buf[offset], lenleft, CN_IODEV_RTIMEOUT);
            if(len > 0)
            {
                offset += len;
                lenleft -= len;
                if(offset == buflen)
                {
                    break;
                }
                else if(offset > buflen)
                {
                    printf("%s:read error--driver error\n\r", __FUNCTION__);
                }
                else
                {
                    //do nothing
                }
            }
            else
            {
                break;
            }
        }
        result = offset;
        if(offset == 0)
        {
            printf("%s:read %s error\n\r", __FUNCTION__, devname);
            goto EXIT_READFAILED;
        }
        else
        {
            memset(&buf[offset], 0, lenleft); //make the bufleft to zero
        }
        if((argc > 0) && (NULL != argv))
        {
            result = __getpara((char *)buf, "\n\r", argv, argc);
            if(result <= 0)
            {
                printf("%s:get para failed\n\r", __FUNCTION__);
                goto EXIT_GETPARA;
            }
        }
    }
    //close the device and return
    iodev_close(dev);
    return result;

EXIT_GETPARA:
EXIT_READFAILED:
EXIT_WRITEFAILED:
    iodev_close(dev);
EXIT_OPENFAILED:
    return  result;
}


//usage:use this function to check if the string is in the argv
//      if exit then return the position it in
static int strinargs(int argc, char *argv[], char *str)
{
    int result = -1;
    int i = 0;
    while(i < argc)
    {
        if(strstr(argv[i], str))
        {
            result = i;
            break;
        }
        else
        {
            i++;
        }
    }
    return result;
}

typedef struct
{
    const char *mcc_mnc;
    const char *apndefault;
} tagImsi;

tagImsi gAtcimi[]={\
    {"46000","CMNET"},\
    {"46002","CMNET"},\
    {"46004","CMNET"},\
    {"46007","CMNET"},\
    {"46001","3gnet"},\
    {"46006","3gnet"},\
    {"46009","3gnet"},\
    {"46003","ctnet"},\
    {"46005","ctnet"},\
    {"46011","ctlte"},\
//		{"46020","cmnet"},  //not support the tietong
};
#define CN_CIMI_SIZE  (sizeof(gAtcimi)/sizeof(tagImsi))

//usage:used to check the lte module
static bool_t checkmi(char *devname, int times)
{
    bool_t result = false;
    char atrcvbuf[CN_AT_LEN];

    char *argv[6];
    int   argc;
    int   i = 0;
    int  position;

    //first we should check if the sim card inserted:at+cpin?
    printf("checkcgmi:");
    for(i = 0; i < times; i++)
    {
        printf("%d->", i);
        memset(argv, 0, sizeof(argv));
        memset(atrcvbuf, 0, sizeof(atrcvbuf));
        argc = AtCmd(devname,"AT+CGMI" , atrcvbuf, CN_AT_LEN, 6, argv);
        if(argc > 0)
        {
            position = strinargs(argc, argv, "OK");
            if((position != 0) && (position != -1))
            {
                result = true;
                break;
            }
        }
        task_sleepms(1000);
    }
    if(result)
    {
        printf(".:OK:%s\n\r", argv[position - 1]);
    }
    else
    {
        printf(".timeout!\n\r");
    }
    return result;
}
//usage:used to check the module type
static bool_t checkmm(char *devname, int times)
{
    bool_t result = false;
    char *argv[6];
    int   argc;
    int   i = 0;
    int  position;
    char atrcvbuf[CN_AT_LEN];

    //first we should check if the sim card inserted:at+cpin?
    printf("checkcgmm:");
    for(i = 0; i < times; i++)
    {
        printf("%d->", i);
        memset(argv, 0, sizeof(argv));
        memset(atrcvbuf, 0, sizeof(atrcvbuf));
        argc = AtCmd(devname,"AT+CGMM",atrcvbuf,CN_AT_LEN,6,argv);
        if(argc > 0)
        {
            position = strinargs(argc, argv, "OK");
            if((position != 0) && (position != -1))
            {
                result = true;
                break;
            }
        }
        task_sleepms(1000);
    }
    if(result)
    {
        printf(".:OK:%s\n\r", argv[position - 1]);
    }
    else
    {
        printf(".timeout!\n\r");
    }
    return result;
}
//usage:used to check the module sn
static bool_t checksn(char *devname, int times)
{
    bool_t result = false;
    char *argv[6];
    int   argc;
    int   i = 0;
    int  position;
    char atrcvbuf[CN_AT_LEN];
    //first we should check if the sim card inserted:at+cpin?
    printf("checkcgsn:");
    for(i = 0; i < times; i++)
    {
        printf("%d->", i);
        memset(argv, 0, sizeof(argv));
        memset(atrcvbuf, 0, sizeof(atrcvbuf));
        argc = AtCmd(devname,"AT+CGSN",atrcvbuf,CN_AT_LEN,6,argv);
        if(argc > 0)
        {
            position = strinargs(argc, argv, "OK");
            if((position != 0) && (position != -1))
            {
                result = true;
                break;
            }
        }
        task_sleepms(1000);

    }
    if(result)
    {
        printf(".:OK:%s\n\r", argv[position - 1]);
    }
    else
    {
        printf(".timeout!\n\r");
    }
    return result;
}
//usage:used to check the module sn
static bool_t checkmr(char *devname, int times)
{
    bool_t result = false;
    char *argv[6];
    int   argc;
    int   i = 0;
    int  position;
    char atrcvbuf[CN_AT_LEN];
    //first we should check if the sim card inserted:at+cpin?
    printf("checkcgmr:");
    for(i = 0; i < times; i++)
    {
        printf("%d->", i);
        memset(argv, 0, sizeof(argv));
        memset(atrcvbuf, 0, sizeof(atrcvbuf));
        argc = AtCmd(devname,"AT+CGMR",atrcvbuf,CN_AT_LEN,6,argv);
        if(argc > 0)
        {
            position = strinargs(argc, argv, "OK");
            if((position != 0) && (position != -1))
            {
                result = true;
                break;
            }
        }
        task_sleepms(1000);

    }
    if(result)
    {
        printf(".:OK:%s\n\r", argv[position - 1]);
    }
    else
    {
        printf(".timeout!\n\r");
    }
    return result;
}
//usage:used to check sim card mnc
static tagImsi *checkcimi(char *devname, int times, char *simapn)
{
    char *argv[6];
    int   argc;
    int   i = 0, tmp = 0;
    int  position = -1;
    char atrcvbuf[CN_AT_LEN];
    //find the mnc here
    tagImsi *result = NULL;
    //first we should check if the sim card inserted:at+cpin?
    printf("checkcimi:");
    for(i = 0; i < times; i++)
    {
        printf("%d->", i);
        memset(argv, 0, sizeof(argv));
        memset(atrcvbuf, 0, sizeof(atrcvbuf));
        argc = AtCmd(devname,"AT+CIMI",atrcvbuf,CN_AT_LEN,6,argv);
        if(argc > 0)
        {
            position = strinargs(argc, argv, "OK");
            if((position != 0) && (position != -1))
            {
                char mnc[6];
                memset(mnc, 0, 6);
                memcpy(mnc, argv[position - 1], 5);
                for(tmp = 0; tmp < CN_CIMI_SIZE; tmp++)
                {
                    if(0 == strcmp(mnc, gAtcimi[tmp].mcc_mnc))
                    {
                        result = &gAtcimi[tmp];
                        if( (simapn == NULL) || (simapn[0] == '\0') )
                            printf(".:OK:cimi:%s apn:%s\n\r", argv[position - 1], result->apndefault);
                        else
                            printf(".:OK:cimi:%s apn:%s\n\r", argv[position - 1], simapn);
                        break;
                    }
                }
                if(NULL == result)
                {
                    printf(".:OK:cimi:%s apn:%s\n\r", argv[position - 1], "unknown");
                }
                break;
            }
        }
        task_sleepms(1000);

    }
    if((position == 0) || (position == -1))
    {
        printf(".timeout!\n\r");
    }
    return result;
}
//usage:used to check if the simcard is inserted
static bool_t checkcpin(char *devname, int times)
{
    bool_t result = false;
    char *argv[6];
    int   argc;
    int   i = 0;
    char atrcvbuf[CN_AT_LEN];
    //first we should check if the sim card inserted:at+cpin?
    printf("checkcpin:");
    for(i = 0; i < times; i++)
    {
        printf("%d->", i);
        memset(argv, 0, sizeof(argv));
        memset(atrcvbuf, 0, sizeof(atrcvbuf));
        argc = AtCmd(devname,"AT+CPIN?",atrcvbuf,CN_AT_LEN,6,argv);
        if(argc > 0)
        {
            if(-1 != strinargs(argc, argv, "READY"))
            {
                result = true;
                break;
            }
        }
        task_sleepms(1000);
    }
    if(result)
    {
        printf(".:OK\n\r");
    }
    else
    {
        printf(".timeout!\n\r");
    }
    return result;
}

//usage:used to check if the net is registered
static bool_t  checkcgreg(char *devname, int times)
{
    bool_t result = false;
    char *argv[6];
    int   argc;
    int   i = 0;
    char atrcvbuf[CN_AT_LEN];
    //first we should check if the sim card inserted:at+cpin?
    printf("checkcreg:");
    for(i = 0; i < times; i++)
    {
        printf("%d->", i);
        memset(argv, 0, sizeof(argv));
        memset(atrcvbuf, 0, sizeof(atrcvbuf));
        argc = AtCmd(devname,"AT+CGREG?",atrcvbuf,CN_AT_LEN,6,argv);
        if(argc > 0)
        {
            if((-1 != strinargs(argc, argv, ",1")) || (-1 != strinargs(argc, argv, ",5")))
            {
                result = true;
                break;
            }
        }
    }
    if(result)
    {
        printf(".:OK\n\r");
    }
    else
    {
        printf(".timeout!\n\r");
    }
    return result;
}
//usage:used to set the apn:set the apn
static bool_t  setnetapn(char *devname, char *apn, int times)
{
    bool_t result = false;
    char *argv[6];
    int   argc;
    int   i = 0;
    char atrcvbuf[CN_AT_LEN];
    printf("setapn:");
    for(i = 0; i < times; i++)
    {
        printf("%d->", i);
        memset(argv, 0, sizeof(argv));
        memset(atrcvbuf, 0, sizeof(atrcvbuf));
        char cgdcont[64];
        memset(cgdcont, 0, 64);
        snprintf(cgdcont,63,"%s%s%s%s","AT+CGDCONT=1,\"IP\",","\"",apn,"\"");
        argc = AtCmd(devname, cgdcont, atrcvbuf, CN_AT_LEN, 6, argv);
        if(argc > 0)
        {
            if((-1 != strinargs(argc, argv, "OK")) || (-1 != strinargs(argc, argv, "ok")))
            {
                result = true;
                break;
            }
        }
        task_sleepms(1000);
    }
    if(result)
    {
        printf(".ready!\n\r");
    }
    else
    {
        printf(".timeout!\n\r");
    }
    return result;
}

//usage:used to call the data connection,form now on, we will change to data mode,any at command will be invalid
static bool_t  atdcall(char *devname, int times)
{
    bool_t result = false;
    char *argv[6];
    int   argc;
    int   i = 0;
    char atrcvbuf[CN_AT_LEN];
    printf("atdcall:");
    for(i = 0; i < times; i++)
    {
        printf("%d->", i);
        memset(argv, 0, sizeof(argv));
        memset(atrcvbuf, 0, sizeof(atrcvbuf));
        argc = AtCmd(devname,"ATD*99***1#",atrcvbuf,CN_AT_LEN,6,argv);
        if(argc > 0)
        {
            if((-1 != strinargs(argc, argv, "CONNECT")) || (-1 != strinargs(argc, argv, "connect")))
            {
                result = true;
                break;
            }
        }
        task_sleepms(1000);
    }
    if(result)
    {
        printf(".ready!\n\r");
    }
    else
    {
        printf(".timeout!\n\r");
    }
    return result;
}

static bool_t atgetsignal(char *devname, int *signal)
{
    bool_t ret = false;
    char *argv[6];
    int   argc;
    int   position = -1;
    char atrcvbuf[CN_AT_LEN];
    int   result = -1;
    memset(argv, 0, sizeof(argv));
    memset(atrcvbuf, 0, sizeof(atrcvbuf));
    argc = AtCmd(devname,"AT+CSQ",atrcvbuf,CN_AT_LEN,6,argv);
    if(argc > 0)
    {
        position = strinargs(argc, argv, "OK");
        if((position != 0) && (position != -1))
        {
            sscanf(argv[position - 1], "+CSQ: %d", &result);
        }
        else
        {
            result = -1;
        }
    }
    if(NULL != signal)
    {
        *signal = result;
    }
    if(result > 0)
    {
        ret = true;
    }
    return ret;
}
//usage:this function used to check the modem state and change the modem state from at to data
int AtDial(char *devname, char *apn)
{
    bool_t result = false;
    int  ret = -1;
    char  *simapn;
    simapn = apn;
    printf("ATCMD CALL BEGIN:\n\r");
    //first we should check the module type
    result = checkmi(devname, 32);
    if(result == false)
    {
        return ret;
    }
    //check the signal
    result = atgetsignal(devname, NULL);
    if(result == false)
    {
        return ret;
    }

    //check the mm
    result = checkmm(devname, 32);
    if(result == false)
    {
        return ret;
    }
    //check the mr
    result = checkmr(devname, 32);
    if(result == false)
    {
        return ret;
    }
    //check the sn
    result = checksn(devname, 32);
    if(result == false)
    {
        return ret;
    }
    //first we should check if the sim card inserted:at+cpin?
    result = checkcpin(devname, 32);
    if(result == false)
    {
        return ret;
    }
    //check the apn here
    if((NULL == simapn) || (simapn[0] == '\0'))
    {
        //do check our self
        tagImsi *imsi;
        imsi = checkcimi(devname, 32, NULL);
        if(NULL == imsi)
        {
            result = false ;
            return ret;
        }
        else
        {
            simapn = (char *)imsi->apndefault;
        }
    }
    else
    {
        checkcimi(devname, 32, simapn);
    }
    //check if we has register the sim card to the carrieroperator
    result = checkcgreg(devname, 32);
    if(result == false)
    {
        return result;
    }
    //OK,now set the apn to the carrieroperator
    result = setnetapn(devname, simapn, 32);
    if(result == false)
    {
        return result;
    }
    //now we begin to atd(call the data service)
    result = atdcall(devname, 32);
    if(result == false)
    {
        return ret;
    }
    //for the end
    ret = 0;
    return ret;
}







