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

#ifndef __EMTC_BG36_H__
#define __EMTC_BG36_H__

#include "at_frame/at_main.h"
#define AT_MODU_NAME        "BG36"
#define AT_USART_PORT       3
#define AT_BUARDRATE        115200
#define BG36_TIMEOUT        10000    //ms
#define MAX_AT_USERDATA_LEN (1024*4)
#define MAX_SEND_DATA_LEN   1400

#define AT_LINE_END 		"\r"
#define AT_CMD_BEGIN		"\r\n"

typedef struct emtc_socket_info_t
{
    int len;
    int offset;
    char *buf;
    bool used_flag;
}emtc_socket_info;


#define ATI "ATI\r"
#define ATE0 "ATE0\r"
#define CMEE "AT+CMEE=2\r"
#define QCFG "AT+QCFG=\"nwscanseq\",03\r"

#define CPIN "AT+CPIN?\r"
#define CREG "AT+CREG?\r"
#define GETQICSGP "AT+QICSGP=1\r"
#define SETCELL "AT+CGREG=2\r"
#define QUERYCELL "AT+CGREG?\r"

#define QICSGP "AT+QICSGP=1,1,\"HUAWEI.COM\",\"\",\"\",1\r"
#define QIACT "AT+QIACT=1\r"
#define QIACTQUERY "AT+QIACT?\r"
#define CSQ "AT+CSQ\r"
#define QIOPEN_SOCKET "AT+QIOPEN=1"
#define QUERYCFATT "AT+CGATT?\r"
#define AT_DATAF_PREFIX "+QIURC:"

#endif
