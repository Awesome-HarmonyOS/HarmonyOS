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
#ifndef __ESP8266_H__
#define __ESP8266_H__

#include "at_frame/at_main.h"

#define WIFI_SSID      		"APAP1234"
#define WIFI_PASSWD    		"87654321"

#define AT_MODU_NAME    	"ESP8266"
#define AT_USART_PORT   	3
#define AT_BUARDRATE   		115200
#define AT_CMD_TIMEOUT		10000    //ms
#define AT_MAX_LINK_NUM     4

#define AT_LINE_END 		"\r\n"
#define AT_CMD_BEGIN		"\r\n"


#define MAX_AT_USERDATA_LEN (1024*5)


#define AT_CMD_RST    		"AT+RST"
#define AT_CMD_ECHO_OFF 	"ATE0"
#define AT_CMD_CWMODE  		"AT+CWMODE_CUR"
#define AT_CMD_JOINAP  		"AT+CWJAP_CUR"
#define AT_CMD_MUX 			"AT+CIPMUX"
#define AT_CMD_CONN			"AT+CIPSTART"
#define AT_CMD_SEND			"AT+CIPSEND"
#define AT_CMD_CLOSE		"AT+CIPCLOSE"
#define AT_CMD_CHECK_IP		"AT+CIPSTA_CUR?"
#define AT_CMD_CHECK_MAC	"AT+CIPSTAMAC_CUR?"
#define AT_CMD_SHOW_DINFO   "AT+CIPDINFO"

#define AT_DATAF_PREFIX      "\r\n+IPD"

typedef enum {
	STA = 1,
	AP, 
	ATA_AP,
}enum_net_mode;
#endif
