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
#ifndef __SIM900A_H__
#define __SIM900A_H__

#include "at_frame/at_main.h"

#define AT_MODU_NAME        "SIM900A"
#define AT_USART_PORT       2
#define AT_BUARDRATE        115200
#define AT_CMD_TIMEOUT      10000    //ms
#define AT_MAX_LINK_NUM     4

#define AT_LINE_END 		"\r"
#define AT_CMD_BEGIN		"\r\n"
#define MAX_AT_USERDATA_LEN (1024*5)

#define AT_CMD_AT    		"AT"
#define AT_CMD_CPIN         "AT+CPIN?"//check sim card
#define AT_CMD_COPS         "AT+COPS?"//check register network
#define AT_CMD_CLOSE    	"AT+CIPCLOSE"
#define AT_CMD_SHUT    		"AT+CIPSHUT"
#define AT_CMD_ECHO_OFF 	"ATE0"
#define AT_CMD_ECHO_ON  	"ATE1"
#define AT_CMD_MUX 			"AT+CIPMUX"
#define AT_CMD_CLASS        "AT+CGCLASS"//set MS type
#define AT_CMD_PDP_CONT   	"AT+CGDCONT"//configure pdp context
#define AT_CMD_PDP_ATT    	"AT+CGATT"//pdp attach network
#define AT_CMD_PDP_ACT		"AT+CGACT"//active pdp context
#define AT_CMD_CSTT			"AT+CSTT"//start task
#define AT_CMD_CIICR		"AT+CIICR"//start gprs connect
#define AT_CMD_CIFSR		"AT+CIFSR"//get local ip
#define AT_CMD_CIPHEAD		"AT+CIPHEAD"
#define AT_CMD_CONN			"AT+CIPSTART"
#define AT_CMD_SEND			"AT+CIPSEND"
#define AT_CMD_CLOSE		"AT+CIPCLOSE"

#define AT_DATAF_PREFIX      "\r\n+IPD"
#define AT_DATAF_PREFIX_MULTI      "\r\n+RECEIVE"
#define SIM900A_DELAY       LOS_TaskDelay

#endif /* __SIM900A_H__ */

