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

/**@defgroup nbiot
 * @ingroup nbiot
 */

#ifndef __NB_IOT_H__
#define __NB_IOT_H__
#include "at_frame/at_main.h"

typedef struct sec_param{
char* psk;
char* pskid;
uint8_t setpsk;
}sec_param_s;

extern at_task at;

/*
Func Name: los_nb_init

@par Description
    This API is used to init nb module and connect to cloud.
@param[in]  host  cloud ip
@param[in]  port  cloud port
@param[in]  psk   if not null,the security param
@par Return value
*  0:on success
*  negative value: on failure
*/
int los_nb_init(const int8_t* host, const int8_t* port, sec_param_s* psk);
/*
Func Name: los_nb_report

@par Description
    This API is used for nb module to report data to cloud.
@param[in] buf point to data to be reported
@param[in] buflen data length
@par Return value
*  0:on success
*  negative value: on failure
*/
int los_nb_report(const char* buf, int buflen);
/*
Func Name: los_nb_notify

@par Description
    This API is used to regist callback when receive the cmd from cloud.
@param[in] featurestr feature string that in cmd
@param[in] cmdlen length of feature string
@param[in] callback callback of device
@par Return value
*  0:on success
*  negative value: on failure
*/

int los_nb_notify(char* featurestr,int cmdlen, oob_callback callback, oob_cmd_match cmd_match);
/*
Func Name: los_nb_deinit

@par Description
    This API is used to deinit the nb module.
@param[in] NULL
@par Return value
*  0:on success
*  negative value: on failure
*/

int los_nb_deinit(void);
#endif
