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

/*******************************************************************************
 *
 * Copyright (c) 2014 Bosch Software Innovations GmbH, Germany.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * The Eclipse Distribution License is available at
 *    http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Bosch Software Innovations GmbH - Please refer to git log
 *
 *******************************************************************************/

#ifndef ATINY_FOTA_STATE_H_
#define ATINY_FOTA_STATE_H_
#include "atiny_fota_manager.h"
#include "log/atiny_log.h"
#include "object_comm.h"
#include "flag_manager.h"
#include "upgrade_flag.h"



#define ASSERT_THIS(do_something) \
        if(NULL == thi)\
        {\
            ATINY_LOG(LOG_ERR, "this null pointer");\
            do_something;\
        }

#define ATINY_GET_STATE(state) (&((state).interface))

#define CALL_MEM_FUNCTION_R(object, func,  ret, ...) do\
{\
    if(NULL != (object) && (NULL != (object)->func))\
    {\
        (ret) = (object)->func(__VA_ARGS__);\
    }\
}while(0)


typedef struct atiny_fota_state_tag_s
{
    int (*start_download)(struct atiny_fota_state_tag_s * thi, const char *uri);
    int (*execute_update)(struct atiny_fota_state_tag_s * thi);
    int (*finish_download)(struct atiny_fota_state_tag_s * thi, int result);
    int (*repot_result)(struct atiny_fota_state_tag_s *thi);
    int (*recv_notify_ack)(struct atiny_fota_state_tag_s *thi, data_send_status_e status);
    atiny_fota_manager_s *manager;
}atiny_fota_state_s;

typedef struct
{
    atiny_fota_state_s interface;
    lwm2m_observe_info_t observe_info;
    int report_result;
    bool report_flag;
}atiny_fota_idle_state_s;

typedef struct atiny_fota_downloading_state_tag_s
{
    atiny_fota_state_s interface;
}atiny_fota_downloading_state_s;

typedef atiny_fota_downloading_state_s atiny_fota_downloaded_state_s;
typedef atiny_fota_downloading_state_s atiny_fota_updating_state_s;


#ifdef __cplusplus
extern "C" {
#endif


void atiny_fota_state_init(atiny_fota_state_s *thi, atiny_fota_manager_s *manager);

void atiny_fota_idle_state_init(atiny_fota_idle_state_s *thi, atiny_fota_manager_s *manager);
int atiny_fota_idle_state_int_report_result(atiny_fota_idle_state_s * thi);



void atiny_fota_downloading_state_init(atiny_fota_downloading_state_s *thi, atiny_fota_manager_s *manager);

void atiny_fota_downloaded_state_init(atiny_fota_downloaded_state_s *thi, atiny_fota_manager_s *manager);

void atiny_fota_updating_state_init(atiny_fota_updating_state_s *thi, atiny_fota_manager_s *manager);

#ifdef __cplusplus
}
#endif

#endif /* ATINY_FOTA_STATE_H_ */
