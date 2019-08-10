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

#ifndef ATINY_FOTA_MANAGER_H_
#define ATINY_FOTA_MANAGER_H_

#include "atiny_lwm2m/agenttiny.h"
#include "liblwm2m.h"
#include "ota/package.h"

#define array_size(a) (sizeof(a)/sizeof(*(a)))


/*
0: Initial value. Once the updating process is initiated (Download /Update), this Resource MUST be reset to Initial value.
1: Firmware updated successfully,
2: Not enough flash memory for the new firmware package.
3. Out of RAM during downloading process.
4: Connection lost during downloading process.
5: Integrity check failure for new downloaded package.
6: Unsupported package type.
7: Invalid URI
8: Firmware update failed
9: Unsupported protocol

*/

typedef int atiny_fota_state_e;
enum //atiny_fota_state_e
{
    ATINY_FOTA_IDLE = 0,
    ATINY_FOTA_DOWNLOADING,
    ATINY_FOTA_DOWNLOADED,
    ATINY_FOTA_UPDATING
};


typedef int atiny_update_result_e;
enum
{
    ATINY_FIRMWARE_UPDATE_NULL = 0,
    ATINY_FIRMWARE_UPDATE_SUCCESS = 1,
    ATINY_FIRMWARE_UPDATE_FAIL = 8
};

struct atiny_fota_manager_tag_s;
typedef struct atiny_fota_manager_tag_s atiny_fota_manager_s;

#ifdef __cplusplus
    extern "C" {
#endif

char * atiny_fota_manager_get_pkg_uri(const atiny_fota_manager_s *thi);
int atiny_fota_manager_get_state(const atiny_fota_manager_s *thi);
int atiny_fota_manager_get_update_result(const atiny_fota_manager_s *thi);
void atiny_fota_manager_set_update_result(atiny_fota_manager_s *thi, atiny_update_result_e result);
int atiny_fota_manager_get_deliver_method(const atiny_fota_manager_s *thi);
int atiny_fota_manager_start_download(atiny_fota_manager_s *thi, const char *uri, uint32_t len);
int atiny_fota_manager_execute_update(atiny_fota_manager_s *thi);
int atiny_fota_manager_repot_result(atiny_fota_manager_s *thi);
int atiny_fota_manager_set_state(atiny_fota_manager_s *thi, atiny_fota_state_e state);
int atiny_fota_manager_set_storage_device(atiny_fota_manager_s *thi);
pack_storage_device_api_s *atiny_fota_manager_get_storage_device(atiny_fota_manager_s *thi);
int atiny_fota_manager_set_lwm2m_context(atiny_fota_manager_s *thi, lwm2m_context_t*  lwm2m_context);
lwm2m_context_t* atiny_fota_manager_get_lwm2m_context(atiny_fota_manager_s *thi);
void atiny_fota_manager_get_data_cfg(const atiny_fota_manager_s *thi, lwm2m_data_cfg_t *data_cfg);

void atiny_fota_manager_destroy(atiny_fota_manager_s *thi);
int atiny_fota_manager_rpt_state(atiny_fota_manager_s *thi, atiny_fota_state_e rpt_state);
void atiny_fota_manager_save_rpt_state(atiny_fota_manager_s *thi, atiny_fota_state_e rpt_state);


atiny_fota_state_e atiny_fota_manager_get_rpt_state(const atiny_fota_manager_s *thi);



atiny_fota_manager_s * atiny_fota_manager_get_instance(void);

#ifdef __cplusplus
}
#endif


#endif /* ATINY_FOTA_MANAGER_H_ */
