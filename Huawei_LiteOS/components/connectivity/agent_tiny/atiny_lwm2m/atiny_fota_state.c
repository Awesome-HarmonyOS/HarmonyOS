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

#include "atiny_fota_state.h"
#include <string.h>
#include "firmware_update.h"


//TODO:set the update detail result
static int atiny_fota_state_default_handle(struct atiny_fota_state_tag_s *thi)
{
    ASSERT_THIS(return ATINY_ERR);

    ATINY_LOG(LOG_ERR, "err state handle state %d", thi->manager ?
              atiny_fota_manager_get_state(thi->manager) : -1);
    return ATINY_ERR;
}


void atiny_fota_state_init(atiny_fota_state_s *thi, atiny_fota_manager_s *manager)
{
    thi->start_download = (int (*)(struct atiny_fota_state_tag_s * thi, const char *uri))atiny_fota_state_default_handle;
    thi->execute_update = atiny_fota_state_default_handle;
    thi->finish_download = (int (*)(struct atiny_fota_state_tag_s * thi, int result))atiny_fota_state_default_handle;
    thi->repot_result = atiny_fota_state_default_handle;
    thi->recv_notify_ack = (int (*)(struct atiny_fota_state_tag_s  * thi, data_send_status_e status))atiny_fota_state_default_handle;
    thi->manager = manager;
}


static int atiny_fota_start_download(atiny_fota_state_s *thi, const char *uri)
{
    ASSERT_THIS(return ATINY_ARG_INVALID);

    atiny_fota_manager_set_update_result(thi->manager, ATINY_FIRMWARE_UPDATE_NULL);


    return atiny_fota_manager_rpt_state(thi->manager, ATINY_FOTA_DOWNLOADING);
}

static int atiny_fota_idle_state_recv_notify_ack(atiny_fota_state_s *thi, data_send_status_e status)
{
    int ret;
    atiny_fota_state_e rpt_state;

    if(SENT_SUCCESS != status)
    {
        ATINY_LOG(LOG_ERR, "idle state notify fail %d", status);
        atiny_fota_manager_set_update_result(thi->manager, ATINY_FIRMWARE_UPDATE_FAIL);
        return atiny_fota_manager_set_state(thi->manager, ATINY_FOTA_IDLE);
    }

    rpt_state = atiny_fota_manager_get_rpt_state(thi->manager) ;

    //idle and downloaded rpt ack
    if((ATINY_FOTA_IDLE == rpt_state) || (ATINY_FOTA_DOWNLOADED == rpt_state))
    {
        return atiny_fota_manager_set_state(thi->manager, rpt_state);
    }

    //updating rpt ack
    if(ATINY_FOTA_UPDATING == rpt_state)
    {
        ATINY_LOG(LOG_ERR, "idle state recv updaing state ack");
        return ATINY_ERR;
    }

    //downloading rpt ack
    //TODO, return then proper result
    ret = start_firmware_download(atiny_fota_manager_get_lwm2m_context(thi->manager), atiny_fota_manager_get_pkg_uri(thi->manager),
                                  atiny_fota_manager_get_storage_device(thi->manager));
    if(ret  == ATINY_OK)
    {
        return atiny_fota_manager_set_state(thi->manager, ATINY_FOTA_DOWNLOADING);

    }
    ATINY_LOG(LOG_ERR, "start_firmware_download fail %d", ret);
    atiny_fota_manager_set_update_result(thi->manager, ATINY_FIRMWARE_UPDATE_FAIL);
    return atiny_fota_manager_rpt_state(thi->manager, ATINY_FOTA_IDLE);

}

static int atiny_fota_idle_state_get_result(void)
{
   upgrade_state_e state;

   if(flag_upgrade_get_result(&state) != ATINY_OK)
   {
        ATINY_LOG(LOG_ERR, "ota_check_update_state fail");
        return ATINY_ERR;
   }

   return (OTA_SUCCEED == state) ? ATINY_OK : ATINY_ERR;
}

int atiny_fota_idle_state_int_report_result(atiny_fota_idle_state_s *thi)
{
    lwm2m_observe_info_t observe_info;
    int ret = ATINY_ERR;
    int result = ATINY_ERR;

    ASSERT_THIS(return ATINY_ARG_INVALID);

    thi->report_flag = false;
    memset(&observe_info, 0, sizeof(lwm2m_observe_info_t));
    if(flag_read(FLAG_APP, &observe_info, sizeof(observe_info)) != ATINY_OK)
    {
        ATINY_LOG(LOG_ERR, "flag_write fail");
        goto EXIT;
    }

    if(0 == observe_info.tokenLen)
    {
        return ATINY_OK;
    }

    ret = atiny_fota_idle_state_get_result();
    if(ret != ATINY_OK)
    {
        ATINY_LOG(LOG_ERR, "get_software_result fail");
    }

    result = ATINY_OK;
    thi->report_result = ret;
    thi->report_flag = true;
    memcpy(&thi->observe_info, &observe_info, sizeof(thi->observe_info));
    ATINY_LOG(LOG_INFO, "need to rpt result %d", ret);
EXIT:
    memset(&observe_info, 0, sizeof(observe_info));
    if(flag_write(FLAG_APP, &observe_info, sizeof(observe_info)) != ATINY_OK)
    {
        ATINY_LOG(LOG_ERR, "flag_write fail");
    }
    return result;
}

static int atiny_fota_idle_state_report_result(atiny_fota_state_s *thi)
{
    int ret = ATINY_ERR;
    atiny_fota_idle_state_s *idle_stat = (atiny_fota_idle_state_s *)thi;
    int state;
    lwm2m_data_cfg_t dataCfg = {0};


    ASSERT_THIS(return ATINY_ARG_INVALID);

    if(!idle_stat->report_flag)
    {
        return ATINY_OK;
    }

    idle_stat->report_flag = false;

    state = ((ATINY_OK == idle_stat->report_result) ?  ATINY_FOTA_IDLE : ATINY_FOTA_DOWNLOADED);
    atiny_fota_manager_set_update_result(thi->manager, (ATINY_OK == idle_stat->report_result) ? ATINY_FIRMWARE_UPDATE_SUCCESS : ATINY_FIRMWARE_UPDATE_FAIL);
    atiny_fota_manager_save_rpt_state(thi->manager, state);
    atiny_fota_manager_get_data_cfg(thi->manager, &dataCfg);
    ret = lwm2m_send_notify(atiny_fota_manager_get_lwm2m_context(thi->manager),
                            &idle_stat->observe_info, state, &dataCfg);
    ATINY_LOG(LOG_INFO, "lwm2m_send_notify result %d, state %d", ret, state);
    return ret;
}


void atiny_fota_idle_state_init(atiny_fota_idle_state_s *thi, atiny_fota_manager_s *manager)
{
    memset(thi, 0, sizeof(*thi));
    atiny_fota_state_init(&thi->interface, manager);
    thi->interface.start_download = atiny_fota_start_download;
    thi->interface.repot_result = atiny_fota_idle_state_report_result;
    thi->interface.recv_notify_ack = atiny_fota_idle_state_recv_notify_ack;
}
static int atiny_fota_downloading_state_finish_download(atiny_fota_state_s *thi, int result)
{
    ASSERT_THIS(return ATINY_ARG_INVALID);
    if(ATINY_OK != result)
    {
        atiny_fota_manager_set_update_result(thi->manager, ATINY_FIRMWARE_UPDATE_FAIL);
    }

    return atiny_fota_manager_rpt_state(thi->manager, (ATINY_OK == result) ? ATINY_FOTA_DOWNLOADED : ATINY_FOTA_IDLE);
}


static int atiny_fota_downloading_state_recv_notify_ack(atiny_fota_state_s *thi, data_send_status_e status)
{
    atiny_fota_state_e rpt_state;

    if(SENT_SUCCESS != status)
    {
        ATINY_LOG(LOG_ERR, "downloading state notify ack fail %d", status);
        atiny_fota_manager_set_update_result(thi->manager, ATINY_FIRMWARE_UPDATE_FAIL);
        return atiny_fota_manager_set_state(thi->manager, ATINY_FOTA_IDLE);
    }

    rpt_state = atiny_fota_manager_get_rpt_state(thi->manager);
    if((ATINY_FOTA_IDLE == rpt_state) || (ATINY_FOTA_DOWNLOADED == rpt_state))
    {
        return atiny_fota_manager_set_state(thi->manager, rpt_state);
    }
    else
    {
        ATINY_LOG(LOG_ERR, "recv notify ack err  in downloading state, rpt state %d", rpt_state);
        return ATINY_ERR;
    }
}
void atiny_fota_downloading_state_init(atiny_fota_downloading_state_s *thi, atiny_fota_manager_s *manager)
{
    atiny_fota_state_init(&thi->interface, manager);
    thi->interface.finish_download = atiny_fota_downloading_state_finish_download;
    thi->interface.recv_notify_ack = atiny_fota_downloading_state_recv_notify_ack;
}

static int atiny_fota_downloaded_state_execute_update(atiny_fota_state_s *thi)
{
    atiny_fota_state_e rpt_state = ATINY_FOTA_UPDATING;

    ASSERT_THIS(return ATINY_ARG_INVALID);

    if(atiny_fota_manager_get_update_result(thi->manager) != ATINY_FIRMWARE_UPDATE_NULL)
    {
        rpt_state = ATINY_FOTA_IDLE;
        atiny_fota_manager_set_update_result(thi->manager, ATINY_FIRMWARE_UPDATE_FAIL);
    }

    return atiny_fota_manager_rpt_state(thi->manager, rpt_state);
}


static int atiny_fota_downloaded_state_recv_notify_ack(atiny_fota_state_s *thi, data_send_status_e status)
{
    int ret;
    atiny_fota_state_e rpt_state;
    lwm2m_observe_info_t observe_info;
    pack_storage_device_api_s *device;

    ASSERT_THIS(return ATINY_ARG_INVALID);
    if(SENT_SUCCESS != status)
    {
        ATINY_LOG(LOG_ERR, "downloaded state notify fail %d", status);
        atiny_fota_manager_set_update_result(thi->manager, ATINY_FIRMWARE_UPDATE_FAIL);
        return atiny_fota_manager_set_state(thi->manager, ATINY_FOTA_IDLE);
    }

    rpt_state = atiny_fota_manager_get_rpt_state(thi->manager);
    //rpt downloading state ack
    if(ATINY_FOTA_DOWNLOADING == rpt_state)
    {
        ret = start_firmware_download(atiny_fota_manager_get_lwm2m_context(thi->manager), atiny_fota_manager_get_pkg_uri(thi->manager),
                                      atiny_fota_manager_get_storage_device(thi->manager));
        if(ret  == ATINY_OK)
        {
            return atiny_fota_manager_set_state(thi->manager, ATINY_FOTA_DOWNLOADING);

        }
        ATINY_LOG(LOG_ERR, "start_firmware_download fail %d", ret);
        atiny_fota_manager_set_update_result(thi->manager, ATINY_FIRMWARE_UPDATE_FAIL);
        (void)atiny_fota_manager_rpt_state(thi->manager, ATINY_FOTA_IDLE);
        return ATINY_ERR;
    }

    //rpt idle state ack
    if(ATINY_FOTA_IDLE == rpt_state)
    {
        return atiny_fota_manager_set_state(thi->manager, rpt_state);
    }

    //rpt downloaded state ack
    if(ATINY_FOTA_DOWNLOADED == rpt_state)
    {
        return ATINY_OK;
    }

    //rpt updating state ack
    if(lwm2m_get_observe_info(atiny_fota_manager_get_lwm2m_context(thi->manager), &observe_info) != COAP_NO_ERROR
            || 0 == observe_info.tokenLen)
    {
        ATINY_LOG(LOG_ERR, "lwm2m_get_observe_info fail");
        goto EXIT_DOWNLOADED;
    }

    device = atiny_fota_manager_get_storage_device(thi->manager);
    if((NULL == device) || (NULL == device->active_software) || (device->active_software(device) != ATINY_OK))
    {
        ATINY_LOG(LOG_ERR, "active_software fail");
        goto EXIT_DOWNLOADED;
    }

    if(flag_write(FLAG_APP, &observe_info, sizeof(observe_info)) != ATINY_OK)
    {
        ATINY_LOG(LOG_ERR, "flag_write fail");
        goto EXIT_DOWNLOADED;
    }

    atiny_set_reboot_flag();
    return atiny_fota_manager_set_state(thi->manager, ATINY_FOTA_UPDATING);

EXIT_DOWNLOADED:

    atiny_fota_manager_set_update_result(thi->manager, ATINY_FIRMWARE_UPDATE_FAIL);
    (void)atiny_fota_manager_rpt_state(thi->manager, ATINY_FOTA_DOWNLOADED);
    return ATINY_ERR;
}


void atiny_fota_downloaded_state_init(atiny_fota_downloaded_state_s *thi, atiny_fota_manager_s *manager)
{
    atiny_fota_state_init(&thi->interface, manager);
    thi->interface.start_download = atiny_fota_start_download;
    thi->interface.execute_update = atiny_fota_downloaded_state_execute_update;
    thi->interface.recv_notify_ack = atiny_fota_downloaded_state_recv_notify_ack;
}

void atiny_fota_updating_state_init(atiny_fota_updating_state_s *thi, atiny_fota_manager_s *manager)
{
    atiny_fota_state_init(&thi->interface, manager);
}

