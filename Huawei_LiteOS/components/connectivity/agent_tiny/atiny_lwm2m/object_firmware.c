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
 * Copyright (c) 2013, 2014 Intel Corporation and others.
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
 *    Julien Vermillard - initial implementation
 *    Fabien Fleutot - Please refer to git log
 *    David Navarro, Intel Corporation - Please refer to git log
 *    Bosch Software Innovations GmbH - Please refer to git log
 *    Pascal Rieux - Please refer to git log
 *
 *******************************************************************************/

/*
 * This object is single instance only, and provide firmware upgrade functionality.
 * Object ID is 5.
 */

/*
 * resources:
 * 0 package                   write
 * 1 package url               write
 * 2 update                    exec
 * 3 state                     read
 * 4 update supported objects  read/write
 * 5 update result             read
 */

#include "internals.h"
#include "atiny_lwm2m/agenttiny.h"
#include "log/atiny_log.h"

#ifdef CONFIG_FEATURE_FOTA
#include "atiny_fota_manager.h"


// ---- private object "Firmware" specific defines ----
// Resource Id's:
#define RES_M_PACKAGE                   0
#define RES_M_PACKAGE_URI               1
#define RES_M_UPDATE                    2
#define RES_M_STATE                     3
#define RES_O_UPDATE_SUPPORTED_OBJECTS  4
#define RES_M_UPDATE_RESULT             5
#define RES_O_PKG_NAME                  6
#define RES_O_PKG_VERSION               7
#define RES_O_FIRMWARE_UPDATE_DELIVER_METHOD               8


static uint8_t prv_firmware_read(uint16_t instanceId,
                                 int *numDataP,
                                 lwm2m_data_t **dataArrayP,
                                 lwm2m_data_cfg_t *dataCfg,
                                 lwm2m_object_t *objectP)
{
    uint32_t i;
    uint8_t result;

    // this is a single instance object
    if (instanceId != 0)
    {
        return COAP_404_NOT_FOUND;
    }

    // is the server asking for the full object ?
    if (*numDataP == 0)
    {
        uint16_t resources[] = {RES_M_PACKAGE_URI, RES_M_STATE,
                                RES_M_UPDATE_RESULT, RES_O_FIRMWARE_UPDATE_DELIVER_METHOD
                               };
        *dataArrayP = lwm2m_data_new(array_size(resources));
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = array_size(resources);
        for(i = 0 ; i < array_size(resources); ++i)
        {
            (*dataArrayP)[i].id = resources[i];
        }
    }

    i = 0;
    do
    {
        switch ((*dataArrayP)[i].id)
        {
        case RES_M_PACKAGE:
            result = COAP_405_METHOD_NOT_ALLOWED;
            break;
        case RES_M_PACKAGE_URI:
        {
#ifdef CONFIG_FEATURE_FOTA
            const char *pkg_uri;
            pkg_uri = atiny_fota_manager_get_pkg_uri(atiny_fota_manager_get_instance());
            if(pkg_uri == NULL)
            {
                pkg_uri = "";
            }
            lwm2m_data_encode_nstring(pkg_uri, strlen(pkg_uri) + 1, *dataArrayP + i);
            result = COAP_205_CONTENT;
#else 
            result = COAP_405_METHOD_NOT_ALLOWED; 
#endif 
            break;
        }

        case RES_M_UPDATE:
            result = COAP_405_METHOD_NOT_ALLOWED;
            break;

        case RES_M_STATE:
            // firmware update state (int)
        {
#ifdef CONFIG_FEATURE_FOTA    
            int state = atiny_fota_manager_get_rpt_state(atiny_fota_manager_get_instance());
            lwm2m_data_encode_int(state, *dataArrayP + i);
            result = COAP_205_CONTENT;
#else
            result = COAP_405_METHOD_NOT_ALLOWED; 
#endif
            break;
        }

        case RES_M_UPDATE_RESULT:
        {
#ifdef CONFIG_FEATURE_FOTA    
            int updateresult = atiny_fota_manager_get_update_result(atiny_fota_manager_get_instance());
            lwm2m_data_encode_int(updateresult, *dataArrayP + i);
            result = COAP_205_CONTENT;
#else
            result = COAP_405_METHOD_NOT_ALLOWED; 
#endif         
            break;
        }

        case RES_O_FIRMWARE_UPDATE_DELIVER_METHOD:
        {
#ifdef CONFIG_FEATURE_FOTA   
            int method = atiny_fota_manager_get_deliver_method(atiny_fota_manager_get_instance());
            lwm2m_data_encode_int(method, *dataArrayP + i);
            result = COAP_205_CONTENT;
#else
            result = COAP_405_METHOD_NOT_ALLOWED; 
#endif

        }
        break;
        default:
            result = COAP_404_NOT_FOUND;
        }

        i++;
    }
    while (i < *numDataP && result == COAP_205_CONTENT);

#ifdef CONFIG_FEATURE_FOTA   
    if(dataCfg && (1 == *numDataP) && (RES_M_STATE == (*dataArrayP)[0].id))
    {
        atiny_fota_manager_get_data_cfg(atiny_fota_manager_get_instance(), dataCfg);
    }
#endif

    return result;
}

static uint8_t prv_firmware_write(uint16_t instanceId,
                                  int numData,
                                  lwm2m_data_t *dataArray,
                                  lwm2m_object_t *objectP)
{
    int i;
    uint8_t result;

    // this is a single instance object
    if (instanceId != 0)
    {
        return COAP_404_NOT_FOUND;
    }

    i = 0;

    do
    {
        switch (dataArray[i].id)
        {

        case RES_M_PACKAGE_URI:
            // URL for download the firmware
        {
            int ret;
            if(dataArray[i].type != LWM2M_TYPE_STRING || NULL == dataArray[i].value.asBuffer.buffer)
            {
                ATINY_LOG(LOG_ERR, "type ERR %d", dataArray[i].type);
                result = COAP_400_BAD_REQUEST;
                break;
            }
#ifdef CONFIG_FEATURE_FOTA   

            ret = atiny_fota_manager_start_download(atiny_fota_manager_get_instance(), \
                                                    (const char *)(dataArray[i].value.asBuffer.buffer), dataArray[i].value.asBuffer.length);
#else
            ret = ATINY_ERR;
#endif
            result = (ATINY_OK == ret ? COAP_204_CHANGED : COAP_405_METHOD_NOT_ALLOWED);
            break;
        }

        default:
            result = COAP_405_METHOD_NOT_ALLOWED;
        }

        i++;
    }
    while (i < numData && result == COAP_204_CHANGED);

    return result;
}

static uint8_t prv_firmware_execute(uint16_t instanceId,
                                    uint16_t resourceId,
                                    uint8_t *buffer,
                                    int length,
                                    lwm2m_object_t *objectP)
{

    // this is a single instance object
    if (instanceId != 0)
    {
        return COAP_404_NOT_FOUND;
    }

    if (length != 0) return COAP_400_BAD_REQUEST;

    // for execute callback, resId is always set.
    switch (resourceId)
    {
    case RES_M_UPDATE:
    {
#ifdef CONFIG_FEATURE_FOTA 
        int ret = atiny_fota_manager_execute_update(atiny_fota_manager_get_instance());
#else
        int ret = ATINY_ERR;
#endif
        if (ATINY_OK == ret)
        {
            return COAP_204_CHANGED;
        }
        else
        {
            // firmware update already running
#ifdef CONFIG_FEATURE_FOTA
            return COAP_400_BAD_REQUEST;
#else
            return COAP_405_METHOD_NOT_ALLOWED;
#endif
        }
    }
    default:
        return COAP_405_METHOD_NOT_ALLOWED;
    }


}


lwm2m_object_t *get_object_firmware(atiny_param_t *atiny_params)
{
    /*
     * The get_object_firmware function create the object itself and return a pointer to the structure that represent it.
     */
    lwm2m_object_t *firmwareObj;

    firmwareObj = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != firmwareObj)
    {
        memset(firmwareObj, 0, sizeof(lwm2m_object_t));

        /*
         * It assigns its unique ID
         * The 5 is the standard ID for the optional object "Object firmware".
         */
        firmwareObj->objID = LWM2M_FIRMWARE_UPDATE_OBJECT_ID;

        /*
         * and its unique instance
         *
         */
        firmwareObj->instanceList = (lwm2m_list_t *)lwm2m_malloc(sizeof(lwm2m_list_t));
        if (NULL != firmwareObj->instanceList)
        {
            memset(firmwareObj->instanceList, 0, sizeof(lwm2m_list_t));
        }
        else
        {
            lwm2m_free(firmwareObj);
            return NULL;
        }

        /*
         * And the private function that will access the object.
         * Those function will be called when a read/write/execute query is made by the server. In fact the library don't need to
         * know the resources of the object, only the server does.
         */
        firmwareObj->readFunc    = prv_firmware_read;
        firmwareObj->writeFunc   = prv_firmware_write;
        firmwareObj->executeFunc = prv_firmware_execute;
        firmwareObj->userData    = NULL;

        /*
         * Also some user data can be stored in the object with a private structure containing the needed variables
         */
    }

    return firmwareObj;
}

void free_object_firmware(lwm2m_object_t *objectP)
{
    if (NULL != objectP->userData)
    {
        lwm2m_free(objectP->userData);
        objectP->userData = NULL;
    }
    if (NULL != objectP->instanceList)
    {
        lwm2m_free(objectP->instanceList);
        objectP->instanceList = NULL;
    }
    lwm2m_free(objectP);
}

#else


lwm2m_object_t *get_object_firmware(atiny_param_t *atiny_params)
{
    (void)atiny_params;
    return NULL;
}


void free_object_firmware(lwm2m_object_t *objectP)
{
    (void)objectP;
}

#endif


