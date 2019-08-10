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

/*
 * Implements an object for binary_app_data_container purpose
 *
 *                  Multiple
 * Object                      |  ID   | Instances | Mandatoty |
 * binary_app_data_container   |  19   |    Yes    |    No     |
 *
 *  Resources:
 *              Supported    Multiple
 *  Name | ID | Operations | Instances | Mandatory |  Type   | Range | Units | Description |
 *  Data |  0 |    R/W     |    No     |    Yes    | Opaque  |       |       |             |
 *
 */
#include "object_comm.h"
#include "atiny_rpt.h"
#include "log/atiny_log.h"



#define PRV_TLV_BUFFER_SIZE 64

#define MIN_SAVE_CNT 1
#define BINARY_APP_DATA_OBJECT_INSTANCE_NUM 2


/*
 * Multiple instance objects can use userdata to store data that will be shared between the different instances.
 * The lwm2m_object_t object structure - which represent every object of the liblwm2m as seen in the single instance
 * object - contain a chained list called instanceList with the object specific structure plat_instance_t:
 */
typedef struct _prv_instance_
{
    /*
     * The first two are mandatories and represent the pointer to the next instance and the ID of this one. The rest
     * is the instance scope user data (uint8_t test in this case)
     */
    struct _prv_instance_ *next;    // matches lwm2m_list_t::next
    uint16_t shortID;               // matches lwm2m_list_t::id
    rpt_list_t header;
    uint8_t  test;
    double   dec;
#define OPAR_NUM 5
    uint8_t  opaq[OPAR_NUM];
} plat_instance_t;

#define ARRAY_MAXNUM  16
static void prv_output_buffer(uint8_t *buffer,
                              int length)
{
    int i;
    uint8_t array[ARRAY_MAXNUM];

    i = 0;
    while (i < length)
    {
        int j;
        fprintf(stderr, "  ");

        memcpy(array, buffer + i, ARRAY_MAXNUM);

        for (j = 0 ; j < ARRAY_MAXNUM && i + j < length; j++)
        {
            fprintf(stderr, "%02X ", array[j]);
        }
        while (j < ARRAY_MAXNUM)
        {
            fprintf(stderr, "   ");
            j++;
        }
        fprintf(stderr, "  ");
        for (j = 0 ; j < ARRAY_MAXNUM && i + j < length; j++)
        {
            if (isprint(array[j]))
                fprintf(stderr, "%c ", array[j]);
            else
                fprintf(stderr, ". ");
        }
        fprintf(stderr, "\n");

        i += ARRAY_MAXNUM;
    }
}

static uint8_t prv_read_data(plat_instance_t *targetP,
                             int data_number,
                             lwm2m_data_t *dataArrayP,
                             lwm2m_data_cfg_t *dataCfg)
{
    int i;
    data_report_t data;
    int ret;

    for (i = 0 ; i < data_number ; i++)
    {
        switch (dataArrayP[i].id)
        {
        case 0:
            //printf("19/0/0 read\r\n");
            if(NULL == dataCfg)
            {
                uint8_t *buf = lwm2m_malloc(sizeof(*buf));
                if(NULL == buf)
                {
                    ATINY_LOG(LOG_ERR, "malloc null");
                    return COAP_404_NOT_FOUND;
                }
                *buf = 0;
                dataArrayP[i].id = 0;
                dataArrayP[i].type = LWM2M_TYPE_OPAQUE;
                dataArrayP[i].value.asBuffer.buffer = buf;
                dataArrayP[i].value.asBuffer.length = 1;
            }
            else
            {
                ret = atiny_dequeue_rpt_data((targetP->header), &data);
                if (ret != ATINY_OK)
                {
                    ATINY_LOG(LOG_INFO, "atiny_dequeue_rpt_data fail,ret=%d", ret);
                    return COAP_404_NOT_FOUND;
                }
                else
                {
                    ATINY_LOG(LOG_DEBUG, "atiny_dequeue_rpt_data sucessfully");
                }

                dataArrayP[i].id = 0;
                dataArrayP[i].type = LWM2M_TYPE_OPAQUE;
                dataArrayP[i].value.asBuffer.buffer = data.buf;
                dataArrayP[i].value.asBuffer.length = data.len;
                dataCfg->type = data.type;
                dataCfg->cookie = data.cookie;
                dataCfg->callback = (lwm2m_data_process)data.callback;
            }
            break;
        default:
            return COAP_404_NOT_FOUND;
        }
    }

    return COAP_205_CONTENT;
}

static void prv_destroy_data_buf(int data_number, lwm2m_data_t *data_array)
{
    int i;
    for (i = 0 ; i < data_number; i++)
    {
        switch (data_array[i].id)
        {
        case 0:
            if(data_array[i].value.asBuffer.buffer != NULL)
            {
                lwm2m_free(data_array[i].value.asBuffer.buffer);
                data_array[i].value.asBuffer.buffer = NULL;
                data_array[i].value.asBuffer.length = 0;
            }
            break;
        default:
            break;
        }

    }

}

static uint8_t prv_read(uint16_t instanceId,
                        int *numDataP,
                        lwm2m_data_t **dataArrayP,
                        lwm2m_data_cfg_t *dataCfg,
                        lwm2m_object_t *objectP)
{
    plat_instance_t *targetP;
    uint8_t ret;
    int new_data_flag = false;


    targetP = (plat_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP)
    {
        ATINY_LOG(LOG_ERR, "plat inst not found %d", instanceId);
        return COAP_404_NOT_FOUND;
    }

    if (*numDataP == 0)
    {
        *dataArrayP = lwm2m_data_new(1);
        if (*dataArrayP == NULL)
        {
            ATINY_LOG(LOG_ERR, "lwm2m_data_new null");
            return COAP_500_INTERNAL_SERVER_ERROR;
        }

        *numDataP = 1;
        (*dataArrayP)[0].id = 0;
        new_data_flag = true;
    }

    ret = prv_read_data(targetP, *numDataP, *dataArrayP, dataCfg);
    if(ret != COAP_205_CONTENT)
    {

        prv_destroy_data_buf(*numDataP, *dataArrayP);
        if(new_data_flag)
        {
            lwm2m_free(*dataArrayP);
            *dataArrayP = NULL;
        }
    }

    return ret;
}

static uint8_t prv_discover(uint16_t instanceId,
                            int *numDataP,
                            lwm2m_data_t **dataArrayP,
                            lwm2m_object_t *objectP)
{
    int i;

    // is the server asking for the full object ?
    if (*numDataP == 0)
    {
        *dataArrayP = lwm2m_data_new(3);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 3;
        (*dataArrayP)[0].id = 1;
        (*dataArrayP)[1].id = 2;
        (*dataArrayP)[2].id = 3;
    }
    else
    {
        for (i = 0; i < *numDataP; i++)
        {
            switch ((*dataArrayP)[i].id)
            {
            case 1:
            case 2:
            case 3:
                break;
            default:
                return COAP_404_NOT_FOUND;
            }
        }
    }

    return COAP_205_CONTENT;
}

static uint8_t prv_write(uint16_t instanceId,
                         int numData,
                         lwm2m_data_t *dataArray,
                         lwm2m_object_t *objectP)
{
    plat_instance_t *targetP;
    int i;

    targetP = (plat_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    for (i = 0 ; i < numData ; i++)
    {
        switch (dataArray[i].id)
        {
        case 0:
        {
            (void)atiny_cmd_ioctl(ATINY_WRITE_APP_DATA, (char *)(dataArray[i].value.asBuffer.buffer), dataArray->value.asBuffer.length);
            break;
        }
        default:
            return COAP_404_NOT_FOUND;
        }
    }

    return COAP_204_CHANGED;
}


static uint8_t prv_delete(uint16_t id,
                          lwm2m_object_t *objectP)
{
    plat_instance_t *targetP;
    lwm2m_uri_t uri;

    objectP->instanceList = lwm2m_list_remove(objectP->instanceList, id, (lwm2m_list_t **)&targetP);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    get_resource_uri(objectP->objID,  targetP->shortID,  BINARY_APP_DATA_RES_ID, &uri);
    (void)atiny_rm_rpt_uri(&uri);
    lwm2m_free(targetP);

    return COAP_202_DELETED;
}

static uint8_t prv_create(uint16_t instanceId,
                          int numData,
                          lwm2m_data_t *dataArray,
                          lwm2m_object_t *objectP)
{
    plat_instance_t *targetP;
    uint8_t result;
    int ret;
    lwm2m_uri_t uri;


    targetP = (plat_instance_t *)lwm2m_malloc(sizeof(plat_instance_t));
    if (NULL == targetP) return COAP_500_INTERNAL_SERVER_ERROR;
    memset(targetP, 0, sizeof(plat_instance_t));
    //atiny_list_init(&(targetP->header));

    //TODO: if instanceId not valid
    get_resource_uri(BINARY_APP_DATA_OBJECT_ID, instanceId, BINARY_APP_DATA_RES_ID, &uri);
    ret = atiny_add_rpt_uri(&uri, &targetP->header);
    if(ret != ATINY_OK)
    {
        ATINY_LOG(LOG_ERR, "atiny_add_rpt_uri fail %d", ret);
        lwm2m_free(targetP);
        return COAP_404_NOT_FOUND;
    }

    targetP->shortID = instanceId;
    objectP->instanceList = LWM2M_LIST_ADD(objectP->instanceList, targetP);

    result = prv_write(instanceId, numData, dataArray, objectP);

    if (result != COAP_204_CHANGED)
    {
        (void)prv_delete(instanceId, objectP);
    }
    else
    {
        result = COAP_201_CREATED;
    }

    return result;
}

static uint8_t prv_exec(uint16_t instanceId,
                        uint16_t resourceId,
                        uint8_t *buffer,
                        int length,
                        lwm2m_object_t *objectP)
{

    if (NULL == lwm2m_list_find(objectP->instanceList, instanceId)) return COAP_404_NOT_FOUND;

    switch (resourceId)
    {
    case 0:
    {
        ATINY_LOG(LOG_INFO, "no in prv_exec+++++++++++++++++++++++++++");
        return COAP_204_CHANGED;
    }
    case 1:
        return COAP_405_METHOD_NOT_ALLOWED;
    case 2:
        fprintf(stdout, "\r\n-----------------\r\n"
                "Execute on %hu/%d/%d\r\n"
                " Parameter (%d bytes):\r\n",
                objectP->objID, instanceId, resourceId, length);
        prv_output_buffer((uint8_t *)buffer, length);
        fprintf(stdout, "-----------------\r\n\r\n");
        return COAP_204_CHANGED;
    case 3:
        return COAP_405_METHOD_NOT_ALLOWED;
    default:
        return COAP_404_NOT_FOUND;
    }
}

void display_binary_app_data_object(lwm2m_object_t *object)
{
#ifdef WITH_LOGS
    fprintf(stdout, "  /%u: Test object, instances:\r\n", object->objID);
    plat_instance_t *instance = (plat_instance_t *)object->instanceList;
    while (instance != NULL)
    {
        fprintf(stdout, "    /%u/%u: shortId: %u, test: %u\r\n",
                object->objID, instance->shortID,
                instance->shortID, instance->test);
        instance = (plat_instance_t *)instance->next;
    }
#endif
}

lwm2m_object_t *get_binary_app_data_object(atiny_param_t *atiny_params)
{
    lwm2m_object_t *appObj;

    appObj = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != appObj)
    {
        int i;
        plat_instance_t *targetP;
        lwm2m_uri_t uri;
        int ret = ATINY_OK;

        memset(appObj, 0, sizeof(lwm2m_object_t));

        appObj->objID = BINARY_APP_DATA_OBJECT_ID;
        for (i = 0 ; i < BINARY_APP_DATA_OBJECT_INSTANCE_NUM; i++)
        {
            targetP = (plat_instance_t *)lwm2m_malloc(sizeof(plat_instance_t));
            if (NULL == targetP)
            {
                break;
            }
            memset(targetP, 0, sizeof(plat_instance_t));
            get_resource_uri(BINARY_APP_DATA_OBJECT_ID, i, BINARY_APP_DATA_RES_ID, &uri);
            ret = atiny_add_rpt_uri(&uri, &targetP->header);
            if(ret != ATINY_OK)
            {
                ATINY_LOG(LOG_ERR, "atiny_add_rpt_uri fail %d", ret);
                lwm2m_free(targetP);
                break;
            }
            (void)atiny_set_max_rpt_cnt(&uri, MAX(MIN_SAVE_CNT, atiny_params->server_params.storing_cnt));
            targetP->shortID = i;
            appObj->instanceList = LWM2M_LIST_ADD(appObj->instanceList, targetP);
        }

        if(i < BINARY_APP_DATA_OBJECT_INSTANCE_NUM)
        {
            free_binary_app_data_object(appObj);
            return NULL;
        }

        /*
         * From a single instance object, two more functions are available.
         * - The first one (createFunc) create a new instance and filled it with the provided informations. If an ID is
         *   provided a check is done for verifying his disponibility, or a new one is generated.
         * - The other one (deleteFunc) delete an instance by removing it from the instance list (and freeing the memory
         *   allocated to it)
         */
        appObj->readFunc = prv_read;
        appObj->discoverFunc = prv_discover;
        appObj->writeFunc = prv_write;
        appObj->executeFunc = prv_exec;
        appObj->createFunc = prv_create;
        appObj->deleteFunc = prv_delete;
    }

    return appObj;
}

static void free_binary_app_data_object_rpt(lwm2m_object_t *object)
{
    lwm2m_list_t *cur = object->instanceList;
    lwm2m_uri_t uri;
    while(cur)
    {
        get_resource_uri(object->objID, ((plat_instance_t *)cur)->shortID, BINARY_APP_DATA_RES_ID, &uri);
        (void)atiny_rm_rpt_uri(&uri);
        cur = cur->next;
    }
}

void free_binary_app_data_object(lwm2m_object_t *object)
{
    free_binary_app_data_object_rpt(object);
    LWM2M_LIST_FREE(object->instanceList);
    if (object->userData != NULL)
    {
        lwm2m_free(object->userData);
        object->userData = NULL;
    }
    lwm2m_free(object);
}

void set_binary_app_data_object_rpt_max_cnt(uint32_t max_rpt_cnt)
{
    lwm2m_uri_t uri;

    get_resource_uri(BINARY_APP_DATA_OBJECT_ID, 0, BINARY_APP_DATA_RES_ID, &uri);
    (void)atiny_set_max_rpt_cnt(&uri, MAX(MIN_SAVE_CNT, max_rpt_cnt));
}


