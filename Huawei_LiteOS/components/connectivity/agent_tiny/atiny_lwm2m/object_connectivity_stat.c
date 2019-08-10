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
 * Copyright (c) 2015 Bosch Software Innovations GmbH Germany.
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

/*
 * This connectivity statistics object is optional and single instance only
 *
 *  Resources:
 *
 *          Name         | ID | Oper. | Inst. | Mand.|  Type   | Range | Units | Description |
 *  SMS Tx Counter       |  0 |   R   | Single|  No  | Integer |       |       |             |
 *  SMS Rx Counter       |  1 |   R   | Single|  No  | Integer |       |       |             |
 *  Tx Data              |  2 |   R   | Single|  No  | Integer |       | kByte |             |
 *  Rx Data              |  3 |   R   | Single|  No  | Integer |       | kByte |             |
 *  Max Message Size     |  4 |   R   | Single|  No  | Integer |       | Byte  |             |
 *  Average Message Size |  5 |   R   | Single|  No  | Integer |       | Byte  |             |
 *  StartOrReset         |  6 |   E   | Single|  Yes | Integer |       |       |             |
 */

#include "internals.h"

// Resource Id's:
#define RES_O_SMS_TX_COUNTER            0
#define RES_O_SMS_RX_COUNTER            1
#define RES_O_TX_DATA                   2
#define RES_O_RX_DATA                   3
#define RES_O_MAX_MESSAGE_SIZE          4
#define RES_O_AVERAGE_MESSAGE_SIZE      5
#define RES_M_START_OR_RESET            6

typedef struct
{
    int64_t   smsTxCounter;
    int64_t   smsRxCounter;
    int64_t   txDataByte;             // report in kByte!
    int64_t   rxDataByte;             // report in kByte!
    int64_t   maxMessageSize;
    int64_t   avrMessageSize;
    int64_t   messageCount;           // private for incremental average calc.
    bool      collectDataStarted;
} conn_s_data_t;

static uint8_t prv_set_tlv(lwm2m_data_t *dataP, conn_s_data_t *connStDataP)
{
    switch (dataP->id)
    {
    case RES_O_SMS_TX_COUNTER:
        lwm2m_data_encode_int(connStDataP->smsTxCounter, dataP);
        return COAP_205_CONTENT;
    case RES_O_SMS_RX_COUNTER:
        lwm2m_data_encode_int(connStDataP->smsRxCounter, dataP);
        return COAP_205_CONTENT;
    case RES_O_TX_DATA:
        lwm2m_data_encode_int(connStDataP->txDataByte / 1024, dataP);
        return COAP_205_CONTENT;
    case RES_O_RX_DATA:
        lwm2m_data_encode_int(connStDataP->rxDataByte / 1024, dataP);
        return COAP_205_CONTENT;
    case RES_O_MAX_MESSAGE_SIZE:
        lwm2m_data_encode_int(connStDataP->maxMessageSize, dataP);
        return COAP_205_CONTENT;
    case RES_O_AVERAGE_MESSAGE_SIZE:
        lwm2m_data_encode_int(connStDataP->avrMessageSize, dataP);
        return COAP_205_CONTENT;
    default:
        return COAP_404_NOT_FOUND ;
    }
}

static uint8_t prv_read(uint16_t instanceId, int *numDataP, lwm2m_data_t **dataArrayP, lwm2m_data_cfg_t *dataCfg, lwm2m_object_t *objectP)
{
    uint8_t result;
    int i;

    // this is a single instance object
    if (instanceId != 0)
    {
        return COAP_404_NOT_FOUND ;
    }

    // is the server asking for the full object ?
    if (*numDataP == 0)
    {
        uint16_t resList[] =
        {
            RES_O_SMS_TX_COUNTER,
            RES_O_SMS_RX_COUNTER,
            RES_O_TX_DATA,
            RES_O_RX_DATA,
            RES_O_MAX_MESSAGE_SIZE,
            RES_O_AVERAGE_MESSAGE_SIZE
        };
        int nbRes = sizeof(resList) / sizeof(uint16_t);

        *dataArrayP = lwm2m_data_new(nbRes);
        if (*dataArrayP == NULL)
            return COAP_500_INTERNAL_SERVER_ERROR ;
        *numDataP = nbRes;
        for (i = 0; i < nbRes; i++)
        {
            (*dataArrayP)[i].id = resList[i];
        }
    }

    i = 0;
    do
    {
        result = prv_set_tlv((*dataArrayP) + i, (conn_s_data_t *) (objectP->userData));
        i++;
    }
    while (i < *numDataP && result == COAP_205_CONTENT );

    return result;
}

static void prv_resetCounter(lwm2m_object_t *objectP, bool start)
{
    conn_s_data_t *myData = (conn_s_data_t *) objectP->userData;
    myData->smsTxCounter        = 0;
    myData->smsRxCounter        = 0;
    myData->txDataByte          = 0;
    myData->rxDataByte          = 0;
    myData->maxMessageSize      = 0;
    myData->avrMessageSize      = 0;
    myData->messageCount        = 0;
    myData->collectDataStarted  = start;
}

static uint8_t prv_exec(uint16_t instanceId, uint16_t resourceId,
                        uint8_t *buffer, int length, lwm2m_object_t *objectP)
{
    // this is a single instance object
    if (instanceId != 0)
    {
        return COAP_404_NOT_FOUND;
    }

    if (length != 0) return COAP_400_BAD_REQUEST;

    switch (resourceId)
    {
    case RES_M_START_OR_RESET:
        prv_resetCounter(objectP, true);
        return COAP_204_CHANGED;
    default:
        return COAP_405_METHOD_NOT_ALLOWED;
    }
}

void conn_s_updateTxStatistic(lwm2m_object_t *objectP, uint16_t txDataByte, bool smsBased)
{
    conn_s_data_t *myData = (conn_s_data_t *) (objectP->userData);
    if (myData->collectDataStarted)
    {
        myData->txDataByte += txDataByte;
        myData->messageCount++;
        myData->avrMessageSize = (myData->txDataByte + myData->rxDataByte) /
                                 myData->messageCount;
        if (txDataByte > myData->maxMessageSize)
            myData->maxMessageSize = txDataByte;
        if (smsBased) myData->smsTxCounter++;
    }
}

void conn_s_updateRxStatistic(lwm2m_object_t *objectP, uint16_t rxDataByte, bool smsBased)
{
    conn_s_data_t *myData = (conn_s_data_t *) (objectP->userData);
    if (myData->collectDataStarted)
    {
        myData->rxDataByte += rxDataByte;
        myData->messageCount++;
        myData->avrMessageSize = (myData->txDataByte + myData->rxDataByte) /
                                 myData->messageCount;
        if (rxDataByte > myData->maxMessageSize)
            myData->maxMessageSize = rxDataByte;
        myData->txDataByte += rxDataByte;
        if (smsBased) myData->smsRxCounter++;
    }
}


lwm2m_object_t *get_object_conn_s(void)
{
    /*
     * The get_object_conn_s() function create the object itself and return
     * a pointer to the structure that represent it.
     */
    lwm2m_object_t *connObj;

    connObj = (lwm2m_object_t *) lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != connObj)
    {
        memset(connObj, 0, sizeof(lwm2m_object_t));

        /*
         * It assign his unique ID
         * The 7 is the standard ID for the optional object "Connectivity Statistics".
         */
        connObj->objID = LWM2M_CONN_STATS_OBJECT_ID;
        connObj->instanceList = lwm2m_malloc(sizeof(lwm2m_list_t));
        if (NULL != connObj->instanceList)
        {
            memset(connObj->instanceList, 0, sizeof(lwm2m_list_t));
        }
        else
        {
            lwm2m_free(connObj);
            return NULL;
        }

        /*
         * And the private function that will access the object.
         * Those function will be called when a read/execute/close
         * query is made by the server or core. In fact the library don't need
         * to know the resources of the object, only the server does.
         */
        connObj->readFunc     = prv_read;
        connObj->executeFunc  = prv_exec;
        connObj->userData     = lwm2m_malloc(sizeof(conn_s_data_t));

        /*
         * Also some user data can be stored in the object with a private
         * structure containing the needed variables.
         */
        if (NULL != connObj->userData)
        {
            prv_resetCounter(connObj, false);
        }
        else
        {
            lwm2m_free(connObj->instanceList);
            lwm2m_free(connObj);
            connObj = NULL;
        }
    }
    return connObj;
}

void free_object_conn_s(lwm2m_object_t *objectP)
{
    lwm2m_free(objectP->userData);
    lwm2m_list_free(objectP->instanceList);
    lwm2m_free(objectP);
}
