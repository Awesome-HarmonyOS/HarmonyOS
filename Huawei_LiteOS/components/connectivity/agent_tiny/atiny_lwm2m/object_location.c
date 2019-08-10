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
 *
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
 *    Pascal Rieux - Please refer to git log
 *
 ******************************************************************************/
/*! \file
  LWM2M object "Location" implementation

  \author Joerg Hubschneider
*/

/*
 *  Object     |      | Multiple  |     | Description                   |
 *  Name       |  ID  | Instances |Mand.|                               |
 *-------------+------+-----------+-----+-------------------------------+
 *  Location   |   6  |    No     |  No |  see TS E.7 page 101          |
 *
 *  Resources:
 *  Name        | ID  | Oper.|Instances|Mand.|  Type   | Range | Units | Description                                                                      |
 * -------------+-----+------+---------+-----+---------+-------+-------+----------------------------------------------------------------------------------+
 *  Latitude    |  0  |  R   | Single  | Yes | Float   |       |  Deg  | The decimal notation of latitude  e.g. -  45.5723  [Worlds Geodetic System 1984].|
 *  Longitude   |  1  |  R   | Single  | Yes | Float   |       |  Deg  | The decimal notation of longitude e.g. - 153.21760 [Worlds Geodetic System 1984].|
 *  Altitude    |  2  |  R   | Single  | No  | Float   |       |   m   | The decimal notation of altitude in meters above sea level.                      |
 *  Radius      |  3  |  R   | Single  | No  | Float   |       |   m   | The value in the Radius Resource indicates the size in meters of a circular area |
 *              |     |      |         |     |         |       |       | around a point of geometry.                                                      |
 *  Velocity    |  4  |  R   | Single  | No  | Opaque  |       |   *   | The velocity of the device as defined in 3GPP 23.032 GAD specification(*).       |
 *              |     |      |         |     |         |       |       | This set of values may not be available if the device is static.                 |
 *              |     |      |         |     |         |       |       | opaque: see OMA_TS 6.3.2                                                         |
 *  Timestamp   |  5  |  R   | Single  | Yes | Time    |       |   s   | The timestamp when the location measurement was performed.                       |
 *  Speed       |  6  |  R   | Single  | No  | Float   |       |  m/s  | Speed is the time rate of change in position of a LwM2M Client without regard    |
 *              |     |      |         |     |         |       |       | for direction: the scalar component of velocity.                                 |
 */

#include "internals.h"
#include "object_comm.h"
#ifdef LWM2M_CLIENT_MODE


// ---- private "object location" specific defines ----
// Resource Id's:
#define RES_M_LATITUDE     0
#define RES_M_LONGITUDE    1
#define RES_O_ALTITUDE     2
#define RES_O_RADIUS       3
#define RES_O_VELOCITY     4
#define RES_M_TIMESTAMP    5
#define RES_O_SPEED        6

/**
implementation for all read-able resources
*/
static uint8_t prv_res2tlv(lwm2m_data_t *dataP)
{
    //-------------------------------------------------------------------- JH --
    uint8_t ret = COAP_205_CONTENT;
    float get_value;
    uint64_t timestamp;
    atiny_velocity_s velocity;

    switch (dataP->id)     // location resourceId
    {
    case RES_M_LATITUDE:
        if (atiny_cmd_ioctl(ATINY_GET_LATITUDE, (char *)&get_value, sizeof(float)) == ATINY_OK)
        {
            lwm2m_data_encode_float(get_value, dataP);
        }
        else
        {
            ret = COAP_400_BAD_REQUEST;
        }
        break;
    case RES_M_LONGITUDE:
        if (atiny_cmd_ioctl(ATINY_GET_LONGITUDE, (char *)&get_value, sizeof(float)) == ATINY_OK)
        {
            lwm2m_data_encode_float(get_value, dataP);
        }
        else
        {
            ret = COAP_400_BAD_REQUEST;
        }
        break;
    case RES_O_ALTITUDE:
        if (atiny_cmd_ioctl(ATINY_GET_ALTITUDE, (char *)&get_value, sizeof(float)) == ATINY_OK)
        {
            lwm2m_data_encode_float(get_value, dataP);
        }
        else
        {
            ret = COAP_400_BAD_REQUEST;
        }
        break;
    case RES_O_RADIUS:
        if (atiny_cmd_ioctl(ATINY_GET_RADIUS, (char *)&get_value, sizeof(float)) == ATINY_OK)
        {
            lwm2m_data_encode_float(get_value, dataP);
        }
        else
        {
            ret = COAP_400_BAD_REQUEST;
        }
        break;
    case RES_O_VELOCITY:
        if (atiny_cmd_ioctl(ATINY_GET_VELOCITY, (char *)&velocity, sizeof(velocity)) == ATINY_OK)
        {
            lwm2m_data_encode_opaque(velocity.opaque, velocity.length, dataP);
        }
        else
        {
            ret = COAP_400_BAD_REQUEST;
        }
        break;
    case RES_M_TIMESTAMP:
        if (atiny_cmd_ioctl(ATINY_GET_TIMESTAMP, (char *)&timestamp, sizeof(uint64_t)) == ATINY_OK)
        {
            lwm2m_data_encode_float(timestamp, dataP);
        }
        else
        {
            ret = COAP_400_BAD_REQUEST;
        }
        break;
    case RES_O_SPEED:
        if (atiny_cmd_ioctl(ATINY_GET_SPEED, (char *)&get_value, sizeof(float)) == ATINY_OK)
        {
            lwm2m_data_encode_float(get_value, dataP);
        }
        else
        {
            ret = COAP_400_BAD_REQUEST;
        }
        break;
    default:
        ret = COAP_404_NOT_FOUND;
        break;
    }

    return ret;
}


/**
  * Implementation (callback-) function of reading object resources. For whole
  * object, single resources or a sequence of resources
  * see 3GPP TS 23.032 V11.0.0(2012-09) page 23,24.
  * implemented for: HORIZONTAL_VELOCITY_WITH_UNCERTAINT
  * @param objInstId    in,     instances ID of the location object to read
  * @param numDataP     in/out, pointer to the number of resource to read. 0 is the
  *                             exception for all readable resource of object instance
  * @param tlvArrayP    in/out, TLV data sequence with initialized resource ID to read
  * @param objectP      in,     private location data structure
  */
static uint8_t prv_location_read(uint16_t objInstId,
                                 int  *numDataP,
                                 lwm2m_data_t **tlvArrayP,
                                 lwm2m_data_cfg_t *dataCfg,
                                 lwm2m_object_t  *objectP)
{
    //-------------------------------------------------------------------- JH --
    int     i;
    uint8_t result = COAP_500_INTERNAL_SERVER_ERROR;

    // defined as single instance object!
    if (objInstId != 0) return COAP_404_NOT_FOUND;

    if (*numDataP == 0)     // full object, readable resources!
    {
        uint16_t readResIds[] =
        {
            RES_M_LATITUDE,
            RES_M_LONGITUDE,
            RES_O_ALTITUDE,
            RES_O_RADIUS,
            RES_O_VELOCITY,
            RES_M_TIMESTAMP,
            RES_O_SPEED
        }; // readable resources!

        *numDataP  = sizeof(readResIds) / sizeof(uint16_t);
        *tlvArrayP = lwm2m_data_new(*numDataP);
        if (*tlvArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;

        // init readable resource id's
        for (i = 0 ; i < *numDataP ; i++)
        {
            (*tlvArrayP)[i].id = readResIds[i];
        }
    }

    for (i = 0 ; i < *numDataP ; i++)
    {
        result = prv_res2tlv ((*tlvArrayP) + i);
        if (result != COAP_205_CONTENT) break;
    }

    return result;
}

void display_location_object(lwm2m_object_t *object)
{
#ifdef WITH_LOGS
    float latitude = 0.0f;
    float longitude = 0.0f;
    float altitude = 0.0f;
    float radius = 0.0f;
    float speed = 0.0f;
    uint64_t timestamp = 0U;

    (void)atiny_cmd_ioctl(ATINY_GET_LATITUDE, &latitude, sizeof(float));
    (void)atiny_cmd_ioctl(ATINY_GET_LONGITUDE, &longitude, sizeof(float));
    (void)atiny_cmd_ioctl(ATINY_GET_ALTITUDE, &altitude, sizeof(float));
    (void)atiny_cmd_ioctl(ATINY_GET_RADIUS, &radius, sizeof(float));
    (void)atiny_cmd_ioctl(ATINY_GET_SPEED, &speed, sizeof(float));
    (void)atiny_cmd_ioctl(ATINY_GET_TIMESTAMP, &timestamp, sizeof(uint64_t));

    fprintf(stdout, "  /%u: Location object:\r\n", object->objID);
    fprintf(stdout, "    latitude: %.6f, longitude: %.6f, altitude: %.6f, radius: %.6f, timestamp: %lu, speed: %.6f\r\n",
            latitude, longitude, altitude, radius, timestamp, speed);

#endif
}

/**
  * This function creates the LWM2M Location.
  * @return gives back allocated LWM2M data object structure pointer. On error,
  * NULL value is returned.
  */
lwm2m_object_t *get_object_location(void)
{
    //-------------------------------------------------------------------- JH --
    lwm2m_object_t *locationObj;

    locationObj = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));
    if (NULL != locationObj)
    {
        memset(locationObj, 0, sizeof(lwm2m_object_t));

        // It assigns its unique ID
        // The 6 is the standard ID for the optional object "Location".
        locationObj->objID = LWM2M_LOCATION_OBJECT_ID;

        // and its unique instance
        locationObj->instanceList = (lwm2m_list_t *)lwm2m_malloc(sizeof(lwm2m_list_t));
        if (NULL != locationObj->instanceList)
        {
            memset(locationObj->instanceList, 0, sizeof(lwm2m_list_t));
        }
        else
        {
            lwm2m_free(locationObj);
            return NULL;
        }

        // And the private function that will access the object.
        // Those function will be called when a read query is made by the server.
        // In fact the library don't need to know the resources of the object, only the server does.
        //
        locationObj->readFunc    = prv_location_read;
    }

    return locationObj;
}

void free_object_location(lwm2m_object_t *object)
{
    lwm2m_list_free(object->instanceList);
    lwm2m_free(object->userData);
    lwm2m_free(object);
}

#endif  //LWM2M_CLIENT_MODE
