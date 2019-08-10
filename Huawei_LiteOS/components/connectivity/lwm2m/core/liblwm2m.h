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
 *    David Navarro, Intel Corporation - initial API and implementation
 *    Fabien Fleutot - Please refer to git log
 *    Simon Bernard - Please refer to git log
 *    Toby Jaffey - Please refer to git log
 *    Julien Vermillard - Please refer to git log
 *    Bosch Software Innovations GmbH - Please refer to git log
 *    Pascal Rieux - Please refer to git log
 *    Ville Skyttä - Please refer to git log
 *
 *******************************************************************************/

/*
 Copyright (c) 2013, 2014 Intel Corporation

 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
     * Neither the name of Intel Corporation nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 THE POSSIBILITY OF SUCH DAMAGE.

 David Navarro <david.navarro@intel.com>

*/

#ifndef _LWM2M_CLIENT_H_
#define _LWM2M_CLIENT_H_

#include "liblwm2m_api.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <time.h>

#include "er-coap-13/er-coap-13.h"
#include "osdepends/atiny_osdep.h"

#ifdef LWM2M_SERVER_MODE
#ifndef LWM2M_SUPPORT_JSON
#define LWM2M_SUPPORT_JSON
#endif
#endif

#if defined(LWM2M_BOOTSTRAP) && defined(LWM2M_BOOTSTRAP_SERVER_MODE)
#error "LWM2M_BOOTSTRAP and LWM2M_BOOTSTRAP_SERVER_MODE cannot be defined at the same time!"
#endif

/*
 * Platform abstraction functions to be implemented by the user
 */

#ifndef LWM2M_MEMORY_TRACE
// Allocate a block of size bytes of memory, returning a pointer to the beginning of the block.
void* lwm2m_malloc(size_t s);
// Deallocate a block of memory previously allocated by lwm2m_malloc() or lwm2m_strdup()
void lwm2m_free(void* p);
// Allocate a memory block, duplicate the string str in it and return a pointer to this new block.
char* lwm2m_strdup(const char* str);
#else
// same functions as above with caller location for debugging purposes
char* lwm2m_trace_strdup(const char* str, const char* file, const char* function, int lineno);
void* lwm2m_trace_malloc(size_t size, const char* file, const char* function, int lineno);
void    lwm2m_trace_free(void* mem, const char* file, const char* function, int lineno);

#define lwm2m_strdup(S) lwm2m_trace_strdup(S, __FILE__, __FUNCTION__, __LINE__)
#define lwm2m_malloc(S) lwm2m_trace_malloc(S, __FILE__, __FUNCTION__, __LINE__)
#define lwm2m_free(M)   lwm2m_trace_free(M, __FILE__, __FUNCTION__, __LINE__)
#endif
// Compare at most the n first bytes of s1 and s2, return 0 if they match
int lwm2m_strncmp(const char* s1, const char* s2, size_t n);
// This function must return the number of seconds elapsed since origin.
// The origin (Epoch, system boot, etc...) does not matter as this
// function is used only to determine the elapsed time since the last
// call to it.
// In case of error, this must return a negative value.
// Per POSIX specifications, time_t is a signed integer.
time_t lwm2m_gettime(void);

//get len of random bytes in output.
int lwm2m_rand(void *output, size_t len);

//delay sometime
void lwm2m_delay(uint32_t second);

//#define LWM2M_WITH_LOGS
#ifdef LWM2M_WITH_LOGS
// Same usage as C89 printf()
#define lwm2m_printf atiny_printf
#endif

// communication layer
#ifdef LWM2M_CLIENT_MODE
// Returns a session handle that MUST uniquely identify a peer.
// secObjInstID: ID of the Securty Object instance to open a connection to
// userData: parameter to lwm2m_init()
// bootstrap_flag: If BootstrapServer
void *lwm2m_connect_server(uint16_t secObjInstID, void *userData, bool isServer);

// Close a session created by lwm2m_connect_server()
// sessionH: session handle identifying the peer (opaque to the core)
// userData: parameter to lwm2m_init()
void lwm2m_close_connection(void* sessionH, void* userData);

#endif
// Send data to a peer
// Returns COAP_NO_ERROR or a COAP_NNN error code
// sessionH: session handle identifying the peer (opaque to the core)
// buffer, length: data to send
// userData: parameter to lwm2m_init()
uint8_t lwm2m_buffer_send(void* sessionH,
                          uint8_t* buffer,
                          size_t length,
                          void* userdata);

// Compare two session handles
// Returns true if the two sessions identify the same peer. false otherwise.
// userData: parameter to lwm2m_init()
bool lwm2m_session_is_equal(void* session1, void* session2, void* userData);

/*
 * Error code
 */

#define COAP_NO_ERROR                   (uint8_t)0x00
#define COAP_IGNORE                     (uint8_t)0x01

#define COAP_201_CREATED                (uint8_t)0x41
#define COAP_202_DELETED                (uint8_t)0x42
#define COAP_204_CHANGED                (uint8_t)0x44
#define COAP_205_CONTENT                (uint8_t)0x45
#define COAP_231_CONTINUE               (uint8_t)0x5F
#define COAP_400_BAD_REQUEST            (uint8_t)0x80
#define COAP_401_UNAUTHORIZED           (uint8_t)0x81
#define COAP_402_BAD_OPTION             (uint8_t)0x82
#define COAP_404_NOT_FOUND              (uint8_t)0x84
#define COAP_405_METHOD_NOT_ALLOWED     (uint8_t)0x85
#define COAP_406_NOT_ACCEPTABLE         (uint8_t)0x86
#define COAP_408_REQ_ENTITY_INCOMPLETE  (uint8_t)0x88
#define COAP_412_PRECONDITION_FAILED    (uint8_t)0x8C
#define COAP_413_ENTITY_TOO_LARGE       (uint8_t)0x8D
#define COAP_500_INTERNAL_SERVER_ERROR  (uint8_t)0xA0
#define COAP_501_NOT_IMPLEMENTED        (uint8_t)0xA1
#define COAP_503_SERVICE_UNAVAILABLE    (uint8_t)0xA3

/*
 * Standard Object IDs
 */
#define LWM2M_SECURITY_OBJECT_ID            0
#define LWM2M_SERVER_OBJECT_ID              1
#define LWM2M_ACL_OBJECT_ID                 2
#define LWM2M_DEVICE_OBJECT_ID              3
#define LWM2M_CONN_MONITOR_OBJECT_ID        4
#define LWM2M_FIRMWARE_UPDATE_OBJECT_ID     5
#define LWM2M_LOCATION_OBJECT_ID            6
#define LWM2M_CONN_STATS_OBJECT_ID          7

/*
 * Resource IDs for the LWM2M Security Object
 */
#define LWM2M_SECURITY_URI_ID                 0
#define LWM2M_SECURITY_BOOTSTRAP_ID           1
#define LWM2M_SECURITY_SECURITY_ID            2
#define LWM2M_SECURITY_PUBLIC_KEY_ID          3
#define LWM2M_SECURITY_SERVER_PUBLIC_KEY_ID   4
#define LWM2M_SECURITY_SECRET_KEY_ID          5
#define LWM2M_SECURITY_SMS_SECURITY_ID        6
#define LWM2M_SECURITY_SMS_KEY_PARAM_ID       7
#define LWM2M_SECURITY_SMS_SECRET_KEY_ID      8
#define LWM2M_SECURITY_SMS_SERVER_NUMBER_ID   9
#define LWM2M_SECURITY_SHORT_SERVER_ID        10
#define LWM2M_SECURITY_HOLD_OFF_ID            11
#define LWM2M_SECURITY_BOOTSTRAP_TIMEOUT_ID   12

/*
 * Resource IDs for the LWM2M Server Object
 */
#define LWM2M_SERVER_SHORT_ID_ID    0
#define LWM2M_SERVER_LIFETIME_ID    1
#define LWM2M_SERVER_MIN_PERIOD_ID  2
#define LWM2M_SERVER_MAX_PERIOD_ID  3
#define LWM2M_SERVER_DISABLE_ID     4
#define LWM2M_SERVER_TIMEOUT_ID     5
#define LWM2M_SERVER_STORING_ID     6
#define LWM2M_SERVER_BINDING_ID     7
#define LWM2M_SERVER_UPDATE_ID      8

#define LWM2M_SECURITY_MODE_PRE_SHARED_KEY  0
#define LWM2M_SECURITY_MODE_RAW_PUBLIC_KEY  1
#define LWM2M_SECURITY_MODE_CERTIFICATE     2
#define LWM2M_SECURITY_MODE_NONE            3

#define LWM2M_TRIGER_SERVER_MODE_INITIATED_TIME 60


#define MAX_FACTORY_BS_RETRY_CNT 3
#define MAX_CLIENT_INITIATED_BS_RETRY_CNT 3
#define FACTORY_BS_DELAY_BASE 0
#define CLIENT_INITIATED_BS_DELAY_BASE 0
#define FACTORY_BS_DELAY_INTERVAL 10
#define CLIENT_INITIATED_BS_DELAY_INTERVAL 10



/*
 * Utility functions for sorted linked list
 */

typedef struct _lwm2m_list_t
{
    struct _lwm2m_list_t* next;
    uint16_t    id;
} lwm2m_list_t;

// defined in list.c
// Add 'node' to the list 'head' and return the new list
lwm2m_list_t* lwm2m_list_add(lwm2m_list_t* head, lwm2m_list_t* node);
// Return the node with ID 'id' from the list 'head' or NULL if not found
lwm2m_list_t* lwm2m_list_find(lwm2m_list_t* head, uint16_t id);
// Remove the node with ID 'id' from the list 'head' and return the new list
lwm2m_list_t* lwm2m_list_remove(lwm2m_list_t* head, uint16_t id, lwm2m_list_t** nodeP);
// Return the lowest unused ID in the list 'head'
uint16_t lwm2m_list_newId(lwm2m_list_t* head);
// Free a list. Do not use if nodes contain allocated pointers as it calls lwm2m_free on nodes only.
// If the nodes of the list need to do more than just "free()" their instances, don't use lwm2m_list_free().
void lwm2m_list_free(lwm2m_list_t* head);

#define LWM2M_LIST_ADD(H,N) lwm2m_list_add((lwm2m_list_t *)H, (lwm2m_list_t *)N);
#define LWM2M_LIST_RM(H,I,N) lwm2m_list_remove((lwm2m_list_t *)H, I, (lwm2m_list_t **)N);
#define LWM2M_LIST_FIND(H,I) lwm2m_list_find((lwm2m_list_t *)H, I)
#define LWM2M_LIST_FREE(H) lwm2m_list_free((lwm2m_list_t *)H)

/*
 * URI
 *
 * objectId is always set
 * instanceId or resourceId are set according to the flag bit-field
 *
 */

#define LWM2M_MAX_ID   ((uint16_t)0xFFFF)

#define LWM2M_URI_FLAG_OBJECT_ID    (uint8_t)0x04
#define LWM2M_URI_FLAG_INSTANCE_ID  (uint8_t)0x02
#define LWM2M_URI_FLAG_RESOURCE_ID  (uint8_t)0x01

#define LWM2M_URI_IS_SET_INSTANCE(uri) (((uri)->flag & LWM2M_URI_FLAG_INSTANCE_ID) != 0)
#define LWM2M_URI_IS_SET_RESOURCE(uri) (((uri)->flag & LWM2M_URI_FLAG_RESOURCE_ID) != 0)

typedef struct
{
    uint8_t     flag;           // indicates which segments are set
    uint16_t    objectId;
    uint16_t    instanceId;
    uint16_t    resourceId;
} lwm2m_uri_t;


#define LWM2M_STRING_ID_MAX_LEN 6

// Parse an URI in LWM2M format and fill the lwm2m_uri_t.
// Return the number of characters read from buffer or 0 in case of error.
// Valid URIs: /1, /1/, /1/2, /1/2/, /1/2/3
// Invalid URIs: /, //, //2, /1//, /1//3, /1/2/3/, /1/2/3/4
int lwm2m_stringToUri(const char* buffer, size_t buffer_len, lwm2m_uri_t* uriP);

/*
 * The lwm2m_data_t is used to store LWM2M resource values in a hierarchical way.
 * Depending on the type the value is different:
 * - LWM2M_TYPE_OBJECT, LWM2M_TYPE_OBJECT_INSTANCE, LWM2M_TYPE_MULTIPLE_RESOURCE: value.asChildren
 * - LWM2M_TYPE_STRING, LWM2M_TYPE_OPAQUE: value.asBuffer
 * - LWM2M_TYPE_INTEGER, LWM2M_TYPE_TIME: value.asInteger
 * - LWM2M_TYPE_FLOAT: value.asFloat
 * - LWM2M_TYPE_BOOLEAN: value.asBoolean
 *
 * LWM2M_TYPE_STRING is also used when the data is in text format.
 */

typedef enum
{
    LWM2M_TYPE_UNDEFINED = 0,
    LWM2M_TYPE_OBJECT,
    LWM2M_TYPE_OBJECT_INSTANCE,
    LWM2M_TYPE_MULTIPLE_RESOURCE,

    LWM2M_TYPE_STRING,
    LWM2M_TYPE_OPAQUE,
    LWM2M_TYPE_INTEGER,
    LWM2M_TYPE_FLOAT,
    LWM2M_TYPE_BOOLEAN,

    LWM2M_TYPE_OBJECT_LINK
} lwm2m_data_type_t;

typedef struct _lwm2m_data_t lwm2m_data_t;

struct _lwm2m_data_t
{
    lwm2m_data_type_t type;
    uint16_t    id;
    union
    {
        bool        asBoolean;
        int64_t     asInteger;
        double      asFloat;
        struct
        {
            size_t    length;
            uint8_t* buffer;
        } asBuffer;
        struct
        {
            size_t         count;
            lwm2m_data_t* array;
        } asChildren;
        struct
        {
            uint16_t objectId;
            uint16_t objectInstanceId;
        } asObjLink;
    } value;
};

typedef void (*lwm2m_data_process) (void*);

typedef struct _lwm2m_data_cfg_t
{
    int type;                     /*Êý¾ÝÉÏ±¨ÀàÐÍ*/
    int cookie;                   /*Êý¾Ýcookie,ÓÃÒÔÔÚack»Øµ÷ÖÐ£¬Çø·Ö²»Í¬µÄÊý¾Ý*/
    lwm2m_data_process callback;  /*ack»Øµ÷*/
} lwm2m_data_cfg_t;

typedef enum
{
    LWM2M_CONTENT_TEXT      = 0,        // Also used as undefined
    LWM2M_CONTENT_LINK      = 40,
    LWM2M_CONTENT_OPAQUE    = 42,
    LWM2M_CONTENT_TLV_OLD   = 1542,     // Keep old value for backward-compatibility
    LWM2M_CONTENT_TLV       = 11542,
    LWM2M_CONTENT_JSON_OLD  = 1543,     // Keep old value for backward-compatibility
    LWM2M_CONTENT_JSON      = 11543
} lwm2m_media_type_t;

lwm2m_data_t* lwm2m_data_new(int size);
int lwm2m_data_parse(lwm2m_uri_t* uriP, uint8_t* buffer, size_t bufferLen, lwm2m_media_type_t format, lwm2m_data_t** dataP);
int lwm2m_data_serialize(lwm2m_uri_t* uriP, int size, lwm2m_data_t* dataP, lwm2m_media_type_t* formatP, uint8_t** bufferP);
void lwm2m_data_free(int size, lwm2m_data_t* dataP);

void lwm2m_data_decode_opaque(const lwm2m_data_t* dataP, int8_t* valueP);
void lwm2m_data_encode_string(const char* string, lwm2m_data_t* dataP);
void lwm2m_data_encode_nstring(const char* string, size_t length, lwm2m_data_t* dataP);
void lwm2m_data_encode_opaque(uint8_t* buffer, size_t length, lwm2m_data_t* dataP);
void lwm2m_data_encode_int(int64_t value, lwm2m_data_t* dataP);
int lwm2m_data_decode_int(const lwm2m_data_t* dataP, int64_t* valueP);
void lwm2m_data_encode_float(double value, lwm2m_data_t* dataP);
int lwm2m_data_decode_float(const lwm2m_data_t* dataP, double* valueP);
void lwm2m_data_encode_bool(bool value, lwm2m_data_t* dataP);
int lwm2m_data_decode_bool(const lwm2m_data_t* dataP, bool* valueP);
void lwm2m_data_encode_objlink(uint16_t objectId, uint16_t objectInstanceId, lwm2m_data_t* dataP);
void lwm2m_data_encode_instances(lwm2m_data_t* subDataP, size_t count, lwm2m_data_t* dataP);
void lwm2m_data_include(lwm2m_data_t* subDataP, size_t count, lwm2m_data_t* dataP);


/*
 * Utility function to parse TLV buffers directly
 *
 * Returned value: number of bytes parsed
 * buffer: buffer to parse
 * buffer_len: length in bytes of buffer
 * oType: (OUT) type of the parsed TLV record. can be:
 *          - LWM2M_TYPE_OBJECT
 *          - LWM2M_TYPE_OBJECT_INSTANCE
 *          - LWM2M_TYPE_MULTIPLE_RESOURCE
 *          - LWM2M_TYPE_OPAQUE
 * oID: (OUT) ID of the parsed TLV record
 * oDataIndex: (OUT) index of the data of the parsed TLV record in the buffer
 * oDataLen: (OUT) length of the data of the parsed TLV record
 */

#define LWM2M_TLV_HEADER_MAX_LENGTH 6

int lwm2m_decode_TLV(const uint8_t* buffer, size_t buffer_len, lwm2m_data_type_t* oType, uint16_t* oID, size_t* oDataIndex, size_t* oDataLen);


/*
 * LWM2M Objects
 *
 * For the read callback, if *numDataP is not zero, *dataArrayP is pre-allocated
 * and contains the list of resources to read.
 *
 */

typedef struct _lwm2m_object_t lwm2m_object_t;

typedef uint8_t (*lwm2m_read_callback_t) (uint16_t instanceId, int* numDataP, lwm2m_data_t** dataArrayP, lwm2m_data_cfg_t* dataCfg, lwm2m_object_t* objectP);
typedef uint8_t (*lwm2m_discover_callback_t) (uint16_t instanceId, int* numDataP, lwm2m_data_t** dataArrayP, lwm2m_object_t* objectP);
typedef uint8_t (*lwm2m_write_callback_t) (uint16_t instanceId, int numData, lwm2m_data_t* dataArray, lwm2m_object_t* objectP);
typedef uint8_t (*lwm2m_execute_callback_t) (uint16_t instanceId, uint16_t resourceId, uint8_t* buffer, int length, lwm2m_object_t* objectP);
typedef uint8_t (*lwm2m_create_callback_t) (uint16_t instanceId, int numData, lwm2m_data_t* dataArray, lwm2m_object_t* objectP);
typedef uint8_t (*lwm2m_delete_callback_t) (uint16_t instanceId, lwm2m_object_t* objectP);
typedef uint8_t (*lwm2m_change_callback_t) (uint16_t instanceId, uint8_t* buffer, int length, lwm2m_object_t* objectP);

struct _lwm2m_object_t
{
    struct _lwm2m_object_t* next;            // for internal use only.
    uint16_t       objID;
    lwm2m_list_t* instanceList;
    lwm2m_read_callback_t     readFunc;
    lwm2m_write_callback_t    writeFunc;
    lwm2m_execute_callback_t  executeFunc;
    lwm2m_change_callback_t   change;
    lwm2m_create_callback_t   createFunc;
    lwm2m_delete_callback_t   deleteFunc;
    lwm2m_discover_callback_t discoverFunc;
    void* userData;
};

/*
 * LWM2M Servers
 *
 * Since LWM2M Server Object instances are not accessible to LWM2M servers,
 * there is no need to store them as lwm2m_objects_t
 */

typedef enum
{
    STATE_DEREGISTERED = 0,        // not registered or boostrap not started
    STATE_REG_PENDING,             // registration pending
    STATE_REGISTERED,              // successfully registered
    STATE_REG_FAILED,              // last registration failed
    STATE_REG_UPDATE_PENDING,      // registration update pending
    STATE_REG_UPDATE_NEEDED,       // registration update required
    STATE_REG_FULL_UPDATE_NEEDED,  // registration update with objects required
    STATE_DEREG_PENDING,           // deregistration pending
    STATE_BS_HOLD_OFF,             // bootstrap hold off time
    STATE_BS_INITIATED,            // bootstrap request sent
    STATE_BS_PENDING,              // boostrap ongoing
    STATE_BS_FINISHING,            // boostrap finish received
    STATE_BS_FINISHED,             // bootstrap done
    STATE_BS_FAILING,              // bootstrap error occurred
    STATE_BS_FAILED,               // bootstrap failed
} lwm2m_status_t;

typedef enum
{
    BINDING_UNKNOWN = 0,
    BINDING_U,   // UDP
    BINDING_UQ,  // UDP queue mode
    BINDING_S,   // SMS
    BINDING_SQ,  // SMS queue mode
    BINDING_US,  // UDP plus SMS
    BINDING_UQS  // UDP queue mode plus SMS
} lwm2m_binding_t;

typedef enum
{
    MODULE_LWM2M,
    MODULE_NET,
    MODULE_URI,
}module_type_t;

enum
{
    OBSERVE_UNSUBSCRIBE,
    OBSERVE_SUBSCRIBE,
};

/*
 * LWM2M block1 data
 *
 * Temporary data needed to handle block1 request.
 * Currently support only one block1 request by server.
 */
typedef struct _lwm2m_block1_data_ lwm2m_block1_data_t;

struct _lwm2m_block1_data_
{
    uint8_t*              block1buffer;     // data buffer
    size_t                block1bufferSize; // buffer size
    uint16_t              lastmid;          // mid of the last message received
};

typedef struct _lwm2m_server_
{
    struct _lwm2m_server_* next;          // matches lwm2m_list_t::next
    uint16_t                secObjInstID; // matches lwm2m_list_t::id
    uint16_t                shortID;      // servers short ID, may be 0 for bootstrap server
    time_t                  lifetime;     // lifetime of the registration in sec or 0 if default value (86400 sec), also used as hold off time for bootstrap servers
    time_t                  registration; // date of the last registration in sec or end of client hold off time for bootstrap servers
    lwm2m_binding_t         binding;      // client connection mode with this server
    void*                   sessionH;
    lwm2m_status_t          status;
    char*                   location;
    bool                    dirty;
    lwm2m_block1_data_t*    block1Data;   // buffer to handle block1 data, should be replace by a list to support several block1 transfer by server.
} lwm2m_server_t;


/*
 * LWM2M result callback
 *
 * When used with an observe, if 'data' is not nil, 'status' holds the observe counter.
 */
typedef void (*lwm2m_result_callback_t) (uint16_t clientID, lwm2m_uri_t* uriP, int status, lwm2m_media_type_t format, uint8_t* data, int dataLength, void* userData);

/*
 * LWM2M Observations
 *
 * Used to store observation of remote clients resources.
 * status STATE_REG_PENDING means the observe request was sent to the client but not yet answered.
 * status STATE_REGISTERED means the client acknowledged the observe request.
 * status STATE_DEREG_PENDING means the user canceled the request before the client answered it.
 */

typedef struct _lwm2m_observation_
{
    struct _lwm2m_observation_* next;   // matches lwm2m_list_t::next
    uint16_t                     id;    // matches lwm2m_list_t::id
    struct _lwm2m_client_* clientP;
    lwm2m_uri_t             uri;
    lwm2m_status_t          status;
    lwm2m_result_callback_t callback;
    void*                   userData;
} lwm2m_observation_t;

/*
 * LWM2M Link Attributes
 *
 * Used for observation parameters.
 *
 */

#define LWM2M_ATTR_FLAG_MIN_PERIOD      (uint8_t)0x01
#define LWM2M_ATTR_FLAG_MAX_PERIOD      (uint8_t)0x02
#define LWM2M_ATTR_FLAG_GREATER_THAN    (uint8_t)0x04
#define LWM2M_ATTR_FLAG_LESS_THAN       (uint8_t)0x08
#define LWM2M_ATTR_FLAG_STEP            (uint8_t)0x10

typedef struct
{
    uint8_t     toSet;
    uint8_t     toClear;
    uint32_t    minPeriod;
    uint32_t    maxPeriod;
    double      greaterThan;
    double      lessThan;
    double      step;
} lwm2m_attributes_t;

/*
 * LWM2M Clients
 *
 * Be careful not to mix lwm2m_client_object_t used to store list of objects of remote clients
 * and lwm2m_object_t describing objects exposed to remote servers.
 *
 */

typedef struct _lwm2m_client_object_
{
    struct _lwm2m_client_object_* next;  // matches lwm2m_list_t::next
    uint16_t                 id;         // matches lwm2m_list_t::id
    lwm2m_list_t*            instanceList;
} lwm2m_client_object_t;

typedef struct _lwm2m_client_
{
    struct _lwm2m_client_* next;        // matches lwm2m_list_t::next
    uint16_t                internalID; // matches lwm2m_list_t::id
    char*                   name;
    lwm2m_binding_t         binding;
    char*                   msisdn;
    char*                   altPath;
    bool                    supportJSON;
    uint32_t                lifetime;
    time_t                  endOfLife;
    void*                   sessionH;
    lwm2m_client_object_t* objectList;
    lwm2m_observation_t*    observationList;
} lwm2m_client_t;


/*
 * LWM2M transaction
 *
 * Adaptation of Erbium's coap_transaction_t
 */

typedef struct _lwm2m_transaction_ lwm2m_transaction_t;

typedef void (*lwm2m_transaction_callback_t) (lwm2m_transaction_t* transacP, void* message);

struct _lwm2m_transaction_
{
    lwm2m_transaction_t* next;   // matches lwm2m_list_t::next
    uint16_t              mID;   // matches lwm2m_list_t::id
    void*                 peerH;
    uint8_t               ack_received; // indicates, that the ACK was received
    time_t                response_timeout; // timeout to wait for response, if token is used. When 0, use calculated acknowledge timeout.
    uint8_t  retrans_counter;
    time_t   retrans_time;
    coap_packet_t* message;
    uint16_t buffer_len;
    uint8_t* buffer;
    lwm2m_data_cfg_t   cfg;
    lwm2m_transaction_callback_t callback;
    void* userData;
};

/*
 * LWM2M observed resources
 */
typedef struct _lwm2m_watcher_
{
    struct _lwm2m_watcher_* next;

    bool active;
    bool update;
    lwm2m_server_t* server;
    lwm2m_attributes_t* parameters;
    lwm2m_media_type_t format;
    uint8_t token[8];
    size_t tokenLen;
    time_t lastTime;
    uint32_t counter;
    uint16_t lastMid;
    union
    {
        int64_t asInteger;
        double  asFloat;
    } lastValue;
} lwm2m_watcher_t;

typedef struct _lwm2m_observed_
{
    struct _lwm2m_observed_* next;

    lwm2m_uri_t uri;
    lwm2m_watcher_t* watcherList;
} lwm2m_observed_t;

#ifdef LWM2M_CLIENT_MODE

typedef enum
{
    STATE_INITIAL = 0,
    STATE_BOOTSTRAP_REQUIRED,
    STATE_BOOTSTRAPPING,
    STATE_REGISTER_REQUIRED,
    STATE_REGISTERING,
    STATE_READY
} lwm2m_client_state_t;

typedef enum
{
    BS_SEQUENCE_STATE_INITIAL = 0,
    BS_SEQUENCE_STATE_FACTORY,
    BS_SEQUENCE_STATE_SERVER_INITIATED,
    BS_SEQUENCE_STATE_CLIENT_INITIATED,
    NO_BS_SEQUENCE_STATE
} lwm2m_bs_sequence_state_t;


#endif
/*
 * LWM2M Context
 */

#ifdef LWM2M_BOOTSTRAP_SERVER_MODE
// In all the following APIs, the session handle MUST uniquely identify a peer.

// LWM2M bootstrap callback
// When a LWM2M client requests bootstrap information, the callback is called with status COAP_NO_ERROR, uriP is nil and
// name is set. The callback must return a COAP_* error code. COAP_204_CHANGED for success.
// After a lwm2m_bootstrap_delete() or a lwm2m_bootstrap_write(), the callback is called with the status returned by the
// client, the URI of the operation (may be nil) and name is nil. The callback return value is ignored.
typedef int (*lwm2m_bootstrap_callback_t) (void* sessionH, uint8_t status, lwm2m_uri_t* uriP, char* name, void* userData);
#endif

#ifdef LWM2M_CLIENT_MODE
/* use to control the bootstrap, factory bootstrap, client initiated bootstrap, server initiated bootstrap.
factory bootstrap is used security object to register.
*/
typedef struct
{
    lwm2m_bootstrap_type_e bsType;
    lwm2m_client_state_t state;
    uint32_t cnt;
}lwm2m_bs_control_t;
#endif

typedef struct _lwm2m_context_t
{
#ifdef LWM2M_CLIENT_MODE
    lwm2m_client_state_t state;
    //char*                bs_server_uri;   //    coaps://     coap://malloc memory
    bool                 regist_first_flag;  //when serverlist and bootstrapServerList are all exist, we use regist or bootstrap.
    char*                endpointName;
    char*                msisdn;
    char*                altPath;
    lwm2m_server_t*      bootstrapServerList;
    lwm2m_server_t*      serverList;
    lwm2m_object_t*      objectList;
    lwm2m_observed_t*    observedList;
    void*                observe_mutex;
    lwm2m_bs_control_t    bsCtrl;
#endif
#ifdef LWM2M_SERVER_MODE
    lwm2m_client_t*         clientList;
    lwm2m_result_callback_t monitorCallback;
    void*                   monitorUserData;
#endif
#ifdef LWM2M_BOOTSTRAP_SERVER_MODE
    lwm2m_bootstrap_callback_t bootstrapCallback;
    void*                      bootstrapUserData;
#endif
    uint16_t                nextMID;
    lwm2m_transaction_t*    transactionList;
    void*                   userData;
} lwm2m_context_t;



// initialize a liblwm2m context.
lwm2m_context_t* lwm2m_init(void* userData);
// close a liblwm2m context.
void lwm2m_close(lwm2m_context_t* contextP);

// perform any required pending operation and adjust timeoutP to the maximal time interval to wait in seconds.
int lwm2m_step(lwm2m_context_t* contextP, time_t* timeoutP);
// dispatch received data to liblwm2m
void lwm2m_handle_packet(lwm2m_context_t* contextP, uint8_t* buffer, int length, void* fromSessionH);

#ifdef LWM2M_CLIENT_MODE
// configure the client side with the Endpoint Name, binding, MSISDN (can be nil), alternative path
// for objects (can be nil) and a list of objects.
// LWM2M Security Object (ID 0) must be present with either a bootstrap server or a LWM2M server and
// its matching LWM2M Server Object (ID 1) instance
int lwm2m_configure(lwm2m_context_t* contextP, const char* endpointName, const char* msisdn, const char* altPath, uint16_t numObject, lwm2m_object_t* objectList[]);
int lwm2m_add_object(lwm2m_context_t* contextP, lwm2m_object_t* objectP);
int lwm2m_remove_object(lwm2m_context_t* contextP, uint16_t id);

// send a registration update to the server specified by the server short identifier
// or all if the ID is 0.
// If withObjects is true, the registration update contains the object list.
int lwm2m_update_registration(lwm2m_context_t* contextP, uint16_t shortServerID, bool withObjects);


void lwm2m_resource_value_changed(lwm2m_context_t* contextP, lwm2m_uri_t* uriP);
void lwm2m_register_observe_ack_call_back(lwm2m_transaction_callback_t callback);
typedef void(*lwm2m_event_handler_t)(module_type_t type, int code, const char* arg, int arg_len);
void lwm2m_register_event_handler(lwm2m_event_handler_t callback);
void lwm2m_notify_even(module_type_t type, int code, const char* arg, int arg_len);
int lwm2m_reconnect(lwm2m_context_t * context);


typedef struct
{
    uint32_t format;
    uint8_t token[8];
    uint32_t tokenLen;
    uint32_t counter;
}lwm2m_observe_info_t;

uint8_t lwm2m_get_observe_info(lwm2m_context_t * contextP, lwm2m_observe_info_t *observe_info);
uint8_t lwm2m_send_notify(lwm2m_context_t * contextP, lwm2m_observe_info_t *observe_info, int firmware_update_state, lwm2m_data_cfg_t  *cfg);

int lwm2m_initBootStrap(lwm2m_context_t *contextP, lwm2m_bootstrap_type_e bsType);


#endif

#ifdef LWM2M_SERVER_MODE
// Clients registration/deregistration monitoring API.
// When a LWM2M client registers, the callback is called with status COAP_201_CREATED.
// When a LWM2M client deregisters, the callback is called with status COAP_202_DELETED.
// clientID is the internal ID of the LWM2M Client.
// The callback's parameters uri, data, dataLength are always NULL.
// The lwm2m_client_t is present in the lwm2m_context_t's clientList when the callback is called. On a deregistration, it deleted when the callback returns.
void lwm2m_set_monitoring_callback(lwm2m_context_t* contextP, lwm2m_result_callback_t callback, void* userData);

// Device Management APIs
int lwm2m_dm_read(lwm2m_context_t* contextP, uint16_t clientID, lwm2m_uri_t* uriP, lwm2m_result_callback_t callback, void* userData);
int lwm2m_dm_discover(lwm2m_context_t* contextP, uint16_t clientID, lwm2m_uri_t* uriP, lwm2m_result_callback_t callback, void* userData);
int lwm2m_dm_write(lwm2m_context_t* contextP, uint16_t clientID, lwm2m_uri_t* uriP, lwm2m_media_type_t format, uint8_t* buffer, int length, lwm2m_result_callback_t callback, void* userData);
int lwm2m_dm_write_attributes(lwm2m_context_t* contextP, uint16_t clientID, lwm2m_uri_t* uriP, lwm2m_attributes_t* attrP, lwm2m_result_callback_t callback, void* userData);
int lwm2m_dm_execute(lwm2m_context_t* contextP, uint16_t clientID, lwm2m_uri_t* uriP, lwm2m_media_type_t format, uint8_t* buffer, int length, lwm2m_result_callback_t callback, void* userData);
int lwm2m_dm_create(lwm2m_context_t* contextP, uint16_t clientID, lwm2m_uri_t* uriP, lwm2m_media_type_t format, uint8_t* buffer, int length, lwm2m_result_callback_t callback, void* userData);
int lwm2m_dm_delete(lwm2m_context_t* contextP, uint16_t clientID, lwm2m_uri_t* uriP, lwm2m_result_callback_t callback, void* userData);

// Information Reporting APIs
int lwm2m_observe(lwm2m_context_t* contextP, uint16_t clientID, lwm2m_uri_t* uriP, lwm2m_result_callback_t callback, void* userData);
int lwm2m_observe_cancel(lwm2m_context_t* contextP, uint16_t clientID, lwm2m_uri_t* uriP, lwm2m_result_callback_t callback, void* userData);
#endif

#ifdef LWM2M_BOOTSTRAP_SERVER_MODE
// Clients bootstrap request monitoring API.
// When a LWM2M client sends a bootstrap request, the callback is called with the client's endpoint name.
void lwm2m_set_bootstrap_callback(lwm2m_context_t* contextP, lwm2m_bootstrap_callback_t callback, void* userData);

// Boostrap Interface APIs
// if uriP is nil, a "Delete /" is sent to the client
int lwm2m_bootstrap_delete(lwm2m_context_t* contextP, void* sessionH, lwm2m_uri_t* uriP);
int lwm2m_bootstrap_write(lwm2m_context_t* contextP, void* sessionH, lwm2m_uri_t* uriP, lwm2m_media_type_t format, uint8_t* buffer, size_t length);
int lwm2m_bootstrap_finish(lwm2m_context_t* contextP, void* sessionH);

#endif

#ifdef __cplusplus
}
#endif

#endif
