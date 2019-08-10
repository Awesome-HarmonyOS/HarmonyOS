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
 * Copyright (c) 2016 Intel Corporation and others.
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
 *    Simon Bernard - initial API and implementation
 *
 *******************************************************************************/
/*
 Copyright (c) 2016 Intel Corporation

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
*/
#include "internals.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// the maximum payload transferred by block1 we accumulate per server
#define MAX_BLOCK1_SIZE 4096

uint8_t coap_block1_handler(lwm2m_block1_data_t **pBlock1Data,
                            uint16_t mid,
                            uint8_t *buffer,
                            size_t length,
                            uint16_t blockSize,
                            uint32_t blockNum,
                            bool blockMore,
                            uint8_t **outputBuffer,
                            size_t *outputLength)
{
    lwm2m_block1_data_t *block1Data = *pBlock1Data;;

    // manage new block1 transfer
    if (blockNum == 0)
    {
        // we already have block1 data for this server, clear it
        if (block1Data != NULL)
        {
            lwm2m_free(block1Data->block1buffer);
        }
        else
        {
            block1Data = lwm2m_malloc(sizeof(lwm2m_block1_data_t));
            *pBlock1Data = block1Data;
            if (NULL == block1Data) return COAP_500_INTERNAL_SERVER_ERROR;
        }

        block1Data->block1buffer = lwm2m_malloc(length);
        block1Data->block1bufferSize = length;
        if (NULL == block1Data->block1buffer) return COAP_500_INTERNAL_SERVER_ERROR;

        // write new block in buffer
        memcpy(block1Data->block1buffer, buffer, length);
        block1Data->lastmid = mid;
    }
    // manage already started block1 transfer
    else
    {
        if (block1Data == NULL)
        {
            // we never receive the first block
            // TODO should we clean block1 data for this server ?
            return COAP_408_REQ_ENTITY_INCOMPLETE;
        }

        // If this is a retransmission, we already did that.
        if (block1Data->lastmid != mid)
        {
            uint8_t *oldBuffer = block1Data->block1buffer;
            size_t oldSize = block1Data->block1bufferSize;

            if (block1Data->block1bufferSize != blockSize * blockNum)
            {
                // we don't receive block in right order
                // TODO should we clean block1 data for this server ?
                return COAP_408_REQ_ENTITY_INCOMPLETE;
            }

            // is it too large?
            if (block1Data->block1bufferSize + length >= MAX_BLOCK1_SIZE)
            {
                return COAP_413_ENTITY_TOO_LARGE;
            }
            // re-alloc new buffer
            block1Data->block1bufferSize = oldSize + length;
            block1Data->block1buffer = lwm2m_malloc(block1Data->block1bufferSize);
            if (NULL == block1Data->block1buffer) return COAP_500_INTERNAL_SERVER_ERROR;
            memcpy(block1Data->block1buffer, oldBuffer, oldSize);
            lwm2m_free(oldBuffer);

            // write new block in buffer
            memcpy(block1Data->block1buffer + oldSize, buffer, length);
            block1Data->lastmid = mid;
        }
    }

    if (blockMore)
    {
        *outputLength = (size_t) - 1;
        return COAP_231_CONTINUE;
    }
    else
    {
        // buffer is full, set output parameter
        // we don't free it to be able to send retransmission
        *outputLength = block1Data->block1bufferSize;
        *outputBuffer = block1Data->block1buffer;

        return NO_ERROR;
    }
}

void free_block1_buffer(lwm2m_block1_data_t *block1Data)
{
    if (block1Data != NULL)
    {
        // free block1 buffer
        lwm2m_free(block1Data->block1buffer);
        block1Data->block1bufferSize = 0 ;

        // free current element
        lwm2m_free(block1Data);
    }
}
