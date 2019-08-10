/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_flashiap.h"

#define HZ_TO_KHZ_DIV 1000

/*******************************************************************************
 * Code
 ******************************************************************************/

static status_t translate_iap_status(uint32_t status)
{
    /* Translate IAP return code to sdk status code */
    if (status == kStatus_Success)
    {
        return status;
    }
    else
    {
        return MAKE_STATUS(kStatusGroup_FLASHIAP, status);
    }
}

status_t FLASHIAP_PrepareSectorForWrite(uint32_t startSector, uint32_t endSector)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_PrepareSectorforWrite;
    command[1] = startSector;
    command[2] = endSector;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_CopyRamToFlash(uint32_t dstAddr, uint32_t *srcAddr, uint32_t numOfBytes, uint32_t systemCoreClock)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_CopyRamToFlash;
    command[1] = dstAddr;
    command[2] = (uint32_t)srcAddr;
    command[3] = numOfBytes;
    command[4] = systemCoreClock / HZ_TO_KHZ_DIV;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_EraseSector(uint32_t startSector, uint32_t endSector, uint32_t systemCoreClock)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_EraseSector;
    command[1] = startSector;
    command[2] = endSector;
    command[3] = systemCoreClock / HZ_TO_KHZ_DIV;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_ErasePage(uint32_t startPage, uint32_t endPage, uint32_t systemCoreClock)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_ErasePage;
    command[1] = startPage;
    command[2] = endPage;
    command[3] = systemCoreClock / HZ_TO_KHZ_DIV;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_BlankCheckSector(uint32_t startSector, uint32_t endSector)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_BlankCheckSector;
    command[1] = startSector;
    command[2] = endSector;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_Compare(uint32_t dstAddr, uint32_t *srcAddr, uint32_t numOfBytes)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_Compare;
    command[1] = dstAddr;
    command[2] = (uint32_t)srcAddr;
    command[3] = numOfBytes;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}
