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

#ifndef _FSL_FLASHIAP_H_
#define _FSL_FLASHIAP_H_

#include "fsl_common.h"

/*!
 * @addtogroup flashiap_driver
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_FLASHIAP_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0. */
                                                            /*@}*/

/*!
 * @brief Flashiap status codes.
 */
enum _flashiap_status
{
    kStatus_FLASHIAP_Success = kStatus_Success,                               /*!< Api is executed successfully */
    kStatus_FLASHIAP_InvalidCommand = MAKE_STATUS(kStatusGroup_FLASHIAP, 1U), /*!< Invalid command */
    kStatus_FLASHIAP_SrcAddrError =
        MAKE_STATUS(kStatusGroup_FLASHIAP, 2U), /*!< Source address is not on word boundary */
    kStatus_FLASHIAP_DstAddrError =
        MAKE_STATUS(kStatusGroup_FLASHIAP, 3U), /*!< Destination address is not on a correct boundary */
    kStatus_FLASHIAP_SrcAddrNotMapped =
        MAKE_STATUS(kStatusGroup_FLASHIAP, 4U), /*!< Source address is not mapped in the memory map */
    kStatus_FLASHIAP_DstAddrNotMapped =
        MAKE_STATUS(kStatusGroup_FLASHIAP, 5U), /*!< Destination address is not mapped in the memory map */
    kStatus_FLASHIAP_CountError =
        MAKE_STATUS(kStatusGroup_FLASHIAP, 6U), /*!< Byte count is not multiple of 4 or is not a permitted value */
    kStatus_FLASHIAP_InvalidSector =
        MAKE_STATUS(kStatusGroup_FLASHIAP,
                    7), /*!< Sector number is invalid or end sector number is greater than start sector number */
    kStatus_FLASHIAP_SectorNotblank = MAKE_STATUS(kStatusGroup_FLASHIAP, 8U), /*!< One or more sectors are not blank */
    kStatus_FLASHIAP_NotPrepared =
        MAKE_STATUS(kStatusGroup_FLASHIAP, 9U), /*!< Command to prepare sector for write operation was not executed */
    kStatus_FLASHIAP_CompareError =
        MAKE_STATUS(kStatusGroup_FLASHIAP, 10U), /*!< Destination and source memory contents do not match */
    kStatus_FLASHIAP_Busy =
        MAKE_STATUS(kStatusGroup_FLASHIAP, 11U), /*!< Flash programming hardware interface is busy */
    kStatus_FLASHIAP_ParamError =
        MAKE_STATUS(kStatusGroup_FLASHIAP, 12U), /*!< Insufficient number of parameters or invalid parameter */
    kStatus_FLASHIAP_AddrError = MAKE_STATUS(kStatusGroup_FLASHIAP, 13U), /*!< Address is not on word boundary */
    kStatus_FLASHIAP_AddrNotMapped =
        MAKE_STATUS(kStatusGroup_FLASHIAP, 14U),                        /*!< Address is not mapped in the memory map */
    kStatus_FLASHIAP_NoPower = MAKE_STATUS(kStatusGroup_FLASHIAP, 24U), /*!< Flash memory block is powered down */
    kStatus_FLASHIAP_NoClock =
        MAKE_STATUS(kStatusGroup_FLASHIAP, 27U), /*!< Flash memory block or controller is not clocked */
};

/*!
 * @brief Flashiap command codes.
 */
enum _flashiap_commands
{
    kIapCmd_FLASHIAP_PrepareSectorforWrite = 50U, /*!< Prepare Sector for write */
    kIapCmd_FLASHIAP_CopyRamToFlash = 51U,        /*!< Copy RAM to flash */
    kIapCmd_FLASHIAP_EraseSector = 52U,           /*!< Erase Sector */
    kIapCmd_FLASHIAP_BlankCheckSector = 53U,      /*!< Blank check sector */
    kIapCmd_FLASHIAP_ReadPartId = 54U,            /*!< Read part id */
    kIapCmd_FLASHIAP_Read_BootromVersion = 55U,   /*!< Read bootrom version */
    kIapCmd_FLASHIAP_Compare = 56U,               /*!< Compare */
    kIapCmd_FLASHIAP_ReinvokeISP = 57U,           /*!< Reinvoke ISP */
    kIapCmd_FLASHIAP_ReadUid = 58U,               /*!< Read Uid isp */
    kIapCmd_FLASHIAP_ErasePage = 59U,             /*!< Erase Page */
    kIapCmd_FLASHIAP_ReadMisr = 70U,              /*!< Read Misr */
    kIapCmd_FLASHIAP_ReinvokeI2cSpiISP = 71U      /*!< Reinvoke I2C/SPI isp */
};

/*! @brief IAP_ENTRY API function type */
typedef void (*IAP_ENTRY_T)(uint32_t cmd[5], uint32_t stat[4]);

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief IAP_ENTRY API function type
 *
 * Wrapper for rom iap call
 *
 * @param cmd_param IAP command and relevant parameter array.
 * @param status_result IAP status result array.
 *
 * @retval None. Status/Result is returned via status_result array.
 */
static inline void iap_entry(uint32_t *cmd_param, uint32_t *status_result)
{
    ((IAP_ENTRY_T)FSL_FEATURE_SYSCON_IAP_ENTRY_LOCATION)(cmd_param, status_result);
}

/*!
 * @brief	Prepare sector for write operation

 * This function prepares sector(s) for write/erase operation. This function must be
 * called before calling the FLASHIAP_CopyRamToFlash() or FLASHIAP_EraseSector() or
 * FLASHIAP_ErasePage() function. The end sector must be greater than or equal to
 * start sector number.
 *
 * @param startSector Start sector number.
 * @param endSector End sector number.
 *
 * @retval #kStatus_FLASHIAP_Success Api was executed successfully.
 * @retval #kStatus_FLASHIAP_NoPower Flash memory block is powered down.
 * @retval #kStatus_FLASHIAP_NoClock Flash memory block or controller is not clocked.
 * @retval #kStatus_FLASHIAP_InvalidSector Sector number is invalid or end sector number
 *         is greater than start sector number.
 * @retval #kStatus_FLASHIAP_Busy Flash programming hardware interface is busy.
 */
status_t FLASHIAP_PrepareSectorForWrite(uint32_t startSector, uint32_t endSector);

/*!
 * @brief	Copy RAM to flash.

 * This function programs the flash memory. Corresponding sectors must be prepared
 * via FLASHIAP_PrepareSectorForWrite before calling calling this function. The addresses
 * should be a 256 byte boundary and the number of bytes should be 256 | 512 | 1024 | 4096.
 *
 * @param dstAddr Destination flash address where data bytes are to be written.
 * @param srcAddr Source ram address from where data bytes are to be read.
 * @param numOfBytes Number of bytes to be written.
 * @param systemCoreClock SystemCoreClock in Hz. It is converted to KHz before calling the
 *                        rom IAP function.
 *
 * @retval #kStatus_FLASHIAP_Success Api was executed successfully.
 * @retval #kStatus_FLASHIAP_NoPower Flash memory block is powered down.
 * @retval #kStatus_FLASHIAP_NoClock Flash memory block or controller is not clocked.
 * @retval #kStatus_FLASHIAP_SrcAddrError Source address is not on word boundary.
 * @retval #kStatus_FLASHIAP_DstAddrError Destination address is not on a correct boundary.
 * @retval #kStatus_FLASHIAP_SrcAddrNotMapped Source address is not mapped in the memory map.
 * @retval #kStatus_FLASHIAP_DstAddrNotMapped Destination address is not mapped in the memory map.
 * @retval #kStatus_FLASHIAP_CountError Byte count is not multiple of 4 or is not a permitted value.
 * @retval #kStatus_FLASHIAP_NotPrepared Command to prepare sector for write operation was not executed.
 * @retval #kStatus_FLASHIAP_Busy Flash programming hardware interface is busy.
 */
status_t FLASHIAP_CopyRamToFlash(uint32_t dstAddr, uint32_t *srcAddr, uint32_t numOfBytes, uint32_t systemCoreClock);

/*!
 * @brief	Erase sector

 * This function erases sector(s). The end sector must be greater than or equal to
 * start sector number. FLASHIAP_PrepareSectorForWrite must be called before
 * calling this function.
 *
 * @param startSector Start sector number.
 * @param endSector End sector number.
 * @param systemCoreClock SystemCoreClock in Hz. It is converted to KHz before calling the
 *                        rom IAP function.
 *
 * @retval #kStatus_FLASHIAP_Success Api was executed successfully.
 * @retval #kStatus_FLASHIAP_NoPower Flash memory block is powered down.
 * @retval #kStatus_FLASHIAP_NoClock Flash memory block or controller is not clocked.
 * @retval #kStatus_FLASHIAP_InvalidSector Sector number is invalid or end sector number
 *         is greater than start sector number.
 * @retval #kStatus_FLASHIAP_NotPrepared Command to prepare sector for write operation was not executed.
 * @retval #kStatus_FLASHIAP_Busy Flash programming hardware interface is busy.
 */
status_t FLASHIAP_EraseSector(uint32_t startSector, uint32_t endSector, uint32_t systemCoreClock);

/*!

 * This function erases page(s). The end page must be greater than or equal to
 * start page number. Corresponding sectors must be prepared via FLASHIAP_PrepareSectorForWrite
 * before calling calling this function.
 *
 * @param startPage Start page number
 * @param endPage End page number
 * @param systemCoreClock SystemCoreClock in Hz. It is converted to KHz before calling the
 *                        rom IAP function.
 *
 * @retval #kStatus_FLASHIAP_Success Api was executed successfully.
 * @retval #kStatus_FLASHIAP_NoPower Flash memory block is powered down.
 * @retval #kStatus_FLASHIAP_NoClock Flash memory block or controller is not clocked.
 * @retval #kStatus_FLASHIAP_InvalidSector Page number is invalid or end page number
 *         is greater than start page number
 * @retval #kStatus_FLASHIAP_NotPrepared Command to prepare sector for write operation was not executed.
 * @retval #kStatus_FLASHIAP_Busy Flash programming hardware interface is busy.
 */
status_t FLASHIAP_ErasePage(uint32_t startPage, uint32_t endPage, uint32_t systemCoreClock);

/*!
 * @brief Blank check sector(s)
 *
 * Blank check single or multiples sectors of flash memory. The end sector must be greater than or equal to
 * start sector number. It can be used to verify the sector eraseure after FLASHIAP_EraseSector call.
 *
 * @param	startSector	: Start sector number. Must be greater than or equal to start sector number
 * @param	endSector	: End sector number
 * @retval #kStatus_FLASHIAP_Success One or more sectors are in erased state.
 * @retval #kStatus_FLASHIAP_NoPower Flash memory block is powered down.
 * @retval #kStatus_FLASHIAP_NoClock Flash memory block or controller is not clocked.
 * @retval #kStatus_FLASHIAP_SectorNotblank One or more sectors are not blank.
 */
status_t FLASHIAP_BlankCheckSector(uint32_t startSector, uint32_t endSector);

/*!
 * @brief Compare memory contents of flash with ram.

 * This function compares the contents of flash and ram. It can be used to verify the flash
 * memory contents after FLASHIAP_CopyRamToFlash call.
 *
 * @param dstAddr Destination flash address.
 * @param srcAddr Source ram address.
 * @param numOfBytes Number of bytes to be compared.
 *
 * @retval #kStatus_FLASHIAP_Success Contents of flash and ram match.
 * @retval #kStatus_FLASHIAP_NoPower Flash memory block is powered down.
 * @retval #kStatus_FLASHIAP_NoClock Flash memory block or controller is not clocked.
 * @retval #kStatus_FLASHIAP_AddrError Address is not on word boundary.
 * @retval #kStatus_FLASHIAP_AddrNotMapped Address is not mapped in the memory map.
 * @retval #kStatus_FLASHIAP_CountError Byte count is not multiple of 4 or is not a permitted value.
 * @retval #kStatus_FLASHIAP_CompareError Destination and source memory contents do not match.
 */
status_t FLASHIAP_Compare(uint32_t dstAddr, uint32_t *srcAddr, uint32_t numOfBytes);

#ifdef __cplusplus
}
#endif

/*@}*/

#endif /* _FSL_FLASHIAP_H_ */
