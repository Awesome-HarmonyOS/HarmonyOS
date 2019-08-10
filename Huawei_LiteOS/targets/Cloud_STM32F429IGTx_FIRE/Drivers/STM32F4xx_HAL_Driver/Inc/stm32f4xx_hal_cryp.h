/**
  ******************************************************************************
  * @file    stm32f4xx_hal_cryp.h
  * @author  MCD Application Team
  * @brief   Header file of CRYP HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_HAL_CRYP_H
#define __STM32F4xx_HAL_CRYP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal_def.h"


#if defined(CRYP)

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @addtogroup CRYP
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup CRYP_Exported_Types CRYP Exported Types
  * @{
  */

/** @defgroup CRYP_Exported_Types_Group1 CRYP Configuration Structure definition
  * @{
  */

typedef struct
{
  uint32_t DataType;    /*!< 32-bit data, 16-bit data, 8-bit data or 1-bit string.
                             This parameter can be a value of @ref CRYP_Data_Type */

  uint32_t KeySize;     /*!< Used only in AES mode only : 128, 192 or 256 bit key length.
                             This parameter can be a value of @ref CRYP_Key_Size */

  uint8_t* pKey;        /*!< The key used for encryption/decryption */

  uint8_t* pInitVect;   /*!< The initialization vector used also as initialization
                             counter in CTR mode */

  uint8_t IVSize;       /*!< The size of initialization vector.
                             This parameter (called nonce size in CCM) is used only
                             in AES-128/192/256 encryption/decryption CCM mode */

  uint8_t TagSize;      /*!< The size of returned authentication TAG.
                             This parameter is used only in AES-128/192/256
                             encryption/decryption CCM mode */

  uint8_t* Header;      /*!< The header used in GCM and CCM modes */

  uint32_t HeaderSize;  /*!< The size of header buffer in bytes */

  uint8_t* pScratch;    /*!< Scratch buffer used to append the header. It's size must be equal to header size + 21 bytes.
                             This parameter is used only in AES-128/192/256 encryption/decryption CCM mode */
}CRYP_InitTypeDef;

/**
  * @}
  */

/** @defgroup CRYP_Exported_Types_Group2 CRYP State structures definition
  * @{
  */


typedef enum
{
  HAL_CRYP_STATE_RESET             = 0x00U,  /*!< CRYP not yet initialized or disabled  */
  HAL_CRYP_STATE_READY             = 0x01U,  /*!< CRYP initialized and ready for use    */
  HAL_CRYP_STATE_BUSY              = 0x02U,  /*!< CRYP internal processing is ongoing   */
  HAL_CRYP_STATE_TIMEOUT           = 0x03U,  /*!< CRYP timeout state                    */
  HAL_CRYP_STATE_ERROR             = 0x04U   /*!< CRYP error state                      */
}HAL_CRYP_STATETypeDef;

/**
  * @}
  */

/** @defgroup CRYP_Exported_Types_Group3 CRYP phase structures definition
  * @{
  */


typedef enum
{
  HAL_CRYP_PHASE_READY             = 0x01U,    /*!< CRYP peripheral is ready for initialization. */
  HAL_CRYP_PHASE_PROCESS           = 0x02U,    /*!< CRYP peripheral is in processing phase */
  HAL_CRYP_PHASE_FINAL             = 0x03U     /*!< CRYP peripheral is in final phase
                                                    This is relevant only with CCM and GCM modes */
}HAL_PhaseTypeDef;

/**
  * @}
  */

/** @defgroup CRYP_Exported_Types_Group4 CRYP handle Structure definition
  * @{
  */

typedef struct
{
      CRYP_TypeDef             *Instance;        /*!< CRYP registers base address */

      CRYP_InitTypeDef         Init;             /*!< CRYP required parameters */

      uint8_t                  *pCrypInBuffPtr;  /*!< Pointer to CRYP processing (encryption, decryption,...) buffer */

      uint8_t                  *pCrypOutBuffPtr; /*!< Pointer to CRYP processing (encryption, decryption,...) buffer */

      __IO uint16_t            CrypInCount;      /*!< Counter of inputed data */

      __IO uint16_t            CrypOutCount;     /*!< Counter of output data */

      HAL_StatusTypeDef        Status;           /*!< CRYP peripheral status */

      HAL_PhaseTypeDef         Phase;            /*!< CRYP peripheral phase */

      DMA_HandleTypeDef        *hdmain;          /*!< CRYP In DMA handle parameters */

      DMA_HandleTypeDef        *hdmaout;         /*!< CRYP Out DMA handle parameters */

      HAL_LockTypeDef          Lock;             /*!< CRYP locking object */

   __IO  HAL_CRYP_STATETypeDef State;            /*!< CRYP peripheral state */
}CRYP_HandleTypeDef;

/**
  * @}
  */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup CRYP_Exported_Constants CRYP Exported Constants
  * @{
  */

/** @defgroup CRYP_Key_Size CRYP Key Size
  * @{
  */
#define CRYP_KEYSIZE_128B         0x00000000U
#define CRYP_KEYSIZE_192B         CRYP_CR_KEYSIZE_0
#define CRYP_KEYSIZE_256B         CRYP_CR_KEYSIZE_1
/**
  * @}
  */

/** @defgroup CRYP_Data_Type CRYP Data Type
  * @{
  */
#define CRYP_DATATYPE_32B         0x00000000U
#define CRYP_DATATYPE_16B         CRYP_CR_DATATYPE_0
#define CRYP_DATATYPE_8B          CRYP_CR_DATATYPE_1
#define CRYP_DATATYPE_1B          CRYP_CR_DATATYPE
/**
  * @}
  */

/** @defgroup CRYP_Exported_Constants_Group3 CRYP CRYP_AlgoModeDirection
  * @{
  */
#define CRYP_CR_ALGOMODE_DIRECTION         0x0008003CU
#define CRYP_CR_ALGOMODE_TDES_ECB_ENCRYPT  0x00000000U
#define CRYP_CR_ALGOMODE_TDES_ECB_DECRYPT  0x00000004U
#define CRYP_CR_ALGOMODE_TDES_CBC_ENCRYPT  0x00000008U
#define CRYP_CR_ALGOMODE_TDES_CBC_DECRYPT  0x0000000CU
#define CRYP_CR_ALGOMODE_DES_ECB_ENCRYPT   0x00000010U
#define CRYP_CR_ALGOMODE_DES_ECB_DECRYPT   0x00000014U
#define CRYP_CR_ALGOMODE_DES_CBC_ENCRYPT   0x00000018U
#define CRYP_CR_ALGOMODE_DES_CBC_DECRYPT   0x0000001CU
#define CRYP_CR_ALGOMODE_AES_ECB_ENCRYPT   0x00000020U
#define CRYP_CR_ALGOMODE_AES_ECB_DECRYPT   0x00000024U
#define CRYP_CR_ALGOMODE_AES_CBC_ENCRYPT   0x00000028U
#define CRYP_CR_ALGOMODE_AES_CBC_DECRYPT   0x0000002CU
#define CRYP_CR_ALGOMODE_AES_CTR_ENCRYPT   0x00000030U
#define CRYP_CR_ALGOMODE_AES_CTR_DECRYPT   0x00000034U
/**
  * @}
  */

/** @defgroup CRYP_Exported_Constants_Group4 CRYP CRYP_Interrupt
  * @{
  */
#define CRYP_IT_INI               ((uint32_t)CRYP_IMSCR_INIM)   /*!< Input FIFO Interrupt */
#define CRYP_IT_OUTI              ((uint32_t)CRYP_IMSCR_OUTIM)  /*!< Output FIFO Interrupt */
/**
  * @}
  */

/** @defgroup CRYP_Exported_Constants_Group5 CRYP CRYP_Flags
  * @{
  */
#define CRYP_FLAG_BUSY   0x00000010U  /*!< The CRYP core is currently
                                           processing a block of data
                                           or a key preparation (for
                                           AES decryption). */
#define CRYP_FLAG_IFEM   0x00000001U  /*!< Input FIFO is empty */
#define CRYP_FLAG_IFNF   0x00000002U  /*!< Input FIFO is not Full */
#define CRYP_FLAG_OFNE   0x00000004U  /*!< Output FIFO is not empty */
#define CRYP_FLAG_OFFU   0x00000008U  /*!< Output FIFO is Full */
#define CRYP_FLAG_OUTRIS 0x01000002U  /*!< Output FIFO service raw
                                           interrupt status */
#define CRYP_FLAG_INRIS  0x01000001U  /*!< Input FIFO service raw
                                           interrupt status */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup CRYP_Exported_Macros CRYP Exported Macros
  * @{
  */

/** @brief Reset CRYP handle state
  * @param  __HANDLE__ specifies the CRYP handle.
  * @retval None
  */
#define __HAL_CRYP_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_CRYP_STATE_RESET)

/**
  * @brief  Enable/Disable the CRYP peripheral.
  * @param  __HANDLE__ specifies the CRYP handle.
  * @retval None
  */
#define __HAL_CRYP_ENABLE(__HANDLE__)  ((__HANDLE__)->Instance->CR |=  CRYP_CR_CRYPEN)
#define __HAL_CRYP_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->CR &=  ~CRYP_CR_CRYPEN)

/**
  * @brief  Flush the data FIFO.
  * @param  __HANDLE__ specifies the CRYP handle.
  * @retval None
  */
#define __HAL_CRYP_FIFO_FLUSH(__HANDLE__) ((__HANDLE__)->Instance->CR |=  CRYP_CR_FFLUSH)

/**
  * @brief  Set the algorithm mode: AES-ECB, AES-CBC, AES-CTR, DES-ECB, DES-CBC.
  * @param  __HANDLE__ specifies the CRYP handle.
  * @param  MODE The algorithm mode.
  * @retval None
  */
#define __HAL_CRYP_SET_MODE(__HANDLE__, MODE)  ((__HANDLE__)->Instance->CR |= (uint32_t)(MODE))

/** @brief  Check whether the specified CRYP flag is set or not.
  * @param  __HANDLE__ specifies the CRYP handle.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg CRYP_FLAG_BUSY: The CRYP core is currently processing a block of data
  *                                 or a key preparation (for AES decryption).
  *            @arg CRYP_FLAG_IFEM: Input FIFO is empty
  *            @arg CRYP_FLAG_IFNF: Input FIFO is not full
  *            @arg CRYP_FLAG_INRIS: Input FIFO service raw interrupt is pending
  *            @arg CRYP_FLAG_OFNE: Output FIFO is not empty
  *            @arg CRYP_FLAG_OFFU: Output FIFO is full
  *            @arg CRYP_FLAG_OUTRIS: Input FIFO service raw interrupt is pending
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */

#define __HAL_CRYP_GET_FLAG(__HANDLE__, __FLAG__) ((((uint8_t)((__FLAG__) >> 24U)) == 0x01U)?((((__HANDLE__)->Instance->RISR) & ((__FLAG__) & CRYP_FLAG_MASK)) == ((__FLAG__) & CRYP_FLAG_MASK)): \
                                                 ((((__HANDLE__)->Instance->RISR) & ((__FLAG__) & CRYP_FLAG_MASK)) == ((__FLAG__) & CRYP_FLAG_MASK)))

/** @brief  Check whether the specified CRYP interrupt is set or not.
  * @param  __HANDLE__ specifies the CRYP handle.
  * @param  __INTERRUPT__ specifies the interrupt to check.
  *         This parameter can be one of the following values:
  *            @arg CRYP_IT_INRIS: Input FIFO service raw interrupt is pending
  *            @arg CRYP_IT_OUTRIS: Output FIFO service raw interrupt is pending
  * @retval The new state of __INTERRUPT__ (TRUE or FALSE).
  */
#define __HAL_CRYP_GET_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->MISR & (__INTERRUPT__)) == (__INTERRUPT__))

/**
  * @brief  Enable the CRYP interrupt.
  * @param  __HANDLE__ specifies the CRYP handle.
  * @param  __INTERRUPT__ CRYP Interrupt.
  * @retval None
  */
#define __HAL_CRYP_ENABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->IMSCR) |= (__INTERRUPT__))

/**
  * @brief  Disable the CRYP interrupt.
  * @param  __HANDLE__ specifies the CRYP handle.
  * @param  __INTERRUPT__ CRYP interrupt.
  * @retval None
  */
#define __HAL_CRYP_DISABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->IMSCR) &= ~(__INTERRUPT__))

/**
  * @}
  */

/* Include CRYP HAL Extension module */
#include "stm32f4xx_hal_cryp_ex.h"

/* Exported functions --------------------------------------------------------*/
/** @defgroup CRYP_Exported_Functions CRYP Exported Functions
  * @{
  */

/** @addtogroup CRYP_Exported_Functions_Group1
  * @{
  */
HAL_StatusTypeDef HAL_CRYP_Init(CRYP_HandleTypeDef *hcryp);
HAL_StatusTypeDef HAL_CRYP_DeInit(CRYP_HandleTypeDef *hcryp);
void HAL_CRYP_MspInit(CRYP_HandleTypeDef *hcryp);
void HAL_CRYP_MspDeInit(CRYP_HandleTypeDef *hcryp);
/**
  * @}
  */

/** @addtogroup CRYP_Exported_Functions_Group2
  * @{
  */
/* AES encryption/decryption using polling  ***********************************/
HAL_StatusTypeDef HAL_CRYP_AESECB_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout);
HAL_StatusTypeDef HAL_CRYP_AESECB_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout);
HAL_StatusTypeDef HAL_CRYP_AESCBC_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout);
HAL_StatusTypeDef HAL_CRYP_AESCBC_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout);
HAL_StatusTypeDef HAL_CRYP_AESCTR_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout);
HAL_StatusTypeDef HAL_CRYP_AESCTR_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout);

/* AES encryption/decryption using interrupt  *********************************/
HAL_StatusTypeDef HAL_CRYP_AESECB_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef HAL_CRYP_AESCBC_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef HAL_CRYP_AESCTR_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef HAL_CRYP_AESECB_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);
HAL_StatusTypeDef HAL_CRYP_AESCTR_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);
HAL_StatusTypeDef HAL_CRYP_AESCBC_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);

/* AES encryption/decryption using DMA  ***************************************/
HAL_StatusTypeDef HAL_CRYP_AESECB_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef HAL_CRYP_AESECB_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);
HAL_StatusTypeDef HAL_CRYP_AESCBC_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef HAL_CRYP_AESCBC_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);
HAL_StatusTypeDef HAL_CRYP_AESCTR_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef HAL_CRYP_AESCTR_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);
/**
  * @}
  */

/** @addtogroup CRYP_Exported_Functions_Group3
  * @{
  */
/* DES encryption/decryption using polling  ***********************************/
HAL_StatusTypeDef HAL_CRYP_DESECB_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout);
HAL_StatusTypeDef HAL_CRYP_DESCBC_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout);
HAL_StatusTypeDef HAL_CRYP_DESECB_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout);
HAL_StatusTypeDef HAL_CRYP_DESCBC_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout);

/* DES encryption/decryption using interrupt  *********************************/
HAL_StatusTypeDef HAL_CRYP_DESECB_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef HAL_CRYP_DESECB_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);
HAL_StatusTypeDef HAL_CRYP_DESCBC_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef HAL_CRYP_DESCBC_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);

/* DES encryption/decryption using DMA  ***************************************/
HAL_StatusTypeDef HAL_CRYP_DESECB_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef HAL_CRYP_DESECB_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);
HAL_StatusTypeDef HAL_CRYP_DESCBC_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef HAL_CRYP_DESCBC_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);
/**
  * @}
  */

/** @addtogroup CRYP_Exported_Functions_Group4
  * @{
  */
/* TDES encryption/decryption using polling  **********************************/
HAL_StatusTypeDef HAL_CRYP_TDESECB_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout);
HAL_StatusTypeDef HAL_CRYP_TDESCBC_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout);
HAL_StatusTypeDef HAL_CRYP_TDESECB_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout);
HAL_StatusTypeDef HAL_CRYP_TDESCBC_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout);

/* TDES encryption/decryption using interrupt  ********************************/
HAL_StatusTypeDef HAL_CRYP_TDESECB_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef HAL_CRYP_TDESECB_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);
HAL_StatusTypeDef HAL_CRYP_TDESCBC_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef HAL_CRYP_TDESCBC_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);

/* TDES encryption/decryption using DMA  **************************************/
HAL_StatusTypeDef HAL_CRYP_TDESECB_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef HAL_CRYP_TDESECB_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);
HAL_StatusTypeDef HAL_CRYP_TDESCBC_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef HAL_CRYP_TDESCBC_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);
/**
  * @}
  */

/** @addtogroup CRYP_Exported_Functions_Group5
  * @{
  */
void HAL_CRYP_InCpltCallback(CRYP_HandleTypeDef *hcryp);
void HAL_CRYP_OutCpltCallback(CRYP_HandleTypeDef *hcryp);
void HAL_CRYP_ErrorCallback(CRYP_HandleTypeDef *hcryp);
/**
  * @}
  */

/** @addtogroup CRYP_Exported_Functions_Group6
  * @{
  */
void HAL_CRYP_IRQHandler(CRYP_HandleTypeDef *hcryp);
/**
  * @}
  */

/** @addtogroup CRYP_Exported_Functions_Group7
  * @{
  */
HAL_CRYP_STATETypeDef HAL_CRYP_GetState(CRYP_HandleTypeDef *hcryp);
/**
  * @}
  */

/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/** @defgroup CRYP_Private_Types CRYP Private Types
  * @{
  */

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup CRYP_Private_Variables CRYP Private Variables
  * @{
  */

/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup CRYP_Private_Constants CRYP Private Constants
  * @{
  */
#define CRYP_FLAG_MASK  0x0000001FU
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup CRYP_Private_Macros CRYP Private Macros
  * @{
  */

#define IS_CRYP_KEYSIZE(__KEYSIZE__)  (((__KEYSIZE__) == CRYP_KEYSIZE_128B)  || \
                                       ((__KEYSIZE__) == CRYP_KEYSIZE_192B)  || \
                                       ((__KEYSIZE__) == CRYP_KEYSIZE_256B))


#define IS_CRYP_DATATYPE(__DATATYPE__) (((__DATATYPE__) == CRYP_DATATYPE_32B) || \
                                        ((__DATATYPE__) == CRYP_DATATYPE_16B) || \
                                        ((__DATATYPE__) == CRYP_DATATYPE_8B)  || \
                                        ((__DATATYPE__) == CRYP_DATATYPE_1B))


 /**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup CRYP_Private_Functions CRYP Private Functions
  * @{
  */

/**
  * @}
  */

#endif /* CRYP */

#if defined (AES)

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @addtogroup CRYP
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup CRYP_Exported_Types CRYP Exported Types
  * @{
  */

/**
  * @brief  CRYP Configuration Structure definition
  */
typedef struct
{
  uint32_t DataType;       /*!< 32-bit data, 16-bit data, 8-bit data or 1-bit string.
                             This parameter can be a value of @ref CRYP_Data_Type */

  uint32_t KeySize;        /*!< 128 or 256-bit key length.
                             This parameter can be a value of @ref CRYP_Key_Size */

  uint32_t OperatingMode;  /*!< AES operating mode.
                             This parameter can be a value of @ref CRYP_AES_OperatingMode */

  uint32_t ChainingMode;   /*!< AES chaining mode.
                             This parameter can be a value of @ref CRYP_AES_ChainingMode */

  uint32_t KeyWriteFlag;   /*!< Allows to bypass or not key write-up before decryption.
                             This parameter can be a value of @ref CRYP_Key_Write */

  uint32_t GCMCMACPhase;   /*!< Indicates the processing phase of the Galois Counter Mode (GCM),
                             Galois Message Authentication Code (GMAC) or Cipher Message
                             Authentication Code (CMAC) or Counter with Cipher Mode (CCM) when
                             the latter is applicable.
                             This parameter can be a value of @ref CRYP_GCM_CMAC_Phase */

  uint8_t* pKey;           /*!< Encryption/Decryption Key */

  uint8_t* pInitVect;      /*!< Initialization Vector used for CTR, CBC, GCM/GMAC, CMAC,
                                (and CCM when applicable) modes */

  uint8_t* Header;         /*!< Header used in GCM/GMAC, CMAC (and CCM when applicable) modes */

  uint64_t HeaderSize;     /*!< Header size in bytes */

}CRYP_InitTypeDef;

/**
  * @brief HAL CRYP State structures definition
  */
typedef enum
{
  HAL_CRYP_STATE_RESET             = 0x00U,  /*!< CRYP not yet initialized or disabled  */
  HAL_CRYP_STATE_READY             = 0x01U,  /*!< CRYP initialized and ready for use    */
  HAL_CRYP_STATE_BUSY              = 0x02U,  /*!< CRYP internal processing is ongoing   */
  HAL_CRYP_STATE_TIMEOUT           = 0x03U,  /*!< CRYP timeout state                    */
  HAL_CRYP_STATE_ERROR             = 0x04U,  /*!< CRYP error state                      */
  HAL_CRYP_STATE_SUSPENDED         = 0x05U   /*!< CRYP suspended                        */
}HAL_CRYP_STATETypeDef;

/**
  * @brief HAL CRYP phase structures definition
  */
typedef enum
{
  HAL_CRYP_PHASE_READY             = 0x01U,    /*!< CRYP peripheral is ready for initialization.           */
  HAL_CRYP_PHASE_PROCESS           = 0x02U,    /*!< CRYP peripheral is in processing phase                 */
  HAL_CRYP_PHASE_START             = 0x03U,    /*!< CRYP peripheral has been initialized but
                                                    GCM/GMAC/CMAC(/CCM) initialization phase has not started */
  HAL_CRYP_PHASE_INIT_OVER         = 0x04U,    /*!< GCM/GMAC/CMAC(/CCM) init phase has been carried out    */
  HAL_CRYP_PHASE_HEADER_OVER       = 0x05U,    /*!< GCM/GMAC/CMAC(/CCM) header phase has been carried out  */
  HAL_CRYP_PHASE_PAYLOAD_OVER      = 0x06U,    /*!< GCM(/CCM) payload phase has been carried out           */
  HAL_CRYP_PHASE_FINAL_OVER        = 0x07U,    /*!< GCM/GMAC/CMAC(/CCM) final phase has been carried out   */
  HAL_CRYP_PHASE_HEADER_SUSPENDED  = 0x08U,    /*!< GCM/GMAC/CMAC(/CCM) header phase has been suspended    */
  HAL_CRYP_PHASE_PAYLOAD_SUSPENDED = 0x09U,    /*!< GCM(/CCM) payload phase has been suspended             */
  HAL_CRYP_PHASE_NOT_USED          = 0x0AU     /*!< Phase is irrelevant to the current chaining mode       */
}HAL_PhaseTypeDef;

/**
  * @brief HAL CRYP mode suspend definitions
  */
typedef enum
{
  HAL_CRYP_SUSPEND_NONE            = 0x00U,    /*!< CRYP peripheral suspension not requested */
  HAL_CRYP_SUSPEND                 = 0x01U     /*!< CRYP peripheral suspension requested     */
}HAL_SuspendTypeDef;


/**
  * @brief  HAL CRYP Error Codes definition
  */
#define HAL_CRYP_ERROR_NONE      0x00000000U  /*!< No error        */
#define HAL_CRYP_WRITE_ERROR     0x00000001U  /*!< Write error     */
#define HAL_CRYP_READ_ERROR      0x00000002U  /*!< Read error      */
#define HAL_CRYP_DMA_ERROR       0x00000004U  /*!< DMA error       */
#define HAL_CRYP_BUSY_ERROR      0x00000008U  /*!< Busy flag error */

/**
  * @brief  CRYP handle Structure definition
  */
typedef struct
{
      AES_TypeDef              *Instance;        /*!< Register base address        */

      CRYP_InitTypeDef         Init;             /*!< CRYP initialization parameters */

      uint8_t                  *pCrypInBuffPtr;  /*!< Pointer to CRYP processing (encryption, decryption,...) input buffer */

      uint8_t                  *pCrypOutBuffPtr; /*!< Pointer to CRYP processing (encryption, decryption,...) output buffer */

      uint32_t                 CrypInCount;      /*!< Input data size in bytes or, after suspension, the remaining
                                                      number of bytes to process */

      uint32_t                 CrypOutCount;     /*!< Output data size in bytes */

      HAL_PhaseTypeDef         Phase;            /*!< CRYP peripheral processing phase for GCM, GMAC, CMAC
                                                      (or CCM when applicable) modes.
                                                      Indicates the last phase carried out to ease
                                                      phase transitions  */

      DMA_HandleTypeDef        *hdmain;          /*!< CRYP peripheral Input DMA handle parameters */

      DMA_HandleTypeDef        *hdmaout;         /*!< CRYP peripheral Output DMA handle parameters */

      HAL_LockTypeDef          Lock;             /*!< CRYP locking object */

   __IO  HAL_CRYP_STATETypeDef State;            /*!< CRYP peripheral state */

    __IO uint32_t              ErrorCode;        /*!< CRYP peripheral error code */

     HAL_SuspendTypeDef        SuspendRequest;   /*!< CRYP peripheral suspension request flag */
}CRYP_HandleTypeDef;

/**
  * @}
  */


/* Exported constants --------------------------------------------------------*/
/** @defgroup CRYP_Exported_Constants CRYP Exported Constants
  * @{
  */

/** @defgroup CRYP_Key_Size  Key size selection
  * @{
  */
#define CRYP_KEYSIZE_128B         0x00000000U             /*!< 128-bit long key */
#define CRYP_KEYSIZE_256B         AES_CR_KEYSIZE          /*!< 256-bit long key */
/**
  * @}
  */

/** @defgroup CRYP_Data_Type  AES Data Type selection
  * @{
  */
#define CRYP_DATATYPE_32B         0x00000000U             /*!< 32-bit data type (no swapping)        */
#define CRYP_DATATYPE_16B         AES_CR_DATATYPE_0       /*!< 16-bit data type (half-word swapping) */
#define CRYP_DATATYPE_8B          AES_CR_DATATYPE_1       /*!< 8-bit data type (byte swapping)       */
#define CRYP_DATATYPE_1B          AES_CR_DATATYPE         /*!< 1-bit data type (bit swapping)        */
/**
  * @}
  */

 /** @defgroup CRYP_AES_State  AES Enable state
  * @{
  */
#define CRYP_AES_DISABLE                  0x00000000U              /*!< Disable AES */
#define CRYP_AES_ENABLE                   AES_CR_EN                /*!< Enable AES  */
/**
  * @}
  */

/** @defgroup CRYP_AES_OperatingMode AES operating mode
  * @{
  */
#define CRYP_ALGOMODE_ENCRYPT                   0x00000000U             /*!< Encryption mode                            */
#define CRYP_ALGOMODE_KEYDERIVATION             AES_CR_MODE_0           /*!< Key derivation mode                        */
#define CRYP_ALGOMODE_DECRYPT                   AES_CR_MODE_1           /*!< Decryption                                 */
#define CRYP_ALGOMODE_KEYDERIVATION_DECRYPT     AES_CR_MODE             /*!< Key derivation and decryption              */
#define CRYP_ALGOMODE_TAG_GENERATION            0x00000000U             /*!< GMAC or CMAC authentication tag generation */
/**
  * @}
  */

/** @defgroup CRYP_AES_ChainingMode AES chaining mode
  * @{
  */
#define CRYP_CHAINMODE_AES_ECB            0x00000000U                       /*!< Electronic codebook chaining algorithm                   */
#define CRYP_CHAINMODE_AES_CBC            AES_CR_CHMOD_0                    /*!< Cipher block chaining algorithm                          */
#define CRYP_CHAINMODE_AES_CTR            AES_CR_CHMOD_1                    /*!< Counter mode chaining algorithm                          */
#define CRYP_CHAINMODE_AES_GCM_GMAC       (AES_CR_CHMOD_0 | AES_CR_CHMOD_1) /*!< Galois counter mode - Galois message authentication code */
#define CRYP_CHAINMODE_AES_CMAC           AES_CR_CHMOD_2                    /*!< Cipher message authentication code                       */
#if defined(AES_CR_NPBLB)
#define CRYP_CHAINMODE_AES_CCM_CMAC       AES_CR_CHMOD_2                    /*!< Counter with Cipher Mode - Cipher message authentication code */
#endif
/**
  * @}
  */

/** @defgroup CRYP_Key_Write AES decryption key write-up flag
  * @{
  */
#define CRYP_KEY_WRITE_ENABLE          0x00000000U               /*!< Enable decryption key writing  */
#define CRYP_KEY_WRITE_DISABLE         0x00000001U               /*!< Disable decryption key writing */
/**
  * @}
  */

/** @defgroup CRYP_DMAIN DMA Input phase management enable state
  * @{
  */
#define CRYP_DMAIN_DISABLE             0x00000000U               /*!< Disable DMA Input phase management */
#define CRYP_DMAIN_ENABLE              AES_CR_DMAINEN            /*!< Enable DMA Input phase management  */
/**
  * @}
  */

/** @defgroup CRYP_DMAOUT DMA Output phase management enable state
  * @{
  */
#define CRYP_DMAOUT_DISABLE             0x00000000U              /*!< Disable DMA Output phase management */
#define CRYP_DMAOUT_ENABLE              AES_CR_DMAOUTEN          /*!< Enable DMA Output phase management  */
/**
  * @}
  */


/** @defgroup CRYP_GCM_CMAC_Phase GCM/GMAC and CMAC processing phase selection
  * @{
  */
#define CRYP_GCM_INIT_PHASE             0x00000000U             /*!< GCM/GMAC (or CCM) init phase        */
#define CRYP_GCMCMAC_HEADER_PHASE       AES_CR_GCMPH_0          /*!< GCM/GMAC or (CCM/)CMAC header phase */
#define CRYP_GCM_PAYLOAD_PHASE          AES_CR_GCMPH_1          /*!< GCM(/CCM) payload phase             */
#define CRYP_GCMCMAC_FINAL_PHASE        AES_CR_GCMPH            /*!< GCM/GMAC or (CCM/)CMAC final phase  */
/* Definitions duplication for code readibility's sake:
   supported or not supported chain modes are not specified for each phase */
#define CRYP_INIT_PHASE                 0x00000000U             /*!< Init phase    */
#define CRYP_HEADER_PHASE               AES_CR_GCMPH_0          /*!< Header phase  */
#define CRYP_PAYLOAD_PHASE              AES_CR_GCMPH_1          /*!< Payload phase */
#define CRYP_FINAL_PHASE                AES_CR_GCMPH            /*!< Final phase   */
/**
  * @}
  */

/** @defgroup CRYP_Flags   AES status flags
  * @{
  */

#define CRYP_FLAG_BUSY    AES_SR_BUSY   /*!< GCM process suspension forbidden */
#define CRYP_FLAG_WRERR   AES_SR_WRERR  /*!< Write Error                      */
#define CRYP_FLAG_RDERR   AES_SR_RDERR  /*!< Read error                       */
#define CRYP_FLAG_CCF     AES_SR_CCF    /*!< Computation completed            */
/**
  * @}
  */

/** @defgroup CRYP_Clear_Flags   AES clearing flags
  * @{
  */

#define CRYP_CCF_CLEAR    AES_CR_CCFC   /*!< Computation Complete Flag Clear */
#define CRYP_ERR_CLEAR    AES_CR_ERRC   /*!< Error Flag Clear                */
/**
  * @}
  */

/** @defgroup AES_Interrupts_Enable AES Interrupts Enable bits
  * @{
  */
#define CRYP_IT_CCFIE     AES_CR_CCFIE  /*!< Computation Complete interrupt enable */
#define CRYP_IT_ERRIE     AES_CR_ERRIE  /*!< Error interrupt enable                */
/**
  * @}
  */

/** @defgroup CRYP_Interrupts_Flags   AES Interrupts flags
  * @{
  */
#define CRYP_IT_WRERR    AES_SR_WRERR   /*!< Write Error           */
#define CRYP_IT_RDERR    AES_SR_RDERR   /*!< Read Error            */
#define CRYP_IT_CCF      AES_SR_CCF     /*!< Computation completed */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup CRYP_Exported_Macros CRYP Exported Macros
  * @{
  */

/** @brief Reset CRYP handle state.
  * @param  __HANDLE__ specifies the CRYP handle.
  * @retval None
  */
#define __HAL_CRYP_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_CRYP_STATE_RESET)

/**
  * @brief  Enable the CRYP AES peripheral.
  * @retval None
  */
#define __HAL_CRYP_ENABLE()  (AES->CR |=  AES_CR_EN)

/**
  * @brief  Disable the CRYP AES peripheral.
  * @retval None
  */
#define __HAL_CRYP_DISABLE() (AES->CR &=  ~AES_CR_EN)

/**
  * @brief  Set the algorithm operating mode.
  * @param  __OPERATING_MODE__ specifies the operating mode
  *          This parameter can be one of the following values:
  *            @arg @ref CRYP_ALGOMODE_ENCRYPT encryption
  *            @arg @ref CRYP_ALGOMODE_KEYDERIVATION key derivation
  *            @arg @ref CRYP_ALGOMODE_DECRYPT decryption
  *            @arg @ref CRYP_ALGOMODE_KEYDERIVATION_DECRYPT key derivation and decryption
  * @retval None
  */
#define __HAL_CRYP_SET_OPERATINGMODE(__OPERATING_MODE__) MODIFY_REG(AES->CR, AES_CR_MODE, (__OPERATING_MODE__))


/**
  * @brief  Set the algorithm chaining mode.
  * @param  __CHAINING_MODE__ specifies the chaining mode
  *          This parameter can be one of the following values:
  *            @arg @ref CRYP_CHAINMODE_AES_ECB Electronic CodeBook
  *            @arg @ref CRYP_CHAINMODE_AES_CBC Cipher Block Chaining
  *            @arg @ref CRYP_CHAINMODE_AES_CTR CounTeR mode
  *            @arg @ref CRYP_CHAINMODE_AES_GCM_GMAC Galois Counter Mode or Galois Message Authentication Code
  *            @arg @ref CRYP_CHAINMODE_AES_CMAC Cipher Message Authentication Code (or Counter with Cipher Mode when applicable)
  * @retval None
  */
#define __HAL_CRYP_SET_CHAININGMODE(__CHAINING_MODE__) MODIFY_REG(AES->CR, AES_CR_CHMOD, (__CHAINING_MODE__))



/** @brief  Check whether the specified CRYP status flag is set or not.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg @ref CRYP_FLAG_BUSY GCM process suspension forbidden
  *            @arg @ref CRYP_IT_WRERR Write Error
  *            @arg @ref CRYP_IT_RDERR Read Error
  *            @arg @ref CRYP_IT_CCF Computation Complete
  * @retval The state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_CRYP_GET_FLAG(__FLAG__) ((AES->SR & (__FLAG__)) == (__FLAG__))


/** @brief  Clear the CRYP pending status flag.
  * @param  __FLAG__ specifies the flag to clear.
  *         This parameter can be one of the following values:
  *            @arg @ref CRYP_ERR_CLEAR Read (RDERR) or Write Error (WRERR) Flag Clear
  *            @arg @ref CRYP_CCF_CLEAR Computation Complete Flag (CCF) Clear
  * @retval None
  */
#define __HAL_CRYP_CLEAR_FLAG(__FLAG__) SET_BIT(AES->CR, (__FLAG__))



/** @brief  Check whether the specified CRYP interrupt source is enabled or not.
  * @param __INTERRUPT__ CRYP interrupt source to check
  *         This parameter can be one of the following values:
  *            @arg @ref CRYP_IT_ERRIE Error interrupt (used for RDERR and WRERR)
  *            @arg @ref CRYP_IT_CCFIE Computation Complete interrupt
  * @retval State of interruption (TRUE or FALSE).
  */
#define __HAL_CRYP_GET_IT_SOURCE(__INTERRUPT__) ((AES->CR & (__INTERRUPT__)) == (__INTERRUPT__))


/** @brief  Check whether the specified CRYP interrupt is set or not.
  * @param  __INTERRUPT__ specifies the interrupt to check.
  *         This parameter can be one of the following values:
  *            @arg @ref CRYP_IT_WRERR Write Error
  *            @arg @ref CRYP_IT_RDERR Read Error
  *            @arg @ref CRYP_IT_CCF  Computation Complete
  * @retval The state of __INTERRUPT__ (TRUE or FALSE).
  */
#define __HAL_CRYP_GET_IT(__INTERRUPT__) ((AES->SR & (__INTERRUPT__)) == (__INTERRUPT__))



/** @brief  Clear the CRYP pending interrupt.
  * @param  __INTERRUPT__ specifies the IT to clear.
  *         This parameter can be one of the following values:
  *            @arg @ref CRYP_ERR_CLEAR Read (RDERR) or Write Error (WRERR) Flag Clear
  *            @arg @ref CRYP_CCF_CLEAR Computation Complete Flag (CCF) Clear
  * @retval None
  */
#define __HAL_CRYP_CLEAR_IT(__INTERRUPT__) SET_BIT(AES->CR, (__INTERRUPT__))


/**
  * @brief  Enable the CRYP interrupt.
  * @param  __INTERRUPT__ CRYP Interrupt.
  *         This parameter can be one of the following values:
  *            @arg @ref CRYP_IT_ERRIE Error interrupt (used for RDERR and WRERR)
  *            @arg @ref CRYP_IT_CCFIE Computation Complete interrupt
  * @retval None
  */
#define __HAL_CRYP_ENABLE_IT(__INTERRUPT__) ((AES->CR) |= (__INTERRUPT__))


/**
  * @brief  Disable the CRYP interrupt.
  * @param  __INTERRUPT__ CRYP Interrupt.
  *         This parameter can be one of the following values:
  *            @arg @ref CRYP_IT_ERRIE Error interrupt (used for RDERR and WRERR)
  *            @arg @ref CRYP_IT_CCFIE Computation Complete interrupt
  * @retval None
  */
#define __HAL_CRYP_DISABLE_IT(__INTERRUPT__) ((AES->CR) &= ~(__INTERRUPT__))

/**
  * @}
  */

/* Private macros --------------------------------------------------------*/
/** @addtogroup  CRYP_Private_Macros   CRYP Private Macros
  * @{
  */

/**
  * @brief Verify the key size length.
  * @param __KEYSIZE__ Ciphering/deciphering algorithm key size.
  * @retval SET (__KEYSIZE__ is a valid value) or RESET (__KEYSIZE__ is invalid)
  */
#define IS_CRYP_KEYSIZE(__KEYSIZE__)  (((__KEYSIZE__) == CRYP_KEYSIZE_128B)  || \
                                       ((__KEYSIZE__) == CRYP_KEYSIZE_256B))

/**
  * @brief Verify the input data type.
  * @param __DATATYPE__ Ciphering/deciphering algorithm input data type.
  * @retval SET (__DATATYPE__ is valid) or RESET (__DATATYPE__ is invalid)
  */
#define IS_CRYP_DATATYPE(__DATATYPE__) (((__DATATYPE__) == CRYP_DATATYPE_32B) || \
                                        ((__DATATYPE__) == CRYP_DATATYPE_16B) || \
                                        ((__DATATYPE__) == CRYP_DATATYPE_8B)  || \
                                        ((__DATATYPE__) == CRYP_DATATYPE_1B))

/**
  * @brief Verify the CRYP AES IP running mode.
  * @param __MODE__ CRYP AES IP running mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_CRYP_AES(__MODE__) (((__MODE__) == CRYP_AES_DISABLE) || \
                               ((__MODE__) == CRYP_AES_ENABLE))

/**
  * @brief Verify the selected CRYP algorithm.
  * @param __ALGOMODE__ Selected CRYP algorithm (ciphering, deciphering, key derivation or a combination of the latter).
  * @retval SET (__ALGOMODE__ is valid) or RESET (__ALGOMODE__ is invalid)
  */
#define IS_CRYP_ALGOMODE(__ALGOMODE__) (((__ALGOMODE__) == CRYP_ALGOMODE_ENCRYPT)        || \
                                        ((__ALGOMODE__) == CRYP_ALGOMODE_KEYDERIVATION)  || \
                                        ((__ALGOMODE__) == CRYP_ALGOMODE_DECRYPT)        || \
                                        ((__ALGOMODE__) == CRYP_ALGOMODE_TAG_GENERATION) || \
                                        ((__ALGOMODE__) == CRYP_ALGOMODE_KEYDERIVATION_DECRYPT))

/**
  * @brief Verify the selected CRYP chaining algorithm.
  * @param __CHAINMODE__ Selected CRYP chaining algorithm.
  * @retval SET (__CHAINMODE__ is valid) or RESET (__CHAINMODE__ is invalid)
  */
#if defined(AES_CR_NPBLB)
#define IS_CRYP_CHAINMODE(__CHAINMODE__) (((__CHAINMODE__) == CRYP_CHAINMODE_AES_ECB)     || \
                                         ((__CHAINMODE__) == CRYP_CHAINMODE_AES_CBC)      || \
                                         ((__CHAINMODE__) == CRYP_CHAINMODE_AES_CTR)      || \
                                         ((__CHAINMODE__) == CRYP_CHAINMODE_AES_GCM_GMAC) || \
                                         ((__CHAINMODE__) == CRYP_CHAINMODE_AES_CCM_CMAC))
#else
#define IS_CRYP_CHAINMODE(__CHAINMODE__) (((__CHAINMODE__) == CRYP_CHAINMODE_AES_ECB)     || \
                                         ((__CHAINMODE__) == CRYP_CHAINMODE_AES_CBC)      || \
                                         ((__CHAINMODE__) == CRYP_CHAINMODE_AES_CTR)      || \
                                         ((__CHAINMODE__) == CRYP_CHAINMODE_AES_GCM_GMAC) || \
                                         ((__CHAINMODE__) == CRYP_CHAINMODE_AES_CMAC))
#endif

/**
  * @brief Verify the deciphering key write option.
  * @param __WRITE__ deciphering key write option.
  * @retval SET (__WRITE__ is valid) or RESET (__WRITE__ is invalid)
  */
#define IS_CRYP_WRITE(__WRITE__)   (((__WRITE__) == CRYP_KEY_WRITE_ENABLE)      || \
                                    ((__WRITE__) == CRYP_KEY_WRITE_DISABLE))

/**
  * @brief Verify the CRYP input data DMA mode.
  * @param __MODE__ CRYP input data DMA mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_CRYP_DMAIN(__MODE__) (((__MODE__) == CRYP_DMAIN_DISABLE) || \
                                 ((__MODE__) == CRYP_DMAIN_ENABLE))

/**
  * @brief Verify the CRYP output data DMA mode.
  * @param __MODE__ CRYP output data DMA mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_CRYP_DMAOUT(__MODE__) (((__MODE__) == CRYP_DMAOUT_DISABLE) || \
                                  ((__MODE__) == CRYP_DMAOUT_ENABLE))

/**
  * @brief Verify the CRYP AES ciphering/deciphering/authentication algorithm phase.
  * @param __PHASE__ CRYP AES ciphering/deciphering/authentication algorithm phase.
  * @retval SET (__PHASE__ is valid) or RESET (__PHASE__ is invalid)
  */
#define IS_CRYP_GCMCMAC_PHASE(__PHASE__) (((__PHASE__) == CRYP_GCM_INIT_PHASE)       || \
                                          ((__PHASE__) == CRYP_GCMCMAC_HEADER_PHASE) || \
                                          ((__PHASE__) == CRYP_GCM_PAYLOAD_PHASE)    || \
                                          ((__PHASE__) == CRYP_GCMCMAC_FINAL_PHASE))

/**
  * @}
  */

/* Include CRYP HAL Extended module */
#include "stm32f4xx_hal_cryp_ex.h"

/* Exported functions --------------------------------------------------------*/
/** @addtogroup CRYP_Exported_Functions CRYP Exported Functions
  * @{
  */

/** @addtogroup CRYP_Exported_Functions_Group1 Initialization and deinitialization functions
  * @{
  */

/* Initialization/de-initialization functions  ********************************/
HAL_StatusTypeDef HAL_CRYP_Init(CRYP_HandleTypeDef *hcryp);
HAL_StatusTypeDef HAL_CRYP_DeInit(CRYP_HandleTypeDef *hcryp);

/* MSP initialization/de-initialization functions  ****************************/
void HAL_CRYP_MspInit(CRYP_HandleTypeDef *hcryp);
void HAL_CRYP_MspDeInit(CRYP_HandleTypeDef *hcryp);

/**
  * @}
  */

/** @addtogroup CRYP_Exported_Functions_Group2 AES processing functions
  * @{
  */

/* AES encryption/decryption processing functions  ****************************/

/* AES encryption/decryption using polling  ***********************************/
HAL_StatusTypeDef     HAL_CRYP_AESECB_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout);
HAL_StatusTypeDef     HAL_CRYP_AESECB_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout);
HAL_StatusTypeDef     HAL_CRYP_AESCBC_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout);
HAL_StatusTypeDef     HAL_CRYP_AESCBC_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout);
HAL_StatusTypeDef     HAL_CRYP_AESCTR_Encrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData, uint32_t Timeout);
HAL_StatusTypeDef     HAL_CRYP_AESCTR_Decrypt(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData, uint32_t Timeout);

/* AES encryption/decryption using interrupt  *********************************/
HAL_StatusTypeDef     HAL_CRYP_AESECB_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef     HAL_CRYP_AESCBC_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef     HAL_CRYP_AESCTR_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef     HAL_CRYP_AESECB_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);
HAL_StatusTypeDef     HAL_CRYP_AESCTR_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);
HAL_StatusTypeDef     HAL_CRYP_AESCBC_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);

/* AES encryption/decryption using DMA  ***************************************/
HAL_StatusTypeDef     HAL_CRYP_AESECB_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef     HAL_CRYP_AESECB_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);
HAL_StatusTypeDef     HAL_CRYP_AESCBC_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef     HAL_CRYP_AESCBC_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);
HAL_StatusTypeDef     HAL_CRYP_AESCTR_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pPlainData, uint16_t Size, uint8_t *pCypherData);
HAL_StatusTypeDef     HAL_CRYP_AESCTR_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint8_t *pCypherData, uint16_t Size, uint8_t *pPlainData);

/**
  * @}
  */

/** @addtogroup CRYP_Exported_Functions_Group3 Callback functions
  * @{
  */
/* CallBack functions  ********************************************************/
void HAL_CRYP_InCpltCallback(CRYP_HandleTypeDef *hcryp);
void HAL_CRYP_OutCpltCallback(CRYP_HandleTypeDef *hcryp);
void HAL_CRYP_ErrorCallback(CRYP_HandleTypeDef *hcryp);

/**
  * @}
  */

/** @addtogroup CRYP_Exported_Functions_Group4 CRYP IRQ handler
  * @{
  */

/* AES interrupt handling function  *******************************************/
void HAL_CRYP_IRQHandler(CRYP_HandleTypeDef *hcryp);

/**
  * @}
  */

/** @addtogroup CRYP_Exported_Functions_Group5 Peripheral State functions
  * @{
  */

/* Peripheral State functions  ************************************************/
HAL_CRYP_STATETypeDef HAL_CRYP_GetState(CRYP_HandleTypeDef *hcryp);
uint32_t              HAL_CRYP_GetError(CRYP_HandleTypeDef *hcryp);

/**
  * @}
  */

/**
  * @}
  */


#endif /* AES */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_CRYP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
