/**
  ******************************************************************************
  * @file    stm32f4xx_hal_hash.h
  * @author  MCD Application Team
  * @brief   Header file of HASH HAL module.
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
#ifndef __STM32F4xx_HAL_HASH_H
#define __STM32F4xx_HAL_HASH_H

#ifdef __cplusplus
 extern "C" {
#endif

#if defined(STM32F415xx) || defined(STM32F417xx) || defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F479xx)

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal_def.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @addtogroup HASH
  * @brief HASH HAL module driver
  *  @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup HASH_Exported_Types HASH Exported Types
  * @{
  */

/** @defgroup HASH_Exported_Types_Group1 HASH Configuration Structure definition
  * @{
  */

typedef struct
{
  uint32_t DataType;  /*!< 32-bit data, 16-bit data, 8-bit data or 1-bit string.
                           This parameter can be a value of @ref HASH_Data_Type */

  uint32_t KeySize;   /*!< The key size is used only in HMAC operation          */

  uint8_t* pKey;      /*!< The key is used only in HMAC operation               */
}HASH_InitTypeDef;

/**
  * @}
  */

/** @defgroup HASH_Exported_Types_Group2 HASH State structures definition
  * @{
  */

typedef enum
{
  HAL_HASH_STATE_RESET     = 0x00U,  /*!< HASH not yet initialized or disabled */
  HAL_HASH_STATE_READY     = 0x01U,  /*!< HASH initialized and ready for use   */
  HAL_HASH_STATE_BUSY      = 0x02U,  /*!< HASH internal process is ongoing     */
  HAL_HASH_STATE_TIMEOUT   = 0x03U,  /*!< HASH timeout state                   */
  HAL_HASH_STATE_ERROR     = 0x04U   /*!< HASH error state                     */
}HAL_HASH_StateTypeDef;

/**
  * @}
  */

/** @defgroup HASH_Exported_Types_Group3 HASH phase structures definition
  * @{
  */

typedef enum
{
  HAL_HASH_PHASE_READY     = 0x01U,  /*!< HASH peripheral is ready for initialization */
  HAL_HASH_PHASE_PROCESS   = 0x02U  /*!< HASH peripheral is in processing phase      */
}HAL_HASH_PhaseTypeDef;

/**
  * @}
  */

/** @defgroup HASH_Exported_Types_Group4 HASH Handle structures definition
  * @{
  */

typedef struct
{
      HASH_InitTypeDef           Init;              /*!< HASH required parameters       */

      uint8_t                    *pHashInBuffPtr;   /*!< Pointer to input buffer        */

      uint8_t                    *pHashOutBuffPtr;  /*!< Pointer to input buffer        */

     __IO uint32_t               HashBuffSize;      /*!< Size of buffer to be processed */

     __IO uint32_t               HashInCount;       /*!< Counter of inputed data        */

     __IO uint32_t               HashITCounter;     /*!< Counter of issued interrupts   */

      HAL_StatusTypeDef          Status;            /*!< HASH peripheral status         */

      HAL_HASH_PhaseTypeDef       Phase;             /*!< HASH peripheral phase          */

      DMA_HandleTypeDef          *hdmain;           /*!< HASH In DMA handle parameters  */

      HAL_LockTypeDef            Lock;              /*!< HASH locking object            */

     __IO HAL_HASH_StateTypeDef  State;             /*!< HASH peripheral state          */
} HASH_HandleTypeDef;

/**
  * @}
  */


/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup HASH_Exported_Constants HASH Exported Constants
  * @{
  */

/** @defgroup HASH_Exported_Constants_Group1 HASH Algorithm Selection
  * @{
  */
#define HASH_ALGOSELECTION_SHA1      0x00000000U         /*!< HASH function is SHA1   */
#define HASH_ALGOSELECTION_SHA224    HASH_CR_ALGO_1      /*!< HASH function is SHA224 */
#define HASH_ALGOSELECTION_SHA256    HASH_CR_ALGO        /*!< HASH function is SHA256 */
#define HASH_ALGOSELECTION_MD5       HASH_CR_ALGO_0      /*!< HASH function is MD5    */
/**
  * @}
  */

/** @defgroup HASH_Exported_Constants_Group2 HASH Algorithm Mode
  * @{
  */
#define HASH_ALGOMODE_HASH         0x00000000U           /*!< Algorithm is HASH */
#define HASH_ALGOMODE_HMAC         HASH_CR_MODE          /*!< Algorithm is HMAC */
/**
  * @}
  */

/** @defgroup HASH_Data_Type HASH Data Type
  * @{
  */
#define HASH_DATATYPE_32B          0x00000000U           /*!< 32-bit data. No swapping                     */
#define HASH_DATATYPE_16B          HASH_CR_DATATYPE_0    /*!< 16-bit data. Each half word is swapped       */
#define HASH_DATATYPE_8B           HASH_CR_DATATYPE_1    /*!< 8-bit data. All bytes are swapped            */
#define HASH_DATATYPE_1B           HASH_CR_DATATYPE      /*!< 1-bit data. In the word all bits are swapped */
/**
  * @}
  */

/** @defgroup HASH_Exported_Constants_Group4 HASH HMAC Long key
  * @brief HASH HMAC Long key used only for HMAC mode
  * @{
  */
#define HASH_HMAC_KEYTYPE_SHORTKEY      0x00000000U      /*!< HMAC Key is <= 64 bytes */
#define HASH_HMAC_KEYTYPE_LONGKEY       HASH_CR_LKEY     /*!< HMAC Key is > 64 bytes  */
/**
  * @}
  */

/** @defgroup HASH_Exported_Constants_Group5 HASH Flags definition
  * @{
  */
#define HASH_FLAG_DINIS            HASH_SR_DINIS         /*!< 16 locations are free in the DIN : A new block can be entered into the input buffer */
#define HASH_FLAG_DCIS             HASH_SR_DCIS          /*!< Digest calculation complete                                                         */
#define HASH_FLAG_DMAS             HASH_SR_DMAS          /*!< DMA interface is enabled (DMAE=1) or a transfer is ongoing                          */
#define HASH_FLAG_BUSY             HASH_SR_BUSY          /*!< The hash core is Busy : processing a block of data                                  */
#define HASH_FLAG_DINNE            HASH_CR_DINNE         /*!< DIN not empty : The input buffer contains at least one word of data                 */
/**
  * @}
  */

/** @defgroup HASH_Exported_Constants_Group6 HASH Interrupts definition
  * @{
  */
#define HASH_IT_DINI               HASH_IMR_DINIE        /*!< A new block can be entered into the input buffer (DIN) */
#define HASH_IT_DCI                HASH_IMR_DCIE         /*!< Digest calculation complete                            */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup HASH_Exported_Macros HASH Exported Macros
  * @{
  */

/** @brief Reset HASH handle state
  * @param  __HANDLE__ specifies the HASH handle.
  * @retval None
  */
#define __HAL_HASH_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_HASH_STATE_RESET)

/** @brief  Check whether the specified HASH flag is set or not.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg HASH_FLAG_DINIS: A new block can be entered into the input buffer.
  *            @arg HASH_FLAG_DCIS: Digest calculation complete
  *            @arg HASH_FLAG_DMAS: DMA interface is enabled (DMAE=1) or a transfer is ongoing
  *            @arg HASH_FLAG_BUSY: The hash core is Busy : processing a block of data
  *            @arg HASH_FLAG_DINNE: DIN not empty : The input buffer contains at least one word of data
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_HASH_GET_FLAG(__FLAG__) (((__FLAG__) > 8U) ? ((HASH->CR & (__FLAG__)) == (__FLAG__)) :\
                                                           ((HASH->SR & (__FLAG__)) == (__FLAG__)))

/**
  * @brief  Enable the multiple DMA mode.
  *         This feature is available only in STM32F429x and STM32F439x devices.
  * @retval None
  */
#define __HAL_HASH_SET_MDMAT()          HASH->CR |= HASH_CR_MDMAT

/**
  * @brief  Disable the multiple DMA mode.
  * @retval None
  */
#define __HAL_HASH_RESET_MDMAT()        HASH->CR &= (uint32_t)(~HASH_CR_MDMAT)

/**
  * @brief  Start the digest computation
  * @retval None
  */
#define __HAL_HASH_START_DIGEST()       HASH->STR |= HASH_STR_DCAL

/**
  * @brief Set the number of valid bits in last word written in Data register
  * @param  SIZE size in byte of last data written in Data register.
  * @retval None
*/
#define __HAL_HASH_SET_NBVALIDBITS(SIZE) do{HASH->STR &= ~(HASH_STR_NBLW);\
                                            HASH->STR |= 8U * ((SIZE) % 4U);\
                                           }while(0)

/**
  * @}
  */

/* Include HASH HAL Extension module */
#include "stm32f4xx_hal_hash_ex.h"
/* Exported functions --------------------------------------------------------*/

/** @defgroup HASH_Exported_Functions HASH Exported Functions
  * @{
  */

/** @addtogroup HASH_Exported_Functions_Group1
  * @{
  */
HAL_StatusTypeDef HAL_HASH_Init(HASH_HandleTypeDef *hhash);
HAL_StatusTypeDef HAL_HASH_DeInit(HASH_HandleTypeDef *hhash);
/**
  * @}
  */

/** @addtogroup HASH_Exported_Functions_Group2
  * @{
  */
HAL_StatusTypeDef HAL_HASH_SHA1_Start(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t* pOutBuffer, uint32_t Timeout);
HAL_StatusTypeDef HAL_HASH_MD5_Start(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t* pOutBuffer, uint32_t Timeout);
HAL_StatusTypeDef HAL_HASH_MD5_Accumulate(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size);
HAL_StatusTypeDef HAL_HASH_SHA1_Accumulate(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size);
/**
  * @}
  */

/** @addtogroup HASH_Exported_Functions_Group3
  * @{
  */
HAL_StatusTypeDef HAL_HMAC_SHA1_Start(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t* pOutBuffer, uint32_t Timeout);
HAL_StatusTypeDef HAL_HMAC_MD5_Start(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t* pOutBuffer, uint32_t Timeout);
/**
  * @}
  */

/** @addtogroup HASH_Exported_Functions_Group4
  * @{
  */
HAL_StatusTypeDef HAL_HASH_SHA1_Start_IT(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t* pOutBuffer);
HAL_StatusTypeDef HAL_HASH_MD5_Start_IT(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t* pOutBuffer);
/**
  * @}
  */

/** @addtogroup HASH_Exported_Functions_Group5
  * @{
  */
HAL_StatusTypeDef HAL_HASH_SHA1_Start_DMA(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size);
HAL_StatusTypeDef HAL_HASH_SHA1_Finish(HASH_HandleTypeDef *hhash, uint8_t* pOutBuffer, uint32_t Timeout);
HAL_StatusTypeDef HAL_HASH_MD5_Start_DMA(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size);
HAL_StatusTypeDef HAL_HASH_MD5_Finish(HASH_HandleTypeDef *hhash, uint8_t* pOutBuffer, uint32_t Timeout);
/**
  * @}
  */

/** @addtogroup HASH_Exported_Functions_Group6
  * @{
  */
HAL_StatusTypeDef HAL_HMAC_SHA1_Start_DMA(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size);
HAL_StatusTypeDef HAL_HMAC_MD5_Start_DMA(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size);
/**
  * @}
  */

/** @addtogroup HASH_Exported_Functions_Group7
  * @{
  */
void HAL_HASH_IRQHandler(HASH_HandleTypeDef *hhash);
/**
  * @}
  */

/** @addtogroup HASH_Exported_Functions_Group8
  * @{
  */
HAL_HASH_StateTypeDef HAL_HASH_GetState(HASH_HandleTypeDef *hhash);
void HAL_HASH_MspInit(HASH_HandleTypeDef *hhash);
void HAL_HASH_MspDeInit(HASH_HandleTypeDef *hhash);
void HAL_HASH_InCpltCallback(HASH_HandleTypeDef *hhash);
void HAL_HASH_DgstCpltCallback(HASH_HandleTypeDef *hhash);
void HAL_HASH_ErrorCallback(HASH_HandleTypeDef *hhash);
/**
  * @}
  */

 /**
  * @}
  */

 /* Private types -------------------------------------------------------------*/
/** @defgroup HASH_Private_Types HASH Private Types
  * @{
  */

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup HASH_Private_Variables HASH Private Variables
  * @{
  */

/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup HASH_Private_Constants HASH Private Constants
  * @{
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup HASH_Private_Macros HASH Private Macros
  * @{
  */
#define IS_HASH_ALGOSELECTION(__ALGOSELECTION__) (((__ALGOSELECTION__) == HASH_ALGOSELECTION_SHA1)   || \
                                                  ((__ALGOSELECTION__) == HASH_ALGOSELECTION_SHA224) || \
                                                  ((__ALGOSELECTION__) == HASH_ALGOSELECTION_SHA256) || \
                                                  ((__ALGOSELECTION__) == HASH_ALGOSELECTION_MD5))


#define IS_HASH_ALGOMODE(__ALGOMODE__) (((__ALGOMODE__) == HASH_ALGOMODE_HASH) || \
                                        ((__ALGOMODE__) == HASH_ALGOMODE_HMAC))


#define IS_HASH_DATATYPE(__DATATYPE__) (((__DATATYPE__) == HASH_DATATYPE_32B)|| \
                                        ((__DATATYPE__) == HASH_DATATYPE_16B)|| \
                                        ((__DATATYPE__) == HASH_DATATYPE_8B) || \
                                        ((__DATATYPE__) == HASH_DATATYPE_1B))


#define IS_HASH_HMAC_KEYTYPE(__KEYTYPE__) (((__KEYTYPE__) == HASH_HMAC_KEYTYPE_SHORTKEY) || \
                                           ((__KEYTYPE__) == HASH_HMAC_KEYTYPE_LONGKEY))

#define IS_HASH_SHA1_BUFFER_SIZE(__SIZE__) ((((__SIZE__)%4U) != 0U)? 0U: 1U)

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup HASH_Private_Functions HASH Private Functions
  * @{
  */

/**
  * @}
  */

#endif /* STM32F415xx || STM32F417xx || STM32F437xx || STM32F439xx || STM32F479xx */
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif


#endif /* __STM32F4xx_HAL_HASH_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
