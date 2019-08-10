/**
  ******************************************************************************
  * @file    stm32f4xx_hal_hash_ex.h
  * @author  MCD Application Team
  * @brief   Header file of HASH HAL Extension module.
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
#ifndef __STM32F4xx_HAL_HASH_EX_H
#define __STM32F4xx_HAL_HASH_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

#if defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F479xx)
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal_def.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @addtogroup HASHEx
  * @brief HASHEx HAL Extension module driver
  *  @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup HASHEx_Exported_Functions HASHEx Exported Functions
  * @{
  */

/** @defgroup HASHEx_Exported_Functions_Group1 HASHEx processing using polling functions
  * @{
  */

HAL_StatusTypeDef HAL_HASHEx_SHA224_Start(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t* pOutBuffer, uint32_t Timeout);
HAL_StatusTypeDef HAL_HASHEx_SHA256_Start(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t* pOutBuffer, uint32_t Timeout);
HAL_StatusTypeDef HAL_HASHEx_SHA224_Accumulate(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size);
HAL_StatusTypeDef HAL_HASHEx_SHA256_Accumulate(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size);

/**
  * @}
  */

/** @defgroup HASHEx_Exported_Functions_Group2 HMAC processing using polling functions
  * @{
  */

HAL_StatusTypeDef HAL_HMACEx_SHA224_Start(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t* pOutBuffer, uint32_t Timeout);
HAL_StatusTypeDef HAL_HMACEx_SHA256_Start(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t* pOutBuffer, uint32_t Timeout);

/**
  * @}
  */

/** @defgroup HASHEx_Exported_Functions_Group3 HASHEx processing using  functions
  * @{
  */

HAL_StatusTypeDef HAL_HASHEx_SHA224_Start_IT(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t* pOutBuffer);
HAL_StatusTypeDef HAL_HASHEx_SHA256_Start_IT(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t* pOutBuffer);

/**
  * @}
  */

/** @defgroup HASHEx_Exported_Functions_Group4 HASHEx processing using DMA
  * @{
  */

HAL_StatusTypeDef HAL_HASHEx_SHA224_Start_DMA(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size);
HAL_StatusTypeDef HAL_HASHEx_SHA224_Finish(HASH_HandleTypeDef *hhash, uint8_t* pOutBuffer, uint32_t Timeout);
HAL_StatusTypeDef HAL_HASHEx_SHA256_Start_DMA(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size);
HAL_StatusTypeDef HAL_HASHEx_SHA256_Finish(HASH_HandleTypeDef *hhash, uint8_t* pOutBuffer, uint32_t Timeout);

/**
  * @}
  */

/** @defgroup HASHEx_Exported_Functions_Group5 HMAC processing using DMA
  * @{
  */

HAL_StatusTypeDef HAL_HMACEx_SHA224_Start_DMA(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size);
HAL_StatusTypeDef HAL_HMACEx_SHA256_Start_DMA(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size);
/**
  * @}
  */

/** @defgroup HASHEx_Exported_Functions_Group6 HASHEx processing functions
  * @{
  */

void HAL_HASHEx_IRQHandler(HASH_HandleTypeDef *hhash);

/**
  * @}
  */

/**
  * @}
  */

 /* Private types -------------------------------------------------------------*/
/** @defgroup HASHEx_Private_Types HASHEx Private Types
  * @{
  */

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup HASHEx_Private_Variables HASHEx Private Variables
  * @{
  */

/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup HASHEx_Private_Constants HASHEx Private Constants
  * @{
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup HASHEx_Private_Macros HASHEx Private Macros
  * @{
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup HASHEx_Private_Functions HASHEx Private Functions
  * @{
  */

/**
  * @}
  */

#endif /* STM32F437xx || STM32F439xx || STM32F479xx */
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_HASH_EX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
