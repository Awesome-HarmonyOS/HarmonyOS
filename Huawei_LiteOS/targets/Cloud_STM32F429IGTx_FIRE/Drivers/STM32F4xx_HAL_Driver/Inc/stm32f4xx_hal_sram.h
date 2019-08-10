/**
  ******************************************************************************
  * @file    stm32f4xx_hal_sram.h
  * @author  MCD Application Team
  * @brief   Header file of SRAM HAL module.
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
#ifndef __STM32F4xx_HAL_SRAM_H
#define __STM32F4xx_HAL_SRAM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F412Zx) ||\
    defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F413xx) || defined(STM32F423xx)
  #include "stm32f4xx_ll_fsmc.h"
#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F412Zx || STM32F412Vx || STM32F412Rx || STM32F413xx || STM32F423xx */

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx)|| defined(STM32F439xx) ||\
 defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
  #include "stm32f4xx_ll_fmc.h"
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F446xx || STM32F469xx || STM32F479xx */


/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) ||\
    defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) ||\
    defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) ||\
    defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F413xx) || defined(STM32F423xx)

/** @addtogroup SRAM
  * @{
  */

/* Exported typedef ----------------------------------------------------------*/

/** @defgroup SRAM_Exported_Types SRAM Exported Types
  * @{
  */
/**
  * @brief  HAL SRAM State structures definition
  */
typedef enum
{
  HAL_SRAM_STATE_RESET     = 0x00U,  /*!< SRAM not yet initialized or disabled           */
  HAL_SRAM_STATE_READY     = 0x01U,  /*!< SRAM initialized and ready for use             */
  HAL_SRAM_STATE_BUSY      = 0x02U,  /*!< SRAM internal process is ongoing               */
  HAL_SRAM_STATE_ERROR     = 0x03U,  /*!< SRAM error state                               */
  HAL_SRAM_STATE_PROTECTED = 0x04U   /*!< SRAM peripheral NORSRAM device write protected */

}HAL_SRAM_StateTypeDef;

/**
  * @brief  SRAM handle Structure definition
  */
typedef struct
{
  FMC_NORSRAM_TypeDef           *Instance;  /*!< Register base address                        */

  FMC_NORSRAM_EXTENDED_TypeDef  *Extended;  /*!< Extended mode register base address          */

  FMC_NORSRAM_InitTypeDef       Init;       /*!< SRAM device control configuration parameters */

  HAL_LockTypeDef               Lock;       /*!< SRAM locking object                          */

  __IO HAL_SRAM_StateTypeDef    State;      /*!< SRAM device access state                     */

  DMA_HandleTypeDef             *hdma;      /*!< Pointer DMA handler                          */

}SRAM_HandleTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup SRAM_Exported_Macros SRAM Exported Macros
  * @{
  */
/** @brief Reset SRAM handle state
  * @param  __HANDLE__ SRAM handle
  * @retval None
  */
#define __HAL_SRAM_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_SRAM_STATE_RESET)

/**
  * @}
  */
/* Exported functions --------------------------------------------------------*/

/** @addtogroup SRAM_Exported_Functions
 *  @{
 */

/** @addtogroup SRAM_Exported_Functions_Group1
 *  @{
 */
/* Initialization/de-initialization functions  **********************************/
HAL_StatusTypeDef HAL_SRAM_Init(SRAM_HandleTypeDef *hsram, FMC_NORSRAM_TimingTypeDef *Timing, FMC_NORSRAM_TimingTypeDef *ExtTiming);
HAL_StatusTypeDef HAL_SRAM_DeInit(SRAM_HandleTypeDef *hsram);
void              HAL_SRAM_MspInit(SRAM_HandleTypeDef *hsram);
void              HAL_SRAM_MspDeInit(SRAM_HandleTypeDef *hsram);

void              HAL_SRAM_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma);
void              HAL_SRAM_DMA_XferErrorCallback(DMA_HandleTypeDef *hdma);
/**
  * @}
  */

/** @addtogroup SRAM_Exported_Functions_Group2
 *  @{
 */
/* I/O operation functions  *****************************************************/
HAL_StatusTypeDef HAL_SRAM_Read_8b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint8_t *pDstBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Write_8b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint8_t *pSrcBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Read_16b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint16_t *pDstBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Write_16b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint16_t *pSrcBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Read_32b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pDstBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Write_32b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pSrcBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Read_DMA(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pDstBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Write_DMA(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pSrcBuffer, uint32_t BufferSize);
/**
  * @}
  */

/** @addtogroup SRAM_Exported_Functions_Group3
 *  @{
 */
/* SRAM Control functions  ******************************************************/
HAL_StatusTypeDef HAL_SRAM_WriteOperation_Enable(SRAM_HandleTypeDef *hsram);
HAL_StatusTypeDef HAL_SRAM_WriteOperation_Disable(SRAM_HandleTypeDef *hsram);
/**
  * @}
  */

/** @addtogroup SRAM_Exported_Functions_Group4
 *  @{
 */
/* SRAM State functions *********************************************************/
HAL_SRAM_StateTypeDef HAL_SRAM_GetState(SRAM_HandleTypeDef *hsram);
/**
  * @}
  */

/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @}
  */

#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F427xx || STM32F437xx ||\
          STM32F429xx || STM32F439xx || STM32F446xx || STM32F469xx || STM32F479xx || STM32F412Zx ||\
          STM32F412Vx || STM32F412Rx || STM32F413xx || STM32F423xx */
/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_SRAM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
