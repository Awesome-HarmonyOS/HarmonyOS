/**
  ******************************************************************************
  * @file    stm32f4xx_hal_sai_ex.h
  * @author  MCD Application Team
  * @brief   Header file of SAI Extension HAL module.
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
#ifndef __STM32F4xx_HAL_SAI_EX_H
#define __STM32F4xx_HAL_SAI_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal_def.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @addtogroup SAIEx
  * @{
  */

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
    defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F413xx) || \
    defined(STM32F423xx)

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup SAI_Clock_Source  SAI Clock Source
  * @{
  */
#if defined(STM32F413xx) || defined(STM32F423xx)
#define SAI_CLKSOURCE_PLLI2S             0x00000000U
#define SAI_CLKSOURCE_EXT                0x00100000U
#define SAI_CLKSOURCE_PLLR               0x00200000U
#define SAI_CLKSOURCE_HS                 0x00300000U
#else
#define SAI_CLKSOURCE_PLLSAI             0x00000000U
#define SAI_CLKSOURCE_PLLI2S             0x00100000U
#define SAI_CLKSOURCE_EXT                0x00200000U
#define SAI_CLKSOURCE_NA                 0x00400000U /*!< No applicable for STM32F446xx */
#endif


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SAIEx_Exported_Functions
  * @{
  */

/** @addtogroup SAIEx_Exported_Functions_Group1
  * @{
  */

/* Extended features functions ************************************************/
void SAI_BlockSynchroConfig(SAI_HandleTypeDef *hsai);
uint32_t SAI_GetInputClock(SAI_HandleTypeDef *hsai);
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
#if defined(STM32F413xx) || defined(STM32F423xx)
#define IS_SAI_CLK_SOURCE(SOURCE) (((SOURCE) == SAI_CLKSOURCE_PLLI2S) ||\
                                   ((SOURCE) == SAI_CLKSOURCE_EXT)||\
                                   ((SOURCE) == SAI_CLKSOURCE_PLLR)||\
                                   ((SOURCE) == SAI_CLKSOURCE_HS))
#else
#define IS_SAI_CLK_SOURCE(SOURCE) (((SOURCE) == SAI_CLKSOURCE_PLLSAI) ||\
                                   ((SOURCE) == SAI_CLKSOURCE_EXT)||\
                                   ((SOURCE) == SAI_CLKSOURCE_PLLI2S)||\
                                   ((SOURCE) == SAI_CLKSOURCE_NA))
#endif
/* Private functions ---------------------------------------------------------*/

#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F446xx || STM32F469xx || STM32F479xx || STM32F413xx || STM32F423xx */
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_SAI_EX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
