/**
  ******************************************************************************
  * @file    stm32f4xx_hal_fmpi2c_ex.h
  * @author  MCD Application Team
  * @brief   Header file of FMPI2C HAL Extended module.
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
#ifndef __STM32F4xx_HAL_FMPI2C_EX_H
#define __STM32F4xx_HAL_FMPI2C_EX_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx) || defined(STM32F446xx) || defined(STM32F412Zx) ||\
    defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx)

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal_def.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @addtogroup FMPI2CEx
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/** @defgroup FMPI2CEx_Exported_Constants FMPI2C Extended Exported Constants
  * @{
  */

/** @defgroup FMPI2CEx_Analog_Filter FMPI2C Extended Analog Filter
  * @{
  */
#define FMPI2C_ANALOGFILTER_ENABLE         0x00000000U
#define FMPI2C_ANALOGFILTER_DISABLE        FMPI2C_CR1_ANFOFF
/**
  * @}
  */

/** @defgroup FMPI2CEx_FastModePlus FMPI2C Extended Fast Mode Plus
  * @{
  */
#define FMPI2C_FASTMODEPLUS_SCL            SYSCFG_CFGR_FMPI2C1_SCL  /*!< Enable Fast Mode Plus on FMPI2C1 SCL pins       */
#define FMPI2C_FASTMODEPLUS_SDA            SYSCFG_CFGR_FMPI2C1_SDA  /*!< Enable Fast Mode Plus on FMPI2C1 SDA pins       */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @addtogroup FMPI2CEx_Exported_Functions FMPI2C Extended Exported Functions
  * @{
  */

/** @addtogroup FMPI2CEx_Exported_Functions_Group1 Extended features functions
  * @brief    Extended features functions
  * @{
  */

/* Peripheral Control functions  ************************************************/
HAL_StatusTypeDef HAL_FMPI2CEx_ConfigAnalogFilter(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t AnalogFilter);
HAL_StatusTypeDef HAL_FMPI2CEx_ConfigDigitalFilter(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t DigitalFilter);
void HAL_FMPI2CEx_EnableFastModePlus(uint32_t ConfigFastModePlus);
void HAL_FMPI2CEx_DisableFastModePlus(uint32_t ConfigFastModePlus);

/* Private constants ---------------------------------------------------------*/
/** @defgroup FMPI2CEx_Private_Constants FMPI2C Extended Private Constants
  * @{
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup FMPI2CEx_Private_Macro FMPI2C Extended Private Macros
  * @{
  */
#define IS_FMPI2C_ANALOG_FILTER(FILTER)    (((FILTER) == FMPI2C_ANALOGFILTER_ENABLE) || \
                                            ((FILTER) == FMPI2C_ANALOGFILTER_DISABLE))

#define IS_FMPI2C_DIGITAL_FILTER(FILTER)   ((FILTER) <= 0x0000000FU)

#define IS_FMPI2C_FASTMODEPLUS(__CONFIG__) ((((__CONFIG__) & (FMPI2C_FASTMODEPLUS_SCL)) == FMPI2C_FASTMODEPLUS_SCL) || \
                                            (((__CONFIG__) & (FMPI2C_FASTMODEPLUS_SDA)) == FMPI2C_FASTMODEPLUS_SDA))
/**
  * @}
  */

/* Private Functions ---------------------------------------------------------*/
/** @defgroup FMPI2CEx_Private_Functions FMPI2C Extended Private Functions
  * @{
  */
/* Private functions are defined in stm32f4xx_hal_fmpi2c_ex.c file */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* STM32F410xx || STM32F446xx || STM32F412Zx || STM32F412Vx || STM32F412Rx || STM32F412Cx || STM32F413xx || STM32F423xx */
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_FMPI2C_EX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
