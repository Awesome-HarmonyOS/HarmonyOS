/**
  ******************************************************************************
  * @file    stm32f1xx_ll_crc.h
  * @author  MCD Application Team
  * @brief   Header file of CRC LL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#ifndef __STM32F1xx_LL_CRC_H
#define __STM32F1xx_LL_CRC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"

/** @addtogroup STM32F1xx_LL_Driver
  * @{
  */

#if defined(CRC)

/** @defgroup CRC_LL CRC
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/** @defgroup CRC_LL_Exported_Macros CRC Exported Macros
  * @{
  */

/** @defgroup CRC_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in CRC register
  * @param  __INSTANCE__ CRC Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_CRC_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in CRC register
  * @param  __INSTANCE__ CRC Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_CRC_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @defgroup CRC_LL_Exported_Functions CRC Exported Functions
  * @{
  */

/** @defgroup CRC_LL_EF_Configuration CRC Configuration functions
  * @{
  */

/**
  * @brief  Reset the CRC calculation unit.
  * @rmtoll CR           RESET         LL_CRC_ResetCRCCalculationUnit
  * @param  CRCx CRC Instance
  * @retval None
  */
__STATIC_INLINE void LL_CRC_ResetCRCCalculationUnit(CRC_TypeDef *CRCx)
{
  WRITE_REG(CRCx->CR, CRC_CR_RESET);
}

/**
  * @}
  */

/** @defgroup CRC_LL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Write given 32-bit data to the CRC calculator
  * @rmtoll DR           DR            LL_CRC_FeedData32
  * @param  CRCx CRC Instance
  * @param  InData value to be provided to CRC calculator between between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void LL_CRC_FeedData32(CRC_TypeDef *CRCx, uint32_t InData)
{
  WRITE_REG(CRCx->DR, InData);
}

/**
  * @brief  Return current CRC calculation result. 32 bits value is returned.
  * @rmtoll DR           DR            LL_CRC_ReadData32
  * @param  CRCx CRC Instance
  * @retval Current CRC calculation result as stored in CRC_DR register (32 bits).
  */
__STATIC_INLINE uint32_t LL_CRC_ReadData32(CRC_TypeDef *CRCx)
{
  return (uint32_t)(READ_REG(CRCx->DR));
}

/**
  * @brief  Return data stored in the Independent Data(IDR) register.
  * @note   This register can be used as a temporary storage location for one byte.
  * @rmtoll IDR          IDR           LL_CRC_Read_IDR
  * @param  CRCx CRC Instance
  * @retval Value stored in CRC_IDR register (General-purpose 8-bit data register).
  */
__STATIC_INLINE uint32_t LL_CRC_Read_IDR(CRC_TypeDef *CRCx)
{
  return (uint32_t)(READ_REG(CRCx->IDR));
}

/**
  * @brief  Store data in the Independent Data(IDR) register.
  * @note   This register can be used as a temporary storage location for one byte.
  * @rmtoll IDR          IDR           LL_CRC_Write_IDR
  * @param  CRCx CRC Instance
  * @param  InData value to be stored in CRC_IDR register (8-bit) between between Min_Data=0 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void LL_CRC_Write_IDR(CRC_TypeDef *CRCx, uint32_t InData)
{
  *((uint8_t __IO *)(&CRCx->IDR)) = (uint8_t) InData;
}
/**
  * @}
  */

#if defined(USE_FULL_LL_DRIVER)
/** @defgroup CRC_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

ErrorStatus LL_CRC_DeInit(CRC_TypeDef *CRCx);

/**
  * @}
  */
#endif /* USE_FULL_LL_DRIVER */

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined(CRC) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_LL_CRC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
