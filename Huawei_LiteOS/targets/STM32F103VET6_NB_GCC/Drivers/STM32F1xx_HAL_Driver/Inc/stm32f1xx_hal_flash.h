/**
  ******************************************************************************
  * @file    stm32f1xx_hal_flash.h
  * @author  MCD Application Team
  * @brief   Header file of Flash HAL module.
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
#ifndef __STM32F1xx_HAL_FLASH_H
#define __STM32F1xx_HAL_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal_def.h"
   
/** @addtogroup STM32F1xx_HAL_Driver
  * @{
  */

/** @addtogroup FLASH
  * @{
  */
  
/** @addtogroup FLASH_Private_Constants
  * @{
  */
#define FLASH_TIMEOUT_VALUE              50000U /* 50 s */
/**
  * @}
  */

/** @addtogroup FLASH_Private_Macros
  * @{
  */

#define IS_FLASH_TYPEPROGRAM(VALUE)  (((VALUE) == FLASH_TYPEPROGRAM_HALFWORD) || \
                                      ((VALUE) == FLASH_TYPEPROGRAM_WORD)     || \
                                      ((VALUE) == FLASH_TYPEPROGRAM_DOUBLEWORD))  

#if   defined(FLASH_ACR_LATENCY)
#define IS_FLASH_LATENCY(__LATENCY__) (((__LATENCY__) == FLASH_LATENCY_0) || \
                                       ((__LATENCY__) == FLASH_LATENCY_1) || \
                                       ((__LATENCY__) == FLASH_LATENCY_2))

#else
#define IS_FLASH_LATENCY(__LATENCY__)   ((__LATENCY__) == FLASH_LATENCY_0)
#endif /* FLASH_ACR_LATENCY */
/**
  * @}
  */  

/* Exported types ------------------------------------------------------------*/ 
/** @defgroup FLASH_Exported_Types FLASH Exported Types
  * @{
  */  

/**
  * @brief  FLASH Procedure structure definition
  */
typedef enum 
{
  FLASH_PROC_NONE              = 0U, 
  FLASH_PROC_PAGEERASE         = 1U,
  FLASH_PROC_MASSERASE         = 2U,
  FLASH_PROC_PROGRAMHALFWORD   = 3U,
  FLASH_PROC_PROGRAMWORD       = 4U,
  FLASH_PROC_PROGRAMDOUBLEWORD = 5U
} FLASH_ProcedureTypeDef;

/** 
  * @brief  FLASH handle Structure definition  
  */
typedef struct
{
  __IO FLASH_ProcedureTypeDef ProcedureOnGoing; /*!< Internal variable to indicate which procedure is ongoing or not in IT context */
  
  __IO uint32_t               DataRemaining;    /*!< Internal variable to save the remaining pages to erase or half-word to program in IT context */

  __IO uint32_t               Address;          /*!< Internal variable to save address selected for program or erase */

  __IO uint64_t               Data;             /*!< Internal variable to save data to be programmed */

  HAL_LockTypeDef             Lock;             /*!< FLASH locking object                */

  __IO uint32_t               ErrorCode;        /*!< FLASH error code                    
                                                     This parameter can be a value of @ref FLASH_Error_Codes  */
} FLASH_ProcessTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup FLASH_Exported_Constants FLASH Exported Constants
  * @{
  */  

/** @defgroup FLASH_Error_Codes FLASH Error Codes
  * @{
  */

#define HAL_FLASH_ERROR_NONE      0x00U  /*!< No error */
#define HAL_FLASH_ERROR_PROG      0x01U  /*!< Programming error */
#define HAL_FLASH_ERROR_WRP       0x02U  /*!< Write protection error */
#define HAL_FLASH_ERROR_OPTV      0x04U  /*!< Option validity error */

/**
  * @}
  */

/** @defgroup FLASH_Type_Program FLASH Type Program
  * @{
  */ 
#define FLASH_TYPEPROGRAM_HALFWORD             0x01U  /*!<Program a half-word (16-bit) at a specified address.*/
#define FLASH_TYPEPROGRAM_WORD                 0x02U  /*!<Program a word (32-bit) at a specified address.*/
#define FLASH_TYPEPROGRAM_DOUBLEWORD           0x03U  /*!<Program a double word (64-bit) at a specified address*/

/**
  * @}
  */

#if   defined(FLASH_ACR_LATENCY)
/** @defgroup FLASH_Latency FLASH Latency
  * @{
  */
#define FLASH_LATENCY_0            0x00000000U               /*!< FLASH Zero Latency cycle */
#define FLASH_LATENCY_1            FLASH_ACR_LATENCY_0       /*!< FLASH One Latency cycle */
#define FLASH_LATENCY_2            FLASH_ACR_LATENCY_1       /*!< FLASH Two Latency cycles */

/**
  * @}
  */

#else
/** @defgroup FLASH_Latency FLASH Latency
  * @{
  */
#define FLASH_LATENCY_0            0x00000000U    /*!< FLASH Zero Latency cycle */

/**
  * @}
  */

#endif /* FLASH_ACR_LATENCY */
/**
  * @}
  */  
  
/* Exported macro ------------------------------------------------------------*/

/** @defgroup FLASH_Exported_Macros FLASH Exported Macros
 *  @brief macros to control FLASH features 
 *  @{
 */
 
/** @defgroup FLASH_Half_Cycle FLASH Half Cycle
 *  @brief macros to handle FLASH half cycle
 * @{
 */

/**
  * @brief  Enable the FLASH half cycle access.
  * @note   half cycle access can only be used with a low-frequency clock of less than
            8 MHz that can be obtained with the use of HSI or HSE but not of PLL.
  * @retval None
  */
#define __HAL_FLASH_HALF_CYCLE_ACCESS_ENABLE()  (FLASH->ACR |= FLASH_ACR_HLFCYA)

/**
  * @brief  Disable the FLASH half cycle access.
  * @note   half cycle access can only be used with a low-frequency clock of less than
            8 MHz that can be obtained with the use of HSI or HSE but not of PLL.
  * @retval None
  */
#define __HAL_FLASH_HALF_CYCLE_ACCESS_DISABLE() (FLASH->ACR &= (~FLASH_ACR_HLFCYA))

/**
  * @}
  */

#if defined(FLASH_ACR_LATENCY)
/** @defgroup FLASH_EM_Latency FLASH Latency
 *  @brief macros to handle FLASH Latency
 * @{
 */ 
  
/**
  * @brief  Set the FLASH Latency.
  * @param  __LATENCY__ FLASH Latency                   
  *         The value of this parameter depend on device used within the same series
  * @retval None
  */ 
#define __HAL_FLASH_SET_LATENCY(__LATENCY__)    (FLASH->ACR = (FLASH->ACR&(~FLASH_ACR_LATENCY)) | (__LATENCY__))


/**
  * @brief  Get the FLASH Latency.
  * @retval FLASH Latency                   
  *         The value of this parameter depend on device used within the same series
  */ 
#define __HAL_FLASH_GET_LATENCY()     (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY))

/**
  * @}
  */

#endif /* FLASH_ACR_LATENCY */
/** @defgroup FLASH_Prefetch FLASH Prefetch
 *  @brief macros to handle FLASH Prefetch buffer
 * @{
 */   
/**
  * @brief  Enable the FLASH prefetch buffer.
  * @retval None
  */ 
#define __HAL_FLASH_PREFETCH_BUFFER_ENABLE()    (FLASH->ACR |= FLASH_ACR_PRFTBE)

/**
  * @brief  Disable the FLASH prefetch buffer.
  * @retval None
  */
#define __HAL_FLASH_PREFETCH_BUFFER_DISABLE()   (FLASH->ACR &= (~FLASH_ACR_PRFTBE))

/**
  * @}
  */
  
/**
  * @}
  */ 

/* Include FLASH HAL Extended module */
#include "stm32f1xx_hal_flash_ex.h"  

/* Exported functions --------------------------------------------------------*/
/** @addtogroup FLASH_Exported_Functions
  * @{
  */
  
/** @addtogroup FLASH_Exported_Functions_Group1
  * @{
  */
/* IO operation functions *****************************************************/
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data);

/* FLASH IRQ handler function */
void       HAL_FLASH_IRQHandler(void);
/* Callbacks in non blocking modes */ 
void       HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void       HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);

/**
  * @}
  */

/** @addtogroup FLASH_Exported_Functions_Group2
  * @{
  */
/* Peripheral Control functions ***********************************************/
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Lock(void);
void HAL_FLASH_OB_Launch(void);

/**
  * @}
  */

/** @addtogroup FLASH_Exported_Functions_Group3
  * @{
  */
/* Peripheral State and Error functions ***************************************/
uint32_t HAL_FLASH_GetError(void);

/**
  * @}
  */

/**
  * @}
  */

/* Private function -------------------------------------------------*/
/** @addtogroup FLASH_Private_Functions
 * @{
 */
HAL_StatusTypeDef       FLASH_WaitForLastOperation(uint32_t Timeout);
#if defined(FLASH_BANK2_END)
HAL_StatusTypeDef       FLASH_WaitForLastOperationBank2(uint32_t Timeout);
#endif /* FLASH_BANK2_END */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_HAL_FLASH_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

