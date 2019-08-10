/**
  ******************************************************************************
  * @file    stm32f1xx_ll_rtc.h
  * @author  MCD Application Team
  * @brief   Header file of RTC LL module.
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
#ifndef __STM32F1xx_LL_RTC_H
#define __STM32F1xx_LL_RTC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"

/** @addtogroup STM32F1xx_LL_Driver
  * @{
  */

#if defined(RTC)

/** @defgroup RTC_LL RTC
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
#if defined(USE_FULL_LL_DRIVER)
/** @defgroup RTC_LL_Private_Macros RTC Private Macros
  * @{
  */
/**
  * @}
  */
#endif /*USE_FULL_LL_DRIVER*/

/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_LL_DRIVER)
/** @defgroup RTC_LL_ES_INIT RTC Exported Init structure
  * @{
  */

/**
  * @brief  RTC Init structures definition
  */
typedef struct
{
  uint32_t AsynchPrescaler; /*!< Specifies the RTC Asynchronous Predivider value.
                              This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFFFFF
                              
                              This feature can be modified afterwards using unitary function
                              @ref LL_RTC_SetAsynchPrescaler(). */

  uint32_t OutPutSource;    /*!< Specifies which signal will be routed to the RTC Tamper pin.
                                 This parameter can be a value of @ref LL_RTC_Output_Source 
                              
                              This feature can be modified afterwards using unitary function
                              @ref LL_RTC_SetOutputSource(). */

} LL_RTC_InitTypeDef;

/**
  * @brief  RTC Time structure definition
  */
typedef struct
{
  uint8_t Hours;       /*!< Specifies the RTC Time Hours.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 23 */

  uint8_t Minutes;     /*!< Specifies the RTC Time Minutes.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

  uint8_t Seconds;     /*!< Specifies the RTC Time Seconds.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 59 */
} LL_RTC_TimeTypeDef;


/**
  * @brief  RTC Alarm structure definition
  */
typedef struct
{
  LL_RTC_TimeTypeDef AlarmTime;  /*!< Specifies the RTC Alarm Time members. */

} LL_RTC_AlarmTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_LL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RTC_LL_Exported_Constants RTC Exported Constants
  * @{
  */

#if defined(USE_FULL_LL_DRIVER)
/** @defgroup RTC_LL_EC_FORMAT FORMAT
  * @{
  */
#define LL_RTC_FORMAT_BIN                  (0x000000000U) /*!< Binary data format */
#define LL_RTC_FORMAT_BCD                  (0x000000001U) /*!< BCD data format */
/**
  * @}
  */
#endif /* USE_FULL_LL_DRIVER */

/** @defgroup RTC_LL_EC_BKP  BACKUP
  * @{
  */
#if RTC_BKP_NUMBER > 0
#define LL_RTC_BKP_DR1                     (0x00000001U)
#define LL_RTC_BKP_DR2                     (0x00000002U)
#define LL_RTC_BKP_DR3                     (0x00000003U)
#define LL_RTC_BKP_DR4                     (0x00000004U)
#define LL_RTC_BKP_DR5                     (0x00000005U)
#define LL_RTC_BKP_DR6                     (0x00000006U)
#define LL_RTC_BKP_DR7                     (0x00000007U)
#define LL_RTC_BKP_DR8                     (0x00000008U)
#define LL_RTC_BKP_DR9                     (0x00000009U)
#define LL_RTC_BKP_DR10                    (0x0000000AU)
#endif /* RTC_BKP_NUMBER > 0 */
#if RTC_BKP_NUMBER > 10
#define LL_RTC_BKP_DR11                    (0x0000000BU)
#define LL_RTC_BKP_DR12                    (0x0000000CU)
#define LL_RTC_BKP_DR13                    (0x0000000DU)
#define LL_RTC_BKP_DR14                    (0x0000000EU)
#define LL_RTC_BKP_DR15                    (0x0000000FU)
#define LL_RTC_BKP_DR16                    (0x00000010U)
#define LL_RTC_BKP_DR17                    (0x00000011U)
#define LL_RTC_BKP_DR18                    (0x00000012U)
#define LL_RTC_BKP_DR19                    (0x00000013U)
#define LL_RTC_BKP_DR20                    (0x00000014U)
#define LL_RTC_BKP_DR21                    (0x00000015U)
#define LL_RTC_BKP_DR22                    (0x00000016U)
#define LL_RTC_BKP_DR23                    (0x00000017U)
#define LL_RTC_BKP_DR24                    (0x00000018U)
#define LL_RTC_BKP_DR25                    (0x00000019U)
#define LL_RTC_BKP_DR26                    (0x0000001AU)
#define LL_RTC_BKP_DR27                    (0x0000001BU)
#define LL_RTC_BKP_DR28                    (0x0000001CU)
#define LL_RTC_BKP_DR29                    (0x0000001DU)
#define LL_RTC_BKP_DR30                    (0x0000001EU)
#define LL_RTC_BKP_DR31                    (0x0000001FU)
#define LL_RTC_BKP_DR32                    (0x00000020U)
#define LL_RTC_BKP_DR33                    (0x00000021U)
#define LL_RTC_BKP_DR34                    (0x00000022U)
#define LL_RTC_BKP_DR35                    (0x00000023U)
#define LL_RTC_BKP_DR36                    (0x00000024U)
#define LL_RTC_BKP_DR37                    (0x00000025U)
#define LL_RTC_BKP_DR38                    (0x00000026U)
#define LL_RTC_BKP_DR39                    (0x00000027U)
#define LL_RTC_BKP_DR40                    (0x00000028U)
#define LL_RTC_BKP_DR41                    (0x00000029U)
#define LL_RTC_BKP_DR42                    (0x0000002AU)
#endif /* RTC_BKP_NUMBER > 10 */

/**
  * @}
  */

/** @defgroup RTC_LL_EC_TAMPLEVEL  Tamper Active Level
  * @{
  */ 
#define LL_RTC_TAMPER_ACTIVELEVEL_LOW          BKP_CR_TPAL           /*!< A high level on the TAMPER pin resets all data backup registers (if TPE bit is set) */
#define LL_RTC_TAMPER_ACTIVELEVEL_HIGH         (0x00000000U)         /*!< A low level on the TAMPER pin resets all data backup registers (if TPE bit is set) */

/**
  * @}
  */

/** @defgroup LL_RTC_Output_Source         Clock Source to output on the Tamper Pin
  * @{
  */
#define LL_RTC_CALIB_OUTPUT_NONE           (0x00000000U)                       /*!< Calibration output disabled */
#define LL_RTC_CALIB_OUTPUT_RTCCLOCK       BKP_RTCCR_CCO                       /*!< Calibration output is RTC Clock with a frequency divided by 64 on the TAMPER Pin */
#define LL_RTC_CALIB_OUTPUT_ALARM          BKP_RTCCR_ASOE                      /*!< Calibration output is Alarm pulse signal on the TAMPER pin */
#define LL_RTC_CALIB_OUTPUT_SECOND        (BKP_RTCCR_ASOS | BKP_RTCCR_ASOE)    /*!< Calibration output is Second pulse signal on the TAMPER pin*/
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup RTC_LL_Exported_Macros RTC Exported Macros
  * @{
  */

/** @defgroup RTC_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in RTC register
  * @param  __INSTANCE__ RTC Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_RTC_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in RTC register
  * @param  __INSTANCE__ RTC Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_RTC_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/** @defgroup RTC_LL_EM_Convert Convert helper Macros
  * @{
  */

/**
  * @brief  Helper macro to convert a value from 2 digit decimal format to BCD format
  * @param  __VALUE__ Byte to be converted
  * @retval Converted byte
  */
#define __LL_RTC_CONVERT_BIN2BCD(__VALUE__) (uint8_t)((((__VALUE__) / 10U) << 4U) | ((__VALUE__) % 10U))

/**
  * @brief  Helper macro to convert a value from BCD format to 2 digit decimal format
  * @param  __VALUE__ BCD value to be converted
  * @retval Converted byte
  */
#define __LL_RTC_CONVERT_BCD2BIN(__VALUE__) (uint8_t)(((uint8_t)((__VALUE__) & (uint8_t)0xF0U) >> (uint8_t)0x4U) * 10U + ((__VALUE__) & (uint8_t)0x0FU))

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup RTC_LL_Exported_Functions RTC Exported Functions
  * @{
  */

/** @defgroup RTC_LL_EF_Configuration Configuration
  * @{
  */

/**
  * @brief  Set Asynchronous prescaler factor
  * @rmtoll PRLH         PRL      LL_RTC_SetAsynchPrescaler\n
  * @rmtoll PRLL         PRL      LL_RTC_SetAsynchPrescaler\n
  * @param  RTCx RTC Instance
  * @param  AsynchPrescaler Value between Min_Data = 0 and Max_Data = 0xFFFFF
  * @retval None
  */
__STATIC_INLINE void LL_RTC_SetAsynchPrescaler(RTC_TypeDef *RTCx, uint32_t AsynchPrescaler)
{
  MODIFY_REG(RTCx->PRLH, RTC_PRLH_PRL, (AsynchPrescaler >> 16));
  MODIFY_REG(RTCx->PRLL, RTC_PRLL_PRL, (AsynchPrescaler & RTC_PRLL_PRL));
}

/**
  * @brief  Get Asynchronous prescaler factor
  * @rmtoll DIVH         DIV      LL_RTC_GetDivider\n
  * @rmtoll DIVL         DIV      LL_RTC_GetDivider\n
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data = 0 and Max_Data = 0xFFFFF
  */
__STATIC_INLINE uint32_t LL_RTC_GetDivider(RTC_TypeDef *RTCx)
{
  register uint16_t Highprescaler = 0 , Lowprescaler = 0;
  Highprescaler = READ_REG(RTCx->DIVH & RTC_DIVH_RTC_DIV);
  Lowprescaler  = READ_REG(RTCx->DIVL & RTC_DIVL_RTC_DIV);
  
  return (((uint32_t) Highprescaler << 16U) | Lowprescaler);
}

/**
  * @brief  Set Output Source
  * @rmtoll RTCCR         CCO      LL_RTC_SetOutputSource
  * @rmtoll RTCCR         ASOE     LL_RTC_SetOutputSource
  * @rmtoll RTCCR         ASOS     LL_RTC_SetOutputSource
  * @param  BKPx BKP Instance
  * @param  OutputSource This parameter can be one of the following values:
  *         @arg @ref LL_RTC_CALIB_OUTPUT_NONE
  *         @arg @ref LL_RTC_CALIB_OUTPUT_RTCCLOCK
  *         @arg @ref LL_RTC_CALIB_OUTPUT_ALARM
  *         @arg @ref LL_RTC_CALIB_OUTPUT_SECOND
  * @retval None
  */
__STATIC_INLINE void LL_RTC_SetOutputSource(BKP_TypeDef *BKPx, uint32_t OutputSource)
{
  MODIFY_REG(BKPx->RTCCR, (BKP_RTCCR_CCO | BKP_RTCCR_ASOE | BKP_RTCCR_ASOS), OutputSource);
}

/**
  * @brief  Get Output Source
  * @rmtoll RTCCR         CCO      LL_RTC_GetOutPutSource
  * @rmtoll RTCCR         ASOE     LL_RTC_GetOutPutSource
  * @rmtoll RTCCR         ASOS     LL_RTC_GetOutPutSource
  * @param  BKPx BKP Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RTC_CALIB_OUTPUT_NONE
  *         @arg @ref LL_RTC_CALIB_OUTPUT_RTCCLOCK
  *         @arg @ref LL_RTC_CALIB_OUTPUT_ALARM
  *         @arg @ref LL_RTC_CALIB_OUTPUT_SECOND
  */
__STATIC_INLINE uint32_t LL_RTC_GetOutPutSource(BKP_TypeDef *BKPx)
{
  return (uint32_t)(READ_BIT(BKPx->RTCCR, (BKP_RTCCR_CCO | BKP_RTCCR_ASOE | BKP_RTCCR_ASOS)));
}

/**
  * @brief  Enable the write protection for RTC registers.
  * @rmtoll CRL          CNF           LL_RTC_EnableWriteProtection
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void LL_RTC_EnableWriteProtection(RTC_TypeDef *RTCx)
{
 CLEAR_BIT(RTCx->CRL, RTC_CRL_CNF);
}

/**
  * @brief  Disable the write protection for RTC registers.
  * @rmtoll CRL          RTC_CRL_CNF           LL_RTC_DisableWriteProtection
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void LL_RTC_DisableWriteProtection(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CRL, RTC_CRL_CNF);
}

/**
  * @}
  */

/** @defgroup RTC_LL_EF_Time Time
  * @{
  */

/**
  * @brief  Set time counter in BCD format
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function should be called before.
  * @note   It can be written in initialization mode only (@ref LL_RTC_EnterInitMode function)
  * @rmtoll CNTH         CNT            LL_RTC_TIME_Set\n
  *         CNTL         CNT            LL_RTC_TIME_Set\n
  * @param  RTCx RTC Instance
  * @param  TimeCounter Value between Min_Data=0x00 and Max_Data=0xFFFFF
  * @retval None
  */
__STATIC_INLINE void LL_RTC_TIME_Set(RTC_TypeDef *RTCx, uint32_t TimeCounter)
{
    /* Set RTC COUNTER MSB word */
    WRITE_REG(RTCx->CNTH, (TimeCounter >> 16U));
    /* Set RTC COUNTER LSB word */
    WRITE_REG(RTCx->CNTL, (TimeCounter & RTC_CNTL_RTC_CNT));
}

/**
  * @brief  Get time counter in BCD format
  * @rmtoll CNTH         CNT            LL_RTC_TIME_Get\n
  *         CNTL         CNT            LL_RTC_TIME_Get\n
  * @param  RTCx RTC Instance
  * @retval Value between Min_Data = 0 and Max_Data = 0xFFFFF
  */
__STATIC_INLINE uint32_t LL_RTC_TIME_Get(RTC_TypeDef *RTCx)
{
  register uint16_t high = 0, low = 0;
  
  high = READ_REG(RTCx->CNTH & RTC_CNTH_RTC_CNT);
  low  = READ_REG(RTCx->CNTL & RTC_CNTL_RTC_CNT);
  return ((uint32_t)(((uint32_t) high << 16U) | low));
}

/**
  * @}
  */

/** @defgroup RTC_LL_EF_ALARM  ALARM
  * @{
  */

/**
  * @brief  Set Alarm Counter
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function should be called before.
  * @rmtoll ALRH           ALR         LL_RTC_ALARM_Set\n
  * @rmtoll ALRL           ALR         LL_RTC_ALARM_Set\n
  * @param  RTCx RTC Instance
  * @param  AlarmCounter Value between Min_Data=0x00 and Max_Data=0xFFFFF
  * @retval None
  */
__STATIC_INLINE void LL_RTC_ALARM_Set(RTC_TypeDef *RTCx, uint32_t AlarmCounter)
{
  /* Set RTC COUNTER MSB word */
  WRITE_REG(RTCx->ALRH, (AlarmCounter >> 16));
  /* Set RTC COUNTER LSB word */
  WRITE_REG(RTCx->ALRL, (AlarmCounter & RTC_ALRL_RTC_ALR));
}

/**
  * @brief  Get Alarm Counter
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function should be called before.
  * @rmtoll ALRH           ALR         LL_RTC_ALARM_Get\n
  * @rmtoll ALRL           ALR         LL_RTC_ALARM_Get\n
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE uint32_t LL_RTC_ALARM_Get(RTC_TypeDef *RTCx)
{
  register uint16_t high = 0, low = 0;

  high  = READ_REG(RTCx->ALRH & RTC_ALRH_RTC_ALR);
  low   = READ_REG(RTCx->ALRL & RTC_ALRL_RTC_ALR);

  return (((uint32_t) high << 16U) | low);
}

/**
  * @}
  */

/** @defgroup RTC_LL_EF_Tamper Tamper
  * @{
  */

/**
  * @brief  Enable RTC_TAMPx input detection
  * @rmtoll CR    TPE        LL_RTC_TAMPER_Enable\n
  * @retval None
  */
__STATIC_INLINE void LL_RTC_TAMPER_Enable(BKP_TypeDef *BKPx)
{
  SET_BIT(BKPx->CR, BKP_CR_TPE);
}

/**
  * @brief  Disable RTC_TAMPx Tamper
  * @rmtoll CR    TPE        LL_RTC_TAMPER_Disable\n
  * @retval None
  */
__STATIC_INLINE void LL_RTC_TAMPER_Disable(BKP_TypeDef *BKPx)
{
  CLEAR_BIT(BKP->CR, BKP_CR_TPE);
}

/**
  * @brief  Enable Active level for Tamper input
  * @rmtoll CR    TPAL        LL_RTC_TAMPER_SetActiveLevel\n
  * @param  BKPx  BKP Instance
  * @param  Tamper This parameter can be a combination of the following values:
  *         @arg @ref LL_RTC_TAMPER_ACTIVELEVEL_LOW
  *         @arg @ref LL_RTC_TAMPER_ACTIVELEVEL_HIGH
  * @retval None
  */
__STATIC_INLINE void LL_RTC_TAMPER_SetActiveLevel(BKP_TypeDef *BKPx, uint32_t Tamper)
{
  MODIFY_REG(BKPx->CR, BKP_CR_TPAL, Tamper);
}

/**
  * @brief  Disable Active level for Tamper input
  * @rmtoll CR    TPAL        LL_RTC_TAMPER_SetActiveLevel\n
  * @retval None
  */
__STATIC_INLINE uint32_t LL_RTC_TAMPER_GetActiveLevel(BKP_TypeDef *BKPx)
{
  return (uint32_t)(READ_BIT(BKPx->CR, BKP_CR_TPAL));
}

/**
  * @}
  */

/** @defgroup RTC_LL_EF_Backup_Registers Backup_Registers
  * @{
  */

/**
  * @brief  Writes a data in a specified RTC Backup data register.
  * @rmtoll BKPDR        DR           LL_RTC_BKP_SetRegister
  * @param  BKPx  BKP Instance
  * @param  BackupRegister This parameter can be one of the following values:
  *         @arg @ref LL_RTC_BKP_DR1
  *         @arg @ref LL_RTC_BKP_DR2
  *         @arg @ref LL_RTC_BKP_DR3
  *         @arg @ref LL_RTC_BKP_DR4
  *         @arg @ref LL_RTC_BKP_DR5 
  *         @arg @ref LL_RTC_BKP_DR6 
  *         @arg @ref LL_RTC_BKP_DR7 
  *         @arg @ref LL_RTC_BKP_DR8 
  *         @arg @ref LL_RTC_BKP_DR9 
  *         @arg @ref LL_RTC_BKP_DR10 
  *         @arg @ref LL_RTC_BKP_DR11 (*)
  *         @arg @ref LL_RTC_BKP_DR12 (*)
  *         @arg @ref LL_RTC_BKP_DR13 (*)
  *         @arg @ref LL_RTC_BKP_DR14 (*)
  *         @arg @ref LL_RTC_BKP_DR15 (*)
  *         @arg @ref LL_RTC_BKP_DR16 (*)
  *         @arg @ref LL_RTC_BKP_DR17 (*)
  *         @arg @ref LL_RTC_BKP_DR18 (*)
  *         @arg @ref LL_RTC_BKP_DR19 (*)
  *         @arg @ref LL_RTC_BKP_DR20 (*)
  *         @arg @ref LL_RTC_BKP_DR21 (*)
  *         @arg @ref LL_RTC_BKP_DR22 (*)
  *         @arg @ref LL_RTC_BKP_DR23 (*)
  *         @arg @ref LL_RTC_BKP_DR24 (*)
  *         @arg @ref LL_RTC_BKP_DR25 (*)
  *         @arg @ref LL_RTC_BKP_DR26 (*)
  *         @arg @ref LL_RTC_BKP_DR27 (*)
  *         @arg @ref LL_RTC_BKP_DR28 (*)
  *         @arg @ref LL_RTC_BKP_DR29 (*)
  *         @arg @ref LL_RTC_BKP_DR30 (*)
  *         @arg @ref LL_RTC_BKP_DR31 (*)
  *         @arg @ref LL_RTC_BKP_DR32 (*)
  *         @arg @ref LL_RTC_BKP_DR33 (*)
  *         @arg @ref LL_RTC_BKP_DR34 (*)
  *         @arg @ref LL_RTC_BKP_DR35 (*)
  *         @arg @ref LL_RTC_BKP_DR36 (*)
  *         @arg @ref LL_RTC_BKP_DR37 (*)
  *         @arg @ref LL_RTC_BKP_DR38 (*)
  *         @arg @ref LL_RTC_BKP_DR39 (*)
  *         @arg @ref LL_RTC_BKP_DR40 (*)
  *         @arg @ref LL_RTC_BKP_DR41 (*)
  *         @arg @ref LL_RTC_BKP_DR42 (*)
  *         (*) value not defined in all devices.
  * @param  Data Value between Min_Data=0x00 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void LL_RTC_BKP_SetRegister(BKP_TypeDef *BKPx, uint32_t BackupRegister, uint32_t Data)
{
  register uint32_t tmp = 0U;

  tmp = (uint32_t)BKP_BASE;
  tmp += (BackupRegister * 4U);

  /* Write the specified register */
  *(__IO uint32_t *)tmp = (uint32_t)Data;
}

/**
  * @brief  Reads data from the specified RTC Backup data Register.
  * @rmtoll BKPDR        DR           LL_RTC_BKP_GetRegister
  * @param  BKPx BKP Instance
  * @param  BackupRegister This parameter can be one of the following values:
  *         @arg @ref LL_RTC_BKP_DR1
  *         @arg @ref LL_RTC_BKP_DR2
  *         @arg @ref LL_RTC_BKP_DR3
  *         @arg @ref LL_RTC_BKP_DR4
  *         @arg @ref LL_RTC_BKP_DR5 
  *         @arg @ref LL_RTC_BKP_DR6 
  *         @arg @ref LL_RTC_BKP_DR7 
  *         @arg @ref LL_RTC_BKP_DR8 
  *         @arg @ref LL_RTC_BKP_DR9 
  *         @arg @ref LL_RTC_BKP_DR10 
  *         @arg @ref LL_RTC_BKP_DR11 (*)
  *         @arg @ref LL_RTC_BKP_DR12 (*)
  *         @arg @ref LL_RTC_BKP_DR13 (*)
  *         @arg @ref LL_RTC_BKP_DR14 (*)
  *         @arg @ref LL_RTC_BKP_DR15 (*)
  *         @arg @ref LL_RTC_BKP_DR16 (*)
  *         @arg @ref LL_RTC_BKP_DR17 (*)
  *         @arg @ref LL_RTC_BKP_DR18 (*)
  *         @arg @ref LL_RTC_BKP_DR19 (*)
  *         @arg @ref LL_RTC_BKP_DR20 (*)
  *         @arg @ref LL_RTC_BKP_DR21 (*)
  *         @arg @ref LL_RTC_BKP_DR22 (*)
  *         @arg @ref LL_RTC_BKP_DR23 (*)
  *         @arg @ref LL_RTC_BKP_DR24 (*)
  *         @arg @ref LL_RTC_BKP_DR25 (*)
  *         @arg @ref LL_RTC_BKP_DR26 (*)
  *         @arg @ref LL_RTC_BKP_DR27 (*)
  *         @arg @ref LL_RTC_BKP_DR28 (*)
  *         @arg @ref LL_RTC_BKP_DR29 (*)
  *         @arg @ref LL_RTC_BKP_DR30 (*)
  *         @arg @ref LL_RTC_BKP_DR31 (*)
  *         @arg @ref LL_RTC_BKP_DR32 (*)
  *         @arg @ref LL_RTC_BKP_DR33 (*)
  *         @arg @ref LL_RTC_BKP_DR34 (*)
  *         @arg @ref LL_RTC_BKP_DR35 (*)
  *         @arg @ref LL_RTC_BKP_DR36 (*)
  *         @arg @ref LL_RTC_BKP_DR37 (*)
  *         @arg @ref LL_RTC_BKP_DR38 (*)
  *         @arg @ref LL_RTC_BKP_DR39 (*)
  *         @arg @ref LL_RTC_BKP_DR40 (*)
  *         @arg @ref LL_RTC_BKP_DR41 (*)
  *         @arg @ref LL_RTC_BKP_DR42 (*)
  * @retval Value between Min_Data=0x00 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t LL_RTC_BKP_GetRegister(BKP_TypeDef *BKPx, uint32_t BackupRegister)
{
  register uint32_t tmp = 0U;

  tmp = (uint32_t)BKP_BASE;
  tmp += (BackupRegister * 4U);

  /* Read the specified register */
  return ((*(__IO uint32_t *)tmp) & BKP_DR1_D);
}

/**
  * @}
  */

/** @defgroup RTC_LL_EF_Calibration Calibration
  * @{
  */

/**
  * @brief  Set the coarse digital calibration
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function should be called before.
  * @note   It can be written in initialization mode only (@ref LL_RTC_EnterInitMode function)
  * @rmtoll RTCCR       CAL           LL_RTC_CAL_SetCoarseDigital\n
  * @param  BKPx RTC Instance
  * @param  Value value of coarse calibration expressed in ppm (coded on 5 bits)
  * @note   This Calibration value should be between 0 and 121 when using positive sign with a 4-ppm step.
  * @retval None
  */
__STATIC_INLINE void LL_RTC_CAL_SetCoarseDigital(BKP_TypeDef* BKPx, uint32_t Value)
{
  MODIFY_REG(BKPx->RTCCR,BKP_RTCCR_CAL, Value);
}

/**
  * @brief  Get the coarse digital calibration value
  * @rmtoll RTCCR       CAL           LL_RTC_CAL_SetCoarseDigital\n
  * @param  BKPx BKP Instance
  * @retval value of coarse calibration expressed in ppm (coded on 5 bits)
  */
__STATIC_INLINE uint32_t LL_RTC_CAL_GetCoarseDigital(BKP_TypeDef *BKPx)
{
  return (uint32_t)(READ_BIT(BKPx->RTCCR, BKP_RTCCR_CAL));
}
/**
  * @}
  */

/** @defgroup RTC_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Get RTC_TAMPI  Interruption detection flag
  * @rmtoll CSR          TIF        LL_RTC_IsActiveFlag_TAMPI
  * @param  BKPx BKP Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_TAMPI(BKP_TypeDef *BKPx)
{
  return (READ_BIT(BKPx->CSR, BKP_CSR_TIF) == (BKP_CSR_TIF));
}

/**
  * @brief  Clear RTC_TAMP Interruption detection flag
  * @rmtoll CSR          CTI         LL_RTC_ClearFlag_TAMPI
  * @param  BKPx BKP Instance
  * @retval None
  */
__STATIC_INLINE void LL_RTC_ClearFlag_TAMPI(BKP_TypeDef *BKPx)
{
  SET_BIT(BKPx->CSR, BKP_CSR_CTI);
}

/**
  * @brief  Get RTC_TAMPE  Event detection flag
  * @rmtoll CSR          TEF        LL_RTC_IsActiveFlag_TAMPE
  * @param  BKPx BKP Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_TAMPE(BKP_TypeDef *BKPx)
{
  return (READ_BIT(BKPx->CSR, BKP_CSR_TEF) == (BKP_CSR_TEF));
}

/**
  * @brief  Clear RTC_TAMPE Even detection flag
  * @rmtoll CSR          CTE         LL_RTC_ClearFlag_TAMPE
  * @param  BKPx BKP Instance
  * @retval None
  */
__STATIC_INLINE void LL_RTC_ClearFlag_TAMPE(BKP_TypeDef *BKPx)
{
  SET_BIT(BKPx->CSR, BKP_CSR_CTE);
}

/**
  * @brief  Get Alarm  flag
  * @rmtoll CRL          ALRF         LL_RTC_IsActiveFlag_ALR
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_ALR(RTC_TypeDef *RTCx)
{
  return (READ_BIT(RTCx->CRL, RTC_CRL_ALRF) == (RTC_CRL_ALRF));
}

/**
  * @brief  Clear Alarm flag
  * @rmtoll CRL          ALRF         LL_RTC_ClearFlag_ALR
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void LL_RTC_ClearFlag_ALR(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CRL, RTC_CRL_ALRF);
}

/**
  * @brief  Get Registers synchronization flag
  * @rmtoll CRL          RSF           LL_RTC_IsActiveFlag_RS
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_RS(RTC_TypeDef *RTCx)
{
  return (READ_BIT(RTCx->CRL, RTC_CRL_RSF) == (RTC_CRL_RSF));
}

/**
  * @brief  Clear Registers synchronization flag
  * @rmtoll CRL          RSF           LL_RTC_ClearFlag_RS
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void LL_RTC_ClearFlag_RS(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CRL, RTC_CRL_RSF);
}

/**
  * @brief  Get Registers OverFlow flag
  * @rmtoll CRL          OWF           LL_RTC_IsActiveFlag_OW
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_OW(RTC_TypeDef *RTCx)
{
  return (READ_BIT(RTCx->CRL, RTC_CRL_OWF) == (RTC_CRL_OWF));
}

/**
  * @brief  Clear Registers OverFlow flag
  * @rmtoll CRL          OWF           LL_RTC_ClearFlag_OW
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void LL_RTC_ClearFlag_OW(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CRL, RTC_CRL_OWF);
}

/**
  * @brief  Get Registers synchronization flag
  * @rmtoll CRL          SECF           LL_RTC_IsActiveFlag_SEC
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_SEC(RTC_TypeDef *RTCx)
{
  return (READ_BIT(RTCx->CRL, RTC_CRL_SECF) == (RTC_CRL_SECF));
}

/**
  * @brief  Clear Registers synchronization flag
  * @rmtoll CRL          SECF           LL_RTC_ClearFlag_SEC
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void LL_RTC_ClearFlag_SEC(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CRL, RTC_CRL_SECF);
}

/**
  * @brief  Get RTC Operation OFF status flag
  * @rmtoll CRL          RTOFF         LL_RTC_IsActiveFlag_RTOF
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RTC_IsActiveFlag_RTOF(RTC_TypeDef *RTCx)
{
  return (READ_BIT(RTCx->CRL, RTC_CRL_RTOFF) == (RTC_CRL_RTOFF));
}

/**
  * @}
  */

/** @defgroup RTC_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable Alarm  interrupt
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function should be called before.
  * @rmtoll CRH           ALRIE        LL_RTC_EnableIT_ALR
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void LL_RTC_EnableIT_ALR(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CRH, RTC_CRH_ALRIE);
}

/**
  * @brief  Disable Alarm  interrupt
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function should be called before.
  * @rmtoll CRH           ALRIE        LL_RTC_DisableIT_ALR
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void LL_RTC_DisableIT_ALR(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CRH, RTC_CRH_ALRIE);
}

/**
  * @brief  Check if  Alarm  interrupt is enabled or not
  * @rmtoll CRH           ALRIE        LL_RTC_IsEnabledIT_ALR
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RTC_IsEnabledIT_ALR(RTC_TypeDef *RTCx)
{
  return (READ_BIT(RTCx->CRH, RTC_CRH_ALRIE) == (RTC_CRH_ALRIE));
}

/**
  * @brief  Enable Second Interrupt interrupt
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function should be called before.
  * @rmtoll CRH           SECIE        LL_RTC_EnableIT_SEC
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void LL_RTC_EnableIT_SEC(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CRH, RTC_CRH_SECIE);
}

/**
  * @brief  Disable Second interrupt
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function should be called before.
  * @rmtoll CRH           SECIE        LL_RTC_DisableIT_SEC
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void LL_RTC_DisableIT_SEC(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CRH, RTC_CRH_SECIE);
}

/**
  * @brief  Check if  Second interrupt is enabled or not
  * @rmtoll CRH           SECIE        LL_RTC_IsEnabledIT_SEC
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RTC_IsEnabledIT_SEC(RTC_TypeDef *RTCx)
{
  return (READ_BIT(RTCx->CRH, RTC_CRH_SECIE) == (RTC_CRH_SECIE));
}

/**
  * @brief  Enable OverFlow interrupt
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function should be called before.
  * @rmtoll CRH           OWIE        LL_RTC_EnableIT_OW
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void LL_RTC_EnableIT_OW(RTC_TypeDef *RTCx)
{
  SET_BIT(RTCx->CRH, RTC_CRH_OWIE);
}

/**
  * @brief  Disable OverFlow interrupt
  * @note   Bit is write-protected. @ref LL_RTC_DisableWriteProtection function should be called before.
  * @rmtoll CRH           OWIE        LL_RTC_DisableIT_OW
  * @param  RTCx RTC Instance
  * @retval None
  */
__STATIC_INLINE void LL_RTC_DisableIT_OW(RTC_TypeDef *RTCx)
{
  CLEAR_BIT(RTCx->CRH, RTC_CRH_OWIE);
}

/**
  * @brief  Check if  OverFlow interrupt is enabled or not
  * @rmtoll CRH            OWIE       LL_RTC_IsEnabledIT_OW
  * @param  RTCx RTC Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RTC_IsEnabledIT_OW(RTC_TypeDef *RTCx)
{
  return (READ_BIT(RTCx->CRH, RTC_CRH_OWIE) == (RTC_CRH_OWIE));
}

/**
  * @brief  Enable Tamper  interrupt
  * @rmtoll CSR        TPIE       LL_RTC_EnableIT_TAMP
  * @param  BKPx BKP Instance
  * @retval None
  */
__STATIC_INLINE void LL_RTC_EnableIT_TAMP(BKP_TypeDef *BKPx)
{
  SET_BIT(BKPx->CSR,BKP_CSR_TPIE);
}

/**
  * @brief  Disable Tamper  interrupt
  * @rmtoll CSR        TPIE       LL_RTC_EnableIT_TAMP
  * @param  BKPx BKP Instance
  * @retval None
  */
__STATIC_INLINE void LL_RTC_DisableIT_TAMP(BKP_TypeDef *BKPx)
{
  CLEAR_BIT(BKPx->CSR,BKP_CSR_TPIE);
}

/**
  * @brief  Check if all the TAMPER interrupts are enabled or not
  * @rmtoll CSR        TPIE        LL_RTC_IsEnabledIT_TAMP
  * @param  BKPx BKP Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RTC_IsEnabledIT_TAMP(BKP_TypeDef *BKPx)
{
  return (READ_BIT(BKPx->CSR,BKP_CSR_TPIE) == BKP_CSR_TPIE);
}
/**
  * @}
  */

#if defined(USE_FULL_LL_DRIVER)
/** @defgroup RTC_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

ErrorStatus LL_RTC_DeInit(RTC_TypeDef *RTCx);
ErrorStatus LL_RTC_Init(RTC_TypeDef *RTCx, LL_RTC_InitTypeDef *RTC_InitStruct);
void        LL_RTC_StructInit(LL_RTC_InitTypeDef *RTC_InitStruct);
ErrorStatus LL_RTC_TIME_Init(RTC_TypeDef *RTCx, uint32_t RTC_Format, LL_RTC_TimeTypeDef *RTC_TimeStruct);
void        LL_RTC_TIME_StructInit(LL_RTC_TimeTypeDef *RTC_TimeStruct);
ErrorStatus LL_RTC_ALARM_Init(RTC_TypeDef *RTCx, uint32_t RTC_Format, LL_RTC_AlarmTypeDef *RTC_AlarmStruct);
void        LL_RTC_ALARM_StructInit(LL_RTC_AlarmTypeDef *RTC_AlarmStruct);
ErrorStatus LL_RTC_EnterInitMode(RTC_TypeDef *RTCx);
ErrorStatus LL_RTC_ExitInitMode(RTC_TypeDef *RTCx);
ErrorStatus LL_RTC_WaitForSynchro(RTC_TypeDef *RTCx);
ErrorStatus LL_RTC_TIME_SetCounter(RTC_TypeDef *RTCx, uint32_t TimeCounter);
ErrorStatus LL_RTC_ALARM_SetCounter(RTC_TypeDef *RTCx, uint32_t AlarmCounter);

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

/**
  * @}
  */

#endif /* defined(RTC) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_LL_RTC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
