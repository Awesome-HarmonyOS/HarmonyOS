/**
  ******************************************************************************
  * @file    stm32f1xx_ll_system.h
  * @author  MCD Application Team
  * @brief   Header file of SYSTEM LL module.
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    [..]
    The LL SYSTEM driver contains a set of generic APIs that can be
    used by user:
      (+) Some of the FLASH features need to be handled in the SYSTEM file.
      (+) Access to DBGCMU registers
      (+) Access to SYSCFG registers

  @endverbatim
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
#ifndef __STM32F1xx_LL_SYSTEM_H
#define __STM32F1xx_LL_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"

/** @addtogroup STM32F1xx_LL_Driver
  * @{
  */

#if defined (FLASH) || defined (DBGMCU)

/** @defgroup SYSTEM_LL SYSTEM
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
/** @defgroup SYSTEM_LL_Private_Constants SYSTEM Private Constants
  * @{
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup SYSTEM_LL_Exported_Constants SYSTEM Exported Constants
  * @{
  */



/** @defgroup SYSTEM_LL_EC_TRACE DBGMCU TRACE Pin Assignment
  * @{
  */
#define LL_DBGMCU_TRACE_NONE               0x00000000U                                     /*!< TRACE pins not assigned (default state) */
#define LL_DBGMCU_TRACE_ASYNCH             DBGMCU_CR_TRACE_IOEN                            /*!< TRACE pin assignment for Asynchronous Mode */
#define LL_DBGMCU_TRACE_SYNCH_SIZE1        (DBGMCU_CR_TRACE_IOEN | DBGMCU_CR_TRACE_MODE_0) /*!< TRACE pin assignment for Synchronous Mode with a TRACEDATA size of 1 */
#define LL_DBGMCU_TRACE_SYNCH_SIZE2        (DBGMCU_CR_TRACE_IOEN | DBGMCU_CR_TRACE_MODE_1) /*!< TRACE pin assignment for Synchronous Mode with a TRACEDATA size of 2 */
#define LL_DBGMCU_TRACE_SYNCH_SIZE4        (DBGMCU_CR_TRACE_IOEN | DBGMCU_CR_TRACE_MODE)   /*!< TRACE pin assignment for Synchronous Mode with a TRACEDATA size of 4 */
/**
  * @}
  */

/** @defgroup SYSTEM_LL_EC_APB1_GRP1_STOP_IP DBGMCU APB1 GRP1 STOP IP
  * @{
  */
#define LL_DBGMCU_APB1_GRP1_TIM2_STOP      DBGMCU_CR_DBG_TIM2_STOP          /*!< TIM2 counter stopped when core is halted */
#define LL_DBGMCU_APB1_GRP1_TIM3_STOP      DBGMCU_CR_DBG_TIM3_STOP          /*!< TIM3 counter stopped when core is halted */
#define LL_DBGMCU_APB1_GRP1_TIM4_STOP      DBGMCU_CR_DBG_TIM4_STOP          /*!< TIM4 counter stopped when core is halted */
#if defined(DBGMCU_CR_DBG_TIM5_STOP)
#define LL_DBGMCU_APB1_GRP1_TIM5_STOP      DBGMCU_CR_DBG_TIM5_STOP          /*!< TIM5 counter stopped when core is halted */
#endif /* DBGMCU_CR_DBG_TIM5_STOP */
#if defined(DBGMCU_CR_DBG_TIM6_STOP)
#define LL_DBGMCU_APB1_GRP1_TIM6_STOP      DBGMCU_CR_DBG_TIM6_STOP          /*!< TIM6 counter stopped when core is halted */
#endif /* DBGMCU_CR_DBG_TIM6_STOP */
#if defined(DBGMCU_CR_DBG_TIM7_STOP)
#define LL_DBGMCU_APB1_GRP1_TIM7_STOP      DBGMCU_CR_DBG_TIM7_STOP          /*!< TIM7 counter stopped when core is halted */
#endif /* DBGMCU_CR_DBG_TIM7_STOP */
#if defined(DBGMCU_CR_DBG_TIM12_STOP)
#define LL_DBGMCU_APB1_GRP1_TIM12_STOP     DBGMCU_CR_DBG_TIM12_STOP         /*!< TIM12 counter stopped when core is halted */
#endif /* DBGMCU_CR_DBG_TIM12_STOP */
#if defined(DBGMCU_CR_DBG_TIM13_STOP)
#define LL_DBGMCU_APB1_GRP1_TIM13_STOP     DBGMCU_CR_DBG_TIM13_STOP         /*!< TIM13 counter stopped when core is halted */
#endif /* DBGMCU_CR_DBG_TIM13_STOP */
#if defined(DBGMCU_CR_DBG_TIM14_STOP)
#define LL_DBGMCU_APB1_GRP1_TIM14_STOP     DBGMCU_CR_DBG_TIM14_STOP         /*!< TIM14 counter stopped when core is halted */
#endif /* DBGMCU_CR_DBG_TIM14_STOP */
#define LL_DBGMCU_APB1_GRP1_WWDG_STOP      DBGMCU_CR_DBG_WWDG_STOP          /*!< Debug Window Watchdog stopped when Core is halted */
#define LL_DBGMCU_APB1_GRP1_IWDG_STOP      DBGMCU_CR_DBG_IWDG_STOP          /*!< Debug Independent Watchdog stopped when Core is halted */
#define LL_DBGMCU_APB1_GRP1_I2C1_STOP      DBGMCU_CR_DBG_I2C1_SMBUS_TIMEOUT /*!< I2C1 SMBUS timeout mode stopped when Core is halted */
#if defined(DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT)
#define LL_DBGMCU_APB1_GRP1_I2C2_STOP      DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT /*!< I2C2 SMBUS timeout mode stopped when Core is halted */
#endif /* DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT */
#if defined(DBGMCU_CR_DBG_CAN1_STOP)
#define LL_DBGMCU_APB1_GRP1_CAN1_STOP      DBGMCU_CR_DBG_CAN1_STOP          /*!< CAN1 debug stopped when Core is halted  */
#endif /* DBGMCU_CR_DBG_CAN1_STOP */
#if defined(DBGMCU_CR_DBG_CAN2_STOP)
#define LL_DBGMCU_APB1_GRP1_CAN2_STOP      DBGMCU_CR_DBG_CAN2_STOP          /*!< CAN2 debug stopped when Core is halted  */
#endif /* DBGMCU_CR_DBG_CAN2_STOP */
/**
  * @}
  */

/** @defgroup SYSTEM_LL_EC_APB2_GRP1_STOP_IP DBGMCU APB2 GRP1 STOP IP
  * @{
  */
#define LL_DBGMCU_APB2_GRP1_TIM1_STOP      DBGMCU_CR_DBG_TIM1_STOP   /*!< TIM1 counter stopped when core is halted */
#if defined(DBGMCU_CR_DBG_TIM8_STOP)
#define LL_DBGMCU_APB2_GRP1_TIM8_STOP      DBGMCU_CR_DBG_TIM8_STOP   /*!< TIM8 counter stopped when core is halted */
#endif /* DBGMCU_CR_DBG_CAN1_STOP */
#if defined(DBGMCU_CR_DBG_TIM9_STOP)
#define LL_DBGMCU_APB2_GRP1_TIM9_STOP      DBGMCU_CR_DBG_TIM9_STOP   /*!< TIM9 counter stopped when core is halted */
#endif /* DBGMCU_CR_DBG_TIM9_STOP */
#if defined(DBGMCU_CR_DBG_TIM10_STOP)
#define LL_DBGMCU_APB2_GRP1_TIM10_STOP     DBGMCU_CR_DBG_TIM10_STOP   /*!< TIM10 counter stopped when core is halted */
#endif /* DBGMCU_CR_DBG_TIM10_STOP */
#if defined(DBGMCU_CR_DBG_TIM11_STOP)
#define LL_DBGMCU_APB2_GRP1_TIM11_STOP     DBGMCU_CR_DBG_TIM11_STOP   /*!< TIM11 counter stopped when core is halted */
#endif /* DBGMCU_CR_DBG_TIM11_STOP */
#if defined(DBGMCU_CR_DBG_TIM15_STOP)
#define LL_DBGMCU_APB2_GRP1_TIM15_STOP     DBGMCU_CR_DBG_TIM15_STOP   /*!< TIM15 counter stopped when core is halted */
#endif /* DBGMCU_CR_DBG_TIM15_STOP */
#if defined(DBGMCU_CR_DBG_TIM16_STOP)
#define LL_DBGMCU_APB2_GRP1_TIM16_STOP     DBGMCU_CR_DBG_TIM16_STOP   /*!< TIM16 counter stopped when core is halted */
#endif /* DBGMCU_CR_DBG_TIM16_STOP */
#if defined(DBGMCU_CR_DBG_TIM17_STOP)
#define LL_DBGMCU_APB2_GRP1_TIM17_STOP     DBGMCU_CR_DBG_TIM17_STOP   /*!< TIM17 counter stopped when core is halted */
#endif /* DBGMCU_CR_DBG_TIM17_STOP */
/**
  * @}
  */

/** @defgroup SYSTEM_LL_EC_LATENCY FLASH LATENCY
  * @{
  */
#if defined(FLASH_ACR_LATENCY)
#define LL_FLASH_LATENCY_0                 0x00000000U             /*!< FLASH Zero Latency cycle */
#define LL_FLASH_LATENCY_1                 FLASH_ACR_LATENCY_0     /*!< FLASH One Latency cycle */
#define LL_FLASH_LATENCY_2                 FLASH_ACR_LATENCY_1     /*!< FLASH Two wait states */
#else
#endif /* FLASH_ACR_LATENCY */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @defgroup SYSTEM_LL_Exported_Functions SYSTEM Exported Functions
  * @{
  */



/** @defgroup SYSTEM_LL_EF_DBGMCU DBGMCU
  * @{
  */

/**
  * @brief  Return the device identifier
  * @note For Low Density devices, the device ID is 0x412
  * @note For Medium Density devices, the device ID is 0x410
  * @note For High Density devices, the device ID is 0x414
  * @note For XL Density devices, the device ID is 0x430
  * @note For Connectivity Line devices, the device ID is 0x418
  * @rmtoll DBGMCU_IDCODE DEV_ID        LL_DBGMCU_GetDeviceID
  * @retval Values between Min_Data=0x00 and Max_Data=0xFFF
  */
__STATIC_INLINE uint32_t LL_DBGMCU_GetDeviceID(void)
{
  return (uint32_t)(READ_BIT(DBGMCU->IDCODE, DBGMCU_IDCODE_DEV_ID));
}

/**
  * @brief  Return the device revision identifier
  * @note This field indicates the revision of the device.
          For example, it is read as revA -> 0x1000,for Low Density devices
          For example, it is read as revA -> 0x0000, revB -> 0x2000, revZ -> 0x2001, rev1,2,3,X or Y -> 0x2003,for Medium Density devices
          For example, it is read as revA or 1 -> 0x1000, revZ -> 0x1001,rev1,2,3,X or Y -> 0x1003,for Medium Density devices
          For example, it is read as revA or 1 -> 0x1003,for XL Density devices
          For example, it is read as revA -> 0x1000, revZ -> 0x1001 for  Connectivity line devices
  * @rmtoll DBGMCU_IDCODE REV_ID        LL_DBGMCU_GetRevisionID
  * @retval Values between Min_Data=0x00 and Max_Data=0xFFFF
  */
__STATIC_INLINE uint32_t LL_DBGMCU_GetRevisionID(void)
{
  return (uint32_t)(READ_BIT(DBGMCU->IDCODE, DBGMCU_IDCODE_REV_ID) >> DBGMCU_IDCODE_REV_ID_Pos);
}

/**
  * @brief  Enable the Debug Module during SLEEP mode
  * @rmtoll DBGMCU_CR    DBG_SLEEP     LL_DBGMCU_EnableDBGSleepMode
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_EnableDBGSleepMode(void)
{
  SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP);
}

/**
  * @brief  Disable the Debug Module during SLEEP mode
  * @rmtoll DBGMCU_CR    DBG_SLEEP     LL_DBGMCU_DisableDBGSleepMode
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_DisableDBGSleepMode(void)
{
  CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP);
}

/**
  * @brief  Enable the Debug Module during STOP mode
  * @rmtoll DBGMCU_CR    DBG_STOP      LL_DBGMCU_EnableDBGStopMode
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_EnableDBGStopMode(void)
{
  SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STOP);
}

/**
  * @brief  Disable the Debug Module during STOP mode
  * @rmtoll DBGMCU_CR    DBG_STOP      LL_DBGMCU_DisableDBGStopMode
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_DisableDBGStopMode(void)
{
  CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STOP);
}

/**
  * @brief  Enable the Debug Module during STANDBY mode
  * @rmtoll DBGMCU_CR    DBG_STANDBY   LL_DBGMCU_EnableDBGStandbyMode
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_EnableDBGStandbyMode(void)
{
  SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STANDBY);
}

/**
  * @brief  Disable the Debug Module during STANDBY mode
  * @rmtoll DBGMCU_CR    DBG_STANDBY   LL_DBGMCU_DisableDBGStandbyMode
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_DisableDBGStandbyMode(void)
{
  CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STANDBY);
}

/**
  * @brief  Set Trace pin assignment control
  * @rmtoll DBGMCU_CR    TRACE_IOEN    LL_DBGMCU_SetTracePinAssignment\n
  *         DBGMCU_CR    TRACE_MODE    LL_DBGMCU_SetTracePinAssignment
  * @param  PinAssignment This parameter can be one of the following values:
  *         @arg @ref LL_DBGMCU_TRACE_NONE
  *         @arg @ref LL_DBGMCU_TRACE_ASYNCH
  *         @arg @ref LL_DBGMCU_TRACE_SYNCH_SIZE1
  *         @arg @ref LL_DBGMCU_TRACE_SYNCH_SIZE2
  *         @arg @ref LL_DBGMCU_TRACE_SYNCH_SIZE4
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_SetTracePinAssignment(uint32_t PinAssignment)
{
  MODIFY_REG(DBGMCU->CR, DBGMCU_CR_TRACE_IOEN | DBGMCU_CR_TRACE_MODE, PinAssignment);
}

/**
  * @brief  Get Trace pin assignment control
  * @rmtoll DBGMCU_CR    TRACE_IOEN    LL_DBGMCU_GetTracePinAssignment\n
  *         DBGMCU_CR    TRACE_MODE    LL_DBGMCU_GetTracePinAssignment
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DBGMCU_TRACE_NONE
  *         @arg @ref LL_DBGMCU_TRACE_ASYNCH
  *         @arg @ref LL_DBGMCU_TRACE_SYNCH_SIZE1
  *         @arg @ref LL_DBGMCU_TRACE_SYNCH_SIZE2
  *         @arg @ref LL_DBGMCU_TRACE_SYNCH_SIZE4
  */
__STATIC_INLINE uint32_t LL_DBGMCU_GetTracePinAssignment(void)
{
  return (uint32_t)(READ_BIT(DBGMCU->CR, DBGMCU_CR_TRACE_IOEN | DBGMCU_CR_TRACE_MODE));
}

/**
  * @brief  Freeze APB1 peripherals (group1 peripherals)
  * @rmtoll DBGMCU_CR_APB1      DBG_TIM2_STOP           LL_DBGMCU_APB1_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_TIM3_STOP           LL_DBGMCU_APB1_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_TIM4_STOP           LL_DBGMCU_APB1_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_TIM5_STOP           LL_DBGMCU_APB1_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_TIM6_STOP           LL_DBGMCU_APB1_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_TIM7_STOP           LL_DBGMCU_APB1_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_TIM12_STOP          LL_DBGMCU_APB1_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_TIM13_STOP          LL_DBGMCU_APB1_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_TIM14_STOP          LL_DBGMCU_APB1_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_RTC_STOP            LL_DBGMCU_APB1_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_WWDG_STOP           LL_DBGMCU_APB1_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_IWDG_STOP           LL_DBGMCU_APB1_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_I2C1_SMBUS_TIMEOUT  LL_DBGMCU_APB1_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_I2C2_SMBUS_TIMEOUT  LL_DBGMCU_APB1_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_CAN1_STOP           LL_DBGMCU_APB1_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_CAN2_STOP           LL_DBGMCU_APB1_GRP1_FreezePeriph
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM2_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM3_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM4_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM5_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM6_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM7_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM12_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM13_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM14_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_WWDG_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_IWDG_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_I2C1_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_I2C2_STOP (*)
  *         @arg @ref LL_DBGMCU_APB1_GRP1_CAN1_STOP (*)
  *         @arg @ref LL_DBGMCU_APB1_GRP1_CAN2_STOP (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_APB1_GRP1_FreezePeriph(uint32_t Periphs)
{
  SET_BIT(DBGMCU->CR, Periphs);
}

/**
  * @brief  Unfreeze APB1 peripherals (group1 peripherals)
  * @rmtoll DBGMCU_CR_APB1      DBG_TIM2_STOP           LL_DBGMCU_APB1_GRP1_UnFreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_TIM3_STOP           LL_DBGMCU_APB1_GRP1_UnFreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_TIM4_STOP           LL_DBGMCU_APB1_GRP1_UnFreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_TIM5_STOP           LL_DBGMCU_APB1_GRP1_UnFreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_TIM6_STOP           LL_DBGMCU_APB1_GRP1_UnFreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_TIM7_STOP           LL_DBGMCU_APB1_GRP1_UnFreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_TIM12_STOP          LL_DBGMCU_APB1_GRP1_UnFreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_TIM13_STOP          LL_DBGMCU_APB1_GRP1_UnFreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_TIM14_STOP          LL_DBGMCU_APB1_GRP1_UnFreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_RTC_STOP            LL_DBGMCU_APB1_GRP1_UnFreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_WWDG_STOP           LL_DBGMCU_APB1_GRP1_UnFreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_IWDG_STOP           LL_DBGMCU_APB1_GRP1_UnFreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_I2C1_SMBUS_TIMEOUT  LL_DBGMCU_APB1_GRP1_UnFreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_I2C2_SMBUS_TIMEOUT  LL_DBGMCU_APB1_GRP1_UnFreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_CAN1_STOP           LL_DBGMCU_APB1_GRP1_UnFreezePeriph\n
  *         DBGMCU_CR_APB1      DBG_CAN2_STOP           LL_DBGMCU_APB1_GRP1_UnFreezePeriph
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM2_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM3_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM4_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM5_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM6_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM7_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM12_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM13_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_TIM14_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_RTC_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_WWDG_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_IWDG_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_I2C1_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_I2C2_STOP (*)
  *         @arg @ref LL_DBGMCU_APB1_GRP1_CAN1_STOP (*)
  *         @arg @ref LL_DBGMCU_APB1_GRP1_CAN2_STOP (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_APB1_GRP1_UnFreezePeriph(uint32_t Periphs)
{
  CLEAR_BIT(DBGMCU->CR, Periphs);
}

/**
  * @brief  Freeze APB2 peripherals
  * @rmtoll DBGMCU_CR_APB2      DBG_TIM1_STOP    LL_DBGMCU_APB2_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB2      DBG_TIM8_STOP    LL_DBGMCU_APB2_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB2      DBG_TIM9_STOP    LL_DBGMCU_APB2_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB2      DBG_TIM10_STOP   LL_DBGMCU_APB2_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB2      DBG_TIM11_STOP   LL_DBGMCU_APB2_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB2      DBG_TIM15_STOP   LL_DBGMCU_APB2_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB2      DBG_TIM16_STOP   LL_DBGMCU_APB2_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB2      DBG_TIM17_STOP   LL_DBGMCU_APB2_GRP1_FreezePeriph
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_DBGMCU_APB2_GRP1_TIM1_STOP
  *         @arg @ref LL_DBGMCU_APB2_GRP1_TIM8_STOP (*)
  *         @arg @ref LL_DBGMCU_APB2_GRP1_TIM9_STOP (*)
  *         @arg @ref LL_DBGMCU_APB2_GRP1_TIM10_STOP (*)
  *         @arg @ref LL_DBGMCU_APB2_GRP1_TIM11_STOP (*)
  *         @arg @ref LL_DBGMCU_APB2_GRP1_TIM15_STOP (*)
  *         @arg @ref LL_DBGMCU_APB2_GRP1_TIM16_STOP (*)
  *         @arg @ref LL_DBGMCU_APB2_GRP1_TIM17_STOP (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_APB2_GRP1_FreezePeriph(uint32_t Periphs)
{
  SET_BIT(DBGMCU->CR, Periphs);
}

/**
  * @brief  Unfreeze APB2 peripherals
  * @rmtoll DBGMCU_CR_APB2      DBG_TIM1_STOP    LL_DBGMCU_APB2_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB2      DBG_TIM8_STOP    LL_DBGMCU_APB2_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB2      DBG_TIM9_STOP    LL_DBGMCU_APB2_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB2      DBG_TIM10_STOP   LL_DBGMCU_APB2_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB2      DBG_TIM11_STOP   LL_DBGMCU_APB2_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB2      DBG_TIM15_STOP   LL_DBGMCU_APB2_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB2      DBG_TIM16_STOP   LL_DBGMCU_APB2_GRP1_FreezePeriph\n
  *         DBGMCU_CR_APB2      DBG_TIM17_STOP   LL_DBGMCU_APB2_GRP1_FreezePeriph
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_DBGMCU_APB2_GRP1_TIM1_STOP
  *         @arg @ref LL_DBGMCU_APB2_GRP1_TIM8_STOP (*)
  *         @arg @ref LL_DBGMCU_APB2_GRP1_TIM9_STOP (*)
  *         @arg @ref LL_DBGMCU_APB2_GRP1_TIM10_STOP (*)
  *         @arg @ref LL_DBGMCU_APB2_GRP1_TIM11_STOP (*)
  *         @arg @ref LL_DBGMCU_APB2_GRP1_TIM15_STOP (*)
  *         @arg @ref LL_DBGMCU_APB2_GRP1_TIM16_STOP (*)
  *         @arg @ref LL_DBGMCU_APB2_GRP1_TIM17_STOP (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_APB2_GRP1_UnFreezePeriph(uint32_t Periphs)
{
  CLEAR_BIT(DBGMCU->CR, Periphs);
}
/**
  * @}
  */

#if defined(FLASH_ACR_LATENCY)
/** @defgroup SYSTEM_LL_EF_FLASH FLASH
  * @{
  */

/**
  * @brief  Set FLASH Latency
  * @rmtoll FLASH_ACR    LATENCY       LL_FLASH_SetLatency
  * @param  Latency This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_LATENCY_0
  *         @arg @ref LL_FLASH_LATENCY_1
  *         @arg @ref LL_FLASH_LATENCY_2
  * @retval None
  */
__STATIC_INLINE void LL_FLASH_SetLatency(uint32_t Latency)
{
  MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, Latency);
}

/**
  * @brief  Get FLASH Latency
  * @rmtoll FLASH_ACR    LATENCY       LL_FLASH_GetLatency
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_LATENCY_0
  *         @arg @ref LL_FLASH_LATENCY_1
  *         @arg @ref LL_FLASH_LATENCY_2
  */
__STATIC_INLINE uint32_t LL_FLASH_GetLatency(void)
{
  return (uint32_t)(READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY));
}

/**
  * @brief  Enable Prefetch
  * @rmtoll FLASH_ACR    PRFTBE        LL_FLASH_EnablePrefetch
  * @retval None
  */
__STATIC_INLINE void LL_FLASH_EnablePrefetch(void)
{
  SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
}

/**
  * @brief  Disable Prefetch
  * @rmtoll FLASH_ACR    PRFTBE        LL_FLASH_DisablePrefetch
  * @retval None
  */
__STATIC_INLINE void LL_FLASH_DisablePrefetch(void)
{
  CLEAR_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
}

/**
  * @brief  Check if Prefetch buffer is enabled
  * @rmtoll FLASH_ACR    PRFTBS        LL_FLASH_IsPrefetchEnabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsPrefetchEnabled(void)
{
  return (READ_BIT(FLASH->ACR, FLASH_ACR_PRFTBS) == (FLASH_ACR_PRFTBS));
}

#endif /* FLASH_ACR_LATENCY */
/**
  * @brief  Enable Flash Half Cycle Access
  * @rmtoll FLASH_ACR    HLFCYA        LL_FLASH_EnableHalfCycleAccess
  * @retval None
  */
__STATIC_INLINE void LL_FLASH_EnableHalfCycleAccess(void)
{
  SET_BIT(FLASH->ACR, FLASH_ACR_HLFCYA);
}

/**
  * @brief  Disable Flash Half Cycle Access
  * @rmtoll FLASH_ACR    HLFCYA        LL_FLASH_DisableHalfCycleAccess
  * @retval None
  */
__STATIC_INLINE void LL_FLASH_DisableHalfCycleAccess(void)
{
  CLEAR_BIT(FLASH->ACR, FLASH_ACR_HLFCYA);
}

/**
  * @brief  Check if  Flash Half Cycle Access is enabled or not
  * @rmtoll FLASH_ACR    HLFCYA        LL_FLASH_IsHalfCycleAccessEnabled
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_FLASH_IsHalfCycleAccessEnabled(void)
{
  return (READ_BIT(FLASH->ACR, FLASH_ACR_HLFCYA) == (FLASH_ACR_HLFCYA));
}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined (FLASH) || defined (DBGMCU) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_LL_SYSTEM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
