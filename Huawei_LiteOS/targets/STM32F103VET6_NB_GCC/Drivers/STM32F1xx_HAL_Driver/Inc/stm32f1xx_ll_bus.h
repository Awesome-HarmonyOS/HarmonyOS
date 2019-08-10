/**
  ******************************************************************************
  * @file    stm32f1xx_ll_bus.h
  * @author  MCD Application Team
  * @brief   Header file of BUS LL module.

  @verbatim                
                      ##### RCC Limitations #####
  ==============================================================================
    [..]  
      A delay between an RCC peripheral clock enable and the effective peripheral 
      enabling should be taken into account in order to manage the peripheral read/write 
      from/to registers.
      (+) This delay depends on the peripheral mapping.
        (++) AHB & APB peripherals, 1 dummy read is necessary

    [..]  
      Workarounds:
      (#) For AHB & APB peripherals, a dummy read to the peripheral register has been
          inserted in each LL_{BUS}_GRP{x}_EnableClock() function.

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
#ifndef __STM32F1xx_LL_BUS_H
#define __STM32F1xx_LL_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"

/** @addtogroup STM32F1xx_LL_Driver
  * @{
  */

#if defined(RCC)

/** @defgroup BUS_LL BUS
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
#if defined(RCC_AHBRSTR_OTGFSRST) || defined(RCC_AHBRSTR_ETHMACRST)
#define RCC_AHBRSTR_SUPPORT
#endif /* RCC_AHBRSTR_OTGFSRST || RCC_AHBRSTR_ETHMACRST */

/* Private macros ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup BUS_LL_Exported_Constants BUS Exported Constants
  * @{
  */

/** @defgroup BUS_LL_EC_AHB1_GRP1_PERIPH  AHB1 GRP1 PERIPH
  * @{
  */
#define LL_AHB1_GRP1_PERIPH_ALL            (uint32_t)0xFFFFFFFFU
#define LL_AHB1_GRP1_PERIPH_CRC            RCC_AHBENR_CRCEN
#define LL_AHB1_GRP1_PERIPH_DMA1           RCC_AHBENR_DMA1EN
#if defined(DMA2)
#define LL_AHB1_GRP1_PERIPH_DMA2           RCC_AHBENR_DMA2EN
#endif /*DMA2*/
#if defined(ETH)
#define LL_AHB1_GRP1_PERIPH_ETHMAC         RCC_AHBENR_ETHMACEN
#define LL_AHB1_GRP1_PERIPH_ETHMACRX       RCC_AHBENR_ETHMACRXEN
#define LL_AHB1_GRP1_PERIPH_ETHMACTX       RCC_AHBENR_ETHMACTXEN
#endif /*ETH*/
#define LL_AHB1_GRP1_PERIPH_FLASH          RCC_AHBENR_FLITFEN
#if defined(FSMC_Bank1)
#define LL_AHB1_GRP1_PERIPH_FSMC           RCC_AHBENR_FSMCEN
#endif /*FSMC_Bank1*/
#if defined(USB_OTG_FS)
#define LL_AHB1_GRP1_PERIPH_OTGFS          RCC_AHBENR_OTGFSEN
#endif /*USB_OTG_FS*/
#if defined(SDIO)
#define LL_AHB1_GRP1_PERIPH_SDIO           RCC_AHBENR_SDIOEN
#endif /*SDIO*/
#define LL_AHB1_GRP1_PERIPH_SRAM           RCC_AHBENR_SRAMEN
/**
  * @}
  */

/** @defgroup BUS_LL_EC_APB1_GRP1_PERIPH  APB1 GRP1 PERIPH
  * @{
  */
#define LL_APB1_GRP1_PERIPH_ALL            (uint32_t)0xFFFFFFFFU
#define LL_APB1_GRP1_PERIPH_BKP            RCC_APB1ENR_BKPEN
#if defined(CAN1)
#define LL_APB1_GRP1_PERIPH_CAN1           RCC_APB1ENR_CAN1EN
#endif /*CAN1*/
#if defined(CAN2)
#define LL_APB1_GRP1_PERIPH_CAN2           RCC_APB1ENR_CAN2EN
#endif /*CAN2*/
#if defined(CEC)
#define LL_APB1_GRP1_PERIPH_CEC            RCC_APB1ENR_CECEN
#endif /*CEC*/
#if defined(DAC)
#define LL_APB1_GRP1_PERIPH_DAC1           RCC_APB1ENR_DACEN
#endif /*DAC*/
#define LL_APB1_GRP1_PERIPH_I2C1           RCC_APB1ENR_I2C1EN
#if defined(I2C2)
#define LL_APB1_GRP1_PERIPH_I2C2           RCC_APB1ENR_I2C2EN
#endif /*I2C2*/
#define LL_APB1_GRP1_PERIPH_PWR            RCC_APB1ENR_PWREN
#if defined(SPI2)
#define LL_APB1_GRP1_PERIPH_SPI2           RCC_APB1ENR_SPI2EN
#endif /*SPI2*/
#if defined(SPI3)
#define LL_APB1_GRP1_PERIPH_SPI3           RCC_APB1ENR_SPI3EN
#endif /*SPI3*/
#if defined(TIM12)
#define LL_APB1_GRP1_PERIPH_TIM12          RCC_APB1ENR_TIM12EN
#endif /*TIM12*/
#if defined(TIM13)
#define LL_APB1_GRP1_PERIPH_TIM13          RCC_APB1ENR_TIM13EN
#endif /*TIM13*/
#if defined(TIM14)
#define LL_APB1_GRP1_PERIPH_TIM14          RCC_APB1ENR_TIM14EN
#endif /*TIM14*/
#define LL_APB1_GRP1_PERIPH_TIM2           RCC_APB1ENR_TIM2EN
#define LL_APB1_GRP1_PERIPH_TIM3           RCC_APB1ENR_TIM3EN
#if defined(TIM4)
#define LL_APB1_GRP1_PERIPH_TIM4           RCC_APB1ENR_TIM4EN
#endif /*TIM4*/
#if defined(TIM5)
#define LL_APB1_GRP1_PERIPH_TIM5           RCC_APB1ENR_TIM5EN
#endif /*TIM5*/
#if defined(TIM6)
#define LL_APB1_GRP1_PERIPH_TIM6           RCC_APB1ENR_TIM6EN
#endif /*TIM6*/
#if defined(TIM7)
#define LL_APB1_GRP1_PERIPH_TIM7           RCC_APB1ENR_TIM7EN
#endif /*TIM7*/
#if defined(UART4)
#define LL_APB1_GRP1_PERIPH_UART4          RCC_APB1ENR_UART4EN
#endif /*UART4*/
#if defined(UART5)
#define LL_APB1_GRP1_PERIPH_UART5          RCC_APB1ENR_UART5EN
#endif /*UART5*/
#define LL_APB1_GRP1_PERIPH_USART2         RCC_APB1ENR_USART2EN
#if defined(USART3)
#define LL_APB1_GRP1_PERIPH_USART3         RCC_APB1ENR_USART3EN
#endif /*USART3*/
#if defined(USB)
#define LL_APB1_GRP1_PERIPH_USB            RCC_APB1ENR_USBEN
#endif /*USB*/
#define LL_APB1_GRP1_PERIPH_WWDG           RCC_APB1ENR_WWDGEN
/**
  * @}
  */

/** @defgroup BUS_LL_EC_APB2_GRP1_PERIPH  APB2 GRP1 PERIPH
  * @{
  */
#define LL_APB2_GRP1_PERIPH_ALL            (uint32_t)0xFFFFFFFFU
#define LL_APB2_GRP1_PERIPH_ADC1           RCC_APB2ENR_ADC1EN
#if defined(ADC2)
#define LL_APB2_GRP1_PERIPH_ADC2           RCC_APB2ENR_ADC2EN
#endif /*ADC2*/
#if defined(ADC3)
#define LL_APB2_GRP1_PERIPH_ADC3           RCC_APB2ENR_ADC3EN
#endif /*ADC3*/
#define LL_APB2_GRP1_PERIPH_AFIO           RCC_APB2ENR_AFIOEN
#define LL_APB2_GRP1_PERIPH_GPIOA          RCC_APB2ENR_IOPAEN
#define LL_APB2_GRP1_PERIPH_GPIOB          RCC_APB2ENR_IOPBEN
#define LL_APB2_GRP1_PERIPH_GPIOC          RCC_APB2ENR_IOPCEN
#define LL_APB2_GRP1_PERIPH_GPIOD          RCC_APB2ENR_IOPDEN
#if defined(GPIOE)
#define LL_APB2_GRP1_PERIPH_GPIOE          RCC_APB2ENR_IOPEEN
#endif /*GPIOE*/
#if defined(GPIOF)
#define LL_APB2_GRP1_PERIPH_GPIOF          RCC_APB2ENR_IOPFEN
#endif /*GPIOF*/
#if defined(GPIOG)
#define LL_APB2_GRP1_PERIPH_GPIOG          RCC_APB2ENR_IOPGEN
#endif /*GPIOG*/
#define LL_APB2_GRP1_PERIPH_SPI1           RCC_APB2ENR_SPI1EN
#if defined(TIM10)
#define LL_APB2_GRP1_PERIPH_TIM10          RCC_APB2ENR_TIM10EN
#endif /*TIM10*/
#if defined(TIM11)
#define LL_APB2_GRP1_PERIPH_TIM11          RCC_APB2ENR_TIM11EN
#endif /*TIM11*/
#if defined(TIM15)
#define LL_APB2_GRP1_PERIPH_TIM15          RCC_APB2ENR_TIM15EN
#endif /*TIM15*/
#if defined(TIM16)
#define LL_APB2_GRP1_PERIPH_TIM16          RCC_APB2ENR_TIM16EN
#endif /*TIM16*/
#if defined(TIM17)
#define LL_APB2_GRP1_PERIPH_TIM17          RCC_APB2ENR_TIM17EN
#endif /*TIM17*/
#define LL_APB2_GRP1_PERIPH_TIM1           RCC_APB2ENR_TIM1EN
#if defined(TIM8)
#define LL_APB2_GRP1_PERIPH_TIM8           RCC_APB2ENR_TIM8EN
#endif /*TIM8*/
#if defined(TIM9)
#define LL_APB2_GRP1_PERIPH_TIM9           RCC_APB2ENR_TIM9EN
#endif /*TIM9*/
#define LL_APB2_GRP1_PERIPH_USART1         RCC_APB2ENR_USART1EN
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @defgroup BUS_LL_Exported_Functions BUS Exported Functions
  * @{
  */

/** @defgroup BUS_LL_EF_AHB1 AHB1
  * @{
  */

/**
  * @brief  Enable AHB1 peripherals clock.
  * @rmtoll AHBENR       CRCEN         LL_AHB1_GRP1_EnableClock\n
  *         AHBENR       DMA1EN        LL_AHB1_GRP1_EnableClock\n
  *         AHBENR       DMA2EN        LL_AHB1_GRP1_EnableClock\n
  *         AHBENR       ETHMACEN      LL_AHB1_GRP1_EnableClock\n
  *         AHBENR       ETHMACRXEN    LL_AHB1_GRP1_EnableClock\n
  *         AHBENR       ETHMACTXEN    LL_AHB1_GRP1_EnableClock\n
  *         AHBENR       FLITFEN       LL_AHB1_GRP1_EnableClock\n
  *         AHBENR       FSMCEN        LL_AHB1_GRP1_EnableClock\n
  *         AHBENR       OTGFSEN       LL_AHB1_GRP1_EnableClock\n
  *         AHBENR       SDIOEN        LL_AHB1_GRP1_EnableClock\n
  *         AHBENR       SRAMEN        LL_AHB1_GRP1_EnableClock
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA2 (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETHMAC (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETHMACRX (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETHMACTX (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FLASH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FSMC (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_OTGFS (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SDIO (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void LL_AHB1_GRP1_EnableClock(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCC->AHBENR, Periphs);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->AHBENR, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if AHB1 peripheral clock is enabled or not
  * @rmtoll AHBENR       CRCEN         LL_AHB1_GRP1_IsEnabledClock\n
  *         AHBENR       DMA1EN        LL_AHB1_GRP1_IsEnabledClock\n
  *         AHBENR       DMA2EN        LL_AHB1_GRP1_IsEnabledClock\n
  *         AHBENR       ETHMACEN      LL_AHB1_GRP1_IsEnabledClock\n
  *         AHBENR       ETHMACRXEN    LL_AHB1_GRP1_IsEnabledClock\n
  *         AHBENR       ETHMACTXEN    LL_AHB1_GRP1_IsEnabledClock\n
  *         AHBENR       FLITFEN       LL_AHB1_GRP1_IsEnabledClock\n
  *         AHBENR       FSMCEN        LL_AHB1_GRP1_IsEnabledClock\n
  *         AHBENR       OTGFSEN       LL_AHB1_GRP1_IsEnabledClock\n
  *         AHBENR       SDIOEN        LL_AHB1_GRP1_IsEnabledClock\n
  *         AHBENR       SRAMEN        LL_AHB1_GRP1_IsEnabledClock
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA2 (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETHMAC (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETHMACRX (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETHMACTX (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FLASH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FSMC (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_OTGFS (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SDIO (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM
  *
  *         (*) value not defined in all devices.
  * @retval State of Periphs (1 or 0).
*/
__STATIC_INLINE uint32_t LL_AHB1_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return (READ_BIT(RCC->AHBENR, Periphs) == Periphs);
}

/**
  * @brief  Disable AHB1 peripherals clock.
  * @rmtoll AHBENR       CRCEN         LL_AHB1_GRP1_DisableClock\n
  *         AHBENR       DMA1EN        LL_AHB1_GRP1_DisableClock\n
  *         AHBENR       DMA2EN        LL_AHB1_GRP1_DisableClock\n
  *         AHBENR       ETHMACEN      LL_AHB1_GRP1_DisableClock\n
  *         AHBENR       ETHMACRXEN    LL_AHB1_GRP1_DisableClock\n
  *         AHBENR       ETHMACTXEN    LL_AHB1_GRP1_DisableClock\n
  *         AHBENR       FLITFEN       LL_AHB1_GRP1_DisableClock\n
  *         AHBENR       FSMCEN        LL_AHB1_GRP1_DisableClock\n
  *         AHBENR       OTGFSEN       LL_AHB1_GRP1_DisableClock\n
  *         AHBENR       SDIOEN        LL_AHB1_GRP1_DisableClock\n
  *         AHBENR       SRAMEN        LL_AHB1_GRP1_DisableClock
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA2 (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETHMAC (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETHMACRX (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETHMACTX (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FLASH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_FSMC (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_OTGFS (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SDIO (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_SRAM
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void LL_AHB1_GRP1_DisableClock(uint32_t Periphs)
{
  CLEAR_BIT(RCC->AHBENR, Periphs);
}

#if defined(RCC_AHBRSTR_SUPPORT)
/**
  * @brief  Force AHB1 peripherals reset.
  * @rmtoll AHBRSTR      ETHMACRST     LL_AHB1_GRP1_ForceReset\n
  *         AHBRSTR      OTGFSRST      LL_AHB1_GRP1_ForceReset
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETHMAC (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_OTGFS (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void LL_AHB1_GRP1_ForceReset(uint32_t Periphs)
{
  SET_BIT(RCC->AHBRSTR, Periphs);
}

/**
  * @brief  Release AHB1 peripherals reset.
  * @rmtoll AHBRSTR      ETHMACRST     LL_AHB1_GRP1_ReleaseReset\n
  *         AHBRSTR      OTGFSRST      LL_AHB1_GRP1_ReleaseReset
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETHMAC (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_OTGFS (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void LL_AHB1_GRP1_ReleaseReset(uint32_t Periphs)
{
  CLEAR_BIT(RCC->AHBRSTR, Periphs);
}
#endif /* RCC_AHBRSTR_SUPPORT */

/**
  * @}
  */

/** @defgroup BUS_LL_EF_APB1 APB1
  * @{
  */

/**
  * @brief  Enable APB1 peripherals clock.
  * @rmtoll APB1ENR      BKPEN         LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      CAN1EN        LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      CAN2EN        LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      CECEN         LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      DACEN         LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      I2C1EN        LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      I2C2EN        LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      PWREN         LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      SPI2EN        LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      SPI3EN        LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      TIM12EN       LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      TIM13EN       LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      TIM14EN       LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      TIM2EN        LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      TIM3EN        LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      TIM4EN        LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      TIM5EN        LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      TIM6EN        LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      TIM7EN        LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      UART4EN       LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      UART5EN       LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      USART2EN      LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      USART3EN      LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      USBEN         LL_APB1_GRP1_EnableClock\n
  *         APB1ENR      WWDGEN        LL_APB1_GRP1_EnableClock
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_BKP
  *         @arg @ref LL_APB1_GRP1_PERIPH_CAN1 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_CAN2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_CEC (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_DAC1 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_PWR
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI3 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM12 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM13 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM14 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM2
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM3
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM4 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM5 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM6 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM7 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART4 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART5 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART2
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART3 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_USB (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_WWDG
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void LL_APB1_GRP1_EnableClock(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCC->APB1ENR, Periphs);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->APB1ENR, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if APB1 peripheral clock is enabled or not
  * @rmtoll APB1ENR      BKPEN         LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      CAN1EN        LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      CAN2EN        LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      CECEN         LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      DACEN         LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      I2C1EN        LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      I2C2EN        LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      PWREN         LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      SPI2EN        LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      SPI3EN        LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      TIM12EN       LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      TIM13EN       LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      TIM14EN       LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      TIM2EN        LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      TIM3EN        LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      TIM4EN        LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      TIM5EN        LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      TIM6EN        LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      TIM7EN        LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      UART4EN       LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      UART5EN       LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      USART2EN      LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      USART3EN      LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      USBEN         LL_APB1_GRP1_IsEnabledClock\n
  *         APB1ENR      WWDGEN        LL_APB1_GRP1_IsEnabledClock
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_BKP
  *         @arg @ref LL_APB1_GRP1_PERIPH_CAN1 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_CAN2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_CEC (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_DAC1 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_PWR
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI3 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM12 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM13 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM14 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM2
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM3
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM4 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM5 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM6 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM7 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART4 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART5 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART2
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART3 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_USB (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_WWDG
  *
  *         (*) value not defined in all devices.
  * @retval State of Periphs (1 or 0).
*/
__STATIC_INLINE uint32_t LL_APB1_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return (READ_BIT(RCC->APB1ENR, Periphs) == Periphs);
}

/**
  * @brief  Disable APB1 peripherals clock.
  * @rmtoll APB1ENR      BKPEN         LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      CAN1EN        LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      CAN2EN        LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      CECEN         LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      DACEN         LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      I2C1EN        LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      I2C2EN        LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      PWREN         LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      SPI2EN        LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      SPI3EN        LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      TIM12EN       LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      TIM13EN       LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      TIM14EN       LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      TIM2EN        LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      TIM3EN        LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      TIM4EN        LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      TIM5EN        LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      TIM6EN        LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      TIM7EN        LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      UART4EN       LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      UART5EN       LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      USART2EN      LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      USART3EN      LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      USBEN         LL_APB1_GRP1_DisableClock\n
  *         APB1ENR      WWDGEN        LL_APB1_GRP1_DisableClock
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_BKP
  *         @arg @ref LL_APB1_GRP1_PERIPH_CAN1 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_CAN2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_CEC (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_DAC1 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_PWR
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI3 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM12 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM13 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM14 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM2
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM3
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM4 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM5 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM6 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM7 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART4 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART5 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART2
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART3 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_USB (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_WWDG
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void LL_APB1_GRP1_DisableClock(uint32_t Periphs)
{
  CLEAR_BIT(RCC->APB1ENR, Periphs);
}

/**
  * @brief  Force APB1 peripherals reset.
  * @rmtoll APB1RSTR     BKPRST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     CAN1RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     CAN2RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     CECRST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     DACRST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     I2C1RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     I2C2RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     PWRRST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     SPI2RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     SPI3RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM12RST      LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM13RST      LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM14RST      LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM2RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM3RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM4RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM5RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM6RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM7RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     UART4RST      LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     UART5RST      LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     USART2RST     LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     USART3RST     LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     USBRST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     WWDGRST       LL_APB1_GRP1_ForceReset
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP1_PERIPH_BKP
  *         @arg @ref LL_APB1_GRP1_PERIPH_CAN1 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_CAN2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_CEC (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_DAC1 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_PWR
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI3 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM12 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM13 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM14 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM2
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM3
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM4 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM5 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM6 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM7 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART4 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART5 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART2
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART3 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_USB (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_WWDG
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void LL_APB1_GRP1_ForceReset(uint32_t Periphs)
{
  SET_BIT(RCC->APB1RSTR, Periphs);
}

/**
  * @brief  Release APB1 peripherals reset.
  * @rmtoll APB1RSTR     BKPRST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     CAN1RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     CAN2RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     CECRST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     DACRST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     I2C1RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     I2C2RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     PWRRST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     SPI2RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     SPI3RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM12RST      LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM13RST      LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM14RST      LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM2RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM3RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM4RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM5RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM6RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM7RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     UART4RST      LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     UART5RST      LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     USART2RST     LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     USART3RST     LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     USBRST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     WWDGRST       LL_APB1_GRP1_ReleaseReset
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB1_GRP1_PERIPH_BKP
  *         @arg @ref LL_APB1_GRP1_PERIPH_CAN1 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_CAN2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_CEC (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_DAC1 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_PWR
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI3 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM12 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM13 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM14 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM2
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM3
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM4 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM5 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM6 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM7 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART4 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART5 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART2
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART3 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_USB (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_WWDG
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void LL_APB1_GRP1_ReleaseReset(uint32_t Periphs)
{
  CLEAR_BIT(RCC->APB1RSTR, Periphs);
}

/**
  * @}
  */

/** @defgroup BUS_LL_EF_APB2 APB2
  * @{
  */

/**
  * @brief  Enable APB2 peripherals clock.
  * @rmtoll APB2ENR      ADC1EN        LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      ADC2EN        LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      ADC3EN        LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      AFIOEN        LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      IOPAEN        LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      IOPBEN        LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      IOPCEN        LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      IOPDEN        LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      IOPEEN        LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      IOPFEN        LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      IOPGEN        LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      SPI1EN        LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      TIM10EN       LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      TIM11EN       LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      TIM15EN       LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      TIM16EN       LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      TIM17EN       LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      TIM1EN        LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      TIM8EN        LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      TIM9EN        LL_APB2_GRP1_EnableClock\n
  *         APB2ENR      USART1EN      LL_APB2_GRP1_EnableClock
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC1
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC2 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC3 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_AFIO
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOE (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOF (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOG (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM10 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM11 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM15 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM16 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM17 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM8 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM9 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART1
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void LL_APB2_GRP1_EnableClock(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCC->APB2ENR, Periphs);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->APB2ENR, Periphs);
  (void)tmpreg;
}

/**
  * @brief  Check if APB2 peripheral clock is enabled or not
  * @rmtoll APB2ENR      ADC1EN        LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      ADC2EN        LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      ADC3EN        LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      AFIOEN        LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      IOPAEN        LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      IOPBEN        LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      IOPCEN        LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      IOPDEN        LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      IOPEEN        LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      IOPFEN        LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      IOPGEN        LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      SPI1EN        LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      TIM10EN       LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      TIM11EN       LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      TIM15EN       LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      TIM16EN       LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      TIM17EN       LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      TIM1EN        LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      TIM8EN        LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      TIM9EN        LL_APB2_GRP1_IsEnabledClock\n
  *         APB2ENR      USART1EN      LL_APB2_GRP1_IsEnabledClock
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC1
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC2 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC3 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_AFIO
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOE (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOF (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOG (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM10 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM11 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM15 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM16 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM17 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM8 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM9 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART1
  *
  *         (*) value not defined in all devices.
  * @retval State of Periphs (1 or 0).
*/
__STATIC_INLINE uint32_t LL_APB2_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return (READ_BIT(RCC->APB2ENR, Periphs) == Periphs);
}

/**
  * @brief  Disable APB2 peripherals clock.
  * @rmtoll APB2ENR      ADC1EN        LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      ADC2EN        LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      ADC3EN        LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      AFIOEN        LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      IOPAEN        LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      IOPBEN        LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      IOPCEN        LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      IOPDEN        LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      IOPEEN        LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      IOPFEN        LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      IOPGEN        LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      SPI1EN        LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      TIM10EN       LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      TIM11EN       LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      TIM15EN       LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      TIM16EN       LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      TIM17EN       LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      TIM1EN        LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      TIM8EN        LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      TIM9EN        LL_APB2_GRP1_DisableClock\n
  *         APB2ENR      USART1EN      LL_APB2_GRP1_DisableClock
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC1
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC2 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC3 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_AFIO
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOE (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOF (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOG (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM10 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM11 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM15 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM16 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM17 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM8 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM9 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART1
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void LL_APB2_GRP1_DisableClock(uint32_t Periphs)
{
  CLEAR_BIT(RCC->APB2ENR, Periphs);
}

/**
  * @brief  Force APB2 peripherals reset.
  * @rmtoll APB2RSTR     ADC1RST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     ADC2RST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     ADC3RST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     AFIORST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     IOPARST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     IOPBRST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     IOPCRST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     IOPDRST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     IOPERST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     IOPFRST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     IOPGRST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     SPI1RST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     TIM10RST      LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     TIM11RST      LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     TIM15RST      LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     TIM16RST      LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     TIM17RST      LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     TIM1RST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     TIM8RST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     TIM9RST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     USART1RST     LL_APB2_GRP1_ForceReset
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB2_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC1
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC2 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC3 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_AFIO
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOE (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOF (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOG (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM10 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM11 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM15 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM16 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM17 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM8 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM9 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART1
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void LL_APB2_GRP1_ForceReset(uint32_t Periphs)
{
  SET_BIT(RCC->APB2RSTR, Periphs);
}

/**
  * @brief  Release APB2 peripherals reset.
  * @rmtoll APB2RSTR     ADC1RST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     ADC2RST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     ADC3RST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     AFIORST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     IOPARST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     IOPBRST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     IOPCRST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     IOPDRST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     IOPERST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     IOPFRST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     IOPGRST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     SPI1RST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     TIM10RST      LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     TIM11RST      LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     TIM15RST      LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     TIM16RST      LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     TIM17RST      LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     TIM1RST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     TIM8RST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     TIM9RST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     USART1RST     LL_APB2_GRP1_ReleaseReset
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB2_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC1
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC2 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC3 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_AFIO
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOE (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOF (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_GPIOG (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM10 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM11 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM15 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM16 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM17 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM8 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM9 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART1
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
__STATIC_INLINE void LL_APB2_GRP1_ReleaseReset(uint32_t Periphs)
{
  CLEAR_BIT(RCC->APB2RSTR, Periphs);
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

#endif /* defined(RCC) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_LL_BUS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
