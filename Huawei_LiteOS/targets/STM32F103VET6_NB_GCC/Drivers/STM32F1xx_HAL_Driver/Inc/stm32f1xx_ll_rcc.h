/**
  ******************************************************************************
  * @file    stm32f1xx_ll_rcc.h
  * @author  MCD Application Team
  * @brief   Header file of RCC LL module.
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
#ifndef __STM32F1xx_LL_RCC_H
#define __STM32F1xx_LL_RCC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"

/** @addtogroup STM32F1xx_LL_Driver
  * @{
  */

#if defined(RCC)

/** @defgroup RCC_LL RCC
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
#if defined(USE_FULL_LL_DRIVER)
/** @defgroup RCC_LL_Private_Macros RCC Private Macros
  * @{
  */
/**
  * @}
  */
#endif /*USE_FULL_LL_DRIVER*/
/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_LL_DRIVER)
/** @defgroup RCC_LL_Exported_Types RCC Exported Types
  * @{
  */

/** @defgroup LL_ES_CLOCK_FREQ Clocks Frequency Structure
  * @{
  */

/**
  * @brief  RCC Clocks Frequency Structure
  */
typedef struct
{
  uint32_t SYSCLK_Frequency;        /*!< SYSCLK clock frequency */
  uint32_t HCLK_Frequency;          /*!< HCLK clock frequency */
  uint32_t PCLK1_Frequency;         /*!< PCLK1 clock frequency */
  uint32_t PCLK2_Frequency;         /*!< PCLK2 clock frequency */
} LL_RCC_ClocksTypeDef;

/**
  * @}
  */

/**
  * @}
  */
#endif /* USE_FULL_LL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RCC_LL_Exported_Constants RCC Exported Constants
  * @{
  */

/** @defgroup RCC_LL_EC_OSC_VALUES Oscillator Values adaptation
  * @brief    Defines used to adapt values of different oscillators
  * @note     These values could be modified in the user environment according to
  *           HW set-up.
  * @{
  */
#if !defined  (HSE_VALUE)
#define HSE_VALUE    8000000U  /*!< Value of the HSE oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
#define HSI_VALUE    8000000U  /*!< Value of the HSI oscillator in Hz */
#endif /* HSI_VALUE */

#if !defined  (LSE_VALUE)
#define LSE_VALUE    32768U    /*!< Value of the LSE oscillator in Hz */
#endif /* LSE_VALUE */

#if !defined  (LSI_VALUE)
#define LSI_VALUE    32000U    /*!< Value of the LSI oscillator in Hz */
#endif /* LSI_VALUE */
/**
  * @}
  */

/** @defgroup RCC_LL_EC_CLEAR_FLAG Clear Flags Defines
  * @brief    Flags defines which can be used with LL_RCC_WriteReg function
  * @{
  */
#define LL_RCC_CIR_LSIRDYC                RCC_CIR_LSIRDYC     /*!< LSI Ready Interrupt Clear */
#define LL_RCC_CIR_LSERDYC                RCC_CIR_LSERDYC     /*!< LSE Ready Interrupt Clear */
#define LL_RCC_CIR_HSIRDYC                RCC_CIR_HSIRDYC     /*!< HSI Ready Interrupt Clear */
#define LL_RCC_CIR_HSERDYC                RCC_CIR_HSERDYC     /*!< HSE Ready Interrupt Clear */
#define LL_RCC_CIR_PLLRDYC                RCC_CIR_PLLRDYC     /*!< PLL Ready Interrupt Clear */
#define LL_RCC_CIR_PLL3RDYC               RCC_CIR_PLL3RDYC    /*!< PLL3(PLLI2S) Ready Interrupt Clear */
#define LL_RCC_CIR_PLL2RDYC               RCC_CIR_PLL2RDYC    /*!< PLL2 Ready Interrupt Clear */
#define LL_RCC_CIR_CSSC                   RCC_CIR_CSSC        /*!< Clock Security System Interrupt Clear */
/**
  * @}
  */

/** @defgroup RCC_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_RCC_ReadReg function
  * @{
  */
#define LL_RCC_CIR_LSIRDYF                RCC_CIR_LSIRDYF     /*!< LSI Ready Interrupt flag */
#define LL_RCC_CIR_LSERDYF                RCC_CIR_LSERDYF     /*!< LSE Ready Interrupt flag */
#define LL_RCC_CIR_HSIRDYF                RCC_CIR_HSIRDYF     /*!< HSI Ready Interrupt flag */
#define LL_RCC_CIR_HSERDYF                RCC_CIR_HSERDYF     /*!< HSE Ready Interrupt flag */
#define LL_RCC_CIR_PLLRDYF                RCC_CIR_PLLRDYF     /*!< PLL Ready Interrupt flag */
#define LL_RCC_CIR_PLL3RDYF               RCC_CIR_PLL3RDYF    /*!< PLL3(PLLI2S) Ready Interrupt flag */
#define LL_RCC_CIR_PLL2RDYF               RCC_CIR_PLL2RDYF    /*!< PLL2 Ready Interrupt flag */
#define LL_RCC_CIR_CSSF                   RCC_CIR_CSSF        /*!< Clock Security System Interrupt flag */
#define LL_RCC_CSR_PINRSTF                RCC_CSR_PINRSTF     /*!< PIN reset flag */
#define LL_RCC_CSR_PORRSTF                RCC_CSR_PORRSTF     /*!< POR/PDR reset flag */
#define LL_RCC_CSR_SFTRSTF                RCC_CSR_SFTRSTF     /*!< Software Reset flag */
#define LL_RCC_CSR_IWDGRSTF               RCC_CSR_IWDGRSTF    /*!< Independent Watchdog reset flag */
#define LL_RCC_CSR_WWDGRSTF               RCC_CSR_WWDGRSTF    /*!< Window watchdog reset flag */
#define LL_RCC_CSR_LPWRRSTF               RCC_CSR_LPWRRSTF    /*!< Low-Power reset flag */
/**
  * @}
  */

/** @defgroup RCC_LL_EC_IT IT Defines
  * @brief    IT defines which can be used with LL_RCC_ReadReg and  LL_RCC_WriteReg functions
  * @{
  */
#define LL_RCC_CIR_LSIRDYIE               RCC_CIR_LSIRDYIE      /*!< LSI Ready Interrupt Enable */
#define LL_RCC_CIR_LSERDYIE               RCC_CIR_LSERDYIE      /*!< LSE Ready Interrupt Enable */
#define LL_RCC_CIR_HSIRDYIE               RCC_CIR_HSIRDYIE      /*!< HSI Ready Interrupt Enable */
#define LL_RCC_CIR_HSERDYIE               RCC_CIR_HSERDYIE      /*!< HSE Ready Interrupt Enable */
#define LL_RCC_CIR_PLLRDYIE               RCC_CIR_PLLRDYIE      /*!< PLL Ready Interrupt Enable */
#define LL_RCC_CIR_PLL3RDYIE              RCC_CIR_PLL3RDYIE     /*!< PLL3(PLLI2S) Ready Interrupt Enable */
#define LL_RCC_CIR_PLL2RDYIE              RCC_CIR_PLL2RDYIE     /*!< PLL2 Ready Interrupt Enable */
/**
  * @}
  */

#if defined(RCC_CFGR2_PREDIV2)
/** @defgroup RCC_LL_EC_HSE_PREDIV2_DIV HSE PREDIV2 Division factor
  * @{
  */
#define LL_RCC_HSE_PREDIV2_DIV_1           RCC_CFGR2_PREDIV2_DIV1   /*!< PREDIV2 input clock not divided */
#define LL_RCC_HSE_PREDIV2_DIV_2           RCC_CFGR2_PREDIV2_DIV2   /*!< PREDIV2 input clock divided by 2 */
#define LL_RCC_HSE_PREDIV2_DIV_3           RCC_CFGR2_PREDIV2_DIV3   /*!< PREDIV2 input clock divided by 3 */
#define LL_RCC_HSE_PREDIV2_DIV_4           RCC_CFGR2_PREDIV2_DIV4   /*!< PREDIV2 input clock divided by 4 */
#define LL_RCC_HSE_PREDIV2_DIV_5           RCC_CFGR2_PREDIV2_DIV5   /*!< PREDIV2 input clock divided by 5 */
#define LL_RCC_HSE_PREDIV2_DIV_6           RCC_CFGR2_PREDIV2_DIV6   /*!< PREDIV2 input clock divided by 6 */
#define LL_RCC_HSE_PREDIV2_DIV_7           RCC_CFGR2_PREDIV2_DIV7   /*!< PREDIV2 input clock divided by 7 */
#define LL_RCC_HSE_PREDIV2_DIV_8           RCC_CFGR2_PREDIV2_DIV8   /*!< PREDIV2 input clock divided by 8 */
#define LL_RCC_HSE_PREDIV2_DIV_9           RCC_CFGR2_PREDIV2_DIV9   /*!< PREDIV2 input clock divided by 9 */
#define LL_RCC_HSE_PREDIV2_DIV_10          RCC_CFGR2_PREDIV2_DIV10  /*!< PREDIV2 input clock divided by 10 */
#define LL_RCC_HSE_PREDIV2_DIV_11          RCC_CFGR2_PREDIV2_DIV11  /*!< PREDIV2 input clock divided by 11 */
#define LL_RCC_HSE_PREDIV2_DIV_12          RCC_CFGR2_PREDIV2_DIV12  /*!< PREDIV2 input clock divided by 12 */
#define LL_RCC_HSE_PREDIV2_DIV_13          RCC_CFGR2_PREDIV2_DIV13  /*!< PREDIV2 input clock divided by 13 */
#define LL_RCC_HSE_PREDIV2_DIV_14          RCC_CFGR2_PREDIV2_DIV14  /*!< PREDIV2 input clock divided by 14 */
#define LL_RCC_HSE_PREDIV2_DIV_15          RCC_CFGR2_PREDIV2_DIV15  /*!< PREDIV2 input clock divided by 15 */
#define LL_RCC_HSE_PREDIV2_DIV_16          RCC_CFGR2_PREDIV2_DIV16  /*!< PREDIV2 input clock divided by 16 */
/**
  * @}
  */

#endif /* RCC_CFGR2_PREDIV2 */

/** @defgroup RCC_LL_EC_SYS_CLKSOURCE  System clock switch
  * @{
  */
#define LL_RCC_SYS_CLKSOURCE_HSI           RCC_CFGR_SW_HSI    /*!< HSI selection as system clock */
#define LL_RCC_SYS_CLKSOURCE_HSE           RCC_CFGR_SW_HSE    /*!< HSE selection as system clock */
#define LL_RCC_SYS_CLKSOURCE_PLL           RCC_CFGR_SW_PLL    /*!< PLL selection as system clock */
/**
  * @}
  */

/** @defgroup RCC_LL_EC_SYS_CLKSOURCE_STATUS  System clock switch status
  * @{
  */
#define LL_RCC_SYS_CLKSOURCE_STATUS_HSI    RCC_CFGR_SWS_HSI   /*!< HSI used as system clock */
#define LL_RCC_SYS_CLKSOURCE_STATUS_HSE    RCC_CFGR_SWS_HSE   /*!< HSE used as system clock */
#define LL_RCC_SYS_CLKSOURCE_STATUS_PLL    RCC_CFGR_SWS_PLL   /*!< PLL used as system clock */
/**
  * @}
  */

/** @defgroup RCC_LL_EC_SYSCLK_DIV  AHB prescaler
  * @{
  */
#define LL_RCC_SYSCLK_DIV_1                RCC_CFGR_HPRE_DIV1   /*!< SYSCLK not divided */
#define LL_RCC_SYSCLK_DIV_2                RCC_CFGR_HPRE_DIV2   /*!< SYSCLK divided by 2 */
#define LL_RCC_SYSCLK_DIV_4                RCC_CFGR_HPRE_DIV4   /*!< SYSCLK divided by 4 */
#define LL_RCC_SYSCLK_DIV_8                RCC_CFGR_HPRE_DIV8   /*!< SYSCLK divided by 8 */
#define LL_RCC_SYSCLK_DIV_16               RCC_CFGR_HPRE_DIV16  /*!< SYSCLK divided by 16 */
#define LL_RCC_SYSCLK_DIV_64               RCC_CFGR_HPRE_DIV64  /*!< SYSCLK divided by 64 */
#define LL_RCC_SYSCLK_DIV_128              RCC_CFGR_HPRE_DIV128 /*!< SYSCLK divided by 128 */
#define LL_RCC_SYSCLK_DIV_256              RCC_CFGR_HPRE_DIV256 /*!< SYSCLK divided by 256 */
#define LL_RCC_SYSCLK_DIV_512              RCC_CFGR_HPRE_DIV512 /*!< SYSCLK divided by 512 */
/**
  * @}
  */

/** @defgroup RCC_LL_EC_APB1_DIV  APB low-speed prescaler (APB1)
  * @{
  */
#define LL_RCC_APB1_DIV_1                  RCC_CFGR_PPRE1_DIV1  /*!< HCLK not divided */
#define LL_RCC_APB1_DIV_2                  RCC_CFGR_PPRE1_DIV2  /*!< HCLK divided by 2 */
#define LL_RCC_APB1_DIV_4                  RCC_CFGR_PPRE1_DIV4  /*!< HCLK divided by 4 */
#define LL_RCC_APB1_DIV_8                  RCC_CFGR_PPRE1_DIV8  /*!< HCLK divided by 8 */
#define LL_RCC_APB1_DIV_16                 RCC_CFGR_PPRE1_DIV16 /*!< HCLK divided by 16 */
/**
  * @}
  */

/** @defgroup RCC_LL_EC_APB2_DIV  APB high-speed prescaler (APB2)
  * @{
  */
#define LL_RCC_APB2_DIV_1                  RCC_CFGR_PPRE2_DIV1  /*!< HCLK not divided */
#define LL_RCC_APB2_DIV_2                  RCC_CFGR_PPRE2_DIV2  /*!< HCLK divided by 2 */
#define LL_RCC_APB2_DIV_4                  RCC_CFGR_PPRE2_DIV4  /*!< HCLK divided by 4 */
#define LL_RCC_APB2_DIV_8                  RCC_CFGR_PPRE2_DIV8  /*!< HCLK divided by 8 */
#define LL_RCC_APB2_DIV_16                 RCC_CFGR_PPRE2_DIV16 /*!< HCLK divided by 16 */
/**
  * @}
  */

/** @defgroup RCC_LL_EC_MCO1SOURCE  MCO1 SOURCE selection
  * @{
  */
#define LL_RCC_MCO1SOURCE_NOCLOCK          RCC_CFGR_MCOSEL_NOCLOCK      /*!< MCO output disabled, no clock on MCO */
#define LL_RCC_MCO1SOURCE_SYSCLK           RCC_CFGR_MCOSEL_SYSCLK       /*!< SYSCLK selection as MCO source */
#define LL_RCC_MCO1SOURCE_HSI              RCC_CFGR_MCOSEL_HSI          /*!< HSI selection as MCO source */
#define LL_RCC_MCO1SOURCE_HSE              RCC_CFGR_MCOSEL_HSE          /*!< HSE selection as MCO source */
#define LL_RCC_MCO1SOURCE_PLLCLK_DIV_2     RCC_CFGR_MCOSEL_PLL_DIV2     /*!< PLL clock divided by 2*/
#if defined(RCC_CFGR_MCOSEL_PLL2CLK)
#define LL_RCC_MCO1SOURCE_PLL2CLK          RCC_CFGR_MCOSEL_PLL2         /*!< PLL2 clock selected as MCO source*/
#endif /* RCC_CFGR_MCOSEL_PLL2CLK */
#if defined(RCC_CFGR_MCOSEL_PLL3CLK_DIV2)
#define LL_RCC_MCO1SOURCE_PLLI2SCLK_DIV2   RCC_CFGR_MCOSEL_PLL3_DIV2    /*!< PLLI2S clock divided by 2 selected as MCO source*/
#endif /* RCC_CFGR_MCOSEL_PLL3CLK_DIV2 */
#if defined(RCC_CFGR_MCOSEL_EXT_HSE)
#define LL_RCC_MCO1SOURCE_EXT_HSE          RCC_CFGR_MCOSEL_EXT_HSE      /*!< XT1 external 3-25 MHz oscillator clock selected as MCO source */
#endif /* RCC_CFGR_MCOSEL_EXT_HSE */
#if defined(RCC_CFGR_MCOSEL_PLL3CLK)
#define LL_RCC_MCO1SOURCE_PLLI2SCLK        RCC_CFGR_MCOSEL_PLL3CLK      /*!< PLLI2S clock selected as MCO source */
#endif /* RCC_CFGR_MCOSEL_PLL3CLK */
/**
  * @}
  */

#if defined(USE_FULL_LL_DRIVER)
/** @defgroup RCC_LL_EC_PERIPH_FREQUENCY Peripheral clock frequency
  * @{
  */
#define LL_RCC_PERIPH_FREQUENCY_NO         0x00000000U      /*!< No clock enabled for the peripheral            */
#define LL_RCC_PERIPH_FREQUENCY_NA         0xFFFFFFFFU      /*!< Frequency cannot be provided as external clock */
/**
  * @}
  */
#endif /* USE_FULL_LL_DRIVER */

#if defined(RCC_CFGR2_I2S2SRC)
/** @defgroup RCC_LL_EC_I2S2CLKSOURCE Peripheral I2S clock source selection
  * @{
  */
#define LL_RCC_I2S2_CLKSOURCE_SYSCLK        RCC_CFGR2_I2S2SRC                                          /*!< System clock (SYSCLK) selected as I2S2 clock entry */
#define LL_RCC_I2S2_CLKSOURCE_PLLI2S_VCO    (uint32_t)(RCC_CFGR2_I2S2SRC | (RCC_CFGR2_I2S2SRC >> 16U)) /*!< PLLI2S VCO clock selected as I2S2 clock entry */
#define LL_RCC_I2S3_CLKSOURCE_SYSCLK        RCC_CFGR2_I2S3SRC                                          /*!< System clock (SYSCLK) selected as I2S3 clock entry */
#define LL_RCC_I2S3_CLKSOURCE_PLLI2S_VCO    (uint32_t)(RCC_CFGR2_I2S3SRC | (RCC_CFGR2_I2S3SRC >> 16U)) /*!< PLLI2S VCO clock selected as I2S3 clock entry */
/**
  * @}
  */
#endif /* RCC_CFGR2_I2S2SRC */

#if defined(USB_OTG_FS) || defined(USB)
/** @defgroup RCC_LL_EC_USB_CLKSOURCE Peripheral USB clock source selection
  * @{
  */
#if defined(RCC_CFGR_USBPRE)
#define LL_RCC_USB_CLKSOURCE_PLL             RCC_CFGR_USBPRE        /*!< PLL clock is not divided */
#define LL_RCC_USB_CLKSOURCE_PLL_DIV_1_5     0x00000000U            /*!< PLL clock is divided by 1.5 */
#endif /*RCC_CFGR_USBPRE*/                   
#if defined(RCC_CFGR_OTGFSPRE)               
#define LL_RCC_USB_CLKSOURCE_PLL_DIV_2       RCC_CFGR_OTGFSPRE      /*!< PLL clock is divided by 2 */
#define LL_RCC_USB_CLKSOURCE_PLL_DIV_3       0x00000000U            /*!< PLL clock is divided by 3 */
#endif /*RCC_CFGR_OTGFSPRE*/
/**
  * @}
  */
#endif /* USB_OTG_FS || USB */

/** @defgroup RCC_LL_EC_ADC_CLKSOURCE_PCLK2 Peripheral ADC clock source selection
  * @{
  */
#define LL_RCC_ADC_CLKSRC_PCLK2_DIV_2    RCC_CFGR_ADCPRE_DIV2 /*ADC prescaler PCLK2 divided by 2*/
#define LL_RCC_ADC_CLKSRC_PCLK2_DIV_4    RCC_CFGR_ADCPRE_DIV4 /*ADC prescaler PCLK2 divided by 4*/
#define LL_RCC_ADC_CLKSRC_PCLK2_DIV_6    RCC_CFGR_ADCPRE_DIV6 /*ADC prescaler PCLK2 divided by 6*/
#define LL_RCC_ADC_CLKSRC_PCLK2_DIV_8    RCC_CFGR_ADCPRE_DIV8 /*ADC prescaler PCLK2 divided by 8*/
/**
  * @}
  */

#if defined(RCC_CFGR2_I2S2SRC)
/** @defgroup RCC_LL_EC_I2S2 Peripheral I2S get clock source
  * @{
  */
#define LL_RCC_I2S2_CLKSOURCE              RCC_CFGR2_I2S2SRC       /*!< I2S2 Clock source selection */
#define LL_RCC_I2S3_CLKSOURCE              RCC_CFGR2_I2S3SRC       /*!< I2S3 Clock source selection */
/**
  * @}
  */

#endif /* RCC_CFGR2_I2S2SRC */

#if defined(USB_OTG_FS) || defined(USB)
/** @defgroup RCC_LL_EC_USB Peripheral USB get clock source
  * @{
  */
#define LL_RCC_USB_CLKSOURCE               0x00400000U     /*!< USB Clock source selection */
/**
  * @}
  */

#endif /* USB_OTG_FS || USB */

/** @defgroup RCC_LL_EC_ADC Peripheral ADC get clock source
  * @{
  */
#define LL_RCC_ADC_CLKSOURCE               RCC_CFGR_ADCPRE /*!< ADC Clock source selection */
/**
  * @}
  */

/** @defgroup RCC_LL_EC_RTC_CLKSOURCE  RTC clock source selection
  * @{
  */
#define LL_RCC_RTC_CLKSOURCE_NONE          0x00000000U             /*!< No clock used as RTC clock */
#define LL_RCC_RTC_CLKSOURCE_LSE           RCC_BDCR_RTCSEL_0       /*!< LSE oscillator clock used as RTC clock */
#define LL_RCC_RTC_CLKSOURCE_LSI           RCC_BDCR_RTCSEL_1       /*!< LSI oscillator clock used as RTC clock */
#define LL_RCC_RTC_CLKSOURCE_HSE_DIV128    RCC_BDCR_RTCSEL         /*!< HSE oscillator clock divided by 128 used as RTC clock */
/**
  * @}
  */

/** @defgroup RCC_LL_EC_PLL_MUL PLL Multiplicator factor
  * @{
  */
#if defined(RCC_CFGR_PLLMULL2)
#define LL_RCC_PLL_MUL_2                   RCC_CFGR_PLLMULL2  /*!< PLL input clock*2 */
#endif /*RCC_CFGR_PLLMULL2*/
#if defined(RCC_CFGR_PLLMULL3)
#define LL_RCC_PLL_MUL_3                   RCC_CFGR_PLLMULL3  /*!< PLL input clock*3 */
#endif /*RCC_CFGR_PLLMULL3*/
#define LL_RCC_PLL_MUL_4                   RCC_CFGR_PLLMULL4  /*!< PLL input clock*4 */
#define LL_RCC_PLL_MUL_5                   RCC_CFGR_PLLMULL5  /*!< PLL input clock*5 */
#define LL_RCC_PLL_MUL_6                   RCC_CFGR_PLLMULL6  /*!< PLL input clock*6 */
#define LL_RCC_PLL_MUL_7                   RCC_CFGR_PLLMULL7  /*!< PLL input clock*7 */
#define LL_RCC_PLL_MUL_8                   RCC_CFGR_PLLMULL8  /*!< PLL input clock*8 */
#define LL_RCC_PLL_MUL_9                   RCC_CFGR_PLLMULL9  /*!< PLL input clock*9 */
#if defined(RCC_CFGR_PLLMULL6_5)
#define LL_RCC_PLL_MUL_6_5                 RCC_CFGR_PLLMULL6_5 /*!< PLL input clock*6 */
#else
#define LL_RCC_PLL_MUL_10                  RCC_CFGR_PLLMULL10  /*!< PLL input clock*10 */
#define LL_RCC_PLL_MUL_11                  RCC_CFGR_PLLMULL11  /*!< PLL input clock*11 */
#define LL_RCC_PLL_MUL_12                  RCC_CFGR_PLLMULL12  /*!< PLL input clock*12 */
#define LL_RCC_PLL_MUL_13                  RCC_CFGR_PLLMULL13  /*!< PLL input clock*13 */
#define LL_RCC_PLL_MUL_14                  RCC_CFGR_PLLMULL14  /*!< PLL input clock*14 */
#define LL_RCC_PLL_MUL_15                  RCC_CFGR_PLLMULL15  /*!< PLL input clock*15 */
#define LL_RCC_PLL_MUL_16                  RCC_CFGR_PLLMULL16  /*!< PLL input clock*16 */
#endif /*RCC_CFGR_PLLMULL6_5*/
/**
  * @}
  */

/** @defgroup RCC_LL_EC_PLLSOURCE PLL SOURCE
  * @{
  */
#define LL_RCC_PLLSOURCE_HSI_DIV_2         0x00000000U                                    /*!< HSI clock divided by 2 selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE               RCC_CFGR_PLLSRC                                /*!< HSE/PREDIV1 clock selected as PLL entry clock source */
#if defined(RCC_CFGR2_PREDIV1SRC)
#define LL_RCC_PLLSOURCE_PLL2              (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1SRC << 4U) /*!< PLL2/PREDIV1 clock selected as PLL entry clock source */
#endif /*RCC_CFGR2_PREDIV1SRC*/

#if defined(RCC_CFGR2_PREDIV1)
#define LL_RCC_PLLSOURCE_HSE_DIV_1         (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV1)    /*!< HSE/1 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_2         (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV2)    /*!< HSE/2 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_3         (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV3)    /*!< HSE/3 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_4         (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV4)    /*!< HSE/4 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_5         (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV5)    /*!< HSE/5 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_6         (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV6)    /*!< HSE/6 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_7         (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV7)    /*!< HSE/7 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_8         (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV8)    /*!< HSE/8 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_9         (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV9)    /*!< HSE/9 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_10        (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV10)   /*!< HSE/10 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_11        (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV11)   /*!< HSE/11 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_12        (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV12)   /*!< HSE/12 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_13        (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV13)   /*!< HSE/13 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_14        (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV14)   /*!< HSE/14 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_15        (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV15)   /*!< HSE/15 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_16        (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV16)   /*!< HSE/16 clock selected as PLL entry clock source */
#if defined(RCC_CFGR2_PREDIV1SRC)
#define LL_RCC_PLLSOURCE_PLL2_DIV_1        (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV1 | RCC_CFGR2_PREDIV1SRC << 4U)    /*!< PLL2/1 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_PLL2_DIV_2        (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV2 | RCC_CFGR2_PREDIV1SRC << 4U)    /*!< PLL2/2 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_PLL2_DIV_3        (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV3 | RCC_CFGR2_PREDIV1SRC << 4U)    /*!< PLL2/3 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_PLL2_DIV_4        (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV4 | RCC_CFGR2_PREDIV1SRC << 4U)    /*!< PLL2/4 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_PLL2_DIV_5        (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV5 | RCC_CFGR2_PREDIV1SRC << 4U)    /*!< PLL2/5 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_PLL2_DIV_6        (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV6 | RCC_CFGR2_PREDIV1SRC << 4U)    /*!< PLL2/6 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_PLL2_DIV_7        (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV7 | RCC_CFGR2_PREDIV1SRC << 4U)    /*!< PLL2/7 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_PLL2_DIV_8        (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV8 | RCC_CFGR2_PREDIV1SRC << 4U)    /*!< PLL2/8 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_PLL2_DIV_9        (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV9 | RCC_CFGR2_PREDIV1SRC << 4U)    /*!< PLL2/9 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_PLL2_DIV_10       (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV10 | RCC_CFGR2_PREDIV1SRC << 4U)   /*!< PLL2/10 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_PLL2_DIV_11       (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV11 | RCC_CFGR2_PREDIV1SRC << 4U)   /*!< PLL2/11 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_PLL2_DIV_12       (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV12 | RCC_CFGR2_PREDIV1SRC << 4U)   /*!< PLL2/12 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_PLL2_DIV_13       (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV13 | RCC_CFGR2_PREDIV1SRC << 4U)   /*!< PLL2/13 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_PLL2_DIV_14       (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV14 | RCC_CFGR2_PREDIV1SRC << 4U)   /*!< PLL2/14 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_PLL2_DIV_15       (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV15 | RCC_CFGR2_PREDIV1SRC << 4U)   /*!< PLL2/15 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_PLL2_DIV_16       (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1_DIV16 | RCC_CFGR2_PREDIV1SRC << 4U)   /*!< PLL2/16 clock selected as PLL entry clock source */
#endif /*RCC_CFGR2_PREDIV1SRC*/
#else
#define LL_RCC_PLLSOURCE_HSE_DIV_1         (RCC_CFGR_PLLSRC | 0x00000000U)               /*!< HSE/1 clock selected as PLL entry clock source */
#define LL_RCC_PLLSOURCE_HSE_DIV_2         (RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE)         /*!< HSE/2 clock selected as PLL entry clock source */
#endif /*RCC_CFGR2_PREDIV1*/
/**
  * @}
  */

/** @defgroup RCC_LL_EC_PREDIV_DIV PREDIV Division factor
  * @{
  */
#if defined(RCC_CFGR2_PREDIV1)
#define LL_RCC_PREDIV_DIV_1                RCC_CFGR2_PREDIV1_DIV1   /*!< PREDIV1 input clock not divided */
#define LL_RCC_PREDIV_DIV_2                RCC_CFGR2_PREDIV1_DIV2   /*!< PREDIV1 input clock divided by 2 */
#define LL_RCC_PREDIV_DIV_3                RCC_CFGR2_PREDIV1_DIV3   /*!< PREDIV1 input clock divided by 3 */
#define LL_RCC_PREDIV_DIV_4                RCC_CFGR2_PREDIV1_DIV4   /*!< PREDIV1 input clock divided by 4 */
#define LL_RCC_PREDIV_DIV_5                RCC_CFGR2_PREDIV1_DIV5   /*!< PREDIV1 input clock divided by 5 */
#define LL_RCC_PREDIV_DIV_6                RCC_CFGR2_PREDIV1_DIV6   /*!< PREDIV1 input clock divided by 6 */
#define LL_RCC_PREDIV_DIV_7                RCC_CFGR2_PREDIV1_DIV7   /*!< PREDIV1 input clock divided by 7 */
#define LL_RCC_PREDIV_DIV_8                RCC_CFGR2_PREDIV1_DIV8   /*!< PREDIV1 input clock divided by 8 */
#define LL_RCC_PREDIV_DIV_9                RCC_CFGR2_PREDIV1_DIV9   /*!< PREDIV1 input clock divided by 9 */
#define LL_RCC_PREDIV_DIV_10               RCC_CFGR2_PREDIV1_DIV10  /*!< PREDIV1 input clock divided by 10 */
#define LL_RCC_PREDIV_DIV_11               RCC_CFGR2_PREDIV1_DIV11  /*!< PREDIV1 input clock divided by 11 */
#define LL_RCC_PREDIV_DIV_12               RCC_CFGR2_PREDIV1_DIV12  /*!< PREDIV1 input clock divided by 12 */
#define LL_RCC_PREDIV_DIV_13               RCC_CFGR2_PREDIV1_DIV13  /*!< PREDIV1 input clock divided by 13 */
#define LL_RCC_PREDIV_DIV_14               RCC_CFGR2_PREDIV1_DIV14  /*!< PREDIV1 input clock divided by 14 */
#define LL_RCC_PREDIV_DIV_15               RCC_CFGR2_PREDIV1_DIV15  /*!< PREDIV1 input clock divided by 15 */
#define LL_RCC_PREDIV_DIV_16               RCC_CFGR2_PREDIV1_DIV16  /*!< PREDIV1 input clock divided by 16 */
#else
#define LL_RCC_PREDIV_DIV_1                0x00000000U              /*!< HSE divider clock clock not divided */
#define LL_RCC_PREDIV_DIV_2                RCC_CFGR_PLLXTPRE        /*!< HSE divider clock divided by 2 for PLL entry */
#endif /*RCC_CFGR2_PREDIV1*/
/**
  * @}
  */

#if defined(RCC_PLLI2S_SUPPORT)
/** @defgroup RCC_LL_EC_PLLI2S_MUL PLLI2S MUL
  * @{
  */
#define LL_RCC_PLLI2S_MUL_8                RCC_CFGR2_PLL3MUL8   /*!< PLLI2S input clock * 8 */
#define LL_RCC_PLLI2S_MUL_9                RCC_CFGR2_PLL3MUL9   /*!< PLLI2S input clock * 9 */
#define LL_RCC_PLLI2S_MUL_10               RCC_CFGR2_PLL3MUL10  /*!< PLLI2S input clock * 10 */
#define LL_RCC_PLLI2S_MUL_11               RCC_CFGR2_PLL3MUL11  /*!< PLLI2S input clock * 11 */
#define LL_RCC_PLLI2S_MUL_12               RCC_CFGR2_PLL3MUL12  /*!< PLLI2S input clock * 12 */
#define LL_RCC_PLLI2S_MUL_13               RCC_CFGR2_PLL3MUL13  /*!< PLLI2S input clock * 13 */
#define LL_RCC_PLLI2S_MUL_14               RCC_CFGR2_PLL3MUL14  /*!< PLLI2S input clock * 14 */
#define LL_RCC_PLLI2S_MUL_16               RCC_CFGR2_PLL3MUL16  /*!< PLLI2S input clock * 16 */
#define LL_RCC_PLLI2S_MUL_20               RCC_CFGR2_PLL3MUL20  /*!< PLLI2S input clock * 20 */
/**
  * @}
  */

#endif /* RCC_PLLI2S_SUPPORT */

#if defined(RCC_PLL2_SUPPORT)
/** @defgroup RCC_LL_EC_PLL2_MUL PLL2 MUL
  * @{
  */
#define LL_RCC_PLL2_MUL_8                  RCC_CFGR2_PLL2MUL8   /*!< PLL2 input clock * 8 */
#define LL_RCC_PLL2_MUL_9                  RCC_CFGR2_PLL2MUL9   /*!< PLL2 input clock * 9 */
#define LL_RCC_PLL2_MUL_10                 RCC_CFGR2_PLL2MUL10  /*!< PLL2 input clock * 10 */
#define LL_RCC_PLL2_MUL_11                 RCC_CFGR2_PLL2MUL11  /*!< PLL2 input clock * 11 */
#define LL_RCC_PLL2_MUL_12                 RCC_CFGR2_PLL2MUL12  /*!< PLL2 input clock * 12 */
#define LL_RCC_PLL2_MUL_13                 RCC_CFGR2_PLL2MUL13  /*!< PLL2 input clock * 13 */
#define LL_RCC_PLL2_MUL_14                 RCC_CFGR2_PLL2MUL14  /*!< PLL2 input clock * 14 */
#define LL_RCC_PLL2_MUL_16                 RCC_CFGR2_PLL2MUL16  /*!< PLL2 input clock * 16 */
#define LL_RCC_PLL2_MUL_20                 RCC_CFGR2_PLL2MUL20  /*!< PLL2 input clock * 20 */
/**
  * @}
  */

#endif /* RCC_PLL2_SUPPORT */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup RCC_LL_Exported_Macros RCC Exported Macros
  * @{
  */

/** @defgroup RCC_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in RCC register
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_RCC_WriteReg(__REG__, __VALUE__) WRITE_REG(RCC->__REG__, (__VALUE__))

/**
  * @brief  Read a value in RCC register
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_RCC_ReadReg(__REG__) READ_REG(RCC->__REG__)
/**
  * @}
  */

/** @defgroup RCC_LL_EM_CALC_FREQ Calculate frequencies
  * @{
  */

#if defined(RCC_CFGR_PLLMULL6_5)
/**
  * @brief  Helper macro to calculate the PLLCLK frequency
  * @note ex: @ref __LL_RCC_CALC_PLLCLK_FREQ (HSE_VALUE / (@ref LL_RCC_PLL_GetPrediv () + 1), @ref LL_RCC_PLL_GetMultiplicator());
  * @param  __INPUTFREQ__ PLL Input frequency (based on HSE div Prediv1 / HSI div 2 / PLL2 div Prediv1)
  * @param  __PLLMUL__: This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLL_MUL_4
  *         @arg @ref LL_RCC_PLL_MUL_5
  *         @arg @ref LL_RCC_PLL_MUL_6
  *         @arg @ref LL_RCC_PLL_MUL_7
  *         @arg @ref LL_RCC_PLL_MUL_8
  *         @arg @ref LL_RCC_PLL_MUL_9
  *         @arg @ref LL_RCC_PLL_MUL_6_5
  * @retval PLL clock frequency (in Hz)
  */
#define __LL_RCC_CALC_PLLCLK_FREQ(__INPUTFREQ__, __PLLMUL__) \
          (((__PLLMUL__) != RCC_CFGR_PLLMULL6_5) ? \
              ((__INPUTFREQ__) * ((((__PLLMUL__) & RCC_CFGR_PLLMULL) >> RCC_CFGR_PLLMULL_Pos) + 2U)) :\
              (((__INPUTFREQ__) * 13U) / 2U))

#else
/**
  * @brief  Helper macro to calculate the PLLCLK frequency
  * @note ex: @ref __LL_RCC_CALC_PLLCLK_FREQ (HSE_VALUE / (@ref LL_RCC_PLL_GetPrediv () + 1), @ref LL_RCC_PLL_GetMultiplicator ());
  * @param  __INPUTFREQ__ PLL Input frequency (based on HSE div Prediv1 or div 2 / HSI div 2)
  * @param  __PLLMUL__: This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLL_MUL_2
  *         @arg @ref LL_RCC_PLL_MUL_3
  *         @arg @ref LL_RCC_PLL_MUL_4
  *         @arg @ref LL_RCC_PLL_MUL_5
  *         @arg @ref LL_RCC_PLL_MUL_6
  *         @arg @ref LL_RCC_PLL_MUL_7
  *         @arg @ref LL_RCC_PLL_MUL_8
  *         @arg @ref LL_RCC_PLL_MUL_9
  *         @arg @ref LL_RCC_PLL_MUL_10
  *         @arg @ref LL_RCC_PLL_MUL_11
  *         @arg @ref LL_RCC_PLL_MUL_12
  *         @arg @ref LL_RCC_PLL_MUL_13
  *         @arg @ref LL_RCC_PLL_MUL_14
  *         @arg @ref LL_RCC_PLL_MUL_15
  *         @arg @ref LL_RCC_PLL_MUL_16
  * @retval PLL clock frequency (in Hz)
  */
#define __LL_RCC_CALC_PLLCLK_FREQ(__INPUTFREQ__, __PLLMUL__) ((__INPUTFREQ__) * (((__PLLMUL__) >> RCC_CFGR_PLLMULL_Pos) + 2U))
#endif /* RCC_CFGR_PLLMULL6_5 */

#if defined(RCC_PLLI2S_SUPPORT)
/**
  * @brief  Helper macro to calculate the PLLI2S frequency
  * @note ex: @ref __LL_RCC_CALC_PLLI2SCLK_FREQ (HSE_VALUE, @ref LL_RCC_PLLI2S_GetMultiplicator (), @ref LL_RCC_HSE_GetPrediv2 ());
  * @param  __INPUTFREQ__ PLLI2S Input frequency (based on HSE value)
  * @param  __PLLI2SMUL__: This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLI2S_MUL_8
  *         @arg @ref LL_RCC_PLLI2S_MUL_9
  *         @arg @ref LL_RCC_PLLI2S_MUL_10
  *         @arg @ref LL_RCC_PLLI2S_MUL_11
  *         @arg @ref LL_RCC_PLLI2S_MUL_12
  *         @arg @ref LL_RCC_PLLI2S_MUL_13
  *         @arg @ref LL_RCC_PLLI2S_MUL_14
  *         @arg @ref LL_RCC_PLLI2S_MUL_16
  *         @arg @ref LL_RCC_PLLI2S_MUL_20
  * @param  __PLLI2SDIV__: This parameter can be one of the following values:
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_1
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_2
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_3
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_4
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_5
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_6
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_7
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_8
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_9
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_10
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_11
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_12
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_13
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_14
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_15
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_16
  * @retval PLLI2S clock frequency (in Hz)
  */
#define __LL_RCC_CALC_PLLI2SCLK_FREQ(__INPUTFREQ__, __PLLI2SMUL__, __PLLI2SDIV__) (((__INPUTFREQ__) * (((__PLLI2SMUL__) >> RCC_CFGR2_PLL3MUL_Pos) + 2U)) / (((__PLLI2SDIV__) >> RCC_CFGR2_PREDIV2_Pos) + 1U))
#endif /* RCC_PLLI2S_SUPPORT */

#if defined(RCC_PLL2_SUPPORT)
/**
  * @brief  Helper macro to calculate the PLL2 frequency
  * @note ex: @ref __LL_RCC_CALC_PLL2CLK_FREQ (HSE_VALUE, @ref LL_RCC_PLL2_GetMultiplicator (), @ref LL_RCC_HSE_GetPrediv2 ());
  * @param  __INPUTFREQ__ PLL2 Input frequency (based on HSE value)
  * @param  __PLL2MUL__: This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLL2_MUL_8
  *         @arg @ref LL_RCC_PLL2_MUL_9
  *         @arg @ref LL_RCC_PLL2_MUL_10
  *         @arg @ref LL_RCC_PLL2_MUL_11
  *         @arg @ref LL_RCC_PLL2_MUL_12
  *         @arg @ref LL_RCC_PLL2_MUL_13
  *         @arg @ref LL_RCC_PLL2_MUL_14
  *         @arg @ref LL_RCC_PLL2_MUL_16
  *         @arg @ref LL_RCC_PLL2_MUL_20
  * @param  __PLL2DIV__: This parameter can be one of the following values:
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_1
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_2
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_3
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_4
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_5
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_6
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_7
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_8
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_9
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_10
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_11
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_12
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_13
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_14
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_15
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_16
  * @retval PLL2 clock frequency (in Hz)
  */
#define __LL_RCC_CALC_PLL2CLK_FREQ(__INPUTFREQ__, __PLL2MUL__, __PLL2DIV__) (((__INPUTFREQ__) * (((__PLL2MUL__) >> RCC_CFGR2_PLL2MUL_Pos) + 2U)) / (((__PLL2DIV__) >> RCC_CFGR2_PREDIV2_Pos) + 1U))
#endif /* RCC_PLL2_SUPPORT */

/**
  * @brief  Helper macro to calculate the HCLK frequency
  * @note: __AHBPRESCALER__ be retrieved by @ref LL_RCC_GetAHBPrescaler
  *        ex: __LL_RCC_CALC_HCLK_FREQ(LL_RCC_GetAHBPrescaler())
  * @param  __SYSCLKFREQ__ SYSCLK frequency (based on HSE/HSI/PLLCLK)
  * @param  __AHBPRESCALER__: This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SYSCLK_DIV_1
  *         @arg @ref LL_RCC_SYSCLK_DIV_2
  *         @arg @ref LL_RCC_SYSCLK_DIV_4
  *         @arg @ref LL_RCC_SYSCLK_DIV_8
  *         @arg @ref LL_RCC_SYSCLK_DIV_16
  *         @arg @ref LL_RCC_SYSCLK_DIV_64
  *         @arg @ref LL_RCC_SYSCLK_DIV_128
  *         @arg @ref LL_RCC_SYSCLK_DIV_256
  *         @arg @ref LL_RCC_SYSCLK_DIV_512
  * @retval HCLK clock frequency (in Hz)
  */
#define __LL_RCC_CALC_HCLK_FREQ(__SYSCLKFREQ__, __AHBPRESCALER__) ((__SYSCLKFREQ__) >> AHBPrescTable[((__AHBPRESCALER__) & RCC_CFGR_HPRE) >>  RCC_CFGR_HPRE_Pos])

/**
  * @brief  Helper macro to calculate the PCLK1 frequency (ABP1)
  * @note: __APB1PRESCALER__ be retrieved by @ref LL_RCC_GetAPB1Prescaler
  *        ex: __LL_RCC_CALC_PCLK1_FREQ(LL_RCC_GetAPB1Prescaler())
  * @param  __HCLKFREQ__ HCLK frequency
  * @param  __APB1PRESCALER__: This parameter can be one of the following values:
  *         @arg @ref LL_RCC_APB1_DIV_1
  *         @arg @ref LL_RCC_APB1_DIV_2
  *         @arg @ref LL_RCC_APB1_DIV_4
  *         @arg @ref LL_RCC_APB1_DIV_8
  *         @arg @ref LL_RCC_APB1_DIV_16
  * @retval PCLK1 clock frequency (in Hz)
  */
#define __LL_RCC_CALC_PCLK1_FREQ(__HCLKFREQ__, __APB1PRESCALER__) ((__HCLKFREQ__) >> APBPrescTable[(__APB1PRESCALER__) >>  RCC_CFGR_PPRE1_Pos])

/**
  * @brief  Helper macro to calculate the PCLK2 frequency (ABP2)
  * @note: __APB2PRESCALER__ be retrieved by @ref LL_RCC_GetAPB2Prescaler
  *        ex: __LL_RCC_CALC_PCLK2_FREQ(LL_RCC_GetAPB2Prescaler())
  * @param  __HCLKFREQ__ HCLK frequency
  * @param  __APB2PRESCALER__: This parameter can be one of the following values:
  *         @arg @ref LL_RCC_APB2_DIV_1
  *         @arg @ref LL_RCC_APB2_DIV_2
  *         @arg @ref LL_RCC_APB2_DIV_4
  *         @arg @ref LL_RCC_APB2_DIV_8
  *         @arg @ref LL_RCC_APB2_DIV_16
  * @retval PCLK2 clock frequency (in Hz)
  */
#define __LL_RCC_CALC_PCLK2_FREQ(__HCLKFREQ__, __APB2PRESCALER__) ((__HCLKFREQ__) >> APBPrescTable[(__APB2PRESCALER__) >>  RCC_CFGR_PPRE2_Pos])

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup RCC_LL_Exported_Functions RCC Exported Functions
  * @{
  */

/** @defgroup RCC_LL_EF_HSE HSE
  * @{
  */

/**
  * @brief  Enable the Clock Security System.
  * @rmtoll CR           CSSON         LL_RCC_HSE_EnableCSS
  * @retval None
  */
__STATIC_INLINE void LL_RCC_HSE_EnableCSS(void)
{
  SET_BIT(RCC->CR, RCC_CR_CSSON);
}

/**
  * @brief  Enable HSE external oscillator (HSE Bypass)
  * @rmtoll CR           HSEBYP        LL_RCC_HSE_EnableBypass
  * @retval None
  */
__STATIC_INLINE void LL_RCC_HSE_EnableBypass(void)
{
  SET_BIT(RCC->CR, RCC_CR_HSEBYP);
}

/**
  * @brief  Disable HSE external oscillator (HSE Bypass)
  * @rmtoll CR           HSEBYP        LL_RCC_HSE_DisableBypass
  * @retval None
  */
__STATIC_INLINE void LL_RCC_HSE_DisableBypass(void)
{
  CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);
}

/**
  * @brief  Enable HSE crystal oscillator (HSE ON)
  * @rmtoll CR           HSEON         LL_RCC_HSE_Enable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_HSE_Enable(void)
{
  SET_BIT(RCC->CR, RCC_CR_HSEON);
}

/**
  * @brief  Disable HSE crystal oscillator (HSE ON)
  * @rmtoll CR           HSEON         LL_RCC_HSE_Disable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_HSE_Disable(void)
{
  CLEAR_BIT(RCC->CR, RCC_CR_HSEON);
}

/**
  * @brief  Check if HSE oscillator Ready
  * @rmtoll CR           HSERDY        LL_RCC_HSE_IsReady
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_HSE_IsReady(void)
{
  return (READ_BIT(RCC->CR, RCC_CR_HSERDY) == (RCC_CR_HSERDY));
}

#if defined(RCC_CFGR2_PREDIV2)
/**
  * @brief  Get PREDIV2 division factor
  * @rmtoll CFGR2        PREDIV2       LL_RCC_HSE_GetPrediv2
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_1
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_2
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_3
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_4
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_5
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_6
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_7
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_8
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_9
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_10
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_11
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_12
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_13
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_14
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_15
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_16
  */
__STATIC_INLINE uint32_t LL_RCC_HSE_GetPrediv2(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR2, RCC_CFGR2_PREDIV2));
}
#endif /* RCC_CFGR2_PREDIV2 */

/**
  * @}
  */

/** @defgroup RCC_LL_EF_HSI HSI
  * @{
  */

/**
  * @brief  Enable HSI oscillator
  * @rmtoll CR           HSION         LL_RCC_HSI_Enable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_HSI_Enable(void)
{
  SET_BIT(RCC->CR, RCC_CR_HSION);
}

/**
  * @brief  Disable HSI oscillator
  * @rmtoll CR           HSION         LL_RCC_HSI_Disable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_HSI_Disable(void)
{
  CLEAR_BIT(RCC->CR, RCC_CR_HSION);
}

/**
  * @brief  Check if HSI clock is ready
  * @rmtoll CR           HSIRDY        LL_RCC_HSI_IsReady
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_HSI_IsReady(void)
{
  return (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == (RCC_CR_HSIRDY));
}

/**
  * @brief  Get HSI Calibration value
  * @note When HSITRIM is written, HSICAL is updated with the sum of
  *       HSITRIM and the factory trim value
  * @rmtoll CR        HSICAL        LL_RCC_HSI_GetCalibration
  * @retval Between Min_Data = 0x00 and Max_Data = 0xFF
  */
__STATIC_INLINE uint32_t LL_RCC_HSI_GetCalibration(void)
{
  return (uint32_t)(READ_BIT(RCC->CR, RCC_CR_HSICAL) >> RCC_CR_HSICAL_Pos);
}

/**
  * @brief  Set HSI Calibration trimming
  * @note user-programmable trimming value that is added to the HSICAL
  * @note Default value is 16, which, when added to the HSICAL value,
  *       should trim the HSI to 16 MHz +/- 1 %
  * @rmtoll CR        HSITRIM       LL_RCC_HSI_SetCalibTrimming
  * @param  Value between Min_Data = 0x00 and Max_Data = 0x1F
  * @retval None
  */
__STATIC_INLINE void LL_RCC_HSI_SetCalibTrimming(uint32_t Value)
{
  MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, Value << RCC_CR_HSITRIM_Pos);
}

/**
  * @brief  Get HSI Calibration trimming
  * @rmtoll CR        HSITRIM       LL_RCC_HSI_GetCalibTrimming
  * @retval Between Min_Data = 0x00 and Max_Data = 0x1F
  */
__STATIC_INLINE uint32_t LL_RCC_HSI_GetCalibTrimming(void)
{
  return (uint32_t)(READ_BIT(RCC->CR, RCC_CR_HSITRIM) >> RCC_CR_HSITRIM_Pos);
}

/**
  * @}
  */

/** @defgroup RCC_LL_EF_LSE LSE
  * @{
  */

/**
  * @brief  Enable  Low Speed External (LSE) crystal.
  * @rmtoll BDCR         LSEON         LL_RCC_LSE_Enable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSE_Enable(void)
{
  SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);
}

/**
  * @brief  Disable  Low Speed External (LSE) crystal.
  * @rmtoll BDCR         LSEON         LL_RCC_LSE_Disable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSE_Disable(void)
{
  CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);
}

/**
  * @brief  Enable external clock source (LSE bypass).
  * @rmtoll BDCR         LSEBYP        LL_RCC_LSE_EnableBypass
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSE_EnableBypass(void)
{
  SET_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);
}

/**
  * @brief  Disable external clock source (LSE bypass).
  * @rmtoll BDCR         LSEBYP        LL_RCC_LSE_DisableBypass
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSE_DisableBypass(void)
{
  CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);
}

/**
  * @brief  Check if LSE oscillator Ready
  * @rmtoll BDCR         LSERDY        LL_RCC_LSE_IsReady
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_LSE_IsReady(void)
{
  return (READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY) == (RCC_BDCR_LSERDY));
}

/**
  * @}
  */

/** @defgroup RCC_LL_EF_LSI LSI
  * @{
  */

/**
  * @brief  Enable LSI Oscillator
  * @rmtoll CSR          LSION         LL_RCC_LSI_Enable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSI_Enable(void)
{
  SET_BIT(RCC->CSR, RCC_CSR_LSION);
}

/**
  * @brief  Disable LSI Oscillator
  * @rmtoll CSR          LSION         LL_RCC_LSI_Disable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_LSI_Disable(void)
{
  CLEAR_BIT(RCC->CSR, RCC_CSR_LSION);
}

/**
  * @brief  Check if LSI is Ready
  * @rmtoll CSR          LSIRDY        LL_RCC_LSI_IsReady
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_LSI_IsReady(void)
{
  return (READ_BIT(RCC->CSR, RCC_CSR_LSIRDY) == (RCC_CSR_LSIRDY));
}

/**
  * @}
  */

/** @defgroup RCC_LL_EF_System System
  * @{
  */

/**
  * @brief  Configure the system clock source
  * @rmtoll CFGR         SW            LL_RCC_SetSysClkSource
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_HSE
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_PLL
  * @retval None
  */
__STATIC_INLINE void LL_RCC_SetSysClkSource(uint32_t Source)
{
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, Source);
}

/**
  * @brief  Get the system clock source
  * @rmtoll CFGR         SWS           LL_RCC_GetSysClkSource
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_STATUS_HSI
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_STATUS_HSE
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_STATUS_PLL
  */
__STATIC_INLINE uint32_t LL_RCC_GetSysClkSource(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_SWS));
}

/**
  * @brief  Set AHB prescaler
  * @rmtoll CFGR         HPRE          LL_RCC_SetAHBPrescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SYSCLK_DIV_1
  *         @arg @ref LL_RCC_SYSCLK_DIV_2
  *         @arg @ref LL_RCC_SYSCLK_DIV_4
  *         @arg @ref LL_RCC_SYSCLK_DIV_8
  *         @arg @ref LL_RCC_SYSCLK_DIV_16
  *         @arg @ref LL_RCC_SYSCLK_DIV_64
  *         @arg @ref LL_RCC_SYSCLK_DIV_128
  *         @arg @ref LL_RCC_SYSCLK_DIV_256
  *         @arg @ref LL_RCC_SYSCLK_DIV_512
  * @retval None
  */
__STATIC_INLINE void LL_RCC_SetAHBPrescaler(uint32_t Prescaler)
{
  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, Prescaler);
}

/**
  * @brief  Set APB1 prescaler
  * @rmtoll CFGR         PPRE1         LL_RCC_SetAPB1Prescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref LL_RCC_APB1_DIV_1
  *         @arg @ref LL_RCC_APB1_DIV_2
  *         @arg @ref LL_RCC_APB1_DIV_4
  *         @arg @ref LL_RCC_APB1_DIV_8
  *         @arg @ref LL_RCC_APB1_DIV_16
  * @retval None
  */
__STATIC_INLINE void LL_RCC_SetAPB1Prescaler(uint32_t Prescaler)
{
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, Prescaler);
}

/**
  * @brief  Set APB2 prescaler
  * @rmtoll CFGR         PPRE2         LL_RCC_SetAPB2Prescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref LL_RCC_APB2_DIV_1
  *         @arg @ref LL_RCC_APB2_DIV_2
  *         @arg @ref LL_RCC_APB2_DIV_4
  *         @arg @ref LL_RCC_APB2_DIV_8
  *         @arg @ref LL_RCC_APB2_DIV_16
  * @retval None
  */
__STATIC_INLINE void LL_RCC_SetAPB2Prescaler(uint32_t Prescaler)
{
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, Prescaler);
}

/**
  * @brief  Get AHB prescaler
  * @rmtoll CFGR         HPRE          LL_RCC_GetAHBPrescaler
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_SYSCLK_DIV_1
  *         @arg @ref LL_RCC_SYSCLK_DIV_2
  *         @arg @ref LL_RCC_SYSCLK_DIV_4
  *         @arg @ref LL_RCC_SYSCLK_DIV_8
  *         @arg @ref LL_RCC_SYSCLK_DIV_16
  *         @arg @ref LL_RCC_SYSCLK_DIV_64
  *         @arg @ref LL_RCC_SYSCLK_DIV_128
  *         @arg @ref LL_RCC_SYSCLK_DIV_256
  *         @arg @ref LL_RCC_SYSCLK_DIV_512
  */
__STATIC_INLINE uint32_t LL_RCC_GetAHBPrescaler(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_HPRE));
}

/**
  * @brief  Get APB1 prescaler
  * @rmtoll CFGR         PPRE1         LL_RCC_GetAPB1Prescaler
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_APB1_DIV_1
  *         @arg @ref LL_RCC_APB1_DIV_2
  *         @arg @ref LL_RCC_APB1_DIV_4
  *         @arg @ref LL_RCC_APB1_DIV_8
  *         @arg @ref LL_RCC_APB1_DIV_16
  */
__STATIC_INLINE uint32_t LL_RCC_GetAPB1Prescaler(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1));
}

/**
  * @brief  Get APB2 prescaler
  * @rmtoll CFGR         PPRE2         LL_RCC_GetAPB2Prescaler
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_APB2_DIV_1
  *         @arg @ref LL_RCC_APB2_DIV_2
  *         @arg @ref LL_RCC_APB2_DIV_4
  *         @arg @ref LL_RCC_APB2_DIV_8
  *         @arg @ref LL_RCC_APB2_DIV_16
  */
__STATIC_INLINE uint32_t LL_RCC_GetAPB2Prescaler(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PPRE2));
}

/**
  * @}
  */

/** @defgroup RCC_LL_EF_MCO MCO
  * @{
  */

/**
  * @brief  Configure MCOx
  * @rmtoll CFGR         MCO           LL_RCC_ConfigMCO
  * @param  MCOxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_MCO1SOURCE_NOCLOCK
  *         @arg @ref LL_RCC_MCO1SOURCE_SYSCLK
  *         @arg @ref LL_RCC_MCO1SOURCE_HSI
  *         @arg @ref LL_RCC_MCO1SOURCE_HSE
  *         @arg @ref LL_RCC_MCO1SOURCE_PLLCLK_DIV_2
  *         @arg @ref LL_RCC_MCO1SOURCE_PLL2CLK (*)
  *         @arg @ref LL_RCC_MCO1SOURCE_PLLI2SCLK_DIV2 (*)
  *         @arg @ref LL_RCC_MCO1SOURCE_EXT_HSE (*)
  *         @arg @ref LL_RCC_MCO1SOURCE_PLLI2SCLK (*)
  *
  *         (*) value not defined in all devices
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ConfigMCO(uint32_t MCOxSource)
{
  MODIFY_REG(RCC->CFGR, RCC_CFGR_MCOSEL, MCOxSource);
}

/**
  * @}
  */

/** @defgroup RCC_LL_EF_Peripheral_Clock_Source Peripheral Clock Source
  * @{
  */

#if defined(RCC_CFGR2_I2S2SRC)
/**
  * @brief  Configure I2Sx clock source
  * @rmtoll CFGR2        I2S2SRC       LL_RCC_SetI2SClockSource\n
  *         CFGR2        I2S3SRC       LL_RCC_SetI2SClockSource
  * @param  I2SxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_I2S2_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_I2S2_CLKSOURCE_PLLI2S_VCO
  *         @arg @ref LL_RCC_I2S3_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_I2S3_CLKSOURCE_PLLI2S_VCO
  * @retval None
  */
__STATIC_INLINE void LL_RCC_SetI2SClockSource(uint32_t I2SxSource)
{
  MODIFY_REG(RCC->CFGR2, (I2SxSource & 0xFFFF0000U), (I2SxSource << 16U));
}
#endif /* RCC_CFGR2_I2S2SRC */

#if defined(USB_OTG_FS) || defined(USB)
/**
  * @brief  Configure USB clock source
  * @rmtoll CFGR         OTGFSPRE      LL_RCC_SetUSBClockSource\n
  *         CFGR         USBPRE        LL_RCC_SetUSBClockSource
  * @param  USBxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_USB_CLKSOURCE_PLL (*)
  *         @arg @ref LL_RCC_USB_CLKSOURCE_PLL_DIV_1_5 (*)
  *         @arg @ref LL_RCC_USB_CLKSOURCE_PLL_DIV_2 (*)
  *         @arg @ref LL_RCC_USB_CLKSOURCE_PLL_DIV_3 (*)
  *
  *         (*) value not defined in all devices
  * @retval None
  */
__STATIC_INLINE void LL_RCC_SetUSBClockSource(uint32_t USBxSource)
{
#if defined(RCC_CFGR_USBPRE)
  MODIFY_REG(RCC->CFGR, RCC_CFGR_USBPRE, USBxSource);
#else /*RCC_CFGR_OTGFSPRE*/
  MODIFY_REG(RCC->CFGR, RCC_CFGR_OTGFSPRE, USBxSource);
#endif /*RCC_CFGR_USBPRE*/
}
#endif /* USB_OTG_FS || USB */

/**
  * @brief  Configure ADC clock source
  * @rmtoll CFGR         ADCPRE        LL_RCC_SetADCClockSource
  * @param  ADCxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_ADC_CLKSRC_PCLK2_DIV_2
  *         @arg @ref LL_RCC_ADC_CLKSRC_PCLK2_DIV_4
  *         @arg @ref LL_RCC_ADC_CLKSRC_PCLK2_DIV_6
  *         @arg @ref LL_RCC_ADC_CLKSRC_PCLK2_DIV_8
  * @retval None
  */
__STATIC_INLINE void LL_RCC_SetADCClockSource(uint32_t ADCxSource)
{
  MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, ADCxSource);
}

#if defined(RCC_CFGR2_I2S2SRC)
/**
  * @brief  Get I2Sx clock source
  * @rmtoll CFGR2        I2S2SRC       LL_RCC_GetI2SClockSource\n
  *         CFGR2        I2S3SRC       LL_RCC_GetI2SClockSource
  * @param  I2Sx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_I2S2_CLKSOURCE
  *         @arg @ref LL_RCC_I2S3_CLKSOURCE
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_I2S2_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_I2S2_CLKSOURCE_PLLI2S_VCO
  *         @arg @ref LL_RCC_I2S3_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_I2S3_CLKSOURCE_PLLI2S_VCO
  */
__STATIC_INLINE uint32_t LL_RCC_GetI2SClockSource(uint32_t I2Sx)
{
  return (uint32_t)(READ_BIT(RCC->CFGR2, I2Sx) >> 16U | I2Sx);
}
#endif /* RCC_CFGR2_I2S2SRC */

#if defined(USB_OTG_FS) || defined(USB)
/**
  * @brief  Get USBx clock source
  * @rmtoll CFGR         OTGFSPRE      LL_RCC_GetUSBClockSource\n
  *         CFGR         USBPRE        LL_RCC_GetUSBClockSource
  * @param  USBx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_USB_CLKSOURCE
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_USB_CLKSOURCE_PLL (*)
  *         @arg @ref LL_RCC_USB_CLKSOURCE_PLL_DIV_1_5 (*)
  *         @arg @ref LL_RCC_USB_CLKSOURCE_PLL_DIV_2 (*)
  *         @arg @ref LL_RCC_USB_CLKSOURCE_PLL_DIV_3 (*)
  *
  *         (*) value not defined in all devices
  */
__STATIC_INLINE uint32_t LL_RCC_GetUSBClockSource(uint32_t USBx)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, USBx));
}
#endif /* USB_OTG_FS || USB */

/**
  * @brief  Get ADCx clock source
  * @rmtoll CFGR         ADCPRE        LL_RCC_GetADCClockSource
  * @param  ADCx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_ADC_CLKSOURCE
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_ADC_CLKSRC_PCLK2_DIV_2
  *         @arg @ref LL_RCC_ADC_CLKSRC_PCLK2_DIV_4
  *         @arg @ref LL_RCC_ADC_CLKSRC_PCLK2_DIV_6
  *         @arg @ref LL_RCC_ADC_CLKSRC_PCLK2_DIV_8
  */
__STATIC_INLINE uint32_t LL_RCC_GetADCClockSource(uint32_t ADCx)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, ADCx));
}

/**
  * @}
  */

/** @defgroup RCC_LL_EF_RTC RTC
  * @{
  */

/**
  * @brief  Set RTC Clock Source
  * @note Once the RTC clock source has been selected, it cannot be changed any more unless
  *       the Backup domain is reset. The BDRST bit can be used to reset them.
  * @rmtoll BDCR         RTCSEL        LL_RCC_SetRTCClockSource
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_RCC_RTC_CLKSOURCE_NONE
  *         @arg @ref LL_RCC_RTC_CLKSOURCE_LSE
  *         @arg @ref LL_RCC_RTC_CLKSOURCE_LSI
  *         @arg @ref LL_RCC_RTC_CLKSOURCE_HSE_DIV128
  * @retval None
  */
__STATIC_INLINE void LL_RCC_SetRTCClockSource(uint32_t Source)
{
  MODIFY_REG(RCC->BDCR, RCC_BDCR_RTCSEL, Source);
}

/**
  * @brief  Get RTC Clock Source
  * @rmtoll BDCR         RTCSEL        LL_RCC_GetRTCClockSource
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_RTC_CLKSOURCE_NONE
  *         @arg @ref LL_RCC_RTC_CLKSOURCE_LSE
  *         @arg @ref LL_RCC_RTC_CLKSOURCE_LSI
  *         @arg @ref LL_RCC_RTC_CLKSOURCE_HSE_DIV128
  */
__STATIC_INLINE uint32_t LL_RCC_GetRTCClockSource(void)
{
  return (uint32_t)(READ_BIT(RCC->BDCR, RCC_BDCR_RTCSEL));
}

/**
  * @brief  Enable RTC
  * @rmtoll BDCR         RTCEN         LL_RCC_EnableRTC
  * @retval None
  */
__STATIC_INLINE void LL_RCC_EnableRTC(void)
{
  SET_BIT(RCC->BDCR, RCC_BDCR_RTCEN);
}

/**
  * @brief  Disable RTC
  * @rmtoll BDCR         RTCEN         LL_RCC_DisableRTC
  * @retval None
  */
__STATIC_INLINE void LL_RCC_DisableRTC(void)
{
  CLEAR_BIT(RCC->BDCR, RCC_BDCR_RTCEN);
}

/**
  * @brief  Check if RTC has been enabled or not
  * @rmtoll BDCR         RTCEN         LL_RCC_IsEnabledRTC
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsEnabledRTC(void)
{
  return (READ_BIT(RCC->BDCR, RCC_BDCR_RTCEN) == (RCC_BDCR_RTCEN));
}

/**
  * @brief  Force the Backup domain reset
  * @rmtoll BDCR         BDRST         LL_RCC_ForceBackupDomainReset
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ForceBackupDomainReset(void)
{
  SET_BIT(RCC->BDCR, RCC_BDCR_BDRST);
}

/**
  * @brief  Release the Backup domain reset
  * @rmtoll BDCR         BDRST         LL_RCC_ReleaseBackupDomainReset
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ReleaseBackupDomainReset(void)
{
  CLEAR_BIT(RCC->BDCR, RCC_BDCR_BDRST);
}

/**
  * @}
  */

/** @defgroup RCC_LL_EF_PLL PLL
  * @{
  */

/**
  * @brief  Enable PLL
  * @rmtoll CR           PLLON         LL_RCC_PLL_Enable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_PLL_Enable(void)
{
  SET_BIT(RCC->CR, RCC_CR_PLLON);
}

/**
  * @brief  Disable PLL
  * @note Cannot be disabled if the PLL clock is used as the system clock
  * @rmtoll CR           PLLON         LL_RCC_PLL_Disable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_PLL_Disable(void)
{
  CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
}

/**
  * @brief  Check if PLL Ready
  * @rmtoll CR           PLLRDY        LL_RCC_PLL_IsReady
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_PLL_IsReady(void)
{
  return (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == (RCC_CR_PLLRDY));
}

/**
  * @brief  Configure PLL used for SYSCLK Domain
  * @rmtoll CFGR         PLLSRC        LL_RCC_PLL_ConfigDomain_SYS\n
  *         CFGR         PLLXTPRE      LL_RCC_PLL_ConfigDomain_SYS\n
  *         CFGR         PLLMULL       LL_RCC_PLL_ConfigDomain_SYS\n
  *         CFGR2        PREDIV1       LL_RCC_PLL_ConfigDomain_SYS\n
  *         CFGR2        PREDIV1SRC    LL_RCC_PLL_ConfigDomain_SYS
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLSOURCE_HSI_DIV_2
  *         @arg @ref LL_RCC_PLLSOURCE_HSE_DIV_1
  *         @arg @ref LL_RCC_PLLSOURCE_HSE_DIV_2 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_HSE_DIV_3 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_HSE_DIV_4 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_HSE_DIV_5 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_HSE_DIV_6 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_HSE_DIV_7 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_HSE_DIV_8 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_HSE_DIV_9 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_HSE_DIV_10 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_HSE_DIV_11 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_HSE_DIV_12 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_HSE_DIV_13 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_HSE_DIV_14 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_HSE_DIV_15 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_HSE_DIV_16 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2_DIV_1 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2_DIV_2 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2_DIV_3 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2_DIV_4 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2_DIV_5 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2_DIV_6 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2_DIV_7 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2_DIV_8 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2_DIV_9 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2_DIV_10 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2_DIV_11 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2_DIV_12 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2_DIV_13 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2_DIV_14 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2_DIV_15 (*)
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2_DIV_16 (*)
  *
  *         (*) value not defined in all devices
  * @param  PLLMul This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLL_MUL_2 (*)
  *         @arg @ref LL_RCC_PLL_MUL_3 (*)
  *         @arg @ref LL_RCC_PLL_MUL_4
  *         @arg @ref LL_RCC_PLL_MUL_5
  *         @arg @ref LL_RCC_PLL_MUL_6
  *         @arg @ref LL_RCC_PLL_MUL_7
  *         @arg @ref LL_RCC_PLL_MUL_8
  *         @arg @ref LL_RCC_PLL_MUL_9
  *         @arg @ref LL_RCC_PLL_MUL_6_5 (*)
  *         @arg @ref LL_RCC_PLL_MUL_10 (*)
  *         @arg @ref LL_RCC_PLL_MUL_11 (*)
  *         @arg @ref LL_RCC_PLL_MUL_12 (*)
  *         @arg @ref LL_RCC_PLL_MUL_13 (*)
  *         @arg @ref LL_RCC_PLL_MUL_14 (*)
  *         @arg @ref LL_RCC_PLL_MUL_15 (*)
  *         @arg @ref LL_RCC_PLL_MUL_16 (*)
  *
  *         (*) value not defined in all devices
  * @retval None
  */
__STATIC_INLINE void LL_RCC_PLL_ConfigDomain_SYS(uint32_t Source, uint32_t PLLMul)
{
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL,
             (Source & (RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE)) | PLLMul);
#if defined(RCC_CFGR2_PREDIV1)
#if defined(RCC_CFGR2_PREDIV1SRC)
  MODIFY_REG(RCC->CFGR2, (RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC),
             (Source & RCC_CFGR2_PREDIV1) | ((Source & (RCC_CFGR2_PREDIV1SRC << 4U)) >> 4U));
#else
  MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PREDIV1, (Source & RCC_CFGR2_PREDIV1));
#endif /*RCC_CFGR2_PREDIV1SRC*/
#endif /*RCC_CFGR2_PREDIV1*/
}

/**
  * @brief  Configure PLL clock source
  * @rmtoll CFGR      PLLSRC        LL_RCC_PLL_SetMainSource\n
  *         CFGR2     PREDIV1SRC    LL_RCC_PLL_SetMainSource
  * @param PLLSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLSOURCE_HSI_DIV_2
  *         @arg @ref LL_RCC_PLLSOURCE_HSE
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2 (*)
  * @retval None
  */
__STATIC_INLINE void LL_RCC_PLL_SetMainSource(uint32_t PLLSource)
{
#if defined(RCC_CFGR2_PREDIV1SRC)
  MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PREDIV1SRC, ((PLLSource & (RCC_CFGR2_PREDIV1SRC << 4U)) >> 4U));
#endif /* RCC_CFGR2_PREDIV1SRC */
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC, PLLSource);
}

/**
  * @brief  Get the oscillator used as PLL clock source.
  * @rmtoll CFGR         PLLSRC        LL_RCC_PLL_GetMainSource\n
  *         CFGR2        PREDIV1SRC    LL_RCC_PLL_GetMainSource
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_PLLSOURCE_HSI_DIV_2
  *         @arg @ref LL_RCC_PLLSOURCE_HSE
  *         @arg @ref LL_RCC_PLLSOURCE_PLL2 (*)
  *
  *         (*) value not defined in all devices
  */
__STATIC_INLINE uint32_t LL_RCC_PLL_GetMainSource(void)
{
#if defined(RCC_CFGR2_PREDIV1SRC)
  register uint32_t pllsrc = READ_BIT(RCC->CFGR, RCC_CFGR_PLLSRC);
  register uint32_t predivsrc = (uint32_t)(READ_BIT(RCC->CFGR2, RCC_CFGR2_PREDIV1SRC) << 4U);
  return (uint32_t)(pllsrc | predivsrc);
#else
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PLLSRC));
#endif /*RCC_CFGR2_PREDIV1SRC*/
}

/**
  * @brief  Get PLL multiplication Factor
  * @rmtoll CFGR         PLLMULL       LL_RCC_PLL_GetMultiplicator
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_PLL_MUL_2 (*)
  *         @arg @ref LL_RCC_PLL_MUL_3 (*)
  *         @arg @ref LL_RCC_PLL_MUL_4
  *         @arg @ref LL_RCC_PLL_MUL_5
  *         @arg @ref LL_RCC_PLL_MUL_6
  *         @arg @ref LL_RCC_PLL_MUL_7
  *         @arg @ref LL_RCC_PLL_MUL_8
  *         @arg @ref LL_RCC_PLL_MUL_9
  *         @arg @ref LL_RCC_PLL_MUL_6_5 (*)
  *         @arg @ref LL_RCC_PLL_MUL_10 (*)
  *         @arg @ref LL_RCC_PLL_MUL_11 (*)
  *         @arg @ref LL_RCC_PLL_MUL_12 (*)
  *         @arg @ref LL_RCC_PLL_MUL_13 (*)
  *         @arg @ref LL_RCC_PLL_MUL_14 (*)
  *         @arg @ref LL_RCC_PLL_MUL_15 (*)
  *         @arg @ref LL_RCC_PLL_MUL_16 (*)
  *
  *         (*) value not defined in all devices
  */
__STATIC_INLINE uint32_t LL_RCC_PLL_GetMultiplicator(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PLLMULL));
}

/**
  * @brief  Get PREDIV1 division factor for the main PLL
  * @note They can be written only when the PLL is disabled
  * @rmtoll CFGR2        PREDIV1       LL_RCC_PLL_GetPrediv\n
  *         CFGR2        PLLXTPRE      LL_RCC_PLL_GetPrediv
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_PREDIV_DIV_1
  *         @arg @ref LL_RCC_PREDIV_DIV_2
  *         @arg @ref LL_RCC_PREDIV_DIV_3 (*)
  *         @arg @ref LL_RCC_PREDIV_DIV_4 (*)
  *         @arg @ref LL_RCC_PREDIV_DIV_5 (*)
  *         @arg @ref LL_RCC_PREDIV_DIV_6 (*)
  *         @arg @ref LL_RCC_PREDIV_DIV_7 (*)
  *         @arg @ref LL_RCC_PREDIV_DIV_8 (*)
  *         @arg @ref LL_RCC_PREDIV_DIV_9 (*)
  *         @arg @ref LL_RCC_PREDIV_DIV_10 (*)
  *         @arg @ref LL_RCC_PREDIV_DIV_11 (*)
  *         @arg @ref LL_RCC_PREDIV_DIV_12 (*)
  *         @arg @ref LL_RCC_PREDIV_DIV_13 (*)
  *         @arg @ref LL_RCC_PREDIV_DIV_14 (*)
  *         @arg @ref LL_RCC_PREDIV_DIV_15 (*)
  *         @arg @ref LL_RCC_PREDIV_DIV_16 (*)
  *
  *         (*) value not defined in all devices
  */
__STATIC_INLINE uint32_t LL_RCC_PLL_GetPrediv(void)
{
#if defined(RCC_CFGR2_PREDIV1)
  return (uint32_t)(READ_BIT(RCC->CFGR2, RCC_CFGR2_PREDIV1));
#else
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PLLXTPRE));
#endif /*RCC_CFGR2_PREDIV1*/
}

/**
  * @}
  */

#if defined(RCC_PLLI2S_SUPPORT)
/** @defgroup RCC_LL_EF_PLLI2S PLLI2S
  * @{
  */

/**
  * @brief  Enable PLLI2S
  * @rmtoll CR           PLL3ON        LL_RCC_PLLI2S_Enable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_PLLI2S_Enable(void)
{
  SET_BIT(RCC->CR, RCC_CR_PLL3ON);
}

/**
  * @brief  Disable PLLI2S
  * @rmtoll CR           PLL3ON        LL_RCC_PLLI2S_Disable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_PLLI2S_Disable(void)
{
  CLEAR_BIT(RCC->CR, RCC_CR_PLL3ON);
}

/**
  * @brief  Check if PLLI2S Ready
  * @rmtoll CR           PLL3RDY       LL_RCC_PLLI2S_IsReady
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_PLLI2S_IsReady(void)
{
  return (READ_BIT(RCC->CR, RCC_CR_PLL3RDY) == (RCC_CR_PLL3RDY));
}

/**
  * @brief  Configure PLLI2S used for I2S Domain
  * @rmtoll CFGR2        PREDIV2       LL_RCC_PLL_ConfigDomain_PLLI2S\n
  *         CFGR2        PLL3MUL       LL_RCC_PLL_ConfigDomain_PLLI2S
  * @param  Divider This parameter can be one of the following values:
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_1
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_2
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_3
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_4
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_5
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_6
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_7
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_8
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_9
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_10
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_11
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_12
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_13
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_14
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_15
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_16
  * @param  Multiplicator This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLI2S_MUL_8
  *         @arg @ref LL_RCC_PLLI2S_MUL_9
  *         @arg @ref LL_RCC_PLLI2S_MUL_10
  *         @arg @ref LL_RCC_PLLI2S_MUL_11
  *         @arg @ref LL_RCC_PLLI2S_MUL_12
  *         @arg @ref LL_RCC_PLLI2S_MUL_13
  *         @arg @ref LL_RCC_PLLI2S_MUL_14
  *         @arg @ref LL_RCC_PLLI2S_MUL_16
  *         @arg @ref LL_RCC_PLLI2S_MUL_20
  * @retval None
  */
__STATIC_INLINE void LL_RCC_PLL_ConfigDomain_PLLI2S(uint32_t Divider, uint32_t Multiplicator)
{
  MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL3MUL, Divider | Multiplicator);
}

/**
  * @brief  Get PLLI2S Multiplication Factor
  * @rmtoll CFGR2        PLL3MUL       LL_RCC_PLLI2S_GetMultiplicator
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_PLLI2S_MUL_8
  *         @arg @ref LL_RCC_PLLI2S_MUL_9
  *         @arg @ref LL_RCC_PLLI2S_MUL_10
  *         @arg @ref LL_RCC_PLLI2S_MUL_11
  *         @arg @ref LL_RCC_PLLI2S_MUL_12
  *         @arg @ref LL_RCC_PLLI2S_MUL_13
  *         @arg @ref LL_RCC_PLLI2S_MUL_14
  *         @arg @ref LL_RCC_PLLI2S_MUL_16
  *         @arg @ref LL_RCC_PLLI2S_MUL_20
  */
__STATIC_INLINE uint32_t LL_RCC_PLLI2S_GetMultiplicator(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR2, RCC_CFGR2_PLL3MUL));
}

/**
  * @}
  */
#endif /* RCC_PLLI2S_SUPPORT */

#if defined(RCC_PLL2_SUPPORT)
/** @defgroup RCC_LL_EF_PLL2 PLL2
  * @{
  */

/**
  * @brief  Enable PLL2
  * @rmtoll CR           PLL2ON        LL_RCC_PLL2_Enable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_PLL2_Enable(void)
{
  SET_BIT(RCC->CR, RCC_CR_PLL2ON);
}

/**
  * @brief  Disable PLL2
  * @rmtoll CR           PLL2ON        LL_RCC_PLL2_Disable
  * @retval None
  */
__STATIC_INLINE void LL_RCC_PLL2_Disable(void)
{
  CLEAR_BIT(RCC->CR, RCC_CR_PLL2ON);
}

/**
  * @brief  Check if PLL2 Ready
  * @rmtoll CR           PLL2RDY       LL_RCC_PLL2_IsReady
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_PLL2_IsReady(void)
{
  return (READ_BIT(RCC->CR, RCC_CR_PLL2RDY) == (RCC_CR_PLL2RDY));
}

/**
  * @brief  Configure PLL2 used for PLL2 Domain
  * @rmtoll CFGR2        PREDIV2       LL_RCC_PLL_ConfigDomain_PLL2\n
  *         CFGR2        PLL2MUL       LL_RCC_PLL_ConfigDomain_PLL2
  * @param  Divider This parameter can be one of the following values:
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_1
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_2
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_3
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_4
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_5
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_6
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_7
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_8
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_9
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_10
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_11
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_12
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_13
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_14
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_15
  *         @arg @ref LL_RCC_HSE_PREDIV2_DIV_16
  * @param  Multiplicator This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLL2_MUL_8
  *         @arg @ref LL_RCC_PLL2_MUL_9
  *         @arg @ref LL_RCC_PLL2_MUL_10
  *         @arg @ref LL_RCC_PLL2_MUL_11
  *         @arg @ref LL_RCC_PLL2_MUL_12
  *         @arg @ref LL_RCC_PLL2_MUL_13
  *         @arg @ref LL_RCC_PLL2_MUL_14
  *         @arg @ref LL_RCC_PLL2_MUL_16
  *         @arg @ref LL_RCC_PLL2_MUL_20
  * @retval None
  */
__STATIC_INLINE void LL_RCC_PLL_ConfigDomain_PLL2(uint32_t Divider, uint32_t Multiplicator)
{
  MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL, Divider | Multiplicator);
}

/**
  * @brief  Get PLL2 Multiplication Factor
  * @rmtoll CFGR2        PLL2MUL       LL_RCC_PLL2_GetMultiplicator
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_PLL2_MUL_8
  *         @arg @ref LL_RCC_PLL2_MUL_9
  *         @arg @ref LL_RCC_PLL2_MUL_10
  *         @arg @ref LL_RCC_PLL2_MUL_11
  *         @arg @ref LL_RCC_PLL2_MUL_12
  *         @arg @ref LL_RCC_PLL2_MUL_13
  *         @arg @ref LL_RCC_PLL2_MUL_14
  *         @arg @ref LL_RCC_PLL2_MUL_16
  *         @arg @ref LL_RCC_PLL2_MUL_20
  */
__STATIC_INLINE uint32_t LL_RCC_PLL2_GetMultiplicator(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR2, RCC_CFGR2_PLL2MUL));
}

/**
  * @}
  */
#endif /* RCC_PLL2_SUPPORT */

/** @defgroup RCC_LL_EF_FLAG_Management FLAG Management
  * @{
  */

/**
  * @brief  Clear LSI ready interrupt flag
  * @rmtoll CIR         LSIRDYC       LL_RCC_ClearFlag_LSIRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ClearFlag_LSIRDY(void)
{
  SET_BIT(RCC->CIR, RCC_CIR_LSIRDYC);
}

/**
  * @brief  Clear LSE ready interrupt flag
  * @rmtoll CIR         LSERDYC       LL_RCC_ClearFlag_LSERDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ClearFlag_LSERDY(void)
{
  SET_BIT(RCC->CIR, RCC_CIR_LSERDYC);
}

/**
  * @brief  Clear HSI ready interrupt flag
  * @rmtoll CIR         HSIRDYC       LL_RCC_ClearFlag_HSIRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ClearFlag_HSIRDY(void)
{
  SET_BIT(RCC->CIR, RCC_CIR_HSIRDYC);
}

/**
  * @brief  Clear HSE ready interrupt flag
  * @rmtoll CIR         HSERDYC       LL_RCC_ClearFlag_HSERDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ClearFlag_HSERDY(void)
{
  SET_BIT(RCC->CIR, RCC_CIR_HSERDYC);
}

/**
  * @brief  Clear PLL ready interrupt flag
  * @rmtoll CIR         PLLRDYC       LL_RCC_ClearFlag_PLLRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ClearFlag_PLLRDY(void)
{
  SET_BIT(RCC->CIR, RCC_CIR_PLLRDYC);
}

#if defined(RCC_PLLI2S_SUPPORT)
/**
  * @brief  Clear PLLI2S ready interrupt flag
  * @rmtoll CIR          PLL3RDYC      LL_RCC_ClearFlag_PLLI2SRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ClearFlag_PLLI2SRDY(void)
{
  SET_BIT(RCC->CIR, RCC_CIR_PLL3RDYC);
}
#endif /* RCC_PLLI2S_SUPPORT */

#if defined(RCC_PLL2_SUPPORT)
/**
  * @brief  Clear PLL2 ready interrupt flag
  * @rmtoll CIR          PLL2RDYC      LL_RCC_ClearFlag_PLL2RDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ClearFlag_PLL2RDY(void)
{
  SET_BIT(RCC->CIR, RCC_CIR_PLL2RDYC);
}
#endif /* RCC_PLL2_SUPPORT */

/**
  * @brief  Clear Clock security system interrupt flag
  * @rmtoll CIR         CSSC          LL_RCC_ClearFlag_HSECSS
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ClearFlag_HSECSS(void)
{
  SET_BIT(RCC->CIR, RCC_CIR_CSSC);
}

/**
  * @brief  Check if LSI ready interrupt occurred or not
  * @rmtoll CIR         LSIRDYF       LL_RCC_IsActiveFlag_LSIRDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_LSIRDY(void)
{
  return (READ_BIT(RCC->CIR, RCC_CIR_LSIRDYF) == (RCC_CIR_LSIRDYF));
}

/**
  * @brief  Check if LSE ready interrupt occurred or not
  * @rmtoll CIR         LSERDYF       LL_RCC_IsActiveFlag_LSERDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_LSERDY(void)
{
  return (READ_BIT(RCC->CIR, RCC_CIR_LSERDYF) == (RCC_CIR_LSERDYF));
}

/**
  * @brief  Check if HSI ready interrupt occurred or not
  * @rmtoll CIR         HSIRDYF       LL_RCC_IsActiveFlag_HSIRDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_HSIRDY(void)
{
  return (READ_BIT(RCC->CIR, RCC_CIR_HSIRDYF) == (RCC_CIR_HSIRDYF));
}

/**
  * @brief  Check if HSE ready interrupt occurred or not
  * @rmtoll CIR         HSERDYF       LL_RCC_IsActiveFlag_HSERDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_HSERDY(void)
{
  return (READ_BIT(RCC->CIR, RCC_CIR_HSERDYF) == (RCC_CIR_HSERDYF));
}

/**
  * @brief  Check if PLL ready interrupt occurred or not
  * @rmtoll CIR         PLLRDYF       LL_RCC_IsActiveFlag_PLLRDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_PLLRDY(void)
{
  return (READ_BIT(RCC->CIR, RCC_CIR_PLLRDYF) == (RCC_CIR_PLLRDYF));
}

#if defined(RCC_PLLI2S_SUPPORT)
/**
  * @brief  Check if PLLI2S ready interrupt occurred or not
  * @rmtoll CIR          PLL3RDYF      LL_RCC_IsActiveFlag_PLLI2SRDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_PLLI2SRDY(void)
{
  return (READ_BIT(RCC->CIR, RCC_CIR_PLL3RDYF) == (RCC_CIR_PLL3RDYF));
}
#endif /* RCC_PLLI2S_SUPPORT */

#if defined(RCC_PLL2_SUPPORT)
/**
  * @brief  Check if PLL2 ready interrupt occurred or not
  * @rmtoll CIR          PLL2RDYF      LL_RCC_IsActiveFlag_PLL2RDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_PLL2RDY(void)
{
  return (READ_BIT(RCC->CIR, RCC_CIR_PLL2RDYF) == (RCC_CIR_PLL2RDYF));
}
#endif /* RCC_PLL2_SUPPORT */

/**
  * @brief  Check if Clock security system interrupt occurred or not
  * @rmtoll CIR         CSSF          LL_RCC_IsActiveFlag_HSECSS
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_HSECSS(void)
{
  return (READ_BIT(RCC->CIR, RCC_CIR_CSSF) == (RCC_CIR_CSSF));
}

/**
  * @brief  Check if RCC flag Independent Watchdog reset is set or not.
  * @rmtoll CSR          IWDGRSTF      LL_RCC_IsActiveFlag_IWDGRST
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_IWDGRST(void)
{
  return (READ_BIT(RCC->CSR, RCC_CSR_IWDGRSTF) == (RCC_CSR_IWDGRSTF));
}

/**
  * @brief  Check if RCC flag Low Power reset is set or not.
  * @rmtoll CSR          LPWRRSTF      LL_RCC_IsActiveFlag_LPWRRST
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_LPWRRST(void)
{
  return (READ_BIT(RCC->CSR, RCC_CSR_LPWRRSTF) == (RCC_CSR_LPWRRSTF));
}

/**
  * @brief  Check if RCC flag Pin reset is set or not.
  * @rmtoll CSR          PINRSTF       LL_RCC_IsActiveFlag_PINRST
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_PINRST(void)
{
  return (READ_BIT(RCC->CSR, RCC_CSR_PINRSTF) == (RCC_CSR_PINRSTF));
}

/**
  * @brief  Check if RCC flag POR/PDR reset is set or not.
  * @rmtoll CSR          PORRSTF       LL_RCC_IsActiveFlag_PORRST
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_PORRST(void)
{
  return (READ_BIT(RCC->CSR, RCC_CSR_PORRSTF) == (RCC_CSR_PORRSTF));
}

/**
  * @brief  Check if RCC flag Software reset is set or not.
  * @rmtoll CSR          SFTRSTF       LL_RCC_IsActiveFlag_SFTRST
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_SFTRST(void)
{
  return (READ_BIT(RCC->CSR, RCC_CSR_SFTRSTF) == (RCC_CSR_SFTRSTF));
}

/**
  * @brief  Check if RCC flag Window Watchdog reset is set or not.
  * @rmtoll CSR          WWDGRSTF      LL_RCC_IsActiveFlag_WWDGRST
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsActiveFlag_WWDGRST(void)
{
  return (READ_BIT(RCC->CSR, RCC_CSR_WWDGRSTF) == (RCC_CSR_WWDGRSTF));
}

/**
  * @brief  Set RMVF bit to clear the reset flags.
  * @rmtoll CSR          RMVF          LL_RCC_ClearResetFlags
  * @retval None
  */
__STATIC_INLINE void LL_RCC_ClearResetFlags(void)
{
  SET_BIT(RCC->CSR, RCC_CSR_RMVF);
}

/**
  * @}
  */

/** @defgroup RCC_LL_EF_IT_Management IT Management
  * @{
  */

/**
  * @brief  Enable LSI ready interrupt
  * @rmtoll CIR         LSIRDYIE      LL_RCC_EnableIT_LSIRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_EnableIT_LSIRDY(void)
{
  SET_BIT(RCC->CIR, RCC_CIR_LSIRDYIE);
}

/**
  * @brief  Enable LSE ready interrupt
  * @rmtoll CIR         LSERDYIE      LL_RCC_EnableIT_LSERDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_EnableIT_LSERDY(void)
{
  SET_BIT(RCC->CIR, RCC_CIR_LSERDYIE);
}

/**
  * @brief  Enable HSI ready interrupt
  * @rmtoll CIR         HSIRDYIE      LL_RCC_EnableIT_HSIRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_EnableIT_HSIRDY(void)
{
  SET_BIT(RCC->CIR, RCC_CIR_HSIRDYIE);
}

/**
  * @brief  Enable HSE ready interrupt
  * @rmtoll CIR         HSERDYIE      LL_RCC_EnableIT_HSERDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_EnableIT_HSERDY(void)
{
  SET_BIT(RCC->CIR, RCC_CIR_HSERDYIE);
}

/**
  * @brief  Enable PLL ready interrupt
  * @rmtoll CIR         PLLRDYIE      LL_RCC_EnableIT_PLLRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_EnableIT_PLLRDY(void)
{
  SET_BIT(RCC->CIR, RCC_CIR_PLLRDYIE);
}

#if defined(RCC_PLLI2S_SUPPORT)
/**
  * @brief  Enable PLLI2S ready interrupt
  * @rmtoll CIR          PLL3RDYIE     LL_RCC_EnableIT_PLLI2SRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_EnableIT_PLLI2SRDY(void)
{
  SET_BIT(RCC->CIR, RCC_CIR_PLL3RDYIE);
}
#endif /* RCC_PLLI2S_SUPPORT */

#if defined(RCC_PLL2_SUPPORT)
/**
  * @brief  Enable PLL2 ready interrupt
  * @rmtoll CIR          PLL2RDYIE     LL_RCC_EnableIT_PLL2RDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_EnableIT_PLL2RDY(void)
{
  SET_BIT(RCC->CIR, RCC_CIR_PLL2RDYIE);
}
#endif /* RCC_PLL2_SUPPORT */

/**
  * @brief  Disable LSI ready interrupt
  * @rmtoll CIR         LSIRDYIE      LL_RCC_DisableIT_LSIRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_DisableIT_LSIRDY(void)
{
  CLEAR_BIT(RCC->CIR, RCC_CIR_LSIRDYIE);
}

/**
  * @brief  Disable LSE ready interrupt
  * @rmtoll CIR         LSERDYIE      LL_RCC_DisableIT_LSERDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_DisableIT_LSERDY(void)
{
  CLEAR_BIT(RCC->CIR, RCC_CIR_LSERDYIE);
}

/**
  * @brief  Disable HSI ready interrupt
  * @rmtoll CIR         HSIRDYIE      LL_RCC_DisableIT_HSIRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_DisableIT_HSIRDY(void)
{
  CLEAR_BIT(RCC->CIR, RCC_CIR_HSIRDYIE);
}

/**
  * @brief  Disable HSE ready interrupt
  * @rmtoll CIR         HSERDYIE      LL_RCC_DisableIT_HSERDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_DisableIT_HSERDY(void)
{
  CLEAR_BIT(RCC->CIR, RCC_CIR_HSERDYIE);
}

/**
  * @brief  Disable PLL ready interrupt
  * @rmtoll CIR         PLLRDYIE      LL_RCC_DisableIT_PLLRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_DisableIT_PLLRDY(void)
{
  CLEAR_BIT(RCC->CIR, RCC_CIR_PLLRDYIE);
}

#if defined(RCC_PLLI2S_SUPPORT)
/**
  * @brief  Disable PLLI2S ready interrupt
  * @rmtoll CIR          PLL3RDYIE     LL_RCC_DisableIT_PLLI2SRDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_DisableIT_PLLI2SRDY(void)
{
  CLEAR_BIT(RCC->CIR, RCC_CIR_PLL3RDYIE);
}
#endif /* RCC_PLLI2S_SUPPORT */

#if defined(RCC_PLL2_SUPPORT)
/**
  * @brief  Disable PLL2 ready interrupt
  * @rmtoll CIR          PLL2RDYIE     LL_RCC_DisableIT_PLL2RDY
  * @retval None
  */
__STATIC_INLINE void LL_RCC_DisableIT_PLL2RDY(void)
{
  CLEAR_BIT(RCC->CIR, RCC_CIR_PLL2RDYIE);
}
#endif /* RCC_PLL2_SUPPORT */

/**
  * @brief  Checks if LSI ready interrupt source is enabled or disabled.
  * @rmtoll CIR         LSIRDYIE      LL_RCC_IsEnabledIT_LSIRDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsEnabledIT_LSIRDY(void)
{
  return (READ_BIT(RCC->CIR, RCC_CIR_LSIRDYIE) == (RCC_CIR_LSIRDYIE));
}

/**
  * @brief  Checks if LSE ready interrupt source is enabled or disabled.
  * @rmtoll CIR         LSERDYIE      LL_RCC_IsEnabledIT_LSERDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsEnabledIT_LSERDY(void)
{
  return (READ_BIT(RCC->CIR, RCC_CIR_LSERDYIE) == (RCC_CIR_LSERDYIE));
}

/**
  * @brief  Checks if HSI ready interrupt source is enabled or disabled.
  * @rmtoll CIR         HSIRDYIE      LL_RCC_IsEnabledIT_HSIRDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsEnabledIT_HSIRDY(void)
{
  return (READ_BIT(RCC->CIR, RCC_CIR_HSIRDYIE) == (RCC_CIR_HSIRDYIE));
}

/**
  * @brief  Checks if HSE ready interrupt source is enabled or disabled.
  * @rmtoll CIR         HSERDYIE      LL_RCC_IsEnabledIT_HSERDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsEnabledIT_HSERDY(void)
{
  return (READ_BIT(RCC->CIR, RCC_CIR_HSERDYIE) == (RCC_CIR_HSERDYIE));
}

/**
  * @brief  Checks if PLL ready interrupt source is enabled or disabled.
  * @rmtoll CIR         PLLRDYIE      LL_RCC_IsEnabledIT_PLLRDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsEnabledIT_PLLRDY(void)
{
  return (READ_BIT(RCC->CIR, RCC_CIR_PLLRDYIE) == (RCC_CIR_PLLRDYIE));
}

#if defined(RCC_PLLI2S_SUPPORT)
/**
  * @brief  Checks if PLLI2S ready interrupt source is enabled or disabled.
  * @rmtoll CIR          PLL3RDYIE     LL_RCC_IsEnabledIT_PLLI2SRDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsEnabledIT_PLLI2SRDY(void)
{
  return (READ_BIT(RCC->CIR, RCC_CIR_PLL3RDYIE) == (RCC_CIR_PLL3RDYIE));
}
#endif /* RCC_PLLI2S_SUPPORT */

#if defined(RCC_PLL2_SUPPORT)
/**
  * @brief  Checks if PLL2 ready interrupt source is enabled or disabled.
  * @rmtoll CIR          PLL2RDYIE     LL_RCC_IsEnabledIT_PLL2RDY
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_RCC_IsEnabledIT_PLL2RDY(void)
{
  return (READ_BIT(RCC->CIR, RCC_CIR_PLL2RDYIE) == (RCC_CIR_PLL2RDYIE));
}
#endif /* RCC_PLL2_SUPPORT */

/**
  * @}
  */

#if defined(USE_FULL_LL_DRIVER)
/** @defgroup RCC_LL_EF_Init De-initialization function
  * @{
  */
ErrorStatus LL_RCC_DeInit(void);
/**
  * @}
  */

/** @defgroup RCC_LL_EF_Get_Freq Get system and peripherals clocks frequency functions
  * @{
  */
void        LL_RCC_GetSystemClocksFreq(LL_RCC_ClocksTypeDef *RCC_Clocks);
#if defined(RCC_CFGR2_I2S2SRC)
uint32_t    LL_RCC_GetI2SClockFreq(uint32_t I2SxSource);
#endif /* RCC_CFGR2_I2S2SRC */
#if defined(USB_OTG_FS) || defined(USB)
uint32_t    LL_RCC_GetUSBClockFreq(uint32_t USBxSource);
#endif /* USB_OTG_FS || USB */
uint32_t    LL_RCC_GetADCClockFreq(uint32_t ADCxSource);
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

#endif /* RCC */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_LL_RCC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
