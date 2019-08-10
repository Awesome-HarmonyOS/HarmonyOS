/**
  ******************************************************************************
  * @file    stm32f1xx_ll_rcc.c
  * @author  MCD Application Team
  * @brief   RCC LL module driver.
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
#if defined(USE_FULL_LL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_rcc.h"
#ifdef  USE_FULL_ASSERT
  #include "stm32_assert.h"
#else
  #define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */
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
/** @addtogroup RCC_LL_Private_Macros
  * @{
  */
#if defined(RCC_PLLI2S_SUPPORT)
#define IS_LL_RCC_I2S_CLKSOURCE(__VALUE__)     (((__VALUE__) == LL_RCC_I2S2_CLKSOURCE) \
                                             || ((__VALUE__) == LL_RCC_I2S3_CLKSOURCE))
#endif /* RCC_PLLI2S_SUPPORT */

#if defined(USB) || defined(USB_OTG_FS)
#define IS_LL_RCC_USB_CLKSOURCE(__VALUE__)    (((__VALUE__) == LL_RCC_USB_CLKSOURCE))
#endif /* USB */

#define IS_LL_RCC_ADC_CLKSOURCE(__VALUE__)    (((__VALUE__) == LL_RCC_ADC_CLKSOURCE))
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup RCC_LL_Private_Functions RCC Private functions
  * @{
  */
uint32_t RCC_GetSystemClockFreq(void);
uint32_t RCC_GetHCLKClockFreq(uint32_t SYSCLK_Frequency);
uint32_t RCC_GetPCLK1ClockFreq(uint32_t HCLK_Frequency);
uint32_t RCC_GetPCLK2ClockFreq(uint32_t HCLK_Frequency);
uint32_t RCC_PLL_GetFreqDomain_SYS(void);
#if defined(RCC_PLLI2S_SUPPORT)
uint32_t RCC_PLLI2S_GetFreqDomain_I2S(void);
#endif /* RCC_PLLI2S_SUPPORT */
#if defined(RCC_PLL2_SUPPORT)
uint32_t RCC_PLL2_GetFreqClockFreq(void);
#endif /* RCC_PLL2_SUPPORT */
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup RCC_LL_Exported_Functions
  * @{
  */

/** @addtogroup RCC_LL_EF_Init
  * @{
  */

/**
  * @brief  Reset the RCC clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  *         - HSI ON and used as system clock source
  *         - HSE PLL, PLL2 & PLL3 are OFF 
  *         - AHB, APB1 and APB2 prescaler set to 1.
  *         - CSS, MCO OFF
  *         - All interrupts disabled
  * @note   This function doesn't modify the configuration of the
  *         - Peripheral clocks
  *         - LSI, LSE and RTC clocks
  * @retval An ErrorStatus enumeration value:
  *         - SUCCESS: RCC registers are de-initialized
  *         - ERROR: not applicable
  */
ErrorStatus LL_RCC_DeInit(void)
{
  /* Set HSION bit */
  LL_RCC_HSI_Enable();

  /* Wait for HSI READY bit */
  while(LL_RCC_HSI_IsReady() != 1U)
  {}

  /* Configure HSI as system clock source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

  /* Wait till clock switch is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {}

  /* Reset PLLON bit */
  CLEAR_BIT(RCC->CR, RCC_CR_PLLON);

  /* Wait for PLL READY bit to be reset */
  while(LL_RCC_PLL_IsReady() != 0U)
  {}

  /* Reset CFGR register */
  LL_RCC_WriteReg(CFGR, 0x00000000U);

  /* Reset HSEON, HSEBYP & CSSON bits */
  CLEAR_BIT(RCC->CR, (RCC_CR_CSSON | RCC_CR_HSEON | RCC_CR_HSEBYP));

#if defined(RCC_CR_PLL2ON)
  /* Reset PLL2ON bit */
  CLEAR_BIT(RCC->CR, RCC_CR_PLL2ON);
#endif /* RCC_CR_PLL2ON */

#if defined(RCC_CR_PLL3ON)
  /* Reset PLL3ON bit */
  CLEAR_BIT(RCC->CR, RCC_CR_PLL3ON);
#endif /* RCC_CR_PLL3ON */

  /* Set HSITRIM bits to the reset value */
  LL_RCC_HSI_SetCalibTrimming(0x10U);

#if defined(RCC_CFGR2_PREDIV1)
  /* Reset CFGR2 register */
  LL_RCC_WriteReg(CFGR2, 0x00000000U);
#endif /* RCC_CFGR2_PREDIV1 */

  /* Disable all interrupts */
  LL_RCC_WriteReg(CIR, 0x00000000U);

  /* Clear reset flags */
  LL_RCC_ClearResetFlags();

  return SUCCESS;
}

/**
  * @}
  */

/** @addtogroup RCC_LL_EF_Get_Freq
  * @brief  Return the frequencies of different on chip clocks;  System, AHB, APB1 and APB2 buses clocks
  *         and different peripheral clocks available on the device.
  * @note   If SYSCLK source is HSI, function returns values based on HSI_VALUE(**)
  * @note   If SYSCLK source is HSE, function returns values based on HSE_VALUE(***)
  * @note   If SYSCLK source is PLL, function returns values based on 
  *         HSI_VALUE(**) or HSE_VALUE(***) multiplied/divided by the PLL factors.
  * @note   (**) HSI_VALUE is a defined constant but the real value may vary 
  *              depending on the variations in voltage and temperature.
  * @note   (***) HSE_VALUE is a defined constant, user has to ensure that
  *               HSE_VALUE is same as the real frequency of the crystal used.
  *               Otherwise, this function may have wrong result.
  * @note   The result of this function could be incorrect when using fractional
  *         value for HSE crystal.
  * @note   This function can be used by the user application to compute the
  *         baud-rate for the communication peripherals or configure other parameters.
  * @{
  */

/**
  * @brief  Return the frequencies of different on chip clocks;  System, AHB, APB1 and APB2 buses clocks
  * @note   Each time SYSCLK, HCLK, PCLK1 and/or PCLK2 clock changes, this function
  *         must be called to update structure fields. Otherwise, any
  *         configuration based on this function will be incorrect.
  * @param  RCC_Clocks pointer to a @ref LL_RCC_ClocksTypeDef structure which will hold the clocks frequencies
  * @retval None
  */
void LL_RCC_GetSystemClocksFreq(LL_RCC_ClocksTypeDef *RCC_Clocks)
{
  /* Get SYSCLK frequency */
  RCC_Clocks->SYSCLK_Frequency = RCC_GetSystemClockFreq();

  /* HCLK clock frequency */
  RCC_Clocks->HCLK_Frequency   = RCC_GetHCLKClockFreq(RCC_Clocks->SYSCLK_Frequency);

  /* PCLK1 clock frequency */
  RCC_Clocks->PCLK1_Frequency  = RCC_GetPCLK1ClockFreq(RCC_Clocks->HCLK_Frequency);

  /* PCLK2 clock frequency */
  RCC_Clocks->PCLK2_Frequency  = RCC_GetPCLK2ClockFreq(RCC_Clocks->HCLK_Frequency);
}

#if defined(RCC_CFGR2_I2S2SRC)
/**
  * @brief  Return I2Sx clock frequency
  * @param  I2SxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_I2S2_CLKSOURCE
  *         @arg @ref LL_RCC_I2S3_CLKSOURCE
  * @retval I2S clock frequency (in Hz)
  */
uint32_t LL_RCC_GetI2SClockFreq(uint32_t I2SxSource)
{
  uint32_t i2s_frequency = LL_RCC_PERIPH_FREQUENCY_NO;

  /* Check parameter */
  assert_param(IS_LL_RCC_I2S_CLKSOURCE(I2SxSource));

  /* I2S1CLK clock frequency */
  switch (LL_RCC_GetI2SClockSource(I2SxSource))
  {
    case LL_RCC_I2S2_CLKSOURCE_SYSCLK:        /*!< System clock selected as I2S clock source */
    case LL_RCC_I2S3_CLKSOURCE_SYSCLK:
      i2s_frequency = RCC_GetSystemClockFreq();
      break;

    case LL_RCC_I2S2_CLKSOURCE_PLLI2S_VCO:    /*!< PLLI2S oscillator clock selected as I2S clock source */
    case LL_RCC_I2S3_CLKSOURCE_PLLI2S_VCO:
    default:
      i2s_frequency = RCC_PLLI2S_GetFreqDomain_I2S() * 2U;
      break;
  }

  return i2s_frequency;
}
#endif /* RCC_CFGR2_I2S2SRC */

#if defined(USB) || defined(USB_OTG_FS)
/**
  * @brief  Return USBx clock frequency
  * @param  USBxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_USB_CLKSOURCE
  * @retval USB clock frequency (in Hz)
  *         @arg @ref LL_RCC_PERIPH_FREQUENCY_NO indicates that oscillator (HSI), HSE or PLL is not ready
  */
uint32_t LL_RCC_GetUSBClockFreq(uint32_t USBxSource)
{
  uint32_t usb_frequency = LL_RCC_PERIPH_FREQUENCY_NO;

  /* Check parameter */
  assert_param(IS_LL_RCC_USB_CLKSOURCE(USBxSource));

  /* USBCLK clock frequency */
  switch (LL_RCC_GetUSBClockSource(USBxSource))
  {
#if defined(RCC_CFGR_USBPRE)  
    case LL_RCC_USB_CLKSOURCE_PLL:        /* PLL clock used as USB clock source */
      if (LL_RCC_PLL_IsReady())
      {
        usb_frequency = RCC_PLL_GetFreqDomain_SYS();
      }
      break;

    case LL_RCC_USB_CLKSOURCE_PLL_DIV_1_5:        /* PLL clock divided by 1.5 used as USB clock source */
    default:
      if (LL_RCC_PLL_IsReady())
      {
        usb_frequency = (RCC_PLL_GetFreqDomain_SYS() * 3U) / 2U;
      }
      break;
#endif /* RCC_CFGR_USBPRE */
#if defined(RCC_CFGR_OTGFSPRE)
    /* USBCLK = PLLVCO/2 
              = (2 x PLLCLK) / 2 
              = PLLCLK */
    case LL_RCC_USB_CLKSOURCE_PLL_DIV_2:        /* PLL clock used as USB clock source */
      if (LL_RCC_PLL_IsReady())
      {
        usb_frequency = RCC_PLL_GetFreqDomain_SYS();
      }
      break;

    /* USBCLK = PLLVCO/3 
              = (2 x PLLCLK) / 3 */
    case LL_RCC_USB_CLKSOURCE_PLL_DIV_3:        /* PLL clock divided by 3 used as USB clock source */
    default:
      if (LL_RCC_PLL_IsReady())
      {
        usb_frequency = (RCC_PLL_GetFreqDomain_SYS() * 2U) / 3U;
      }
      break;
#endif /* RCC_CFGR_OTGFSPRE */
  }

  return usb_frequency;
}
#endif /* USB */

/**
  * @brief  Return ADCx clock frequency
  * @param  ADCxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_ADC_CLKSOURCE
  * @retval ADC clock frequency (in Hz)
  */
uint32_t LL_RCC_GetADCClockFreq(uint32_t ADCxSource)
{
  uint32_t adc_prescaler = 0U;
  uint32_t adc_frequency = 0U;

  /* Check parameter */
  assert_param(IS_LL_RCC_ADC_CLKSOURCE(ADCxSource));

  /* Get ADC prescaler */
  adc_prescaler = LL_RCC_GetADCClockSource(ADCxSource);

  /* ADC frequency = PCLK2 frequency / ADC prescaler (2, 4, 6 or 8) */
  adc_frequency = RCC_GetPCLK2ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()))
                  / (((adc_prescaler >> POSITION_VAL(ADCxSource)) + 1U) * 2U);

  return adc_frequency;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup RCC_LL_Private_Functions
  * @{
  */

/**
  * @brief  Return SYSTEM clock frequency
  * @retval SYSTEM clock frequency (in Hz)
  */
uint32_t RCC_GetSystemClockFreq(void)
{
  uint32_t frequency = 0U;

  /* Get SYSCLK source -------------------------------------------------------*/
  switch (LL_RCC_GetSysClkSource())
  {
    case LL_RCC_SYS_CLKSOURCE_STATUS_HSI:  /* HSI used as system clock  source */
      frequency = HSI_VALUE;
      break;

    case LL_RCC_SYS_CLKSOURCE_STATUS_HSE:  /* HSE used as system clock  source */
      frequency = HSE_VALUE;
      break;

    case LL_RCC_SYS_CLKSOURCE_STATUS_PLL:  /* PLL used as system clock  source */
      frequency = RCC_PLL_GetFreqDomain_SYS();
      break;

    default:
      frequency = HSI_VALUE;
      break;
  }

  return frequency;
}

/**
  * @brief  Return HCLK clock frequency
  * @param  SYSCLK_Frequency SYSCLK clock frequency
  * @retval HCLK clock frequency (in Hz)
  */
uint32_t RCC_GetHCLKClockFreq(uint32_t SYSCLK_Frequency)
{
  /* HCLK clock frequency */
  return __LL_RCC_CALC_HCLK_FREQ(SYSCLK_Frequency, LL_RCC_GetAHBPrescaler());
}

/**
  * @brief  Return PCLK1 clock frequency
  * @param  HCLK_Frequency HCLK clock frequency
  * @retval PCLK1 clock frequency (in Hz)
  */
uint32_t RCC_GetPCLK1ClockFreq(uint32_t HCLK_Frequency)
{
  /* PCLK1 clock frequency */
  return __LL_RCC_CALC_PCLK1_FREQ(HCLK_Frequency, LL_RCC_GetAPB1Prescaler());
}

/**
  * @brief  Return PCLK2 clock frequency
  * @param  HCLK_Frequency HCLK clock frequency
  * @retval PCLK2 clock frequency (in Hz)
  */
uint32_t RCC_GetPCLK2ClockFreq(uint32_t HCLK_Frequency)
{
  /* PCLK2 clock frequency */
  return __LL_RCC_CALC_PCLK2_FREQ(HCLK_Frequency, LL_RCC_GetAPB2Prescaler());
}

/**
  * @brief  Return PLL clock frequency used for system domain
  * @retval PLL clock frequency (in Hz)
  */
uint32_t RCC_PLL_GetFreqDomain_SYS(void)
{
  uint32_t pllinputfreq = 0U, pllsource = 0U;

  /* PLL_VCO = (HSE_VALUE, HSI_VALUE or PLL2 / PLL Predivider) * PLL Multiplicator */

  /* Get PLL source */
  pllsource = LL_RCC_PLL_GetMainSource();

  switch (pllsource)
  {
    case LL_RCC_PLLSOURCE_HSI_DIV_2: /* HSI used as PLL clock source */
      pllinputfreq = HSI_VALUE / 2U;
      break;

    case LL_RCC_PLLSOURCE_HSE:       /* HSE used as PLL clock source */
      pllinputfreq = HSE_VALUE / (LL_RCC_PLL_GetPrediv() + 1U);
      break;

#if defined(RCC_PLL2_SUPPORT)
    case LL_RCC_PLLSOURCE_PLL2:       /* PLL2 used as PLL clock source */
      pllinputfreq = RCC_PLL2_GetFreqClockFreq() / (LL_RCC_PLL_GetPrediv() + 1U);
      break;
#endif /* RCC_PLL2_SUPPORT */

    default:
      pllinputfreq = HSI_VALUE / 2U;
      break;
  }
  return __LL_RCC_CALC_PLLCLK_FREQ(pllinputfreq, LL_RCC_PLL_GetMultiplicator());
}

#if defined(RCC_PLL2_SUPPORT)
/**
  * @brief  Return PLL clock frequency used for system domain
  * @retval PLL clock frequency (in Hz)
  */
uint32_t RCC_PLL2_GetFreqClockFreq(void)
{
  return __LL_RCC_CALC_PLL2CLK_FREQ(HSE_VALUE, LL_RCC_PLL2_GetMultiplicator(), LL_RCC_HSE_GetPrediv2());
}
#endif /* RCC_PLL2_SUPPORT */

#if defined(RCC_PLLI2S_SUPPORT)
/**
  * @brief  Return PLL clock frequency used for system domain
  * @retval PLL clock frequency (in Hz)
  */
uint32_t RCC_PLLI2S_GetFreqDomain_I2S(void)
{
  return __LL_RCC_CALC_PLLI2SCLK_FREQ(HSE_VALUE, LL_RCC_PLLI2S_GetMultiplicator(), LL_RCC_HSE_GetPrediv2());
}
#endif /* RCC_PLLI2S_SUPPORT */

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

#endif /* USE_FULL_LL_DRIVER */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
