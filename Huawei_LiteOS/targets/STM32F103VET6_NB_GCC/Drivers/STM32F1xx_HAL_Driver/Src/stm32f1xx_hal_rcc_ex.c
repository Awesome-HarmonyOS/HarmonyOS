/**
  ******************************************************************************
  * @file    stm32f1xx_hal_rcc_ex.c
  * @author  MCD Application Team
  * @brief   Extended RCC HAL module driver.
  *          This file provides firmware functions to manage the following 
  *          functionalities RCC extension peripheral:
  *           + Extended Peripheral Control functions
  *  
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/** @addtogroup STM32F1xx_HAL_Driver
  * @{
  */

#ifdef HAL_RCC_MODULE_ENABLED

/** @defgroup RCCEx RCCEx
  * @brief RCC Extension HAL module driver.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup RCCEx_Private_Constants RCCEx Private Constants
  * @{
  */
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/** @defgroup RCCEx_Private_Macros RCCEx Private Macros
  * @{
  */
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup RCCEx_Exported_Functions RCCEx Exported Functions
  * @{
  */

/** @defgroup RCCEx_Exported_Functions_Group1 Peripheral Control functions 
  *  @brief  Extended Peripheral Control functions  
  *
@verbatim   
 ===============================================================================
                ##### Extended Peripheral Control functions  #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to control the RCC Clocks 
    frequencies.
    [..] 
    (@) Important note: Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to
        select the RTC clock source; in this case the Backup domain will be reset in  
        order to modify the RTC Clock source, as consequence RTC registers (including 
        the backup registers) are set to their reset values.
      
@endverbatim
  * @{
  */

/**
  * @brief  Initializes the RCC extended peripherals clocks according to the specified parameters in the
  *         RCC_PeriphCLKInitTypeDef.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         contains the configuration information for the Extended Peripherals clocks(RTC clock).
  *
  * @note   Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to select 
  *         the RTC clock source; in this case the Backup domain will be reset in  
  *         order to modify the RTC Clock source, as consequence RTC registers (including 
  *         the backup registers) are set to their reset values.
  *
  * @note   In case of STM32F105xC or STM32F107xC devices, PLLI2S will be enabled if requested on 
  *         one of 2 I2S interfaces. When PLLI2S is enabled, you need to call HAL_RCCEx_DisablePLLI2S to
  *         manually disable it.
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tickstart = 0U, temp_reg = 0U;
#if defined(STM32F105xC) || defined(STM32F107xC)
  uint32_t  pllactive = 0U;
#endif /* STM32F105xC || STM32F107xC */

  /* Check the parameters */
  assert_param(IS_RCC_PERIPHCLOCK(PeriphClkInit->PeriphClockSelection));
  
  /*------------------------------- RTC/LCD Configuration ------------------------*/ 
  if ((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC) == RCC_PERIPHCLK_RTC))
  {
    /* check for RTC Parameters used to output RTCCLK */
    assert_param(IS_RCC_RTCCLKSOURCE(PeriphClkInit->RTCClockSelection));

    FlagStatus       pwrclkchanged = RESET;

    /* As soon as function is called to change RTC clock source, activation of the 
       power domain is done. */
    /* Requires to enable write access to Backup Domain of necessary */
    if(__HAL_RCC_PWR_IS_CLK_DISABLED())
    {
    __HAL_RCC_PWR_CLK_ENABLE();
      pwrclkchanged = SET;
    }
    
    if(HAL_IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
    {
      /* Enable write access to Backup domain */
      SET_BIT(PWR->CR, PWR_CR_DBP);
      
      /* Wait for Backup domain Write protection disable */
      tickstart = HAL_GetTick();
      
      while(HAL_IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
      {
        if((HAL_GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }
      
    /* Reset the Backup domain only if the RTC Clock source selection is modified from reset value */ 
    temp_reg = (RCC->BDCR & RCC_BDCR_RTCSEL);
    if((temp_reg != 0x00000000U) && (temp_reg != (PeriphClkInit->RTCClockSelection & RCC_BDCR_RTCSEL)))
    {
      /* Store the content of BDCR register before the reset of Backup Domain */
      temp_reg = (RCC->BDCR & ~(RCC_BDCR_RTCSEL));
      /* RTC Clock selection can be changed only if the Backup Domain is reset */
      __HAL_RCC_BACKUPRESET_FORCE();
      __HAL_RCC_BACKUPRESET_RELEASE();
      /* Restore the Content of BDCR register */
      RCC->BDCR = temp_reg;

      /* Wait for LSERDY if LSE was enabled */
      if (HAL_IS_BIT_SET(temp_reg, RCC_BDCR_LSEON))
      {
        /* Get Start Tick */
        tickstart = HAL_GetTick();
      
        /* Wait till LSE is ready */  
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
        {
          if((HAL_GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }      
        }  
      }
    }
    __HAL_RCC_RTC_CONFIG(PeriphClkInit->RTCClockSelection); 

    /* Require to disable power clock if necessary */
    if(pwrclkchanged == SET)
    {
      __HAL_RCC_PWR_CLK_DISABLE();
    }
  }

  /*------------------------------ ADC clock Configuration ------------------*/ 
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_ADC) == RCC_PERIPHCLK_ADC)
  {
    /* Check the parameters */
    assert_param(IS_RCC_ADCPLLCLK_DIV(PeriphClkInit->AdcClockSelection));
    
    /* Configure the ADC clock source */
    __HAL_RCC_ADC_CONFIG(PeriphClkInit->AdcClockSelection);
  }

#if defined(STM32F105xC) || defined(STM32F107xC)
  /*------------------------------ I2S2 Configuration ------------------------*/ 
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S2) == RCC_PERIPHCLK_I2S2)
  {
    /* Check the parameters */
    assert_param(IS_RCC_I2S2CLKSOURCE(PeriphClkInit->I2s2ClockSelection));

    /* Configure the I2S2 clock source */
    __HAL_RCC_I2S2_CONFIG(PeriphClkInit->I2s2ClockSelection);
  }

  /*------------------------------ I2S3 Configuration ------------------------*/ 
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S3) == RCC_PERIPHCLK_I2S3)
  {
    /* Check the parameters */
    assert_param(IS_RCC_I2S3CLKSOURCE(PeriphClkInit->I2s3ClockSelection));
    
    /* Configure the I2S3 clock source */
    __HAL_RCC_I2S3_CONFIG(PeriphClkInit->I2s3ClockSelection);
  }

  /*------------------------------ PLL I2S Configuration ----------------------*/ 
  /* Check that PLLI2S need to be enabled */
  if (HAL_IS_BIT_SET(RCC->CFGR2, RCC_CFGR2_I2S2SRC) || HAL_IS_BIT_SET(RCC->CFGR2, RCC_CFGR2_I2S3SRC))
  {
    /* Update flag to indicate that PLL I2S should be active */
    pllactive = 1;
  }

  /* Check if PLL I2S need to be enabled */
  if (pllactive == 1)
  {
    /* Enable PLL I2S only if not active */
    if (HAL_IS_BIT_CLR(RCC->CR, RCC_CR_PLL3ON))
    {
      /* Check the parameters */
      assert_param(IS_RCC_PLLI2S_MUL(PeriphClkInit->PLLI2S.PLLI2SMUL));
      assert_param(IS_RCC_HSE_PREDIV2(PeriphClkInit->PLLI2S.HSEPrediv2Value));

      /* Prediv2 can be written only when the PLL2 is disabled. */
      /* Return an error only if new value is different from the programmed value */
      if (HAL_IS_BIT_SET(RCC->CR,RCC_CR_PLL2ON) && \
        (__HAL_RCC_HSE_GET_PREDIV2() != PeriphClkInit->PLLI2S.HSEPrediv2Value))
      {
        return HAL_ERROR;
      }

      /* Configure the HSE prediv2 factor --------------------------------*/
      __HAL_RCC_HSE_PREDIV2_CONFIG(PeriphClkInit->PLLI2S.HSEPrediv2Value);

      /* Configure the main PLLI2S multiplication factors. */
      __HAL_RCC_PLLI2S_CONFIG(PeriphClkInit->PLLI2S.PLLI2SMUL);
      
      /* Enable the main PLLI2S. */
      __HAL_RCC_PLLI2S_ENABLE();
      
      /* Get Start Tick*/
      tickstart = HAL_GetTick();
      
      /* Wait till PLLI2S is ready */
      while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  == RESET)
      {
        if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }
    else
    {
      /* Return an error only if user wants to change the PLLI2SMUL whereas PLLI2S is active */
      if (READ_BIT(RCC->CFGR2, RCC_CFGR2_PLL3MUL) != PeriphClkInit->PLLI2S.PLLI2SMUL)
      {
          return HAL_ERROR;
      }
    }
  }
#endif /* STM32F105xC || STM32F107xC */

#if defined(STM32F102x6) || defined(STM32F102xB) || defined(STM32F103x6)\
 || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG)\
 || defined(STM32F105xC) || defined(STM32F107xC)
  /*------------------------------ USB clock Configuration ------------------*/ 
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USB) == RCC_PERIPHCLK_USB)
  {
    /* Check the parameters */
    assert_param(IS_RCC_USBPLLCLK_DIV(PeriphClkInit->UsbClockSelection));
    
    /* Configure the USB clock source */
    __HAL_RCC_USB_CONFIG(PeriphClkInit->UsbClockSelection);
  }
#endif /* STM32F102x6 || STM32F102xB || STM32F103x6 || STM32F103xB || STM32F103xE || STM32F103xG || STM32F105xC || STM32F107xC */

  return HAL_OK;
}

/**
  * @brief  Get the PeriphClkInit according to the internal
  * RCC configuration registers.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that 
  *         returns the configuration information for the Extended Peripherals clocks(RTC, I2S, ADC clocks).
  * @retval None
  */
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t srcclk = 0U;
  
  /* Set all possible values for the extended clock type parameter------------*/
  PeriphClkInit->PeriphClockSelection = RCC_PERIPHCLK_RTC;

  /* Get the RTC configuration -----------------------------------------------*/
  srcclk = __HAL_RCC_GET_RTC_SOURCE();
  /* Source clock is LSE or LSI*/
  PeriphClkInit->RTCClockSelection = srcclk;

  /* Get the ADC clock configuration -----------------------------------------*/
  PeriphClkInit->PeriphClockSelection |= RCC_PERIPHCLK_ADC;
  PeriphClkInit->AdcClockSelection = __HAL_RCC_GET_ADC_SOURCE();

#if defined(STM32F105xC) || defined(STM32F107xC)
  /* Get the I2S2 clock configuration -----------------------------------------*/
  PeriphClkInit->PeriphClockSelection |= RCC_PERIPHCLK_I2S2;
  PeriphClkInit->I2s2ClockSelection = __HAL_RCC_GET_I2S2_SOURCE();

  /* Get the I2S3 clock configuration -----------------------------------------*/
  PeriphClkInit->PeriphClockSelection |= RCC_PERIPHCLK_I2S3;
  PeriphClkInit->I2s3ClockSelection = __HAL_RCC_GET_I2S3_SOURCE();

#endif /* STM32F105xC || STM32F107xC */

#if defined(STM32F103xE) || defined(STM32F103xG)
  /* Get the I2S2 clock configuration -----------------------------------------*/
  PeriphClkInit->PeriphClockSelection |= RCC_PERIPHCLK_I2S2;
  PeriphClkInit->I2s2ClockSelection = RCC_I2S2CLKSOURCE_SYSCLK;

  /* Get the I2S3 clock configuration -----------------------------------------*/
  PeriphClkInit->PeriphClockSelection |= RCC_PERIPHCLK_I2S3;
  PeriphClkInit->I2s3ClockSelection = RCC_I2S3CLKSOURCE_SYSCLK;

#endif /* STM32F103xE || STM32F103xG */

#if defined(STM32F102x6) || defined(STM32F102xB) || defined(STM32F103x6)\
 || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG)\
 || defined(STM32F105xC) || defined(STM32F107xC)
  /* Get the USB clock configuration -----------------------------------------*/
  PeriphClkInit->PeriphClockSelection |= RCC_PERIPHCLK_USB;
  PeriphClkInit->UsbClockSelection = __HAL_RCC_GET_USB_SOURCE();
#endif /* STM32F102x6 || STM32F102xB || STM32F103x6 || STM32F103xB || STM32F103xE || STM32F103xG || STM32F105xC || STM32F107xC */
}

/**
  * @brief  Returns the peripheral clock frequency
  * @note   Returns 0 if peripheral clock is unknown
  * @param  PeriphClk Peripheral clock identifier
  *         This parameter can be one of the following values:
  *            @arg @ref RCC_PERIPHCLK_RTC  RTC peripheral clock
  *            @arg @ref RCC_PERIPHCLK_ADC  ADC peripheral clock
  @if STM32F103xE
  *            @arg @ref RCC_PERIPHCLK_I2S2 I2S2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S3 I2S3 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S3 I2S3 peripheral clock
  @endif
  @if STM32F103xG
  *            @arg @ref RCC_PERIPHCLK_I2S2 I2S2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S3 I2S3 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S3 I2S3 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S2 I2S2 peripheral clock
  @endif
  @if STM32F105xC
  *            @arg @ref RCC_PERIPHCLK_I2S2 I2S2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S3 I2S3 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S3 I2S3 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S2 I2S2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S3 I2S3 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S3 I2S3 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S2 I2S2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_USB  USB peripheral clock
  @endif
  @if STM32F107xC
  *            @arg @ref RCC_PERIPHCLK_I2S2 I2S2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S3 I2S3 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S3 I2S3 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S2 I2S2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S3 I2S3 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S3 I2S3 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2S2 I2S2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_USB  USB peripheral clock
  @endif
  @if STM32F102xx
  *            @arg @ref RCC_PERIPHCLK_USB  USB peripheral clock
  @endif
  @if STM32F103xx
  *            @arg @ref RCC_PERIPHCLK_USB  USB peripheral clock
  @endif
  * @retval Frequency in Hz (0: means that no available frequency for the peripheral)
  */
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk)
{
#if defined(STM32F105xC) || defined(STM32F107xC)
  const uint8_t aPLLMULFactorTable[14] = {0, 0, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 13};
  const uint8_t aPredivFactorTable[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

  uint32_t prediv1 = 0U, pllclk = 0U, pllmul = 0U;
  uint32_t pll2mul = 0U, pll3mul = 0U, prediv2 = 0U;
#endif /* STM32F105xC || STM32F107xC */
#if defined(STM32F102x6) || defined(STM32F102xB) || defined(STM32F103x6) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG)
  const uint8_t aPLLMULFactorTable[16] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 16};
  const uint8_t aPredivFactorTable[2] = {1, 2};

  uint32_t prediv1 = 0U, pllclk = 0U, pllmul = 0U;
#endif /* STM32F102x6 || STM32F102xB || STM32F103x6 || STM32F103xB || STM32F103xE || STM32F103xG */
  uint32_t temp_reg = 0U, frequency = 0U;

  /* Check the parameters */
  assert_param(IS_RCC_PERIPHCLOCK(PeriphClk));
  
  switch (PeriphClk)
  {
#if defined(STM32F102x6) || defined(STM32F102xB) || defined(STM32F103x6)\
 || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG)\
 || defined(STM32F105xC) || defined(STM32F107xC)
  case RCC_PERIPHCLK_USB:  
    {
      /* Get RCC configuration ------------------------------------------------------*/
      temp_reg = RCC->CFGR;
  
      /* Check if PLL is enabled */
      if (HAL_IS_BIT_SET(RCC->CR,RCC_CR_PLLON))
      {
        pllmul = aPLLMULFactorTable[(uint32_t)(temp_reg & RCC_CFGR_PLLMULL) >> RCC_CFGR_PLLMULL_Pos];
        if ((temp_reg & RCC_CFGR_PLLSRC) != RCC_PLLSOURCE_HSI_DIV2)
        {
#if defined(STM32F105xC) || defined(STM32F107xC) || defined(STM32F100xB)\
 || defined(STM32F100xE)
          prediv1 = aPredivFactorTable[(uint32_t)(RCC->CFGR2 & RCC_CFGR2_PREDIV1) >> RCC_CFGR2_PREDIV1_Pos];
#else
          prediv1 = aPredivFactorTable[(uint32_t)(RCC->CFGR & RCC_CFGR_PLLXTPRE) >> RCC_CFGR_PLLXTPRE_Pos];
#endif /* STM32F105xC || STM32F107xC || STM32F100xB || STM32F100xE */

#if defined(STM32F105xC) || defined(STM32F107xC)
          if(HAL_IS_BIT_SET(RCC->CFGR2, RCC_CFGR2_PREDIV1SRC))
          {
            /* PLL2 selected as Prediv1 source */
            /* PLLCLK = PLL2CLK / PREDIV1 * PLLMUL with PLL2CLK = HSE/PREDIV2 * PLL2MUL */
            prediv2 = ((RCC->CFGR2 & RCC_CFGR2_PREDIV2) >> RCC_CFGR2_PREDIV2_Pos) + 1;
            pll2mul = ((RCC->CFGR2 & RCC_CFGR2_PLL2MUL) >> RCC_CFGR2_PLL2MUL_Pos) + 2;
            pllclk = (uint32_t)((((HSE_VALUE / prediv2) * pll2mul) / prediv1) * pllmul);
          }
          else
          {
            /* HSE used as PLL clock source : PLLCLK = HSE/PREDIV1 * PLLMUL */
            pllclk = (uint32_t)((HSE_VALUE / prediv1) * pllmul);
          }
          
          /* If PLLMUL was set to 13 means that it was to cover the case PLLMUL 6.5 (avoid using float) */
          /* In this case need to divide pllclk by 2 */
          if (pllmul == aPLLMULFactorTable[(uint32_t)(RCC_CFGR_PLLMULL6_5) >> RCC_CFGR_PLLMULL_Pos])
          {
              pllclk = pllclk / 2;
          }
#else
          if ((temp_reg & RCC_CFGR_PLLSRC) != RCC_PLLSOURCE_HSI_DIV2)
          {
            /* HSE used as PLL clock source : PLLCLK = HSE/PREDIV1 * PLLMUL */
            pllclk = (uint32_t)((HSE_VALUE / prediv1) * pllmul);
          }
#endif /* STM32F105xC || STM32F107xC */
        }
        else
        {
          /* HSI used as PLL clock source : PLLCLK = HSI/2 * PLLMUL */
          pllclk = (uint32_t)((HSI_VALUE >> 1) * pllmul);
        }

        /* Calcul of the USB frequency*/
#if defined(STM32F105xC) || defined(STM32F107xC)
        /* USBCLK = PLLVCO = (2 x PLLCLK) / USB prescaler */
        if (__HAL_RCC_GET_USB_SOURCE() == RCC_USBCLKSOURCE_PLL_DIV2)
        {
          /* Prescaler of 2 selected for USB */ 
          frequency = pllclk;
        }
        else
        {
          /* Prescaler of 3 selected for USB */ 
          frequency = (2 * pllclk) / 3;
        }
#else
        /* USBCLK = PLLCLK / USB prescaler */
        if (__HAL_RCC_GET_USB_SOURCE() == RCC_USBCLKSOURCE_PLL)
        {
          /* No prescaler selected for USB */
          frequency = pllclk;
        }
        else
        {
          /* Prescaler of 1.5 selected for USB */ 
          frequency = (pllclk * 2) / 3;
        }
#endif
      }
      break;
    }
#endif /* STM32F102x6 || STM32F102xB || STM32F103x6 || STM32F103xB || STM32F103xE || STM32F103xG || STM32F105xC || STM32F107xC */
#if defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC)
  case RCC_PERIPHCLK_I2S2:  
    {
#if defined(STM32F103xE) || defined(STM32F103xG)
      /* SYSCLK used as source clock for I2S2 */
      frequency = HAL_RCC_GetSysClockFreq();
#else
      if (__HAL_RCC_GET_I2S2_SOURCE() == RCC_I2S2CLKSOURCE_SYSCLK)
      {
        /* SYSCLK used as source clock for I2S2 */
        frequency = HAL_RCC_GetSysClockFreq();
      }
      else
      {
         /* Check if PLLI2S is enabled */
        if (HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLL3ON))
        {
          /* PLLI2SVCO = 2 * PLLI2SCLK = 2 * (HSE/PREDIV2 * PLL3MUL) */
          prediv2 = ((RCC->CFGR2 & RCC_CFGR2_PREDIV2) >> RCC_CFGR2_PREDIV2_Pos) + 1;
          pll3mul = ((RCC->CFGR2 & RCC_CFGR2_PLL3MUL) >> RCC_CFGR2_PLL3MUL_Pos) + 2;
          frequency = (uint32_t)(2 * ((HSE_VALUE / prediv2) * pll3mul));
        }
      }
#endif /* STM32F103xE || STM32F103xG */
      break;
    }
  case RCC_PERIPHCLK_I2S3:
    {
#if defined(STM32F103xE) || defined(STM32F103xG)
      /* SYSCLK used as source clock for I2S3 */
      frequency = HAL_RCC_GetSysClockFreq();
#else
      if (__HAL_RCC_GET_I2S3_SOURCE() == RCC_I2S3CLKSOURCE_SYSCLK)
      {
        /* SYSCLK used as source clock for I2S3 */
        frequency = HAL_RCC_GetSysClockFreq();
      }
      else
      {
         /* Check if PLLI2S is enabled */
        if (HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLL3ON))
        {
          /* PLLI2SVCO = 2 * PLLI2SCLK = 2 * (HSE/PREDIV2 * PLL3MUL) */
          prediv2 = ((RCC->CFGR2 & RCC_CFGR2_PREDIV2) >> RCC_CFGR2_PREDIV2_Pos) + 1;
          pll3mul = ((RCC->CFGR2 & RCC_CFGR2_PLL3MUL) >> RCC_CFGR2_PLL3MUL_Pos) + 2;
          frequency = (uint32_t)(2 * ((HSE_VALUE / prediv2) * pll3mul));
        }
      }
#endif /* STM32F103xE || STM32F103xG */
      break;
    }
#endif /* STM32F103xE || STM32F103xG || STM32F105xC || STM32F107xC */
  case RCC_PERIPHCLK_RTC:  
    {
      /* Get RCC BDCR configuration ------------------------------------------------------*/
      temp_reg = RCC->BDCR;

      /* Check if LSE is ready if RTC clock selection is LSE */
      if (((temp_reg & RCC_BDCR_RTCSEL) == RCC_RTCCLKSOURCE_LSE) && (HAL_IS_BIT_SET(temp_reg, RCC_BDCR_LSERDY)))
      {
        frequency = LSE_VALUE;
      }
      /* Check if LSI is ready if RTC clock selection is LSI */
      else if (((temp_reg & RCC_BDCR_RTCSEL) == RCC_RTCCLKSOURCE_LSI) && (HAL_IS_BIT_SET(RCC->CSR, RCC_CSR_LSIRDY)))
      {
        frequency = LSI_VALUE;
      }
      else if (((temp_reg & RCC_BDCR_RTCSEL) == RCC_RTCCLKSOURCE_HSE_DIV128) && (HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSERDY)))
      {
        frequency = HSE_VALUE / 128U;
      }
      /* Clock not enabled for RTC*/
      else
      {
        frequency = 0U;
      }
      break;
    }
  case RCC_PERIPHCLK_ADC:  
    {
      frequency = HAL_RCC_GetPCLK2Freq() / (((__HAL_RCC_GET_ADC_SOURCE() >> RCC_CFGR_ADCPRE_Pos) + 1) * 2);
      break;
    }
  default: 
    {
      break;
    }
  }
  return(frequency);
}

/**
  * @}
  */

#if defined(STM32F105xC) || defined(STM32F107xC)
/** @defgroup RCCEx_Exported_Functions_Group2 PLLI2S Management function
  *  @brief  PLLI2S Management functions
  *
@verbatim   
 ===============================================================================
                ##### Extended PLLI2S Management functions  #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to control the PLLI2S
    activation or deactivation
@endverbatim
  * @{
  */

/**
  * @brief  Enable PLLI2S
  * @param  PLLI2SInit pointer to an RCC_PLLI2SInitTypeDef structure that
  *         contains the configuration information for the PLLI2S
  * @note   The PLLI2S configuration not modified if used by I2S2 or I2S3 Interface.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_EnablePLLI2S(RCC_PLLI2SInitTypeDef  *PLLI2SInit)
{
  uint32_t tickstart = 0U;

  /* Check that PLL I2S has not been already enabled by I2S2 or I2S3*/
  if (HAL_IS_BIT_CLR(RCC->CFGR2, RCC_CFGR2_I2S2SRC) && HAL_IS_BIT_CLR(RCC->CFGR2, RCC_CFGR2_I2S3SRC))
  {
    /* Check the parameters */
    assert_param(IS_RCC_PLLI2S_MUL(PLLI2SInit->PLLI2SMUL));
    assert_param(IS_RCC_HSE_PREDIV2(PLLI2SInit->HSEPrediv2Value));

    /* Prediv2 can be written only when the PLL2 is disabled. */
    /* Return an error only if new value is different from the programmed value */
    if (HAL_IS_BIT_SET(RCC->CR,RCC_CR_PLL2ON) && \
      (__HAL_RCC_HSE_GET_PREDIV2() != PLLI2SInit->HSEPrediv2Value))
    {
      return HAL_ERROR;
    }

    /* Disable the main PLLI2S. */
    __HAL_RCC_PLLI2S_DISABLE();

    /* Get Start Tick*/
    tickstart = HAL_GetTick();
    
    /* Wait till PLLI2S is ready */  
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  != RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }

    /* Configure the HSE prediv2 factor --------------------------------*/
    __HAL_RCC_HSE_PREDIV2_CONFIG(PLLI2SInit->HSEPrediv2Value);
    

    /* Configure the main PLLI2S multiplication factors. */
    __HAL_RCC_PLLI2S_CONFIG(PLLI2SInit->PLLI2SMUL);
    
    /* Enable the main PLLI2S. */
    __HAL_RCC_PLLI2S_ENABLE();
    
    /* Get Start Tick*/
    tickstart = HAL_GetTick();
    
    /* Wait till PLLI2S is ready */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  == RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
  }
  else
  {
    /* PLLI2S cannot be modified as already used by I2S2 or I2S3 */
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Disable PLLI2S
  * @note   PLLI2S is not disabled if used by I2S2 or I2S3 Interface.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_DisablePLLI2S(void)
{
  uint32_t tickstart = 0U;

  /* Disable PLL I2S as not requested by I2S2 or I2S3*/
  if (HAL_IS_BIT_CLR(RCC->CFGR2, RCC_CFGR2_I2S2SRC) && HAL_IS_BIT_CLR(RCC->CFGR2, RCC_CFGR2_I2S3SRC))
  {
    /* Disable the main PLLI2S. */
    __HAL_RCC_PLLI2S_DISABLE();

    /* Get Start Tick*/
    tickstart = HAL_GetTick();
    
    /* Wait till PLLI2S is ready */  
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  != RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
  }
  else
  {
    /* PLLI2S is currently used by I2S2 or I2S3. Cannot be disabled.*/
    return HAL_ERROR;
  }
  
  return HAL_OK;
}

/**
  * @}
  */

/** @defgroup RCCEx_Exported_Functions_Group3 PLL2 Management function
  *  @brief  PLL2 Management functions
  *
@verbatim   
 ===============================================================================
                ##### Extended PLL2 Management functions  #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to control the PLL2
    activation or deactivation
@endverbatim
  * @{
  */

/**
  * @brief  Enable PLL2
  * @param  PLL2Init pointer to an RCC_PLL2InitTypeDef structure that
  *         contains the configuration information for the PLL2
  * @note   The PLL2 configuration not modified if used indirectly as system clock.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_EnablePLL2(RCC_PLL2InitTypeDef  *PLL2Init)
{
  uint32_t tickstart = 0U;

  /* This bit can not be cleared if the PLL2 clock is used indirectly as system 
    clock (i.e. it is used as PLL clock entry that is used as system clock). */
  if((__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSE) && \
        (__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) && \
        ((READ_BIT(RCC->CFGR2,RCC_CFGR2_PREDIV1SRC)) == RCC_CFGR2_PREDIV1SRC_PLL2))
  {
    return HAL_ERROR;
  }
  else
  {
    /* Check the parameters */
    assert_param(IS_RCC_PLL2_MUL(PLL2Init->PLL2MUL));
    assert_param(IS_RCC_HSE_PREDIV2(PLL2Init->HSEPrediv2Value));

    /* Prediv2 can be written only when the PLLI2S is disabled. */
    /* Return an error only if new value is different from the programmed value */
    if (HAL_IS_BIT_SET(RCC->CR,RCC_CR_PLL3ON) && \
      (__HAL_RCC_HSE_GET_PREDIV2() != PLL2Init->HSEPrediv2Value))
    {
      return HAL_ERROR;
    }

    /* Disable the main PLL2. */
    __HAL_RCC_PLL2_DISABLE();
    
    /* Get Start Tick*/
    tickstart = HAL_GetTick();
    
    /* Wait till PLL2 is disabled */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY) != RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLL2_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
    
    /* Configure the HSE prediv2 factor --------------------------------*/
    __HAL_RCC_HSE_PREDIV2_CONFIG(PLL2Init->HSEPrediv2Value);

    /* Configure the main PLL2 multiplication factors. */
    __HAL_RCC_PLL2_CONFIG(PLL2Init->PLL2MUL);
    
    /* Enable the main PLL2. */
    __HAL_RCC_PLL2_ENABLE();
    
    /* Get Start Tick*/
    tickstart = HAL_GetTick();
    
    /* Wait till PLL2 is ready */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY)  == RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLL2_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
  }

  return HAL_OK;
}

/**
  * @brief  Disable PLL2
  * @note   PLL2 is not disabled if used indirectly as system clock.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_DisablePLL2(void)
{
  uint32_t tickstart = 0U;

  /* This bit can not be cleared if the PLL2 clock is used indirectly as system 
    clock (i.e. it is used as PLL clock entry that is used as system clock). */
  if((__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSE) && \
        (__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) && \
        ((READ_BIT(RCC->CFGR2,RCC_CFGR2_PREDIV1SRC)) == RCC_CFGR2_PREDIV1SRC_PLL2))
  {
    return HAL_ERROR;
  }
  else
  {
    /* Disable the main PLL2. */
    __HAL_RCC_PLL2_DISABLE();

    /* Get Start Tick*/
    tickstart = HAL_GetTick();
    
    /* Wait till PLL2 is disabled */  
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY)  != RESET)
    {
      if((HAL_GetTick() - tickstart ) > PLL2_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
  }

  return HAL_OK;
}

/**
  * @}
  */
#endif /* STM32F105xC || STM32F107xC */

/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_RCC_MODULE_ENABLED */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

