/**
  ******************************************************************************
  * @file    stm32f4xx_hal_fmpi2c_ex.c
  * @author  MCD Application Team
  * @brief   FMPI2C Extended HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of FMPI2C Extended peripheral:
  *           + Extended features functions
  *
  @verbatim
  ==============================================================================
               ##### FMPI2C peripheral Extended features  #####
  ==============================================================================

  [..] Comparing to other previous devices, the FMPI2C interface for STM32F4xx
       devices contains the following additional features

       (+) Possibility to disable or enable Analog Noise Filter
       (+) Use of a configured Digital Noise Filter
       (+) Disable or enable Fast Mode Plus

                     ##### How to use this driver #####
  ==============================================================================
  [..] This driver provides functions to configure Noise Filter and Wake Up Feature
    (#) Configure FMPI2C Analog noise filter using the function HAL_FMPI2CEx_ConfigAnalogFilter()
    (#) Configure FMPI2C Digital noise filter using the function HAL_FMPI2CEx_ConfigDigitalFilter()
    (#) Configure the enable or disable of fast mode plus driving capability using the functions :
          (++) HAL_FMPI2CEx_EnableFastModePlus()
          (++) HAL_FMPI2CEx_DisableFastModePlus()
  @endverbatim
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup FMPI2CEx FMPI2CEx
  * @brief FMPI2C Extended HAL module driver
  * @{
  */

#ifdef HAL_FMPI2C_MODULE_ENABLED

#if defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx) || defined(STM32F446xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || \
    defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup FMPI2CEx_Exported_Functions FMPI2C Extended Exported Functions
  * @{
  */

/** @defgroup FMPI2CEx_Exported_Functions_Group1 Extended features functions
  * @brief    Extended features functions
 *
@verbatim
 ===============================================================================
                      ##### Extended features functions #####
 ===============================================================================
    [..] This section provides functions allowing to:
      (+) Configure Noise Filters
      (+) Configure Fast Mode Plus

@endverbatim
  * @{
  */

/**
  * @brief  Configure FMPI2C Analog noise filter.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2Cx peripheral.
  * @param  AnalogFilter New state of the Analog filter.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2CEx_ConfigAnalogFilter(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t AnalogFilter)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_INSTANCE(hfmpi2c->Instance));
  assert_param(IS_FMPI2C_ANALOG_FILTER(AnalogFilter));

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    hfmpi2c->State = HAL_FMPI2C_STATE_BUSY;

    /* Disable the selected FMPI2C peripheral */
    __HAL_FMPI2C_DISABLE(hfmpi2c);

    /* Reset FMPI2Cx ANOFF bit */
    hfmpi2c->Instance->CR1 &= ~(FMPI2C_CR1_ANFOFF);

    /* Set analog filter bit*/
    hfmpi2c->Instance->CR1 |= AnalogFilter;

    __HAL_FMPI2C_ENABLE(hfmpi2c);

    hfmpi2c->State = HAL_FMPI2C_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Configure FMPI2C Digital noise filter.
  * @param  hfmpi2c Pointer to a FMPI2C_HandleTypeDef structure that contains
  *                the configuration information for the specified FMPI2Cx peripheral.
  * @param  DigitalFilter Coefficient of digital noise filter between Min_Data=0x00 and Max_Data=0x0F.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_FMPI2CEx_ConfigDigitalFilter(FMPI2C_HandleTypeDef *hfmpi2c, uint32_t DigitalFilter)
{
  uint32_t tmpreg = 0U;

  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_INSTANCE(hfmpi2c->Instance));
  assert_param(IS_FMPI2C_DIGITAL_FILTER(DigitalFilter));

  if (hfmpi2c->State == HAL_FMPI2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hfmpi2c);

    hfmpi2c->State = HAL_FMPI2C_STATE_BUSY;

    /* Disable the selected FMPI2C peripheral */
    __HAL_FMPI2C_DISABLE(hfmpi2c);

    /* Get the old register value */
    tmpreg = hfmpi2c->Instance->CR1;

    /* Reset FMPI2Cx DNF bits [11:8] */
    tmpreg &= ~(FMPI2C_CR1_DFN);

    /* Set FMPI2Cx DNF coefficient */
    tmpreg |= DigitalFilter << 8U;

    /* Store the new register value */
    hfmpi2c->Instance->CR1 = tmpreg;

    __HAL_FMPI2C_ENABLE(hfmpi2c);

    hfmpi2c->State = HAL_FMPI2C_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hfmpi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief Enable the FMPI2C fast mode plus driving capability.
  * @param ConfigFastModePlus Selects the pin.
  *   This parameter can be one of the @ref FMPI2CEx_FastModePlus values
  * @retval None
  */
void HAL_FMPI2CEx_EnableFastModePlus(uint32_t ConfigFastModePlus)
{
  /* Check the parameter */
  assert_param(IS_FMPI2C_FASTMODEPLUS(ConfigFastModePlus));

  /* Enable SYSCFG clock */
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* Enable fast mode plus driving capability for selected pin */
  SET_BIT(SYSCFG->CFGR, (uint32_t)ConfigFastModePlus);
}

/**
  * @brief Disable the FMPI2C fast mode plus driving capability.
  * @param ConfigFastModePlus Selects the pin.
  *   This parameter can be one of the @ref FMPI2CEx_FastModePlus values
  * @retval None
  */
void HAL_FMPI2CEx_DisableFastModePlus(uint32_t ConfigFastModePlus)
{
  /* Check the parameter */
  assert_param(IS_FMPI2C_FASTMODEPLUS(ConfigFastModePlus));

  /* Enable SYSCFG clock */
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* Disable fast mode plus driving capability for selected pin */
  CLEAR_BIT(SYSCFG->CFGR, (uint32_t)ConfigFastModePlus);
}

/**
  * @}
  */

/**
  * @}
  */
#endif /* STM32F410xx || STM32F446xx || STM32F412Zx || STM32F412Vx || STM32F412Rx || STM32F412Cx ||\
          STM32F413xx || STM32F423xx */
#endif /* HAL_FMPI2C_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
